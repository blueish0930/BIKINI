/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#include <optional>
#include <regex>

#include "NOD_expression_to_nodes.hh"
#include "NOD_geometry_nodes_execute.hh"
#include "NOD_menu_value.hh"
#include "NOD_multi_function.hh"
#include "NOD_node_declaration.hh"
#include "NOD_node_in_compute_context.hh"
#include "NOD_socket_usage_inference.hh"

#include "DNA_anim_types.h"
#include "DNA_material_types.h"
#include "DNA_node_types.h"

#include "BKE_compute_context_cache.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_runtime.hh"
#include "BKE_type_conversions.hh"

#include "ANIM_action.hh"
#include "ANIM_action_iterators.hh"

#include "BLI_listbase.hh"
#include "BLI_stack.hh"

namespace blender::nodes::socket_usage_inference {

/** True when descending into `group` would walk a tree already on this call path. */
static bool group_call_reenters(const ComputeContext *context,
                                const bNodeTree &owner,
                                const bNodeTree &group)
{
  const uint32_t uid = group.id.session_uid;
  if (owner.id.session_uid == uid) {
    return true;
  }
  for (const ComputeContext *ctx = context; ctx != nullptr; ctx = ctx->parent()) {
    const auto *group_ctx = dynamic_cast<const bke::GroupNodeComputeContext *>(ctx);
    if (group_ctx == nullptr || group_ctx->tree() == nullptr) {
      continue;
    }
    if (group_ctx->tree()->id.session_uid == uid) {
      return true;
    }
  }
  return false;
}

/** Utility class to simplify passing global state into all the functions during inferencing. */
class SocketUsageInferencerImpl {
 private:
  friend SocketUsageParams;

  bke::ComputeContextCache &compute_context_cache_;

  /** Inferences the socket values if possible. */
  SocketValueInferencer &value_inferencer_;

  /** Root node tree. */
  const bNodeTree &root_tree_;

  /**
   * Stack of tasks that allows depth-first (partial) evaluation of the tree.
   */
  Stack<SocketInContext> usage_tasks_;

  /**
   * If the usage of a socket is known, it is added to this map. Sockets not in this map are not
   * known yet.
   */
  Map<SocketInContext, bool> all_socket_usages_;

  /**
   * Stack of tasks that allows depth-first traversal of the tree to check if outputs are disabled.
   */
  Stack<SocketInContext> disabled_output_tasks_;

  /**
   * Contains whether a socket is disabled. Sockets not in this map are not known yet.
   */
  Map<SocketInContext, bool> all_socket_disable_states_;

  /**
   * Treat top-level nodes as if they are never muted for usage-inferencing. This is used when
   * computing the socket usage that is displayed in the node editor (through grayed out or hidden
   * sockets). Which inputs/outputs of a node is visible should never depend on whether it is muted
   * or not.
   */
  bool ignore_top_level_node_muting_ = false;

  Map<SocketInContext, std::shared_ptr<expression::ExpressionNodeGroup>>
      expression_node_groups_cache_;

 public:
  SocketUsageInferencer *owner_ = nullptr;

  SocketUsageInferencerImpl(const bNodeTree &tree,
                            SocketValueInferencer &value_inferencer,
                            bke::ComputeContextCache &compute_context_cache,
                            const bool ignore_top_level_node_muting)
      : compute_context_cache_(compute_context_cache),
        value_inferencer_(value_inferencer),
        root_tree_(tree),
        ignore_top_level_node_muting_(ignore_top_level_node_muting)
  {
    root_tree_.ensure_topology_cache();
    root_tree_.ensure_interface_cache();
  }

  void mark_top_level_node_outputs_as_used()
  {
    for (const bNode *node : root_tree_.all_nodes()) {
      if (node == nullptr || node->runtime == nullptr) {
        continue;
      }
      if (node->is_group_input()) {
        /* Can skip these sockets, because they don't affect usage anyway, and there may be a lot
         * of them. See #144756. */
        continue;
      }
      for (const bNodeSocket *socket : node->output_sockets()) {
        if (socket == nullptr || socket->runtime == nullptr) {
          continue;
        }
        all_socket_usages_.add_new({nullptr, socket}, true);
      }
    }
  }

  bool is_group_input_used(const int input_i)
  {
    for (const bNode *node : root_tree_.group_input_nodes()) {
      if (node == nullptr || node->runtime == nullptr) {
        continue;
      }
      if (!node->output_sockets().index_range().contains(input_i)) {
        continue;
      }
      const bNodeSocket &socket = node->output_socket(input_i);
      if (socket.runtime == nullptr || !socket.is_directly_linked()) {
        continue;
      }
      const SocketInContext socket_ctx{nullptr, &socket};
      if (this->is_socket_used(socket_ctx)) {
        return true;
      }
    }
    return false;
  }

  bool is_socket_used(const SocketInContext &socket)
  {
    if (!socket || socket.socket == nullptr || socket.socket->runtime == nullptr) {
      return false;
    }
    const std::optional<bool> is_used = all_socket_usages_.lookup_try(socket);
    if (is_used.has_value()) {
      return *is_used;
    }
    if (socket->is_output() && !socket->is_directly_linked()) {
      /* In this case we can return early because the socket can't be used if it's not linked. */
      return false;
    }
    if (socket->owner_tree().has_available_link_cycle()) {
      return false;
    }

    BLI_assert(usage_tasks_.is_empty());
    usage_tasks_.push(socket);

    while (!usage_tasks_.is_empty()) {
      const SocketInContext &socket = usage_tasks_.peek();
      this->usage_task(socket);
      if (&socket == &usage_tasks_.peek()) {
        /* The task is finished if it hasn't added any new task it depends on. */
        usage_tasks_.pop();
      }
    }

    return all_socket_usages_.lookup(socket);
  }

  InferenceValue get_socket_value(const SocketInContext &socket)
  {
    return value_inferencer_.get_socket_value(socket);
  }

  bool is_disabled_group_output(const int output_i)
  {
    const bNode *group_output_node = root_tree_.group_output_node();
    if (group_output_node == nullptr || group_output_node->runtime == nullptr) {
      return true;
    }
    if (!group_output_node->input_sockets().index_range().contains(output_i)) {
      return true;
    }
    const bNodeSocket &input = group_output_node->input_socket(output_i);
    if (input.runtime == nullptr) {
      return true;
    }
    const SocketInContext socket{nullptr, &input};
    return this->is_disabled_output(socket);
  }

  bool is_disabled_output(const SocketInContext &socket)
  {
    if (!socket || socket.socket == nullptr || socket.socket->runtime == nullptr) {
      return true;
    }
    const std::optional<bool> is_disabled = all_socket_disable_states_.lookup_try(socket);
    if (is_disabled.has_value()) {
      return *is_disabled;
    }
    if (socket->owner_tree().has_available_link_cycle()) {
      return true;
    }
    BLI_assert(disabled_output_tasks_.is_empty());
    disabled_output_tasks_.push(socket);

    while (!disabled_output_tasks_.is_empty()) {
      const SocketInContext &socket = disabled_output_tasks_.peek();
      this->disabled_output_task(socket);
      if (&socket == &disabled_output_tasks_.peek()) {
        /* The task is finished if it hasn't added any new task it depends on. */
        disabled_output_tasks_.pop();
      }
    }
    return all_socket_disable_states_.lookup(socket);
  }

 private:
  void usage_task(const SocketInContext &socket)
  {
    if (!socket || socket.socket == nullptr || socket.socket->runtime == nullptr) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    if (all_socket_usages_.contains(socket)) {
      return;
    }
    if (socket->runtime->owner_node == nullptr) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    const bNode &node = socket->owner_node();
    if (!socket->is_available()) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    if (node.is_undefined() && !node.is_custom_group()) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    if (socket->is_input()) {
      this->usage_task__input(socket);
    }
    else {
      this->usage_task__output(socket);
    }
  }

  void usage_task__input(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    if (!node) {
      all_socket_usages_.add_new(socket, false);
      return;
    }

    if (node->is_muted()) {
      const bool is_top_level = socket.context == nullptr;
      if (!this->ignore_top_level_node_muting_ || !is_top_level) {
        this->usage_task__input__muted_node(socket);
        return;
      }
    }

    switch (node->type_legacy) {
      case NODE_GROUP:
      case NODE_CUSTOM_GROUP: {
        this->usage_task__input__group_node(socket);
        break;
      }
      case NODE_GROUP_OUTPUT: {
        this->usage_task__input__group_output_node(socket);
        break;
      }
      case GEO_NODE_SWITCH: {
        this->usage_task__input__generic_switch(
            socket, switch_node_inference_utils::is_socket_selected__switch);
        break;
      }
      case GEO_NODE_INDEX_SWITCH: {
        this->usage_task__input__generic_switch(
            socket, switch_node_inference_utils::is_socket_selected__index_switch);
        break;
      }
      case GEO_NODE_MENU_SWITCH: {
        if (socket->index() == 0) {
          this->usage_task__input__fallback(socket);
        }
        else {
          this->usage_task__input__generic_switch(
              socket, switch_node_inference_utils::is_socket_selected__menu_switch);
        }
        break;
      }
      case SH_NODE_MIX: {
        this->usage_task__input__generic_switch(
            socket, switch_node_inference_utils::is_socket_selected__mix_node);
        break;
      }
      case SH_NODE_MIX_SHADER: {
        this->usage_task__input__generic_switch(
            socket, switch_node_inference_utils::is_socket_selected__shader_mix_node);
        break;
      }
      case GEO_NODE_SIMULATION_INPUT: {
        this->usage_task__input__simulation_input_node(socket);
        break;
      }
      case GEO_NODE_REPEAT_INPUT: {
        this->usage_task__input__repeat_input_node(socket);
        break;
      }
      case GEO_NODE_FOREACH_GEOMETRY_ELEMENT_INPUT: {
        this->usage_task__input__foreach_element_input_node(socket);
        break;
      }
      case GEO_NODE_FOREACH_GEOMETRY_ELEMENT_OUTPUT: {
        this->usage_task__input__foreach_element_output_node(socket);
        break;
      }
      case GEO_NODE_CAPTURE_ATTRIBUTE: {
        this->usage_task__input__capture_attribute_node(socket);
        break;
      }
      case GEO_NODE_STORE_NAMED_PORTAL:
      case SH_NODE_STORE_NAMED_PORTAL:
      case CMP_NODE_STORE_NAMED_PORTAL: {
        this->usage_task__input__store_named_portal(socket);
        break;
      }
      case GEO_NODE_WARNING: {
        this->usage_task__input__warning_node(socket);
        break;
      }
      case SH_NODE_OUTPUT_AOV:
      case SH_NODE_OUTPUT_LIGHT:
      case SH_NODE_OUTPUT_WORLD:
      case SH_NODE_OUTPUT_LINESTYLE:
      case SH_NODE_OUTPUT_MATERIAL:
      case CMP_NODE_OUTPUT_FILE:
      case TEX_NODE_OUTPUT: {
        this->usage_task__input__output_node(socket);
        break;
      }
      default: {
        if (node->is_type("NodeEnableInput"_ustr) ||
            node->is_type("NodeEnableOutput"_ustr))
        {
          this->usage_task__input__enable_value(socket);
          break;
        }
        if (node->is_type("NodeExpression"_ustr)) {
          this->usage_task__input__expression_node(socket);
          break;
        }
        this->usage_task__input__fallback(socket);
        break;
      }
    }
  }

  void usage_task__input__output_node(const SocketInContext &socket)
  {
    all_socket_usages_.add_new(socket, true);
  }

  /**
   * Assumes that the first input is a condition that selects one of the remaining inputs which is
   * then output. If necessary, this can trigger a value task for the condition socket.
   */
  void usage_task__input__generic_switch(
      const SocketInContext &socket,
      const FunctionRef<bool(const SocketInContext &socket, const InferenceValue &condition)>
          is_selected_socket)
  {
    const NodeInContext node = socket.owner_node();
    BLI_assert(node->input_sockets().size() >= 1);
    BLI_assert(node->output_sockets().size() >= 1);

    if (socket->type == SOCK_CUSTOM && STREQ(socket->idname, "NodeSocketVirtual")) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    const bNodeSocket *first_output = get_first_available_bsocket(node->output_sockets());
    if (first_output == nullptr) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    const SocketInContext output_socket{socket.context, first_output};
    const std::optional<bool> output_is_used = all_socket_usages_.lookup_try(output_socket);
    if (!output_is_used.has_value()) {
      this->push_usage_task(output_socket);
      return;
    }
    if (!*output_is_used) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    const bNodeSocket *first_input = get_first_available_bsocket(node->input_sockets());
    if (first_input == nullptr) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    const SocketInContext condition_socket{socket.context, first_input};
    if (socket == condition_socket) {
      all_socket_usages_.add_new(socket, true);
      return;
    }
    const InferenceValue condition_value = this->get_socket_value(condition_socket);
    if (condition_value.is_unknown()) {
      /* The exact condition value is unknown, so any input may be used. */
      all_socket_usages_.add_new(socket, true);
      return;
    }
    const bool is_used = is_selected_socket(socket, condition_value);
    all_socket_usages_.add_new(socket, is_used);
  }

  void usage_task__input__group_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    const bNodeTree *group = reinterpret_cast<const bNodeTree *>(node->id);
    if (!group || ID_MISSING(&group->id)) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    group->ensure_topology_cache();
    if (group->has_available_link_cycle()) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    if (group_call_reenters(socket.context, node->owner_tree(), *group)) {
      /* A recursive call is not statically unused. Do not walk into it again. */
      all_socket_usages_.add_new(socket, true);
      return;
    }

    /* The group node input is used if any of the matching group inputs within the group is
     * used. */
    const ComputeContext &group_context = compute_context_cache_.for_group_node(
        socket.context, node->identifier, &node->owner_tree());
    Vector<const bNodeSocket *> dependent_sockets;
    for (const bNode *group_input_node : group->group_input_nodes()) {
      const bNodeSocket &group_input_socket = group_input_node->output_socket(socket->index());
      if (group_input_socket.is_directly_linked()) {
        /* Skip unlinked group inputs to avoid further unnecessary processing of them further down
         * the line. */
        dependent_sockets.append(&group_input_socket);
      }
    }
    this->usage_task__with_dependent_sockets(socket, dependent_sockets, {}, &group_context);
  }

  void usage_task__input__expression_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    const NodeExpression &storage = *static_cast<const NodeExpression *>(node->storage);
    const int socket_i = socket->index();
    const bool is_expression_input = socket_i < storage.expression_items.items_num;
    const bool is_variable_input = socket_i > storage.expression_items.items_num &&
                                   socket_i < storage.expression_items.items_num + 1 +
                                                  storage.input_items.items_num;
    if (is_expression_input) {
      const int expr_i = socket_i;
      this->usage_task__with_dependent_sockets(
          socket, {&node->output_socket(expr_i)}, {}, node.context);
      return;
    }
    if (is_variable_input) {
      const int variable_i = socket_i - storage.expression_items.items_num - 1;
      Vector<SocketInContext> dependent_sockets;
      for (const int expr_i : IndexRange(storage.expression_items.items_num)) {
        const SocketInContext expression_socket = node.input_socket(expr_i);
        const InferenceValue value = this->get_socket_value(expression_socket);
        const std::optional<std::string> expression = value.get_if_primitive<std::string>();
        if (!expression.has_value()) {
          this->usage_task__input__any_output(socket);
          return;
        }
        const SocketInContext result_output_socket = node.output_socket(expr_i);
        expression::ExpressionNodeGroup &group = *expression_node_groups_cache_.lookup_or_add_cb(
            result_output_socket, [&]() {
              return expression::expression_node_to_group(
                  nullptr, *node, root_tree_.idname, {*expression}, {expr_i});
            });
        if (!group.tree) {
          continue;
        }
        group.tree->ensure_interface_cache();
        group.tree->ensure_topology_cache();
        const ComputeContext &group_context = compute_context_cache_.for_expression_node_output(
            node.context, node->identifier, expr_i, &node->owner_tree());
        for (const bNode *group_input_node : group.tree->group_input_nodes()) {
          const bNodeSocket &group_input_socket = group_input_node->output_socket(variable_i);
          if (group_input_socket.is_directly_linked()) {
            dependent_sockets.append({&group_context, &group_input_socket});
          }
        }
      }
      this->usage_task__with_dependent_sockets(socket, dependent_sockets);
      return;
    }
    /* This is for the extent sockets. */
    all_socket_usages_.add_new(socket, false);
  }

  void usage_task__input__group_output_node(const SocketInContext &socket)
  {
    const int output_i = socket->index();
    if (socket.context == nullptr) {
      /* This is a final output which is always used. */
      all_socket_usages_.add_new(socket, true);
      return;
    }
    /* The group output node is used if the matching output of the parent group node is used. */
    if (const auto *group_context = dynamic_cast<const bke::GroupNodeComputeContext *>(
            socket.context))
    {
      const bNodeSocket &group_node_output = group_context->node()->output_socket(output_i);
      this->usage_task__with_dependent_sockets(
          socket, {&group_node_output}, {}, group_context->parent());
      return;
    }
    if (const auto *expression_output_context =
            dynamic_cast<const bke::ExpressionNodeOutputComputeContext *>(socket.context))
    {
      const bNodeSocket &expression_node_output = expression_output_context->node()->output_socket(
          expression_output_context->output_index());
      this->usage_task__with_dependent_sockets(socket, {&expression_node_output}, {}, nullptr);
      return;
    }
    BLI_assert_unreachable();
    all_socket_usages_.add_new(socket, false);
  }

  void usage_task__output(const SocketInContext &socket)
  {
    /* An output socket is used if any of the sockets it is connected to is used. */
    Vector<const bNodeSocket *> dependent_sockets;
    for (const bNodeLink *link : socket->directly_linked_links()) {
      if (link->is_used()) {
        dependent_sockets.append(link->tosock);
      }
    }
    this->usage_task__with_dependent_sockets(socket, dependent_sockets, {}, socket.context);
  }

  void usage_task__input__simulation_input_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    const bNodeTree &tree = socket->owner_tree();

    const NodeGeometrySimulationInput &storage = *static_cast<const NodeGeometrySimulationInput *>(
        node->storage);
    const bNode *sim_output_node = tree.node_by_id(storage.output_node_id);
    if (!sim_output_node) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    /* Simulation inputs are also used when any of the simulation outputs are used. */
    Vector<const bNodeSocket *, 16> dependent_sockets;
    dependent_sockets.extend(node->output_sockets());
    dependent_sockets.extend(sim_output_node->output_sockets());
    this->usage_task__with_dependent_sockets(socket, dependent_sockets, {}, socket.context);
  }

  void usage_task__input__repeat_input_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    const bNodeTree &tree = socket->owner_tree();

    const NodeGeometryRepeatInput &storage = *static_cast<const NodeGeometryRepeatInput *>(
        node->storage);
    const bNode *repeat_output_node = tree.node_by_id(storage.output_node_id);
    if (!repeat_output_node) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    /* Assume that all repeat inputs are used when any of the outputs are used. This check could
     * become more precise in the future if necessary. */
    Vector<const bNodeSocket *, 16> dependent_sockets;
    dependent_sockets.extend(node->output_sockets());
    dependent_sockets.extend(repeat_output_node->output_sockets());
    this->usage_task__with_dependent_sockets(socket, dependent_sockets, {}, socket.context);
  }

  void usage_task__input__foreach_element_output_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    this->usage_task__with_dependent_sockets(
        socket, {node->output_by_identifier(socket->identifier_ustr())}, {}, socket.context);
  }

  void usage_task__input__capture_attribute_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    this->usage_task__with_dependent_sockets(
        socket, {&node->output_socket(socket->index())}, {}, socket.context);
  }

  void usage_task__input__store_named_portal(const SocketInContext &socket)
  {
    const NodeInContext store_node = socket.owner_node();
    const bNodeTree &tree = socket->owner_tree();

    Vector<const bNodeSocket *> dependent_sockets;
    dependent_sockets.extend(store_node->output_sockets());

    if (store_node->storage) {
      const StringRef portal_name =
          static_cast<const NodeGeometryPortal *>(store_node->storage)->name;
      if (!portal_name.is_empty()) {
        int matching_store_count = 0;
        for (const bNode *node : tree.all_nodes()) {
          if (!bke::node_is_store_named_portal(*node) || !node->storage) {
            continue;
          }
          const StringRef other_name =
              static_cast<const NodeGeometryPortal *>(node->storage)->name;
          if (other_name == portal_name) {
            matching_store_count++;
          }
        }

        /* A hidden binding exists only when the Store name is unique in this direct tree. */
        if (matching_store_count == 1) {
          for (const bNode *node : tree.all_nodes()) {
            if (!bke::node_is_named_portal(*node) || !node->storage) {
              continue;
            }
            const StringRef reader_name =
                static_cast<const NodeGeometryPortal *>(node->storage)->name;
            if (reader_name == portal_name) {
              dependent_sockets.extend(node->output_sockets());
            }
          }
        }
      }
    }

    this->usage_task__with_dependent_sockets(
        socket, dependent_sockets, {}, socket.context);
  }

  void usage_task__input__warning_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    const SocketInContext show_input_socket = node.input_socket(0);
    const SocketInContext message_input_socket = node.input_socket(1);
    const SocketInContext show_output_socket = node.output_socket(0);
    if (show_output_socket->is_directly_linked()) {
      if (socket == show_input_socket) {
        this->usage_task__with_dependent_sockets(
            show_input_socket, {&*show_output_socket}, {}, socket.context);
      }
      if (socket == message_input_socket) {
        this->usage_task__with_dependent_sockets(
            message_input_socket, {&*show_output_socket}, {&*show_input_socket}, socket.context);
      }
      return;
    }
    const ComputeContext *context = node.context;
    const bool is_top_level = context == nullptr;
    if (is_top_level) {
      if (socket == show_input_socket) {
        /* At the top level, the show input is always used. */
        all_socket_usages_.add_new(socket, true);
      }
      if (socket == message_input_socket) {
        /* The message input is used if the show input is true. */
        const InferenceValue show_inference_value = this->get_socket_value(show_input_socket);
        const bool show_value = show_inference_value.get_if_primitive<bool>().value_or(true);
        all_socket_usages_.add_new(socket, show_value);
      }
      return;
    }
    const bool is_in_zone = !dynamic_cast<const bke::GroupNodeComputeContext *>(context);
    if (is_in_zone) {
      /* Warning nodes where the output is not linked must not be in a zone. */
      all_socket_usages_.add_new(socket, false);
      return;
    }
    const bNode *output_node = node->owner_tree().group_output_node();
    if (!output_node) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    const Span<const bNodeSocket *> group_output_sockets = output_node->input_sockets().drop_back(
        1);
    /* The warning is used if any of the group outputs is used. */
    if (socket == show_input_socket) {
      this->usage_task__with_dependent_sockets(
          show_input_socket, group_output_sockets, {}, context);
    }
    if (socket == message_input_socket) {
      this->usage_task__with_dependent_sockets(
          message_input_socket, group_output_sockets, {&*show_input_socket}, context);
    }
  }

  void usage_task__input__enable_value(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    const SocketInContext enable_socket = node.input_socket(0);
    const SocketInContext output_socket = node.output_socket(0);
    if (socket == enable_socket) {
      this->usage_task__with_dependent_sockets(socket, {&*output_socket}, {}, socket.context);
    }
    else {
      this->usage_task__with_dependent_sockets(
          socket, {&*output_socket}, {&*enable_socket}, socket.context);
    }
  }

  void usage_task__input__fallback(const SocketInContext &socket)
  {
    const SocketDeclaration *socket_decl = socket->runtime->declaration;
    if (!socket_decl) {
      all_socket_usages_.add_new(socket, true);
      return;
    }
    if (!socket_decl->usage_inference_fn) {
      this->usage_task__input__any_output(socket);
      return;
    }
    SocketUsageParams params{
        *owner_, socket.context, socket->owner_tree(), socket->owner_node(), *socket};
    const std::optional<bool> is_used = (*socket_decl->usage_inference_fn)(params);
    if (!is_used.has_value()) {
      /* Some value was requested, come back later when that value is available. */
      return;
    }
    all_socket_usages_.add_new(socket, *is_used);
  }

  void usage_task__input__foreach_element_input_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    const bNodeTree &tree = socket->owner_tree();

    const NodeGeometryForeachGeometryElementInput &storage =
        *static_cast<const NodeGeometryForeachGeometryElementInput *>(node->storage);
    const bNode *foreach_output_node = tree.node_by_id(storage.output_node_id);
    if (!foreach_output_node) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    Vector<const bNodeSocket *, 16> dependent_sockets;
    if (StringRef(socket->identifier).startswith("Input_")) {
      dependent_sockets.append(node->output_by_identifier(socket->identifier_ustr()));
    }
    else {
      /* The geometry and selection inputs are used whenever any of the zone outputs is used. */
      dependent_sockets.extend(node->output_sockets());
      dependent_sockets.extend(foreach_output_node->output_sockets());
    }
    this->usage_task__with_dependent_sockets(socket, dependent_sockets, {}, socket.context);
  }

  void usage_task__input__muted_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    Vector<const bNodeSocket *> dependent_sockets;
    for (const bNodeInternalLink &internal_link : node->internal_links()) {
      if (internal_link.in != socket.socket) {
        continue;
      }
      dependent_sockets.append(internal_link.out);
    }
    this->usage_task__with_dependent_sockets(socket, dependent_sockets, {}, socket.context);
  }

  void usage_task__input__any_output(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    this->usage_task__with_dependent_sockets(socket, node->output_sockets(), {}, node.context);
  }

  /**
   * Utility that handles simple cases where a socket is used if any of its dependent sockets is
   * used and all of the boolean condition inputs are true.
   */
  void usage_task__with_dependent_sockets(const SocketInContext &socket,
                                          const Span<const bNodeSocket *> dependent_sockets,
                                          const Span<const bNodeSocket *> condition_inputs,
                                          const ComputeContext *dependent_socket_context)
  {
    /* Check if any of the dependent outputs are used. */
    SocketInContext next_unknown_socket;
    bool any_output_used = false;
    for (const bNodeSocket *dependent_socket_ptr : dependent_sockets) {
      if (dependent_socket_ptr == nullptr || dependent_socket_ptr->runtime == nullptr) {
        continue;
      }
      const SocketInContext dependent_socket{dependent_socket_context, dependent_socket_ptr};
      const std::optional<bool> is_used = all_socket_usages_.lookup_try(dependent_socket);
      if (!is_used.has_value()) {
        if (dependent_socket_ptr->is_output() && !dependent_socket_ptr->is_directly_linked()) {
          continue;
        }
        if (!next_unknown_socket) {
          next_unknown_socket = dependent_socket;
          continue;
        }
      }
      if (is_used.value_or(false)) {
        any_output_used = true;
        break;
      }
    }
    if (next_unknown_socket) {
      /* Create a task that checks if the next dependent socket is used. Intentionally only create
       * a task for the very next one and not for all, because that could potentially trigger a lot
       * of unnecessary evaluations. */
      this->push_usage_task(next_unknown_socket);
      return;
    }
    if (!any_output_used) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    bool all_condition_inputs_true = true;
    for (const bNodeSocket *condition_input_ptr : condition_inputs) {
      const SocketInContext condition_input{dependent_socket_context, condition_input_ptr};
      const InferenceValue condition_value = this->get_socket_value(condition_input);
      if (!condition_value.is_primitive_value()) {
        /* The condition is not known, so it may be true. */
        continue;
      }
      BLI_assert(condition_input_ptr->type == SOCK_BOOLEAN);
      if (!condition_value.get_primitive<bool>()) {
        all_condition_inputs_true = false;
        break;
      }
    }
    all_socket_usages_.add_new(socket, all_condition_inputs_true);
  }

  void usage_task__with_dependent_sockets(const SocketInContext &socket,
                                          const Span<SocketInContext> dependent_outputs)
  {
    /* Check if any of the dependent outputs are used. */
    SocketInContext next_unknown_socket;
    bool any_output_used = false;
    for (const SocketInContext dependent_socket : dependent_outputs) {
      if (!dependent_socket || dependent_socket.socket == nullptr ||
          dependent_socket.socket->runtime == nullptr)
      {
        continue;
      }
      const std::optional<bool> is_used = all_socket_usages_.lookup_try(dependent_socket);
      if (!is_used.has_value()) {
        if (dependent_socket->is_output() && !dependent_socket->is_directly_linked()) {
          continue;
        }
        if (!next_unknown_socket) {
          next_unknown_socket = dependent_socket;
          continue;
        }
      }
      if (is_used.value_or(false)) {
        any_output_used = true;
        break;
      }
    }
    if (next_unknown_socket) {
      /* Create a task that checks if the next dependent socket is used. Intentionally only create
       * a task for the very next one and not for all, because that could potentially trigger a lot
       * of unnecessary evaluations. */
      this->push_usage_task(next_unknown_socket);
      return;
    }
    if (!any_output_used) {
      all_socket_usages_.add_new(socket, false);
      return;
    }
    all_socket_usages_.add_new(socket, true);
  }

  void push_usage_task(const SocketInContext &socket)
  {
    usage_tasks_.push(socket);
  }

  void disabled_output_task(const SocketInContext &socket)
  {
    if (all_socket_disable_states_.contains(socket)) {
      return;
    }
    const bNode &node = socket->owner_node();
    if (!socket->is_available()) {
      all_socket_disable_states_.add_new(socket, true);
      return;
    }
    if (node.is_undefined() && !node.is_custom_group()) {
      all_socket_disable_states_.add_new(socket, true);
      return;
    }
    if (socket->is_input()) {
      this->disabled_output_task__input(socket);
    }
    else {
      this->disabled_output_task__output(socket);
    }
  }

  void disabled_output_task__input(const SocketInContext &socket)
  {
    const Span<const bNodeLink *> links = socket->directly_linked_links();
    const bNodeLink *single_link = links.size() == 1 && links[0]->is_used() ? links[0] : nullptr;
    if (links.size() != 1 || !links[0]->is_used()) {
      /* The socket is not linked, thus it is not disabled. */
      all_socket_disable_states_.add_new(socket, false);
      return;
    }
    const SocketInContext origin_socket{socket.context, single_link->fromsock};
    this->disabled_output_task__with_origin_socket(socket, origin_socket);
  }

  void disabled_output_task__output(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    if (node->is_muted()) {
      const bool is_top_level = socket.context == nullptr;
      if (!this->ignore_top_level_node_muting_ || !is_top_level) {
        this->disabled_output_task__output__muted_node(socket);
        return;
      }
    }

    switch (node->type_legacy) {
      case NODE_GROUP:
      case NODE_CUSTOM_GROUP: {
        this->disabled_output_task__output__group_node(socket);
        break;
      }
      case NODE_REROUTE: {
        this->disabled_output_task__with_origin_socket(socket, node.input_socket(0));
        break;
      }
      case GEO_NODE_SWITCH: {
        this->disabled_output_task__output__generic_switch(
            socket, switch_node_inference_utils::is_socket_selected__switch);
        break;
      }
      case GEO_NODE_INDEX_SWITCH: {
        this->disabled_output_task__output__generic_switch(
            socket, switch_node_inference_utils::is_socket_selected__index_switch);
        break;
      }
      case GEO_NODE_MENU_SWITCH: {
        this->disabled_output_task__output__generic_switch(
            socket, switch_node_inference_utils::is_socket_selected__menu_switch);
        break;
      }
      default: {
        if (node->is_type("NodeEnableOutput"_ustr)) {
          this->disabled_output_task__output__enable_output_node(socket);
          break;
        }

        const SocketDeclaration *socket_declaration = socket->runtime->declaration;
        if (socket_declaration && socket_declaration->usage_inference_fn) {
          SocketUsageParams params{
              *owner_, socket.context, socket->owner_tree(), socket->owner_node(), *socket};
          const std::optional<bool> is_used = (*socket_declaration->usage_inference_fn)(params);
          if (!is_used.has_value()) {
            /* Some value was requested, come back later when that value is available. */
            return;
          }
          if (!*is_used) {
            all_socket_disable_states_.add_new(socket, true);
            break;
          }
        }

        /* By default, all output sockets are enabled unless they are explicitly disabled by some
         * rule above. */
        all_socket_disable_states_.add_new(socket, false);
        break;
      }
    }
  }

  void disabled_output_task__output__muted_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    for (const bNodeInternalLink &internal_link : node->internal_links()) {
      if (internal_link.out != socket.socket) {
        continue;
      }
      this->disabled_output_task__with_origin_socket(socket, {socket.context, internal_link.in});
      return;
    }
    all_socket_disable_states_.add_new(socket, false);
  }

  void disabled_output_task__output__group_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    const bNodeTree *group = reinterpret_cast<const bNodeTree *>(node->id);
    if (!group || ID_MISSING(&group->id)) {
      all_socket_disable_states_.add_new(socket, false);
      return;
    }
    group->ensure_topology_cache();
    if (group->has_available_link_cycle()) {
      all_socket_disable_states_.add_new(socket, false);
      return;
    }
    if (group_call_reenters(socket.context, node->owner_tree(), *group)) {
      all_socket_disable_states_.add_new(socket, false);
      return;
    }
    const bNode *group_output_node = group->group_output_node();
    if (!group_output_node) {
      all_socket_disable_states_.add_new(socket, false);
      return;
    }
    const ComputeContext &group_context = compute_context_cache_.for_group_node(
        socket.context, node->identifier, &node->owner_tree());
    const SocketInContext origin_socket{&group_context,
                                        &group_output_node->input_socket(socket->index())};
    this->disabled_output_task__with_origin_socket(socket, origin_socket);
  }

  void disabled_output_task__output__generic_switch(
      const SocketInContext &socket,
      const FunctionRef<bool(const SocketInContext &socket, const InferenceValue &condition)>
          is_selected_socket)
  {
    const NodeInContext node = socket.owner_node();
    BLI_assert(node->input_sockets().size() >= 1);
    BLI_assert(node->output_sockets().size() >= 1);

    if (socket->type == SOCK_CUSTOM && STREQ(socket->idname, "NodeSocketVirtual")) {
      all_socket_disable_states_.add_new(socket, false);
      return;
    }
    const SocketInContext condition_socket{socket.context,
                                           get_first_available_bsocket(node->input_sockets())};
    const InferenceValue condition_value = this->get_socket_value(condition_socket);
    if (condition_value.is_unknown()) {
      all_socket_disable_states_.add_new(socket, false);
      return;
    }
    for (const bNodeSocket *input_socket : node->input_sockets()) {
      if (!input_socket->is_available()) {
        continue;
      }
      if (input_socket->type == SOCK_CUSTOM && STREQ(input_socket->idname, "NodeSocketVirtual")) {
        continue;
      }
      if (input_socket == condition_socket.socket) {
        continue;
      }
      const SocketInContext origin_socket{socket.context, input_socket};
      const bool is_selected = is_selected_socket(origin_socket, condition_value);
      if (is_selected) {
        this->disabled_output_task__with_origin_socket(socket, origin_socket);
        return;
      }
    }
    all_socket_disable_states_.add_new(socket, false);
  }

  void disabled_output_task__output__enable_output_node(const SocketInContext &socket)
  {
    const NodeInContext node = socket.owner_node();
    const SocketInContext enable_socket = node.input_socket(0);
    const InferenceValue enable_value = this->get_socket_value(enable_socket);
    const std::optional<bool> is_enabled_opt = enable_value.get_if_primitive<bool>();
    const bool is_enabled = is_enabled_opt.value_or(true);
    all_socket_disable_states_.add_new(socket, !is_enabled);
  }

  void disabled_output_task__with_origin_socket(const SocketInContext &socket,
                                                const SocketInContext &origin_socket)
  {
    const std::optional<bool> is_disabled = all_socket_disable_states_.lookup_try(origin_socket);
    if (is_disabled.has_value()) {
      all_socket_disable_states_.add_new(socket, *is_disabled);
      return;
    }
    this->push_disabled_output_task(origin_socket);
  }

  void push_disabled_output_task(const SocketInContext &socket)
  {
    disabled_output_tasks_.push(socket);
  }

  static const bNodeSocket *get_first_available_bsocket(const Span<const bNodeSocket *> sockets)
  {
    for (const bNodeSocket *socket : sockets) {
      if (socket && socket->is_available()) {
        return socket;
      }
    }
    return nullptr;
  }
};

SocketUsageInferencer::SocketUsageInferencer(const bNodeTree &tree,
                                             ResourceScope &scope,
                                             SocketValueInferencer &value_inferencer,
                                             bke::ComputeContextCache &compute_context_cache,
                                             const bool ignore_top_level_node_muting)
    : impl_(scope.construct<SocketUsageInferencerImpl>(
          tree, value_inferencer, compute_context_cache, ignore_top_level_node_muting))
{
  impl_.owner_ = this;
}

static bool interface_input_may_affect_visibility(const bNodeTree &tree, const int input_i);

static bool input_may_affect_visibility(const bNodeSocket &socket)
{
  if (socket.type == SOCK_MENU) {
    return true;
  }
  if (socket.type != SOCK_BOOLEAN || !socket.is_input()) {
    return false;
  }

  const bNode &node = socket.owner_node();
  if (socket.identifier_ustr() == "Enable"_ustr &&
      (node.is_type("NodeEnableInput"_ustr) || node.is_type("NodeEnableOutput"_ustr)))
  {
    return true;
  }

  if (ELEM(node.type_legacy, NODE_GROUP, NODE_CUSTOM_GROUP)) {
    const bNodeTree *group = reinterpret_cast<const bNodeTree *>(node.id);
    if (group && !ID_MISSING(&group->id)) {
      group->ensure_topology_cache();
      group->ensure_interface_cache();
      if (group->interface_inputs().index_range().contains(socket.index())) {
        return interface_input_may_affect_visibility(*group, socket.index());
      }
    }
  }
  return false;
}

static const bNodeSocket *interface_input_visibility_socket(const bNodeTree &tree,
                                                           const int input_i)
{
  tree.ensure_topology_cache();
  tree.ensure_interface_cache();
  for (const bNode *group_input_node : tree.group_input_nodes()) {
    if (group_input_node == nullptr || group_input_node->runtime == nullptr) {
      continue;
    }
    if (!group_input_node->output_sockets().index_range().contains(input_i)) {
      continue;
    }
    const bNodeSocket *interface_output = &group_input_node->output_socket(input_i);
    for (const bNodeLink *link : tree.all_links()) {
      if (link == nullptr || link->fromsock != interface_output || link->tonode == nullptr ||
          link->tosock == nullptr || link->tosock->runtime == nullptr ||
          !link->tonode->is_type("NodeEnableInput"_ustr) ||
          link->tosock->identifier_ustr() != "Value"_ustr)
      {
        continue;
      }
      return link->tonode->input_by_identifier("Enable"_ustr);
    }
  }
  return nullptr;
}

static std::optional<int> interface_input_visibility_controller_index(const bNodeTree &tree,
                                                                      const int input_i)
{
  const bNodeSocket *visibility_socket = interface_input_visibility_socket(tree, input_i);
  if (!visibility_socket) {
    return std::nullopt;
  }
  for (const bNodeLink *link : visibility_socket->directly_linked_links()) {
    if (link->tosock != visibility_socket || !link->fromnode->is_group_input()) {
      continue;
    }
    return tree.interface_input_index_by_identifier(link->fromsock->identifier);
  }
  return std::nullopt;
}

/**
 * Decide whether an Enable-Input-controlled group interface socket must be fully hidden.
 *
 * Enable=false must hide the interface completely (not merely gray it out).
 *
 * \param get_linked_enable_value: Called when Enable is linked; should return true to hide,
 *   false to show, or nullopt when Enable is not a static boolean.
 *
 * Returns nullopt when the input is not controlled by Enable Input.
 */
static std::optional<bool> enable_input_should_hide_interface(
    const bNodeTree &group,
    const int input_i,
    const FunctionRef<std::optional<bool>()> get_linked_enable_value)
{
  const bNodeSocket *visibility_socket = interface_input_visibility_socket(group, input_i);
  if (!visibility_socket) {
    return std::nullopt;
  }

  if (!visibility_socket->is_directly_linked()) {
    /* Constant Enable on the Enable Input node. */
    return !visibility_socket->default_value_typed<bNodeSocketValueBoolean>()->value;
  }

  return get_linked_enable_value();
}

static bool interface_input_may_affect_visibility(const bNodeTree &tree, const int input_i)
{
  const bNodeTreeInterfaceSocket &socket = *tree.interface_inputs()[input_i];
  if (socket.socket_type == StringRef("NodeSocketMenu")) {
    return true;
  }
  if (socket.socket_type != StringRef("NodeSocketBool")) {
    return false;
  }
  for (const bNode *group_input_node : tree.group_input_nodes()) {
    for (const bNodeSocket *target_socket :
         group_input_node->output_socket(input_i).logically_linked_sockets())
    {
      if (input_may_affect_visibility(*target_socket)) {
        return true;
      }
    }
  }
  return false;
}

Array<SocketUsage> infer_all_sockets_usage(const bNodeTree &tree)
{
  tree.ensure_topology_cache();
  const Span<const bNodeSocket *> all_input_sockets = tree.all_input_sockets();
  const Span<const bNodeSocket *> all_output_sockets = tree.all_output_sockets();
  Array<SocketUsage> all_usages(tree.all_sockets().size());

  if (tree.has_available_link_cycle()) {
    return all_usages;
  }

  ResourceScope scope;
  bke::ComputeContextCache compute_context_cache;

  const bool ignore_top_level_node_muting = true;

  {
    /* Find actual socket usages. */
    SocketValueInferencer value_inferencer{tree, scope, compute_context_cache};
    SocketUsageInferencer usage_inferencer{
        tree, scope, value_inferencer, compute_context_cache, ignore_top_level_node_muting};
    usage_inferencer.mark_top_level_node_outputs_as_used();
    for (const bNodeSocket *socket : all_input_sockets) {
      if (socket == nullptr || socket->runtime == nullptr) {
        continue;
      }
      const int index = socket->index_in_tree();
      if (!all_usages.index_range().contains(index)) {
        continue;
      }
      all_usages[index].is_used = usage_inferencer.is_socket_used({nullptr, socket});
    }
  }

  /* Find input sockets that should be hidden. */
  Array<bool> only_controllers_used(all_input_sockets.size(), NoInitialization{});
  Array<bool> all_ignored_inputs(all_input_sockets.size(), true);
  threading::parallel_for(all_input_sockets.index_range(), 1024, [&](const IndexRange range) {
    for (const int i : range) {
      const bNodeSocket *socket = all_input_sockets[i];
      if (socket == nullptr || socket->runtime == nullptr) {
        only_controllers_used[i] = true;
        continue;
      }
      only_controllers_used[i] = !input_may_affect_visibility(*socket);
    }
  });
  SocketValueInferencer value_inferencer_all_unknown{
      tree, scope, compute_context_cache, nullptr, all_ignored_inputs};
  SocketUsageInferencer usage_inferencer_all_unknown{tree,
                                                     scope,
                                                     value_inferencer_all_unknown,
                                                     compute_context_cache,
                                                     ignore_top_level_node_muting};
  SocketValueInferencer value_inferencer_only_controllers{
      tree, scope, compute_context_cache, nullptr, only_controllers_used};
  SocketUsageInferencer usage_inferencer_only_controllers{tree,
                                                          scope,
                                                          value_inferencer_only_controllers,
                                                          compute_context_cache,
                                                          ignore_top_level_node_muting};
  usage_inferencer_all_unknown.mark_top_level_node_outputs_as_used();
  usage_inferencer_only_controllers.mark_top_level_node_outputs_as_used();
  for (const bNodeSocket *socket : all_input_sockets) {
    if (socket == nullptr || socket->runtime == nullptr) {
      continue;
    }
    const int usage_i = socket->index_in_tree();
    if (!all_usages.index_range().contains(usage_i)) {
      continue;
    }
    SocketUsage &usage = all_usages[usage_i];

    /* A group-node input controlled by Enable Input must disappear completely, independently of
     * whether the value still participates in evaluation. Constant Enable=false used to fall
     * through to the generic "never used" path, which only greys the socket — hide it instead. */
    const bNode &node = socket->owner_node();
    if (ELEM(node.type_legacy, NODE_GROUP, NODE_CUSTOM_GROUP)) {
      const bNodeTree *group = reinterpret_cast<const bNodeTree *>(node.id);
      if (group && !ID_MISSING(&group->id) &&
          group->interface_inputs().index_range().contains(socket->index()))
      {
        const std::optional<bool> hide = enable_input_should_hide_interface(
            *group, socket->index(), [&]() -> std::optional<bool> {
              /* Prefer the parent group-node controller socket when Enable is exposed as a
               * group input; otherwise no static decision from the parent tree alone. */
              const std::optional<int> controller_i =
                  interface_input_visibility_controller_index(*group, socket->index());
              if (!controller_i ||
                  !node.input_sockets().index_range().contains(*controller_i))
              {
                return std::nullopt;
              }
              const bNodeSocket &controller_socket = node.input_socket(*controller_i);
              if (controller_socket.runtime == nullptr) {
                return std::nullopt;
              }
              if (!controller_socket.is_directly_linked()) {
                return !controller_socket.default_value_typed<bNodeSocketValueBoolean>()->value;
              }
              SocketInContext controller_ctx{nullptr, &controller_socket};
              const InferenceValue controller_value =
                  value_inferencer_only_controllers.get_socket_value(controller_ctx);
              if (controller_value.is_primitive_value()) {
                return !controller_value.get_primitive<bool>();
              }
              return std::nullopt;
            });
        if (hide.has_value()) {
          if (*hide) {
            usage.is_visible = false;
            usage.is_used = false;
          }
          /* Controlled by Enable Input: do not apply generic unused-dependent hiding. */
          continue;
        }
      }
    }

    if (usage.is_used) {
      /* Used inputs are always visible. */
      continue;
    }
    const SocketInContext socket_ctx{nullptr, socket};
    if (usage_inferencer_only_controllers.is_socket_used(socket_ctx)) {
      /* The input should be visible if it's used if only visibility-controlling inputs are
       * considered. */
      continue;
    }
    if (!usage_inferencer_all_unknown.is_socket_used(socket_ctx)) {
      /* The input should be visible if it's never used, regardless of any inputs. Its usage does
       * not depend on any visibility-controlling input. */
      continue;
    }
    usage.is_visible = false;
  }
  for (const bNodeSocket *socket : all_output_sockets) {
    if (socket == nullptr || socket->runtime == nullptr) {
      continue;
    }
    const int usage_i = socket->index_in_tree();
    if (!all_usages.index_range().contains(usage_i)) {
      continue;
    }
    /* Only hide outputs that are explicitly disabled by Enable Output (and similar) rules.
     * Do not hide Group Input sockets in the node editor: the group interface is controlled
     * separately via #infer_group_interface_usage when drawing group nodes and modifiers. */
    const SocketInContext socket_ctx{nullptr, socket};
    if (usage_inferencer_only_controllers.is_disabled_output(socket_ctx)) {
      SocketUsage &usage = all_usages[usage_i];
      usage.is_visible = false;
    }
  }

  return all_usages;
}

void infer_group_interface_usage(const bNodeTree &group,
                                 const Span<InferenceValue> group_input_values,
                                 const MutableSpan<SocketUsage> r_input_usages,
                                 const std::optional<MutableSpan<SocketUsage>> r_output_usages)
{
  SocketUsage default_usage;
  default_usage.is_used = false;
  default_usage.is_visible = true;
  r_input_usages.fill(default_usage);
  if (r_output_usages) {
    r_output_usages->fill({true, true});
  }

  ResourceScope scope;
  bke::ComputeContextCache compute_context_cache;

  {
    /* Detect actually used inputs. */
    const auto get_input_value = [&](const int group_input_i) {
      return group_input_values[group_input_i];
    };
    SocketValueInferencer value_inferencer{group, scope, compute_context_cache, get_input_value};
    SocketUsageInferencer usage_inferencer{group, scope, value_inferencer, compute_context_cache};
    for (const int i : group.interface_inputs().index_range()) {
      r_input_usages[i].is_used |= usage_inferencer.is_group_input_used(i);
    }
  }
  bool visibility_control_exists = false;
  for (const int i : group.interface_inputs().index_range()) {
    if (interface_input_may_affect_visibility(group, i) ||
        interface_input_visibility_socket(group, i) != nullptr)
    {
      visibility_control_exists = true;
    }
  }
  if (!visibility_control_exists) {
    /* If there are no visibility controls, all inputs are always visible. */
    return;
  }
  SocketValueInferencer value_inferencer_all_unknown{group, scope, compute_context_cache};
  SocketUsageInferencer usage_inferencer_all_unknown{
      group, scope, value_inferencer_all_unknown, compute_context_cache};
  const auto get_only_controllers_input_value = [&](const int group_input_i) {
    if (interface_input_may_affect_visibility(group, group_input_i)) {
      return group_input_values[group_input_i];
    }
    return InferenceValue::Unknown();
  };
  SocketValueInferencer value_inferencer_only_controllers{
      group, scope, compute_context_cache, get_only_controllers_input_value};
  SocketUsageInferencer usage_inferencer_only_controllers{
      group, scope, value_inferencer_only_controllers, compute_context_cache};
  for (const int i : group.interface_inputs().index_range()) {
    /* Enable Input controls visibility independently of whether its Value participates in
     * evaluation. When hidden, external wires are ignored at evaluation time and the group
     * interface default is used; the Enable Input node still pass-throughs that value. */
    const std::optional<bool> hide = enable_input_should_hide_interface(
        group, i, [&]() -> std::optional<bool> {
          /* Resolve linked Enable via the group's value inference. */
          if (const bNodeSocket *visibility_socket = interface_input_visibility_socket(group, i))
          {
            const InferenceValue controller_value =
                value_inferencer_only_controllers.get_socket_value(
                    SocketInContext{nullptr, visibility_socket});
            if (controller_value.is_primitive_value()) {
              return !controller_value.get_primitive<bool>();
            }
          }
          return std::nullopt;
        });
    if (hide.has_value()) {
      if (*hide) {
        r_input_usages[i].is_visible = false;
        r_input_usages[i].is_used = false;
      }
      continue;
    }
    if (r_input_usages[i].is_used) {
      /* Used inputs are always visible. */
      continue;
    }
    if (usage_inferencer_only_controllers.is_group_input_used(i)) {
      /* The input should be visible if it's used if only visibility-controlling inputs are
       * considered. */
      continue;
    }
    if (!usage_inferencer_all_unknown.is_group_input_used(i)) {
      /* The input should be visible if it's never used, regardless of any inputs. Its usage does
       * not depend on any visibility-controlling input. */
      continue;
    }
    r_input_usages[i].is_visible = false;
  }
  if (r_output_usages) {
    for (const int i : group.interface_outputs().index_range()) {
      if (usage_inferencer_only_controllers.is_disabled_group_output(i)) {
        SocketUsage &usage = (*r_output_usages)[i];
        usage.is_used = false;
        usage.is_visible = false;
      }
    }
  }
}

void infer_group_interface_inputs_usage(const bNodeTree &group,
                                        Span<const bNodeSocket *> input_sockets,
                                        MutableSpan<SocketUsage> r_input_usages)
{
  BLI_assert(group.interface_inputs().size() == input_sockets.size());

  AlignedBuffer<1024, 8> allocator_buffer;
  ResourceScope scope(allocator_buffer);

  Array<InferenceValue> input_values(input_sockets.size(), InferenceValue::Unknown());
  for (const int i : input_sockets.index_range()) {
    const bNodeSocket &socket = *input_sockets[i];
    if (socket.is_directly_linked()) {
      continue;
    }

    const bke::bNodeSocketType &stype = *socket.typeinfo;
    const CPPType *base_type = stype.base_cpp_type;
    if (base_type == nullptr) {
      continue;
    }
    void *value = scope.allocate_owned(*base_type);
    stype.get_base_cpp_value(socket.default_value, value);
    input_values[i] = InferenceValue::from_primitive(value);
  }

  infer_group_interface_usage(group, input_values, r_input_usages, {});  // TODO
}

void infer_group_interface_inputs_usage(const bNodeTree &group,
                                        const PointerRNA &properties_ptr,
                                        MutableSpan<SocketUsage> r_input_usages,
                                        std::optional<MutableSpan<SocketUsage>> r_output_usages)
{
  ResourceScope scope;
  const Vector<InferenceValue> group_input_values =
      nodes::get_geometry_nodes_input_inference_values(group, properties_ptr, scope);
  nodes::socket_usage_inference::infer_group_interface_usage(
      group, group_input_values, r_input_usages, r_output_usages);
}

SocketUsageParams::SocketUsageParams(SocketUsageInferencer &inferencer,
                                     const ComputeContext *compute_context,
                                     const bNodeTree &tree,
                                     const bNode &node,
                                     const bNodeSocket &socket)
    : inferencer_(inferencer),
      compute_context_(compute_context),
      tree(tree),
      node(node),
      socket(socket)
{
}

InferenceValue SocketUsageParams::get_input(const UString identifier) const
{
  const SocketInContext input_socket{compute_context_, this->node.input_by_identifier(identifier)};
  return inferencer_.impl_.get_socket_value(input_socket);
}

std::optional<bool> SocketUsageParams::any_output_is_used() const
{
  const bNodeSocket *first_missing = nullptr;
  for (const bNodeSocket *output_socket : this->node.output_sockets()) {
    if (const std::optional<bool> is_used = inferencer_.impl_.all_socket_usages_.lookup_try(
            {compute_context_, output_socket}))
    {
      if (*is_used) {
        return true;
      }
    }
    else {
      first_missing = output_socket;
    }
  }
  if (first_missing) {
    inferencer_.impl_.push_usage_task({compute_context_, first_missing});
    return std::nullopt;
  }
  return false;
}

bool SocketUsageParams::menu_input_may_be(const UString identifier, const int enum_value) const
{
  BLI_assert(this->node.input_by_identifier(identifier)->type == SOCK_MENU);
  const InferenceValue value = this->get_input(identifier);
  if (!value.is_primitive_value()) {
    /* The value is unknown, so it may be the requested enum value. */
    return true;
  }
  return value.get_primitive<MenuValue>().item_is_selected(enum_value);
}

bool SocketUsageParams::bool_input_may_be(const UString identifier, const bool bool_value) const
{
  BLI_assert(this->node.input_by_identifier(identifier)->type == SOCK_BOOLEAN);
  const InferenceValue value = this->get_input(identifier);
  if (!value.is_primitive_value()) {
    /* The value is unknown, so it may be the requested enum value. */
    return true;
  }
  return value.get_primitive<bool>() == bool_value;
}

void SocketUsageInferencer::mark_top_level_node_outputs_as_used()
{
  impl_.mark_top_level_node_outputs_as_used();
}

bool SocketUsageInferencer::is_group_input_used(const int input_i)
{
  return impl_.is_group_input_used(input_i);
}

bool SocketUsageInferencer::is_socket_used(const SocketInContext &socket)
{
  return impl_.is_socket_used(socket);
}

bool SocketUsageInferencer::is_disabled_group_output(const int output_i)
{
  return impl_.is_disabled_group_output(output_i);
}

bool SocketUsageInferencer::is_disabled_output(const SocketInContext &socket)
{
  return impl_.is_disabled_output(socket);
}

}  // namespace blender::nodes::socket_usage_inference
