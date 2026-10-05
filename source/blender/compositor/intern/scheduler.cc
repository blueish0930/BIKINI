/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <type_traits>

#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "DNA_node_types.h"

#include "BKE_compositor.hh"
#include "BKE_compute_context_cache.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_node.hh"
#include "BKE_node_enum.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_zones.hh"
#include "BKE_type_conversions.hh"

#include "NOD_geo_index_switch.hh"
#include "NOD_geo_menu_switch.hh"

#include "COM_context.hh"
#include "COM_scheduler.hh"
#include "COM_undefined_node_operation.hh"
#include "COM_utilities.hh"

namespace blender::compositor {

/* Checks if the given node group with the given compute context has an active Viewer node in it or
 * in one of its descendants with the given viewer compute context. */
static bool has_viewer_node(const bNodeTree &node_group,
                            const ComputeContext &compute_context,
                            const ComputeContext &viewer_compute_context,
                            bke::ComputeContextCache &compute_context_cache)
{
  node_group.ensure_topology_cache();

  /* Check if any of the nodes match the viewer compute context. */
  for (const bNode *node : node_group.nodes_by_type("CompositorNodeViewer"_ustr)) {
    if (!(node->flag & NODE_DO_OUTPUT) || node->is_muted()) {
      continue;
    }

    const ComputeContext &zone_viewer_compute_context =
        bke::compositor::get_zone_viewer_compute_context(
            *node, nullptr, compute_context, compute_context_cache);
    if (&viewer_compute_context == &zone_viewer_compute_context) {
      return true;
    }
    /* --- IMAGE_NODES_MVP begin --- */
    for (const bNode *node : node_group.nodes_by_type("ImageNodeViewer"_ustr)) {
      if (node->flag & NODE_DO_OUTPUT && !node->is_muted()) {
        return true;
      }
    }
    /* --- IMAGE_NODES_MVP end --- */
  }

  /* Otherwise, we have to check node groups recursively. */
  for (const bNode *group_node : node_group.group_nodes()) {
    if (group_node->is_muted() || !group_node->id) {
      continue;
    }

    const ComputeContext &zone_viewer_compute_context =
        bke::compositor::get_zone_viewer_compute_context(
            *group_node, nullptr, compute_context, compute_context_cache);

    const bNodeTree &child_node_group = *reinterpret_cast<const bNodeTree *>(group_node->id);
    const bke::GroupNodeComputeContext &node_compute_context =
        compute_context_cache.for_group_node(
            &zone_viewer_compute_context, group_node->identifier, &group_node->owner_tree());
    if (has_viewer_node(
            child_node_group, node_compute_context, viewer_compute_context, compute_context_cache))
    {
      return true;
    }
  }

  return false;
}

/* Checks if the node group has a File Output node in it or in one of its descendants. */
static bool has_file_output_recursive(const bNodeTree &node_group)
{
  node_group.ensure_topology_cache();
  for (const bNode *node : node_group.nodes_by_type("CompositorNodeOutputFile"_ustr)) {
    if (!node->is_muted()) {
      return true;
    }
  }
  /* GPU Texture Editor Image Output writes a named Image datablock (not a disk file). */
  for (const bNode *node : node_group.nodes_by_type("ImageNodeFileOutput"_ustr)) {
    if (!node->is_muted()) {
      return true;
    }
  }
  /* Bake Image only counts as a file output while the Bake operator is running. */
  if (image_process_bake_write_enabled()) {
    for (const bNode *node : node_group.nodes_by_type("ImageNodeBakeImage"_ustr)) {
      if (!node->is_muted()) {
        return true;
      }
    }
  }

  for (const bNode *group_node : node_group.group_nodes()) {
    if (!group_node->is_muted() && group_node->id) {
      if (has_file_output_recursive(*reinterpret_cast<const bNodeTree *>(group_node->id))) {
        return true;
      }
    }
  }

  return false;
}

/* Returns the value of the given input if it can be determined statically, otherwise nullopt is
 * returned. The value is only known statically if the socket_result_fn returns an allocated result
 * for it or the input is connected to a context-based nodes like the Is Viewport node. */
template<typename T, typename SocketT>
static std::optional<T> get_input_socket_value(const Context &context,
                                               const bNodeSocket &input,
                                               SocketResultFn socket_result_fn)
{
  if (!input.is_logically_linked()) {
    return T(input.default_value_typed<SocketT>()->value);
  }

  const bNodeSocket *linked_output = input.logically_linked_sockets()[0];
  if (linked_output->owner_node().is_type("GeometryNodeIsViewport"_ustr)) {
    /* Is Viewport node's value can be determined statically.
     * Convert according to the same implicit conversion table used at runtime. */
    const bool is_viewport = context.is_viewport();
    if constexpr (std::is_same_v<T, bool>) {
      return is_viewport;
    }
    else {
      const CPPType &from_type = CPPType::get<bool>();
      const CPPType &to_type = CPPType::get<T>();
      const bke::DataTypeConversions &conversions = bke::get_implicit_type_conversions();
      if (!conversions.is_convertible(from_type, to_type)) {
        return std::nullopt;
      }
      T converted_value;
      conversions.convert_to_uninitialized(from_type, to_type, &is_viewport, &converted_value);
      return converted_value;
    }
  }

  if (linked_output->type != input.type) {
    return std::nullopt;
  }

  const Result *result = socket_result_fn(*linked_output);
  if (!result || !result->is_allocated()) {
    return std::nullopt;
  }

  return result->get_single_value_default<T>();
}

/* Returns true if the given input of the given Switch node in the given operation is needed by the
 * node. */
static bool is_switch_node_input_needed(const Context &context,
                                        const bNode &node,
                                        const bNodeSocket &input,
                                        SocketResultFn socket_result_fn)
{
  const UString condition_identifier = "Switch"_ustr;
  if (input.identifier_ustr() == condition_identifier) {
    return true;
  }

  const std::optional<bool> condition = get_input_socket_value<bool, bNodeSocketValueBoolean>(
      context, *node.input_by_identifier(condition_identifier), socket_result_fn);
  if (!condition.has_value()) {
    return true;
  }

  return (input.identifier_ustr() == "True"_ustr) == condition.value();
}

static bool menu_socket_dna_is_multi(const bNodeSocket &socket)
{
  if (socket.type != SOCK_MENU || !is_plausible_pointer(socket.default_value)) {
    return false;
  }
  return (static_cast<const bNodeSocketValueMenu *>(socket.default_value)->runtime_flag &
          bke::NODE_MENU_MULTI_SELECTION) != 0;
}

/* Group Input uses the interface flag. Do not sticky-keep the Menu Switch socket's own bit. */
static bool menu_switch_int_is_multi(const bNode &node, const bNodeSocket &menu_input)
{
  if (!menu_input.is_logically_linked()) {
    return menu_socket_dna_is_multi(menu_input);
  }
  const Span<const bNodeSocket *> linked = menu_input.logically_linked_sockets();
  if (linked.is_empty()) {
    return menu_socket_dna_is_multi(menu_input);
  }
  const bNodeSocket &from = *linked[0];
  if (from.owner_node().is_group_input()) {
    const bNodeTree &ntree = node.owner_tree();
    if (ntree.tree_interface.items_cache_is_available() && from.identifier[0] != '\0') {
      const int index = ntree.interface_input_index_by_identifier(from.identifier);
      const Span<const bNodeTreeInterfaceSocket *> iface_inputs = ntree.interface_inputs();
      if (index >= 0 && index < iface_inputs.size() && iface_inputs[index] != nullptr) {
        return (iface_inputs[index]->flag & NODE_INTERFACE_SOCKET_MENU_MULTI_SELECTION) != 0;
      }
      return false;
    }
    return menu_socket_dna_is_multi(from);
  }
  return menu_socket_dna_is_multi(from);
}

/* Static menu, or nullopt when it is not a single value yet (keep every item input). */
static std::optional<nodes::MenuValue> get_static_menu_value(const bNodeSocket &input,
                                                            const bool multi_if_int,
                                                            SocketResultFn socket_result_fn)
{
  if (!input.is_logically_linked()) {
    if (!is_plausible_pointer(input.default_value)) {
      return nodes::MenuValue(0);
    }
    const bNodeSocketValueMenu *storage = input.default_value_typed<bNodeSocketValueMenu>();
    const bool multi = (storage->runtime_flag & bke::NODE_MENU_MULTI_SELECTION) != 0;
    return nodes::MenuValue(storage->value, multi);
  }

  const Span<const bNodeSocket *> linked = input.logically_linked_sockets();
  if (linked.is_empty()) {
    return std::nullopt;
  }
  const bNodeSocket &linked_output = *linked[0];
  if (linked_output.owner_node().is_type("GeometryNodeIsViewport"_ustr)) {
    return std::nullopt;
  }
  if (linked_output.type != input.type) {
    return std::nullopt;
  }

  const Result *result = socket_result_fn(linked_output);
  if (result == nullptr) {
    return std::nullopt;
  }
  return menu_value_from_result(*result, multi_if_int);
}

/* Returns true if the given input of the given menu switch node in the given operation is needed
 * by the node. */
static bool is_menu_switch_node_input_needed(const Context & /*context*/,
                                             const bNode &node,
                                             const bNodeSocket &input,
                                             SocketResultFn socket_result_fn)
{
  const UString menu_identifier = "Menu"_ustr;
  if (input.identifier_ustr() == menu_identifier) {
    return true;
  }

  const bNodeSocket *menu_socket = node.input_by_identifier(menu_identifier);
  if (menu_socket == nullptr || node.storage == nullptr) {
    return true;
  }

  const bool multi_if_int = menu_switch_int_is_multi(node, *menu_socket);
  const std::optional<nodes::MenuValue> menu = get_static_menu_value(
      *menu_socket, multi_if_int, socket_result_fn);
  if (!menu.has_value()) {
    return true;
  }

  const NodeMenuSwitch &storage = *static_cast<const NodeMenuSwitch *>(node.storage);
  const Span<NodeEnumItem> items = storage.enum_definition.items();
  /* Same reading as the node UI: identifier 3 is item 3, not bits 0 and 1. */
  const nodes::MenuValue resolved = menu->normalized(
      multi_if_int, int(items.size()), [&](const int i) { return items[i].identifier; });
  const int selected = resolved.first_selected_enum_index(
      int(items.size()), [&](const int i) { return items[i].identifier; });
  if (selected < 0) {
    /* No item matches. Leave the output invalid instead of forwarding a default. */
    return false;
  }
  const NodeEnumItem menu_item{nullptr, nullptr, items[selected].identifier};
  const std::string identifier = nodes::MenuSwitchItemsAccessor::socket_identifier_for_item(
      menu_item);
  return input.identifier == identifier;
}

/* Returns true if the given input of the given Index Switch node in the given operation is needed
 * by the node. */
static bool is_index_switch_node_input_needed(const Context &context,
                                              const bNode &node,
                                              const bNodeSocket &input,
                                              SocketResultFn socket_result_fn)
{
  const UString index_identifier = "Index"_ustr;
  if (input.identifier_ustr() == index_identifier) {
    return true;
  }

  const std::optional<int> index = get_input_socket_value<int, bNodeSocketValueInt>(
      context, *node.input_by_identifier(index_identifier), socket_result_fn);
  if (!index.has_value()) {
    return true;
  }

  const NodeIndexSwitch &storage = *static_cast<const NodeIndexSwitch *>(node.storage);
  if (!IndexRange(storage.items_num).contains(index.value())) {
    return false;
  }

  const std::string identifier = nodes::IndexSwitchItemsAccessor::socket_identifier_for_item(
      storage.items[index.value()]);
  return input.identifier == identifier;
}

/* Returns true if the given input of the given node in the given operation is needed by the
 * compositor. */
static bool is_input_needed(const Context &context,
                            const bNode &node,
                            const bNodeSocket &input,
                            SocketResultFn socket_result_fn)
{
  /* Lazy evaluation of inputs that reference outputs outside of their own zone are currently not
   * supported. */
  const bke::bNodeTreeZones &zones = *node.owner_tree().zones();
  const bNodeSocket *output = get_output_linked_to_input(input);
  if (output && zones.get_zone_by_socket(input) != zones.get_zone_by_socket(*output)) {
    return true;
  }

  if (node.is_group_output()) {
    const Result *result = socket_result_fn(input);
    if (!result) {
      return true;
    }
    return result->should_compute();
  }

  if (node.is_type("GeometryNodeSwitch"_ustr)) {
    return is_switch_node_input_needed(context, node, input, socket_result_fn);
  }

  if (node.is_type("GeometryNodeMenuSwitch"_ustr)) {
    return is_menu_switch_node_input_needed(context, node, input, socket_result_fn);
  }

  if (node.is_type("GeometryNodeIndexSwitch"_ustr)) {
    return is_index_switch_node_input_needed(context, node, input, socket_result_fn);
  }

  return true;
}

/* Get a stack of the output nodes whose result should be computed. This typically includes the
 * main output node like the Group Output node, as well as side-effect nodes if requested by the
 * context like the File Output, Viewer nodes, group nodes whose group that have those side effect
 * nodes, or zones that have those side effect nodes. If zone is not nullptr, only output nodes in
 * that zone will be included. */
static Stack<const bNode *> get_output_nodes(const Context &context,
                                             const bNodeTree &node_group,
                                             const ComputeContext &compute_context,
                                             const bke::bNodeTreeZone *zone,
                                             SocketResultFn socket_result_fn)
{
  node_group.ensure_topology_cache();
  const bke::bNodeTreeZones &zones = *node_group.zones();
  const SideEffectOutputTypes needed_side_effect_output_types =
      context.needed_side_effect_output_types();

  Stack<const bNode *> node_stack;

  /* Paint Edit bake: ONLY the forced Paint node (and its Color deps). Must run before any
   * Viewer/File roots — otherwise Rasterize/Fluid hang Enter Paint. */
  if (!zone && node_group.type == NTREE_IMAGE && image_process_paint_force_schedule_active() &&
      node_group.id.session_uid == image_process_paint_force_tree_session_uid())
  {
    const int force_id = image_process_paint_force_node_identifier();
    for (const bNode *node : node_group.nodes_by_type("ImageNodePaint"_ustr)) {
      if (!node->is_muted() && node->identifier == force_id) {
        node_stack.push(node);
      }
    }
    return node_stack;
  }

  /* If the node is in the same zone, we push the node to the stack, otherwise, we push the output
   * node of the immediate child zone that contain the node. */
  auto push_node_to_stack = [&](const bNode &node) {
    const bke::bNodeTreeZone *node_zone = zones.get_zone_by_node(node.identifier);
    if (node_zone == zone) {
      node_stack.push(&node);
    }
    else {
      node_stack.push(zones.get_zones_to_enter(zone, node_zone)[0]->output_node());
    }
  };

  /* Add group nodes that contain File Output and Viewer nodes. */
  for (const bNode *group_node : node_group.group_nodes()) {
    if (group_node->is_muted() || !group_node->id) {
      continue;
    }

    /* Only consider nodes inside the same zone being scheduled. */
    if (zone && !zone->contains_node_recursively(*group_node)) {
      continue;
    }

    const ComputeContext &zone_viewer_compute_context =
        bke::compositor::get_zone_viewer_compute_context(
            *group_node, zone, compute_context, context.compute_context_cache());

    const bNodeTree &child_tree = *reinterpret_cast<const bNodeTree *>(group_node->id);
    const bke::GroupNodeComputeContext &node_compute_context =
        context.compute_context_cache().for_group_node(
            &zone_viewer_compute_context, group_node->identifier, &group_node->owner_tree());
    const ComputeContext *viewer_compute_context = context.viewer_compute_context();
    if (flag_is_set(needed_side_effect_output_types, SideEffectOutputTypes::ViewerNode) &&
        viewer_compute_context &&
        has_viewer_node(child_tree,
                        node_compute_context,
                        *viewer_compute_context,
                        context.compute_context_cache()))
    {
      push_node_to_stack(*group_node);
      continue;
    }

    if (flag_is_set(needed_side_effect_output_types, SideEffectOutputTypes::FileOutputNode) &&
        has_file_output_recursive(child_tree))
    {
      push_node_to_stack(*group_node);
      continue;
    }
  }

  /* Add Warning nodes. */
  for (const bNode *node : node_group.nodes_by_type("GeometryNodeWarning"_ustr)) {
    if (node->is_muted()) {
      continue;
    }

    /* Only consider nodes inside the same zone being scheduled. */
    if (zone && !zone->contains_node_recursively(*node)) {
      continue;
    }

    push_node_to_stack(*node);
  }

  /* Image Process partial cook: when a terminal filter is set, only those node identifiers may
   * become schedule roots (e.g. frame update on Fluid Viewer must not re-run independent Image
   * Output modules). Empty filter = all terminals (full dirty edit). */
  const bool terminal_filter = node_group.type == NTREE_IMAGE &&
                               image_process_terminal_filter_active();
  auto allow_terminal = [&](const bNode &node) -> bool {
    if (!terminal_filter) {
      return true;
    }
    return image_process_terminal_filter_contains(node.identifier);
  };

  /* Add File Output nodes. */
  if (flag_is_set(needed_side_effect_output_types, SideEffectOutputTypes::FileOutputNode)) {
    for (const bNode *node : node_group.nodes_by_type("CompositorNodeOutputFile"_ustr)) {
      if (node->is_muted() || !allow_terminal(*node)) {
        continue;
      }

      /* Only consider nodes inside the same zone being scheduled. */
      if (zone && !zone->contains_node_recursively(*node)) {
        continue;
      }

      push_node_to_stack(*node);
    }
    for (const bNode *node : node_group.nodes_by_type("ImageNodeFileOutput"_ustr)) {
      if (node->is_muted() || !allow_terminal(*node)) {
        continue;
      }

      /* Only consider nodes in the same zone being scheduled. */
      if (zones.get_zone_by_node(node->identifier) != zone) {
        continue;
      }

      node_stack.push(node);
    }
    /* Only schedule Bake Image when the Bake operator enables disk write. */
    if (image_process_bake_write_enabled()) {
      for (const bNode *node : node_group.nodes_by_type("ImageNodeBakeImage"_ustr)) {
        if (node->is_muted() || !allow_terminal(*node)) {
          continue;
        }

        /* Only consider nodes in the same zone being scheduled. */
        if (zones.get_zone_by_node(node->identifier) != zone) {
          continue;
        }

        node_stack.push(node);
      }
    }
  }

  /* Add Viewer node. */
  const ComputeContext *viewer_compute_context = context.viewer_compute_context();
  if (flag_is_set(needed_side_effect_output_types, SideEffectOutputTypes::ViewerNode) &&
      viewer_compute_context)
  {
    for (const bNode *node : node_group.nodes_by_type("CompositorNodeViewer"_ustr)) {
      if (!(node->flag & NODE_DO_OUTPUT) || node->is_muted() || !allow_terminal(*node)) {
        continue;
      }

      /* Only consider nodes inside the same zone being scheduled. */
      if (zone && !zone->contains_node_recursively(*node)) {
        continue;
      }

      /* Only consider nodes in the viewer compute context. */
      const ComputeContext &zone_viewer_compute_context =
          bke::compositor::get_zone_viewer_compute_context(
              *node, zone, compute_context, context.compute_context_cache());
      if (viewer_compute_context != &zone_viewer_compute_context) {
        continue;
      }

      push_node_to_stack(*node);
      break;
    }
    /* --- IMAGE_NODES_MVP begin --- */
    for (const bNode *node : node_group.nodes_by_type("ImageNodeViewer"_ustr)) {
      if (!(node->flag & NODE_DO_OUTPUT) || node->is_muted() || !allow_terminal(*node)) {
        continue;
      }

      /* Only consider nodes in the same zone being scheduled. */
      if (zones.get_zone_by_node(node->identifier) != zone) {
        continue;
      }

      node_stack.push(node);
      break;
    }
    /* --- IMAGE_NODES_MVP end --- */
  }

  /* Add Group Output node if any of its inputs are needed and we are not scheduling a zone. */
  const bNode *output_node = node_group.group_output_node();
  if (!zone && output_node && !output_node->is_muted()) {
    for (const bNodeSocket *input : output_node->input_sockets()) {
      if (!is_socket_available(input)) {
        continue;
      }

      if (is_input_needed(context, *output_node, *input, socket_result_fn)) {
        node_stack.push(output_node);
        break;
      }
    }
  }

  /* Add the Zone Output node if we are scheduling a zone.
   * Image Process: do NOT force unused Repeat/Simulation zones into the demand graph. */
  if (zone) {
    node_stack.push(zone->output_node());
  }

  return node_stack;
}

/* A type representing a mapping that associates each node with a heuristic estimation of the
 * number of intermediate buffers needed to compute it and all of its dependencies. See the
 * compute_number_of_needed_buffers function for more information. */
using NeededBuffers = Map<const bNode *, int>;

/* Compute a heuristic estimation of the number of intermediate buffers needed to compute each node
 * and all of its dependencies for all nodes that the given node depends on. The output is a map
 * that maps each node with the number of intermediate buffers needed to compute it and all of its
 * dependencies.
 *
 * Consider a node that takes n number of buffers as an input from a number of node dependencies,
 * which we shall call the input nodes. The node also computes and outputs m number of buffers.
 * In order for the node to compute its output, a number of intermediate buffers will be needed.
 * Since the node takes n buffers and outputs m buffers, then the number of buffers directly
 * needed by the node is (n + m). But each of the input buffers are computed by a node that, in
 * turn, needs a number of buffers to compute its output. So the total number of buffers needed
 * to compute the output of the node is max(n + m, d) where d is the number of buffers needed by
 * the input node that needs the largest number of buffers. We only consider the input node that
 * needs the largest number of buffers, because those buffers can be reused by any input node
 * that needs a lesser number of buffers.
 *
 * Pixel nodes, however, are a special case because links between two pixel nodes inside the same
 * pixel operation don't pass a buffer, but a single value in the pixel processor. So for pixel
 * nodes, only inputs and outputs linked to nodes that are not pixel nodes should be considered.
 * Note that this might not actually be true, because the compiler may decide to split a pixel
 * operation into multiples ones that will pass buffers, but this is not something that can be
 * known at scheduling-time. See the description of NodeTreeEvaluator and COM_shader_operation.hh
 * for more information. In the node tree shown below, node 4 will have exactly the same number of
 * needed buffers by node 3, because its inputs and outputs are all internally linked in the pixel
 * operation.
 *
 *                                      Pixel Operation
 *                   +------------------------------------------------------+
 * .------------.    |  .------------.  .------------.      .------------.  |  .------------.
 * |   Node 1   |    |  |   Node 3   |  |   Node 4   |      |   Node 5   |  |  |   Node 6   |
 * |            |----|--|            |--|            |------|            |--|--|            |
 * |            |  .-|--|            |  |            |  .---|            |  |  |            |
 * '------------'  | |  '------------'  '------------'  |   '------------'  |  '------------'
 *                 | +----------------------------------|-------------------+
 * .------------.  |                                    |
 * |   Node 2   |  |                                    |
 * |            |--'------------------------------------'
 * |            |
 * '------------'
 *
 * Note that the computed output is not guaranteed to be accurate, and will not be in most cases.
 * The computation is merely a heuristic estimation that works well in most cases. This is due to a
 * number of reasons:
 * - The node tree is actually a graph that allows output sharing, which is not something that was
 *   taken into consideration in this implementation because it is difficult to correctly consider.
 * - Each node may allocate any number of internal buffers, which is not taken into account in this
 *   implementation because it rarely affects the output and is done by very few nodes.
 * - The compiler may decide to compiler the schedule differently depending on runtime information
 *   which we can merely speculate at scheduling-time as described above. */
static NeededBuffers compute_number_of_needed_buffers(const Context &context,
                                                      Stack<const bNode *> &output_nodes,
                                                      SocketResultFn socket_result_fn)
{
  NeededBuffers needed_buffers;

  /* A stack of nodes used to traverse the node group starting from the output nodes. */
  Stack<const bNode *> node_stack = output_nodes;

  /* Traverse the node group in a post order depth first manner and compute the number of needed
   * buffers for each node. Post order traversal guarantee that all the node dependencies of each
   * node are computed before it. This is done by pushing all the uncomputed node dependencies to
   * the node stack first and only popping and computing the node when all its node dependencies
   * were computed. */
  while (!node_stack.is_empty()) {
    /* Do not pop the node immediately, as it may turn out that we can't compute its number of
     * needed buffers just yet because its dependencies weren't computed, it will be popped later
     * when needed. */
    const bNode &node = *node_stack.peek();

    /* Go over the node dependencies connected to the inputs of the node and push them to the node
     * stack if they were not computed already. Multi-input sockets may have many links — visit all
     * (e.g. Point Stamp multi-stamp sources). */
    Set<const bNode *> pushed_nodes;
    for (const bNodeSocket *input : node.input_sockets()) {
      if (!is_socket_available(input)) {
        continue;
      }

      if (!is_input_needed(context, node, *input, socket_result_fn)) {
        continue;
      }

      foreach_output_linked_to_input(*input, [&](const bNodeSocket &output) {
        /* The node dependency was already computed or pushed before, so skip it. */
        if (needed_buffers.contains(&output.owner_node()) ||
            pushed_nodes.contains(&output.owner_node()))
        {
          return;
        }

        /* The output node needs to be computed, push the node dependency to the node stack and
         * indicate that it was pushed. */
        node_stack.push(&output.owner_node());
        pushed_nodes.add_new(&output.owner_node());
      });
    }

    /* If any of the node dependencies were pushed, that means that not all of them were computed
     * and consequently we can't compute the number of needed buffers for this node just yet. */
    if (!pushed_nodes.is_empty()) {
      continue;
    }

    /* We don't need to store the result of the pop because we already peeked at it before. */
    node_stack.pop();

    /* Compute the number of buffers that the node takes as an input as well as the number of
     * buffers needed to compute the most demanding of the node dependencies. Multi-input: count
     * every linked source. */
    int number_of_input_buffers = 0;
    int buffers_needed_by_dependencies = 0;
    for (const bNodeSocket *input : node.input_sockets()) {
      if (!is_socket_available(input)) {
        continue;
      }

      if (!is_input_needed(context, node, *input, socket_result_fn)) {
        continue;
      }

      foreach_output_linked_to_input(*input, [&](const bNodeSocket &output) {
        /* Since this input is linked, if the link is not between two pixel nodes, it means that the
         * node takes a buffer through this input and so we increment the number of input buffers. */
        if (!is_pixel_node(node) || !is_pixel_node(output.owner_node())) {
          number_of_input_buffers++;
        }

        /* If the number of buffers needed by the node dependency is more than the total number of
         * buffers needed by the dependencies, then update the latter to be the former. This is
         * computing the "d" in the aforementioned equation "max(n + m, d)". */
        const int buffers_needed_by_dependency = needed_buffers.lookup(&output.owner_node());
        buffers_needed_by_dependencies = std::max(buffers_needed_by_dependency,
                                                  buffers_needed_by_dependencies);
      });
    }

    /* Compute the number of buffers that will be computed/output by this node. */
    int number_of_output_buffers = 0;
    for (const bNodeSocket *output : node.output_sockets()) {
      if (!is_socket_available(output)) {
        continue;
      }

      /* The output is not linked, it outputs no buffer. */
      if (!output->is_logically_linked()) {
        continue;
      }

      /* If any of the links is not between two pixel nodes, it means that the node outputs
       * a buffer through this output and so we increment the number of output buffers. */
      if (!is_pixel_node(node) ||
          is_output_linked_to_input_conditioned(
              *output,
              [&](const bNodeSocket &input) { return !is_pixel_node(input.owner_node()); }))
      {
        number_of_output_buffers++;
      }
    }

    /* Compute the heuristic estimation of the number of needed intermediate buffers to compute
     * this node and all of its dependencies. This is computing the aforementioned equation
     * "max(n + m, d)". */
    const int total_buffers = std::max(number_of_input_buffers + number_of_output_buffers,
                                       buffers_needed_by_dependencies);
    needed_buffers.add(&node, total_buffers);
  }

  return needed_buffers;
}

/* Find the nodes that the given node depends on. Nodes already scheduled are not included. Only
 * nodes in the given zone are included, except for zone output nodes of child zones, which acts as
 * representatives for the zone in the schedule. Unneeded inputs are marked in the schedule.
 * Multi-input sockets contribute every linked source (e.g. Point Stamp). Named Portal GET nodes
 * also depend on their matching Store. Image Process trees flatten demand-reachable zones. */
static Vector<const bNode *> find_node_dependency_nodes(const Context &context,
                                                        Schedule &schedule,
                                                        const bNode &node,
                                                        const bke::bNodeTreeZone *zone,
                                                        SocketResultFn socket_result_fn)
{
  const bool flatten_zones = context.is_image_process();
  const bke::bNodeTreeZones *zones = nullptr;
  if (!flatten_zones) {
    const bNodeTree *tree = (node.runtime != nullptr) ? node.runtime->owner_tree : nullptr;
    if (tree == nullptr) {
      return {};
    }
    zones = tree->zones();
  }

  VectorSet<const bNode *> dependency_nodes;
  for (const bNodeSocket *input : node.input_sockets()) {
    if (!is_socket_available(input)) {
      continue;
    }

    if (!is_input_needed(context, node, *input, socket_result_fn)) {
      schedule.unneeded_inputs.add(input);
      continue;
    }

    /* Multi-input (e.g. Point Stamp): schedule every linked source, not only the first. */
    foreach_output_linked_to_input(*input, [&](const bNodeSocket &output) {
      if (schedule.nodes.contains(&output.owner_node())) {
        return;
      }

      if (!flatten_zones && zones != nullptr) {
        /* Dependency node is not in the same zone, so skip it. */
        const bke::bNodeTreeZone *dependency_node_zone = zones->get_zone_by_socket(output);
        if (dependency_node_zone != zone) {
          return;
        }
      }

      dependency_nodes.add(&output.owner_node());
    });
  }

  /* Named Portal GET has no visible links; still depend on the matching Store. */
  if (bke::node_is_named_portal(node) && node.type_legacy != GEO_NODE_NAMED_PORTAL && node.storage)
  {
    if (const bNode *store = bke::node_find_unique_store_named_portal(
            node.owner_tree(), bke::node_named_portal_name(node)))
    {
      if (!schedule.nodes.contains(store)) {
        dependency_nodes.add(store);
      }
    }
  }

  return dependency_nodes.extract_vector();
}

/* Find the nodes that the given zone depends on, this includes both source nodes of zone border
 * links as well as the dependencies of the inputs of the zone input node. Nodes already scheduled
 * are not included. Only dependencies in the parent zone are included. */
static Vector<const bNode *> find_zone_dependency_nodes(const Context &context,
                                                        Schedule &schedule,
                                                        const bke::bNodeTreeZone &zone,
                                                        SocketResultFn socket_result_fn)
{
  VectorSet<const bNode *> dependency_nodes;
  for (const bNodeLink *link : zone.border_links) {
    const bNodeSocket *output = get_output_linked_to_input(*link->tosock);
    if (!output) {
      continue;
    }

    /* The dependency node was already scheduled, so skip it. */
    if (schedule.nodes.contains(&output->owner_node())) {
      continue;
    }

    /* Only consider nodes in the same zone being scheduled. */
    if (zone.owner->get_zone_by_socket(*output) != zone.parent_zone) {
      continue;
    }

    dependency_nodes.add(&output->owner_node());
  }

  Vector<const bNode *> zone_input_dependency_nodes = find_node_dependency_nodes(
      context, schedule, *zone.input_node(), zone.parent_zone, socket_result_fn);
  dependency_nodes.add_multiple(zone_input_dependency_nodes);
  return dependency_nodes.extract_vector();
}

/* Find the nodes that the given node depends on. Nodes already scheduled are not included. Only
 * nodes in the given zone are included, except for zone output nodes of child zones, which acts as
 * representatives for the zone in the schedule. Unneeded inputs are marked in the schedule. */
static Vector<const bNode *> find_dependency_nodes(const Context &context,
                                                   Schedule &schedule,
                                                   const bNode &node,
                                                   const bke::bNodeTreeZone *zone,
                                                   SocketResultFn socket_result_fn)
{
  /* Image Process flattens demand-reachable zones (pass-through boundaries). */
  if (context.is_image_process()) {
    return find_node_dependency_nodes(context, schedule, node, zone, socket_result_fn);
  }

  /* If the node is a zone output of a child zone, we find the dependencies of the zone not its
   * inner nodes. */
  const bke::bNodeTreeZones &zones = *node.owner_tree().zones();
  const bke::bNodeTreeZone *node_zone = zones.get_zone_by_node(node.identifier);
  if (node_zone && node_zone->output_node() == &node && node_zone->parent_zone == zone) {
    return find_zone_dependency_nodes(context, schedule, *node_zone, socket_result_fn);
  }

  return find_node_dependency_nodes(context, schedule, node, zone, socket_result_fn);
}

/* Schedule from a prepared root stack (Sethi-Ullman-style post-order). Consumes #node_stack. */
static Schedule schedule_from_root_stack(const Context &context,
                                         const bNodeTree &node_group,
                                         Stack<const bNode *> node_stack,
                                         const bke::bNodeTreeZone *zone,
                                         SocketResultFn socket_result_fn)
{
  Schedule schedule{node_group, zone};
  if (node_stack.is_empty()) {
    return schedule;
  }

  /* Compute the number of buffers needed by each node connected to the outputs. */
  const NeededBuffers needed_buffers = compute_number_of_needed_buffers(
      context, node_stack, socket_result_fn);

  /* Traverse the node group in a post order depth first manner, scheduling the nodes in an order
   * informed by the number of buffers needed by each node. Post order traversal guarantee that all
   * the node dependencies of each node are scheduled before it. This is done by pushing all the
   * unscheduled node dependencies to the node stack first and only popping and scheduling the node
   * when all its node dependencies were scheduled. */
  while (!node_stack.is_empty()) {
    /* Do not pop the node immediately, as it may turn out that we can't schedule it just yet
     * because its dependencies weren't scheduled, it will be popped later when needed. */
    const bNode &node = *node_stack.peek();

    Vector<const bNode *> dependency_nodes = find_dependency_nodes(
        context, schedule, node, zone, socket_result_fn);

    /* Push the dependency nodes to the node stack such that the node with the highest number of
     * needed buffers is scheduled first, so we push the nodes in ascending order. */
    std::ranges::stable_sort(dependency_nodes, [&](const bNode *a, const bNode *b) {
      return needed_buffers.lookup(a) < needed_buffers.lookup(b);
    });
    node_stack.push_multiple(dependency_nodes);

    /* If there are no sorted dependency nodes, that means they were all already scheduled or that
     * none exists in the first place, so we can pop and schedule the node now. */
    if (dependency_nodes.is_empty()) {
      /* The node might have already been scheduled, so we don't use add_new here and simply don't
       * add it if it was already scheduled. */
      schedule.nodes.add(node_stack.pop());
    }
  }

  return schedule;
}

static const bNode *zone_output_for_input_node(const bNodeTree &tree, const bNode &input_node)
{
  int32_t output_id = 0;
  if (input_node.is_type("GeometryNodeSimulationInput"_ustr)) {
    output_id = static_cast<const NodeGeometrySimulationInput *>(input_node.storage)->output_node_id;
  }
  else if (input_node.is_type("GeometryNodeRepeatInput"_ustr)) {
    output_id = static_cast<const NodeGeometryRepeatInput *>(input_node.storage)->output_node_id;
  }
  else if (input_node.is_type("ImageNodeFluidSimInput"_ustr)) {
    output_id = static_cast<const NodeImageFluidSimInput *>(input_node.storage)->output_node_id;
  }
  else {
    return nullptr;
  }
  if (output_id == 0) {
    return nullptr;
  }
  const bNode *output_node = tree.node_by_id(output_id);
  if (!output_node || output_node->is_muted()) {
    return nullptr;
  }
  return output_node;
}

/* There are multiple different possible orders of evaluating a node graph, each of which needs
 * to allocate a number of intermediate buffers to store its intermediate results. It follows
 * that we need to find the evaluation order which uses the least amount of intermediate buffers.
 * For instance, consider a node that takes two input buffers A and B. Each of those buffers is
 * computed through a number of nodes constituting a sub-graph whose root is the node that
 * outputs that buffer. Suppose the number of intermediate buffers needed to compute A and B are
 * N(A) and N(B) respectively and N(A) > N(B). Then evaluating the sub-graph computing A would be
 * a better option than that of B, because had B was computed first, its outputs will need to be
 * stored in extra buffers in addition to the buffers needed by A. The number of buffers needed by
 * each node is estimated as described in the compute_number_of_needed_buffers function.
 *
 * This is a heuristic generalization of the Sethi-Ullman algorithm, a generalization that
 * doesn't always guarantee an optimal evaluation order, as the optimal evaluation order is very
 * difficult to compute, however, this method works well in most cases. Moreover it assumes that
 * all buffers will have roughly the same size, which may not always be the case. */
Schedule compute_schedule(const Context &context,
                          const bNodeTree &node_group,
                          const ComputeContext &compute_context,
                          const SocketResultFn socket_result_fn,
                          const bke::bNodeTreeZone *zone)
{
  Schedule schedule{node_group, zone};

  /* Validate node group. */
  node_group.ensure_topology_cache();
  if (node_group.has_available_link_cycle() || node_group.zones() == nullptr) {
    return schedule;
  }

  /* Get a stack of the initial output nodes used to traverse the node group. */
  Stack<const bNode *> node_stack = get_output_nodes(
      context, node_group, compute_context, zone, socket_result_fn);

  /* Image Process: Houdini-style demand cook. Start from outputs, then pull in zone Outputs only
   * when their Input is already demand-reachable (so unused Repeat/Sim zones are excluded). */
  if (node_group.type == NTREE_IMAGE && zone == nullptr) {
    VectorSet<const bNode *> roots;
    Stack<const bNode *> tmp = node_stack;
    while (!tmp.is_empty()) {
      roots.add(tmp.pop());
    }

    for (int expand = 0; expand < 8; expand++) {
      Stack<const bNode *> stack;
      for (const bNode *root : roots) {
        stack.push(root);
      }
      /* Schedule holds references, so it cannot be reassigned. */
      Schedule cooked = schedule_from_root_stack(context, node_group, stack, zone, socket_result_fn);

      bool added = false;
      for (const bNode *node : cooked.nodes) {
        if (!node->is_type("GeometryNodeSimulationInput"_ustr) &&
            !node->is_type("GeometryNodeRepeatInput"_ustr) &&
            !node->is_type("ImageNodeFluidSimInput"_ustr))
        {
          continue;
        }
        const bNode *zone_out = zone_output_for_input_node(node_group, *node);
        if (zone_out && !roots.contains(zone_out)) {
          roots.add(zone_out);
          added = true;
        }
      }
      if (!added || expand == 7) {
        return cooked;
      }
    }
  }

  return schedule_from_root_stack(context, node_group, node_stack, zone, socket_result_fn);
}

int compute_output_reference_count(const bNodeSocket &output,
                                   const Schedule &schedule,
                                   const VectorSet<const bNode *> *ignored_nodes)
{
  int count = 0;
  /* Image Process evaluates zone boundaries as ordinary nodes in one flat schedule. Count their
   * links like all other node links so results survive until their consumers have run. */
  if (schedule.node_group.type == NTREE_IMAGE && schedule.zone == nullptr) {
    for (const bNodeSocket *input : output.logically_linked_sockets()) {
      if (!schedule.unneeded_inputs.contains(input) &&
          (!ignored_nodes || !ignored_nodes->contains(&input->owner_node())) &&
          schedule.nodes.contains(&input->owner_node()))
      {
        count++;
      }
    }
    return count;
  }
  Set<const bke::bNodeTreeZone *> child_zones_counted;
  const bke::bNodeTreeZones &zones = *schedule.node_group.zones();
  for (const bNodeSocket *input : output.logically_linked_sockets()) {
    /* Input is explicitly ignored by the schedule and is thus not referenced. */
    if (schedule.unneeded_inputs.contains(input)) {
      continue;
    }

    /* The node of the input is explicitly ignored and is thus not referenced. */
    if (ignored_nodes && ignored_nodes->contains(&input->owner_node())) {
      continue;
    }

    /* If the input is in the same zone being scheduled and is part of the schedule, it is
     * referenced. */
    const bke::bNodeTreeZone *target_zone = zones.get_zone_by_socket(*input);
    if (target_zone == schedule.zone && schedule.nodes.contains(&input->owner_node())) {
      count++;
      continue;
    }

    /* An exception to the above condition for inputs of zone input nodes. If the zone retrieved
     * from the input socket is not the same as that of the owner node, that means the input is on
     * a zone input node. In that case, we check the schedule for the zone output node, since the
     * zone input does not exist in the schedule. */
    const bke::bNodeTreeZone *target_node_zone = zones.get_zone_by_node(
        input->owner_node().identifier);
    if (target_zone == schedule.zone && target_zone != target_node_zone &&
        schedule.nodes.contains(target_node_zone->output_node()))
    {
      count++;
      continue;
    }

    /* If the input is not in a zone that is a descendant of the zone being scheduled, then it is
     * not referenced. */
    if (!target_zone || (schedule.zone && !schedule.zone->contains_zone_recursively(*target_zone)))
    {
      continue;
    }

    /* Get the immediate child of the zone being scheduled in the nested zone chain toward the
     * target. */
    const bke::bNodeTreeZone *child_zone = zones.get_zones_to_enter(schedule.zone, target_zone)[0];

    /* The child zone is not scheduled, so the input is not referenced. */
    if (!schedule.nodes.contains(child_zone->output_node())) {
      continue;
    }

    /* Zones consider all incoming links from the same output as a single input, so we skip
     * counting the child zone if it was already counted. */
    if (child_zones_counted.contains(child_zone)) {
      continue;
    }

    count++;
    child_zones_counted.add_new(child_zone);
  }
  return count;
}

}  // namespace blender::compositor
