/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_compute_contexts.hh"
#include "BKE_node_runtime.hh"

#include "NOD_geometry_nodes_bundle.hh"
#include "NOD_geometry_nodes_closure.hh"
#include "NOD_geometry_nodes_lazy_function.hh"
#include "NOD_node_extra_info.hh"
#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_enum_types.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender {

namespace nodes::node_geo_set_group_input_default_cc {

}  // namespace nodes::node_geo_set_group_input_default_cc

namespace nodes {

/** Geometry is always valid. Every other type needs Hide Value on the group interface. */
static bool group_input_socket_is_supported(const bNodeTree &tree, const bNodeSocket &group_input)
{
  const bNodeTreeInterfaceSocket *iface = bke::node_find_interface_input_by_identifier(
      tree, group_input.identifier);
  eNodeSocketDatatype type = eNodeSocketDatatype(group_input.type);
  if (iface != nullptr && iface->socket_typeinfo() != nullptr) {
    type = eNodeSocketDatatype(iface->socket_typeinfo()->type);
  }
  if (type == SOCK_GEOMETRY) {
    return true;
  }
  if (iface != nullptr) {
    return (iface->flag & NODE_INTERFACE_SOCKET_HIDE_VALUE) != 0;
  }
  return (group_input.flag & SOCK_HIDE_VALUE) != 0;
}

const bNodeSocket *find_set_group_input_default_source_socket(const bNode &node)
{
  const bNodeTree &tree = node.owner_tree();
  tree.ensure_topology_cache();
  const bNodeSocket *value_socket = node.input_by_identifier("Value"_ustr);
  if (!value_socket) {
    return nullptr;
  }
  for (const bNodeSocket *origin : value_socket->logically_linked_sockets()) {
    if (origin->is_output() && origin->owner_node().is_group_input()) {
      const int input_i = tree.interface_input_index_by_identifier(origin->identifier);
      if (input_i != -1) {
        return origin;
      }
    }
  }
  return nullptr;
}

const bNode *find_set_group_input_default_for_identifier(const bNodeTree &tree,
                                                         const StringRef identifier)
{
  tree.ensure_topology_cache();
  for (const bNode *node : tree.nodes_by_type("GeometryNodeSetGroupInputDefault"_ustr)) {
    if (node->is_muted()) {
      continue;
    }
    const bNodeSocket *source = find_set_group_input_default_source_socket(*node);
    if (source && source->identifier == identifier &&
        group_input_socket_is_supported(tree, *source))
    {
      return node;
    }
  }
  return nullptr;
}

}  // namespace nodes

namespace nodes::node_geo_set_group_input_default_cc {

static const bke::GroupNodeComputeContext *find_caller_group_context(
    const ComputeContext *context)
{
  for (const ComputeContext *ctx = context; ctx; ctx = ctx->parent()) {
    if (const auto *group_ctx = dynamic_cast<const bke::GroupNodeComputeContext *>(ctx)) {
      return group_ctx;
    }
  }
  return nullptr;
}

/**
 * True when the calling group-node socket is wired (including through reroutes).
 * No caller group (modifier root / operator) counts as linked: those inputs are
 * provided by the modifier/operator UI, not by this node.
 */
static bool caller_group_input_is_linked(const ComputeContext *context,
                                         const bNodeSocket &group_input_socket)
{
  const bke::GroupNodeComputeContext *group_ctx = find_caller_group_context(context);
  if (!group_ctx) {
    return true;
  }
  const bNode *group_node = group_ctx->node();
  const bNodeTree *caller_tree = group_ctx->tree();
  if (!group_node || !caller_tree) {
    return true;
  }
  caller_tree->ensure_topology_cache();
  const bNodeSocket *caller_socket = group_node->input_by_identifier(
      group_input_socket.identifier_ustr());
  if (!caller_socket) {
    return true;
  }
  return caller_socket->is_logically_linked();
}

static bool caller_group_input_is_linked(const lf::Context &lf_context,
                                         const bNodeSocket &group_input_socket)
{
  const auto *user_data = dynamic_cast<const GeoNodesUserData *>(lf_context.user_data);
  return caller_group_input_is_linked(user_data ? user_data->compute_context : nullptr,
                                      group_input_socket);
}

static void add_node_warning(const lf::Context &context,
                             const bNode &node,
                             const NodeWarningType type,
                             const StringRef message)
{
  if (context.user_data == nullptr || context.local_user_data == nullptr) {
    return;
  }
  const auto &user_data = *static_cast<const GeoNodesUserData *>(context.user_data);
  const auto &local_user_data = *static_cast<const GeoNodesLocalUserData *>(
      context.local_user_data);
  eval_log::NodeTreeLogger *tree_logger = local_user_data.try_get_tree_logger(user_data);
  if (tree_logger == nullptr) {
    return;
  }
  tree_logger->node_warnings.append(*tree_logger->allocator,
                                    {node.identifier, NodeWarning(type, message)});
}

static bool socket_value_is_empty_fallback(const SocketValueVariant &value)
{
  if (!value.is_single() && !value.is_field() && !value.is_list() && !value.is_volume_grid()) {
    return true;
  }
  if (!value.is_single()) {
    return false;
  }
  switch (value.socket_type()) {
    case SOCK_GEOMETRY:
      return value.copy_as<GeometrySet>().is_empty();
    case SOCK_CLOSURE:
      return !value.copy_as<ClosurePtr>();
    case SOCK_BUNDLE: {
      const BundlePtr bundle = value.copy_as<BundlePtr>();
      return !bundle || bundle->is_empty();
    }
    default:
      return false;
  }
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_default_layout();

  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }
  const eNodeSocketDatatype data_type = eNodeSocketDatatype(node->custom1);

  b.add_input(data_type, "Value"_ustr)
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Group input used when that socket is linked outside the group");
  b.add_output(data_type, "Value"_ustr)
      .align_with_previous()
      .inferred_structure_type()
      .propagate_all()
      .description("The group input, or Default when the group node input is unlinked");
  b.add_input(data_type, "Default"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Used when the matching group input is not linked outside the group");
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

class LazyFunctionForSetGroupInputDefaultNode : public LazyFunction {
  const bNode &node_;

 public:
  LazyFunctionForSetGroupInputDefaultNode(const bNode &node,
                                          MutableSpan<int> r_lf_index_by_bsocket)
      : node_(node)
  {
    debug_name_ = node.name;
    r_lf_index_by_bsocket[node.input_socket(0).index_in_tree()] = inputs_.append_and_get_index_as(
        "Value", CPPType::get<SocketValueVariant>(), lf::ValueUsage::Maybe);
    r_lf_index_by_bsocket[node.input_socket(1).index_in_tree()] = inputs_.append_and_get_index_as(
        "Default", CPPType::get<SocketValueVariant>(), lf::ValueUsage::Maybe);
    r_lf_index_by_bsocket[node.output_socket(0).index_in_tree()] =
        outputs_.append_and_get_index_as("Value", CPPType::get<SocketValueVariant>());
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const override
  {
    const int value_i = 0;
    const int default_i = 1;

    if (const bNodeSocket *group_input = find_set_group_input_default_source_socket(node_)) {
      if (!group_input_socket_is_supported(node_.owner_tree(), *group_input)) {
        /* Not Hide Value and not Geometry: ignore this node and keep the group socket value. */
        add_node_warning(context,
                         node_,
                         NodeWarningType::Info,
                         TIP_("Interface is not Hide Value. This node is ignored; the group "
                              "socket value is used"));
        params.set_input_unused(default_i);
        const SocketValueVariant *value =
            params.try_get_input_data_ptr_or_request<SocketValueVariant>(value_i);
        if (!value) {
          return;
        }
        params.set_output(0, *value);
        return;
      }
      const bool is_linked = caller_group_input_is_linked(context, *group_input);
      const int used_i = is_linked ? value_i : default_i;
      const int unused_i = is_linked ? default_i : value_i;
      params.set_input_unused(unused_i);
      const SocketValueVariant *used = params.try_get_input_data_ptr_or_request<SocketValueVariant>(
          used_i);
      if (!used) {
        return;
      }
      params.set_output(0, *used);
      return;
    }

    const bNodeSocket *value_socket = node_.input_by_identifier("Value"_ustr);
    if (value_socket && value_socket->is_logically_linked()) {
      add_node_warning(
          context, node_, NodeWarningType::Error, TIP_("Value is not a Group Input"));
      params.set_input_unused(default_i);
      const SocketValueVariant *value =
          params.try_get_input_data_ptr_or_request<SocketValueVariant>(value_i);
      if (!value) {
        return;
      }
      params.set_output(0, *value);
      return;
    }

    /* Nothing connected yet: keep the empty-value fallback so an unlinked socket still previews. */
    const SocketValueVariant *value = params.try_get_input_data_ptr_or_request<SocketValueVariant>(
        value_i);
    if (!value) {
      return;
    }
    if (socket_value_is_empty_fallback(*value)) {
      const SocketValueVariant *default_value =
          params.try_get_input_data_ptr_or_request<SocketValueVariant>(default_i);
      if (!default_value) {
        return;
      }
      params.set_output(0, *default_value);
      return;
    }
    params.set_input_unused(default_i);
    params.set_output(0, *value);
  }
};

class LazyFunctionForSetGroupInputDefaultSocketUsage : public LazyFunction {
  const bNode &node_;

 public:
  explicit LazyFunctionForSetGroupInputDefaultSocketUsage(const bNode &node) : node_(node)
  {
    debug_name_ = "Set Group Input Default Socket Usage";
    inputs_.append_as("Output Used", CPPType::get<bool>());
    outputs_.append_as("Value Used", CPPType::get<bool>());
    outputs_.append_as("Default Used", CPPType::get<bool>());
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const override
  {
    const bool output_used = params.get_input<bool>(0);
    if (!output_used) {
      params.set_output(0, false);
      params.set_output(1, false);
      return;
    }
    if (const bNodeSocket *group_input = find_set_group_input_default_source_socket(node_)) {
      if (!group_input_socket_is_supported(node_.owner_tree(), *group_input)) {
        params.set_output(0, true);
        params.set_output(1, false);
        return;
      }
      const bool is_linked = caller_group_input_is_linked(context, *group_input);
      params.set_output(0, is_linked);
      params.set_output(1, !is_linked);
      return;
    }
    const bNodeSocket *value_socket = node_.input_by_identifier("Value"_ustr);
    if (value_socket && value_socket->is_logically_linked()) {
      /* Wrong link: pass Value through and do not evaluate Default. */
      params.set_output(0, true);
      params.set_output(1, false);
      return;
    }
    /* Empty-fallback path may need both inputs. */
    params.set_output(0, true);
    params.set_output(1, true);
  }
};

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = SOCK_GEOMETRY;
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const eNodeSocketDatatype other_type = eNodeSocketDatatype(params.other_socket().type);
  if (!params.node_tree().typeinfo->valid_socket_type(
          params.node_tree().typeinfo, bke::node_socket_type_find_static(other_type)))
  {
    return;
  }

  if (params.in_out() == SOCK_OUT) {
    params.add_item(IFACE_("Value"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("GeometryNodeSetGroupInputDefault"_ustr);
      node.custom1 = params.socket.type;
      params.update_and_connect_available_socket(node, "Value"_ustr);
    });
    return;
  }

  params.add_item(IFACE_("Value"), [](LinkSearchOpParams &params) {
    bNode &node = params.add_node("GeometryNodeSetGroupInputDefault"_ustr);
    node.custom1 = params.socket.type;
    params.update_and_connect_available_socket(node, "Value"_ustr);
  });
  params.add_item(IFACE_("Default"), [](LinkSearchOpParams &params) {
    bNode &node = params.add_node("GeometryNodeSetGroupInputDefault"_ustr);
    node.custom1 = params.socket.type;
    params.update_and_connect_available_socket(node, "Default"_ustr);
  });
}

static void node_extra_info(NodeExtraInfoParams &params)
{
  params.tree.ensure_topology_cache();
  const bNodeSocket &value_socket = *params.node.input_by_identifier("Value"_ustr);
  if (!value_socket.is_logically_linked()) {
    NodeExtraInfoRow row;
    row.text = RPT_("Connect Group Input");
    row.tooltip = TIP_(
        "Link Value from the Group Input. When that group socket is unlinked outside the group, "
        "every Group Input node returns Default for this socket");
    row.icon = ICON_INFO;
    params.rows.append(std::move(row));
    return;
  }
  const bNodeSocket *group_input = find_set_group_input_default_source_socket(params.node);
  if (!group_input) {
    NodeExtraInfoRow row;
    row.text = RPT_("Value is not a Group Input");
    row.tooltip = TIP_("Value must be linked from a Group Input node in this tree");
    row.icon = ICON_ERROR;
    params.rows.append(std::move(row));
    return;
  }
  if (!group_input_socket_is_supported(params.tree, *group_input)) {
    NodeExtraInfoRow row;
    row.text = RPT_("Interface is not Hide Value");
    row.tooltip = TIP_(
        "This node is ignored. The group keeps the value on the group node. Enable Hide Value "
        "on this interface, or use a Geometry socket, for Default to apply");
    row.icon = ICON_INFO;
    params.rows.append(std::move(row));
    return;
  }
  NodeExtraInfoRow row;
  row.text = RPT_("Applies to all Group Inputs");
  row.tooltip = TIP_(
      "Every Group Input node in this group uses Default for this socket when the group is "
      "unlinked outside");
  row.icon = ICON_INFO;
  params.rows.append(std::move(row));
}

static const EnumPropertyItem *data_type_items_callback(bContext * /*C*/,
                                                        PointerRNA *ptr,
                                                        PropertyRNA * /*prop*/,
                                                        bool *r_free)
{
  *r_free = true;
  const bNodeTree &ntree = *reinterpret_cast<bNodeTree *>(ptr->owner_id);
  bke::bNodeTreeType *ntree_type = ntree.typeinfo;
  return enum_items_filter(
      rna_enum_node_socket_data_type_items, [&](const EnumPropertyItem &item) -> bool {
        bke::bNodeSocketType *socket_type = bke::node_socket_type_find_static(item.value);
        return ntree_type->valid_socket_type(ntree_type, socket_type);
      });
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "",
                    rna_enum_node_socket_data_type_items,
                    NOD_inline_enum_accessors(custom1),
                    SOCK_GEOMETRY,
                    data_type_items_callback);
}

static const bNodeSocket *node_internally_linked_input(const bNodeTree & /*tree*/,
                                                       const bNode &node,
                                                       const bNodeSocket &output_socket)
{
  return node.input_by_identifier(output_socket.identifier_ustr());
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, "GeometryNodeSetGroupInputDefault"_ustr, GEO_NODE_SET_GROUP_INPUT_DEFAULT);
  ntype.ui_name = "Set Group Input Default";
  ntype.ui_description =
      "When the matching group input is not linked outside the group, every Group Input "
      "node in this tree returns Default for that socket";
  ntype.enum_name_legacy = "SET_GROUP_INPUT_DEFAULT";
  ntype.nclass = NODE_CLASS_INTERFACE;
  ntype.ignore_inferred_input_socket_visibility = true;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.get_extra_info = node_extra_info;
  ntype.internally_linked_input = node_internally_linked_input;
  bke::node_register_type(ntype);

  /* RNA may be missing if this exe was linked against a stale bf_rna. Never call
   * RNA_def_property on a null srna — that crashes Blender during startup. */
  if (ntype.rna_ext.srna) {
    node_rna(ntype.rna_ext.srna);
  }
}
NOD_REGISTER_NODE(node_register)

}  // namespace nodes::node_geo_set_group_input_default_cc

namespace nodes {

std::unique_ptr<LazyFunction> get_set_group_input_default_node_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &own_lf_graph_info)
{
  using namespace node_geo_set_group_input_default_cc;
  return std::make_unique<LazyFunctionForSetGroupInputDefaultNode>(
      node, own_lf_graph_info.mapping.lf_index_by_bsocket);
}

std::unique_ptr<LazyFunction> get_set_group_input_default_socket_usage_lazy_function(
    const bNode &node)
{
  using namespace node_geo_set_group_input_default_cc;
  return std::make_unique<LazyFunctionForSetGroupInputDefaultSocketUsage>(node);
}

}  // namespace nodes
}  // namespace blender
