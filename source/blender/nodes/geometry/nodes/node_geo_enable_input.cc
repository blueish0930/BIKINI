/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_scene.hh"

#include "DEG_depsgraph_query.hh"

#include "DNA_node_tree_interface_types.h"

#include "NOD_geometry_nodes_lazy_function.hh"
#include "NOD_node_declaration.hh"
#include "NOD_node_extra_info.hh"
#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"

namespace blender {

namespace nodes::node_geo_enable_input_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_default_layout();
  b.add_input<decl::Bool>("Enable"_ustr)
      .default_value(false)
      .structure_type(StructureType::Single)
      .description("When true, the Value input is shown and used");

  const bNode *node = b.node_or_null();
  if (!node) {
    return;
  }
  const eNodeSocketDatatype data_type = eNodeSocketDatatype(node->custom1);

  b.add_input(data_type, "Value"_ustr).hide_value().structure_type(StructureType::Dynamic);
  b.add_output(data_type, "Value"_ustr)
      .align_with_previous()
      .inferred_structure_type()
      .propagate_all();
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

/**
 * Find which group-interface input is controlled by this Enable Input (Value ← Group Input).
 * Uses logical links so reroutes between Group Input and Enable Input still resolve.
 */
static int find_controlled_interface_input_index(const bNode &enable_input_node)
{
  const bNodeTree &tree = enable_input_node.owner_tree();
  tree.ensure_topology_cache();
  tree.ensure_interface_cache();
  const bNodeSocket *value_socket = enable_input_node.input_by_identifier("Value"_ustr);
  if (!value_socket) {
    return -1;
  }
  for (const bNodeSocket *origin : value_socket->logically_linked_sockets()) {
    if (!origin->is_output() || !origin->owner_node().is_group_input()) {
      continue;
    }
    const int input_i = tree.interface_input_index_by_identifier(origin->identifier);
    if (input_i != -1) {
      return input_i;
    }
  }
  return -1;
}

static bool socket_value_variant_is_set(const SocketValueVariant &value)
{
  return value.is_single() || value.is_field() || value.is_list() || value.is_volume_grid();
}

/**
 * Build the value the group interface would use when nothing is wired in from the outside:
 * either a static interface default (#NODE_DEFAULT_INPUT_VALUE) or an implicit default input
 * (Position / Normal / Index / Scene Frame / Self Object / …).
 */
static SocketValueVariant read_group_interface_socket_value(const bNodeTree &tree,
                                                           const int input_i,
                                                           const bNode &context_node,
                                                           const lf::Context *lf_context)
{
  tree.ensure_interface_cache();
  if (!tree.interface_inputs().index_range().contains(input_i)) {
    return {};
  }
  const bNodeTreeInterfaceSocket &io_socket = *tree.interface_inputs()[input_i];
  const NodeDefaultInputType default_input = NodeDefaultInputType(io_socket.default_input);

  /* Field-style defaults (Position, Normal, Index, …). */
  if (std::optional<ImplicitInputValueFn> implicit_fn = get_implicit_input_value_fn(default_input))
  {
    alignas(SocketValueVariant) char storage[sizeof(SocketValueVariant)];
    (*implicit_fn)(context_node, storage);
    SocketValueVariant value = std::move(*reinterpret_cast<SocketValueVariant *>(storage));
    std::destroy_at(reinterpret_cast<SocketValueVariant *>(storage));
    return value;
  }

  /* Context-dependent defaults. */
  if (default_input == NODE_DEFAULT_INPUT_SCENE_FRAME && lf_context) {
    const auto *user_data = dynamic_cast<const GeoNodesUserData *>(lf_context->user_data);
    const Depsgraph *depsgraph = nullptr;
    if (user_data && user_data->call_data) {
      if (user_data->call_data->modifier_data) {
        depsgraph = user_data->call_data->modifier_data->depsgraph;
      }
      else if (user_data->call_data->operator_data) {
        depsgraph = user_data->call_data->operator_data->depsgraphs->active;
      }
    }
    const bke::bNodeSocketType *stype = io_socket.socket_typeinfo();
    if (!depsgraph || !stype) {
      if (stype && stype->geometry_nodes_default_value) {
        return *stype->geometry_nodes_default_value;
      }
      return {};
    }
    const Scene *scene = DEG_get_input_scene(depsgraph);
    const float scene_ctime = BKE_scene_ctime_get(scene);
    if (stype->type == SOCK_INT) {
      return SocketValueVariant(int(scene_ctime));
    }
    if (stype->type == SOCK_FLOAT) {
      return SocketValueVariant(scene_ctime);
    }
  }
  if (default_input == NODE_DEFAULT_INPUT_SELF_OBJECT && lf_context) {
    const auto *user_data = dynamic_cast<const GeoNodesUserData *>(lf_context->user_data);
    const Object *self_object = (user_data && user_data->call_data) ?
                                    user_data->call_data->self_object() :
                                    nullptr;
    return SocketValueVariant::From(const_cast<Object *>(self_object));
  }

  /* Static interface DNA value (default_input == VALUE, or fallback). */
  const bke::bNodeSocketType *stype = io_socket.socket_typeinfo();
  if (!stype || !io_socket.socket_data) {
    return {};
  }
  if (stype->get_geometry_nodes_cpp_value) {
    return stype->get_geometry_nodes_cpp_value(io_socket.socket_data);
  }
  if (stype->geometry_nodes_default_value) {
    return *stype->geometry_nodes_default_value;
  }
  return {};
}

class LazyFunctionForEnableInputNode : public LazyFunction {
  const bNode &node_;
  /**
   * Index of the group interface input controlled by Value (from Group Input). When Enable is
   * false the interface is hidden: external wires into that group input are ignored and the
   * interface's own current value / Default Input is used instead.
   */
  int interface_input_index_ = -1;

 public:
  LazyFunctionForEnableInputNode(const bNode &node, MutableSpan<int> r_lf_index_by_bsocket)
      : node_(node)
  {
    debug_name_ = node.name;
    r_lf_index_by_bsocket[node.input_socket(0).index_in_tree()] = inputs_.append_and_get_index_as(
        "Enable", CPPType::get<SocketValueVariant>());
    /* Value is optional: when Enable is false we deliberately do not request it so the parent
     * does not evaluate external wires into the controlled group input. */
    r_lf_index_by_bsocket[node.input_socket(1).index_in_tree()] = inputs_.append_and_get_index_as(
        "Value", CPPType::get<SocketValueVariant>(), lf::ValueUsage::Maybe);
    r_lf_index_by_bsocket[node.output_socket(0).index_in_tree()] =
        outputs_.append_and_get_index_as("Value", CPPType::get<SocketValueVariant>());

    interface_input_index_ = find_controlled_interface_input_index(node);
  }

  SocketValueVariant interface_value_for_disabled(const lf::Context &context) const
  {
    const bNodeTree &tree = node_.owner_tree();
    auto try_index = [&](const int input_i) -> SocketValueVariant {
      if (input_i < 0) {
        return {};
      }
      return read_group_interface_socket_value(tree, input_i, node_, &context);
    };

    SocketValueVariant value = try_index(interface_input_index_);
    if (socket_value_variant_is_set(value)) {
      return value;
    }
    /* Re-resolve if topology changed after the LF graph was built. */
    value = try_index(find_controlled_interface_input_index(node_));
    if (socket_value_variant_is_set(value)) {
      return value;
    }
    const bNodeSocket &value_socket = *node_.input_by_identifier("Value"_ustr);
    if (value_socket.typeinfo && value_socket.typeinfo->geometry_nodes_default_value) {
      return *value_socket.typeinfo->geometry_nodes_default_value;
    }
    return {};
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const override
  {
    const bke::SocketValueVariant enable_variant = params.get_input<bke::SocketValueVariant>(0);
    /* Enable=false: ignore the Value wire (external group input). Use the group interface's
     * static default or its Default Input (Position / Normal / …). */
    if (enable_variant.is_single() && !enable_variant.copy_as<bool>()) {
      params.set_output(0, this->interface_value_for_disabled(context));
      return;
    }
    const bke::SocketValueVariant *value_variant =
        params.try_get_input_data_ptr_or_request<bke::SocketValueVariant>(1);
    if (!value_variant) {
      /* Wait until the external / group-input Value is available. */
      return;
    }
    /* Enable=true (or non-single): pass Value through unchanged. */
    params.set_output(0, std::move(*value_variant));
  }
};

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = SOCK_FLOAT;
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
      bNode &node = params.add_node("NodeEnableInput"_ustr);
      node.custom1 = params.socket.type;
      params.update_and_connect_available_socket(node, "Value"_ustr);
    });
    return;
  }

  /* Prefer connecting to the controlled value. Only promote Enable for boolean drags. */
  int value_weight = 0;
  if (other_type == SOCK_BOOLEAN) {
    params.add_item(IFACE_("Enable"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("NodeEnableInput"_ustr);
      params.update_and_connect_available_socket(node, "Enable"_ustr);
    });
    value_weight = -1;
  }

  params.add_item(
      IFACE_("Value"),
      [](LinkSearchOpParams &params) {
        bNode &node = params.add_node("NodeEnableInput"_ustr);
        node.custom1 = params.socket.type;
        params.update_and_connect_available_socket(node, "Value"_ustr);
      },
      value_weight);
}

using namespace blender::compositor;

class EnableInputOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    const Result &input = this->get_input("Value");
    Result &output = this->get_result("Value");
    output.share_data(input);
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new EnableInputOperation(context, node);
}

static void node_extra_info(NodeExtraInfoParams &params)
{
  params.tree.ensure_topology_cache();
  const bNodeSocket &value_socket = *params.node.input_by_identifier("Value"_ustr);
  if (!value_socket.is_directly_linked()) {
    return;
  }
  for (const bNodeSocket *origin_socket : value_socket.logically_linked_sockets()) {
    if (!origin_socket->owner_node().is_group_input()) {
      NodeExtraInfoRow row;
      row.text = RPT_("Invalid Input Link");
      row.tooltip = TIP_("The value should be linked from the group input node");
      row.icon = ICON_STATUS_ERROR;
      params.rows.append(std::move(row));
      return;
    }
  }
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
                    SOCK_FLOAT,
                    data_type_items_callback);
}

static const bNodeSocket *node_internally_linked_input(const bNodeTree & /*tree*/,
                                                       const bNode &node,
                                                       const bNodeSocket &output_socket)
{
  /* Internal links should always map corresponding input and output sockets. */
  return node.input_by_identifier(output_socket.identifier_ustr());
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_cmp_node_type_base(&ntype, "NodeEnableInput"_ustr);
  ntype.ui_name = "Enable Input";
  ntype.ui_description = "Show and use an input only when the enable value is true";
  ntype.nclass = NODE_CLASS_INTERFACE;
  /* Keep this node's own sockets always visible; only the controlled group interface should hide. */
  ntype.ignore_inferred_input_socket_visibility = true;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.get_compositor_operation = get_compositor_operation;
  ntype.get_extra_info = node_extra_info;
  ntype.internally_linked_input = node_internally_linked_input;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace nodes::node_geo_enable_input_cc

namespace nodes {

std::unique_ptr<LazyFunction> get_enable_input_node_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &own_lf_graph_info)
{
  using namespace node_geo_enable_input_cc;
  return std::make_unique<LazyFunctionForEnableInputNode>(
      node, own_lf_graph_info.mapping.lf_index_by_bsocket);
}

}  // namespace nodes
}  // namespace blender