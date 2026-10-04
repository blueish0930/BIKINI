/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "NOD_geometry_nodes_lazy_function.hh"

namespace blender {

namespace nodes::node_geo_modal_timer_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Bool>("Enable"_ustr)
      .default_value(true)
      .structure_type(StructureType::Single)
      .description(
          "Keep the tool running so that it can react to more events instead of ending execution");
  b.add_output<decl::Bool>("Is Timer Event"_ustr)
      .structure_type(StructureType::Single)
      .description("True when this execution was started by the timer instead of by user input");
}

class LazyFunctionForModalTimerNode : public LazyFunction {
  const bNode &node_;

 public:
  LazyFunctionForModalTimerNode(const bNode &node, MutableSpan<int> r_lf_index_by_bsocket)
      : node_(node)
  {
    debug_name_ = node.name;
    r_lf_index_by_bsocket[node.input_socket(0).index_in_tree()] = inputs_.append_and_get_index_as(
        "Enable", CPPType::get<SocketValueVariant>());
    r_lf_index_by_bsocket[node.output_socket(0).index_in_tree()] =
        outputs_.append_and_get_index_as("Is Timer Event", CPPType::get<SocketValueVariant>());
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const override
  {
    const GeoNodesUserData &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
    const GeoNodesOperatorData *operator_data = user_data.call_data->operator_data;
    if (!operator_data || !operator_data->modal_requested) {
      GeoNodesLocalUserData &local_user_data = *static_cast<GeoNodesLocalUserData *>(
          context.local_user_data);
      if (eval_log::NodeTreeLogger *tree_logger = local_user_data.try_get_tree_logger(user_data)) {
        tree_logger->node_warnings.append(
            *tree_logger->allocator,
            {node_.identifier, {NodeWarningType::Error, TIP_("Node must be run as tool")}});
      }
      set_default_remaining_node_outputs(params, node_);
      return;
    }

    /* The node is evaluated as a side-effect node, so the "Enable" input is always available. */
    const SocketValueVariant &enable_variant = params.get_input<SocketValueVariant>(0);
    if (enable_variant.is_single() && enable_variant.copy_as<bool>()) {
      /* Other modal nodes may set these at the same time, so they must never be reset here. */
      operator_data->modal_requested->store(true, std::memory_order_relaxed);
      operator_data->timer_requested->store(true, std::memory_order_relaxed);
    }

    params.set_output(0, SocketValueVariant(operator_data->is_timer_event));
  }
};

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeModalTimer"_ustr);
  ntype.ui_name = "Modal Timer";
  ntype.ui_description =
      "Keep the tool running after the first execution, and run it repeatedly with a timer";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops_for_tool_node;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace nodes::node_geo_modal_timer_cc

namespace nodes {

std::unique_ptr<LazyFunction> get_modal_timer_node_lazy_function(
    const bNode &node, GeometryNodesLazyFunctionGraphInfo &own_lf_graph_info)
{
  using namespace node_geo_modal_timer_cc;
  return std::make_unique<LazyFunctionForModalTimerNode>(
      node, own_lf_graph_info.mapping.lf_index_by_bsocket);
}

}  // namespace nodes
}  // namespace blender
