/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_geometry_nodes_closure.hh"
#include "NOD_socket_search_link.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_set_closure_default_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Closure>("Closure"_ustr).description("Main closure. Empty falls back to Default Closure");
  b.add_input<decl::Closure>("Default Closure"_ustr)
      .description(
          "Closure used when the main Closure is empty, and copied as the template for new "
          "Closure Zones");
  b.add_output<decl::Closure>("Closure"_ustr)
      .pass_through_input_index(0)
      .propagate_all()
      .description("The main closure, or Default Closure when the main input is empty");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  ClosurePtr closure = params.extract_input<ClosurePtr>("Closure"_ustr);
  ClosurePtr default_closure = params.extract_input<ClosurePtr>("Default Closure"_ustr);
  if (!closure) {
    closure = std::move(default_closure);
  }
  params.set_output("Closure"_ustr, std::move(closure));
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  if (params.other_socket().type != SOCK_CLOSURE) {
    return;
  }
  search_link_ops_for_basic_node(params);
}

static const bNodeSocket *node_internally_linked_input(const bNodeTree & /*tree*/,
                                                       const bNode &node,
                                                       const bNodeSocket &output_socket)
{
  return output_socket.type == SOCK_CLOSURE ? &node.input_socket(0) : nullptr;
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeSetClosureDefault"_ustr, GEO_NODE_SET_CLOSURE_DEFAULT);
  ntype.ui_name = "Set Default Closure";
  ntype.ui_description =
      "Use a default closure when the main closure is empty and as the template for new Closure "
      "Zones";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.internally_linked_input = node_internally_linked_input;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_set_closure_default_cc
