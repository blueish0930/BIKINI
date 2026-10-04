/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_object_util.hh"

namespace blender::nodes::node_object_output_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Object>("Object"_ustr)
      .multi_input()
      .description("Objects whose node graph is written to the scene");
}

static void node_register()
{
  static bke::bNodeType ntype;
  obj_node_type_base(&ntype, "ObjectNodeOutput"_ustr);
  ntype.ui_name = "Object Output";
  ntype.ui_description =
      "Apply every connected object graph to the scene, like Join Geometry. "
      "All used inputs stay live; nodes that do not reach an Object Output are not written";
  ntype.nclass = NODE_CLASS_OUTPUT;
  ntype.declare = node_declare;
  ntype.no_muting = true;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_object_output_cc
