/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_object_util.hh"

namespace blender::nodes::node_object_set_visibility_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Object>("Object"_ustr);
  b.add_input<decl::Bool>("Hide Viewport"_ustr);
  b.add_input<decl::Bool>("Hide Select"_ustr);
  b.add_input<decl::Bool>("Hide Render"_ustr);
  b.add_input<decl::Bool>("Holdout"_ustr);
  b.add_input<decl::Bool>("Shadow Catcher"_ustr);
  b.add_output<decl::Object>("Object"_ustr);
}

static void node_register()
{
  static bke::bNodeType ntype;
  obj_node_type_base(&ntype, "ObjectNodeSetVisibility"_ustr);
  ntype.ui_name = "Set Visibility";
  ntype.ui_description = "Set viewport, select and render visibility, holdout and shadow catcher";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_object_set_visibility_cc
