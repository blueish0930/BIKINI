/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_node_types.h"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_object_util.hh"

namespace blender::nodes::node_object_set_transform_cc {

NODE_STORAGE_FUNCS(NodeObjectSetTransform)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Object>("Object"_ustr);
  b.add_input<decl::Vector>("Location"_ustr).subtype(PROP_TRANSLATION);
  b.add_input<decl::Vector>("Rotation"_ustr).subtype(PROP_EULER);
  b.add_input<decl::Vector>("Scale"_ustr).default_value({1.0f, 1.0f, 1.0f}).subtype(PROP_XYZ);
  b.add_output<decl::Object>("Object"_ustr);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->storage = MEM_new<NodeObjectSetTransform>(__func__);
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "apply_location", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "apply_rotation", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "apply_scale", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

static void node_register()
{
  static bke::bNodeType ntype;
  obj_node_type_base(&ntype, "ObjectNodeSetTransform"_ustr);
  ntype.ui_name = "Set Transform";
  ntype.ui_description = "Write location, rotation and scale of the incoming object";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  bke::node_type_storage(
      ntype, "NodeObjectSetTransform", node_free_standard_storage, node_copy_standard_storage);
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_object_set_transform_cc
