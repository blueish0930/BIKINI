/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_object_enums.h"
#include "DNA_object_types.h"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_object_util.hh"

namespace blender::nodes::node_object_viewport_display_cc {

NODE_STORAGE_FUNCS(NodeObjectViewportDisplay)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Object>("Object"_ustr);
  b.add_input<decl::Bool>("Show Name"_ustr);
  b.add_input<decl::Bool>("Show Axis"_ustr);
  b.add_input<decl::Bool>("Show Wire"_ustr);
  b.add_input<decl::Bool>("In Front"_ustr);
  b.add_input<decl::Bool>("Show Bounds"_ustr);
  b.add_input<decl::Color>("Color"_ustr).default_value(ColorGeometry4f(1.0f, 1.0f, 1.0f, 1.0f));
  b.add_output<decl::Object>("Object"_ustr);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  auto *storage = MEM_new<NodeObjectViewportDisplay>(__func__);
  storage->display_type = OB_TEXTURE;
  storage->bounds_type = OB_BOUND_BOX;
  node->storage = storage;
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "display_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "bounds_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
}

static void node_register()
{
  static bke::bNodeType ntype;
  obj_node_type_base(&ntype, "ObjectNodeViewportDisplay"_ustr);
  ntype.ui_name = "Viewport Display";
  ntype.ui_description = "Set object viewport display type, extras, bounds and object color";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  bke::node_type_storage(
      ntype, "NodeObjectViewportDisplay", node_free_standard_storage, node_copy_standard_storage);
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_object_viewport_display_cc
