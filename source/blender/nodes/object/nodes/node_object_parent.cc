/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_node_types.h"

#include "RNA_access.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_object_util.hh"

namespace blender::nodes::node_object_parent_cc {

NODE_STORAGE_FUNCS(NodeObjectParent)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Object>("Object"_ustr).description("Child object");
  b.add_input<decl::Object>("Parent"_ustr).description("Parent object (Bind mode only)");
  b.add_output<decl::Object>("Object"_ustr);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->storage = MEM_new<NodeObjectParent>(__func__);
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "mode", ui::ITEM_R_EXPAND, IFACE_("Mode"), ICON_NONE);

  const NodeObjectParent &storage = node_storage(*ptr->data_as<bNode>());
  if (storage.mode == 0) {
    layout.prop(ptr, "bind_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
    layout.prop(ptr, "keep_transform", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  }
  else {
    layout.prop(ptr, "clear_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
  }
}

static void node_register()
{
  static bke::bNodeType ntype;
  obj_node_type_base(&ntype, "ObjectNodeSetParent"_ustr);
  ntype.ui_name = "Set Parent";
  ntype.ui_description =
      "Bind or clear object parenting. Bind: Object / Object without inverse, with optional "
      "Keep Transform. Clear: Clear Parent, keep transform, or reset the parent inverse";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.default_width = 200;
  bke::node_type_storage(
      ntype, "NodeObjectParent", node_free_standard_storage, node_copy_standard_storage);
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_object_parent_cc
