/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string.hh"

#include "DNA_node_types.h"
#include "DNA_object_types.h"

#include "BKE_node.hh"

#include "RNA_enum_types.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_object_util.hh"

namespace blender::nodes::node_object_create_cc {

NODE_STORAGE_FUNCS(NodeObjectCreate)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Object>("Object"_ustr);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  auto *storage = MEM_new<NodeObjectCreate>(__func__);
  storage->object_type = OB_MESH;
  STRNCPY(storage->name, "Object");
  node->storage = storage;
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "object_type", UI_ITEM_NONE, std::nullopt, ICON_NONE);
  layout.prop(ptr, "object_name", UI_ITEM_NONE, IFACE_("Name"), ICON_NONE);
}

static void node_register()
{
  static bke::bNodeType ntype;
  obj_node_type_base(&ntype, "ObjectNodeCreate"_ustr);
  ntype.ui_name = "Create Object";
  ntype.ui_description =
      "Create a scene object (reused on recook). Connect to Object Output to instantiate";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  bke::node_type_storage(
      ntype, "NodeObjectCreate", node_free_standard_storage, node_copy_standard_storage);
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_object_create_cc
