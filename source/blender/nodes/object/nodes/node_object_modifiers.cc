/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Set Modifiers draws the incoming object's real modifier stack.
 *
 * Node button layouts have no root #Panel (`block->panel == nullptr`). Calling
 * `layout.panel()` / `layout.panel_prop()` or native modifier PanelType draw
 * functions crashes. Expand/collapse uses Modifier.show_expanded and the body
 * is drawn from RNA (including Geometry Nodes `node_group` and Subsurf levels).
 */

#include "BLI_set.hh"
#include "BLI_string_utf8.hh"
#include "BLI_string_ref.hh"

#include "DNA_ID.h"
#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"

#include "BLI_listbase.hh"

#include "BKE_context.hh"
#include "BKE_modifier.hh"
#include "BKE_node.hh"
#include "BKE_object.hh"

#include "BLO_read_write.hh"

#include "BLT_translation.hh"

#include "MEM_guardedalloc.h"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "node_object_util.hh"

namespace blender::nodes::node_object_modifiers_cc {

NODE_STORAGE_FUNCS(NodeObjectModifiers)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Object>("Object"_ustr);
  b.add_output<decl::Object>("Object"_ustr);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  /* Kept for .blend compatibility. The node edits the object's real modifier stack. */
  node->storage = MEM_new<NodeObjectModifiers>(__func__);
}

static void node_free_storage(bNode *node)
{
  auto *storage = static_cast<NodeObjectModifiers *>(node->storage);
  if (storage) {
    MEM_SAFE_DELETE(storage->items);
    MEM_delete(storage);
  }
}

static void node_copy_storage(bNodeTree * /*dst*/, bNode *dst_node, const bNode *src_node)
{
  const NodeObjectModifiers &src = node_storage(*src_node);
  auto *dst = MEM_new<NodeObjectModifiers>(__func__);
  dst->items_num = src.items_num;
  dst->active_index = src.active_index;
  if (src.items && src.items_num > 0) {
    dst->items = MEM_new_array<NodeObjectModifierItem>(src.items_num, __func__);
    std::copy_n(src.items, src.items_num, dst->items);
  }
  else {
    dst->items = nullptr;
  }
  dst_node->storage = dst;
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeObjectModifiers &storage = node_storage(node);
  writer.write_struct_array(storage.items_num, storage.items);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeObjectModifiers &storage = node_storage(node);
  (void)BLO_read_array(&reader, &storage.items, storage.items_num);
}

static Object *object_from_socket_default(const bNodeSocket *socket)
{
  if (socket == nullptr || socket->type != SOCK_OBJECT || socket->default_value == nullptr) {
    return nullptr;
  }
  Object *ob = static_cast<bNodeSocketValueObject *>(socket->default_value)->value;
  if (ob && !ID_MISSING(&ob->id)) {
    return ob;
  }
  return nullptr;
}

static const bNodeLink *find_incoming_link(const bNodeTree &ntree, const bNodeSocket &input)
{
  for (const bNodeLink &link : ntree.links) {
    if (link.tosock != &input) {
      continue;
    }
    if (link.flag & NODE_LINK_MUTED) {
      continue;
    }
    if (link.fromnode && (link.fromnode->flag & NODE_MUTED)) {
      continue;
    }
    return &link;
  }
  return nullptr;
}

static Object *resolve_input_object(const bContext *C, const bNodeTree &ntree, const bNode &start)
{
  const bNodeSocket *socket = bke::node_find_socket(start, SOCK_IN, "Object"_ustr);
  Set<const bNode *> visited;
  while (socket != nullptr) {
    const bNodeLink *link = find_incoming_link(ntree, *socket);
    if (link == nullptr) {
      return object_from_socket_default(socket);
    }
    const bNode *from = link->fromnode;
    if (from == nullptr || !visited.add(from)) {
      return object_from_socket_default(socket);
    }
    if (STREQ(from->idname, "ObjectNodeObject")) {
      const auto *storage = static_cast<const NodeObjectInput *>(from->storage);
      if (storage && storage->use_active) {
        return CTX_data_active_object(C);
      }
      socket = bke::node_find_socket(*from, SOCK_IN, "Object"_ustr);
      continue;
    }
    if (STREQ(from->idname, "ObjectNodeCreate")) {
      if (from->id && GS(from->id->name) == ID_OB && !ID_MISSING(from->id)) {
        return reinterpret_cast<Object *>(from->id);
      }
      return nullptr;
    }
    if (STREQ(from->idname, "NodeReroute")) {
      socket = static_cast<const bNodeSocket *>(from->inputs.first());
      continue;
    }
    if (const bNodeSocket *in = bke::node_find_socket(*from, SOCK_IN, "Object"_ustr)) {
      socket = in;
      continue;
    }
    return object_from_socket_default(link->fromsock);
  }
  return nullptr;
}

static bool skip_modifier_prop(const char *identifier)
{
  if (identifier == nullptr || identifier[0] == '\0') {
    return true;
  }
  if (StringRef(identifier).startswith("open_")) {
    return true;
  }
  static const char *skip[] = {
      "name",
      "type",
      "show_expanded",
      "show_in_editmode",
      "show_viewport",
      "show_render",
      "show_on_cage",
      "is_active",
      "use_pin_to_last",
      "use_apply_on_spline",
      "show_group_selector",
      "show_manage_panel",
      "execution_time",
      "persistent_uid",
      "is_override_data",
      nullptr,
  };
  for (int i = 0; skip[i]; i++) {
    if (STREQ(identifier, skip[i])) {
      return true;
    }
  }
  return false;
}

static void draw_modifier_rna_body(ui::Layout &layout, bContext *C, PointerRNA *md_ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);

  if (RNA_struct_find_property(md_ptr, "node_group")) {
    const bool has_group = RNA_pointer_get(md_ptr, "node_group").data != nullptr;
    const char *newop = has_group ? "object.geometry_node_tree_copy_assign" :
                                    "node.new_geometry_node_group_assign";
    ui::template_id(&layout, C, md_ptr, "node_group", newop, nullptr, nullptr);
  }

  RNA_STRUCT_BEGIN (md_ptr, prop) {
    const char *identifier = RNA_property_identifier(prop);
    if (skip_modifier_prop(identifier)) {
      continue;
    }
    if (RNA_property_flag(prop) & PROP_HIDDEN) {
      continue;
    }
    const PropertyType ptype = RNA_property_type(prop);
    if (ELEM(ptype, PROP_COLLECTION, PROP_POINTER)) {
      if (ptype == PROP_POINTER) {
        StructRNA *ptype_srna = RNA_property_pointer_type(md_ptr, prop);
        if (ptype_srna && RNA_struct_is_ID(ptype_srna) && !STREQ(identifier, "node_group")) {
          layout.prop(md_ptr, identifier, UI_ITEM_NONE, std::nullopt, ICON_NONE);
        }
      }
      continue;
    }
    layout.prop(md_ptr, identifier, UI_ITEM_NONE, std::nullopt, ICON_NONE);
  }
  RNA_STRUCT_END;
}

static void draw_one_modifier(ui::Layout &layout, bContext *C, ModifierData &md, PointerRNA &md_ptr)
{
  layout.context_ptr_set("modifier", &md_ptr);

  const ModifierTypeInfo *mti = BKE_modifier_get_info(md.type);
  ui::Layout &header = layout.row(true);
  header.prop(&md_ptr, "show_expanded", UI_ITEM_NONE, "", ICON_NONE);
  header.label("", mti ? mti->icon : ICON_MODIFIER);
  header.prop(&md_ptr, "name", UI_ITEM_NONE, "", ICON_NONE);
  if (RNA_struct_find_property(&md_ptr, "show_in_editmode")) {
    header.prop(&md_ptr, "show_in_editmode", UI_ITEM_NONE, "", ICON_NONE);
  }
  header.prop(&md_ptr, "show_viewport", UI_ITEM_NONE, "", ICON_NONE);
  header.prop(&md_ptr, "show_render", UI_ITEM_NONE, "", ICON_NONE);

  PointerRNA apply = header.op("object.modifier_apply", "", ICON_CHECKMARK);
  if (apply.data) {
    RNA_string_set(&apply, "modifier", md.name);
  }
  PointerRNA remove = header.op("object.modifier_remove", "", ICON_X);
  if (remove.data) {
    RNA_string_set(&remove, "modifier", md.name);
  }

  if (!RNA_boolean_get(&md_ptr, "show_expanded")) {
    return;
  }

  ui::Layout &body = layout.column(false);
  draw_modifier_rna_body(body, C, &md_ptr);
}

static void node_layout(ui::Layout &layout, bContext *C, PointerRNA *ptr)
{
  bNode *node = ptr->data_as<bNode>();
  bNodeTree *ntree = reinterpret_cast<bNodeTree *>(ptr->owner_id);
  if (node == nullptr || ntree == nullptr) {
    return;
  }

  layout.use_property_split_set(false);
  layout.use_property_decorate_set(false);

  Object *ob = resolve_input_object(C, *ntree, *node);
  if (ob == nullptr || ID_MISSING(&ob->id)) {
    layout.label(IFACE_("Connect an object"), ICON_INFO);
    return;
  }
  if (!BKE_object_supports_modifiers(ob)) {
    layout.label(IFACE_("Object type does not support modifiers"), ICON_INFO);
    return;
  }

  PointerRNA ob_ptr = RNA_id_pointer_create(&ob->id);
  layout.context_ptr_set("object", &ob_ptr);
  layout.operator_context_set(wm::OpCallContext::InvokeDefault);

  const bContextStore *previous_ctx = CTX_store_get(C);
  CTX_store_set(C, layout.context_store());

  PointerRNA add_op = layout.op("wm.call_menu", IFACE_("Add Modifier"), ICON_ADD);
  if (add_op.data) {
    RNA_string_set(&add_op, "name", "OBJECT_MT_modifier_add");
  }

  if (BLI_listbase_is_empty(&ob->modifiers)) {
    layout.label(IFACE_("No modifiers"), ICON_NONE);
    CTX_store_set(C, previous_ctx);
    return;
  }

  for (ModifierData &md : ob->modifiers) {
    PointerRNA md_ptr = RNA_pointer_create_id_subdata(ob->id, RNA_Modifier, &md);
    draw_one_modifier(layout, C, md, md_ptr);
  }

  CTX_store_set(C, previous_ctx);
}

static void node_register()
{
  static bke::bNodeType ntype;
  obj_node_type_base(&ntype, "ObjectNodeSetModifiers"_ustr);
  ntype.ui_name = "Set Modifiers";
  ntype.ui_description =
      "Edit the object's modifier stack. Expand a modifier to set Geometry Nodes groups, "
      "Subdivision levels, Apply, and the rest of its settings";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.default_width = 320;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  bke::node_type_storage(ntype, "NodeObjectModifiers", node_free_storage, node_copy_storage);
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_object_modifiers_cc
