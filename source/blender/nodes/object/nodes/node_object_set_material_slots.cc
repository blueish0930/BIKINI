/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_string.hh"
#include "BLI_string_utf8.hh"

#include "DNA_array_utils.hh"
#include "DNA_node_types.h"

#include "DNA_material_types.h"
#include "DNA_screen_types.h"

#include "BKE_context.hh"
#include "BKE_main_invariants.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_screen.hh"

#include "BLO_read_write.hh"

#include "MEM_guardedalloc.h"

#include "BLT_translation.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_prototypes.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_node.hh"
#include "ED_screen.hh"

#include "node_object_util.hh"

namespace blender::nodes::node_object_set_material_slots_cc {

NODE_STORAGE_FUNCS(NodeObjectMaterialSlots)

static void destruct_item(NodeObjectMaterialSlotItem * /*item*/) {}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Object>("Object"_ustr);
  b.add_output<decl::Object>("Object"_ustr);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  auto *storage = MEM_new<NodeObjectMaterialSlots>(__func__);
  storage->items = MEM_new_array<NodeObjectMaterialSlotItem>(1, __func__);
  storage->items[0] = {};
  STRNCPY(storage->items[0].name, "Material");
  storage->items_num = 1;
  storage->active_index = 0;
  node->storage = storage;
}

static void node_free_storage(bNode *node)
{
  auto *storage = static_cast<NodeObjectMaterialSlots *>(node->storage);
  if (storage) {
    MEM_SAFE_DELETE(storage->items);
    MEM_delete(storage);
  }
}

static void node_copy_storage(bNodeTree * /*dst*/, bNode *dst_node, const bNode *src_node)
{
  const NodeObjectMaterialSlots &src = node_storage(*src_node);
  auto *dst = MEM_new<NodeObjectMaterialSlots>(__func__);
  dst->items_num = src.items_num;
  dst->active_index = src.active_index;
  if (src.items && src.items_num > 0) {
    dst->items = MEM_new_array<NodeObjectMaterialSlotItem>(src.items_num, __func__);
    std::copy_n(src.items, src.items_num, dst->items);
  }
  else {
    dst->items = nullptr;
  }
  dst_node->storage = dst;
}

static void node_blend_write(const bNodeTree & /*tree*/, const bNode &node, BlendWriter &writer)
{
  const NodeObjectMaterialSlots &storage = node_storage(node);
  writer.write_struct_array(storage.items_num, storage.items);
}

static void node_blend_read(bNodeTree & /*tree*/, bNode &node, BlendDataReader &reader)
{
  NodeObjectMaterialSlots &storage = node_storage(node);
  (void)BLO_read_array(&reader, &storage.items, storage.items_num);
}

static bNode *active_slots_node(bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || ID_IS_LINKED(snode->edittree)) {
    return nullptr;
  }
  bNode *node = bke::node_get_active(*snode->edittree);
  if (!node || !node->is_type("ObjectNodeSetMaterialSlots"_ustr)) {
    return nullptr;
  }
  return node;
}

static void tag_node_changed(bContext *C, bNode &node)
{
  bNodeTree *ntree = CTX_wm_space_node(C)->edittree;
  BKE_ntree_update_tag_node_property(ntree, &node);
  BKE_main_ensure_invariants(*CTX_data_main(C), ntree->id);
  WM_main_add_notifier(NC_NODE | NA_EDITED, ntree);
}

static wmOperatorStatus slot_add_exec(bContext *C, wmOperator * /*op*/)
{
  bNode *node = active_slots_node(C);
  if (!node) {
    return OPERATOR_CANCELLED;
  }
  NodeObjectMaterialSlots &storage = node_storage(*node);
  const int old_num = storage.items_num;
  auto *new_items = MEM_new_array<NodeObjectMaterialSlotItem>(old_num + 1, __func__);
  if (storage.items && old_num > 0) {
    std::copy_n(storage.items, old_num, new_items);
  }
  new_items[old_num] = {};
  SNPRINTF_UTF8(new_items[old_num].name, "Slot %d", old_num + 1);
  MEM_SAFE_DELETE(storage.items);
  storage.items = new_items;
  storage.items_num = old_num + 1;
  storage.active_index = old_num;
  tag_node_changed(C, *node);
  return OPERATOR_FINISHED;
}

static wmOperatorStatus slot_remove_exec(bContext *C, wmOperator * /*op*/)
{
  bNode *node = active_slots_node(C);
  if (!node) {
    return OPERATOR_CANCELLED;
  }
  NodeObjectMaterialSlots &storage = node_storage(*node);
  if (storage.items_num <= 0) {
    return OPERATOR_CANCELLED;
  }
  const int index = std::clamp(storage.active_index, 0, storage.items_num - 1);
  dna::array::remove_index(
      &storage.items, &storage.items_num, &storage.active_index, index, destruct_item);
  tag_node_changed(C, *node);
  return OPERATOR_FINISHED;
}

static wmOperatorStatus slot_move_exec(bContext *C, wmOperator *op)
{
  bNode *node = active_slots_node(C);
  if (!node) {
    return OPERATOR_CANCELLED;
  }
  NodeObjectMaterialSlots &storage = node_storage(*node);
  if (storage.items_num <= 1) {
    return OPERATOR_CANCELLED;
  }
  const int direction = RNA_enum_get(op->ptr, "direction");
  const int from = std::clamp(storage.active_index, 0, storage.items_num - 1);
  const int to = from + direction;
  if (to < 0 || to >= storage.items_num) {
    return OPERATOR_CANCELLED;
  }
  dna::array::move_index(storage.items, storage.items_num, from, to);
  storage.active_index = to;
  tag_node_changed(C, *node);
  return OPERATOR_FINISHED;
}

static void NODE_OT_object_material_slot_add(wmOperatorType *ot)
{
  ot->name = "Add Object Material Slot";
  ot->idname = "NODE_OT_object_material_slot_add";
  ot->description = "Add a material slot to the Object Editor material slots node";
  ot->exec = slot_add_exec;
  ot->poll = ED_operator_node_editable;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static void NODE_OT_object_material_slot_remove(wmOperatorType *ot)
{
  ot->name = "Remove Object Material Slot";
  ot->idname = "NODE_OT_object_material_slot_remove";
  ot->description = "Remove the active material slot from the Object Editor node";
  ot->exec = slot_remove_exec;
  ot->poll = ED_operator_node_editable;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static void NODE_OT_object_material_slot_move(wmOperatorType *ot)
{
  static const EnumPropertyItem direction_items[] = {
      {-1, "UP", 0, "Up", ""},
      {1, "DOWN", 0, "Down", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };
  ot->name = "Move Object Material Slot";
  ot->idname = "NODE_OT_object_material_slot_move";
  ot->description = "Move the active material slot up or down";
  ot->exec = slot_move_exec;
  ot->poll = ED_operator_node_editable;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
  RNA_def_enum(ot->srna, "direction", direction_items, -1, "Direction", "");
}

static void node_operators()
{
  WM_operatortype_append(NODE_OT_object_material_slot_add);
  WM_operatortype_append(NODE_OT_object_material_slot_remove);
  WM_operatortype_append(NODE_OT_object_material_slot_move);
}

static void draw_slot_item(uiList * /*ui_list*/,
                           const bContext * /*C*/,
                           ui::Layout &layout,
                           PointerRNA * /*idataptr*/,
                           PointerRNA *itemptr,
                           int /*icon*/,
                           PointerRNA * /*active_dataptr*/,
                           const char * /*active_propname*/,
                           int /*index*/,
                           int /*flt_flag*/)
{
  NodeObjectMaterialSlotItem *item = static_cast<NodeObjectMaterialSlotItem *>(itemptr->data);
  ui::Layout &row = layout.row(true);
  row.emboss_set(ui::EmbossType::None);
  if (item && item->material) {
    PointerRNA ma_ptr = RNA_id_pointer_create(&item->material->id);
    row.prop(&ma_ptr, "name", UI_ITEM_NONE, "", ICON_MATERIAL_DATA);
  }
  else {
    row.label(IFACE_("(empty)"), ICON_BLANK1);
  }
}

static void node_layout(ui::Layout &layout, bContext *C, PointerRNA *ptr)
{
  NodeObjectMaterialSlots &storage = node_storage(*ptr->data_as<bNode>());

  layout.use_property_split_set(false);
  layout.use_property_decorate_set(false);

  static const uiListType *slots_list = []() {
    uiListType *list = MEM_new_zeroed<uiListType>("DATA_UL_object_material_slots");
    STRNCPY_UTF8(list->idname, "DATA_UL_object_material_slots");
    list->draw_item = draw_slot_item;
    WM_uilisttype_add(list);
    return list;
  }();

  ui::Layout &row = layout.row(false);
  ui::template_uilist(&row,
                      C,
                      slots_list->idname,
                      "",
                      ptr,
                      "slots",
                      ptr,
                      "active_index",
                      nullptr,
                      3,
                      5,
                      UILST_LAYOUT_DEFAULT,
                      ui::TEMPLATE_LIST_NO_GRIP);

  ui::Layout &ops = row.column(true);
  ops.op("node.object_material_slot_add", "", ICON_ADD);
  ops.op("node.object_material_slot_remove", "", ICON_REMOVE);
  ops.separator();
  PointerRNA up = ops.op("node.object_material_slot_move", "", ICON_TRIA_UP);
  if (up.data) {
    RNA_enum_set(&up, "direction", -1);
  }
  PointerRNA down = ops.op("node.object_material_slot_move", "", ICON_TRIA_DOWN);
  if (down.data) {
    RNA_enum_set(&down, "direction", 1);
  }

  if (storage.items && storage.items_num > 0) {
    const int index = std::clamp(storage.active_index, 0, storage.items_num - 1);
    PointerRNA itemptr = RNA_pointer_create_discrete(
        ptr->owner_id, RNA_NodeObjectMaterialSlotItem, &storage.items[index]);
    ui::template_id(&layout, C, &itemptr, "material", "material.new", nullptr, nullptr);
  }
}

static void node_register()
{
  static bke::bNodeType ntype;
  obj_node_type_base(&ntype, "ObjectNodeSetMaterialSlots"_ustr);
  ntype.ui_name = "Set Material Slots";
  ntype.ui_description =
      "Assign object material slots. Add and remove slots like the Properties editor";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.default_width = 240;
  ntype.register_operators = node_operators;
  ntype.blend_write_storage_content = node_blend_write;
  ntype.blend_data_read_storage_content = node_blend_read;
  bke::node_type_storage(ntype, "NodeObjectMaterialSlots", node_free_storage, node_copy_storage);
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_object_set_material_slots_cc
