/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#pragma once

#include "DNA_node_types.h"

#include "WM_api.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "BLI_function_ref.hh"
#include "BLI_string_utf8.hh"

#include "BKE_screen.hh"

#include "NOD_node_declaration.hh"
#include "NOD_socket_items.hh"

namespace blender::nodes::socket_items::ui {

template<typename Accessor>
static void draw_item_in_list(uiList * /*ui_list*/,
                              const bContext *C,
                              blender::ui::Layout &layout,
                              PointerRNA * /*idataptr*/,
                              PointerRNA *itemptr,
                              int /*icon*/,
                              PointerRNA * /*active_dataptr*/,
                              const char * /*active_propname*/,
                              int /*index*/,
                              int /*flt_flag*/)
{
  blender::ui::Layout &row = layout.row(true);
  if constexpr (Accessor::has_type) {
    float4 color;
    RNA_float_get_array(itemptr, "color", color);
    template_node_socket(&row, const_cast<bContext *>(C), color);
  }
  row.emboss_set(blender::ui::EmbossType::None);
  row.prop(itemptr, "name", blender::UI_ITEM_NONE, "", ICON_NONE);
}

/**
 * Draws a ui-list that contains the items. The list also has operators to add, remove and reorder
 * items.
 */
template<typename Accessor>
static void draw_items_list_with_operators(const bContext *C,
                                           blender::ui::Layout *layout,
                                           const bNodeTree &tree,
                                           const bNode &node)
{
  BLI_assert(Accessor::node_idname == node.idname);
  PointerRNA node_ptr = RNA_pointer_create_discrete(
      const_cast<ID *>(&tree.id), RNA_Node, const_cast<bNode *>(&node));

  static const uiListType *items_list = []() {
    uiListType *list = MEM_new_zeroed<uiListType>(Accessor::ui_idnames::list.c_str());
    STRNCPY_UTF8(list->idname, Accessor::ui_idnames::list.c_str());
    list->draw_item = draw_item_in_list<Accessor>;
    WM_uilisttype_add(list);
    return list;
  }();

  blender::ui::Layout *row = &layout->row(false);
  template_uilist(row,
                  C,
                  items_list->idname,
                  "",
                  &node_ptr,
                  Accessor::rna_names::items,
                  &node_ptr,
                  Accessor::rna_names::active_index.c_str(),
                  nullptr,
                  3,
                  5,
                  UILST_LAYOUT_DEFAULT,
                  blender::ui::TEMPLATE_LIST_FLAG_NONE);

  blender::ui::Layout *ops_col = &row->column(false);
  {
    blender::ui::Layout *add_remove_col = &ops_col->column(true);
    add_remove_col->op(Accessor::operator_idnames::add_item, "", ICON_ADD);
    add_remove_col->op(Accessor::operator_idnames::remove_item, "", ICON_REMOVE);
  }
  {
    blender::ui::Layout *up_down_col = &ops_col->column(true);
    PointerRNA op_ptr = up_down_col->op(Accessor::operator_idnames::move_item, "", ICON_TRIA_UP);
    RNA_enum_set(&op_ptr, "direction", 0);
    op_ptr = up_down_col->op(Accessor::operator_idnames::move_item, "", ICON_TRIA_DOWN);
    RNA_enum_set(&op_ptr, "direction", 1);
  }
}

/** Draw properties of the active item if there is any. */
template<typename Accessor>
static void draw_active_item_props(const bNodeTree &tree,
                                   const bNode &node,
                                   const FunctionRef<void(PointerRNA *item_ptr)> draw_item)
{
  using ItemT = typename Accessor::ItemT;
  BLI_assert(Accessor::node_idname == node.idname);

  SocketItemsRef<ItemT> ref = Accessor::get_items_from_node(const_cast<bNode &>(node));
  if (*ref.active_index < 0) {
    return;
  }
  if (*ref.active_index >= *ref.items_num) {
    return;
  }

  ItemT &item = (*ref.items)[*ref.active_index];
  PointerRNA item_ptr = RNA_pointer_create_discrete(
      const_cast<ID *>(&tree.id), *Accessor::item_srna, &item);
  draw_item(&item_ptr);
}

template<typename Accessor> static auto draw_extend_socket_fn()
{
  return [](CustomSocketDrawParams &params) {
    ::blender::ui::Layout &layout = params.layout;
    layout.emboss_set(::blender::ui::EmbossType::None);
    PointerRNA op_ptr = layout.op(Accessor::operator_idnames::add_item, "", ICON_ADD);
    RNA_int_set(&op_ptr, "node_identifier", params.node.identifier);
    RNA_boolean_set(&op_ptr, "show_dialog", true);
    RNA_boolean_set(&op_ptr, "init_from_active", false);
  };
}

/**
 * Which side of the socket row the per-item remove button occupies.
 * - Right: input-item lists and zone state rows (Index Switch, Simulation, Repeat).
 * - Left: freely added output-item lists (Evaluate Closure outputs, Separate Bundle).
 */
enum class ItemRemoveButtonSide {
  Left,
  Right,
};

template<typename Accessor> static int accessor_min_items_num()
{
  if constexpr (requires { Accessor::min_items_num; }) {
    return Accessor::min_items_num;
  }
  return 0;
}

template<typename Accessor>
static void draw_item_remove_button(::blender::ui::Layout &parent_row,
                                    CustomSocketDrawParams &params,
                                    const int item_index)
{
  ::blender::ui::Layout &btn_row = parent_row.row(false);
  btn_row.fixed_size_set(true);
  btn_row.ui_units_x_set(1.0f);
  btn_row.emboss_set(::blender::ui::EmbossType::None);
  btn_row.active_set(true);

  /* Zone input nodes remap to the output node in the operator. */
  bool can_remove = true;
  if (params.node.idname == Accessor::node_idname) {
    SocketItemsRef<typename Accessor::ItemT> ref = Accessor::get_items_from_node(params.node);
    can_remove = *ref.items_num > accessor_min_items_num<Accessor>();
  }
  btn_row.enabled_set(can_remove);

  PointerRNA op_ptr = btn_row.op(Accessor::operator_idnames::remove_item, "", ICON_REMOVE);
  RNA_int_set(&op_ptr, "index", item_index);
  RNA_int_set(&op_ptr, "node_identifier", params.node.identifier);
}

/**
 * Draw the standard socket widget plus a remove button for the item at #item_index.
 * Used on Index Switch / Menu Switch / Capture Attribute and other + item lists.
 *
 * Layout notes:
 * - Value widgets go in a child row so inactive greying does not affect the remove control.
 * - The remove column is fixed-width so sliders and label-only sockets share one icon column.
 * - Emboss is None (flat icon), matching the extend-socket + control.
 * - Disabled when only the last required item remains (#Accessor::min_items_num).
 */
template<typename Accessor>
static void draw_item_socket_with_remove(CustomSocketDrawParams &params,
                                         const int item_index,
                                         const std::optional<StringRefNull> label_override =
                                             std::nullopt,
                                         const ItemRemoveButtonSide side =
                                             ItemRemoveButtonSide::Right)
{
  ::blender::ui::Layout &row = params.layout.row(true);
  row.alignment_set(::blender::ui::LayoutAlign::Expand);

  if (side == ItemRemoveButtonSide::Left) {
    draw_item_remove_button<Accessor>(row, params, item_index);
  }

  ::blender::ui::Layout &value_row = row.row(true);
  params.draw_standard(value_row, label_override);

  if (side == ItemRemoveButtonSide::Right) {
    draw_item_remove_button<Accessor>(row, params, item_index);
  }
}

/** Same as #draw_item_socket_with_remove, but with an existing custom value-row draw. */
template<typename Accessor>
static void draw_item_socket_with_remove(CustomSocketDrawParams &params,
                                         const int item_index,
                                         const ItemRemoveButtonSide side,
                                         const FunctionRef<void(CustomSocketDrawParams &)> draw_inner)
{
  ::blender::ui::Layout &row = params.layout.row(true);
  row.alignment_set(::blender::ui::LayoutAlign::Expand);

  if (side == ItemRemoveButtonSide::Left) {
    draw_item_remove_button<Accessor>(row, params, item_index);
  }

  ::blender::ui::Layout &value_row = row.row(true);
  CustomSocketDrawParams inner_params{params.C,
                                      value_row,
                                      params.tree,
                                      params.node,
                                      params.socket,
                                      params.node_ptr,
                                      params.socket_ptr,
                                      params.label,
                                      params.menu_switch_source_by_index_switch};
  draw_inner(inner_params);

  if (side == ItemRemoveButtonSide::Right) {
    draw_item_remove_button<Accessor>(row, params, item_index);
  }
}

}  // namespace blender::nodes::socket_items::ui
