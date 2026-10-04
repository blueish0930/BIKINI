/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <climits>
#include <cmath>
#include <cstring>

#include "BLI_lasso_2d.hh"
#include "BLI_listbase.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_rect.hh"
#include "BLI_string.hh"


#include "BKE_context.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "ED_screen.hh"
#include "ED_select_utils.hh"

#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "UI_view2d.hh"

#include "WM_api.hh"
#include "WM_keymap.hh"
#include "WM_types.hh"
#include "wm_cursors.hh"

#include "MEM_guardedalloc.h"

#include "datablock_graph_intern.hh"

namespace blender::ed::datablock_graph {

static GraphNode *find_node_by_key(SpaceDataBlockGraph &sdbg, const uint64_t key)
{
  return datablock_graph_find_node_mut(sdbg, uint(key >> 32), uint(key & 0xffffffffu));
}

static uint64_t node_key(const GraphNode &n)
{
  return datablock_graph_node_key(n.seed_uid, n.id_session_uid);
}

static bool region_window_poll(bContext *C)
{
  if (!ED_operator_datablock_graph_active(C)) {
    return false;
  }
  ARegion *region = CTX_wm_region(C);
  return region && region->regiontype == RGN_TYPE_WINDOW;
}

static void mouse_to_view(const ARegion *region, const int mval[2], float2 &r_view)
{
  ui::view2d_region_to_view(&region->v2d, float(mval[0]), float(mval[1]), &r_view.x, &r_view.y);
}

static void notify_select(bContext *C)
{
  ED_area_tag_redraw(CTX_wm_area(C));
}

static void restore_cursor(bContext *C)
{
  if (wmWindow *win = CTX_wm_window(C)) {
    WM_cursor_modal_restore(win);
  }
}

/**
 * Call *before* mutating seeds / layout so Ctrl+Z can restore this state.
 * Uses an editor-local undo stack (Global memfile undo is unreliable for space-only DNA).
 */
static void datablock_graph_push_undo(bContext * /*C*/, SpaceDataBlockGraph *sdbg, const char * /*msg*/)
{
  if (sdbg) {
    datablock_graph_undo_push(*sdbg);
  }
}

/* -------------------------------------------------------------------- */
/** \name Clear / Delete / Relayout / Add Seed
 * \{ */

static wmOperatorStatus datablock_graph_clear_exec(bContext *C, wmOperator * /*op*/)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  if (!sdbg) {
    return OPERATOR_CANCELLED;
  }
  datablock_graph_push_undo(C, sdbg, "Clear Data-Block Graph");
  datablock_graph_clear_seeds(*sdbg);
  ED_area_tag_redraw(CTX_wm_area(C));
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_DATABLOCK_GRAPH, nullptr);
  return OPERATOR_FINISHED;
}

static wmOperatorStatus datablock_graph_delete_exec(bContext *C, wmOperator *op)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  if (!sdbg) {
    return OPERATOR_CANCELLED;
  }
  if (datablock_graph_count_selected(*sdbg) == 0) {
    return OPERATOR_CANCELLED;
  }
  datablock_graph_push_undo(C, sdbg, "Delete Data-Block Graph");
  const int removed = datablock_graph_delete_selected(*sdbg);
  if (removed == 0) {
    return OPERATOR_CANCELLED;
  }
  if (op->reports) {
    BKE_reportf(op->reports,
                RPT_INFO,
                "Removed %d reference graph(s) from the editor (data-blocks kept)",
                removed);
  }
  ED_area_tag_redraw(CTX_wm_area(C));
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_DATABLOCK_GRAPH, nullptr);
  return OPERATOR_FINISHED;
}

static bool datablock_graph_delete_poll(bContext *C)
{
  if (!region_window_poll(C)) {
    return false;
  }
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  return sdbg && datablock_graph_count_selected(*sdbg) > 0;
}

static void DATABLOCK_GRAPH_OT_delete(wmOperatorType *ot)
{
  ot->name = "Delete";
  ot->idname = "DATABLOCK_GRAPH_OT_delete";
  ot->description =
      "Remove selected reference graphs from the editor (does not delete data-blocks)";
  ot->exec = datablock_graph_delete_exec;
  ot->poll = datablock_graph_delete_poll;
  /* Undo pushed explicitly after DNA flush (OPTYPE_UNDO alone was unreliable for space data). */
  ot->flag = OPTYPE_REGISTER;
}

static void DATABLOCK_GRAPH_OT_clear(wmOperatorType *ot)
{
  ot->name = "Clear Data-Block Graph";
  ot->idname = "DATABLOCK_GRAPH_OT_clear";
  ot->description = "Remove all seed data-blocks and clear the reference graph";
  ot->exec = datablock_graph_clear_exec;
  ot->poll = ED_operator_datablock_graph_active;
  ot->flag = OPTYPE_REGISTER;
}

static wmOperatorStatus datablock_graph_relayout_exec(bContext *C, wmOperator * /*op*/)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  if (!sdbg) {
    return OPERATOR_CANCELLED;
  }
  datablock_graph_ensure_runtime(*sdbg);
  datablock_graph_push_undo(C, sdbg, "Relayout Data-Block Graph");
  datablock_graph_layout(*sdbg, true);
  sdbg->runtime->frame_view = true;
  if (ScrArea *area = CTX_wm_area(C)) {
    if (ARegion *winreg = BKE_area_find_region_type(area, RGN_TYPE_WINDOW)) {
      datablock_graph_update_view2d(*winreg, *sdbg, true);
    }
  }
  ED_area_tag_redraw(CTX_wm_area(C));
  return OPERATOR_FINISHED;
}

static void DATABLOCK_GRAPH_OT_relayout(wmOperatorType *ot)
{
  ot->name = "Relayout Data-Block Graph";
  ot->idname = "DATABLOCK_GRAPH_OT_relayout";
  ot->description = "Recalculate automatic layout and frame all nodes";
  ot->exec = datablock_graph_relayout_exec;
  ot->poll = ED_operator_datablock_graph_active;
  ot->flag = OPTYPE_REGISTER;
}

static wmOperatorStatus datablock_graph_refresh_exec(bContext *C, wmOperator * /*op*/)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  Main *bmain = CTX_data_main(C);
  if (!sdbg || !bmain) {
    return OPERATOR_CANCELLED;
  }
  datablock_graph_force_refresh(*sdbg, *bmain);
  if (ScrArea *area = CTX_wm_area(C)) {
    if (ARegion *winreg = BKE_area_find_region_type(area, RGN_TYPE_WINDOW)) {
      datablock_graph_update_view2d(*winreg, *sdbg, false);
    }
  }
  ED_area_tag_redraw(CTX_wm_area(C));
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_DATABLOCK_GRAPH, nullptr);
  return OPERATOR_FINISHED;
}

static void DATABLOCK_GRAPH_OT_refresh(wmOperatorType *ot)
{
  ot->name = "Refresh Reference Graph";
  ot->idname = "DATABLOCK_GRAPH_OT_refresh";
  ot->description =
      "Force re-evaluate Main relations and rebuild the reference graph (keeps node positions)";
  ot->exec = datablock_graph_refresh_exec;
  ot->poll = ED_operator_datablock_graph_active;
  ot->flag = OPTYPE_REGISTER;
}

static wmOperatorStatus datablock_graph_add_seed_exec(bContext *C, wmOperator *op)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  Main *bmain = CTX_data_main(C);
  ARegion *region = CTX_wm_region(C);
  if (!sdbg || !bmain) {
    return OPERATOR_CANCELLED;
  }
  const uint32_t session_uid = uint32_t(RNA_int_get(op->ptr, "session_uid"));
  ID *id = BKE_libblock_find_session_uid(bmain, session_uid);
  if (!id) {
    BKE_report(op->reports, RPT_ERROR, "Data-block not found");
    return OPERATOR_CANCELLED;
  }

  float2 drop_co;
  const float2 *drop_ptr = nullptr;
  if (region && RNA_struct_property_is_set(op->ptr, "view_x") &&
      RNA_struct_property_is_set(op->ptr, "view_y"))
  {
    drop_co.x = RNA_float_get(op->ptr, "view_x");
    drop_co.y = RNA_float_get(op->ptr, "view_y");
    drop_ptr = &drop_co;
  }
  else if (region && RNA_struct_property_is_set(op->ptr, "cursor_x")) {
    /* Region pixel coords from drop. */
    const int cx = RNA_int_get(op->ptr, "cursor_x");
    const int cy = RNA_int_get(op->ptr, "cursor_y");
    ui::view2d_region_to_view(&region->v2d, float(cx), float(cy), &drop_co.x, &drop_co.y);
    drop_ptr = &drop_co;
  }

  /* Create independent tree or highlight existing; new tree selects all new nodes. */
  const bool already = datablock_graph_has_seed(*sdbg, id);
  if (!already) {
    datablock_graph_push_undo(C, sdbg, "Add Data-Block Graph");
  }
  datablock_graph_activate_or_create(*sdbg, *bmain, id, drop_ptr);
  ED_area_tag_redraw(CTX_wm_area(C));
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_DATABLOCK_GRAPH, nullptr);
  return OPERATOR_FINISHED;
}

static void DATABLOCK_GRAPH_OT_add_seed(wmOperatorType *ot)
{
  ot->name = "Add Seed Data-Block";
  ot->idname = "DATABLOCK_GRAPH_OT_add_seed";
  ot->description =
      "Create an independent ref-graph for a data-block (or highlight if it already exists)";
  ot->exec = datablock_graph_add_seed_exec;
  ot->poll = ED_operator_datablock_graph_active;
  /* Undo pushed in exec after DNA update. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_INTERNAL;
  RNA_def_int(ot->srna, "session_uid", 0, INT_MIN, INT_MAX, "Session UID", "", INT_MIN, INT_MAX);
  PropertyRNA *prop;
  prop = RNA_def_float(ot->srna, "view_x", 0.0f, -1e6f, 1e6f, "View X", "", -1e6f, 1e6f);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE | PROP_HIDDEN);
  prop = RNA_def_float(ot->srna, "view_y", 0.0f, -1e6f, 1e6f, "View Y", "", -1e6f, 1e6f);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE | PROP_HIDDEN);
  prop = RNA_def_int(ot->srna, "cursor_x", 0, INT_MIN, INT_MAX, "Cursor X", "", INT_MIN, INT_MAX);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE | PROP_HIDDEN);
  prop = RNA_def_int(ot->srna, "cursor_y", 0, INT_MIN, INT_MAX, "Cursor Y", "", INT_MIN, INT_MAX);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE | PROP_HIDDEN);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Select (click) — node-editor style exclusive pick
 * \{ */

static wmOperatorStatus select_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  ARegion *region = CTX_wm_region(C);
  Main *bmain = CTX_data_main(C);
  if (!sdbg || !region || !bmain) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  datablock_graph_rebuild_if_needed(*sdbg, *bmain);
  datablock_graph_update_view2d(*region, *sdbg, false);

  float2 view_co;
  mouse_to_view(region, event->mval, view_co);
  GraphNode *node = datablock_graph_find_node_at(*sdbg, view_co);
  if (!node) {
    if (RNA_boolean_get(op->ptr, "deselect_all") && !RNA_boolean_get(op->ptr, "extend") &&
        !RNA_boolean_get(op->ptr, "toggle"))
    {
      if (datablock_graph_count_selected(*sdbg) > 0) {
        datablock_graph_push_undo(C, sdbg, "Deselect Data-Block Graph");
        datablock_graph_deselect_all(*sdbg);
        notify_select(C);
      }
      return OPERATOR_FINISHED | OPERATOR_PASS_THROUGH;
    }
    return OPERATOR_PASS_THROUGH;
  }

  const uint64_t nkey = datablock_graph_node_key(node->seed_uid, node->id_session_uid);

  const bool extend = RNA_boolean_get(op->ptr, "extend");
  const bool deselect = RNA_boolean_get(op->ptr, "deselect");
  const bool toggle = RNA_boolean_get(op->ptr, "toggle");

  /* Snapshot selection (and layout) before change so Ctrl+Z restores it. */
  datablock_graph_push_undo(C, sdbg, "Select Data-Block Graph");

  if (extend) {
    node->selected = true;
    sdbg->runtime->active_key = nkey;
  }
  else if (deselect) {
    node->selected = false;
  }
  else if (toggle) {
    node->selected = !node->selected;
    if (node->selected) {
      sdbg->runtime->active_key = nkey;
    }
  }
  else {
    /* Exclusive select (including re-click on an already-selected node in a multi-selection). */
    datablock_graph_deselect_all(*sdbg);
    node->selected = true;
    sdbg->runtime->active_key = nkey;
  }
  notify_select(C);
  return OPERATOR_FINISHED | OPERATOR_PASS_THROUGH;
}

static void DATABLOCK_GRAPH_OT_select(wmOperatorType *ot)
{
  ot->name = "Select";
  ot->idname = "DATABLOCK_GRAPH_OT_select";
  ot->description = "Select a node (Shift to extend)";
  ot->invoke = select_invoke;
  ot->poll = region_window_poll;
  /* Selection is runtime-only — do not push Global Undo (would flood stack / hide real undos). */
  ot->flag = OPTYPE_REGISTER;

  PropertyRNA *prop;
  prop = RNA_def_boolean(ot->srna, "extend", false, "Extend", "Extend selection");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  prop = RNA_def_boolean(ot->srna, "deselect", false, "Deselect", "Remove from selection");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  prop = RNA_def_boolean(ot->srna, "toggle", false, "Toggle", "Toggle selection");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  prop = RNA_def_boolean(
      ot->srna, "deselect_all", true, "Deselect On Nothing", "Deselect when clicking empty space");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Select All
 * \{ */

static wmOperatorStatus select_all_exec(bContext *C, wmOperator *op)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  if (!sdbg) {
    return OPERATOR_CANCELLED;
  }
  datablock_graph_ensure_runtime(*sdbg);
  datablock_graph_push_undo(C, sdbg, "Select All Data-Block Graph");
  const int action = RNA_enum_get(op->ptr, "action");
  switch (action) {
    case SEL_TOGGLE: {
      const bool any = datablock_graph_count_selected(*sdbg) > 0;
      if (any) {
        datablock_graph_deselect_all(*sdbg);
      }
      else {
        datablock_graph_select_all(*sdbg);
      }
      break;
    }
    case SEL_SELECT:
      datablock_graph_select_all(*sdbg);
      break;
    case SEL_DESELECT:
      datablock_graph_deselect_all(*sdbg);
      break;
    case SEL_INVERT:
      for (GraphNode &n : sdbg->runtime->nodes) {
        n.selected = !n.selected;
      }
      break;
  }
  notify_select(C);
  return OPERATOR_FINISHED;
}

static void DATABLOCK_GRAPH_OT_select_all(wmOperatorType *ot)
{
  ot->name = "Select All";
  ot->idname = "DATABLOCK_GRAPH_OT_select_all";
  ot->description = "Select or deselect all nodes";
  ot->exec = select_all_exec;
  ot->poll = region_window_poll;
  ot->flag = OPTYPE_REGISTER;
  WM_operator_properties_select_all(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Box Select (node-editor gesture)
 * \{ */

static wmOperatorStatus box_select_exec(bContext *C, wmOperator *op)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  ARegion *region = CTX_wm_region(C);
  Main *bmain = CTX_data_main(C);
  if (!sdbg || !region || !bmain) {
    restore_cursor(C);
    return OPERATOR_CANCELLED;
  }
  datablock_graph_rebuild_if_needed(*sdbg, *bmain);
  datablock_graph_push_undo(C, sdbg, "Box Select Data-Block Graph");

  rctf rectf;
  WM_operator_properties_border_to_rctf(op, &rectf);
  ui::view2d_region_to_view_rctf(&region->v2d, &rectf, &rectf);

  /* Shift/Ctrl may arrive as modifiers on invoke; honor them if mode still default SET. */
  eSelectOp sel_op = eSelectOp(RNA_enum_get(op->ptr, "mode"));
  if (SEL_OP_USE_PRE_DESELECT(sel_op)) {
    datablock_graph_deselect_all(*sdbg);
  }

  for (GraphNode &node : sdbg->runtime->nodes) {
    const rctf nr = datablock_graph_node_rect(node);
    if (!BLI_rctf_isect(&rectf, &nr, nullptr)) {
      continue;
    }
    switch (sel_op) {
      case SEL_OP_ADD:
        node.selected = true;
        sdbg->runtime->active_key = datablock_graph_node_key(node.seed_uid, node.id_session_uid);
        break;
      case SEL_OP_SUB:
        node.selected = false;
        break;
      case SEL_OP_SET:
      default:
        node.selected = true;
        sdbg->runtime->active_key = datablock_graph_node_key(node.seed_uid, node.id_session_uid);
        break;
    }
  }
  notify_select(C);
  restore_cursor(C);
  return OPERATOR_FINISHED;
}

static wmOperatorStatus box_select_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  if (event->keymodifier == EVT_UKEY) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }

  /* Map modifiers → mode when operator property is still default SET. */
  if (RNA_enum_get(op->ptr, "mode") == SEL_OP_SET) {
    if ((event->modifier & KM_SHIFT) && !(event->modifier & KM_CTRL)) {
      RNA_enum_set(op->ptr, "mode", SEL_OP_ADD);
    }
    else if ((event->modifier & KM_CTRL) && !(event->modifier & KM_SHIFT)) {
      RNA_enum_set(op->ptr, "mode", SEL_OP_SUB);
    }
  }

  const bool tweak = RNA_boolean_get(op->ptr, "tweak");
  if (tweak) {
    SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
    ARegion *region = CTX_wm_region(C);
    if (sdbg && region) {
      float2 view_co;
      mouse_to_view(region, event->mval, view_co);
      if (datablock_graph_find_node_at(*sdbg, view_co)) {
        return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
      }
    }
  }
  return WM_gesture_box_invoke(C, op, event);
}

static void box_select_cancel(bContext *C, wmOperator *op)
{
  WM_gesture_box_cancel(C, op);
  restore_cursor(C);
}

static void DATABLOCK_GRAPH_OT_select_box(wmOperatorType *ot)
{
  ot->name = "Box Select";
  ot->idname = "DATABLOCK_GRAPH_OT_select_box";
  ot->description = "Select nodes using box selection";
  ot->invoke = box_select_invoke;
  ot->exec = box_select_exec;
  ot->modal = WM_gesture_box_modal;
  ot->cancel = box_select_cancel;
  ot->poll = region_window_poll;
  ot->flag = OPTYPE_REGISTER;

  PropertyRNA *prop = RNA_def_boolean(
      ot->srna, "tweak", false, "Tweak", "Only activate when mouse is not over a node");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  WM_operator_properties_gesture_box(ot);
  WM_operator_properties_select_operation_simple(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Circle Select
 * \{ */

static wmOperatorStatus circle_select_exec(bContext *C, wmOperator *op)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  ARegion *region = CTX_wm_region(C);
  Main *bmain = CTX_data_main(C);
  if (!sdbg || !region || !bmain) {
    return OPERATOR_CANCELLED;
  }
  datablock_graph_rebuild_if_needed(*sdbg, *bmain);

  const int x = RNA_int_get(op->ptr, "x");
  const int y = RNA_int_get(op->ptr, "y");
  const int radius = RNA_int_get(op->ptr, "radius");
  const float zoom = float(BLI_rcti_size_x(&region->winrct)) /
                     std::max(BLI_rctf_size_x(&region->v2d.cur), 1e-6f);

  float2 view_co;
  ui::view2d_region_to_view(&region->v2d, float(x), float(y), &view_co.x, &view_co.y);
  const float rad_view = float(radius) / zoom;

  const bool is_first = WM_gesture_is_modal_first(static_cast<const wmGesture *>(op->customdata));
  eSelectOp sel_op = ED_select_op_modal(eSelectOp(RNA_enum_get(op->ptr, "mode")), is_first);
  if (is_first) {
    datablock_graph_push_undo(C, sdbg, "Circle Select Data-Block Graph");
  }
  if (SEL_OP_USE_PRE_DESELECT(sel_op)) {
    datablock_graph_deselect_all(*sdbg);
  }

  for (GraphNode &node : sdbg->runtime->nodes) {
    const rctf nr = datablock_graph_node_rect(node);
    if (!BLI_rctf_isect_circle(&nr, view_co, rad_view)) {
      continue;
    }
    switch (sel_op) {
      case SEL_OP_ADD:
        node.selected = true;
        sdbg->runtime->active_key = datablock_graph_node_key(node.seed_uid, node.id_session_uid);
        break;
      case SEL_OP_SUB:
        node.selected = false;
        break;
      case SEL_OP_SET:
      default:
        node.selected = true;
        sdbg->runtime->active_key = datablock_graph_node_key(node.seed_uid, node.id_session_uid);
        break;
    }
  }
  notify_select(C);
  return OPERATOR_FINISHED;
}

static wmOperatorStatus circle_select_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  if (RNA_enum_get(op->ptr, "mode") == SEL_OP_SET) {
    if ((event->modifier & KM_SHIFT) && !(event->modifier & KM_CTRL)) {
      RNA_enum_set(op->ptr, "mode", SEL_OP_ADD);
    }
    else if ((event->modifier & KM_CTRL) && !(event->modifier & KM_SHIFT)) {
      RNA_enum_set(op->ptr, "mode", SEL_OP_SUB);
    }
  }
  return WM_gesture_circle_invoke(C, op, event);
}

static void DATABLOCK_GRAPH_OT_select_circle(wmOperatorType *ot)
{
  ot->name = "Circle Select";
  ot->idname = "DATABLOCK_GRAPH_OT_select_circle";
  ot->description = "Select nodes using circle selection (Shift extend, Ctrl subtract)";
  ot->invoke = circle_select_invoke;
  ot->modal = WM_gesture_circle_modal;
  ot->exec = circle_select_exec;
  ot->poll = region_window_poll;
  ot->get_name = ED_select_circle_get_name;
  ot->flag = OPTYPE_REGISTER;
  WM_operator_properties_gesture_circle(ot);
  WM_operator_properties_select_operation_simple(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Lasso Select
 * \{ */

static wmOperatorStatus lasso_select_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  const bool tweak = RNA_boolean_get(op->ptr, "tweak");
  if (tweak) {
    SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
    ARegion *region = CTX_wm_region(C);
    if (sdbg && region) {
      float2 view_co;
      mouse_to_view(region, event->mval, view_co);
      if (datablock_graph_find_node_at(*sdbg, view_co)) {
        return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
      }
    }
  }
  return WM_gesture_lasso_invoke(C, op, event);
}

static wmOperatorStatus lasso_select_exec(bContext *C, wmOperator *op)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  ARegion *region = CTX_wm_region(C);
  Main *bmain = CTX_data_main(C);
  if (!sdbg || !region || !bmain) {
    return OPERATOR_CANCELLED;
  }
  datablock_graph_rebuild_if_needed(*sdbg, *bmain);

  const Array<int2> mcoords = WM_gesture_lasso_path_to_array(C, op);
  if (mcoords.is_empty()) {
    return OPERATOR_PASS_THROUGH;
  }

  datablock_graph_push_undo(C, sdbg, "Lasso Select Data-Block Graph");

  const eSelectOp sel_op = eSelectOp(RNA_enum_get(op->ptr, "mode"));
  const bool select = (sel_op != SEL_OP_SUB);
  if (SEL_OP_USE_PRE_DESELECT(sel_op)) {
    datablock_graph_deselect_all(*sdbg);
  }

  rcti rect;
  BLI_lasso_boundbox(&rect, mcoords);

  for (GraphNode &node : sdbg->runtime->nodes) {
    if (select && node.selected) {
      continue;
    }
    const rctf nr = datablock_graph_node_rect(node);
    const float2 center = {BLI_rctf_cent_x(&nr), BLI_rctf_cent_y(&nr)};
    int2 screen_co;
    if (ui::view2d_view_to_region_clip(
            &region->v2d, center.x, center.y, &screen_co.x, &screen_co.y) &&
        BLI_rcti_isect_pt(&rect, screen_co.x, screen_co.y) &&
        BLI_lasso_is_point_inside(mcoords, screen_co.x, screen_co.y, INT_MAX))
    {
      node.selected = select;
      if (select) {
        sdbg->runtime->active_key = datablock_graph_node_key(node.seed_uid, node.id_session_uid);
      }
    }
  }
  notify_select(C);
  return OPERATOR_FINISHED;
}

static void DATABLOCK_GRAPH_OT_select_lasso(wmOperatorType *ot)
{
  ot->name = "Lasso Select";
  ot->idname = "DATABLOCK_GRAPH_OT_select_lasso";
  ot->description = "Select nodes using lasso selection";
  ot->invoke = lasso_select_invoke;
  ot->modal = WM_gesture_lasso_modal;
  ot->exec = lasso_select_exec;
  ot->cancel = WM_gesture_lasso_cancel;
  ot->poll = region_window_poll;
  ot->flag = OPTYPE_DEPENDS_ON_CURSOR;

  PropertyRNA *prop = RNA_def_boolean(
      ot->srna, "tweak", false, "Tweak", "Only activate when mouse is not over a node");
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  WM_operator_properties_gesture_lasso(ot);
  WM_operator_properties_select_operation_simple(ot);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Transform: Translate (G / LMB-over-node), Rotate (R), Scale (S)
 * \{ */

enum class XFormMode { Translate, Rotate, Scale };

enum class AxisLock {
  None = 0,
  X = 1,
  Y = 2,
};

struct XFormData {
  XFormMode mode = XFormMode::Translate;
  float2 start_mouse_view = float2(0.0f);
  float2 pivot = float2(0.0f);
  Map<uint64_t, float2> start_locations;
  /** Keyboard invoke (G/R/S): confirm with LMB/Enter, cancel RMB/Esc. */
  bool from_key = false;
  bool has_moved = false;
  /** Middle-mouse / X / Y axis constraint (node-editor style). */
  AxisLock axis_lock = AxisLock::None;
};

static float2 selection_pivot(SpaceDataBlockGraph &sdbg)
{
  float2 sum = float2(0.0f);
  int n = 0;
  for (const GraphNode &node : sdbg.runtime->nodes) {
    if (node.selected) {
      sum += node.location + float2(node.size.x * 0.5f, 0.0f);
      n++;
    }
  }
  if (n == 0) {
    return float2(0.0f);
  }
  return sum / float(n);
}

static bool xform_begin(bContext *C,
                        wmOperator *op,
                        const wmEvent *event,
                        const XFormMode mode,
                        const bool require_hit)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  ARegion *region = CTX_wm_region(C);
  Main *bmain = CTX_data_main(C);
  if (!sdbg || !region || !bmain) {
    return false;
  }

  if (event->keymodifier == EVT_UKEY) {
    return false;
  }

  datablock_graph_rebuild_if_needed(*sdbg, *bmain);
  datablock_graph_update_view2d(*region, *sdbg, false);

  float2 view_co;
  mouse_to_view(region, event->mval, view_co);
  GraphNode *hit = datablock_graph_find_node_at(*sdbg, view_co);

  const bool from_key = !ELEM(event->type, LEFTMOUSE, MIDDLEMOUSE, RIGHTMOUSE, MOUSEMOVE);

  if (require_hit) {
    /* LMB path: only grab when the cursor is over a node (never steal empty-space box-select). */
    if (!hit) {
      return false;
    }
    if (!hit->selected) {
      datablock_graph_deselect_all(*sdbg);
      hit->selected = true;
    }
    sdbg->runtime->active_key = datablock_graph_node_key(hit->seed_uid, hit->id_session_uid);
  }
  else {
    /* G/R/S path: operate on current selection. */
    if (datablock_graph_count_selected(*sdbg) == 0) {
      if (hit) {
        datablock_graph_deselect_all(*sdbg);
        hit->selected = true;
        sdbg->runtime->active_key = datablock_graph_node_key(hit->seed_uid, hit->id_session_uid);
      }
      else {
        return false;
      }
    }
  }

  if (datablock_graph_count_selected(*sdbg) == 0) {
    return false;
  }

  /* Snapshot before transform so Ctrl+Z restores pre-drag positions. */
  datablock_graph_push_undo(C, sdbg, "Transform Data-Block Graph");

  XFormData *data = MEM_new<XFormData>(__func__);
  data->mode = mode;
  data->start_mouse_view = view_co;
  data->pivot = selection_pivot(*sdbg);
  data->from_key = from_key;
  for (GraphNode &n : sdbg->runtime->nodes) {
    if (n.selected) {
      data->start_locations.add(node_key(n), n.location);
      n.position_locked = true;
    }
  }
  op->customdata = data;
  WM_cursor_modal_set(CTX_wm_window(C), WM_CURSOR_NSEW_SCROLL);
  WM_event_add_modal_handler(C, op);
  ED_region_tag_redraw(region);
  return true;
}

/** View-space grid for node move snap (similar to node editor increment snap). */
static constexpr float DATABLOCK_GRAPH_SNAP_GRID = 20.0f;

static float2 snap_location_to_grid(const float2 start_loc,
                                    const float2 delta,
                                    const float grid)
{
  /* Snap start to grid, then offset in grid steps — keeps multi-node selection in sync. */
  const float2 snapped_start = math::round(start_loc / grid) * grid;
  const float2 snapped_delta = math::round(delta / grid) * grid;
  return snapped_start + snapped_delta;
}

static float2 apply_axis_lock(const float2 delta, const AxisLock lock)
{
  switch (lock) {
    case AxisLock::X:
      return float2(delta.x, 0.0f);
    case AxisLock::Y:
      return float2(0.0f, delta.y);
    case AxisLock::None:
      break;
  }
  return delta;
}

static void xform_apply(SpaceDataBlockGraph &sdbg,
                        const XFormData &data,
                        const float2 &view_co,
                        const bool snap_grid)
{
  switch (data.mode) {
    case XFormMode::Translate: {
      float2 delta = apply_axis_lock(view_co - data.start_mouse_view, data.axis_lock);
      const float grid = DATABLOCK_GRAPH_SNAP_GRID;
      for (auto item : data.start_locations.items()) {
        if (GraphNode *n = find_node_by_key(sdbg, item.key)) {
          n->location = snap_grid ? snap_location_to_grid(item.value, delta, grid) :
                                    (item.value + delta);
          n->position_locked = true;
        }
      }
      break;
    }
    case XFormMode::Rotate: {
      /* 2D rotate: axis lock is not used (full plane rotation). */
      const float2 a = data.start_mouse_view - data.pivot;
      const float2 b = view_co - data.pivot;
      const float angle = atan2f(a.x * b.y - a.y * b.x, math::dot(a, b));
      const float ca = cosf(angle);
      const float sa = sinf(angle);
      for (auto item : data.start_locations.items()) {
        if (GraphNode *n = find_node_by_key(sdbg, item.key)) {
          const float2 rel = item.value + float2(n->size.x * 0.5f, 0.0f) - data.pivot;
          float2 rot(rel.x * ca - rel.y * sa, rel.x * sa + rel.y * ca);
          if (data.axis_lock == AxisLock::X) {
            rot.y = rel.y;
          }
          else if (data.axis_lock == AxisLock::Y) {
            rot.x = rel.x;
          }
          n->location = data.pivot + rot - float2(n->size.x * 0.5f, 0.0f);
          n->position_locked = true;
        }
      }
      break;
    }
    case XFormMode::Scale: {
      const float2 d0v = data.start_mouse_view - data.pivot;
      const float2 d1v = view_co - data.pivot;
      float factor_x = 1.0f;
      float factor_y = 1.0f;
      if (data.axis_lock == AxisLock::X) {
        factor_x = (math::abs(d0v.x) > 1e-4f) ? (d1v.x / d0v.x) : 1.0f;
        factor_x = math::clamp(factor_x, 0.05f, 20.0f);
        factor_y = 1.0f;
      }
      else if (data.axis_lock == AxisLock::Y) {
        factor_y = (math::abs(d0v.y) > 1e-4f) ? (d1v.y / d0v.y) : 1.0f;
        factor_y = math::clamp(factor_y, 0.05f, 20.0f);
        factor_x = 1.0f;
      }
      else {
        const float d0 = math::length(d0v);
        const float d1 = math::length(d1v);
        float factor = (d0 > 1e-4f) ? (d1 / d0) : 1.0f;
        factor = math::clamp(factor, 0.05f, 20.0f);
        factor_x = factor_y = factor;
      }
      for (auto item : data.start_locations.items()) {
        if (GraphNode *n = find_node_by_key(sdbg, item.key)) {
          const float2 center = item.value + float2(n->size.x * 0.5f, 0.0f);
          const float2 offset = center - data.pivot;
          const float2 scaled = data.pivot + float2(offset.x * factor_x, offset.y * factor_y);
          n->location = scaled - float2(n->size.x * 0.5f, 0.0f);
          n->position_locked = true;
        }
      }
      break;
    }
  }
}

static wmOperatorStatus xform_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  ARegion *region = CTX_wm_region(C);
  XFormData *data = static_cast<XFormData *>(op->customdata);
  if (!sdbg || !region || !data) {
    restore_cursor(C);
    return OPERATOR_CANCELLED;
  }

  auto finish = [&](const bool ok) -> wmOperatorStatus {
    if (!ok) {
      for (auto item : data->start_locations.items()) {
        if (GraphNode *n = find_node_by_key(*sdbg, item.key)) {
          n->location = item.value;
        }
      }
      /* Drop the pre-transform snapshot — nothing committed. */
      if (sdbg->runtime && !sdbg->runtime->undo_stack.is_empty()) {
        sdbg->runtime->undo_stack.remove_last();
      }
    }
    else {
      /* Positions already changed during modal; snapshot was taken at invoke. */
      datablock_graph_store_positions(*sdbg);
    }
    MEM_delete(data);
    op->customdata = nullptr;
    restore_cursor(C);
    datablock_graph_update_view2d(*region, *sdbg, false);
    ED_region_tag_redraw(region);
    return ok ? OPERATOR_FINISHED : OPERATOR_CANCELLED;
  };

  switch (event->type) {
    case LEFTMOUSE: {
      if (data->from_key) {
        /* G/R/S: confirm with LMB press. */
        if (event->val == KM_PRESS) {
          return finish(true);
        }
      }
      else {
        /* Mouse drag started with LMB press: confirm only on release. */
        if (event->val == KM_RELEASE) {
          return finish(true);
        }
      }
      break;
    }
    case EVT_RETKEY:
    case EVT_PADENTER: {
      if (event->val == KM_PRESS) {
        return finish(true);
      }
      break;
    }
    case EVT_XKEY: {
      if (event->val == KM_PRESS) {
        data->axis_lock = (data->axis_lock == AxisLock::X) ? AxisLock::None : AxisLock::X;
        float2 view_co;
        mouse_to_view(region, event->mval, view_co);
        const bool snap_grid = (sdbg->flag & DATABLOCK_GRAPH_FLAG_SNAP) != 0 &&
                               (event->modifier & KM_CTRL) == 0;
        xform_apply(*sdbg, *data, view_co, snap_grid);
        ED_region_tag_redraw(region);
      }
      break;
    }
    case EVT_YKEY: {
      if (event->val == KM_PRESS) {
        data->axis_lock = (data->axis_lock == AxisLock::Y) ? AxisLock::None : AxisLock::Y;
        float2 view_co;
        mouse_to_view(region, event->mval, view_co);
        const bool snap_grid = (sdbg->flag & DATABLOCK_GRAPH_FLAG_SNAP) != 0 &&
                               (event->modifier & KM_CTRL) == 0;
        xform_apply(*sdbg, *data, view_co, snap_grid);
        ED_region_tag_redraw(region);
      }
      break;
    }
    case MIDDLEMOUSE: {
      /* Cycle free → X → Y → free (same idea as transform constrain). */
      if (event->val == KM_PRESS) {
        if (data->axis_lock == AxisLock::None) {
          data->axis_lock = AxisLock::X;
        }
        else if (data->axis_lock == AxisLock::X) {
          data->axis_lock = AxisLock::Y;
        }
        else {
          data->axis_lock = AxisLock::None;
        }
        float2 view_co;
        mouse_to_view(region, event->mval, view_co);
        const bool snap_grid = (sdbg->flag & DATABLOCK_GRAPH_FLAG_SNAP) != 0 &&
                               (event->modifier & KM_CTRL) == 0;
        xform_apply(*sdbg, *data, view_co, snap_grid);
        ED_region_tag_redraw(region);
      }
      break;
    }
    case EVT_ESCKEY:
    case RIGHTMOUSE: {
      if (event->val == KM_PRESS || event->type == EVT_ESCKEY) {
        return finish(false);
      }
      break;
    }
    case MOUSEMOVE: {
      float2 view_co;
      mouse_to_view(region, event->mval, view_co);
      data->has_moved = true;
      /* Header magnet enables snap; hold Ctrl to temporarily free-move. */
      const bool snap_grid = (sdbg->flag & DATABLOCK_GRAPH_FLAG_SNAP) != 0 &&
                             (event->modifier & KM_CTRL) == 0;
      xform_apply(*sdbg, *data, view_co, snap_grid);
      ED_region_tag_redraw(region);
      break;
    }
    default:
      break;
  }
  return OPERATOR_RUNNING_MODAL;
}

static void xform_cancel(bContext *C, wmOperator *op)
{
  XFormData *data = static_cast<XFormData *>(op->customdata);
  if (data) {
    SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
    if (sdbg) {
      for (auto item : data->start_locations.items()) {
        if (GraphNode *n = find_node_by_key(*sdbg, item.key)) {
          n->location = item.value;
        }
      }
    }
    MEM_delete(data);
    op->customdata = nullptr;
  }
  restore_cursor(C);
}

static wmOperatorStatus translate_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  /* Mouse: only when over a node. Keyboard G: selection. */
  const bool require_hit = ELEM(event->type, LEFTMOUSE);
  if (!xform_begin(C, op, event, XFormMode::Translate, require_hit)) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus rotate_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  if (!xform_begin(C, op, event, XFormMode::Rotate, false)) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus scale_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  if (!xform_begin(C, op, event, XFormMode::Scale, false)) {
    return OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH;
  }
  return OPERATOR_RUNNING_MODAL;
}

static void DATABLOCK_GRAPH_OT_translate_node(wmOperatorType *ot)
{
  ot->name = "Move Nodes";
  ot->idname = "DATABLOCK_GRAPH_OT_translate_node";
  ot->description = "Move selected nodes (G or drag on a node)";
  ot->invoke = translate_invoke;
  ot->modal = xform_modal;
  ot->cancel = xform_cancel;
  ot->poll = region_window_poll;
  ot->flag = OPTYPE_BLOCKING;
}

static void DATABLOCK_GRAPH_OT_rotate_node(wmOperatorType *ot)
{
  ot->name = "Rotate Nodes";
  ot->idname = "DATABLOCK_GRAPH_OT_rotate_node";
  ot->description = "Rotate selected nodes around their center (R)";
  ot->invoke = rotate_invoke;
  ot->modal = xform_modal;
  ot->cancel = xform_cancel;
  ot->poll = region_window_poll;
  ot->flag = OPTYPE_BLOCKING;
}

static void DATABLOCK_GRAPH_OT_scale_node(wmOperatorType *ot)
{
  ot->name = "Scale Nodes";
  ot->idname = "DATABLOCK_GRAPH_OT_scale_node";
  ot->description = "Scale selected node positions around their center (S)";
  ot->invoke = scale_invoke;
  ot->modal = xform_modal;
  ot->cancel = xform_cancel;
  ot->poll = region_window_poll;
  ot->flag = OPTYPE_BLOCKING;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Hold-U Align
 * \{ */

struct AlignModalData {
  float2 start_mval = float2(0.0f);
  bool axis_locked = false;
  bool horizontal = true;
  Map<uint64_t, float2> start_locations;
  bool dragging = false;
};

static bool align_poll(bContext *C)
{
  if (!region_window_poll(C)) {
    return false;
  }
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  if (!sdbg) {
    return false;
  }
  datablock_graph_ensure_runtime(*sdbg);
  return datablock_graph_count_selected(*sdbg) >= 2;
}

static wmOperatorStatus align_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  if (!sdbg || datablock_graph_count_selected(*sdbg) < 2) {
    BKE_report(op->reports, RPT_WARNING, "Select at least two nodes to align");
    return OPERATOR_CANCELLED;
  }

  datablock_graph_push_undo(C, sdbg, "Align Data-Block Graph");

  AlignModalData *data = MEM_new<AlignModalData>(__func__);
  for (GraphNode &n : sdbg->runtime->nodes) {
    if (n.selected) {
      data->start_locations.add(node_key(n), n.location);
    }
  }
  op->customdata = data;
  WM_event_add_modal_handler(C, op);
  WM_cursor_modal_set(CTX_wm_window(C), WM_CURSOR_NSEW_SCROLL);
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus align_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  ARegion *region = CTX_wm_region(C);
  AlignModalData *data = static_cast<AlignModalData *>(op->customdata);
  if (!sdbg || !region || !data) {
    return OPERATOR_CANCELLED;
  }

  auto restore_and_finish = [&](wmOperatorStatus status) {
    if (status == OPERATOR_CANCELLED) {
      for (auto item : data->start_locations.items()) {
        if (GraphNode *n = find_node_by_key(*sdbg, item.key)) {
          n->location = item.value;
        }
      }
    }
    else if (status == OPERATOR_FINISHED) {
      datablock_graph_store_positions(*sdbg);
    }
    MEM_delete(data);
    op->customdata = nullptr;
    restore_cursor(C);
    ED_region_tag_redraw(region);
    return status;
  };

  switch (event->type) {
    case LEFTMOUSE: {
      if (event->val != KM_PRESS) {
        if (event->val == KM_RELEASE && data->dragging) {
          return restore_and_finish(OPERATOR_FINISHED);
        }
        return OPERATOR_RUNNING_MODAL;
      }
      data->dragging = true;
      data->start_mval = float2(float(event->mval[0]), float(event->mval[1]));
      data->axis_locked = false;
      return OPERATOR_RUNNING_MODAL;
    }
    case MOUSEMOVE: {
      if (!data->dragging) {
        return OPERATOR_RUNNING_MODAL;
      }
      const float2 mval(float(event->mval[0]), float(event->mval[1]));
      const float2 delta = mval - data->start_mval;
      constexpr float deadzone = 10.0f;
      if (!data->axis_locked) {
        if (math::max(math::abs(delta.x), math::abs(delta.y)) < deadzone) {
          for (auto item : data->start_locations.items()) {
            if (GraphNode *n = find_node_by_key(*sdbg, item.key)) {
              n->location = item.value;
            }
          }
          ED_region_tag_redraw(region);
          return OPERATOR_RUNNING_MODAL;
        }
        data->horizontal = math::abs(delta.x) >= math::abs(delta.y);
        data->axis_locked = true;
      }
      for (auto item : data->start_locations.items()) {
        if (GraphNode *n = find_node_by_key(*sdbg, item.key)) {
          n->location = item.value;
        }
      }
      datablock_graph_align_selection(*sdbg, data->horizontal);
      ED_region_tag_redraw(region);
      return OPERATOR_RUNNING_MODAL;
    }
    case EVT_UKEY:
      if (event->val == KM_RELEASE && !data->dragging) {
        return restore_and_finish(OPERATOR_CANCELLED);
      }
      return OPERATOR_RUNNING_MODAL;
    case EVT_ESCKEY:
    case RIGHTMOUSE:
      if (event->val == KM_PRESS) {
        return restore_and_finish(OPERATOR_CANCELLED);
      }
      return OPERATOR_RUNNING_MODAL;
    case MIDDLEMOUSE:
    case WHEELUPMOUSE:
    case WHEELDOWNMOUSE:
    case MOUSEPAN:
    case MOUSEZOOM:
      return OPERATOR_PASS_THROUGH;
    default:
      return OPERATOR_RUNNING_MODAL;
  }
}

static void align_cancel(bContext *C, wmOperator *op)
{
  AlignModalData *data = static_cast<AlignModalData *>(op->customdata);
  if (data) {
    SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
    if (sdbg) {
      for (auto item : data->start_locations.items()) {
        if (GraphNode *n = find_node_by_key(*sdbg, item.key)) {
          n->location = item.value;
        }
      }
    }
    MEM_delete(data);
    op->customdata = nullptr;
  }
  restore_cursor(C);
}

static void DATABLOCK_GRAPH_OT_align_selection(wmOperatorType *ot)
{
  ot->name = "Align Selected Nodes";
  ot->idname = "DATABLOCK_GRAPH_OT_align_selection";
  ot->description =
      "Hold U then left-drag anywhere to align and space selected nodes (Houdini-style)";
  ot->invoke = align_invoke;
  ot->modal = align_modal;
  ot->cancel = align_cancel;
  ot->poll = align_poll;
  ot->flag = OPTYPE_BLOCKING;
}

/** \} */

static wmOperatorStatus datablock_graph_undo_exec(bContext *C, wmOperator * /*op*/)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  Main *bmain = CTX_data_main(C);
  if (!sdbg || !bmain) {
    return OPERATOR_CANCELLED;
  }
  if (!datablock_graph_undo_step(*sdbg, *bmain)) {
    return OPERATOR_CANCELLED;
  }
  if (ScrArea *area = CTX_wm_area(C)) {
    if (ARegion *winreg = BKE_area_find_region_type(area, RGN_TYPE_WINDOW)) {
      datablock_graph_update_view2d(*winreg, *sdbg, false);
    }
  }
  ED_area_tag_redraw(CTX_wm_area(C));
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_DATABLOCK_GRAPH, nullptr);
  return OPERATOR_FINISHED;
}

static bool datablock_graph_undo_poll(bContext *C)
{
  if (!ED_operator_datablock_graph_active(C)) {
    return false;
  }
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  return sdbg && datablock_graph_undo_can_undo(*sdbg);
}

static void DATABLOCK_GRAPH_OT_undo(wmOperatorType *ot)
{
  ot->name = "Undo";
  ot->idname = "DATABLOCK_GRAPH_OT_undo";
  ot->description = "Undo the last Data-Block Graph edit (seeds, layout)";
  ot->exec = datablock_graph_undo_exec;
  ot->poll = datablock_graph_undo_poll;
  ot->flag = OPTYPE_REGISTER;
}

static wmOperatorStatus datablock_graph_redo_exec(bContext *C, wmOperator * /*op*/)
{
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  Main *bmain = CTX_data_main(C);
  if (!sdbg || !bmain) {
    return OPERATOR_CANCELLED;
  }
  if (!datablock_graph_redo_step(*sdbg, *bmain)) {
    return OPERATOR_CANCELLED;
  }
  if (ScrArea *area = CTX_wm_area(C)) {
    if (ARegion *winreg = BKE_area_find_region_type(area, RGN_TYPE_WINDOW)) {
      datablock_graph_update_view2d(*winreg, *sdbg, false);
    }
  }
  ED_area_tag_redraw(CTX_wm_area(C));
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_DATABLOCK_GRAPH, nullptr);
  return OPERATOR_FINISHED;
}

static bool datablock_graph_redo_poll(bContext *C)
{
  if (!ED_operator_datablock_graph_active(C)) {
    return false;
  }
  SpaceDataBlockGraph *sdbg = CTX_wm_space_datablock_graph(C);
  return sdbg && datablock_graph_undo_can_redo(*sdbg);
}

static void DATABLOCK_GRAPH_OT_redo(wmOperatorType *ot)
{
  ot->name = "Redo";
  ot->idname = "DATABLOCK_GRAPH_OT_redo";
  ot->description = "Redo the last undone Data-Block Graph edit";
  ot->exec = datablock_graph_redo_exec;
  ot->poll = datablock_graph_redo_poll;
  ot->flag = OPTYPE_REGISTER;
}

void datablock_graph_operatortypes()
{
  WM_operatortype_append(DATABLOCK_GRAPH_OT_clear);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_delete);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_relayout);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_refresh);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_undo);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_redo);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_add_seed);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_select);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_select_all);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_select_box);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_select_circle);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_select_lasso);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_translate_node);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_rotate_node);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_scale_node);
  WM_operatortype_append(DATABLOCK_GRAPH_OT_align_selection);
}

static void ensure_item(wmKeyMap *keymap, const char *idname, const KeyMapItem_Params &params)
{
  for (wmKeyMapItem &kmi : keymap->items) {
    if (STREQ(kmi.idname, idname) && kmi.type == params.type && kmi.val == params.value &&
        kmi.shift == ((params.modifier & KM_SHIFT) ? KM_MOD_HELD : 0) &&
        kmi.ctrl == ((params.modifier & KM_CTRL) ? KM_MOD_HELD : 0) &&
        kmi.alt == ((params.modifier & KM_ALT) ? KM_MOD_HELD : 0))
    {
      return;
    }
  }
  WM_keymap_add_item(keymap, idname, &params);
}

static void ensure_tool_keymaps(wmKeyConfig *keyconf)
{
  auto ensure_tool_map = [&](const char *name) -> wmKeyMap * {
    wmKeyMap *km = WM_keymap_ensure(keyconf, name, SPACE_DATABLOCK_GRAPH, RGN_TYPE_WINDOW);
    km->flag |= KEYMAP_TOOL;
    return km;
  };

  KeyMapItem_Params p{};
  p.direction = KM_ANY;
  p.keymodifier = 0;
  p.modifier = 0;

  {
    wmKeyMap *km = ensure_tool_map("Data-Block Graph Tool: Tweak");
    p.type = LEFTMOUSE;
    p.value = KM_CLICK;
    ensure_item(km, "DATABLOCK_GRAPH_OT_select", p);
    /* Drag-move only on PRESS over node (operator enforces hit). */
    p.value = KM_PRESS;
    ensure_item(km, "DATABLOCK_GRAPH_OT_translate_node", p);
  }

  {
    wmKeyMap *km = ensure_tool_map("Data-Block Graph Tool: Select Box");
    p.type = LEFTMOUSE;
    p.value = KM_CLICK;
    ensure_item(km, "DATABLOCK_GRAPH_OT_select", p);
    p.value = KM_PRESS_DRAG;
    wmKeyMapItem *kmi = WM_keymap_add_item(km, "DATABLOCK_GRAPH_OT_select_box", &p);
    if (kmi && kmi->ptr) {
      RNA_boolean_set(kmi->ptr, "tweak", true);
    }
  }

  {
    wmKeyMap *km = ensure_tool_map("Data-Block Graph Tool: Select Circle");
    p.type = LEFTMOUSE;
    p.value = KM_PRESS;
    wmKeyMapItem *kmi = WM_keymap_add_item(km, "DATABLOCK_GRAPH_OT_select_circle", &p);
    if (kmi && kmi->ptr && RNA_struct_find_property(kmi->ptr, "wait_for_input")) {
      RNA_boolean_set(kmi->ptr, "wait_for_input", false);
    }
  }

  {
    wmKeyMap *km = ensure_tool_map("Data-Block Graph Tool: Select Lasso");
    p.type = LEFTMOUSE;
    p.value = KM_PRESS_DRAG;
    wmKeyMapItem *kmi = WM_keymap_add_item(km, "DATABLOCK_GRAPH_OT_select_lasso", &p);
    if (kmi && kmi->ptr) {
      RNA_boolean_set(kmi->ptr, "tweak", true);
    }
  }
}

void datablock_graph_keymap(wmKeyConfig *keyconf)
{
  wmKeyMap *keymap = WM_keymap_ensure(
      keyconf, "Data-Block Graph Generic", SPACE_DATABLOCK_GRAPH, RGN_TYPE_WINDOW);

  KeyMapItem_Params p{};
  p.direction = KM_ANY;
  p.keymodifier = 0;
  p.modifier = 0;

  /* T: toolbar. */
  p.type = EVT_TKEY;
  p.value = KM_PRESS;
  {
    bool found = false;
    for (wmKeyMapItem &it : keymap->items) {
      if (STREQ(it.idname, "WM_OT_context_toggle") && it.type == EVT_TKEY) {
        found = true;
        if (it.ptr) {
          RNA_string_set(it.ptr, "data_path", "space_data.show_region_toolbar");
        }
        break;
      }
    }
    if (!found) {
      wmKeyMapItem *kmi = WM_keymap_add_item(keymap, "WM_OT_context_toggle", &p);
      if (kmi && kmi->ptr) {
        RNA_string_set(kmi->ptr, "data_path", "space_data.show_region_toolbar");
      }
    }
  }

  /* G / R / S — multi-select transform (node editor parity). */
  p.type = EVT_GKEY;
  p.value = KM_PRESS;
  p.modifier = 0;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_translate_node", p);
  p.type = EVT_RKEY;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_rotate_node", p);
  p.type = EVT_SKEY;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_scale_node", p);

  /* Hold-U align. */
  p.type = EVT_UKEY;
  p.value = KM_PRESS;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_align_selection", p);

  /* Delete / X / Ctrl+X — remove selected seed trees from the editor. */
  p.type = EVT_XKEY;
  p.value = KM_PRESS;
  p.modifier = 0;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_delete", p);
  p.type = EVT_DELKEY;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_delete", p);
  p.type = EVT_XKEY;
  p.modifier = KM_CTRL;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_delete", p);
  p.modifier = 0;

  /* Ctrl+Z / Ctrl+Shift+Z — editor-local undo (space DNA not reliable in Global Undo). */
  p.type = EVT_ZKEY;
  p.value = KM_PRESS;
  p.modifier = KM_CTRL;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_undo", p);
  p.modifier = KM_CTRL | KM_SHIFT;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_redo", p);
  p.modifier = 0;
  /* Ctrl+Y redo (Windows convention). */
  p.type = EVT_YKEY;
  p.modifier = KM_CTRL;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_redo", p);
  p.modifier = 0;

  /* Click select. */
  p.type = LEFTMOUSE;
  p.value = KM_CLICK;
  p.modifier = 0;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_select", p);
  p.modifier = KM_SHIFT;
  {
    wmKeyMapItem *kmi = nullptr;
    for (wmKeyMapItem &it : keymap->items) {
      if (STREQ(it.idname, "DATABLOCK_GRAPH_OT_select") && it.type == LEFTMOUSE &&
          it.val == KM_CLICK && it.shift)
      {
        kmi = &it;
        break;
      }
    }
    if (!kmi) {
      kmi = WM_keymap_add_item(keymap, "DATABLOCK_GRAPH_OT_select", &p);
    }
    if (kmi) {
      kmi->shift = KM_MOD_HELD;
      if (kmi->ptr) {
        RNA_boolean_set(kmi->ptr, "extend", true);
      }
    }
  }

  /* LMB press = drag-move ONLY when over a node (operator passes through empty). */
  p.type = LEFTMOUSE;
  p.value = KM_PRESS;
  p.modifier = 0;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_translate_node", p);

  /* Box: B / Shift+B (add) / Ctrl+B (sub), and empty-space drag variants. */
  auto add_box_key = [&](short type, short value, int modifier, int mode, bool tweak) {
    KeyMapItem_Params bp = p;
    bp.type = type;
    bp.value = value;
    bp.modifier = modifier;
    wmKeyMapItem *kmi = WM_keymap_add_item(keymap, "DATABLOCK_GRAPH_OT_select_box", &bp);
    if (!kmi) {
      return;
    }
    if (modifier & KM_SHIFT) {
      kmi->shift = KM_MOD_HELD;
    }
    if (modifier & KM_CTRL) {
      kmi->ctrl = KM_MOD_HELD;
    }
    if (kmi->ptr) {
      RNA_enum_set(kmi->ptr, "mode", mode);
      if (tweak) {
        RNA_boolean_set(kmi->ptr, "tweak", true);
      }
    }
  };
  add_box_key(EVT_BKEY, KM_PRESS, 0, SEL_OP_SET, false);
  add_box_key(EVT_BKEY, KM_PRESS, KM_SHIFT, SEL_OP_ADD, false);
  add_box_key(EVT_BKEY, KM_PRESS, KM_CTRL, SEL_OP_SUB, false);
  add_box_key(LEFTMOUSE, KM_PRESS_DRAG, 0, SEL_OP_SET, true);
  add_box_key(LEFTMOUSE, KM_PRESS_DRAG, KM_SHIFT, SEL_OP_ADD, true);
  add_box_key(LEFTMOUSE, KM_PRESS_DRAG, KM_CTRL, SEL_OP_SUB, true);

  /* Lasso. */
  p.type = LEFTMOUSE;
  p.value = KM_PRESS_DRAG;
  p.modifier = KM_CTRL | KM_ALT;
  {
    wmKeyMapItem *kmi = WM_keymap_add_item(keymap, "DATABLOCK_GRAPH_OT_select_lasso", &p);
    if (kmi) {
      kmi->ctrl = KM_MOD_HELD;
      kmi->alt = KM_MOD_HELD;
      if (kmi->ptr) {
        RNA_enum_set(kmi->ptr, "mode", SEL_OP_ADD);
      }
    }
  }
  p.modifier = KM_SHIFT | KM_CTRL | KM_ALT;
  {
    wmKeyMapItem *kmi = WM_keymap_add_item(keymap, "DATABLOCK_GRAPH_OT_select_lasso", &p);
    if (kmi) {
      kmi->shift = KM_MOD_HELD;
      kmi->ctrl = KM_MOD_HELD;
      kmi->alt = KM_MOD_HELD;
      if (kmi->ptr) {
        RNA_enum_set(kmi->ptr, "mode", SEL_OP_SUB);
      }
    }
  }

  /* Circle brush: C / Shift+C / Ctrl+C. */
  auto add_circle_key = [&](int modifier, int mode) {
    KeyMapItem_Params cp{};
    cp.type = EVT_CKEY;
    cp.value = KM_PRESS;
    cp.modifier = modifier;
    cp.direction = KM_ANY;
    cp.keymodifier = 0;
    wmKeyMapItem *kmi = WM_keymap_add_item(keymap, "DATABLOCK_GRAPH_OT_select_circle", &cp);
    if (!kmi) {
      return;
    }
    if (modifier & KM_SHIFT) {
      kmi->shift = KM_MOD_HELD;
    }
    if (modifier & KM_CTRL) {
      kmi->ctrl = KM_MOD_HELD;
    }
    if (kmi->ptr) {
      RNA_enum_set(kmi->ptr, "mode", mode);
    }
  };
  add_circle_key(0, SEL_OP_SET);
  add_circle_key(KM_SHIFT, SEL_OP_ADD);
  add_circle_key(KM_CTRL, SEL_OP_SUB);

  p.type = EVT_AKEY;
  p.value = KM_PRESS;
  p.modifier = 0;
  ensure_item(keymap, "DATABLOCK_GRAPH_OT_select_all", p);

  ensure_tool_keymaps(keyconf);
}

void datablock_graph_ensure_region_keymap(wmWindowManager *wm, ARegion *region)
{
  if (!wm || !region) {
    return;
  }
  datablock_graph_keymap(wm->runtime->defaultconf);
  if (wm->runtime->userconf) {
    datablock_graph_keymap(wm->runtime->userconf);
  }
  wmKeyMap *keymap = WM_keymap_ensure(wm->runtime->defaultconf,
                                      "Data-Block Graph Generic",
                                      SPACE_DATABLOCK_GRAPH,
                                      RGN_TYPE_WINDOW);
  WM_event_add_keymap_handler_v2d_mask(&region->runtime->handlers, keymap);
}

static bool id_drop_poll(bContext * /*C*/, wmDrag *drag, const wmEvent * /*event*/)
{
  return WM_drag_is_ID_type(drag, 0);
}

static void id_drop_copy(bContext *C, wmDrag *drag, wmDropBox *drop)
{
  ID *id = WM_drag_get_local_ID(drag, 0);
  if (id) {
    RNA_int_set(drop->ptr, "session_uid", int(id->session_uid));
  }
  /* Store drop cursor in region space so the new tree is placed at release position. */
  if (ARegion *region = CTX_wm_region(C)) {
    if (const wmWindow *win = CTX_wm_window(C)) {
      if (win->runtime && win->runtime->eventstate) {
        const int mx = win->runtime->eventstate->xy[0] - region->winrct.xmin;
        const int my = win->runtime->eventstate->xy[1] - region->winrct.ymin;
        RNA_int_set(drop->ptr, "cursor_x", mx);
        RNA_int_set(drop->ptr, "cursor_y", my);
      }
    }
  }
}

void datablock_graph_dropboxes()
{
  ListBaseT<wmDropBox> *lb = WM_dropboxmap_find(
      "Data-Block Graph", SPACE_DATABLOCK_GRAPH, RGN_TYPE_WINDOW);
  WM_dropbox_add(
      lb, "DATABLOCK_GRAPH_OT_add_seed", id_drop_poll, id_drop_copy, nullptr, nullptr);
}

}  // namespace blender::ed::datablock_graph
