/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 *
 * Houdini-style selection align: press/hold U, then LMB-drag anywhere in the node editor.
 * Modal owns mouse events so box-select cannot steal empty-space drags.
 */

#include "BLI_math_vector.hh"
#include "BLI_utildefines.hh"

#include "BKE_context.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_report.hh"

#include "ED_node.hh"
#include "ED_screen.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "UI_interface.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

static int count_alignable_selected_nodes(const bNodeTree &ntree)
{
  int count = 0;
  for (const bNode *node : ntree.all_nodes()) {
    if ((node->flag & NODE_SELECT) && !node->is_frame()) {
      count++;
    }
  }
  return count;
}

static bool node_align_selection_poll(bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return false;
  }
  if (!ED_operator_node_editable(C)) {
    return false;
  }
  return count_alignable_selected_nodes(*snode->edittree) >= 2;
}

/**
 * Hold-U modal: block empty-space box-select; on LMB, hand off to transform.translate(node_align).
 */
static wmOperatorStatus node_align_selection_modal(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  switch (event->type) {
    case LEFTMOUSE: {
      if (event->val != KM_PRESS) {
        return OPERATOR_RUNNING_MODAL;
      }
      /* Hand off to transform: node_align latched, confirm on LMB release. */
      wmOperatorType *ot = WM_operatortype_find("TRANSFORM_OT_translate", false);
      if (!ot) {
        WM_cursor_modal_restore(CTX_wm_window(C));
        return OPERATOR_CANCELLED;
      }
      PointerRNA props = WM_operator_properties_create_ptr(ot);
      RNA_boolean_set(&props, "node_align", true);
      RNA_boolean_set(&props, "release_confirm", true);
      RNA_boolean_set(&props, "view2d_edge_pan", true);
      WM_cursor_modal_restore(CTX_wm_window(C));
      WM_operator_name_call_ptr(C, ot, wm::OpCallContext::InvokeDefault, &props, event);
      WM_operator_properties_free(&props);
      return OPERATOR_FINISHED;
    }

    case EVT_UKEY:
      if (event->val == KM_RELEASE) {
        /* Hold U then release without LMB: no layout (matches "U alone does nothing"). */
        return OPERATOR_CANCELLED;
      }
      return OPERATOR_RUNNING_MODAL;

    case EVT_ESCKEY:
    case RIGHTMOUSE:
      if (event->val == KM_PRESS) {
        return OPERATOR_CANCELLED;
      }
      return OPERATOR_RUNNING_MODAL;

    case MIDDLEMOUSE:
    case BUTTON4MOUSE:
    case BUTTON5MOUSE:
    case WHEELUPMOUSE:
    case WHEELDOWNMOUSE:
    case MOUSEPAN:
    case MOUSEZOOM:
    case MOUSEROTATE:
      /* Allow view navigation while waiting for LMB. */
      return OPERATOR_PASS_THROUGH;

    case MOUSEMOVE:
      /* Explicitly consume move events so box-select never starts under held U. */
      return OPERATOR_RUNNING_MODAL;

    default:
      return OPERATOR_RUNNING_MODAL;
  }
}

static wmOperatorStatus node_align_selection_invoke(bContext *C,
                                                    wmOperator *op,
                                                    const wmEvent * /*event*/)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return OPERATOR_CANCELLED;
  }
  if (count_alignable_selected_nodes(*snode->edittree) < 2) {
    BKE_report(op->reports, RPT_WARNING, "Select at least two nodes to align");
    return OPERATOR_CANCELLED;
  }

  WM_event_add_modal_handler(C, op);
  WM_cursor_modal_set(CTX_wm_window(C), WM_CURSOR_NSEW_SCROLL);
  return OPERATOR_RUNNING_MODAL;
}

static void node_align_selection_cancel(bContext *C, wmOperator * /*op*/)
{
  WM_cursor_modal_restore(CTX_wm_window(C));
}

void NODE_OT_align_selection(wmOperatorType *ot)
{
  ot->name = "Align Selected Nodes";
  ot->idname = "NODE_OT_align_selection";
  ot->description =
      "Hold U then left-drag anywhere to align and space selected nodes (Houdini-style)";

  ot->invoke = node_align_selection_invoke;
  ot->modal = node_align_selection_modal;
  ot->cancel = node_align_selection_cancel;
  ot->poll = node_align_selection_poll;

  ot->flag = OPTYPE_UNDO | OPTYPE_BLOCKING;
}

}  // namespace blender::ed::space_node
