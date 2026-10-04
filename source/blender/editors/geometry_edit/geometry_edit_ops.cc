/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgeometry_edit
 *
 * Lightweight operators (select all / mode). Click/box/circle are driven by the
 * parent NODE_OT_select_elements_edit modal so they always use View3D coordinates.
 */

#include "BKE_context.hh"

#include "ED_geometry_edit.hh"
#include "ED_select_utils.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "geometry_edit_intern.hh"

namespace blender::ed::geometry_edit {

static bool geometry_edit_poll(bContext * /*C*/)
{
  return session_is_active();
}

static wmOperatorStatus geometry_edit_select_all_exec(bContext *C, wmOperator *op)
{
  const int action = RNA_enum_get(op->ptr, "action");
  if (select_all(C, action)) {
    return OPERATOR_FINISHED;
  }
  return OPERATOR_CANCELLED;
}

void GEOMETRY_EDIT_OT_select_all(wmOperatorType *ot)
{
  ot->name = "Geometry Edit Select All";
  ot->idname = "GEOMETRY_EDIT_OT_select_all";
  ot->description = "Select or deselect all geometry elements";

  ot->exec = geometry_edit_select_all_exec;
  ot->poll = geometry_edit_poll;
  ot->flag = OPTYPE_UNDO;

  WM_operator_properties_select_all(ot);
}

static wmOperatorStatus geometry_edit_select_mode_exec(bContext *C, wmOperator *op)
{
  const int type = RNA_enum_get(op->ptr, "type");
  bke::AttrDomain domain = bke::AttrDomain::Point;
  if (type == 2) {
    domain = bke::AttrDomain::Edge;
  }
  else if (type == 3) {
    domain = bke::AttrDomain::Face;
  }
  session_set_domain(domain);
  session_tag_redraw(C);
  return OPERATOR_FINISHED;
}

void GEOMETRY_EDIT_OT_select_mode(wmOperatorType *ot)
{
  ot->name = "Geometry Edit Select Mode";
  ot->idname = "GEOMETRY_EDIT_OT_select_mode";
  ot->description = "Change Geometry Edit selection domain (vertex/edge/face)";

  ot->exec = geometry_edit_select_mode_exec;
  ot->poll = geometry_edit_poll;
  ot->flag = OPTYPE_UNDO;

  static const EnumPropertyItem type_items[] = {
      {1, "VERT", 0, "Vertex", ""},
      {2, "EDGE", 0, "Edge", ""},
      {3, "FACE", 0, "Face", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };
  RNA_def_enum(ot->srna, "type", type_items, 1, "Type", "");
}

void operatortypes_geometry_edit()
{
  WM_operatortype_append(GEOMETRY_EDIT_OT_select_all);
  WM_operatortype_append(GEOMETRY_EDIT_OT_select_mode);
}

}  // namespace blender::ed::geometry_edit
