/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgeometry_edit
 *
 * Temp mesh object + native Mesh Edit Mode for Select Elements / Edit Elements.
 */

#pragma once

#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"

#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"
#include "DNA_view3d_types.h"

#include "ED_geometry_edit.hh"

struct BMesh;
struct Mesh;
struct Object;
struct SpaceNode;
struct ScrArea;
struct bContext;
struct wmOperatorType;

namespace blender::ed::geometry_edit {

struct View3DGizmoBackup {
  View3D *v3d = nullptr;
  eView3D_GizmoFlag gizmo_flag = eView3D_GizmoFlag(0);
};

struct GeometryEditSession {
  bool active = false;
  SessionKind kind = SessionKind::SelectOnly;

  bNodeTree *ntree = nullptr;
  int32_t node_identifier = 0;

  Object *host_object = nullptr;
  eObject_VisibilityFlag prev_visibility_flag = eObject_VisibilityFlag(0);
  bool prev_base_selected = false;
  Object *prev_active_object = nullptr;

  Object *temp_object = nullptr;
  Mesh *temp_mesh = nullptr;

  float4x4 object_to_world = float4x4::identity();

  bke::AttrDomain domain = bke::AttrDomain::Point;

  int seed_verts = 0;
  int seed_edges = 0;
  int seed_faces = 0;
  /** Local-space rest positions at session start (select-only: must not change). */
  Vector<float3> seed_positions;

  Vector<View3DGizmoBackup> gizmo_backups;
  char prev_prop_edit = 0;
  bool prop_edit_saved = false;

  /**
   * Geometry Node editors we pinned on Enter (user-already-pinned are not listed).
   * Unpin only these on exit so user pins stay.
   */
  Vector<SpaceNode *> pinned_editors;
};

GeometryEditSession &session_state();

void session_tag_redraw(bContext *C);

short domain_to_selectmode(bke::AttrDomain domain);
bke::AttrDomain selectmode_to_domain(short selectmode);

void apply_seed_selection(BMesh &bm, bke::AttrDomain domain, Span<int> indices);
void read_selection_indices(BMesh &bm, bke::AttrDomain domain, Vector<int> &r_indices);
void read_vertex_positions(BMesh &bm, Vector<float3> &r_positions);

bool session_topology_ok();
bool session_positions_ok();

/** Enforce mode constraints (undo illegal edits). */
bool session_enforce_constraints(bContext *C);

void GEOMETRY_EDIT_OT_select_all(wmOperatorType *ot);
void GEOMETRY_EDIT_OT_select_mode(wmOperatorType *ot);

}  // namespace blender::ed::geometry_edit
