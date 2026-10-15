/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgeometry_edit
 *
 * Temp object + native Edit Mode for Select Elements / Edit Elements.
 */

#pragma once

#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_geometry_set.hh"

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
  /** Component the edited data stands for, instances are edited as a point cloud. */
  bke::GeometryComponent::Type component_type = bke::GeometryComponent::Type::Mesh;

  bNodeTree *ntree = nullptr;
  int32_t node_identifier = 0;

  Object *host_object = nullptr;
  eObject_VisibilityFlag prev_visibility_flag = eObject_VisibilityFlag(0);
  bool prev_base_selected = false;
  Object *prev_active_object = nullptr;

  /** Mesh, curves or point cloud object that is in Edit Mode. */
  Object *temp_object = nullptr;
  /** Data of #temp_object. */
  ID *temp_data = nullptr;
  /** Wireframe of the rest of the geometry, may be null. */
  Object *reference_object = nullptr;

  /**
   * Whole curves or instances are selected as one object each, in Object Mode. The objects are
   * referred to by #ID.session_uid, because they may be gone or reallocated when the session
   * ends: Object Mode allows deleting them and global undo.
   */
  bool elements_as_objects = false;
  /** Object of every element, by element index. */
  Vector<uint> element_object_uids;
  /** Collection that holds the objects. */
  uint elements_collection_uid = 0;

  float4x4 object_to_world = float4x4::identity();

  bke::AttrDomain domain = bke::AttrDomain::Point;

  /** Vertices, control points or points at session start. */
  int seed_verts = 0;
  int seed_edges = 0;
  int seed_faces = 0;
  int seed_curves = 0;
  /** Local-space rest positions at session start (select-only: must not change). */
  Vector<float3> seed_positions;
  /**
   * Curves or point cloud at session start, including the selection. A select-only session
   * goes back to it when the geometry was changed anyway.
   */
  bke::GeometrySet seed_geometry;

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

bool session_topology_ok();
bool session_positions_ok();

/** Enforce mode constraints (undo illegal edits). */
bool session_enforce_constraints(bContext *C);

void GEOMETRY_EDIT_OT_select_all(wmOperatorType *ot);
void GEOMETRY_EDIT_OT_select_mode(wmOperatorType *ot);

}  // namespace blender::ed::geometry_edit
