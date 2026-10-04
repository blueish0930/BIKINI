/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 *
 * Select Elements / Edit Elements: temp mesh + native Edit Mode sessions.
 */

#pragma once

#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"

namespace blender {

struct Mesh;
struct Object;
struct ReportList;
struct bContext;
struct bNodeTree;
struct wmOperatorType;

namespace ed::geometry_edit {

enum class SessionKind : int8_t {
  /** Select Elements: selection tools only, topology/positions fixed. */
  SelectOnly = 0,
  /**
   * Edit Elements: full Mesh Edit Mode (topology, extrude, knife, …).
   * Commit writes the resulting mesh datablock back to the node.
   */
  MeshEdit = 1,
  /** @deprecated Same as #MeshEdit (kept for any leftover call sites). */
  PositionEdit = 1,
};

bool session_begin(bContext *C,
                   bNodeTree *ntree,
                   int32_t node_identifier,
                   Object *host_object,
                   const Mesh &src_mesh,
                   const float4x4 &object_to_world,
                   bke::AttrDomain domain,
                   Span<int> seed_indices,
                   SessionKind kind,
                   ReportList *reports);

bool session_is_active();
SessionKind session_kind();
void session_cancel(bContext *C);

/** Select-only commit: read domain + indices. */
bool session_commit_selection(bContext *C, bke::AttrDomain &r_domain, Vector<int> &r_indices);

/**
 * Full mesh-edit commit: convert the temp object's edit mesh into a new #Mesh in #Main.
 * Caller owns the returned mesh (typically assigns it to the Edit Elements node).
 */
bool session_commit_mesh(bContext *C, Mesh **r_mesh);

/** @deprecated Prefer #session_commit_mesh. Still works for MeshEdit (positions only). */
bool session_commit_positions(bContext *C, Vector<float3> &r_positions);

bke::AttrDomain session_domain();
void session_set_domain(bke::AttrDomain domain);
void session_tag_redraw(bContext *C);

bool session_enforce_constraints(bContext *C);
bool session_topology_ok();
bool session_positions_ok();

void draw_set_gesture_box(const int start[2], const int curr[2]);
void draw_set_gesture_lasso(Span<int2> points);
void draw_set_gesture_circle(const int center[2], float radius);
void draw_clear_gesture();

bool select_pick_xy(bContext *C, const int xy[2], bool extend, bool deselect, bool toggle);
bool select_all(bContext *C, int action);
bool select_box_xy(bContext *C,
                   const int xy_min[2],
                   const int xy_max[2],
                   bool extend,
                   bool deselect);
bool select_circle_xy(
    bContext *C, const int xy[2], float radius_px, bool extend, bool deselect);
bool select_lasso_xy(bContext *C, Span<int2> mcoords_window, bool extend, bool deselect);
bool select_more(bContext *C);
bool select_less(bContext *C);

void operatortypes_geometry_edit();

}  // namespace ed::geometry_edit

}  // namespace blender
