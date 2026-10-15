/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup editors
 *
 * Select Elements / Edit Elements: temp object + native Edit Mode sessions.
 */

#pragma once

#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_geometry_set.hh"

namespace blender {

namespace bke {
class Instances;
}

struct Curves;
struct ID;
struct Mesh;
struct Object;
struct PointCloud;
struct ReportList;
struct bContext;
struct bNodeTree;
struct wmOperatorType;

namespace ed::geometry_edit {

enum class SessionKind : int8_t {
  /** Select Elements: selection tools only, topology/positions fixed. */
  SelectOnly = 0,
  /**
   * Edit Elements: full Edit Mode (topology, extrude, knife, …).
   * Commit writes the resulting data-block back to the node.
   */
  MeshEdit = 1,
  /** @deprecated Same as #MeshEdit (kept for any leftover call sites). */
  PositionEdit = 1,
};

/**
 * Whole curves and instances are selected as one object each. With more elements than this the
 * objects would take too long to make, then points are selected instead.
 */
constexpr int session_max_element_objects = 20000;

struct SessionParams {
  bNodeTree *ntree = nullptr;
  int32_t node_identifier = 0;
  /** Object with the Geometry Nodes modifier, hidden while the session runs. */
  Object *host_object = nullptr;
  float4x4 object_to_world = float4x4::identity();
  SessionKind kind = SessionKind::SelectOnly;

  /**
   * Component that is edited: Mesh, Curve, PointCloud or Instance. Instances have no edit mode,
   * they are edited as #pointcloud with one point per instance.
   */
  bke::GeometryComponent::Type component_type = bke::GeometryComponent::Type::Mesh;
  /** The data of #component_type. It is copied, the caller keeps ownership. */
  const Mesh *mesh = nullptr;
  const Curves *curves = nullptr;
  const PointCloud *pointcloud = nullptr;
  /** Shown as wireframe that cannot be selected, to see the rest of the geometry. Optional. */
  const Mesh *reference_mesh = nullptr;

  /**
   * Select only, on whole curves or on instances: every curve of #curves or every instance of
   * #instances becomes a temporary object, and the objects are selected in Object Mode.
   */
  bool elements_as_objects = false;
  const bke::Instances *instances = nullptr;

  /** Select only: the domain to select on, and the selection to start with on its own domain. */
  bke::AttrDomain domain = bke::AttrDomain::Point;
  bke::AttrDomain seed_domain = bke::AttrDomain::Point;
  Span<int> seed_indices;
};

bool session_begin(bContext *C, const SessionParams &params, ReportList *reports);

bool session_is_active();
SessionKind session_kind();
bke::GeometryComponent::Type session_component_type();
/** True while the elements are selected as objects in Object Mode. */
bool session_selects_objects();
void session_cancel(bContext *C);

/** Select-only commit: read domain + indices. */
bool session_commit_selection(bContext *C, bke::AttrDomain &r_domain, Vector<int> &r_indices);

/**
 * Full edit commit: the edited data as a #Mesh, #Curves or #PointCloud in #Main.
 * The caller owns the user of the returned data-block (typically hands it to the Edit Elements
 * node and releases its own).
 */
bool session_commit_result(bContext *C, ID **r_result);

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
