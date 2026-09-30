/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 44 (2026-08-20, new only — do not restore deleted ids):
 *  - Polar Dual 3D (convex hull polar dual about an interior point)
 *  - Min Annulus 3D (Min_annulus_d spherical shell)
 *  - PCA Ellipsoid (covariance inertia ellipsoid)
 *  - Collapse Short Edges (SMS Edge_length_cost + Edge_length_stop)
 *  - Polyline Line Fitting (PCA line region growing along each spline)
 *  - Snap Meshes (experimental snap_borders between two meshes + stitch)
 *
 * Skipped: OpenGR, Kinetic EPECK, Mesh_3 volume, PolyFit SCIP, Ridges_3,
 * Classification, OSQP, Hyperbolic Delaunay, weighted Delaunay / Lloyd,
 * banned measure/query/remesh, previously deleted nodes.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Min_annulus_d.h>
#include <CGAL/Min_sphere_annulus_d_traits_3.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/internal/Snapping/snap.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/linear_least_squares_fitting_3.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace PMP = CGAL::Polygon_mesh_processing;
namespace SMS = CGAL::Surface_mesh_simplification;

namespace blender::cgal_bridge {
namespace {

using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Line_3 = Kernel::Line_3;
using FT = Kernel::FT;

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  PMP::triangulate_faces(sm);
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

static void append_uv_sphere(float cx,
                             float cy,
                             float cz,
                             float radius,
                             int segments,
                             MeshResult &result)
{
  segments = std::clamp(segments, 8, 64);
  const int rings = std::max(4, segments / 2);
  result.positions.clear();
  result.corner_verts.clear();
  result.face_offsets.clear();

  result.positions.push_back(cx);
  result.positions.push_back(cy);
  result.positions.push_back(cz + radius);
  for (int i = 1; i < rings; i++) {
    const float phi = float(M_PI) * float(i) / float(rings);
    const float y = std::cos(phi);
    const float r = std::sin(phi);
    for (int j = 0; j < segments; j++) {
      const float u = 2.0f * float(M_PI) * float(j) / float(segments);
      result.positions.push_back(cx + radius * r * std::cos(u));
      result.positions.push_back(cy + radius * r * std::sin(u));
      result.positions.push_back(cz + radius * y);
    }
  }
  result.positions.push_back(cx);
  result.positions.push_back(cy);
  result.positions.push_back(cz - radius);

  const int south = int(result.positions.size() / 3) - 1;
  for (int j = 0; j < segments; j++) {
    const int a = 1 + j;
    const int b = 1 + (j + 1) % segments;
    result.corner_verts.push_back(0);
    result.corner_verts.push_back(a);
    result.corner_verts.push_back(b);
  }
  for (int i = 0; i < rings - 2; i++) {
    const int row0 = 1 + i * segments;
    const int row1 = 1 + (i + 1) * segments;
    for (int j = 0; j < segments; j++) {
      const int j1 = (j + 1) % segments;
      result.corner_verts.push_back(row0 + j);
      result.corner_verts.push_back(row1 + j);
      result.corner_verts.push_back(row1 + j1);
      result.corner_verts.push_back(row0 + j);
      result.corner_verts.push_back(row1 + j1);
      result.corner_verts.push_back(row0 + j1);
    }
  }
  const int last_ring = 1 + (rings - 2) * segments;
  for (int j = 0; j < segments; j++) {
    const int a = last_ring + j;
    const int b = last_ring + (j + 1) % segments;
    result.corner_verts.push_back(south);
    result.corner_verts.push_back(b);
    result.corner_verts.push_back(a);
  }
  result.ok = result.corners_num() >= 3;
}

static void append_uv_sphere_onto(float cx,
                                  float cy,
                                  float cz,
                                  float radius,
                                  int segments,
                                  int tag,
                                  MeshResult &result)
{
  MeshResult tmp;
  append_uv_sphere(cx, cy, cz, radius, segments, tmp);
  if (!tmp.ok) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets = {0};
  }
  const int vbase = result.verts_num();
  result.positions.insert(result.positions.end(), tmp.positions.begin(), tmp.positions.end());
  if (tmp.face_offsets.empty()) {
    for (int i = 0; i + 2 < tmp.corners_num(); i += 3) {
      result.corner_verts.push_back(vbase + tmp.corner_verts[size_t(i)]);
      result.corner_verts.push_back(vbase + tmp.corner_verts[size_t(i) + 1]);
      result.corner_verts.push_back(vbase + tmp.corner_verts[size_t(i) + 2]);
      result.face_offsets.push_back(int(result.corner_verts.size()));
      result.face_tag.push_back(tag);
    }
  }
  result.ok = result.faces_num() > 0;
}

}  // namespace

MeshResult points_min_annulus_3(const float *positions, int n, int segments)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 4) {
    result.error = "Min Annulus 3D needs at least 4 points";
    return result;
  }
  try {
    using Traits = CGAL::Min_sphere_annulus_d_traits_3<Kernel>;
    using Min_annulus = CGAL::Min_annulus_d<Traits>;
    std::vector<Point_3> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);
    }
    Min_annulus ma(pts.begin(), pts.end());
    if (ma.is_empty()) {
      result.error = "Min Annulus 3D empty / degenerate";
      return result;
    }
    const Point_3 c = ma.center();
    const double cx = CGAL::to_double(c.x());
    const double cy = CGAL::to_double(c.y());
    const double cz = CGAL::to_double(c.z());
    const double r_out = std::sqrt(std::max(0.0, CGAL::to_double(ma.squared_outer_radius())));
    const double r_in = std::sqrt(std::max(0.0, CGAL::to_double(ma.squared_inner_radius())));
    result.center[0] = float(cx);
    result.center[1] = float(cy);
    result.center[2] = float(cz);
    result.radius = float(r_out);
    result.alpha_used = r_in;
    append_uv_sphere_onto(float(cx), float(cy), float(cz), float(r_out), segments, 0, result);
    if (r_in > 1e-8) {
      append_uv_sphere_onto(float(cx), float(cy), float(cz), float(r_in), segments, 1, result);
    }
    result.ok = result.faces_num() > 0;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Min Annulus 3D failed";
  }
  return result;
}

MeshResult mesh_collapse_short_edges(const MeshIn &mesh, double max_length)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Collapse Short Edges needs a mesh with faces" : error;
    return result;
  }
  try {
    max_length = std::max(0.0, max_length);
    SMS::Edge_length_stop_predicate<double> stop(max_length);
    SMS::edge_collapse(
        sm,
        stop,
        CGAL::parameters::get_cost(SMS::Edge_length_cost<Surface_mesh>()).get_placement(
            SMS::Midpoint_placement<Surface_mesh>()));
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Collapse Short Edges failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
