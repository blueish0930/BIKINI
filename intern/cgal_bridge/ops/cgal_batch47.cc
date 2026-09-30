/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 47 (2026-08-20, new only — do not restore deleted ids):
 *  - Cone Slice (mesh ∩ finite cone side via PMP::surface_intersection)
 *  - Clip Box (PMP::clip by Iso_cuboid_3, keep inside AABB)
 *  - Clip by Mesh (PMP::clip by a closed clipper mesh)
 *
 * Y-Monotone Partition 2D and Polyline Simplify 2D reuse batch 28 intern.
 *
 * Skipped: OpenGR, Kinetic EPECK, Mesh_3 volume, PolyFit SCIP, Ridges_3,
 * Classification, OSQP, Hyperbolic Delaunay, weighted Delaunay / Lloyd,
 * banned measure/query/remesh, previously deleted nodes (Split by Mesh,
 * Keep Largest, Remesh Planar, Clip-as-restore, etc.).
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/intersection_polylines.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/number_utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Iso_cuboid_3 = Kernel::Iso_cuboid_3;
using VI = Surface_mesh::Vertex_index;

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  PMP::triangulate_faces(sm);
  sm.collect_garbage();
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

static void pack_polylines(const std::vector<std::vector<Point_3>> &polys,
                           std::vector<std::vector<float>> &out)
{
  out.clear();
  out.reserve(polys.size());
  for (const auto &poly : polys) {
    if (poly.size() < 2) {
      continue;
    }
    std::vector<float> xyz;
    xyz.reserve(poly.size() * 3);
    for (const Point_3 &p : poly) {
      xyz.push_back(float(CGAL::to_double(p.x())));
      xyz.push_back(float(CGAL::to_double(p.y())));
      xyz.push_back(float(CGAL::to_double(p.z())));
    }
    out.push_back(std::move(xyz));
  }
}

static bool intersect_meshes(Surface_mesh &a,
                             Surface_mesh &b,
                             std::vector<std::vector<float>> &out_polylines,
                             std::string &error)
{
  try {
    std::vector<std::vector<Point_3>> polys;
    PMP::intersection_polylines(a, b, std::back_inserter(polys));
    pack_polylines(polys, out_polylines);
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Surface intersection failed";
    return false;
  }
}

static Vector_3 orthonormal_u(const Vector_3 &axis)
{
  Vector_3 u(1, 0, 0);
  if (std::abs(CGAL::to_double(axis.z())) > 0.9) {
    u = Vector_3(1, 0, 0);
  }
  else {
    u = CGAL::cross_product(axis, Vector_3(0, 0, 1));
  }
  const double ul = std::sqrt(CGAL::to_double(u.squared_length()));
  return u / std::max(ul, 1.0e-12);
}

static void add_cone(Surface_mesh &sm,
                     const Point_3 &apex,
                     Vector_3 axis,
                     double r,
                     double height,
                     int segments)
{
  const double alen = std::sqrt(CGAL::to_double(axis.squared_length()));
  if (alen < 1.0e-12) {
    axis = Vector_3(0, 0, 1);
  }
  else {
    axis = axis / alen;
  }
  r = std::max(r, 1.0e-8);
  height = std::max(height, 1.0e-8);
  const Vector_3 u = orthonormal_u(axis);
  const Vector_3 v = CGAL::cross_product(axis, u);
  const int nseg = std::max(8, segments);
  const Point_3 base_c = apex + axis * height;
  const VI apex_v = sm.add_vertex(apex);
  std::vector<VI> ring;
  ring.resize(size_t(nseg));
  for (int i = 0; i < nseg; i++) {
    const double th = 2.0 * 3.14159265358979323846 * double(i) / double(nseg);
    const Vector_3 off = u * (r * std::cos(th)) + v * (r * std::sin(th));
    ring[size_t(i)] = sm.add_vertex(base_c + off);
  }
  auto tri = [&](VI a, VI b, VI c) { sm.add_face(std::array<VI, 3>{a, b, c}); };
  for (int i = 0; i < nseg; i++) {
    const int j = (i + 1) % nseg;
    tri(apex_v, ring[size_t(i)], ring[size_t(j)]);
  }
}

}  // namespace

bool mesh_cone_slice(const MeshIn &mesh,
                     float ax,
                     float ay,
                     float az,
                     float dx,
                     float dy,
                     float dz,
                     float radius,
                     float height,
                     int segments,
                     std::vector<std::vector<float>> &out_polylines,
                     std::string &error)
{
  out_polylines.clear();
  CgalThrowGuard guard;
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    error = error.empty() ? "Cone Slice needs a mesh with faces" : error;
    return false;
  }
  Surface_mesh cone;
  add_cone(cone,
           Point_3(ax, ay, az),
           Vector_3(dx, dy, dz),
           double(std::max(radius, 1.0e-6f)),
           double(std::max(height, 1.0e-6f)),
           segments);
  PMP::triangulate_faces(cone);
  return intersect_meshes(sm, cone, out_polylines, error);
}

MeshResult mesh_clip_box(const MeshIn &mesh,
                         float cx,
                         float cy,
                         float cz,
                         float sx,
                         float sy,
                         float sz,
                         bool clip_volume)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    result.error = error.empty() ? "Clip Box needs a mesh with faces" : error;
    return result;
  }
  triangulate_faces_keep_ids(sm);
  const double hx = std::max(double(std::abs(sx)) * 0.5, 1.0e-8);
  const double hy = std::max(double(std::abs(sy)) * 0.5, 1.0e-8);
  const double hz = std::max(double(std::abs(sz)) * 0.5, 1.0e-8);
  try {
    Iso_cuboid_3 box(Point_3(double(cx) - hx, double(cy) - hy, double(cz) - hz),
                     Point_3(double(cx) + hx, double(cy) + hy, double(cz) + hz));
    const bool ok = PMP::clip(sm, box, CGAL::parameters::clip_volume(clip_volume));
    if (!ok) {
      result.error = "Clip Box failed (self-intersection or empty clip)";
      return result;
    }
    fill_missing_vert_maps_from_edges(sm);
    sm.collect_garbage();
    result = surface_mesh_to_result(sm);
    if (!result.ok) {
      result.error = "Clip Box produced an empty mesh";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Clip Box failed";
  }
  return result;
}

MeshResult mesh_clip_by_mesh(const MeshIn &mesh, const MeshIn &clipper, bool clip_volume)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  Surface_mesh cut;
  std::string error;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    result.error = error.empty() ? "Clip by Mesh needs a mesh with faces" : error;
    return result;
  }
  error.clear();
  if (!mesh_in_to_surface_mesh(clipper, cut, error, true)) {
    result.error = error.empty() ? "Clip by Mesh needs a closed clipper mesh" : error;
    return result;
  }
  triangulate_faces_keep_ids(sm);
  PMP::triangulate_faces(cut);
  cut.collect_garbage();
  if (!CGAL::is_closed(cut)) {
    result.error = "Clip by Mesh: clipper must be a closed volume";
    return result;
  }
  try {
    /* Copy clipper so PMP can mutate it; do_not_modify forces clip_volume off. */
    Surface_mesh cut_work = cut;
    const bool ok = PMP::clip(sm, cut_work, CGAL::parameters::clip_volume(clip_volume));
    if (!ok) {
      result.error = "Clip by Mesh failed (self-intersection or empty clip)";
      return result;
    }
    fill_missing_vert_maps_from_edges(sm);
    sm.collect_garbage();
    result = surface_mesh_to_result(sm);
    if (!result.ok) {
      result.error = "Clip by Mesh produced an empty mesh";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Clip by Mesh failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
