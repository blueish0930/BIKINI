/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/manifoldness.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

using K = Kernel;
using Point = Point_3;

static bool load_any(const MeshIn &mesh, Surface_mesh &sm, std::string &error, bool stamp = true)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, stamp)) {
    return false;
  }
  return sm.number_of_faces() > 0;
}

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!load_any(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  return sm.number_of_faces() > 0;
}

static int vert_src_index(const Surface_mesh &sm, Surface_mesh::Vertex_index v, int fallback)
{
  auto vsrc = sm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
  if (vsrc.has_value()) {
    const int s = (*vsrc)[v];
    if (s >= 0) {
      return s;
    }
  }
  return fallback;
}

static uint64_t edge_key(int a, int b)
{
  const int lo = std::min(a, b);
  const int hi = std::max(a, b);
  return (uint64_t(uint32_t(lo)) << 32) | uint64_t(uint32_t(hi));
}

bool mesh_is_outward_oriented(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  if (!CGAL::is_closed(sm)) {
    error = "Is Outward Oriented needs a closed mesh";
    return false;
  }
  try {
    return PMP::is_outward_oriented(sm);
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "is_outward_oriented failed";
    return false;
  }
}

MeshResult mesh_reverse_orientation(const MeshIn &mesh)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, true)) {
    result.error = error.empty() ? "Empty mesh" : error;
    return result;
  }
  try {
    PMP::reverse_face_orientations(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "reverse_face_orientations failed";
  }
  return result;
}

int mesh_hole_count(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_any(mesh, sm, error, false)) {
    return 0;
  }
  try {
    std::vector<Surface_mesh::Halfedge_index> cycles;
    CGAL::extract_boundary_cycles(sm, std::back_inserter(cycles));
    return int(cycles.size());
  }
  catch (const std::exception &e) {
    error = e.what();
    return 0;
  }
  catch (...) {
    error = "hole count failed";
    return 0;
  }
}

bool mesh_border_edge_flags(const MeshIn &mesh,
                            const int *edge_v0,
                            const int *edge_v1,
                            int edges_num,
                            std::vector<int8_t> &out_border,
                            std::string &error)
{
  if (!edge_v0 || !edge_v1 || edges_num < 0) {
    error = "Invalid edge buffers";
    return false;
  }
  Surface_mesh sm;
  if (!load_any(mesh, sm, error, true)) {
    return false;
  }
  try {
    out_border.assign(size_t(edges_num), 0);
    std::map<uint64_t, int> key_to_edge;
    for (int e = 0; e < edges_num; e++) {
      key_to_edge[edge_key(edge_v0[e], edge_v1[e])] = e;
    }

    int vi = 0;
    std::map<Surface_mesh::Vertex_index, int> v_to_src;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      v_to_src[v] = vert_src_index(sm, v, vi);
      vi++;
    }

    for (const auto e : sm.edges()) {
      if (sm.is_removed(e)) {
        continue;
      }
      const auto h = sm.halfedge(e);
      if (!sm.is_border(h) && !sm.is_border(sm.opposite(h))) {
        continue;
      }
      const int a = v_to_src[sm.source(h)];
      const int b = v_to_src[sm.target(h)];
      const auto it = key_to_edge.find(edge_key(a, b));
      if (it != key_to_edge.end()) {
        out_border[size_t(it->second)] = 1;
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "border edge flags failed";
    return false;
  }
}

bool mesh_dihedral_angles(const MeshIn &mesh,
                          const int *edge_v0,
                          const int *edge_v1,
                          int edges_num,
                          std::vector<float> &out_angle_rad,
                          std::string &error)
{
  if (!edge_v0 || !edge_v1 || edges_num < 0) {
    error = "Invalid edge buffers";
    return false;
  }
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  try {
    out_angle_rad.assign(size_t(edges_num), 0.0f);
    std::map<uint64_t, int> key_to_edge;
    for (int e = 0; e < edges_num; e++) {
      key_to_edge[edge_key(edge_v0[e], edge_v1[e])] = e;
    }

    int vi = 0;
    std::map<Surface_mesh::Vertex_index, int> v_to_src;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      v_to_src[v] = vert_src_index(sm, v, vi);
      vi++;
    }

    auto face_third = [&](Surface_mesh::Halfedge_index h) -> Surface_mesh::Vertex_index {
      /* Triangle: next next of h is the opposite vertex from edge h. */
      return sm.target(sm.next(h));
    };

    for (const auto e : sm.edges()) {
      if (sm.is_removed(e)) {
        continue;
      }
      const auto h0 = sm.halfedge(e);
      const auto h1 = sm.opposite(h0);
      if (sm.is_border(h0) || sm.is_border(h1)) {
        continue;
      }
      const Point p = sm.point(sm.source(h0));
      const Point q = sm.point(sm.target(h0));
      const Point r = sm.point(face_third(h0));
      const Point s = sm.point(face_third(h1));
      /* Degrees in (-180, 180]; convert to radians for Blender angle sockets. */
      const double deg = CGAL::to_double(CGAL::approximate_dihedral_angle(p, q, r, s));
      const float rad = float(deg * (3.14159265358979323846 / 180.0));

      const int a = v_to_src[sm.source(h0)];
      const int b = v_to_src[sm.target(h0)];
      const auto it = key_to_edge.find(edge_key(a, b));
      if (it != key_to_edge.end()) {
        /* Keep the max absolute dihedral when multiple tris map to one edge. */
        if (std::abs(rad) >= std::abs(out_angle_rad[size_t(it->second)])) {
          out_angle_rad[size_t(it->second)] = rad;
        }
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "dihedral angles failed";
    return false;
  }
}

MeshResult mesh_duplicate_non_manifold_vertices(const MeshIn &mesh)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, true)) {
    result.error = error.empty() ? "Empty mesh" : error;
    return result;
  }
  try {
    PMP::duplicate_non_manifold_vertices(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "duplicate_non_manifold_vertices failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
