/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 24:
 *  - M25 refine_mesh_at_isolevel
 *  - M64 exact multi-source geodesic distances (Surface_mesh_shortest_path)
 *  - G04 Skin Surface reconstruction from weighted balls
 */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/refine_mesh_at_isolevel.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/make_skin_surface_mesh_3.h>

#include <algorithm>
#include <cmath>
#include <list>
#include <map>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point = Point_3;
using Weighted_point = Kernel::Weighted_point_3;
using Polyhedron = CGAL::Polyhedron_3<Kernel>;

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  PMP::triangulate_faces(sm);
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

/** Map original vertex index (0..verts_num-1) to current Surface_mesh vertex. */
static Surface_mesh::Vertex_index vertex_from_orig_index(const Surface_mesh &sm, int orig_i)
{
  /* With mesh_in_to_surface_mesh, vertex indices usually match until removals. */
  if (orig_i >= 0 && orig_i < int(sm.number_of_vertices())) {
    Surface_mesh::Vertex_index v(orig_i);
    if (sm.is_valid(v) && !sm.is_removed(v)) {
      return v;
    }
  }
  int i = 0;
  for (const auto v : sm.vertices()) {
    if (sm.is_removed(v)) {
      continue;
    }
    if (i == orig_i) {
      return v;
    }
    i++;
  }
  return Surface_mesh::null_vertex();
}

}  // namespace

MeshResult mesh_refine_at_isolevel(const MeshIn &mesh,
                                   const float *vertex_values,
                                   double isovalue)
{
  MeshResult result;
  if (!vertex_values || mesh.verts_num <= 0) {
    result.error = "Refine at Isolevel needs per-vertex values";
    return result;
  }
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Empty mesh" : error;
    return result;
  }
  try {
    /* Vertex property map for scalar function. */
    auto vmap = sm.add_property_map<Surface_mesh::Vertex_index, double>("v:iso", 0.0).first;
    int i = 0;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      if (i < mesh.verts_num) {
        vmap[v] = double(vertex_values[i]);
      }
      i++;
    }

    auto econst = sm.add_property_map<Surface_mesh::Edge_index, bool>("e:iso", false).first;
    PMP::refine_mesh_at_isolevel(
        sm, vmap, isovalue, CGAL::parameters::edge_is_constrained_map(econst));

    result = surface_mesh_to_result(sm);
    if (!result.ok) {
      if (result.error.empty()) {
        result.error = "Refine at Isolevel produced empty mesh";
      }
      return result;
    }

    /* Export isoline edges as seam pairs (for Edge selection). */
    std::map<Surface_mesh::Vertex_index, int> v2i;
    int vi = 0;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      v2i[v] = vi++;
    }
    for (const auto e : sm.edges()) {
      if (sm.is_removed(e) || !econst[e]) {
        continue;
      }
      const auto v0 = sm.vertex(e, 0);
      const auto v1 = sm.vertex(e, 1);
      auto it0 = v2i.find(v0);
      auto it1 = v2i.find(v1);
      if (it0 == v2i.end() || it1 == v2i.end()) {
        continue;
      }
      result.seam_vert_a.push_back(it0->second);
      result.seam_vert_b.push_back(it1->second);
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "Refine at Isolevel failed";
    result.ok = false;
  }
  return result;
}

bool mesh_exact_geodesic_distances(const MeshIn &mesh,
                                   const std::vector<int> &source_verts,
                                   std::vector<float> &out_dist,
                                   std::string &error)
{
  out_dist.clear();
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  if (source_verts.empty()) {
    error = "Need at least one source vertex";
    return false;
  }
  try {
    using Traits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Surface_mesh>;
    using Shortest_paths = CGAL::Surface_mesh_shortest_path<Traits>;
    Shortest_paths sp(sm);

    int n_src = 0;
    for (const int vi : source_verts) {
      const auto v = vertex_from_orig_index(sm, vi);
      if (v == Surface_mesh::null_vertex()) {
        continue;
      }
      sp.add_source_point(v);
      n_src++;
    }
    if (n_src == 0) {
      error = "No valid source vertices on mesh";
      return false;
    }

    /* Distance per original vertex index (0..verts_num-1). */
    out_dist.assign(size_t(mesh.verts_num), -1.0f);
    int i = 0;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      if (i >= mesh.verts_num) {
        break;
      }
      const auto res = sp.shortest_distance_to_source_points(v);
      const double d = CGAL::to_double(res.first);
      out_dist[size_t(i)] = (d < 0.0) ? -1.0f : float(d);
      i++;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Exact geodesic distances failed";
    return false;
  }
}

MeshResult points_skin_surface(const float *positions,
                               const float *radii,
                               int n,
                               double shrink_factor,
                               int subdivisions)
{
  MeshResult result;
  if (!positions || n < 2) {
    result.error = "Skin Surface needs at least 2 points";
    return result;
  }
  try {
    std::list<Weighted_point> balls;
    for (int i = 0; i < n; i++) {
      const float r = radii ? radii[i] : 1.0f;
      const double rr = double(std::max(r, 1e-6f));
      /* CGAL Weighted_point weight is typically squared radius. */
      balls.emplace_back(Point(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]),
                         rr * rr);
    }
    const double shrink = std::clamp(shrink_factor, 0.01, 0.999);
    const int subdiv = std::clamp(subdivisions, 0, 4);

    Polyhedron poly;
    CGAL::make_skin_surface_mesh_3(poly, balls.begin(), balls.end(), shrink, subdiv, true);
    if (poly.empty()) {
      result.error = "Skin Surface produced empty mesh (try larger radii or more points)";
      return result;
    }

    Surface_mesh sm;
    CGAL::copy_face_graph(poly, sm);
    if (sm.number_of_faces() == 0) {
      result.error = "Skin Surface conversion failed";
      return result;
    }
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "Skin Surface reconstruction failed";
    result.ok = false;
  }
  return result;
}

}  // namespace blender::cgal_bridge
