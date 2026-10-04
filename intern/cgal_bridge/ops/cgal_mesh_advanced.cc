/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Approximate_min_ellipsoid_d.h>
#include <CGAL/Approximate_min_ellipsoid_d_traits_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Heat_method_3/Surface_mesh_geodesic_distances_3.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/extract_mean_curvature_flow_skeleton.h>
#include <CGAL/mesh_segmentation.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

using K = Kernel;
using Point = Point_3;

static bool load_tri_closed(const MeshIn &mesh, Surface_mesh &sm, std::string &error, bool require_closed)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  if (sm.number_of_faces() == 0) {
    error = "Empty mesh after triangulation";
    return false;
  }
  if (require_closed && !CGAL::is_closed(sm)) {
    error = "Mesh must be closed (no boundary)";
    return false;
  }
  return true;
}

WireResult mesh_skeleton(const MeshIn &mesh)
{
  WireResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_tri_closed(mesh, sm, error, true)) {
    result.error = error.empty() ? "Skeleton needs a closed triangle mesh" : error;
    return result;
  }
  try {
    using Skel = CGAL::Mean_curvature_flow_skeletonization<Surface_mesh>::Skeleton;
    using vertex_descriptor = boost::graph_traits<Skel>::vertex_descriptor;
    using edge_descriptor = boost::graph_traits<Skel>::edge_descriptor;
    Skel skeleton;
    CGAL::extract_mean_curvature_flow_skeleton(sm, skeleton);

    std::map<vertex_descriptor, int> v2i;
    for (vertex_descriptor v : CGAL::make_range(vertices(skeleton))) {
      const Point &p = skeleton[v].point;
      v2i[v] = int(result.positions.size() / 3);
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
    }
    for (edge_descriptor e : CGAL::make_range(edges(skeleton))) {
      const auto s = source(e, skeleton);
      const auto t = target(e, skeleton);
      result.edge_v0.push_back(v2i[s]);
      result.edge_v1.push_back(v2i[t]);
    }
    if (result.positions.empty() || result.edge_v0.empty()) {
      result.error = "Skeleton extraction produced empty curve";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = std::string("Skeleton failed: ") + e.what();
  }
  catch (...) {
    result.error = "CGAL mean curvature flow skeleton failed";
  }
  return result;
}

WireResult mesh_extract_border(const MeshIn &mesh)
{
  WireResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_tri_closed(mesh, sm, error, false)) {
    result.error = error.empty() ? "Border extract input failed" : error;
    return result;
  }
  try {
    std::map<Surface_mesh::Vertex_index, int> v2i;
    auto ensure_v = [&](Surface_mesh::Vertex_index v) -> int {
      auto it = v2i.find(v);
      if (it != v2i.end()) {
        return it->second;
      }
      const Point &p = sm.point(v);
      const int idx = int(result.positions.size() / 3);
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
      v2i[v] = idx;
      return idx;
    };

    std::vector<Surface_mesh::Halfedge_index> borders;
    CGAL::border_halfedges(sm, std::back_inserter(borders));
    for (const auto h : borders) {
      if (!sm.is_border(h)) {
        continue;
      }
      const auto va = sm.source(h);
      const auto vb = sm.target(h);
      result.edge_v0.push_back(ensure_v(va));
      result.edge_v1.push_back(ensure_v(vb));
    }
    if (result.edge_v0.empty()) {
      result.error = "Mesh has no border edges (closed?)";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Border extract failed";
  }
  return result;
}

bool mesh_geodesic_distances(const MeshIn &mesh,
                             const std::vector<int> &source_verts,
                             std::vector<float> &out_dist,
                             std::string &error)
{
  Surface_mesh sm;
  if (!load_tri_closed(mesh, sm, error, false)) {
    return false;
  }
  if (source_verts.empty()) {
    error = "Need at least one source vertex";
    return false;
  }
  try {
    std::vector<Surface_mesh::Vertex_index> sources;
    sources.reserve(source_verts.size());
    for (const int vi : source_verts) {
      if (vi < 0 || vi >= int(sm.number_of_vertices())) {
        continue;
      }
      /* Vertex indices from MeshIn match Surface_mesh order when no vertices removed. */
      Surface_mesh::Vertex_index v(vi);
      if (sm.is_removed(v) || !sm.is_valid(v)) {
        /* Fall back to sequential non-removed enumeration. */
        int i = 0;
        bool found = false;
        for (const auto vv : sm.vertices()) {
          if (sm.is_removed(vv)) {
            continue;
          }
          if (i == vi) {
            sources.push_back(vv);
            found = true;
            break;
          }
          i++;
        }
        if (!found) {
          continue;
        }
      }
      else {
        sources.push_back(v);
      }
    }
    if (sources.empty()) {
      error = "No valid source vertices";
      return false;
    }

    auto dist_map = sm.add_property_map<Surface_mesh::Vertex_index, double>("v:geo_dist", 0.0).first;
    CGAL::Heat_method_3::estimate_geodesic_distances(sm, dist_map, sources);

    out_dist.assign(size_t(mesh.verts_num), 0.0f);
    int i = 0;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      if (i < mesh.verts_num) {
        out_dist[size_t(i)] = float(dist_map[v]);
      }
      i++;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL heat method geodesic failed";
    return false;
  }
}

MeshResult mesh_segmentation(const MeshIn &mesh, int n_clusters, double smoothing_lambda)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_tri_closed(mesh, sm, error, true)) {
    result.error = error.empty() ? "Segmentation needs closed triangle mesh" : error;
    return result;
  }
  try {
    n_clusters = std::max(n_clusters, 2);
    smoothing_lambda = std::clamp(smoothing_lambda, 0.0, 1.0);

    auto sdf = sm.add_property_map<Surface_mesh::Face_index, double>("f:sdf", 0.0).first;
    auto seg = sm.add_property_map<Surface_mesh::Face_index, std::size_t>("f:seg", 0).first;
    CGAL::sdf_values(sm, sdf);
    CGAL::segmentation_from_sdf_values(sm, sdf, seg, std::size_t(n_clusters), smoothing_lambda);

    result = surface_mesh_to_result(sm);
    if (!result.ok) {
      return result;
    }
    /* Keep face_src / vert maps from surface_mesh_to_result for attribute transfer.
     * Store segment id in face_tag and SDF in face_scalar (same face iteration order). */
    result.face_tag.clear();
    result.face_scalar.clear();
    result.face_tag.reserve(size_t(result.faces_num()));
    result.face_scalar.reserve(size_t(result.faces_num()));
    for (const auto f : sm.faces()) {
      if (sm.is_removed(f)) {
        continue;
      }
      result.face_tag.push_back(int(seg[f]));
      result.face_scalar.push_back(float(sdf[f]));
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL mesh segmentation failed";
  }
  return result;
}

MeshResult min_ellipsoid(const float *positions, int points_num, int sphere_segments)
{
  MeshResult result;
  if (!positions || points_num < 4) {
    result.error = "Min Ellipsoid needs at least 4 points";
    return result;
  }
  try {
    typedef CGAL::Approximate_min_ellipsoid_d_traits_3<K, double> Traits;
    typedef CGAL::Approximate_min_ellipsoid_d<Traits> Min_ell;
    std::vector<Point> pts;
    pts.reserve(size_t(points_num));
    for (int i = 0; i < points_num; i++) {
      pts.emplace_back(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);
    }
    Traits traits;
    /*
     * Smaller eps => tighter (1+r)-approx of the true MEL.
     * Axis export is floating-point only; we re-pair / re-scale below so every
     * input point (and the discrete UV-sphere mesh) hard-encloses the set.
     */
    Min_ell ell(0.001, pts.begin(), pts.end(), traits);
    if (ell.is_empty() || !ell.is_full_dimensional()) {
      result.error = "Points are empty/degenerate for min ellipsoid";
      return result;
    }

    double center[3] = {0, 0, 0};
    int c = 0;
    for (auto it = ell.center_cartesian_begin(); it != ell.center_cartesian_end() && c < 3;
         ++it, ++c)
    {
      center[c] = *it;
    }
    double lengths[3] = {1, 1, 1};
    c = 0;
    for (auto it = ell.axes_lengths_begin(); it != ell.axes_lengths_end() && c < 3; ++it, ++c) {
      lengths[c] = std::max(*it, 1e-12);
    }
    double dirs[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int axis = 0; axis < 3; axis++) {
      int k = 0;
      for (auto it = ell.axis_direction_cartesian_begin(axis);
           it != ell.axis_direction_cartesian_end(axis) && k < 3;
           ++it, ++k)
      {
        dirs[axis][k] = *it;
      }
    }

    /*
     * CGAL 3D compute_axes_3 pairs directions and lengths incorrectly:
     *   lengths[i]  <-> eigenvalue[i]  (ascending λ → descending semi-axis)
     *   directions[0] <-> largest λ  (shortest axis)
     *   directions[2] <-> smallest λ (longest axis)
     * 2D path reverses lengths; 3D does not. Swap dir 0 and 2 so dir[i]
     * matches lengths[i] (docs: lengths sorted descending by axis length).
     */
    std::swap(dirs[0][0], dirs[2][0]);
    std::swap(dirs[0][1], dirs[2][1]);
    std::swap(dirs[0][2], dirs[2][2]);

    /* Normalize + Gram-Schmidt so dirs form an orthonormal right-handed basis. */
    auto normalize3 = [](double v[3]) {
      const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
      if (n > 1e-18) {
        v[0] /= n;
        v[1] /= n;
        v[2] /= n;
      }
    };
    auto dot3 = [](const double a[3], const double b[3]) {
      return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
    normalize3(dirs[0]);
    {
      const double p = dot3(dirs[1], dirs[0]);
      dirs[1][0] -= p * dirs[0][0];
      dirs[1][1] -= p * dirs[0][1];
      dirs[1][2] -= p * dirs[0][2];
      normalize3(dirs[1]);
    }
    /* d2 = d0 × d1 for a right-handed frame (drop CGAL's third dir). */
    dirs[2][0] = dirs[0][1] * dirs[1][2] - dirs[0][2] * dirs[1][1];
    dirs[2][1] = dirs[0][2] * dirs[1][0] - dirs[0][0] * dirs[1][2];
    dirs[2][2] = dirs[0][0] * dirs[1][1] - dirs[0][1] * dirs[1][0];
    normalize3(dirs[2]);

    /*
     * Hard enclosure: Mahalanobis radius of every input point must be <= 1.
     *   local_i = (p - c) · dir_i
     *   r2 = sum_i (local_i / length_i)^2
     */
    double max_r2 = 0.0;
    for (int i = 0; i < points_num; i++) {
      const double dx = double(positions[i * 3 + 0]) - center[0];
      const double dy = double(positions[i * 3 + 1]) - center[1];
      const double dz = double(positions[i * 3 + 2]) - center[2];
      double r2 = 0.0;
      for (int a = 0; a < 3; a++) {
        const double local = dx * dirs[a][0] + dy * dirs[a][1] + dz * dirs[a][2];
        const double t = local / lengths[a];
        r2 += t * t;
      }
      max_r2 = std::max(max_r2, r2);
    }

    sphere_segments = std::clamp(sphere_segments, 8, 64);
    const int rings = std::max(sphere_segments / 2, 4);
    /*
     * Expand only as much as needed so every input point lies inside the
     * continuous ellipsoid, plus a tiny float margin. Avoid large faceting
     * inflation (that made the mesh bigger than a bounding sphere).
     * Chord error of the UV sphere is small at typical Segments (>=16).
     */
    const double enclose = (max_r2 > 1.0) ? std::sqrt(max_r2) : 1.0;
    const double scale = enclose * 1.001;
    for (int a = 0; a < 3; a++) {
      lengths[a] *= scale;
    }

    /* UV sphere in unit ball → center + sum u_i * length_i * dir_i */
    std::vector<float> unit_pos;
    std::vector<int> corners;
    unit_pos.push_back(0);
    unit_pos.push_back(0);
    unit_pos.push_back(1);
    for (int i = 1; i < rings; i++) {
      const float v = float(M_PI) * float(i) / float(rings);
      const float y = std::cos(v);
      const float r = std::sin(v);
      for (int j = 0; j < sphere_segments; j++) {
        const float u = 2.0f * float(M_PI) * float(j) / float(sphere_segments);
        unit_pos.push_back(r * std::cos(u));
        unit_pos.push_back(r * std::sin(u));
        unit_pos.push_back(y);
      }
    }
    unit_pos.push_back(0);
    unit_pos.push_back(0);
    unit_pos.push_back(-1);
    const int south = int(unit_pos.size() / 3) - 1;
    for (int j = 0; j < sphere_segments; j++) {
      corners.push_back(0);
      corners.push_back(1 + j);
      corners.push_back(1 + (j + 1) % sphere_segments);
    }
    for (int i = 0; i < rings - 2; i++) {
      const int row0 = 1 + i * sphere_segments;
      const int row1 = 1 + (i + 1) * sphere_segments;
      for (int j = 0; j < sphere_segments; j++) {
        const int j1 = (j + 1) % sphere_segments;
        corners.push_back(row0 + j);
        corners.push_back(row1 + j);
        corners.push_back(row1 + j1);
        corners.push_back(row0 + j);
        corners.push_back(row1 + j1);
        corners.push_back(row0 + j1);
      }
    }
    const int last_ring = 1 + (rings - 2) * sphere_segments;
    for (int j = 0; j < sphere_segments; j++) {
      corners.push_back(south);
      corners.push_back(last_ring + (j + 1) % sphere_segments);
      corners.push_back(last_ring + j);
    }

    const int vn = int(unit_pos.size() / 3);
    result.positions.resize(size_t(vn) * 3);
    for (int i = 0; i < vn; i++) {
      const double ux = unit_pos[size_t(i) * 3 + 0];
      const double uy = unit_pos[size_t(i) * 3 + 1];
      const double uz = unit_pos[size_t(i) * 3 + 2];
      /* p = c + R * diag(L) * u , R columns = dirs[0..2] */
      const double x = center[0] + lengths[0] * ux * dirs[0][0] + lengths[1] * uy * dirs[1][0] +
                       lengths[2] * uz * dirs[2][0];
      const double y = center[1] + lengths[0] * ux * dirs[0][1] + lengths[1] * uy * dirs[1][1] +
                       lengths[2] * uz * dirs[2][1];
      const double z = center[2] + lengths[0] * ux * dirs[0][2] + lengths[1] * uy * dirs[1][2] +
                       lengths[2] * uz * dirs[2][2];
      result.positions[size_t(i) * 3 + 0] = float(x);
      result.positions[size_t(i) * 3 + 1] = float(y);
      result.positions[size_t(i) * 3 + 2] = float(z);
    }
    result.corner_verts = std::move(corners);
    result.center[0] = float(center[0]);
    result.center[1] = float(center[1]);
    result.center[2] = float(center[2]);
    result.radius = float(std::max({lengths[0], lengths[1], lengths[2]}));
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL Approximate_min_ellipsoid failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
