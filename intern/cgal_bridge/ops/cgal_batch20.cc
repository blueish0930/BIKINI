/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Min_sphere_of_points_d_traits_3.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/estimate_scale.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/property_map.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

using K = Kernel;
using Point = K::Point_3;
using Vector = K::Vector_3;
using Line = K::Line_3;

bool points_local_neighbor_scales(const float *in_xyz,
                                  int n,
                                  std::vector<int> &out_k,
                                  std::string &error)
{
  if (!in_xyz || n < 6) {
    error = "Local Neighbor Scales needs at least 6 points";
    return false;
  }
  try {
    std::vector<Point> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]);
    }
    std::vector<std::size_t> scales;
    scales.reserve(size_t(n));
    CGAL::estimate_local_k_neighbor_scales(pts, pts, std::back_inserter(scales));
    out_k.resize(size_t(n));
    for (int i = 0; i < n; i++) {
      out_k[size_t(i)] = int(std::max<std::size_t>(1, scales[size_t(i)]));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "estimate_local_k_neighbor_scales failed";
    return false;
  }
}

MeshResult mesh_remove_small_components(const MeshIn &mesh, int min_faces)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    result.error = error.empty() ? "Remove Small Components input failed" : error;
    return result;
  }
  try {
    min_faces = std::max(1, min_faces);
    auto fccmap = sm.add_property_map<Surface_mesh::Face_index, std::size_t>("f:cc", 0).first;
    const std::size_t ncc = PMP::connected_components(sm, fccmap);
    std::vector<int> counts(ncc, 0);
    for (const auto f : sm.faces()) {
      if (!sm.is_removed(f)) {
        counts[fccmap[f]]++;
      }
    }
    std::vector<std::size_t> remove_ids;
    for (std::size_t i = 0; i < ncc; i++) {
      if (counts[i] < min_faces) {
        remove_ids.push_back(i);
      }
    }
    if (!remove_ids.empty()) {
      PMP::remove_connected_components(sm, remove_ids, fccmap);
      sm.collect_garbage();
    }
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "remove_connected_components failed";
  }
  return result;
}

bool points_fit_line(const float *in_xyz,
                     int n,
                     float out_center[3],
                     float out_direction[3],
                     float &out_half_extent,
                     std::string &error)
{
  if (!in_xyz || n < 2 || !out_center || !out_direction) {
    error = "Fit Line needs at least 2 points";
    return false;
  }
  try {
    std::vector<Point> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]);
    }
    Line line;
    Point centroid;
    CGAL::linear_least_squares_fitting_3(
        pts.begin(), pts.end(), line, centroid, CGAL::Dimension_tag<0>());
    Vector dir = line.to_vector();
    const double len = std::sqrt(CGAL::to_double(dir.squared_length()));
    if (!(len > 1e-18)) {
      error = "Fit Line produced a degenerate direction";
      return false;
    }
    dir = dir / len;
    out_center[0] = float(CGAL::to_double(centroid.x()));
    out_center[1] = float(CGAL::to_double(centroid.y()));
    out_center[2] = float(CGAL::to_double(centroid.z()));
    out_direction[0] = float(CGAL::to_double(dir.x()));
    out_direction[1] = float(CGAL::to_double(dir.y()));
    out_direction[2] = float(CGAL::to_double(dir.z()));
    double max_t = 0.0;
    for (const Point &p : pts) {
      Vector d(centroid, p);
      const double t = std::abs(CGAL::to_double(d * dir));
      max_t = std::max(max_t, t);
    }
    out_half_extent = float(std::max(max_t, 1e-4));
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "linear_least_squares_fitting_3 (line) failed";
    return false;
  }
}

bool points_diameter(const float *in_xyz, int n, float &out_diameter, std::string &error)
{
  if (!in_xyz || n < 2) {
    error = "Points Diameter needs at least 2 points";
    return false;
  }
  try {
    typedef CGAL::Min_sphere_of_points_d_traits_3<K, double> Traits;
    typedef CGAL::Min_sphere_of_spheres_d<Traits> Min_sphere;
    std::vector<Point> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]);
    }
    Min_sphere ms(pts.begin(), pts.end());
    const double r = CGAL::to_double(ms.radius());
    out_diameter = float(2.0 * r);
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Min_sphere diameter failed";
    return false;
  }
}

MeshResult mesh_orient_polygon_soup(const MeshIn &mesh)
{
  MeshResult result;
  if (mesh.faces_num <= 0 || mesh.verts_num <= 0) {
    result.error = "Orient Polygon Soup needs a mesh";
    return result;
  }
  try {
    std::vector<Point> points;
    points.reserve(size_t(mesh.verts_num));
    for (int i = 0; i < mesh.verts_num; i++) {
      points.emplace_back(mesh.positions[i * 3],
                          mesh.positions[i * 3 + 1],
                          mesh.positions[i * 3 + 2]);
    }
    std::vector<std::vector<std::size_t>> polygons;
    polygons.reserve(size_t(mesh.faces_num));
    if (mesh.face_offsets) {
      for (int f = 0; f < mesh.faces_num; f++) {
        const int a = mesh.face_offsets[f];
        const int b = mesh.face_offsets[f + 1];
        if (b - a < 3) {
          continue;
        }
        std::vector<std::size_t> poly;
        poly.reserve(size_t(b - a));
        for (int c = a; c < b; c++) {
          poly.push_back(std::size_t(mesh.corner_verts[c]));
        }
        polygons.push_back(std::move(poly));
      }
    }
    else {
      for (int f = 0; f < mesh.faces_num; f++) {
        polygons.push_back({std::size_t(mesh.corner_verts[f * 3 + 0]),
                            std::size_t(mesh.corner_verts[f * 3 + 1]),
                            std::size_t(mesh.corner_verts[f * 3 + 2])});
      }
    }
    PMP::orient_polygon_soup(points, polygons);
    if (!PMP::is_polygon_soup_a_polygon_mesh(polygons)) {
      result.error = "Polygon soup is not manifold after orientation";
      return result;
    }
    Surface_mesh sm;
    PMP::polygon_soup_to_polygon_mesh(points, polygons, sm);
    if (sm.is_empty()) {
      result.error = "polygon_soup_to_polygon_mesh produced empty mesh";
      return result;
    }
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "orient_polygon_soup failed";
  }
  return result;
}

bool mesh_self_intersection_count(const MeshIn &mesh, int &out_count, std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  try {
    std::vector<std::pair<Surface_mesh::face_index, Surface_mesh::face_index>> pairs;
    PMP::self_intersections(sm, std::back_inserter(pairs));
    out_count = int(pairs.size());
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "self_intersections failed";
    return false;
  }
}

bool points_local_density(const float *in_xyz,
                          int n,
                          int k,
                          std::vector<float> &out_density,
                          std::string &error)
{
  if (!in_xyz || n < 3) {
    error = "Local Density needs at least 3 points";
    return false;
  }
  k = std::clamp(k, 3, std::max(3, n - 1));
  try {
    using Tree_traits = CGAL::Search_traits_3<K>;
    using Neighbor_search = CGAL::Orthogonal_k_neighbor_search<Tree_traits>;
    using Tree = Neighbor_search::Tree;
    std::vector<Point> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]);
    }
    Tree tree(pts.begin(), pts.end());
    out_density.resize(size_t(n));
    for (int i = 0; i < n; i++) {
      Neighbor_search search(tree, pts[size_t(i)], k + 1);
      double sum = 0.0;
      int count = 0;
      for (auto it = search.begin(); it != search.end(); ++it) {
        const double d2 = CGAL::to_double(it->second);
        if (d2 <= 1e-24) {
          continue; /* self */
        }
        sum += std::sqrt(d2);
        count++;
        if (count >= k) {
          break;
        }
      }
      const double mean = (count > 0) ? (sum / double(count)) : 0.0;
      out_density[size_t(i)] = (mean > 1e-12) ? float(1.0 / mean) : 0.0f;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "local density failed";
    return false;
  }
}

bool mesh_compactness(const MeshIn &mesh, float &out_compactness, std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  try {
    if (!CGAL::is_closed(sm)) {
      error = "Mesh Compactness requires a closed mesh";
      return false;
    }
    const double vol = std::abs(CGAL::to_double(PMP::volume(sm)));
    const double area = CGAL::to_double(PMP::area(sm));
    if (!(area > 1e-18)) {
      error = "Mesh Compactness: surface area is zero";
      return false;
    }
    /* Sphere has compactness 1: 36*pi*V^2 / A^3 */
    const double c = (36.0 * CGAL_PI * vol * vol) / (area * area * area);
    out_compactness = float(c);
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "mesh compactness failed";
    return false;
  }
}

bool mesh_face_component_sizes(const MeshIn &mesh,
                               std::vector<int> &out_size,
                               std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  try {
    auto fccmap = sm.add_property_map<Surface_mesh::Face_index, std::size_t>("f:CC", 0).first;
    const std::size_t ncc = PMP::connected_components(sm, fccmap);
    std::vector<int> counts(ncc, 0);
    for (const auto f : sm.faces()) {
      if (sm.is_removed(f)) {
        continue;
      }
      counts[fccmap[f]]++;
    }
    out_size.assign(size_t(mesh.faces_num), 0);
    auto fsp = sm.property_map<Surface_mesh::Face_index, int>("f:src");
    for (const auto f : sm.faces()) {
      if (sm.is_removed(f)) {
        continue;
      }
      int idx = -1;
      if (fsp.has_value()) {
        idx = (*fsp)[f];
      }
      if (idx >= 0 && idx < mesh.faces_num) {
        out_size[size_t(idx)] = counts[fccmap[f]];
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "connected_components size failed";
    return false;
  }
}

bool mesh_face_planarity(const MeshIn &mesh, std::vector<float> &out_planarity, std::string &error)
{
  if (mesh.faces_num <= 0 || !mesh.positions || !mesh.corner_verts) {
    error = "Face Planarity needs a mesh with faces";
    return false;
  }
  out_planarity.assign(size_t(mesh.faces_num), 0.0f);
  auto face_range = [&](int f, int &start, int &end) {
    if (mesh.face_offsets) {
      start = mesh.face_offsets[f];
      end = mesh.face_offsets[f + 1];
    }
    else {
      start = f * 3;
      end = start + 3;
    }
  };
  for (int f = 0; f < mesh.faces_num; f++) {
    int start = 0, end = 0;
    face_range(f, start, end);
    const int nv = end - start;
    if (nv < 3) {
      continue;
    }
    if (nv == 3) {
      out_planarity[size_t(f)] = 0.0f; /* triangles are planar */
      continue;
    }
    /* Fit plane to face verts; RMS distance / mean edge as unitless measure. */
    std::vector<Point> pts;
    pts.reserve(size_t(nv));
    for (int c = start; c < end; c++) {
      const int v = mesh.corner_verts[c];
      if (v < 0 || v >= mesh.verts_num) {
        continue;
      }
      pts.emplace_back(mesh.positions[v * 3],
                       mesh.positions[v * 3 + 1],
                       mesh.positions[v * 3 + 2]);
    }
    if (pts.size() < 3) {
      continue;
    }
    try {
      K::Plane_3 plane;
      Point centroid;
      CGAL::linear_least_squares_fitting_3(
          pts.begin(), pts.end(), plane, centroid, CGAL::Dimension_tag<0>());
      double sum_d2 = 0.0;
      double sum_e = 0.0;
      int ne = 0;
      for (const Point &p : pts) {
        const double d = CGAL::to_double(CGAL::squared_distance(p, plane));
        sum_d2 += d;
      }
      for (size_t i = 0; i < pts.size(); i++) {
        const Point &a = pts[i];
        const Point &b = pts[(i + 1) % pts.size()];
        sum_e += std::sqrt(CGAL::to_double(CGAL::squared_distance(a, b)));
        ne++;
      }
      const double rms = std::sqrt(sum_d2 / double(pts.size()));
      const double mean_e = (ne > 0) ? (sum_e / double(ne)) : 1.0;
      out_planarity[size_t(f)] = (mean_e > 1e-12) ? float(rms / mean_e) : float(rms);
    }
    catch (...) {
      out_planarity[size_t(f)] = 0.0f;
    }
  }
  return true;
}

bool mesh_is_triangle_mesh(const MeshIn &mesh, bool &out_is_triangle, std::string &error)
{
  if (mesh.faces_num <= 0) {
    error = "Empty mesh";
    return false;
  }
  if (!mesh.face_offsets) {
    out_is_triangle = (mesh.corners_num == mesh.faces_num * 3);
    return true;
  }
  for (int f = 0; f < mesh.faces_num; f++) {
    if (mesh.face_offsets[f + 1] - mesh.face_offsets[f] != 3) {
      out_is_triangle = false;
      return true;
    }
  }
  out_is_triangle = true;
  return true;
}

}  // namespace blender::cgal_bridge
