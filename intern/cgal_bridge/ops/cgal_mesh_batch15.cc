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
#include <CGAL/Polygon_mesh_processing/autorefinement.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Polygon_mesh_processing/interpolated_corrected_curvatures.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numeric>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

using K = Kernel;
using Point = Point_3;
using Vector = K::Vector_3;
using Pwn = std::pair<Point, Vector>;

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  return sm.number_of_faces() > 0;
}

bool mesh_is_closed(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  return CGAL::is_closed(sm);
}

bool mesh_does_bound_volume(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  try {
    return CGAL::is_closed(sm) && PMP::does_bound_a_volume(sm);
  }
  catch (...) {
    error = "does_bound_a_volume failed";
    return false;
  }
}

int mesh_component_count(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return 0;
  }
  try {
    auto fcc = sm.add_property_map<Surface_mesh::Face_index, std::size_t>("f:cc", 0).first;
    return int(PMP::connected_components(sm, fcc));
  }
  catch (const std::exception &e) {
    error = e.what();
    return 0;
  }
}

bool mesh_centroid(const MeshIn &mesh, float out_xyz[3], std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  try {
    if (!CGAL::is_closed(sm)) {
      /* Surface centroid of triangle mesh (area-weighted). */
      double ax = 0, ay = 0, az = 0, area_sum = 0;
      for (const auto f : sm.faces()) {
        if (sm.is_removed(f)) {
          continue;
        }
        std::vector<Point> pts;
        for (const auto v : vertices_around_face(sm.halfedge(f), sm)) {
          pts.push_back(sm.point(v));
        }
        if (pts.size() < 3) {
          continue;
        }
        const Point &p0 = pts[0];
        for (size_t i = 1; i + 1 < pts.size(); i++) {
          const Point &p1 = pts[i];
          const Point &p2 = pts[i + 1];
          const Vector n = CGAL::cross_product(p1 - p0, p2 - p0);
          const double a = 0.5 * std::sqrt(CGAL::to_double(n.squared_length()));
          if (!(a > 0.0)) {
            continue;
          }
          const double cx = (CGAL::to_double(p0.x()) + CGAL::to_double(p1.x()) +
                             CGAL::to_double(p2.x())) /
                            3.0;
          const double cy = (CGAL::to_double(p0.y()) + CGAL::to_double(p1.y()) +
                             CGAL::to_double(p2.y())) /
                            3.0;
          const double cz = (CGAL::to_double(p0.z()) + CGAL::to_double(p1.z()) +
                             CGAL::to_double(p2.z())) /
                            3.0;
          ax += a * cx;
          ay += a * cy;
          az += a * cz;
          area_sum += a;
        }
      }
      if (!(area_sum > 0.0)) {
        error = "Zero area mesh";
        return false;
      }
      out_xyz[0] = float(ax / area_sum);
      out_xyz[1] = float(ay / area_sum);
      out_xyz[2] = float(az / area_sum);
      return true;
    }
    const Point c = PMP::centroid(sm);
    out_xyz[0] = float(CGAL::to_double(c.x()));
    out_xyz[1] = float(CGAL::to_double(c.y()));
    out_xyz[2] = float(CGAL::to_double(c.z()));
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
}

double mesh_border_length(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return 0.0;
  }
  try {
    double len = 0.0;
    for (const auto e : sm.edges()) {
      if (sm.is_removed(e)) {
        continue;
      }
      if (sm.is_border(e)) {
        len += CGAL::to_double(PMP::edge_length(sm.halfedge(e), sm));
      }
    }
    return len;
  }
  catch (const std::exception &e) {
    error = e.what();
    return 0.0;
  }
}

double mesh_hausdorff_distance(const MeshIn &a, const MeshIn &b, std::string &error)
{
  Surface_mesh sma, smb;
  if (!load_tri(a, sma, error) || !load_tri(b, smb, error)) {
    return 0.0;
  }
  try {
    return PMP::approximate_symmetric_Hausdorff_distance<CGAL::Sequential_tag>(sma, smb);
  }
  catch (const std::exception &e) {
    error = e.what();
    return 0.0;
  }
  catch (...) {
    error = "Hausdorff distance failed";
    return 0.0;
  }
}

bool mesh_closest_points(const MeshIn &mesh,
                         const float *query_xyz,
                         int query_n,
                         std::vector<float> &out_xyz,
                         std::vector<float> &out_dist,
                         std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error) || !query_xyz || query_n <= 0) {
    error = error.empty() ? "Closest points needs mesh and queries" : error;
    return false;
  }
  try {
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
    using Traits = CGAL::AABB_traits_3<K, Primitive>;
    using Tree = CGAL::AABB_tree<Traits>;
    Tree tree(faces(sm).first, faces(sm).second, sm);
    tree.accelerate_distance_queries();
    out_xyz.resize(size_t(query_n) * 3);
    out_dist.resize(size_t(query_n));
    for (int i = 0; i < query_n; i++) {
      const Point q(query_xyz[i * 3], query_xyz[i * 3 + 1], query_xyz[i * 3 + 2]);
      const Point closest = tree.closest_point(q);
      out_xyz[size_t(i) * 3 + 0] = float(CGAL::to_double(closest.x()));
      out_xyz[size_t(i) * 3 + 1] = float(CGAL::to_double(closest.y()));
      out_xyz[size_t(i) * 3 + 2] = float(CGAL::to_double(closest.z()));
      out_dist[size_t(i)] = float(
          std::sqrt(CGAL::to_double(CGAL::squared_distance(q, closest))));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Closest points failed";
    return false;
  }
}

MeshResult mesh_stitch_borders(const MeshIn &mesh)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Stitch borders input failed" : error;
    return result;
  }
  try {
    PMP::stitch_borders(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "stitch_borders failed";
  }
  return result;
}

MeshResult mesh_autorefine(const MeshIn &mesh)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Autorefine input failed" : error;
    return result;
  }
  try {
    PMP::autorefine(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "autorefine failed";
  }
  return result;
}

MeshResult mesh_remove_degenerate(const MeshIn &mesh)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Remove degenerate input failed" : error;
    return result;
  }
  try {
    PMP::remove_degenerate_faces(sm);
    PMP::remove_degenerate_edges(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "remove_degenerate failed";
  }
  return result;
}

bool mesh_mean_curvature(const MeshIn &mesh, std::vector<float> &out_h, std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  try {
    auto hmap = sm.add_property_map<Surface_mesh::Vertex_index, double>("v:H", 0.0).first;
    PMP::interpolated_corrected_curvatures(
        sm, CGAL::parameters::vertex_mean_curvature_map(hmap));
    out_h.assign(size_t(mesh.verts_num), 0.0f);
    int i = 0;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      if (i < mesh.verts_num) {
        out_h[size_t(i)] = float(hmap[v]);
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
    error = "mean curvature failed";
    return false;
  }
}

bool mesh_gaussian_curvature(const MeshIn &mesh, std::vector<float> &out_k, std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  try {
    auto kmap = sm.add_property_map<Surface_mesh::Vertex_index, double>("v:K", 0.0).first;
    PMP::interpolated_corrected_curvatures(
        sm, CGAL::parameters::vertex_Gaussian_curvature_map(kmap));
    out_k.assign(size_t(mesh.verts_num), 0.0f);
    int i = 0;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      if (i < mesh.verts_num) {
        out_k[size_t(i)] = float(kmap[v]);
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
    error = "gaussian curvature failed";
    return false;
  }
}

bool mesh_face_areas(const MeshIn &mesh, std::vector<float> &out_area, std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  try {
    out_area.clear();
    out_area.reserve(size_t(sm.number_of_faces()));
    for (const auto f : sm.faces()) {
      if (sm.is_removed(f)) {
        continue;
      }
      out_area.push_back(float(CGAL::to_double(PMP::face_area(f, sm))));
    }
    return !out_area.empty();
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
}

static void append_aabb_box(const float bmin[3], const float bmax[3], MeshResult &result)
{
  /* 8 corners of axis-aligned box. */
  const float c[8][3] = {
      {bmin[0], bmin[1], bmin[2]},
      {bmax[0], bmin[1], bmin[2]},
      {bmax[0], bmax[1], bmin[2]},
      {bmin[0], bmax[1], bmin[2]},
      {bmin[0], bmin[1], bmax[2]},
      {bmax[0], bmin[1], bmax[2]},
      {bmax[0], bmax[1], bmax[2]},
      {bmin[0], bmax[1], bmax[2]},
  };
  result.positions.clear();
  for (int i = 0; i < 8; i++) {
    result.positions.push_back(c[i][0]);
    result.positions.push_back(c[i][1]);
    result.positions.push_back(c[i][2]);
  }
  static const int quads[6][4] = {
      {0, 1, 2, 3},
      {4, 7, 6, 5},
      {0, 4, 5, 1},
      {1, 5, 6, 2},
      {2, 6, 7, 3},
      {3, 7, 4, 0},
  };
  result.corner_verts.clear();
  result.face_offsets.clear();
  result.face_offsets.push_back(0);
  for (int f = 0; f < 6; f++) {
    for (int k = 0; k < 4; k++) {
      result.corner_verts.push_back(quads[f][k]);
    }
    result.face_offsets.push_back(int(result.corner_verts.size()));
  }
  result.ok = true;
}

MeshResult points_aabb(const float *positions, int n)
{
  MeshResult result;
  if (!positions || n < 1) {
    result.error = "AABB needs points";
    return result;
  }
  float bmin[3] = {positions[0], positions[1], positions[2]};
  float bmax[3] = {bmin[0], bmin[1], bmin[2]};
  for (int i = 1; i < n; i++) {
    for (int k = 0; k < 3; k++) {
      bmin[k] = std::min(bmin[k], positions[i * 3 + k]);
      bmax[k] = std::max(bmax[k], positions[i * 3 + k]);
    }
  }
  append_aabb_box(bmin, bmax, result);
  return result;
}

WireResult points_principal_axes(const float *positions, int n, float scale)
{
  WireResult result;
  if (!positions || n < 2) {
    result.error = "Principal axes need at least 2 points";
    return result;
  }
  try {
    double mean[3] = {0, 0, 0};
    for (int i = 0; i < n; i++) {
      mean[0] += positions[i * 3 + 0];
      mean[1] += positions[i * 3 + 1];
      mean[2] += positions[i * 3 + 2];
    }
    mean[0] /= n;
    mean[1] /= n;
    mean[2] /= n;

    double cov[3][3] = {{0}};
    for (int i = 0; i < n; i++) {
      const double d[3] = {positions[i * 3 + 0] - mean[0],
                           positions[i * 3 + 1] - mean[1],
                           positions[i * 3 + 2] - mean[2]};
      for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) {
          cov[a][b] += d[a] * d[b];
        }
      }
    }
    for (int a = 0; a < 3; a++) {
      for (int b = 0; b < 3; b++) {
        cov[a][b] /= std::max(n - 1, 1);
      }
    }

    /* Power iteration for 3 principal directions (descending). */
    double axes[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double lens[3] = {1, 1, 1};
    for (int ax = 0; ax < 3; ax++) {
      double v[3] = {axes[ax][0], axes[ax][1], axes[ax][2]};
      for (int iter = 0; iter < 32; iter++) {
        /* Deflate previous axes. */
        double cv[3] = {0, 0, 0};
        for (int a = 0; a < 3; a++) {
          for (int b = 0; b < 3; b++) {
            cv[a] += cov[a][b] * v[b];
          }
        }
        for (int p = 0; p < ax; p++) {
          double dot = cv[0] * axes[p][0] + cv[1] * axes[p][1] + cv[2] * axes[p][2];
          cv[0] -= dot * axes[p][0];
          cv[1] -= dot * axes[p][1];
          cv[2] -= dot * axes[p][2];
        }
        const double nrm = std::sqrt(cv[0] * cv[0] + cv[1] * cv[1] + cv[2] * cv[2]);
        if (nrm < 1e-18) {
          break;
        }
        v[0] = cv[0] / nrm;
        v[1] = cv[1] / nrm;
        v[2] = cv[2] / nrm;
      }
      axes[ax][0] = v[0];
      axes[ax][1] = v[1];
      axes[ax][2] = v[2];
      /* Rayleigh quotient length. */
      double cv[3] = {0, 0, 0};
      for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) {
          cv[a] += cov[a][b] * v[b];
        }
      }
      lens[ax] = std::sqrt(std::max(0.0, cv[0] * v[0] + cv[1] * v[1] + cv[2] * v[2]));
    }

    scale = std::max(scale, 1e-6f);
    /* 1 center + 6 ends (positive and negative for 3 axes). */
    result.positions = {
        float(mean[0]), float(mean[1]), float(mean[2]),
    };
    for (int ax = 0; ax < 3; ax++) {
      const double L = lens[ax] * double(scale);
      result.positions.push_back(float(mean[0] + axes[ax][0] * L));
      result.positions.push_back(float(mean[1] + axes[ax][1] * L));
      result.positions.push_back(float(mean[2] + axes[ax][2] * L));
      result.positions.push_back(float(mean[0] - axes[ax][0] * L));
      result.positions.push_back(float(mean[1] - axes[ax][1] * L));
      result.positions.push_back(float(mean[2] - axes[ax][2] * L));
      result.edge_v0.push_back(0);
      result.edge_v1.push_back(1 + ax * 2);
      result.edge_v0.push_back(0);
      result.edge_v1.push_back(2 + ax * 2);
    }
    result.ok = true;
  }
  catch (...) {
    result.error = "Principal axes failed";
  }
  return result;
}

bool points_detect_planes(const float *positions,
                          int n,
                          int min_points,
                          double epsilon,
                          double normal_threshold,
                          std::vector<int> &out_plane_id,
                          std::string &error)
{
  if (!positions || n < 10) {
    error = "Detect planes needs at least 10 points";
    return false;
  }
  try {
    std::vector<Pwn> points;
    points.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      points.emplace_back(Point(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]),
                          Vector(0, 0, 1));
    }
    const int k = std::min(24, std::max(6, n / 20));
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>(
        points,
        k,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
            .normal_map(CGAL::Second_of_pair_property_map<Pwn>()));
    CGAL::mst_orient_normals(
        points,
        k,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
            .normal_map(CGAL::Second_of_pair_property_map<Pwn>()));

    using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<K,
                                                                  std::vector<Pwn>,
                                                                  CGAL::First_of_pair_property_map<Pwn>,
                                                                  CGAL::Second_of_pair_property_map<Pwn>>;
    using Efficient_ransac = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
    using Plane = CGAL::Shape_detection::Plane<Traits>;

    Efficient_ransac ransac;
    ransac.set_input(points);
    ransac.add_shape_factory<Plane>();

    Efficient_ransac::Parameters params;
    params.probability = 0.05;
    params.min_points = std::size_t(std::max(min_points, 10));
    params.epsilon = std::max(epsilon, 1e-6);
    params.cluster_epsilon = params.epsilon * 2.0;
    params.normal_threshold = std::clamp(normal_threshold, 0.0, 1.0);
    ransac.detect(params);

    out_plane_id.assign(size_t(n), -1);
    int plane_i = 0;
    for (const auto &shape : ransac.shapes()) {
      for (const std::size_t idx : shape->indices_of_assigned_points()) {
        if (idx < size_t(n)) {
          out_plane_id[idx] = plane_i;
        }
      }
      plane_i++;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Efficient RANSAC plane detection failed";
    return false;
  }
}

}  // namespace blender::cgal_bridge
