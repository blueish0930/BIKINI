/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Polygon_mesh_processing/distance.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/jet_smooth_point_set.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/random_simplify_point_set.h>
#include <CGAL/remove_outliers.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/hierarchy_simplify_point_set.h>
#include <CGAL/edge_aware_upsample_point_set.h>
#include <CGAL/vcm_estimate_normals.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

using K = Kernel;
using Point = K::Point_3;
using Vector = K::Vector_3;
using Pwn = std::pair<Point, Vector>;

static std::vector<Point> to_points(const float *positions, int n)
{
  std::vector<Point> pts;
  pts.reserve(size_t(n));
  for (int i = 0; i < n; i++) {
    pts.emplace_back(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
  }
  return pts;
}

double points_average_spacing(const float *positions, int n, int k, std::string &error)
{
  if (!positions || n < 3) {
    error = "Need at least 3 points";
    return 0.0;
  }
  try {
    k = std::max(k, 1);
    auto pts = to_points(positions, n);
    return CGAL::compute_average_spacing<CGAL::Sequential_tag>(pts, k);
  }
  catch (const std::exception &e) {
    error = e.what();
    return 0.0;
  }
  catch (...) {
    error = "CGAL compute_average_spacing failed";
    return 0.0;
  }
}

bool points_jet_smooth(
    const float *in_xyz, int n, int k, int iter, float *out_xyz, std::string &error)
{
  if (!in_xyz || !out_xyz || n < 3) {
    error = "Need at least 3 points";
    return false;
  }
  try {
    auto pts = to_points(in_xyz, n);
    k = std::max(k, 6);
    iter = std::max(iter, 1);
    for (int i = 0; i < iter; i++) {
      CGAL::jet_smooth_point_set<CGAL::Sequential_tag>(pts, k);
    }
    for (int i = 0; i < n; i++) {
      out_xyz[i * 3 + 0] = float(CGAL::to_double(pts[i].x()));
      out_xyz[i * 3 + 1] = float(CGAL::to_double(pts[i].y()));
      out_xyz[i * 3 + 2] = float(CGAL::to_double(pts[i].z()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL jet_smooth_point_set failed";
    return false;
  }
}

bool points_bilateral_smooth(const float *in_xyz,
                             int n,
                             int k,
                             int iter,
                             double sharpness,
                             float *out_xyz,
                             std::string &error)
{
  if (!in_xyz || !out_xyz || n < 3) {
    error = "Need at least 3 points";
    return false;
  }
  try {
    std::vector<Pwn> points;
    points.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      points.emplace_back(Point(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]), Vector(0, 0, 1));
    }
    k = std::max(k, 6);
    iter = std::max(iter, 1);
    sharpness = std::max(sharpness, 0.001);
    CGAL::jet_estimate_normals<CGAL::Sequential_tag>(
        points, k, CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>()).normal_map(CGAL::Second_of_pair_property_map<Pwn>()));
    for (int i = 0; i < iter; i++) {
      CGAL::bilateral_smooth_point_set<CGAL::Sequential_tag>(
          points,
          k,
          CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
              .normal_map(CGAL::Second_of_pair_property_map<Pwn>())
              .sharpness_angle(sharpness));
    }
    for (int i = 0; i < n; i++) {
      out_xyz[i * 3 + 0] = float(CGAL::to_double(points[i].first.x()));
      out_xyz[i * 3 + 1] = float(CGAL::to_double(points[i].first.y()));
      out_xyz[i * 3 + 2] = float(CGAL::to_double(points[i].first.z()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL bilateral_smooth_point_set failed";
    return false;
  }
}

bool points_remove_outliers(const float *in_xyz,
                            int n,
                            int k,
                            double percent,
                            std::vector<float> &out_xyz,
                            std::string &error)
{
  if (!in_xyz || n < 3) {
    error = "Need at least 3 points";
    return false;
  }
  try {
    auto pts = to_points(in_xyz, n);
    k = std::max(k, 6);
    percent = std::clamp(percent, 0.0, 100.0);
    auto it = CGAL::remove_outliers<CGAL::Sequential_tag>(
        pts, k, CGAL::parameters::threshold_percent(percent));
    pts.erase(it, pts.end());
    out_xyz.resize(pts.size() * 3);
    for (size_t i = 0; i < pts.size(); i++) {
      out_xyz[i * 3 + 0] = float(CGAL::to_double(pts[i].x()));
      out_xyz[i * 3 + 1] = float(CGAL::to_double(pts[i].y()));
      out_xyz[i * 3 + 2] = float(CGAL::to_double(pts[i].z()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL remove_outliers failed";
    return false;
  }
}

bool points_grid_simplify(const float *in_xyz,
                          int n,
                          double cell_size,
                          std::vector<float> &out_xyz,
                          std::string &error)
{
  if (!in_xyz || n < 1) {
    error = "Need points";
    return false;
  }
  try {
    auto pts = to_points(in_xyz, n);
    cell_size = std::max(cell_size, 1e-12);
    auto it = CGAL::grid_simplify_point_set(pts, cell_size);
    pts.erase(it, pts.end());
    out_xyz.resize(pts.size() * 3);
    for (size_t i = 0; i < pts.size(); i++) {
      out_xyz[i * 3 + 0] = float(CGAL::to_double(pts[i].x()));
      out_xyz[i * 3 + 1] = float(CGAL::to_double(pts[i].y()));
      out_xyz[i * 3 + 2] = float(CGAL::to_double(pts[i].z()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL grid_simplify_point_set failed";
    return false;
  }
}

bool points_random_simplify(const float *in_xyz,
                            int n,
                            double percent,
                            std::vector<float> &out_xyz,
                            std::string &error)
{
  if (!in_xyz || n < 1) {
    error = "Need points";
    return false;
  }
  try {
    auto pts = to_points(in_xyz, n);
    percent = std::clamp(percent, 0.0, 100.0);
    auto it = CGAL::random_simplify_point_set(pts, percent);
    pts.erase(it, pts.end());
    out_xyz.resize(pts.size() * 3);
    for (size_t i = 0; i < pts.size(); i++) {
      out_xyz[i * 3 + 0] = float(CGAL::to_double(pts[i].x()));
      out_xyz[i * 3 + 1] = float(CGAL::to_double(pts[i].y()));
      out_xyz[i * 3 + 2] = float(CGAL::to_double(pts[i].z()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL random_simplify_point_set failed";
    return false;
  }
}

bool points_estimate_normals(
    const float *in_xyz, int n, int k, float *out_nxyz, std::string &error)
{
  if (!in_xyz || !out_nxyz || n < 3) {
    error = "Need at least 3 points";
    return false;
  }
  try {
    std::vector<Pwn> points;
    points.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      points.emplace_back(Point(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]), Vector(0, 0, 0));
    }
    k = std::max(k, 6);
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
    for (int i = 0; i < n; i++) {
      out_nxyz[i * 3 + 0] = float(CGAL::to_double(points[i].second.x()));
      out_nxyz[i * 3 + 1] = float(CGAL::to_double(points[i].second.y()));
      out_nxyz[i * 3 + 2] = float(CGAL::to_double(points[i].second.z()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL jet_estimate_normals failed";
    return false;
  }
}

MeshResult points_poisson_reconstruct(const float *positions,
                                      const float *normals,
                                      int n,
                                      double spacing)
{
  MeshResult result;
  if (!positions || !normals || n < 10) {
    result.error = "Poisson needs points with normals (at least 10)";
    return result;
  }
  try {
    std::vector<Pwn> points;
    points.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      points.emplace_back(
          Point(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]),
          Vector(normals[i * 3], normals[i * 3 + 1], normals[i * 3 + 2]));
    }
    spacing = std::max(spacing, 1e-8);
    Surface_mesh sm;
    if (!CGAL::poisson_surface_reconstruction_delaunay(
            points.begin(),
            points.end(),
            CGAL::First_of_pair_property_map<Pwn>(),
            CGAL::Second_of_pair_property_map<Pwn>(),
            sm,
            spacing))
    {
      result.error = "CGAL poisson_surface_reconstruction failed";
      return result;
    }
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL poisson reconstruction failed";
  }
  return result;
}

MeshResult points_scale_space_reconstruct(const float *positions, int n, int iterations)
{
  /* Default: Weighted PCA smoother + Alpha-shape mesher. */
  return points_scale_space_reconstruct_ex(positions, n, iterations, 0, 0);
}

bool points_wlop(const float *in_xyz,
                 int n,
                 double select_percentage,
                 double neighbor_radius,
                 int iterations,
                 bool require_uniform,
                 std::vector<float> &out_xyz,
                 std::string &error)
{
  if (!in_xyz || n < 3) {
    error = "WLOP needs at least 3 points";
    return false;
  }
  try {
    auto pts = to_points(in_xyz, n);
    select_percentage = std::clamp(select_percentage, 0.1, 100.0);
    iterations = std::max(1, iterations);
    std::vector<Point> out;
    out.reserve(size_t(n));
    auto np = CGAL::parameters::select_percentage(select_percentage)
                  .number_of_iterations(unsigned(iterations))
                  .require_uniform_sampling(require_uniform);
    if (neighbor_radius > 0.0) {
      CGAL::wlop_simplify_and_regularize_point_set<CGAL::Sequential_tag>(
          pts, std::back_inserter(out), np.neighbor_radius(neighbor_radius));
    }
    else {
      CGAL::wlop_simplify_and_regularize_point_set<CGAL::Sequential_tag>(
          pts, std::back_inserter(out), np);
    }
    out_xyz.resize(out.size() * 3);
    for (size_t i = 0; i < out.size(); i++) {
      out_xyz[i * 3 + 0] = float(CGAL::to_double(out[i].x()));
      out_xyz[i * 3 + 1] = float(CGAL::to_double(out[i].y()));
      out_xyz[i * 3 + 2] = float(CGAL::to_double(out[i].z()));
    }
    return !out.empty();
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL WLOP failed";
    return false;
  }
}

bool points_hierarchy_simplify(const float *in_xyz,
                               int n,
                               int cluster_size,
                               double max_variation,
                               std::vector<float> &out_xyz,
                               std::string &error)
{
  if (!in_xyz || n < 3) {
    error = "Hierarchy Simplify needs at least 3 points";
    return false;
  }
  try {
    auto pts = to_points(in_xyz, n);
    cluster_size = std::max(1, cluster_size);
    max_variation = std::clamp(max_variation, 1e-6, 1.0 / 3.0);
    auto it = CGAL::hierarchy_simplify_point_set(
        pts,
        CGAL::parameters::size(unsigned(cluster_size)).maximum_variation(max_variation));
    pts.erase(it, pts.end());
    out_xyz.resize(pts.size() * 3);
    for (size_t i = 0; i < pts.size(); i++) {
      out_xyz[i * 3 + 0] = float(CGAL::to_double(pts[i].x()));
      out_xyz[i * 3 + 1] = float(CGAL::to_double(pts[i].y()));
      out_xyz[i * 3 + 2] = float(CGAL::to_double(pts[i].z()));
    }
    return !pts.empty();
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL hierarchy_simplify_point_set failed";
    return false;
  }
}

bool points_edge_aware_upsample(const float *in_xyz,
                                const float *in_nxyz,
                                int n,
                                int number_of_output_points,
                                double sharpness_angle_deg,
                                double edge_sensitivity,
                                double neighbor_radius,
                                int normal_neighbors,
                                std::vector<float> &out_xyz,
                                std::vector<float> &out_nxyz,
                                std::string &error)
{
  if (!in_xyz || n < 3) {
    error = "Edge Aware Upsample needs at least 3 points";
    return false;
  }
  try {
    std::vector<Pwn> points;
    points.reserve(size_t(n));

    /* CGAL requires non-zero normals. Accept input normals only if all valid. */
    bool use_input_normals = false;
    if (in_nxyz) {
      use_input_normals = true;
      for (int i = 0; i < n; i++) {
        const double nx = in_nxyz[i * 3], ny = in_nxyz[i * 3 + 1], nz = in_nxyz[i * 3 + 2];
        if (!(nx * nx + ny * ny + nz * nz > 1e-12)) {
          use_input_normals = false;
          break;
        }
      }
    }
    if (use_input_normals) {
      for (int i = 0; i < n; i++) {
        Vector nrm(in_nxyz[i * 3], in_nxyz[i * 3 + 1], in_nxyz[i * 3 + 2]);
        const double len = std::sqrt(CGAL::to_double(nrm.squared_length()));
        if (len > 1e-12) {
          nrm = nrm / len;
        }
        points.emplace_back(Point(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]), nrm);
      }
    }
    else {
      for (int i = 0; i < n; i++) {
        points.emplace_back(Point(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                            Vector(0, 0, 0));
      }
      const int k = std::max(6, normal_neighbors);
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
    }

    /*
     * CGAL semantics (edge_aware_upsample_point_set.h):
     * - number_of_output_points = target TOTAL size of internal set (must be > input)
     * - Output iterator receives ONLY newly inserted points (not the originals)
     * We reassemble: originals + new => full densified cloud of ~target size.
     */
    const int target_total = std::max(n + 1, number_of_output_points);
    sharpness_angle_deg = std::clamp(sharpness_angle_deg, 0.0, 90.0);
    edge_sensitivity = std::clamp(edge_sensitivity, 0.0, 1.0);

    std::vector<Pwn> new_pts;
    new_pts.reserve(size_t(std::max(0, target_total - n)));
    auto np =
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
            .normal_map(CGAL::Second_of_pair_property_map<Pwn>())
            .sharpness_angle(sharpness_angle_deg)
            .edge_sensitivity(edge_sensitivity)
            .number_of_output_points(std::size_t(target_total));
    if (neighbor_radius > 0.0) {
      CGAL::edge_aware_upsample_point_set<CGAL::Sequential_tag>(
          points, std::back_inserter(new_pts), np.neighbor_radius(neighbor_radius));
    }
    else {
      CGAL::edge_aware_upsample_point_set<CGAL::Sequential_tag>(
          points, std::back_inserter(new_pts), np);
    }

    /* Originals first (preserve order for attribute copy), then CGAL new points. */
    const size_t total = size_t(n) + new_pts.size();
    out_xyz.resize(total * 3);
    out_nxyz.resize(total * 3);
    for (int i = 0; i < n; i++) {
      out_xyz[size_t(i) * 3 + 0] = float(CGAL::to_double(points[size_t(i)].first.x()));
      out_xyz[size_t(i) * 3 + 1] = float(CGAL::to_double(points[size_t(i)].first.y()));
      out_xyz[size_t(i) * 3 + 2] = float(CGAL::to_double(points[size_t(i)].first.z()));
      out_nxyz[size_t(i) * 3 + 0] = float(CGAL::to_double(points[size_t(i)].second.x()));
      out_nxyz[size_t(i) * 3 + 1] = float(CGAL::to_double(points[size_t(i)].second.y()));
      out_nxyz[size_t(i) * 3 + 2] = float(CGAL::to_double(points[size_t(i)].second.z()));
    }
    for (size_t i = 0; i < new_pts.size(); i++) {
      const size_t o = size_t(n) + i;
      out_xyz[o * 3 + 0] = float(CGAL::to_double(new_pts[i].first.x()));
      out_xyz[o * 3 + 1] = float(CGAL::to_double(new_pts[i].first.y()));
      out_xyz[o * 3 + 2] = float(CGAL::to_double(new_pts[i].first.z()));
      out_nxyz[o * 3 + 0] = float(CGAL::to_double(new_pts[i].second.x()));
      out_nxyz[o * 3 + 1] = float(CGAL::to_double(new_pts[i].second.y()));
      out_nxyz[o * 3 + 2] = float(CGAL::to_double(new_pts[i].second.z()));
    }
    if (total == size_t(n)) {
      error =
          "No new points inserted (density already high or bad normals / Neighbor Radius). "
          "Try lower Edge Sensitivity, larger Number of Points, or fix normals.";
      /* Still return originals so the graph stays usable. */
    }
    return total > 0;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL edge_aware_upsample_point_set failed";
    return false;
  }
}

bool points_vcm_estimate_normals(const float *in_xyz,
                                 int n,
                                 double offset_radius,
                                 double convolution_radius,
                                 float *out_nxyz,
                                 std::string &error)
{
  if (!in_xyz || !out_nxyz || n < 3) {
    error = "VCM Estimate Normals needs at least 3 points";
    return false;
  }
  if (!(offset_radius > 0.0) || !(convolution_radius > 0.0)) {
    error = "VCM radii must be positive";
    return false;
  }
  try {
    std::vector<Pwn> points;
    points.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      points.emplace_back(Point(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                          Vector(0, 0, 0));
    }
    CGAL::vcm_estimate_normals(
        points,
        offset_radius,
        convolution_radius,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
            .normal_map(CGAL::Second_of_pair_property_map<Pwn>()));
    const int k = std::min(24, std::max(6, n / 20));
    CGAL::mst_orient_normals(
        points,
        k,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
            .normal_map(CGAL::Second_of_pair_property_map<Pwn>()));
    for (int i = 0; i < n; i++) {
      out_nxyz[i * 3 + 0] = float(CGAL::to_double(points[size_t(i)].second.x()));
      out_nxyz[i * 3 + 1] = float(CGAL::to_double(points[size_t(i)].second.y()));
      out_nxyz[i * 3 + 2] = float(CGAL::to_double(points[size_t(i)].second.z()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL vcm_estimate_normals failed";
    return false;
  }
}

bool mesh_side_of(const MeshIn &mesh,
                  const float *query_xyz,
                  int query_n,
                  std::vector<int8_t> &out_side,
                  std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  try {
    if (!CGAL::is_closed(sm)) {
      error = "Side of mesh requires a closed triangle mesh";
      return false;
    }
    CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel> side_of(sm);
    out_side.resize(size_t(query_n));
    for (int i = 0; i < query_n; i++) {
      const Point p(query_xyz[i * 3], query_xyz[i * 3 + 1], query_xyz[i * 3 + 2]);
      const auto s = side_of(p);
      if (s == CGAL::ON_BOUNDED_SIDE) {
        out_side[size_t(i)] = 1;
      }
      else if (s == CGAL::ON_UNBOUNDED_SIDE) {
        out_side[size_t(i)] = -1;
      }
      else {
        out_side[size_t(i)] = 0;
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL Side_of_triangle_mesh failed";
    return false;
  }
}

bool mesh_distance_to(const MeshIn &mesh,
                      const float *query_xyz,
                      int query_n,
                      std::vector<float> &out_dist,
                      std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  try {
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
    using Tree = CGAL::AABB_tree<CGAL::AABB_traits_3<Kernel, Primitive>>;
    Tree tree(faces(sm).first, faces(sm).second, sm);
    tree.accelerate_distance_queries();
    out_dist.resize(size_t(query_n));
    for (int i = 0; i < query_n; i++) {
      const Point p(query_xyz[i * 3], query_xyz[i * 3 + 1], query_xyz[i * 3 + 2]);
      out_dist[size_t(i)] = float(
          std::sqrt(CGAL::to_double(tree.squared_distance(p))));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL mesh distance failed";
    return false;
  }
}

bool mesh_sample_points(const MeshIn &mesh,
                        int count,
                        std::vector<float> &out_xyz,
                        std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  try {
    count = std::max(count, 1);
    std::vector<Point> samples;
    samples.reserve(size_t(count));
    PMP::sample_triangle_mesh(sm, std::back_inserter(samples),
                              CGAL::parameters::number_of_points_on_faces(static_cast<std::size_t>(count)));
    out_xyz.resize(samples.size() * 3);
    for (size_t i = 0; i < samples.size(); i++) {
      out_xyz[i * 3 + 0] = float(CGAL::to_double(samples[i].x()));
      out_xyz[i * 3 + 1] = float(CGAL::to_double(samples[i].y()));
      out_xyz[i * 3 + 2] = float(CGAL::to_double(samples[i].z()));
    }
    return !samples.empty();
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL sample_triangle_mesh failed";
    return false;
  }
}

}  // namespace blender::cgal_bridge
