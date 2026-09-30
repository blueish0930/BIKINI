/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 45 (2026-08-20, new only — do not restore deleted ids):
 *  - Regularize Open Contour 2D (regularize_open_contour)
 *  - Constrained Simplify (QEM; keep boundary / selected verts)
 *
 * Skipped: OpenGR, Kinetic EPECK, Mesh_3 volume, PolyFit SCIP, Ridges_3,
 * Classification, OSQP, Hyperbolic Delaunay, weighted Delaunay / Lloyd,
 * banned measure/query/remesh, previously deleted nodes (including Polar Dual,
 * PCA Ellipsoid, Polyline Line Fitting, Snap Meshes, Apollonius Graph,
 * Hole Fill, RANSAC, Voronoi Slice, Volume Components, Convex Offset).
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Barycentric_coordinates_2/Delaunay_domain_2.h>
#include <CGAL/Barycentric_coordinates_2/Harmonic_coordinates_2.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Shape_regularization/regularize_contours.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_plane_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Polyhedral_envelope_filter.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/centroid.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;
namespace SMS = CGAL::Surface_mesh_simplification;

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using FT = Kernel::FT;

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  PMP::triangulate_faces(sm);
  sm.collect_garbage();
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

static Point_2 to_p2_xy(const float *p)
{
  return Point_2(p[0], p[1]);
}

static std::vector<Point_2> open_collapse_short(std::vector<Point_2> contour, double min_len)
{
  if (min_len <= 0.0 || contour.size() < 2) {
    return contour;
  }
  const double min_sq = min_len * min_len;
  std::vector<Point_2> out;
  out.reserve(contour.size());
  out.push_back(contour.front());
  for (size_t i = 1; i < contour.size(); i++) {
    const double dx = CGAL::to_double(contour[i].x() - out.back().x());
    const double dy = CGAL::to_double(contour[i].y() - out.back().y());
    const bool last = (i + 1 == contour.size());
    if (dx * dx + dy * dy >= min_sq) {
      out.push_back(contour[i]);
    }
    else if (last) {
      if (out.size() == 1) {
        out.push_back(contour[i]);
      }
      else {
        out.back() = contour[i];
      }
    }
  }
  if (out.size() < 2 && !contour.empty()) {
    out.push_back(contour.back());
  }
  return out;
}

static void dp_open_rec(const std::vector<Point_2> &c,
                        int a,
                        int b,
                        double tol_sq,
                        std::vector<int8_t> &keep)
{
  if (b <= a + 1) {
    return;
  }
  const double ax = CGAL::to_double(c[size_t(a)].x());
  const double ay = CGAL::to_double(c[size_t(a)].y());
  const double bx = CGAL::to_double(c[size_t(b)].x());
  const double by = CGAL::to_double(c[size_t(b)].y());
  const double vx = bx - ax;
  const double vy = by - ay;
  const double vlen2 = vx * vx + vy * vy;
  int best = -1;
  double best_d = 0.0;
  for (int i = a + 1; i < b; i++) {
    const double px = CGAL::to_double(c[size_t(i)].x()) - ax;
    const double py = CGAL::to_double(c[size_t(i)].y()) - ay;
    double d2;
    if (vlen2 < 1e-24) {
      d2 = px * px + py * py;
    }
    else {
      const double t = std::clamp((px * vx + py * vy) / vlen2, 0.0, 1.0);
      const double dx = px - t * vx;
      const double dy = py - t * vy;
      d2 = dx * dx + dy * dy;
    }
    if (d2 > best_d) {
      best_d = d2;
      best = i;
    }
  }
  if (best >= 0 && best_d > tol_sq) {
    keep[size_t(best)] = 1;
    dp_open_rec(c, a, best, tol_sq, keep);
    dp_open_rec(c, best, b, tol_sq, keep);
  }
}

static std::vector<Point_2> open_simplify_dp(std::vector<Point_2> contour, double tol)
{
  if (tol <= 0.0 || contour.size() < 3) {
    return contour;
  }
  std::vector<int8_t> keep(contour.size(), 0);
  keep.front() = 1;
  keep.back() = 1;
  dp_open_rec(contour, 0, int(contour.size()) - 1, tol * tol, keep);
  std::vector<Point_2> out;
  out.reserve(contour.size());
  for (size_t i = 0; i < contour.size(); i++) {
    if (keep[i]) {
      out.push_back(contour[i]);
    }
  }
  return out.size() >= 2 ? out : contour;
}

static std::vector<Point_2> open_apply_min_length(std::vector<Point_2> contour, double min_len)
{
  if (min_len <= 0.0 || contour.size() < 2) {
    return contour;
  }
  contour = open_collapse_short(std::move(contour), min_len);
  contour = open_simplify_dp(std::move(contour), min_len);
  contour = open_collapse_short(std::move(contour), min_len);
  return contour;
}

}  // namespace

bool regularize_open_polyline_xy(const float *xyz,
                                 int n,
                                 bool closed,
                                 double max_offset,
                                 double min_length,
                                 std::vector<float> &out_xyz,
                                 std::string &error)
{
  out_xyz.clear();
  CgalThrowGuard guard;
  if (!xyz || n < (closed ? 3 : 2)) {
    error = closed ? "Regularize Open Contour 2D needs ≥3 points on a loop" :
                     "Regularize Open Contour 2D needs ≥2 points";
    return false;
  }
  try {
    std::vector<Point_2> contour;
    contour.reserve(size_t(n));
    double zsum = 0.0;
    for (int i = 0; i < n; i++) {
      contour.push_back(to_p2_xy(xyz + i * 3));
      zsum += double(xyz[i * 3 + 2]);
    }
    const float zmean = float(zsum / double(n));
    if (closed && contour.size() >= 2) {
      const double dx = CGAL::to_double(contour.front().x() - contour.back().x());
      const double dy = CGAL::to_double(contour.front().y() - contour.back().y());
      if (dx * dx + dy * dy < 1e-24) {
        contour.pop_back();
      }
    }
    min_length = std::max(0.0, min_length);
    max_offset = std::max(0.0, max_offset);
    if (min_length > 0.0) {
      contour = open_apply_min_length(std::move(contour), min_length);
    }
    const int need = closed ? 3 : 2;
    if (int(contour.size()) < need) {
      error = "Regularize Open Contour 2D: too few points after Min Length";
      return false;
    }
    namespace SRC = CGAL::Shape_regularization::Contours;
    std::vector<Point_2> regularized;
    if (max_offset > 0.0) {
      if (closed) {
        SRC::Longest_direction_2<Kernel, std::vector<Point_2>> directions(contour, true);
        SRC::regularize_closed_contour(contour,
                                       directions,
                                       std::back_inserter(regularized),
                                       CGAL::parameters::maximum_offset(max_offset));
      }
      else {
        SRC::Longest_direction_2<Kernel, std::vector<Point_2>> directions(contour, false);
        SRC::regularize_open_contour(contour,
                                     directions,
                                     std::back_inserter(regularized),
                                     CGAL::parameters::maximum_offset(max_offset));
      }
    }
    if (int(regularized.size()) < need) {
      regularized = contour;
    }
    if (min_length > 0.0) {
      regularized = open_apply_min_length(std::move(regularized), min_length);
    }
    if (int(regularized.size()) < need) {
      error = "Regularize Open Contour 2D produced too few points";
      return false;
    }
    out_xyz.reserve(regularized.size() * 3);
    for (const Point_2 &p : regularized) {
      out_xyz.push_back(float(CGAL::to_double(p.x())));
      out_xyz.push_back(float(CGAL::to_double(p.y())));
      out_xyz.push_back(zmean);
    }
    return out_xyz.size() >= 6;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Regularize Open Contour 2D failed";
    return false;
  }
}

MeshResult mesh_constrained_simplify(const MeshIn &mesh,
                                     double stop_ratio,
                                     bool keep_boundary,
                                     const uint8_t *preserve,
                                     int preserve_n)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Constrained Simplify needs a mesh with faces" : error;
    return result;
  }
  try {
    stop_ratio = std::max(0.01, std::min(stop_ratio, 1.0));
    auto ecm = sm.add_property_map<Surface_mesh::Edge_index, bool>("e:keep", false).first;
    auto vsrc_opt = sm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
    const bool have_src = vsrc_opt.has_value();
    for (auto e : sm.edges()) {
      bool keep = false;
      if (keep_boundary && sm.is_border(e)) {
        keep = true;
      }
      if (!keep && preserve && preserve_n > 0 && have_src) {
        const auto vsrc = vsrc_opt.value();
        const auto v0 = sm.vertex(e, 0);
        const auto v1 = sm.vertex(e, 1);
        const int i0 = vsrc[v0];
        const int i1 = vsrc[v1];
        if (i0 >= 0 && i0 < preserve_n && preserve[i0]) {
          keep = true;
        }
        if (i1 >= 0 && i1 < preserve_n && preserve[i1]) {
          keep = true;
        }
      }
      ecm[e] = keep;
    }
    SMS::Edge_count_ratio_stop_predicate<Surface_mesh> stop(stop_ratio);
    SMS::GarlandHeckbert_plane_policies<Surface_mesh, Kernel> policies(sm);
    auto base_place = policies.get_placement();
    SMS::Constrained_placement<decltype(base_place), decltype(ecm)> placement(ecm, base_place);
    SMS::edge_collapse(sm,
                       stop,
                       CGAL::parameters::get_cost(policies.get_cost())
                           .get_placement(placement)
                           .edge_is_constrained_map(ecm));
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Constrained Simplify failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
