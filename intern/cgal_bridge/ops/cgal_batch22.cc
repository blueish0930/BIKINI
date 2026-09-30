/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 23:
 *  - M07 repair_degeneracies (almost-degenerate needles/caps)
 *  - M36 surface_intersection (polylines)
 *  - P20 structure_point_set (planes via Efficient RANSAC)
 *  - R06/R07 Scale Space smoother/mesher variants
 */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/intersection_polylines.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Scale_space_reconstruction_3/Advancing_front_mesher.h>
#include <CGAL/Scale_space_reconstruction_3/Alpha_shape_mesher.h>
#include <CGAL/Scale_space_reconstruction_3/Jet_smoother.h>
#include <CGAL/Scale_space_reconstruction_3/Weighted_PCA_smoother.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/structure_point_set.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Plane_3 = Kernel::Plane_3;
using Pwn = std::pair<Point_3, Vector_3>;
using IndexedPwn = std::tuple<Point_3, Vector_3, int>;

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  PMP::triangulate_faces(sm);
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

}  // namespace

MeshResult mesh_repair_degeneracies(const MeshIn &mesh,
                                    double cap_threshold,
                                    double needle_threshold,
                                    double collapse_length_threshold)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Repair degeneracies needs a non-empty mesh" : error;
    return result;
  }
  try {
    /* Exact degenerates first, then almost-degenerate needles/caps.
     * cap_threshold is cos(angle): default ~ cos(160°) ≈ -0.94. */
    PMP::remove_degenerate_faces(sm);
    PMP::remove_degenerate_edges(sm);

    const double cap = std::clamp(cap_threshold, -1.0, 1.0);
    const double needle = std::max(1.0, needle_threshold);
    const double collapse = std::max(0.0, collapse_length_threshold);
    PMP::remove_almost_degenerate_faces(
        sm,
        CGAL::parameters::cap_threshold(cap)
            .needle_threshold(needle)
            .collapse_length_threshold(collapse));

    result = surface_mesh_to_result(sm);
    if (!result.ok && result.error.empty()) {
      result.error = "Repair degeneracies produced empty mesh";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "Repair degeneracies failed";
    result.ok = false;
  }
  return result;
}

bool mesh_surface_intersection_polylines(const MeshIn &mesh_a,
                                         const MeshIn &mesh_b,
                                         std::vector<std::vector<float>> &out_polylines,
                                         std::string &error)
{
  out_polylines.clear();
  Surface_mesh a, b;
  if (!load_tri(mesh_a, a, error)) {
    if (error.empty()) {
      error = "Mesh Intersection: Mesh 1 empty/invalid";
    }
    return false;
  }
  if (!load_tri(mesh_b, b, error)) {
    if (error.empty()) {
      error = "Mesh Intersection: Mesh 2 empty/invalid";
    }
    return false;
  }
  try {
    std::vector<std::vector<Point_3>> polys;
    PMP::intersection_polylines(a, b, std::back_inserter(polys));
    out_polylines.reserve(polys.size());
    for (const auto &poly : polys) {
      if (poly.size() < 2) {
        continue;
      }
      std::vector<float> xyz;
      xyz.reserve(poly.size() * 3);
      for (const Point_3 &p : poly) {
        xyz.push_back(float(p.x()));
        xyz.push_back(float(p.y()));
        xyz.push_back(float(p.z()));
      }
      out_polylines.push_back(std::move(xyz));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Mesh surface intersection failed";
    return false;
  }
}

bool points_structure(const float *in_xyz,
                      const float *in_nxyz,
                      int n,
                      double epsilon,
                      double attraction_factor,
                      double ransac_epsilon,
                      double ransac_cluster_epsilon,
                      int min_points,
                      std::vector<float> &out_xyz,
                      std::vector<float> &out_nxyz,
                      std::string &error)
{
  out_xyz.clear();
  out_nxyz.clear();
  if (!in_xyz || n < 12) {
    error = "Structure Point Set needs at least 12 points";
    return false;
  }
  if (!(epsilon > 0.0)) {
    error = "Structure Point Set needs epsilon > 0 (sampling / adjacency size)";
    return false;
  }
  try {
    /* Normals: use input or jet estimate. */
    std::vector<float> estimated;
    const float *normals = in_nxyz;
    if (!normals) {
      std::vector<Pwn> tmp;
      tmp.reserve(size_t(n));
      for (int i = 0; i < n; i++) {
        tmp.emplace_back(Point_3(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                         Vector_3(0, 0, 1));
      }
      const int k = std::min(24, std::max(3, n - 1));
      CGAL::jet_estimate_normals<CGAL::Sequential_tag>(
          tmp,
          k,
          CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>()).normal_map(
              CGAL::Second_of_pair_property_map<Pwn>()));
      CGAL::mst_orient_normals(
          tmp,
          k,
          CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>()).normal_map(
              CGAL::Second_of_pair_property_map<Pwn>()));
      estimated.resize(size_t(n) * 3);
      for (int i = 0; i < n; i++) {
        estimated[size_t(i) * 3 + 0] = float(tmp[size_t(i)].second.x());
        estimated[size_t(i) * 3 + 1] = float(tmp[size_t(i)].second.y());
        estimated[size_t(i) * 3 + 2] = float(tmp[size_t(i)].second.z());
      }
      normals = estimated.data();
    }

    std::vector<IndexedPwn> cloud;
    cloud.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      cloud.emplace_back(Point_3(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                         Vector_3(normals[i * 3], normals[i * 3 + 1], normals[i * 3 + 2]),
                         i);
    }

    using Point_map = CGAL::Nth_of_tuple_property_map<0, IndexedPwn>;
    using Normal_map = CGAL::Nth_of_tuple_property_map<1, IndexedPwn>;
    using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel,
                                                                   std::vector<IndexedPwn>,
                                                                   Point_map,
                                                                   Normal_map>;
    using Efficient_ransac = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
    using Plane_shape = CGAL::Shape_detection::Plane<Traits>;

    Efficient_ransac ransac;
    ransac.set_input(cloud, Point_map(), Normal_map());
    ransac.template add_shape_factory<Plane_shape>();

    Efficient_ransac::Parameters params;
    params.min_points = std::size_t(std::max(3, min_points > 0 ? min_points : n / 50));
    if (ransac_epsilon > 0.0) {
      params.epsilon = ransac_epsilon;
    }
    else {
      params.epsilon = epsilon;
    }
    if (ransac_cluster_epsilon > 0.0) {
      params.cluster_epsilon = ransac_cluster_epsilon;
    }
    else {
      params.cluster_epsilon = epsilon * 2.0;
    }
    params.normal_threshold = 0.9;
    ransac.detect(params);

    std::vector<Plane_3> planes;
    std::vector<int> plane_index(size_t(n), -1);
    int sid = 0;
    for (const auto &shape_ptr : ransac.shapes()) {
      auto plane_shape = std::dynamic_pointer_cast<Plane_shape>(shape_ptr);
      if (!plane_shape) {
        continue;
      }
      /* Efficient RANSAC Plane converts to Kernel::Plane_3. */
      const Plane_3 pl = static_cast<Plane_3>(*plane_shape);
      planes.push_back(pl);
      for (const std::size_t idx : shape_ptr->indices_of_assigned_points()) {
        if (idx >= cloud.size()) {
          continue;
        }
        const int orig = std::get<2>(cloud[idx]);
        if (orig >= 0 && orig < n) {
          plane_index[size_t(orig)] = sid;
        }
      }
      sid++;
    }
    if (planes.empty()) {
      error = "Structure Point Set: no planes detected (try larger Epsilon or fewer Min Points)";
      return false;
    }

    /* Point range as Pwn for structure_point_set. */
    std::vector<Pwn> pwns;
    pwns.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      const Point_3 pt = std::get<0>(cloud[size_t(i)]);
      const Vector_3 nm = std::get<1>(cloud[size_t(i)]);
      pwns.emplace_back(pt, nm);
    }

    std::vector<std::pair<Point_3, Vector_3>> structured;
    const double attr = (attraction_factor > 0.0) ? attraction_factor : 3.0;
    CGAL::structure_point_set(
        pwns,
        planes,
        std::back_inserter(structured),
        epsilon,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
            .normal_map(CGAL::Second_of_pair_property_map<Pwn>())
            .plane_map(CGAL::Identity_property_map<Plane_3>())
            .plane_index_map(CGAL::make_property_map(plane_index))
            .attraction_factor(attr));

    if (structured.empty()) {
      error = "Structure Point Set produced no points";
      return false;
    }
    out_xyz.resize(structured.size() * 3);
    out_nxyz.resize(structured.size() * 3);
    for (std::size_t i = 0; i < structured.size(); i++) {
      out_xyz[i * 3 + 0] = float(structured[i].first.x());
      out_xyz[i * 3 + 1] = float(structured[i].first.y());
      out_xyz[i * 3 + 2] = float(structured[i].first.z());
      out_nxyz[i * 3 + 0] = float(structured[i].second.x());
      out_nxyz[i * 3 + 1] = float(structured[i].second.y());
      out_nxyz[i * 3 + 2] = float(structured[i].second.z());
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Structure Point Set failed";
    return false;
  }
}

MeshResult points_scale_space_reconstruct_ex(const float *positions,
                                             int n,
                                             int iterations,
                                             int /*smoother*/,
                                             int /*mesher*/)
{
  /* Always use the original CGAL default path:
   *   increase_scale(iters)  — Weighted PCA + stores squared_radius for Alpha mesher
   *   reconstruct_surface()  — Alpha_shape_mesher(squared_radius)
   * Exposing Jet / Advancing Front without wiring squared_radius made meshes unusable. */
  MeshResult result;
  if (!positions || n < 4) {
    result.error = "Scale Space needs at least 4 points";
    return result;
  }
  try {
    using Reconstruction = CGAL::Scale_space_surface_reconstruction_3<Kernel>;
    std::vector<Point_3> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);
    }
    Reconstruction reconstruct(pts.begin(), pts.end());
    const std::size_t iters = std::size_t(std::max(1, iterations));
    reconstruct.increase_scale(iters);
    reconstruct.reconstruct_surface();

    /* Interpolating mesh: original point positions + connectivity at current scale. */
    result.positions.resize(size_t(n) * 3);
    for (int i = 0; i < n; i++) {
      result.positions[size_t(i) * 3 + 0] = positions[i * 3 + 0];
      result.positions[size_t(i) * 3 + 1] = positions[i * 3 + 1];
      result.positions[size_t(i) * 3 + 2] = positions[i * 3 + 2];
    }
    result.corner_verts.clear();
    result.corner_verts.reserve(reconstruct.number_of_facets() * 3);
    for (auto it = reconstruct.facets_begin(); it != reconstruct.facets_end(); ++it) {
      const auto &f = *it;
      if (f[0] >= std::size_t(n) || f[1] >= std::size_t(n) || f[2] >= std::size_t(n)) {
        continue;
      }
      result.corner_verts.push_back(int(f[0]));
      result.corner_verts.push_back(int(f[1]));
      result.corner_verts.push_back(int(f[2]));
    }
    if (result.corner_verts.empty()) {
      result.error = "Scale space produced no facets (try more points or different iterations)";
      result.ok = false;
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "CGAL scale-space reconstruction failed";
    result.ok = false;
  }
  return result;
}

}  // namespace blender::cgal_bridge
