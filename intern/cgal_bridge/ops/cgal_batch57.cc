/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 57: marching cubes isosurface, extreme_point_3,
 * 3D barycentric coordinates.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Barycentric_coordinates_3.h>
#include <CGAL/Isosurfacing_3/Cartesian_grid_3.h>
#include <CGAL/Isosurfacing_3/Interpolated_discrete_values_3.h>
#include <CGAL/Isosurfacing_3/Marching_cubes_domain_3.h>
#include <CGAL/Isosurfacing_3/marching_cubes_3.h>
#include <CGAL/Named_function_parameters.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/convexity_check_3.h>
#include <CGAL/extreme_point_3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace blender::cgal_bridge {
namespace {

using Point_3 = Kernel::Point_3;

}  // namespace

MeshResult mesh_marching_cubes_grid(const float *sdf,
                                    int nx,
                                    int ny,
                                    int nz,
                                    double x0,
                                    double y0,
                                    double z0,
                                    double dx,
                                    double dy,
                                    double dz,
                                    double isovalue,
                                    bool tcmc)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!sdf || nx < 3 || ny < 3 || nz < 3) {
    result.error = "Marching Cubes needs an SDF grid of at least 3×3×3";
    return result;
  }
  try {
    using Grid = CGAL::Isosurfacing::Cartesian_grid_3<Kernel>;
    using Values = CGAL::Isosurfacing::Interpolated_discrete_values_3<Grid>;
    using Domain = CGAL::Isosurfacing::Marching_cubes_domain_3<Grid, Values>;
    const Kernel::Iso_cuboid_3 span(Point_3(x0, y0, z0),
                                    Point_3(x0 + dx * double(nx - 1),
                                            y0 + dy * double(ny - 1),
                                            z0 + dz * double(nz - 1)));
    const std::array<std::size_t, 3> dims{std::size_t(nx), std::size_t(ny), std::size_t(nz)};
    Grid grid(span, dims);
    Values values(grid);
    for (int k = 0; k < nz; k++) {
      for (int j = 0; j < ny; j++) {
        for (int i = 0; i < nx; i++) {
          const size_t id = (size_t(k) * size_t(ny) + size_t(j)) * size_t(nx) + size_t(i);
          values(std::size_t(i), std::size_t(j), std::size_t(k)) = Kernel::FT(sdf[id]);
        }
      }
    }
    Domain domain(grid, values);
    std::vector<Point_3> pts;
    std::vector<std::vector<std::size_t>> tris;
    CGAL::Isosurfacing::marching_cubes<CGAL::Sequential_tag>(
        domain,
        Kernel::FT(isovalue),
        pts,
        tris,
        CGAL::parameters::use_topologically_correct_marching_cubes(tcmc));
    for (const Point_3 &p : pts) {
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
    }
    result.face_offsets.push_back(0);
    for (const std::vector<std::size_t> &t : tris) {
      if (t.size() < 3) {
        continue;
      }
      result.corner_verts.push_back(int(t[0]));
      result.corner_verts.push_back(int(t[1]));
      result.corner_verts.push_back(int(t[2]));
      result.face_offsets.push_back(int(result.corner_verts.size()));
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Isosurface is empty (the SDF field never crosses Isovalue in this box)";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Marching Cubes failed";
  }
  return result;
}

bool points_extreme_point_3(const float *positions,
                            int n,
                            float dx,
                            float dy,
                            float dz,
                            float out_xyz[3],
                            std::string &error)
{
  CgalThrowGuard guard;
  if (!positions || n < 1 || !out_xyz) {
    error = "Extreme Point 3 needs at least 1 point";
    return false;
  }
  if (dx == 0.0f && dy == 0.0f && dz == 0.0f) {
    error = "Extreme Point 3 needs a non-zero Direction";
    return false;
  }
  try {
    std::vector<Point_3> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.push_back(Point_3(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]));
    }
    const Kernel::Direction_3 dir{double(dx), double(dy), double(dz)};
    const Point_3 p = CGAL::extreme_point_3(pts, dir);
    out_xyz[0] = float(CGAL::to_double(p.x()));
    out_xyz[1] = float(CGAL::to_double(p.y()));
    out_xyz[2] = float(CGAL::to_double(p.z()));
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Extreme Point 3 failed";
    return false;
  }
}

bool mesh_barycentric_3(const MeshIn &cage,
                        const float *query,
                        int n_query,
                        int method,
                        std::vector<float> &weights,
                        int &n_cage,
                        std::string &error)
{
  CgalThrowGuard guard;
  weights.clear();
  n_cage = 0;
  if (!query || n_query < 1) {
    error = "Barycentric 3 needs query points";
    return false;
  }
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(cage, sm, error, true)) {
    if (error.empty()) {
      error = "Barycentric 3 needs a closed convex triangle mesh";
    }
    return false;
  }
  triangulate_faces_keep_ids(sm);
  sm.collect_garbage();
  if (sm.number_of_vertices() < 4 || sm.number_of_faces() < 4) {
    error = "Barycentric 3 needs a closed triangle mesh with at least 4 vertices";
    return false;
  }
  if (!CGAL::is_closed(sm)) {
    error = "Barycentric 3 needs a closed triangle mesh";
    return false;
  }
  n_cage = int(sm.number_of_vertices());
  if (int64_t(n_query) * int64_t(n_cage) > 4000000) {
    error = "Barycentric 3 weight table is too large (query × cage verts). Reduce either count.";
    return false;
  }
  const int mode = std::max(0, std::min(2, method));
  if (mode != 1 && !CGAL::is_strongly_convex_3(sm)) {
    error = "Wachspress and Discrete Harmonic need a strongly convex cage. Use Mean Value, or convexify the cage.";
    return false;
  }
  try {
    using WP = CGAL::Barycentric_coordinates::Wachspress_coordinates_3<Surface_mesh>;
    using MV = CGAL::Barycentric_coordinates::Mean_value_coordinates_3<Surface_mesh>;
    using DH = CGAL::Barycentric_coordinates::Discrete_harmonic_coordinates_3<Surface_mesh>;
    const auto policy = CGAL::Barycentric_coordinates::Computation_policy_3::FAST_WITH_EDGE_CASES;
    weights.assign(size_t(n_query) * size_t(n_cage), 0.0f);
    auto dump = [&](auto &coord_fn) {
      std::vector<Kernel::FT> coord;
      coord.reserve(size_t(n_cage));
      for (int q = 0; q < n_query; q++) {
        const Point_3 p(query[q * 3 + 0], query[q * 3 + 1], query[q * 3 + 2]);
        coord.clear();
        coord_fn(p, std::back_inserter(coord));
        const int nout = std::min(n_cage, int(coord.size()));
        for (int i = 0; i < nout; i++) {
          weights[size_t(q) * size_t(n_cage) + size_t(i)] = float(CGAL::to_double(coord[size_t(i)]));
        }
      }
    };
    if (mode == 1) {
      MV mv_coord(sm, policy);
      dump(mv_coord);
    }
    else if (mode == 2) {
      DH dh_coord(sm, policy);
      dump(dh_coord);
    }
    else {
      WP wp_coord(sm, policy);
      dump(wp_coord);
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Barycentric 3 failed";
    return false;
  }
}

}  // namespace blender::cgal_bridge
