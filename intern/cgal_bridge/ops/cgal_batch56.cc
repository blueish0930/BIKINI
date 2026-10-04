/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 56: CGAL 6.2 approximate convex decomposition.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Named_function_parameters.h>
#include <CGAL/approximate_convex_decomposition.h>
#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <array>
#include <utility>
#include <vector>

namespace blender::cgal_bridge {
namespace {

using Point_3 = Kernel::Point_3;

static void append_soup(MeshResult &result,
                        const std::vector<Point_3> &pts,
                        const std::vector<std::array<unsigned int, 3>> &tris,
                        int tag)
{
  const int base = result.verts_num();
  for (const Point_3 &p : pts) {
    result.positions.push_back(float(CGAL::to_double(p.x())));
    result.positions.push_back(float(CGAL::to_double(p.y())));
    result.positions.push_back(float(CGAL::to_double(p.z())));
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  for (const auto &t : tris) {
    result.corner_verts.push_back(base + int(t[0]));
    result.corner_verts.push_back(base + int(t[1]));
    result.corner_verts.push_back(base + int(t[2]));
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_tag.push_back(tag);
  }
}

}  // namespace

MeshResult mesh_approximate_convex_decomposition(const MeshIn &mesh, int count, int resolution)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    result.error = error.empty() ? "Approximate Convex Decomposition needs a mesh" : error;
    return result;
  }
  triangulate_faces_keep_ids(sm);
  sm.collect_garbage();
  if (!CGAL::is_closed(sm)) {
    result.error = "Approximate Convex Decomposition needs a closed triangle mesh";
    return result;
  }
  try {
    using Soup = std::pair<std::vector<Point_3>, std::vector<std::array<unsigned int, 3>>>;
    std::vector<Soup> volumes;
    const unsigned int nvol = unsigned(std::max(1, std::min(256, count)));
    /* Longest bbox axis gets `resolution` cells; the other two axes scale with
     * bbox extent (CGAL calculate_grid_size uses cbrt of the voxel budget). */
    const int res = std::max(8, std::min(100, resolution));
    const unsigned int nvox = unsigned(res) * unsigned(res) * unsigned(res);
    CGAL::approximate_convex_decomposition(
        sm,
        std::back_inserter(volumes),
        CGAL::parameters::maximum_number_of_convex_volumes(nvol)
            .maximum_number_of_voxels(nvox)
            .refitting(false)
            .split_at_concavity(false)
            .maximum_depth(8u));
    for (size_t i = 0; i < volumes.size(); i++) {
      append_soup(result, volumes[i].first, volumes[i].second, int(i));
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Approximate Convex Decomposition produced no volumes";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Approximate Convex Decomposition failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
