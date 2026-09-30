/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#include <CGAL/convex_hull_3.h>

#include <vector>

namespace blender::cgal_bridge {

TriangleMeshResult convex_hull_3(const float *positions, int points_num)
{
  TriangleMeshResult result;
  if (!positions || points_num < 4) {
    result.error = "Need at least 4 points for a 3D convex hull";
    return result;
  }

  try {
    std::vector<Point_3> cgal_pts;
    cgal_pts.reserve(size_t(points_num));
    for (int i = 0; i < points_num; i++) {
      const float *p = positions + i * 3;
      cgal_pts.emplace_back(p[0], p[1], p[2]);
    }

    Surface_mesh sm;
    CGAL::convex_hull_3(cgal_pts.begin(), cgal_pts.end(), sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "CGAL convex_hull_3 failed";
    result.ok = false;
  }
  return result;
}

}  // namespace blender::cgal_bridge
