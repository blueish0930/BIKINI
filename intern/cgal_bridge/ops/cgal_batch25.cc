/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 25:
 *  - Union of balls mesh (no shrink; pure ball union surface)
 */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#include <CGAL/Polyhedron_3.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/make_union_of_balls_3.h>

#include <algorithm>
#include <cmath>
#include <list>

namespace blender::cgal_bridge {
namespace {

using Point = Point_3;
using Weighted_point = Kernel::Weighted_point_3;
using Polyhedron = CGAL::Polyhedron_3<Kernel>;

}  // namespace

MeshResult points_union_of_balls(const float *positions,
                                 const float *radii,
                                 int n,
                                 int subdivisions)
{
  MeshResult result;
  if (!positions || n < 2) {
    result.error = "Union of Balls needs at least 2 points";
    return result;
  }
  try {
    std::list<Weighted_point> balls;
    for (int i = 0; i < n; i++) {
      const float r = radii ? radii[i] : 1.0f;
      const double rr = double(std::max(r, 1e-6f));
      balls.emplace_back(Point(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]),
                         rr * rr);
    }
    const int subdiv = std::clamp(subdivisions, 0, 2);
    Polyhedron poly;
    CGAL::make_union_of_balls_mesh_3(poly, balls.begin(), balls.end(), subdiv);
    if (poly.empty()) {
      result.error = "Union of Balls produced empty mesh (try larger radii)";
      return result;
    }
    Surface_mesh sm;
    CGAL::copy_face_graph(poly, sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "Union of Balls failed";
    result.ok = false;
  }
  return result;
}

}  // namespace blender::cgal_bridge
