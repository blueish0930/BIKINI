/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 26 — new only (not previously deleted / already existing):
 *  - Variational Shape Approximation (VSA) mesh approximation
 *  - Min Circle 2D (XY / XZ / YZ projection)
 *  - Min Rectangle 2D (oriented min area rectangle on projection)
 */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Min_circle_2.h>
#include <CGAL/Min_circle_2_traits_2.h>
#include <CGAL/Surface_mesh_approximation/approximate_triangle_mesh.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/min_quadrilateral_2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace blender::cgal_bridge {
namespace {

using Point = Point_3;
using Point_2 = Kernel::Point_2;

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  if (sm.number_of_faces() == 0) {
    error = "Empty mesh after triangulation";
    return false;
  }
  return true;
}

/** Project 3D point to 2D: plane 0=XY, 1=XZ, 2=YZ. */
static Point_2 project_pt(const float *xyz, int plane)
{
  const double x = double(xyz[0]);
  const double y = double(xyz[1]);
  const double z = double(xyz[2]);
  switch (plane) {
    case 1:
      return Point_2(x, z);
    case 2:
      return Point_2(y, z);
    default:
      return Point_2(x, y);
  }
}

/** Lift 2D back to 3D with dropped axis at 0 (or mid_z for centering). */
static void unproject(double a, double b, int plane, float mid_drop, float out[3])
{
  switch (plane) {
    case 1:
      out[0] = float(a);
      out[1] = mid_drop;
      out[2] = float(b);
      break;
    case 2:
      out[0] = mid_drop;
      out[1] = float(a);
      out[2] = float(b);
      break;
    default:
      out[0] = float(a);
      out[1] = float(b);
      out[2] = mid_drop;
      break;
  }
}

static float mid_dropped_axis(const float *positions, int n, int plane)
{
  if (!positions || n <= 0) {
    return 0.0f;
  }
  const int axis = (plane == 0) ? 2 : (plane == 1) ? 1 : 0;
  double s = 0.0;
  for (int i = 0; i < n; i++) {
    s += double(positions[i * 3 + axis]);
  }
  return float(s / double(n));
}

/** Flat disk as a single n-gon (no triangulation) in the projection plane. */
static void append_disk(float cx,
                        float cy,
                        float cz,
                        float radius,
                        int plane,
                        int segments,
                        MeshResult &result)
{
  segments = std::clamp(segments, 8, 128);
  result.positions.clear();
  result.corner_verts.clear();
  result.face_offsets.clear();
  for (int j = 0; j < segments; j++) {
    const double u = 2.0 * M_PI * double(j) / double(segments);
    const double ca = std::cos(u) * double(radius);
    const double sa = std::sin(u) * double(radius);
    float p[3];
    switch (plane) {
      case 1: /* XZ */
        p[0] = cx + float(ca);
        p[1] = cy;
        p[2] = cz + float(sa);
        break;
      case 2: /* YZ */
        p[0] = cx;
        p[1] = cy + float(ca);
        p[2] = cz + float(sa);
        break;
      default: /* XY */
        p[0] = cx + float(ca);
        p[1] = cy + float(sa);
        p[2] = cz;
        break;
    }
    result.positions.push_back(p[0]);
    result.positions.push_back(p[1]);
    result.positions.push_back(p[2]);
    result.corner_verts.push_back(j);
  }
  result.face_offsets = {0, segments};
  result.ok = true;
}

}  // namespace

MeshResult mesh_vsa_approximate(const MeshIn &mesh,
                                int max_proxies,
                                int iterations,
                                int seeding_method)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "VSA needs a triangle mesh" : error;
    return result;
  }
  const int nfaces = int(sm.number_of_faces());
  if (nfaces < 4) {
    result.error = "VSA needs at least 4 faces";
    return result;
  }
  try {
    max_proxies = std::clamp(max_proxies, 1, std::min(512, nfaces));
    iterations = std::clamp(iterations, 1, 200);
    seeding_method = std::clamp(seeding_method, 0, 2);

    using Face = Surface_mesh::Face_index;
    auto fproxy = sm.add_property_map<Face, std::size_t>("f:proxy", 0).first;

    std::vector<Point> anchors;
    std::vector<std::array<std::size_t, 3>> triangles;

    CGAL::Surface_mesh_approximation::Seeding_method seed =
        CGAL::Surface_mesh_approximation::HIERARCHICAL;
    if (seeding_method == 1) {
      seed = CGAL::Surface_mesh_approximation::INCREMENTAL;
    }
    else if (seeding_method == 2) {
      seed = CGAL::Surface_mesh_approximation::RANDOM;
    }

    /* approximate_triangle_mesh returns true if anchors+triangles form a manifold. */
    (void)CGAL::Surface_mesh_approximation::approximate_triangle_mesh(
        sm,
        CGAL::parameters::seeding_method(seed)
            .max_number_of_proxies(std::size_t(max_proxies))
            .number_of_iterations(std::size_t(iterations))
            .number_of_relaxations(5)
            .face_proxy_map(fproxy)
            .anchors(std::back_inserter(anchors))
            .triangles(std::back_inserter(triangles)));

    if (anchors.empty() || triangles.empty()) {
      result.error = "VSA produced empty approximation (try more proxies or a denser mesh)";
      return result;
    }

    result.positions.reserve(anchors.size() * 3);
    for (const Point &p : anchors) {
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
    }
    result.corner_verts.reserve(triangles.size() * 3);
    const int nverts = int(anchors.size());
    for (const auto &t : triangles) {
      if (int(t[0]) >= nverts || int(t[1]) >= nverts || int(t[2]) >= nverts) {
        continue;
      }
      if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) {
        continue;
      }
      result.corner_verts.push_back(int(t[0]));
      result.corner_verts.push_back(int(t[1]));
      result.corner_verts.push_back(int(t[2]));
    }
    if (result.corner_verts.size() < 3) {
      result.error = "VSA triangles degenerate after filtering";
      return result;
    }

    /* Proxy count for callers (stored in face_tag size / first element convention:
     * face_tag[0] = number of proxies when face_tag size == 1). */
    std::size_t proxy_count = 0;
    for (const Face f : sm.faces()) {
      proxy_count = std::max(proxy_count, fproxy[f] + 1);
    }
    result.face_tag.clear();
    result.face_tag.push_back(int(proxy_count));
    result.radius = float(proxy_count); /* also expose via radius for simple consumers */
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "VSA approximation failed";
    result.ok = false;
  }
  return result;
}

MeshResult points_min_circle_2(const float *positions, int n, int plane, int segments)
{
  MeshResult result;
  if (!positions || n < 2) {
    result.error = "Min Circle 2D needs at least 2 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    using Traits = CGAL::Min_circle_2_traits_2<Kernel>;
    using Min_circle = CGAL::Min_circle_2<Traits>;

    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.push_back(project_pt(positions + i * 3, plane));
    }
    Min_circle mc(pts.begin(), pts.end(), true /* randomize */);
    if (mc.is_empty()) {
      result.error = "Min Circle 2D is empty";
      return result;
    }
    const auto &c = mc.circle();
    const Point_2 &ctr = c.center();
    const double r = std::sqrt(CGAL::to_double(c.squared_radius()));
    const float mid = mid_dropped_axis(positions, n, plane);
    float center3[3];
    unproject(CGAL::to_double(ctr.x()), CGAL::to_double(ctr.y()), plane, mid, center3);
    result.center[0] = center3[0];
    result.center[1] = center3[1];
    result.center[2] = center3[2];
    result.radius = float(r);
    append_disk(center3[0], center3[1], center3[2], float(r), plane, segments, result);
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "Min Circle 2D failed";
    result.ok = false;
  }
  return result;
}

MeshResult points_min_rectangle_2(const float *positions, int n, int plane)
{
  MeshResult result;
  if (!positions || n < 2) {
    result.error = "Min Rectangle 2D needs at least 2 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.push_back(project_pt(positions + i * 3, plane));
    }
    std::vector<Point_2> hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
    if (hull.size() < 2) {
      result.error = "Min Rectangle 2D: degenerate point set";
      return result;
    }
    /* Degenerate hull: expand to a tiny axis-aligned box. */
    if (hull.size() == 2) {
      const Point_2 &a = hull[0];
      const Point_2 &b = hull[1];
      const double dx = CGAL::to_double(b.x() - a.x());
      const double dy = CGAL::to_double(b.y() - a.y());
      const double len = std::sqrt(dx * dx + dy * dy);
      const double nx = (len > 1e-12) ? (-dy / len) * 1e-4 : 1e-4;
      const double ny = (len > 1e-12) ? (dx / len) * 1e-4 : 0.0;
      hull.clear();
      hull.emplace_back(a.x() - nx, a.y() - ny);
      hull.emplace_back(b.x() - nx, b.y() - ny);
      hull.emplace_back(b.x() + nx, b.y() + ny);
      hull.emplace_back(a.x() + nx, a.y() + ny);
    }
    else if (hull.size() == 3) {
      /* Triangle is already convex; min_rectangle_2 accepts it. */
    }

    std::vector<Point_2> rect;
    rect.reserve(4);
    CGAL::min_rectangle_2(hull.begin(), hull.end(), std::back_inserter(rect));
    if (rect.size() < 4) {
      result.error = "Min Rectangle 2D produced fewer than 4 corners";
      return result;
    }

    const float mid = mid_dropped_axis(positions, n, plane);
    result.positions.reserve(12);
    double cx = 0.0, cy = 0.0;
    for (int i = 0; i < 4; i++) {
      float p[3];
      unproject(CGAL::to_double(rect[i].x()),
                CGAL::to_double(rect[i].y()),
                plane,
                mid,
                p);
      result.positions.push_back(p[0]);
      result.positions.push_back(p[1]);
      result.positions.push_back(p[2]);
      cx += CGAL::to_double(rect[i].x());
      cy += CGAL::to_double(rect[i].y());
    }
    cx *= 0.25;
    cy *= 0.25;
    float center3[3];
    unproject(cx, cy, plane, mid, center3);
    result.center[0] = center3[0];
    result.center[1] = center3[1];
    result.center[2] = center3[2];

    /* Single n-gon quad (CCW from min_rectangle_2). */
    result.corner_verts = {0, 1, 2, 3};
    result.face_offsets = {0, 4};
    /* Approximate half-diagonal as "radius" for convenience. */
    double max_d2 = 0.0;
    for (int i = 0; i < 4; i++) {
      const double dx = CGAL::to_double(rect[i].x()) - cx;
      const double dy = CGAL::to_double(rect[i].y()) - cy;
      max_d2 = std::max(max_d2, dx * dx + dy * dy);
    }
    result.radius = float(std::sqrt(max_d2));
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "Min Rectangle 2D failed";
    result.ok = false;
  }
  return result;
}

}  // namespace blender::cgal_bridge
