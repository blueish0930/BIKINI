/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 31 — new only (not previously deleted / already existing):
 *  - OTR Reconstruct 2D
 *  - Regular Triangulation 2D
 *  - Power Diagram 2D
 *  - Voronoi 3D
 *  - Visibility 2D
 *  - Refine Mesh 2D (Mesh_2)
 *  - Alpha Complex 3D (G09)
 *  - Convex Minkowski Sum 3D (G08 light: hull(A)+hull(B))
 *
 * Skipped: P22 Registration (OpenGR), R08 deprecated stub, G10 Kinetic (EPECK),
 * S11/S12 user-deleted, previously banned measure/query nodes.
 */

#include "cgal_bridge.hh"
#include "cgal_convex_clip3.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_walk_along_line_point_location.h>
#include <CGAL/Rotational_sweep_visibility_2.h>
#include <CGAL/assertions_behaviour.h>
#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Optimal_transportation_reconstruction_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Regular_triangulation_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_data_structure_3.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/mark_domain_in_triangulation.h>

#include <boost/property_map/property_map.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <variant>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;

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
  double s = 0.0;
  for (int i = 0; i < n; i++) {
    s += double(positions[i * 3 + (plane == 0 ? 2 : (plane == 1 ? 1 : 0))]);
  }
  return float(s / double(n));
}

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  return sm.number_of_faces() > 0;
}

static double signed_area_2(const std::vector<Point_2> &loop)
{
  double a = 0.0;
  const size_t n = loop.size();
  for (size_t i = 0; i < n; i++) {
    const Point_2 &p = loop[i];
    const Point_2 &q = loop[(i + 1) % n];
    a += CGAL::to_double(p.x() * q.y() - q.x() * p.y());
  }
  return 0.5 * a;
}

static std::vector<Point_2> clean_loop_2(const std::vector<Point_2> &raw)
{
  std::vector<Point_2> cleaned;
  cleaned.reserve(raw.size());
  for (const Point_2 &p : raw) {
    if (cleaned.empty()) {
      cleaned.push_back(p);
      continue;
    }
    const double dx = CGAL::to_double(p.x() - cleaned.back().x());
    const double dy = CGAL::to_double(p.y() - cleaned.back().y());
    if (dx * dx + dy * dy > 1e-24) {
      cleaned.push_back(p);
    }
  }
  if (cleaned.size() >= 2) {
    const double dx = CGAL::to_double(cleaned.front().x() - cleaned.back().x());
    const double dy = CGAL::to_double(cleaned.front().y() - cleaned.back().y());
    if (dx * dx + dy * dy < 1e-24) {
      cleaned.pop_back();
    }
  }
  return cleaned;
}

static Point_2 poly_centroid(const std::vector<Point_2> &loop)
{
  double cx = 0.0, cy = 0.0;
  for (const Point_2 &p : loop) {
    cx += CGAL::to_double(p.x());
    cy += CGAL::to_double(p.y());
  }
  const double inv = loop.empty() ? 1.0 : 1.0 / double(loop.size());
  return Point_2(cx * inv, cy * inv);
}

static bool point_in_poly(const Point_2 &q, const std::vector<Point_2> &poly)
{
  bool inside = false;
  const size_t n = poly.size();
  if (n < 3) {
    return false;
  }
  const double qx = CGAL::to_double(q.x());
  const double qy = CGAL::to_double(q.y());
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = CGAL::to_double(poly[i].x());
    const double yi = CGAL::to_double(poly[i].y());
    const double xj = CGAL::to_double(poly[j].x());
    const double yj = CGAL::to_double(poly[j].y());
    const bool intersect = ((yi > qy) != (yj > qy)) &&
                           (qx < (xj - xi) * (qy - yi) / ((yj - yi) + 1e-30) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

static bool loop_inside_loop(const std::vector<Point_2> &inner, const std::vector<Point_2> &outer)
{
  if (inner.size() < 3 || outer.size() < 3) {
    return false;
  }
  int hits = 0;
  int tests = 0;
  auto test = [&](const Point_2 &q) {
    tests++;
    if (point_in_poly(q, outer)) {
      hits++;
    }
  };
  test(poly_centroid(inner));
  const size_t step = std::max(size_t(1), inner.size() / 8);
  for (size_t i = 0; i < inner.size(); i += step) {
    test(inner[i]);
  }
  return tests > 0 && hits * 2 >= tests;
}

struct PolyIsland {
  std::vector<Point_2> outer;
  std::vector<std::vector<Point_2>> holes;
};

static bool extract_polygon_islands(const MeshIn &mesh,
                                    int plane,
                                    std::vector<PolyIsland> &islands,
                                    float &mid_drop,
                                    std::string &error)
{
  islands.clear();
  if (!mesh.positions || mesh.verts_num < 3) {
    error = "Need at least 3 vertices";
    return false;
  }
  mid_drop = mid_dropped_axis(mesh.positions, mesh.verts_num, plane);

  struct Loop {
    std::vector<Point_2> verts;
    double area_abs = 0.0;
  };

  auto make_loop = [](std::vector<Point_2> verts) -> Loop {
    Loop L;
    L.verts = clean_loop_2(verts);
    L.area_abs = std::abs(signed_area_2(L.verts));
    return L;
  };

  Surface_mesh sm;
  std::string load_err;
  std::vector<Loop> loops;
  if (load_tri(mesh, sm, load_err) && sm.number_of_faces() > 0) {
    std::vector<Surface_mesh::Halfedge_index> cycle_starts;
    CGAL::extract_boundary_cycles(sm, std::back_inserter(cycle_starts));
    for (auto start : cycle_starts) {
      std::vector<Point_2> raw;
      for (auto h : CGAL::halfedges_around_face(start, sm)) {
        const Point_3 &p = sm.point(sm.source(h));
        float xyz[3] = {float(CGAL::to_double(p.x())),
                        float(CGAL::to_double(p.y())),
                        float(CGAL::to_double(p.z()))};
        raw.push_back(project_pt(xyz, plane));
      }
      Loop L = make_loop(std::move(raw));
      if (L.verts.size() >= 3 && L.area_abs > 1e-20) {
        loops.push_back(std::move(L));
      }
    }
  }

  if (!loops.empty()) {
    std::sort(loops.begin(), loops.end(), [](const Loop &a, const Loop &b) {
      return a.area_abs > b.area_abs;
    });
    std::vector<char> used(loops.size(), 0);
    for (size_t oi = 0; oi < loops.size(); oi++) {
      if (used[oi]) {
        continue;
      }
      used[oi] = 1;
      PolyIsland isl;
      isl.outer = loops[oi].verts;
      if (signed_area_2(isl.outer) < 0.0) {
        std::reverse(isl.outer.begin(), isl.outer.end());
      }
      for (size_t hi = oi + 1; hi < loops.size(); hi++) {
        if (used[hi] || !loop_inside_loop(loops[hi].verts, isl.outer)) {
          continue;
        }
        used[hi] = 1;
        std::vector<Point_2> hole = loops[hi].verts;
        if (signed_area_2(hole) > 0.0) {
          std::reverse(hole.begin(), hole.end());
        }
        isl.holes.push_back(std::move(hole));
      }
      islands.push_back(std::move(isl));
    }
  }

  if (islands.empty()) {
    std::vector<Point_2> pts;
    pts.reserve(size_t(mesh.verts_num));
    for (int i = 0; i < mesh.verts_num; i++) {
      pts.push_back(project_pt(mesh.positions + i * 3, plane));
    }
    std::vector<Point_2> hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
    Loop L = make_loop(std::move(hull));
    if (L.verts.size() < 3) {
      error = load_err.empty() ? "No polygon border / hull" : load_err;
      return false;
    }
    PolyIsland isl;
    isl.outer = std::move(L.verts);
    if (signed_area_2(isl.outer) < 0.0) {
      std::reverse(isl.outer.begin(), isl.outer.end());
    }
    islands.push_back(std::move(isl));
  }
  return !islands.empty();
}

static Point_2 guaranteed_interior(const std::vector<Point_2> &outer)
{
  const size_t n = outer.size();
  for (size_t i = 0; i < n; i++) {
    const Point_2 &a = outer[(i + n - 1) % n];
    const Point_2 &b = outer[i];
    const Point_2 &c = outer[(i + 1) % n];
    const double ax = CGAL::to_double(a.x());
    const double ay = CGAL::to_double(a.y());
    const double bx = CGAL::to_double(b.x());
    const double by = CGAL::to_double(b.y());
    const double cx = CGAL::to_double(c.x());
    const double cy = CGAL::to_double(c.y());
    const double cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (cross > 1e-18) {
      return Point_2((ax + bx + cx) / 3.0, (ay + by + cy) / 3.0);
    }
  }
  return poly_centroid(outer);
}

static void append_tri_2(MeshResult &result,
                         const Point_2 &a,
                         const Point_2 &b,
                         const Point_2 &c,
                         int plane,
                         float mid)
{
  const int base = result.verts_num();
  float qa[3], qb[3], qc[3];
  unproject(CGAL::to_double(a.x()), CGAL::to_double(a.y()), plane, mid, qa);
  unproject(CGAL::to_double(b.x()), CGAL::to_double(b.y()), plane, mid, qb);
  unproject(CGAL::to_double(c.x()), CGAL::to_double(c.y()), plane, mid, qc);
  result.positions.insert(result.positions.end(), {qa[0], qa[1], qa[2]});
  result.positions.insert(result.positions.end(), {qb[0], qb[1], qb[2]});
  result.positions.insert(result.positions.end(), {qc[0], qc[1], qc[2]});
  result.corner_verts.push_back(base);
  result.corner_verts.push_back(base + 1);
  result.corner_verts.push_back(base + 2);
}

static void fan_fill_ring(MeshResult &result,
                          const std::vector<Point_2> &ring,
                          int plane,
                          float mid)
{
  if (ring.size() < 3) {
    return;
  }
  for (size_t i = 1; i + 1 < ring.size(); i++) {
    append_tri_2(result, ring[0], ring[i], ring[i + 1], plane, mid);
  }
}

static void emit_wire_seg(WireResult &result,
                          const Point_2 &a,
                          const Point_2 &b,
                          int plane,
                          float mid)
{
  float qa[3], qb[3];
  unproject(CGAL::to_double(a.x()), CGAL::to_double(a.y()), plane, mid, qa);
  unproject(CGAL::to_double(b.x()), CGAL::to_double(b.y()), plane, mid, qb);
  const int ia = int(result.positions.size() / 3);
  result.positions.insert(result.positions.end(), {qa[0], qa[1], qa[2]});
  const int ib = int(result.positions.size() / 3);
  result.positions.insert(result.positions.end(), {qb[0], qb[1], qb[2]});
  result.edge_v0.push_back(ia);
  result.edge_v1.push_back(ib);
}

static void emit_welded_segments(WireResult &result,
                                 const std::vector<std::pair<Point_2, Point_2>> &segs,
                                 int plane,
                                 float mid)
{
  if (segs.empty()) {
    return;
  }
  double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
  for (const auto &s : segs) {
    for (const Point_2 *p : {&s.first, &s.second}) {
      const double x = CGAL::to_double(p->x());
      const double y = CGAL::to_double(p->y());
      minx = std::min(minx, x);
      miny = std::min(miny, y);
      maxx = std::max(maxx, x);
      maxy = std::max(maxy, y);
    }
  }
  const double diag = std::max(1e-12, std::hypot(maxx - minx, maxy - miny));
  /* Snap ≈ 1e-4 of bbox so coincident OTR endpoints become one vertex. */
  const double scale = 1.0e4 / diag;

  std::map<std::array<long long, 2>, int> weld;
  auto vid = [&](const Point_2 &p) -> int {
    const double x = CGAL::to_double(p.x());
    const double y = CGAL::to_double(p.y());
    const std::array<long long, 2> key = {std::llround(x * scale), std::llround(y * scale)};
    const auto it = weld.find(key);
    if (it != weld.end()) {
      return it->second;
    }
    float q[3];
    unproject(x, y, plane, mid, q);
    const int idx = int(result.positions.size() / 3);
    result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
    weld.emplace(key, idx);
    return idx;
  };

  std::set<std::pair<int, int>> seen;
  for (const auto &s : segs) {
    int i = vid(s.first);
    int j = vid(s.second);
    if (i == j) {
      continue;
    }
    if (i > j) {
      std::swap(i, j);
    }
    if (!seen.insert({i, j}).second) {
      continue;
    }
    result.edge_v0.push_back(i);
    result.edge_v1.push_back(j);
  }
}

static void emit_wire_seg3(WireResult &result, const Point_3 &a, const Point_3 &b)
{
  const int ia = int(result.positions.size() / 3);
  result.positions.push_back(float(CGAL::to_double(a.x())));
  result.positions.push_back(float(CGAL::to_double(a.y())));
  result.positions.push_back(float(CGAL::to_double(a.z())));
  const int ib = int(result.positions.size() / 3);
  result.positions.push_back(float(CGAL::to_double(b.x())));
  result.positions.push_back(float(CGAL::to_double(b.y())));
  result.positions.push_back(float(CGAL::to_double(b.z())));
  result.edge_v0.push_back(ia);
  result.edge_v1.push_back(ib);
}

static void unique_points_3(const MeshIn &mesh, std::vector<Point_3> &pts)
{
  pts.clear();
  if (!mesh.positions || mesh.verts_num <= 0) {
    return;
  }
  std::set<std::array<long long, 3>> seen;
  const double scale = 1e7;
  pts.reserve(size_t(mesh.verts_num));
  for (int i = 0; i < mesh.verts_num; i++) {
    const double x = double(mesh.positions[i * 3 + 0]);
    const double y = double(mesh.positions[i * 3 + 1]);
    const double z = double(mesh.positions[i * 3 + 2]);
    const std::array<long long, 3> key = {std::llround(x * scale),
                                          std::llround(y * scale),
                                          std::llround(z * scale)};
    if (seen.insert(key).second) {
      pts.emplace_back(x, y, z);
    }
  }
}

}  // namespace

WireResult points_otr_reconstruct_2(const float *positions,
                                    int n,
                                    int plane,
                                    double keep_percent,
                                    int relocation)
{
  WireResult result;
  if (!positions || n < 3) {
    result.error = "OTR Reconstruct 2D needs at least 3 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  keep_percent = std::clamp(keep_percent, 1.0, 100.0);
  relocation = std::clamp(relocation, 0, 8);
  try {
    std::vector<Point_2> pts;
    pts.reserve(size_t(std::min(n, 8000)));
    std::set<std::array<long long, 2>> seen_in;
    const int stride = n > 8000 ? ((n + 7999) / 8000) : 1;
    for (int i = 0; i < n; i += stride) {
      const Point_2 p = project_pt(positions + i * 3, plane);
      const std::array<long long, 2> key = {std::llround(CGAL::to_double(p.x()) * 1e7),
                                            std::llround(CGAL::to_double(p.y()) * 1e7)};
      if (seen_in.insert(key).second) {
        pts.push_back(p);
      }
    }
    if (pts.size() < 3) {
      result.error = "OTR Reconstruct 2D degenerate";
      return result;
    }
    CGAL::Optimal_transportation_reconstruction_2<Kernel> otr(pts);
    otr.set_verbose(0);
    otr.set_use_flip(true);
    /* Relocation slides Steiner vertices after every collapse. On a closed
     * contour that usually tears the loop into scraps — leave at 0. */
    otr.set_relocation(unsigned(relocation));
    otr.set_relevance(1);
    int keep = int(std::lround(double(pts.size()) * keep_percent / 100.0));
    keep = std::clamp(keep, 2, int(pts.size()));
    otr.run_until(std::size_t(keep));
    /* Do not call relocate_all_points() — it reassigns mass and drops edges. */

    std::vector<Point_2> isolated;
    std::vector<Kernel::Segment_2> raw_segs;
    otr.list_output(std::back_inserter(isolated), std::back_inserter(raw_segs));
    std::vector<std::pair<Point_2, Point_2>> segs;
    segs.reserve(raw_segs.size());
    for (const Kernel::Segment_2 &s : raw_segs) {
      if (s.source() != s.target()) {
        segs.emplace_back(s.source(), s.target());
      }
    }
    const float mid = mid_dropped_axis(positions, n, plane);
    emit_welded_segments(result, segs, plane, mid);
    /* Isolated leftover samples are not part of the 1-skeleton — skip them. */
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "OTR Reconstruct 2D produced no edges";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "OTR Reconstruct 2D failed";
  }
  return result;
}

MeshResult points_regular_triangulation_2(const float *positions,
                                          const float *radii,
                                          int n,
                                          int plane)
{
  MeshResult result;
  if (!positions || n < 3) {
    result.error = "Regular Triangulation 2D needs at least 3 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    using Rt = CGAL::Regular_triangulation_2<Kernel>;
    using Wp = typename Rt::Weighted_point;
    Rt rt;
    for (int i = 0; i < n; i++) {
      const Point_2 p = project_pt(positions + i * 3, plane);
      const double r = radii ? std::max(0.0, double(radii[i])) : 0.0;
      rt.insert(Wp(p, r * r));
    }
    if (rt.dimension() != 2) {
      result.error = "Regular Triangulation 2D degenerate";
      return result;
    }
    const float mid = mid_dropped_axis(positions, n, plane);
    /* Each triangle is its own island (no shared verts with neighbors). */
    int tag = 0;
    for (auto fit = rt.finite_faces_begin(); fit != rt.finite_faces_end(); ++fit) {
      const int base = result.verts_num();
      for (int i = 0; i < 3; i++) {
        float q[3];
        const Point_2 bp = fit->vertex(i)->point().point();
        unproject(CGAL::to_double(bp.x()), CGAL::to_double(bp.y()), plane, mid, q);
        result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
        result.corner_verts.push_back(base + i);
      }
      result.face_tag.push_back(tag++);
    }
    result.ok = result.corners_num() >= 3;
    if (!result.ok) {
      result.error = "Regular Triangulation 2D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Regular Triangulation 2D failed";
  }
  return result;
}

static bool clip_seg_aabb2(double x0,
                           double y0,
                           double x1,
                           double y1,
                           double minx,
                           double miny,
                           double maxx,
                           double maxy,
                           double &ox0,
                           double &oy0,
                           double &ox1,
                           double &oy1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  double t0 = 0.0;
  double t1 = 1.0;
  auto clip = [&](double p, double q) {
    if (std::abs(p) < 1e-18) {
      return q >= -1e-18;
    }
    const double r = q / p;
    if (p < 0.0) {
      if (r > t1) {
        return false;
      }
      if (r > t0) {
        t0 = r;
      }
    }
    else {
      if (r < t0) {
        return false;
      }
      if (r < t1) {
        t1 = r;
      }
    }
    return true;
  };
  if (!clip(-dx, x0 - minx) || !clip(dx, maxx - x0) || !clip(-dy, y0 - miny) ||
      !clip(dy, maxy - y0))
  {
    return false;
  }
  ox0 = x0 + t0 * dx;
  oy0 = y0 + t0 * dy;
  ox1 = x0 + t1 * dx;
  oy1 = y0 + t1 * dy;
  return true;
}

static bool clip_ray_aabb2(double ox,
                           double oy,
                           double dx,
                           double dy,
                           double minx,
                           double miny,
                           double maxx,
                           double maxy,
                           double &hx,
                           double &hy)
{
  double t_hit = 1e300;
  auto consider = [&](double t) {
    if (!(t > 1e-12) || t >= t_hit) {
      return;
    }
    const double x = ox + t * dx;
    const double y = oy + t * dy;
    if (x >= minx - 1e-8 && x <= maxx + 1e-8 && y >= miny - 1e-8 && y <= maxy + 1e-8) {
      t_hit = t;
    }
  };
  if (std::abs(dx) > 1e-18) {
    consider((minx - ox) / dx);
    consider((maxx - ox) / dx);
  }
  if (std::abs(dy) > 1e-18) {
    consider((miny - oy) / dy);
    consider((maxy - oy) / dy);
  }
  if (!(t_hit < 1e299)) {
    return false;
  }
  hx = ox + t_hit * dx;
  hy = oy + t_hit * dy;
  return true;
}

static std::vector<Point_2> unique_points_2(std::vector<Point_2> pts, double eps2)
{
  std::vector<Point_2> out;
  out.reserve(pts.size());
  for (const Point_2 &p : pts) {
    bool dup = false;
    const double px = CGAL::to_double(p.x());
    const double py = CGAL::to_double(p.y());
    for (const Point_2 &q : out) {
      const double dx = px - CGAL::to_double(q.x());
      const double dy = py - CGAL::to_double(q.y());
      if (dx * dx + dy * dy <= eps2) {
        dup = true;
        break;
      }
    }
    if (!dup) {
      out.push_back(p);
    }
  }
  return out;
}

static std::vector<Point_2> sh_clip_aabb2(std::vector<Point_2> poly,
                                          double minx,
                                          double miny,
                                          double maxx,
                                          double maxy)
{
  auto clip_side = [](const std::vector<Point_2> &in, int side, double c) {
    std::vector<Point_2> out;
    if (in.empty()) {
      return out;
    }
    auto inside = [&](const Point_2 &p) {
      const double x = CGAL::to_double(p.x());
      const double y = CGAL::to_double(p.y());
      switch (side) {
        case 0:
          return x >= c - 1e-14;
        case 1:
          return x <= c + 1e-14;
        case 2:
          return y >= c - 1e-14;
        default:
          return y <= c + 1e-14;
      }
    };
    auto isect = [&](const Point_2 &a, const Point_2 &b) {
      const double ax = CGAL::to_double(a.x());
      const double ay = CGAL::to_double(a.y());
      const double bx = CGAL::to_double(b.x());
      const double by = CGAL::to_double(b.y());
      const double da = (side < 2) ? ax : ay;
      const double db = (side < 2) ? bx : by;
      const double t = (std::abs(db - da) < 1e-18) ? 0.0 : (c - da) / (db - da);
      return Point_2(ax + t * (bx - ax), ay + t * (by - ay));
    };
    Point_2 prev = in.back();
    bool prev_in = inside(prev);
    for (const Point_2 &cur : in) {
      const bool cur_in = inside(cur);
      if (cur_in) {
        if (!prev_in) {
          out.push_back(isect(prev, cur));
        }
        out.push_back(cur);
      }
      else if (prev_in) {
        out.push_back(isect(prev, cur));
      }
      prev = cur;
      prev_in = cur_in;
    }
    return out;
  };
  poly = clip_side(poly, 0, minx);
  poly = clip_side(poly, 1, maxx);
  poly = clip_side(poly, 2, miny);
  poly = clip_side(poly, 3, maxy);
  return poly;
}

static void emit_isolated_cell_2(MeshResult &result,
                                 std::vector<Point_2> cand,
                                 int plane,
                                 float mid,
                                 int tag,
                                 double minx,
                                 double miny,
                                 double maxx,
                                 double maxy)
{
  const double diag2 = std::max(1e-24,
                                (maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny));
  cand = unique_points_2(std::move(cand), 1e-20 * diag2);
  if (cand.size() < 3) {
    return;
  }
  std::vector<Point_2> hull;
  CGAL::convex_hull_2(cand.begin(), cand.end(), std::back_inserter(hull));
  hull = sh_clip_aabb2(std::move(hull), minx, miny, maxx, maxy);
  hull = unique_points_2(std::move(hull), 1e-20 * diag2);
  if (hull.size() < 3 || std::abs(signed_area_2(hull)) <= 1e-20) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  const int base = result.verts_num();
  for (const Point_2 &p : hull) {
    float q[3];
    unproject(CGAL::to_double(p.x()), CGAL::to_double(p.y()), plane, mid, q);
    result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
    result.corner_verts.push_back(base + int(&p - hull.data()));
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
}

static std::vector<Point_3> unique_points_3(std::vector<Point_3> pts, double eps2)
{
  std::vector<Point_3> out;
  out.reserve(pts.size());
  for (const Point_3 &p : pts) {
    bool dup = false;
    const double px = CGAL::to_double(p.x());
    const double py = CGAL::to_double(p.y());
    const double pz = CGAL::to_double(p.z());
    for (const Point_3 &q : out) {
      const double dx = px - CGAL::to_double(q.x());
      const double dy = py - CGAL::to_double(q.y());
      const double dz = pz - CGAL::to_double(q.z());
      if (dx * dx + dy * dy + dz * dz <= eps2) {
        dup = true;
        break;
      }
    }
    if (!dup) {
      out.push_back(p);
    }
  }
  return out;
}

static Point_3 clamp_aabb3(const Point_3 &p, const double mn[3], const double mx[3])
{
  return Point_3(std::min(mx[0], std::max(mn[0], CGAL::to_double(p.x()))),
                 std::min(mx[1], std::max(mn[1], CGAL::to_double(p.y()))),
                 std::min(mx[2], std::max(mn[2], CGAL::to_double(p.z()))));
}

static Point_3 lerp3(const Point_3 &a, const Point_3 &b, double t)
{
  const double ax = CGAL::to_double(a.x());
  const double ay = CGAL::to_double(a.y());
  const double az = CGAL::to_double(a.z());
  return Point_3(ax + t * (CGAL::to_double(b.x()) - ax),
                 ay + t * (CGAL::to_double(b.y()) - ay),
                 az + t * (CGAL::to_double(b.z()) - az));
}

static bool hull_from_points3(const std::vector<Point_3> &pts, Surface_mesh &sm)
{
  sm.clear();
  if (pts.size() < 3) {
    return false;
  }
  try {
    if (pts.size() == 3) {
      const auto va = sm.add_vertex(pts[0]);
      const auto vb = sm.add_vertex(pts[1]);
      const auto vc = sm.add_vertex(pts[2]);
      return bool(sm.add_face(va, vb, vc));
    }
    CGAL::convex_hull_3(pts.begin(), pts.end(), sm);
    return sm.number_of_vertices() >= 3;
  }
  catch (...) {
    return false;
  }
}

/* Keep the halfspace n·x <= c of a convex point set (Sutherland–Hodgman 3D). */
static std::vector<Point_3> clip_convex_halfspace3(std::vector<Point_3> pts,
                                                   double nx,
                                                   double ny,
                                                   double nz,
                                                   double c)
{
  pts = unique_points_3(std::move(pts), 1e-20);
  if (pts.size() < 3) {
    return {};
  }
  Surface_mesh sm;
  if (!hull_from_points3(pts, sm)) {
    return {};
  }
  auto side = [&](const Point_3 &p) {
    return nx * CGAL::to_double(p.x()) + ny * CGAL::to_double(p.y()) +
           nz * CGAL::to_double(p.z()) - c;
  };
  const double scale = std::abs(nx) + std::abs(ny) + std::abs(nz) + std::abs(c) + 1.0;
  const double eps = 1e-10 * scale;
  std::vector<Point_3> next;
  next.reserve(sm.number_of_vertices() + sm.number_of_edges());
  for (auto v : sm.vertices()) {
    const Point_3 p = sm.point(v);
    if (side(p) <= eps) {
      next.push_back(p);
    }
  }
  for (auto e : sm.edges()) {
    const auto h = sm.halfedge(e);
    const Point_3 pa = sm.point(sm.source(h));
    const Point_3 pb = sm.point(sm.target(h));
    const double sa = side(pa);
    const double sb = side(pb);
    if ((sa > eps && sb < -eps) || (sa < -eps && sb > eps)) {
      const double t = sa / (sa - sb);
      next.push_back(lerp3(pa, pb, std::min(1.0, std::max(0.0, t))));
    }
  }
  return unique_points_3(std::move(next), 1e-20);
}

/* Convex polyhedron ∩ AABB via 6 halfspaces (keeps box corners / box-edge hits). */
static std::vector<Point_3> clip_convex_to_aabb3(std::vector<Point_3> cand,
                                                 const double mn[3],
                                                 const double mx[3])
{
  cand = clip_convex_halfspace3(std::move(cand), -1.0, 0.0, 0.0, -mn[0]);
  cand = clip_convex_halfspace3(std::move(cand), 1.0, 0.0, 0.0, mx[0]);
  cand = clip_convex_halfspace3(std::move(cand), 0.0, -1.0, 0.0, -mn[1]);
  cand = clip_convex_halfspace3(std::move(cand), 0.0, 1.0, 0.0, mx[1]);
  cand = clip_convex_halfspace3(std::move(cand), 0.0, 0.0, -1.0, -mn[2]);
  cand = clip_convex_halfspace3(std::move(cand), 0.0, 0.0, 1.0, mx[2]);
  return cand;
}

static void aabb_corners3(const double mn[3], const double mx[3], std::vector<Point_3> &out)
{
  out.clear();
  out.reserve(8);
  for (int xi = 0; xi < 2; xi++) {
    for (int yi = 0; yi < 2; yi++) {
      for (int zi = 0; zi < 2; zi++) {
        out.emplace_back(xi ? mx[0] : mn[0], yi ? mx[1] : mn[1], zi ? mx[2] : mn[2]);
      }
    }
  }
}

static void emit_isolated_hull3(MeshResult &result,
                                std::vector<Point_3> cand,
                                int tag,
                                const double mn[3],
                                const double mx[3])
{
  cand = clip_convex_to_aabb3(std::move(cand), mn, mx);
  if (cand.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  if (cand.size() == 3) {
    const int base = result.verts_num();
    for (const Point_3 &p : cand) {
      const Point_3 q = clamp_aabb3(p, mn, mx);
      result.positions.push_back(float(CGAL::to_double(q.x())));
      result.positions.push_back(float(CGAL::to_double(q.y())));
      result.positions.push_back(float(CGAL::to_double(q.z())));
    }
    result.corner_verts.push_back(base);
    result.corner_verts.push_back(base + 1);
    result.corner_verts.push_back(base + 2);
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_tag.push_back(tag);
    return;
  }
  Surface_mesh sm;
  try {
    CGAL::convex_hull_3(cand.begin(), cand.end(), sm);
  }
  catch (...) {
    return;
  }
  if (sm.number_of_faces() == 0) {
    return;
  }
  const int base = result.verts_num();
  std::map<Surface_mesh::Vertex_index, int> vmap;
  int local = 0;
  for (auto v : sm.vertices()) {
    vmap[v] = base + local++;
    const Point_3 p = clamp_aabb3(sm.point(v), mn, mx);
    result.positions.push_back(float(CGAL::to_double(p.x())));
    result.positions.push_back(float(CGAL::to_double(p.y())));
    result.positions.push_back(float(CGAL::to_double(p.z())));
  }
  for (auto f : sm.faces()) {
    int count = 0;
    for (auto v : CGAL::vertices_around_face(sm.halfedge(f), sm)) {
      result.corner_verts.push_back(vmap[v]);
      count++;
    }
    if (count < 3) {
      result.corner_verts.resize(result.corner_verts.size() - size_t(count));
      continue;
    }
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_tag.push_back(tag);
  }
}

MeshResult points_power_diagram_2(const float *positions,
                                  const float *radii,
                                  int n,
                                  int plane,
                                  float clip_margin)
{
  MeshResult result;
  if (!positions || n < 2) {
    result.error = "Power Diagram 2D needs points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    using Rt = CGAL::Regular_triangulation_2<Kernel>;
    using Wp = typename Rt::Weighted_point;
    using Vec2 = Kernel::Vector_2;
    Rt rt;
    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (int i = 0; i < n; i++) {
      const Point_2 p = project_pt(positions + i * 3, plane);
      const double r = radii ? std::max(0.0, double(radii[i])) : 0.0;
      rt.insert(Wp(p, r * r));
      minx = std::min(minx, CGAL::to_double(p.x()));
      miny = std::min(miny, CGAL::to_double(p.y()));
      maxx = std::max(maxx, CGAL::to_double(p.x()));
      maxy = std::max(maxy, CGAL::to_double(p.y()));
    }
    if (rt.dimension() < 2) {
      result.error = "Power Diagram 2D degenerate";
      return result;
    }
    const double m = std::max(double(clip_margin), 0.0);
    minx -= m;
    miny -= m;
    maxx += m;
    maxy += m;
    const Point_2 corners[4] = {Point_2(minx, miny),
                                Point_2(maxx, miny),
                                Point_2(maxx, maxy),
                                Point_2(minx, maxy)};
    const float mid = mid_dropped_axis(positions, n, plane);
    int tag = 0;
    for (auto vit = rt.finite_vertices_begin(); vit != rt.finite_vertices_end(); ++vit) {
      std::vector<Point_2> cand;
      std::vector<typename Rt::Face_handle> faces;
      auto fc = rt.incident_faces(vit);
      auto done = fc;
      do {
        faces.push_back(fc);
      } while (++fc != done);
      for (typename Rt::Face_handle f : faces) {
        if (rt.is_infinite(f)) {
          continue;
        }
        cand.push_back(rt.weighted_circumcenter(f));
      }
      for (size_t i = 0; i < faces.size(); i++) {
        typename Rt::Face_handle f1 = faces[i];
        typename Rt::Face_handle f2 = faces[(i + 1) % faces.size()];
        if (!rt.is_infinite(f1) && !rt.is_infinite(f2)) {
          const Point_2 c1 = rt.weighted_circumcenter(f1);
          const Point_2 c2 = rt.weighted_circumcenter(f2);
          double x0, y0, x1, y1;
          if (clip_seg_aabb2(CGAL::to_double(c1.x()),
                             CGAL::to_double(c1.y()),
                             CGAL::to_double(c2.x()),
                             CGAL::to_double(c2.y()),
                             minx,
                             miny,
                             maxx,
                             maxy,
                             x0,
                             y0,
                             x1,
                             y1))
          {
            cand.emplace_back(x0, y0);
            cand.emplace_back(x1, y1);
          }
        }
        else if (rt.is_infinite(f1) != rt.is_infinite(f2)) {
          typename Rt::Face_handle inf = rt.is_infinite(f1) ? f1 : f2;
          typename Rt::Face_handle fin = rt.is_infinite(f1) ? f2 : f1;
          const Point_2 c = rt.weighted_circumcenter(fin);
          const int inf_i = inf->index(rt.infinite_vertex());
          const Point_2 a = inf->vertex(Rt::ccw(inf_i))->point().point();
          const Point_2 b = inf->vertex(Rt::cw(inf_i))->point().point();
          const double ex = CGAL::to_double(b.x() - a.x());
          const double ey = CGAL::to_double(b.y() - a.y());
          const Point_2 s = vit->point().point();
          Vec2 nrm(-ey, ex);
          if (CGAL::orientation(a, b, s) == CGAL::LEFT_TURN) {
            nrm = Vec2(ey, -ex);
          }
          double hx, hy;
          if (clip_ray_aabb2(CGAL::to_double(c.x()),
                             CGAL::to_double(c.y()),
                             CGAL::to_double(nrm.x()),
                             CGAL::to_double(nrm.y()),
                             minx,
                             miny,
                             maxx,
                             maxy,
                             hx,
                             hy))
          {
            cand.emplace_back(hx, hy);
          }
        }
      }
      const typename Rt::Vertex_handle vh = vit;
      for (const Point_2 &q : corners) {
        if (rt.nearest_power_vertex(q) == vh) {
          cand.push_back(q);
        }
      }
      emit_isolated_cell_2(result, std::move(cand), plane, mid, tag, minx, miny, maxx, maxy);
      tag++;
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Power Diagram 2D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Power Diagram 2D failed";
  }
  return result;
}

MeshResult points_voronoi_3(const float *positions, int n, float clip_margin)
{
  MeshResult result;
  if (!positions || n < 5) {
    result.error = "Voronoi 3D needs at least 5 non-coplanar points";
    return result;
  }
  try {
    using DT = CGAL::Delaunay_triangulation_3<Kernel>;
    std::vector<Point_3> pts;
    pts.reserve(size_t(n));
    double mn[3] = {1e300, 1e300, 1e300};
    double mx[3] = {-1e300, -1e300, -1e300};
    for (int i = 0; i < n; i++) {
      const double x = double(positions[i * 3 + 0]);
      const double y = double(positions[i * 3 + 1]);
      const double z = double(positions[i * 3 + 2]);
      pts.emplace_back(x, y, z);
      mn[0] = std::min(mn[0], x);
      mn[1] = std::min(mn[1], y);
      mn[2] = std::min(mn[2], z);
      mx[0] = std::max(mx[0], x);
      mx[1] = std::max(mx[1], y);
      mx[2] = std::max(mx[2], z);
    }
    DT dt(pts.begin(), pts.end());
    if (dt.dimension() != 3) {
      result.error = "Voronoi 3D degenerate (need non-coplanar points)";
      return result;
    }
    clip3::pad_aabb_extent(mn, mx, double(clip_margin));
    int tag = 0;
    for (auto vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); ++vit) {
      const typename DT::Vertex_handle site = vit;
      const Point_3 p = site->point();
      const double px = CGAL::to_double(p.x());
      const double py = CGAL::to_double(p.y());
      const double pz = CGAL::to_double(p.z());
      clip3::Poly3 cell = clip3::make_aabb(mn, mx);
      std::vector<typename DT::Cell_handle> cells;
      dt.incident_cells(site, std::back_inserter(cells));
      std::set<typename DT::Vertex_handle> nbrs;
      for (typename DT::Cell_handle c : cells) {
        for (int i = 0; i < 4; i++) {
          const typename DT::Vertex_handle v = c->vertex(i);
          if (v != site && !dt.is_infinite(v)) {
            nbrs.insert(v);
          }
        }
      }
      bool alive = true;
      for (const typename DT::Vertex_handle qh : nbrs) {
        const Point_3 q = qh->point();
        const double qx = CGAL::to_double(q.x());
        const double qy = CGAL::to_double(q.y());
        const double qz = CGAL::to_double(q.z());
        const double nx = qx - px;
        const double ny = qy - py;
        const double nz = qz - pz;
        const double rhs = 0.5 * ((qx * qx + qy * qy + qz * qz) - (px * px + py * py + pz * pz));
        if (!clip3::clip_halfspace(cell, nx, ny, nz, rhs)) {
          alive = false;
          break;
        }
      }
      if (alive) {
        clip3::emit_isolated(result, cell, tag);
      }
      tag++;
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Voronoi 3D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Voronoi 3D failed";
  }
  return result;
}

static bool proper_seg_intersect(const Point_2 &a,
                                 const Point_2 &b,
                                 const Point_2 &c,
                                 const Point_2 &d)
{
  const auto ab_c = CGAL::orientation(a, b, c);
  const auto ab_d = CGAL::orientation(a, b, d);
  const auto cd_a = CGAL::orientation(c, d, a);
  const auto cd_b = CGAL::orientation(c, d, b);
  return ((ab_c == CGAL::LEFT_TURN && ab_d == CGAL::RIGHT_TURN) ||
          (ab_c == CGAL::RIGHT_TURN && ab_d == CGAL::LEFT_TURN)) &&
         ((cd_a == CGAL::LEFT_TURN && cd_b == CGAL::RIGHT_TURN) ||
          (cd_a == CGAL::RIGHT_TURN && cd_b == CGAL::LEFT_TURN));
}

static bool visible_from_q(const Point_2 &q,
                           const Point_2 &p,
                           const std::vector<Point_2> &poly,
                           int skip_edge)
{
  const int n = int(poly.size());
  if (n < 3) {
    return false;
  }
  const double px = CGAL::to_double(p.x());
  const double py = CGAL::to_double(p.y());
  const double qx = CGAL::to_double(q.x());
  const double qy = CGAL::to_double(q.y());
  const double dqp = (px - qx) * (px - qx) + (py - qy) * (py - qy);
  if (dqp < 1e-24) {
    return true;
  }
  for (int i = 0; i < n; i++) {
    if (i == skip_edge) {
      continue;
    }
    const Point_2 &a = poly[size_t(i)];
    const Point_2 &b = poly[size_t((i + 1) % n)];
    /* Skip edges incident to p (p on a vertex of this edge). */
    const double ax = CGAL::to_double(a.x()), ay = CGAL::to_double(a.y());
    const double bx = CGAL::to_double(b.x()), by = CGAL::to_double(b.y());
    if ((ax - px) * (ax - px) + (ay - py) * (ay - py) < 1e-20 ||
        (bx - px) * (bx - px) + (by - py) * (by - py) < 1e-20)
    {
      continue;
    }
    if (proper_seg_intersect(q, p, a, b)) {
      return false;
    }
  }
  return true;
}

static bool ray_hit_seg(const Point_2 &q,
                        const Point_2 &r,
                        const Point_2 &a,
                        const Point_2 &b,
                        Point_2 &out)
{
  const double qx = CGAL::to_double(q.x()), qy = CGAL::to_double(q.y());
  const double rx = CGAL::to_double(r.x()), ry = CGAL::to_double(r.y());
  const double ax = CGAL::to_double(a.x()), ay = CGAL::to_double(a.y());
  const double bx = CGAL::to_double(b.x()), by = CGAL::to_double(b.y());
  const double dx = rx - qx, dy = ry - qy;
  const double ex = bx - ax, ey = by - ay;
  const double det = dx * ey - dy * ex;
  if (std::abs(det) < 1e-18) {
    return false;
  }
  const double t = ((ax - qx) * ey - (ay - qy) * ex) / det;
  const double u = ((ax - qx) * dy - (ay - qy) * dx) / det;
  if (u < -1e-8 || u > 1.0 + 1e-8 || t < 1e-10) {
    return false;
  }
  out = Point_2(qx + t * dx, qy + t * dy);
  return true;
}

static double param_on_seg(const Point_2 &a, const Point_2 &b, const Point_2 &p)
{
  const double ax = CGAL::to_double(a.x()), ay = CGAL::to_double(a.y());
  const double bx = CGAL::to_double(b.x()), by = CGAL::to_double(b.y());
  const double px = CGAL::to_double(p.x()), py = CGAL::to_double(p.y());
  const double ex = bx - ax, ey = by - ay;
  const double len2 = ex * ex + ey * ey;
  if (len2 < 1e-24) {
    return 0.0;
  }
  return ((px - ax) * ex + (py - ay) * ey) / len2;
}

static bool first_hit_beyond(const Point_2 &q,
                             const Point_2 &v,
                             const std::vector<Point_2> &poly,
                             Point_2 &hit)
{
  const double qx = CGAL::to_double(q.x());
  const double qy = CGAL::to_double(q.y());
  const double vx = CGAL::to_double(v.x()) - qx;
  const double vy = CGAL::to_double(v.y()) - qy;
  const double v2 = vx * vx + vy * vy;
  if (v2 < 1e-24) {
    return false;
  }
  double best_t = 1e300;
  bool found = false;
  const int n = int(poly.size());
  for (int i = 0; i < n; i++) {
    const Point_2 &a = poly[size_t(i)];
    const Point_2 &b = poly[size_t((i + 1) % n)];
    const double ax = CGAL::to_double(a.x()) - CGAL::to_double(v.x());
    const double ay = CGAL::to_double(a.y()) - CGAL::to_double(v.y());
    const double bx = CGAL::to_double(b.x()) - CGAL::to_double(v.x());
    const double by = CGAL::to_double(b.y()) - CGAL::to_double(v.y());
    if (ax * ax + ay * ay < 1e-20 || bx * bx + by * by < 1e-20) {
      continue;
    }
    Point_2 h;
    if (!ray_hit_seg(q, v, a, b, h)) {
      continue;
    }
    const double hx = CGAL::to_double(h.x()) - qx;
    const double hy = CGAL::to_double(h.y()) - qy;
    const double t = (hx * vx + hy * vy) / v2;
    if (t > 1.0 + 1e-6 && t < best_t) {
      best_t = t;
      hit = h;
      found = true;
    }
  }
  return found;
}

/**
 * Visibility polygon by walking the boundary.
 * A visible wall keeps every vertex on that chain (including concave ones).
 * Occluded pockets are skipped and closed with a window chord.
 */
static std::vector<Point_2> visibility_polygon_safe(const std::vector<Point_2> &raw_outer,
                                                    Point_2 &q_inout)
{
  std::vector<Point_2> poly = clean_loop_2(raw_outer);
  if (poly.size() < 3) {
    return {};
  }
  if (signed_area_2(poly) < 0.0) {
    std::reverse(poly.begin(), poly.end());
  }
  Point_2 q = q_inout;
  if (!point_in_poly(q, poly)) {
    q = guaranteed_interior(poly);
  }
  if (!point_in_poly(q, poly)) {
    return {};
  }
  q_inout = q;

  const int n = int(poly.size());
  struct Event {
    int edge = 0;
    double t = 0.0;
    Point_2 p;
  };
  std::vector<Event> ev;
  ev.reserve(size_t(n) * 2);
  for (int i = 0; i < n; i++) {
    ev.push_back({i, 0.0, poly[size_t(i)]});
    const Point_2 &prv = poly[size_t((i + n - 1) % n)];
    const Point_2 &cur = poly[size_t(i)];
    const Point_2 &nxt = poly[size_t((i + 1) % n)];
    if (CGAL::orientation(prv, cur, nxt) != CGAL::RIGHT_TURN) {
      continue;
    }
    for (int e = 0; e < n; e++) {
      if (e == i || e == (i + n - 1) % n) {
        continue;
      }
      Point_2 hit;
      if (!ray_hit_seg(q, cur, poly[size_t(e)], poly[size_t((e + 1) % n)], hit)) {
        continue;
      }
      const double t = param_on_seg(poly[size_t(e)], poly[size_t((e + 1) % n)], hit);
      if (t <= 1e-8 || t >= 1.0 - 1e-8) {
        continue;
      }
      const double qx = CGAL::to_double(q.x());
      const double qy = CGAL::to_double(q.y());
      const double vx = CGAL::to_double(cur.x()) - qx;
      const double vy = CGAL::to_double(cur.y()) - qy;
      const double hx = CGAL::to_double(hit.x()) - qx;
      const double hy = CGAL::to_double(hit.y()) - qy;
      const double v2 = vx * vx + vy * vy;
      if (v2 < 1e-24) {
        continue;
      }
      const double ray_t = (hx * vx + hy * vy) / v2;
      if (ray_t > 1.0 + 1e-6) {
        ev.push_back({e, t, hit});
      }
    }
  }
  std::sort(ev.begin(), ev.end(), [](const Event &a, const Event &b) {
    if (a.edge != b.edge) {
      return a.edge < b.edge;
    }
    return a.t < b.t;
  });
  /* Unique (edge, t). */
  {
    std::vector<Event> uniq;
    uniq.reserve(ev.size());
    for (const Event &e : ev) {
      if (uniq.empty() || uniq.back().edge != e.edge || std::abs(uniq.back().t - e.t) > 1e-9) {
        uniq.push_back(e);
      }
    }
    ev.swap(uniq);
  }
  if (ev.size() < 2) {
    return {};
  }

  std::vector<Point_2> vis;
  vis.reserve(ev.size());
  auto push_pt = [&](const Point_2 &p) {
    if (vis.empty()) {
      vis.push_back(p);
      return;
    }
    const double dx = CGAL::to_double(p.x() - vis.back().x());
    const double dy = CGAL::to_double(p.y() - vis.back().y());
    if (dx * dx + dy * dy > 1e-20) {
      vis.push_back(p);
    }
  };
  const int m = int(ev.size());
  for (int i = 0; i < m; i++) {
    const Event &a = ev[size_t(i)];
    const Event &b = ev[size_t((i + 1) % m)];
    const Point_2 mid(0.5 * (a.p.x() + b.p.x()), 0.5 * (a.p.y() + b.p.y()));
    int skip = a.edge;
    if (a.edge != b.edge && b.t <= 1e-12) {
      /* Wrap or vertex: midpoint sits on edge a. */
      skip = a.edge;
    }
    if (visible_from_q(q, mid, poly, skip)) {
      push_pt(a.p);
      push_pt(b.p);
    }
  }
  if (vis.size() >= 2) {
    const double dx = CGAL::to_double(vis.front().x() - vis.back().x());
    const double dy = CGAL::to_double(vis.front().y() - vis.back().y());
    if (dx * dx + dy * dy < 1e-20) {
      vis.pop_back();
    }
  }
  return vis;
}

static void fan_from_query(MeshResult &result,
                           const Point_2 &q,
                           const std::vector<Point_2> &ring,
                           int plane,
                           float mid)
{
  const int n = int(ring.size());
  if (n < 2) {
    return;
  }
  for (int i = 0; i < n; i++) {
    const Point_2 &a = ring[size_t(i)];
    const Point_2 &b = ring[size_t((i + 1) % n)];
    if (CGAL::collinear(q, a, b)) {
      continue;
    }
    append_tri_2(result, q, a, b, plane, mid);
  }
}

/* One n-gon, shared vertices — not an isolated triangle fan. */
static void emit_connected_ring_2(MeshResult &result,
                                  const std::vector<Point_2> &ring,
                                  int plane,
                                  float mid)
{
  if (ring.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  const int base = result.verts_num();
  for (size_t i = 0; i < ring.size(); i++) {
    float q[3];
    unproject(CGAL::to_double(ring[i].x()), CGAL::to_double(ring[i].y()), plane, mid, q);
    result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
    result.corner_verts.push_back(base + int(i));
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
}

static void collect_wall_segments(const MeshIn &mesh,
                                  int plane,
                                  std::vector<std::pair<Point_2, Point_2>> &walls,
                                  std::vector<std::vector<Point_2>> &closed_rings,
                                  float &mid)
{
  walls.clear();
  closed_rings.clear();
  mid = (mesh.positions && mesh.verts_num > 0) ?
            mid_dropped_axis(mesh.positions, mesh.verts_num, plane) :
            0.0f;

  auto add_seg = [&](const Point_2 &a, const Point_2 &b) {
    const double dx = CGAL::to_double(a.x() - b.x());
    const double dy = CGAL::to_double(a.y() - b.y());
    if (dx * dx + dy * dy > 1e-20) {
      walls.emplace_back(a, b);
    }
  };

  std::set<std::array<int, 2>> used_by_face;
  if (mesh.corner_verts && mesh.faces_num > 0) {
    for (int f = 0; f < mesh.faces_num; f++) {
      int start = 0, end = 0;
      if (mesh.face_offsets) {
        start = mesh.face_offsets[f];
        end = mesh.face_offsets[f + 1];
      }
      else {
        start = f * 3;
        end = start + 3;
      }
      const int n = end - start;
      if (n < 2) {
        continue;
      }
      for (int k = 0; k < n; k++) {
        const int a = mesh.corner_verts[start + k];
        const int b = mesh.corner_verts[start + ((k + 1) % n)];
        if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num) {
          continue;
        }
        used_by_face.insert({std::min(a, b), std::max(a, b)});
      }
    }
  }

  Surface_mesh sm;
  std::string load_err;
  if (load_tri(mesh, sm, load_err) && sm.number_of_faces() > 0) {
    std::vector<Surface_mesh::Halfedge_index> cycle_starts;
    CGAL::extract_boundary_cycles(sm, std::back_inserter(cycle_starts));
    for (auto start : cycle_starts) {
      std::vector<Point_2> ring;
      for (auto h : CGAL::halfedges_around_face(start, sm)) {
        const Point_3 &p = sm.point(sm.source(h));
        float xyz[3] = {float(CGAL::to_double(p.x())),
                        float(CGAL::to_double(p.y())),
                        float(CGAL::to_double(p.z()))};
        ring.push_back(project_pt(xyz, plane));
      }
      ring = clean_loop_2(ring);
      if (ring.size() < 3) {
        continue;
      }
      closed_rings.push_back(ring);
      for (size_t i = 0; i < ring.size(); i++) {
        add_seg(ring[i], ring[(i + 1) % ring.size()]);
      }
    }
  }

  /* Loose edges (not used by any face) are thin walls. */
  if (mesh.edge_v0 && mesh.edge_v1 && mesh.edges_num > 0 && mesh.positions) {
    for (int e = 0; e < mesh.edges_num; e++) {
      const int a = mesh.edge_v0[e];
      const int b = mesh.edge_v1[e];
      if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
        continue;
      }
      if (!used_by_face.empty() && used_by_face.count({std::min(a, b), std::max(a, b)})) {
        continue;
      }
      add_seg(project_pt(mesh.positions + a * 3, plane),
              project_pt(mesh.positions + b * 3, plane));
    }
  }
}

template<typename FaceHandle>
static bool extract_ccb_ring(FaceHandle face, std::vector<Point_2> &ring)
{
  ring.clear();
  if (face->is_unbounded()) {
    return false;
  }
  auto circ = face->outer_ccb();
  auto cur = circ;
  int guard = 0;
  do {
    ring.push_back(cur->source()->point());
    ++cur;
    if (++guard > 100000) {
      ring.clear();
      return false;
    }
  } while (cur != circ);
  ring = clean_loop_2(ring);
  return ring.size() >= 3;
}

MeshResult mesh_visibility_2(const MeshIn &mesh, float qx, float qy, float qz)
{
  MeshResult result;
  const int plane = 0;
  const CGAL::Failure_behaviour old_err = CGAL::set_error_behaviour(CGAL::THROW_EXCEPTION);
  try {
    std::vector<std::pair<Point_2, Point_2>> walls;
    std::vector<std::vector<Point_2>> closed_rings;
    float mid = 0.0f;
    collect_wall_segments(mesh, plane, walls, closed_rings, mid);
    if (walls.empty()) {
      CGAL::set_error_behaviour(old_err);
      result.error = "Visibility 2D needs wall faces (borders) or edges";
      return result;
    }

    const float qxyz[3] = {qx, qy, qz};
    Point_2 q = project_pt(qxyz, plane);

    double minx = 1e300, maxx = -1e300, miny = 1e300, maxy = -1e300;
    for (const auto &s : walls) {
      for (const Point_2 *p : {&s.first, &s.second}) {
        minx = std::min(minx, CGAL::to_double(p->x()));
        maxx = std::max(maxx, CGAL::to_double(p->x()));
        miny = std::min(miny, CGAL::to_double(p->y()));
        maxy = std::max(maxy, CGAL::to_double(p->y()));
      }
    }
    const double qx2 = CGAL::to_double(q.x());
    const double qy2 = CGAL::to_double(q.y());
    if (qx2 < minx || qx2 > maxx || qy2 < miny || qy2 > maxy) {
      CGAL::set_error_behaviour(old_err);
      result.error = "Visibility 2D: Query is outside the walls bounding box";
      return result;
    }
    const double diag = std::max(1e-6, std::hypot(maxx - minx, maxy - miny));
    const double pad = std::max(diag * 0.25, 1e-3);
    minx -= pad;
    miny -= pad;
    maxx += pad;
    maxy += pad;
    const Point_2 c0(minx, miny), c1(maxx, miny), c2(maxx, maxy), c3(minx, maxy);
    walls.emplace_back(c0, c1);
    walls.emplace_back(c1, c2);
    walls.emplace_back(c2, c3);
    walls.emplace_back(c3, c0);

    using Traits = CGAL::Arr_segment_traits_2<Kernel>;
    using Arrangement = CGAL::Arrangement_2<Traits>;
    using Seg = Traits::Segment_2;
    using RSV = CGAL::Rotational_sweep_visibility_2<Arrangement, CGAL::Tag_true>;
    using PL = CGAL::Arr_walk_along_line_point_location<Arrangement>;

    Arrangement arr;
    for (const auto &s : walls) {
      try {
        CGAL::insert(arr, Seg(s.first, s.second));
      }
      catch (...) {
      }
    }
    if (arr.number_of_edges() < 1) {
      CGAL::set_error_behaviour(old_err);
      result.error = "Visibility 2D: no valid wall segments";
      return result;
    }

    PL pl(arr);
    const auto loc = pl.locate(q);
    RSV rsv(arr);
    Arrangement out_arr;
    bool computed = false;

    if (const auto *fh = std::get_if<Arrangement::Face_const_handle>(&loc)) {
      if (!(*fh)->is_unbounded()) {
        rsv.compute_visibility(q, *fh, out_arr);
        computed = true;
      }
    }
    else if (const auto *eh = std::get_if<Arrangement::Halfedge_const_handle>(&loc)) {
      rsv.compute_visibility(q, *eh, out_arr);
      computed = true;
    }
    else if (const auto *vh = std::get_if<Arrangement::Vertex_const_handle>(&loc)) {
      auto he = (*vh)->incident_halfedges();
      const auto he0 = he;
      do {
        if (!he->face()->is_unbounded()) {
          rsv.compute_visibility(q, he, out_arr);
          computed = true;
          break;
        }
      } while (++he != he0);
    }

    std::vector<Point_2> vis;
    if (computed) {
      for (auto f = out_arr.faces_begin(); f != out_arr.faces_end(); ++f) {
        std::vector<Point_2> ring;
        if (extract_ccb_ring(f, ring) && point_in_poly(q, ring)) {
          vis = std::move(ring);
          break;
        }
        if (vis.empty() && extract_ccb_ring(f, ring)) {
          vis = std::move(ring);
        }
      }
    }

    /* Fallback: single closed room with the boundary-walk algorithm. */
    if (vis.size() < 3 && !closed_rings.empty()) {
      for (const auto &ring : closed_rings) {
        if (point_in_poly(q, ring)) {
          Point_2 qq = q;
          vis = visibility_polygon_safe(ring, qq);
          q = qq;
          if (vis.size() >= 3) {
            break;
          }
        }
      }
    }

    CGAL::set_error_behaviour(old_err);
    if (vis.size() < 3) {
      result.error =
          "Visibility 2D empty. Place Query in free space (not inside a solid wall).";
      return result;
    }
    emit_connected_ring_2(result, vis, plane, mid);
    result.ok = result.faces_num() > 0 && result.corners_num() >= 3;
    if (!result.ok) {
      result.error = "Visibility 2D produced no faces";
    }
  }
  catch (const std::exception &e) {
    CGAL::set_error_behaviour(old_err);
    result.error = e.what();
  }
  catch (...) {
    CGAL::set_error_behaviour(old_err);
    result.error = "Visibility 2D failed";
  }
  return result;
}

MeshResult mesh_refine_2(const MeshIn &mesh, double max_edge, double shape_bound, int /*lloyd_iters*/)
{
  MeshResult result;
  const int plane = 0;
  max_edge = std::max(0.0, max_edge);
  shape_bound = std::clamp(shape_bound, 0.01, 0.5);
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, plane, islands, mid, error) || islands.empty()) {
      result.error = error.empty() ? "Refine Mesh 2D needs a polygon" : error;
      return result;
    }

    using Vb = CGAL::Delaunay_mesh_vertex_base_2<Kernel>;
    using Fb = CGAL::Delaunay_mesh_face_base_2<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, Tds, CGAL::Exact_predicates_tag>;
    using Criteria = CGAL::Delaunay_mesh_size_criteria_2<CDT>;

    CDT cdt;
    std::vector<Point_2> hole_seeds;
    auto insert_ring = [&](const std::vector<Point_2> &ring) {
      if (ring.size() < 3) {
        return;
      }
      std::vector<typename CDT::Vertex_handle> vhs;
      vhs.reserve(ring.size());
      for (const Point_2 &p : ring) {
        vhs.push_back(cdt.insert(p));
      }
      const int n = int(vhs.size());
      for (int i = 0; i < n; i++) {
        if (vhs[size_t(i)] != vhs[size_t((i + 1) % n)]) {
          try {
            cdt.insert_constraint(vhs[size_t(i)], vhs[size_t((i + 1) % n)]);
          }
          catch (...) {
          }
        }
      }
    };

    for (const PolyIsland &isl : islands) {
      insert_ring(isl.outer);
      for (const auto &h : isl.holes) {
        insert_ring(h);
        if (h.size() >= 3) {
          hole_seeds.push_back(guaranteed_interior(h));
        }
      }
    }
    if (cdt.dimension() != 2) {
      result.error = "Refine Mesh 2D degenerate CDT";
      return result;
    }

    Criteria criteria(shape_bound, max_edge);
    CGAL::Delaunay_mesher_2<CDT, Criteria> mesher(cdt, criteria);
    if (!hole_seeds.empty()) {
      mesher.set_seeds(hole_seeds.begin(), hole_seeds.end(), false);
    }
    mesher.refine_mesh();

    std::map<typename CDT::Face_handle, bool> in_domain_map;
    boost::associative_property_map<std::map<typename CDT::Face_handle, bool>> in_domain(
        in_domain_map);
    CGAL::mark_domain_in_triangulation(cdt, in_domain);

    std::map<typename CDT::Vertex_handle, int> v2i;
    int local = 0;
    auto vid = [&](typename CDT::Vertex_handle vh) {
      auto it = v2i.find(vh);
      if (it != v2i.end()) {
        return it->second;
      }
      const int idx = local++;
      v2i[vh] = idx;
      float q[3];
      unproject(CGAL::to_double(vh->point().x()),
                CGAL::to_double(vh->point().y()),
                plane,
                mid,
                q);
      result.positions.push_back(q[0]);
      result.positions.push_back(q[1]);
      result.positions.push_back(q[2]);
      return idx;
    };
    for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
      if (!get(in_domain, fit)) {
        continue;
      }
      result.corner_verts.push_back(vid(fit->vertex(0)));
      result.corner_verts.push_back(vid(fit->vertex(1)));
      result.corner_verts.push_back(vid(fit->vertex(2)));
    }
    result.ok = result.corners_num() >= 3;
    if (!result.ok) {
      result.error = "Refine Mesh 2D produced no interior faces";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Refine Mesh 2D failed";
  }
  return result;
}

MeshResult points_alpha_complex_3(const float *positions,
                                  int n,
                                  double alpha,
                                  bool use_optimal_alpha,
                                  bool separate_tets)
{
  MeshResult result;
  if (!positions || n < 4) {
    result.error = "Alpha Complex 3D needs at least 4 points";
    return result;
  }
  try {
    using Vb = CGAL::Alpha_shape_vertex_base_3<Kernel>;
    using Fb = CGAL::Alpha_shape_cell_base_3<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_3<Vb, Fb>;
    using Triangulation = CGAL::Delaunay_triangulation_3<Kernel, Tds>;
    using Alpha_shape = CGAL::Alpha_shape_3<Triangulation>;

    std::vector<Point_3> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(double(positions[i * 3 + 0]),
                       double(positions[i * 3 + 1]),
                       double(positions[i * 3 + 2]));
    }
    Alpha_shape as(pts.begin(), pts.end(), 0, Alpha_shape::GENERAL);
    if (use_optimal_alpha) {
      auto opt = as.find_optimal_alpha(1);
      as.set_alpha(*opt);
      result.alpha_used = CGAL::to_double(*opt);
    }
    else {
      double a = alpha;
      if (!(a > 0.0)) {
        double minx = pts[0].x(), maxx = minx;
        double miny = pts[0].y(), maxy = miny;
        double minz = pts[0].z(), maxz = minz;
        for (const Point_3 &p : pts) {
          minx = std::min(minx, CGAL::to_double(p.x()));
          maxx = std::max(maxx, CGAL::to_double(p.x()));
          miny = std::min(miny, CGAL::to_double(p.y()));
          maxy = std::max(maxy, CGAL::to_double(p.y()));
          minz = std::min(minz, CGAL::to_double(p.z()));
          maxz = std::max(maxz, CGAL::to_double(p.z()));
        }
        const double dx = maxx - minx, dy = maxy - miny, dz = maxz - minz;
        a = std::sqrt(dx * dx + dy * dy + dz * dz) / 20.0;
        if (!(a > 0.0)) {
          a = 0.1;
        }
      }
      as.set_alpha(a);
      result.alpha_used = a;
    }

    auto push_pt = [&](const Point_3 &p) {
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
    };
    auto push_tri = [&](int a, int b, int c, int tag) {
      result.corner_verts.push_back(a);
      result.corner_verts.push_back(b);
      result.corner_verts.push_back(c);
      result.face_tag.push_back(tag);
    };

    std::map<typename Alpha_shape::Vertex_handle, int> v2i;
    int local = 0;
    auto vid = [&](typename Alpha_shape::Vertex_handle vh) {
      auto it = v2i.find(vh);
      if (it != v2i.end()) {
        return it->second;
      }
      const int idx = local++;
      v2i[vh] = idx;
      push_pt(vh->point());
      return idx;
    };

    int tet_id = 0;
    for (auto cit = as.finite_cells_begin(); cit != as.finite_cells_end(); ++cit) {
      if (as.classify(cit) == Alpha_shape::EXTERIOR) {
        continue;
      }
      if (separate_tets) {
        const int base = result.verts_num();
        push_pt(cit->vertex(0)->point());
        push_pt(cit->vertex(1)->point());
        push_pt(cit->vertex(2)->point());
        push_pt(cit->vertex(3)->point());
        push_tri(base + 0, base + 1, base + 2, tet_id);
        push_tri(base + 0, base + 1, base + 3, tet_id);
        push_tri(base + 0, base + 2, base + 3, tet_id);
        push_tri(base + 1, base + 2, base + 3, tet_id);
      }
      else {
        const int i0 = vid(cit->vertex(0));
        const int i1 = vid(cit->vertex(1));
        const int i2 = vid(cit->vertex(2));
        const int i3 = vid(cit->vertex(3));
        push_tri(i0, i1, i2, tet_id);
        push_tri(i0, i1, i3, tet_id);
        push_tri(i0, i2, i3, tet_id);
        push_tri(i1, i2, i3, tet_id);
      }
      tet_id++;
    }
    result.ok = result.corners_num() >= 3 && tet_id > 0;
    result.alpha_used = result.alpha_used;
    if (result.ok) {
      result.radius = float(tet_id);
    }
    else {
      result.error = "Alpha Complex 3D empty at this alpha";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Alpha Complex 3D failed";
  }
  return result;
}

MeshResult mesh_minkowski_sum_3_convex(const MeshIn &mesh_a, const MeshIn &mesh_b)
{
  MeshResult result;
  try {
    std::vector<Point_3> pa, pb;
    unique_points_3(mesh_a, pa);
    unique_points_3(mesh_b, pb);
    if (pa.size() < 1 || pb.size() < 1) {
      result.error = "Convex Minkowski Sum 3D needs vertices on both meshes";
      return result;
    }
    Surface_mesh ha, hb;
    if (pa.size() >= 4) {
      try {
        CGAL::convex_hull_3(pa.begin(), pa.end(), ha);
      }
      catch (...) {
      }
    }
    if (pb.size() >= 4) {
      try {
        CGAL::convex_hull_3(pb.begin(), pb.end(), hb);
      }
      catch (...) {
      }
    }
    std::vector<Point_3> va, vb;
    if (ha.number_of_vertices() >= 4) {
      for (const auto v : ha.vertices()) {
        va.push_back(ha.point(v));
      }
    }
    else {
      va = pa;
    }
    if (hb.number_of_vertices() >= 4) {
      for (const auto v : hb.vertices()) {
        vb.push_back(hb.point(v));
      }
    }
    else {
      vb = pb;
    }
    if (va.size() * vb.size() > 250000) {
      result.error = "Convex Minkowski Sum 3D: too many hull vertices (cap 250k pairwise)";
      return result;
    }
    std::vector<Point_3> sums;
    sums.reserve(va.size() * vb.size());
    for (const Point_3 &a : va) {
      for (const Point_3 &b : vb) {
        sums.emplace_back(a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
      }
    }
    Surface_mesh hull;
    if (sums.size() < 4) {
      result.error = "Convex Minkowski Sum 3D: not enough summed points";
      return result;
    }
    CGAL::convex_hull_3(sums.begin(), sums.end(), hull);
    if (hull.number_of_faces() == 0) {
      result.error = "Convex Minkowski Sum 3D hull empty";
      return result;
    }
    result = surface_mesh_to_result(hull);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Convex Minkowski Sum 3D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
