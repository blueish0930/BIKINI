/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 29 鈥?new only (not previously deleted / already existing):
 *  - Boolean Ops 2D (union / intersection / difference / xor)
 *  - Largest Empty Iso Rectangle 2D
 *  - Polygon Repair 2D
 *  - Regularize Contour 2D
 *  - Monge Jet Fit (point attributes)
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/assertions_behaviour.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Largest_empty_iso_rectangle_2.h>
#include <CGAL/Monge_via_jet_fitting.h>
#include <CGAL/Multipolygon_with_holes_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Polygon_repair/repair.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Shape_regularization/regularize_contours.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/mark_domain_in_triangulation.h>

#include <boost/property_map/property_map.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <list>
#include <map>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point = Point_3;
using Point_2 = Kernel::Point_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using Polygon_with_holes_2 = CGAL::Polygon_with_holes_2<Kernel>;
using Multipolygon_with_holes_2 = CGAL::Multipolygon_with_holes_2<Kernel>;

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
        const Point &p = sm.point(sm.source(h));
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

/** Drop vertices that sit on a straight edge (grid outline → true corners). */
static std::vector<Point_2> collapse_collinear(std::vector<Point_2> ring)
{
  if (ring.size() < 4) {
    return ring;
  }
  for (int pass = 0; pass < 16; pass++) {
    const int n = int(ring.size());
    if (n < 3) {
      break;
    }
    std::vector<Point_2> out;
    out.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      const Point_2 &a = ring[size_t((i + n - 1) % n)];
      const Point_2 &b = ring[size_t(i)];
      const Point_2 &c = ring[size_t((i + 1) % n)];
      const double ax = CGAL::to_double(a.x());
      const double ay = CGAL::to_double(a.y());
      const double bx = CGAL::to_double(b.x());
      const double by = CGAL::to_double(b.y());
      const double cx = CGAL::to_double(c.x());
      const double cy = CGAL::to_double(c.y());
      const double abx = bx - ax;
      const double aby = by - ay;
      const double bcx = cx - bx;
      const double bcy = cy - by;
      const double cross = abx * bcy - aby * bcx;
      const double ab2 = abx * abx + aby * aby;
      const double bc2 = bcx * bcx + bcy * bcy;
      const double scale = std::sqrt(std::max(0.0, ab2 * bc2));
      if (scale > 1e-30 && std::abs(cross) <= 1e-8 * scale) {
        continue;
      }
      out.push_back(b);
    }
    if (out.size() < 3 || out.size() == ring.size()) {
      if (out.size() >= 3) {
        ring = std::move(out);
      }
      break;
    }
    ring = std::move(out);
  }
  return ring;
}

/**
 * Each original mesh face is one 2D n-gon island (no pre-triangulation).
 * Used so boolean operands keep the input face rings.
 */
static double xy_snap_scale(const MeshIn &mesh, int plane)
{
  if (!mesh.positions || mesh.verts_num <= 0) {
    return 1.0e6;
  }
  double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
  for (int i = 0; i < mesh.verts_num; i++) {
    const Point_2 p = project_pt(mesh.positions + i * 3, plane);
    const double x = CGAL::to_double(p.x());
    const double y = CGAL::to_double(p.y());
    if (!std::isfinite(x) || !std::isfinite(y)) {
      continue;
    }
    minx = std::min(minx, x);
    miny = std::min(miny, y);
    maxx = std::max(maxx, x);
    maxy = std::max(maxy, y);
  }
  const double diag = std::hypot(std::max(0.0, maxx - minx), std::max(0.0, maxy - miny));
  if (!(diag > 1e-18)) {
    return 1.0e6;
  }
  return std::min(1.0e9, std::max(1.0e4, 1.0e7 / diag));
}

static Point_2 snap_pt2(const Point_2 &p, double scale)
{
  const double x = CGAL::to_double(p.x());
  const double y = CGAL::to_double(p.y());
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return p;
  }
  return Point_2(std::round(x * scale) / scale, std::round(y * scale) / scale);
}

static bool proper_intersect_2(const Point_2 &a,
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

static bool ring_self_intersects(const std::vector<Point_2> &ring)
{
  const int n = int(ring.size());
  if (n < 4) {
    return false;
  }
  for (int i = 0; i < n; i++) {
    const int i1 = (i + 1) % n;
    for (int j = i + 1; j < n; j++) {
      const int j1 = (j + 1) % n;
      if (i == j || i == j1 || i1 == j || i1 == j1) {
        continue;
      }
      if (proper_intersect_2(ring[size_t(i)], ring[size_t(i1)], ring[size_t(j)], ring[size_t(j1)])) {
        return true;
      }
    }
  }
  return false;
}

static bool extract_faces_as_islands(const MeshIn &mesh,
                                     int plane,
                                     std::vector<PolyIsland> &islands,
                                     float &mid_drop,
                                     std::string &error)
{
  islands.clear();
  if (!mesh.positions || mesh.verts_num < 3 || !mesh.corner_verts || mesh.corners_num < 3) {
    error = "Need faces";
    return false;
  }
  mid_drop = mid_dropped_axis(mesh.positions, mesh.verts_num, plane);
  const double scale = xy_snap_scale(mesh, plane);

  int faces_num = mesh.faces_num;
  const bool have_offsets = mesh.face_offsets != nullptr && mesh.faces_num > 0;
  if (!have_offsets) {
    if (mesh.corners_num % 3 != 0) {
      error = "Need face offsets or triangles";
      return false;
    }
    faces_num = mesh.corners_num / 3;
  }
  if (faces_num <= 0) {
    error = "Need faces";
    return false;
  }

  for (int f = 0; f < faces_num; f++) {
    const int begin = have_offsets ? mesh.face_offsets[f] : f * 3;
    const int end = have_offsets ? mesh.face_offsets[f + 1] : begin + 3;
    if (end - begin < 3 || begin < 0 || end > mesh.corners_num) {
      continue;
    }
    std::vector<Point_2> raw;
    raw.reserve(size_t(end - begin));
    bool bad = false;
    for (int i = begin; i < end; i++) {
      const int v = mesh.corner_verts[i];
      if (v < 0 || v >= mesh.verts_num) {
        bad = true;
        break;
      }
      const Point_2 p = snap_pt2(project_pt(mesh.positions + v * 3, plane), scale);
      const double x = CGAL::to_double(p.x());
      const double y = CGAL::to_double(p.y());
      if (!std::isfinite(x) || !std::isfinite(y)) {
        bad = true;
        break;
      }
      raw.push_back(p);
    }
    if (bad) {
      continue;
    }
    raw = clean_loop_2(raw);
    if (raw.size() < 3 || std::abs(signed_area_2(raw)) <= 1e-20) {
      continue;
    }
    if (ring_self_intersects(raw)) {
      continue;
    }
    PolyIsland isl;
    isl.outer = std::move(raw);
    if (signed_area_2(isl.outer) < 0.0) {
      std::reverse(isl.outer.begin(), isl.outer.end());
    }
    islands.push_back(std::move(isl));
  }
  if (islands.empty()) {
    error = "No usable 2D faces";
    return false;
  }
  return true;
}

static bool extract_boolean_islands(const MeshIn &mesh,
                                    int plane,
                                    std::vector<PolyIsland> &islands,
                                    float &mid_drop,
                                    std::string &error)
{
  /* Prefer original face n-gons so a pentagon stays a pentagon, not 3 tris. */
  std::string face_err;
  if (extract_faces_as_islands(mesh, plane, islands, mid_drop, face_err)) {
    return true;
  }
  return extract_polygon_islands(mesh, plane, islands, mid_drop, error);
}

static void append_ngon_face(MeshResult &result, const std::vector<Point_2> &ring, int plane, float mid)
{
  if (ring.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  const int base = result.verts_num();
  for (const Point_2 &p : ring) {
    float q[3];
    unproject(CGAL::to_double(p.x()), CGAL::to_double(p.y()), plane, mid, q);
    result.positions.push_back(q[0]);
    result.positions.push_back(q[1]);
    result.positions.push_back(q[2]);
  }
  for (int i = 0; i < int(ring.size()); i++) {
    result.corner_verts.push_back(base + i);
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
}

static void append_pwh_triangulated(MeshResult &result,
                                    const Polygon_with_holes_2 &pwh,
                                    int plane,
                                    float mid)
{
  using Vb = CGAL::Triangulation_vertex_base_2<Kernel>;
  using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
  using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
  using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, CGAL::Exact_predicates_tag>;

  const Polygon_2 &outer = pwh.outer_boundary();
  if (outer.size() < 3) {
    return;
  }

  CDT cdt;
  auto insert_closed_ring = [&](const Polygon_2 &poly) {
    if (poly.size() < 3) {
      return;
    }
    std::vector<typename CDT::Vertex_handle> vhs;
    vhs.reserve(size_t(poly.size()));
    for (auto vit = poly.vertices_begin(); vit != poly.vertices_end(); ++vit) {
      try {
        vhs.push_back(cdt.insert(*vit));
      }
      catch (...) {
        return;
      }
    }
    if (vhs.size() >= 2 && vhs.front() == vhs.back()) {
      vhs.pop_back();
    }
    const int n = int(vhs.size());
    if (n < 3) {
      return;
    }
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

  insert_closed_ring(outer);
  for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
    insert_closed_ring(*hit);
  }
  if (cdt.dimension() != 2) {
    return;
  }

  std::map<typename CDT::Face_handle, bool> in_domain_map;
  boost::associative_property_map<std::map<typename CDT::Face_handle, bool>> in_domain(
      in_domain_map);
  CGAL::mark_domain_in_triangulation(cdt, in_domain);

  std::map<typename CDT::Vertex_handle, int> v2i;
  const int base = result.verts_num();
  int local = 0;
  auto vert_index = [&](typename CDT::Vertex_handle vh) -> int {
    auto it = v2i.find(vh);
    if (it != v2i.end()) {
      return it->second;
    }
    const int idx = base + local++;
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

  for (typename CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
       fit != cdt.finite_faces_end();
       ++fit)
  {
    typename CDT::Face_handle fh = fit;
    if (!get(in_domain, fh)) {
      continue;
    }
    const int i0 = vert_index(fh->vertex(0));
    const int i1 = vert_index(fh->vertex(1));
    const int i2 = vert_index(fh->vertex(2));
    if (i0 == i1 || i1 == i2 || i2 == i0) {
      continue;
    }
    result.corner_verts.push_back(i0);
    result.corner_verts.push_back(i1);
    result.corner_verts.push_back(i2);
  }
}

static Polygon_2 force_ccw(Polygon_2 poly)
{
  /* Avoid Polygon_2::is_clockwise_oriented() — it aborts on non-simple rings. */
  if (poly.size() >= 3) {
    std::vector<Point_2> pts(poly.vertices_begin(), poly.vertices_end());
    if (signed_area_2(pts) < 0.0) {
      poly.reverse_orientation();
    }
  }
  return poly;
}

static Polygon_2 island_to_poly(const PolyIsland &isl)
{
  Polygon_2 outer(isl.outer.begin(), isl.outer.end());
  return force_ccw(outer);
}

static Polygon_with_holes_2 island_to_pwh(const PolyIsland &isl)
{
  Polygon_2 outer = island_to_poly(isl);
  std::vector<Polygon_2> holes;
  for (const auto &h : isl.holes) {
    if (h.size() < 3) {
      continue;
    }
    Polygon_2 hp(h.begin(), h.end());
    std::vector<Point_2> hpts(hp.vertices_begin(), hp.vertices_end());
    if (signed_area_2(hpts) > 0.0) {
      hp.reverse_orientation();
    }
    holes.push_back(std::move(hp));
  }
  return Polygon_with_holes_2(outer, holes.begin(), holes.end());
}

static bool join_pwh_or_repair(CGAL::Polygon_set_2<Kernel> &set, const Polygon_with_holes_2 &pwh)
{
  try {
    set.join(pwh);
    return true;
  }
  catch (...) {
  }
  try {
    Multipolygon_with_holes_2 multi = CGAL::Polygon_repair::repair(pwh);
    bool any = false;
    for (auto it = multi.polygons_with_holes_begin(); it != multi.polygons_with_holes_end(); ++it) {
      if (it->outer_boundary().size() < 3) {
        continue;
      }
      try {
        set.join(*it);
        any = true;
      }
      catch (...) {
      }
    }
    return any;
  }
  catch (...) {
  }
  return false;
}

static bool islands_to_polygon_set(const std::vector<PolyIsland> &islands,
                                   CGAL::Polygon_set_2<Kernel> &set)
{
  bool any = false;
  for (const PolyIsland &isl : islands) {
    if (isl.outer.size() < 3) {
      continue;
    }
    try {
      Polygon_with_holes_2 pwh = island_to_pwh(isl);
      if (pwh.outer_boundary().size() < 3) {
        continue;
      }
      if (join_pwh_or_repair(set, pwh)) {
        any = true;
      }
    }
    catch (...) {
    }
  }
  return any;
}

}  // namespace

/* Boolean Ops 2D lives in ops/cgal_overlay_2.cc (flattened mesh = 2D silhouette). */

#if 0
MeshResult mesh_boolean_ops_2_OLD(const MeshIn &mesh_a, const MeshIn &mesh_b, int mode)
{
  MeshResult result;
  const int plane = 0;
  mode = std::clamp(mode, 0, 3);
  try {
    std::vector<PolyIsland> ia, ib;
    float mida = 0.0f, midb = 0.0f;
    std::string error;
    if (!extract_boolean_islands(mesh_a, plane, ia, mida, error) || ia.empty()) {
      result.error = error.empty() ? "Boolean Ops 2D: mesh A needs polygon" : error;
      return result;
    }
    const bool has_b = extract_boolean_islands(mesh_b, plane, ib, midb, error) && !ib.empty();
    const float mid = has_b ? 0.5f * (mida + midb) : mida;
    CgalThrowGuard cgal_guard;
    std::vector<Polygon_with_holes_2> out_list;
    try {
      CGAL::Polygon_set_2<Kernel> sa, sb;
      if (!islands_to_polygon_set(ia, sa)) {
        result.error = "Boolean Ops 2D: mesh A has no usable islands";
        return result;
      }
      if (has_b) {
        if (!islands_to_polygon_set(ib, sb)) {
          result.error = "Boolean Ops 2D: mesh B has no usable islands";
          return result;
        }
        switch (mode) {
          case 0:
            sa.join(sb);
            break;
          case 1:
            sa.intersection(sb);
            break;
          case 2:
            sa.difference(sb);
            break;
          default:
            sa.symmetric_difference(sb);
            break;
        }
      }
      /* No B: emit the already-joined union of every island in A (grid outline). */
      sa.polygons_with_holes(std::back_inserter(out_list));
    }
    catch (const std::exception &e) {
      result.error = std::string("Boolean Ops 2D failed: ") + e.what();
      return result;
    }
    catch (...) {
      result.error = "Boolean Ops 2D failed (degenerate projected faces were skipped)";
      return result;
    }

    auto emit_pwh = [&](const Polygon_with_holes_2 &pwh) {
      std::vector<Point_2> outer(pwh.outer_boundary().vertices_begin(),
                                 pwh.outer_boundary().vertices_end());
      if (outer.size() >= 2 && outer.front() == outer.back()) {
        outer.pop_back();
      }
      outer = collapse_collinear(clean_loop_2(outer));
      if (outer.size() < 3) {
        return;
      }
      if (signed_area_2(outer) < 0.0) {
        std::reverse(outer.begin(), outer.end());
      }
      append_ngon_face(result, outer, plane, mid);
      result.face_tag.push_back(0);
      for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        std::vector<Point_2> hole(hit->vertices_begin(), hit->vertices_end());
        if (hole.size() >= 2 && hole.front() == hole.back()) {
          hole.pop_back();
        }
        hole = collapse_collinear(clean_loop_2(hole));
        if (hole.size() < 3) {
          continue;
        }
        if (signed_area_2(hole) > 0.0) {
          std::reverse(hole.begin(), hole.end());
        }
        append_ngon_face(result, hole, plane, mid);
        result.face_tag.push_back(1);
      }
    };

    for (const Polygon_with_holes_2 &pwh : out_list) {
      emit_pwh(pwh);
    }
    if (result.corner_verts.size() < 3) {
      result.error = "Boolean Ops 2D empty result";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Boolean Ops 2D failed";
  }
  return result;
}
#endif

/* -------------------------------------------------------------------------- */
/* 2. Largest Empty Iso Rectangle 2D                                          */
/* -------------------------------------------------------------------------- */

MeshResult points_largest_empty_iso_rectangle_2(const float *positions, int n, int plane)
{
  MeshResult result;
  if (!positions || n < 1) {
    result.error = "Largest Empty Iso Rectangle needs points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (int i = 0; i < n; i++) {
      Point_2 p = project_pt(positions + i * 3, plane);
      pts.push_back(p);
      minx = std::min(minx, CGAL::to_double(p.x()));
      miny = std::min(miny, CGAL::to_double(p.y()));
      maxx = std::max(maxx, CGAL::to_double(p.x()));
      maxy = std::max(maxy, CGAL::to_double(p.y()));
    }
    if (!(maxx > minx) || !(maxy > miny)) {
      result.error = "Degenerate bounding box";
      return result;
    }
    /* Expand bbox slightly so boundary points don't collapse the search. */
    const double dx = std::max(1e-6, (maxx - minx) * 0.05);
    const double dy = std::max(1e-6, (maxy - miny) * 0.05);
    Kernel::Iso_rectangle_2 bbox(Point_2(minx - dx, miny - dy), Point_2(maxx + dx, maxy + dy));
    CGAL::Largest_empty_iso_rectangle_2<Kernel> ler(bbox);
    ler.insert(pts.begin(), pts.end());
    Kernel::Iso_rectangle_2 empty = ler.get_largest_empty_iso_rectangle();
    const float mid = mid_dropped_axis(positions, n, plane);
    std::vector<Point_2> ring = {empty[0], empty[1], empty[2], empty[3]};
    result.face_offsets = {0};
    append_ngon_face(result, ring, plane, mid);
    const double cx = 0.5 * (CGAL::to_double(empty.xmin()) + CGAL::to_double(empty.xmax()));
    const double cy = 0.5 * (CGAL::to_double(empty.ymin()) + CGAL::to_double(empty.ymax()));
    float c[3];
    unproject(cx, cy, plane, mid, c);
    result.center[0] = c[0];
    result.center[1] = c[1];
    result.center[2] = c[2];
    result.radius = float(std::sqrt(CGAL::to_double(empty.xmax() - empty.xmin()) *
                                        CGAL::to_double(empty.ymax() - empty.ymin())));
    result.ok = result.face_offsets.size() >= 2;
    if (!result.ok) {
      result.error = "Largest empty rectangle failed";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Largest Empty Iso Rectangle 2D failed";
  }
  return result;
}

/* -------------------------------------------------------------------------- */
/* 3. Polygon Repair 2D                                                       */
/* -------------------------------------------------------------------------- */

MeshResult mesh_polygon_repair_2(const MeshIn &mesh)
{
  MeshResult result;
  const int plane = 0;
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, plane, islands, mid, error) || islands.empty()) {
      result.error = error.empty() ? "Polygon Repair 2D needs polygons" : error;
      return result;
    }
    for (const PolyIsland &isl : islands) {
      if (isl.outer.size() < 3) {
        continue;
      }
      try {
        Polygon_with_holes_2 pwh = island_to_pwh(isl);
        Multipolygon_with_holes_2 multi = CGAL::Polygon_repair::repair(pwh);
        for (auto it = multi.polygons_with_holes_begin(); it != multi.polygons_with_holes_end();
             ++it)
        {
          append_pwh_triangulated(result, *it, plane, mid);
        }
      }
      catch (...) {
        /* Fall back: try outer-only repair. */
        try {
          Polygon_2 outer = island_to_poly(isl);
          Multipolygon_with_holes_2 multi = CGAL::Polygon_repair::repair(outer);
          for (auto it = multi.polygons_with_holes_begin(); it != multi.polygons_with_holes_end();
               ++it)
          {
            append_pwh_triangulated(result, *it, plane, mid);
          }
        }
        catch (...) {
        }
      }
    }
    if (result.corner_verts.size() < 3) {
      result.error = "Polygon Repair 2D empty";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Polygon Repair 2D failed";
  }
  return result;
}

/* -------------------------------------------------------------------------- */
/* 5-7. Apollonius Graph 2D / Rectangular P-Center 2D / Regularize Contour 2D — removed */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* 8. Monge Jet Fit (points)                                                  */
/* -------------------------------------------------------------------------- */

bool points_monge_jet_fit(const float *positions,
                          int n,
                          int knn,
                          int degree_fitting,
                          int degree_monge,
                          std::vector<float> &out_k1,
                          std::vector<float> &out_k2,
                          std::vector<float> &out_d1,
                          std::vector<float> &out_d2,
                          std::vector<float> &out_n,
                          std::string &error)
{
  error.clear();
  out_k1.assign(size_t(n), 0.0f);
  out_k2.assign(size_t(n), 0.0f);
  out_d1.assign(size_t(n) * 3, 0.0f);
  out_d2.assign(size_t(n) * 3, 0.0f);
  out_n.assign(size_t(n) * 3, 0.0f);
  if (!positions || n < 6) {
    error = "Monge Jet Fit needs at least 6 points";
    return false;
  }
  degree_fitting = std::clamp(degree_fitting, 1, 4);
  degree_monge = std::clamp(degree_monge, 1, std::min(degree_fitting, 4));
  /* Minimum points for jet of degree d is (d+1)(d+2)/2. */
  const int min_pts = (degree_fitting + 1) * (degree_fitting + 2) / 2;
  knn = std::clamp(knn, min_pts, n);
  if (n < min_pts) {
    error = "Not enough points for chosen jet degree";
    return false;
  }

  try {
    /*
     * Fast path designed for large point counts:
     *  1) float SoA + uniform hash grid kNN (no CGAL kd-tree overhead)
     *  2) Monge_via_jet_fitting on Simple_cartesian<double> (Eigen SVD)
     *  3) multi-threaded over points (std::thread)
     *
     * Previous 鈥渵400 ms flat鈥?was only realistic for small-to-medium n; cost is
     * inherently 螛(n 路 SVD(k)). Grid + threads restore that ballpark for denser clouds.
     */
    using FastK = CGAL::Simple_cartesian<double>;
    using Fast_point = FastK::Point_3;
    using Fast_vector = FastK::Vector_3;
    using Monge_fit = CGAL::Monge_via_jet_fitting<FastK>;

    std::vector<float> xyz(size_t(n) * 3);
    float bmin[3] = {positions[0], positions[1], positions[2]};
    float bmax[3] = {positions[0], positions[1], positions[2]};
    for (int i = 0; i < n; i++) {
      const float x = positions[i * 3 + 0];
      const float y = positions[i * 3 + 1];
      const float z = positions[i * 3 + 2];
      xyz[size_t(i) * 3 + 0] = x;
      xyz[size_t(i) * 3 + 1] = y;
      xyz[size_t(i) * 3 + 2] = z;
      bmin[0] = std::min(bmin[0], x);
      bmin[1] = std::min(bmin[1], y);
      bmin[2] = std::min(bmin[2], z);
      bmax[0] = std::max(bmax[0], x);
      bmax[1] = std::max(bmax[1], y);
      bmax[2] = std::max(bmax[2], z);
    }
    const float ext[3] = {std::max(1e-6f, bmax[0] - bmin[0]),
                          std::max(1e-6f, bmax[1] - bmin[1]),
                          std::max(1e-6f, bmax[2] - bmin[2])};
    /* Cell size 鈮?mean spacing so each cell has O(1) points. */
    const float vol = ext[0] * ext[1] * ext[2];
    const float spacing = std::cbrt(vol / float(std::max(n, 1)));
    const float cell = std::max(spacing * 1.5f, 1e-6f);
    const float inv_cell = 1.0f / cell;

    auto cell_key = [&](float x, float y, float z) -> uint64_t {
      const int64_t ix = int64_t(std::floor((x - bmin[0]) * inv_cell));
      const int64_t iy = int64_t(std::floor((y - bmin[1]) * inv_cell));
      const int64_t iz = int64_t(std::floor((z - bmin[2]) * inv_cell));
      /* Pack signed 21-bit axes into 64 bits. */
      const uint64_t ux = uint64_t(ix + (int64_t(1) << 20)) & 0x1FFFFFull;
      const uint64_t uy = uint64_t(iy + (int64_t(1) << 20)) & 0x1FFFFFull;
      const uint64_t uz = uint64_t(iz + (int64_t(1) << 20)) & 0x1FFFFFull;
      return (ux << 42) | (uy << 21) | uz;
    };

    std::unordered_map<uint64_t, std::vector<int>> grid;
    grid.reserve(size_t(n) * 2);
    for (int i = 0; i < n; i++) {
      grid[cell_key(xyz[size_t(i) * 3 + 0], xyz[size_t(i) * 3 + 1], xyz[size_t(i) * 3 + 2])]
          .push_back(i);
    }

    auto knn_of = [&](int i, std::vector<std::pair<float, int>> &heap) {
      heap.clear();
      const float ix = xyz[size_t(i) * 3 + 0];
      const float iy = xyz[size_t(i) * 3 + 1];
      const float iz = xyz[size_t(i) * 3 + 2];
      const int64_t cx = int64_t(std::floor((ix - bmin[0]) * inv_cell));
      const int64_t cy = int64_t(std::floor((iy - bmin[1]) * inv_cell));
      const int64_t cz = int64_t(std::floor((iz - bmin[2]) * inv_cell));
      /* Expand rings until k filled and ring beyond max-heap radius. */
      for (int ring = 0; ring <= 8; ring++) {
        float worst = (int(heap.size()) < knn) ? std::numeric_limits<float>::max() :
                                                 heap.front().first;
        const float ring_r = float(ring) * cell;
        if (int(heap.size()) >= knn && ring_r * ring_r > worst) {
          break;
        }
        for (int dx = -ring; dx <= ring; dx++) {
          for (int dy = -ring; dy <= ring; dy++) {
            for (int dz = -ring; dz <= ring; dz++) {
              if (ring > 0 && std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != ring) {
                continue;
              }
              const uint64_t ux = uint64_t((cx + dx) + (int64_t(1) << 20)) & 0x1FFFFFull;
              const uint64_t uy = uint64_t((cy + dy) + (int64_t(1) << 20)) & 0x1FFFFFull;
              const uint64_t uz = uint64_t((cz + dz) + (int64_t(1) << 20)) & 0x1FFFFFull;
              const uint64_t key = (ux << 42) | (uy << 21) | uz;
              auto it = grid.find(key);
              if (it == grid.end()) {
                continue;
              }
              for (int j : it->second) {
                const float jx = xyz[size_t(j) * 3 + 0] - ix;
                const float jy = xyz[size_t(j) * 3 + 1] - iy;
                const float jz = xyz[size_t(j) * 3 + 2] - iz;
                const float d2 = jx * jx + jy * jy + jz * jz;
                if (int(heap.size()) < knn) {
                  heap.emplace_back(d2, j);
                  if (int(heap.size()) == knn) {
                    std::make_heap(heap.begin(), heap.end());
                  }
                }
                else if (d2 < heap.front().first) {
                  std::pop_heap(heap.begin(), heap.end());
                  heap.back() = {d2, j};
                  std::push_heap(heap.begin(), heap.end());
                }
              }
            }
          }
        }
      }
      /* Fallback: if grid underfilled (pathological clustering), brute-force rest. */
      if (int(heap.size()) < min_pts) {
        heap.clear();
        for (int j = 0; j < n; j++) {
          const float jx = xyz[size_t(j) * 3 + 0] - ix;
          const float jy = xyz[size_t(j) * 3 + 1] - iy;
          const float jz = xyz[size_t(j) * 3 + 2] - iz;
          const float d2 = jx * jx + jy * jy + jz * jz;
          if (int(heap.size()) < knn) {
            heap.emplace_back(d2, j);
            if (int(heap.size()) == knn) {
              std::make_heap(heap.begin(), heap.end());
            }
          }
          else if (d2 < heap.front().first) {
            std::pop_heap(heap.begin(), heap.end());
            heap.back() = {d2, j};
            std::push_heap(heap.begin(), heap.end());
          }
        }
      }
    };

    const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
    /* Don't oversubscribe tiny clouds. */
    const int nthreads = (n < 2000) ? 1 : int(std::min(hw, 16u));

    auto worker = [&](int i0, int i1) {
      Monge_fit monge_fit;
      std::vector<std::pair<float, int>> heap;
      heap.reserve(size_t(knn) + 1);
      std::vector<Fast_point> nb;
      nb.reserve(size_t(knn));
      for (int i = i0; i < i1; i++) {
        knn_of(i, heap);
        if (int(heap.size()) < min_pts) {
          continue;
        }
        nb.clear();
        for (const auto &pr : heap) {
          const int j = pr.second;
          nb.emplace_back(double(xyz[size_t(j) * 3 + 0]),
                          double(xyz[size_t(j) * 3 + 1]),
                          double(xyz[size_t(j) * 3 + 2]));
        }
        try {
          typename Monge_fit::Monge_form monge = monge_fit(
              nb.begin(), nb.end(), degree_fitting, degree_monge);
          const double k1 = monge.principal_curvatures(0);
          const double k2 = monge.principal_curvatures(1);
          /* Guard NaN / explode from degenerate neighborhoods. */
          out_k1[size_t(i)] = (std::isfinite(k1) && std::abs(k1) < 1e6) ? float(k1) : 0.0f;
          out_k2[size_t(i)] = (std::isfinite(k2) && std::abs(k2) < 1e6) ? float(k2) : 0.0f;
          auto set_vec = [](std::vector<float> &dst, int ii, const Fast_vector &v) {
            const double vx = v.x(), vy = v.y(), vz = v.z();
            if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
              return;
            }
            dst[size_t(ii) * 3 + 0] = float(vx);
            dst[size_t(ii) * 3 + 1] = float(vy);
            dst[size_t(ii) * 3 + 2] = float(vz);
          };
          set_vec(out_d1, i, monge.maximal_principal_direction());
          set_vec(out_d2, i, monge.minimal_principal_direction());
          set_vec(out_n, i, monge.normal_direction());
        }
        catch (...) {
        }
      }
    };

    if (nthreads == 1) {
      worker(0, n);
    }
    else {
      std::vector<std::thread> threads;
      threads.reserve(size_t(nthreads));
      const int chunk = (n + nthreads - 1) / nthreads;
      for (int t = 0; t < nthreads; t++) {
        const int i0 = t * chunk;
        const int i1 = std::min(n, i0 + chunk);
        if (i0 >= i1) {
          break;
        }
        threads.emplace_back(worker, i0, i1);
      }
      for (auto &th : threads) {
        th.join();
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Monge Jet Fit failed";
    return false;
  }
}

}  // namespace blender::cgal_bridge
