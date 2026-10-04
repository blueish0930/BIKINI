/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 35 - new only (not previously deleted):
 *  - Halfspace Intersection 3D
 *  - Lloyd Optimize 2D
 *  - Conforming Delaunay 2D
 *  - Max Perimeter K-gon 2D
 *  - Weighted Skeleton 2D
 *  - Constrained Delaunay 2D
 *
 * Skipped: P22 OpenGR, G10 Kinetic EPECK, Mesh_3 volume, PolyFit SCIP,
 * Ridges_3, Classification, OSQP Regularize Segments, previously banned
 * measure/query/remesh/component nodes.
 */

#define CGAL_CH3_DUAL_WITHOUT_QP_SOLVER 1

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Convex_hull_3/dual/halfspace_intersection_with_constructions_3.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_vertex_base_2.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Triangulation_conformer_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/create_weighted_straight_skeleton_2.h>
#include <CGAL/extremal_polygon_2.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/number_utils.h>

#include <boost/property_map/property_map.hpp>

#include <algorithm>
#include <iterator>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Plane_3 = Kernel::Plane_3;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using Vb = CGAL::Delaunay_mesh_vertex_base_2<Kernel>;
using Fb = CGAL::Delaunay_mesh_face_base_2<Kernel>;
using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, Tds, CGAL::Exact_predicates_tag>;

static Point_2 to_p2(const float *xyz)
{
  return Point_2(double(xyz[0]), double(xyz[1]));
}

static void unproject_xy(double a, double b, float mid_z, float out[3])
{
  out[0] = float(a);
  out[1] = float(b);
  out[2] = mid_z;
}

static float mid_z_of(const MeshIn &mesh)
{
  if (!mesh.positions || mesh.verts_num <= 0) {
    return 0.0f;
  }
  double s = 0.0;
  for (int i = 0; i < mesh.verts_num; i++) {
    s += double(mesh.positions[i * 3 + 2]);
  }
  return float(s / double(mesh.verts_num));
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

/* Nonzero winding of a ring (Hormann / PNPoly). CCW around q contributes +1. */
static int winding_poly(const Point_2 &q, const std::vector<Point_2> &poly)
{
  const size_t n = poly.size();
  if (n < 3) {
    return 0;
  }
  const double qx = CGAL::to_double(q.x());
  const double qy = CGAL::to_double(q.y());
  int wn = 0;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = CGAL::to_double(poly[i].x());
    const double yi = CGAL::to_double(poly[i].y());
    const double xj = CGAL::to_double(poly[j].x());
    const double yj = CGAL::to_double(poly[j].y());
    if (yj <= qy) {
      if (yi > qy) {
        const double cross = (xi - xj) * (qy - yj) - (qx - xj) * (yi - yj);
        if (cross > 0.0) {
          wn++;
        }
      }
    }
    else if (yi <= qy) {
      const double cross = (xi - xj) * (qy - yj) - (qx - xj) * (yi - yj);
      if (cross < 0.0) {
        wn--;
      }
    }
  }
  return wn;
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
  const size_t step = (std::max)(size_t(1), inner.size() / 8);
  for (size_t i = 0; i < inner.size(); i += step) {
    test(inner[i]);
  }
  return tests > 0 && hits * 2 >= tests;
}

struct PolyIsland {
  std::vector<Point_2> outer;
  std::vector<std::vector<Point_2>> holes;
};

struct Loop2 {
  std::vector<Point_2> verts;
  double area_abs = 0.0;
};

static Loop2 make_loop2(std::vector<Point_2> verts)
{
  Loop2 L;
  L.verts = clean_loop_2(verts);
  L.area_abs = std::abs(signed_area_2(L.verts));
  return L;
}

static void nest_loops_to_islands(std::vector<Loop2> &loops, std::vector<PolyIsland> &islands)
{
  if (loops.empty()) {
    return;
  }
  std::sort(loops.begin(), loops.end(), [](const Loop2 &a, const Loop2 &b) {
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

struct Tri2 {
  Point_2 a, b, c;
};

static bool point_in_tri(const Point_2 &q, const Tri2 &t)
{
  const auto o1 = CGAL::orientation(t.a, t.b, q);
  const auto o2 = CGAL::orientation(t.b, t.c, q);
  const auto o3 = CGAL::orientation(t.c, t.a, q);
  const bool has_pos = o1 == CGAL::LEFT_TURN || o2 == CGAL::LEFT_TURN || o3 == CGAL::LEFT_TURN;
  const bool has_neg = o1 == CGAL::RIGHT_TURN || o2 == CGAL::RIGHT_TURN || o3 == CGAL::RIGHT_TURN;
  return !(has_pos && has_neg);
}

static void collect_input_tris(const MeshIn &mesh, std::vector<Tri2> &tris)
{
  tris.clear();
  if (!mesh.positions || mesh.faces_num <= 0 || !mesh.corner_verts) {
    return;
  }
  auto push_fan = [&](int i0, int i1, int i2) {
    if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= mesh.verts_num || i1 >= mesh.verts_num ||
        i2 >= mesh.verts_num || i0 == i1 || i1 == i2 || i2 == i0)
    {
      return;
    }
    Tri2 t{to_p2(mesh.positions + i0 * 3),
           to_p2(mesh.positions + i1 * 3),
           to_p2(mesh.positions + i2 * 3)};
    if (CGAL::collinear(t.a, t.b, t.c)) {
      return;
    }
    tris.push_back(t);
  };
  if (mesh.face_offsets) {
    for (int f = 0; f < mesh.faces_num; f++) {
      const int begin = mesh.face_offsets[f];
      const int end = mesh.face_offsets[f + 1];
      const int n = end - begin;
      if (n < 3) {
        continue;
      }
      const int v0 = mesh.corner_verts[begin];
      for (int i = 1; i + 1 < n; i++) {
        push_fan(v0, mesh.corner_verts[begin + i], mesh.corner_verts[begin + i + 1]);
      }
    }
  }
  else {
    for (int f = 0; f < mesh.faces_num; f++) {
      push_fan(mesh.corner_verts[f * 3 + 0],
               mesh.corner_verts[f * 3 + 1],
               mesh.corner_verts[f * 3 + 2]);
    }
  }
}

struct TriGrid2 {
  double xmin = 0.0, ymin = 0.0, inv = 1.0;
  int nx = 1, ny = 1;
  std::vector<std::vector<int>> bins;

  void build(const std::vector<Tri2> &tris)
  {
    bins.clear();
    if (tris.empty()) {
      return;
    }
    xmin = ymin = 1e300;
    double xmax = -1e300, ymax = -1e300;
    for (const Tri2 &t : tris) {
      for (const Point_2 *p : {&t.a, &t.b, &t.c}) {
        const double x = CGAL::to_double(p->x());
        const double y = CGAL::to_double(p->y());
        xmin = (std::min)(xmin, x);
        ymin = (std::min)(ymin, y);
        xmax = (std::max)(xmax, x);
        ymax = (std::max)(ymax, y);
      }
    }
    const double dx = (std::max)(xmax - xmin, 1e-12);
    const double dy = (std::max)(ymax - ymin, 1e-12);
    const int n = int(tris.size());
    nx = (std::max)(1, (std::min)(128, int(std::ceil(std::sqrt(double(n))))));
    ny = nx;
    inv = double(nx) / (std::max)(dx, dy);
    bins.assign(size_t(nx * ny), {});
    auto clampi = [](int v, int hi) { return (std::max)(0, (std::min)(v, hi)); };
    for (int i = 0; i < n; i++) {
      const Tri2 &t = tris[size_t(i)];
      const double ax = CGAL::to_double(t.a.x()), ay = CGAL::to_double(t.a.y());
      const double bx = CGAL::to_double(t.b.x()), by = CGAL::to_double(t.b.y());
      const double cx = CGAL::to_double(t.c.x()), cy = CGAL::to_double(t.c.y());
      const int x0 = clampi(int(std::floor(((std::min)({ax, bx, cx}) - xmin) * inv)), nx - 1);
      const int y0 = clampi(int(std::floor(((std::min)({ay, by, cy}) - ymin) * inv)), ny - 1);
      const int x1 = clampi(int(std::floor(((std::max)({ax, bx, cx}) - xmin) * inv)), nx - 1);
      const int y1 = clampi(int(std::floor(((std::max)({ay, by, cy}) - ymin) * inv)), ny - 1);
      for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
          bins[size_t(y * nx + x)].push_back(i);
        }
      }
    }
  }

  bool covers(const Point_2 &q, const std::vector<Tri2> &tris) const
  {
    int count = 0, wind = 0;
    hits(q, tris, count, wind);
    return count > 0;
  }

  void hits(const Point_2 &q, const std::vector<Tri2> &tris, int &r_count, int &r_wind) const
  {
    r_count = 0;
    r_wind = 0;
    if (bins.empty() || tris.empty()) {
      return;
    }
    const double x = CGAL::to_double(q.x());
    const double y = CGAL::to_double(q.y());
    int ix = int(std::floor((x - xmin) * inv));
    int iy = int(std::floor((y - ymin) * inv));
    ix = (std::max)(0, (std::min)(ix, nx - 1));
    iy = (std::max)(0, (std::min)(iy, ny - 1));
    std::vector<char> seen(tris.size(), 0);
    for (int dy = -1; dy <= 1; dy++) {
      for (int dx = -1; dx <= 1; dx++) {
        const int x2 = ix + dx;
        const int y2 = iy + dy;
        if (x2 < 0 || y2 < 0 || x2 >= nx || y2 >= ny) {
          continue;
        }
        for (int idx : bins[size_t(y2 * nx + x2)]) {
          if (seen[size_t(idx)]) {
            continue;
          }
          seen[size_t(idx)] = 1;
          if (!point_in_tri(q, tris[size_t(idx)])) {
            continue;
          }
          r_count++;
          r_wind += (CGAL::orientation(tris[size_t(idx)].a,
                                       tris[size_t(idx)].b,
                                       tris[size_t(idx)].c) == CGAL::LEFT_TURN) ?
                        1 :
                        -1;
        }
      }
    }
  }
};

static void stitch_index_edges(const MeshIn &mesh,
                               const std::vector<std::pair<int, int>> &edges,
                               std::vector<Loop2> &loops)
{
  std::map<int, std::vector<int>> adj;
  for (const auto &e : edges) {
    if (e.first == e.second || e.first < 0 || e.second < 0 || e.first >= mesh.verts_num ||
        e.second >= mesh.verts_num)
    {
      continue;
    }
    adj[e.first].push_back(e.second);
    adj[e.second].push_back(e.first);
  }
  for (auto &kv : adj) {
    auto &nb = kv.second;
    std::sort(nb.begin(), nb.end());
    nb.erase(std::unique(nb.begin(), nb.end()), nb.end());
  }
  std::set<std::pair<int, int>> used_dir;
  for (const auto &kv : adj) {
    const int start = kv.first;
    for (int nb0 : kv.second) {
      if (used_dir.count({start, nb0})) {
        continue;
      }
      std::vector<int> cyc;
      int prev = start;
      int cur = nb0;
      used_dir.insert({start, nb0});
      cyc.push_back(start);
      bool closed = false;
      for (int step = 0; step < mesh.verts_num + 4; step++) {
        cyc.push_back(cur);
        if (cur == start) {
          closed = true;
          break;
        }
        int nxt = -1;
        const auto it = adj.find(cur);
        if (it != adj.end()) {
          for (int nb : it->second) {
            if (nb != prev && !used_dir.count({cur, nb})) {
              nxt = nb;
              break;
            }
          }
        }
        if (nxt < 0) {
          break;
        }
        used_dir.insert({cur, nxt});
        prev = cur;
        cur = nxt;
      }
      if (!closed || cyc.size() < 4) {
        continue;
      }
      cyc.pop_back();
      std::vector<Point_2> raw;
      raw.reserve(cyc.size());
      for (int vi : cyc) {
        raw.push_back(to_p2(mesh.positions + vi * 3));
      }
      Loop2 L = make_loop2(std::move(raw));
      if (L.verts.size() >= 3 && L.area_abs > 1e-20) {
        loops.push_back(std::move(L));
      }
    }
  }
}

static void collect_outline_loops(const MeshIn &mesh, std::vector<Loop2> &loops, std::string &load_err)
{
  loops.clear();

  /* A handful of n-gons (square / circle / square) are independent contours.
   * Do not do this for a triangulation — every triangle would become a ring. */
  const bool few_ngons = mesh.faces_num > 0 && mesh.faces_num <= 16 && mesh.face_offsets &&
                         mesh.corner_verts && mesh.corners_num >= mesh.faces_num * 4;
  if (few_ngons) {
    for (int f = 0; f < mesh.faces_num; f++) {
      const int begin = mesh.face_offsets[f];
      const int end = mesh.face_offsets[f + 1];
      if (end - begin < 3) {
        continue;
      }
      std::vector<Point_2> raw;
      raw.reserve(size_t(end - begin));
      for (int i = begin; i < end; i++) {
        const int v = mesh.corner_verts[i];
        if (v >= 0 && v < mesh.verts_num) {
          raw.push_back(to_p2(mesh.positions + v * 3));
        }
      }
      Loop2 L = make_loop2(std::move(raw));
      if (L.verts.size() >= 3 && L.area_abs > 1e-20) {
        loops.push_back(std::move(L));
      }
    }
    if (!loops.empty()) {
      return;
    }
  }

  Surface_mesh sm;
  if (mesh_in_to_surface_mesh(mesh, sm, load_err, true) && sm.number_of_faces() > 0) {
    triangulate_faces_keep_ids(sm);
    std::vector<Surface_mesh::Halfedge_index> cycle_starts;
    CGAL::extract_boundary_cycles(sm, std::back_inserter(cycle_starts));
    for (auto start : cycle_starts) {
      std::vector<Point_2> raw;
      for (auto h : CGAL::halfedges_around_face(start, sm)) {
        const Point_3 &p = sm.point(sm.source(h));
        float xyz[3] = {float(CGAL::to_double(p.x())),
                        float(CGAL::to_double(p.y())),
                        float(CGAL::to_double(p.z()))};
        raw.push_back(to_p2(xyz));
      }
      Loop2 L = make_loop2(std::move(raw));
      if (L.verts.size() >= 3 && L.area_abs > 1e-20) {
        loops.push_back(std::move(L));
      }
    }
  }

  if (loops.empty() && mesh.faces_num <= 0 && mesh.edges_num > 0 && mesh.edge_v0 && mesh.edge_v1) {
    std::vector<std::pair<int, int>> wire;
    wire.reserve(size_t(mesh.edges_num));
    for (int e = 0; e < mesh.edges_num; e++) {
      wire.emplace_back(mesh.edge_v0[e], mesh.edge_v1[e]);
    }
    stitch_index_edges(mesh, wire, loops);
  }
}

static bool extract_polygon_islands(const MeshIn &mesh,
                                    std::vector<PolyIsland> &islands,
                                    float &mid_z,
                                    std::string &error,
                                    bool allow_hull_fallback = true)
{
  islands.clear();
  if (!mesh.positions || mesh.verts_num < 3) {
    error = "Need at least 3 vertices";
    return false;
  }
  mid_z = mid_z_of(mesh);

  std::string load_err;
  std::vector<Loop2> loops;
  collect_outline_loops(mesh, loops, load_err);
  nest_loops_to_islands(loops, islands);

  if (islands.empty() && allow_hull_fallback) {
    std::vector<Point_2> pts;
    pts.reserve(size_t(mesh.verts_num));
    for (int i = 0; i < mesh.verts_num; i++) {
      pts.push_back(to_p2(mesh.positions + i * 3));
    }
    std::vector<Point_2> hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
    Loop2 L = make_loop2(std::move(hull));
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

static void insert_ring_cdt(CDT &cdt, const std::vector<Point_2> &ring)
{
  if (ring.size() < 2) {
    return;
  }
  std::vector<CDT::Vertex_handle> vhs;
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
}

static bool build_cdt_from_mesh(const MeshIn &mesh,
                                CDT &cdt,
                                float &mid_z,
                                std::string &error,
                                bool constrain_edges = true)
{
  mid_z = mid_z_of(mesh);
  if (!mesh.positions || mesh.verts_num < 3) {
    error = "Need at least 3 vertices";
    return false;
  }

  std::vector<CDT::Vertex_handle> vhs(size_t(mesh.verts_num));
  for (int i = 0; i < mesh.verts_num; i++) {
    vhs[size_t(i)] = cdt.insert(to_p2(mesh.positions + i * 3));
  }
  /* 3D projected meshes have many crossing XY edges — skip constraints and
   * keep triangles by coverage instead. 2D PSLGs still constrain every edge. */
  if (constrain_edges) {
    auto add_edge = [&](int a, int b) {
      if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
        return;
      }
      if (vhs[size_t(a)] == vhs[size_t(b)]) {
        return;
      }
      try {
        cdt.insert_constraint(vhs[size_t(a)], vhs[size_t(b)]);
      }
      catch (...) {
      }
    };
    if (mesh.edges_num > 0 && mesh.edge_v0 && mesh.edge_v1) {
      for (int e = 0; e < mesh.edges_num; e++) {
        add_edge(mesh.edge_v0[e], mesh.edge_v1[e]);
      }
    }
    if (mesh.faces_num > 0 && mesh.corner_verts && mesh.face_offsets) {
      for (int f = 0; f < mesh.faces_num; f++) {
        const int begin = mesh.face_offsets[f];
        const int end = mesh.face_offsets[f + 1];
        const int n = end - begin;
        for (int i = 0; i < n; i++) {
          add_edge(mesh.corner_verts[begin + i], mesh.corner_verts[begin + ((i + 1) % n)]);
        }
      }
    }
  }

  if (cdt.dimension() != 2) {
    std::vector<PolyIsland> islands;
    std::string island_error;
    if (extract_polygon_islands(mesh, islands, mid_z, island_error)) {
      for (const PolyIsland &isl : islands) {
        insert_ring_cdt(cdt, isl.outer);
        for (const auto &h : isl.holes) {
          insert_ring_cdt(cdt, h);
        }
      }
    }
  }
  if (cdt.dimension() != 2) {
    error = error.empty() ? "Degenerate 2D triangulation" : error;
    return false;
  }
  return true;
}

enum class DomainMode {
  Contours, /* Fill Curve style: edge loops / silhouette. */
  Faces,    /* 2D mesh faces: even-odd = odd containment, nonzero = signed winding. */
  Coverage, /* Union of projected faces (fallback when a closed mesh has no silhouette). */
};

struct Domain2 {
  DomainMode mode = DomainMode::Contours;
  std::vector<PolyIsland> islands;
  std::vector<std::vector<Point_2>> rings;
  std::vector<Tri2> tris;
  TriGrid2 grid;
};

static void loops_to_rings(const std::vector<Loop2> &loops, std::vector<std::vector<Point_2>> &rings)
{
  rings.clear();
  rings.reserve(loops.size());
  for (const Loop2 &L : loops) {
    if (L.verts.size() >= 3) {
      rings.push_back(L.verts);
    }
  }
}

static double xy_signed_area3(const Point_2 &a, const Point_2 &b, const Point_2 &c)
{
  return CGAL::to_double((b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y()));
}

/* Fill Curve: all contour edges as one soup. */
static int contour_even_odd_crossings(const Point_2 &q,
                                      const std::vector<std::vector<Point_2>> &rings)
{
  int crossings = 0;
  const double qx = CGAL::to_double(q.x());
  const double qy = CGAL::to_double(q.y());
  for (const auto &ring : rings) {
    const size_t n = ring.size();
    if (n < 3) {
      continue;
    }
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
      const double xi = CGAL::to_double(ring[i].x());
      const double yi = CGAL::to_double(ring[i].y());
      const double xj = CGAL::to_double(ring[j].x());
      const double yj = CGAL::to_double(ring[j].y());
      const bool hit = ((yi > qy) != (yj > qy)) &&
                       (qx < (xj - xi) * (qy - yi) / ((yj - yi) + 1e-30) + xi);
      if (hit) {
        crossings++;
      }
    }
  }
  return crossings;
}

static int contour_winding(const Point_2 &q, const std::vector<std::vector<Point_2>> &rings)
{
  int wn = 0;
  for (const auto &ring : rings) {
    wn += winding_poly(q, ring);
  }
  return wn;
}

static void extract_domain_islands(const MeshIn &mesh, Domain2 &domain, float &mid_z)
{
  domain.islands.clear();
  domain.rings.clear();
  domain.mode = DomainMode::Contours;
  collect_input_tris(mesh, domain.tris);
  if (!domain.tris.empty()) {
    domain.grid.build(domain.tris);
  }
  mid_z = mid_z_of(mesh);

  const bool wire_only = mesh.faces_num <= 0 && mesh.edges_num > 0;
  std::string dummy;
  std::vector<Loop2> loops;

  if (wire_only) {
    collect_outline_loops(mesh, loops, dummy);
    loops_to_rings(loops, domain.rings);
    nest_loops_to_islands(loops, domain.islands);
    domain.mode = DomainMode::Contours;
    return;
  }

  if (!domain.tris.empty()) {
    int pos = 0, neg = 0;
    for (const Tri2 &t : domain.tris) {
      const double s = xy_signed_area3(t.a, t.b, t.c);
      if (s > 1e-18) {
        pos++;
      }
      else if (s < -1e-18) {
        neg++;
      }
    }
    /* Front+back in the XY projection = 3D (or a flipped stack). Union of
     * the projection is the 2D shape. Do not use signed face winding. */
    if (pos > 0 && neg > 0) {
      domain.mode = DomainMode::Coverage;
      return;
    }
    domain.mode = DomainMode::Faces;
    return;
  }

  collect_outline_loops(mesh, loops, dummy);
  loops_to_rings(loops, domain.rings);
  nest_loops_to_islands(loops, domain.islands);
  domain.mode = DomainMode::Contours;
}

static bool centroid_in_domain(const Point_2 &c, const Domain2 &domain, int fill_rule)
{
  if (domain.mode == DomainMode::Faces && !domain.tris.empty()) {
    int count = 0, wind = 0;
    domain.grid.hits(c, domain.tris, count, wind);
    return (fill_rule == 1) ? (wind != 0) : ((count % 2) == 1);
  }
  if (domain.mode == DomainMode::Coverage && !domain.tris.empty()) {
    /* Closed mesh with no usable silhouette: union of the projection. */
    return domain.grid.covers(c, domain.tris);
  }
  if (!domain.rings.empty()) {
    if (fill_rule == 1) {
      return contour_winding(c, domain.rings) != 0;
    }
    return (contour_even_odd_crossings(c, domain.rings) % 2) == 1;
  }
  if (!domain.tris.empty()) {
    int count = 0, wind = 0;
    domain.grid.hits(c, domain.tris, count, wind);
    return (fill_rule == 1) ? (wind != 0) : ((count % 2) == 1);
  }
  return false;
}

static void emit_cdt(MeshResult &result,
                     const CDT &cdt,
                     float mid_z,
                     int fill_rule,
                     const Domain2 *domain)
{
  const bool have_domain = domain &&
                           (!domain->rings.empty() || !domain->tris.empty() ||
                            !domain->islands.empty());
  std::map<CDT::Face_handle, bool> in_domain_map;
  boost::associative_property_map<std::map<CDT::Face_handle, bool>> in_domain(in_domain_map);
  if (!have_domain) {
    /* Wire PSLG fallback: even-odd on every constraint (they are the outline). */
    CGAL::mark_domain_in_triangulation(cdt, in_domain);
  }
  std::map<CDT::Vertex_handle, int> v2i;
  int local = 0;
  auto vid = [&](CDT::Vertex_handle vh) {
    const auto it = v2i.find(vh);
    if (it != v2i.end()) {
      return it->second;
    }
    const int idx = local++;
    v2i[vh] = idx;
    float q[3];
    unproject_xy(CGAL::to_double(vh->point().x()), CGAL::to_double(vh->point().y()), mid_z, q);
    result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
    return idx;
  };
  for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
    if (have_domain) {
      const Point_2 &p0 = fit->vertex(0)->point();
      const Point_2 &p1 = fit->vertex(1)->point();
      const Point_2 &p2 = fit->vertex(2)->point();
      const Point_2 c((p0.x() + p1.x() + p2.x()) / 3.0, (p0.y() + p1.y() + p2.y()) / 3.0);
      if (!centroid_in_domain(c, *domain, fill_rule)) {
        continue;
      }
    }
    else if (!get(in_domain, fit)) {
      continue;
    }
    result.corner_verts.push_back(vid(fit->vertex(0)));
    result.corner_verts.push_back(vid(fit->vertex(1)));
    result.corner_verts.push_back(vid(fit->vertex(2)));
  }
  result.ok = result.corners_num() >= 3;
}

static void append_ngon_xy(MeshResult &result, const std::vector<Point_2> &ring, float mid_z)
{
  if (ring.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets = {0};
  }
  const int base = result.verts_num();
  for (size_t i = 0; i < ring.size(); i++) {
    float q[3];
    unproject_xy(CGAL::to_double(ring[i].x()), CGAL::to_double(ring[i].y()), mid_z, q);
    result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
    result.corner_verts.push_back(base + int(i));
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
}

static float weight_at(const float *weights, int n, int index)
{
  if (!weights || n <= 0 || index < 0 || index >= n) {
    return 1.0f;
  }
  const float w = weights[index];
  return (w > 1e-8f && std::isfinite(w)) ? w : 1.0f;
}

}  // namespace

/* Keep the interior of each face (negative side of an outward-facing plane). */
static void collect_planes_inward(const Surface_mesh &sm, std::vector<Plane_3> &planes)
{
  planes.clear();
  for (const auto f : sm.faces()) {
    auto h = sm.halfedge(f);
    const Point_3 &a = sm.point(sm.source(h));
    const Point_3 &b = sm.point(sm.target(h));
    const Point_3 &c = sm.point(sm.target(sm.next(h)));
    if (CGAL::collinear(a, b, c)) {
      continue;
    }
    planes.emplace_back(a, b, c);
  }
}

static bool strictly_inside_all(const Point_3 &p, const std::vector<Plane_3> &planes)
{
  for (const Plane_3 &pl : planes) {
    if (!pl.has_on_negative_side(p)) {
      return false;
    }
  }
  return true;
}

static Point_3 inward_guess(const Surface_mesh &sm)
{
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  for (const auto f : sm.faces()) {
    auto h = sm.halfedge(f);
    const Point_3 &a = sm.point(sm.source(h));
    const Point_3 &b = sm.point(sm.target(h));
    const Point_3 &c = sm.point(sm.target(sm.next(h)));
    if (CGAL::collinear(a, b, c)) {
      continue;
    }
    const auto v = CGAL::cross_product(b - a, c - a);
    const double len = std::sqrt(CGAL::to_double(v.squared_length()));
    if (len < 1e-18) {
      continue;
    }
    const double cx = (CGAL::to_double(a.x()) + CGAL::to_double(b.x()) + CGAL::to_double(c.x())) /
                      3.0;
    const double cy = (CGAL::to_double(a.y()) + CGAL::to_double(b.y()) + CGAL::to_double(c.y())) /
                      3.0;
    const double cz = (CGAL::to_double(a.z()) + CGAL::to_double(b.z()) + CGAL::to_double(c.z())) /
                      3.0;
    const double eps = 1e-4 * len;
    sx += cx - eps * CGAL::to_double(v.x()) / len;
    sy += cy - eps * CGAL::to_double(v.y()) / len;
    sz += cz - eps * CGAL::to_double(v.z()) / len;
    n++;
  }
  if (n == 0) {
    return Point_3(0, 0, 0);
  }
  return Point_3(sx / double(n), sy / double(n), sz / double(n));
}

static Point_3 push_inside(Point_3 p, const std::vector<Plane_3> &planes)
{
  for (int pass = 0; pass < 8; pass++) {
    bool moved = false;
    for (const Plane_3 &pl : planes) {
      if (pl.has_on_negative_side(p)) {
        continue;
      }
      const auto n = pl.orthogonal_vector();
      const double n2 = CGAL::to_double(n.squared_length());
      if (n2 < 1e-30) {
        continue;
      }
      const double sd = CGAL::to_double(pl.a() * p.x() + pl.b() * p.y() + pl.c() * p.z() + pl.d());
      /* Move onto the negative side. */
      const double step = (sd + 1e-8) / n2;
      p = Point_3(CGAL::to_double(p.x()) - step * CGAL::to_double(n.x()),
                  CGAL::to_double(p.y()) - step * CGAL::to_double(n.y()),
                  CGAL::to_double(p.z()) - step * CGAL::to_double(n.z()));
      moved = true;
    }
    if (!moved) {
      break;
    }
  }
  return p;
}

MeshResult mesh_halfspace_intersection_3(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    Surface_mesh sm;
    std::string error;
    if (!mesh_in_to_surface_mesh(mesh, sm, error, true) || sm.number_of_faces() == 0) {
      result.error = error.empty() ? "Halfspace Intersection 3D needs a mesh with faces" : error;
      return result;
    }
    triangulate_faces_keep_ids(sm);

    std::vector<Plane_3> planes;
    planes.reserve(size_t(sm.number_of_faces()));
    collect_planes_inward(sm, planes);
    if (planes.size() < 4) {
      result.error = "Halfspace Intersection 3D needs ≥4 non-degenerate face planes";
      return result;
    }

    Point_3 origin = inward_guess(sm);
    int neg = 0;
    for (const Plane_3 &pl : planes) {
      if (pl.has_on_negative_side(origin)) {
        neg++;
      }
    }
    if (neg * 2 < int(planes.size())) {
      for (Plane_3 &pl : planes) {
        pl = pl.opposite();
      }
    }
    origin = push_inside(origin, planes);
    if (!strictly_inside_all(origin, planes)) {
      result.error =
          "Halfspace Intersection 3D: no interior point (kernel empty or inverted faces)";
      return result;
    }

    using Polyhedron = CGAL::Polyhedron_3<Kernel>;
    Polyhedron poly;
    CGAL::halfspace_intersection_with_constructions_3(
        planes.begin(), planes.end(), poly, std::make_optional(origin));
    if (poly.size_of_vertices() < 4 || poly.size_of_facets() < 4) {
      result.error =
          "Halfspace Intersection 3D empty - origin must lie strictly inside every half-space";
      return result;
    }

    std::map<const Polyhedron::Vertex *, int> vmap;
    int vi = 0;
    for (auto v = poly.vertices_begin(); v != poly.vertices_end(); ++v) {
      vmap[&*v] = vi++;
      result.positions.push_back(float(CGAL::to_double(v->point().x())));
      result.positions.push_back(float(CGAL::to_double(v->point().y())));
      result.positions.push_back(float(CGAL::to_double(v->point().z())));
    }
    result.face_offsets = {0};
    for (auto f = poly.facets_begin(); f != poly.facets_end(); ++f) {
      auto h = f->facet_begin();
      const auto h0 = h;
      do {
        result.corner_verts.push_back(vmap[&*(h->vertex())]);
      } while (++h != h0);
      if (int(result.corner_verts.size()) - result.face_offsets.back() < 3) {
        result.corner_verts.resize(size_t(result.face_offsets.back()));
        continue;
      }
      result.face_offsets.push_back(int(result.corner_verts.size()));
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Halfspace Intersection 3D produced no faces";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Halfspace Intersection 3D failed";
  }
  return result;
}

bool points_lloyd_relax(const MeshIn &surface,
                        const float *sites_xyz,
                        int sites_n,
                        int max_iters,
                        double convergence,
                        std::vector<float> &out_xyz,
                        std::string &error)
{
  out_xyz.clear();
  if (!sites_xyz || sites_n < 1) {
    error = "Lloyd Relax needs a point cloud";
    return false;
  }
  CgalThrowGuard guard;
  try {
    max_iters = (std::max)(1, max_iters);
    convergence = std::clamp(convergence, 0.0, 1.0);

    std::vector<Point_3> sites;
    sites.resize(size_t(sites_n));
    for (int i = 0; i < sites_n; i++) {
      sites[size_t(i)] = Point_3(double(sites_xyz[i * 3 + 0]),
                                 double(sites_xyz[i * 3 + 1]),
                                 double(sites_xyz[i * 3 + 2]));
    }

    Surface_mesh sm;
    std::string load_err;
    const bool have_faces = mesh_in_to_surface_mesh(surface, sm, load_err, true) &&
                            sm.number_of_faces() > 0;
    if (have_faces) {
      triangulate_faces_keep_ids(sm);
    }

    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
    using AABBTraits = CGAL::AABB_traits_3<Kernel, Primitive>;
    using Tree = CGAL::AABB_tree<AABBTraits>;
    std::unique_ptr<Tree> tree;
    if (have_faces) {
      tree = std::make_unique<Tree>(faces(sm).first, faces(sm).second, sm);
      tree->accelerate_distance_queries();
      for (Point_3 &p : sites) {
        p = tree->closest_point(p);
      }
    }

    struct Sample {
      Point_3 p;
      double w = 1.0;
    };
    std::vector<Sample> samples;
    const int n_target = (std::min)(60000, (std::max)(16 * sites_n, 1500));
    samples.reserve(size_t(n_target) + 64);

    auto add_tri_samples = [&](const Point_3 &a, const Point_3 &b, const Point_3 &c, int n_want) {
      const auto cr = CGAL::cross_product(b - a, c - a);
      const double area = 0.5 * std::sqrt(CGAL::to_double(cr.squared_length()));
      if (area < 1e-20 || n_want < 1) {
        return;
      }
      if (n_want == 1) {
        samples.push_back({Point_3((a.x() + b.x() + c.x()) / 3.0,
                                   (a.y() + b.y() + c.y()) / 3.0,
                                   (a.z() + b.z() + c.z()) / 3.0),
                           area});
        return;
      }
      const int k = (std::max)(2, int(std::ceil(std::sqrt(2.0 * double(n_want)))));
      int count = 0;
      for (int i = 0; i < k; i++) {
        for (int j = 0; j < k - i; j++) {
          count++;
        }
      }
      if (count < 1) {
        return;
      }
      const double w = area / double(count);
      for (int i = 0; i < k; i++) {
        for (int j = 0; j < k - i; j++) {
          const double u = (double(i) + 0.5) / double(k);
          const double v = (double(j) + 0.5) / double(k);
          const double t = (std::max)(0.0, 1.0 - u - v);
          samples.push_back({Point_3(a.x() * t + b.x() * u + c.x() * v,
                                     a.y() * t + b.y() * u + c.y() * v,
                                     a.z() * t + b.z() * u + c.z() * v),
                             w});
        }
      }
    };

    if (have_faces) {
      struct FaceA {
        Point_3 a, b, c;
        double area = 0.0;
      };
      std::vector<FaceA> faces_a;
      double total_area = 0.0;
      for (const auto f : sm.faces()) {
        auto h = sm.halfedge(f);
        FaceA fa;
        fa.a = sm.point(sm.source(h));
        fa.b = sm.point(sm.target(h));
        fa.c = sm.point(sm.target(sm.next(h)));
        const auto cr = CGAL::cross_product(fa.b - fa.a, fa.c - fa.a);
        fa.area = 0.5 * std::sqrt(CGAL::to_double(cr.squared_length()));
        if (fa.area < 1e-20) {
          continue;
        }
        total_area += fa.area;
        faces_a.push_back(fa);
      }
      for (size_t i = 0; i < faces_a.size(); i++) {
        int n_want = 0;
        if (total_area > 0.0) {
          n_want = int(double(n_target) * faces_a[i].area / total_area + 0.5);
        }
        if (n_want < 1) {
          continue;
        }
        add_tri_samples(faces_a[i].a, faces_a[i].b, faces_a[i].c, n_want);
      }
      if (samples.empty() && !faces_a.empty()) {
        for (size_t i = 0; i < faces_a.size() && int(samples.size()) < n_target; i++) {
          add_tri_samples(faces_a[i].a, faces_a[i].b, faces_a[i].c, 1);
        }
      }
    }
    else {
      Domain2 domain;
      float mid_z = 0.0f;
      extract_domain_islands(surface, domain, mid_z);
      if (domain.rings.empty() && domain.islands.empty() && domain.tris.empty()) {
        error = load_err.empty() ? "Lloyd Relax needs a surface mesh or an XY polygon" : load_err;
        return false;
      }
      double xmin = 1e300, ymin = 1e300, xmax = -1e300, ymax = -1e300;
      auto acc = [&](const Point_2 &p) {
        xmin = (std::min)(xmin, CGAL::to_double(p.x()));
        ymin = (std::min)(ymin, CGAL::to_double(p.y()));
        xmax = (std::max)(xmax, CGAL::to_double(p.x()));
        ymax = (std::max)(ymax, CGAL::to_double(p.y()));
      };
      for (const auto &ring : domain.rings) {
        for (const Point_2 &p : ring) {
          acc(p);
        }
      }
      for (const PolyIsland &isl : domain.islands) {
        for (const Point_2 &p : isl.outer) {
          acc(p);
        }
      }
      for (const Tri2 &t : domain.tris) {
        acc(t.a);
        acc(t.b);
        acc(t.c);
      }
      const double area = (std::max)((xmax - xmin) * (ymax - ymin), 1e-12);
      const double step = std::sqrt(area / double((std::max)(n_target, 1)));
      const double w = step * step;
      for (double y = ymin + 0.5 * step; y < ymax; y += step) {
        for (double x = xmin + 0.5 * step; x < xmax; x += step) {
          const Point_2 q(x, y);
          if (!centroid_in_domain(q, domain, 0)) {
            continue;
          }
          samples.push_back({Point_3(x, y, double(mid_z)), w});
        }
      }
      for (Point_3 &p : sites) {
        const Point_2 q(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
        if (!centroid_in_domain(q, domain, 0) && !samples.empty()) {
          p = samples.front().p;
        }
        else {
          p = Point_3(CGAL::to_double(p.x()), CGAL::to_double(p.y()), double(mid_z));
        }
      }
    }

    if (samples.empty()) {
      error = "Lloyd Relax: no surface samples";
      return false;
    }

    const int nsamp = int(samples.size());
    std::vector<float> smp_x;
    std::vector<float> smp_y;
    std::vector<float> smp_z;
    std::vector<float> smp_w;
    smp_x.resize(size_t(nsamp));
    smp_y.resize(size_t(nsamp));
    smp_z.resize(size_t(nsamp));
    smp_w.resize(size_t(nsamp));
    float xmin = 1e30f, ymin = 1e30f, zmin = 1e30f;
    float xmax = -1e30f, ymax = -1e30f, zmax = -1e30f;
    for (int i = 0; i < nsamp; i++) {
      smp_x[size_t(i)] = float(CGAL::to_double(samples[size_t(i)].p.x()));
      smp_y[size_t(i)] = float(CGAL::to_double(samples[size_t(i)].p.y()));
      smp_z[size_t(i)] = float(CGAL::to_double(samples[size_t(i)].p.z()));
      smp_w[size_t(i)] = float(samples[size_t(i)].w);
      xmin = (std::min)(xmin, smp_x[size_t(i)]);
      ymin = (std::min)(ymin, smp_y[size_t(i)]);
      zmin = (std::min)(zmin, smp_z[size_t(i)]);
      xmax = (std::max)(xmax, smp_x[size_t(i)]);
      ymax = (std::max)(ymax, smp_y[size_t(i)]);
      zmax = (std::max)(zmax, smp_z[size_t(i)]);
    }
    samples.clear();
    samples.shrink_to_fit();

    const float dxb = (std::max)(xmax - xmin, 1e-6f);
    const float dyb = (std::max)(ymax - ymin, 1e-6f);
    const float dzb = (std::max)(zmax - zmin, 1e-6f);
    const double diag2 = double(dxb) * dxb + double(dyb) * dyb + double(dzb) * dzb;
    const double stop = convergence * convergence * (std::max)(diag2, 1e-12);

    std::vector<float> site_x;
    std::vector<float> site_y;
    std::vector<float> site_z;
    site_x.resize(size_t(sites_n));
    site_y.resize(size_t(sites_n));
    site_z.resize(size_t(sites_n));
    for (int i = 0; i < sites_n; i++) {
      site_x[size_t(i)] = float(CGAL::to_double(sites[size_t(i)].x()));
      site_y[size_t(i)] = float(CGAL::to_double(sites[size_t(i)].y()));
      site_z[size_t(i)] = float(CGAL::to_double(sites[size_t(i)].z()));
    }

    struct SiteHash {
      float ox = 0, oy = 0, oz = 0, inv = 1;
      int nx = 1, ny = 1, nz = 1;
      std::vector<int> head;
      std::vector<int> next;
      void build(const std::vector<float> &px,
                 const std::vector<float> &py,
                 const std::vector<float> &pz,
                 int n,
                 float xmin,
                 float ymin,
                 float zmin,
                 float dx,
                 float dy,
                 float dz)
      {
        const int side = (std::max)(8, (std::min)(48, int(std::cbrt(double((std::max)(n, 1))) * 2.0f + 1)));
        nx = ny = nz = side;
        ox = xmin;
        oy = ymin;
        oz = zmin;
        const float ext = (std::max)(dx, (std::max)(dy, dz));
        inv = float(side) / (std::max)(ext, 1e-6f);
        head.assign(size_t(nx * ny * nz), -1);
        next.assign(size_t(n), -1);
        auto clampi = [](int v, int hi) { return (std::max)(0, (std::min)(v, hi)); };
        for (int i = 0; i < n; i++) {
          const int ix = clampi(int(std::floor((px[size_t(i)] - ox) * inv)), nx - 1);
          const int iy = clampi(int(std::floor((py[size_t(i)] - oy) * inv)), ny - 1);
          const int iz = clampi(int(std::floor((pz[size_t(i)] - oz) * inv)), nz - 1);
          const int c = (iz * ny + iy) * nx + ix;
          next[size_t(i)] = head[size_t(c)];
          head[size_t(c)] = i;
        }
      }
      int nearest(float qx,
                  float qy,
                  float qz,
                  const std::vector<float> &px,
                  const std::vector<float> &py,
                  const std::vector<float> &pz,
                  int n) const
      {
        auto clampi = [](int v, int hi) { return (std::max)(0, (std::min)(v, hi)); };
        const int ix = clampi(int(std::floor((qx - ox) * inv)), nx - 1);
        const int iy = clampi(int(std::floor((qy - oy) * inv)), ny - 1);
        const int iz = clampi(int(std::floor((qz - oz) * inv)), nz - 1);
        int best = -1;
        float best_d = 1e30f;
        const float cell = 1.0f / (std::max)(inv, 1e-12f);
        for (int r = 0; r <= 6; r++) {
          const int x0 = clampi(ix - r, nx - 1), x1 = clampi(ix + r, nx - 1);
          const int y0 = clampi(iy - r, ny - 1), y1 = clampi(iy + r, ny - 1);
          const int z0 = clampi(iz - r, nz - 1), z1 = clampi(iz + r, nz - 1);
          for (int z = z0; z <= z1; z++) {
            for (int y = y0; y <= y1; y++) {
              for (int x = x0; x <= x1; x++) {
                if (r > 0 && x != x0 && x != x1 && y != y0 && y != y1 && z != z0 && z != z1) {
                  continue;
                }
                for (int i = head[size_t((z * ny + y) * nx + x)]; i >= 0; i = next[size_t(i)]) {
                  const float ddx = px[size_t(i)] - qx;
                  const float ddy = py[size_t(i)] - qy;
                  const float ddz = pz[size_t(i)] - qz;
                  const float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                  if (d2 < best_d) {
                    best_d = d2;
                    best = i;
                  }
                }
              }
            }
          }
          const float lim = (float(r) + 0.75f) * cell;
          if (best >= 0 && best_d <= lim * lim) {
            return best;
          }
        }
        if (best >= 0) {
          return best;
        }
        for (int i = 0; i < n; i++) {
          const float ddx = px[size_t(i)] - qx;
          const float ddy = py[size_t(i)] - qy;
          const float ddz = pz[size_t(i)] - qz;
          const float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
          if (d2 < best_d) {
            best_d = d2;
            best = i;
          }
        }
        return (std::max)(best, 0);
      }
    };

    std::vector<double> acc_x(size_t(sites_n), 0.0);
    std::vector<double> acc_y(size_t(sites_n), 0.0);
    std::vector<double> acc_z(size_t(sites_n), 0.0);
    std::vector<double> acc_w(size_t(sites_n), 0.0);
    SiteHash hash;
    for (int it = 0; it < max_iters; it++) {
      hash.build(site_x, site_y, site_z, sites_n, xmin, ymin, zmin, dxb, dyb, dzb);
      std::fill(acc_x.begin(), acc_x.end(), 0.0);
      std::fill(acc_y.begin(), acc_y.end(), 0.0);
      std::fill(acc_z.begin(), acc_z.end(), 0.0);
      std::fill(acc_w.begin(), acc_w.end(), 0.0);
      for (int s = 0; s < nsamp; s++) {
        const int i = hash.nearest(
            smp_x[size_t(s)], smp_y[size_t(s)], smp_z[size_t(s)], site_x, site_y, site_z, sites_n);
        acc_x[size_t(i)] += double(smp_x[size_t(s)]) * double(smp_w[size_t(s)]);
        acc_y[size_t(i)] += double(smp_y[size_t(s)]) * double(smp_w[size_t(s)]);
        acc_z[size_t(i)] += double(smp_z[size_t(s)]) * double(smp_w[size_t(s)]);
        acc_w[size_t(i)] += double(smp_w[size_t(s)]);
      }
      double max_move2 = 0.0;
      for (int i = 0; i < sites_n; i++) {
        float nxp = site_x[size_t(i)];
        float nyp = site_y[size_t(i)];
        float nzp = site_z[size_t(i)];
        if (acc_w[size_t(i)] > 0.0) {
          nxp = float(acc_x[size_t(i)] / acc_w[size_t(i)]);
          nyp = float(acc_y[size_t(i)] / acc_w[size_t(i)]);
          nzp = float(acc_z[size_t(i)] / acc_w[size_t(i)]);
        }
        else {
          const int nb = hash.nearest(nxp, nyp, nzp, site_x, site_y, site_z, sites_n);
          if (nb >= 0 && nb != i) {
            nxp += 0.35f * (site_x[size_t(i)] - site_x[size_t(nb)]);
            nyp += 0.35f * (site_y[size_t(i)] - site_y[size_t(nb)]);
            nzp += 0.35f * (site_z[size_t(i)] - site_z[size_t(nb)]);
          }
        }
        if (tree) {
          const Point_3 np = tree->closest_point(Point_3(double(nxp), double(nyp), double(nzp)));
          nxp = float(CGAL::to_double(np.x()));
          nyp = float(CGAL::to_double(np.y()));
          nzp = float(CGAL::to_double(np.z()));
        }
        const float ddx = nxp - site_x[size_t(i)];
        const float ddy = nyp - site_y[size_t(i)];
        const float ddz = nzp - site_z[size_t(i)];
        max_move2 = (std::max)(max_move2, double(ddx * ddx + ddy * ddy + ddz * ddz));
        site_x[size_t(i)] = nxp;
        site_y[size_t(i)] = nyp;
        site_z[size_t(i)] = nzp;
      }
      if (max_move2 <= stop) {
        break;
      }
    }

    out_xyz.resize(size_t(sites_n) * 3);
    for (int i = 0; i < sites_n; i++) {
      out_xyz[size_t(i) * 3 + 0] = site_x[size_t(i)];
      out_xyz[size_t(i) * 3 + 1] = site_y[size_t(i)];
      out_xyz[size_t(i) * 3 + 2] = site_z[size_t(i)];
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Lloyd Relax failed";
    return false;
  }
}

MeshResult mesh_conforming_delaunay_2(const MeshIn &mesh, int mode)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    CDT cdt;
    float mid_z = 0.0f;
    std::string error;
    Domain2 domain;
    extract_domain_islands(mesh, domain, mid_z);
    if (!build_cdt_from_mesh(mesh, cdt, mid_z, error)) {
      result.error = error.empty() ? "Conforming Delaunay 2D needs an XY polygon" : error;
      return result;
    }
    if (mode == 1) {
      CGAL::make_conforming_Gabriel_2(cdt);
    }
    else {
      CGAL::make_conforming_Delaunay_2(cdt);
    }
    emit_cdt(result, cdt, mid_z, 0, &domain);
    if (!result.ok) {
      result.error = "Conforming Delaunay 2D produced no interior faces";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Conforming Delaunay 2D failed";
  }
  return result;
}

MeshResult points_max_perimeter_k_gon_2(const float *positions, int n, int k)
{
  MeshResult result;
  if (!positions || n < 3) {
    result.error = "Max Perimeter K-gon 2D needs at least 3 points";
    return result;
  }
  CgalThrowGuard guard;
  try {
    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    double sz = 0.0;
    for (int i = 0; i < n; i++) {
      pts.push_back(to_p2(positions + i * 3));
      sz += double(positions[i * 3 + 2]);
    }
    const float mid_z = float(sz / double(n));
    std::vector<Point_2> hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
    if (int(hull.size()) < 3) {
      result.error = "Degenerate hull";
      return result;
    }
    k = std::clamp(k, 3, int(hull.size()));
    std::vector<Point_2> gon;
    CGAL::maximum_perimeter_inscribed_k_gon_2(
        hull.begin(), hull.end(), k, std::back_inserter(gon));
    if (gon.size() < 3) {
      result.error = "K-gon failed";
      return result;
    }
    append_ngon_xy(result, gon, mid_z);
    result.ok = result.faces_num() > 0;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Max Perimeter K-gon 2D failed";
  }
  return result;
}

WireResult mesh_weighted_skeleton_2(const MeshIn &mesh, const float *vert_weights, int weights_n)
{
  WireResult result;
  CgalThrowGuard guard;
  try {
    Surface_mesh sm;
    std::string error;
    float mid_z = mid_z_of(mesh);
    std::vector<std::pair<std::vector<Point_2>, std::vector<double>>> contours;

    if (mesh_in_to_surface_mesh(mesh, sm, error, true) && sm.number_of_faces() > 0) {
      triangulate_faces_keep_ids(sm);
      auto src_map = sm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
      std::vector<Surface_mesh::Halfedge_index> cycle_starts;
      CGAL::extract_boundary_cycles(sm, std::back_inserter(cycle_starts));
      for (auto start : cycle_starts) {
        std::vector<Point_2> ring;
        std::vector<double> wts;
        for (auto h : CGAL::halfedges_around_face(start, sm)) {
          const auto v = sm.source(h);
          const Point_3 &p = sm.point(v);
          float xyz[3] = {float(CGAL::to_double(p.x())),
                          float(CGAL::to_double(p.y())),
                          float(CGAL::to_double(p.z()))};
          ring.push_back(to_p2(xyz));
          int src = int(v);
          if (src_map.has_value()) {
            src = (*src_map)[v];
          }
          wts.push_back(double(weight_at(vert_weights, weights_n, src)));
        }
        ring = clean_loop_2(ring);
        if (ring.size() < 3) {
          continue;
        }
        if (wts.size() != ring.size()) {
          wts.assign(ring.size(), 1.0);
        }
        if (signed_area_2(ring) < 0.0) {
          std::reverse(ring.begin(), ring.end());
          std::reverse(wts.begin(), wts.end());
        }
        contours.push_back({std::move(ring), std::move(wts)});
      }
    }

    if (contours.empty()) {
      std::vector<PolyIsland> islands;
      if (!extract_polygon_islands(mesh, islands, mid_z, error) || islands.empty()) {
        result.error = error.empty() ? "Weighted Skeleton 2D needs a polygon" : error;
        return result;
      }
      for (const PolyIsland &isl : islands) {
        std::vector<double> wts(isl.outer.size(), 1.0);
        contours.push_back({isl.outer, std::move(wts)});
      }
    }

    int vert_base = 0;
    for (const auto &cw : contours) {
      if (cw.first.size() < 3 || cw.second.size() != cw.first.size()) {
        continue;
      }
      auto ss = CGAL::create_interior_weighted_straight_skeleton_2(
          cw.first.begin(), cw.first.end(), cw.second.begin(), cw.second.end());
      if (!ss) {
        continue;
      }
      using Ss = std::remove_reference_t<decltype(*ss)>;
      std::map<typename Ss::Vertex_const_handle, int> vmap;
      for (auto vit = ss->vertices_begin(); vit != ss->vertices_end(); ++vit) {
        float q[3];
        unproject_xy(CGAL::to_double(vit->point().x()),
                     CGAL::to_double(vit->point().y()),
                     mid_z,
                     q);
        vmap[vit] = vert_base++;
        result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
      }
      for (auto hit = ss->halfedges_begin(); hit != ss->halfedges_end(); ++hit) {
        if (!hit->is_bisector()) {
          continue;
        }
        if (&*hit < &*(hit->opposite())) {
          continue;
        }
        auto ia = vmap.find(hit->vertex());
        auto ib = vmap.find(hit->opposite()->vertex());
        if (ia == vmap.end() || ib == vmap.end()) {
          continue;
        }
        result.edge_v0.push_back(ia->second);
        result.edge_v1.push_back(ib->second);
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Weighted Skeleton 2D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Weighted Skeleton 2D failed";
  }
  return result;
}

MeshResult mesh_constrained_delaunay_2(const MeshIn &mesh, int fill_rule)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    CDT cdt;
    float mid_z = 0.0f;
    std::string error;
    Domain2 domain;
    extract_domain_islands(mesh, domain, mid_z);
    if (!build_cdt_from_mesh(mesh, cdt, mid_z, error)) {
      result.error = error.empty() ? "Constrained Delaunay 2D needs an XY polygon" : error;
      return result;
    }
    emit_cdt(result, cdt, mid_z, fill_rule, &domain);
    if (!result.ok) {
      result.error = "Constrained Delaunay 2D produced no faces";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Constrained Delaunay 2D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
