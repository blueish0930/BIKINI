/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 36 - new only (not previously deleted):
 *  - Exterior Skeleton 2D
 *  - Weighted Offset 2D
 *  - Voronoi on Sphere
 *  - Round Offset 2D
 *  Stream Lines 2D deleted (legacy 2513 reserved).
 *  Upper Envelope 3D deleted (legacy 2514 reserved).
 *
 * Skipped: P22 OpenGR, G10 Kinetic EPECK, Mesh_3 volume, PolyFit SCIP,
 * Ridges_3, Classification, OSQP Regularize Segments, previously banned
 * measure/query/remesh/component nodes. Hyperbolic Delaunay needs
 * EPECK-with-sqrt.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Bbox_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/arrange_offset_polygons_2.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/compute_outer_frame_margin.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <CGAL/create_weighted_offset_polygons_2.h>
#include <CGAL/create_weighted_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/create_weighted_straight_skeleton_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/number_utils.h>
#include <CGAL/squared_distance_2.h>

#include <boost/property_map/property_map.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Vector_2 = Kernel::Vector_2;
using Vector_3 = Kernel::Vector_3;
using Segment_2 = Kernel::Segment_2;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using Polygon_with_holes_2 = CGAL::Polygon_with_holes_2<Kernel>;

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

static std::vector<Point_2> collapse_collinear(std::vector<Point_2> ring)
{
  if (ring.size() < 4) {
    return ring;
  }
  for (int pass = 0; pass < 8; pass++) {
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
      const double abx = CGAL::to_double(b.x() - a.x());
      const double aby = CGAL::to_double(b.y() - a.y());
      const double bcx = CGAL::to_double(c.x() - b.x());
      const double bcy = CGAL::to_double(c.y() - b.y());
      const double ab = std::hypot(abx, aby);
      const double bc = std::hypot(bcx, bcy);
      if (ab < 1e-14 || bc < 1e-14) {
        continue;
      }
      const double cross = abx * bcy - aby * bcx;
      if (std::abs(cross) < 1e-9 * ab * bc) {
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
  const double qx = CGAL::to_double(q.x());
  const double qy = CGAL::to_double(q.y());
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = CGAL::to_double(poly[i].x());
    const double yi = CGAL::to_double(poly[i].y());
    const double xj = CGAL::to_double(poly[j].x());
    const double yj = CGAL::to_double(poly[j].y());
    const bool hit = ((yi > qy) != (yj > qy)) &&
                     (qx < (xj - xi) * (qy - yi) / ((yj - yi) + 1e-30) + xi);
    if (hit) {
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
  const size_t n = loops.size();
  /* Smallest containing loop is the parent. Even depth = solid, odd = hole.
   * This keeps an island sitting inside a hole as its own solid, instead of
   * incorrectly tagging it as a second hole of the outer. */
  std::vector<int> parent(n, -1);
  for (size_t i = 0; i < n; i++) {
    for (size_t j = i + 1; j < n; j++) {
      if (!loop_inside_loop(loops[j].verts, loops[i].verts)) {
        continue;
      }
      if (parent[j] < 0 || loops[size_t(parent[j])].area_abs > loops[i].area_abs) {
        parent[j] = int(i);
      }
    }
  }
  std::vector<int> depth(n, 0);
  for (size_t i = 0; i < n; i++) {
    int d = 0;
    int p = parent[i];
    int guard = 0;
    while (p >= 0 && guard++ < int(n) + 2) {
      d++;
      p = parent[size_t(p)];
    }
    depth[i] = d;
  }
  for (size_t i = 0; i < n; i++) {
    if ((depth[i] % 2) != 0) {
      continue;
    }
    PolyIsland isl;
    isl.outer = loops[i].verts;
    if (signed_area_2(isl.outer) < 0.0) {
      std::reverse(isl.outer.begin(), isl.outer.end());
    }
    for (size_t j = 0; j < n; j++) {
      if (parent[j] != int(i)) {
        continue;
      }
      std::vector<Point_2> hole = loops[j].verts;
      if (signed_area_2(hole) > 0.0) {
        std::reverse(hole.begin(), hole.end());
      }
      isl.holes.push_back(std::move(hole));
    }
    islands.push_back(std::move(isl));
  }
}

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

static void collect_face_loops(const MeshIn &mesh, std::vector<Loop2> &loops)
{
  if (mesh.faces_num <= 0 || !mesh.corner_verts) {
    return;
  }
  for (int f = 0; f < mesh.faces_num; f++) {
    int begin = 0;
    int end = 0;
    if (mesh.face_offsets) {
      begin = mesh.face_offsets[f];
      end = mesh.face_offsets[f + 1];
    }
    else {
      begin = f * 3;
      end = begin + 3;
    }
    if (end - begin < 3 || begin < 0 || end > mesh.corners_num) {
      continue;
    }
    std::vector<Point_2> raw;
    raw.reserve(size_t(end - begin));
    bool ok = true;
    for (int i = begin; i < end; i++) {
      const int v = mesh.corner_verts[i];
      if (v < 0 || v >= mesh.verts_num) {
        ok = false;
        break;
      }
      raw.push_back(to_p2(mesh.positions + v * 3));
    }
    if (!ok) {
      continue;
    }
    Loop2 L = make_loop2(std::move(raw));
    if (L.verts.size() >= 3 && L.area_abs > 1e-20) {
      loops.push_back(std::move(L));
    }
  }
}

static void collect_boundary_loops(const MeshIn &mesh, std::vector<Loop2> &loops, std::string &load_err)
{
  Surface_mesh sm;
  if (mesh.faces_num <= 0 || !mesh_in_to_surface_mesh(mesh, sm, load_err, true) ||
      sm.number_of_faces() == 0)
  {
    return;
  }
  auto pull_cycles = [&]() {
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
        if (raw.size() > size_t(mesh.verts_num) + 16) {
          break;
        }
      }
      Loop2 L = make_loop2(std::move(raw));
      if (L.verts.size() >= 3 && L.area_abs > 1e-20) {
        loops.push_back(std::move(L));
      }
    }
  };
  /* Prefer the original n-gon borders so a hole is one cycle, not a fan. */
  pull_cycles();
  if (loops.empty() && !CGAL::is_triangle_mesh(sm)) {
    triangulate_faces_keep_ids(sm);
    pull_cycles();
  }
}

/* Walk MeshIn face half-edges directly. Shared edges cancel; leftover cycles
 * are the outer border and every hole — no Surface_mesh required. */
static void collect_mesh_boundary_loops(const MeshIn &mesh, std::vector<Loop2> &loops)
{
  if (mesh.faces_num <= 0 || !mesh.corner_verts || !mesh.positions) {
    return;
  }
  std::map<std::pair<int, int>, int> dir_count;
  std::map<std::pair<int, int>, int> undirected;
  for (int f = 0; f < mesh.faces_num; f++) {
    int begin = 0;
    int end = 0;
    if (mesh.face_offsets) {
      begin = mesh.face_offsets[f];
      end = mesh.face_offsets[f + 1];
    }
    else {
      begin = f * 3;
      end = begin + 3;
    }
    if (end - begin < 3 || begin < 0 || end > mesh.corners_num) {
      continue;
    }
    std::vector<int> vs;
    vs.reserve(size_t(end - begin));
    bool ok = true;
    for (int i = begin; i < end; i++) {
      const int v = mesh.corner_verts[i];
      if (v < 0 || v >= mesh.verts_num) {
        ok = false;
        break;
      }
      if (vs.empty() || vs.back() != v) {
        vs.push_back(v);
      }
    }
    if (!ok) {
      continue;
    }
    if (vs.size() >= 2 && vs.front() == vs.back()) {
      vs.pop_back();
    }
    if (vs.size() < 3) {
      continue;
    }
    for (size_t i = 0; i < vs.size(); i++) {
      const int a = vs[i];
      const int b = vs[(i + 1) % vs.size()];
      dir_count[{a, b}]++;
      const int lo = (a < b) ? a : b;
      const int hi = (a < b) ? b : a;
      undirected[{lo, hi}]++;
    }
  }
  if (dir_count.empty()) {
    return;
  }

  std::map<int, std::vector<int>> succ;
  std::vector<std::pair<int, int>> boundary;
  for (const auto &kv : dir_count) {
    const int a = kv.first.first;
    const int b = kv.first.second;
    const int lo = (a < b) ? a : b;
    const int hi = (a < b) ? b : a;
    /* Undirected count==1 is a true border. Opposite-winding neighbours
     * still cancel; same-winding duplicates are treated as interior. */
    const auto uit = undirected.find({lo, hi});
    if (uit == undirected.end() || uit->second != 1) {
      continue;
    }
    succ[a].push_back(b);
    boundary.emplace_back(a, b);
  }
  if (boundary.empty()) {
    return;
  }

  auto pick_next = [&](int from, int via, const std::set<std::pair<int, int>> &used) -> int {
    const auto it = succ.find(via);
    if (it == succ.end()) {
      return -1;
    }
    int best = -1;
    double best_ang = -1e300;
    const double ix = double(mesh.positions[via * 3 + 0] - mesh.positions[from * 3 + 0]);
    const double iy = double(mesh.positions[via * 3 + 1] - mesh.positions[from * 3 + 1]);
    const double iL = std::hypot(ix, iy);
    const double inx = (iL > 1e-18) ? ix / iL : 1.0;
    const double iny = (iL > 1e-18) ? iy / iL : 0.0;
    for (int cand : it->second) {
      if (used.count({via, cand})) {
        continue;
      }
      const double ox = double(mesh.positions[cand * 3 + 0] - mesh.positions[via * 3 + 0]);
      const double oy = double(mesh.positions[cand * 3 + 1] - mesh.positions[via * 3 + 1]);
      const double oL = std::hypot(ox, oy);
      const double onx = (oL > 1e-18) ? ox / oL : 1.0;
      const double ony = (oL > 1e-18) ? oy / oL : 0.0;
      /* Left-most continuation keeps us on the region boundary. */
      const double cross = inx * ony - iny * onx;
      const double dot = inx * onx + iny * ony;
      const double ang = std::atan2(cross, dot);
      if (best < 0 || ang > best_ang) {
        best_ang = ang;
        best = cand;
      }
    }
    return best;
  };

  std::set<std::pair<int, int>> used;
  for (const auto &e0 : boundary) {
    if (used.count(e0)) {
      continue;
    }
    std::vector<int> cyc;
    int a = e0.first;
    int b = e0.second;
    bool closed = false;
    for (int step = 0; step < mesh.verts_num + 8; step++) {
      if (used.count({a, b})) {
        break;
      }
      used.insert({a, b});
      cyc.push_back(a);
      const int nxt = pick_next(a, b, used);
      if (nxt < 0) {
        break;
      }
      a = b;
      b = nxt;
      if (a == e0.first && b == e0.second) {
        closed = true;
        break;
      }
    }
    if (!closed || cyc.size() < 3) {
      continue;
    }
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

static void collect_outline_loops(const MeshIn &mesh, std::vector<Loop2> &loops, std::string &load_err)
{
  loops.clear();
  /* Face half-edges first: this is the reliable hole source (donut quads,
   * nested n-gons, Fill Curve). Surface_mesh is only a fallback. */
  collect_mesh_boundary_loops(mesh, loops);
  if (loops.empty()) {
    collect_boundary_loops(mesh, loops, load_err);
  }
  if (loops.empty()) {
    collect_face_loops(mesh, loops);
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

static std::vector<Point_2> poly_to_ring(const Polygon_2 &poly)
{
  return std::vector<Point_2>(poly.vertices_begin(), poly.vertices_end());
}

static double poly_area_abs(const Polygon_2 &poly)
{
  return std::abs(signed_area_2(poly_to_ring(poly)));
}

/* Never call Polygon_2::is_clockwise_oriented / is_simple — they precondition
 * simplicity and abort() on typical Blender outlines (collinear, keyhole). */
static Polygon_2 ring_to_oriented_poly(std::vector<Point_2> ring, bool ccw)
{
  ring = clean_loop_2(std::move(ring));
  if (ring.size() < 3) {
    return Polygon_2();
  }
  const double a = signed_area_2(ring);
  if (ccw && a < 0.0) {
    std::reverse(ring.begin(), ring.end());
  }
  if (!ccw && a > 0.0) {
    std::reverse(ring.begin(), ring.end());
  }
  return Polygon_2(ring.begin(), ring.end());
}

static Polygon_2 ring_to_ccw_poly(const std::vector<Point_2> &ring)
{
  return ring_to_oriented_poly(ring, true);
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

static void append_poly_xy(MeshResult &result, const Polygon_2 &poly, float mid_z)
{
  std::vector<Point_2> ring;
  ring.reserve(size_t(poly.size()));
  for (auto it = poly.vertices_begin(); it != poly.vertices_end(); ++it) {
    ring.push_back(*it);
  }
  ring = clean_loop_2(ring);
  if (signed_area_2(ring) < 0.0) {
    std::reverse(ring.begin(), ring.end());
  }
  append_ngon_xy(result, ring, mid_z);
}

static float weight_at(const float *weights, int n, int index)
{
  if (!weights || n <= 0 || index < 0 || index >= n) {
    return 1.0f;
  }
  const float w = weights[index];
  return (w > 1e-8f && std::isfinite(w)) ? w : 1.0f;
}

static std::vector<double> weights_for_ring(const std::vector<Point_2> &ring,
                                            const MeshIn &mesh,
                                            const float *vert_weights,
                                            int weights_n)
{
  std::vector<double> wts(ring.size(), 1.0);
  if (!mesh.positions || mesh.verts_num <= 0) {
    return wts;
  }
  for (size_t i = 0; i < ring.size(); i++) {
    const double qx = CGAL::to_double(ring[i].x());
    const double qy = CGAL::to_double(ring[i].y());
    int best = 0;
    double best_d = 1e300;
    for (int v = 0; v < mesh.verts_num; v++) {
      const double dx = double(mesh.positions[v * 3 + 0]) - qx;
      const double dy = double(mesh.positions[v * 3 + 1]) - qy;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d) {
        best_d = d2;
        best = v;
      }
    }
    wts[i] = double(weight_at(vert_weights, weights_n, best));
  }
  return wts;
}

static void uniquify_ring(std::vector<int> &ring)
{
  if (ring.empty()) {
    return;
  }
  std::vector<int> out;
  out.reserve(ring.size());
  for (int v : ring) {
    if (out.empty() || out.back() != v) {
      out.push_back(v);
    }
  }
  if (out.size() >= 2 && out.front() == out.back()) {
    out.pop_back();
  }
  ring.swap(out);
}

static void append_isolated_xyz_ngon(MeshResult &result,
                                     const std::vector<float> &xyz,
                                     int tag)
{
  const int n = int(xyz.size() / 3);
  if (n < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets = {0};
  }
  const int base = result.verts_num();
  result.positions.insert(result.positions.end(), xyz.begin(), xyz.end());
  for (int i = 0; i < n; i++) {
    result.corner_verts.push_back(base + i);
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
}

static int weld_xyz_vert(MeshResult &result,
                         std::map<std::array<long long, 3>, int> &weld,
                         float x,
                         float y,
                         float z,
                         double scale)
{
  const std::array<long long, 3> key = {llround(double(x) * scale),
                                        llround(double(y) * scale),
                                        llround(double(z) * scale)};
  const auto it = weld.find(key);
  if (it != weld.end()) {
    return it->second;
  }
  const int i = result.verts_num();
  result.positions.push_back(x);
  result.positions.push_back(y);
  result.positions.push_back(z);
  weld.emplace(key, i);
  return i;
}

static void append_welded_xyz_ngon(MeshResult &result,
                                   std::map<std::array<long long, 3>, int> &weld,
                                   const std::vector<float> &xyz,
                                   int tag,
                                   double scale)
{
  const int n = int(xyz.size() / 3);
  if (n < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets = {0};
  }
  std::vector<int> ring;
  ring.reserve(size_t(n));
  for (int i = 0; i < n; i++) {
    ring.push_back(weld_xyz_vert(result,
                                 weld,
                                 xyz[size_t(i) * 3 + 0],
                                 xyz[size_t(i) * 3 + 1],
                                 xyz[size_t(i) * 3 + 2],
                                 scale));
  }
  uniquify_ring(ring);
  if (ring.size() < 3) {
    return;
  }
  result.corner_verts.insert(result.corner_verts.end(), ring.begin(), ring.end());
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
}

static double min_dist_to_ring(const Point_2 &q, const std::vector<Point_2> &ring)
{
  if (ring.size() < 2) {
    return 1e300;
  }
  double best = 1e300;
  const size_t n = ring.size();
  for (size_t i = 0; i < n; i++) {
    const Segment_2 s(ring[i], ring[(i + 1) % n]);
    const double d2 = CGAL::to_double(CGAL::squared_distance(q, s));
    if (d2 < best) {
      best = d2;
    }
  }
  return std::sqrt((std::max)(0.0, best));
}

static bool point_on_frame(double x, double y, double fx0, double fy0, double fx1, double fy1, double eps)
{
  const bool on_v = (x <= fx0 + eps) || (x >= fx1 - eps);
  const bool on_h = (y <= fy0 + eps) || (y >= fy1 - eps);
  const bool in_x = (x >= fx0 - eps) && (x <= fx1 + eps);
  const bool in_y = (y >= fy0 - eps) && (y <= fy1 + eps);
  return (on_v && in_y) || (on_h && in_x);
}

static void reverse_weights_like_orientation(std::vector<double> &w)
{
  if (w.size() > 2) {
    std::reverse(w.begin() + 1, w.end());
  }
}

static bool island_to_pwh(const PolyIsland &isl, Polygon_with_holes_2 &pwh, std::vector<std::vector<double>> &weights, const MeshIn &mesh, const float *vert_weights, int weights_n)
{
  if (isl.outer.size() < 3) {
    return false;
  }
  std::vector<Point_2> outer_r = collapse_collinear(clean_loop_2(isl.outer));
  Polygon_2 outer = ring_to_oriented_poly(outer_r, true);
  if (outer.size() < 3) {
    return false;
  }
  std::vector<Point_2> outer_pts = poly_to_ring(outer);
  std::vector<double> ow = weights_for_ring(outer_pts, mesh, vert_weights, weights_n);
  if (ow.size() != size_t(outer.size())) {
    ow.assign(size_t(outer.size()), 1.0);
  }
  std::vector<Polygon_2> holes;
  weights.clear();
  weights.push_back(std::move(ow));
  for (const auto &hr : isl.holes) {
    if (hr.size() < 3) {
      continue;
    }
    std::vector<Point_2> hole_r = collapse_collinear(clean_loop_2(hr));
    Polygon_2 h = ring_to_oriented_poly(hole_r, false); /* PWH holes are CW */
    if (h.size() < 3) {
      continue;
    }
    std::vector<Point_2> hole_pts = poly_to_ring(h);
    if (!loop_inside_loop(hole_pts, outer_pts)) {
      continue;
    }
    std::vector<double> hw = weights_for_ring(hole_pts, mesh, vert_weights, weights_n);
    if (hw.size() != size_t(h.size())) {
      hw.assign(size_t(h.size()), 1.0);
    }
    holes.push_back(std::move(h));
    weights.push_back(std::move(hw));
  }
  pwh = Polygon_with_holes_2(outer, holes.begin(), holes.end());
  return true;
}

static void ensure_face_offsets(MeshResult &result)
{
  if (!result.face_offsets.empty()) {
    return;
  }
  result.face_offsets = {0};
  const int n = result.corners_num();
  for (int i = 3; i <= n; i += 3) {
    result.face_offsets.push_back(i);
  }
}

static void append_pwh_xy(MeshResult &result, const Polygon_with_holes_2 &pwh, float mid_z)
{
  using Vb = CGAL::Triangulation_vertex_base_2<Kernel>;
  using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
  using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
  using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, CGAL::Exact_predicates_tag>;

  const Polygon_2 &outer = pwh.outer_boundary();
  if (outer.size() < 3) {
    return;
  }
  std::vector<std::vector<Point_2>> holes;
  for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
    std::vector<Point_2> hr = clean_loop_2(poly_to_ring(*hit));
    if (hr.size() >= 3) {
      holes.push_back(std::move(hr));
    }
  }
  const bool has_holes = !holes.empty();

  /* Constrain the outer only. Inserting hole edges into the CDT aborts when
   * they nearly touch the outer (typical after offset). Drop triangles whose
   * centroid falls in a hole instead. */
  CDT cdt;
  {
    std::vector<Point_2> ring = clean_loop_2(poly_to_ring(outer));
    if (ring.size() < 3) {
      return;
    }
    std::vector<typename CDT::Vertex_handle> vhs;
    vhs.reserve(ring.size());
    for (const Point_2 &p : ring) {
      try {
        vhs.push_back(cdt.insert(p));
      }
      catch (...) {
        if (!has_holes) {
          append_poly_xy(result, outer, mid_z);
        }
        return;
      }
    }
    if (vhs.size() >= 2 && vhs.front() == vhs.back()) {
      vhs.pop_back();
    }
    const int n = int(vhs.size());
    if (n < 3) {
      if (!has_holes) {
        append_poly_xy(result, outer, mid_z);
      }
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
  }
  if (cdt.dimension() != 2) {
    if (!has_holes) {
      append_poly_xy(result, outer, mid_z);
    }
    return;
  }
  std::map<typename CDT::Face_handle, bool> in_domain_map;
  boost::associative_property_map<std::map<typename CDT::Face_handle, bool>> in_domain(
      in_domain_map);
  CGAL::mark_domain_in_triangulation(cdt, in_domain);
  ensure_face_offsets(result);
  std::map<typename CDT::Vertex_handle, int> v2i;
  auto vid = [&](typename CDT::Vertex_handle vh) {
    auto it = v2i.find(vh);
    if (it != v2i.end()) {
      return it->second;
    }
    const int idx = result.verts_num();
    v2i[vh] = idx;
    float q[3];
    unproject_xy(CGAL::to_double(vh->point().x()), CGAL::to_double(vh->point().y()), mid_z, q);
    result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
    return idx;
  };
  int added = 0;
  for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
    if (!in_domain_map[fit]) {
      continue;
    }
    const Point_2 c0 = fit->vertex(0)->point();
    const Point_2 c1 = fit->vertex(1)->point();
    const Point_2 c2 = fit->vertex(2)->point();
    const Point_2 mid((CGAL::to_double(c0.x()) + CGAL::to_double(c1.x()) + CGAL::to_double(c2.x())) /
                          3.0,
                      (CGAL::to_double(c0.y()) + CGAL::to_double(c1.y()) + CGAL::to_double(c2.y())) /
                          3.0);
    bool in_hole = false;
    for (const auto &hr : holes) {
      if (point_in_poly(mid, hr)) {
        in_hole = true;
        break;
      }
    }
    if (in_hole) {
      continue;
    }
    const int i0 = vid(fit->vertex(0));
    const int i1 = vid(fit->vertex(1));
    const int i2 = vid(fit->vertex(2));
    if (i0 == i1 || i1 == i2 || i2 == i0) {
      continue;
    }
    result.corner_verts.push_back(i0);
    result.corner_verts.push_back(i1);
    result.corner_verts.push_back(i2);
    result.face_offsets.push_back(int(result.corner_verts.size()));
    added++;
  }
  if (added == 0 && !has_holes) {
    append_poly_xy(result, outer, mid_z);
  }
}

static void emit_offset_pwhs(MeshResult &result,
                             const std::vector<std::shared_ptr<Polygon_with_holes_2>> &pwhs,
                             float mid_z)
{
  double max_a = 0.0;
  for (const auto &sp : pwhs) {
    if (!sp || sp->outer_boundary().size() < 3) {
      continue;
    }
    max_a = (std::max)(max_a, poly_area_abs(sp->outer_boundary()));
  }
  for (const auto &sp : pwhs) {
    if (!sp || sp->outer_boundary().size() < 3) {
      continue;
    }
    const double a = poly_area_abs(sp->outer_boundary());
    if (max_a > 0.0 && a < max_a * 1e-4) {
      continue;
    }
    append_pwh_xy(result, *sp, mid_z);
  }
}

static bool assemble_offset_pwh(const std::vector<Point_2> &outer,
                                const std::vector<std::vector<Point_2>> &holes,
                                std::vector<std::shared_ptr<Polygon_with_holes_2>> &pwhs)
{
  std::vector<Point_2> o = clean_loop_2(outer);
  if (o.size() < 3) {
    return false;
  }
  if (signed_area_2(o) < 0.0) {
    std::reverse(o.begin(), o.end());
  }
  std::vector<Polygon_2> hp;
  for (const auto &hr : holes) {
    std::vector<Point_2> h = clean_loop_2(hr);
    if (h.size() < 3) {
      continue;
    }
    if (signed_area_2(h) > 0.0) {
      std::reverse(h.begin(), h.end());
    }
    if (!loop_inside_loop(h, o)) {
      continue;
    }
    hp.emplace_back(h.begin(), h.end());
  }
  Polygon_2 op(o.begin(), o.end());
  if (op.size() < 3) {
    return false;
  }
  pwhs.push_back(std::make_shared<Polygon_with_holes_2>(op, hp.begin(), hp.end()));
  return true;
}

/* Parallel miter offset. signed_r > 0 = left of directed edges. */
static std::vector<Point_2> offset_ring_miter(const std::vector<Point_2> &rin,
                                              const std::vector<double> &wts,
                                              double signed_r)
{
  std::vector<Point_2> ring = clean_loop_2(rin);
  const int n = int(ring.size());
  if (n < 3 || !std::isfinite(signed_r) || !(std::abs(signed_r) > 1e-18)) {
    return ring;
  }
  std::vector<Point_2> out;
  out.reserve(size_t(n));
  for (int i = 0; i < n; i++) {
    const Point_2 &prv = ring[size_t((i + n - 1) % n)];
    const Point_2 &cur = ring[size_t(i)];
    const Point_2 &nxt = ring[size_t((i + 1) % n)];
    const double d0x = CGAL::to_double(cur.x() - prv.x());
    const double d0y = CGAL::to_double(cur.y() - prv.y());
    const double d1x = CGAL::to_double(nxt.x() - cur.x());
    const double d1y = CGAL::to_double(nxt.y() - cur.y());
    const double L0 = std::hypot(d0x, d0y);
    const double L1 = std::hypot(d1x, d1y);
    if (L0 < 1e-18 || L1 < 1e-18) {
      continue;
    }
    const double n0x = -d0y / L0;
    const double n0y = d0x / L0;
    const double n1x = -d1y / L1;
    const double n1y = d1x / L1;
    double bx = n0x + n1x;
    double by = n0y + n1y;
    double bL = std::hypot(bx, by);
    if (bL < 1e-12) {
      bx = n0x;
      by = n0y;
      bL = 1.0;
    }
    const double nx = bx / bL;
    const double ny = by / bL;
    double c = n0x * nx + n0y * ny;
    if (std::abs(c) < 1e-6) {
      c = (c >= 0.0 ? 1.0 : -1.0) * 1e-6;
    }
    double w = (size_t(i) < wts.size()) ? wts[size_t(i)] : 1.0;
    if (!(w > 1e-8) || !std::isfinite(w)) {
      w = 1.0;
    }
    double dist = signed_r * w / c;
    const double cap = std::abs(signed_r) * 20.0;
    if (dist > cap) {
      dist = cap;
    }
    if (dist < -cap) {
      dist = -cap;
    }
    out.emplace_back(CGAL::to_double(cur.x()) + dist * nx,
                     CGAL::to_double(cur.y()) + dist * ny);
  }
  return clean_loop_2(out);
}

static std::vector<Point_2> try_ss_offset_simple(const std::vector<Point_2> &ring,
                                                 const std::vector<double> &wts,
                                                 double r,
                                                 bool exterior,
                                                 const Kernel &k)
{
  /* Left-normal of a CCW ring points inward. Expand = negative left offset. */
  std::vector<Point_2> fallback = offset_ring_miter(ring, wts, exterior ? -r : r);
  Polygon_2 poly = ring_to_oriented_poly(ring, true);
  if (poly.size() < 3) {
    return fallback;
  }
  std::vector<double> hw = wts;
  if (hw.size() != size_t(poly.size())) {
    hw.assign(size_t(poly.size()), 1.0);
  }
  try {
    if (exterior) {
      std::vector<std::vector<double>> pack = {hw};
      auto raw = CGAL::create_exterior_weighted_skeleton_and_offset_polygons_2(
          r, poly, pack, k, k);
      if (raw.size() >= 1) {
        std::swap(raw.front(), raw.back());
        raw.pop_back();
      }
      if (!raw.empty() && raw.front() && raw.front()->size() >= 3) {
        raw.front()->reverse_orientation();
        return poly_to_ring(*raw.front());
      }
    }
    else {
      std::vector<Polygon_2> no_holes;
      std::vector<std::vector<double>> no_hw;
      auto raw = CGAL::create_interior_weighted_skeleton_and_offset_polygons_2(
          r, poly, no_holes.begin(), no_holes.end(), hw, no_hw.begin(), no_hw.end(), k, k);
      if (!raw.empty() && raw.front() && raw.front()->size() >= 3) {
        return poly_to_ring(*raw.front());
      }
    }
  }
  catch (...) {
  }
  return fallback;
}

static bool segs_proper_cross(const Point_2 &a, const Point_2 &b, const Point_2 &c, const Point_2 &d)
{
  const auto o1 = CGAL::orientation(a, b, c);
  const auto o2 = CGAL::orientation(a, b, d);
  const auto o3 = CGAL::orientation(c, d, a);
  const auto o4 = CGAL::orientation(c, d, b);
  if (o1 == CGAL::COLLINEAR || o2 == CGAL::COLLINEAR || o3 == CGAL::COLLINEAR ||
      o4 == CGAL::COLLINEAR)
  {
    return false;
  }
  return (o1 != o2) && (o3 != o4);
}

static bool ring_is_simple(const std::vector<Point_2> &r)
{
  const int n = int(r.size());
  if (n < 3) {
    return false;
  }
  for (int i = 0; i < n; i++) {
    const int i1 = (i + 1) % n;
    for (int j = i + 1; j < n; j++) {
      const int j1 = (j + 1) % n;
      if (i == j || i1 == j || i == j1 || i1 == j1) {
        continue;
      }
      if (segs_proper_cross(r[size_t(i)], r[size_t(i1)], r[size_t(j)], r[size_t(j1)])) {
        return false;
      }
    }
  }
  return true;
}

static bool pwh_rings_simple(const Polygon_with_holes_2 &pwh)
{
  if (!ring_is_simple(poly_to_ring(pwh.outer_boundary()))) {
    return false;
  }
  for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
    if (!ring_is_simple(poly_to_ring(*hit))) {
      return false;
    }
  }
  return true;
}

static bool weights_are_uniform(const std::vector<std::vector<double>> &wpack)
{
  for (const auto &row : wpack) {
    for (double w : row) {
      if (!std::isfinite(w) || std::abs(w - 1.0) > 1e-6) {
        return false;
      }
    }
  }
  return true;
}

static bool weighted_offset_pwh(const Polygon_with_holes_2 &pwh,
                                const std::vector<std::vector<double>> &wpack,
                                double r,
                                bool outward,
                                const Kernel &k,
                                std::vector<std::shared_ptr<Polygon_with_holes_2>> &pwhs)
{
  /* Same PWH skeleton as Polygon Offset 2D — one wavefront for outer + holes.
   * Independent ring offsets flip holes and ignore thin walls. */
  const bool simple = pwh_rings_simple(pwh);
  if (simple && (weights_are_uniform(wpack) || wpack.empty())) {
    try {
      if (outward) {
        pwhs = CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(r, pwh, k, k);
      }
      else {
        pwhs = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(r, pwh, k, k);
      }
      if (!pwhs.empty()) {
        return true;
      }
    }
    catch (...) {
    }
  }
  else if (simple) {
    try {
      if (!outward) {
        pwhs = CGAL::create_interior_weighted_skeleton_and_offset_polygons_with_holes_2(
            r, pwh, wpack, k, k);
        if (!pwhs.empty()) {
          return true;
        }
      }
    }
    catch (...) {
    }
    try {
      if (outward) {
        pwhs = CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(r, pwh, k, k);
      }
      else {
        pwhs = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(r, pwh, k, k);
      }
      if (!pwhs.empty()) {
        return true;
      }
    }
    catch (...) {
    }
  }

  /* Left-normal of CCW outer / CW hole: expand = negative (outward / into hole). */
  const double signed_r = outward ? -r : r;
  std::vector<Point_2> outer = poly_to_ring(pwh.outer_boundary());
  std::vector<double> ow = wpack.empty() ? std::vector<double>(outer.size(), 1.0) : wpack[0];
  std::vector<Point_2> o2;
  if (ring_is_simple(outer)) {
    o2 = try_ss_offset_simple(outer, ow, r, outward, k);
  }
  if (o2.size() < 3) {
    o2 = offset_ring_miter(outer, ow, signed_r);
  }

  std::vector<std::vector<Point_2>> holes;
  size_t hi = 0;
  for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit, ++hi) {
    std::vector<Point_2> hole = poly_to_ring(*hit);
    std::vector<double> hw = (hi + 1 < wpack.size()) ?
                                 wpack[hi + 1] :
                                 std::vector<double>(hole.size(), 1.0);
    std::vector<Point_2> h2 = offset_ring_miter(hole, hw, signed_r);
    if (h2.size() >= 3) {
      holes.push_back(std::move(h2));
    }
  }
  return assemble_offset_pwh(o2, holes, pwhs);
}

static bool weighted_offset_outward(const Polygon_with_holes_2 &pwh,
                                    const std::vector<std::vector<double>> &wpack,
                                    double r,
                                    const Kernel &k,
                                    std::vector<std::shared_ptr<Polygon_with_holes_2>> &pwhs)
{
  return weighted_offset_pwh(pwh, wpack, r, true, k, pwhs);
}

static bool weighted_offset_inward(const Polygon_with_holes_2 &pwh,
                                   const std::vector<std::vector<double>> &wpack,
                                   double r,
                                   const Kernel &k,
                                   std::vector<std::shared_ptr<Polygon_with_holes_2>> &pwhs)
{
  return weighted_offset_pwh(pwh, wpack, r, false, k, pwhs);
}

static Polygon_2 make_disk_poly(double cx, double cy, double r, int segs)
{
  Polygon_2 disk;
  segs = std::clamp(segs, 8, 128);
  for (int i = 0; i < segs; i++) {
    const double a = (2.0 * M_PI * double(i)) / double(segs);
    disk.push_back(Point_2(cx + r * std::cos(a), cy + r * std::sin(a)));
  }
  if (disk.size() >= 3 && disk.is_clockwise_oriented()) {
    disk.reverse_orientation();
  }
  return disk;
}

/* Capsule of radius r around segment ab (convex, CCW). */
static Polygon_2 make_stadium(const Point_2 &a, const Point_2 &b, double r, int segs)
{
  const double ax = CGAL::to_double(a.x());
  const double ay = CGAL::to_double(a.y());
  const double bx = CGAL::to_double(b.x());
  const double by = CGAL::to_double(b.y());
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len = std::hypot(dx, dy);
  if (!(r > 1e-18)) {
    return Polygon_2();
  }
  if (len < 1e-18) {
    return make_disk_poly(ax, ay, r, segs);
  }
  const double ang_n = std::atan2(-dx / len, dy / len); /* left normal of ab */
  const int half = (std::max)(4, segs / 2);
  Polygon_2 poly;
  for (int i = 0; i <= half; i++) {
    const double t = ang_n + M_PI * double(i) / double(half);
    poly.push_back(Point_2(ax + r * std::cos(t), ay + r * std::sin(t)));
  }
  for (int i = 0; i <= half; i++) {
    const double t = ang_n + M_PI + M_PI * double(i) / double(half);
    poly.push_back(Point_2(bx + r * std::cos(t), by + r * std::sin(t)));
  }
  std::vector<Point_2> ring(poly.vertices_begin(), poly.vertices_end());
  ring = clean_loop_2(ring);
  if (ring.size() < 3) {
    return Polygon_2();
  }
  Polygon_2 out(ring.begin(), ring.end());
  if (out.size() >= 3 && out.is_clockwise_oriented()) {
    out.reverse_orientation();
  }
  return out;
}

static void collect_pwh_edges(const Polygon_with_holes_2 &pwh, std::vector<Segment_2> &edges)
{
  auto pull = [&](const Polygon_2 &poly) {
    const int n = int(poly.size());
    if (n < 2) {
      return;
    }
    for (int i = 0; i < n; i++) {
      const Point_2 &s = poly.vertex(i);
      const Point_2 &t = poly.vertex((i + 1) % n);
      if (s != t) {
        edges.emplace_back(s, t);
      }
    }
  };
  pull(pwh.outer_boundary());
  for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
    pull(*hit);
  }
}

static bool pwhs_from_bool_list(const std::list<Polygon_with_holes_2> &in,
                                std::vector<std::shared_ptr<Polygon_with_holes_2>> &out)
{
  out.clear();
  for (const auto &p : in) {
    if (p.outer_boundary().size() >= 3) {
      out.push_back(std::make_shared<Polygon_with_holes_2>(p));
    }
  }
  return !out.empty();
}

/* Offset a closed ring by signed distance r (positive = left of each edge).
 * Convex corners get a circular arc; reflex corners become a miter. */
static std::vector<Point_2> offset_ring_round(const std::vector<Point_2> &rin, double r, int segs)
{
  std::vector<Point_2> ring = clean_loop_2(rin);
  const int n = int(ring.size());
  if (n < 3 || !std::isfinite(r) || !(std::abs(r) > 1e-18)) {
    return ring;
  }
  auto left_n = [](const Point_2 &a, const Point_2 &b, double &nx, double &ny) -> bool {
    const double dx = CGAL::to_double(b.x() - a.x());
    const double dy = CGAL::to_double(b.y() - a.y());
    const double L = std::hypot(dx, dy);
    if (L < 1e-18) {
      return false;
    }
    nx = -dy / L;
    ny = dx / L;
    return true;
  };
  std::vector<Point_2> out;
  out.reserve(size_t(n) * 4);
  segs = std::clamp(segs, 8, 128);
  for (int i = 0; i < n; i++) {
    const Point_2 &prv = ring[size_t((i + n - 1) % n)];
    const Point_2 &cur = ring[size_t(i)];
    const Point_2 &nxt = ring[size_t((i + 1) % n)];
    double n0x = 0.0, n0y = 0.0, n1x = 0.0, n1y = 0.0;
    if (!left_n(prv, cur, n0x, n0y) || !left_n(cur, nxt, n1x, n1y)) {
      continue;
    }
    const double cx = CGAL::to_double(cur.x());
    const double cy = CGAL::to_double(cur.y());
    const double d0x = CGAL::to_double(cur.x() - prv.x());
    const double d0y = CGAL::to_double(cur.y() - prv.y());
    const double d1x = CGAL::to_double(nxt.x() - cur.x());
    const double d1y = CGAL::to_double(nxt.y() - cur.y());
    const double cross = d0x * d1y - d0y * d1x;
    /* r is the left-normal offset. Morphological disk:
     *   arc on the reflex side of the vertex (angle > 180 on the offset side).
     * Left side is reflex iff right turn (cross < 0). */
    const bool need_arc = (r > 0.0) ? (cross < -1e-14) : (cross > 1e-14);
    if (need_arc) {
      double a0 = std::atan2(n0y, n0x);
      double a1 = std::atan2(n1y, n1x);
      double da = a1 - a0;
      if (r > 0.0) {
        while (da < 0.0) {
          da += 2.0 * M_PI;
        }
        while (da >= 2.0 * M_PI) {
          da -= 2.0 * M_PI;
        }
      }
      else {
        while (da > 0.0) {
          da -= 2.0 * M_PI;
        }
        while (da <= -2.0 * M_PI) {
          da += 2.0 * M_PI;
        }
      }
      const int steps = (std::max)(1, int(std::abs(da) / (2.0 * M_PI) * double(segs) + 0.5));
      for (int s = 0; s <= steps; s++) {
        const double t = a0 + da * double(s) / double(steps);
        out.emplace_back(cx + r * std::cos(t), cy + r * std::sin(t));
      }
    }
    else {
      const double p0x = cx + r * n0x;
      const double p0y = cy + r * n0y;
      const double p1x = cx + r * n1x;
      const double p1y = cy + r * n1y;
      const double den = d0x * d1y - d0y * d1x;
      if (std::abs(den) > 1e-18) {
        const double t = ((p1x - p0x) * d1y - (p1y - p0y) * d1x) / den;
        out.emplace_back(p0x + t * d0x, p0y + t * d0y);
      }
      else {
        out.emplace_back(p0x, p0y);
        out.emplace_back(p1x, p1y);
      }
    }
  }
  return clean_loop_2(out);
}

static bool round_offset_rings(const Polygon_with_holes_2 &pwh,
                               double signed_r,
                               int segs,
                               std::vector<std::shared_ptr<Polygon_with_holes_2>> &pwhs)
{
  std::vector<Point_2> outer(pwh.outer_boundary().vertices_begin(),
                             pwh.outer_boundary().vertices_end());
  std::vector<Point_2> o2 = offset_ring_round(outer, signed_r, segs);
  o2 = collapse_collinear(o2);
  if (o2.size() < 3) {
    return false;
  }
  if (signed_area_2(o2) < 0.0) {
    std::reverse(o2.begin(), o2.end());
  }
  std::vector<Polygon_2> holes;
  for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
    std::vector<Point_2> h(hit->vertices_begin(), hit->vertices_end());
    std::vector<Point_2> h2 = offset_ring_round(h, signed_r, segs);
    h2 = collapse_collinear(h2);
    if (h2.size() < 3) {
      continue;
    }
    if (signed_area_2(h2) > 0.0) {
      std::reverse(h2.begin(), h2.end());
    }
    if (!loop_inside_loop(h2, o2)) {
      continue;
    }
    holes.emplace_back(h2.begin(), h2.end());
  }
  Polygon_2 op(o2.begin(), o2.end());
  if (op.size() < 3) {
    return false;
  }
  pwhs.push_back(std::make_shared<Polygon_with_holes_2>(op, holes.begin(), holes.end()));
  return true;
}

/* Morphological dilation / erosion of a PWH by a disk via boundary capsules.
 * Dilation = P ∪ (∂P ⊕ disk), erosion = P \ (∂P ⊕ disk). Holes shrink / grow. */
static bool round_offset_by_rim(const Polygon_with_holes_2 &pwh,
                                double r,
                                int segs,
                                bool outward,
                                std::vector<std::shared_ptr<Polygon_with_holes_2>> &pwhs)
{
  std::vector<Segment_2> edges;
  collect_pwh_edges(pwh, edges);
  if (edges.empty()) {
    return false;
  }
  std::vector<Polygon_2> stadiums;
  stadiums.reserve(edges.size());
  for (const Segment_2 &e : edges) {
    Polygon_2 st = make_stadium(e.source(), e.target(), r, segs);
    if (st.size() < 3) {
      continue;
    }
    try {
      if (!st.is_simple()) {
        continue;
      }
    }
    catch (...) {
      continue;
    }
    stadiums.push_back(std::move(st));
  }
  if (stadiums.empty()) {
    return false;
  }
  std::list<Polygon_with_holes_2> cur;
  cur.push_back(pwh);
  for (const Polygon_2 &st : stadiums) {
    std::list<Polygon_with_holes_2> nxt;
    for (const Polygon_with_holes_2 &c : cur) {
      try {
        if (outward) {
          Polygon_with_holes_2 uni;
          if (CGAL::join(c, st, uni)) {
            nxt.push_back(std::move(uni));
          }
          else {
            nxt.push_back(c);
            nxt.emplace_back(st);
          }
        }
        else {
          CGAL::difference(c, st, std::back_inserter(nxt));
        }
      }
      catch (...) {
        nxt.push_back(c);
      }
    }
    if (nxt.empty()) {
      if (!outward) {
        pwhs.clear();
        return true; /* fully eroded */
      }
      return false;
    }
    cur.swap(nxt);
  }
  return pwhs_from_bool_list(cur, pwhs);
}

}  // namespace

WireResult mesh_exterior_skeleton_2(const MeshIn &mesh, double max_offset)
{
  WireResult result;
  CgalThrowGuard guard;
  try {
    if (!(max_offset > 1e-12) || !std::isfinite(max_offset)) {
      result.error = "Max Offset must be > 0";
      return result;
    }
    std::vector<PolyIsland> islands;
    float mid_z = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, islands, mid_z, error) || islands.empty()) {
      result.error = error.empty() ? "Exterior Skeleton 2D needs a polygon" : error;
      return result;
    }

    int vert_base = 0;
    for (const PolyIsland &isl : islands) {
      if (isl.outer.size() < 3) {
        continue;
      }
      Polygon_2 outer = ring_to_ccw_poly(isl.outer);
      if (outer.size() < 3) {
        continue;
      }
      /* Exterior skeleton of the outer contour. Holes belong to the interior
       * skeleton (Straight Skeleton 2D), not the exterior. */
      auto ss = CGAL::create_exterior_straight_skeleton_2(max_offset, outer);
      if (!ss) {
        continue;
      }
      using Ss = std::remove_reference_t<decltype(*ss)>;

      double fx0 = 0.0, fy0 = 0.0, fx1 = 0.0, fy1 = 0.0;
      auto margin = CGAL::compute_outer_frame_margin(
          outer.vertices_begin(), outer.vertices_end(), max_offset);
      const CGAL::Bbox_2 bb = CGAL::bbox_2(outer.vertices_begin(), outer.vertices_end());
      const double lm = margin ? CGAL::to_double(*margin) : (max_offset * 2.0);
      fx0 = bb.xmin() - lm;
      fy0 = bb.ymin() - lm;
      fx1 = bb.xmax() + lm;
      fy1 = bb.ymax() + lm;
      const double span = (std::max)(fx1 - fx0, fy1 - fy0);
      const double eps = (std::max)(1e-8, span * 1e-7);
      const double max_d = max_offset * 1.001;

      struct KeepPt {
        double x, y;
        double dist;
        bool keep;
      };
      std::map<typename Ss::Vertex_const_handle, KeepPt> info;
      for (auto vit = ss->vertices_begin(); vit != ss->vertices_end(); ++vit) {
        const double x = CGAL::to_double(vit->point().x());
        const double y = CGAL::to_double(vit->point().y());
        const double dist = min_dist_to_ring(vit->point(), isl.outer);
        const bool on_fr = point_on_frame(x, y, fx0, fy0, fx1, fy1, eps);
        KeepPt kp{x, y, dist, !on_fr && dist <= max_d};
        info[vit] = kp;
      }

      auto emit_pt = [&](double x, double y) -> int {
        float q[3];
        unproject_xy(x, y, mid_z, q);
        const int id = vert_base++;
        result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
        return id;
      };
      std::map<typename Ss::Vertex_const_handle, int> vmap;
      for (const auto &kv : info) {
        if (kv.second.keep) {
          vmap[kv.first] = emit_pt(kv.second.x, kv.second.y);
        }
      }

      for (auto hit = ss->halfedges_begin(); hit != ss->halfedges_end(); ++hit) {
        if (!hit->is_bisector()) {
          continue;
        }
        if (&*hit < &*(hit->opposite())) {
          continue;
        }
        auto va = hit->opposite()->vertex();
        auto vb = hit->vertex();
        auto ia = info.find(va);
        auto ib = info.find(vb);
        if (ia == info.end() || ib == info.end()) {
          continue;
        }
        const KeepPt &a = ia->second;
        const KeepPt &b = ib->second;
        if (a.keep && b.keep) {
          result.edge_v0.push_back(vmap[va]);
          result.edge_v1.push_back(vmap[vb]);
          continue;
        }
        if (a.keep == b.keep) {
          continue;
        }
        const KeepPt &in = a.keep ? a : b;
        const KeepPt &out = a.keep ? b : a;
        const double den = out.dist - in.dist;
        if (!(std::abs(den) > 1e-14)) {
          continue;
        }
        const double t = std::clamp((max_offset - in.dist) / den, 0.0, 1.0);
        const double x = in.x + t * (out.x - in.x);
        const double y = in.y + t * (out.y - in.y);
        const int id_keep = vmap[a.keep ? va : vb];
        const int id_clip = emit_pt(x, y);
        result.edge_v0.push_back(id_keep);
        result.edge_v1.push_back(id_clip);
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Exterior Skeleton 2D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Exterior Skeleton 2D failed";
  }
  return result;
}

MeshResult mesh_weighted_offset_2(const MeshIn &mesh,
                                  const float *vert_weights,
                                  int weights_n,
                                  double offset)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    if (!std::isfinite(offset) || offset == 0.0) {
      result.error = "Offset must be non-zero";
      return result;
    }
    std::vector<PolyIsland> islands;
    float mid_z = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, islands, mid_z, error) || islands.empty()) {
      result.error = error.empty() ? "Weighted Offset 2D needs a polygon" : error;
      return result;
    }
    const bool outward = offset > 0.0;
    const double r = std::abs(offset);
    Kernel k;
    for (const PolyIsland &isl : islands) {
      Polygon_with_holes_2 pwh;
      std::vector<std::vector<double>> wpack;
      if (!island_to_pwh(isl, pwh, wpack, mesh, vert_weights, weights_n)) {
        continue;
      }
      try {
        std::vector<std::shared_ptr<Polygon_with_holes_2>> pwhs;
        const bool ok = outward ? weighted_offset_outward(pwh, wpack, r, k, pwhs) :
                                  weighted_offset_inward(pwh, wpack, r, k, pwhs);
        if (!ok) {
          continue;
        }
        emit_offset_pwhs(result, pwhs, mid_z);
      }
      catch (...) {
      }
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Weighted Offset 2D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Weighted Offset 2D failed";
  }
  return result;
}

MeshResult points_voronoi_on_sphere(const float *positions, int n)
{
  MeshResult result;
  if (!positions || n < 4) {
    result.error = "Voronoi on Sphere needs at least 4 points";
    return result;
  }
  CgalThrowGuard guard;
  try {
    double cx = 0.0, cy = 0.0, cz = 0.0;
    for (int i = 0; i < n; i++) {
      cx += double(positions[i * 3 + 0]);
      cy += double(positions[i * 3 + 1]);
      cz += double(positions[i * 3 + 2]);
    }
    cx /= double(n);
    cy /= double(n);
    cz /= double(n);
    double r2max = 0.0;
    for (int i = 0; i < n; i++) {
      const double dx = double(positions[i * 3 + 0]) - cx;
      const double dy = double(positions[i * 3 + 1]) - cy;
      const double dz = double(positions[i * 3 + 2]) - cz;
      r2max = (std::max)(r2max, dx * dx + dy * dy + dz * dz);
    }
    const double radius = std::sqrt((std::max)(r2max, 1e-18));
    const Point_3 center(cx, cy, cz);

    std::vector<Point_3> on_sphere;
    on_sphere.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      const double dx = double(positions[i * 3 + 0]) - cx;
      const double dy = double(positions[i * 3 + 1]) - cy;
      const double dz = double(positions[i * 3 + 2]) - cz;
      const double L = std::sqrt((std::max)(1e-30, dx * dx + dy * dy + dz * dz));
      on_sphere.emplace_back(cx + dx / L * radius, cy + dy / L * radius, cz + dz / L * radius);
    }

    using Traits = CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel>;
    using Dt = CGAL::Delaunay_triangulation_on_sphere_2<Traits>;
    Dt dt(center, radius);
    dt.insert(on_sphere.begin(), on_sphere.end());
    if (dt.dimension() != 2) {
      result.error = "Voronoi on Sphere degenerate";
      return result;
    }

    auto sph_cc = [&](typename Dt::Face_handle f) -> Point_3 {
      const Point_3 a = f->vertex(0)->point();
      const Point_3 b = f->vertex(1)->point();
      const Point_3 c = f->vertex(2)->point();
      const Vector_3 ab = b - a;
      const Vector_3 ac = c - a;
      Vector_3 n = CGAL::cross_product(ab, ac);
      const double nx = CGAL::to_double(n.x());
      const double ny = CGAL::to_double(n.y());
      const double nz = CGAL::to_double(n.z());
      const double Ln = std::sqrt((std::max)(1e-30, nx * nx + ny * ny + nz * nz));
      double ux = nx / Ln, uy = ny / Ln, uz = nz / Ln;
      const double ax = CGAL::to_double(a.x()) - cx;
      const double ay = CGAL::to_double(a.y()) - cy;
      const double az = CGAL::to_double(a.z()) - cz;
      if (ux * ax + uy * ay + uz * az < 0.0) {
        ux = -ux;
        uy = -uy;
        uz = -uz;
      }
      return Point_3(cx + ux * radius, cy + uy * radius, cz + uz * radius);
    };

    std::map<typename Dt::Face_handle, Point_3> face_cc;
    std::map<typename Dt::Vertex_handle, std::vector<typename Dt::Face_handle>> incident;
    for (auto fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
      if (dt.is_ghost(fit)) {
        continue;
      }
      face_cc[fit] = sph_cc(fit);
      for (int i = 0; i < 3; i++) {
        incident[fit->vertex(i)].push_back(fit);
      }
    }

    int cell = 0;
    for (auto vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); ++vit) {
      auto it = incident.find(vit);
      if (it == incident.end() || it->second.size() < 3) {
        continue;
      }
      const Point_3 site = vit->point();
      const double sx = CGAL::to_double(site.x()) - cx;
      const double sy = CGAL::to_double(site.y()) - cy;
      const double sz = CGAL::to_double(site.z()) - cz;
      const double sL = std::sqrt((std::max)(1e-30, sx * sx + sy * sy + sz * sz));
      const double rx = sx / sL, ry = sy / sL, rz = sz / sL;
      double ux = 0.0, uy = 1.0, uz = 0.0;
      if (std::abs(ry) > 0.9) {
        ux = 1.0;
        uy = 0.0;
      }
      double tx = uy * rz - uz * ry;
      double ty = uz * rx - ux * rz;
      double tz = ux * ry - uy * rx;
      const double tL = std::sqrt((std::max)(1e-30, tx * tx + ty * ty + tz * tz));
      tx /= tL;
      ty /= tL;
      tz /= tL;
      const double bx = ry * tz - rz * ty;
      const double by = rz * tx - rx * tz;
      const double bz = rx * ty - ry * tx;

      struct AngFace {
        double ang;
        typename Dt::Face_handle f;
      };
      std::vector<AngFace> ordered;
      ordered.reserve(it->second.size());
      for (auto f : it->second) {
        const Point_3 &cc = face_cc[f];
        const double dx = CGAL::to_double(cc.x()) - cx;
        const double dy = CGAL::to_double(cc.y()) - cy;
        const double dz = CGAL::to_double(cc.z()) - cz;
        const double px = dx - rx * (dx * rx + dy * ry + dz * rz);
        const double py = dy - ry * (dx * rx + dy * ry + dz * rz);
        const double pz = dz - rz * (dx * rx + dy * ry + dz * rz);
        ordered.push_back({std::atan2(px * bx + py * by + pz * bz, px * tx + py * ty + pz * tz), f});
      }
      std::sort(ordered.begin(), ordered.end(), [](const AngFace &a, const AngFace &b) {
        return a.ang < b.ang;
      });

      std::vector<float> xyz;
      xyz.reserve(ordered.size() * 3);
      for (const AngFace &af : ordered) {
        const Point_3 &p = face_cc[af.f];
        xyz.push_back(float(CGAL::to_double(p.x())));
        xyz.push_back(float(CGAL::to_double(p.y())));
        xyz.push_back(float(CGAL::to_double(p.z())));
      }
      append_isolated_xyz_ngon(result, xyz, cell++);
    }

    result.center[0] = float(cx);
    result.center[1] = float(cy);
    result.center[2] = float(cz);
    result.radius = float(radius);
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Voronoi on Sphere produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Voronoi on Sphere failed";
  }
  return result;
}

#if 0
WireResult points_stream_lines_2_deleted(const float *positions,
                                 const float *vectors,
                                 int n,
                                 int resolution,
                                 double separation,
                                 double saturation)
{
  WireResult result;
  if (!positions || !vectors || n < 2) {
    result.error = "Stream Lines 2D needs at least 2 points with vectors";
    return result;
  }
  CgalThrowGuard guard;
  try {
    double xmin = 1e300, ymin = 1e300, xmax = -1e300, ymax = -1e300;
    double mid_z = 0.0;
    int nonzero = 0;
    for (int i = 0; i < n; i++) {
      xmin = (std::min)(xmin, double(positions[i * 3 + 0]));
      ymin = (std::min)(ymin, double(positions[i * 3 + 1]));
      xmax = (std::max)(xmax, double(positions[i * 3 + 0]));
      ymax = (std::max)(ymax, double(positions[i * 3 + 1]));
      mid_z += double(positions[i * 3 + 2]);
      const double vx = double(vectors[i * 3 + 0]);
      const double vy = double(vectors[i * 3 + 1]);
      if (vx * vx + vy * vy > 1e-16) {
        nonzero++;
      }
    }
    mid_z /= double(n);
    if (nonzero < 1) {
      result.error = "Stream Lines 2D needs non-zero XY vectors";
      return result;
    }
    double width = (std::max)(xmax - xmin, 1e-6);
    double height = (std::max)(ymax - ymin, 1e-6);
    const double pad = 0.02 * (std::max)(width, height);
    xmin -= pad;
    ymin -= pad;
    width += 2.0 * pad;
    height += 2.0 * pad;

    resolution = std::clamp(resolution, 8, 256);
    if (!(separation > 0.0) || !std::isfinite(separation)) {
      separation = 0.05 * (std::max)(width, height);
    }
    if (!(saturation > 0.0) || !std::isfinite(saturation)) {
      saturation = 1.6;
    }
    saturation = std::clamp(saturation, 0.2, 8.0);

    /* Stream_lines_2 casts separating_distance to int in a seed loop.
     * Keep the scaled separation >= 2 so that increment cannot be 0. */
    const double min_sep = 2.0;
    const double scale = min_sep / (std::max)(separation, 1e-9);
    const double dom_x = width * scale;
    const double dom_y = height * scale;
    const double sep_s = separation * scale;

    using Field = CGAL::Regular_grid_2<Kernel>;
    using Integrator = CGAL::Runge_kutta_integrator_2<Field>;
    using StreamLines = CGAL::Stream_lines_2<Field, Integrator>;

    Field field(resolution, resolution, Kernel::FT(dom_x), Kernel::FT(dom_y));
    for (int j = 0; j < resolution; j++) {
      for (int i = 0; i < resolution; i++) {
        const double wx = xmin + (double(i) + 0.5) / double(resolution) * width;
        const double wy = ymin + (double(j) + 0.5) / double(resolution) * height;
        int best = 0;
        double best_d = 1e300;
        for (int p = 0; p < n; p++) {
          const double dx = double(positions[p * 3 + 0]) - wx;
          const double dy = double(positions[p * 3 + 1]) - wy;
          const double d2 = dx * dx + dy * dy;
          if (d2 < best_d) {
            best_d = d2;
            best = p;
          }
        }
        const double vx = double(vectors[best * 3 + 0]) * scale;
        const double vy = double(vectors[best * 3 + 1]) * scale;
        field.set_field(i, j, Vector_2(vx, vy));
      }
    }

    Integrator rk;
    StreamLines sl(field, rk, Kernel::FT(sep_s), Kernel::FT(saturation));
    int vert_base = 0;
    for (auto it = sl.begin(); it != sl.end(); ++it) {
      int prev = -1;
      for (auto pit = it->first; pit != it->second; ++pit) {
        const double sx = CGAL::to_double(pit->x());
        const double sy = CGAL::to_double(pit->y());
        const double wx = xmin + sx / scale;
        const double wy = ymin + sy / scale;
        result.positions.push_back(float(wx));
        result.positions.push_back(float(wy));
        result.positions.push_back(float(mid_z));
        const int cur = vert_base++;
        if (prev >= 0) {
          result.edge_v0.push_back(prev);
          result.edge_v1.push_back(cur);
        }
        prev = cur;
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Stream Lines 2D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Stream Lines 2D failed";
  }
  return result;
}
#endif

MeshResult mesh_round_offset_2(const MeshIn &mesh, double radius, int segments)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    if (!(std::abs(radius) > 1e-12) || !std::isfinite(radius)) {
      result.error = "Radius must be non-zero";
      return result;
    }
    segments = std::clamp(segments, 8, 128);
    std::vector<PolyIsland> islands;
    float mid_z = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, islands, mid_z, error) || islands.empty()) {
      result.error = error.empty() ? "Round Offset 2D needs a polygon" : error;
      return result;
    }

    const bool outward = radius > 0.0;
    const double r = std::abs(radius);

    for (const PolyIsland &isl : islands) {
      Polygon_with_holes_2 pwh;
      std::vector<std::vector<double>> unused_w;
      if (!island_to_pwh(isl, pwh, unused_w, mesh, nullptr, 0)) {
        continue;
      }
      /* Round-join per contour only. Minkowski / Boolean_set abort() on the
       * non-strictly-simple outlines Blender actually produces. */
      std::vector<std::shared_ptr<Polygon_with_holes_2>> pwhs;
      bool got = false;
      try {
        /* Left-normal offset: CCW outer / CW hole expand = negative. */
        got = round_offset_rings(pwh, outward ? -r : r, segments, pwhs);
      }
      catch (...) {
        got = false;
      }
      if (!got || pwhs.empty()) {
        continue;
      }
      emit_offset_pwhs(result, pwhs, mid_z);
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Round Offset 2D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Round Offset 2D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
