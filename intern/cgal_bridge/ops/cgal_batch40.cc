/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 40 — 8 new nodes (not previously deleted):
 *  - QEM Simplify (Garland–Heckbert plane quadrics)
 *  - Line Arrangement 2D
 *  - Vertical Decomposition 2D
 *  - Circle Arrangement 2D
 *  - Crust 3D
 *  - Clipped Voronoi 3D
 *  - Complement 2D
 *  - Simple Polygon 2D
 *
 * Skipped: OpenGR, Kinetic EPECK, Mesh_3 volume, PolyFit SCIP, Ridges_3,
 * Classification, OSQP, Hyperbolic Delaunay, weighted Delaunay / Lloyd,
 * banned measure/query/remesh, previously deleted nodes.
 */

#define CGAL_CH3_DUAL_WITHOUT_QP_SOLVER 1

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Convex_hull_3/dual/halfspace_intersection_with_constructions_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Random_polygon_2_sweep.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_plane_policies.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/intersections.h>
#include <CGAL/number_utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;
namespace SMS = CGAL::Surface_mesh_simplification;

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Segment_2 = Kernel::Segment_2;
using Plane_3 = Kernel::Plane_3;
using Vector_3 = Kernel::Vector_3;
using FT = Kernel::FT;
using Polyhedron = CGAL::Polyhedron_3<Kernel>;

static Point_2 xy(double x, double y)
{
  return Point_2(FT(x), FT(y));
}

static Point_3 xyz(double x, double y, double z)
{
  return Point_3(FT(x), FT(y), FT(z));
}

static void push_xyz(std::vector<float> &pos, const Point_3 &p)
{
  pos.push_back(float(CGAL::to_double(p.x())));
  pos.push_back(float(CGAL::to_double(p.y())));
  pos.push_back(float(CGAL::to_double(p.z())));
}

static void push_xy0(std::vector<float> &pos, const Point_2 &p, float z)
{
  pos.push_back(float(CGAL::to_double(p.x())));
  pos.push_back(float(CGAL::to_double(p.y())));
  pos.push_back(z);
}

static std::pair<long long, long long> key2(const Point_2 &p)
{
  return {llround(CGAL::to_double(p.x()) * 1e7), llround(CGAL::to_double(p.y()) * 1e7)};
}

static std::array<long long, 3> key3(const Point_3 &p)
{
  return {llround(CGAL::to_double(p.x()) * 1e7),
          llround(CGAL::to_double(p.y()) * 1e7),
          llround(CGAL::to_double(p.z()) * 1e7)};
}

static double signed_area_2(const std::vector<Point_2> &loop)
{
  double a = 0.0;
  const size_t n = loop.size();
  for (size_t i = 0; i < n; i++) {
    const Point_2 &p = loop[i];
    const Point_2 &q = loop[(i + 1) % n];
    a += CGAL::to_double(p.x()) * CGAL::to_double(q.y()) -
         CGAL::to_double(q.x()) * CGAL::to_double(p.y());
  }
  return 0.5 * a;
}

static float mid_z(const float *positions, int n)
{
  if (!positions || n <= 0) {
    return 0.0f;
  }
  double s = 0.0;
  for (int i = 0; i < n; i++) {
    s += double(positions[i * 3 + 2]);
  }
  return float(s / double(n));
}

static double xy_snap_scale(const MeshIn &mesh)
{
  if (!mesh.positions || mesh.verts_num <= 0) {
    return 1.0e6;
  }
  double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
  for (int i = 0; i < mesh.verts_num; i++) {
    const double x = double(mesh.positions[i * 3 + 0]);
    const double y = double(mesh.positions[i * 3 + 1]);
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
  return xy(std::round(x * scale) / scale, std::round(y * scale) / scale);
}

static void collect_segments_xy(const MeshIn &mesh, std::vector<Segment_2> &segs)
{
  const double scale = xy_snap_scale(mesh);
  auto add = [&](int a, int b) {
    if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
      return;
    }
    const Point_2 p = snap_pt2(
        xy(double(mesh.positions[a * 3 + 0]), double(mesh.positions[a * 3 + 1])), scale);
    const Point_2 q = snap_pt2(
        xy(double(mesh.positions[b * 3 + 0]), double(mesh.positions[b * 3 + 1])), scale);
    if (p == q) {
      return;
    }
    segs.emplace_back(p, q);
  };
  if (mesh.edges_num > 0 && mesh.edge_v0 && mesh.edge_v1) {
    for (int i = 0; i < mesh.edges_num; i++) {
      add(mesh.edge_v0[i], mesh.edge_v1[i]);
    }
    return;
  }
  if (!mesh.corner_verts || mesh.corners_num < 3) {
    return;
  }
  const bool have_off = mesh.face_offsets != nullptr && mesh.faces_num > 0;
  const int faces = have_off ? mesh.faces_num : mesh.corners_num / 3;
  for (int f = 0; f < faces; f++) {
    const int begin = have_off ? mesh.face_offsets[f] : f * 3;
    const int end = have_off ? mesh.face_offsets[f + 1] : begin + 3;
    if (end - begin < 2) {
      continue;
    }
    for (int i = begin; i < end; i++) {
      const int j = (i + 1 == end) ? begin : i + 1;
      add(mesh.corner_verts[i], mesh.corner_verts[j]);
    }
  }
}

static void collect_face_rings_xy(const MeshIn &mesh, std::vector<std::vector<Point_2>> &rings)
{
  rings.clear();
  if (!mesh.corner_verts || mesh.corners_num < 3) {
    return;
  }
  const double scale = xy_snap_scale(mesh);
  const bool have_off = mesh.face_offsets != nullptr && mesh.faces_num > 0;
  const int faces = have_off ? mesh.faces_num : mesh.corners_num / 3;
  for (int f = 0; f < faces; f++) {
    const int begin = have_off ? mesh.face_offsets[f] : f * 3;
    const int end = have_off ? mesh.face_offsets[f + 1] : begin + 3;
    if (end - begin < 3) {
      continue;
    }
    std::vector<Point_2> ring;
    ring.reserve(size_t(end - begin));
    for (int i = begin; i < end; i++) {
      const int v = mesh.corner_verts[i];
      if (v < 0 || v >= mesh.verts_num) {
        ring.clear();
        break;
      }
      const Point_2 p = snap_pt2(
          xy(double(mesh.positions[v * 3 + 0]), double(mesh.positions[v * 3 + 1])), scale);
      if (ring.empty() || ring.back() != p) {
        ring.push_back(p);
      }
    }
    if (ring.size() >= 3 && ring.front() == ring.back()) {
      ring.pop_back();
    }
    if (ring.size() >= 3 && std::abs(signed_area_2(ring)) > 1e-16) {
      rings.push_back(std::move(ring));
    }
  }
}

static bool point_in_poly(const Point_2 &q, const std::vector<Point_2> &poly)
{
  const size_t n = poly.size();
  if (n < 3) {
    return false;
  }
  const double qx = CGAL::to_double(q.x());
  const double qy = CGAL::to_double(q.y());
  bool inside = false;
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

static bool evenodd_rings(const Point_2 &q, const std::vector<std::vector<Point_2>> &rings)
{
  int w = 0;
  for (const auto &r : rings) {
    if (point_in_poly(q, r)) {
      w++;
    }
  }
  return (w % 2) == 1;
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

static void append_shared_ngon(MeshResult &result, const std::vector<int> &ring, int tag)
{
  if (ring.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  result.corner_verts.insert(result.corner_verts.end(), ring.begin(), ring.end());
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
}

static void append_isolated_ngon(MeshResult &result,
                                 const std::vector<Point_2> &ring,
                                 float z,
                                 int tag)
{
  if (ring.size() < 3 || std::abs(signed_area_2(ring)) <= 1e-20) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  std::vector<Point_2> use = ring;
  if (signed_area_2(use) < 0.0) {
    std::reverse(use.begin(), use.end());
  }
  for (const Point_2 &p : use) {
    push_xy0(result.positions, p, z);
    result.corner_verts.push_back(int(result.corner_verts.size()));
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
}

static void append_mesh_isolated(MeshResult &dst, const MeshResult &src, int tag)
{
  if (!src.ok || src.faces_num() <= 0) {
    return;
  }
  const int base = dst.verts_num();
  dst.positions.insert(dst.positions.end(), src.positions.begin(), src.positions.end());
  if (dst.face_offsets.empty()) {
    dst.face_offsets.push_back(0);
  }
  if (src.face_offsets.empty()) {
    const int tris = src.corners_num() / 3;
    for (int t = 0; t < tris; t++) {
      dst.corner_verts.push_back(base + src.corner_verts[size_t(t * 3 + 0)]);
      dst.corner_verts.push_back(base + src.corner_verts[size_t(t * 3 + 1)]);
      dst.corner_verts.push_back(base + src.corner_verts[size_t(t * 3 + 2)]);
      dst.face_offsets.push_back(int(dst.corner_verts.size()));
      dst.face_tag.push_back(tag);
    }
  }
  else {
    for (int f = 0; f < src.faces_num(); f++) {
      const int a = src.face_offsets[size_t(f)];
      const int b = src.face_offsets[size_t(f + 1)];
      for (int c = a; c < b; c++) {
        dst.corner_verts.push_back(base + src.corner_verts[size_t(c)]);
      }
      dst.face_offsets.push_back(int(dst.corner_verts.size()));
      dst.face_tag.push_back(tag);
    }
  }
}

template<typename Arrangement>
static Point_2 face_seed(typename Arrangement::Ccb_halfedge_circulator circ)
{
  const Point_2 a = circ->source()->point();
  const Point_2 b = circ->target()->point();
  const double ax = CGAL::to_double(a.x());
  const double ay = CGAL::to_double(a.y());
  const double bx = CGAL::to_double(b.x());
  const double by = CGAL::to_double(b.y());
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len = std::hypot(dx, dy);
  const double eps = (len > 1e-18) ? (1e-8 * len) : 1e-8;
  const double inv = (len > 1e-18) ? (1.0 / len) : 1.0;
  return xy(0.5 * (ax + bx) - dy * inv * eps, 0.5 * (ay + by) + dx * inv * eps);
}

template<typename Arrangement>
static void export_arr_faces(Arrangement &arr,
                             MeshResult &result,
                             float zmid,
                             const std::function<int(const Point_2 &)> &tag_of)
{
  using Vh = typename Arrangement::Vertex_handle;
  std::map<Vh, int> vmap;
  for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {
    vmap[vit] = result.verts_num();
    push_xy0(result.positions, vit->point(), zmid);
  }
  const int max_ccb = int(arr.number_of_halfedges()) + 8;
  for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
    if (fit->is_unbounded() || !fit->has_outer_ccb()) {
      continue;
    }
    std::vector<int> ring;
    auto circ = fit->outer_ccb();
    auto cur = circ;
    int steps = 0;
    do {
      auto it = vmap.find(cur->source());
      if (it != vmap.end()) {
        ring.push_back(it->second);
      }
      ++cur;
    } while (cur != circ && ++steps < max_ccb);
    if (steps >= max_ccb) {
      continue;
    }
    uniquify_ring(ring);
    if (ring.size() < 3) {
      continue;
    }
    const int tag = tag_of(face_seed<Arrangement>(circ));
    if (tag < 0) {
      continue;
    }
    append_shared_ngon(result, ring, tag);
  }
}

static MeshResult polyhedron_to_result(const Polyhedron &poly)
{
  MeshResult r;
  std::map<const typename Polyhedron::Vertex *, int> vmap;
  for (auto v = poly.vertices_begin(); v != poly.vertices_end(); ++v) {
    vmap[&*v] = r.verts_num();
    push_xyz(r.positions, v->point());
  }
  r.face_offsets = {0};
  for (auto f = poly.facets_begin(); f != poly.facets_end(); ++f) {
    auto h = f->facet_begin();
    const auto h0 = h;
    const int before = int(r.corner_verts.size());
    do {
      r.corner_verts.push_back(vmap[&*(h->vertex())]);
    } while (++h != h0);
    if (int(r.corner_verts.size()) - before < 3) {
      r.corner_verts.resize(size_t(before));
      continue;
    }
    r.face_offsets.push_back(int(r.corner_verts.size()));
  }
  r.ok = r.faces_num() > 0;
  return r;
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

static Point_3 push_inside(Point_3 p, const std::vector<Plane_3> &planes)
{
  for (int it = 0; it < 8; it++) {
    bool moved = false;
    for (const Plane_3 &pl : planes) {
      if (pl.has_on_negative_side(p)) {
        continue;
      }
      const Vector_3 n = pl.orthogonal_vector();
      const double n2 = CGAL::to_double(n.squared_length());
      if (!(n2 > 1e-30)) {
        continue;
      }
      const double sd = CGAL::to_double(pl.a() * p.x() + pl.b() * p.y() + pl.c() * p.z() + pl.d());
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

static bool halfspace_cell(const std::vector<Plane_3> &planes, const Point_3 &origin_in, MeshResult &cell)
{
  if (planes.size() < 4) {
    return false;
  }
  Point_3 origin = push_inside(origin_in, planes);
  if (!strictly_inside_all(origin, planes)) {
    return false;
  }
  Polyhedron poly;
  CGAL::halfspace_intersection_with_constructions_3(
      planes.begin(), planes.end(), poly, std::make_optional(origin));
  if (poly.size_of_vertices() < 4 || poly.size_of_facets() < 4) {
    return false;
  }
  cell = polyhedron_to_result(poly);
  return cell.ok;
}

}  // namespace

MeshResult mesh_line_arrangement_2(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<Segment_2> segs;
    collect_segments_xy(mesh, segs);
    if (segs.empty()) {
      result.error = "Line Arrangement 2D needs edges";
      return result;
    }
    using Traits = CGAL::Arr_linear_traits_2<Kernel>;
    using Arrangement = CGAL::Arrangement_2<Traits>;
    Arrangement arr;
    std::set<std::array<long long, 3>> seen;
    for (const Segment_2 &s : segs) {
      const Point_2 &p = s.source();
      const Point_2 &q = s.target();
      if (p == q) {
        continue;
      }
      /* Normalize the line ax+by+c=0 so duplicates collapse. */
      const double ax = CGAL::to_double(p.x());
      const double ay = CGAL::to_double(p.y());
      const double bx = CGAL::to_double(q.x());
      const double by = CGAL::to_double(q.y());
      double a = ay - by;
      double b = bx - ax;
      double c = ax * by - ay * bx;
      const double n = std::hypot(a, b);
      if (!(n > 1e-18)) {
        continue;
      }
      a /= n;
      b /= n;
      c /= n;
      if (a < 0.0 || (a == 0.0 && b < 0.0)) {
        a = -a;
        b = -b;
        c = -c;
      }
      const std::array<long long, 3> k{llround(a * 1e6), llround(b * 1e6), llround(c * 1e6)};
      if (!seen.insert(k).second) {
        continue;
      }
      try {
        CGAL::insert(arr, Traits::Line_2(p, q));
      }
      catch (...) {
      }
    }
    if (arr.number_of_vertices() < 3) {
      result.error = "Line Arrangement 2D empty (need crossing lines)";
      return result;
    }
    int island = 0;
    export_arr_faces<Arrangement>(arr, result, mid_z(mesh.positions, mesh.verts_num), [&](const Point_2 &) {
      return island++;
    });
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Line Arrangement 2D produced no bounded cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Line Arrangement 2D failed";
  }
  return result;
}

MeshResult mesh_vertical_decomposition_2(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<Segment_2> segs;
    collect_segments_xy(mesh, segs);
    if (segs.empty()) {
      result.error = "Vertical Decomposition 2D needs edges";
      return result;
    }
    using Traits = CGAL::Arr_segment_traits_2<Kernel>;
    using Arrangement = CGAL::Arrangement_2<Traits>;
    Arrangement arr;
    for (const Segment_2 &s : segs) {
      try {
        CGAL::insert(arr, s);
      }
      catch (...) {
      }
    }
    if (arr.number_of_vertices() < 3) {
      result.error = "Vertical Decomposition 2D arrangement empty";
      return result;
    }

    std::vector<Segment_2> verticals;
    verticals.reserve(size_t(arr.number_of_vertices()) * 2);
    for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {
      const double vx = CGAL::to_double(vit->point().x());
      const double vy = CGAL::to_double(vit->point().y());
      double best_up = 1e300;
      double best_dn = -1e300;
      Point_2 hit_up = vit->point();
      Point_2 hit_dn = vit->point();
      bool have_up = false;
      bool have_dn = false;
      for (auto eit = arr.edges_begin(); eit != arr.edges_end(); ++eit) {
        if (eit->source() == vit || eit->target() == vit) {
          continue;
        }
        const Point_2 a = eit->source()->point();
        const Point_2 b = eit->target()->point();
        const double ax = CGAL::to_double(a.x());
        const double ay = CGAL::to_double(a.y());
        const double bx = CGAL::to_double(b.x());
        const double by = CGAL::to_double(b.y());
        const double xmin = std::min(ax, bx);
        const double xmax = std::max(ax, bx);
        if (vx < xmin - 1e-14 || vx > xmax + 1e-14) {
          continue;
        }
        if (std::abs(bx - ax) < 1e-18) {
          continue;
        }
        const double t = (vx - ax) / (bx - ax);
        if (t < -1e-9 || t > 1.0 + 1e-9) {
          continue;
        }
        const double y = ay + t * (by - ay);
        if (y > vy + 1e-12 && y < best_up) {
          best_up = y;
          hit_up = xy(vx, y);
          have_up = true;
        }
        if (y < vy - 1e-12 && y > best_dn) {
          best_dn = y;
          hit_dn = xy(vx, y);
          have_dn = true;
        }
      }
      if (have_up) {
        verticals.emplace_back(vit->point(), hit_up);
      }
      if (have_dn) {
        verticals.emplace_back(vit->point(), hit_dn);
      }
    }
    for (const Segment_2 &s : verticals) {
      if (s.source() == s.target()) {
        continue;
      }
      try {
        CGAL::insert(arr, s);
      }
      catch (...) {
      }
    }

    int island = 0;
    export_arr_faces<Arrangement>(arr, result, mid_z(mesh.positions, mesh.verts_num), [&](const Point_2 &) {
      return island++;
    });
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Vertical Decomposition 2D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Vertical Decomposition 2D failed";
  }
  return result;
}

MeshResult points_circle_arrangement_2(const float *positions,
                                       const float *radii,
                                       int n,
                                       int segments)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 1) {
    result.error = "Circle Arrangement 2D needs at least 1 point";
    return result;
  }
  try {
    segments = std::max(8, std::min(segments, 64));
    if (n > 256) {
      result.error = "Circle Arrangement 2D: too many circles (cap 256)";
      return result;
    }
    struct Disk {
      double x, y, r2, r;
    };
    std::vector<Disk> disks;
    disks.reserve(size_t(n));
    using Traits = CGAL::Arr_segment_traits_2<Kernel>;
    using Arrangement = CGAL::Arrangement_2<Traits>;
    Arrangement arr;
    int inserted = 0;
    for (int i = 0; i < n; i++) {
      const double cx = double(positions[i * 3 + 0]);
      const double cy = double(positions[i * 3 + 1]);
      double r = radii ? double(radii[i]) : 0.0;
      if (!(r > 1e-12) || !std::isfinite(r) || !std::isfinite(cx) || !std::isfinite(cy)) {
        continue;
      }
      disks.push_back({cx, cy, r * r, r});
      for (int k = 0; k < segments; k++) {
        const double t0 = (2.0 * 3.14159265358979323846 * double(k)) / double(segments);
        const double t1 = (2.0 * 3.14159265358979323846 * double(k + 1)) / double(segments);
        const Point_2 p = xy(cx + r * std::cos(t0), cy + r * std::sin(t0));
        const Point_2 q = xy(cx + r * std::cos(t1), cy + r * std::sin(t1));
        if (p == q) {
          continue;
        }
        try {
          CGAL::insert(arr, Segment_2(p, q));
          inserted++;
        }
        catch (...) {
        }
      }
    }
    if (inserted < 3 || arr.number_of_vertices() < 3) {
      result.error = "Circle Arrangement 2D empty (need positive radii)";
      return result;
    }
    const float zmid = mid_z(positions, n);
    export_arr_faces<Arrangement>(arr, result, zmid, [&](const Point_2 &q) {
      const double qx = CGAL::to_double(q.x());
      const double qy = CGAL::to_double(q.y());
      int cover = 0;
      for (const Disk &d : disks) {
        const double dx = qx - d.x;
        const double dy = qy - d.y;
        if (dx * dx + dy * dy <= d.r2 + 1e-12) {
          cover++;
        }
      }
      return cover;
    });
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Circle Arrangement 2D produced no bounded cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Circle Arrangement 2D failed";
  }
  return result;
}

MeshResult points_crust_3(const float *positions, int n)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 4) {
    result.error = "Crust 3D needs at least 4 points";
    return result;
  }
  try {
    using Vb = CGAL::Triangulation_vertex_base_with_info_3<int, Kernel>;
    using Cb = CGAL::Delaunay_triangulation_cell_base_3<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_3<Vb, Cb>;
    using DT = CGAL::Delaunay_triangulation_3<Kernel, Tds>;

    std::vector<Point_3> samples;
    samples.reserve(size_t(n));
    double minx = 1e300, miny = 1e300, minz = 1e300;
    double maxx = -1e300, maxy = -1e300, maxz = -1e300;
    for (int i = 0; i < n; i++) {
      const double x = double(positions[i * 3 + 0]);
      const double y = double(positions[i * 3 + 1]);
      const double z = double(positions[i * 3 + 2]);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      samples.push_back(xyz(x, y, z));
      minx = std::min(minx, x);
      miny = std::min(miny, y);
      minz = std::min(minz, z);
      maxx = std::max(maxx, x);
      maxy = std::max(maxy, y);
      maxz = std::max(maxz, z);
    }
    if (samples.size() < 4) {
      result.error = "Crust 3D needs at least 4 finite points";
      return result;
    }
    const double diag = std::sqrt(std::max(0.0,
                                           (maxx - minx) * (maxx - minx) +
                                               (maxy - miny) * (maxy - miny) +
                                               (maxz - minz) * (maxz - minz)));
    const double rcap = std::max(diag * 50.0, 1e-6);

    DT poles;
    for (size_t i = 0; i < samples.size(); i++) {
      auto vh = poles.insert(samples[i]);
      if (vh != DT::Vertex_handle()) {
        vh->info() = int(i);
      }
    }
    if (poles.number_of_vertices() < 4) {
      result.error = "Crust 3D: Delaunay of samples is empty";
      return result;
    }

    DT mixed = poles;
    for (auto cit = poles.finite_cells_begin(); cit != poles.finite_cells_end(); ++cit) {
      const Point_3 c = poles.dual(cit);
      const double cx = CGAL::to_double(c.x());
      const double cy = CGAL::to_double(c.y());
      const double cz = CGAL::to_double(c.z());
      if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(cz)) {
        continue;
      }
      const Point_3 p0 = cit->vertex(0)->point();
      const double rx = cx - CGAL::to_double(p0.x());
      const double ry = cy - CGAL::to_double(p0.y());
      const double rz = cz - CGAL::to_double(p0.z());
      if (rx * rx + ry * ry + rz * rz > rcap * rcap) {
        continue;
      }
      const std::size_t before = mixed.number_of_vertices();
      auto vh = mixed.insert(c);
      if (vh != DT::Vertex_handle() && mixed.number_of_vertices() > before) {
        vh->info() = -1;
      }
    }

    std::set<std::array<int, 3>> faces;
    for (auto cit = mixed.finite_cells_begin(); cit != mixed.finite_cells_end(); ++cit) {
      int id[4];
      for (int i = 0; i < 4; i++) {
        id[i] = cit->vertex(i)->info();
      }
      for (int i = 0; i < 4; i++) {
        const int a = id[(i + 1) & 3];
        const int b = id[(i + 2) & 3];
        const int c = id[(i + 3) & 3];
        if (a < 0 || b < 0 || c < 0) {
          continue;
        }
        std::array<int, 3> tri{a, b, c};
        std::sort(tri.begin(), tri.end());
        if (tri[0] == tri[1] || tri[1] == tri[2]) {
          continue;
        }
        faces.insert(tri);
      }
    }
    if (faces.empty()) {
      result.error = "Crust 3D produced no faces (need a denser sampling)";
      return result;
    }
    std::map<int, int> vmap;
    auto ensure = [&](int si) {
      auto it = vmap.find(si);
      if (it != vmap.end()) {
        return it->second;
      }
      const int vid = result.verts_num();
      push_xyz(result.positions, samples[size_t(si)]);
      vmap[si] = vid;
      return vid;
    };
    result.face_offsets = {0};
    for (const auto &tri : faces) {
      result.corner_verts.push_back(ensure(tri[0]));
      result.corner_verts.push_back(ensure(tri[1]));
      result.corner_verts.push_back(ensure(tri[2]));
      result.face_offsets.push_back(int(result.corner_verts.size()));
    }
    result.ok = result.faces_num() > 0;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Crust 3D failed";
  }
  return result;
}

MeshResult points_clipped_voronoi_3(const float *positions, int n, float clip_margin)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 2) {
    result.error = "Clipped Voronoi 3D needs at least 2 points";
    return result;
  }
  try {
    using DT = CGAL::Delaunay_triangulation_3<Kernel>;
    std::vector<Point_3> pts;
    pts.reserve(size_t(n));
    double minx = 1e300, miny = 1e300, minz = 1e300;
    double maxx = -1e300, maxy = -1e300, maxz = -1e300;
    std::set<std::array<long long, 3>> seen;
    for (int i = 0; i < n; i++) {
      const double x = double(positions[i * 3 + 0]);
      const double y = double(positions[i * 3 + 1]);
      const double z = double(positions[i * 3 + 2]);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const Point_3 p = xyz(x, y, z);
      if (!seen.insert(key3(p)).second) {
        continue;
      }
      pts.push_back(p);
      minx = std::min(minx, x);
      miny = std::min(miny, y);
      minz = std::min(minz, z);
      maxx = std::max(maxx, x);
      maxy = std::max(maxy, y);
      maxz = std::max(maxz, z);
    }
    if (pts.size() < 2) {
      result.error = "Clipped Voronoi 3D needs at least 2 distinct points";
      return result;
    }
    if (pts.size() > 2000) {
      result.error = "Clipped Voronoi 3D: too many sites (cap 2000)";
      return result;
    }
    const double diag = std::sqrt(std::max(0.0,
                                           (maxx - minx) * (maxx - minx) +
                                               (maxy - miny) * (maxy - miny) +
                                               (maxz - minz) * (maxz - minz)));
    const double pad = std::max(double(std::max(0.0f, clip_margin)), std::max(1e-4 * std::max(diag, 1.0), 1e-6));
    minx -= pad;
    miny -= pad;
    minz -= pad;
    maxx += pad;
    maxy += pad;
    maxz += pad;

    DT dt(pts.begin(), pts.end());
    if (dt.number_of_vertices() < 1) {
      result.error = "Clipped Voronoi 3D Delaunay empty";
      return result;
    }

    auto bbox_planes = [&]() {
      std::vector<Plane_3> pl;
      pl.emplace_back(Point_3(FT(minx), FT(0), FT(0)), Vector_3(FT(-1), FT(0), FT(0)));
      pl.emplace_back(Point_3(FT(maxx), FT(0), FT(0)), Vector_3(FT(1), FT(0), FT(0)));
      pl.emplace_back(Point_3(FT(0), FT(miny), FT(0)), Vector_3(FT(0), FT(-1), FT(0)));
      pl.emplace_back(Point_3(FT(0), FT(maxy), FT(0)), Vector_3(FT(0), FT(1), FT(0)));
      pl.emplace_back(Point_3(FT(0), FT(0), FT(minz)), Vector_3(FT(0), FT(0), FT(-1)));
      pl.emplace_back(Point_3(FT(0), FT(0), FT(maxz)), Vector_3(FT(0), FT(0), FT(1)));
      return pl;
    };

    int site = 0;
    for (auto vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); ++vit, ++site) {
      std::vector<Plane_3> planes = bbox_planes();
      std::vector<DT::Vertex_handle> nbs;
      dt.finite_adjacent_vertices(vit, std::back_inserter(nbs));
      const Point_3 a = vit->point();
      for (const DT::Vertex_handle &uh : nbs) {
        const Point_3 b = uh->point();
        const double ax = CGAL::to_double(a.x());
        const double ay = CGAL::to_double(a.y());
        const double az = CGAL::to_double(a.z());
        const double bx = CGAL::to_double(b.x());
        const double by = CGAL::to_double(b.y());
        const double bz = CGAL::to_double(b.z());
        const Point_3 mid = xyz(0.5 * (ax + bx), 0.5 * (ay + by), 0.5 * (az + bz));
        const Vector_3 n(FT(bx - ax), FT(by - ay), FT(bz - az));
        if (n.squared_length() <= FT(0)) {
          continue;
        }
        planes.emplace_back(mid, n);
      }
      MeshResult cell;
      if (!halfspace_cell(planes, a, cell)) {
        continue;
      }
      append_mesh_isolated(result, cell, site);
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Clipped Voronoi 3D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Clipped Voronoi 3D failed";
  }
  return result;
}

MeshResult mesh_complement_2(const MeshIn &mesh, float padding)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<Segment_2> segs;
    collect_segments_xy(mesh, segs);
    std::vector<std::vector<Point_2>> rings;
    collect_face_rings_xy(mesh, rings);
    if (!mesh.positions || mesh.verts_num < 1) {
      result.error = "Complement 2D needs a mesh";
      return result;
    }
    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (int i = 0; i < mesh.verts_num; i++) {
      const double x = double(mesh.positions[i * 3 + 0]);
      const double y = double(mesh.positions[i * 3 + 1]);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      minx = std::min(minx, x);
      miny = std::min(miny, y);
      maxx = std::max(maxx, x);
      maxy = std::max(maxy, y);
    }
    if (!(maxx >= minx) || !(maxy >= miny)) {
      result.error = "Complement 2D: empty XY bounds";
      return result;
    }
    const double diag = std::hypot(std::max(0.0, maxx - minx), std::max(0.0, maxy - miny));
    const double pad = std::max(double(std::max(0.0f, padding)), 1e-6 * std::max(diag, 1.0));
    minx -= pad;
    miny -= pad;
    maxx += pad;
    maxy += pad;
    const Point_2 c0 = xy(minx, miny);
    const Point_2 c1 = xy(maxx, miny);
    const Point_2 c2 = xy(maxx, maxy);
    const Point_2 c3 = xy(minx, maxy);
    segs.emplace_back(c0, c1);
    segs.emplace_back(c1, c2);
    segs.emplace_back(c2, c3);
    segs.emplace_back(c3, c0);

    if (rings.empty() && segs.size() == 4) {
      append_isolated_ngon(result, {c0, c1, c2, c3}, mid_z(mesh.positions, mesh.verts_num), 0);
      result.ok = true;
      return result;
    }

    using Traits = CGAL::Arr_segment_traits_2<Kernel>;
    using Arrangement = CGAL::Arrangement_2<Traits>;
    Arrangement arr;
    for (const Segment_2 &s : segs) {
      try {
        CGAL::insert(arr, s);
      }
      catch (...) {
      }
    }
    if (arr.number_of_vertices() < 3) {
      result.error = "Complement 2D arrangement empty";
      return result;
    }
    int island = 0;
    const float zmid = mid_z(mesh.positions, mesh.verts_num);
    export_arr_faces<Arrangement>(arr, result, zmid, [&](const Point_2 &q) {
      const double qx = CGAL::to_double(q.x());
      const double qy = CGAL::to_double(q.y());
      if (qx < minx - 1e-9 || qx > maxx + 1e-9 || qy < miny - 1e-9 || qy > maxy + 1e-9) {
        return -1;
      }
      if (!rings.empty() && evenodd_rings(q, rings)) {
        return -1;
      }
      return island++;
    });
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Complement 2D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Complement 2D failed";
  }
  return result;
}

MeshResult points_simple_polygon_2(const float *positions, int n)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 3) {
    result.error = "Simple Polygon 2D needs at least 3 points";
    return result;
  }
  try {
    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    std::set<std::pair<long long, long long>> seen;
    double zsum = 0.0;
    int zcount = 0;
    for (int i = 0; i < n; i++) {
      const double x = double(positions[i * 3 + 0]);
      const double y = double(positions[i * 3 + 1]);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      const Point_2 p = xy(x, y);
      if (!seen.insert(key2(p)).second) {
        continue;
      }
      pts.push_back(p);
      zsum += double(positions[i * 3 + 2]);
      zcount++;
    }
    if (pts.size() < 3) {
      result.error = "Simple Polygon 2D needs at least 3 distinct XY points";
      return result;
    }
    bool all_col = true;
    for (size_t i = 2; i < pts.size(); i++) {
      if (CGAL::orientation(pts[0], pts[1], pts[i]) != CGAL::COLLINEAR) {
        all_col = false;
        break;
      }
    }
    if (all_col) {
      result.error = "Simple Polygon 2D: points are collinear";
      return result;
    }
    CGAL::make_simple_polygon(pts.begin(), pts.end(), Kernel());
    if (pts.size() >= 3 && pts.front() == pts.back()) {
      pts.pop_back();
    }
    if (pts.size() < 3) {
      result.error = "Simple Polygon 2D failed to uncross";
      return result;
    }
    append_isolated_ngon(result, pts, zcount ? float(zsum / double(zcount)) : 0.0f, 0);
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Simple Polygon 2D produced an empty polygon";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Simple Polygon 2D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
