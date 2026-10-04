/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 39 — 8 new nodes (not previously deleted):
 *  - Farthest Voronoi 2D
 *  - Convex Layers 3D
 *  - Overlay 2D
 *  - Polygon Kernel 2D
 *  - Self Intersection Curves
 *  - Crust 2D
 *  - Geodesic Voronoi
 *  - Isosurface 3D
 *
 * Skipped: OpenGR, Kinetic EPECK, Mesh_3 volume, PolyFit SCIP, Ridges_3,
 * Classification, OSQP, Hyperbolic Delaunay, weighted Delaunay / Lloyd,
 * banned measure/query/remesh, previously deleted nodes.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/intersections.h>
#include <CGAL/Intersections_3/Triangle_3_Triangle_3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <variant>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Segment_2 = Kernel::Segment_2;
using Segment_3 = Kernel::Segment_3;
using Triangle_3 = Kernel::Triangle_3;
using Vector_3 = Kernel::Vector_3;
using FT = Kernel::FT;

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

static void add_wire(WireResult &w, int a, int b)
{
  if (a == b) {
    return;
  }
  w.edge_v0.push_back(a);
  w.edge_v1.push_back(b);
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
      return xy(ax + t * (bx - ax), ay + t * (by - ay));
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

static std::vector<Point_2> sh_clip_halfplane(const std::vector<Point_2> &in,
                                              const Point_2 &a,
                                              const Point_2 &b)
{
  /* Keep the closed left half-plane of directed edge a→b. */
  std::vector<Point_2> out;
  if (in.empty()) {
    return out;
  }
  auto orient = [&](const Point_2 &p) { return CGAL::orientation(a, b, p); };
  auto isect = [&](const Point_2 &p, const Point_2 &q) {
    const Kernel::Line_2 line(a, b);
    const Kernel::Line_2 edge(p, q);
    const auto hit = CGAL::intersection(line, edge);
    if (const Point_2 *pt = hit ? std::get_if<Point_2>(&*hit) : nullptr) {
      return *pt;
    }
    const double ax = CGAL::to_double(a.x());
    const double ay = CGAL::to_double(a.y());
    const double bx = CGAL::to_double(b.x());
    const double by = CGAL::to_double(b.y());
    const double px = CGAL::to_double(p.x());
    const double py = CGAL::to_double(p.y());
    const double qx = CGAL::to_double(q.x());
    const double qy = CGAL::to_double(q.y());
    const double dx = bx - ax;
    const double dy = by - ay;
    const double ex = qx - px;
    const double ey = qy - py;
    const double den = dx * ey - dy * ex;
    if (std::abs(den) < 1e-18) {
      return p;
    }
    const double t = ((ax - px) * dy - (ay - py) * dx) / den;
    return xy(px + t * ex, py + t * ey);
  };
  Point_2 prev = in.back();
  bool prev_in = orient(prev) != CGAL::RIGHT_TURN;
  for (const Point_2 &cur : in) {
    const bool cur_in = orient(cur) != CGAL::RIGHT_TURN;
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
    const Point_2 p = snap_pt2(xy(double(mesh.positions[a * 3 + 0]), double(mesh.positions[a * 3 + 1])),
                               scale);
    const Point_2 q = snap_pt2(xy(double(mesh.positions[b * 3 + 0]), double(mesh.positions[b * 3 + 1])),
                               scale);
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

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  PMP::triangulate_faces(sm);
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

static Surface_mesh::Vertex_index vertex_from_orig_index(const Surface_mesh &sm, int orig_i)
{
  if (orig_i >= 0 && orig_i < int(sm.number_of_vertices())) {
    Surface_mesh::Vertex_index v(orig_i);
    if (sm.is_valid(v) && !sm.is_removed(v)) {
      return v;
    }
  }
  int i = 0;
  for (const auto v : sm.vertices()) {
    if (sm.is_removed(v)) {
      continue;
    }
    if (i == orig_i) {
      return v;
    }
    i++;
  }
  return Surface_mesh::null_vertex();
}

static bool clip_ray_aabb2(
    double ox, double oy, double dx, double dy, double minx, double miny, double maxx, double maxy, double &hx, double &hy)
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

static int add_result_vert(MeshResult &r, const Point_3 &p)
{
  const int i = r.verts_num();
  push_xyz(r.positions, p);
  return i;
}

static void add_result_tri(MeshResult &r, int a, int b, int c, int tag)
{
  if (a == b || b == c || c == a) {
    return;
  }
  r.corner_verts.push_back(a);
  r.corner_verts.push_back(b);
  r.corner_verts.push_back(c);
  r.face_tag.push_back(tag);
}

}  // namespace

MeshResult points_farthest_voronoi_2(const float *positions, int n, float clip_margin)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 3) {
    result.error = "Farthest Voronoi 2D needs at least 3 points";
    return result;
  }
  try {
    std::vector<Point_2> sites;
    sites.reserve(size_t(n));
    std::vector<Point_3> lifted;
    lifted.reserve(size_t(n));
    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (int i = 0; i < n; i++) {
      const double x = double(positions[i * 3 + 0]);
      const double y = double(positions[i * 3 + 1]);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }
      sites.push_back(xy(x, y));
      lifted.push_back(xyz(x, y, x * x + y * y));
      minx = std::min(minx, x);
      miny = std::min(miny, y);
      maxx = std::max(maxx, x);
      maxy = std::max(maxy, y);
    }
    if (sites.size() < 3) {
      result.error = "Farthest Voronoi 2D needs at least 3 finite XY points";
      return result;
    }
    const double m = std::max(double(clip_margin), 0.0);
    minx -= m;
    miny -= m;
    maxx += m;
    maxy += m;
    if (!(maxx > minx) || !(maxy > miny)) {
      result.error = "Farthest Voronoi 2D degenerate bbox";
      return result;
    }

    Surface_mesh hull;
    CGAL::convex_hull_3(lifted.begin(), lifted.end(), hull);
    if (hull.number_of_faces() < 1) {
      result.error = "Farthest Voronoi 2D: lift hull empty";
      return result;
    }

    /* Upper hull faces (outward +Z) are farthest-point Delaunay triangles. */
    struct FTri {
      Point_2 p[3];
      Point_2 c;
    };
    std::vector<FTri> fdt;
    for (const auto f : hull.faces()) {
      std::array<Point_3, 3> q;
      int k = 0;
      for (auto h : CGAL::halfedges_around_face(hull.halfedge(f), hull)) {
        if (k < 3) {
          q[size_t(k++)] = hull.point(hull.target(h));
        }
      }
      if (k != 3) {
        continue;
      }
      const Vector_3 n = CGAL::cross_product(q[1] - q[0], q[2] - q[0]);
      if (!(CGAL::to_double(n.z()) > 1e-18)) {
        continue;
      }
      FTri t;
      t.p[0] = xy(CGAL::to_double(q[0].x()), CGAL::to_double(q[0].y()));
      t.p[1] = xy(CGAL::to_double(q[1].x()), CGAL::to_double(q[1].y()));
      t.p[2] = xy(CGAL::to_double(q[2].x()), CGAL::to_double(q[2].y()));
      if (CGAL::collinear(t.p[0], t.p[1], t.p[2])) {
        continue;
      }
      t.c = CGAL::circumcenter(t.p[0], t.p[1], t.p[2]);
      fdt.push_back(t);
    }
    if (fdt.empty()) {
      result.error = "Farthest Voronoi 2D: no upper-hull triangles (points almost cocircular?)";
      return result;
    }

    std::vector<Point_2> hull2;
    CGAL::convex_hull_2(sites.begin(), sites.end(), std::back_inserter(hull2));
    if (hull2.size() < 3) {
      result.error = "Farthest Voronoi 2D: XY hull degenerate";
      return result;
    }

    const Point_2 corners[4] = {xy(minx, miny), xy(maxx, miny), xy(maxx, maxy), xy(minx, maxy)};
    const float zmid = mid_z(positions, n);
    const double diag2 = std::max(1e-24, (maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny));

    auto farthest_site = [&](const Point_2 &q) -> int {
      int best = 0;
      double best_d = -1.0;
      for (size_t i = 0; i < hull2.size(); i++) {
        const double dx = CGAL::to_double(q.x() - hull2[i].x());
        const double dy = CGAL::to_double(q.y() - hull2[i].y());
        const double d = dx * dx + dy * dy;
        if (d > best_d) {
          best_d = d;
          best = int(i);
        }
      }
      return best;
    };

    for (size_t si = 0; si < hull2.size(); si++) {
      const Point_2 &s = hull2[si];
      std::vector<Point_2> cand;
      for (const FTri &t : fdt) {
        bool inc = false;
        for (int k = 0; k < 3; k++) {
          if (key2(t.p[k]) == key2(s)) {
            inc = true;
            break;
          }
        }
        if (inc) {
          cand.push_back(t.c);
        }
      }
      /* Unbounded rays: hull edges incident to s that have only one FDT triangle. */
      const Point_2 &prev = hull2[(si + hull2.size() - 1) % hull2.size()];
      const Point_2 &next = hull2[(si + 1) % hull2.size()];
      const Point_2 hull_nb[2] = {prev, next};
      for (const Point_2 &nb : hull_nb) {
        int hits = 0;
        Point_2 cc;
        for (const FTri &t : fdt) {
          bool has_s = false, has_n = false;
          for (int k = 0; k < 3; k++) {
            if (key2(t.p[k]) == key2(s)) {
              has_s = true;
            }
            if (key2(t.p[k]) == key2(nb)) {
              has_n = true;
            }
          }
          if (has_s && has_n) {
            hits++;
            cc = t.c;
          }
        }
        if (hits != 1) {
          continue;
        }
        const double mx = 0.5 * (CGAL::to_double(s.x()) + CGAL::to_double(nb.x()));
        const double my = 0.5 * (CGAL::to_double(s.y()) + CGAL::to_double(nb.y()));
        double dx = CGAL::to_double(nb.x()) - CGAL::to_double(s.x());
        double dy = CGAL::to_double(nb.y()) - CGAL::to_double(s.y());
        /* Outward normal of hull edge s→nb (hull is CCW). */
        double nx = dy;
        double ny = -dx;
        const double cx = CGAL::to_double(cc.x());
        const double cy = CGAL::to_double(cc.y());
        /* Ray from circumcenter through the outward side of the mid-edge. */
        if ((mx - cx) * nx + (my - cy) * ny < 0.0) {
          nx = -nx;
          ny = -ny;
        }
        double hx, hy;
        if (clip_ray_aabb2(cx, cy, nx, ny, minx, miny, maxx, maxy, hx, hy)) {
          cand.push_back(xy(hx, hy));
        }
      }
      for (const Point_2 &q : corners) {
        if (farthest_site(q) == int(si)) {
          cand.push_back(q);
        }
      }
      cand = unique_points_2(std::move(cand), 1e-20 * diag2);
      if (cand.size() < 3) {
        continue;
      }
      std::vector<Point_2> cell;
      CGAL::convex_hull_2(cand.begin(), cand.end(), std::back_inserter(cell));
      cell = sh_clip_aabb2(std::move(cell), minx, miny, maxx, maxy);
      cell = unique_points_2(std::move(cell), 1e-20 * diag2);
      append_isolated_ngon(result, cell, zmid, int(si));
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Farthest Voronoi 2D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Farthest Voronoi 2D failed";
  }
  return result;
}

MeshResult points_convex_layers_3(const float *positions, int n)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 4) {
    result.error = "Convex Layers 3D needs at least 4 points";
    return result;
  }
  try {
    std::vector<Point_3> remain;
    remain.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      remain.push_back(xyz(double(positions[i * 3 + 0]),
                           double(positions[i * 3 + 1]),
                           double(positions[i * 3 + 2])));
    }
    int layer = 0;
    while (remain.size() >= 4) {
      Surface_mesh hull;
      CGAL::convex_hull_3(remain.begin(), remain.end(), hull);
      MeshResult piece = surface_mesh_to_result(hull);
      if (!piece.ok || piece.faces_num() < 1) {
        break;
      }
      append_mesh_isolated(result, piece, layer++);
      std::set<std::array<long long, 3>> on_hull;
      for (const auto v : hull.vertices()) {
        on_hull.insert(key3(hull.point(v)));
      }
      std::vector<Point_3> next;
      next.reserve(remain.size());
      for (const Point_3 &p : remain) {
        if (on_hull.find(key3(p)) == on_hull.end()) {
          next.push_back(p);
        }
      }
      if (next.size() == remain.size()) {
        break;
      }
      remain.swap(next);
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Convex Layers 3D produced no hulls";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Convex Layers 3D failed";
  }
  return result;
}

MeshResult mesh_overlay_2(const MeshIn &mesh_a, const MeshIn &mesh_b)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<Segment_2> segs;
    collect_segments_xy(mesh_a, segs);
    collect_segments_xy(mesh_b, segs);
    if (segs.empty()) {
      result.error = "Overlay 2D needs edges on Mesh A or Mesh B";
      return result;
    }
    std::vector<std::vector<Point_2>> rings_a, rings_b;
    collect_face_rings_xy(mesh_a, rings_a);
    collect_face_rings_xy(mesh_b, rings_b);

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
      result.error = "Overlay 2D arrangement empty";
      return result;
    }

    const float zmid = 0.5f * (mid_z(mesh_a.positions, mesh_a.verts_num) +
                               mid_z(mesh_b.positions, mesh_b.verts_num));
    std::map<typename Arrangement::Vertex_handle, int> vmap;
    for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {
      vmap[vit] = result.verts_num();
      push_xy0(result.positions, vit->point(), zmid);
    }

    const int max_ccb = int(arr.number_of_halfedges()) + 8;
    auto collect_ccb = [&](typename Arrangement::Ccb_halfedge_circulator circ) {
      std::vector<int> ring;
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
        return std::vector<int>();
      }
      uniquify_ring(ring);
      return ring;
    };

    auto seed = [](typename Arrangement::Ccb_halfedge_circulator circ) {
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
      return xy(0.5 * (ax + bx) - dy / (len > 1e-18 ? len : 1.0) * eps,
                0.5 * (ay + by) + dx / (len > 1e-18 ? len : 1.0) * eps);
    };

    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
      if (fit->is_unbounded() || !fit->has_outer_ccb()) {
        continue;
      }
      const std::vector<int> outer = collect_ccb(fit->outer_ccb());
      if (outer.size() < 3) {
        continue;
      }
      const Point_2 q = seed(fit->outer_ccb());
      const bool have_a = !rings_a.empty();
      const bool have_b = !rings_b.empty();
      bool in_a = have_a && evenodd_rings(q, rings_a);
      bool in_b = have_b && evenodd_rings(q, rings_b);
      if (!have_a && !have_b) {
        /* Pure wires: keep every bounded cell. */
        in_a = in_b = true;
      }
      if (!in_a && !in_b) {
        continue;
      }
      const int tag = (in_a ? 1 : 0) + (in_b ? 2 : 0);
      append_shared_ngon(result, outer, tag);
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Overlay 2D produced no labeled cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Overlay 2D failed";
  }
  return result;
}

MeshResult mesh_polygon_kernel_2(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!mesh.positions || mesh.verts_num < 3) {
    result.error = "Polygon Kernel 2D needs at least 3 vertices";
    return result;
  }
  try {
    std::vector<std::vector<Point_2>> rings;
    collect_face_rings_xy(mesh, rings);
    if (rings.empty() && mesh.edges_num > 0) {
      /* Wire loops: treat each as a ring via arrangement-less walk is hard;
       * fall back to the convex hull of the projected verts. */
      std::vector<Point_2> pts;
      for (int i = 0; i < mesh.verts_num; i++) {
        pts.push_back(xy(double(mesh.positions[i * 3 + 0]), double(mesh.positions[i * 3 + 1])));
      }
      std::vector<Point_2> hull;
      CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
      if (hull.size() >= 3) {
        rings.push_back(std::move(hull));
      }
    }
    if (rings.empty()) {
      result.error = "Polygon Kernel 2D found no XY faces";
      return result;
    }
    const float zmid = mid_z(mesh.positions, mesh.verts_num);
    int tag = 0;
    for (std::vector<Point_2> ring : rings) {
      if (ring.size() < 3) {
        continue;
      }
      if (signed_area_2(ring) < 0.0) {
        std::reverse(ring.begin(), ring.end());
      }
      double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
      for (const Point_2 &p : ring) {
        minx = std::min(minx, CGAL::to_double(p.x()));
        miny = std::min(miny, CGAL::to_double(p.y()));
        maxx = std::max(maxx, CGAL::to_double(p.x()));
        maxy = std::max(maxy, CGAL::to_double(p.y()));
      }
      const double pad = std::max(1e-6, 0.05 * std::hypot(maxx - minx, maxy - miny));
      std::vector<Point_2> kern = {xy(minx - pad, miny - pad),
                                   xy(maxx + pad, miny - pad),
                                   xy(maxx + pad, maxy + pad),
                                   xy(minx - pad, maxy + pad)};
      for (size_t i = 0; i < ring.size(); i++) {
        kern = sh_clip_halfplane(kern, ring[i], ring[(i + 1) % ring.size()]);
        if (kern.size() < 3) {
          break;
        }
      }
      kern = unique_points_2(std::move(kern), 1e-20);
      append_isolated_ngon(result, kern, zmid, tag++);
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Polygon Kernel 2D empty (not star-shaped / half-planes miss)";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Polygon Kernel 2D failed";
  }
  return result;
}

bool mesh_self_intersection_polylines(const MeshIn &mesh,
                                      std::vector<std::vector<float>> &out_polylines,
                                      std::string &error)
{
  out_polylines.clear();
  CgalThrowGuard guard;
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    if (error.empty()) {
      error = "Self Intersection Curves: empty/invalid mesh";
    }
    return false;
  }
  try {
    using Face = Surface_mesh::Face_index;
    std::vector<std::pair<Face, Face>> pairs;
    PMP::self_intersections(sm, std::back_inserter(pairs));
    if (pairs.empty()) {
      return true;
    }

    struct Seg {
      Point_3 a, b;
    };
    std::vector<Seg> segs;
    segs.reserve(pairs.size());
    auto face_tri = [&](Face f) -> Triangle_3 {
      std::array<Point_3, 3> p;
      int k = 0;
      for (auto h : CGAL::halfedges_around_face(sm.halfedge(f), sm)) {
        if (k < 3) {
          p[size_t(k++)] = sm.point(sm.target(h));
        }
      }
      return Triangle_3(p[0], p[1], p[2]);
    };
    auto push_seg = [&](Point_3 a, Point_3 b) {
      if (CGAL::squared_distance(a, b) <= 1e-24) {
        return;
      }
      segs.push_back(Seg{a, b});
    };

    for (const auto &pr : pairs) {
      if (pr.first == pr.second) {
        continue;
      }
      const Triangle_3 t1 = face_tri(pr.first);
      const Triangle_3 t2 = face_tri(pr.second);
      if (t1.is_degenerate() || t2.is_degenerate()) {
        continue;
      }
      const auto hit = CGAL::intersection(t1, t2);
      if (!hit) {
        continue;
      }
      if (const Segment_3 *s = std::get_if<Segment_3>(&*hit)) {
        push_seg(s->source(), s->target());
      }
      else if (const Point_3 *p = std::get_if<Point_3>(&*hit)) {
        (void)p;
      }
      else if (const Triangle_3 *tr = std::get_if<Triangle_3>(&*hit)) {
        push_seg(tr->vertex(0), tr->vertex(1));
        push_seg(tr->vertex(1), tr->vertex(2));
        push_seg(tr->vertex(2), tr->vertex(0));
      }
      else if (const std::vector<Point_3> *poly = std::get_if<std::vector<Point_3>>(&*hit)) {
        for (size_t i = 0; i + 1 < poly->size(); i++) {
          push_seg((*poly)[i], (*poly)[i + 1]);
        }
      }
    }

    /* Greedy stitch by snapped endpoints. */
    auto kpt = [](const Point_3 &p) {
      return std::array<long long, 3>{llround(CGAL::to_double(p.x()) * 1e6),
                                      llround(CGAL::to_double(p.y()) * 1e6),
                                      llround(CGAL::to_double(p.z()) * 1e6)};
    };
    std::vector<char> used(segs.size(), 0);
    for (size_t i = 0; i < segs.size(); i++) {
      if (used[i]) {
        continue;
      }
      used[i] = 1;
      std::vector<Point_3> poly = {segs[i].a, segs[i].b};
      bool grew = true;
      while (grew) {
        grew = false;
        const auto head = kpt(poly.front());
        const auto tail = kpt(poly.back());
        for (size_t j = 0; j < segs.size(); j++) {
          if (used[j]) {
            continue;
          }
          const auto sa = kpt(segs[j].a);
          const auto sb = kpt(segs[j].b);
          if (sa == tail) {
            poly.push_back(segs[j].b);
            used[j] = 1;
            grew = true;
          }
          else if (sb == tail) {
            poly.push_back(segs[j].a);
            used[j] = 1;
            grew = true;
          }
          else if (sa == head) {
            poly.insert(poly.begin(), segs[j].b);
            used[j] = 1;
            grew = true;
          }
          else if (sb == head) {
            poly.insert(poly.begin(), segs[j].a);
            used[j] = 1;
            grew = true;
          }
          if (grew) {
            break;
          }
        }
      }
      if (poly.size() < 2) {
        continue;
      }
      std::vector<float> xyz;
      xyz.reserve(poly.size() * 3);
      for (const Point_3 &p : poly) {
        push_xyz(xyz, p);
      }
      out_polylines.push_back(std::move(xyz));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Self Intersection Curves failed";
    return false;
  }
}

WireResult points_crust_2(const float *positions, int n)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!positions || n < 3) {
    result.error = "Crust 2D needs at least 3 points";
    return result;
  }
  try {
    using Vb = CGAL::Triangulation_vertex_base_with_info_2<int, Kernel>;
    using Fb = CGAL::Triangulation_face_base_2<Kernel>;
    using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using DT = CGAL::Delaunay_triangulation_2<Kernel, TDS>;

    DT dt_s;
    std::vector<typename DT::Vertex_handle> samples;
    samples.reserve(size_t(n));
    double zacc = 0.0;
    for (int i = 0; i < n; i++) {
      const Point_2 p = xy(double(positions[i * 3 + 0]), double(positions[i * 3 + 1]));
      auto v = dt_s.insert(p);
      v->info() = 1;
      samples.push_back(v);
      zacc += double(positions[i * 3 + 2]);
      push_xy0(result.positions, p, 0.0f);
    }
    const float zmid = float(zacc / double(n));
    for (size_t i = 0; i < result.positions.size(); i += 3) {
      result.positions[i + 2] = zmid;
    }
    if (dt_s.number_of_vertices() < 3 || dt_s.dimension() != 2) {
      result.error = "Crust 2D degenerate Delaunay";
      return result;
    }

    std::vector<Point_2> vor;
    for (auto fit = dt_s.finite_faces_begin(); fit != dt_s.finite_faces_end(); ++fit) {
      vor.push_back(dt_s.circumcenter(fit));
    }

    DT dt;
    for (int i = 0; i < n; i++) {
      auto v = dt.insert(xy(double(positions[i * 3 + 0]), double(positions[i * 3 + 1])));
      v->info() = i; /* sample index, >=0 */
    }
    for (const Point_2 &p : vor) {
      const size_t nv = dt.number_of_vertices();
      auto v = dt.insert(p);
      if (dt.number_of_vertices() == nv) {
        /* Coincided with a sample — leave as sample. */
        continue;
      }
      v->info() = -1;
    }

    std::set<std::pair<int, int>> seen;
    for (auto eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
      auto va = eit->first->vertex(DT::cw(eit->second));
      auto vb = eit->first->vertex(DT::ccw(eit->second));
      if (va->info() < 0 || vb->info() < 0) {
        continue;
      }
      int a = va->info();
      int b = vb->info();
      if (a > b) {
        std::swap(a, b);
      }
      if (a == b || !seen.insert({a, b}).second) {
        continue;
      }
      add_wire(result, a, b);
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Crust 2D produced no sample-sample edges";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Crust 2D failed";
  }
  return result;
}

MeshResult mesh_geodesic_voronoi(const MeshIn &mesh, const uint8_t *source_mask)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!source_mask) {
    result.error = "Geodesic Voronoi needs a source mask";
    return result;
  }
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Geodesic Voronoi: empty mesh" : error;
    return result;
  }
  try {
    using Traits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Surface_mesh>;
    using Shortest_paths = CGAL::Surface_mesh_shortest_path<Traits>;
    using Face_loc = typename Shortest_paths::Face_location;
    using SrcIt = typename Shortest_paths::Source_point_iterator;
    Shortest_paths sp(sm);

    std::vector<int> src_orig;
    std::vector<SrcIt> src_its;
    for (int i = 0; i < mesh.verts_num; i++) {
      if (!source_mask[i]) {
        continue;
      }
      const auto v = vertex_from_orig_index(sm, i);
      if (v == Surface_mesh::null_vertex()) {
        continue;
      }
      src_its.push_back(sp.add_source_point(v));
      src_orig.push_back(i);
    }
    if (src_orig.empty()) {
      result.error = "Geodesic Voronoi needs at least one source vertex";
      return result;
    }

    auto src_orig_of = [&](SrcIt it) -> int {
      if (it == sp.source_points_end()) {
        return -1;
      }
      for (int k = 0; k < int(src_its.size()); k++) {
        if (it == src_its[size_t(k)]) {
          return src_orig[size_t(k)];
        }
      }
      int k = 0;
      for (auto jt = sp.source_points_begin(); jt != sp.source_points_end(); ++jt, ++k) {
        if (jt == it && k < int(src_orig.size())) {
          return src_orig[size_t(k)];
        }
      }
      return -1;
    };

    std::map<Surface_mesh::Vertex_index, int> cell;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      const auto res = sp.shortest_distance_to_source_points(v);
      const double d = CGAL::to_double(res.first);
      cell[v] = (d >= 0.0) ? src_orig_of(res.second) : -1;
    }

    auto query_he_t = [&](Surface_mesh::Halfedge_index he, double t) -> int {
      if (he == Surface_mesh::null_halfedge() || sm.is_border(he)) {
        return -1;
      }
      t = std::min(1.0, std::max(0.0, t));
      const Face_loc loc = sp.face_location(he, FT(t));
      if (loc.first == Surface_mesh::null_face()) {
        return -1;
      }
      const auto res = sp.shortest_distance_to_source_points(loc.first, loc.second);
      return src_orig_of(res.second);
    };

    std::map<Surface_mesh::Vertex_index, int> emit_of;
    auto emit_existing = [&](Surface_mesh::Vertex_index v) {
      auto it = emit_of.find(v);
      if (it != emit_of.end()) {
        return it->second;
      }
      const int id = add_result_vert(result, sm.point(v));
      emit_of[v] = id;
      return id;
    };

    auto lerp_he = [&](Surface_mesh::Halfedge_index he, double t) -> Point_3 {
      const Point_3 pa = sm.point(sm.source(he));
      const Point_3 pb = sm.point(sm.target(he));
      return Point_3(pa.x() + FT(t) * (pb.x() - pa.x()),
                     pa.y() + FT(t) * (pb.y() - pa.y()),
                     pa.z() + FT(t) * (pb.z() - pa.z()));
    };

    /* Crossing of the geodesic Voronoi bisector on a mesh edge.
     * t is MMP-bisection along `he_store` (0 at source, 1 at target). */
    struct Crossing {
      double t;
      int emit_id;
      int src0;
      int src1;
    };
    struct EdgeHits {
      Surface_mesh::Halfedge_index he_store;
      std::vector<Crossing> hits;
    };
    std::map<Surface_mesh::Edge_index, EdgeHits> edge_hits;

    auto bisect_he = [&](Surface_mesh::Halfedge_index he, double t0, double t1, int s0, int s1) {
      for (int it = 0; it < 32; it++) {
        const double tm = 0.5 * (t0 + t1);
        const int smid = query_he_t(he, tm);
        if (smid == s0) {
          t0 = tm;
        }
        else {
          t1 = tm;
        }
      }
      Crossing c;
      c.t = 0.5 * (t0 + t1);
      c.emit_id = add_result_vert(result, lerp_he(he, c.t));
      c.src0 = s0;
      c.src1 = s1;
      return c;
    };

    auto fill_unknown = [](std::vector<int> &s) {
      int last = -1;
      for (int i = 0; i < int(s.size()); i++) {
        if (s[size_t(i)] >= 0) {
          last = s[size_t(i)];
        }
        else if (last >= 0) {
          s[size_t(i)] = last;
        }
      }
      last = -1;
      for (int i = int(s.size()) - 1; i >= 0; i--) {
        if (s[size_t(i)] >= 0) {
          last = s[size_t(i)];
        }
        else if (last >= 0) {
          s[size_t(i)] = last;
        }
      }
    };

    auto collect_edge_hits = [&](Surface_mesh::Halfedge_index he, int sa, int sb, int nsamp) {
      std::vector<Crossing> hits;
      if (he == Surface_mesh::null_halfedge() || sm.is_border(he)) {
        return hits;
      }
      std::vector<int> samples(size_t(nsamp + 1), -1);
      std::vector<double> ts(size_t(nsamp + 1), 0.0);
      samples[0] = sa;
      samples[size_t(nsamp)] = sb;
      ts[size_t(nsamp)] = 1.0;
      for (int i = 1; i < nsamp; i++) {
        ts[size_t(i)] = double(i) / double(nsamp);
        samples[size_t(i)] = query_he_t(he, ts[size_t(i)]);
      }
      fill_unknown(samples);
      /* Ignore 1-sample flicker (A B A) unless the endpoints themselves differ. */
      for (int i = 1; i < nsamp; i++) {
        if (samples[size_t(i)] < 0) {
          continue;
        }
        if (samples[size_t(i)] == samples[0] || samples[size_t(i)] == samples[size_t(nsamp)]) {
          continue;
        }
        const int prev = samples[size_t(i - 1)];
        const int next = samples[size_t(i + 1)];
        if (prev >= 0 && next >= 0 && prev == next && prev != samples[size_t(i)]) {
          samples[size_t(i)] = prev;
        }
      }
      for (int i = 0; i < nsamp; i++) {
        const int s0 = samples[size_t(i)];
        const int s1 = samples[size_t(i + 1)];
        if (s0 < 0 || s1 < 0 || s0 == s1) {
          continue;
        }
        hits.push_back(bisect_he(he, ts[size_t(i)], ts[size_t(i + 1)], s0, s1));
      }
      /* At most two geodesic crossings on one mesh edge. Keep first and last. */
      if (hits.size() > 2) {
        Crossing a = hits.front();
        Crossing b = hits.back();
        hits.clear();
        hits.push_back(a);
        if (std::abs(b.t - a.t) > 1e-4) {
          hits.push_back(b);
        }
      }
      return hits;
    };

    for (const auto e : sm.edges()) {
      if (sm.is_removed(e)) {
        continue;
      }
      auto he = sm.halfedge(e);
      if (sm.is_border(he)) {
        he = sm.opposite(he);
      }
      if (he == Surface_mesh::null_halfedge() || sm.is_border(he)) {
        continue;
      }
      const auto va = sm.source(he);
      const auto vb = sm.target(he);
      const int sa = cell.count(va) ? cell[va] : -1;
      const int sb = cell.count(vb) ? cell[vb] : -1;
      const int nsamp = (sa != sb && sa >= 0 && sb >= 0) ? 10 : 6;
      EdgeHits rec;
      rec.he_store = he;
      rec.hits = collect_edge_hits(he, sa, sb, nsamp);
      /* Endpoints disagree but samples stayed constant — still a bisector near one end. */
      if (rec.hits.empty() && sa >= 0 && sb >= 0 && sa != sb) {
        rec.hits.push_back(bisect_he(he, 0.0, 1.0, sa, sb));
      }
      if (!rec.hits.empty()) {
        edge_hits[e] = std::move(rec);
      }
    }

    auto hits_on_he = [&](Surface_mesh::Halfedge_index he) {
      std::vector<Crossing> out;
      if (he == Surface_mesh::null_halfedge()) {
        return out;
      }
      auto it = edge_hits.find(sm.edge(he));
      if (it == edge_hits.end()) {
        return out;
      }
      const bool flipped = (sm.source(he) != sm.source(it->second.he_store));
      for (const Crossing &c : it->second.hits) {
        Crossing d = c;
        if (flipped) {
          d.t = 1.0 - c.t;
          d.src0 = c.src1;
          d.src1 = c.src0;
        }
        out.push_back(d);
      }
      std::sort(out.begin(), out.end(), [](const Crossing &a, const Crossing &b) {
        return a.t < b.t;
      });
      return out;
    };

    auto fan_poly = [&](const std::vector<int> &poly_ids,
                        const std::vector<int> &poly_src,
                        int from,
                        int to,
                        int tag) {
      if (tag < 0 || poly_ids.empty()) {
        return false;
      }
      std::vector<int> chain;
      chain.push_back(poly_ids[size_t(from)]);
      const int n = int(poly_ids.size());
      int k = (from + 1) % n;
      while (k != to) {
        chain.push_back(poly_ids[size_t(k)]);
        k = (k + 1) % n;
      }
      chain.push_back(poly_ids[size_t(to)]);
      if (chain.size() < 3) {
        return false;
      }
      for (int t = 1; t + 1 < int(chain.size()); t++) {
        add_result_tri(result, chain[0], chain[size_t(t)], chain[size_t(t + 1)], tag);
      }
      return true;
    };

    auto split_two_crossings = [&](const std::vector<int> &poly_ids,
                                   const std::vector<int> &poly_src,
                                   const std::vector<int> &poly_after) -> bool {
      std::vector<int> cx;
      for (int i = 0; i < int(poly_ids.size()); i++) {
        if (poly_src[size_t(i)] == -2) {
          cx.push_back(i);
        }
      }
      if (cx.size() != 2) {
        return false;
      }
      auto tag_arc = [&](int from, int to) -> int {
        if (from >= 0 && from < int(poly_after.size()) && poly_after[size_t(from)] >= 0) {
          return poly_after[size_t(from)];
        }
        int tag = -1;
        const int n = int(poly_ids.size());
        int k = (from + 1) % n;
        while (k != to) {
          if (poly_src[size_t(k)] >= 0) {
            tag = poly_src[size_t(k)];
            break;
          }
          k = (k + 1) % n;
        }
        return tag;
      };
      int t01 = tag_arc(cx[0], cx[1]);
      int t10 = tag_arc(cx[1], cx[0]);
      if (t01 < 0) {
        t01 = t10;
      }
      if (t10 < 0) {
        t10 = t01;
      }
      if (t01 < 0 && t10 < 0) {
        return false;
      }
      return fan_poly(poly_ids, poly_src, cx[0], cx[1], t01) &&
             fan_poly(poly_ids, poly_src, cx[1], cx[0], t10);
    };

    auto emit_plain = [&](const std::array<Surface_mesh::Vertex_index, 3> &vv, int tag) {
      if (tag < 0) {
        return;
      }
      add_result_tri(result, emit_existing(vv[0]), emit_existing(vv[1]), emit_existing(vv[2]), tag);
    };

    auto majority3 = [](int a, int b, int c) {
      if (a >= 0 && a == b) {
        return a;
      }
      if (a >= 0 && a == c) {
        return a;
      }
      if (b >= 0 && b == c) {
        return b;
      }
      return std::max(a, std::max(b, c));
    };

    for (const auto f : sm.faces()) {
      if (sm.is_removed(f)) {
        continue;
      }
      const auto h0 = sm.halfedge(f);
      const std::array<Surface_mesh::Halfedge_index, 3> hes = {
          h0, sm.next(h0), sm.next(sm.next(h0))};
      const std::array<Surface_mesh::Vertex_index, 3> vv = {
          sm.source(hes[0]), sm.source(hes[1]), sm.source(hes[2])};
      int cells3[3];
      for (int i = 0; i < 3; i++) {
        cells3[i] = cell.count(vv[size_t(i)]) ? cell[vv[size_t(i)]] : -1;
      }
      std::array<std::vector<Crossing>, 3> hits;
      for (int i = 0; i < 3; i++) {
        hits[size_t(i)] = hits_on_he(hes[size_t(i)]);
      }

      /* Drop 1-edge double hits (usually MMP flicker). Keep one crossing per
       * edge unless the two are well separated and no other edges are hit. */
      int single_edges = 0;
      int double_edges = 0;
      for (int i = 0; i < 3; i++) {
        if (hits[size_t(i)].size() == 1) {
          single_edges++;
        }
        else if (hits[size_t(i)].size() >= 2) {
          double_edges++;
        }
      }
      if (double_edges > 0 && single_edges >= 1) {
        for (int i = 0; i < 3; i++) {
          if (hits[size_t(i)].size() >= 2) {
            hits[size_t(i)].clear();
          }
        }
      }
      else if (double_edges > 0 && single_edges == 0) {
        for (int i = 0; i < 3; i++) {
          hits[size_t(i)].clear();
        }
      }

      int n_hits = 0;
      int n_hit_edges = 0;
      for (int i = 0; i < 3; i++) {
        n_hits += int(hits[size_t(i)].size());
        if (!hits[size_t(i)].empty()) {
          n_hit_edges++;
        }
      }

      const int face_tag = majority3(cells3[0], cells3[1], cells3[2]);

      /* No bisector on the boundary: keep the input triangle. Do not densify. */
      if (n_hits == 0) {
        emit_plain(vv, face_tag);
        continue;
      }

      int ids[3] = {emit_existing(vv[0]), emit_existing(vv[1]), emit_existing(vv[2])};
      std::vector<int> poly_ids;
      std::vector<int> poly_src;
      std::vector<int> poly_after;
      poly_ids.reserve(3 + size_t(n_hits));
      poly_src.reserve(3 + size_t(n_hits));
      poly_after.reserve(3 + size_t(n_hits));
      for (int i = 0; i < 3; i++) {
        poly_ids.push_back(ids[i]);
        poly_src.push_back(cells3[i]);
        poly_after.push_back(-1);
        for (const Crossing &c : hits[size_t(i)]) {
          poly_ids.push_back(c.emit_id);
          poly_src.push_back(-2);
          poly_after.push_back(c.src1);
        }
      }

      if (n_hits == 2 && n_hit_edges == 2 && split_two_crossings(poly_ids, poly_src, poly_after)) {
        continue;
      }

      if (n_hits == 3 && n_hit_edges == 3 && hits[0].size() == 1 && hits[1].size() == 1 &&
          hits[2].size() == 1)
      {
        const Point_3 c0 = lerp_he(hes[0], hits[0][0].t);
        const Point_3 c1 = lerp_he(hes[1], hits[1][0].t);
        const Point_3 c2 = lerp_he(hes[2], hits[2][0].t);
        const Point_3 Q((c0.x() + c1.x() + c2.x()) / FT(3),
                        (c0.y() + c1.y() + c2.y()) / FT(3),
                        (c0.z() + c1.z() + c2.z()) / FT(3));
        const int qid = add_result_vert(result, Q);
        const int s01 = hits[0][0].emit_id;
        const int s12 = hits[1][0].emit_id;
        const int s20 = hits[2][0].emit_id;
        const int t0 = cells3[0] >= 0 ? cells3[0] : face_tag;
        const int t1 = cells3[1] >= 0 ? cells3[1] : face_tag;
        const int t2 = cells3[2] >= 0 ? cells3[2] : face_tag;
        if (t0 >= 0) {
          add_result_tri(result, ids[0], s01, qid, t0);
          add_result_tri(result, ids[0], qid, s20, t0);
        }
        if (t1 >= 0) {
          add_result_tri(result, ids[1], s12, qid, t1);
          add_result_tri(result, ids[1], qid, s01, t1);
        }
        if (t2 >= 0) {
          add_result_tri(result, ids[2], s20, qid, t2);
          add_result_tri(result, ids[2], qid, s12, t2);
        }
        continue;
      }

      /* Anything else: keep the original triangle instead of exploding it. */
      emit_plain(vv, face_tag);
    }

    result.ok = result.corners_num() >= 3;
    if (!result.ok) {
      result.error = "Geodesic Voronoi produced no faces";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Geodesic Voronoi failed";
  }
  return result;
}

MeshResult points_isosurface_3(const float *positions, const float *values, int n, double isolevel)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || !values || n < 4) {
    result.error = "Isosurface 3D needs at least 4 samples with values";
    return result;
  }
  try {
    using DT = CGAL::Delaunay_triangulation_3<Kernel>;
    DT dt;
    std::map<typename DT::Vertex_handle, double> val;
    int finite = 0;
    for (int i = 0; i < n; i++) {
      if (!std::isfinite(double(values[i]))) {
        continue;
      }
      auto v = dt.insert(xyz(double(positions[i * 3 + 0]),
                             double(positions[i * 3 + 1]),
                             double(positions[i * 3 + 2])));
      val[v] = double(values[i]);
      finite++;
    }
    if (finite < 4 || dt.dimension() != 3) {
      result.error = "Isosurface 3D: Delaunay tetrahedralization failed";
      return result;
    }

    auto lerp = [](const Point_3 &a, const Point_3 &b, double sa, double sb, double iso) {
      const double den = sb - sa;
      double t = (std::abs(den) < 1e-18) ? 0.5 : (iso - sa) / den;
      t = std::min(1.0 - 1e-6, std::max(1e-6, t));
      return Point_3(a.x() + FT(t) * (b.x() - a.x()),
                     a.y() + FT(t) * (b.y() - a.y()),
                     a.z() + FT(t) * (b.z() - a.z()));
    };

    auto emit_tri_pts = [&](Point_3 p0, Point_3 p1, Point_3 p2, const Point_3 &toward) {
      Vector_3 n = CGAL::cross_product(p1 - p0, p2 - p0);
      const Vector_3 to = toward - p0;
      if (CGAL::to_double(n * to) < 0.0) {
        std::swap(p1, p2);
      }
      const int a = add_result_vert(result, p0);
      const int b = add_result_vert(result, p1);
      const int c = add_result_vert(result, p2);
      add_result_tri(result, a, b, c, 0);
    };

    for (auto cit = dt.finite_cells_begin(); cit != dt.finite_cells_end(); ++cit) {
      Point_3 p[4];
      double s[4];
      int above[4];
      int na = 0;
      bool ok = true;
      for (int i = 0; i < 4; i++) {
        auto v = cit->vertex(i);
        auto it = val.find(v);
        if (it == val.end()) {
          ok = false;
          break;
        }
        p[i] = v->point();
        s[i] = it->second;
        if (s[i] >= isolevel) {
          above[na++] = i;
        }
      }
      if (!ok || na == 0 || na == 4) {
        continue;
      }
      if (na == 1 || na == 3) {
        const int hi = (na == 1) ? above[0] : -1;
        int lone = hi;
        if (na == 3) {
          bool used[4] = {false, false, false, false};
          for (int k = 0; k < 3; k++) {
            used[above[k]] = true;
          }
          for (int i = 0; i < 4; i++) {
            if (!used[i]) {
              lone = i;
              break;
            }
          }
        }
        Point_3 q[3];
        int t = 0;
        for (int i = 0; i < 4; i++) {
          if (i == lone) {
            continue;
          }
          q[t++] = lerp(p[lone], p[i], s[lone], s[i], isolevel);
        }
        Point_3 toward = p[lone];
        if (s[lone] < isolevel) {
          /* Face should still point toward the high side. */
          toward = Point_3(0, 0, 0);
          int c = 0;
          for (int i = 0; i < 4; i++) {
            if (s[i] >= isolevel) {
              toward = Point_3(toward.x() + p[i].x(), toward.y() + p[i].y(), toward.z() + p[i].z());
              c++;
            }
          }
          if (c > 0) {
            toward = Point_3(toward.x() / FT(c), toward.y() / FT(c), toward.z() / FT(c));
          }
        }
        emit_tri_pts(q[0], q[1], q[2], toward);
      }
      else if (na == 2) {
        const int a = above[0];
        const int b = above[1];
        int outv[2];
        int no = 0;
        for (int i = 0; i < 4; i++) {
          if (i != a && i != b) {
            outv[no++] = i;
          }
        }
        const int c = outv[0];
        const int d = outv[1];
        const Point_3 qac = lerp(p[a], p[c], s[a], s[c], isolevel);
        const Point_3 qad = lerp(p[a], p[d], s[a], s[d], isolevel);
        const Point_3 qbc = lerp(p[b], p[c], s[b], s[c], isolevel);
        const Point_3 qbd = lerp(p[b], p[d], s[b], s[d], isolevel);
        const Point_3 toward(
            (p[a].x() + p[b].x()) / FT(2), (p[a].y() + p[b].y()) / FT(2), (p[a].z() + p[b].z()) / FT(2));
        emit_tri_pts(qac, qad, qbc, toward);
        emit_tri_pts(qad, qbd, qbc, toward);
      }
    }
    result.ok = result.corners_num() >= 3;
    if (!result.ok) {
      result.error = "Isosurface 3D: no tet crossed the isolevel";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Isosurface 3D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
