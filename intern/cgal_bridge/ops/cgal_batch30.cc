/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 30:
 *  - S10 Convex Decomposition 3D (Nef + convex_decomposition_3)
 *  - Laplace Deform (cotangent Laplacian + handle constraints, Eigen)
 *  - M28 Surface Delaunay Remeshing (heavy)
 *
 * Skipped (this batch): R08 deprecated stub; G10 needs EPECK/GMP;
 * P22 Registration needs OpenGR/pointmatcher; S11/S12 user-deleted.
 */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Lazy_exact_nt.h>
#include <CGAL/MP_Float.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/surface_Delaunay_remeshing.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Quotient.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/boost/graph/helpers.h>
#include <Eigen/Dense>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace blender::cgal_bridge {
namespace {

namespace PMP = CGAL::Polygon_mesh_processing;


static double cotangent(const Point_3 &a, const Point_3 &b, const Point_3 &c)
{
  const Kernel::Vector_3 u = a - b;
  const Kernel::Vector_3 v = c - b;
  const double dot = CGAL::to_double(u * v);
  const Kernel::Vector_3 cross = CGAL::cross_product(u, v);
  const double area2 = std::sqrt(CGAL::to_double(cross.squared_length()));
  if (area2 < 1e-30) {
    return 0.0;
  }
  return dot / area2;
}

}  // namespace


static void append_sm_piece(MeshResult &result, const Surface_mesh &sm, int piece_id)
{
  std::map<Surface_mesh::Vertex_index, int> vmap;
  const int base = result.verts_num();
  int local = 0;
  for (const auto v : sm.vertices()) {
    vmap[v] = base + local++;
    const Point_3 &p = sm.point(v);
    result.positions.push_back(float(CGAL::to_double(p.x())));
    result.positions.push_back(float(CGAL::to_double(p.y())));
    result.positions.push_back(float(CGAL::to_double(p.z())));
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  for (const auto f : sm.faces()) {
    std::vector<int> ring;
    for (const auto v : vertices_around_face(sm.halfedge(f), sm)) {
      ring.push_back(vmap[v]);
    }
    if (ring.size() < 3) {
      continue;
    }
    for (const int vi : ring) {
      result.corner_verts.push_back(vi);
    }
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_tag.push_back(piece_id);
  }
}

static void append_tris_piece(MeshResult &result,
                              const std::vector<std::array<Point_3, 3>> &tris,
                              int piece_id)
{
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  for (const std::array<Point_3, 3> &t : tris) {
    const int base = result.verts_num();
    for (int i = 0; i < 3; i++) {
      result.positions.push_back(float(CGAL::to_double(t[i].x())));
      result.positions.push_back(float(CGAL::to_double(t[i].y())));
      result.positions.push_back(float(CGAL::to_double(t[i].z())));
    }
    result.corner_verts.push_back(base);
    result.corner_verts.push_back(base + 1);
    result.corner_verts.push_back(base + 2);
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_tag.push_back(piece_id);
  }
}

static bool hull_of_points(const std::vector<Point_3> &pts, Surface_mesh &hull)
{
  hull.clear();
  if (pts.size() < 4) {
    return false;
  }
  try {
    CGAL::convex_hull_3(pts.begin(), pts.end(), hull);
  }
  catch (...) {
    hull.clear();
    return false;
  }
  return hull.number_of_faces() > 0;
}

static std::vector<std::array<Point_3, 3>> mesh_in_to_tris(const MeshIn &mesh)
{
  std::vector<std::array<Point_3, 3>> tris;
  if (!mesh.positions || !mesh.corner_verts || mesh.faces_num <= 0) {
    return tris;
  }
  auto pt = [&](int v) -> Point_3 {
    return Point_3(double(mesh.positions[v * 3 + 0]),
                   double(mesh.positions[v * 3 + 1]),
                   double(mesh.positions[v * 3 + 2]));
  };
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
    if (end - start < 3) {
      continue;
    }
    const int v0 = mesh.corner_verts[start];
    if (v0 < 0 || v0 >= mesh.verts_num) {
      continue;
    }
    const Point_3 p0 = pt(v0);
    for (int c = start + 1; c + 1 < end; c++) {
      const int va = mesh.corner_verts[c];
      const int vb = mesh.corner_verts[c + 1];
      if (va < 0 || vb < 0 || va >= mesh.verts_num || vb >= mesh.verts_num) {
        continue;
      }
      tris.push_back({p0, pt(va), pt(vb)});
    }
  }
  return tris;
}

static std::vector<Point_3> unique_pts_from_tris(const std::vector<std::array<Point_3, 3>> &tris)
{
  std::vector<Point_3> pts;
  std::set<std::array<long long, 3>> seen;
  const double scale = 1e7;
  for (const std::array<Point_3, 3> &t : tris) {
    for (int i = 0; i < 3; i++) {
      const double x = CGAL::to_double(t[i].x());
      const double y = CGAL::to_double(t[i].y());
      const double z = CGAL::to_double(t[i].z());
      const std::array<long long, 3> key = {std::llround(x * scale),
                                            std::llround(y * scale),
                                            std::llround(z * scale)};
      if (seen.insert(key).second) {
        pts.push_back(t[i]);
      }
    }
  }
  return pts;
}

static double bbox_diag_pts(const std::vector<Point_3> &pts)
{
  if (pts.empty()) {
    return 1.0;
  }
  double mn[3] = {1e30, 1e30, 1e30};
  double mx[3] = {-1e30, -1e30, -1e30};
  for (const Point_3 &p : pts) {
    const double q[3] = {CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())};
    for (int k = 0; k < 3; k++) {
      mn[k] = std::min(mn[k], q[k]);
      mx[k] = std::max(mx[k], q[k]);
    }
  }
  const double dx = mx[0] - mn[0], dy = mx[1] - mn[1], dz = mx[2] - mn[2];
  return std::max(1e-12, std::sqrt(dx * dx + dy * dy + dz * dz));
}

static double plane_signed_dist(const Kernel::Plane_3 &pl, const Point_3 &p)
{
  const double a = CGAL::to_double(pl.a());
  const double b = CGAL::to_double(pl.b());
  const double c = CGAL::to_double(pl.c());
  const double d = CGAL::to_double(pl.d());
  const double n = std::sqrt(a * a + b * b + c * c);
  if (n < 1e-30) {
    return 0.0;
  }
  return (a * CGAL::to_double(p.x()) + b * CGAL::to_double(p.y()) + c * CGAL::to_double(p.z()) + d) /
         n;
}

static double dist2_point_triangle(double px, double py, double pz, const std::array<Point_3, 3> &t)
{
  const double ax = CGAL::to_double(t[0].x());
  const double ay = CGAL::to_double(t[0].y());
  const double az = CGAL::to_double(t[0].z());
  const double bx = CGAL::to_double(t[1].x());
  const double by = CGAL::to_double(t[1].y());
  const double bz = CGAL::to_double(t[1].z());
  const double cx = CGAL::to_double(t[2].x());
  const double cy = CGAL::to_double(t[2].y());
  const double cz = CGAL::to_double(t[2].z());

  const double abx = bx - ax, aby = by - ay, abz = bz - az;
  const double acx = cx - ax, acy = cy - ay, acz = cz - az;
  const double apx = px - ax, apy = py - ay, apz = pz - az;
  const double d1 = abx * apx + aby * apy + abz * apz;
  const double d2 = acx * apx + acy * apy + acz * apz;
  auto d2p = [&](double qx, double qy, double qz) {
    const double dx = px - qx, dy = py - qy, dz = pz - qz;
    return dx * dx + dy * dy + dz * dz;
  };
  if (d1 <= 0.0 && d2 <= 0.0) {
    return d2p(ax, ay, az);
  }

  const double bpx = px - bx, bpy = py - by, bpz = pz - bz;
  const double d3 = abx * bpx + aby * bpy + abz * bpz;
  const double d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0.0 && d4 <= d3) {
    return d2p(bx, by, bz);
  }

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return d2p(ax + v * abx, ay + v * aby, az + v * abz);
  }

  const double cpx = px - cx, cpy = py - cy, cpz = pz - cz;
  const double d5 = abx * cpx + aby * cpy + abz * cpz;
  const double d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0.0 && d5 <= d6) {
    return d2p(cx, cy, cz);
  }

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    return d2p(ax + w * acx, ay + w * acy, az + w * acz);
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return d2p(bx + w * (cx - bx), by + w * (cy - by), bz + w * (cz - bz));
  }

  const double denom = va + vb + vc;
  if (std::abs(denom) < 1e-30) {
    return d2p(ax, ay, az);
  }
  const double v = vb / denom;
  const double w = vc / denom;
  return d2p(ax + abx * v + acx * w, ay + aby * v + acy * w, az + abz * v + acz * w);
}

static double dist_point_tris(const Point_3 &p, const std::vector<std::array<Point_3, 3>> &tris)
{
  const double px = CGAL::to_double(p.x());
  const double py = CGAL::to_double(p.y());
  const double pz = CGAL::to_double(p.z());
  double best = 1e30;
  for (const std::array<Point_3, 3> &t : tris) {
    best = std::min(best, dist2_point_triangle(px, py, pz, t));
    if (best <= 1e-24) {
      return 0.0;
    }
  }
  return std::sqrt(std::max(0.0, best));
}

static Point_3 face_centroid_sm(const Surface_mesh &hull, Surface_mesh::Face_index f)
{
  double x = 0.0, y = 0.0, z = 0.0;
  int n = 0;
  for (const auto v : vertices_around_face(hull.halfedge(f), hull)) {
    const Point_3 &p = hull.point(v);
    x += CGAL::to_double(p.x());
    y += CGAL::to_double(p.y());
    z += CGAL::to_double(p.z());
    n++;
  }
  if (n == 0) {
    return Point_3(0, 0, 0);
  }
  const double inv = 1.0 / double(n);
  return Point_3(x * inv, y * inv, z * inv);
}

static Kernel::Plane_3 face_plane_sm(const Surface_mesh &hull, Surface_mesh::Face_index f)
{
  const auto h = hull.halfedge(f);
  return Kernel::Plane_3(hull.point(hull.source(h)),
                         hull.point(hull.target(h)),
                         hull.point(hull.target(hull.next(h))));
}

struct ConvexProbe {
  Surface_mesh hull;
  bool is_convex = true;
  bool has_cavity = false;
  Point_3 cavity_centroid = Point_3(0, 0, 0);
  Kernel::Vector_3 cavity_normal = Kernel::Vector_3(0, 0, 1);
};

/* A part is convex iff it equals its hull as a solid, not merely "all verts
 * lie on the hull". For a 2.5D L the notch verts are coplanar with the hull
 * top/bottom, so vertex-to-hull distance is ~0 even though the hull covers
 * the missing corner. Sample hull-face interiors against the triangle soup. */
static ConvexProbe probe_part_convex(const std::vector<std::array<Point_3, 3>> &tris,
                                     const std::vector<Point_3> &pts,
                                     double diag)
{
  ConvexProbe pr;
  if (!hull_of_points(pts, pr.hull) || pr.hull.number_of_faces() == 0) {
    pr.is_convex = true;
    return pr;
  }
  const double eps = std::max(1e-8, 1e-3 * diag);
  double worst = -1.0;
  for (const auto f : pr.hull.faces()) {
    std::vector<Point_3> ring;
    for (const auto v : vertices_around_face(pr.hull.halfedge(f), pr.hull)) {
      ring.push_back(pr.hull.point(v));
    }
    if (ring.size() < 3) {
      continue;
    }
    std::vector<Point_3> samples;
    samples.push_back(face_centroid_sm(pr.hull, f));
    /* A few interior samples so a centroid that happens to graze the mesh
     * still sees a cavity cover. */
    if (ring.size() >= 3) {
      const Point_3 &a = ring[0];
      const Point_3 &b = ring[1];
      const Point_3 &c = ring[2];
      samples.push_back(Point_3((CGAL::to_double(a.x()) + CGAL::to_double(b.x()) + CGAL::to_double(c.x())) / 3.0,
                                (CGAL::to_double(a.y()) + CGAL::to_double(b.y()) + CGAL::to_double(c.y())) / 3.0,
                                (CGAL::to_double(a.z()) + CGAL::to_double(b.z()) + CGAL::to_double(c.z())) / 3.0));
    }
    for (const Point_3 &s : samples) {
      const double d = dist_point_tris(s, tris);
      if (d > worst) {
        worst = d;
        pr.cavity_centroid = s;
        const Kernel::Plane_3 pl = face_plane_sm(pr.hull, f);
        Kernel::Vector_3 n(CGAL::to_double(pl.a()), CGAL::to_double(pl.b()), CGAL::to_double(pl.c()));
        if (n.squared_length() > 1e-30) {
          pr.cavity_normal = n;
        }
        pr.has_cavity = true;
      }
    }
  }
  if (worst > eps) {
    pr.is_convex = false;
    return pr;
  }
  pr.has_cavity = false;
  pr.is_convex = true;
  return pr;
}

static Point_3 lerp_plane(const Point_3 &a, const Point_3 &b, double da, double db)
{
  const double denom = da - db;
  double t = (std::abs(denom) < 1e-30) ? 0.5 : (da / denom);
  t = std::clamp(t, 0.0, 1.0);
  const double ax = CGAL::to_double(a.x()), ay = CGAL::to_double(a.y()), az = CGAL::to_double(a.z());
  const double bx = CGAL::to_double(b.x()), by = CGAL::to_double(b.y()), bz = CGAL::to_double(b.z());
  return Point_3(ax + t * (bx - ax), ay + t * (by - ay), az + t * (bz - az));
}

static void fan_poly(const std::vector<Point_3> &poly, std::vector<std::array<Point_3, 3>> &out)
{
  if (poly.size() < 3) {
    return;
  }
  for (size_t i = 1; i + 1 < poly.size(); i++) {
    out.push_back({poly[0], poly[i], poly[i + 1]});
  }
}

static void clip_triangle(const std::array<Point_3, 3> &tri,
                          const Kernel::Plane_3 &pl,
                          double eps,
                          std::vector<std::array<Point_3, 3>> &neg,
                          std::vector<std::array<Point_3, 3>> &pos,
                          std::vector<std::pair<Point_3, Point_3>> *cut_segs)
{
  const Point_3 p[3] = {tri[0], tri[1], tri[2]};
  double d[3];
  int s[3];
  for (int i = 0; i < 3; i++) {
    d[i] = plane_signed_dist(pl, p[i]);
    s[i] = (d[i] > eps) ? 1 : ((d[i] < -eps) ? -1 : 0);
  }
  if (s[0] <= 0 && s[1] <= 0 && s[2] <= 0) {
    neg.push_back(tri);
    return;
  }
  if (s[0] >= 0 && s[1] >= 0 && s[2] >= 0) {
    pos.push_back(tri);
    return;
  }
  std::vector<Point_3> npoly, ppoly, cut;
  npoly.reserve(4);
  ppoly.reserve(4);
  cut.reserve(3);
  for (int i = 0; i < 3; i++) {
    const int j = (i + 1) % 3;
    if (s[i] == 0) {
      cut.push_back(p[i]);
    }
    if (s[i] <= 0) {
      npoly.push_back(p[i]);
    }
    if (s[i] >= 0) {
      ppoly.push_back(p[i]);
    }
    if (s[i] * s[j] < 0) {
      const Point_3 ip = lerp_plane(p[i], p[j], d[i], d[j]);
      npoly.push_back(ip);
      ppoly.push_back(ip);
      cut.push_back(ip);
    }
  }
  fan_poly(npoly, neg);
  fan_poly(ppoly, pos);
  if (cut_segs && cut.size() >= 2) {
    cut_segs->push_back({cut[0], cut[1]});
    if (cut.size() >= 3) {
      cut_segs->push_back({cut[1], cut[2]});
    }
  }
}

static std::array<long long, 3> qkey3(const Point_3 &p)
{
  const double s = 1e7;
  return {std::llround(CGAL::to_double(p.x()) * s),
          std::llround(CGAL::to_double(p.y()) * s),
          std::llround(CGAL::to_double(p.z()) * s)};
}

/* Stitch plane-intersection segments into cap loops so clipped parts stay
 * closed solids. Without caps, hull faces on the cut look like cavities. */
static void cap_from_segments(const std::vector<std::pair<Point_3, Point_3>> &segs,
                              std::vector<std::array<Point_3, 3>> &caps)
{
  if (segs.empty()) {
    return;
  }
  using Key = std::array<long long, 3>;
  std::map<Key, Point_3> pts;
  std::map<Key, std::vector<Key>> adj;
  auto add_e = [&](const Point_3 &a, const Point_3 &b) {
    const Key ka = qkey3(a);
    const Key kb = qkey3(b);
    if (ka == kb) {
      return;
    }
    pts[ka] = a;
    pts[kb] = b;
    adj[ka].push_back(kb);
    adj[kb].push_back(ka);
  };
  for (const auto &s : segs) {
    add_e(s.first, s.second);
  }
  std::set<std::pair<Key, Key>> used;
  auto ek = [](const Key &a, const Key &b) {
    return (a < b) ? std::pair<Key, Key>{a, b} : std::pair<Key, Key>{b, a};
  };
  for (const auto &kv : adj) {
    for (const Key &nb0 : kv.second) {
      if (used.count(ek(kv.first, nb0))) {
        continue;
      }
      std::vector<Point_3> loop;
      Key prev = kv.first;
      Key cur = nb0;
      used.insert(ek(prev, cur));
      loop.push_back(pts[prev]);
      for (int guard = 0; guard < 64; guard++) {
        loop.push_back(pts[cur]);
        Key nxt = cur;
        bool found = false;
        for (const Key &nb : adj[cur]) {
          if (nb == prev) {
            continue;
          }
          if (used.count(ek(cur, nb))) {
            continue;
          }
          nxt = nb;
          found = true;
          break;
        }
        if (!found) {
          break;
        }
        used.insert(ek(cur, nxt));
        prev = cur;
        cur = nxt;
        if (cur == kv.first) {
          break;
        }
      }
      fan_poly(loop, caps);
    }
  }
}

static void clip_mesh(const std::vector<std::array<Point_3, 3>> &tris,
                      const Kernel::Plane_3 &plane,
                      double eps,
                      std::vector<std::array<Point_3, 3>> &L,
                      std::vector<std::array<Point_3, 3>> &R)
{
  std::vector<std::pair<Point_3, Point_3>> segs;
  L.reserve(tris.size() + 8);
  R.reserve(tris.size() + 8);
  for (const std::array<Point_3, 3> &t : tris) {
    clip_triangle(t, plane, eps, L, R, &segs);
  }
  std::vector<std::array<Point_3, 3>> caps;
  cap_from_segments(segs, caps);
  L.insert(L.end(), caps.begin(), caps.end());
  R.insert(R.end(), caps.begin(), caps.end());
}

static void bbox_of_pts(const std::vector<Point_3> &pts, double mn[3], double mx[3])
{
  mn[0] = mn[1] = mn[2] = 1e30;
  mx[0] = mx[1] = mx[2] = -1e30;
  for (const Point_3 &p : pts) {
    const double q[3] = {CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z())};
    for (int k = 0; k < 3; k++) {
      mn[k] = std::min(mn[k], q[k]);
      mx[k] = std::max(mx[k], q[k]);
    }
  }
}

static Kernel::Plane_3 aabb_cut_plane(const double mn[3], const double mx[3], int axis)
{
  const Point_3 origin(0.5 * (mn[0] + mx[0]), 0.5 * (mn[1] + mx[1]), 0.5 * (mn[2] + mx[2]));
  const Kernel::Vector_3 n(axis == 0 ? 1.0 : 0.0, axis == 1 ? 1.0 : 0.0, axis == 2 ? 1.0 : 0.0);
  return Kernel::Plane_3(origin, n);
}

static void collect_cut_planes(const std::vector<Point_3> &pts,
                               const ConvexProbe &probe,
                               std::vector<Kernel::Plane_3> &planes)
{
  double mn[3], mx[3];
  bbox_of_pts(pts, mn, mx);

  if (probe.has_cavity && probe.cavity_normal.squared_length() > 1e-30 && !pts.empty()) {
    /* Prefer a cut through the deepest dent: mesh point closest to the
     * cavity-cover sample, normal along that hull face. */
    size_t best_i = 0;
    double best_d = 1e30;
    const double cx = CGAL::to_double(probe.cavity_centroid.x());
    const double cy = CGAL::to_double(probe.cavity_centroid.y());
    const double cz = CGAL::to_double(probe.cavity_centroid.z());
    for (size_t i = 0; i < pts.size(); i++) {
      const double dx = CGAL::to_double(pts[i].x()) - cx;
      const double dy = CGAL::to_double(pts[i].y()) - cy;
      const double dz = CGAL::to_double(pts[i].z()) - cz;
      const double d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < best_d) {
        best_d = d2;
        best_i = i;
      }
    }
    planes.push_back(Kernel::Plane_3(pts[best_i], probe.cavity_normal));
  }

  int order[3] = {0, 1, 2};
  const double span[3] = {mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]};
  std::sort(order, order + 3, [&](int a, int b) { return span[a] > span[b]; });
  for (int k = 0; k < 3; k++) {
    if (span[order[k]] > 1e-12) {
      planes.push_back(aabb_cut_plane(mn, mx, order[k]));
    }
  }
}

static void acd_recurse_mesh(const std::vector<std::array<Point_3, 3>> &tris,
                             int depth,
                             int max_depth,
                             int max_pieces,
                             std::vector<Surface_mesh> &pieces,
                             std::vector<std::vector<std::array<Point_3, 3>>> &fallback)
{
  if (tris.empty()) {
    return;
  }
  if (int(pieces.size()) + int(fallback.size()) >= max_pieces) {
    /* Never drop leftover triangles. */
    fallback.push_back(tris);
    return;
  }
  const std::vector<Point_3> pts = unique_pts_from_tris(tris);
  const double diag = bbox_diag_pts(pts);
  ConvexProbe probe = probe_part_convex(tris, pts, diag);

  auto emit_hull = [&]() {
    Surface_mesh hull = std::move(probe.hull);
    if (hull.number_of_faces() == 0) {
      hull_of_points(pts, hull);
    }
    if (hull.number_of_faces() > 0) {
      pieces.push_back(std::move(hull));
    }
    else {
      fallback.push_back(tris);
    }
  };

  if (probe.is_convex) {
    emit_hull();
    return;
  }
  if (depth >= max_depth) {
    fallback.push_back(tris);
    return;
  }

  std::vector<Kernel::Plane_3> planes;
  collect_cut_planes(pts, probe, planes);
  const double eps = std::max(1e-10, 1e-6 * diag);

  std::vector<std::array<Point_3, 3>> best_L, best_R;
  bool have_split = false;

  for (const Kernel::Plane_3 &plane : planes) {
    std::vector<std::array<Point_3, 3>> L, R;
    clip_mesh(tris, plane, eps, L, R);
    if (L.empty() || R.empty()) {
      continue;
    }
    if (!have_split) {
      best_L = L;
      best_R = R;
      have_split = true;
    }
    const std::vector<Point_3> pL = unique_pts_from_tris(L);
    const std::vector<Point_3> pR = unique_pts_from_tris(R);
    if (pL.size() < 4 || pR.size() < 4) {
      continue;
    }
    ConvexProbe cL = probe_part_convex(L, pL, bbox_diag_pts(pL));
    ConvexProbe cR = probe_part_convex(R, pR, bbox_diag_pts(pR));
    if (cL.is_convex && cR.is_convex) {
      acd_recurse_mesh(L, depth + 1, max_depth, max_pieces, pieces, fallback);
      acd_recurse_mesh(R, depth + 1, max_depth, max_pieces, pieces, fallback);
      return;
    }
  }
  if (have_split) {
    acd_recurse_mesh(best_L, depth + 1, max_depth, max_pieces, pieces, fallback);
    acd_recurse_mesh(best_R, depth + 1, max_depth, max_pieces, pieces, fallback);
    return;
  }
  fallback.push_back(tris);
}

MeshResult mesh_convex_decomposition_3(const MeshIn &mesh)
{
  MeshResult result;
  if (mesh.faces_num <= 0 || mesh.verts_num < 4 || !mesh.positions) {
    result.error = "Convex Decomposition 3D needs a mesh";
    return result;
  }
  try {
    const std::vector<std::array<Point_3, 3>> tris = mesh_in_to_tris(mesh);
    if (tris.empty()) {
      result.error = "Convex Decomposition 3D needs faces";
      return result;
    }
    std::vector<Surface_mesh> pieces;
    std::vector<std::vector<std::array<Point_3, 3>>> fallback;
    /* Recursive plane clip of the actual triangles + convex hull of each part.
     * Do not bucket vertices: spanning faces keep their cut geometry. */
    acd_recurse_mesh(tris, 0, 8, 32, pieces, fallback);
    if (pieces.empty() && fallback.empty()) {
      Surface_mesh hull;
      const std::vector<Point_3> pts = unique_pts_from_tris(tris);
      if (hull_of_points(pts, hull)) {
        pieces.push_back(std::move(hull));
      }
      else {
        fallback.push_back(tris);
      }
    }
    result.face_offsets = {0};
    int piece = 0;
    for (const Surface_mesh &sm : pieces) {
      append_sm_piece(result, sm, piece++);
    }
    for (const auto &ft : fallback) {
      append_tris_piece(result, ft, piece++);
    }
    if (result.corner_verts.empty()) {
      result.error = "Convex Decomposition 3D produced no pieces";
      return result;
    }
    result.ok = true;
    result.alpha_used = double(piece);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Convex Decomposition 3D failed";
  }
  return result;
}


MeshResult mesh_laplace_deform(const MeshIn &mesh,
                               const uint8_t *roi_mask,
                               const uint8_t *control_mask,
                               const float *target_xyz,
                               double weight)
{
  MeshResult result;
  if (mesh.faces_num <= 0 || mesh.verts_num < 3 || !target_xyz || !control_mask) {
    result.error = "Laplace Deform needs a mesh, control mask, and targets";
    return result;
  }
  Surface_mesh sm;
  std::string error;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, false)) {
    result.error = error.empty() ? "Failed to import mesh" : error;
    return result;
  }
  try {
    PMP::triangulate_faces(sm);
    const int n = int(sm.number_of_vertices());
    if (n != mesh.verts_num) {
      result.error = "Laplace Deform vertex count mismatch after triangulate";
      return result;
    }

    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(size_t(n) * 8);
    std::vector<double> diag(size_t(n), 0.0);
    for (const auto f : sm.faces()) {
      Surface_mesh::Vertex_index v[3];
      int k = 0;
      for (const auto vv : vertices_around_face(sm.halfedge(f), sm)) {
        if (k < 3) {
          v[k++] = vv;
        }
      }
      if (k != 3) {
        continue;
      }
      const Point_3 p0 = sm.point(v[0]);
      const Point_3 p1 = sm.point(v[1]);
      const Point_3 p2 = sm.point(v[2]);
      const double c0 = 0.5 * cotangent(p1, p0, p2);
      const double c1 = 0.5 * cotangent(p2, p1, p0);
      const double c2 = 0.5 * cotangent(p0, p2, p1);
      const int i0 = int(v[0].idx());
      const int i1 = int(v[1].idx());
      const int i2 = int(v[2].idx());
      auto add_w = [&](int a, int b, double w) {
        if (!std::isfinite(w) || std::abs(w) < 1e-18) {
          return;
        }
        w = std::clamp(w, -1e3, 1e3);
        trips.emplace_back(a, b, -w);
        trips.emplace_back(b, a, -w);
        diag[size_t(a)] += w;
        diag[size_t(b)] += w;
      };
      add_w(i1, i2, c0);
      add_w(i2, i0, c1);
      add_w(i0, i1, c2);
    }
    for (int i = 0; i < n; i++) {
      trips.emplace_back(i, i, diag[size_t(i)] + 1e-12);
    }
    Eigen::SparseMatrix<double> L(n, n);
    L.setFromTriplets(trips.begin(), trips.end());

    Eigen::MatrixXd X0(n, 3);
    for (const auto v : sm.vertices()) {
      const int i = int(v.idx());
      const Point_3 &p = sm.point(v);
      X0(i, 0) = CGAL::to_double(p.x());
      X0(i, 1) = CGAL::to_double(p.y());
      X0(i, 2) = CGAL::to_double(p.z());
    }
    Eigen::MatrixXd Delta = L * X0;

    std::vector<char> is_fixed(size_t(n), 0);
    Eigen::MatrixXd X = X0;
    int control_count = 0;
    int free_count = 0;
    for (int i = 0; i < n; i++) {
      const bool control = control_mask[i] != 0;
      const bool roi = roi_mask ? (roi_mask[i] != 0) : true;
      if (control) {
        is_fixed[size_t(i)] = 1;
        X(i, 0) = double(target_xyz[i * 3 + 0]);
        X(i, 1) = double(target_xyz[i * 3 + 1]);
        X(i, 2) = double(target_xyz[i * 3 + 2]);
        control_count++;
      }
      else if (!roi) {
        is_fixed[size_t(i)] = 1;
      }
      else {
        free_count++;
      }
    }
    if (control_count == 0) {
      result.error = "Laplace Deform needs at least one control vertex";
      return result;
    }
    if (free_count == 0) {
      if (!surface_mesh_positions_to_buffer(sm, result.positions)) {
        result.error = "Failed to read positions";
        return result;
      }
      for (int i = 0; i < n; i++) {
        if (control_mask[i]) {
          result.positions[size_t(i) * 3 + 0] = target_xyz[i * 3 + 0];
          result.positions[size_t(i) * 3 + 1] = target_xyz[i * 3 + 1];
          result.positions[size_t(i) * 3 + 2] = target_xyz[i * 3 + 2];
        }
      }
      result.positions_only = true;
      result.ok = true;
      return result;
    }

    const double w_fix = std::max(1e-6, weight);

    auto factor_and_solve = [&](const Eigen::MatrixXd &delta, Eigen::MatrixXd &Xio) -> bool {
      std::vector<Eigen::Triplet<double>> AtA_trips;
      Eigen::SparseMatrix<double> LtL = L.transpose() * L;
      for (int k = 0; k < LtL.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator itL(LtL, k); itL; ++itL) {
          AtA_trips.emplace_back(int(itL.row()), int(itL.col()), itL.value());
        }
      }
      for (int i = 0; i < n; i++) {
        if (is_fixed[size_t(i)]) {
          AtA_trips.emplace_back(i, i, w_fix);
        }
      }
      Eigen::SparseMatrix<double> A(n, n);
      A.setFromTriplets(AtA_trips.begin(), AtA_trips.end());
      Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
      solver.compute(A);
      if (solver.info() != Eigen::Success) {
        result.error = "Laplace Deform factorization failed";
        return false;
      }
      for (int c = 0; c < 3; c++) {
        Eigen::VectorXd rhs = L.transpose() * delta.col(c);
        for (int i = 0; i < n; i++) {
          if (is_fixed[size_t(i)]) {
            rhs[i] += w_fix * Xio(i, c);
          }
        }
        Eigen::VectorXd sol = solver.solve(rhs);
        if (solver.info() != Eigen::Success) {
          result.error = "Laplace Deform solve failed";
          return false;
        }
        Xio.col(c) = sol;
      }
      for (int i = 0; i < n; i++) {
        if (control_mask[i]) {
          Xio(i, 0) = double(target_xyz[i * 3 + 0]);
          Xio(i, 1) = double(target_xyz[i * 3 + 1]);
          Xio(i, 2) = double(target_xyz[i * 3 + 2]);
        }
        else if (!roi_mask || !roi_mask[i]) {
          Xio(i, 0) = X0(i, 0);
          Xio(i, 1) = X0(i, 1);
          Xio(i, 2) = X0(i, 2);
        }
      }
      return true;
    };

    /* 1) Position least squares (pin handles, preserve Laplacian coords). */
    if (!factor_and_solve(Delta, X)) {
      return result;
    }

    /* 2) One rotation-compensation LS: Kabsch on each 1-ring, rotate deltas, re-solve. */
    Eigen::MatrixXd DeltaR = Delta;
    for (const auto v : sm.vertices()) {
      const int i = int(v.idx());
      std::vector<Eigen::Vector3d> P, Q;
      const Eigen::Vector3d pi(X0(i, 0), X0(i, 1), X0(i, 2));
      const Eigen::Vector3d qi(X(i, 0), X(i, 1), X(i, 2));
      for (const auto nb : vertices_around_target(sm.halfedge(v), sm)) {
        const int j = int(nb.idx());
        P.emplace_back(X0(j, 0) - pi[0], X0(j, 1) - pi[1], X0(j, 2) - pi[2]);
        Q.emplace_back(X(j, 0) - qi[0], X(j, 1) - qi[1], X(j, 2) - qi[2]);
      }
      if (P.size() < 2) {
        continue;
      }
      Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
      for (size_t k = 0; k < P.size(); k++) {
        H += P[k] * Q[k].transpose();
      }
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3d U = svd.matrixU();
      Eigen::Matrix3d V = svd.matrixV();
      Eigen::Matrix3d R = V * U.transpose();
      if (R.determinant() < 0.0) {
        V.col(2) *= -1.0;
        R = V * U.transpose();
      }
      const Eigen::Vector3d d(Delta(i, 0), Delta(i, 1), Delta(i, 2));
      const Eigen::Vector3d rd = R * d;
      DeltaR(i, 0) = rd[0];
      DeltaR(i, 1) = rd[1];
      DeltaR(i, 2) = rd[2];
    }
    if (!factor_and_solve(DeltaR, X)) {
      return result;
    }

    for (const auto v : sm.vertices()) {
      const int i = int(v.idx());
      sm.point(v) = Point_3(X(i, 0), X(i, 1), X(i, 2));
    }
    if (!surface_mesh_positions_to_buffer(sm, result.positions)) {
      result.error = "Failed to read deformed positions";
      return result;
    }
    result.positions_only = true;
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Laplace Deform failed";
  }
  return result;
}

MeshResult mesh_surface_delaunay_remesh(const MeshIn &mesh,
                                        double facet_size,
                                        double facet_angle,
                                        double facet_distance,
                                        double features_angle_bound,
                                        bool protect_constraints)
{
  MeshResult result;
  if (mesh.faces_num <= 0 || mesh.verts_num < 3) {
    result.error = "Surface Delaunay Remesh needs a triangle mesh";
    return result;
  }
  Surface_mesh sm;
  std::string error;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, false)) {
    result.error = error.empty() ? "Failed to import mesh" : error;
    return result;
  }
  try {
    PMP::triangulate_faces(sm);
    if (sm.number_of_faces() == 0) {
      result.error = "Empty mesh";
      return result;
    }
    facet_size = std::max(0.0, facet_size);
    facet_angle = std::clamp(facet_angle, 0.0, 30.0);
    facet_distance = std::max(0.0, facet_distance);
    features_angle_bound = std::clamp(features_angle_bound, 0.0, 180.0);

    Surface_mesh out = PMP::surface_Delaunay_remeshing(
        sm,
        CGAL::parameters::mesh_facet_size(facet_size)
            .mesh_facet_angle(facet_angle)
            .mesh_facet_distance(facet_distance)
            .protect_constraints(protect_constraints)
            .features_angle_bound(features_angle_bound));

    if (out.number_of_faces() == 0) {
      result.error = "Surface Delaunay Remesh produced empty mesh";
      return result;
    }
    result = surface_mesh_to_triangles(out);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Surface Delaunay Remesh failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
