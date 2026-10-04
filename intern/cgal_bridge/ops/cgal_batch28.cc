/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 28 — new only (not previously deleted / already existing):
 *  - Min Annulus 2D
 *  - Convex Partition 2D (approx / greene / optimal)
 *  - Y-Monotone Partition 2D
 *  - Max Area K-gon 2D
 *  - Polyline Simplify 2D
 *  - Width 3D
 *  - Minkowski Sum 2D
 *  - Extrude Skeleton (roof)
 *  - Polygon Fill 2D (CDT with holes)
 *  - Do Intersect 2D
 */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Min_annulus_d.h>
#include <CGAL/Min_sphere_annulus_d_traits_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Multipolygon_with_holes_2.h>
#include <CGAL/Polyline_simplification_2/Squared_distance_cost.h>
#include <CGAL/Polyline_simplification_2/Stop_above_cost_threshold.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/Width_3.h>
#include <CGAL/Width_default_traits_3.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>
#include <CGAL/extremal_polygon_2.h>
#include <CGAL/extrude_skeleton.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/minkowski_sum_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/Polygon_repair/repair.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Quotient.h>
#include <CGAL/Lazy_exact_nt.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/assertions_behaviour.h>
#include <CGAL/exceptions.h>

#include <boost/property_map/property_map.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <vector>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace PMP = CGAL::Polygon_mesh_processing;
namespace PS = CGAL::Polyline_simplification_2;

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



static double loop_bbox_diag(const std::vector<Point_2> &pts)
{
  if (pts.empty()) {
    return 1.0;
  }
  double minx = 1e30, miny = 1e30, maxx = -1e30, maxy = -1e30;
  for (const Point_2 &p : pts) {
    const double x = CGAL::to_double(p.x());
    const double y = CGAL::to_double(p.y());
    minx = std::min(minx, x);
    miny = std::min(miny, y);
    maxx = std::max(maxx, x);
    maxy = std::max(maxy, y);
  }
  return std::max(1e-12, std::hypot(maxx - minx, maxy - miny));
}

static std::vector<Point_2> simplify_collinear_loop(const std::vector<Point_2> &loop)
{
  std::vector<Point_2> pts = clean_loop_2(loop);
  if (pts.size() < 4) {
    return pts;
  }
  /* Drop near-collinear verts so Hertel-Mehlhorn is not stuck triangulating a
   * densely sampled outline. 1 degree, or height relative to bbox diagonal. */
  const double diag = loop_bbox_diag(pts);
  const double sin_eps = std::sin(1.0 * M_PI / 180.0);
  const double height_eps = 1e-3 * diag;
  for (int pass = 0; pass < 32; pass++) {
    const int n = int(pts.size());
    if (n < 4) {
      break;
    }
    std::vector<Point_2> next;
    next.reserve(size_t(n));
    int dropped = 0;
    for (int i = 0; i < n; i++) {
      const Point_2 &a = pts[(i + n - 1) % n];
      const Point_2 &b = pts[i];
      const Point_2 &c = pts[(i + 1) % n];
      const double ax = CGAL::to_double(a.x());
      const double ay = CGAL::to_double(a.y());
      const double bx = CGAL::to_double(b.x());
      const double by = CGAL::to_double(b.y());
      const double cx = CGAL::to_double(c.x());
      const double cy = CGAL::to_double(c.y());
      const double abx = bx - ax, aby = by - ay;
      const double bcx = cx - bx, bcy = cy - by;
      const double acx = cx - ax, acy = cy - ay;
      const double cross = abx * bcy - aby * bcx;
      const double lab2 = abx * abx + aby * aby;
      const double lbc2 = bcx * bcx + bcy * bcy;
      const double lac = std::sqrt(std::max(0.0, acx * acx + acy * acy));
      const double denom = std::sqrt(std::max(0.0, lab2 * lbc2));
      const double cross_ac = abx * acy - aby * acx;
      const double dist_ac = (lac > 1e-30) ? std::abs(cross_ac) / lac : 0.0;
      const bool collinear_ang = (denom > 1e-30) && (std::abs(cross) / denom < sin_eps);
      const bool collinear_h = dist_ac <= height_eps;
      if (collinear_ang || collinear_h) {
        dropped++;
        continue;
      }
      next.push_back(b);
    }
    if (next.size() < 3) {
      break;
    }
    pts.swap(next);
    if (dropped == 0) {
      break;
    }
  }
  return pts;
}

static bool same_pt_2(const Point_2 &a, const Point_2 &b, double eps2)
{
  const double dx = CGAL::to_double(a.x() - b.x());
  const double dy = CGAL::to_double(a.y() - b.y());
  return dx * dx + dy * dy <= eps2;
}

static bool ring_is_convex_ccw(const std::vector<Point_2> &ring)
{
  const int n = int(ring.size());
  if (n < 3) {
    return false;
  }
  int sign = 0;
  for (int i = 0; i < n; i++) {
    const Point_2 &a = ring[i];
    const Point_2 &b = ring[(i + 1) % n];
    const Point_2 &c = ring[(i + 2) % n];
    const double ax = CGAL::to_double(a.x());
    const double ay = CGAL::to_double(a.y());
    const double bx = CGAL::to_double(b.x());
    const double by = CGAL::to_double(b.y());
    const double cx = CGAL::to_double(c.x());
    const double cy = CGAL::to_double(c.y());
    const double cross = (bx - ax) * (cy - by) - (by - ay) * (cx - bx);
    if (std::abs(cross) < 1e-18) {
      continue;
    }
    const int s = (cross > 0.0) ? 1 : -1;
    if (sign == 0) {
      sign = s;
    }
    else if (s != sign) {
      return false;
    }
  }
  return sign >= 0;
}

static bool try_merge_convex_rings(const std::vector<Point_2> &A,
                                   const std::vector<Point_2> &B,
                                   double eps2,
                                   std::vector<Point_2> &out)
{
  const int na = int(A.size());
  const int nb = int(B.size());
  if (na < 3 || nb < 3) {
    return false;
  }
  for (int i = 0; i < na; i++) {
    const Point_2 &s0 = A[i];
    const Point_2 &s1 = A[(i + 1) % na];
    for (int j = 0; j < nb; j++) {
      const Point_2 &t0 = B[j];
      const Point_2 &t1 = B[(j + 1) % nb];
      if (!same_pt_2(s0, t1, eps2) || !same_pt_2(s1, t0, eps2)) {
        continue;
      }
      std::vector<Point_2> u;
      u.reserve(size_t(na + nb - 2));
      for (int k = 1; k < na; k++) {
        u.push_back(A[(i + 1 + k) % na]);
      }
      for (int k = 1; k < nb; k++) {
        u.push_back(B[(j + 1 + k) % nb]);
      }
      u = simplify_collinear_loop(u);
      if (u.size() >= 3 && ring_is_convex_ccw(u)) {
        out.swap(u);
        return true;
      }
    }
  }
  return false;
}

static void merge_convex_piece_list(std::vector<std::vector<Point_2>> &pieces, double eps2)
{
  bool progressed = true;
  int guard = 0;
  while (progressed && guard++ < 256) {
    progressed = false;
    for (size_t i = 0; i < pieces.size(); i++) {
      for (size_t j = i + 1; j < pieces.size(); j++) {
        std::vector<Point_2> u;
        if (try_merge_convex_rings(pieces[i], pieces[j], eps2, u)) {
          pieces[i].swap(u);
          pieces.erase(pieces.begin() + int(j));
          progressed = true;
          break;
        }
      }
      if (progressed) {
        break;
      }
    }
  }
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
                                    std::string &error,
                                    const bool allow_point_hull_fallback = true,
                                    const bool allow_face_fallback = true)
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

  if (islands.empty() && allow_face_fallback) {
    /* Also try each mesh face as its own XY polygon (keeps concave n-gons). */
    if (mesh.faces_num > 0 && mesh.face_offsets && mesh.corner_verts) {
      for (int f = 0; f < mesh.faces_num; f++) {
        const int start = mesh.face_offsets[f];
        const int end = mesh.face_offsets[f + 1];
        if (end - start < 3) {
          continue;
        }
        std::vector<Point_2> raw;
        raw.reserve(size_t(end - start));
        for (int c = start; c < end; c++) {
          const int v = mesh.corner_verts[c];
          if (v < 0 || v >= mesh.verts_num) {
            raw.clear();
            break;
          }
          raw.push_back(project_pt(mesh.positions + v * 3, plane));
        }
        if (raw.size() < 3) {
          continue;
        }
        Loop L = make_loop(std::move(raw));
        if (L.verts.size() < 3 || L.area_abs <= 1e-20) {
          continue;
        }
        PolyIsland isl;
        isl.outer = std::move(L.verts);
        if (signed_area_2(isl.outer) < 0.0) {
          std::reverse(isl.outer.begin(), isl.outer.end());
        }
        islands.push_back(std::move(isl));
      }
    }
  }

  if (islands.empty()) {
    if (!allow_point_hull_fallback) {
      error = load_err.empty() ?
                  "No polygon border found (refusing point-set convex-hull fallback)" :
                  load_err;
      return false;
    }
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
      vhs.push_back(cdt.insert(*vit));
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
        cdt.insert_constraint(vhs[size_t(i)], vhs[size_t((i + 1) % n)]);
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

static Polygon_2 force_ccw_simple(Polygon_2 poly)
{
  if (poly.size() >= 3 && poly.is_clockwise_oriented()) {
    poly.reverse_orientation();
  }
  return poly;
}

static bool segments_intersect_proper(const Point_2 &a,
                                      const Point_2 &b,
                                      const Point_2 &c,
                                      const Point_2 &d)
{
  const auto o1 = CGAL::orientation(a, b, c);
  const auto o2 = CGAL::orientation(a, b, d);
  const auto o3 = CGAL::orientation(c, d, a);
  const auto o4 = CGAL::orientation(c, d, b);
  return (o1 != o2) && (o3 != o4);
}

static bool polygons_do_intersect(const Polygon_2 &a, const Polygon_2 &b)
{
  if (a.size() < 3 || b.size() < 3) {
    return false;
  }
  const int na = int(a.size());
  const int nb = int(b.size());
  for (int i = 0; i < na; i++) {
    for (int j = 0; j < nb; j++) {
      if (segments_intersect_proper(
              a[i], a[(i + 1) % na], b[j], b[(j + 1) % nb]))
      {
        return true;
      }
    }
  }
  /* Containment: a vertex of one inside the other. */
  std::vector<Point_2> va(a.vertices_begin(), a.vertices_end());
  std::vector<Point_2> vb(b.vertices_begin(), b.vertices_end());
  if (point_in_poly(a[0], vb) || point_in_poly(b[0], va)) {
    return true;
  }
  return false;
}

}  // namespace

/* -------------------------------------------------------------------------- */
/* 1. Min Annulus 2D                                                          */
/* -------------------------------------------------------------------------- */

MeshResult points_min_annulus_2(const float *positions, int n, int plane, int segments)
{
  MeshResult result;
  if (!positions || n < 3) {
    result.error = "Min Annulus 2D needs at least 3 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  segments = std::clamp(segments, 8, 256);
  try {
    using Traits = CGAL::Min_sphere_annulus_d_traits_2<Kernel>;
    using Min_annulus = CGAL::Min_annulus_d<Traits>;
    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.push_back(project_pt(positions + i * 3, plane));
    }
    const float mid = mid_dropped_axis(positions, n, plane);
    Min_annulus ma(pts.begin(), pts.end());
    if (ma.is_empty()) {
      result.error = "Min annulus empty / degenerate";
      return result;
    }
    const Point_2 center_pt = ma.center();
    const double c2[2] = {CGAL::to_double(center_pt.x()), CGAL::to_double(center_pt.y())};
    const double r_out = std::sqrt(std::max(0.0, CGAL::to_double(ma.squared_outer_radius())));
    const double r_in = std::sqrt(std::max(0.0, CGAL::to_double(ma.squared_inner_radius())));
    unproject(c2[0], c2[1], plane, mid, result.center);
    result.radius = float(r_out);
    /* Store inner radius in alpha_used (reused diagnostic field). */
    result.alpha_used = r_in;

    /* Annulus = ring of quads (not triangulated). */
    result.face_offsets = {0};
    for (int i = 0; i < segments; i++) {
      const double ang = 2.0 * M_PI * double(i) / double(segments);
      const double ca = std::cos(ang);
      const double sa = std::sin(ang);
      float po[3], pi[3];
      unproject(c2[0] + r_out * ca, c2[1] + r_out * sa, plane, mid, po);
      unproject(c2[0] + r_in * ca, c2[1] + r_in * sa, plane, mid, pi);
      result.positions.push_back(po[0]);
      result.positions.push_back(po[1]);
      result.positions.push_back(po[2]);
      result.positions.push_back(pi[0]);
      result.positions.push_back(pi[1]);
      result.positions.push_back(pi[2]);
    }
    for (int i = 0; i < segments; i++) {
      const int o0 = i * 2;
      const int i0 = o0 + 1;
      const int o1 = ((i + 1) % segments) * 2;
      const int i1 = o1 + 1;
      result.corner_verts.push_back(o0);
      result.corner_verts.push_back(o1);
      result.corner_verts.push_back(i1);
      result.corner_verts.push_back(i0);
      result.face_offsets.push_back(int(result.corner_verts.size()));
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Min Annulus 2D failed";
  }
  return result;
}


static bool union_projected_faces(const MeshIn &mesh,
                                  int plane,
                                  std::vector<PolyIsland> &islands,
                                  float &mid_drop,
                                  std::string &error)
{
  islands.clear();
  if (!mesh.positions || mesh.faces_num <= 0 || !mesh.face_offsets || !mesh.corner_verts) {
    error = "No faces to union";
    return false;
  }
  mid_drop = mid_dropped_axis(mesh.positions, mesh.verts_num, plane);

  /* EPICK Polygon_set_2 asserts and aborts on typical meshes. Use exact NT. */
  using EFT = CGAL::Lazy_exact_nt<CGAL::Quotient<CGAL::MP_Float>>;
  using EKernel = CGAL::Simple_cartesian<EFT>;
  using EPoint_2 = EKernel::Point_2;
  using EPolygon_2 = CGAL::Polygon_2<EKernel>;
  using EPolygon_with_holes_2 = CGAL::Polygon_with_holes_2<EKernel>;
  using EPolygon_set_2 = CGAL::Polygon_set_2<EKernel>;

  const CGAL::Failure_behaviour old_err = CGAL::set_error_behaviour(CGAL::THROW_EXCEPTION);
  const CGAL::Failure_behaviour old_warn = CGAL::set_warning_behaviour(CGAL::CONTINUE);

  auto restore = [&]() {
    CGAL::set_error_behaviour(old_err);
    CGAL::set_warning_behaviour(old_warn);
  };

  try {
    EPolygon_set_2 ps;
    int joined = 0;
    for (int f = 0; f < mesh.faces_num; f++) {
      const int start = mesh.face_offsets[f];
      const int end = mesh.face_offsets[f + 1];
      if (end - start < 3) {
        continue;
      }
      std::vector<EPoint_2> raw;
      raw.reserve(size_t(end - start));
      bool bad = false;
      for (int c = start; c < end; c++) {
        const int v = mesh.corner_verts[c];
        if (v < 0 || v >= mesh.verts_num) {
          bad = true;
          break;
        }
        const Point_2 q = project_pt(mesh.positions + v * 3, plane);
        raw.emplace_back(EFT(CGAL::to_double(q.x())), EFT(CGAL::to_double(q.y())));
      }
      if (bad || raw.size() < 3) {
        continue;
      }
      EPolygon_2 poly(raw.begin(), raw.end());
      if (poly.is_clockwise_oriented()) {
        poly.reverse_orientation();
      }
      try {
        if (poly.is_simple() && poly.size() >= 3) {
          ps.join(poly);
          joined++;
        }
      }
      catch (...) {
        continue;
      }
    }
    if (joined == 0) {
      restore();
      error = "No projected faces";
      return false;
    }
    std::vector<EPolygon_with_holes_2> repaired;
    ps.polygons_with_holes(std::back_inserter(repaired));
    restore();

    auto to_epick = [](const EPoint_2 &p) {
      return Point_2(CGAL::to_double(p.x()), CGAL::to_double(p.y()));
    };
    for (const EPolygon_with_holes_2 &pwh : repaired) {
      const EPolygon_2 &outer = pwh.outer_boundary();
      if (outer.size() < 3) {
        continue;
      }
      PolyIsland isl;
      isl.outer.reserve(outer.size());
      for (auto v = outer.vertices_begin(); v != outer.vertices_end(); ++v) {
        isl.outer.push_back(to_epick(*v));
      }
      if (signed_area_2(isl.outer) < 0.0) {
        std::reverse(isl.outer.begin(), isl.outer.end());
      }
      for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        std::vector<Point_2> hole;
        for (auto v = hit->vertices_begin(); v != hit->vertices_end(); ++v) {
          hole.push_back(to_epick(*v));
        }
        if (hole.size() < 3) {
          continue;
        }
        if (signed_area_2(hole) > 0.0) {
          std::reverse(hole.begin(), hole.end());
        }
        isl.holes.push_back(std::move(hole));
      }
      islands.push_back(std::move(isl));
    }
  }
  catch (const std::exception &e) {
    restore();
    error = e.what();
    return false;
  }
  catch (...) {
    restore();
    error = "XY boolean union failed";
    return false;
  }
  if (islands.empty()) {
    error = "XY union produced no polygons";
    return false;
  }
  return true;
}

/* -------------------------------------------------------------------------- */
/* 2. Convex Partition 2D                                                     */
/* -------------------------------------------------------------------------- */


MeshResult mesh_convex_partition_2(const MeshIn &mesh, int method)
{
  MeshResult result;
  const int plane = 0;
  method = std::clamp(method, 0, 2);
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    const CGAL::Failure_behaviour old_err = CGAL::set_error_behaviour(CGAL::THROW_EXCEPTION);
    const CGAL::Failure_behaviour old_warn = CGAL::set_warning_behaviour(CGAL::CONTINUE);

    /* Always union projected faces into one (multi)polygon so a triangulated L
     * becomes a single outline, not one island per triangle. */
    bool got = false;
    {
      std::vector<PolyIsland> uni;
      std::string uerr;
      float umid = 0.0f;
      if (union_projected_faces(mesh, plane, uni, umid, uerr) && !uni.empty()) {
        islands.swap(uni);
        mid = umid;
        got = true;
      }
      else {
        error = uerr;
      }
    }
    if (!got) {
      got = extract_polygon_islands(mesh, plane, islands, mid, error, false, false);
    }
    if (!got || islands.empty()) {
      CGAL::set_error_behaviour(old_err);
      CGAL::set_warning_behaviour(old_warn);
      result.error = error.empty() ? "Convex Partition 2D needs a polygon" : error;
      return result;
    }

    result.face_offsets = {0};
    using Traits = CGAL::Partition_traits_2<Kernel>;
    using Partition_poly = Traits::Polygon_2;

    auto emit_pieces = [&](std::vector<std::vector<Point_2>> &pieces, double diag) {
      const double eps2 = std::max(1e-16, (1e-4 * diag) * (1e-4 * diag));
      for (std::vector<Point_2> &ring : pieces) {
        ring = simplify_collinear_loop(ring);
      }
      pieces.erase(std::remove_if(pieces.begin(),
                                  pieces.end(),
                                  [](const std::vector<Point_2> &r) { return r.size() < 3; }),
                   pieces.end());
      merge_convex_piece_list(pieces, eps2);
      for (const std::vector<Point_2> &ring : pieces) {
        append_ngon_face(result, ring, plane, mid);
        result.face_tag.push_back(int(result.face_tag.size()));
      }
    };

    auto partition_simple_ring = [&](std::vector<Point_2> raw) {
      raw = simplify_collinear_loop(raw);
      if (raw.size() < 3) {
        return;
      }
      Polygon_2 outer(raw.begin(), raw.end());
      outer = force_ccw_simple(outer);
      if (!outer.is_simple() || outer.size() < 3) {
        Multipolygon_with_holes_2 multi = CGAL::Polygon_repair::repair(outer);
        for (const Polygon_with_holes_2 &pwh : multi.polygons_with_holes()) {
          if (pwh.number_of_holes() != 0) {
            continue;
          }
          std::vector<Point_2> o(pwh.outer_boundary().vertices_begin(),
                                 pwh.outer_boundary().vertices_end());
          o = simplify_collinear_loop(o);
          if (o.size() < 3) {
            continue;
          }
          Polygon_2 po(o.begin(), o.end());
          po = force_ccw_simple(po);
          if (!po.is_simple() || po.size() < 3) {
            continue;
          }
          Partition_poly poly(po.vertices_begin(), po.vertices_end());
          std::list<Partition_poly> parts;
          int use = method;
          if (use == 2 && int(po.size()) > 24) {
            use = 0;
          }
          try {
            if (use == 1) {
              CGAL::greene_approx_convex_partition_2(
                  poly.vertices_begin(), poly.vertices_end(), std::back_inserter(parts));
            }
            else if (use == 2) {
              CGAL::optimal_convex_partition_2(
                  poly.vertices_begin(), poly.vertices_end(), std::back_inserter(parts));
            }
            else {
              CGAL::approx_convex_partition_2(
                  poly.vertices_begin(), poly.vertices_end(), std::back_inserter(parts));
            }
          }
          catch (...) {
            parts.clear();
          }
          std::vector<std::vector<Point_2>> pieces;
          if (parts.empty()) {
            pieces.push_back(o);
          }
          else {
            for (const Partition_poly &pp : parts) {
              pieces.emplace_back(pp.vertices_begin(), pp.vertices_end());
            }
          }
          emit_pieces(pieces, loop_bbox_diag(o));
        }
        return;
      }
      Partition_poly poly(outer.vertices_begin(), outer.vertices_end());
      std::list<Partition_poly> parts;
      int use = method;
      if (use == 2 && int(outer.size()) > 24) {
        use = 0;
      }
      try {
        if (use == 1) {
          CGAL::greene_approx_convex_partition_2(
              poly.vertices_begin(), poly.vertices_end(), std::back_inserter(parts));
        }
        else if (use == 2) {
          CGAL::optimal_convex_partition_2(
              poly.vertices_begin(), poly.vertices_end(), std::back_inserter(parts));
        }
        else {
          CGAL::approx_convex_partition_2(
              poly.vertices_begin(), poly.vertices_end(), std::back_inserter(parts));
        }
      }
      catch (...) {
        parts.clear();
      }
      std::vector<std::vector<Point_2>> pieces;
      if (parts.empty()) {
        pieces.push_back(raw);
      }
      else {
        for (const Partition_poly &pp : parts) {
          pieces.emplace_back(pp.vertices_begin(), pp.vertices_end());
        }
      }
      emit_pieces(pieces, loop_bbox_diag(raw));
    };

    auto partition_pwh_cdt = [&](const Polygon_with_holes_2 &pwh) {
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
          vhs.push_back(cdt.insert(*vit));
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
            cdt.insert_constraint(vhs[size_t(i)], vhs[size_t((i + 1) % n)]);
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
      std::vector<std::vector<Point_2>> pieces;
      for (typename CDT::Finite_faces_iterator fit = cdt.finite_faces_begin();
           fit != cdt.finite_faces_end();
           ++fit)
      {
        typename CDT::Face_handle fh = fit;
        if (!get(in_domain, fh)) {
          continue;
        }
        std::vector<Point_2> ring = {fh->vertex(0)->point(),
                                     fh->vertex(1)->point(),
                                     fh->vertex(2)->point()};
        if (signed_area_2(ring) < 0.0) {
          std::reverse(ring.begin(), ring.end());
        }
        pieces.push_back(std::move(ring));
      }
      std::vector<Point_2> outline(outer.vertices_begin(), outer.vertices_end());
      emit_pieces(pieces, loop_bbox_diag(outline));
    };

    for (const PolyIsland &isl : islands) {
      const std::vector<Point_2> outline = simplify_collinear_loop(isl.outer);
      if (outline.size() < 3) {
        continue;
      }
      if (isl.holes.empty()) {
        partition_simple_ring(outline);
        continue;
      }
      Polygon_2 outer(outline.begin(), outline.end());
      outer = force_ccw_simple(outer);
      Polygon_with_holes_2 pwh(outer);
      for (const std::vector<Point_2> &h : isl.holes) {
        std::vector<Point_2> hr = simplify_collinear_loop(h);
        if (hr.size() < 3) {
          continue;
        }
        Polygon_2 hole(hr.begin(), hr.end());
        if (hole.is_counterclockwise_oriented()) {
          hole.reverse_orientation();
        }
        pwh.add_hole(hole);
      }
      if (pwh.number_of_holes() == 0) {
        partition_simple_ring(outline);
      }
      else {
        partition_pwh_cdt(pwh);
      }
    }
    CGAL::set_error_behaviour(old_err);
    CGAL::set_warning_behaviour(old_warn);
    if (result.face_offsets.size() < 2) {
      result.error = "Convex Partition 2D produced no pieces";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    CGAL::set_error_behaviour(CGAL::CONTINUE);
    result.error = e.what();
  }
  catch (...) {
    CGAL::set_error_behaviour(CGAL::CONTINUE);
    result.error = "Convex Partition 2D failed";
  }
  return result;
}


/* -------------------------------------------------------------------------- */
/* 3. Y-Monotone Partition 2D                                                 */
/* -------------------------------------------------------------------------- */

MeshResult mesh_y_monotone_partition_2(const MeshIn &mesh)
{
  MeshResult result;
  const int plane = 0;
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    const bool extracted = extract_polygon_islands(mesh, plane, islands, mid, error, false);
    const bool extract_is_one_ngon = extracted && islands.size() == 1 &&
                                     islands[0].outer.size() >= 5;
    if (!extract_is_one_ngon) {
      std::vector<PolyIsland> uni;
      std::string uerr;
      float umid = mid;
      if (union_projected_faces(mesh, plane, uni, umid, uerr) && !uni.empty()) {
        islands.swap(uni);
        mid = umid;
      }
      else if (!extracted || islands.empty()) {
        result.error = error.empty() ?
                           (uerr.empty() ? "Y-Monotone Partition 2D needs a polygon" : uerr) :
                           error;
        return result;
      }
    }
    if (false) {
      result.error = "unreachable";
      return result;
    }
    result.face_offsets = {0};
    using Traits = CGAL::Partition_traits_2<Kernel>;
    using Partition_poly = Traits::Polygon_2;
    for (const PolyIsland &isl : islands) {
      if (isl.outer.size() < 3) {
        continue;
      }
      Polygon_2 outer(isl.outer.begin(), isl.outer.end());
      outer = force_ccw_simple(outer);
      if (!outer.is_simple()) {
        continue;
      }
      Partition_poly poly(outer.vertices_begin(), outer.vertices_end());
      std::list<Partition_poly> parts;
      CGAL::y_monotone_partition_2(
          poly.vertices_begin(), poly.vertices_end(), std::back_inserter(parts));
      for (const Partition_poly &pp : parts) {
        std::vector<Point_2> ring(pp.vertices_begin(), pp.vertices_end());
        append_ngon_face(result, ring, plane, mid);
      }
    }
    if (result.face_offsets.size() < 2) {
      result.error = "Y-Monotone Partition 2D produced no pieces";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Y-Monotone Partition 2D failed";
  }
  return result;
}

/* -------------------------------------------------------------------------- */
/* 4. Max Area K-gon 2D                                                       */
/* -------------------------------------------------------------------------- */

MeshResult points_max_area_k_gon_2(const float *positions, int n, int plane, int k)
{
  MeshResult result;
  if (!positions || n < 3) {
    result.error = "Max Area K-gon 2D needs at least 3 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  k = std::max(3, k);
  try {
    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.push_back(project_pt(positions + i * 3, plane));
    }
    const float mid = mid_dropped_axis(positions, n, plane);
    std::vector<Point_2> hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
    if (int(hull.size()) < 3) {
      result.error = "Degenerate hull";
      return result;
    }
    k = std::min(k, int(hull.size()));
    std::vector<Point_2> gon;
    CGAL::maximum_area_inscribed_k_gon_2(hull.begin(), hull.end(), k, std::back_inserter(gon));
    if (gon.size() < 3) {
      result.error = "K-gon failed";
      return result;
    }
    result.face_offsets = {0};
    append_ngon_face(result, gon, plane, mid);
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Max Area K-gon 2D failed";
  }
  return result;
}

/* -------------------------------------------------------------------------- */
/* 5. Polyline Simplify 2D                                                    */
/* -------------------------------------------------------------------------- */

MeshResult mesh_polyline_simplify_2(const MeshIn &mesh, double cost_threshold)
{
  MeshResult result;
  const int plane = 0;
  cost_threshold = std::max(cost_threshold, 1.0e-12);
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, plane, islands, mid, error)) {
      result.error = error.empty() ? "Shape Simplify 2D needs polygons" : error;
      return result;
    }
    result.face_offsets = {0};
    PS::Squared_distance_cost cost;
    PS::Stop_above_cost_threshold stop(cost_threshold);
    for (const PolyIsland &isl : islands) {
      if (isl.outer.size() < 3) {
        continue;
      }
      Polygon_2 outer(isl.outer.begin(), isl.outer.end());
      outer = force_ccw_simple(outer);
      Polygon_2 simplified = PS::simplify(outer, cost, stop);
      if (simplified.size() < 3) {
        simplified = outer;
      }
      std::vector<Point_2> ring(simplified.vertices_begin(), simplified.vertices_end());
      append_ngon_face(result, ring, plane, mid);
      for (const auto &h : isl.holes) {
        if (h.size() < 3) {
          continue;
        }
        Polygon_2 hp(h.begin(), h.end());
        if (hp.is_counterclockwise_oriented()) {
          hp.reverse_orientation();
        }
        Polygon_2 sh = PS::simplify(hp, cost, stop);
        if (sh.size() < 3) {
          sh = hp;
        }
        std::vector<Point_2> hr(sh.vertices_begin(), sh.vertices_end());
        append_ngon_face(result, hr, plane, mid);
      }
    }
    if (result.face_offsets.size() < 2) {
      result.error = "Shape Simplify 2D empty";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Shape Simplify 2D failed";
  }
  return result;
}

/* -------------------------------------------------------------------------- */
/* 6. Width 3D                                                                */
/* -------------------------------------------------------------------------- */

MeshResult points_width_3(const float *positions, int n, float plane_scale)
{
  MeshResult result;
  if (!positions || n < 4) {
    result.error = "Width 3D needs at least 4 points";
    return result;
  }
  plane_scale = std::max(plane_scale, 0.01f);
  try {
    /* Width_3 needs exact constructions; EPICK hits CGAL assertions on many meshes. */
    using EFT = CGAL::Lazy_exact_nt<CGAL::Quotient<CGAL::MP_Float>>;
    using EKernel = CGAL::Simple_cartesian<EFT>;
    using Traits = CGAL::Width_default_traits_3<EKernel>;
    using Width = CGAL::Width_3<Traits>;
    using EPoint = EKernel::Point_3;
    const CGAL::Failure_behaviour old_err = CGAL::set_error_behaviour(CGAL::THROW_EXCEPTION);
    const CGAL::Failure_behaviour old_warn = CGAL::set_warning_behaviour(CGAL::CONTINUE);
    std::vector<EPoint> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(EFT(double(positions[i * 3 + 0])),
                       EFT(double(positions[i * 3 + 1])),
                       EFT(double(positions[i * 3 + 2])));
    }
    Width W(pts.begin(), pts.end());
    CGAL::set_error_behaviour(old_err);
    CGAL::set_warning_behaviour(old_warn);
    EFT num, den;
    W.get_squared_width(num, den);
    const double w2 = CGAL::to_double(num) / std::max(1e-30, CGAL::to_double(den));
    const double w = std::sqrt(std::max(0.0, w2));
    result.radius = float(w);
    result.alpha_used = w;

    EKernel::Plane_3 e1e, e2e;
    W.get_width_planes(e1e, e2e);
    auto to_epick_plane = [](const EKernel::Plane_3 &pl) {
      return Kernel::Plane_3(CGAL::to_double(pl.a()),
                             CGAL::to_double(pl.b()),
                             CGAL::to_double(pl.c()),
                             CGAL::to_double(pl.d()));
    };
    const Kernel::Plane_3 e1 = to_epick_plane(e1e);
    const Kernel::Plane_3 e2 = to_epick_plane(e2e);

    /* Build two quads on supporting planes using a centroid and two axes. */
    double cx = 0, cy = 0, cz = 0;
    for (const EPoint &p : pts) {
      cx += CGAL::to_double(p.x());
      cy += CGAL::to_double(p.y());
      cz += CGAL::to_double(p.z());
    }
    cx /= double(n);
    cy /= double(n);
    cz /= double(n);
    Point center(cx, cy, cz);

    auto make_quad_on_plane = [&](const Kernel::Plane_3 &pl) {
      Kernel::Vector_3 nrm = pl.orthogonal_vector();
      const double nl = std::sqrt(CGAL::to_double(nrm.squared_length()));
      if (nl < 1e-20) {
        return;
      }
      nrm = nrm / nl;
      /* Project center onto plane. */
      Point on = pl.projection(center);
      Kernel::Vector_3 tmp(1, 0, 0);
      if (std::abs(CGAL::to_double(nrm * tmp)) > 0.9) {
        tmp = Kernel::Vector_3(0, 1, 0);
      }
      Kernel::Vector_3 u = CGAL::cross_product(nrm, tmp);
      const double ul = std::sqrt(CGAL::to_double(u.squared_length()));
      u = u / ul;
      Kernel::Vector_3 v = CGAL::cross_product(nrm, u);
      const double s = double(plane_scale);
      Point p0 = on + (-s) * u + (-s) * v;
      Point p1 = on + (s)*u + (-s) * v;
      Point p2 = on + (s)*u + (s)*v;
      Point p3 = on + (-s) * u + (s)*v;
      const int base = result.verts_num();
      auto push_p = [&](const Point &p) {
        result.positions.push_back(float(CGAL::to_double(p.x())));
        result.positions.push_back(float(CGAL::to_double(p.y())));
        result.positions.push_back(float(CGAL::to_double(p.z())));
      };
      push_p(p0);
      push_p(p1);
      push_p(p2);
      push_p(p3);
      if (result.face_offsets.empty()) {
        result.face_offsets.push_back(0);
      }
      result.corner_verts.push_back(base + 0);
      result.corner_verts.push_back(base + 1);
      result.corner_verts.push_back(base + 2);
      result.corner_verts.push_back(base + 3);
      result.face_offsets.push_back(int(result.corner_verts.size()));
    };

    make_quad_on_plane(e1);
    make_quad_on_plane(e2);
    result.center[0] = float(cx);
    result.center[1] = float(cy);
    result.center[2] = float(cz);
    result.ok = result.face_offsets.size() >= 2;
    if (!result.ok) {
      result.error = "Width 3D failed to build planes";
    }
  }
  catch (const std::exception &e) {
    CGAL::set_error_behaviour(CGAL::CONTINUE);
    result.error = e.what();
  }
  catch (...) {
    CGAL::set_error_behaviour(CGAL::CONTINUE);
    result.error = "Width 3D failed (need a non-coplanar 3D point set)";
  }
  return result;
}

/* -------------------------------------------------------------------------- */
/* 7. Minkowski Sum 2D                                                        */
/* -------------------------------------------------------------------------- */

MeshResult mesh_minkowski_sum_2(const MeshIn &mesh_a, const MeshIn &mesh_b)
{
  MeshResult result;
  const int plane = 0;
  try {
    std::vector<PolyIsland> ia, ib;
    float mida = 0.0f, midb = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh_a, plane, ia, mida, error) || ia.empty()) {
      result.error = error.empty() ? "Minkowski Sum 2D: mesh A needs polygon" : error;
      return result;
    }
    if (!extract_polygon_islands(mesh_b, plane, ib, midb, error) || ib.empty()) {
      result.error = error.empty() ? "Minkowski Sum 2D: mesh B needs polygon" : error;
      return result;
    }
    const float mid = 0.5f * (mida + midb);
    Polygon_2 a(ia[0].outer.begin(), ia[0].outer.end());
    Polygon_2 b(ib[0].outer.begin(), ib[0].outer.end());
    a = force_ccw_simple(a);
    b = force_ccw_simple(b);
    if (!a.is_simple() || !b.is_simple() || a.size() < 3 || b.size() < 3) {
      result.error = "Minkowski Sum 2D needs simple polygons";
      return result;
    }
    Polygon_with_holes_2 sum = CGAL::minkowski_sum_by_reduced_convolution_2(a, b);
    append_pwh_triangulated(result, sum, plane, mid);
    if (result.corner_verts.size() < 3) {
      result.error = "Minkowski Sum 2D empty";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Minkowski Sum 2D failed";
  }
  return result;
}

/* -------------------------------------------------------------------------- */
/* 8. Extrude Skeleton                                                        */
/* -------------------------------------------------------------------------- */

MeshResult mesh_extrude_skeleton(const MeshIn &mesh, double height, bool outward)
{
  MeshResult result;
  const int plane = 0;
  if (std::abs(height) < 1e-12) {
    result.error = "Height must be non-zero";
    return result;
  }
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, plane, islands, mid, error) || islands.empty()) {
      result.error = error.empty() ? "Extrude Skeleton needs polygons" : error;
      return result;
    }
    for (const PolyIsland &isl : islands) {
      if (isl.outer.size() < 3) {
        continue;
      }
      Polygon_2 outer(isl.outer.begin(), isl.outer.end());
      outer = force_ccw_simple(outer);
      if (!outer.is_simple()) {
        continue;
      }
      std::vector<Polygon_2> holes;
      for (const auto &h : isl.holes) {
        if (h.size() < 3) {
          continue;
        }
        Polygon_2 hp(h.begin(), h.end());
        if (hp.is_counterclockwise_oriented()) {
          hp.reverse_orientation();
        }
        holes.push_back(std::move(hp));
      }
      Polygon_with_holes_2 pwh(outer, holes.begin(), holes.end());
      Surface_mesh sm_out;
      /* Outward uses negative height in CGAL API for outward slope when weights positive...
       * Use angles/weights: positive weight = inward. For outward, use negative weights. */
      std::vector<std::vector<double>> weights;
      const double w = outward ? -1.0 : 1.0;
      weights.push_back(std::vector<double>(pwh.outer_boundary().size(), w));
      for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
        weights.push_back(std::vector<double>(hit->size(), w));
      }
      const bool ok = CGAL::extrude_skeleton(
          pwh, sm_out, CGAL::parameters::maximum_height(height).weights(weights));
      if (!ok || sm_out.number_of_faces() == 0) {
        continue;
      }
      PMP::triangulate_faces(sm_out);
      /* Append triangles (pure triangle encoding). */
      std::map<Surface_mesh::Vertex_index, int> vmap;
      const int base = result.verts_num();
      int local = 0;
      for (auto v : sm_out.vertices()) {
        vmap[v] = base + local++;
        const Point &p = sm_out.point(v);
        result.positions.push_back(float(CGAL::to_double(p.x())));
        result.positions.push_back(float(CGAL::to_double(p.y())));
        result.positions.push_back(float(CGAL::to_double(p.z())));
      }
      for (auto f : sm_out.faces()) {
        std::vector<int> idxs;
        for (auto v : CGAL::vertices_around_face(sm_out.halfedge(f), sm_out)) {
          idxs.push_back(vmap[v]);
        }
        if (idxs.size() == 3) {
          result.corner_verts.push_back(idxs[0]);
          result.corner_verts.push_back(idxs[1]);
          result.corner_verts.push_back(idxs[2]);
        }
        else if (idxs.size() > 3) {
          for (size_t i = 1; i + 1 < idxs.size(); i++) {
            result.corner_verts.push_back(idxs[0]);
            result.corner_verts.push_back(idxs[i]);
            result.corner_verts.push_back(idxs[i + 1]);
          }
        }
      }
    }
    if (result.corner_verts.size() < 3) {
      result.error = "Extrude Skeleton empty (check simple polygon / height)";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Extrude Skeleton failed";
  }
  return result;
}

/* -------------------------------------------------------------------------- */
/* 9. Polygon Fill 2D                                                         */
/* -------------------------------------------------------------------------- */

MeshResult mesh_polygon_fill_2(const MeshIn &mesh)
{
  MeshResult result;
  const int plane = 0;
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, plane, islands, mid, error)) {
      result.error = error.empty() ? "Polygon Fill 2D needs polygons" : error;
      return result;
    }
    for (const PolyIsland &isl : islands) {
      if (isl.outer.size() < 3) {
        continue;
      }
      Polygon_2 outer(isl.outer.begin(), isl.outer.end());
      outer = force_ccw_simple(outer);
      std::vector<Polygon_2> holes;
      for (const auto &h : isl.holes) {
        if (h.size() < 3) {
          continue;
        }
        Polygon_2 hp(h.begin(), h.end());
        if (hp.is_counterclockwise_oriented()) {
          hp.reverse_orientation();
        }
        holes.push_back(std::move(hp));
      }
      Polygon_with_holes_2 pwh(outer, holes.begin(), holes.end());
      append_pwh_triangulated(result, pwh, plane, mid);
    }
    if (result.corner_verts.size() < 3) {
      result.error = "Polygon Fill 2D empty";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Polygon Fill 2D failed";
  }
  return result;
}

/* -------------------------------------------------------------------------- */
/* 10. Do Intersect 2D                                                        */
/* -------------------------------------------------------------------------- */

bool mesh_do_intersect_2(const MeshIn &mesh_a, const MeshIn &mesh_b, std::string &error)
{
  error.clear();
  const int plane = 0;
  try {
    std::vector<PolyIsland> ia, ib;
    float mida = 0.0f, midb = 0.0f;
    std::string e;
    if (!extract_polygon_islands(mesh_a, plane, ia, mida, e) || ia.empty()) {
      error = e.empty() ? "Do Intersect 2D: mesh A needs polygon" : e;
      return false;
    }
    if (!extract_polygon_islands(mesh_b, plane, ib, midb, e) || ib.empty()) {
      error = e.empty() ? "Do Intersect 2D: mesh B needs polygon" : e;
      return false;
    }
    for (const PolyIsland &a : ia) {
      if (a.outer.size() < 3) {
        continue;
      }
      Polygon_2 pa(a.outer.begin(), a.outer.end());
      pa = force_ccw_simple(pa);
      for (const PolyIsland &b : ib) {
        if (b.outer.size() < 3) {
          continue;
        }
        Polygon_2 pb(b.outer.begin(), b.outer.end());
        pb = force_ccw_simple(pb);
        if (polygons_do_intersect(pa, pb)) {
          return true;
        }
      }
    }
    return false;
  }
  catch (const std::exception &ex) {
    error = ex.what();
    return false;
  }
  catch (...) {
    error = "Do Intersect 2D failed";
    return false;
  }
}

}  // namespace blender::cgal_bridge
