/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 32 — new only (not previously deleted / already existing):
 *  - Regular Triangulation 3D
 *  - Power Diagram 3D
 *  - Largest Empty Circle 2D
 *  - Largest Inscribed Circle 2D
 *  - Min Width 2D
 *  - Periodic Delaunay 2D
 *  - Constrained Voronoi 2D
 *  - Connect Holes 2D (deleted; 2486 reserved)
 *
 * Skipped: P22 OpenGR, R08 deprecated, G10 Kinetic EPECK, S11/S12 user-deleted,
 * Polygonal Surface Reconstruction (needs SCIP/GLPK), Mesh_3 volume (heavy),
 * previously banned measure/query / remesh / component nodes.
 */

#include "cgal_bridge.hh"
#include "cgal_convex_clip3.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Periodic_2_Delaunay_triangulation_2.h>
#include <CGAL/Periodic_2_Delaunay_triangulation_traits_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Regular_triangulation_3.h>
#include <CGAL/Segment_Delaunay_graph_2.h>
#include <CGAL/Segment_Delaunay_graph_filtered_traits_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/assertions_behaviour.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/intersections.h>
#include <CGAL/mark_domain_in_triangulation.h>

#include <boost/property_map/property_map.hpp>

#include <CGAL/squared_distance_2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <list>
#include <variant>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <vector>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

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

  if (loops.empty() && mesh.edges_num > 0 && mesh.edge_v0 && mesh.edge_v1) {
    /* Wire loops: walk cyclic edges. */
    const int ne = mesh.edges_num;
    std::vector<std::vector<int>> adj(size_t(mesh.verts_num));
    for (int e = 0; e < ne; e++) {
      const int a = mesh.edge_v0[e];
      const int b = mesh.edge_v1[e];
      if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
        continue;
      }
      adj[size_t(a)].push_back(b);
      adj[size_t(b)].push_back(a);
    }
    std::vector<char> used(size_t(mesh.verts_num), 0);
    for (int s = 0; s < mesh.verts_num; s++) {
      if (used[size_t(s)] || adj[size_t(s)].size() != 2) {
        continue;
      }
      std::vector<Point_2> raw;
      int prev = -1;
      int cur = s;
      for (int step = 0; step < mesh.verts_num + 2; step++) {
        if (used[size_t(cur)]) {
          break;
        }
        used[size_t(cur)] = 1;
        raw.push_back(project_pt(mesh.positions + cur * 3, plane));
        int nxt = -1;
        for (int nb : adj[size_t(cur)]) {
          if (nb != prev) {
            nxt = nb;
            break;
          }
        }
        if (nxt < 0) {
          break;
        }
        prev = cur;
        cur = nxt;
        if (cur == s) {
          break;
        }
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

  if (islands.empty() && mesh.faces_num > 0 && mesh.corner_verts) {
    /* Closed mesh: XY footprint = union of projected faces (silhouette). */
    try {
      CGAL::Polygon_set_2<Kernel> pset;
      bool any = false;
      const int nf = mesh.faces_num;
      for (int f = 0; f < nf; f++) {
        int a = 0, b = 0;
        if (mesh.face_offsets) {
          a = mesh.face_offsets[f];
          b = mesh.face_offsets[f + 1];
        }
        else {
          a = f * 3;
          b = a + 3;
        }
        if (b - a < 3) {
          continue;
        }
        std::vector<Point_2> raw;
        raw.reserve(size_t(b - a));
        for (int c = a; c < b; c++) {
          const int v = mesh.corner_verts[c];
          if (v < 0 || v >= mesh.verts_num) {
            raw.clear();
            break;
          }
          raw.push_back(project_pt(mesh.positions + v * 3, plane));
        }
        Loop L = make_loop(std::move(raw));
        if (L.verts.size() < 3 || L.area_abs < 1e-18) {
          continue;
        }
        if (signed_area_2(L.verts) < 0.0) {
          std::reverse(L.verts.begin(), L.verts.end());
        }
        try {
          Polygon_2 poly(L.verts.begin(), L.verts.end());
          if (!any) {
            pset.insert(poly);
            any = true;
          }
          else {
            pset.join(poly);
          }
        }
        catch (...) {
        }
      }
      if (any && !pset.is_empty()) {
        std::list<CGAL::Polygon_with_holes_2<Kernel>> pwhs;
        pset.polygons_with_holes(std::back_inserter(pwhs));
        for (const auto &pwh : pwhs) {
          PolyIsland isl;
          isl.outer.assign(pwh.outer_boundary().vertices_begin(),
                           pwh.outer_boundary().vertices_end());
          isl.outer = clean_loop_2(isl.outer);
          if (isl.outer.size() < 3) {
            continue;
          }
          if (signed_area_2(isl.outer) < 0.0) {
            std::reverse(isl.outer.begin(), isl.outer.end());
          }
          for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
            std::vector<Point_2> hole(hit->vertices_begin(), hit->vertices_end());
            hole = clean_loop_2(hole);
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
    }
    catch (...) {
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
      case 1:
        p[0] = cx + float(ca);
        p[1] = cy;
        p[2] = cz + float(sa);
        break;
      case 2:
        p[0] = cx;
        p[1] = cy + float(ca);
        p[2] = cz + float(sa);
        break;
      default:
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

static double dist_point_seg(const Point_2 &q, const Point_2 &a, const Point_2 &b)
{
  const double qx = CGAL::to_double(q.x());
  const double qy = CGAL::to_double(q.y());
  const double ax = CGAL::to_double(a.x());
  const double ay = CGAL::to_double(a.y());
  const double bx = CGAL::to_double(b.x());
  const double by = CGAL::to_double(b.y());
  const double ex = bx - ax;
  const double ey = by - ay;
  const double len2 = ex * ex + ey * ey;
  double t = 0.0;
  if (len2 > 1e-30) {
    t = std::clamp(((qx - ax) * ex + (qy - ay) * ey) / len2, 0.0, 1.0);
  }
  const double dx = qx - (ax + t * ex);
  const double dy = qy - (ay + t * ey);
  return std::sqrt(dx * dx + dy * dy);
}

static bool point_in_island(const Point_2 &q, const PolyIsland &isl)
{
  if (!point_in_poly(q, isl.outer)) {
    return false;
  }
  for (const auto &hole : isl.holes) {
    if (point_in_poly(q, hole)) {
      return false;
    }
  }
  return true;
}

static double clearance_to_island(const Point_2 &q, const PolyIsland &isl)
{
  double best = std::numeric_limits<double>::infinity();
  auto consider = [&](const std::vector<Point_2> &ring) {
    const size_t n = ring.size();
    for (size_t i = 0; i < n; i++) {
      best = std::min(best, dist_point_seg(q, ring[i], ring[(i + 1) % n]));
    }
  };
  consider(isl.outer);
  for (const auto &h : isl.holes) {
    consider(h);
  }
  return best;
}

static void emit_regular_tets(MeshResult &result,
                              const std::vector<Point_3> &verts,
                              const std::vector<std::array<int, 4>> &tets,
                              bool separate_tets)
{
  result.positions.clear();
  result.corner_verts.clear();
  result.face_tag.clear();
  if (separate_tets) {
    result.positions.reserve(tets.size() * 12);
    result.corner_verts.reserve(tets.size() * 12);
    result.face_tag.reserve(tets.size() * 4);
    int tet_id = 0;
    for (const auto &t : tets) {
      const int base = result.verts_num();
      for (int v = 0; v < 4; v++) {
        const Point_3 &p = verts[size_t(t[size_t(v)])];
        result.positions.push_back(float(CGAL::to_double(p.x())));
        result.positions.push_back(float(CGAL::to_double(p.y())));
        result.positions.push_back(float(CGAL::to_double(p.z())));
      }
      for (int i = 0; i < 4; i++) {
        result.corner_verts.push_back(base + ((i + 1) & 3));
        result.corner_verts.push_back(base + ((i + 2) & 3));
        result.corner_verts.push_back(base + ((i + 3) & 3));
        result.face_tag.push_back(tet_id);
      }
      tet_id++;
    }
  }
  else {
    result.positions.reserve(verts.size() * 3);
    for (const Point_3 &p : verts) {
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
    }
    result.corner_verts.reserve(tets.size() * 12);
    result.face_tag.reserve(tets.size() * 4);
    int tet_id = 0;
    for (const auto &t : tets) {
      for (int i = 0; i < 4; i++) {
        result.corner_verts.push_back(t[size_t((i + 1) & 3)]);
        result.corner_verts.push_back(t[size_t((i + 2) & 3)]);
        result.corner_verts.push_back(t[size_t((i + 3) & 3)]);
        result.face_tag.push_back(tet_id);
      }
      tet_id++;
    }
  }
  result.radius = float(tets.size());
  result.ok = result.corners_num() >= 3;
}

}  // namespace

MeshResult points_regular_triangulation_3(const float *positions,
                                          const float *radii,
                                          int n,
                                          bool separate_tets)
{
  MeshResult result;
  if (!positions || n < 4) {
    result.error = "Regular Triangulation 3D needs at least 4 points";
    return result;
  }
  try {
    using Rt = CGAL::Regular_triangulation_3<Kernel>;
    using Wp = Rt::Weighted_point;
    using Bp = Rt::Bare_point;
    Rt rt;
    for (int i = 0; i < n; i++) {
      const double r = radii ? std::max(0.0, double(radii[i])) : 0.0;
      rt.insert(Wp(Bp(double(positions[i * 3 + 0]),
                      double(positions[i * 3 + 1]),
                      double(positions[i * 3 + 2])),
                   r * r));
    }
    if (rt.dimension() != 3) {
      result.error = "Regular Triangulation 3D degenerate (need non-coplanar points)";
      return result;
    }
    std::map<Rt::Vertex_handle, int> v2i;
    std::vector<Point_3> verts;
    verts.reserve(size_t(rt.number_of_vertices()));
    for (auto vit = rt.finite_vertices_begin(); vit != rt.finite_vertices_end(); ++vit) {
      v2i[vit] = int(verts.size());
      verts.push_back(vit->point().point());
    }
    std::vector<std::array<int, 4>> tets;
    tets.reserve(size_t(rt.number_of_finite_cells()));
    for (auto cit = rt.finite_cells_begin(); cit != rt.finite_cells_end(); ++cit) {
      std::array<int, 4> t;
      bool ok = true;
      for (int i = 0; i < 4; i++) {
        auto it = v2i.find(cit->vertex(i));
        if (it == v2i.end()) {
          ok = false;
          break;
        }
        t[size_t(i)] = it->second;
      }
      if (ok) {
        tets.push_back(t);
      }
    }
    emit_regular_tets(result, verts, tets, separate_tets);
    if (!result.ok) {
      result.error = "Regular Triangulation 3D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Regular Triangulation 3D failed";
  }
  return result;
}

static std::vector<Point_3> unique_points_3_pd(std::vector<Point_3> pts, double eps2)
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

static Point_3 clamp_aabb3_pd(const Point_3 &p, const double mn[3], const double mx[3])
{
  return Point_3(std::min(mx[0], std::max(mn[0], CGAL::to_double(p.x()))),
                 std::min(mx[1], std::max(mn[1], CGAL::to_double(p.y()))),
                 std::min(mx[2], std::max(mn[2], CGAL::to_double(p.z()))));
}

static Point_3 lerp3_pd(const Point_3 &a, const Point_3 &b, double t)
{
  const double ax = CGAL::to_double(a.x());
  const double ay = CGAL::to_double(a.y());
  const double az = CGAL::to_double(a.z());
  return Point_3(ax + t * (CGAL::to_double(b.x()) - ax),
                 ay + t * (CGAL::to_double(b.y()) - ay),
                 az + t * (CGAL::to_double(b.z()) - az));
}

static bool hull_from_points3_pd(const std::vector<Point_3> &pts, Surface_mesh &sm)
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

static std::vector<Point_3> clip_convex_halfspace3_pd(std::vector<Point_3> pts,
                                                      double nx,
                                                      double ny,
                                                      double nz,
                                                      double c)
{
  pts = unique_points_3_pd(std::move(pts), 1e-20);
  if (pts.size() < 3) {
    return {};
  }
  Surface_mesh sm;
  if (!hull_from_points3_pd(pts, sm)) {
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
      next.push_back(lerp3_pd(pa, pb, std::min(1.0, std::max(0.0, t))));
    }
  }
  return unique_points_3_pd(std::move(next), 1e-20);
}

static std::vector<Point_3> clip_convex_to_aabb3_pd(std::vector<Point_3> cand,
                                                    const double mn[3],
                                                    const double mx[3])
{
  cand = clip_convex_halfspace3_pd(std::move(cand), -1.0, 0.0, 0.0, -mn[0]);
  cand = clip_convex_halfspace3_pd(std::move(cand), 1.0, 0.0, 0.0, mx[0]);
  cand = clip_convex_halfspace3_pd(std::move(cand), 0.0, -1.0, 0.0, -mn[1]);
  cand = clip_convex_halfspace3_pd(std::move(cand), 0.0, 1.0, 0.0, mx[1]);
  cand = clip_convex_halfspace3_pd(std::move(cand), 0.0, 0.0, -1.0, -mn[2]);
  cand = clip_convex_halfspace3_pd(std::move(cand), 0.0, 0.0, 1.0, mx[2]);
  return cand;
}

static void aabb_corners3_pd(const double mn[3], const double mx[3], std::vector<Point_3> &out)
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

static void emit_isolated_hull3_pd(MeshResult &result,
                                   std::vector<Point_3> cand,
                                   int tag,
                                   const double mn[3],
                                   const double mx[3])
{
  cand = clip_convex_to_aabb3_pd(std::move(cand), mn, mx);
  if (cand.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  if (cand.size() == 3) {
    const int base = result.verts_num();
    for (const Point_3 &p : cand) {
      const Point_3 q = clamp_aabb3_pd(p, mn, mx);
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
    const Point_3 p = clamp_aabb3_pd(sm.point(v), mn, mx);
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

MeshResult points_power_diagram_3(const float *positions,
                                  const float *radii,
                                  int n,
                                  float clip_margin)
{
  MeshResult result;
  if (!positions || n < 5) {
    result.error = "Power Diagram 3D needs at least 5 non-coplanar points";
    return result;
  }
  try {
    using Rt = CGAL::Regular_triangulation_3<Kernel>;
    using Wp = Rt::Weighted_point;
    using Bp = Rt::Bare_point;
    Rt rt;
    double mn[3] = {1e300, 1e300, 1e300};
    double mx[3] = {-1e300, -1e300, -1e300};
    for (int i = 0; i < n; i++) {
      const double x = double(positions[i * 3 + 0]);
      const double y = double(positions[i * 3 + 1]);
      const double z = double(positions[i * 3 + 2]);
      const double r = radii ? std::max(0.0, double(radii[i])) : 0.0;
      rt.insert(Wp(Bp(x, y, z), r * r));
      mn[0] = std::min(mn[0], x);
      mn[1] = std::min(mn[1], y);
      mn[2] = std::min(mn[2], z);
      mx[0] = std::max(mx[0], x);
      mx[1] = std::max(mx[1], y);
      mx[2] = std::max(mx[2], z);
    }
    if (rt.dimension() != 3) {
      result.error = "Power Diagram 3D degenerate (need non-coplanar points)";
      return result;
    }
    clip3::pad_aabb_extent(mn, mx, double(clip_margin));
    int tag = 0;
    for (auto vit = rt.finite_vertices_begin(); vit != rt.finite_vertices_end(); ++vit) {
      const typename Rt::Vertex_handle site = vit;
      const Point_3 p = site->point().point();
      const double px = CGAL::to_double(p.x());
      const double py = CGAL::to_double(p.y());
      const double pz = CGAL::to_double(p.z());
      const double wp = CGAL::to_double(site->point().weight());
      clip3::Poly3 cell = clip3::make_aabb(mn, mx);
      std::vector<typename Rt::Cell_handle> cells;
      rt.incident_cells(site, std::back_inserter(cells));
      std::set<typename Rt::Vertex_handle> nbrs;
      for (typename Rt::Cell_handle c : cells) {
        for (int i = 0; i < 4; i++) {
          const typename Rt::Vertex_handle v = c->vertex(i);
          if (v != site && !rt.is_infinite(v)) {
            nbrs.insert(v);
          }
        }
      }
      bool alive = true;
      for (const typename Rt::Vertex_handle qh : nbrs) {
        const Point_3 q = qh->point().point();
        const double qx = CGAL::to_double(q.x());
        const double qy = CGAL::to_double(q.y());
        const double qz = CGAL::to_double(q.z());
        const double wq = CGAL::to_double(qh->point().weight());
        const double nx = qx - px;
        const double ny = qy - py;
        const double nz = qz - pz;
        const double rhs = 0.5 * ((qx * qx + qy * qy + qz * qz) - (px * px + py * py + pz * pz) -
                                  (wq - wp));
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
      result.error = "Power Diagram 3D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Power Diagram 3D failed";
  }
  return result;
}

MeshResult points_largest_empty_circle_2(const float *positions, int n, int plane, int segments)
{
  MeshResult result;
  if (!positions || n < 2) {
    result.error = "Largest Empty Circle 2D needs at least 2 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  segments = std::clamp(segments, 8, 128);
  try {
    using DT = CGAL::Delaunay_triangulation_2<Kernel>;
    using Seg2 = Kernel::Segment_2;
    using Ray2 = Kernel::Ray_2;
    using Vec2 = Kernel::Vector_2;

    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.push_back(project_pt(positions + i * 3, plane));
    }
    DT dt(pts.begin(), pts.end());
    std::vector<Point_2> hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
    if (hull.size() < 2) {
      result.error = "Largest Empty Circle 2D: degenerate point set";
      return result;
    }
    if (signed_area_2(hull) < 0.0) {
      std::reverse(hull.begin(), hull.end());
    }

    auto min_site_dist = [&](const Point_2 &c) {
      double best = std::numeric_limits<double>::infinity();
      for (auto vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); ++vit) {
        const Point_2 &p = vit->point();
        const double dx = CGAL::to_double(c.x() - p.x());
        const double dy = CGAL::to_double(c.y() - p.y());
        best = std::min(best, std::sqrt(dx * dx + dy * dy));
      }
      return best;
    };
    auto in_hull = [&](const Point_2 &q) {
      if (hull.size() < 3) {
        return true;
      }
      const auto side = CGAL::bounded_side_2(hull.begin(), hull.end(), q, Kernel());
      return side != CGAL::ON_UNBOUNDED_SIDE;
    };

    Point_2 best_c = hull[0];
    double best_r = -1.0;
    auto consider = [&](const Point_2 &c) {
      if (!in_hull(c)) {
        return;
      }
      const double r = min_site_dist(c);
      if (r > best_r) {
        best_r = r;
        best_c = c;
      }
    };

    auto hit_hull = [&](const auto &curve) {
      for (int i = 0; i < int(hull.size()); i++) {
        const Seg2 he(hull[size_t(i)], hull[size_t((i + 1) % int(hull.size()))]);
        auto isect = CGAL::intersection(curve, he);
        if (!isect) {
          continue;
        }
        if (const Point_2 *p = std::get_if<Point_2>(&*isect)) {
          consider(*p);
        }
        else if (const Seg2 *s = std::get_if<Seg2>(&*isect)) {
          consider(s->source());
          consider(s->target());
          consider(CGAL::midpoint(s->source(), s->target()));
        }
      }
    };

    if (dt.number_of_vertices() < 2) {
      result.error = "Largest Empty Circle 2D: need distinct XY points";
      return result;
    }
    if (dt.dimension() < 2) {
      /* Collinear: largest empty gap is half the longest consecutive spacing,
       * but the LEC in the hull is the diametral circle of the two extremes. */
      auto vit = dt.finite_vertices_begin();
      Point_2 a = vit->point();
      Point_2 b = a;
      for (; vit != dt.finite_vertices_end(); ++vit) {
        b = vit->point();
      }
      consider(CGAL::midpoint(a, b));
    }
    else {
      for (auto fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
        consider(dt.circumcenter(fit));
      }
      /* Voronoi edges / rays ∩ convex hull — required when all circumcenters
       * lie outside (obtuse hull triangles). */
      for (auto eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
        auto f = eit->first;
        const int i = eit->second;
        auto nb = f->neighbor(i);
        const bool f_inf = dt.is_infinite(f);
        const bool n_inf = dt.is_infinite(nb);
        if (!f_inf && !n_inf) {
          hit_hull(Seg2(dt.circumcenter(f), dt.circumcenter(nb)));
          continue;
        }
        auto fin = f_inf ? nb : f;
        const int fi = f_inf ? nb->index(f) : i;
        const Point_2 c = dt.circumcenter(fin);
        const Point_2 a = fin->vertex(dt.cw(fi))->point();
        const Point_2 b = fin->vertex(dt.ccw(fi))->point();
        const Point_2 v = fin->vertex(fi)->point();
        Vec2 e = b - a;
        Vec2 nrm(-e.y(), e.x());
        if (nrm * (v - a) > 0) {
          nrm = -nrm;
        }
        hit_hull(Ray2(c, nrm));
      }
    }

    if (best_r <= 1e-15) {
      result.error = "Largest Empty Circle 2D produced zero radius";
      return result;
    }
    const float mid = mid_dropped_axis(positions, n, plane);
    float center3[3];
    unproject(CGAL::to_double(best_c.x()), CGAL::to_double(best_c.y()), plane, mid, center3);
    result.center[0] = center3[0];
    result.center[1] = center3[1];
    result.center[2] = center3[2];
    result.radius = float(best_r);
    append_disk(center3[0], center3[1], center3[2], float(best_r), plane, segments, result);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Largest Empty Circle 2D failed";
  }
  return result;
}

MeshResult mesh_largest_inscribed_circle_2(const MeshIn &mesh, int segments)
{
  MeshResult result;
  segments = std::clamp(segments, 8, 128);
  const int plane = 0;
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, plane, islands, mid, error) || islands.empty()) {
      result.error = error.empty() ? "Largest Inscribed Circle 2D needs a polygon" : error;
      return result;
    }

    using SDG_Gt = CGAL::Segment_Delaunay_graph_filtered_traits_2<Kernel>;
    using SDG = CGAL::Segment_Delaunay_graph_2<SDG_Gt>;
    using Site_2 = SDG::Site_2;

    Point_2 best_c(0, 0);
    double best_r = -1.0;
    bool have = false;

    for (const PolyIsland &isl : islands) {
      if (isl.outer.size() < 3) {
        continue;
      }
      auto consider = [&](const Point_2 &c) {
        if (!point_in_island(c, isl)) {
          return;
        }
        const double r = clearance_to_island(c, isl);
        if (r > best_r) {
          best_r = r;
          best_c = c;
          have = true;
        }
      };

      /* Segment Voronoi vertices = centers of circles tangent to 3 boundary
       * elements. The max-clearance one is the true inscribed circle. */
      try {
        SDG sdg;
        auto insert_ring = [&](const std::vector<Point_2> &ring) {
          const size_t n = ring.size();
          for (size_t i = 0; i < n; i++) {
            const Point_2 &a = ring[i];
            const Point_2 &b = ring[(i + 1) % n];
            if (CGAL::squared_distance(a, b) < 1e-24) {
              continue;
            }
            sdg.insert(Site_2::construct_site_2(a, b));
          }
        };
        insert_ring(isl.outer);
        for (const auto &h : isl.holes) {
          insert_ring(h);
        }
        if (sdg.dimension() == 2) {
          for (auto fit = sdg.finite_faces_begin(); fit != sdg.finite_faces_end(); ++fit) {
            try {
              consider(sdg.primal(fit));
            }
            catch (...) {
            }
          }
        }
      }
      catch (...) {
      }

      /* Also evaluate CDT incenters (always inside their triangle). */
      using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
      using Tds = CGAL::Triangulation_data_structure_2<CGAL::Triangulation_vertex_base_2<Kernel>,
                                                       Fb>;
      using CDT =
          CGAL::Constrained_Delaunay_triangulation_2<Kernel, Tds, CGAL::Exact_predicates_tag>;
      try {
        CDT cdt;
        auto insert_ring = [&](const std::vector<Point_2> &ring) {
          std::vector<typename CDT::Vertex_handle> vhs;
          vhs.reserve(ring.size());
          for (const Point_2 &p : ring) {
            vhs.push_back(cdt.insert(p));
          }
          const size_t n = vhs.size();
          for (size_t i = 0; i < n; i++) {
            if (vhs[i] != vhs[(i + 1) % n]) {
              cdt.insert_constraint(vhs[i], vhs[(i + 1) % n]);
            }
          }
        };
        insert_ring(isl.outer);
        for (const auto &h : isl.holes) {
          insert_ring(h);
        }
        if (cdt.dimension() == 2) {
          std::map<typename CDT::Face_handle, bool> in_domain_map;
          boost::associative_property_map<std::map<typename CDT::Face_handle, bool>> in_domain(
              in_domain_map);
          CGAL::mark_domain_in_triangulation(cdt, in_domain);
          for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
            if (!get(in_domain, fit)) {
              continue;
            }
            const Point_2 a = fit->vertex(0)->point();
            const Point_2 b = fit->vertex(1)->point();
            const Point_2 c = fit->vertex(2)->point();
            const double ax = CGAL::to_double(a.x()), ay = CGAL::to_double(a.y());
            const double bx = CGAL::to_double(b.x()), by = CGAL::to_double(b.y());
            const double cx = CGAL::to_double(c.x()), cy = CGAL::to_double(c.y());
            const double lab = std::hypot(bx - ax, by - ay);
            const double lbc = std::hypot(cx - bx, cy - by);
            const double lca = std::hypot(ax - cx, ay - cy);
            const double per = lab + lbc + lca;
            if (per > 1e-20) {
              consider(Point_2((lab * cx + lbc * ax + lca * bx) / per,
                               (lab * cy + lbc * ay + lca * by) / per));
            }
          }
        }
      }
      catch (...) {
      }
    }

    if (!have || best_r <= 1e-15) {
      result.error = "Largest Inscribed Circle 2D produced zero radius";
      return result;
    }
    float center3[3];
    unproject(CGAL::to_double(best_c.x()), CGAL::to_double(best_c.y()), plane, mid, center3);
    result.center[0] = center3[0];
    result.center[1] = center3[1];
    result.center[2] = center3[2];
    result.radius = float(best_r);
    append_disk(center3[0], center3[1], center3[2], float(best_r), plane, segments, result);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Largest Inscribed Circle 2D failed";
  }
  return result;
}

MeshResult points_min_width_2(const float *positions, int n, int plane, float plane_scale)
{
  MeshResult result;
  if (!positions || n < 2) {
    result.error = "Min Width 2D needs at least 2 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  plane_scale = std::max(plane_scale, 0.01f);
  try {
    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.push_back(project_pt(positions + i * 3, plane));
    }
    std::vector<Point_2> hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
    if (hull.size() < 2) {
      result.error = "Min Width 2D: degenerate point set";
      return result;
    }
    if (signed_area_2(hull) < 0.0) {
      std::reverse(hull.begin(), hull.end());
    }

    const int hn = int(hull.size());
    double best_w = std::numeric_limits<double>::infinity();
    Point_2 e0 = hull[0], e1 = hull[hn > 1 ? 1 : 0], anti = hull[0];

    auto dist_line = [](const Point_2 &a, const Point_2 &b, const Point_2 &p) {
      const double ax = CGAL::to_double(a.x()), ay = CGAL::to_double(a.y());
      const double bx = CGAL::to_double(b.x()), by = CGAL::to_double(b.y());
      const double px = CGAL::to_double(p.x()), py = CGAL::to_double(p.y());
      const double ex = bx - ax, ey = by - ay;
      const double el = std::hypot(ex, ey);
      if (el < 1e-20) {
        return 0.0;
      }
      return std::abs(ex * (py - ay) - ey * (px - ax)) / el;
    };

    if (hn == 2) {
      best_w = 0.0;
      e0 = hull[0];
      e1 = hull[1];
      anti = hull[0];
    }
    else {
      int j = 1;
      for (int i = 0; i < hn; i++) {
        const Point_2 &a = hull[size_t(i)];
        const Point_2 &b = hull[size_t((i + 1) % hn)];
        while (dist_line(a, b, hull[size_t((j + 1) % hn)]) + 1e-15 >=
               dist_line(a, b, hull[size_t(j)]))
        {
          j = (j + 1) % hn;
          if (j == i) {
            break;
          }
        }
        const double w = dist_line(a, b, hull[size_t(j)]);
        if (w < best_w) {
          best_w = w;
          e0 = a;
          e1 = b;
          anti = hull[size_t(j)];
        }
      }
    }

    if (!std::isfinite(best_w)) {
      result.error = "Min Width 2D failed to compute width";
      return result;
    }

    const float mid = mid_dropped_axis(positions, n, plane);
    const double ax = CGAL::to_double(e0.x()), ay = CGAL::to_double(e0.y());
    const double bx = CGAL::to_double(e1.x()), by = CGAL::to_double(e1.y());
    double ex = bx - ax, ey = by - ay;
    double el = std::hypot(ex, ey);
    if (el < 1e-20) {
      ex = 1.0;
      ey = 0.0;
      el = 1.0;
    }
    ex /= el;
    ey /= el;
    const double nx = -ey;
    const double ny = ex;
    const double px = CGAL::to_double(anti.x());
    const double py = CGAL::to_double(anti.y());
    const double side = (px - ax) * nx + (py - ay) * ny;
    const double sx = (side >= 0.0) ? nx : -nx;
    const double sy = (side >= 0.0) ? ny : -ny;

    double cx = 0.0, cy = 0.0;
    for (const Point_2 &p : hull) {
      cx += CGAL::to_double(p.x());
      cy += CGAL::to_double(p.y());
    }
    cx /= double(hn);
    cy /= double(hn);

    const double half = double(plane_scale);
    const double mx0 = cx - ex * half;
    const double my0 = cy - ey * half;
    const double mx1 = cx + ex * half;
    const double my1 = cy + ey * half;
    const double d0 = (ax - cx) * sx + (ay - cy) * sy;
    const double d1 = d0 + ((best_w > 0.0) ? best_w : 0.0);

    auto push_pt = [&](double x, double y) {
      float q[3];
      unproject(x, y, plane, mid, q);
      result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
    };
    push_pt(mx0 + sx * d0, my0 + sy * d0);
    push_pt(mx1 + sx * d0, my1 + sy * d0);
    push_pt(mx1 + sx * d1, my1 + sy * d1);
    push_pt(mx0 + sx * d1, my0 + sy * d1);
    result.corner_verts = {0, 1, 2, 3};
    result.face_offsets = {0, 4};
    result.radius = float(best_w);
    result.alpha_used = best_w;
    float c3[3];
    unproject(cx + sx * (d0 + d1) * 0.5, cy + sy * (d0 + d1) * 0.5, plane, mid, c3);
    result.center[0] = c3[0];
    result.center[1] = c3[1];
    result.center[2] = c3[2];
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Min Width 2D failed";
  }
  return result;
}

MeshResult points_periodic_delaunay_2(const float *positions,
                                      int n,
                                      int plane,
                                      double domain_x,
                                      double domain_y)
{
  MeshResult result;
  if (!positions || n < 3) {
    result.error = "Periodic Delaunay 2D needs at least 3 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    using Gt = CGAL::Periodic_2_Delaunay_triangulation_traits_2<Kernel>;
    using PDT = CGAL::Periodic_2_Delaunay_triangulation_2<Gt>;

    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    std::vector<Point_2> raw;
    raw.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      const Point_2 p = project_pt(positions + i * 3, plane);
      raw.push_back(p);
      minx = std::min(minx, CGAL::to_double(p.x()));
      miny = std::min(miny, CGAL::to_double(p.y()));
      maxx = std::max(maxx, CGAL::to_double(p.x()));
      maxy = std::max(maxy, CGAL::to_double(p.y()));
    }
    const double spanx = std::max(1e-6, maxx - minx);
    const double spany = std::max(1e-6, maxy - miny);
    double wx = domain_x;
    double wy = domain_y;
    if (!(wx > 0.0)) {
      wx = spanx * 1.05;
    }
    if (!(wy > 0.0)) {
      wy = spany * 1.05;
    }
    wx = std::max(wx, 1e-6);
    wy = std::max(wy, 1e-6);
    const double ox = minx - (wx - spanx) * 0.5;
    const double oy = miny - (wy - spany) * 0.5;

    auto wrap = [](double v, double o, double w) {
      double t = std::fmod(v - o, w);
      if (t < 0.0) {
        t += w;
      }
      /* Keep strictly inside [o, o+w). */
      if (t >= w - 1e-12) {
        t = 0.0;
      }
      if (t < 1e-12) {
        t = 1e-12;
      }
      return o + t;
    };

    PDT::Iso_rectangle domain(ox, oy, ox + wx, oy + wy);
    PDT T(domain);
    for (const Point_2 &p : raw) {
      T.insert(Point_2(wrap(CGAL::to_double(p.x()), ox, wx),
                       wrap(CGAL::to_double(p.y()), oy, wy)));
    }
    if (T.dimension() != 2) {
      result.error = "Periodic Delaunay 2D degenerate";
      return result;
    }
    if (T.is_triangulation_in_1_sheet()) {
      T.convert_to_1_sheeted_covering();
    }

    const float mid = mid_dropped_axis(positions, n, plane);
    for (auto fit = T.finite_faces_begin(); fit != T.finite_faces_end(); ++fit) {
      const int base = result.verts_num();
      for (int i = 0; i < 3; i++) {
        const Point_2 p = T.point(fit, i);
        float q[3];
        unproject(CGAL::to_double(p.x()), CGAL::to_double(p.y()), plane, mid, q);
        result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
        result.corner_verts.push_back(base + i);
      }
    }
    result.ok = result.corners_num() >= 3;
    if (!result.ok) {
      result.error = "Periodic Delaunay 2D produced no faces";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Periodic Delaunay 2D failed";
  }
  return result;
}

WireResult mesh_constrained_voronoi_2(const MeshIn &mesh)
{
  WireResult result;
  const int plane = 0;
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, plane, islands, mid, error) || islands.empty()) {
      result.error = error.empty() ? "Constrained Voronoi 2D needs a polygon" : error;
      return result;
    }

    using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_2<CGAL::Triangulation_vertex_base_2<Kernel>,
                                                     Fb>;
    using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, Tds, CGAL::Exact_predicates_tag>;

    CDT cdt;
    for (const PolyIsland &isl : islands) {
      auto insert_ring = [&](const std::vector<Point_2> &ring) {
        std::vector<CDT::Vertex_handle> vhs;
        vhs.reserve(ring.size());
        for (const Point_2 &p : ring) {
          vhs.push_back(cdt.insert(p));
        }
        const size_t n = vhs.size();
        for (size_t i = 0; i < n; i++) {
          if (vhs[i] != vhs[(i + 1) % n]) {
            cdt.insert_constraint(vhs[i], vhs[(i + 1) % n]);
          }
        }
      };
      insert_ring(isl.outer);
      for (const auto &h : isl.holes) {
        insert_ring(h);
      }
    }
    if (cdt.dimension() != 2) {
      result.error = "Constrained Voronoi 2D degenerate";
      return result;
    }
    std::map<typename CDT::Face_handle, bool> in_domain_map;
    boost::associative_property_map<std::map<typename CDT::Face_handle, bool>> in_domain(
        in_domain_map);
    CGAL::mark_domain_in_triangulation(cdt, in_domain);

    auto circum = [&](typename CDT::Face_handle f) -> Point_2 {
      return cdt.circumcenter(f);
    };
    auto midpoint = [](const Point_2 &a, const Point_2 &b) {
      return Point_2((CGAL::to_double(a.x()) + CGAL::to_double(b.x())) * 0.5,
                     (CGAL::to_double(a.y()) + CGAL::to_double(b.y())) * 0.5);
    };

    std::set<std::pair<void *, void *>> seen;
    for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
      if (!get(in_domain, fit)) {
        continue;
      }
      Point_2 c1;
      try {
        c1 = circum(fit);
      }
      catch (...) {
        continue;
      }
      for (int i = 0; i < 3; i++) {
        CDT::Face_handle nb = fit->neighbor(i);
        if (cdt.is_infinite(nb) || !in_domain[nb]) {
          const Point_2 a = fit->vertex(cdt.cw(i))->point();
          const Point_2 b = fit->vertex(cdt.ccw(i))->point();
          emit_wire_seg(result, c1, midpoint(a, b), plane, mid);
          continue;
        }
        void *lo = static_cast<void *>(&*fit);
        void *hi = static_cast<void *>(&*nb);
        if (hi < lo) {
          std::swap(lo, hi);
        }
        if (!seen.insert({lo, hi}).second) {
          continue;
        }
        try {
          emit_wire_seg(result, c1, circum(nb), plane, mid);
        }
        catch (...) {
        }
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Constrained Voronoi 2D produced no edges";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Constrained Voronoi 2D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
