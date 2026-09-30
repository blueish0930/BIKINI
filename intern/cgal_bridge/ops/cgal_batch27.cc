/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 27:
 *  - Min Ellipse / Parallelogram / Convex Hull 2D
 *  - Mesh Plane Slicer, VCM Feature Edges
 *  - Delaunay / Alpha Shape / Voronoi 2D
 *  - Straight Skeleton 2D
 *  - Polygon Offset 2D (Skeleton | Euclidean) — clean CGAL re-port
 *  - Medial Axis 2D
 */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Approximate_min_ellipsoid_d.h>
#include <CGAL/Approximate_min_ellipsoid_d_traits_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/min_quadrilateral_2.h>
#include <CGAL/Default_diagonalize_traits.h>
#include <CGAL/vcm_estimate_edges.h>
#include <CGAL/vcm_estimate_normals.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/assertions_behaviour.h>

#include <boost/property_map/property_map.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <type_traits>
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

namespace blender::cgal_bridge {
namespace {

using Point = Point_3;
using Point_2 = Kernel::Point_2;
using Plane_3 = Kernel::Plane_3;
using Vector_3 = Kernel::Vector_3;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using Polygon_with_holes_2 = CGAL::Polygon_with_holes_2<Kernel>;

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
  const int axis = (plane == 0) ? 2 : (plane == 1) ? 1 : 0;
  double s = 0.0;
  for (int i = 0; i < n; i++) {
    s += double(positions[i * 3 + axis]);
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
  int hits = 0, tests = 0;
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
  /**
   * Wire contour (no faces): offset as a strip to BOTH sides of the polyline.
   * Not a filled region. Open or closed (rectangle border lines, Bezier, etc.).
   */
  bool is_polyline = false;
  bool polyline_closed = false;
};

struct Contour2 {
  std::vector<Point_2> pts;
  bool closed = false;
};

static void collect_wire_contours(const MeshIn &mesh, int plane, std::vector<Contour2> &contours)
{
  const int *e0 = !mesh.owned_edge_v0.empty() ? mesh.owned_edge_v0.data() : mesh.edge_v0;
  const int *e1 = !mesh.owned_edge_v1.empty() ? mesh.owned_edge_v1.data() : mesh.edge_v1;
  const int nE = !mesh.owned_edge_v0.empty() ? int(mesh.owned_edge_v0.size()) : mesh.edges_num;
  if (!e0 || !e1 || nE <= 0 || !mesh.positions || mesh.verts_num < 2) {
    return;
  }

  struct Link {
    int to = -1;
    int ei = -1;
  };
  std::vector<std::vector<Link>> adj(size_t(mesh.verts_num));
  std::vector<char> edge_used(size_t(nE), 0);
  std::vector<int> degree(size_t(mesh.verts_num), 0);

  for (int i = 0; i < nE; i++) {
    const int a = e0[i], b = e1[i];
    if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
      edge_used[size_t(i)] = 1;
      continue;
    }
    adj[size_t(a)].push_back(Link{b, i});
    adj[size_t(b)].push_back(Link{a, i});
    degree[size_t(a)]++;
    degree[size_t(b)]++;
  }

  auto emit_path = [&](const std::vector<int> &path_idx, bool closed) {
    if (path_idx.size() < (closed ? 3u : 2u)) {
      return;
    }
    std::vector<int> idx = path_idx;
    if (closed && idx.size() >= 2 && idx.front() == idx.back()) {
      idx.pop_back();
    }
    if (idx.size() < (closed ? 3u : 2u)) {
      return;
    }
    Contour2 c;
    c.closed = closed;
    for (int vi : idx) {
      c.pts.push_back(project_pt(mesh.positions + vi * 3, plane));
    }
    if (closed) {
      c.pts = clean_loop_2(c.pts);
      if (c.pts.size() < 3 || std::abs(signed_area_2(c.pts)) < 1e-20) {
        return;
      }
    }
    else {
      std::vector<Point_2> cleaned;
      for (const Point_2 &q : c.pts) {
        if (cleaned.empty()) {
          cleaned.push_back(q);
          continue;
        }
        const double dx = CGAL::to_double(q.x() - cleaned.back().x());
        const double dy = CGAL::to_double(q.y() - cleaned.back().y());
        if (dx * dx + dy * dy > 1e-24) {
          cleaned.push_back(q);
        }
      }
      if (cleaned.size() < 2) {
        return;
      }
      c.pts = std::move(cleaned);
      c.closed = false;
    }
    contours.push_back(std::move(c));
  };

  std::vector<int> seed_order;
  for (int i = 0; i < nE; i++) {
    if (edge_used[size_t(i)]) {
      continue;
    }
    const int a = e0[i], b = e1[i];
    if (degree[size_t(a)] == 1 || degree[size_t(b)] == 1) {
      seed_order.push_back(i);
    }
  }
  for (int i = 0; i < nE; i++) {
    if (!edge_used[size_t(i)]) {
      seed_order.push_back(i);
    }
  }

  for (int seed_ei : seed_order) {
    if (edge_used[size_t(seed_ei)]) {
      continue;
    }
    int a0 = e0[seed_ei], b0 = e1[seed_ei];
    if (degree[size_t(b0)] == 1 && degree[size_t(a0)] != 1) {
      std::swap(a0, b0);
    }
    std::vector<int> path;
    path.push_back(a0);
    int cur = b0, last_ei = seed_ei;
    edge_used[size_t(seed_ei)] = 1;
    bool closed = false;
    for (int step = 0; step < mesh.verts_num + 8; step++) {
      path.push_back(cur);
      if (cur == a0 && path.size() >= 4) {
        closed = true;
        break;
      }
      int next_v = -1, next_ei = -1;
      for (const Link &lk : adj[size_t(cur)]) {
        if (lk.ei == last_ei || edge_used[size_t(lk.ei)]) {
          continue;
        }
        next_v = lk.to;
        next_ei = lk.ei;
        break;
      }
      if (next_v < 0) {
        break;
      }
      edge_used[size_t(next_ei)] = 1;
      cur = next_v;
      last_ei = next_ei;
    }
    emit_path(path, closed);
  }
}

static bool extract_polygon_islands(const MeshIn &mesh,
                                    int plane,
                                    std::vector<PolyIsland> &islands,
                                    float &mid_drop,
                                    std::string &error)
{
  islands.clear();
  if (!mesh.positions || mesh.verts_num < 2) {
    error = "Need vertices";
    return false;
  }
  mid_drop = mid_dropped_axis(mesh.positions, mesh.verts_num, plane);

  struct Loop {
    std::vector<Point_2> verts;
    double area_abs = 0.0;
  };
  auto make_loop = [](std::vector<Point_2> verts) -> Loop {
    Loop L;
    L.verts = clean_loop_2(std::move(verts));
    L.area_abs = std::abs(signed_area_2(L.verts));
    return L;
  };

  Surface_mesh sm;
  std::string load_err;
  std::vector<Loop> closed_loops;
  const bool has_faces = (mesh.faces_num > 0 && mesh.corners_num >= 3);
  const bool wire_only = !has_faces;

  if (has_faces && load_tri(mesh, sm, load_err) && sm.number_of_faces() > 0) {
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
        if (raw.size() > size_t(mesh.verts_num) + 16) {
          break;
        }
      }
      Loop L = make_loop(std::move(raw));
      if (L.verts.size() >= 3 && L.area_abs > 1e-20) {
        closed_loops.push_back(std::move(L));
      }
    }
  }

  /*
   * Wire-only (no faces): every open chain AND closed edge ring is a *polyline*
   * for bilateral strip offset — never treat a rectangle of edges as a filled
   * solid (that would close the interior then expand only outward).
   *
   * Mesh with faces: border cycles are filled-region polygons (holes nested).
   */
  if (wire_only) {
    std::vector<Contour2> wire;
    collect_wire_contours(mesh, plane, wire);
    for (Contour2 &c : wire) {
      if (c.pts.size() < (c.closed ? 3u : 2u)) {
        continue;
      }
      PolyIsland isl;
      isl.outer = std::move(c.pts);
      isl.is_polyline = true;
      isl.polyline_closed = c.closed;
      islands.push_back(std::move(isl));
    }
  }
  else if (closed_loops.empty()) {
    /* Faces failed to produce borders: fall back to wire walk as polylines. */
    std::vector<Contour2> wire;
    collect_wire_contours(mesh, plane, wire);
    for (Contour2 &c : wire) {
      if (c.pts.size() < (c.closed ? 3u : 2u)) {
        continue;
      }
      PolyIsland isl;
      isl.outer = std::move(c.pts);
      isl.is_polyline = true;
      isl.polyline_closed = c.closed;
      islands.push_back(std::move(isl));
    }
  }
  else {
    /* Face borders → filled islands (+ nested holes). */
    std::sort(closed_loops.begin(), closed_loops.end(), [](const Loop &a, const Loop &b) {
      return a.area_abs > b.area_abs;
    });
    std::vector<char> used(closed_loops.size(), 0);
    for (size_t oi = 0; oi < closed_loops.size(); oi++) {
      if (used[oi]) {
        continue;
      }
      used[oi] = 1;
      PolyIsland isl;
      isl.outer = closed_loops[oi].verts;
      if (signed_area_2(isl.outer) < 0.0) {
        std::reverse(isl.outer.begin(), isl.outer.end());
      }
      for (size_t hi = oi + 1; hi < closed_loops.size(); hi++) {
        if (used[hi] || !loop_inside_loop(closed_loops[hi].verts, isl.outer)) {
          continue;
        }
        used[hi] = 1;
        std::vector<Point_2> hole = closed_loops[hi].verts;
        if (signed_area_2(hole) > 0.0) {
          std::reverse(hole.begin(), hole.end());
        }
        isl.holes.push_back(std::move(hole));
      }
      islands.push_back(std::move(isl));
    }
  }

  if (islands.empty() && has_faces) {
    std::vector<Point_2> pts;
    for (int i = 0; i < mesh.verts_num; i++) {
      pts.push_back(project_pt(mesh.positions + i * 3, plane));
    }
    std::vector<Point_2> hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
    Loop L = make_loop(std::move(hull));
    if (L.verts.size() < 3) {
      error = load_err.empty() ? "No borders or closed edge rings" : load_err;
      return false;
    }
    PolyIsland isl;
    isl.outer = std::move(L.verts);
    if (signed_area_2(isl.outer) < 0.0) {
      std::reverse(isl.outer.begin(), isl.outer.end());
    }
    islands.push_back(std::move(isl));
  }

  if (islands.empty()) {
    error = "No closed rings or open edge chains on XY";
    return false;
  }
  return true;
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
  try {
    using Vb = CGAL::Triangulation_vertex_base_2<Kernel>;
    using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
    using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, CGAL::Exact_predicates_tag>;

    const Polygon_2 &outer = pwh.outer_boundary();
    if (outer.size() < 3) {
      return;
    }
    try {
      if (!outer.is_simple()) {
        return;
      }
    }
    catch (...) {
      return;
    }

    CDT cdt;
    auto insert_ring = [&](const Polygon_2 &poly) {
      if (poly.size() < 3) {
        return;
      }
      try {
        if (!poly.is_simple()) {
          return;
        }
      }
      catch (...) {
        return;
      }
      std::vector<typename CDT::Vertex_handle> vhs;
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
        if (vhs[size_t(i)] == vhs[size_t((i + 1) % n)]) {
          continue;
        }
        try {
          cdt.insert_constraint(vhs[size_t(i)], vhs[size_t((i + 1) % n)]);
        }
        catch (...) {
        }
      }
    };

    insert_ring(outer);
    for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
      insert_ring(*hit);
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
    auto vert_index = [&](typename CDT::Vertex_handle vh) {
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

    for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
      if (!get(in_domain, fit)) {
        continue;
      }
      const int i0 = vert_index(fit->vertex(0));
      const int i1 = vert_index(fit->vertex(1));
      const int i2 = vert_index(fit->vertex(2));
      if (i0 == i1 || i1 == i2 || i2 == i0) {
        continue;
      }
      result.corner_verts.push_back(i0);
      result.corner_verts.push_back(i1);
      result.corner_verts.push_back(i2);
    }
  }
  catch (...) {
  }
}

static void append_ellipse_ngon(float cx,
                                float cy,
                                float cz,
                                double a_len,
                                double b_len,
                                double dir_ax,
                                double dir_ay,
                                double dir_bx,
                                double dir_by,
                                int /*plane*/,
                                int segments,
                                MeshResult &result)
{
  segments = std::clamp(segments, 8, 128);
  result.positions.clear();
  result.corner_verts.clear();
  result.face_offsets.clear();
  result.face_offsets.push_back(0);
  for (int i = 0; i < segments; i++) {
    const double t = 2.0 * M_PI * double(i) / double(segments);
    const double ca = std::cos(t), sa = std::sin(t);
    const double x = double(cx) + a_len * ca * dir_ax + b_len * sa * dir_bx;
    const double y = double(cy) + a_len * ca * dir_ay + b_len * sa * dir_by;
    result.positions.push_back(float(x));
    result.positions.push_back(float(y));
    result.positions.push_back(cz);
    result.corner_verts.push_back(i);
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.ok = result.corners_num() >= 3;
}

/* -------------------------------------------------------------------------- */
/* Polygon Offset 2D — straight-skeleton only (CGAL)                          */
/* -------------------------------------------------------------------------- */

static Polygon_2 offset_prepare_ccw(const std::vector<Point_2> &raw_in)
{
  std::vector<Point_2> pts = clean_loop_2(raw_in);
  if (pts.size() < 3) {
    return Polygon_2();
  }
  if (signed_area_2(pts) < 0.0) {
    std::reverse(pts.begin(), pts.end());
  }
  if (pts.size() >= 3) {
    std::vector<Point_2> simp;
    const int n = int(pts.size());
    for (int i = 0; i < n; i++) {
      const Point_2 &a = pts[size_t((i + n - 1) % n)];
      const Point_2 &b = pts[size_t(i)];
      const Point_2 &c = pts[size_t((i + 1) % n)];
      const double abx = CGAL::to_double(b.x() - a.x());
      const double aby = CGAL::to_double(b.y() - a.y());
      const double bcx = CGAL::to_double(c.x() - b.x());
      const double bcy = CGAL::to_double(c.y() - b.y());
      const double cross = abx * bcy - aby * bcx;
      const double ab = std::sqrt(abx * abx + aby * aby);
      const double bc = std::sqrt(bcx * bcx + bcy * bcy);
      if (ab < 1e-14 || bc < 1e-14) {
        continue;
      }
      if (std::abs(cross) < 1e-9 * ab * bc) {
        continue;
      }
      simp.push_back(b);
    }
    if (simp.size() >= 3) {
      pts = std::move(simp);
    }
  }
  if (pts.size() < 3) {
    return Polygon_2();
  }
  Polygon_2 poly(pts.begin(), pts.end());
  if (poly.is_clockwise_oriented()) {
    poly.reverse_orientation();
  }
  return poly;
}

static Polygon_2 offset_prepare_ccw_poly(const Polygon_2 &p)
{
  return offset_prepare_ccw(std::vector<Point_2>(p.vertices_begin(), p.vertices_end()));
}

static double offset_poly_area_abs(const Polygon_2 &poly)
{
  try {
    return std::abs(CGAL::to_double(poly.area()));
  }
  catch (...) {
    std::vector<Point_2> pts(poly.vertices_begin(), poly.vertices_end());
    return std::abs(signed_area_2(pts));
  }
}

/**
 * Simplify for offset: merge short edges relative to r, then hard-cap verts.
 * Critical for letter outlines / dense Bezier — prevents CGAL hang.
 */
static Polygon_2 offset_decimate(Polygon_2 poly, size_t max_verts, double r = 0.0)
{
  if (poly.size() < 3) {
    return poly;
  }
  std::vector<Point_2> pts(poly.vertices_begin(), poly.vertices_end());
  pts = clean_loop_2(pts);
  if (pts.size() < 3) {
    return Polygon_2();
  }

  const double diag = [&]() {
    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (const Point_2 &p : pts) {
      minx = std::min(minx, CGAL::to_double(p.x()));
      miny = std::min(miny, CGAL::to_double(p.y()));
      maxx = std::max(maxx, CGAL::to_double(p.x()));
      maxy = std::max(maxy, CGAL::to_double(p.y()));
    }
    return std::sqrt((maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny));
  }();

  /* Edge collapse threshold grows with r so large offsets don't keep hairline detail. */
  const double min_e = std::max(1e-9,
                                std::min(std::max(r * 0.08, diag * 1e-3), diag * 0.04));
  const double min_e2 = min_e * min_e;
  {
    std::vector<Point_2> merged;
    merged.reserve(pts.size());
    for (const Point_2 &q : pts) {
      if (merged.empty()) {
        merged.push_back(q);
        continue;
      }
      const double dx = CGAL::to_double(q.x() - merged.back().x());
      const double dy = CGAL::to_double(q.y() - merged.back().y());
      if (dx * dx + dy * dy >= min_e2) {
        merged.push_back(q);
      }
    }
    if (merged.size() >= 3) {
      const double dx = CGAL::to_double(merged.front().x() - merged.back().x());
      const double dy = CGAL::to_double(merged.front().y() - merged.back().y());
      if (dx * dx + dy * dy < min_e2) {
        merged.pop_back();
      }
    }
    if (merged.size() >= 3) {
      pts = std::move(merged);
    }
  }

  if (max_verts >= 3 && pts.size() > max_verts) {
    std::vector<Point_2> dec;
    dec.reserve(max_verts);
    for (size_t i = 0; i < max_verts; i++) {
      dec.push_back(pts[(i * pts.size()) / max_verts]);
    }
    pts = std::move(dec);
  }
  return offset_prepare_ccw(pts);
}

static void offset_append_pwh(MeshResult &result,
                              const Polygon_with_holes_2 &pwh,
                              int plane,
                              float mid)
{
  append_pwh_triangulated(result, pwh, plane, mid);
}

static void offset_append_poly(MeshResult &result, Polygon_2 poly, int plane, float mid)
{
  if (poly.size() < 3) {
    return;
  }
  if (poly.is_clockwise_oriented()) {
    poly.reverse_orientation();
  }
  /* Refuse non-simple — CDT would create peripheral junk faces. */
  try {
    if (!poly.is_simple()) {
      return;
    }
  }
  catch (...) {
    return;
  }
  offset_append_pwh(result, Polygon_with_holes_2(poly), plane, mid);
}

static std::vector<Point_2> offset_clean_chain(const std::vector<Point_2> &chain_in, bool closed)
{
  std::vector<Point_2> p;
  for (const Point_2 &q : chain_in) {
    if (p.empty()) {
      p.push_back(q);
      continue;
    }
    const double dx = CGAL::to_double(q.x() - p.back().x());
    const double dy = CGAL::to_double(q.y() - p.back().y());
    if (dx * dx + dy * dy > 1e-20) {
      p.push_back(q);
    }
  }
  if (closed && p.size() >= 2) {
    const double dx = CGAL::to_double(p.front().x() - p.back().x());
    const double dy = CGAL::to_double(p.front().y() - p.back().y());
    if (dx * dx + dy * dy < 1e-20) {
      p.pop_back();
    }
  }
  return p;
}

static bool offset_unit(double x, double y, double &ox, double &oy)
{
  const double len = std::sqrt(x * x + y * y);
  if (len < 1e-14) {
    ox = 1.0;
    oy = 0.0;
    return false;
  }
  ox = x / len;
  oy = y / len;
  return true;
}

static bool offset_line_isect(double ax,
                              double ay,
                              double adx,
                              double ady,
                              double bx,
                              double by,
                              double bdx,
                              double bdy,
                              double &ox,
                              double &oy)
{
  const double c = adx * bdy - ady * bdx;
  if (std::abs(c) < 1e-14) {
    return false;
  }
  const double dx = bx - ax;
  const double dy = by - ay;
  const double s = (dx * bdy - dy * bdx) / c;
  ox = ax + s * adx;
  oy = ay + s * ady;
  return std::isfinite(ox) && std::isfinite(oy);
}

/**
 * Build left/right offset polylines (half-width r, miter joins).
 * Does NOT form a filled region — callers mesh as a band only.
 */
static bool offset_build_sides(const std::vector<Point_2> &chain_in,
                               double r,
                               bool closed,
                               std::vector<Point_2> &left_side,
                               std::vector<Point_2> &right_side,
                               std::vector<double> &etx,
                               std::vector<double> &ety)
{
  left_side.clear();
  right_side.clear();
  etx.clear();
  ety.clear();
  std::vector<Point_2> p = offset_clean_chain(chain_in, closed);
  if (closed) {
    if (p.size() < 3 || !(r > 0.0)) {
      return false;
    }
  }
  else if (p.size() < 2 || !(r > 0.0)) {
    return false;
  }

  const size_t maxn = closed ? 256u : 128u;
  if (p.size() > maxn) {
    std::vector<Point_2> dec;
    dec.reserve(maxn);
    if (closed) {
      for (size_t i = 0; i < maxn; i++) {
        dec.push_back(p[(i * p.size()) / maxn]);
      }
    }
    else {
      for (size_t i = 0; i < maxn; i++) {
        dec.push_back(p[(i * (p.size() - 1)) / (maxn - 1)]);
      }
    }
    p.swap(dec);
  }

  const int n = int(p.size());
  const int nE = closed ? n : (n - 1);
  etx.assign(size_t(nE), 0.0);
  ety.assign(size_t(nE), 0.0);
  std::vector<double> enx(size_t(nE), 0.0);
  std::vector<double> eny(size_t(nE), 0.0);
  for (int i = 0; i < nE; i++) {
    const Point_2 &a = p[size_t(i)];
    const Point_2 &b = p[size_t((i + 1) % n)];
    offset_unit(CGAL::to_double(b.x() - a.x()),
                CGAL::to_double(b.y() - a.y()),
                etx[size_t(i)],
                ety[size_t(i)]);
    enx[size_t(i)] = -ety[size_t(i)];
    eny[size_t(i)] = etx[size_t(i)];
  }

  const double miter_limit = 4.0;
  auto side_at_vertex = [&](int vi, double side_sign, double &ox, double &oy) -> void {
    int e_in, e_out;
    if (closed) {
      e_in = (vi + n - 1) % n;
      e_out = vi;
    }
    else {
      if (vi == 0) {
        e_in = e_out = 0;
      }
      else if (vi == n - 1) {
        e_in = e_out = nE - 1;
      }
      else {
        e_in = vi - 1;
        e_out = vi;
      }
    }
    const double px = CGAL::to_double(p[size_t(vi)].x());
    const double py = CGAL::to_double(p[size_t(vi)].y());
    const double nix = side_sign * enx[size_t(e_in)];
    const double niy = side_sign * eny[size_t(e_in)];
    const double nox = side_sign * enx[size_t(e_out)];
    const double noy = side_sign * eny[size_t(e_out)];
    const double ax = px + r * nix, ay = py + r * niy;
    const double bx = px + r * nox, by = py + r * noy;
    if (e_in == e_out) {
      ox = ax;
      oy = ay;
      return;
    }
    double ix = 0, iy = 0;
    if (offset_line_isect(ax,
                          ay,
                          etx[size_t(e_in)],
                          ety[size_t(e_in)],
                          bx,
                          by,
                          etx[size_t(e_out)],
                          ety[size_t(e_out)],
                          ix,
                          iy))
    {
      const double mlen = std::hypot(ix - px, iy - py);
      if (mlen <= miter_limit * r && std::isfinite(ix) && std::isfinite(iy)) {
        ox = ix;
        oy = iy;
        return;
      }
    }
    /* Bevel fallback. */
    ox = 0.5 * (ax + bx);
    oy = 0.5 * (ay + by);
  };

  left_side.resize(size_t(n));
  right_side.resize(size_t(n));
  for (int i = 0; i < n; i++) {
    double lx = 0, ly = 0, rx = 0, ry = 0;
    side_at_vertex(i, +1.0, lx, ly);
    side_at_vertex(i, -1.0, rx, ry);
    left_side[size_t(i)] = Point_2(lx, ly);
    right_side[size_t(i)] = Point_2(rx, ry);
  }
  return true;
}

static int offset_push_vert(MeshResult &result, const Point_2 &p, int plane, float mid)
{
  const int idx = result.verts_num();
  float q[3];
  unproject(CGAL::to_double(p.x()), CGAL::to_double(p.y()), plane, mid, q);
  result.positions.push_back(q[0]);
  result.positions.push_back(q[1]);
  result.positions.push_back(q[2]);
  return idx;
}

static void offset_push_tri(MeshResult &result, int i0, int i1, int i2)
{
  if (i0 == i1 || i1 == i2 || i2 == i0) {
    return;
  }
  result.corner_verts.push_back(i0);
  result.corner_verts.push_back(i1);
  result.corner_verts.push_back(i2);
}

/**
 * Emit a hollow band: per segment two triangles L0-L1-R1 / L0-R1-R0.
 * Closed cyclic wires NEVER fill the interior — only the strip around the edges.
 * Open chains get rectangular end caps of length r along ±tangent.
 */
static void offset_emit_quad_strip(MeshResult &result,
                                   const std::vector<Point_2> &left_side,
                                   const std::vector<Point_2> &right_side,
                                   const std::vector<double> &etx,
                                   const std::vector<double> &ety,
                                   double r,
                                   bool closed,
                                   int plane,
                                   float mid)
{
  const int n = int(left_side.size());
  if (n < 2 || int(right_side.size()) != n) {
    return;
  }
  const int nE = closed ? n : (n - 1);
  if (nE < 1 || int(etx.size()) < nE || int(ety.size()) < nE) {
    return;
  }

  /* Vertices: L[0..n-1], R[0..n-1], then optional open end-cap verts. */
  std::vector<int> Li(size_t(n), -1);
  std::vector<int> Ri(size_t(n), -1);
  for (int i = 0; i < n; i++) {
    Li[size_t(i)] = offset_push_vert(result, left_side[size_t(i)], plane, mid);
    Ri[size_t(i)] = offset_push_vert(result, right_side[size_t(i)], plane, mid);
  }

  auto emit_quad = [&](int l0, int l1, int r1, int r0) {
    /* Two triangles covering the strip segment (L0-L1-R1-R0). */
    offset_push_tri(result, l0, l1, r1);
    offset_push_tri(result, l0, r1, r0);
  };

  for (int i = 0; i < nE; i++) {
    const int j = closed ? ((i + 1) % n) : (i + 1);
    emit_quad(Li[size_t(i)], Li[size_t(j)], Ri[size_t(j)], Ri[size_t(i)]);
  }

  if (!closed) {
    /* Start cap: extend -tangent by r from first cross-section. */
    {
      const double tx = etx[0], ty = ety[0];
      const double lx = CGAL::to_double(left_side[0].x());
      const double ly = CGAL::to_double(left_side[0].y());
      const double rx = CGAL::to_double(right_side[0].x());
      const double ry = CGAL::to_double(right_side[0].y());
      const int lcap = offset_push_vert(result, Point_2(lx - r * tx, ly - r * ty), plane, mid);
      const int rcap = offset_push_vert(result, Point_2(rx - r * tx, ry - r * ty), plane, mid);
      emit_quad(lcap, Li[0], Ri[0], rcap);
    }
    /* End cap: extend +tangent by r from last cross-section. */
    {
      const int e = nE - 1;
      const double tx = etx[size_t(e)], ty = ety[size_t(e)];
      const double lx = CGAL::to_double(left_side[size_t(n - 1)].x());
      const double ly = CGAL::to_double(left_side[size_t(n - 1)].y());
      const double rx = CGAL::to_double(right_side[size_t(n - 1)].x());
      const double ry = CGAL::to_double(right_side[size_t(n - 1)].y());
      const int lcap = offset_push_vert(
          result, Point_2(lx + r * tx, ly + r * ty), plane, mid);
      const int rcap = offset_push_vert(
          result, Point_2(rx + r * tx, ry + r * ty), plane, mid);
      emit_quad(Li[size_t(n - 1)], lcap, rcap, Ri[size_t(n - 1)]);
    }
  }
}

static void offset_skeleton_closed(MeshResult &result,
                                   const Polygon_with_holes_2 &pwh_in,
                                   double r,
                                   bool outward,
                                   int plane,
                                   float mid)
{
  try {
    Polygon_2 outer = offset_decimate(offset_prepare_ccw_poly(pwh_in.outer_boundary()), 200, r);
    if (outer.size() < 3) {
      return;
    }
    try {
      if (!outer.is_simple()) {
        return;
      }
    }
    catch (...) {
      return;
    }
    std::vector<Polygon_2> holes;
    for (auto hit = pwh_in.holes_begin(); hit != pwh_in.holes_end(); ++hit) {
      Polygon_2 h = offset_decimate(offset_prepare_ccw_poly(*hit), 100, r);
      if (h.size() < 3) {
        continue;
      }
      if (h.is_counterclockwise_oriented()) {
        h.reverse_orientation();
      }
      holes.push_back(std::move(h));
    }
    Polygon_with_holes_2 pwh(outer, holes.begin(), holes.end());
    Kernel k;
    std::vector<std::shared_ptr<Polygon_with_holes_2>> pwhs;
    if (outward) {
      pwhs = CGAL::create_exterior_skeleton_and_offset_polygons_with_holes_2(r, pwh, k, k);
    }
    else {
      pwhs = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(r, pwh, k, k);
    }
    /* Keep only significant components (drop tiny SS debris). */
    double max_a = 0.0;
    std::vector<std::shared_ptr<Polygon_with_holes_2>> keep;
    for (const auto &sp : pwhs) {
      if (!sp || sp->outer_boundary().size() < 3) {
        continue;
      }
      const double a = offset_poly_area_abs(sp->outer_boundary());
      max_a = std::max(max_a, a);
      keep.push_back(sp);
    }
    for (const auto &sp : keep) {
      const double a = offset_poly_area_abs(sp->outer_boundary());
      if (max_a > 0.0 && a < max_a * 1e-4) {
        continue;
      }
      offset_append_pwh(result, *sp, plane, mid);
    }
  }
  catch (...) {
  }
}

/**
 * Wire-only polyline: |offset| = half-width to BOTH sides of each edge.
 * Closed (cyclic rectangle borders, etc.): hollow frame ONLY — never solid-fill
 * the interior. Open: strip with rectangular end caps.
 */
static void offset_polyline_chain(MeshResult &result,
                                  const std::vector<Point_2> &chain,
                                  double r,
                                  bool closed,
                                  int plane,
                                  float mid)
{
  if (!(r > 0.0)) {
    return;
  }
  try {
    std::vector<Point_2> left_side, right_side;
    std::vector<double> etx, ety;
    if (!offset_build_sides(chain, r, closed, left_side, right_side, etx, ety)) {
      return;
    }
    /* Pure band mesh — no PWH, no CDT, no solid disk. */
    offset_emit_quad_strip(
        result, left_side, right_side, etx, ety, r, closed, plane, mid);
  }
  catch (...) {
  }
}

}  // namespace

/* ========================================================================== */
/* Public APIs                                                                */
/* ========================================================================== */

MeshResult points_min_ellipse_2(const float *positions, int n, int plane, int segments)
{
  MeshResult result;
  if (!positions || n < 3) {
    result.error = "Min Ellipse 2D needs at least 3 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  segments = std::clamp(segments, 8, 128);
  try {
    using Traits = CGAL::Approximate_min_ellipsoid_d_traits_2<Kernel, double>;
    using Point_d = Traits::Point;
    using Min_ell = CGAL::Approximate_min_ellipsoid_d<Traits>;
    std::vector<Point_d> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      Point_2 p = project_pt(positions + i * 3, plane);
      pts.push_back(Point_d(CGAL::to_double(p.x()), CGAL::to_double(p.y())));
    }
    Traits traits;
    Min_ell me(0.01, pts.begin(), pts.end(), traits);
    if (!me.is_full_dimensional()) {
      result.error = "Min Ellipse 2D: points are degenerate";
      return result;
    }
    const float mid = mid_dropped_axis(positions, n, plane);
    auto c_it = me.center_cartesian_begin();
    const double cx = *c_it;
    const double cy = *(c_it + 1);
    auto al = me.axes_lengths_begin();
    const double a0 = *al;
    const double a1 = *(al + 1);
    auto d0 = me.axis_direction_cartesian_begin(0);
    auto d1 = me.axis_direction_cartesian_begin(1);
    const double d0x = *d0, d0y = *(d0 + 1);
    const double d1x = *d1, d1y = *(d1 + 1);
    float c3[3];
    unproject(cx, cy, plane, mid, c3);
    result.center[0] = c3[0];
    result.center[1] = c3[1];
    result.center[2] = c3[2];
    result.radius = float(std::max(a0, a1));
    append_ellipse_ngon(
        c3[0], c3[1], c3[2], a0, a1, d0x, d0y, d1x, d1y, plane, segments, result);
    result.ok = result.corners_num() >= 3;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Min Ellipse 2D failed";
  }
  return result;
}

MeshResult points_min_parallelogram_2(const float *positions, int n, int plane)
{
  MeshResult result;
  if (!positions || n < 3) {
    result.error = "Min Parallelogram 2D needs at least 3 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    std::vector<Point_2> pts;
    for (int i = 0; i < n; i++) {
      pts.push_back(project_pt(positions + i * 3, plane));
    }
    std::vector<Point_2> hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
    if (hull.size() < 3) {
      result.error = "Min Parallelogram 2D: degenerate hull";
      return result;
    }
    Polygon_2 para;
    CGAL::min_parallelogram_2(hull.begin(), hull.end(), std::back_inserter(para));
    if (para.size() < 3) {
      result.error = "Min Parallelogram 2D empty";
      return result;
    }
    const float mid = mid_dropped_axis(positions, n, plane);
    std::vector<Point_2> ring(para.vertices_begin(), para.vertices_end());
    append_ngon_face(result, ring, plane, mid);
    result.ok = result.corners_num() >= 3;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Min Parallelogram 2D failed";
  }
  return result;
}

MeshResult points_convex_hull_2(const float *positions, int n, int plane)
{
  MeshResult result;
  if (!positions || n < 3) {
    result.error = "Convex Hull 2D needs at least 3 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    std::vector<Point_2> pts;
    for (int i = 0; i < n; i++) {
      pts.push_back(project_pt(positions + i * 3, plane));
    }
    std::vector<Point_2> hull;
    CGAL::convex_hull_2(pts.begin(), pts.end(), std::back_inserter(hull));
    if (hull.size() < 3) {
      result.error = "Convex Hull 2D empty";
      return result;
    }
    append_ngon_face(result, hull, plane, mid_dropped_axis(positions, n, plane));
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Convex Hull 2D failed";
  }
  return result;
}

bool mesh_plane_slice(const MeshIn &mesh,
                      float px,
                      float py,
                      float pz,
                      float nx,
                      float ny,
                      float nz,
                      std::vector<std::vector<float>> &out_polylines,
                      std::string &error)
{
  out_polylines.clear();
  try {
    Surface_mesh sm;
    if (!load_tri(mesh, sm, error) || sm.number_of_faces() == 0) {
      error = error.empty() ? "Plane slice needs a mesh" : error;
      return false;
    }
    const double ln = std::sqrt(double(nx) * nx + double(ny) * ny + double(nz) * nz);
    if (ln < 1e-12) {
      error = "Plane normal is zero";
      return false;
    }
    Plane_3 plane(Point(px, py, pz), Vector_3(nx / float(ln), ny / float(ln), nz / float(ln)));
    CGAL::Polygon_mesh_slicer<Surface_mesh, Kernel> slicer(sm);
    std::vector<std::vector<Point>> polylines;
    slicer(plane, std::back_inserter(polylines));
    for (const auto &pl : polylines) {
      if (pl.size() < 2) {
        continue;
      }
      std::vector<float> xyz;
      xyz.reserve(pl.size() * 3);
      for (const Point &p : pl) {
        xyz.push_back(float(CGAL::to_double(p.x())));
        xyz.push_back(float(CGAL::to_double(p.y())));
        xyz.push_back(float(CGAL::to_double(p.z())));
      }
      out_polylines.push_back(std::move(xyz));
    }
    return !out_polylines.empty();
  }
  catch (const std::exception &e) {
    error = e.what();
  }
  catch (...) {
    error = "Plane slice failed";
  }
  return false;
}

bool points_vcm_feature_edges(const float *in_xyz,
                              int n,
                              double offset_radius,
                              double convolution_radius,
                              double threshold,
                              std::vector<int8_t> &out_feature,
                              std::vector<float> &out_ratio,
                              int &out_feature_count,
                              std::string &error)
{
  out_feature.assign(size_t(n), 0);
  out_ratio.assign(size_t(n), 0.0f);
  out_feature_count = 0;
  if (!in_xyz || n < 3) {
    error = "VCM needs points";
    return false;
  }
  try {
    std::vector<Point> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]);
    }
    if (offset_radius <= 0.0 || convolution_radius <= 0.0) {
      double avg = 0.0;
      int cnt = 0;
      for (int i = 0; i < std::min(n, 64); i++) {
        double best = 1e300;
        for (int j = 0; j < n; j++) {
          if (i == j) {
            continue;
          }
          const double dx = in_xyz[i * 3] - in_xyz[j * 3];
          const double dy = in_xyz[i * 3 + 1] - in_xyz[j * 3 + 1];
          const double dz = in_xyz[i * 3 + 2] - in_xyz[j * 3 + 2];
          const double d = dx * dx + dy * dy + dz * dz;
          best = std::min(best, d);
        }
        if (best < 1e300) {
          avg += std::sqrt(best);
          cnt++;
        }
      }
      const double spacing = (cnt > 0) ? (avg / cnt) : 0.01;
      if (offset_radius <= 0.0) {
        offset_radius = spacing * 4.0;
      }
      if (convolution_radius <= 0.0) {
        convolution_radius = spacing * 8.0;
      }
    }
    std::vector<std::array<double, 6>> cov;
    CGAL::compute_vcm(pts, cov, offset_radius, convolution_radius,
                      CGAL::parameters::geom_traits(Kernel()));
    for (int i = 0; i < n; i++) {
      /* Feature score: ratio of smallest to largest eigenvalue of VCM (approx
       * from diagonal entries of the packed 6-element symmetric matrix). */
      const auto &c = cov[size_t(i)];
      const double e0 = std::abs(c[0]);
      const double e1 = std::abs(c[3]);
      const double e2 = std::abs(c[5]);
      const double emax = std::max(e0, std::max(e1, e2)) + 1e-30;
      const double emin = std::min(e0, std::min(e1, e2));
      const double r = emin / emax;
      out_ratio[size_t(i)] = float(r);
      if (r >= threshold) {
        out_feature[size_t(i)] = 1;
        out_feature_count++;
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
  }
  catch (...) {
    error = "VCM feature edges failed";
  }
  return false;
}

MeshResult points_delaunay_2(const float *positions, int n, int plane)
{
  MeshResult result;
  if (!positions || n < 3) {
    result.error = "Delaunay 2D needs at least 3 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    using DT = CGAL::Delaunay_triangulation_2<Kernel>;
    DT dt;
    for (int i = 0; i < n; i++) {
      dt.insert(project_pt(positions + i * 3, plane));
    }
    if (dt.dimension() != 2) {
      result.error = "Delaunay 2D degenerate";
      return result;
    }
    const float mid = mid_dropped_axis(positions, n, plane);
    std::map<typename DT::Vertex_handle, int> v2i;
    int local = 0;
    auto vid = [&](typename DT::Vertex_handle vh) {
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
    for (auto fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
      result.corner_verts.push_back(vid(fit->vertex(0)));
      result.corner_verts.push_back(vid(fit->vertex(1)));
      result.corner_verts.push_back(vid(fit->vertex(2)));
    }
    result.ok = result.corners_num() >= 3;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Delaunay 2D failed";
  }
  return result;
}

MeshResult points_alpha_shape_2(const float *positions,
                                int n,
                                int plane,
                                double alpha,
                                bool use_optimal_alpha)
{
  MeshResult result;
  if (!positions || n < 3) {
    result.error = "Alpha Shape 2D needs at least 3 points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    using K = Kernel;
    using Vb = CGAL::Alpha_shape_vertex_base_2<K>;
    using Fb = CGAL::Alpha_shape_face_base_2<K>;
    using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using Triangulation_2 = CGAL::Delaunay_triangulation_2<K, Tds>;
    using Alpha_shape_2 = CGAL::Alpha_shape_2<Triangulation_2>;

    std::vector<Point_2> pts;
    for (int i = 0; i < n; i++) {
      pts.push_back(project_pt(positions + i * 3, plane));
    }
    Alpha_shape_2 as(pts.begin(), pts.end(), 0, Alpha_shape_2::GENERAL);
    if (use_optimal_alpha) {
      alpha = CGAL::to_double(*as.find_optimal_alpha(1));
    }
    if (!(alpha > 0.0)) {
      alpha = 0.01;
    }
    as.set_alpha(alpha);
    result.alpha_used = alpha;
    const float mid = mid_dropped_axis(positions, n, plane);
    std::map<typename Alpha_shape_2::Vertex_handle, int> v2i;
    int local = 0;
    auto vid = [&](typename Alpha_shape_2::Vertex_handle vh) {
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
    for (auto fit = as.finite_faces_begin(); fit != as.finite_faces_end(); ++fit) {
      if (as.classify(fit) != Alpha_shape_2::INTERIOR) {
        continue;
      }
      result.corner_verts.push_back(vid(fit->vertex(0)));
      result.corner_verts.push_back(vid(fit->vertex(1)));
      result.corner_verts.push_back(vid(fit->vertex(2)));
    }
    result.ok = result.corners_num() >= 3;
    if (!result.ok) {
      result.error = "Alpha Shape 2D empty at this alpha";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Alpha Shape 2D failed";
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
  const double diag2 = std::max(1e-24, (maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny));
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
  append_ngon_face(result, hull, plane, mid);
  result.face_tag.push_back(tag);
}

MeshResult points_voronoi_2(const float *positions, int n, int plane, float clip_margin)
{
  MeshResult result;
  if (!positions || n < 2) {
    result.error = "Voronoi 2D needs points";
    return result;
  }
  plane = std::clamp(plane, 0, 2);
  try {
    using DT = CGAL::Delaunay_triangulation_2<Kernel>;
    using Vec2 = Kernel::Vector_2;
    DT dt;
    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (int i = 0; i < n; i++) {
      const Point_2 p = project_pt(positions + i * 3, plane);
      dt.insert(p);
      minx = std::min(minx, CGAL::to_double(p.x()));
      miny = std::min(miny, CGAL::to_double(p.y()));
      maxx = std::max(maxx, CGAL::to_double(p.x()));
      maxy = std::max(maxy, CGAL::to_double(p.y()));
    }
    if (dt.number_of_vertices() < 2) {
      result.error = "Voronoi 2D degenerate";
      return result;
    }
    const double m = std::max(double(clip_margin), 0.0);
    minx -= m;
    miny -= m;
    maxx += m;
    maxy += m;
    if (!(maxx > minx) || !(maxy > miny)) {
      result.error = "Voronoi 2D degenerate bbox";
      return result;
    }
    const Point_2 corners[4] = {Point_2(minx, miny),
                                Point_2(maxx, miny),
                                Point_2(maxx, maxy),
                                Point_2(minx, maxy)};
    const float mid = mid_dropped_axis(positions, n, plane);

    auto add_pt = [&](std::vector<Point_2> &cand, double x, double y) {
      cand.emplace_back(x, y);
    };

    int tag = 0;
    for (auto vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); ++vit) {
      std::vector<Point_2> cand;
      std::vector<typename DT::Face_handle> faces;
      auto fc = dt.incident_faces(vit);
      auto done = fc;
      do {
        faces.push_back(fc);
      } while (++fc != done);

      for (typename DT::Face_handle f : faces) {
        if (dt.is_infinite(f)) {
          continue;
        }
        const Point_2 c = dt.circumcenter(f);
        cand.push_back(c);
      }
      for (size_t i = 0; i < faces.size(); i++) {
        typename DT::Face_handle f1 = faces[i];
        typename DT::Face_handle f2 = faces[(i + 1) % faces.size()];
        if (!dt.is_infinite(f1) && !dt.is_infinite(f2)) {
          const Point_2 c1 = dt.circumcenter(f1);
          const Point_2 c2 = dt.circumcenter(f2);
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
            add_pt(cand, x0, y0);
            add_pt(cand, x1, y1);
          }
        }
        else if (dt.is_infinite(f1) != dt.is_infinite(f2)) {
          typename DT::Face_handle inf = dt.is_infinite(f1) ? f1 : f2;
          typename DT::Face_handle fin = dt.is_infinite(f1) ? f2 : f1;
          const Point_2 c = dt.circumcenter(fin);
          const int inf_i = inf->index(dt.infinite_vertex());
          const Point_2 a = inf->vertex(DT::ccw(inf_i))->point();
          const Point_2 b = inf->vertex(DT::cw(inf_i))->point();
          const double ax = CGAL::to_double(a.x());
          const double ay = CGAL::to_double(a.y());
          const double bx = CGAL::to_double(b.x());
          const double by = CGAL::to_double(b.y());
          const double ex = bx - ax;
          const double ey = by - ay;
          const Point_2 s = vit->point();
          Vec2 n(-ey, ex);
          if (CGAL::orientation(a, b, s) == CGAL::LEFT_TURN) {
            n = Vec2(ey, -ex);
          }
          double hx, hy;
          if (clip_ray_aabb2(CGAL::to_double(c.x()),
                             CGAL::to_double(c.y()),
                             CGAL::to_double(n.x()),
                             CGAL::to_double(n.y()),
                             minx,
                             miny,
                             maxx,
                             maxy,
                             hx,
                             hy))
          {
            add_pt(cand, hx, hy);
          }
        }
      }
      const typename DT::Vertex_handle vh = vit;
      for (const Point_2 &q : corners) {
        if (dt.nearest_vertex(q) == vh) {
          cand.push_back(q);
        }
      }
      emit_isolated_cell_2(result, std::move(cand), plane, mid, tag, minx, miny, maxx, maxy);
      tag++;
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Voronoi 2D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Voronoi 2D failed";
  }
  return result;
}

static void nest_closed_loops_as_islands(std::vector<std::vector<Point_2>> loops,
                                         std::vector<PolyIsland> &islands)
{
  struct Loop {
    std::vector<Point_2> verts;
    double area_abs = 0.0;
  };
  std::vector<Loop> closed;
  closed.reserve(loops.size());
  for (std::vector<Point_2> &raw : loops) {
    std::vector<Point_2> verts = clean_loop_2(std::move(raw));
    const double a = std::abs(signed_area_2(verts));
    if (verts.size() >= 3 && a > 1e-20) {
      closed.push_back(Loop{std::move(verts), a});
    }
  }
  if (closed.empty()) {
    return;
  }
  std::sort(closed.begin(), closed.end(), [](const Loop &a, const Loop &b) {
    return a.area_abs > b.area_abs;
  });

  /* Depth = number of containing loops. Even depth → island outer, odd → hole. */
  const int n = int(closed.size());
  std::vector<int> parent(size_t(n), -1);
  std::vector<int> depth(size_t(n), 0);
  for (int i = 0; i < n; i++) {
    for (int j = i - 1; j >= 0; j--) {
      if (loop_inside_loop(closed[size_t(i)].verts, closed[size_t(j)].verts)) {
        parent[size_t(i)] = j;
        depth[size_t(i)] = depth[size_t(j)] + 1;
        break;
      }
    }
  }

  std::vector<int> island_of(size_t(n), -1);
  for (int i = 0; i < n; i++) {
    if ((depth[size_t(i)] % 2) != 0) {
      continue;
    }
    PolyIsland isl;
    isl.outer = closed[size_t(i)].verts;
    if (signed_area_2(isl.outer) < 0.0) {
      std::reverse(isl.outer.begin(), isl.outer.end());
    }
    island_of[size_t(i)] = int(islands.size());
    islands.push_back(std::move(isl));
  }
  for (int i = 0; i < n; i++) {
    if ((depth[size_t(i)] % 2) == 0 || parent[size_t(i)] < 0) {
      continue;
    }
    const int outer_i = parent[size_t(i)];
    const int isl_i = island_of[size_t(outer_i)];
    if (isl_i < 0) {
      continue;
    }
    std::vector<Point_2> hole = closed[size_t(i)].verts;
    if (signed_area_2(hole) > 0.0) {
      std::reverse(hole.begin(), hole.end());
    }
    islands[size_t(isl_i)].holes.push_back(std::move(hole));
  }
}

static bool extract_skeleton_islands(const MeshIn &mesh,
                                     int plane,
                                     std::vector<PolyIsland> &islands,
                                     float &mid_drop,
                                     std::string &error)
{
  islands.clear();
  if (!mesh.positions || mesh.verts_num < 2) {
    error = "Need vertices";
    return false;
  }
  mid_drop = mid_dropped_axis(mesh.positions, mesh.verts_num, plane);

  const bool has_faces = (mesh.faces_num > 0 && mesh.corners_num >= 3);
  bool has_boundary = false;
  if (has_faces) {
    Surface_mesh sm;
    std::string load_err;
    if (load_tri(mesh, sm, load_err) && sm.number_of_faces() > 0) {
      std::vector<Surface_mesh::Halfedge_index> cycle_starts;
      CGAL::extract_boundary_cycles(sm, std::back_inserter(cycle_starts));
      has_boundary = !cycle_starts.empty();
    }
    std::string dummy;
    if (has_boundary && !extract_polygon_islands(mesh, plane, islands, mid_drop, dummy)) {
      islands.clear();
    }
    /* Drop leftover wire-polyline flags: skeleton always wants filled regions. */
    for (auto it = islands.begin(); it != islands.end();) {
      if (it->is_polyline || it->outer.size() < 3) {
        it = islands.erase(it);
      }
      else {
        ++it;
      }
    }
    /* Closed solid (no border): XY union of faces, keeps silhouette holes. */
    if (islands.empty()) {
      try {
        CGAL::Polygon_set_2<Kernel> pset;
        bool any = false;
        const int *corner_verts = mesh.corner_verts;
        const int *face_offsets = mesh.face_offsets;
        if (corner_verts && face_offsets) {
          for (int f = 0; f < mesh.faces_num; f++) {
            const int a = face_offsets[f];
            const int b = face_offsets[f + 1];
            if (b - a < 3) {
              continue;
            }
            std::vector<Point_2> raw;
            raw.reserve(size_t(b - a));
            for (int k = a; k < b; k++) {
              const int vi = corner_verts[k];
              if (vi < 0 || vi >= mesh.verts_num) {
                raw.clear();
                break;
              }
              raw.push_back(project_pt(mesh.positions + vi * 3, plane));
            }
            raw = clean_loop_2(std::move(raw));
            if (raw.size() < 3) {
              continue;
            }
            if (signed_area_2(raw) < 0.0) {
              std::reverse(raw.begin(), raw.end());
            }
            Polygon_2 poly(raw.begin(), raw.end());
            try {
              if (!poly.is_simple() || poly.size() < 3) {
                continue;
              }
              pset.join(poly);
              any = true;
            }
            catch (...) {
            }
          }
        }
        if (any) {
          std::vector<Polygon_with_holes_2> pwhs;
          pset.polygons_with_holes(std::back_inserter(pwhs));
          for (const Polygon_with_holes_2 &pwh : pwhs) {
            PolyIsland isl;
            for (auto vit = pwh.outer_boundary().vertices_begin();
                 vit != pwh.outer_boundary().vertices_end();
                 ++vit)
            {
              isl.outer.push_back(*vit);
            }
            if (isl.outer.size() < 3) {
              continue;
            }
            for (auto hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
              std::vector<Point_2> hole;
              for (auto vit = hit->vertices_begin(); vit != hit->vertices_end(); ++vit) {
                hole.push_back(*vit);
              }
              if (hole.size() >= 3) {
                isl.holes.push_back(std::move(hole));
              }
            }
            islands.push_back(std::move(isl));
          }
        }
      }
      catch (...) {
      }
    }
  }

  if (islands.empty()) {
    std::vector<Contour2> wire;
    collect_wire_contours(mesh, plane, wire);
    std::vector<std::vector<Point_2>> closed;
    for (Contour2 &c : wire) {
      if (c.closed && c.pts.size() >= 3) {
        closed.push_back(std::move(c.pts));
      }
    }
    nest_closed_loops_as_islands(std::move(closed), islands);
  }

  if (islands.empty()) {
    error = "Straight Skeleton 2D needs filled islands (faces or closed edge rings)";
    return false;
  }
  return true;
}

static Polygon_2 skeleton_prepare_ring(const std::vector<Point_2> &raw, bool want_ccw)
{
  std::vector<Point_2> pts = clean_loop_2(raw);
  if (pts.size() < 3) {
    return Polygon_2();
  }
  /* Light decimate only if extremely dense (font / Bezier). Keep shape. */
  constexpr size_t kMax = 800;
  if (pts.size() > kMax) {
    std::vector<Point_2> dec;
    dec.reserve(kMax);
    for (size_t i = 0; i < kMax; i++) {
      dec.push_back(pts[(i * pts.size()) / kMax]);
    }
    pts = clean_loop_2(std::move(dec));
  }
  if (pts.size() < 3) {
    return Polygon_2();
  }
  const double area = signed_area_2(pts);
  if ((want_ccw && area < 0.0) || (!want_ccw && area > 0.0)) {
    std::reverse(pts.begin(), pts.end());
  }
  Polygon_2 poly(pts.begin(), pts.end());
  try {
    if (want_ccw && poly.is_clockwise_oriented()) {
      poly.reverse_orientation();
    }
    if (!want_ccw && poly.is_counterclockwise_oriented()) {
      poly.reverse_orientation();
    }
  }
  catch (...) {
  }
  return poly;
}

WireResult mesh_straight_skeleton_2(const MeshIn &mesh)
{
  WireResult result;
  const int plane = 0;
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    if (!extract_skeleton_islands(mesh, plane, islands, mid, error)) {
      result.error = error.empty() ? "Straight Skeleton 2D needs polygons" : error;
      return result;
    }

    int vert_base = 0;
    for (const PolyIsland &isl : islands) {
      if (isl.outer.size() < 3) {
        continue;
      }
      Polygon_2 outer = skeleton_prepare_ring(isl.outer, true);
      if (outer.size() < 3) {
        continue;
      }
      std::vector<Polygon_2> holes;
      for (const auto &h : isl.holes) {
        Polygon_2 hp = skeleton_prepare_ring(h, false);
        if (hp.size() < 3) {
          continue;
        }
        holes.push_back(std::move(hp));
      }
      Polygon_with_holes_2 pwh(outer, holes.begin(), holes.end());
      auto ss = CGAL::create_interior_straight_skeleton_2(pwh);
      if (!ss) {
        continue;
      }

      /* Complete interior skeleton: contour vertices + inner nodes + every bisector
       * (including spokes from the boundary). Original contour edges are not bisectors. */
      using Ss = std::remove_reference_t<decltype(*ss)>;
      std::map<typename Ss::Vertex_const_handle, int> vmap;
      for (auto vit = ss->vertices_begin(); vit != ss->vertices_end(); ++vit) {
        float q[3];
        unproject(CGAL::to_double(vit->point().x()),
                  CGAL::to_double(vit->point().y()),
                  plane,
                  mid,
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
        auto va = hit->vertex();
        auto vb = hit->opposite()->vertex();
        auto ia = vmap.find(va);
        auto ib = vmap.find(vb);
        if (ia == vmap.end() || ib == vmap.end()) {
          continue;
        }
        result.edge_v0.push_back(ia->second);
        result.edge_v1.push_back(ib->second);
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Straight Skeleton 2D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Straight Skeleton 2D failed";
  }
  return result;
}

MeshResult mesh_polygon_offset_2(const MeshIn &mesh, double offset_distance, int /*mode*/)
{
  MeshResult result;
  const int plane = 0;

  if (offset_distance == 0.0 || !std::isfinite(offset_distance)) {
    result.error = "Offset must be non-zero";
    return result;
  }
  const bool outward = (offset_distance > 0.0);
  const double r = std::abs(offset_distance);
  if (!(r > 0.0)) {
    result.error = "Offset must be finite and non-zero";
    return result;
  }

  const CGAL::Failure_behaviour prev_err = CGAL::set_error_behaviour(CGAL::THROW_EXCEPTION);
  const CGAL::Failure_behaviour prev_warn = CGAL::set_warning_behaviour(CGAL::CONTINUE);

  try {
    /*
     * extract_polygon_islands:
     *  - Mesh WITH faces → filled islands (skeleton expand/shrink of regions)
     *  - Mesh edges ONLY (no faces) → polylines (open or cyclic wire)
     *    cyclic wire: strip expands BOTH sides (frame, not solid fill)
     */
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, plane, islands, mid, error)) {
      result.error = error.empty() ?
                         "Polygon Offset 2D needs face borders, closed rings, or open edge chains" :
                         error;
      CGAL::set_error_behaviour(prev_err);
      CGAL::set_warning_behaviour(prev_warn);
      return result;
    }

    int ok = 0, fail = 0, wire_n = 0, filled_n = 0;
    for (const PolyIsland &isl : islands) {
      if (isl.is_polyline) {
        wire_n++;
      }
      else {
        filled_n++;
      }
    }
    result.face_tag = {int(islands.size()), filled_n, wire_n};

    for (const PolyIsland &isl : islands) {
      const int before = int(result.corner_verts.size());
      try {
        if (isl.is_polyline) {
          /* Edge-only mesh: |r| half-width to BOTH sides of the line.
           * Cyclic closed loop → hollow frame; open chain → strip with caps. */
          offset_polyline_chain(
              result, isl.outer, r, isl.polyline_closed, plane, mid);
        }
        else {
          /* Filled face region: straight-skeleton solid offset. */
          if (isl.outer.size() < 3) {
            fail++;
            continue;
          }
          Polygon_2 outer = offset_prepare_ccw(isl.outer);
          if (outer.size() < 3) {
            fail++;
            continue;
          }
          std::vector<Polygon_2> holes;
          for (const auto &hpts : isl.holes) {
            if (hpts.size() < 3) {
              continue;
            }
            Polygon_2 h = offset_prepare_ccw(hpts);
            if (h.size() < 3) {
              continue;
            }
            if (h.is_counterclockwise_oriented()) {
              h.reverse_orientation();
            }
            holes.push_back(std::move(h));
          }
          Polygon_with_holes_2 pwh(outer, holes.begin(), holes.end());
          offset_skeleton_closed(result, pwh, r, outward, plane, mid);
        }
      }
      catch (...) {
      }
      if (int(result.corner_verts.size()) > before) {
        ok++;
      }
      else {
        fail++;
      }
    }

    if (result.corner_verts.size() < 3 || (result.corner_verts.size() % 3) != 0) {
      result.error = std::string("Polygon Offset 2D empty. islands=") +
                     std::to_string(islands.size()) + " ok=" + std::to_string(ok) +
                     " fail=" + std::to_string(fail) + " filled=" + std::to_string(filled_n) +
                     " wire=" + std::to_string(wire_n);
      result.ok = false;
    }
    else {
      result.ok = true;
      result.error.clear();
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "Polygon Offset 2D failed (CGAL error)";
    result.ok = false;
  }

  CGAL::set_error_behaviour(prev_err);
  CGAL::set_warning_behaviour(prev_warn);
  return result;
}

WireResult mesh_medial_axis_2(const MeshIn &mesh)
{
  WireResult result;
  const int plane = 0;
  try {
    std::vector<PolyIsland> islands;
    float mid = 0.0f;
    std::string error;
    if (!extract_polygon_islands(mesh, plane, islands, mid, error)) {
      result.error = error.empty() ? "Medial Axis 2D needs polygons" : error;
      return result;
    }

    using DT = CGAL::Delaunay_triangulation_2<Kernel>;

    auto inside_island = [](const Point_2 &q, const PolyIsland &isl) -> bool {
      if (isl.outer.size() < 3 || !point_in_poly(q, isl.outer)) {
        return false;
      }
      for (const auto &h : isl.holes) {
        if (h.size() >= 3 && point_in_poly(q, h)) {
          return false;
        }
      }
      return true;
    };

    for (const PolyIsland &isl : islands) {
      if (isl.is_polyline || isl.outer.size() < 3) {
        continue;
      }
      double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
      for (const Point_2 &p : isl.outer) {
        minx = std::min(minx, CGAL::to_double(p.x()));
        miny = std::min(miny, CGAL::to_double(p.y()));
        maxx = std::max(maxx, CGAL::to_double(p.x()));
        maxy = std::max(maxy, CGAL::to_double(p.y()));
      }
      const double diag = std::max(1e-6, std::sqrt((maxx - minx) * (maxx - minx) +
                                                   (maxy - miny) * (maxy - miny)));
      double spacing = std::clamp(diag / 48.0, diag / 200.0, diag / 16.0);

      std::vector<Point_2> samples;
      auto sample_loop = [&](const std::vector<Point_2> &loop) {
        const int n = int(loop.size());
        for (int i = 0; i < n; i++) {
          const Point_2 &a = loop[size_t(i)];
          const Point_2 &b = loop[size_t((i + 1) % n)];
          const double ax = CGAL::to_double(a.x()), ay = CGAL::to_double(a.y());
          const double bx = CGAL::to_double(b.x()), by = CGAL::to_double(b.y());
          const double len = std::sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
          const int steps = std::max(1, int(std::ceil(len / spacing)));
          for (int s = 0; s < steps; s++) {
            const double t = double(s) / double(steps);
            samples.push_back(Point_2(ax + t * (bx - ax), ay + t * (by - ay)));
          }
        }
      };
      sample_loop(isl.outer);
      for (const auto &h : isl.holes) {
        sample_loop(h);
      }
      if (samples.size() < 4) {
        continue;
      }
      if (samples.size() > 800) {
        std::vector<Point_2> dec;
        const size_t step = (samples.size() + 799) / 800;
        for (size_t i = 0; i < samples.size(); i += step) {
          dec.push_back(samples[i]);
        }
        samples.swap(dec);
      }

      DT dt;
      for (const Point_2 &p : samples) {
        dt.insert(p);
      }
      if (dt.dimension() != 2) {
        continue;
      }

      for (auto eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
        auto f = eit->first;
        int i = eit->second;
        auto f2 = f->neighbor(i);
        if (dt.is_infinite(f) || dt.is_infinite(f2)) {
          continue;
        }
        Point_2 c1 = dt.circumcenter(f);
        Point_2 c2 = dt.circumcenter(f2);
        if (!inside_island(c1, isl) || !inside_island(c2, isl)) {
          continue;
        }
        float qa[3], qb[3];
        unproject(CGAL::to_double(c1.x()), CGAL::to_double(c1.y()), plane, mid, qa);
        unproject(CGAL::to_double(c2.x()), CGAL::to_double(c2.y()), plane, mid, qb);
        const int ia = int(result.positions.size() / 3);
        result.positions.insert(result.positions.end(), {qa[0], qa[1], qa[2]});
        const int ib = int(result.positions.size() / 3);
        result.positions.insert(result.positions.end(), {qb[0], qb[1], qb[2]});
        result.edge_v0.push_back(ia);
        result.edge_v1.push_back(ib);
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Medial Axis 2D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Medial Axis 2D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
