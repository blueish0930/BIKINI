/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 33 — remaining:
 *  - Arrangement 2D (welded n-gon mesh)
 *  - Delaunay on Sphere
 *  - Periodic Delaunay 3D
 *
 * Deleted: Segment Voronoi 2D, Snap Rounding 2D, Theta/Yao Graph 2D,
 * All Furthest Neighbors 2D (IDs reserved).
 *
 * Skipped: P22 OpenGR, G10 Kinetic EPECK, Mesh_3 volume, PolyFit SCIP,
 * Ridges_3, Classification, OSQP segment regularization.
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
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_face_base_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_2.h>
#include <CGAL/Delaunay_triangulation_on_sphere_traits_2.h>
#include <CGAL/Periodic_3_Delaunay_triangulation_3.h>
#include <CGAL/Periodic_3_Delaunay_triangulation_traits_3.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/assertions_behaviour.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <boost/property_map/property_map.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Segment_2 = Kernel::Segment_2;

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

static bool point_in_ring_evenodd(const Point_2 &q, const std::vector<Point_2> &poly)
{
  const size_t n = poly.size();
  if (n < 3) {
    return false;
  }
  const double qx = CGAL::to_double(q.x());
  const double qy = CGAL::to_double(q.y());
  if (!std::isfinite(qx) || !std::isfinite(qy)) {
    return false;
  }
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

static void collect_segments_2(const MeshIn &mesh, int plane, std::vector<Segment_2> &segs)
{
  segs.clear();
  const double scale = xy_snap_scale(mesh, plane);
  auto add = [&](int a, int b) {
    if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
      return;
    }
    const Point_2 p = snap_pt2(project_pt(mesh.positions + a * 3, plane), scale);
    const Point_2 q = snap_pt2(project_pt(mesh.positions + b * 3, plane), scale);
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

static void emit_tets(MeshResult &result,
                      const std::vector<Point_3> &verts,
                      const std::vector<std::array<int, 4>> &tets,
                      bool separate_tets)
{
  result.positions.clear();
  result.corner_verts.clear();
  result.face_tag.clear();
  if (separate_tets) {
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
    for (const Point_3 &p : verts) {
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
    }
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

static bool point_in_any_face(const Point_2 &q, const std::vector<std::vector<Point_2>> &faces)
{
  for (const auto &ring : faces) {
    if (point_in_ring_evenodd(q, ring)) {
      return true;
    }
  }
  return false;
}

static void collect_faces_2(const MeshIn &mesh, int plane, std::vector<std::vector<Point_2>> &faces)
{
  faces.clear();
  if (!mesh.corner_verts || mesh.corners_num < 3) {
    return;
  }
  const bool have_off = mesh.face_offsets != nullptr && mesh.faces_num > 0;
  const int nfaces = have_off ? mesh.faces_num : mesh.corners_num / 3;
  const double scale = xy_snap_scale(mesh, plane);
  faces.reserve(size_t(nfaces));
  for (int f = 0; f < nfaces; f++) {
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
      const Point_2 p = snap_pt2(project_pt(mesh.positions + v * 3, plane), scale);
      if (ring.empty() || ring.back() != p) {
        ring.push_back(p);
      }
    }
    if (ring.size() >= 3 && ring.front() == ring.back()) {
      ring.pop_back();
    }
    if (ring.size() < 3) {
      continue;
    }
    double area2 = 0.0;
    for (size_t i = 0; i < ring.size(); i++) {
      const Point_2 &a = ring[i];
      const Point_2 &b = ring[(i + 1) % ring.size()];
      area2 += CGAL::to_double(a.x()) * CGAL::to_double(b.y()) -
               CGAL::to_double(b.x()) * CGAL::to_double(a.y());
    }
    if (std::abs(area2) <= 1e-16) {
      continue;
    }
    faces.push_back(std::move(ring));
  }
}

template<typename Circ> static Point_2 interior_point_ccb(Circ circ)
{
  auto cur = circ;
  const Point_2 a = cur->source()->point();
  const Point_2 b = cur->target()->point();
  const double ax = CGAL::to_double(a.x());
  const double ay = CGAL::to_double(a.y());
  const double bx = CGAL::to_double(b.x());
  const double by = CGAL::to_double(b.y());
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len = std::hypot(dx, dy);
  const double eps = (len > 1e-18) ? (1e-8 * len) : 1e-8;
  /* Face lies to the left of the halfedge. */
  return Point_2(0.5 * (ax + bx) - dy / (len > 1e-18 ? len : 1.0) * eps,
                 0.5 * (ay + by) + dx / (len > 1e-18 ? len : 1.0) * eps);
}

#if 0
MeshResult mesh_arrangement_2_OLD(const MeshIn &mesh, bool inside_faces_only)
{
  MeshResult result;
  const int plane = 0;
  try {
    std::vector<Segment_2> segs;
    collect_segments_2(mesh, plane, segs);
    if (segs.size() < 1) {
      result.error = "Arrangement 2D needs mesh edges";
      return result;
    }
    auto seg_key = [](const Segment_2 &s) {
      Point_2 a = s.source();
      Point_2 b = s.target();
      if (b < a) {
        std::swap(a, b);
      }
      return std::make_pair(a, b);
    };
    std::sort(segs.begin(), segs.end(), [&](const Segment_2 &a, const Segment_2 &b) {
      const auto ka = seg_key(a);
      const auto kb = seg_key(b);
      if (ka.first < kb.first) {
        return true;
      }
      if (kb.first < ka.first) {
        return false;
      }
      return ka.second < kb.second;
    });
    segs.erase(std::unique(segs.begin(),
                           segs.end(),
                           [&](const Segment_2 &a, const Segment_2 &b) {
                             return seg_key(a) == seg_key(b);
                           }),
               segs.end());

    std::vector<std::vector<Point_2>> cover;
    if (inside_faces_only) {
      collect_faces_2(mesh, plane, cover);
      if (cover.empty()) {
        inside_faces_only = false;
      }
    }

    using Traits = CGAL::Arr_segment_traits_2<Kernel>;
    using Arrangement = CGAL::Arrangement_2<Traits>;
    Arrangement arr;
    CgalThrowGuard cgal_guard;
    try {
      CGAL::insert(arr, segs.begin(), segs.end());
    }
    catch (...) {
      arr.clear();
      for (const Segment_2 &s : segs) {
        try {
          if (s.source() != s.target()) {
            CGAL::insert(arr, s);
          }
        }
        catch (...) {
        }
      }
    }
    if (arr.number_of_vertices() < 3) {
      result.error = "Arrangement 2D failed to insert segments";
      return result;
    }

    const float mid = mid_dropped_axis(mesh.positions, mesh.verts_num, plane);

    /* Shared vertex pool: one mesh vert per arrangement vertex. */
    std::map<typename Arrangement::Vertex_handle, int> vmap;
    std::map<Point_2, int> p2i;
    for (auto vit = arr.vertices_begin(); vit != arr.vertices_end(); ++vit) {
      const int idx = result.verts_num();
      vmap[vit] = idx;
      p2i[vit->point()] = idx;
      float q[3];
      unproject(CGAL::to_double(vit->point().x()),
                CGAL::to_double(vit->point().y()),
                plane,
                mid,
                q);
      result.positions.insert(result.positions.end(), {q[0], q[1], q[2]});
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

    using Vb = CGAL::Triangulation_vertex_base_2<Kernel>;
    using Fb = CGAL::Constrained_triangulation_face_base_2<Kernel>;
    using TDS = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using CDT = CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, CGAL::Exact_predicates_tag>;

    int tag = 0;
    for (auto fit = arr.faces_begin(); fit != arr.faces_end(); ++fit) {
      try {
      if (fit->is_unbounded() || !fit->has_outer_ccb()) {
        continue;
      }
      const std::vector<int> outer = collect_ccb(fit->outer_ccb());
      if (outer.size() < 3) {
        continue;
      }
      if (inside_faces_only) {
        const Point_2 q = interior_point_ccb(fit->outer_ccb());
        if (!point_in_any_face(q, cover)) {
          continue;
        }
      }
      const bool has_hole = fit->holes_begin() != fit->holes_end();
      if (!has_hole) {
        append_shared_ngon(result, outer, tag++);
        continue;
      }

      /* Faces with holes cannot be a single Blender n-gon — CDT fill, still
       * sharing the arrangement vertices so the mesh stays one piece. */
      try {
        CDT cdt;
        auto insert_ccb = [&](typename Arrangement::Ccb_halfedge_circulator circ) {
          std::vector<typename CDT::Vertex_handle> vhs;
          auto cur = circ;
          int steps = 0;
          do {
            vhs.push_back(cdt.insert(cur->source()->point()));
            ++cur;
          } while (cur != circ && ++steps < max_ccb);
          if (steps >= max_ccb) {
            return;
          }
          const size_t n = vhs.size();
          for (size_t i = 0; i < n; i++) {
            if (vhs[i] != vhs[(i + 1) % n]) {
              cdt.insert_constraint(vhs[i], vhs[(i + 1) % n]);
            }
          }
        };
        insert_ccb(fit->outer_ccb());
        for (auto hit = fit->holes_begin(); hit != fit->holes_end(); ++hit) {
          insert_ccb(*hit);
        }
        if (cdt.dimension() != 2) {
          append_shared_ngon(result, outer, tag++);
          continue;
        }
        std::map<typename CDT::Face_handle, bool> in_domain_map;
        boost::associative_property_map<std::map<typename CDT::Face_handle, bool>> in_domain(
            in_domain_map);
        CGAL::mark_domain_in_triangulation(cdt, in_domain);
        bool any = false;
        for (auto cfit = cdt.finite_faces_begin(); cfit != cdt.finite_faces_end(); ++cfit) {
          if (!in_domain_map[cfit]) {
            continue;
          }
          int ids[3];
          bool ok = true;
          for (int i = 0; i < 3; i++) {
            auto pit = p2i.find(cfit->vertex(i)->point());
            if (pit == p2i.end()) {
              ok = false;
              break;
            }
            ids[i] = pit->second;
          }
          if (!ok) {
            continue;
          }
          append_shared_ngon(result, {ids[0], ids[1], ids[2]}, tag);
          any = true;
        }
        if (any) {
          tag++;
        }
        else {
          append_shared_ngon(result, outer, tag++);
        }
      }
      catch (...) {
        append_shared_ngon(result, outer, tag++);
      }
      }
      catch (...) {
      }
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Arrangement 2D produced no bounded faces";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Arrangement 2D failed";
  }
  return result;
}
#endif

MeshResult points_delaunay_on_sphere(const float *positions, int n)
{
  MeshResult result;
  if (!positions || n < 4) {
    result.error = "Delaunay on Sphere needs at least 4 points";
    return result;
  }
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
      r2max = std::max(r2max, dx * dx + dy * dy + dz * dz);
    }
    const double radius = std::sqrt(std::max(r2max, 1e-18));
    const Point_3 center(cx, cy, cz);

    std::vector<Point_3> on_sphere;
    on_sphere.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      const double dx = double(positions[i * 3 + 0]) - cx;
      const double dy = double(positions[i * 3 + 1]) - cy;
      const double dz = double(positions[i * 3 + 2]) - cz;
      const double L = std::sqrt(std::max(1e-30, dx * dx + dy * dy + dz * dz));
      on_sphere.emplace_back(cx + dx / L * radius, cy + dy / L * radius, cz + dz / L * radius);
    }

    using Traits = CGAL::Delaunay_triangulation_on_sphere_traits_2<Kernel>;
    using Dt = CGAL::Delaunay_triangulation_on_sphere_2<Traits>;
    Dt dt(center, radius);
    dt.insert(on_sphere.begin(), on_sphere.end());
    if (dt.dimension() != 2) {
      result.error = "Delaunay on Sphere degenerate";
      return result;
    }

    std::map<typename Dt::Vertex_handle, int> v2i;
    for (auto vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); ++vit) {
      if (v2i.count(vit)) {
        continue;
      }
      const int idx = result.verts_num();
      v2i[vit] = idx;
      const Point_3 p = vit->point();
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
    }
    for (auto fit = dt.finite_faces_begin(); fit != dt.finite_faces_end(); ++fit) {
      if (dt.is_ghost(fit)) {
        continue;
      }
      int ids[3];
      bool ok = true;
      for (int i = 0; i < 3; i++) {
        auto it = v2i.find(fit->vertex(i));
        if (it == v2i.end()) {
          ok = false;
          break;
        }
        ids[i] = it->second;
      }
      if (!ok || ids[0] == ids[1] || ids[1] == ids[2] || ids[2] == ids[0]) {
        continue;
      }
      result.corner_verts.push_back(ids[0]);
      result.corner_verts.push_back(ids[1]);
      result.corner_verts.push_back(ids[2]);
    }
    result.center[0] = float(cx);
    result.center[1] = float(cy);
    result.center[2] = float(cz);
    result.radius = float(radius);
    result.ok = result.corners_num() >= 3;
    if (!result.ok) {
      result.error = "Delaunay on Sphere produced no faces";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Delaunay on Sphere failed";
  }
  return result;
}

MeshResult points_periodic_delaunay_3(const float *positions,
                                      int n,
                                      double domain_x,
                                      double domain_y,
                                      double domain_z,
                                      bool separate_tets)
{
  MeshResult result;
  if (!positions || n < 4) {
    result.error = "Periodic Delaunay 3D needs at least 4 points";
    return result;
  }
  try {
    using Gt = CGAL::Periodic_3_Delaunay_triangulation_traits_3<Kernel>;
    using PDT = CGAL::Periodic_3_Delaunay_triangulation_3<Gt>;

    double mn[3] = {1e300, 1e300, 1e300};
    double mx[3] = {-1e300, -1e300, -1e300};
    for (int i = 0; i < n; i++) {
      for (int k = 0; k < 3; k++) {
        const double v = double(positions[i * 3 + k]);
        mn[k] = std::min(mn[k], v);
        mx[k] = std::max(mx[k], v);
      }
    }
    double w[3] = {domain_x, domain_y, domain_z};
    for (int k = 0; k < 3; k++) {
      const double span = std::max(1e-6, mx[k] - mn[k]);
      if (!(w[k] > 0.0)) {
        w[k] = span * 1.05;
      }
      w[k] = std::max(w[k], 1e-6);
    }
    const double ox = mn[0] - (w[0] - (mx[0] - mn[0])) * 0.5;
    const double oy = mn[1] - (w[1] - (mx[1] - mn[1])) * 0.5;
    const double oz = mn[2] - (w[2] - (mx[2] - mn[2])) * 0.5;

    auto wrap = [](double v, double o, double width) {
      double t = std::fmod(v - o, width);
      if (t < 0.0) {
        t += width;
      }
      if (t >= width - 1e-12) {
        t = 0.0;
      }
      if (t < 1e-12) {
        t = 1e-12;
      }
      return o + t;
    };

    PDT::Iso_cuboid domain(ox, oy, oz, ox + w[0], oy + w[1], oz + w[2]);
    PDT T(domain);
    for (int i = 0; i < n; i++) {
      T.insert(Point_3(wrap(double(positions[i * 3 + 0]), ox, w[0]),
                       wrap(double(positions[i * 3 + 1]), oy, w[1]),
                       wrap(double(positions[i * 3 + 2]), oz, w[2])));
    }
    if (T.dimension() != 3) {
      result.error = "Periodic Delaunay 3D degenerate";
      return result;
    }
    if (!T.is_1_cover() && T.is_triangulation_in_1_sheet()) {
      T.convert_to_1_sheeted_covering();
    }

    /* Use T.point(cell, i): applies the periodic offset so wrapping tets
     * sit on the domain boundary instead of stretching across the box. */
    std::vector<Point_3> verts;
    std::vector<std::array<int, 4>> tets;
    using P3T = CGAL::Periodic_3_triangulation_3<Gt>;
    using VKey = std::tuple<typename PDT::Vertex_handle, int, int, int>;
    std::map<VKey, int> vkey;
    const P3T &base = T;
    for (auto cit = T.finite_cells_begin(); cit != T.finite_cells_end(); ++cit) {
      /* PDT hides Base::is_canonical(Cell_handle) with a private segment helper. */
      if (!base.is_canonical(typename PDT::Cell_handle(cit))) {
        continue;
      }
      std::array<int, 4> t;
      for (int i = 0; i < 4; i++) {
        const auto pp = T.periodic_point(cit, i);
        const VKey key = std::make_tuple(
            cit->vertex(i), pp.second.x(), pp.second.y(), pp.second.z());
        auto it = vkey.find(key);
        if (it == vkey.end()) {
          const int idx = int(verts.size());
          vkey[key] = idx;
          verts.push_back(T.point(cit, i));
          t[size_t(i)] = idx;
        }
        else {
          t[size_t(i)] = it->second;
        }
      }
      tets.push_back(t);
    }
    emit_tets(result, verts, tets, separate_tets);
    if (!result.ok) {
      result.error = "Periodic Delaunay 3D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Periodic Delaunay 3D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
