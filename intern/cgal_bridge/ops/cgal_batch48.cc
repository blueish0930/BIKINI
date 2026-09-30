/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 48 — new only (not previously deleted):
 *  - Rectangular P-Center 2D (p axis-aligned covering squares)
 *  - Polyline Hull 2D (Melkman convex hull of ordered rings)
 *
 * Proximity Graph 2D and Conforming Delaunay 2D reuse existing intern.
 *
 * Skipped: OpenGR, Kinetic EPECK, Mesh_3 volume, PolyFit SCIP, Ridges_3,
 * Classification, OSQP, Hyperbolic Delaunay, weighted Delaunay / Lloyd,
 * banned measure/query/remesh, previously deleted nodes.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/ch_melkman.h>
#include <CGAL/number_utils.h>
#include <CGAL/rectangular_p_center_2.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using FT = Kernel::FT;

static Point_2 xy_of(const float *positions, int i)
{
  return Point_2(FT(double(positions[i * 3 + 0])), FT(double(positions[i * 3 + 1])));
}

static void append_ngon_xy(MeshResult &result,
                           const std::vector<Point_2> &ring,
                           float mid_z,
                           int tag)
{
  if (ring.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets = {0};
  }
  const int base = result.verts_num();
  for (const Point_2 &p : ring) {
    result.positions.push_back(float(CGAL::to_double(p.x())));
    result.positions.push_back(float(CGAL::to_double(p.y())));
    result.positions.push_back(mid_z);
  }
  for (int i = 0; i < int(ring.size()); i++) {
    result.corner_verts.push_back(base + i);
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
}

static float mid_z_of(const float *positions, int n)
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

static void collect_face_rings(const MeshIn &mesh, std::vector<std::vector<int>> &loops)
{
  if (mesh.faces_num <= 0 || !mesh.corner_verts) {
    return;
  }
  for (int f = 0; f < mesh.faces_num; f++) {
    int a = 0;
    int b = 0;
    if (mesh.face_offsets) {
      a = mesh.face_offsets[f];
      b = mesh.face_offsets[f + 1];
    }
    else {
      a = f * 3;
      b = a + 3;
    }
    std::vector<int> ring;
    ring.reserve(size_t((std::max)(0, b - a)));
    for (int c = a; c < b; c++) {
      if (c < 0 || c >= mesh.corners_num) {
        continue;
      }
      const int v = mesh.corner_verts[c];
      if (v < 0 || v >= mesh.verts_num) {
        continue;
      }
      ring.push_back(v);
    }
    if (ring.size() >= 2 && ring.front() == ring.back()) {
      ring.pop_back();
    }
    if (ring.size() >= 3) {
      loops.push_back(std::move(ring));
    }
  }
}

static void collect_wire_rings(const MeshIn &mesh, std::vector<std::vector<int>> &loops)
{
  if (mesh.edges_num <= 0 || !mesh.edge_v0 || !mesh.edge_v1 || mesh.verts_num <= 0) {
    return;
  }
  std::vector<std::vector<int>> adj(size_t(mesh.verts_num));
  for (int i = 0; i < mesh.edges_num; i++) {
    const int a = mesh.edge_v0[i];
    const int b = mesh.edge_v1[i];
    if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
      continue;
    }
    adj[size_t(a)].push_back(b);
    adj[size_t(b)].push_back(a);
  }
  std::vector<char> used(size_t(mesh.verts_num), 0);
  auto walk = [&](int start) {
    std::vector<int> chain;
    int prev = -1;
    int cur = start;
    while (cur >= 0 && used[size_t(cur)] == 0) {
      used[size_t(cur)] = 1;
      chain.push_back(cur);
      int next = -1;
      for (const int nb : adj[size_t(cur)]) {
        if (nb == prev) {
          continue;
        }
        if (used[size_t(nb)] == 0) {
          next = nb;
          break;
        }
        if (nb == start && int(chain.size()) >= 3) {
          next = -2;
          break;
        }
      }
      if (next == -2) {
        break;
      }
      prev = cur;
      cur = next;
    }
    if (chain.size() >= 3) {
      loops.push_back(std::move(chain));
    }
  };
  for (int v = 0; v < mesh.verts_num; v++) {
    if (used[size_t(v)] == 0 && adj[size_t(v)].size() == 1) {
      walk(v);
    }
  }
  for (int v = 0; v < mesh.verts_num; v++) {
    if (used[size_t(v)] == 0 && !adj[size_t(v)].empty()) {
      walk(v);
    }
  }
}

}  // namespace

MeshResult points_rectangular_p_center_2(const float *positions, int n, int p)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 2) {
    result.error = "Rectangular P-Center 2D needs at least 2 points";
    return result;
  }
  p = std::clamp(p, 2, 4);
  try {
    std::vector<Point_2> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.push_back(xy_of(positions, i));
    }
    const float mid_z = mid_z_of(positions, n);
    FT radius = 0;
    std::vector<Point_2> centers;
    CGAL::rectangular_p_center_2(pts.begin(), pts.end(), std::back_inserter(centers), radius, p);
    if (centers.empty()) {
      result.error = "Rectangular P-Center 2D produced no centers";
      return result;
    }
    const double r = std::max(CGAL::to_double(radius), 1e-12);
    for (int i = 0; i < int(centers.size()); i++) {
      const double cx = CGAL::to_double(centers[size_t(i)].x());
      const double cy = CGAL::to_double(centers[size_t(i)].y());
      std::vector<Point_2> sq = {
          Point_2(FT(cx - r), FT(cy - r)),
          Point_2(FT(cx + r), FT(cy - r)),
          Point_2(FT(cx + r), FT(cy + r)),
          Point_2(FT(cx - r), FT(cy + r)),
      };
      append_ngon_xy(result, sq, mid_z, i);
    }
    result.radius = float(r);
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Rectangular P-Center 2D produced no squares";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Rectangular P-Center 2D failed";
  }
  return result;
}

MeshResult mesh_polyline_hull_2(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!mesh.positions || mesh.verts_num < 3) {
    result.error = "Polyline Hull 2D needs a polygon or polyline mesh";
    return result;
  }
  try {
    std::vector<std::vector<int>> loops;
    collect_face_rings(mesh, loops);
    if (loops.empty()) {
      collect_wire_rings(mesh, loops);
    }
    if (loops.empty()) {
      result.error = "Polyline Hull 2D needs ordered face rings or a polyline";
      return result;
    }
    const float mid_z = mid_z_of(mesh.positions, mesh.verts_num);
    int tag = 0;
    for (const std::vector<int> &loop : loops) {
      std::vector<Point_2> poly;
      poly.reserve(loop.size());
      for (const int v : loop) {
        poly.push_back(xy_of(mesh.positions, v));
      }
      std::vector<Point_2> hull;
      CGAL::ch_melkman(poly.begin(), poly.end(), std::back_inserter(hull));
      if (hull.size() >= 2 && hull.front() == hull.back()) {
        hull.pop_back();
      }
      if (hull.size() >= 3) {
        append_ngon_xy(result, hull, mid_z, tag);
      }
      tag++;
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Polyline Hull 2D produced no hulls (need a simple ordered polyline)";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Polyline Hull 2D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
