/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 38 — 10 new nodes (not previously deleted):
 *  - Octree 3D
 *  - Hilbert Path 3D
 *  - Shortest Cycle (non-contractible)
 *  - Periodic Voronoi 2D
 *  - Periodic Voronoi 3D
 *  - Gabriel Graph 3D
 *  - Euclidean MST 3D
 *  - Beta Skeleton 2D
 *  - Convex Layers 2D
 *
 * Not added: Periodic Regular / Lloyd / weighted Delaunay,
 * OpenGR, Kinetic EPECK, Mesh_3 volume, PolyFit SCIP, Ridges_3,
 * Classification, OSQP, Hyperbolic Delaunay, banned measure/query/remesh.
 */

#include "cgal_bridge.hh"
#include "cgal_convex_clip3.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Octree.h>
#include <CGAL/Orthtree/Traversals.h>
#include <CGAL/Path_on_surface.h>
#include <CGAL/Periodic_2_Delaunay_triangulation_2.h>
#include <CGAL/Periodic_2_Delaunay_triangulation_traits_2.h>
#include <CGAL/Periodic_3_Delaunay_triangulation_3.h>
#include <CGAL/Periodic_3_Delaunay_triangulation_traits_3.h>
#include <CGAL/Periodic_3_triangulation_3.h>
#include <CGAL/Spatial_sort_traits_adapter_3.h>
#include <CGAL/Triangulation_data_structure_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/hilbert_sort.h>
#include <CGAL/property_map.h>
#include <CGAL/spatial_sort.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/squared_distance_3.h>

#include <CGAL/Curves_on_surface_topology.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <utility>
#include <vector>

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using FT = Kernel::FT;
using Iso_cuboid = Kernel::Iso_cuboid_3;

static Point_2 xy(double x, double y)
{
  const FT fx(x);
  const FT fy(y);
  return Point_2(fx, fy);
}

static Point_3 xyz(double x, double y, double z)
{
  const FT fx(x);
  const FT fy(y);
  const FT fz(z);
  return Point_3(fx, fy, fz);
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

static int add_vert(WireResult &w, const Point_3 &p)
{
  const int i = int(w.positions.size() / 3);
  push_xyz(w.positions, p);
  return i;
}

static void emit_box_wire(WireResult &w, const Point_3 &lo, const Point_3 &hi)
{
  const double x0 = CGAL::to_double(lo.x()), y0 = CGAL::to_double(lo.y()),
               z0 = CGAL::to_double(lo.z());
  const double x1 = CGAL::to_double(hi.x()), y1 = CGAL::to_double(hi.y()),
               z1 = CGAL::to_double(hi.z());
  const Point_3 c[8] = {
      xyz(x0, y0, z0), xyz(x1, y0, z0), xyz(x1, y1, z0), xyz(x0, y1, z0),
      xyz(x0, y0, z1), xyz(x1, y0, z1), xyz(x1, y1, z1), xyz(x0, y1, z1),
  };
  const int b = int(w.positions.size() / 3);
  for (int i = 0; i < 8; i++) {
    push_xyz(w.positions, c[i]);
  }
  const int e[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                        {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (int i = 0; i < 12; i++) {
    add_wire(w, b + e[i][0], b + e[i][1]);
  }
}

static void emit_box_solid(MeshResult &r, const Point_3 &lo, const Point_3 &hi)
{
  const double x0 = CGAL::to_double(lo.x()), y0 = CGAL::to_double(lo.y()),
               z0 = CGAL::to_double(lo.z());
  const double x1 = CGAL::to_double(hi.x()), y1 = CGAL::to_double(hi.y()),
               z1 = CGAL::to_double(hi.z());
  const Point_3 c[8] = {
      xyz(x0, y0, z0), xyz(x1, y0, z0), xyz(x1, y1, z0), xyz(x0, y1, z0),
      xyz(x0, y0, z1), xyz(x1, y0, z1), xyz(x1, y1, z1), xyz(x0, y1, z1),
  };
  const int b = r.verts_num();
  for (int i = 0; i < 8; i++) {
    push_xyz(r.positions, c[i]);
  }
  const int faces[6][4] = {{0, 1, 2, 3},
                           {4, 7, 6, 5},
                           {0, 4, 5, 1},
                           {2, 6, 7, 3},
                           {0, 3, 7, 4},
                           {1, 5, 6, 2}};
  if (r.face_offsets.empty()) {
    r.face_offsets.push_back(0);
  }
  for (int f = 0; f < 6; f++) {
    for (int k = 0; k < 4; k++) {
      r.corner_verts.push_back(b + faces[f][k]);
    }
    r.face_offsets.push_back(int(r.corner_verts.size()));
  }
}

static std::pair<long long, long long> key2(const Point_2 &p)
{
  return {llround(CGAL::to_double(p.x()) * 1e7), llround(CGAL::to_double(p.y()) * 1e7)};
}

}  // namespace

MeshResult points_octree_3(const float *positions, int n, int max_depth, int bucket, bool solid)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 1) {
    result.error = "Octree 3D needs points";
    return result;
  }
  max_depth = std::clamp(max_depth, 1, 12);
  bucket = std::max(1, bucket);
  try {
    std::vector<Point_3> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.push_back(xyz(double(positions[i * 3 + 0]),
                        double(positions[i * 3 + 1]),
                        double(positions[i * 3 + 2])));
    }
    using Tree = CGAL::Octree<Kernel, std::vector<Point_3>>;
    Tree tree(pts);
    tree.refine(size_t(max_depth), size_t(bucket));
    WireResult wire;
    for (auto node : tree.traverse(CGAL::Orthtrees::Leaves_traversal<Tree>(tree))) {
      const Iso_cuboid box = tree.bbox(node);
      if (solid) {
        emit_box_solid(result, box.min(), box.max());
      }
      else {
        emit_box_wire(wire, box.min(), box.max());
      }
    }
    if (!solid) {
      result.positions = std::move(wire.positions);
      result.ok = !wire.edge_v0.empty();
      /* Encode wire as degenerate faces? Callers of MeshResult expect faces.
       * Keep a side-channel: store edges via face_tag unused. Better return
       * through a dedicated wire wrapper. We fill seam pairs as edges. */
      result.seam_vert_a = std::move(wire.edge_v0);
      result.seam_vert_b = std::move(wire.edge_v1);
      result.ok = !result.seam_vert_a.empty();
    }
    else {
      result.ok = result.faces_num() > 0;
    }
    if (!result.ok) {
      result.error = "Octree 3D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Octree 3D failed";
  }
  return result;
}

WireResult points_hilbert_path_3(const float *positions, int n)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!positions || n < 2) {
    result.error = "Hilbert Path 3D needs at least 2 points";
    return result;
  }
  try {
    std::vector<Point_3> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.push_back(xyz(double(positions[i * 3 + 0]),
                        double(positions[i * 3 + 1]),
                        double(positions[i * 3 + 2])));
    }
    CGAL::hilbert_sort(pts.begin(), pts.end());
    result.positions.reserve(size_t(n) * 3);
    for (const Point_3 &p : pts) {
      push_xyz(result.positions, p);
    }
    for (int i = 0; i + 1 < n; i++) {
      add_wire(result, i, i + 1);
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Hilbert Path 3D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Hilbert Path 3D failed";
  }
  return result;
}

WireResult mesh_shortest_cycle(const MeshIn &mesh)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!mesh.positions || mesh.faces_num < 1) {
    result.error = "Shortest Cycle needs a surface mesh";
    return result;
  }
  try {
    Surface_mesh sm;
    std::string load_err;
    if (!mesh_in_to_surface_mesh(mesh, sm, load_err, false)) {
      result.error = load_err.empty() ? "Shortest Cycle: cannot load mesh" : load_err;
      return result;
    }
    using CST = CGAL::Surface_mesh_topology::Curves_on_surface_topology<Surface_mesh>;
    CST cst(sm);
    auto path = cst.compute_shortest_non_contractible_cycle();
    if (path.length() < 3) {
      result.error =
          "Shortest Cycle empty (needs a closed mesh of genus >= 1, e.g. a torus)";
      return result;
    }
    std::map<Surface_mesh::Vertex_index, int> vid;
    auto vert_of = [&](Surface_mesh::Halfedge_index h) -> int {
      const Surface_mesh::Vertex_index v = sm.source(h);
      const auto it = vid.find(v);
      if (it != vid.end()) {
        return it->second;
      }
      const int i = add_vert(result, sm.point(v));
      vid.emplace(v, i);
      return i;
    };
    for (std::size_t i = 0; i < path.length(); i++) {
      const auto dh = path.get_ith_real_dart(i);
      const Surface_mesh::Halfedge_index h(dh);
      const int a = vert_of(h);
      const int b = vert_of(sm.opposite(h));
      add_wire(result, a, b);
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Shortest Cycle produced no edges";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Shortest Cycle failed";
  }
  return result;
}

MeshResult points_periodic_voronoi_2(const float *positions, int n, double domain_x, double domain_y)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 3) {
    result.error = "Periodic Voronoi 2D needs at least 3 points";
    return result;
  }
  try {
    using Gt = CGAL::Periodic_2_Delaunay_triangulation_traits_2<Kernel>;
    using PDT = CGAL::Periodic_2_Delaunay_triangulation_2<Gt>;

    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    std::vector<Point_2> raw;
    raw.reserve(size_t(n));
    double zacc = 0.0;
    for (int i = 0; i < n; i++) {
      const Point_2 p = xy(double(positions[i * 3 + 0]), double(positions[i * 3 + 1]));
      raw.push_back(p);
      zacc += double(positions[i * 3 + 2]);
      minx = std::min(minx, CGAL::to_double(p.x()));
      miny = std::min(miny, CGAL::to_double(p.y()));
      maxx = std::max(maxx, CGAL::to_double(p.x()));
      maxy = std::max(maxy, CGAL::to_double(p.y()));
    }
    const float zmid = float(zacc / double(n));
    const double spanx = std::max(1e-6, maxx - minx);
    const double spany = std::max(1e-6, maxy - miny);
    double wx = (domain_x > 0.0) ? domain_x : spanx * 1.05;
    double wy = (domain_y > 0.0) ? domain_y : spany * 1.05;
    wx = std::max(wx, 1e-6);
    wy = std::max(wy, 1e-6);
    const double ox = minx - (wx - spanx) * 0.5;
    const double oy = miny - (wy - spany) * 0.5;
    auto wrap = [](double v, double o, double w) {
      double t = std::fmod(v - o, w);
      if (t < 0.0) {
        t += w;
      }
      if (t >= w - 1e-12) {
        t = 0.0;
      }
      if (t < 1e-12) {
        t = 1e-12;
      }
      return o + t;
    };
    PDT T(PDT::Iso_rectangle(ox, oy, ox + wx, oy + wy));
    for (const Point_2 &p : raw) {
      T.insert(xy(wrap(CGAL::to_double(p.x()), ox, wx), wrap(CGAL::to_double(p.y()), oy, wy)));
    }
    if (T.dimension() != 2) {
      result.error = "Periodic Voronoi 2D degenerate";
      return result;
    }
    if (T.is_triangulation_in_1_sheet()) {
      T.convert_to_1_sheeted_covering();
    }

    auto unwrap = [&](const Point_2 &p, const Point_2 &ref) {
      double x = CGAL::to_double(p.x());
      double y = CGAL::to_double(p.y());
      const double rx = CGAL::to_double(ref.x());
      const double ry = CGAL::to_double(ref.y());
      while (x - rx > wx * 0.5) {
        x -= wx;
      }
      while (rx - x > wx * 0.5) {
        x += wx;
      }
      while (y - ry > wy * 0.5) {
        y -= wy;
      }
      while (ry - y > wy * 0.5) {
        y += wy;
      }
      return xy(x, y);
    };

    int tag = 0;
    result.face_offsets = {0};
    for (auto vit = T.unique_vertices_begin(); vit != T.unique_vertices_end(); ++vit) {
      std::vector<Point_2> ring;
      auto fc = T.incident_faces(vit);
      const auto done = fc;
      do {
        const Point_2 c = CGAL::circumcenter(T.point(fc, 0), T.point(fc, 1), T.point(fc, 2));
        ring.push_back(c);
      } while (++fc != done);
      if (ring.size() < 3) {
        continue;
      }
      for (size_t i = 1; i < ring.size(); i++) {
        ring[i] = unwrap(ring[i], ring[0]);
      }
      const int base = result.verts_num();
      for (const Point_2 &p : ring) {
        push_xy0(result.positions, p, zmid);
        result.corner_verts.push_back(int(result.corner_verts.size()));
      }
      result.face_offsets.push_back(int(result.corner_verts.size()));
      result.face_tag.push_back(tag++);
      (void)base;
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Periodic Voronoi 2D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Periodic Voronoi 2D failed";
  }
  return result;
}

MeshResult points_periodic_voronoi_3(const float *positions,
                                     int n,
                                     double domain_x,
                                     double domain_y,
                                     double domain_z)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 4) {
    result.error = "Periodic Voronoi 3D needs at least 4 points";
    return result;
  }
  try {
    using Gt = CGAL::Periodic_3_Delaunay_triangulation_traits_3<Kernel>;
    using PDT = CGAL::Periodic_3_Delaunay_triangulation_3<Gt>;
    using Vertex = PDT::Vertex_handle;
    using Cell = PDT::Cell_handle;

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
    PDT T(PDT::Iso_cuboid(ox, oy, oz, ox + w[0], oy + w[1], oz + w[2]));
    for (int i = 0; i < n; i++) {
      T.insert(xyz(wrap(double(positions[i * 3 + 0]), ox, w[0]),
                   wrap(double(positions[i * 3 + 1]), oy, w[1]),
                   wrap(double(positions[i * 3 + 2]), oz, w[2])));
    }
    if (T.dimension() != 3) {
      result.error = "Periodic Voronoi 3D degenerate";
      return result;
    }
    if (!T.is_1_cover() && T.is_triangulation_in_1_sheet()) {
      T.convert_to_1_sheeted_covering();
    }

    auto unwrap3 = [&](double x, double y, double z, double rx, double ry, double rz) {
      double v[3] = {x, y, z};
      const double r[3] = {rx, ry, rz};
      for (int k = 0; k < 3; k++) {
        while (v[k] - r[k] > w[k] * 0.5) {
          v[k] -= w[k];
        }
        while (r[k] - v[k] > w[k] * 0.5) {
          v[k] += w[k];
        }
      }
      return std::array<double, 3>{v[0], v[1], v[2]};
    };

    int tag = 0;
    for (auto vit = T.unique_vertices_begin(); vit != T.unique_vertices_end(); ++vit) {
      const Vertex v = vit;
      const Point_3 site_p = T.point(v);
      const double sx = CGAL::to_double(site_p.x());
      const double sy = CGAL::to_double(site_p.y());
      const double sz = CGAL::to_double(site_p.z());
      /* A periodic Voronoi cell fits in the cube site ± period/2. */
      double cmin[3] = {sx - w[0] * 0.5, sy - w[1] * 0.5, sz - w[2] * 0.5};
      double cmax[3] = {sx + w[0] * 0.5, sy + w[1] * 0.5, sz + w[2] * 0.5};
      clip3::Poly3 cell = clip3::make_aabb(cmin, cmax);

      std::vector<Cell> cells;
      T.incident_cells(v, std::back_inserter(cells));
      std::set<std::array<long long, 3>> seen;
      bool alive = true;
      for (Cell c : cells) {
        for (int i = 0; i < 4; i++) {
          if (c->vertex(i) == v) {
            continue;
          }
          const Point_3 q0 = T.point(c, i);
          const auto q = unwrap3(CGAL::to_double(q0.x()),
                                 CGAL::to_double(q0.y()),
                                 CGAL::to_double(q0.z()),
                                 sx,
                                 sy,
                                 sz);
          const long long key[3] = {llround(q[0] * 1e7), llround(q[1] * 1e7), llround(q[2] * 1e7)};
          if (!seen.insert({key[0], key[1], key[2]}).second) {
            continue;
          }
          const double nx = q[0] - sx;
          const double ny = q[1] - sy;
          const double nz = q[2] - sz;
          if (nx * nx + ny * ny + nz * nz < 1e-24) {
            continue;
          }
          const double rhs = 0.5 * ((q[0] * q[0] + q[1] * q[1] + q[2] * q[2]) -
                                    (sx * sx + sy * sy + sz * sz));
          if (!clip3::clip_halfspace(cell, nx, ny, nz, rhs)) {
            alive = false;
            break;
          }
        }
        if (!alive) {
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
      result.error = "Periodic Voronoi 3D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Periodic Voronoi 3D failed";
  }
  return result;
}

WireResult points_gabriel_graph_3(const float *positions, int n)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!positions || n < 2) {
    result.error = "Gabriel Graph 3D needs at least 2 points";
    return result;
  }
  try {
    using Dt = CGAL::Delaunay_triangulation_3<Kernel>;
    Dt dt;
    std::vector<Point_3> pts;
    pts.reserve(size_t(n));
    result.positions.assign(positions, positions + n * 3);
    for (int i = 0; i < n; i++) {
      pts.push_back(xyz(double(positions[i * 3 + 0]),
                        double(positions[i * 3 + 1]),
                        double(positions[i * 3 + 2])));
      dt.insert(pts.back());
    }
    if (dt.number_of_vertices() < 2) {
      result.error = "Gabriel Graph 3D degenerate";
      return result;
    }
    std::map<Point_3, int> idx;
    for (int i = 0; i < n; i++) {
      idx[pts[size_t(i)]] = i;
    }
    auto id_of = [&](const Point_3 &p) -> int {
      const auto it = idx.find(p);
      return (it == idx.end()) ? -1 : it->second;
    };
    std::set<std::pair<int, int>> seen;
    for (auto eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
      const Point_3 &pa = eit->first->vertex(eit->second)->point();
      const Point_3 &pb = eit->first->vertex(eit->third)->point();
      const int ia = id_of(pa);
      const int ib = id_of(pb);
      if (ia < 0 || ib < 0 || ia == ib) {
        continue;
      }
      const Point_3 mid = CGAL::midpoint(pa, pb);
      const double r2 = CGAL::to_double(CGAL::squared_distance(pa, mid));
      bool ok = true;
      for (int i = 0; i < n; i++) {
        if (i == ia || i == ib) {
          continue;
        }
        if (CGAL::to_double(CGAL::squared_distance(pts[size_t(i)], mid)) < r2 - 1e-18) {
          ok = false;
          break;
        }
      }
      if (!ok) {
        continue;
      }
      const int a = std::min(ia, ib);
      const int b = std::max(ia, ib);
      if (seen.insert({a, b}).second) {
        add_wire(result, a, b);
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Gabriel Graph 3D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Gabriel Graph 3D failed";
  }
  return result;
}

WireResult points_euclidean_mst_3(const float *positions, int n)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!positions || n < 2) {
    result.error = "Euclidean MST 3D needs at least 2 points";
    return result;
  }
  try {
    /* Exact EMST ⊆ Delaunay. Build DT with spatial sort, then Borůvka:
     * each tree finds the shortest outgoing DT edge and the two trees merge.
     * O(log n) rounds, no global edge sort. */
    using Vb = CGAL::Triangulation_vertex_base_with_info_3<int, Kernel>;
    using Cb = CGAL::Delaunay_triangulation_cell_base_3<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_3<Vb, Cb>;
    using Dt = CGAL::Delaunay_triangulation_3<Kernel, Tds>;
    using IPoint = std::pair<Point_3, int>;
    using SortTraits = CGAL::Spatial_sort_traits_adapter_3<Kernel,
                                                           CGAL::First_of_pair_property_map<IPoint>>;

    result.positions.assign(positions, positions + n * 3);
    std::vector<IPoint> items;
    items.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      items.emplace_back(xyz(double(positions[i * 3 + 0]),
                             double(positions[i * 3 + 1]),
                             double(positions[i * 3 + 2])),
                         i);
    }
    CGAL::spatial_sort(items.begin(), items.end(), SortTraits());

    Dt dt;
    Dt::Cell_handle hint;
    std::vector<std::pair<int, int>> dupes;
    for (const IPoint &ip : items) {
      const int prev = int(dt.number_of_vertices());
      const auto vh = dt.insert(ip.first, hint);
      if (vh == Dt::Vertex_handle()) {
        continue;
      }
      hint = vh->cell();
      if (int(dt.number_of_vertices()) > prev) {
        vh->info() = ip.second;
      }
      else {
        dupes.emplace_back(vh->info(), ip.second);
      }
    }

    std::vector<std::vector<std::pair<int, double>>> adj;
    adj.resize(size_t(n));
    auto add_adj = [&](int a, int b, double d2) {
      if (a < 0 || b < 0 || a >= n || b >= n || a == b) {
        return;
      }
      adj[size_t(a)].push_back({b, d2});
      adj[size_t(b)].push_back({a, d2});
    };
    for (const auto &d : dupes) {
      add_adj(d.first, d.second, 0.0);
    }
    if (dt.dimension() >= 1) {
      for (auto eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
        const auto va = eit->first->vertex(eit->second);
        const auto vb = eit->first->vertex(eit->third);
        const int ia = va->info();
        const int ib = vb->info();
        add_adj(ia, ib, CGAL::to_double(CGAL::squared_distance(va->point(), vb->point())));
      }
    }
    if (adj.empty() || std::all_of(adj.begin(), adj.end(), [](const auto &v) { return v.empty(); })) {
      /* Degenerate: complete graph on tiny clouds. */
      for (int i = 0; i < n; i++) {
        const Point_3 a = xyz(double(positions[i * 3 + 0]),
                              double(positions[i * 3 + 1]),
                              double(positions[i * 3 + 2]));
        for (int j = i + 1; j < n; j++) {
          const Point_3 b = xyz(double(positions[j * 3 + 0]),
                                double(positions[j * 3 + 1]),
                                double(positions[j * 3 + 2]));
          add_adj(i, j, CGAL::to_double(CGAL::squared_distance(a, b)));
        }
      }
    }

    std::vector<int> parent;
    parent.resize(size_t(n));
    for (int i = 0; i < n; i++) {
      parent[size_t(i)] = i;
    }
    auto find = [&](auto &&self, int x) -> int {
      if (parent[size_t(x)] != x) {
        parent[size_t(x)] = self(self, parent[size_t(x)]);
      }
      return parent[size_t(x)];
    };

    std::vector<double> best_d;
    std::vector<int> best_a;
    std::vector<int> best_b;
    best_d.resize(size_t(n));
    best_a.resize(size_t(n));
    best_b.resize(size_t(n));

    int ncomp = n;
    const double inf = std::numeric_limits<double>::infinity();
    while (ncomp > 1) {
      std::fill(best_d.begin(), best_d.end(), inf);
      std::fill(best_a.begin(), best_a.end(), -1);
      std::fill(best_b.begin(), best_b.end(), -1);
      for (int i = 0; i < n; i++) {
        const int ri = find(find, i);
        for (const auto &nb : adj[size_t(i)]) {
          const int rj = find(find, nb.first);
          if (rj == ri) {
            continue;
          }
          if (nb.second < best_d[size_t(ri)]) {
            best_d[size_t(ri)] = nb.second;
            best_a[size_t(ri)] = i;
            best_b[size_t(ri)] = nb.first;
          }
        }
      }
      int merged = 0;
      for (int c = 0; c < n; c++) {
        if (best_b[size_t(c)] < 0) {
          continue;
        }
        const int a = best_a[size_t(c)];
        const int b = best_b[size_t(c)];
        const int ra = find(find, a);
        const int rb = find(find, b);
        if (ra == rb) {
          continue;
        }
        parent[size_t(ra)] = rb;
        add_wire(result, a, b);
        merged++;
        ncomp--;
        if (ncomp == 1) {
          break;
        }
      }
      if (merged == 0) {
        /* Remaining components have no DT edge — brute-force the shortest
         * bichromatic edge (same merge step, just a complete candidate set). */
        std::fill(best_d.begin(), best_d.end(), inf);
        std::fill(best_a.begin(), best_a.end(), -1);
        std::fill(best_b.begin(), best_b.end(), -1);
        for (int i = 0; i < n; i++) {
          const int ri = find(find, i);
          const Point_3 a = xyz(double(positions[i * 3 + 0]),
                                double(positions[i * 3 + 1]),
                                double(positions[i * 3 + 2]));
          for (int j = i + 1; j < n; j++) {
            const int rj = find(find, j);
            if (rj == ri) {
              continue;
            }
            const Point_3 b = xyz(double(positions[j * 3 + 0]),
                                  double(positions[j * 3 + 1]),
                                  double(positions[j * 3 + 2]));
            const double d2 = CGAL::to_double(CGAL::squared_distance(a, b));
            if (d2 < best_d[size_t(ri)]) {
              best_d[size_t(ri)] = d2;
              best_a[size_t(ri)] = i;
              best_b[size_t(ri)] = j;
            }
            if (d2 < best_d[size_t(rj)]) {
              best_d[size_t(rj)] = d2;
              best_a[size_t(rj)] = j;
              best_b[size_t(rj)] = i;
            }
          }
        }
        merged = 0;
        for (int c = 0; c < n; c++) {
          if (best_b[size_t(c)] < 0) {
            continue;
          }
          const int a = best_a[size_t(c)];
          const int b = best_b[size_t(c)];
          const int ra = find(find, a);
          const int rb = find(find, b);
          if (ra == rb) {
            continue;
          }
          parent[size_t(ra)] = rb;
          add_wire(result, a, b);
          merged++;
          ncomp--;
          if (ncomp == 1) {
            break;
          }
        }
        if (merged == 0) {
          break;
        }
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Euclidean MST 3D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Euclidean MST 3D failed";
  }
  return result;
}

WireResult mesh_euclidean_mst_3(const MeshIn &mesh)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!mesh.positions || mesh.verts_num < 2) {
    result.error = "Euclidean MST 3D needs at least 2 vertices";
    return result;
  }
  try {
    /* Mesh path: Borůvka on existing edges only. No Delaunay. */
    const int n = mesh.verts_num;
    result.positions.assign(mesh.positions, mesh.positions + n * 3);
    std::vector<std::vector<std::pair<int, double>>> adj;
    adj.resize(size_t(n));
    auto add_adj = [&](int a, int b) {
      if (a < 0 || b < 0 || a >= n || b >= n || a == b) {
        return;
      }
      const double dx = double(mesh.positions[a * 3 + 0]) - double(mesh.positions[b * 3 + 0]);
      const double dy = double(mesh.positions[a * 3 + 1]) - double(mesh.positions[b * 3 + 1]);
      const double dz = double(mesh.positions[a * 3 + 2]) - double(mesh.positions[b * 3 + 2]);
      const double d2 = dx * dx + dy * dy + dz * dz;
      adj[size_t(a)].push_back({b, d2});
      adj[size_t(b)].push_back({a, d2});
    };
    std::set<std::pair<int, int>> seen;
    auto consider = [&](int a, int b) {
      const int lo = (a < b) ? a : b;
      const int hi = (a < b) ? b : a;
      if (lo == hi || !seen.insert({lo, hi}).second) {
        return;
      }
      add_adj(lo, hi);
    };
    if (mesh.edges_num > 0 && mesh.edge_v0 && mesh.edge_v1) {
      for (int e = 0; e < mesh.edges_num; e++) {
        consider(mesh.edge_v0[e], mesh.edge_v1[e]);
      }
    }
    if (mesh.faces_num > 0 && mesh.corner_verts) {
      for (int f = 0; f < mesh.faces_num; f++) {
        int begin = 0, end = 0;
        if (mesh.face_offsets) {
          begin = mesh.face_offsets[f];
          end = mesh.face_offsets[f + 1];
        }
        else {
          begin = f * 3;
          end = begin + 3;
        }
        for (int i = begin; i < end; i++) {
          consider(mesh.corner_verts[i],
                   mesh.corner_verts[(i + 1 == end) ? begin : (i + 1)]);
        }
      }
    }
    if (seen.empty()) {
      result.error = "Mesh has no edges — MST cannot skip Delaunay without a graph";
      return result;
    }

    std::vector<int> parent;
    parent.resize(size_t(n));
    for (int i = 0; i < n; i++) {
      parent[size_t(i)] = i;
    }
    auto find = [&](auto &&self, int x) -> int {
      if (parent[size_t(x)] != x) {
        parent[size_t(x)] = self(self, parent[size_t(x)]);
      }
      return parent[size_t(x)];
    };
    std::vector<double> best_d;
    std::vector<int> best_a;
    std::vector<int> best_b;
    best_d.resize(size_t(n));
    best_a.resize(size_t(n));
    best_b.resize(size_t(n));
    int ncomp = n;
    const double inf = std::numeric_limits<double>::infinity();
    while (ncomp > 1) {
      std::fill(best_d.begin(), best_d.end(), inf);
      std::fill(best_a.begin(), best_a.end(), -1);
      std::fill(best_b.begin(), best_b.end(), -1);
      for (int i = 0; i < n; i++) {
        const int ri = find(find, i);
        for (const auto &nb : adj[size_t(i)]) {
          const int rj = find(find, nb.first);
          if (rj == ri) {
            continue;
          }
          if (nb.second < best_d[size_t(ri)]) {
            best_d[size_t(ri)] = nb.second;
            best_a[size_t(ri)] = i;
            best_b[size_t(ri)] = nb.first;
          }
        }
      }
      int merged = 0;
      for (int c = 0; c < n; c++) {
        if (best_b[size_t(c)] < 0) {
          continue;
        }
        const int a = best_a[size_t(c)];
        const int b = best_b[size_t(c)];
        const int ra = find(find, a);
        const int rb = find(find, b);
        if (ra == rb) {
          continue;
        }
        parent[size_t(ra)] = rb;
        add_wire(result, a, b);
        merged++;
        ncomp--;
        if (ncomp == 1) {
          break;
        }
      }
      if (merged == 0) {
        break;
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Euclidean MST 3D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Euclidean MST 3D failed";
  }
  return result;
}

WireResult points_beta_skeleton_2(const float *positions, int n, double beta)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!positions || n < 2) {
    result.error = "Beta Skeleton 2D needs at least 2 points";
    return result;
  }
  if (!(beta > 0.0) || !std::isfinite(beta)) {
    result.error = "Beta must be > 0";
    return result;
  }
  try {
    using Dt = CGAL::Delaunay_triangulation_2<Kernel>;
    Dt dt;
    std::vector<Point_2> pts;
    float zmid = 0.0f;
    pts.reserve(size_t(n));
    double zacc = 0.0;
    for (int i = 0; i < n; i++) {
      pts.push_back(xy(double(positions[i * 3 + 0]), double(positions[i * 3 + 1])));
      zacc += double(positions[i * 3 + 2]);
      dt.insert(pts.back());
    }
    zmid = float(zacc / double(n));
    result.positions.assign(positions, positions + n * 3);
    /* Keep Z of the original sites. */
    (void)zmid;
    std::map<Point_2, int> idx;
    for (int i = 0; i < n; i++) {
      idx[pts[size_t(i)]] = i;
    }
    auto empty_disk = [&](const Point_2 &c, double r2, int ia, int ib) {
      for (int i = 0; i < n; i++) {
        if (i == ia || i == ib) {
          continue;
        }
        if (CGAL::to_double(CGAL::squared_distance(pts[size_t(i)], c)) < r2 - 1e-18) {
          return false;
        }
      }
      return true;
    };
    std::set<std::pair<int, int>> seen;
    for (auto eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
      const Point_2 &pa = eit->first->vertex(eit->first->cw(eit->second))->point();
      const Point_2 &pb = eit->first->vertex(eit->first->ccw(eit->second))->point();
      const auto iait = idx.find(pa);
      const auto ibit = idx.find(pb);
      if (iait == idx.end() || ibit == idx.end()) {
        continue;
      }
      const int ia = iait->second;
      const int ib = ibit->second;
      const double ax = CGAL::to_double(pa.x());
      const double ay = CGAL::to_double(pa.y());
      const double bx = CGAL::to_double(pb.x());
      const double by = CGAL::to_double(pb.y());
      const double dx = bx - ax;
      const double dy = by - ay;
      const double d2 = dx * dx + dy * dy;
      if (d2 < 1e-24) {
        continue;
      }
      const double d = std::sqrt(d2);
      const double r = 0.5 * beta * d;
      const double r2 = r * r;
      const double mx = 0.5 * (ax + bx);
      const double my = 0.5 * (ay + by);
      bool keep = false;
      if (beta <= 1.0 + 1e-12) {
        /* Diametral / lune collapses toward the segment. */
        keep = empty_disk(xy(mx, my), r2, ia, ib);
      }
      else {
        const double h2 = r2 - 0.25 * d2;
        if (h2 < 0.0) {
          keep = empty_disk(xy(mx, my), r2, ia, ib);
        }
        else {
          const double h = std::sqrt(h2);
          const double px = -dy / d;
          const double py = dx / d;
          const Point_2 c1 = xy(mx + h * px, my + h * py);
          const Point_2 c2 = xy(mx - h * px, my - h * py);
          keep = empty_disk(c1, r2, ia, ib) && empty_disk(c2, r2, ia, ib);
        }
      }
      if (!keep) {
        continue;
      }
      const int a = std::min(ia, ib);
      const int b = std::max(ia, ib);
      if (seen.insert({a, b}).second) {
        add_wire(result, a, b);
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Beta Skeleton 2D empty (try a smaller Beta)";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Beta Skeleton 2D failed";
  }
  return result;
}

MeshResult points_convex_layers_2(const float *positions, int n)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!positions || n < 3) {
    result.error = "Convex Layers 2D needs at least 3 points";
    return result;
  }
  try {
    std::vector<Point_2> remain;
    remain.reserve(size_t(n));
    double zacc = 0.0;
    for (int i = 0; i < n; i++) {
      remain.push_back(xy(double(positions[i * 3 + 0]), double(positions[i * 3 + 1])));
      zacc += double(positions[i * 3 + 2]);
    }
    const float zmid = float(zacc / double(n));
    result.face_offsets = {0};
    int layer = 0;
    while (remain.size() >= 3) {
      std::vector<Point_2> hull;
      CGAL::convex_hull_2(remain.begin(), remain.end(), std::back_inserter(hull));
      if (hull.size() < 3) {
        break;
      }
      std::set<std::pair<long long, long long>> on_hull;
      for (const Point_2 &p : hull) {
        on_hull.insert(key2(p));
        push_xy0(result.positions, p, zmid);
        result.corner_verts.push_back(int(result.corner_verts.size()));
      }
      result.face_offsets.push_back(int(result.corner_verts.size()));
      result.face_tag.push_back(layer++);
      std::vector<Point_2> next;
      next.reserve(remain.size() - hull.size());
      for (const Point_2 &p : remain) {
        if (on_hull.find(key2(p)) == on_hull.end()) {
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
      result.error = "Convex Layers 2D produced no hulls";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Convex Layers 2D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
