/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 49 (2026-08-21, new only — do not restore deleted ids):
 *  - Curve Mesh Intersect (AABB Segment_3 vs triangles)
 *  - Geodesic Isolines (MMP geodesic distance isolines)
 *  - Overlap Faces (AABB + triangle-triangle, subset of mesh A)
 *  - Contour Stack (parallel Polygon_mesh_slicer planes)
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

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/intersections.h>
#include <CGAL/number_utils.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <list>
#include <map>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point = Point_3;
using Vector_3 = Kernel::Vector_3;
using Segment_3 = Kernel::Segment_3;
using Triangle_3 = Kernel::Triangle_3;
using Plane_3 = Kernel::Plane_3;
using VI = Surface_mesh::Vertex_index;
using FI = Surface_mesh::Face_index;

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  sm.collect_garbage();
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

static Triangle_3 triangle_of(const Surface_mesh &sm, FI f)
{
  const auto h = sm.halfedge(f);
  return Triangle_3(sm.point(sm.source(h)),
                    sm.point(sm.target(h)),
                    sm.point(sm.target(sm.next(h))));
}

static void pack_point(std::vector<float> &xyz, const Point &p)
{
  xyz.push_back(float(CGAL::to_double(p.x())));
  xyz.push_back(float(CGAL::to_double(p.y())));
  xyz.push_back(float(CGAL::to_double(p.z())));
}

static void pack_segment(std::vector<std::vector<float>> &out, const Point &a, const Point &b)
{
  if (CGAL::squared_distance(a, b) <= Kernel::FT(0)) {
    return;
  }
  std::vector<float> xyz;
  pack_point(xyz, a);
  pack_point(xyz, b);
  out.push_back(std::move(xyz));
}

struct QKey {
  long long x, y, z;
  bool operator<(const QKey &o) const
  {
    if (x != o.x) {
      return x < o.x;
    }
    if (y != o.y) {
      return y < o.y;
    }
    return z < o.z;
  }
};

static QKey quant(const Point &p)
{
  const double s = 1.0e5;
  return {std::llround(CGAL::to_double(p.x()) * s),
          std::llround(CGAL::to_double(p.y()) * s),
          std::llround(CGAL::to_double(p.z()) * s)};
}

static void chain_segments(const std::vector<std::pair<Point, Point>> &segs,
                           std::vector<std::vector<float>> &out)
{
  if (segs.empty()) {
    return;
  }
  struct Adj {
    QKey other;
    Point pa;
    Point pb;
    bool used = false;
  };
  std::map<QKey, std::vector<size_t>> at;
  std::vector<Adj> adj;
  adj.reserve(segs.size() * 2);
  for (const auto &e : segs) {
    if (CGAL::squared_distance(e.first, e.second) <= Kernel::FT(0)) {
      continue;
    }
    const QKey ka = quant(e.first);
    const QKey kb = quant(e.second);
    if (!(ka < kb) && !(kb < ka)) {
      continue;
    }
    const size_t i0 = adj.size();
    adj.push_back({kb, e.first, e.second, false});
    at[ka].push_back(i0);
    const size_t i1 = adj.size();
    adj.push_back({ka, e.second, e.first, false});
    at[kb].push_back(i1);
  }
  auto unused_degree = [&](const QKey &k) {
    int n = 0;
    auto it = at.find(k);
    if (it == at.end()) {
      return 0;
    }
    for (const size_t i : it->second) {
      if (!adj[i].used) {
        n++;
      }
    }
    return n;
  };
  auto walk = [&](QKey start) {
    std::vector<float> xyz;
    QKey cur = start;
    while (true) {
      auto it = at.find(cur);
      if (it == at.end()) {
        break;
      }
      size_t pick = size_t(-1);
      for (const size_t i : it->second) {
        if (!adj[i].used) {
          pick = i;
          break;
        }
      }
      if (pick == size_t(-1)) {
        break;
      }
      adj[pick].used = true;
      /* Mark reverse. */
      auto oit = at.find(adj[pick].other);
      if (oit != at.end()) {
        for (const size_t j : oit->second) {
          if (!adj[j].used && !(adj[j].other < cur) && !(cur < adj[j].other)) {
            adj[j].used = true;
            break;
          }
        }
      }
      if (xyz.empty()) {
        pack_point(xyz, adj[pick].pa);
      }
      pack_point(xyz, adj[pick].pb);
      cur = adj[pick].other;
    }
    if (xyz.size() >= 6) {
      out.push_back(std::move(xyz));
    }
  };
  /* Open chains first, then leftover loops. */
  for (const auto &kv : at) {
    if (unused_degree(kv.first) == 1) {
      walk(kv.first);
    }
  }
  for (const auto &kv : at) {
    if (unused_degree(kv.first) > 0) {
      walk(kv.first);
    }
  }
}

static Vector_3 unit_or_z(float dx, float dy, float dz)
{
  const double ln = std::sqrt(double(dx) * dx + double(dy) * dy + double(dz) * dz);
  if (ln < 1e-12) {
    return Vector_3(0, 0, 1);
  }
  return Vector_3(dx / float(ln), dy / float(ln), dz / float(ln));
}

}  // namespace

bool mesh_curve_intersect(const MeshIn &mesh,
                          const float *curve_xyz,
                          int points_num,
                          const int *curve_offsets,
                          int curves_num,
                          std::vector<std::vector<float>> &out_segments,
                          std::vector<float> &out_points,
                          std::string &error)
{
  out_segments.clear();
  out_points.clear();
  CgalThrowGuard guard;
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    error = error.empty() ? "Curve Mesh Intersect needs a mesh with faces" : error;
    return false;
  }
  if (!curve_xyz || !curve_offsets || curves_num <= 0 || points_num < 2) {
    error = "Curve Mesh Intersect needs polylines";
    return false;
  }
  try {
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
    using Traits = CGAL::AABB_traits_3<Kernel, Primitive>;
    using Tree = CGAL::AABB_tree<Traits>;
    Tree tree(faces(sm).first, faces(sm).second, sm);
    tree.accelerate_distance_queries();

    for (int c = 0; c < curves_num; c++) {
      const int a = curve_offsets[c];
      const int b = curve_offsets[c + 1];
      if (a < 0 || b > points_num || b - a < 2) {
        continue;
      }
      for (int i = a; i + 1 < b; i++) {
        const Point p0(curve_xyz[i * 3 + 0], curve_xyz[i * 3 + 1], curve_xyz[i * 3 + 2]);
        const Point p1(curve_xyz[(i + 1) * 3 + 0],
                       curve_xyz[(i + 1) * 3 + 1],
                       curve_xyz[(i + 1) * 3 + 2]);
        if (CGAL::squared_distance(p0, p1) <= Kernel::FT(0)) {
          continue;
        }
        const Segment_3 seg(p0, p1);
        std::vector<FI> prims;
        tree.all_intersected_primitives(seg, std::back_inserter(prims));
        for (const FI f : prims) {
          const Triangle_3 tri = triangle_of(sm, f);
          const auto hit = CGAL::intersection(seg, tri);
          if (!hit) {
            continue;
          }
          if (const Segment_3 *s = std::get_if<Segment_3>(&*hit)) {
            pack_segment(out_segments, s->source(), s->target());
          }
          else if (const Point *p = std::get_if<Point>(&*hit)) {
            pack_point(out_points, *p);
          }
        }
      }
    }
    if (out_segments.empty() && out_points.size() < 3) {
      error = "No curve-mesh intersections";
      return false;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
  }
  catch (...) {
    error = "Curve Mesh Intersect failed";
  }
  return false;
}

bool mesh_geodesic_isolines(const MeshIn &mesh,
                            const uint8_t *source_mask,
                            int count,
                            float max_distance,
                            std::vector<std::vector<float>> &out_polylines,
                            std::string &error)
{
  out_polylines.clear();
  CgalThrowGuard guard;
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    error = error.empty() ? "Geodesic Isolines needs a mesh with faces" : error;
    return false;
  }
  if (!source_mask) {
    error = "Geodesic Isolines needs source vertices";
    return false;
  }
  count = std::clamp(count, 1, 256);
  try {
    using Traits = CGAL::Surface_mesh_shortest_path_traits<Kernel, Surface_mesh>;
    using Shortest_paths = CGAL::Surface_mesh_shortest_path<Traits>;
    Shortest_paths sp(sm);
    int n_src = 0;
    int i = 0;
    for (const VI v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      if (i < mesh.verts_num && source_mask[i]) {
        sp.add_source_point(v);
        n_src++;
      }
      i++;
    }
    if (n_src == 0) {
      error = "No valid source vertices on mesh";
      return false;
    }

    std::vector<double> dist(size_t(mesh.verts_num), -1.0);
    i = 0;
    double dmax = 0.0;
    for (const VI v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      if (i >= mesh.verts_num) {
        break;
      }
      const auto res = sp.shortest_distance_to_source_points(v);
      const double d = CGAL::to_double(res.first);
      dist[size_t(i)] = (d < 0.0) ? -1.0 : d;
      if (d > dmax) {
        dmax = d;
      }
      i++;
    }
    if (max_distance > 0.0f) {
      dmax = std::min(dmax, double(max_distance));
    }
    if (dmax <= 1e-12) {
      error = "Geodesic distances are zero";
      return false;
    }

    auto vid = [](VI v) -> unsigned { return unsigned(v); };

    for (int k = 1; k <= count; k++) {
      const double L = dmax * double(k) / double(count + 1);
      std::vector<std::pair<Point, Point>> segs;
      for (const FI f : sm.faces()) {
        if (sm.is_removed(f)) {
          continue;
        }
        const auto h = sm.halfedge(f);
        const VI vs[3] = {sm.source(h), sm.target(h), sm.target(sm.next(h))};
        const unsigned is[3] = {vid(vs[0]), vid(vs[1]), vid(vs[2])};
        if (is[0] >= dist.size() || is[1] >= dist.size() || is[2] >= dist.size()) {
          continue;
        }
        const double d[3] = {dist[is[0]], dist[is[1]], dist[is[2]]};
        if (d[0] < 0.0 || d[1] < 0.0 || d[2] < 0.0) {
          continue;
        }
        Point hits[3];
        int nh = 0;
        for (int e = 0; e < 3; e++) {
          const int a = e;
          const int b = (e + 1) % 3;
          const double da = d[a] - L;
          const double db = d[b] - L;
          if (da == 0.0 && db == 0.0) {
            segs.push_back({sm.point(vs[a]), sm.point(vs[b])});
            nh = 0;
            break;
          }
          if (da * db < 0.0) {
            const double t = da / (da - db);
            const Point &pa = sm.point(vs[a]);
            const Point &pb = sm.point(vs[b]);
            hits[nh++] = pa + (pb - pa) * Kernel::FT(t);
          }
        }
        if (nh >= 2) {
          segs.push_back({hits[0], hits[1]});
        }
      }
      chain_segments(segs, out_polylines);
    }
    if (out_polylines.empty()) {
      error = "No geodesic isolines";
      return false;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
  }
  catch (...) {
    error = "Geodesic Isolines failed";
  }
  return false;
}

MeshResult mesh_overlap_faces(const MeshIn &mesh, const MeshIn &other)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm_a;
  Surface_mesh sm_b;
  std::string error;
  if (!load_tri(mesh, sm_a, error) || !load_tri(other, sm_b, error)) {
    result.error = error.empty() ? "Overlap Faces needs two meshes with faces" : error;
    return result;
  }
  try {
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
    using Traits = CGAL::AABB_traits_3<Kernel, Primitive>;
    using Tree = CGAL::AABB_tree<Traits>;
    Tree tree(faces(sm_b).first, faces(sm_b).second, sm_b);

    auto fsrc = sm_a.property_map<FI, int>(PROP_F_SRC);
    auto vsrc0 = sm_a.property_map<VI, int>(PROP_V_SRC0);

    std::map<int, int> vmap;
    auto emit_vert = [&](VI v) -> int {
      const int orig = (vsrc0.has_value() && (*vsrc0)[v] >= 0) ? (*vsrc0)[v] : int(v);
      auto it = vmap.find(int(v));
      if (it != vmap.end()) {
        return it->second;
      }
      const Point &p = sm_a.point(v);
      const int ni = result.verts_num();
      pack_point(result.positions, p);
      result.vert_src0.push_back(orig);
      result.vert_src1.push_back(-1);
      result.vert_factor.push_back(0.0f);
      vmap[int(v)] = ni;
      return ni;
    };

    int kept = 0;
    for (const FI f : sm_a.faces()) {
      if (sm_a.is_removed(f)) {
        continue;
      }
      const Triangle_3 tri = triangle_of(sm_a, f);
      if (!tree.do_intersect(tri)) {
        continue;
      }
      const auto h = sm_a.halfedge(f);
      const int a = emit_vert(sm_a.source(h));
      const int b = emit_vert(sm_a.target(h));
      const int c = emit_vert(sm_a.target(sm_a.next(h)));
      if (a == b || b == c || c == a) {
        continue;
      }
      result.corner_verts.push_back(a);
      result.corner_verts.push_back(b);
      result.corner_verts.push_back(c);
      const int src = (fsrc.has_value() && (*fsrc)[f] >= 0) ? (*fsrc)[f] : int(f);
      result.face_src.push_back(src);
      kept++;
    }
    if (kept == 0) {
      result.error = "No overlapping faces";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Overlap Faces failed";
  }
  return result;
}

bool mesh_contour_stack(const MeshIn &mesh,
                        float ox,
                        float oy,
                        float oz,
                        float dx,
                        float dy,
                        float dz,
                        int count,
                        float spacing,
                        std::vector<std::vector<float>> &out_polylines,
                        std::string &error)
{
  out_polylines.clear();
  CgalThrowGuard guard;
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    error = error.empty() ? "Contour Stack needs a mesh with faces" : error;
    return false;
  }
  count = std::clamp(count, 1, 256);
  try {
    const Vector_3 n = unit_or_z(dx, dy, dz);
    const Point origin(ox, oy, oz);
    std::vector<double> ts;
    ts.reserve(size_t(count));
    if (spacing > 0.0f) {
      for (int i = 0; i < count; i++) {
        ts.push_back(double(i) * double(spacing));
      }
    }
    else {
      double tmin = 1e300;
      double tmax = -1e300;
      for (const VI v : sm.vertices()) {
        if (sm.is_removed(v)) {
          continue;
        }
        const Vector_3 d = sm.point(v) - origin;
        const double t = CGAL::to_double(d * n);
        tmin = std::min(tmin, t);
        tmax = std::max(tmax, t);
      }
      if (!(tmax > tmin + 1e-12)) {
        error = "Mesh is flat along Direction";
        return false;
      }
      for (int i = 1; i <= count; i++) {
        ts.push_back(tmin + (tmax - tmin) * double(i) / double(count + 1));
      }
    }

    CGAL::Polygon_mesh_slicer<Surface_mesh, Kernel> slicer(sm);
    for (const double t : ts) {
      const Point q = origin + n * Kernel::FT(t);
      const Plane_3 plane(q, n);
      std::vector<std::vector<Point>> polylines;
      slicer(plane, std::back_inserter(polylines));
      for (const auto &pl : polylines) {
        if (pl.size() < 2) {
          continue;
        }
        std::vector<float> xyz;
        xyz.reserve(pl.size() * 3);
        for (const Point &p : pl) {
          pack_point(xyz, p);
        }
        out_polylines.push_back(std::move(xyz));
      }
    }
    if (out_polylines.empty()) {
      error = "No contour curves";
      return false;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
  }
  catch (...) {
    error = "Contour Stack failed";
  }
  return false;
}

}  // namespace blender::cgal_bridge
