/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 50 (2026-08-21, new only — do not restore deleted ids):
 *  - Alpha Edges 3D (Alpha_shape_3 REGULAR+SINGULAR edges)
 *  - Delaunay Edges 3D (DT 1-skeleton)
 *  - Mesh Edge Path (Dijkstra on mesh edges)
 *  - Curvature Isolines (mean/Gaussian isolines)
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

#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Polygon_mesh_processing/interpolated_corrected_curvatures.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/number_utils.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point = Point_3;
using VI = Surface_mesh::Vertex_index;
using FI = Surface_mesh::Face_index;

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error, bool garbage)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  if (garbage) {
    sm.collect_garbage();
  }
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

static void pack_point(std::vector<float> &xyz, const Point &p)
{
  xyz.push_back(float(CGAL::to_double(p.x())));
  xyz.push_back(float(CGAL::to_double(p.y())));
  xyz.push_back(float(CGAL::to_double(p.z())));
}

static WireResult positions_copy(const float *positions, int n)
{
  WireResult result;
  result.positions.assign(positions, positions + n * 3);
  return result;
}

static void add_edge(WireResult &result, std::set<std::pair<int, int>> &seen, int a, int b, int n)
{
  if (a < 0 || b < 0 || a >= n || b >= n || a == b) {
    return;
  }
  if (a > b) {
    std::swap(a, b);
  }
  if (!seen.insert({a, b}).second) {
    return;
  }
  result.edge_v0.push_back(a);
  result.edge_v1.push_back(b);
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

static double bbox_alpha_fallback(const std::vector<Point> &pts)
{
  double minx = CGAL::to_double(pts[0].x()), maxx = minx;
  double miny = CGAL::to_double(pts[0].y()), maxy = miny;
  double minz = CGAL::to_double(pts[0].z()), maxz = minz;
  for (const Point &p : pts) {
    const double x = CGAL::to_double(p.x());
    const double y = CGAL::to_double(p.y());
    const double z = CGAL::to_double(p.z());
    minx = std::min(minx, x);
    maxx = std::max(maxx, x);
    miny = std::min(miny, y);
    maxy = std::max(maxy, y);
    minz = std::min(minz, z);
    maxz = std::max(maxz, z);
  }
  const double dx = maxx - minx, dy = maxy - miny, dz = maxz - minz;
  const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
  return (diag > 0.0) ? (diag * diag) * 0.01 : 1.0;
}

}  // namespace

WireResult points_alpha_edges_3(const float *positions,
                                int n,
                                double alpha,
                                bool use_optimal_alpha,
                                int solid_components)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!positions || n < 4) {
    result.error = "Alpha Edges 3D needs at least 4 points";
    return result;
  }
  try {
    using Vb0 = CGAL::Triangulation_vertex_base_with_info_3<int, Kernel>;
    using Vb = CGAL::Alpha_shape_vertex_base_3<Kernel, Vb0>;
    using Fb = CGAL::Alpha_shape_cell_base_3<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_3<Vb, Fb>;
    using Triangulation = CGAL::Delaunay_triangulation_3<Kernel, Tds>;
    using Alpha_shape = CGAL::Alpha_shape_3<Triangulation>;

    std::vector<Point> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
    }
    Alpha_shape as(pts.begin(), pts.end(), 0, Alpha_shape::GENERAL);
    std::map<QKey, int> orig;
    for (int k = 0; k < n; k++) {
      orig[quant(pts[size_t(k)])] = k;
    }
    for (auto vit = as.finite_vertices_begin(); vit != as.finite_vertices_end(); ++vit) {
      auto it = orig.find(quant(vit->point()));
      vit->info() = (it == orig.end()) ? -1 : it->second;
    }

    solid_components = std::max(solid_components, 1);
    if (use_optimal_alpha) {
      auto opt_it = as.find_optimal_alpha(solid_components);
      as.set_alpha(*opt_it);
    }
    else {
      double alpha_val = alpha;
      if (!(alpha_val > 0.0)) {
        alpha_val = bbox_alpha_fallback(pts);
      }
      as.set_alpha(Alpha_shape::NT(alpha_val));
    }

    std::vector<Alpha_shape::Edge> edges;
    as.get_alpha_shape_edges(std::back_inserter(edges), Alpha_shape::REGULAR);
    as.get_alpha_shape_edges(std::back_inserter(edges), Alpha_shape::SINGULAR);
    result = positions_copy(positions, n);
    std::set<std::pair<int, int>> seen;
    for (const Alpha_shape::Edge &e : edges) {
      const auto va = e.first->vertex(e.second);
      const auto vb = e.first->vertex(e.third);
      add_edge(result, seen, va->info(), vb->info(), n);
    }
    if (result.edge_v0.empty()) {
      result.error =
          "Alpha Edges 3D produced no edges. Try manual Alpha around "
          "(point spacing)^2, or denser points.";
      result.positions.clear();
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Alpha Edges 3D failed";
  }
  return result;
}

WireResult points_delaunay_edges_3(const float *positions, int n)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!positions || n < 2) {
    result.error = "Delaunay Edges 3D needs at least 2 points";
    return result;
  }
  try {
    using Vb = CGAL::Triangulation_vertex_base_with_info_3<int, Kernel>;
    using Cb = CGAL::Delaunay_triangulation_cell_base_3<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_3<Vb, Cb>;
    using Dt = CGAL::Delaunay_triangulation_3<Kernel, Tds>;

    Dt dt;
    Dt::Cell_handle hint;
    for (int i = 0; i < n; i++) {
      const Point p(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
      const int prev = int(dt.number_of_vertices());
      const auto vh = dt.insert(p, hint);
      if (vh == Dt::Vertex_handle()) {
        continue;
      }
      hint = vh->cell();
      if (int(dt.number_of_vertices()) > prev) {
        vh->info() = i;
      }
    }
    result = positions_copy(positions, n);
    std::set<std::pair<int, int>> seen;
    if (dt.dimension() >= 1) {
      for (auto eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
        const auto va = eit->first->vertex(eit->second);
        const auto vb = eit->first->vertex(eit->third);
        add_edge(result, seen, va->info(), vb->info(), n);
      }
    }
    if (result.edge_v0.empty()) {
      result.error = "Delaunay Edges 3D produced no edges";
      result.positions.clear();
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Delaunay Edges 3D failed";
  }
  return result;
}

bool mesh_edge_path(const MeshIn &mesh,
                    const uint8_t *source_mask,
                    const uint8_t *target_mask,
                    std::vector<float> &out_xyz,
                    std::string &error)
{
  out_xyz.clear();
  CgalThrowGuard guard;
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error, false)) {
    error = error.empty() ? "Mesh Edge Path needs a mesh with faces" : error;
    return false;
  }
  if (!source_mask || !target_mask) {
    error = "Mesh Edge Path needs Source and Target vertices";
    return false;
  }
  try {
    const int n = mesh.verts_num;
    std::vector<std::vector<std::pair<int, double>>> adjacency;
    adjacency.resize(std::size_t(n));
    auto vid = [&](VI v) -> int {
      const unsigned u = unsigned(v);
      return (u < unsigned(n)) ? int(u) : -1;
    };
    const int ecount = int(sm.number_of_edges());
    for (int ei = 0; ei < ecount; ei++) {
      const Surface_mesh::Edge_index e(static_cast<Surface_mesh::size_type>(ei));
      if (!sm.is_valid(e) || sm.is_removed(e)) {
        continue;
      }
      const Surface_mesh::Halfedge_index h = sm.halfedge(e);
      const int a = vid(sm.source(h));
      const int b = vid(sm.target(h));
      if (a < 0 || b < 0 || a == b) {
        continue;
      }
      const double w = std::sqrt(
          CGAL::to_double(CGAL::squared_distance(sm.point(sm.source(h)), sm.point(sm.target(h)))));
      adjacency[size_t(a)].emplace_back(b, w);
      adjacency[size_t(b)].emplace_back(a, w);
    }

    constexpr double inf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(size_t(n), inf);
    std::vector<int> parent(size_t(n), -1);
    std::vector<char> is_tgt(size_t(n), 0);
    using Node = std::pair<double, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    int n_src = 0;
    int n_tgt = 0;
    for (int i = 0; i < n; i++) {
      if (source_mask[i]) {
        dist[size_t(i)] = 0.0;
        pq.push(Node(0.0, i));
        n_src++;
      }
      if (target_mask[i]) {
        is_tgt[size_t(i)] = 1;
        n_tgt++;
      }
    }
    if (n_src == 0 || n_tgt == 0) {
      error = "Mesh Edge Path needs at least one Source and one Target vertex";
      return false;
    }

    int hit = -1;
    while (!pq.empty()) {
      const Node cur = pq.top();
      pq.pop();
      const double d = cur.first;
      const int u = cur.second;
      if (d > dist[size_t(u)]) {
        continue;
      }
      if (is_tgt[size_t(u)]) {
        hit = u;
        break;
      }
      for (size_t k = 0; k < adjacency[size_t(u)].size(); k++) {
        const int nb = adjacency[size_t(u)][k].first;
        const double nd = d + adjacency[size_t(u)][k].second;
        if (nd + 1e-15 < dist[size_t(nb)]) {
          dist[size_t(nb)] = nd;
          parent[size_t(nb)] = u;
          pq.push(Node(nd, nb));
        }
      }
    }
    if (hit < 0) {
      error = "No edge-path from Source to Target (disconnected mesh)";
      return false;
    }
    std::vector<int> path;
    for (int v = hit; v >= 0; v = parent[size_t(v)]) {
      path.push_back(v);
    }
    std::reverse(path.begin(), path.end());
    if (path.size() < 2) {
      error = "Source and Target are the same vertex";
      return false;
    }
    for (const int v : path) {
      if (v < 0 || v >= n || !mesh.positions) {
        continue;
      }
      out_xyz.push_back(mesh.positions[v * 3 + 0]);
      out_xyz.push_back(mesh.positions[v * 3 + 1]);
      out_xyz.push_back(mesh.positions[v * 3 + 2]);
    }
    return out_xyz.size() >= 6;
  }
  catch (const std::exception &e) {
    error = e.what();
  }
  catch (...) {
    error = "Mesh Edge Path failed";
  }
  return false;
}

bool mesh_curvature_isolines(const MeshIn &mesh,
                             int mode,
                             int count,
                             std::vector<std::vector<float>> &out_polylines,
                             std::string &error)
{
  out_polylines.clear();
  CgalThrowGuard guard;
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error, true)) {
    error = error.empty() ? "Curvature Isolines needs a mesh with faces" : error;
    return false;
  }
  count = std::clamp(count, 1, 256);
  try {
    auto hmap = sm.add_property_map<VI, double>("v:H", 0.0).first;
    auto kmap = sm.add_property_map<VI, double>("v:K", 0.0).first;
    if (mode == 1) {
      PMP::interpolated_corrected_curvatures(
          sm, CGAL::parameters::vertex_Gaussian_curvature_map(kmap));
    }
    else {
      PMP::interpolated_corrected_curvatures(
          sm, CGAL::parameters::vertex_mean_curvature_map(hmap));
    }

    std::vector<double> val(size_t(mesh.verts_num), 0.0);
    int i = 0;
    double vmin = 1e300;
    double vmax = -1e300;
    for (const VI v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      if (i >= mesh.verts_num) {
        break;
      }
      const double x = (mode == 1) ? kmap[v] : hmap[v];
      val[size_t(i)] = x;
      vmin = std::min(vmin, x);
      vmax = std::max(vmax, x);
      i++;
    }
    if (!(vmax > vmin + 1e-12)) {
      error = "Curvature is constant; no isolines";
      return false;
    }

    auto vid = [](VI v) -> unsigned { return unsigned(v); };
    for (int k = 1; k <= count; k++) {
      const double L = vmin + (vmax - vmin) * double(k) / double(count + 1);
      std::vector<std::pair<Point, Point>> segs;
      for (const FI f : sm.faces()) {
        if (sm.is_removed(f)) {
          continue;
        }
        const auto h = sm.halfedge(f);
        const VI vs[3] = {sm.source(h), sm.target(h), sm.target(sm.next(h))};
        const unsigned is[3] = {vid(vs[0]), vid(vs[1]), vid(vs[2])};
        if (is[0] >= val.size() || is[1] >= val.size() || is[2] >= val.size()) {
          continue;
        }
        const double d[3] = {val[is[0]], val[is[1]], val[is[2]]};
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
      error = "No curvature isolines";
      return false;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
  }
  catch (...) {
    error = "Curvature Isolines failed";
  }
  return false;
}

}  // namespace blender::cgal_bridge
