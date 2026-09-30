/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 34 — new only (not previously deleted):
 *  - Do Intersect 3D
 *  - Split by Mesh
 *  - Natural Neighbor 2D
 *  - Cage Deform 2D
 *  - Proximity Graph 2D
 *  - Fill Polyline
 *  - Regularize Planes
 *
 * Skipped: P22 OpenGR, G10 Kinetic EPECK, Mesh_3 volume, PolyFit SCIP,
 * Ridges_3, Classification, OSQP Regularize Segments, previously banned
 * measure/query/remesh/component nodes.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Barycentric_coordinates_2/Discrete_harmonic_coordinates_2.h>
#include <CGAL/Barycentric_coordinates_2/Mean_value_coordinates_2.h>
#include <CGAL/Barycentric_coordinates_2/Wachspress_coordinates_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Spatial_sort_traits_adapter_2.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/number_utils.h>
#include <CGAL/spatial_sort.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Shape_regularization/regularize_planes.h>
#include <CGAL/assertions_behaviour.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/natural_neighbor_coordinates_2.h>
#include <CGAL/property_map.h>
#include <CGAL/squared_distance_2.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <set>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Plane_3 = Kernel::Plane_3;
using Triangle_3 = Kernel::Triangle_3;
using Dt2 = CGAL::Delaunay_triangulation_2<Kernel>;

struct NnSite {
  int index = -1;
};
using NnVb = CGAL::Triangulation_vertex_base_with_info_2<NnSite, Kernel>;
using NnFb = CGAL::Triangulation_face_base_2<Kernel>;
using NnTds = CGAL::Triangulation_data_structure_2<NnVb, NnFb>;
using NnDt2 = CGAL::Delaunay_triangulation_2<Kernel, NnTds>;
using NnIPoint = std::pair<Point_2, int>;
using NnSortTraits = CGAL::Spatial_sort_traits_adapter_2<Kernel,
                                                         CGAL::First_of_pair_property_map<NnIPoint>>;

static void ensure_cgal_exceptions()
{
  static std::once_flag once;
  std::call_once(once, []() {
    CGAL::set_error_behaviour(CGAL::THROW_EXCEPTION);
    CGAL::set_warning_behaviour(CGAL::CONTINUE);
    CGAL::set_error_handler(&cgal_throw_failure);
    CGAL::set_warning_handler(&cgal_ignore_failure);
  });
}

static Point_2 to_p2(const float *xyz)
{
  return Point_2(double(xyz[0]), double(xyz[1]));
}

static Point_3 to_p3(const float *xyz)
{
  return Point_3(double(xyz[0]), double(xyz[1]), double(xyz[2]));
}

struct UnionFind {
  std::vector<int> p;
  explicit UnionFind(int n) : p(size_t(n))
  {
    std::iota(p.begin(), p.end(), 0);
  }
  int find(int x)
  {
    while (p[size_t(x)] != x) {
      p[size_t(x)] = p[size_t(p[size_t(x)])];
      x = p[size_t(x)];
    }
    return x;
  }
  bool unite(int a, int b)
  {
    a = find(a);
    b = find(b);
    if (a == b) {
      return false;
    }
    p[size_t(b)] = a;
    return true;
  }
};

static bool mesh_to_sm(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  if (sm.number_of_faces() == 0) {
    error = "Mesh needs faces";
    return false;
  }
  triangulate_faces_keep_ids(sm);
  return sm.number_of_faces() > 0;
}

}  // namespace

bool mesh_do_intersect_3(const MeshIn &mesh_a, const MeshIn &mesh_b, std::string &error)
{
  CgalThrowGuard guard;
  try {
    Surface_mesh a, b;
    if (!mesh_to_sm(mesh_a, a, error) || !mesh_to_sm(mesh_b, b, error)) {
      return false;
    }
    return PMP::do_intersect(a, b);
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Do Intersect 3D failed";
    return false;
  }
}

MeshResult mesh_split_by_mesh(const MeshIn &mesh, const MeshIn &cutter)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    Surface_mesh tm, splitter;
    std::string error;
    if (!mesh_to_sm(mesh, tm, error)) {
      result.error = error.empty() ? "Split by Mesh needs a surface" : error;
      return result;
    }
    if (!mesh_to_sm(cutter, splitter, error)) {
      result.error = error.empty() ? "Split by Mesh needs a cutter surface" : error;
      return result;
    }
    PMP::split(tm, splitter);
    result = surface_mesh_to_result(tm);
    if (result.ok && result.faces_num() > 0) {
      auto fccmap = tm.add_property_map<Surface_mesh::Face_index, int>("f:cc", -1).first;
      const int ncc = int(PMP::connected_components(tm, fccmap));
      result.face_tag.assign(size_t(result.faces_num()), -1);
      result.radius = float(ncc);
      int fi = 0;
      for (const Surface_mesh::Face_index f : tm.faces()) {
        if (fi < result.faces_num()) {
          result.face_tag[size_t(fi)] = fccmap[f];
        }
        fi++;
      }
    }
    if (!result.ok) {
      result.error = result.error.empty() ? "Split by Mesh produced an empty mesh" : result.error;
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
    result.ok = false;
  }
  catch (...) {
    result.error = "Split by Mesh failed";
    result.ok = false;
  }
  return result;
}

struct NaturalNeighbor2 {
  NnDt2 dt;
  int sites_n = 0;
};

NaturalNeighbor2 *natural_neighbor_2_new(const float *sites_xyz, int sites_n, std::string &error)
{
  if (!sites_xyz || sites_n < 3) {
    error = "Natural Neighbor 2D needs ≥3 sites";
    return nullptr;
  }
  ensure_cgal_exceptions();
  try {
    std::vector<NnIPoint> points;
    points.reserve(size_t(sites_n));
    for (int i = 0; i < sites_n; i++) {
      points.emplace_back(to_p2(sites_xyz + i * 3), i);
    }
    CGAL::spatial_sort(points.begin(), points.end(), NnSortTraits());

    std::unique_ptr<NaturalNeighbor2> nn(new NaturalNeighbor2());
    nn->sites_n = sites_n;
    NnDt2::Face_handle hint;
    for (const NnIPoint &ip : points) {
      const NnDt2::Vertex_handle vh = nn->dt.insert(ip.first, hint);
      if (vh->info().index < 0) {
        vh->info().index = ip.second;
      }
      hint = vh->face();
    }
    if (nn->dt.dimension() < 2) {
      error = "Natural Neighbor 2D: sites are not 2D (need a non-collinear XY set)";
      return nullptr;
    }
    return nn.release();
  }
  catch (const std::exception &e) {
    error = e.what();
    return nullptr;
  }
  catch (...) {
    error = "Natural Neighbor 2D: failed to build Delaunay";
    return nullptr;
  }
}

void natural_neighbor_2_free(NaturalNeighbor2 *nn)
{
  delete nn;
}

int natural_neighbor_2_site_count(const NaturalNeighbor2 *nn)
{
  return nn ? nn->sites_n : 0;
}

void natural_neighbor_2_query_many(const NaturalNeighbor2 *nn,
                                   const float *query_xyz,
                                   int query_n,
                                   std::vector<int> &r_offsets,
                                   std::vector<int> &r_site_index,
                                   std::vector<float> &r_weight,
                                   bool *r_valid)
{
  r_offsets.clear();
  r_site_index.clear();
  r_weight.clear();
  if (!nn || !query_xyz || query_n < 1 || nn->dt.dimension() < 2) {
    if (query_n > 0) {
      r_offsets.assign(size_t(query_n) + 1, 0);
      if (r_valid) {
        std::fill(r_valid, r_valid + query_n, false);
      }
    }
    return;
  }

  r_offsets.resize(size_t(query_n) + 1, 0);
  r_site_index.reserve(size_t(query_n) * 6);
  r_weight.reserve(size_t(query_n) * 6);

  using Vertex_handle = NnDt2::Vertex_handle;
  using Face_handle = NnDt2::Face_handle;
  using FT = Kernel::FT;

  std::vector<std::pair<Vertex_handle, FT>> coords;
  coords.reserve(16);
  Face_handle start;

  ensure_cgal_exceptions();
  for (int q = 0; q < query_n; q++) {
    coords.clear();
    bool ok = false;
    try {
      const auto nnc = CGAL::natural_neighbors_2(
          nn->dt, to_p2(query_xyz + q * 3), std::back_inserter(coords), start);
      if (nnc.third && nnc.second > 0 && !coords.empty()) {
        const double inv = 1.0 / CGAL::to_double(nnc.second);
        for (const auto &cw : coords) {
          const int site_i = cw.first->info().index;
          if (site_i < 0) {
            continue;
          }
          r_site_index.push_back(site_i);
          r_weight.push_back(float(CGAL::to_double(cw.second) * inv));
        }
        ok = r_site_index.size() > size_t(r_offsets[size_t(q)]);
        if (ok) {
          start = coords.front().first->face();
        }
      }
    }
    catch (...) {
      ok = false;
    }
    if (!ok) {
      r_site_index.resize(size_t(r_offsets[size_t(q)]));
      r_weight.resize(size_t(r_offsets[size_t(q)]));
    }
    r_offsets[size_t(q) + 1] = int(r_site_index.size());
    if (r_valid) {
      r_valid[q] = ok;
    }
  }
}

static bool gabriel_edge_empty(const Dt2 &dt, const Dt2::Edge &edge)
{
  const Dt2::Face_handle face = edge.first;
  const int opp = edge.second;
  const Point_2 p = face->vertex(Dt2::cw(opp))->point();
  const Point_2 q = face->vertex(Dt2::ccw(opp))->point();
  const Point_2 mid((p.x() + q.x()) / 2, (p.y() + q.y()) / 2);
  const double r2 = CGAL::to_double(CGAL::squared_distance(p, q)) * 0.25;
  auto inside = [&](const Dt2::Vertex_handle &vert) {
    if (dt.is_infinite(vert)) {
      return false;
    }
    return CGAL::to_double(CGAL::squared_distance(vert->point(), mid)) < r2 - 1e-18;
  };
  if (inside(face->vertex(opp))) {
    return false;
  }
  const Dt2::Face_handle neigh = face->neighbor(opp);
  if (!dt.is_infinite(neigh) && inside(neigh->vertex(neigh->index(face)))) {
    return false;
  }
  return true;
}

static double xyz_dist2(const float *positions, int u, int v)
{
  const double dx = double(positions[u * 3 + 0]) - double(positions[v * 3 + 0]);
  const double dy = double(positions[u * 3 + 1]) - double(positions[v * 3 + 1]);
  const double dz = double(positions[u * 3 + 2]) - double(positions[v * 3 + 2]);
  return dx * dx + dy * dy + dz * dz;
}

WireResult points_proximity_graph_2(const float *positions, int n, int mode)
{
  WireResult result;
  if (!positions || n < 2) {
    result.error = "Proximity Graph 2D needs at least 2 points";
    return result;
  }
  CgalThrowGuard guard;
  try {
    Dt2 dt;
    std::map<Dt2::Vertex_handle, int> vert_index;
    for (int i = 0; i < n; i++) {
      const Dt2::Vertex_handle site = dt.insert(to_p2(positions + i * 3));
      if (site != Dt2::Vertex_handle()) {
        vert_index[site] = i;
      }
    }

    struct EdgeCand {
      int a, b;
      double len2;
    };
    std::vector<EdgeCand> edges;
    for (auto eit = dt.finite_edges_begin(); eit != dt.finite_edges_end(); ++eit) {
      const Dt2::Face_handle f = eit->first;
      const int opp = eit->second;
      const Dt2::Vertex_handle va = f->vertex(Dt2::cw(opp));
      const Dt2::Vertex_handle vb = f->vertex(Dt2::ccw(opp));
      const auto ia = vert_index.find(va);
      const auto ib = vert_index.find(vb);
      if (ia == vert_index.end() || ib == vert_index.end()) {
        continue;
      }
      EdgeCand cand;
      cand.a = ia->second;
      cand.b = ib->second;
      cand.len2 = xyz_dist2(positions, cand.a, cand.b);
      if (mode == 0) {
        if (gabriel_edge_empty(dt, *eit)) {
          edges.push_back(cand);
        }
      }
      else if (mode == 1) {
        bool keep = true;
        for (int k = 0; k < n && keep; k++) {
          if (k == cand.a || k == cand.b) {
            continue;
          }
          const double d_ak = xyz_dist2(positions, cand.a, k);
          const double d_bk = xyz_dist2(positions, cand.b, k);
          if ((std::max)(d_ak, d_bk) < cand.len2 - 1e-18) {
            keep = false;
          }
        }
        if (keep) {
          edges.push_back(cand);
        }
      }
      else {
        edges.push_back(cand);
      }
    }

    if (mode == 2) {
      std::sort(edges.begin(), edges.end(), [](const EdgeCand &u, const EdgeCand &v) {
        return u.len2 < v.len2;
      });
      UnionFind uf(n);
      std::vector<EdgeCand> mst;
      mst.reserve(size_t((std::max)(0, n - 1)));
      for (const EdgeCand &e : edges) {
        if (uf.unite(e.a, e.b)) {
          mst.push_back(e);
          if (int(mst.size()) == n - 1) {
            break;
          }
        }
      }
      edges.swap(mst);
    }

    result.positions.assign(positions, positions + n * 3);
    result.edge_v0.reserve(edges.size());
    result.edge_v1.reserve(edges.size());
    for (const EdgeCand &e : edges) {
      result.edge_v0.push_back(e.a);
      result.edge_v1.push_back(e.b);
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Proximity Graph 2D produced no edges";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Proximity Graph 2D failed";
  }
  return result;
}

static std::pair<int, int> fill_edge_key(int a, int b)
{
  return (a < b) ? std::pair<int, int>(a, b) : std::pair<int, int>(b, a);
}

static bool fill_xyz_close(const float *a, const float *b)
{
  const double dx = double(a[0]) - double(b[0]);
  const double dy = double(a[1]) - double(b[1]);
  const double dz = double(a[2]) - double(b[2]);
  const double d2 = dx * dx + dy * dy + dz * dz;
  const double s2 = double(a[0]) * double(a[0]) + double(a[1]) * double(a[1]) +
                    double(a[2]) * double(a[2]) + double(b[0]) * double(b[0]) +
                    double(b[1]) * double(b[1]) + double(b[2]) * double(b[2]);
  return d2 <= 1e-12 * std::max(1.0, s2);
}

static bool fill_p3_close(const Point_3 &a, const Point_3 &b)
{
  const double dx = CGAL::to_double(a.x()) - CGAL::to_double(b.x());
  const double dy = CGAL::to_double(a.y()) - CGAL::to_double(b.y());
  const double dz = CGAL::to_double(a.z()) - CGAL::to_double(b.z());
  const double d2 = dx * dx + dy * dy + dz * dz;
  const double ax = CGAL::to_double(a.x());
  const double ay = CGAL::to_double(a.y());
  const double az = CGAL::to_double(a.z());
  const double bx = CGAL::to_double(b.x());
  const double by = CGAL::to_double(b.y());
  const double bz = CGAL::to_double(b.z());
  const double s2 = ax * ax + ay * ay + az * az + bx * bx + by * by + bz * bz;
  return d2 <= 1e-12 * std::max(1.0, s2);
}

struct FillLoop {
  std::vector<int> verts;
  int src_face = -1;
};

static void fill_append_loop(std::vector<FillLoop> &loops, std::vector<int> chain, int src_face)
{
  if (chain.size() >= 2 && chain.front() == chain.back()) {
    chain.pop_back();
  }
  std::vector<int> out;
  out.reserve(chain.size());
  for (int v : chain) {
    if (out.empty() || out.back() != v) {
      out.push_back(v);
    }
  }
  if (out.size() >= 2 && out.front() == out.back()) {
    out.pop_back();
  }
  if (out.size() >= 3) {
    loops.push_back(FillLoop{std::move(out), src_face});
  }
}

static void fill_maybe_closed_chain(const MeshIn &mesh,
                                   std::vector<FillLoop> &loops,
                                   std::vector<int> chain)
{
  if (chain.size() >= 2 && chain.front() == chain.back()) {
    fill_append_loop(loops, std::move(chain), -1);
    return;
  }
  if (chain.size() >= 4 && mesh.positions != nullptr) {
    const float *p0 = mesh.positions + chain.front() * 3;
    const float *p1 = mesh.positions + chain.back() * 3;
    if (fill_xyz_close(p0, p1)) {
      chain.pop_back();
      fill_append_loop(loops, std::move(chain), -1);
    }
  }
}

static bool fill_ear_clip(const std::vector<Point_3> &pts,
                          std::vector<CGAL::Triple<int, int, int>> &faces)
{
  const int n = int(pts.size());
  if (n < 3) {
    return false;
  }
  std::vector<double> px3;
  std::vector<double> py3;
  std::vector<double> pz3;
  px3.resize(size_t(n));
  py3.resize(size_t(n));
  pz3.resize(size_t(n));
  for (int i = 0; i < n; i++) {
    px3[size_t(i)] = CGAL::to_double(pts[size_t(i)].x());
    py3[size_t(i)] = CGAL::to_double(pts[size_t(i)].y());
    pz3[size_t(i)] = CGAL::to_double(pts[size_t(i)].z());
  }
  double nx = 0.0, ny = 0.0, nz = 0.0;
  for (int i = 0; i < n; i++) {
    const int j = (i + 1) % n;
    nx += (py3[size_t(i)] - py3[size_t(j)]) * (pz3[size_t(i)] + pz3[size_t(j)]);
    ny += (pz3[size_t(i)] - pz3[size_t(j)]) * (px3[size_t(i)] + px3[size_t(j)]);
    nz += (px3[size_t(i)] - px3[size_t(j)]) * (py3[size_t(i)] + py3[size_t(j)]);
  }
  const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
  if (nlen < 1e-20) {
    return false;
  }
  nx /= nlen;
  ny /= nlen;
  nz /= nlen;
  double ax = 0.0, ay = 1.0, az = 0.0;
  if (std::abs(ny) > 0.9) {
    ax = 1.0;
    ay = 0.0;
    az = 0.0;
  }
  double ux = ny * az - nz * ay;
  double uy = nz * ax - nx * az;
  double uz = nx * ay - ny * ax;
  const double ulen = std::sqrt(ux * ux + uy * uy + uz * uz);
  if (ulen < 1e-20) {
    return false;
  }
  ux /= ulen;
  uy /= ulen;
  uz /= ulen;
  const double vx = ny * uz - nz * uy;
  const double vy = nz * ux - nx * uz;
  const double vz = nx * uy - ny * ux;
  std::vector<double> pu;
  std::vector<double> pv;
  pu.resize(size_t(n));
  pv.resize(size_t(n));
  for (int i = 0; i < n; i++) {
    pu[size_t(i)] = px3[size_t(i)] * ux + py3[size_t(i)] * uy + pz3[size_t(i)] * uz;
    pv[size_t(i)] = px3[size_t(i)] * vx + py3[size_t(i)] * vy + pz3[size_t(i)] * vz;
  }
  auto area2 = [&](int ia, int ib, int ic) {
    return (pu[size_t(ib)] - pu[size_t(ia)]) * (pv[size_t(ic)] - pv[size_t(ia)]) -
           (pv[size_t(ib)] - pv[size_t(ia)]) * (pu[size_t(ic)] - pu[size_t(ia)]);
  };
  double poly_area = 0.0;
  for (int i = 0; i < n; i++) {
    const int j = (i + 1) % n;
    poly_area += pu[size_t(i)] * pv[size_t(j)] - pu[size_t(j)] * pv[size_t(i)];
  }
  const double orient = (poly_area >= 0.0) ? 1.0 : -1.0;
  auto in_tri = [&](int ia, int ib, int ic, int ip) {
    const double aa = orient * area2(ia, ib, ip);
    const double bb = orient * area2(ib, ic, ip);
    const double cc = orient * area2(ic, ia, ip);
    return aa >= -1e-18 && bb >= -1e-18 && cc >= -1e-18;
  };
  std::vector<int> idx;
  idx.resize(size_t(n));
  std::iota(idx.begin(), idx.end(), 0);
  int guard = 0;
  const int guard_max = n * n + 8;
  while (int(idx.size()) > 3 && guard++ < guard_max) {
    bool clipped = false;
    const int m = int(idx.size());
    for (int i = 0; i < m; i++) {
      const int ia = idx[size_t((i + m - 1) % m)];
      const int ib = idx[size_t(i)];
      const int ic = idx[size_t((i + 1) % m)];
      if (orient * area2(ia, ib, ic) <= 1e-18) {
        continue;
      }
      bool inside = false;
      for (int t : idx) {
        if (t == ia || t == ib || t == ic) {
          continue;
        }
        if (in_tri(ia, ib, ic, t)) {
          inside = true;
          break;
        }
      }
      if (inside) {
        continue;
      }
      faces.push_back(CGAL::Triple<int, int, int>(ia, ib, ic));
      idx.erase(idx.begin() + i);
      clipped = true;
      break;
    }
    if (!clipped) {
      break;
    }
  }
  if (idx.size() == 3 && orient * area2(idx[0], idx[1], idx[2]) > 1e-18) {
    faces.push_back(CGAL::Triple<int, int, int>(idx[0], idx[1], idx[2]));
  }
  return !faces.empty();
}

static void extract_closed_fill_loops(const MeshIn &mesh, std::vector<FillLoop> &loops)
{
  loops.clear();
  std::set<std::pair<int, int>> face_edges;
  /* Every n-gon face is its own closed ring. */
  if (mesh.faces_num > 0 && mesh.face_offsets && mesh.corner_verts) {
    for (int f = 0; f < mesh.faces_num; f++) {
      const int a = mesh.face_offsets[f];
      const int b = mesh.face_offsets[f + 1];
      std::vector<int> ring;
      ring.reserve(size_t(std::max(0, b - a)));
      for (int c = a; c < b; c++) {
        const int v = mesh.corner_verts[c];
        if (v < 0 || v >= mesh.verts_num) {
          ring.clear();
          break;
        }
        if (ring.empty() || ring.back() != v) {
          ring.push_back(v);
        }
      }
      if (ring.size() >= 2 && ring.front() == ring.back()) {
        ring.pop_back();
      }
      if (ring.size() < 3) {
        continue;
      }
      for (int i = 0; i < int(ring.size()); i++) {
        face_edges.insert(fill_edge_key(ring[size_t(i)], ring[size_t((i + 1) % int(ring.size()))]));
      }
      fill_append_loop(loops, std::move(ring), f);
    }
  }
  /* Loose wires that are not already a face boundary: every cycle, plus open
   * chains whose endpoints coincide (imported DXF/SVG rings, non-cyclic curves). */
  if (mesh.edges_num > 0 && mesh.edge_v0 && mesh.edge_v1 && mesh.verts_num > 0) {
    std::vector<std::vector<int>> adj;
    adj.resize(size_t(mesh.verts_num));
    for (int e = 0; e < mesh.edges_num; e++) {
      const int a = mesh.edge_v0[e];
      const int b = mesh.edge_v1[e];
      if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
        continue;
      }
      if (face_edges.find(fill_edge_key(a, b)) != face_edges.end()) {
        continue;
      }
      adj[size_t(a)].push_back(b);
      adj[size_t(b)].push_back(a);
    }
    for (std::vector<int> &nbs : adj) {
      std::sort(nbs.begin(), nbs.end());
      nbs.erase(std::unique(nbs.begin(), nbs.end()), nbs.end());
    }
    std::set<std::pair<int, int>> used_e;
    auto walk = [&](int start, int next) {
      std::vector<int> chain;
      chain.push_back(start);
      int prev = start;
      int cur = next;
      used_e.insert(fill_edge_key(prev, cur));
      const int max_steps = std::max(3, mesh.edges_num + 2);
      for (int step = 0; step < max_steps; step++) {
        chain.push_back(cur);
        if (cur == start && int(chain.size()) >= 4) {
          break;
        }
        int nxt = -1;
        if (cur >= 0 && cur < mesh.verts_num) {
          for (int nb : adj[size_t(cur)]) {
            if (nb == prev) {
              continue;
            }
            if (used_e.find(fill_edge_key(cur, nb)) != used_e.end()) {
              continue;
            }
            nxt = nb;
            break;
          }
        }
        if (nxt < 0) {
          break;
        }
        used_e.insert(fill_edge_key(cur, nxt));
        prev = cur;
        cur = nxt;
        if (cur == start) {
          chain.push_back(start);
          break;
        }
      }
      return chain;
    };
    for (int v = 0; v < mesh.verts_num; v++) {
      if (adj[size_t(v)].size() != 1) {
        continue;
      }
      const int nb = adj[size_t(v)][0];
      if (used_e.find(fill_edge_key(v, nb)) != used_e.end()) {
        continue;
      }
      fill_maybe_closed_chain(mesh, loops, walk(v, nb));
    }
    for (int e = 0; e < mesh.edges_num; e++) {
      const int a = mesh.edge_v0[e];
      const int b = mesh.edge_v1[e];
      if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num) {
        continue;
      }
      if (face_edges.find(fill_edge_key(a, b)) != face_edges.end()) {
        continue;
      }
      if (used_e.find(fill_edge_key(a, b)) != used_e.end()) {
        continue;
      }
      fill_maybe_closed_chain(mesh, loops, walk(a, b));
    }
  }
  /* Point sequence with no topology: one ring, in vertex order. */
  if (loops.empty() && mesh.verts_num >= 3 && mesh.edges_num <= 0 && mesh.faces_num <= 0) {
    std::vector<int> chain;
    chain.resize(size_t(mesh.verts_num));
    std::iota(chain.begin(), chain.end(), 0);
    fill_append_loop(loops, std::move(chain), -1);
  }
}

static bool fill_one_loop(const MeshIn &mesh, const FillLoop &loop, MeshResult &result)
{
  std::vector<Point_3> pts;
  std::vector<int> src_vi;
  pts.reserve(loop.verts.size());
  src_vi.reserve(loop.verts.size());
  for (int vi : loop.verts) {
    if (vi < 0 || vi >= mesh.verts_num) {
      return false;
    }
    /* Keep distinct vertex indices even if they sit on the same coordinate.
     * Only skip a repeated index, so point attributes stay 1:1 with the ring. */
    if (!src_vi.empty() && src_vi.back() == vi) {
      continue;
    }
    pts.push_back(to_p3(mesh.positions + vi * 3));
    src_vi.push_back(vi);
  }
  if (pts.size() >= 2 &&
      (src_vi.front() == src_vi.back() || fill_p3_close(pts.front(), pts.back())))
  {
    pts.pop_back();
    src_vi.pop_back();
  }
  if (pts.size() < 3) {
    return false;
  }
  std::vector<CGAL::Triple<int, int, int>> faces;
  PMP::triangulate_hole_polyline(pts, std::back_inserter(faces));
  if (faces.empty()) {
    fill_ear_clip(pts, faces);
  }
  if (faces.empty()) {
    const int nv = int(pts.size());
    for (int i = 1; i + 1 < nv; i++) {
      faces.push_back(CGAL::Triple<int, int, int>(0, i, i + 1));
    }
  }
  if (faces.empty()) {
    return false;
  }
  const int vbase = result.verts_num();
  const int nv = int(pts.size());
  result.positions.reserve(result.positions.size() + size_t(nv) * 3);
  result.vert_src0.reserve(result.vert_src0.size() + size_t(nv));
  result.vert_src1.reserve(result.vert_src1.size() + size_t(nv));
  result.vert_factor.reserve(result.vert_factor.size() + size_t(nv));
  for (int i = 0; i < nv; i++) {
    const Point_3 &p = pts[size_t(i)];
    result.positions.push_back(float(CGAL::to_double(p.x())));
    result.positions.push_back(float(CGAL::to_double(p.y())));
    result.positions.push_back(float(CGAL::to_double(p.z())));
    result.vert_src0.push_back(src_vi[size_t(i)]);
    result.vert_src1.push_back(-1);
    result.vert_factor.push_back(0.0f);
  }
  int added = 0;
  for (const CGAL::Triple<int, int, int> &t : faces) {
    if (t.first < 0 || t.second < 0 || t.third < 0 || t.first >= nv || t.second >= nv ||
        t.third >= nv || t.first == t.second || t.second == t.third || t.first == t.third)
    {
      continue;
    }
    result.corner_verts.push_back(vbase + t.first);
    result.corner_verts.push_back(vbase + t.second);
    result.corner_verts.push_back(vbase + t.third);
    result.face_src.push_back(loop.src_face);
    result.face_src_mesh.push_back(0);
    result.face_tag.push_back(loop.src_face);
    added++;
  }
  return added > 0;
}

MeshResult mesh_fill_polyline(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<FillLoop> loops;
    extract_closed_fill_loops(mesh, loops);
    if (loops.empty()) {
      result.error = "Fill Polyline needs at least one closed loop with ≥3 verts";
      return result;
    }
    int filled = 0;
    int failed = 0;
    for (int i = 0; i < int(loops.size()); i++) {
      if (fill_one_loop(mesh, loops[size_t(i)], result)) {
        filled++;
      }
      else {
        failed++;
      }
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Fill Polyline could not triangulate any loop";
    }
    else if (failed > 0) {
      result.alpha_used = double(failed);
    }
    (void)filled;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Fill Polyline failed";
  }
  return result;
}

bool points_regularize_planes(const float *positions,
                              const int *plane_id,
                              int n,
                              bool parallelism,
                              bool orthogonality,
                              bool coplanarity,
                              bool axis_symmetry,
                              double angle_deg,
                              double coplanar_tol,
                              std::vector<float> &out_xyz,
                              int &out_plane_count,
                              std::string &error)
{
  out_xyz.clear();
  out_plane_count = 0;
  if (!positions || !plane_id || n < 3) {
    error = "Regularize Planes needs ≥3 points with plane IDs";
    return false;
  }
  CgalThrowGuard guard;
  try {
    std::map<int, std::vector<int>> groups;
    for (int i = 0; i < n; i++) {
      if (plane_id[i] >= 0) {
        groups[plane_id[i]].push_back(i);
      }
    }
    if (groups.empty()) {
      error = "Regularize Planes: no point has a plane ID ≥ 0";
      return false;
    }

    std::vector<Plane_3> planes;
    std::vector<int> compact_id(size_t(n), -1);
    planes.reserve(groups.size());
    for (const auto &kv : groups) {
      if (kv.second.size() < 3) {
        continue;
      }
      std::vector<Point_3> pts;
      pts.reserve(kv.second.size());
      for (int i : kv.second) {
        pts.push_back(to_p3(positions + i * 3));
      }
      Plane_3 plane;
      Point_3 centroid;
      CGAL::linear_least_squares_fitting_3(
          pts.begin(), pts.end(), plane, centroid, CGAL::Dimension_tag<0>());
      (void)centroid;
      const int pid = int(planes.size());
      planes.push_back(plane);
      for (int i : kv.second) {
        compact_id[size_t(i)] = pid;
      }
    }
    if (planes.size() < 1) {
      error = "Regularize Planes: each used plane needs ≥3 points";
      return false;
    }
    out_plane_count = int(planes.size());

    std::vector<Point_3> pts;
    std::vector<int> idx;
    pts.reserve(size_t(n));
    idx.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      if (compact_id[size_t(i)] >= 0) {
        pts.push_back(to_p3(positions + i * 3));
        idx.push_back(compact_id[size_t(i)]);
      }
    }

    CGAL::Shape_regularization::Planes::regularize_planes(
        pts,
        CGAL::Identity_property_map<Point_3>(),
        planes,
        CGAL::Identity_property_map<Plane_3>(),
        CGAL::make_property_map(idx),
        Kernel(),
        parallelism,
        orthogonality,
        coplanarity,
        axis_symmetry,
        Kernel::FT(angle_deg),
        Kernel::FT(coplanar_tol));

    out_xyz.assign(positions, positions + n * 3);
    for (int i = 0; i < n; i++) {
      const int pid = compact_id[size_t(i)];
      if (pid < 0) {
        continue;
      }
      const Point_3 q = planes[size_t(pid)].projection(to_p3(positions + i * 3));
      out_xyz[size_t(i) * 3 + 0] = float(CGAL::to_double(q.x()));
      out_xyz[size_t(i) * 3 + 1] = float(CGAL::to_double(q.y()));
      out_xyz[size_t(i) * 3 + 2] = float(CGAL::to_double(q.z()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Regularize Planes failed";
    return false;
  }
}

}  // namespace blender::cgal_bridge
