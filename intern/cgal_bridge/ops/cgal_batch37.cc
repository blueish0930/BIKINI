/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 37 — new only (not previously deleted):
 *  - Radius Graph 3D
 *  - KNN Graph 3D
 *  - Natural Neighbor 3D
 *  - Sibson Gradient 2D
 *
 * Deleted this turn (legacy reserved): Weighted Skeleton 2D (2507),
 * Weighted Offset 2D (2511), Round Offset 2D (2515).
 * Not added: Periodic Regular 3D (weighted Delaunay), Optimize Mesh 2D (Lloyd).
 *
 * Skipped: P22 OpenGR, G10 Kinetic EPECK, Mesh_3 volume, PolyFit SCIP,
 * Ridges_3, Classification, OSQP Regularize Segments, Hyperbolic Delaunay,
 * previously banned measure/query/remesh/component nodes.
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_walk_along_line_point_location.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Fuzzy_sphere.h>
#include <CGAL/Interpolation_gradient_fitting_traits_2.h>
#include <CGAL/Kd_tree.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/Triangulation_data_structure_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/interpolation_functions.h>
#include <CGAL/natural_neighbor_coordinates_2.h>
#include <CGAL/natural_neighbor_coordinates_3.h>
#include <CGAL/property_map.h>
#include <CGAL/sibson_gradient_fitting.h>
#include <CGAL/spatial_sort.h>
#include <CGAL/Spatial_sort_traits_adapter_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Triangulation_data_structure_2.h>

#include <boost/property_map/property_map.hpp>

#include <CGAL/squared_distance_3.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <set>
#include <utility>
#include <variant>
#include <vector>

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Segment_2 = Kernel::Segment_2;
using FT = Kernel::FT;

/* Named temps 鈥?MSVC parses `Point_2 p(double(x), double(y))` as a function. */
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

}  // namespace

WireResult points_radius_graph_3(const float *positions, int n, double radius)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!positions || n < 2) {
    result.error = "Radius Graph 3D needs at least 2 points";
    return result;
  }
  if (!(radius > 0.0) || !std::isfinite(radius)) {
    result.error = "Radius must be > 0";
    return result;
  }
  try {
    using Pair = std::pair<Point_3, int>;
    using Traits_base = CGAL::Search_traits_3<Kernel>;
    using Traits = CGAL::Search_traits_adapter<Pair,
                                               CGAL::First_of_pair_property_map<Pair>,
                                               Traits_base>;
    using Tree = CGAL::Kd_tree<Traits>;
    using Fuzzy = CGAL::Fuzzy_sphere<Traits>;

    std::vector<Pair> items;
    items.reserve(size_t(n));
    result.positions.assign(positions, positions + n * 3);
    for (int i = 0; i < n; i++) {
      items.emplace_back(xyz(double(positions[i * 3 + 0]),
                             double(positions[i * 3 + 1]),
                             double(positions[i * 3 + 2])),
                         i);
    }
    Tree tree(items.begin(), items.end());
    std::set<std::pair<int, int>> seen;
    const FT rad(radius);
    for (int i = 0; i < n; i++) {
      std::vector<Pair> found;
      tree.search(std::back_inserter(found), Fuzzy(items[size_t(i)].first, rad));
      for (const Pair &nb : found) {
        const int j = nb.second;
        if (j <= i) {
          continue;
        }
        if (seen.insert({i, j}).second) {
          result.edge_v0.push_back(i);
          result.edge_v1.push_back(j);
        }
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Radius Graph 3D empty (increase Radius)";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Radius Graph 3D failed";
  }
  return result;
}


WireResult points_knn_graph_3(const float *positions, int n, int k)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!positions || n < 2) {
    result.error = "KNN Graph 3D needs at least 2 points";
    return result;
  }
  k = std::clamp(k, 1, n - 1);
  try {
    using Pair = std::pair<Point_3, int>;
    using Traits_base = CGAL::Search_traits_3<Kernel>;
    using Traits = CGAL::Search_traits_adapter<Pair,
                                               CGAL::First_of_pair_property_map<Pair>,
                                               Traits_base>;
    using KSearch = CGAL::Orthogonal_k_neighbor_search<Traits>;
    using Tree = typename KSearch::Tree;

    std::vector<Pair> items;
    items.reserve(size_t(n));
    result.positions.assign(positions, positions + n * 3);
    for (int i = 0; i < n; i++) {
      items.emplace_back(xyz(double(positions[i * 3 + 0]),
                             double(positions[i * 3 + 1]),
                             double(positions[i * 3 + 2])),
                         i);
    }
    Tree tree(items.begin(), items.end());
    std::set<std::pair<int, int>> seen;
    for (int i = 0; i < n; i++) {
      KSearch search(tree, items[size_t(i)].first, unsigned(k + 1));
      for (auto it = search.begin(); it != search.end(); ++it) {
        const int j = it->first.second;
        if (j == i) {
          continue;
        }
        const int a = std::min(i, j);
        const int b = std::max(i, j);
        if (seen.insert({a, b}).second) {
          result.edge_v0.push_back(a);
          result.edge_v1.push_back(b);
        }
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "KNN Graph 3D empty";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "KNN Graph 3D failed";
  }
  return result;
}

bool points_natural_neighbor_3(const float *sites,
                               const float *site_values,
                               int n_sites,
                               const float *queries,
                               int n_queries,
                               std::vector<float> &out_values,
                               std::string &error)
{
  CgalThrowGuard guard;
  out_values.clear();
  if (!sites || !site_values || n_sites < 4 || !queries || n_queries < 1) {
    error = "Natural Neighbor 3D needs 4+ sites and at least one query";
    return false;
  }
  try {
    using Vb = CGAL::Triangulation_vertex_base_with_info_3<int, Kernel>;
    using Cb = CGAL::Delaunay_triangulation_cell_base_3<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_3<Vb, Cb>;
    using Dt = CGAL::Delaunay_triangulation_3<Kernel, Tds>;

    Dt dt;
    for (int i = 0; i < n_sites; i++) {
      auto vh = dt.insert(xyz(double(sites[i * 3 + 0]),
                              double(sites[i * 3 + 1]),
                              double(sites[i * 3 + 2])));
      if (vh != Dt::Vertex_handle()) {
        vh->info() = i;
      }
    }
    if (dt.dimension() != 3) {
      error = "Natural Neighbor 3D degenerate (need non-coplanar sites)";
      return false;
    }
    out_values.assign(size_t(n_queries), 0.0f);
    for (int q = 0; q < n_queries; q++) {
      const Point_3 Q = xyz(double(queries[q * 3 + 0]),
                            double(queries[q * 3 + 1]),
                            double(queries[q * 3 + 2]));
      std::vector<std::pair<Dt::Vertex_handle, Dt::Geom_traits::FT>> coords;
      Dt::Geom_traits::FT norm(0);
      auto triple = CGAL::laplace_natural_neighbor_coordinates_3(
          dt, Q, std::back_inserter(coords), norm);
      if (!triple.third || !(norm > 0) || coords.empty()) {
        /* Outside the hull: nearest site. */
        int best = 0;
        double best_d = 1e300;
        for (int i = 0; i < n_sites; i++) {
          const double dx = double(sites[i * 3 + 0]) - double(queries[q * 3 + 0]);
          const double dy = double(sites[i * 3 + 1]) - double(queries[q * 3 + 1]);
          const double dz = double(sites[i * 3 + 2]) - double(queries[q * 3 + 2]);
          const double d2 = dx * dx + dy * dy + dz * dz;
          if (d2 < best_d) {
            best_d = d2;
            best = i;
          }
        }
        out_values[size_t(q)] = site_values[best];
        continue;
      }
      double acc = 0.0;
      const double nrm = CGAL::to_double(norm);
      for (const auto &c : coords) {
        const int idx = c.first->info();
        if (idx >= 0 && idx < n_sites) {
          acc += double(site_values[idx]) * CGAL::to_double(c.second);
        }
      }
      out_values[size_t(q)] = float(acc / nrm);
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
  }
  catch (...) {
    error = "Natural Neighbor 3D failed";
  }
  return false;
}

bool points_sibson_gradient_2(const float *positions,
                              const float *values,
                              int n,
                              std::vector<float> &out_grad_xyz,
                              std::string &error)
{
  CgalThrowGuard guard;
  out_grad_xyz.clear();
  if (!positions || !values || n < 3) {
    error = "Sibson Gradient 2D needs at least 3 points";
    return false;
  }
  try {
    using Vb = CGAL::Triangulation_vertex_base_with_info_2<int, Kernel>;
    using Fb = CGAL::Triangulation_face_base_2<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using Dt = CGAL::Delaunay_triangulation_2<Kernel, Tds>;
    using Traits = CGAL::Interpolation_gradient_fitting_traits_2<Kernel>;
    Dt dt;
    std::map<Point_2, double> function_values;
    for (int i = 0; i < n; i++) {
      const Point_2 p = xy(double(positions[i * 3 + 0]), double(positions[i * 3 + 1]));
      auto vh = dt.insert(p);
      if (vh != Dt::Vertex_handle()) {
        vh->info() = i;
      }
      function_values[p] = double(values[i]);
    }
    if (dt.dimension() != 2) {
      error = "Sibson Gradient 2D degenerate";
      return false;
    }
    auto value_function = CGAL::Data_access<std::map<Point_2, double>>(function_values);
    std::vector<std::pair<Point_2, Kernel::Vector_2>> grads;
    CGAL::sibson_gradient_fitting_nn_2(
        dt, std::back_inserter(grads), value_function, Traits());
    out_grad_xyz.assign(size_t(n) * 3, 0.0f);
    for (const auto &g : grads) {
      const auto vh = dt.nearest_vertex(g.first);
      if (vh == Dt::Vertex_handle()) {
        continue;
      }
      const int i = vh->info();
      if (i < 0 || i >= n) {
        continue;
      }
      out_grad_xyz[size_t(i) * 3 + 0] = float(CGAL::to_double(g.second.x()));
      out_grad_xyz[size_t(i) * 3 + 1] = float(CGAL::to_double(g.second.y()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
  }
  catch (...) {
    error = "Sibson Gradient 2D failed";
  }
  return false;
}

struct NaturalNeighbor3 {
  using Vb = CGAL::Triangulation_vertex_base_with_info_3<int, Kernel>;
  using Cb = CGAL::Delaunay_triangulation_cell_base_3<Kernel>;
  using Tds = CGAL::Triangulation_data_structure_3<Vb, Cb>;
  using Dt = CGAL::Delaunay_triangulation_3<Kernel, Tds>;
  using Vb2 = CGAL::Triangulation_vertex_base_with_info_2<int, Kernel>;
  using Fb2 = CGAL::Triangulation_face_base_2<Kernel>;
  using Tds2 = CGAL::Triangulation_data_structure_2<Vb2, Fb2>;
  using Dt2 = CGAL::Delaunay_triangulation_2<Kernel, Tds2>;
  Dt dt;
  Dt2 dt2;
  int sites_n = 0;
  int drop_axis = 2;
  bool planar = false;
  double snap2 = 1e-24;
  mutable std::mutex mutex;
};

NaturalNeighbor3 *natural_neighbor_3_new(const float *sites_xyz, int sites_n, std::string &error)
{
  if (!sites_xyz || sites_n < 4) {
    error = "Natural Neighbor 3D needs at least 4 sites";
    return nullptr;
  }
  CgalThrowGuard guard;
  try {
    using IPoint = std::pair<Point_3, int>;
    using SortTraits = CGAL::Spatial_sort_traits_adapter_3<Kernel,
                                                           CGAL::First_of_pair_property_map<IPoint>>;
    std::vector<IPoint> points;
    points.reserve(size_t(sites_n));
    double mn[3] = {1e300, 1e300, 1e300};
    double mx[3] = {-1e300, -1e300, -1e300};
    for (int i = 0; i < sites_n; i++) {
      const double x = double(sites_xyz[i * 3 + 0]);
      const double y = double(sites_xyz[i * 3 + 1]);
      const double z = double(sites_xyz[i * 3 + 2]);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      points.emplace_back(xyz(x, y, z), i);
      mn[0] = std::min(mn[0], x);
      mn[1] = std::min(mn[1], y);
      mn[2] = std::min(mn[2], z);
      mx[0] = std::max(mx[0], x);
      mx[1] = std::max(mx[1], y);
      mx[2] = std::max(mx[2], z);
    }
    if (int(points.size()) < 4) {
      error = "Natural Neighbor 3D needs at least 4 finite sites";
      return nullptr;
    }
    CGAL::spatial_sort(points.begin(), points.end(), SortTraits());

    std::unique_ptr<NaturalNeighbor3> nn(new NaturalNeighbor3());
    nn->sites_n = sites_n;
    const double diag = std::hypot(mx[0] - mn[0], std::hypot(mx[1] - mn[1], mx[2] - mn[2]));
    const double snap = std::max(1e-12, diag * 1e-9);
    nn->snap2 = snap * snap;
    NaturalNeighbor3::Dt::Cell_handle hint;
    bool have_hint = false;
    for (const IPoint &ip : points) {
      const int prev = int(nn->dt.number_of_vertices());
      const auto vh = have_hint ? nn->dt.insert(ip.first, hint) : nn->dt.insert(ip.first);
      if (vh == NaturalNeighbor3::Dt::Vertex_handle()) {
        continue;
      }
      hint = vh->cell();
      have_hint = true;
      if (int(nn->dt.number_of_vertices()) > prev) {
        vh->info() = ip.second;
      }
    }
    if (nn->dt.dimension() == 3) {
      return nn.release();
    }
    /* Nearly / fully coplanar sites: 3D Laplace only succeeds inside a thin
     * volume, so almost every sample is "outside". Fall back to 2D NN in the
     * two axes with the largest span. */
    if (nn->dt.number_of_vertices() < 3) {
      error = "Natural Neighbor 3D degenerate";
      return nullptr;
    }
    const double sx = mx[0] - mn[0];
    const double sy = mx[1] - mn[1];
    const double sz = mx[2] - mn[2];
    nn->drop_axis = 2;
    if (sx <= sy && sx <= sz) {
      nn->drop_axis = 0;
    }
    else if (sy <= sz) {
      nn->drop_axis = 1;
    }
    nn->planar = true;
    for (const IPoint &ip : points) {
      const double x = CGAL::to_double(ip.first.x());
      const double y = CGAL::to_double(ip.first.y());
      const double z = CGAL::to_double(ip.first.z());
      const Point_2 p2 = (nn->drop_axis == 0) ? xy(y, z) :
                         (nn->drop_axis == 1) ? xy(x, z) : xy(x, y);
      const int prev = int(nn->dt2.number_of_vertices());
      auto vh = nn->dt2.insert(p2);
      if (vh != NaturalNeighbor3::Dt2::Vertex_handle() &&
          int(nn->dt2.number_of_vertices()) > prev)
      {
        vh->info() = ip.second;
      }
    }
    if (nn->dt2.dimension() < 2) {
      error = "Natural Neighbor 3D degenerate (need a 2D or 3D site set)";
      return nullptr;
    }
    return nn.release();
  }
  catch (const std::exception &e) {
    error = e.what();
    return nullptr;
  }
  catch (...) {
    error = "Natural Neighbor 3D: failed to build Delaunay";
    return nullptr;
  }
}

void natural_neighbor_3_free(NaturalNeighbor3 *nn)
{
  delete nn;
}

int natural_neighbor_3_site_count(const NaturalNeighbor3 *nn)
{
  return nn ? nn->sites_n : 0;
}

void natural_neighbor_3_query_many(const NaturalNeighbor3 *nn,
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
  if (!nn || !query_xyz || query_n < 1 ||
      (nn->dt.dimension() < 2 && !nn->planar))
  {
    if (query_n > 0) {
      r_offsets.assign(size_t(query_n) + 1, 0);
      if (r_valid) {
        std::fill(r_valid, r_valid + query_n, false);
      }
    }
    return;
  }

  r_offsets.resize(size_t(query_n) + 1, 0);
  r_site_index.reserve(size_t(query_n) * 8);
  r_weight.reserve(size_t(query_n) * 8);

  using Vertex_handle = NaturalNeighbor3::Dt::Vertex_handle;
  using Cell_handle = NaturalNeighbor3::Dt::Cell_handle;
  using FT = Kernel::FT;
  using Vector_3 = Kernel::Vector_3;
  std::vector<std::pair<Vertex_handle, FT>> coords;
  coords.reserve(16);
  CgalThrowGuard guard;
  std::lock_guard<std::mutex> lock(nn->mutex);

  auto push_site = [&](int site_i, float w) {
    if (site_i >= 0 && site_i < nn->sites_n && w > 0.0f && std::isfinite(w)) {
      r_site_index.push_back(site_i);
      r_weight.push_back(w);
    }
  };
  auto nearest = [&](const Point_3 &Q) -> bool {
    const auto nv = nn->dt.nearest_vertex(Q);
    if (nv == Vertex_handle() || nn->dt.is_infinite(nv)) {
      return false;
    }
    const int site_i = nv->info();
    if (site_i < 0 || site_i >= nn->sites_n) {
      return false;
    }
    push_site(site_i, 1.0f);
    return true;
  };
  auto facet_bary = [&](Cell_handle c, int fi, const Point_3 &Q) -> bool {
    if (c == Cell_handle()) {
      return false;
    }
    Cell_handle fc = c;
    int fl = fi;
    if (nn->dt.is_infinite(fc)) {
      fc = c->neighbor(fi);
      if (fc == Cell_handle() || nn->dt.is_infinite(fc)) {
        return false;
      }
      fl = fc->index(c);
    }
    Vertex_handle v[3];
    int k = 0;
    for (int i = 0; i < 4; i++) {
      if (i == fl) {
        continue;
      }
      v[k++] = fc->vertex(i);
    }
    if (k != 3) {
      return false;
    }
    for (int i = 0; i < 3; i++) {
      if (v[i] == Vertex_handle() || nn->dt.is_infinite(v[i])) {
        return false;
      }
    }
    const Point_3 A = v[0]->point();
    const Point_3 B = v[1]->point();
    const Point_3 C = v[2]->point();
    const Vector_3 u = B - A;
    const Vector_3 w = C - A;
    const Vector_3 n = CGAL::cross_product(u, w);
    const double nn2 = CGAL::to_double(n.squared_length());
    if (!(nn2 > 1e-30)) {
      return false;
    }
    const Vector_3 r = Q - A;
    const double beta = CGAL::to_double(CGAL::cross_product(r, w) * n) / nn2;
    const double gamma = CGAL::to_double(CGAL::cross_product(u, r) * n) / nn2;
    double alpha = 1.0 - beta - gamma;
    double bw[3] = {alpha, beta, gamma};
    double sum = 0.0;
    for (int i = 0; i < 3; i++) {
      if (bw[i] < 0.0) {
        bw[i] = 0.0;
      }
      sum += bw[i];
    }
    if (!(sum > 1e-18)) {
      return false;
    }
    for (int i = 0; i < 3; i++) {
      push_site(v[i]->info(), float(bw[i] / sum));
    }
    return true;
  };

  for (int q = 0; q < query_n; q++) {
    coords.clear();
    bool ok = false;
    const int start = int(r_site_index.size());
    try {
      const double qx = double(query_xyz[q * 3 + 0]);
      const double qy = double(query_xyz[q * 3 + 1]);
      const double qz = double(query_xyz[q * 3 + 2]);
      if (!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz)) {
        throw std::runtime_error("non-finite query");
      }
      const Point_3 Q = xyz(qx, qy, qz);

      if (nn->planar && nn->dt2.dimension() == 2) {
        const Point_2 Q2 = (nn->drop_axis == 0) ? xy(qy, qz) :
                           (nn->drop_axis == 1) ? xy(qx, qz) : xy(qx, qy);
        std::vector<std::pair<NaturalNeighbor3::Dt2::Vertex_handle, FT>> c2;
        const auto nnc = CGAL::natural_neighbors_2(
            nn->dt2, Q2, std::back_inserter(c2));
        if (nnc.third && nnc.second > 0 && !c2.empty()) {
          const double inv = 1.0 / CGAL::to_double(nnc.second);
          for (const auto &cw : c2) {
            push_site(cw.first->info(), float(CGAL::to_double(cw.second) * inv));
          }
          ok = int(r_site_index.size()) > start;
        }
      }
      else if (nn->dt.dimension() == 3) {
        NaturalNeighbor3::Dt::Locate_type lt;
        int li = 0, lj = 0;
        Cell_handle c = nn->dt.locate(Q, lt, li, lj);
        if (lt == NaturalNeighbor3::Dt::VERTEX) {
          push_site(c->vertex(li)->info(), 1.0f);
          ok = int(r_site_index.size()) > start;
        }
        else if (lt == NaturalNeighbor3::Dt::EDGE) {
          const Vertex_handle va = c->vertex(li);
          const Vertex_handle vb = c->vertex(lj);
          const double da = CGAL::to_double(CGAL::squared_distance(va->point(), Q));
          const double db = CGAL::to_double(CGAL::squared_distance(vb->point(), Q));
          const double sa = 1.0 / std::max(da, 1e-30);
          const double sb = 1.0 / std::max(db, 1e-30);
          const double inv = 1.0 / (sa + sb);
          push_site(va->info(), float(sa * inv));
          push_site(vb->info(), float(sb * inv));
          ok = int(r_site_index.size()) > start;
        }
        else if (lt == NaturalNeighbor3::Dt::FACET) {
          ok = facet_bary(c, li, Q);
          if (ok) {
            ok = int(r_site_index.size()) > start;
          }
        }
        else if (lt == NaturalNeighbor3::Dt::CELL && !nn->dt.is_infinite(c)) {
          FT norm(0);
          auto triple = CGAL::laplace_natural_neighbor_coordinates_3(
              nn->dt, Q, std::back_inserter(coords), norm);
          if (triple.third && norm > 0 && !coords.empty()) {
            const double inv = 1.0 / CGAL::to_double(norm);
            for (const auto &cw : coords) {
              if (cw.first == Vertex_handle() || nn->dt.is_infinite(cw.first)) {
                continue;
              }
              push_site(cw.first->info(), float(CGAL::to_double(cw.second) * inv));
            }
            ok = int(r_site_index.size()) > start;
          }
        }
      }
      if (!ok) {
        r_site_index.resize(size_t(start));
        r_weight.resize(size_t(start));
        ok = nearest(Q);
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

}  // namespace blender::cgal_bridge
