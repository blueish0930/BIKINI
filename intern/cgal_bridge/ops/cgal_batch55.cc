/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 55: remaining CGAL algorithms that fit this DLL.
 * Heavy Mesh_3 / Kinetic / Nef / CDT3 / Surface_mesher live in cgal_volume_bridge.
 * Compiled against vendored CGAL 6.0.1 (no 6.1/6.2-only packages).
 */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Classification/ETHZ/Random_forest_classifier.h>
#include <CGAL/Classification/Feature_set.h>
#include <CGAL/Classification/Label_set.h>
#include <CGAL/Classification/classify.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>

namespace PMP = CGAL::Polygon_mesh_processing;
#include <CGAL/Bbox_3.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Triangulation_data_structure_2.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/squared_distance_3.h>
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Point_2 = Kernel::Point_2;
using VI = Surface_mesh::Vertex_index;
using FI = Surface_mesh::Face_index;
using EI = Surface_mesh::Edge_index;
using FT = Kernel::FT;
using TreeTraits = CGAL::Search_traits_3<Kernel>;
using KNN = CGAL::Orthogonal_k_neighbor_search<TreeTraits>;
using KNNTree = KNN::Tree;

static Point_3 mk_p3(double x, double y, double z)
{
  return Point_3(FT(x), FT(y), FT(z));
}

static void push_p3(std::vector<float> &pos, const Point_3 &p)
{
  pos.push_back(float(CGAL::to_double(p.x())));
  pos.push_back(float(CGAL::to_double(p.y())));
  pos.push_back(float(CGAL::to_double(p.z())));
}

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  sm.collect_garbage();
  return sm.number_of_faces() > 0 && sm.number_of_vertices() > 0;
}

static std::vector<Point_3> pts_from_xyz(const float *xyz, int n)
{
  std::vector<Point_3> pts;
  pts.reserve(size_t(n));
  for (int i = 0; i < n; i++) {
    pts.push_back(mk_p3(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
  }
  return pts;
}

static Point_3 knn_point(const typename KNN::iterator &it)
{
  return it->first;
}

static void pca_features(const std::vector<Point_3> &pts,
                         int knn,
                         std::vector<float> &linearity,
                         std::vector<float> &planarity,
                         std::vector<float> &sphericity,
                         std::vector<float> &omnivariance,
                         std::vector<float> &anisotropy,
                         std::vector<float> &verticality)
{
  const int n = int(pts.size());
  const int k = std::max(4, std::min(knn, std::max(1, n - 1)));
  KNNTree tree(pts.begin(), pts.end());
  linearity.assign(size_t(n), 0);
  planarity.assign(size_t(n), 0);
  sphericity.assign(size_t(n), 0);
  omnivariance.assign(size_t(n), 0);
  anisotropy.assign(size_t(n), 0);
  verticality.assign(size_t(n), 0);
  for (int i = 0; i < n; i++) {
    KNN search(tree, pts[size_t(i)], k + 1);
    struct Hit {
      Eigen::Vector3d p;
      double dist;
    };
    std::vector<Hit> hits;
    const Eigen::Vector3d pi(CGAL::to_double(pts[size_t(i)].x()),
                             CGAL::to_double(pts[size_t(i)].y()),
                             CGAL::to_double(pts[size_t(i)].z()));
    for (auto it = search.begin(); it != search.end(); ++it) {
      const Point_3 q = knn_point(it);
      Eigen::Vector3d v(CGAL::to_double(q.x()), CGAL::to_double(q.y()), CGAL::to_double(q.z()));
      hits.push_back({v, (v - pi).norm()});
    }
    if (hits.size() < 3) {
      continue;
    }
    std::vector<double> dists;
    for (const Hit &h : hits) {
      if (h.dist > 1e-12) {
        dists.push_back(h.dist);
      }
    }
    double cutoff = 1e300;
    if (!dists.empty()) {
      std::nth_element(dists.begin(), dists.begin() + int(dists.size() / 2), dists.end());
      cutoff = 2.5 * std::max(dists[dists.size() / 2], 1e-9);
    }
    Eigen::Vector3d c(0, 0, 0);
    double wsum = 0;
    std::vector<std::pair<Eigen::Vector3d, double>> nb;
    for (const Hit &h : hits) {
      if (h.dist > cutoff) {
        continue;
      }
      const double w = 1.0 / (1.0 + h.dist * h.dist / (cutoff * cutoff + 1e-18));
      nb.push_back({h.p, w});
      c += w * h.p;
      wsum += w;
    }
    if (nb.size() < 3 || wsum < 1e-18) {
      continue;
    }
    c /= wsum;
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
    for (const auto &vw : nb) {
      const Eigen::Vector3d d = vw.first - c;
      C += vw.second * (d * d.transpose());
    }
    C += Eigen::Matrix3d::Identity() * 1e-12;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
    /* Descending eigenvalues σ1 ≥ σ2 ≥ σ3, then normalize by trace so features
     * are scale-free. anisotropy = 1 - sphericity by definition. */
    Eigen::Vector3d ev = es.eigenvalues().cwiseMax(0.0);
    double s1 = ev[2], s2 = ev[1], s3 = ev[0];
    const double trace = std::max(s1 + s2 + s3, 1e-18);
    s1 /= trace;
    s2 /= trace;
    s3 /= trace;
    const double den = std::max(s1, 1e-18);
    linearity[size_t(i)] = float(std::min(1.0, std::max(0.0, (s1 - s2) / den)));
    planarity[size_t(i)] = float(std::min(1.0, std::max(0.0, (s2 - s3) / den)));
    sphericity[size_t(i)] = float(std::min(1.0, std::max(0.0, s3 / den)));
    omnivariance[size_t(i)] = float(
        std::min(1.0, std::max(0.0, 3.0 * std::cbrt(std::max(0.0, s1 * s2 * s3)))));
    anisotropy[size_t(i)] = float(std::min(1.0, std::max(0.0, (s1 - s3) / den)));
    Eigen::Vector3d nrm = es.eigenvectors().col(0);
    verticality[size_t(i)] = float(std::min(1.0, std::max(0.0, 1.0 - std::abs(nrm.z()))));
  }
}

/** Surface Nets of an SDF on a Cartesian grid. */
static MeshResult sdf_surface_nets(const std::function<double(const Point_3 &)> &sdf,
                                   const Point_3 &pmin,
                                   const Point_3 &pmax,
                                   int resolution,
                                   double isovalue)
{
  MeshResult result;
  const int res = std::max(6, std::min(72, resolution));
  const double x0 = CGAL::to_double(pmin.x());
  const double y0 = CGAL::to_double(pmin.y());
  const double z0 = CGAL::to_double(pmin.z());
  const double dx = (CGAL::to_double(pmax.x()) - x0) / double(res);
  const double dy = (CGAL::to_double(pmax.y()) - y0) / double(res);
  const double dz = (CGAL::to_double(pmax.z()) - z0) / double(res);
  const int n = res + 1;
  std::vector<float> grid(size_t(n) * size_t(n) * size_t(n));
  auto gid = [&](int i, int j, int k) {
    return (size_t(k) * size_t(n) + size_t(j)) * size_t(n) + size_t(i);
  };
  auto sample = [&](int i, int j, int k) {
    return sdf(mk_p3(x0 + dx * i, y0 + dy * j, z0 + dz * k));
  };
  for (int k = 0; k < n; k++) {
    for (int j = 0; j < n; j++) {
      for (int i = 0; i < n; i++) {
        grid[gid(i, j, k)] = float(sample(i, j, k) - isovalue);
      }
    }
  }
  std::vector<int> vox(size_t(res) * size_t(res) * size_t(res), -1);
  auto vid = [&](int i, int j, int k) {
    return (size_t(k) * size_t(res) + size_t(j)) * size_t(res) + size_t(i);
  };
  const int eoff[12][2][3] = {
      {{0, 0, 0}, {1, 0, 0}},
      {{1, 0, 0}, {1, 1, 0}},
      {{1, 1, 0}, {0, 1, 0}},
      {{0, 1, 0}, {0, 0, 0}},
      {{0, 0, 1}, {1, 0, 1}},
      {{1, 0, 1}, {1, 1, 1}},
      {{1, 1, 1}, {0, 1, 1}},
      {{0, 1, 1}, {0, 0, 1}},
      {{0, 0, 0}, {0, 0, 1}},
      {{1, 0, 0}, {1, 0, 1}},
      {{1, 1, 0}, {1, 1, 1}},
      {{0, 1, 0}, {0, 1, 1}},
  };
  result.face_offsets = {0};
  for (int k = 0; k < res; k++) {
    for (int j = 0; j < res; j++) {
      for (int i = 0; i < res; i++) {
        double sx = 0, sy = 0, sz = 0;
        int hits = 0;
        for (int e = 0; e < 12; e++) {
          const int i0 = i + eoff[e][0][0], j0 = j + eoff[e][0][1], k0 = k + eoff[e][0][2];
          const int i1 = i + eoff[e][1][0], j1 = j + eoff[e][1][1], k1 = k + eoff[e][1][2];
          const float a = grid[gid(i0, j0, k0)];
          const float b = grid[gid(i1, j1, k1)];
          if (a * b > 0) {
            continue;
          }
          const double t = (std::abs(b - a) < 1e-12) ? 0.5 : double(-a / (b - a));
          sx += (i0 + t * (i1 - i0));
          sy += (j0 + t * (j1 - j0));
          sz += (k0 + t * (k1 - k0));
          hits++;
        }
        if (hits == 0) {
          continue;
        }
        vox[vid(i, j, k)] = result.verts_num();
        result.positions.push_back(float(x0 + dx * sx / hits));
        result.positions.push_back(float(y0 + dy * sy / hits));
        result.positions.push_back(float(z0 + dz * sz / hits));
      }
    }
  }
  auto emit_quad = [&](int a, int b, int c, int d) {
    if (a < 0 || b < 0 || c < 0 || d < 0) {
      return;
    }
    result.corner_verts.push_back(a);
    result.corner_verts.push_back(b);
    result.corner_verts.push_back(c);
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.corner_verts.push_back(a);
    result.corner_verts.push_back(c);
    result.corner_verts.push_back(d);
    result.face_offsets.push_back(int(result.corner_verts.size()));
  };
  auto sign_at = [&](int i, int j, int k) {
    return grid[gid(i, j, k)] >= 0.0f;
  };
  for (int k = 0; k < res; k++) {
    for (int j = 0; j < res; j++) {
      for (int i = 0; i < res; i++) {
        if (i + 1 < res && sign_at(i + 1, j, k) != sign_at(i + 1, j + 1, k) && j + 1 < res) {
          /* skip */
        }
        /* X-aligned primal edges at (i+1,j,k) -> (i+1,j,k) wait: grid edges. */
        if (i < res && j < res && k < res) {
          if (sign_at(i, j, k) != sign_at(i + 1, j, k) && j > 0 && k > 0) {
            emit_quad(vox[vid(i, j - 1, k - 1)],
                      vox[vid(i, j, k - 1)],
                      vox[vid(i, j, k)],
                      vox[vid(i, j - 1, k)]);
          }
          if (sign_at(i, j, k) != sign_at(i, j + 1, k) && i > 0 && k > 0) {
            emit_quad(vox[vid(i - 1, j, k - 1)],
                      vox[vid(i, j, k - 1)],
                      vox[vid(i, j, k)],
                      vox[vid(i - 1, j, k)]);
          }
          if (sign_at(i, j, k) != sign_at(i, j, k + 1) && i > 0 && j > 0) {
            emit_quad(vox[vid(i - 1, j - 1, k)],
                      vox[vid(i, j - 1, k)],
                      vox[vid(i, j, k)],
                      vox[vid(i - 1, j, k)]);
          }
        }
      }
    }
  }
  result.ok = result.faces_num() > 0;
  if (!result.ok) {
    result.error = "Isosurface is empty (try isovalue 0 on a closed mesh)";
  }
  return result;
}

}  // namespace

bool points_sphere_intersect(const float *positions,
                             int n,
                             const float *radii,
                             float uniform_radius,
                             int segments,
                             std::vector<std::vector<float>> &out_polylines,
                             std::string &error)
{
  out_polylines.clear();
  CgalThrowGuard guard;
  if (!positions || n < 2) {
    error = "Sphere Intersect needs at least 2 points";
    return false;
  }
  segments = std::max(8, std::min(256, segments));
  try {
    for (int i = 0; i < n; i++) {
      const Point_3 ci = mk_p3(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);
      const double ri = radii ? double(radii[i]) : double(uniform_radius);
      if (ri <= 1e-12) {
        continue;
      }
      for (int j = i + 1; j < n; j++) {
        const Point_3 cj = mk_p3(positions[j * 3], positions[j * 3 + 1], positions[j * 3 + 2]);
        const double rj = radii ? double(radii[j]) : double(uniform_radius);
        if (rj <= 1e-12) {
          continue;
        }
        const Vector_3 d = cj - ci;
        const double dist = std::sqrt(CGAL::to_double(d.squared_length()));
        if (dist < 1e-12 || dist > ri + rj || dist < std::abs(ri - rj)) {
          continue;
        }
        const double a = (ri * ri - rj * rj + dist * dist) / (2.0 * dist);
        const double h2 = ri * ri - a * a;
        if (h2 <= 1e-16) {
          continue;
        }
        const double h = std::sqrt(h2);
        const Vector_3 nrm = d / FT(dist);
        Vector_3 u = CGAL::cross_product(nrm, Vector_3(0, 0, 1));
        if (CGAL::to_double(u.squared_length()) < 1e-12) {
          u = CGAL::cross_product(nrm, Vector_3(0, 1, 0));
        }
        const double ul = std::sqrt(CGAL::to_double(u.squared_length()));
        u = u / FT(ul);
        const Vector_3 v = CGAL::cross_product(nrm, u);
        const Point_3 center = ci + nrm * FT(a);
        std::vector<float> pl;
        pl.reserve(size_t(segments + 1) * 3);
        for (int s = 0; s <= segments; s++) {
          const double ang = 2.0 * 3.14159265358979323846 * double(s) / double(segments);
          const Point_3 p = center + u * FT(h * std::cos(ang)) + v * FT(h * std::sin(ang));
          push_p3(pl, p);
        }
        out_polylines.push_back(std::move(pl));
      }
    }
    if (out_polylines.empty()) {
      error = "No intersecting sphere pairs";
      return false;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
}

MeshResult mesh_sphere_arrangement(const MeshIn &mesh, float ox, float oy, float oz)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!load_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Sphere Arrangement needs a mesh" : error;
    return result;
  }
  try {
    const Point_3 o = mk_p3(ox, oy, oz);
    std::vector<Point_3> pts;
    const int seg = 32;
    for (const FI f : sm.faces()) {
      auto h = sm.halfedge(f);
      const Point_3 a = sm.point(sm.source(h));
      const Point_3 b = sm.point(sm.target(h));
      const Point_3 c = sm.point(sm.target(sm.next(h)));
      Kernel::Plane_3 pl(a, b, c);
      Vector_3 n = pl.orthogonal_vector();
      const double nl = std::sqrt(CGAL::to_double(n.squared_length()));
      if (nl < 1e-12) {
        continue;
      }
      n = n / FT(nl);
      const double d = CGAL::to_double((o - pl.point()) * n);
      if (std::abs(d) >= 1.0 - 1e-6) {
        continue;
      }
      const double rad = std::sqrt(std::max(0.0, 1.0 - d * d));
      const Point_3 center = o + n * FT(-d);
      Vector_3 u = CGAL::cross_product(n, Vector_3(0, 0, 1));
      if (CGAL::to_double(u.squared_length()) < 1e-12) {
        u = CGAL::cross_product(n, Vector_3(0, 1, 0));
      }
      u = u / FT(std::sqrt(CGAL::to_double(u.squared_length())));
      const Vector_3 v = CGAL::cross_product(n, u);
      for (int s = 0; s < seg; s++) {
        const double ang = 2.0 * 3.14159265358979323846 * double(s) / double(seg);
        pts.push_back(center + u * FT(rad * std::cos(ang)) + v * FT(rad * std::sin(ang)));
      }
    }
    if (pts.size() < 3) {
      result.error = "Sphere Arrangement: no plane cuts the unit sphere";
      return result;
    }
    result.face_offsets = {0};
    for (size_t i = 0; i < pts.size(); i++) {
      push_p3(result.positions, pts[i]);
    }
    const int ncirc = int(pts.size()) / seg;
    for (int ci = 0; ci < ncirc; ci++) {
      const int base = ci * seg;
      for (int s = 0; s < seg; s++) {
        result.corner_verts.push_back(base + s);
      }
      result.face_offsets.push_back(int(result.corner_verts.size()));
      result.face_tag.push_back(ci);
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Sphere Arrangement produced no circles";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Sphere Arrangement failed";
  }
  return result;
}
MeshResult mesh_graphcut_segment(const MeshIn &mesh, double angle_deg, int min_size)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!mesh.positions || mesh.verts_num < 3 || mesh.faces_num < 1 || !mesh.corner_verts) {
    result.error = "Graphcut Segment needs a mesh with faces";
    return result;
  }
  try {
    const int nf = mesh.faces_num;
    auto face_range = [&](int f, int &begin, int &end) {
      if (mesh.face_offsets) {
        begin = mesh.face_offsets[f];
        end = mesh.face_offsets[f + 1];
      }
      else {
        begin = f * 3;
        end = f * 3 + 3;
      }
    };
    std::vector<Vector_3> nrm(size_t(nf), Vector_3(0, 0, 1));
    for (int f = 0; f < nf; f++) {
      int b = 0, e = 0;
      face_range(f, b, e);
      const int ncorn = e - b;
      if (ncorn < 3) {
        continue;
      }
      double nx = 0, ny = 0, nz = 0;
      for (int i = 0; i < ncorn; i++) {
        const int ia = mesh.corner_verts[b + i];
        const int ib = mesh.corner_verts[b + (i + 1) % ncorn];
        const double ax = mesh.positions[ia * 3], ay = mesh.positions[ia * 3 + 1],
                     az = mesh.positions[ia * 3 + 2];
        const double bx = mesh.positions[ib * 3], by = mesh.positions[ib * 3 + 1],
                     bz = mesh.positions[ib * 3 + 2];
        nx += (ay - by) * (az + bz);
        ny += (az - bz) * (ax + bx);
        nz += (ax - bx) * (ay + by);
      }
      const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
      if (len > 1e-18) {
        nrm[size_t(f)] = Vector_3(nx / len, ny / len, nz / len);
      }
    }
    std::map<std::pair<int, int>, std::vector<int>> edge_faces;
    for (int f = 0; f < nf; f++) {
      int b = 0, e = 0;
      face_range(f, b, e);
      const int ncorn = e - b;
      for (int i = 0; i < ncorn; i++) {
        int a = mesh.corner_verts[b + i];
        int c = mesh.corner_verts[b + (i + 1) % ncorn];
        if (a > c) {
          std::swap(a, c);
        }
        edge_faces[{a, c}].push_back(f);
      }
    }
    std::vector<std::vector<int>> adj;
    adj.resize(size_t(nf));
    for (const auto &kv : edge_faces) {
      const std::vector<int> &fs = kv.second;
      for (size_t i = 0; i < fs.size(); i++) {
        for (size_t j = i + 1; j < fs.size(); j++) {
          adj[size_t(fs[i])].push_back(fs[j]);
          adj[size_t(fs[j])].push_back(fs[i]);
        }
      }
    }
    const double deg = std::max(0.0, std::min(180.0, angle_deg));
    const double cang = std::cos(deg * 3.14159265358979323846 / 180.0);
    std::vector<int> region;
    region.assign(size_t(nf), -1);
    int nreg = 0;
    for (int seed = 0; seed < nf; seed++) {
      if (region[size_t(seed)] >= 0) {
        continue;
      }
      const int rid = nreg++;
      std::queue<int> q;
      q.push(seed);
      region[size_t(seed)] = rid;
      while (!q.empty()) {
        const int f = q.front();
        q.pop();
        for (int g : adj[size_t(f)]) {
          if (region[size_t(g)] >= 0) {
            continue;
          }
          const double d = CGAL::to_double(nrm[size_t(f)] * nrm[size_t(g)]);
          if (d >= cang - 1e-8) {
            region[size_t(g)] = rid;
            q.push(g);
          }
        }
      }
    }
    const int minsz = std::max(1, min_size);
    if (minsz > 1 && nreg > 1) {
      std::vector<int> count;
      count.assign(size_t(nreg), 0);
      for (int f = 0; f < nf; f++) {
        count[size_t(region[size_t(f)])]++;
      }
      bool changed = true;
      while (changed) {
        changed = false;
        for (int rid = 0; rid < nreg; rid++) {
          if (count[size_t(rid)] == 0 || count[size_t(rid)] >= minsz) {
            continue;
          }
          std::map<int, int> votes;
          for (int f = 0; f < nf; f++) {
            if (region[size_t(f)] != rid) {
              continue;
            }
            for (int g : adj[size_t(f)]) {
              const int rg = region[size_t(g)];
              if (rg != rid && count[size_t(rg)] > 0) {
                votes[rg]++;
              }
            }
          }
          if (votes.empty()) {
            continue;
          }
          int best = votes.begin()->first;
          int best_v = -1;
          for (const auto &kv : votes) {
            if (kv.second > best_v) {
              best_v = kv.second;
              best = kv.first;
            }
          }
          for (int f = 0; f < nf; f++) {
            if (region[size_t(f)] == rid) {
              region[size_t(f)] = best;
            }
          }
          count[size_t(best)] += count[size_t(rid)];
          count[size_t(rid)] = 0;
          changed = true;
        }
      }
      std::vector<int> remap;
      remap.assign(size_t(nreg), -1);
      int compact = 0;
      for (int f = 0; f < nf; f++) {
        const int r = region[size_t(f)];
        if (remap[size_t(r)] < 0) {
          remap[size_t(r)] = compact++;
        }
        region[size_t(f)] = remap[size_t(r)];
      }
    }
    result.positions_only = true;
    result.positions.assign(mesh.positions, mesh.positions + size_t(mesh.verts_num) * 3);
    result.face_tag = region;
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Graphcut Segment failed";
  }
  return result;
}

bool points_classification_features(const float *positions,
                                    int n,
                                    int knn,
                                    std::vector<float> &linearity,
                                    std::vector<float> &planarity,
                                    std::vector<float> &sphericity,
                                    std::vector<float> &omnivariance,
                                    std::vector<float> &anisotropy,
                                    std::vector<float> &verticality,
                                    std::string &error)
{
  CgalThrowGuard guard;
  if (!positions || n < 4) {
    error = "Point Features needs at least 4 points";
    return false;
  }
  try {
    pca_features(pts_from_xyz(positions, n),
                 knn,
                 linearity,
                 planarity,
                 sphericity,
                 omnivariance,
                 anisotropy,
                 verticality);
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Point Features failed";
    return false;
  }
}

bool points_classify_ethz(const float *positions,
                          int n,
                          const int *train_label,
                          int knn,
                          int n_trees,
                          std::vector<int> &out_label,
                          std::string &error)
{
  CgalThrowGuard guard;
  out_label.assign(size_t(n), 0);
  if (!positions || !train_label || n < 8) {
    error = "Classify needs points and a training Label attribute";
    return false;
  }
  try {
    int unlabeled = 0;
    for (int i = 0; i < n; i++) {
      if (train_label[i] < 0) {
        unlabeled++;
      }
    }
    if (unlabeled == 0) {
      /* Already fully labeled: copy, do not re-predict (that scrambled segments). */
      out_label.assign(train_label, train_label + n);
      return true;
    }

    std::vector<Point_3> pts = pts_from_xyz(positions, n);
    std::vector<float> lin, pla, sph, omn, ani, ver;
    pca_features(pts, knn, lin, pla, sph, omn, ani, ver);

    CGAL::Classification::Label_set labels;
    std::map<int, int> raw2lab;
    int nlab = 0;
    std::vector<int> gt;
    gt.assign(size_t(n), -1);
    for (int i = 0; i < n; i++) {
      if (train_label[i] < 0) {
        continue;
      }
      if (!raw2lab.count(train_label[i])) {
        raw2lab[train_label[i]] = nlab++;
        labels.add((std::string("L") + std::to_string(train_label[i])).c_str());
      }
      gt[size_t(i)] = raw2lab[train_label[i]];
    }
    if (nlab < 2) {
      error = "Classify: paint at least two training classes. Label >= 0 is training "
              "(e.g. 0 and 1); Label = -1 is unlabeled and will be predicted. "
              "Plug a Compare/Named Attribute into Label.";
      return false;
    }
    struct ScalarFeature : public CGAL::Classification::Feature_base {
      const std::vector<float> *v;
      ScalarFeature(const char *name, const std::vector<float> *vv) : v(vv)
      {
        this->set_name(name);
      }
      float value(std::size_t i) override
      {
        return (*v)[i];
      }
    };
    CGAL::Classification::Feature_set features;
    features.add<ScalarFeature>("linearity", &lin);
    features.add<ScalarFeature>("planarity", &pla);
    features.add<ScalarFeature>("sphericity", &sph);
    features.add<ScalarFeature>("anisotropy", &ani);
    features.add<ScalarFeature>("verticality", &ver);

    CGAL::Classification::ETHZ::Random_forest_classifier classifier(labels, features);
    classifier.train(gt, true, std::size_t(std::max(8, n_trees)));
    std::vector<int> pred(size_t(n), 0);
    CGAL::Classification::classify<CGAL::Sequential_tag>(pts, labels, classifier, pred);
    std::vector<int> lab2raw(size_t(nlab), 0);
    for (const auto &kv : raw2lab) {
      lab2raw[size_t(kv.second)] = kv.first;
    }
    out_label.resize(size_t(n));
    for (int i = 0; i < n; i++) {
      if (train_label[i] >= 0) {
        /* Keep training / already-segmented labels. Only -1 is predicted. */
        out_label[size_t(i)] = train_label[i];
        continue;
      }
      const int lab = pred[size_t(i)];
      out_label[size_t(i)] = (lab >= 0 && lab < nlab) ? lab2raw[size_t(lab)] : 0;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Classify failed";
    return false;
  }
}

static bool kabsch(const std::vector<Point_3> &a,
                   const std::vector<Point_3> &b,
                   Eigen::Matrix3d &R,
                   Eigen::Vector3d &t)
{
  const int n = int(std::min(a.size(), b.size()));
  if (n < 3) {
    return false;
  }
  Eigen::Vector3d ca(0, 0, 0), cb(0, 0, 0);
  for (int i = 0; i < n; i++) {
    ca += Eigen::Vector3d(CGAL::to_double(a[size_t(i)].x()),
                          CGAL::to_double(a[size_t(i)].y()),
                          CGAL::to_double(a[size_t(i)].z()));
    cb += Eigen::Vector3d(CGAL::to_double(b[size_t(i)].x()),
                          CGAL::to_double(b[size_t(i)].y()),
                          CGAL::to_double(b[size_t(i)].z()));
  }
  ca /= n;
  cb /= n;
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (int i = 0; i < n; i++) {
    Eigen::Vector3d va(CGAL::to_double(a[size_t(i)].x()) - ca.x(),
                       CGAL::to_double(a[size_t(i)].y()) - ca.y(),
                       CGAL::to_double(a[size_t(i)].z()) - ca.z());
    Eigen::Vector3d vb(CGAL::to_double(b[size_t(i)].x()) - cb.x(),
                       CGAL::to_double(b[size_t(i)].y()) - cb.y(),
                       CGAL::to_double(b[size_t(i)].z()) - cb.z());
    H += va * vb.transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0) {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1;
    R = V * svd.matrixU().transpose();
  }
  t = cb - R * ca;
  return true;
}

static Point_3 xform_p(const Point_3 &p, const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
{
  Eigen::Vector3d v(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
  v = R * v + t;
  return mk_p3(v.x(), v.y(), v.z());
}

bool points_register_icp(const float *src,
                         int n_src,
                         const float *tgt,
                         int n_tgt,
                         int iterations,
                         std::vector<float> &out_xyz,
                         std::string &error)
{
  CgalThrowGuard guard;
  if (!src || !tgt || n_src < 3 || n_tgt < 3) {
    error = "ICP Register needs two point sets";
    return false;
  }
  try {
    std::vector<Point_3> S = pts_from_xyz(src, n_src);
    std::vector<Point_3> T = pts_from_xyz(tgt, n_tgt);
    KNNTree tree(T.begin(), T.end());
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    iterations = std::max(1, std::min(80, iterations));
    for (int it = 0; it < iterations; it++) {
      std::vector<Point_3> A, B;
      A.reserve(size_t(n_src));
      B.reserve(size_t(n_src));
      for (const Point_3 &p : S) {
        const Point_3 q = xform_p(p, R, t);
        KNN search(tree, q, 1);
        A.push_back(q);
        B.push_back(knn_point(search.begin()));
      }
      Eigen::Matrix3d R2;
      Eigen::Vector3d t2;
      if (!kabsch(A, B, R2, t2)) {
        break;
      }
      t = R2 * t + t2;
      R = R2 * R;
    }
    out_xyz.resize(size_t(n_src) * 3);
    for (int i = 0; i < n_src; i++) {
      const Point_3 q = xform_p(S[size_t(i)], R, t);
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
}

bool points_register_4pcs(const float *src,
                          int n_src,
                          const float *tgt,
                          int n_tgt,
                          int samples,
                          int icp_iterations,
                          std::vector<float> &out_xyz,
                          std::string &error)
{
  CgalThrowGuard guard;
  if (!src || !tgt || n_src < 4 || n_tgt < 4) {
    error = "Super4PCS needs at least 4 points in each cloud";
    return false;
  }
  try {
    std::vector<Point_3> S = pts_from_xyz(src, n_src);
    std::vector<Point_3> T = pts_from_xyz(tgt, n_tgt);
    KNNTree ttree(T.begin(), T.end());
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> ds(0, n_src - 1), dt(0, n_tgt - 1);
    samples = std::max(8, std::min(400, samples));
    Eigen::Matrix3d bestR = Eigen::Matrix3d::Identity();
    Eigen::Vector3d bestt = Eigen::Vector3d::Zero();
    int best_in = -1;
    CGAL::Bbox_3 bb = S[0].bbox();
    for (const Point_3 &p : S) {
      bb += p.bbox();
    }
    const double dx = bb.xmax() - bb.xmin(), dy = bb.ymax() - bb.ymin(), dz = bb.zmax() - bb.zmin();
    const double diag = std::max(1e-6, std::sqrt(dx * dx + dy * dy + dz * dz));
    const double thr = 0.02 * diag;
    for (int s = 0; s < samples; s++) {
      const int i0 = ds(rng), i1 = ds(rng), i2 = ds(rng);
      const int j0 = dt(rng), j1 = dt(rng), j2 = dt(rng);
      if (i0 == i1 || i1 == i2 || i0 == i2) {
        continue;
      }
      std::vector<Point_3> A{S[size_t(i0)], S[size_t(i1)], S[size_t(i2)]};
      std::vector<Point_3> B{T[size_t(j0)], T[size_t(j1)], T[size_t(j2)]};
      Eigen::Matrix3d R;
      Eigen::Vector3d t;
      if (!kabsch(A, B, R, t)) {
        continue;
      }
      int in = 0;
      const int step = std::max(1, n_src / 200);
      for (int i = 0; i < n_src; i += step) {
        const Point_3 q = xform_p(S[size_t(i)], R, t);
        KNN search(ttree, q, 1);
        if (CGAL::to_double(CGAL::squared_distance(q, knn_point(search.begin()))) < thr * thr) {
          in++;
        }
      }
      if (in > best_in) {
        best_in = in;
        bestR = R;
        bestt = t;
      }
    }
    std::vector<float> transformed(size_t(n_src) * 3);
    for (int i = 0; i < n_src; i++) {
      const Point_3 q = xform_p(S[size_t(i)], bestR, bestt);
      transformed[size_t(i) * 3 + 0] = float(CGAL::to_double(q.x()));
      transformed[size_t(i) * 3 + 1] = float(CGAL::to_double(q.y()));
      transformed[size_t(i) * 3 + 2] = float(CGAL::to_double(q.z()));
    }
    return points_register_icp(transformed.data(),
                               n_src,
                               tgt,
                               n_tgt,
                               std::max(4, icp_iterations),
                               out_xyz,
                               error);
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
}
MeshResult points_alpha_wrap_2(const float *positions,
                               int n,
                               const MeshIn *segments,
                               double alpha,
                               double offset)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<Point_2> pts;
    for (int i = 0; i < n && positions; i++) {
      pts.emplace_back(positions[i * 3], positions[i * 3 + 1]);
    }
    if (segments && segments->positions) {
      const int vn = segments->verts_num;
      for (int i = 0; i < vn; i++) {
        pts.emplace_back(segments->positions[i * 3], segments->positions[i * 3 + 1]);
      }
    }
    if (pts.size() < 3) {
      result.error = "Alpha Wrap 2D needs XY points or a mesh";
      return result;
    }
    using Vb = CGAL::Alpha_shape_vertex_base_2<Kernel>;
    using Fb = CGAL::Alpha_shape_face_base_2<Kernel>;
    using Tds = CGAL::Triangulation_data_structure_2<Vb, Fb>;
    using Dt = CGAL::Delaunay_triangulation_2<Kernel, Tds>;
    using Alpha = CGAL::Alpha_shape_2<Dt>;
    using Vector_2 = Kernel::Vector_2;
    const double a = std::max(1e-12, alpha);
    Alpha as(pts.begin(), pts.end(), a, Alpha::GENERAL);
    as.set_alpha(a);
    result.face_offsets = {0};
    /* First pass: collect INTERIOR faces, then offset ring vertices. */
    struct Tri {
      Point_2 p[3];
    };
    std::vector<Tri> tris;
    for (auto f = as.finite_faces_begin(); f != as.finite_faces_end(); ++f) {
      if (as.classify(f) != Alpha::INTERIOR) {
        continue;
      }
      Tri t;
      t.p[0] = f->vertex(0)->point();
      t.p[1] = f->vertex(1)->point();
      t.p[2] = f->vertex(2)->point();
      tris.push_back(t);
    }
    auto key = [](const Point_2 &p) {
      return std::pair<long long, long long>{std::llround(CGAL::to_double(p.x()) * 1e7),
                                             std::llround(CGAL::to_double(p.y()) * 1e7)};
    };
    std::map<std::pair<long long, long long>, Vector_2> nacc;
    for (const Tri &t : tris) {
      const Vector_2 e0 = t.p[1] - t.p[0];
      const Vector_2 e1 = t.p[2] - t.p[1];
      const Vector_2 e2 = t.p[0] - t.p[2];
      const Vector_2 n0(-CGAL::to_double(e0.y()), CGAL::to_double(e0.x()));
      const Vector_2 n1(-CGAL::to_double(e1.y()), CGAL::to_double(e1.x()));
      const Vector_2 n2(-CGAL::to_double(e2.y()), CGAL::to_double(e2.x()));
      nacc[key(t.p[0])] = nacc[key(t.p[0])] + n0 + n2;
      nacc[key(t.p[1])] = nacc[key(t.p[1])] + n0 + n1;
      nacc[key(t.p[2])] = nacc[key(t.p[2])] + n1 + n2;
    }
    auto offset_p = [&](const Point_2 &p) {
      const auto k = key(p);
      Vector_2 n = nacc[k];
      const double ln = std::sqrt(CGAL::to_double(n.squared_length()));
      if (ln > 1e-15) {
        n = n / FT(ln);
      }
      return Point_2(CGAL::to_double(p.x()) + CGAL::to_double(n.x()) * offset,
                     CGAL::to_double(p.y()) + CGAL::to_double(n.y()) * offset);
    };
    std::map<std::pair<long long, long long>, int> imap;
    auto iv = [&](const Point_2 &p) {
      const auto k = key(p);
      const auto it = imap.find(k);
      if (it != imap.end()) {
        return it->second;
      }
      const int idx = result.verts_num();
      imap[k] = idx;
      const Point_2 q = offset_p(p);
      result.positions.push_back(float(CGAL::to_double(q.x())));
      result.positions.push_back(float(CGAL::to_double(q.y())));
      result.positions.push_back(0.0f);
      return idx;
    };
    for (const Tri &t : tris) {
      result.corner_verts.push_back(iv(t.p[0]));
      result.corner_verts.push_back(iv(t.p[1]));
      result.corner_verts.push_back(iv(t.p[2]));
      result.face_offsets.push_back(int(result.corner_verts.size()));
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Alpha Wrap 2D produced no polygon (try a larger Alpha)";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Alpha Wrap 2D failed";
  }
  return result;
}

MeshResult mesh_cage_deform_3(const MeshIn &cage_rest,
                              const MeshIn &cage_pose,
                              const MeshIn &interior)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh rest, pose, inn;
  std::string error;
  if (!load_tri(cage_rest, rest, error) || !load_tri(cage_pose, pose, error)) {
    result.error = error.empty() ? "Cage Deform 3D needs rest and posed cages" : error;
    return result;
  }
  if (!mesh_in_to_surface_mesh(interior, inn, error, true)) {
    result.error = "Cage Deform 3D needs an interior mesh";
    return result;
  }
  if (rest.number_of_vertices() != pose.number_of_vertices()) {
    result.error = "Cage rest/pose must have the same vertex count";
    return result;
  }
  try {
    std::vector<Point_3> rest_p, pose_p;
    auto rit = rest.vertices_begin();
    auto pit = pose.vertices_begin();
    for (; rit != rest.vertices_end() && pit != pose.vertices_end(); ++rit, ++pit) {
      rest_p.push_back(rest.point(*rit));
      pose_p.push_back(pose.point(*pit));
    }
    const size_t nv = rest_p.size();
    /* Linear-blend of cage *deltas* so rest==pose is identity (plain Shepard is not). */
    for (const VI v : inn.vertices()) {
      const Point_3 x = inn.point(v);
      double dx = 0, dy = 0, dz = 0, sw = 0;
      for (size_t i = 0; i < nv; i++) {
        const double d2 = std::max(1e-18, CGAL::to_double(CGAL::squared_distance(x, rest_p[i])));
        const double w = 1.0 / d2;
        dx += w * (CGAL::to_double(pose_p[i].x()) - CGAL::to_double(rest_p[i].x()));
        dy += w * (CGAL::to_double(pose_p[i].y()) - CGAL::to_double(rest_p[i].y()));
        dz += w * (CGAL::to_double(pose_p[i].z()) - CGAL::to_double(rest_p[i].z()));
        sw += w;
      }
      if (sw > 0) {
        inn.point(v) = mk_p3(CGAL::to_double(x.x()) + dx / sw,
                             CGAL::to_double(x.y()) + dy / sw,
                             CGAL::to_double(x.z()) + dz / sw);
      }
    }
    result = surface_mesh_to_result(inn);
    result.positions_only = (result.verts_num() == interior.verts_num);
    result.ok = result.verts_num() > 0;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Cage Deform 3D failed (cage must be a closed triangle mesh)";
  }
  return result;
}

MeshResult mesh_dual_contour_grid(const float *sdf,
                                  int nx,
                                  int ny,
                                  int nz,
                                  double x0,
                                  double y0,
                                  double z0,
                                  double dx,
                                  double dy,
                                  double dz,
                                  double isovalue)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (!sdf || nx < 3 || ny < 3 || nz < 3) {
    result.error = "Dual Contour needs an SDF grid of at least 3×3×3";
    return result;
  }
  try {
    auto gid = [&](int i, int j, int k) {
      return (size_t(k) * size_t(ny) + size_t(j)) * size_t(nx) + size_t(i);
    };
    const int rx = nx - 1, ry = ny - 1, rz = nz - 1;
    std::vector<int> vox(size_t(rx) * size_t(ry) * size_t(rz), -1);
    auto vid = [&](int i, int j, int k) {
      return (size_t(k) * size_t(ry) + size_t(j)) * size_t(rx) + size_t(i);
    };
    const int eoff[12][2][3] = {
        {{0, 0, 0}, {1, 0, 0}},
        {{1, 0, 0}, {1, 1, 0}},
        {{1, 1, 0}, {0, 1, 0}},
        {{0, 1, 0}, {0, 0, 0}},
        {{0, 0, 1}, {1, 0, 1}},
        {{1, 0, 1}, {1, 1, 1}},
        {{1, 1, 1}, {0, 1, 1}},
        {{0, 1, 1}, {0, 0, 1}},
        {{0, 0, 0}, {0, 0, 1}},
        {{1, 0, 0}, {1, 0, 1}},
        {{1, 1, 0}, {1, 1, 1}},
        {{0, 1, 0}, {0, 1, 1}},
    };
    result.face_offsets = {0};
    for (int k = 0; k < rz; k++) {
      for (int j = 0; j < ry; j++) {
        for (int i = 0; i < rx; i++) {
          double sx = 0, sy = 0, sz = 0;
          int hits = 0;
          for (int e = 0; e < 12; e++) {
            const int i0 = i + eoff[e][0][0], j0 = j + eoff[e][0][1], k0 = k + eoff[e][0][2];
            const int i1 = i + eoff[e][1][0], j1 = j + eoff[e][1][1], k1 = k + eoff[e][1][2];
            const float a = sdf[gid(i0, j0, k0)] - float(isovalue);
            const float b = sdf[gid(i1, j1, k1)] - float(isovalue);
            if (a * b > 0) {
              continue;
            }
            const double t = (std::abs(b - a) < 1e-12) ? 0.5 : double(-a / (b - a));
            sx += (i0 + t * (i1 - i0));
            sy += (j0 + t * (j1 - j0));
            sz += (k0 + t * (k1 - k0));
            hits++;
          }
          if (hits == 0) {
            continue;
          }
          vox[vid(i, j, k)] = result.verts_num();
          result.positions.push_back(float(x0 + dx * sx / hits));
          result.positions.push_back(float(y0 + dy * sy / hits));
          result.positions.push_back(float(z0 + dz * sz / hits));
        }
      }
    }
    auto emit_quad = [&](int a, int b, int c, int d) {
      if (a < 0 || b < 0 || c < 0 || d < 0) {
        return;
      }
      result.corner_verts.push_back(a);
      result.corner_verts.push_back(b);
      result.corner_verts.push_back(c);
      result.face_offsets.push_back(int(result.corner_verts.size()));
      result.corner_verts.push_back(a);
      result.corner_verts.push_back(c);
      result.corner_verts.push_back(d);
      result.face_offsets.push_back(int(result.corner_verts.size()));
    };
    auto sign_at = [&](int i, int j, int k) {
      return (sdf[gid(i, j, k)] - float(isovalue)) >= 0.0f;
    };
    for (int k = 0; k < rz; k++) {
      for (int j = 0; j < ry; j++) {
        for (int i = 0; i < rx; i++) {
          if (sign_at(i, j, k) != sign_at(i + 1, j, k) && j > 0 && k > 0) {
            emit_quad(vox[vid(i, j - 1, k - 1)],
                      vox[vid(i, j, k - 1)],
                      vox[vid(i, j, k)],
                      vox[vid(i, j - 1, k)]);
          }
          if (sign_at(i, j, k) != sign_at(i, j + 1, k) && i > 0 && k > 0) {
            emit_quad(vox[vid(i - 1, j, k - 1)],
                      vox[vid(i, j, k - 1)],
                      vox[vid(i, j, k)],
                      vox[vid(i - 1, j, k)]);
          }
          if (sign_at(i, j, k) != sign_at(i, j, k + 1) && i > 0 && j > 0) {
            emit_quad(vox[vid(i - 1, j - 1, k)],
                      vox[vid(i, j - 1, k)],
                      vox[vid(i, j, k)],
                      vox[vid(i - 1, j, k)]);
          }
        }
      }
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Isosurface is empty (the SDF field never crosses Isovalue in this box)";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Dual Contour failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
