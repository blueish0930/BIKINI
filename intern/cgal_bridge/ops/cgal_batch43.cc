/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 43 (2026-08-19, new only — do not restore deleted ids):
 *  - Fair Hole Fill (triangulate_refine_and_fair_hole)
 *  - Simplify Polyline 3D (PMP experimental::simplify_polyline)
 *  - Sphere Region Growing (Least_squares_sphere_fit_region)
 *  - Cylinder Region Growing (Least_squares_cylinder_fit_region)
 *  - Circle Region Growing (Least_squares_circle_fit_region on PCA plane)
 *  - Line Region Growing (PCA line + sphere neighbors)
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

#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Polygon_mesh_processing/internal/simplify_polyline.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Shape_detection/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Point_set.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/squared_distance_3.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {
namespace {

using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Vector_2 = Kernel::Vector_2;
using Vector_3 = Kernel::Vector_3;
using Plane_3 = Kernel::Plane_3;
using Line_3 = Kernel::Line_3;
using Sphere_3 = Kernel::Sphere_3;
using FT = Kernel::FT;

using PwnRG = std::pair<Point_3, Vector_3>;
using PwnRGIt = std::vector<PwnRG>::const_iterator;

struct PwnRGPointMap {
  using key_type = PwnRGIt;
  using value_type = Point_3;
  using reference = const Point_3 &;
  using category = boost::readable_property_map_tag;
};
struct PwnRGNormalMap {
  using key_type = PwnRGIt;
  using value_type = Vector_3;
  using reference = const Vector_3 &;
  using category = boost::readable_property_map_tag;
};
inline const Point_3 &get(const PwnRGPointMap & /*m*/, const PwnRGIt it)
{
  return it->first;
}
inline const Vector_3 &get(const PwnRGNormalMap & /*m*/, const PwnRGIt it)
{
  return it->second;
}

using Pwn2 = std::pair<Point_2, Vector_2>;
using Pwn2It = std::vector<Pwn2>::const_iterator;
struct Pwn2PointMap {
  using key_type = Pwn2It;
  using value_type = Point_2;
  using reference = const Point_2 &;
  using category = boost::readable_property_map_tag;
};
struct Pwn2NormalMap {
  using key_type = Pwn2It;
  using value_type = Vector_2;
  using reference = const Vector_2 &;
  using category = boost::readable_property_map_tag;
};
inline const Point_2 &get(const Pwn2PointMap & /*m*/, const Pwn2It it)
{
  return it->first;
}
inline const Vector_2 &get(const Pwn2NormalMap & /*m*/, const Pwn2It it)
{
  return it->second;
}

static Point_3 to_p3(const float *xyz)
{
  return Point_3(xyz[0], xyz[1], xyz[2]);
}

static void orthonormal_frame(const Vector_3 &axis, Vector_3 &u, Vector_3 &v)
{
  Vector_3 tmp = (std::abs(CGAL::to_double(axis.z())) < 0.9) ? Vector_3(0, 0, 1) : Vector_3(0, 1, 0);
  u = CGAL::cross_product(axis, tmp);
  const double ul = std::sqrt(CGAL::to_double(u.squared_length()));
  if (ul < 1e-18) {
    u = Vector_3(1, 0, 0);
  }
  else {
    u = Vector_3(CGAL::to_double(u.x()) / ul,
                 CGAL::to_double(u.y()) / ul,
                 CGAL::to_double(u.z()) / ul);
  }
  v = CGAL::cross_product(axis, u);
  const double vl = std::sqrt(CGAL::to_double(v.squared_length()));
  if (vl > 1e-18) {
    v = Vector_3(CGAL::to_double(v.x()) / vl,
                 CGAL::to_double(v.y()) / vl,
                 CGAL::to_double(v.z()) / vl);
  }
}

static void append_uv_sphere(float cx,
                             float cy,
                             float cz,
                             float radius,
                             int segments,
                             MeshResult &result)
{
  segments = std::clamp(segments, 8, 64);
  const int rings = std::max(4, segments / 2);
  result.positions.clear();
  result.corner_verts.clear();
  result.face_offsets.clear();

  result.positions.push_back(cx);
  result.positions.push_back(cy);
  result.positions.push_back(cz + radius);
  for (int i = 1; i < rings; i++) {
    const float phi = float(M_PI) * float(i) / float(rings);
    const float y = std::cos(phi);
    const float r = std::sin(phi);
    for (int j = 0; j < segments; j++) {
      const float u = 2.0f * float(M_PI) * float(j) / float(segments);
      result.positions.push_back(cx + radius * r * std::cos(u));
      result.positions.push_back(cy + radius * r * std::sin(u));
      result.positions.push_back(cz + radius * y);
    }
  }
  result.positions.push_back(cx);
  result.positions.push_back(cy);
  result.positions.push_back(cz - radius);

  const int south = int(result.positions.size() / 3) - 1;
  for (int j = 0; j < segments; j++) {
    const int a = 1 + j;
    const int b = 1 + (j + 1) % segments;
    result.corner_verts.push_back(0);
    result.corner_verts.push_back(a);
    result.corner_verts.push_back(b);
  }
  for (int i = 0; i < rings - 2; i++) {
    const int row0 = 1 + i * segments;
    const int row1 = 1 + (i + 1) * segments;
    for (int j = 0; j < segments; j++) {
      const int j1 = (j + 1) % segments;
      result.corner_verts.push_back(row0 + j);
      result.corner_verts.push_back(row1 + j);
      result.corner_verts.push_back(row1 + j1);
      result.corner_verts.push_back(row0 + j);
      result.corner_verts.push_back(row1 + j1);
      result.corner_verts.push_back(row0 + j1);
    }
  }
  const int last_ring = 1 + (rings - 2) * segments;
  for (int j = 0; j < segments; j++) {
    const int a = last_ring + j;
    const int b = last_ring + (j + 1) % segments;
    result.corner_verts.push_back(south);
    result.corner_verts.push_back(b);
    result.corner_verts.push_back(a);
  }
  result.ok = result.corners_num() >= 3;
}

static void append_uv_sphere_onto(float cx,
                                  float cy,
                                  float cz,
                                  float radius,
                                  int segments,
                                  int tag,
                                  MeshResult &result)
{
  MeshResult tmp;
  append_uv_sphere(cx, cy, cz, radius, segments, tmp);
  if (!tmp.ok) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets = {0};
  }
  const int vbase = result.verts_num();
  result.positions.insert(result.positions.end(), tmp.positions.begin(), tmp.positions.end());
  if (tmp.face_offsets.empty()) {
    for (int i = 0; i + 2 < tmp.corners_num(); i += 3) {
      result.corner_verts.push_back(vbase + tmp.corner_verts[size_t(i)]);
      result.corner_verts.push_back(vbase + tmp.corner_verts[size_t(i) + 1]);
      result.corner_verts.push_back(vbase + tmp.corner_verts[size_t(i) + 2]);
      result.face_offsets.push_back(int(result.corner_verts.size()));
      result.face_tag.push_back(tag);
    }
  }
  result.ok = result.faces_num() > 0;
}

static void append_cylinder(const Point_3 &a,
                            const Point_3 &b,
                            double radius,
                            int segments,
                            int tag,
                            bool caps,
                            MeshResult &result)
{
  segments = std::clamp(segments, 8, 64);
  Vector_3 axis(b.x() - a.x(), b.y() - a.y(), b.z() - a.z());
  const double len = std::sqrt(CGAL::to_double(axis.squared_length()));
  if (len < 1e-12 || radius <= 0.0) {
    return;
  }
  axis = Vector_3(CGAL::to_double(axis.x()) / len,
                  CGAL::to_double(axis.y()) / len,
                  CGAL::to_double(axis.z()) / len);
  Vector_3 u, v;
  orthonormal_frame(axis, u, v);
  if (result.face_offsets.empty()) {
    result.face_offsets = {0};
  }
  const int vbase = result.verts_num();
  for (int end = 0; end < 2; end++) {
    const Point_3 &p = (end == 0) ? a : b;
    for (int j = 0; j < segments; j++) {
      const double ang = 2.0 * M_PI * double(j) / double(segments);
      const double ca = std::cos(ang) * radius;
      const double sa = std::sin(ang) * radius;
      result.positions.push_back(float(CGAL::to_double(p.x()) + ca * CGAL::to_double(u.x()) +
                                       sa * CGAL::to_double(v.x())));
      result.positions.push_back(float(CGAL::to_double(p.y()) + ca * CGAL::to_double(u.y()) +
                                       sa * CGAL::to_double(v.y())));
      result.positions.push_back(float(CGAL::to_double(p.z()) + ca * CGAL::to_double(u.z()) +
                                       sa * CGAL::to_double(v.z())));
    }
  }
  for (int j = 0; j < segments; j++) {
    const int j1 = (j + 1) % segments;
    result.corner_verts.push_back(vbase + j);
    result.corner_verts.push_back(vbase + segments + j);
    result.corner_verts.push_back(vbase + segments + j1);
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_tag.push_back(tag);
    result.corner_verts.push_back(vbase + j);
    result.corner_verts.push_back(vbase + segments + j1);
    result.corner_verts.push_back(vbase + j1);
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_tag.push_back(tag);
  }
  if (caps) {
    for (int end = 0; end < 2; end++) {
      const int base = vbase + end * segments;
      for (int j = 1; j + 1 < segments; j++) {
        if (end == 0) {
          result.corner_verts.push_back(base);
          result.corner_verts.push_back(base + j);
          result.corner_verts.push_back(base + j + 1);
        }
        else {
          result.corner_verts.push_back(base);
          result.corner_verts.push_back(base + j + 1);
          result.corner_verts.push_back(base + j);
        }
        result.face_offsets.push_back(int(result.corner_verts.size()));
        result.face_tag.push_back(tag);
      }
    }
  }
  result.ok = result.faces_num() > 0;
}

static void append_disk(const Point_3 &center,
                        const Vector_3 &u,
                        const Vector_3 &v,
                        double radius,
                        int segments,
                        int tag,
                        MeshResult &result)
{
  segments = std::clamp(segments, 8, 64);
  if (radius <= 0.0) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets = {0};
  }
  const int vbase = result.verts_num();
  result.positions.push_back(float(CGAL::to_double(center.x())));
  result.positions.push_back(float(CGAL::to_double(center.y())));
  result.positions.push_back(float(CGAL::to_double(center.z())));
  for (int j = 0; j < segments; j++) {
    const double ang = 2.0 * M_PI * double(j) / double(segments);
    const double ca = std::cos(ang) * radius;
    const double sa = std::sin(ang) * radius;
    result.positions.push_back(float(CGAL::to_double(center.x()) + ca * CGAL::to_double(u.x()) +
                                     sa * CGAL::to_double(v.x())));
    result.positions.push_back(float(CGAL::to_double(center.y()) + ca * CGAL::to_double(u.y()) +
                                     sa * CGAL::to_double(v.y())));
    result.positions.push_back(float(CGAL::to_double(center.z()) + ca * CGAL::to_double(u.z()) +
                                     sa * CGAL::to_double(v.z())));
  }
  for (int j = 0; j < segments; j++) {
    const int j1 = 1 + (j + 1) % segments;
    result.corner_verts.push_back(vbase);
    result.corner_verts.push_back(vbase + 1 + j);
    result.corner_verts.push_back(vbase + j1);
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_tag.push_back(tag);
  }
  result.ok = result.faces_num() > 0;
}

static const float *ensure_normals(const float *in_xyz,
                                   const float *in_nxyz,
                                   int n,
                                   std::vector<float> &estimated)
{
  if (in_nxyz) {
    return in_nxyz;
  }
  using Pair = std::pair<Point_3, Vector_3>;
  std::vector<Pair> tmp;
  tmp.reserve(size_t(n));
  for (int i = 0; i < n; i++) {
    tmp.emplace_back(Point_3(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                     Vector_3(0, 0, 1));
  }
  const int k = std::min(24, std::max(3, n - 1));
  CGAL::jet_estimate_normals<CGAL::Sequential_tag>(
      tmp,
      k,
      CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pair>()).normal_map(
          CGAL::Second_of_pair_property_map<Pair>()));
  CGAL::mst_orient_normals(
      tmp,
      k,
      CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pair>()).normal_map(
          CGAL::Second_of_pair_property_map<Pair>()));
  estimated.resize(size_t(n) * 3);
  for (int i = 0; i < n; i++) {
    estimated[size_t(i) * 3 + 0] = float(tmp[size_t(i)].second.x());
    estimated[size_t(i) * 3 + 1] = float(tmp[size_t(i)].second.y());
    estimated[size_t(i) * 3 + 2] = float(tmp[size_t(i)].second.z());
  }
  return estimated.data();
}

static std::vector<PwnRG> make_pwn(const float *in_xyz, const float *nxyz, int n)
{
  std::vector<PwnRG> pwn;
  pwn.reserve(size_t(n));
  for (int i = 0; i < n; i++) {
    pwn.emplace_back(Point_3(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                     Vector_3(nxyz[i * 3], nxyz[i * 3 + 1], nxyz[i * 3 + 2]));
  }
  return pwn;
}

static void fill_region_ids(const std::vector<PwnRG> &pwn,
                            const std::vector<PwnRGIt> &items,
                            int rid,
                            int n,
                            std::vector<int> *out_region)
{
  if (!out_region) {
    return;
  }
  for (const PwnRGIt it : items) {
    const int idx = int(std::distance(pwn.cbegin(), it));
    if (idx >= 0 && idx < n) {
      (*out_region)[size_t(idx)] = rid;
    }
  }
}

static std::pair<Point_3, Point_3> axis_span(const Line_3 &axis, const std::vector<PwnRGIt> &items)
{
  const Point_3 o = axis.point(0);
  Vector_3 d = axis.to_vector();
  const double dd = CGAL::to_double(d.squared_length());
  if (dd < 1e-24 || items.empty()) {
    return {o, o};
  }
  d = Vector_3(CGAL::to_double(d.x()) / std::sqrt(dd),
               CGAL::to_double(d.y()) / std::sqrt(dd),
               CGAL::to_double(d.z()) / std::sqrt(dd));
  double tmin = std::numeric_limits<double>::infinity();
  double tmax = -std::numeric_limits<double>::infinity();
  for (const PwnRGIt it : items) {
    const Vector_3 v(it->first.x() - o.x(), it->first.y() - o.y(), it->first.z() - o.z());
    const double t = CGAL::to_double(v.x() * d.x() + v.y() * d.y() + v.z() * d.z());
    tmin = std::min(tmin, t);
    tmax = std::max(tmax, t);
  }
  if (!(tmin < tmax)) {
    tmax = tmin + 1e-4;
  }
  const Point_3 a(CGAL::to_double(o.x()) + tmin * CGAL::to_double(d.x()),
                  CGAL::to_double(o.y()) + tmin * CGAL::to_double(d.y()),
                  CGAL::to_double(o.z()) + tmin * CGAL::to_double(d.z()));
  const Point_3 b(CGAL::to_double(o.x()) + tmax * CGAL::to_double(d.x()),
                  CGAL::to_double(o.y()) + tmax * CGAL::to_double(d.y()),
                  CGAL::to_double(o.z()) + tmax * CGAL::to_double(d.z()));
  return {a, b};
}

class LineRegion3 {
 public:
  using Item = PwnRGIt;
  using Region = std::vector<Item>;
  using Primitive = Line_3;
  using Region_unordered_map = std::unordered_map<Item,
                                                  std::size_t,
                                                  CGAL::Shape_detection::internal::hash_item<Item>>;
  using Region_index_map = boost::associative_property_map<Region_unordered_map>;

  LineRegion3(double max_distance, int min_region_size)
      : m_distance_threshold(std::max(0.0, max_distance)),
        m_min_region_size(std::max(2, min_region_size)),
        m_line(Point_3(0, 0, 0), Vector_3(1, 0, 0))
  {
  }

  Region_index_map region_index_map()
  {
    return Region_index_map(m_region_map);
  }

  Primitive primitive() const
  {
    return m_line;
  }

  bool is_part_of_region(const Item query, const Region &region) const
  {
    const double thresh2 = m_distance_threshold * m_distance_threshold;
    if (region.size() < 2) {
      if (region.empty()) {
        return true;
      }
      return CGAL::to_double(CGAL::squared_distance(query->first, region[0]->first)) <=
             std::max(thresh2, 1e-18);
    }
    return CGAL::to_double(CGAL::squared_distance(query->first, m_line)) <= thresh2;
  }

  bool is_valid_region(const Region &region) const
  {
    return region.size() >= size_t(m_min_region_size);
  }

  bool update(const Region &region)
  {
    if (region.empty()) {
      return false;
    }
    if (region.size() == 1) {
      m_line = Line_3(region[0]->first, Vector_3(1, 0, 0));
      return true;
    }
    std::vector<Point_3> pts;
    pts.reserve(region.size());
    for (const Item it : region) {
      pts.push_back(it->first);
    }
    Line_3 line;
    CGAL::linear_least_squares_fitting_3(pts.begin(), pts.end(), line, CGAL::Dimension_tag<0>());
    m_line = line;
    return true;
  }

 private:
  double m_distance_threshold;
  int m_min_region_size;
  Line_3 m_line;
  Region_unordered_map m_region_map;
};

static std::pair<int, int> ek(int a, int b)
{
  return (a < b) ? std::pair<int, int>(a, b) : std::pair<int, int>(b, a);
}

static void extract_polylines(const MeshIn &mesh, std::vector<std::vector<int>> &loops)
{
  loops.clear();
  if (mesh.edges_num > 0 && mesh.edge_v0 && mesh.edge_v1 && mesh.verts_num > 0) {
    std::vector<std::vector<int>> adj(size_t(mesh.verts_num));
    for (int e = 0; e < mesh.edges_num; e++) {
      const int a = mesh.edge_v0[e];
      const int b = mesh.edge_v1[e];
      if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num || a == b) {
        continue;
      }
      adj[size_t(a)].push_back(b);
      adj[size_t(b)].push_back(a);
    }
    std::set<std::pair<int, int>> used_e;
    auto walk = [&](int start, int next) {
      std::vector<int> chain;
      chain.push_back(start);
      int prev = start;
      int cur = next;
      used_e.insert(ek(prev, cur));
      while (cur >= 0 && cur < mesh.verts_num) {
        chain.push_back(cur);
        int nxt = -1;
        if (adj[size_t(cur)].size() == 2) {
          for (int nb : adj[size_t(cur)]) {
            if (nb != prev && used_e.find(ek(cur, nb)) == used_e.end()) {
              nxt = nb;
              break;
            }
          }
        }
        if (nxt < 0) {
          break;
        }
        used_e.insert(ek(cur, nxt));
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
      if (used_e.find(ek(v, nb)) != used_e.end()) {
        continue;
      }
      std::vector<int> chain = walk(v, nb);
      if (chain.size() >= 2) {
        loops.push_back(std::move(chain));
      }
    }
    for (int e = 0; e < mesh.edges_num; e++) {
      const int a = mesh.edge_v0[e];
      const int b = mesh.edge_v1[e];
      if (a < 0 || b < 0 || a >= mesh.verts_num || b >= mesh.verts_num) {
        continue;
      }
      if (used_e.find(ek(a, b)) != used_e.end()) {
        continue;
      }
      std::vector<int> chain = walk(a, b);
      if (chain.size() >= 2) {
        loops.push_back(std::move(chain));
      }
    }
    return;
  }
  if (mesh.verts_num >= 2) {
    std::vector<int> chain(size_t(mesh.verts_num));
    std::iota(chain.begin(), chain.end(), 0);
    loops.push_back(std::move(chain));
  }
}

}  // namespace

MeshResult mesh_fair_hole_fill(const MeshIn &mesh, int continuity)
{
  MeshResult result;
  CgalThrowGuard guard;
  Surface_mesh sm;
  std::string error;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true) || sm.number_of_faces() == 0) {
    result.error = error.empty() ? "Fair Hole Fill input failed" : error;
    return result;
  }
  try {
    continuity = std::clamp(continuity, 0, 2);
    /* Fairing requires a triangle mesh. Keep original-face ids so we can
     * restore n-gons afterwards; new hole faces stay triangles (f:src = -1). */
    if (!CGAL::is_triangle_mesh(sm)) {
      triangulate_faces_keep_ids(sm);
    }
    std::vector<Surface_mesh::Halfedge_index> border_loops;
    CGAL::extract_boundary_cycles(sm, std::back_inserter(border_loops));
    for (const auto h : border_loops) {
      if (!CGAL::is_valid_halfedge_descriptor(h, sm) || !sm.is_border(h)) {
        continue;
      }
      PMP::triangulate_refine_and_fair_hole(
          sm, h, CGAL::parameters::fairing_continuity(unsigned(continuity)));
    }
    fill_missing_vert_maps_from_edges(sm);
    detriangulate_by_original_face(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Fair Hole Fill failed";
  }
  return result;
}

WireResult mesh_simplify_polyline_3(const MeshIn &mesh, double max_distance, bool iterative)
{
  WireResult result;
  CgalThrowGuard guard;
  if (!mesh.positions || mesh.verts_num < 2) {
    result.error = "Simplify Polyline 3D needs at least 2 vertices";
    return result;
  }
  try {
    std::vector<std::vector<int>> loops;
    extract_polylines(mesh, loops);
    if (loops.empty()) {
      result.error = "Simplify Polyline 3D found no polylines";
      return result;
    }
    const double max_sq = std::max(0.0, max_distance) * std::max(0.0, max_distance);
    for (const std::vector<int> &loop : loops) {
      if (loop.size() < 2) {
        continue;
      }
      std::vector<Point_3> input;
      input.reserve(loop.size());
      for (int vi : loop) {
        if (vi < 0 || vi >= mesh.verts_num) {
          continue;
        }
        input.push_back(to_p3(mesh.positions + vi * 3));
      }
      if (input.size() < 2) {
        continue;
      }
      std::vector<Point_3> output;
      if (iterative) {
        PMP::experimental::simplify_polyline(
            input,
            output,
            max_sq,
            CGAL::parameters::algorithm(PMP::experimental::ITERATIVE));
      }
      else {
        PMP::experimental::simplify_polyline(input, output, max_sq);
      }
      if (output.size() < 2) {
        output = input;
      }
      const int nv_all = int(output.size());
      const bool closed = (nv_all >= 3 && output.front() == output.back());
      const int nv = closed ? nv_all - 1 : nv_all;
      if (nv < 2) {
        continue;
      }
      const int vbase = int(result.positions.size() / 3);
      for (int i = 0; i < nv; i++) {
        const Point_3 &p = output[size_t(i)];
        result.positions.push_back(float(CGAL::to_double(p.x())));
        result.positions.push_back(float(CGAL::to_double(p.y())));
        result.positions.push_back(float(CGAL::to_double(p.z())));
      }
      for (int i = 0; i < nv - 1; i++) {
        result.edge_v0.push_back(vbase + i);
        result.edge_v1.push_back(vbase + i + 1);
      }
      if (closed) {
        result.edge_v0.push_back(vbase + nv - 1);
        result.edge_v1.push_back(vbase);
      }
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = "Simplify Polyline 3D produced no edges";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Simplify Polyline 3D failed";
  }
  return result;
}

MeshResult points_sphere_region_growing(const float *in_xyz,
                                        const float *in_nxyz,
                                        int n,
                                        double neighbor_radius,
                                        double max_distance,
                                        double max_angle_deg,
                                        int min_region_size,
                                        double min_radius,
                                        double max_radius,
                                        int segments,
                                        std::vector<int> *out_region)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (out_region) {
    out_region->assign(size_t(std::max(0, n)), -1);
  }
  if (n < 3 || !in_xyz) {
    result.error = "Sphere Region Growing needs at least 3 points";
    return result;
  }
  if (neighbor_radius <= 0.0) {
    result.error = "Neighbor radius must be > 0";
    return result;
  }
  try {
    std::vector<float> estimated;
    const float *nxyz = ensure_normals(in_xyz, in_nxyz, n, estimated);
    std::vector<PwnRG> pwn = make_pwn(in_xyz, nxyz, n);

    using Neighbor_query = CGAL::Shape_detection::Point_set::Sphere_neighbor_query<Kernel,
                                                                                   PwnRGIt,
                                                                                   PwnRGPointMap>;
    using Region_type = CGAL::Shape_detection::Point_set::Least_squares_sphere_fit_region<
        Kernel,
        PwnRGIt,
        PwnRGPointMap,
        PwnRGNormalMap>;
    using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

    const double max_r = (max_radius > 0.0) ? max_radius :
                                              std::numeric_limits<double>::max();
    Neighbor_query neighbor_query(
        pwn, CGAL::parameters::sphere_radius(neighbor_radius).point_map(PwnRGPointMap()));
    Region_type region_type(CGAL::parameters::maximum_distance(max_distance)
                                .maximum_angle(max_angle_deg)
                                .minimum_region_size(std::max(3, min_region_size))
                                .minimum_radius(std::max(0.0, min_radius))
                                .maximum_radius(max_r)
                                .point_map(PwnRGPointMap())
                                .normal_map(PwnRGNormalMap()));
    Region_growing region_growing(pwn, neighbor_query, region_type);
    std::vector<typename Region_growing::Primitive_and_region> regions;
    region_growing.detect(std::back_inserter(regions));

    int rid = 0;
    for (const auto &pr : regions) {
      const Sphere_3 &sph = pr.first;
      const double r = std::sqrt(std::max(0.0, CGAL::to_double(sph.squared_radius())));
      if (r > 1e-12) {
        append_uv_sphere_onto(float(CGAL::to_double(sph.center().x())),
                              float(CGAL::to_double(sph.center().y())),
                              float(CGAL::to_double(sph.center().z())),
                              float(r),
                              segments,
                              rid,
                              result);
      }
      fill_region_ids(pwn, pr.second, rid, n, out_region);
      rid++;
    }
    result.alpha_used = double(rid);
    if (!result.ok) {
      result.error = (rid == 0) ? "Sphere Region Growing found no regions" :
                                  "Sphere Region Growing produced no mesh";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Sphere Region Growing failed";
  }
  return result;
}

MeshResult points_cylinder_region_growing(const float *in_xyz,
                                          const float *in_nxyz,
                                          int n,
                                          double neighbor_radius,
                                          double max_distance,
                                          double max_angle_deg,
                                          int min_region_size,
                                          double min_radius,
                                          double max_radius,
                                          int segments,
                                          std::vector<int> *out_region)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (out_region) {
    out_region->assign(size_t(std::max(0, n)), -1);
  }
  if (n < 3 || !in_xyz) {
    result.error = "Cylinder Region Growing needs at least 3 points";
    return result;
  }
  if (neighbor_radius <= 0.0) {
    result.error = "Neighbor radius must be > 0";
    return result;
  }
  try {
    std::vector<float> estimated;
    const float *nxyz = ensure_normals(in_xyz, in_nxyz, n, estimated);
    std::vector<PwnRG> pwn = make_pwn(in_xyz, nxyz, n);

    using Neighbor_query = CGAL::Shape_detection::Point_set::Sphere_neighbor_query<Kernel,
                                                                                   PwnRGIt,
                                                                                   PwnRGPointMap>;
    using Region_type = CGAL::Shape_detection::Point_set::Least_squares_cylinder_fit_region<
        Kernel,
        PwnRGIt,
        PwnRGPointMap,
        PwnRGNormalMap>;
    using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

    const double max_r = (max_radius > 0.0) ? max_radius :
                                              std::numeric_limits<double>::max();
    Neighbor_query neighbor_query(
        pwn, CGAL::parameters::sphere_radius(neighbor_radius).point_map(PwnRGPointMap()));
    Region_type region_type(CGAL::parameters::maximum_distance(max_distance)
                                .maximum_angle(max_angle_deg)
                                .minimum_region_size(std::max(3, min_region_size))
                                .minimum_radius(std::max(0.0, min_radius))
                                .maximum_radius(max_r)
                                .point_map(PwnRGPointMap())
                                .normal_map(PwnRGNormalMap()));
    Region_growing region_growing(pwn, neighbor_query, region_type);
    std::vector<typename Region_growing::Primitive_and_region> regions;
    region_growing.detect(std::back_inserter(regions));

    int rid = 0;
    for (const auto &pr : regions) {
      const double radius = CGAL::to_double(pr.first.radius);
      if (radius > 1e-12 && !pr.second.empty()) {
        const auto ends = axis_span(pr.first.axis, pr.second);
        append_cylinder(ends.first, ends.second, radius, segments, rid, true, result);
      }
      fill_region_ids(pwn, pr.second, rid, n, out_region);
      rid++;
    }
    result.alpha_used = double(rid);
    if (!result.ok) {
      result.error = (rid == 0) ? "Cylinder Region Growing found no regions" :
                                  "Cylinder Region Growing produced no mesh";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Cylinder Region Growing failed";
  }
  return result;
}

MeshResult points_circle_region_growing(const float *in_xyz,
                                        const float *in_nxyz,
                                        int n,
                                        double neighbor_radius,
                                        double max_distance,
                                        double max_angle_deg,
                                        int min_region_size,
                                        double min_radius,
                                        double max_radius,
                                        int segments,
                                        std::vector<int> *out_region)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (out_region) {
    out_region->assign(size_t(std::max(0, n)), -1);
  }
  if (n < 3 || !in_xyz) {
    result.error = "Circle Region Growing needs at least 3 points";
    return result;
  }
  if (neighbor_radius <= 0.0) {
    result.error = "Neighbor radius must be > 0";
    return result;
  }
  try {
    std::vector<float> estimated;
    const float *nxyz = ensure_normals(in_xyz, in_nxyz, n, estimated);

    std::vector<Point_3> pts3;
    pts3.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts3.emplace_back(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]);
    }
    Plane_3 plane;
    CGAL::linear_least_squares_fitting_3(
        pts3.begin(), pts3.end(), plane, CGAL::Dimension_tag<0>());
    Vector_3 normal = plane.orthogonal_vector();
    const double nl = std::sqrt(CGAL::to_double(normal.squared_length()));
    if (nl < 1e-18) {
      normal = Vector_3(0, 0, 1);
    }
    else {
      normal = Vector_3(CGAL::to_double(normal.x()) / nl,
                        CGAL::to_double(normal.y()) / nl,
                        CGAL::to_double(normal.z()) / nl);
    }
    Vector_3 u, v;
    orthonormal_frame(normal, u, v);
    const Point_3 origin = plane.point();

    std::vector<Pwn2> pwn;
    pwn.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      const Vector_3 d(pts3[size_t(i)].x() - origin.x(),
                       pts3[size_t(i)].y() - origin.y(),
                       pts3[size_t(i)].z() - origin.z());
      const double x = CGAL::to_double(d.x() * u.x() + d.y() * u.y() + d.z() * u.z());
      const double y = CGAL::to_double(d.x() * v.x() + d.y() * v.y() + d.z() * v.z());
      Vector_3 n3(nxyz[i * 3], nxyz[i * 3 + 1], nxyz[i * 3 + 2]);
      /* Project normal into the PCA plane. */
      const double ndn = CGAL::to_double(n3.x() * normal.x() + n3.y() * normal.y() +
                                         n3.z() * normal.z());
      n3 = Vector_3(CGAL::to_double(n3.x()) - ndn * CGAL::to_double(normal.x()),
                    CGAL::to_double(n3.y()) - ndn * CGAL::to_double(normal.y()),
                    CGAL::to_double(n3.z()) - ndn * CGAL::to_double(normal.z()));
      double nx = CGAL::to_double(n3.x() * u.x() + n3.y() * u.y() + n3.z() * u.z());
      double ny = CGAL::to_double(n3.x() * v.x() + n3.y() * v.y() + n3.z() * v.z());
      if (nx * nx + ny * ny < 1e-18) {
        nx = 1.0;
        ny = 0.0;
      }
      pwn.emplace_back(Point_2(x, y), Vector_2(nx, ny));
    }

    using Neighbor_query = CGAL::Shape_detection::Point_set::Sphere_neighbor_query<Kernel,
                                                                                   Pwn2It,
                                                                                   Pwn2PointMap>;
    using Region_type = CGAL::Shape_detection::Point_set::Least_squares_circle_fit_region<
        Kernel,
        Pwn2It,
        Pwn2PointMap,
        Pwn2NormalMap>;
    using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

    const double max_r = (max_radius > 0.0) ? max_radius :
                                              std::numeric_limits<double>::max();
    Neighbor_query neighbor_query(
        pwn, CGAL::parameters::sphere_radius(neighbor_radius).point_map(Pwn2PointMap()));
    Region_type region_type(CGAL::parameters::maximum_distance(max_distance)
                                .maximum_angle(max_angle_deg)
                                .minimum_region_size(std::max(3, min_region_size))
                                .minimum_radius(std::max(0.0, min_radius))
                                .maximum_radius(max_r)
                                .point_map(Pwn2PointMap())
                                .normal_map(Pwn2NormalMap()));
    Region_growing region_growing(pwn, neighbor_query, region_type);
    std::vector<typename Region_growing::Primitive_and_region> regions;
    region_growing.detect(std::back_inserter(regions));

    int rid = 0;
    for (const auto &pr : regions) {
      const double radius = CGAL::to_double(pr.first.radius);
      if (radius > 1e-12) {
        const double cx = CGAL::to_double(pr.first.center.x());
        const double cy = CGAL::to_double(pr.first.center.y());
        const Point_3 c(CGAL::to_double(origin.x()) + cx * CGAL::to_double(u.x()) +
                            cy * CGAL::to_double(v.x()),
                        CGAL::to_double(origin.y()) + cx * CGAL::to_double(u.y()) +
                            cy * CGAL::to_double(v.y()),
                        CGAL::to_double(origin.z()) + cx * CGAL::to_double(u.z()) +
                            cy * CGAL::to_double(v.z()));
        append_disk(c, u, v, radius, segments, rid, result);
      }
      if (out_region) {
        for (const Pwn2It it : pr.second) {
          const int idx = int(std::distance(pwn.cbegin(), it));
          if (idx >= 0 && idx < n) {
            (*out_region)[size_t(idx)] = rid;
          }
        }
      }
      rid++;
    }
    result.alpha_used = double(rid);
    if (!result.ok) {
      result.error = (rid == 0) ? "Circle Region Growing found no regions" :
                                  "Circle Region Growing produced no mesh";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Circle Region Growing failed";
  }
  return result;
}

WireResult points_line_region_growing(const float *in_xyz,
                                      const float *in_nxyz,
                                      int n,
                                      double neighbor_radius,
                                      double max_distance,
                                      int min_region_size,
                                      std::vector<int> *out_region)
{
  WireResult result;
  CgalThrowGuard guard;
  if (out_region) {
    out_region->assign(size_t(std::max(0, n)), -1);
  }
  if (n < 2 || !in_xyz) {
    result.error = "Line Region Growing needs at least 2 points";
    return result;
  }
  if (neighbor_radius <= 0.0) {
    result.error = "Neighbor radius must be > 0";
    return result;
  }
  try {
    std::vector<float> estimated;
    const float *nxyz = ensure_normals(in_xyz, in_nxyz, n, estimated);
    std::vector<PwnRG> pwn = make_pwn(in_xyz, nxyz, n);

    using Neighbor_query = CGAL::Shape_detection::Point_set::Sphere_neighbor_query<Kernel,
                                                                                   PwnRGIt,
                                                                                   PwnRGPointMap>;
    using Region_type = LineRegion3;
    using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

    Neighbor_query neighbor_query(
        pwn, CGAL::parameters::sphere_radius(neighbor_radius).point_map(PwnRGPointMap()));
    Region_type region_type(max_distance, min_region_size);
    Region_growing region_growing(pwn, neighbor_query, region_type);
    std::vector<typename Region_growing::Primitive_and_region> regions;
    region_growing.detect(std::back_inserter(regions));

    int rid = 0;
    for (const auto &pr : regions) {
      if (pr.second.size() >= 2) {
        const auto ends = axis_span(pr.first, pr.second);
        const int vbase = int(result.positions.size() / 3);
        result.positions.push_back(float(CGAL::to_double(ends.first.x())));
        result.positions.push_back(float(CGAL::to_double(ends.first.y())));
        result.positions.push_back(float(CGAL::to_double(ends.first.z())));
        result.positions.push_back(float(CGAL::to_double(ends.second.x())));
        result.positions.push_back(float(CGAL::to_double(ends.second.y())));
        result.positions.push_back(float(CGAL::to_double(ends.second.z())));
        result.edge_v0.push_back(vbase);
        result.edge_v1.push_back(vbase + 1);
      }
      fill_region_ids(pwn, pr.second, rid, n, out_region);
      rid++;
    }
    result.ok = !result.edge_v0.empty();
    if (!result.ok) {
      result.error = (rid == 0) ? "Line Region Growing found no regions" :
                                  "Line Region Growing produced no segments";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Line Region Growing failed";
  }
  return result;
}

bool simplify_polyline_xyz(const float *xyz,
                           int n,
                           bool closed,
                           double max_distance,
                           bool iterative,
                           std::vector<float> &out_xyz,
                           std::vector<int> &out_src,
                           std::string &error)
{
  out_xyz.clear();
  out_src.clear();
  CgalThrowGuard guard;
  if (!xyz || n < 2) {
    error = "Simplify Polyline 3D needs at least 2 points";
    return false;
  }
  try {
    std::vector<Point_3> input;
    input.reserve(size_t(n) + 1);
    for (int i = 0; i < n; i++) {
      input.push_back(Point_3(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
    }
    if (closed && n >= 3 && input.front() != input.back()) {
      input.push_back(input.front());
    }
    std::vector<Point_3> output;
    const double max_sq = std::max(0.0, max_distance) * std::max(0.0, max_distance);
    if (iterative) {
      PMP::experimental::simplify_polyline(
          input,
          output,
          max_sq,
          CGAL::parameters::algorithm(PMP::experimental::ITERATIVE));
    }
    else {
      PMP::experimental::simplify_polyline(input, output, max_sq);
    }
    if (output.size() < 2) {
      output = input;
    }
    const bool out_closed = (output.size() >= 3 && output.front() == output.back());
    const int nv = out_closed ? int(output.size()) - 1 : int(output.size());
    auto nearest_src = [&](const Point_3 &p) {
      int best = 0;
      double best_d = 1e300;
      for (int i = 0; i < n; i++) {
        const double dx = CGAL::to_double(p.x()) - double(xyz[i * 3]);
        const double dy = CGAL::to_double(p.y()) - double(xyz[i * 3 + 1]);
        const double dz = CGAL::to_double(p.z()) - double(xyz[i * 3 + 2]);
        const double d = dx * dx + dy * dy + dz * dz;
        if (d < best_d) {
          best_d = d;
          best = i;
        }
      }
      return best;
    };
    for (int i = 0; i < nv; i++) {
      const Point_3 &p = output[size_t(i)];
      out_xyz.push_back(float(CGAL::to_double(p.x())));
      out_xyz.push_back(float(CGAL::to_double(p.y())));
      out_xyz.push_back(float(CGAL::to_double(p.z())));
      out_src.push_back(nearest_src(p));
    }
    return out_xyz.size() >= 6;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Simplify Polyline 3D failed";
    return false;
  }
}

static void append_plane_ngon(const std::vector<Point_3> &pts,
                              const Plane_3 &plane,
                              int tag,
                              MeshResult &result)
{
  if (pts.size() < 3) {
    return;
  }
  Vector_3 nrm = plane.orthogonal_vector();
  const double nl = std::sqrt(CGAL::to_double(nrm.squared_length()));
  if (nl < 1e-18) {
    nrm = Vector_3(0, 0, 1);
  }
  else {
    nrm = Vector_3(CGAL::to_double(nrm.x()) / nl,
                   CGAL::to_double(nrm.y()) / nl,
                   CGAL::to_double(nrm.z()) / nl);
  }
  Vector_3 u, v;
  orthonormal_frame(nrm, u, v);
  const Point_3 origin = plane.point();
  std::vector<Point_2> p2;
  p2.reserve(pts.size());
  for (const Point_3 &p : pts) {
    const Vector_3 d(p.x() - origin.x(), p.y() - origin.y(), p.z() - origin.z());
    p2.emplace_back(CGAL::to_double(d.x() * u.x() + d.y() * u.y() + d.z() * u.z()),
                    CGAL::to_double(d.x() * v.x() + d.y() * v.y() + d.z() * v.z()));
  }
  std::vector<Point_2> hull;
  CGAL::convex_hull_2(p2.begin(), p2.end(), std::back_inserter(hull));
  if (hull.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets = {0};
  }
  const int vbase = result.verts_num();
  for (const Point_2 &h : hull) {
    const double hx = CGAL::to_double(h.x());
    const double hy = CGAL::to_double(h.y());
    result.positions.push_back(float(CGAL::to_double(origin.x()) + hx * CGAL::to_double(u.x()) +
                                     hy * CGAL::to_double(v.x())));
    result.positions.push_back(float(CGAL::to_double(origin.y()) + hx * CGAL::to_double(u.y()) +
                                     hy * CGAL::to_double(v.y())));
    result.positions.push_back(float(CGAL::to_double(origin.z()) + hx * CGAL::to_double(u.z()) +
                                     hy * CGAL::to_double(v.z())));
  }
  for (int i = 0; i < int(hull.size()); i++) {
    result.corner_verts.push_back(vbase + i);
  }
  result.face_offsets.push_back(int(result.corner_verts.size()));
  result.face_tag.push_back(tag);
  result.ok = result.faces_num() > 0;
}

MeshResult points_shape_fitting(const float *in_xyz,
                                const float *in_nxyz,
                                int n,
                                int mode,
                                double neighbor_radius,
                                double max_distance,
                                double max_angle_deg,
                                int min_region_size,
                                double min_radius,
                                double max_radius,
                                int segments,
                                std::vector<int> *out_region)
{
  mode = std::clamp(mode, 0, 4);
  if (mode == 1) {
    return points_sphere_region_growing(in_xyz,
                                        in_nxyz,
                                        n,
                                        neighbor_radius,
                                        max_distance,
                                        max_angle_deg,
                                        min_region_size,
                                        min_radius,
                                        max_radius,
                                        segments,
                                        out_region);
  }
  if (mode == 2) {
    return points_cylinder_region_growing(in_xyz,
                                          in_nxyz,
                                          n,
                                          neighbor_radius,
                                          max_distance,
                                          max_angle_deg,
                                          min_region_size,
                                          min_radius,
                                          max_radius,
                                          segments,
                                          out_region);
  }
  if (mode == 3) {
    return points_circle_region_growing(in_xyz,
                                        in_nxyz,
                                        n,
                                        neighbor_radius,
                                        max_distance,
                                        max_angle_deg,
                                        min_region_size,
                                        min_radius,
                                        max_radius,
                                        segments,
                                        out_region);
  }
  if (mode == 4) {
    WireResult wire = points_line_region_growing(
        in_xyz, in_nxyz, n, neighbor_radius, max_distance, min_region_size, out_region);
    MeshResult result;
    result.error = wire.error;
    if (!wire.ok) {
      return result;
    }
    result.positions = std::move(wire.positions);
    /* Represent each segment as a degenerate 2-vert strip isn't valid.
     * Pack as isolated edges via  degenerate triangles skipped by result_to_mesh.
     * Caller uses wire path when mode==line. */
    result.ok = false;
    result.alpha_used = 0.0;
    if (out_region) {
      int mx = -1;
      for (int id : *out_region) {
        mx = std::max(mx, id);
      }
      result.alpha_used = double(mx + 1);
    }
    result.error = "line";
    result.seam_vert_a = std::move(wire.edge_v0);
    result.seam_vert_b = std::move(wire.edge_v1);
    result.ok = !result.seam_vert_a.empty();
    return result;
  }

  /* Plane. */
  MeshResult result;
  CgalThrowGuard guard;
  if (out_region) {
    out_region->assign(size_t(std::max(0, n)), -1);
  }
  if (n < 3 || !in_xyz) {
    result.error = "Point Shape Fitting needs at least 3 points";
    return result;
  }
  if (neighbor_radius <= 0.0) {
    result.error = "Neighbor radius must be > 0";
    return result;
  }
  try {
    std::vector<float> estimated;
    const float *nxyz = ensure_normals(in_xyz, in_nxyz, n, estimated);
    std::vector<PwnRG> pwn = make_pwn(in_xyz, nxyz, n);
    using Neighbor_query = CGAL::Shape_detection::Point_set::Sphere_neighbor_query<Kernel,
                                                                                   PwnRGIt,
                                                                                   PwnRGPointMap>;
    using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<
        Kernel,
        PwnRGIt,
        PwnRGPointMap,
        PwnRGNormalMap>;
    using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;
    Neighbor_query neighbor_query(
        pwn, CGAL::parameters::sphere_radius(neighbor_radius).point_map(PwnRGPointMap()));
    Region_type region_type(CGAL::parameters::maximum_distance(max_distance)
                                .maximum_angle(max_angle_deg)
                                .minimum_region_size(std::max(3, min_region_size))
                                .point_map(PwnRGPointMap())
                                .normal_map(PwnRGNormalMap()));
    Region_growing region_growing(pwn, neighbor_query, region_type);
    std::vector<typename Region_growing::Primitive_and_region> regions;
    region_growing.detect(std::back_inserter(regions));
    int rid = 0;
    for (const auto &pr : regions) {
      std::vector<Point_3> pts;
      pts.reserve(pr.second.size());
      for (const PwnRGIt it : pr.second) {
        pts.push_back(it->first);
      }
      append_plane_ngon(pts, pr.first, rid, result);
      fill_region_ids(pwn, pr.second, rid, n, out_region);
      rid++;
    }
    result.alpha_used = double(rid);
    if (!result.ok) {
      result.error = (rid == 0) ? "Point Shape Fitting found no planar regions" :
                                  "Point Shape Fitting produced no mesh";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Point Shape Fitting failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
