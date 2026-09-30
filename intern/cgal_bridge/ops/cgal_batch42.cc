/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Catalog batch 42 (2026-08-19, new only — do not restore deleted ids):
 *  - Largest Empty Sphere 3D
 *  - RANSAC Primitives (shape meshes)
 *  - Voronoi Slice 3D (3D Voronoi ∩ plane)
 *  - Convex Offset 3D
 *  - Volume Components
 *  - Inscribed Sphere 3D (Chebyshev of the convex kernel)
 *  - Min Cylinder 3D
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

#include <CGAL/Convex_hull_3/dual/halfspace_intersection_with_constructions_3.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Min_circle_2.h>
#include <CGAL/Min_circle_2_traits_2.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Regular_triangulation_2.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Voronoi_intersection_2_traits_3.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Random.h>
#include <CGAL/jet_estimate_normals.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/mst_orient_normals.h>

#include <CGAL/Shape_detection/Efficient_RANSAC.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <tuple>
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
using Vector_3 = Kernel::Vector_3;
using Plane_3 = Kernel::Plane_3;
using Line_3 = Kernel::Line_3;
using FT = Kernel::FT;

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
    const float v = float(M_PI) * float(i) / float(rings);
    const float y = std::cos(v);
    const float r = std::sin(v);
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

static void orthonormal_frame(const Vector_3 &axis, Vector_3 &u, Vector_3 &v)
{
  const double ax = CGAL::to_double(axis.x());
  const double ay = CGAL::to_double(axis.y());
  const double az = CGAL::to_double(axis.z());
  Vector_3 tmp = (std::abs(az) < 0.9) ? Vector_3(0, 0, 1) : Vector_3(0, 1, 0);
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

static void append_cone_frustum(const Point_3 &a,
                                double ra,
                                const Point_3 &b,
                                double rb,
                                int segments,
                                int tag,
                                MeshResult &result)
{
  segments = std::clamp(segments, 8, 64);
  Vector_3 axis(b.x() - a.x(), b.y() - a.y(), b.z() - a.z());
  const double len = std::sqrt(CGAL::to_double(axis.squared_length()));
  if (len < 1e-12 || (ra <= 0.0 && rb <= 0.0)) {
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
  const double rad[2] = {std::max(0.0, ra), std::max(0.0, rb)};
  const Point_3 ends[2] = {a, b};
  for (int end = 0; end < 2; end++) {
    for (int j = 0; j < segments; j++) {
      const double ang = 2.0 * M_PI * double(j) / double(segments);
      const double ca = std::cos(ang) * rad[end];
      const double sa = std::sin(ang) * rad[end];
      result.positions.push_back(float(CGAL::to_double(ends[end].x()) +
                                       ca * CGAL::to_double(u.x()) + sa * CGAL::to_double(v.x())));
      result.positions.push_back(float(CGAL::to_double(ends[end].y()) +
                                       ca * CGAL::to_double(u.y()) + sa * CGAL::to_double(v.y())));
      result.positions.push_back(float(CGAL::to_double(ends[end].z()) +
                                       ca * CGAL::to_double(u.z()) + sa * CGAL::to_double(v.z())));
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
  result.ok = result.faces_num() > 0;
}

static void collect_planes_inward(const Surface_mesh &sm, std::vector<Plane_3> &planes)
{
  planes.clear();
  for (const auto f : sm.faces()) {
    auto h = sm.halfedge(f);
    const Point_3 &a = sm.point(sm.source(h));
    const Point_3 &b = sm.point(sm.target(h));
    const Point_3 &c = sm.point(sm.target(sm.next(h)));
    if (CGAL::collinear(a, b, c)) {
      continue;
    }
    planes.emplace_back(a, b, c);
  }
}

static bool strictly_inside_all(const Point_3 &p, const std::vector<Plane_3> &planes)
{
  for (const Plane_3 &pl : planes) {
    if (!pl.has_on_negative_side(p)) {
      return false;
    }
  }
  return true;
}

static Point_3 vertex_centroid(const Surface_mesh &sm)
{
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  for (const auto v : sm.vertices()) {
    const Point_3 &p = sm.point(v);
    sx += CGAL::to_double(p.x());
    sy += CGAL::to_double(p.y());
    sz += CGAL::to_double(p.z());
    n++;
  }
  if (n == 0) {
    return Point_3(0, 0, 0);
  }
  return Point_3(sx / double(n), sy / double(n), sz / double(n));
}

static Point_3 inward_guess(const Surface_mesh &sm)
{
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  for (const auto f : sm.faces()) {
    auto h = sm.halfedge(f);
    const Point_3 &a = sm.point(sm.source(h));
    const Point_3 &b = sm.point(sm.target(h));
    const Point_3 &c = sm.point(sm.target(sm.next(h)));
    if (CGAL::collinear(a, b, c)) {
      continue;
    }
    const auto vec = CGAL::cross_product(b - a, c - a);
    const double len = std::sqrt(CGAL::to_double(vec.squared_length()));
    if (len < 1e-18) {
      continue;
    }
    const double cx = (CGAL::to_double(a.x()) + CGAL::to_double(b.x()) + CGAL::to_double(c.x())) /
                      3.0;
    const double cy = (CGAL::to_double(a.y()) + CGAL::to_double(b.y()) + CGAL::to_double(c.y())) /
                      3.0;
    const double cz = (CGAL::to_double(a.z()) + CGAL::to_double(b.z()) + CGAL::to_double(c.z())) /
                      3.0;
    const double eps = 1e-4 * len;
    sx += cx - eps * CGAL::to_double(vec.x()) / len;
    sy += cy - eps * CGAL::to_double(vec.y()) / len;
    sz += cz - eps * CGAL::to_double(vec.z()) / len;
    n++;
  }
  if (n == 0) {
    return Point_3(0, 0, 0);
  }
  return Point_3(sx / double(n), sy / double(n), sz / double(n));
}

static Point_3 push_inside(Point_3 p, const std::vector<Plane_3> &planes)
{
  for (int pass = 0; pass < 8; pass++) {
    bool moved = false;
    for (const Plane_3 &pl : planes) {
      if (pl.has_on_negative_side(p)) {
        continue;
      }
      const auto n = pl.orthogonal_vector();
      const double n2 = CGAL::to_double(n.squared_length());
      if (n2 < 1e-30) {
        continue;
      }
      const double sd = CGAL::to_double(pl.a() * p.x() + pl.b() * p.y() + pl.c() * p.z() + pl.d());
      const double step = (sd + 1e-8) / n2;
      p = Point_3(CGAL::to_double(p.x()) - step * CGAL::to_double(n.x()),
                  CGAL::to_double(p.y()) - step * CGAL::to_double(n.y()),
                  CGAL::to_double(p.z()) - step * CGAL::to_double(n.z()));
      moved = true;
    }
    if (!moved) {
      break;
    }
  }
  return p;
}

static std::vector<Plane_3> offset_planes(const std::vector<Plane_3> &planes, double signed_off)
{
  /* signed_off > 0 expands the solid (planes move outward). */
  std::vector<Plane_3> out;
  out.reserve(planes.size());
  for (const Plane_3 &pl : planes) {
    const double nx = CGAL::to_double(pl.a());
    const double ny = CGAL::to_double(pl.b());
    const double nz = CGAL::to_double(pl.c());
    const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (nlen < 1e-18) {
      continue;
    }
    const double nd = CGAL::to_double(pl.d()) - signed_off * nlen;
    out.emplace_back(FT(nx), FT(ny), FT(nz), FT(nd));
  }
  return out;
}

static bool halfspace_to_result(const std::vector<Plane_3> &planes,
                                const Point_3 &origin_hint,
                                MeshResult &result,
                                Point_3 *r_centroid = nullptr)
{
  result = MeshResult();
  if (planes.size() < 4) {
    result.error = "Need ≥4 planes";
    return false;
  }
  Point_3 origin = origin_hint;
  int neg = 0;
  for (const Plane_3 &pl : planes) {
    if (pl.has_on_negative_side(origin)) {
      neg++;
    }
  }
  std::vector<Plane_3> oriented = planes;
  if (neg * 2 < int(planes.size())) {
    for (Plane_3 &pl : oriented) {
      pl = pl.opposite();
    }
  }
  origin = push_inside(origin, oriented);
  if (!strictly_inside_all(origin, oriented)) {
    result.error = "Empty kernel (no interior point)";
    return false;
  }
  using Polyhedron = CGAL::Polyhedron_3<Kernel>;
  Polyhedron poly;
  CGAL::halfspace_intersection_with_constructions_3(
      oriented.begin(), oriented.end(), poly, std::make_optional(origin));
  if (poly.size_of_vertices() < 4 || poly.size_of_facets() < 4) {
    result.error = "Halfspace intersection empty";
    return false;
  }
  std::map<const Polyhedron::Vertex *, int> vmap;
  int vi = 0;
  double sx = 0, sy = 0, sz = 0;
  for (auto v = poly.vertices_begin(); v != poly.vertices_end(); ++v) {
    vmap[&*v] = vi++;
    const double x = CGAL::to_double(v->point().x());
    const double y = CGAL::to_double(v->point().y());
    const double z = CGAL::to_double(v->point().z());
    result.positions.push_back(float(x));
    result.positions.push_back(float(y));
    result.positions.push_back(float(z));
    sx += x;
    sy += y;
    sz += z;
  }
  if (r_centroid && vi > 0) {
    *r_centroid = Point_3(sx / double(vi), sy / double(vi), sz / double(vi));
  }
  result.face_offsets = {0};
  for (auto f = poly.facets_begin(); f != poly.facets_end(); ++f) {
    auto h = f->facet_begin();
    const auto h0 = h;
    do {
      result.corner_verts.push_back(vmap[&*(h->vertex())]);
    } while (++h != h0);
    if (int(result.corner_verts.size()) - result.face_offsets.back() < 3) {
      result.corner_verts.resize(size_t(result.face_offsets.back()));
      continue;
    }
    result.face_offsets.push_back(int(result.corner_verts.size()));
  }
  result.ok = result.faces_num() > 0;
  if (!result.ok) {
    result.error = "Halfspace intersection produced no faces";
  }
  return result.ok;
}

static bool load_points(const float *positions, int n, std::vector<Point_3> &pts, std::string &error)
{
  pts.clear();
  if (!positions || n < 1) {
    error = "Need points";
    return false;
  }
  pts.reserve(size_t(n));
  for (int i = 0; i < n; i++) {
    pts.emplace_back(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);
  }
  return true;
}

}  // namespace

MeshResult points_largest_empty_sphere_3(const float *positions, int n, int segments)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<Point_3> pts;
    std::string error;
    if (!load_points(positions, n, pts, error) || pts.size() < 4) {
      result.error = "Largest Empty Sphere 3D needs at least 4 points";
      return result;
    }
    using Dt = CGAL::Delaunay_triangulation_3<Kernel>;
    Dt dt(pts.begin(), pts.end());
    if (dt.dimension() < 3) {
      result.error = "Points are coplanar or degenerate";
      return result;
    }
    Surface_mesh hull;
    CGAL::convex_hull_3(pts.begin(), pts.end(), hull);
    if (hull.number_of_faces() < 4) {
      result.error = "Convex hull is degenerate";
      return result;
    }
    PMP::triangulate_faces(hull);
    CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel> side(hull);

    double best_r2 = -1.0;
    Point_3 best_c(0, 0, 0);
    for (auto cit = dt.finite_cells_begin(); cit != dt.finite_cells_end(); ++cit) {
      const Point_3 c = dt.dual(cit);
      if (side(c) != CGAL::ON_BOUNDED_SIDE) {
        continue;
      }
      const double r2 = CGAL::to_double(CGAL::squared_distance(c, cit->vertex(0)->point()));
      if (r2 > best_r2) {
        best_r2 = r2;
        best_c = c;
      }
    }
    if (best_r2 <= 0.0) {
      result.error = "No empty sphere whose center lies inside the convex hull";
      return result;
    }
    const float r = float(std::sqrt(best_r2));
    result.center[0] = float(CGAL::to_double(best_c.x()));
    result.center[1] = float(CGAL::to_double(best_c.y()));
    result.center[2] = float(CGAL::to_double(best_c.z()));
    result.radius = r;
    append_uv_sphere(result.center[0], result.center[1], result.center[2], r, segments, result);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Largest Empty Sphere 3D failed";
  }
  return result;
}

MeshResult points_ransac_primitives(const float *in_xyz,
                                    const float *in_nxyz,
                                    int n,
                                    double epsilon,
                                    double cluster_epsilon,
                                    double normal_threshold,
                                    int min_points,
                                    double probability,
                                    int shape_flags,
                                    unsigned int random_seed,
                                    int segments)
{
  MeshResult result;
  CgalThrowGuard guard;
  if (n < 10 || !in_xyz) {
    result.error = "RANSAC Primitives needs at least 10 points";
    return result;
  }
  if (shape_flags == 0) {
    shape_flags = 1;
  }
  segments = std::clamp(segments, 8, 64);

  try {
    CGAL::get_default_random() = CGAL::Random(random_seed);

    using IndexedPwn = std::tuple<Point_3, Vector_3, int>;
    std::vector<float> estimated;
    const float *normals = in_nxyz;
    if (!normals) {
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
      normals = estimated.data();
    }

    std::vector<IndexedPwn> points;
    points.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      points.emplace_back(Point_3(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                          Vector_3(normals[i * 3], normals[i * 3 + 1], normals[i * 3 + 2]),
                          i);
    }

    using Point_map = CGAL::Nth_of_tuple_property_map<0, IndexedPwn>;
    using Normal_map = CGAL::Nth_of_tuple_property_map<1, IndexedPwn>;
    using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel,
                                                                   std::vector<IndexedPwn>,
                                                                   Point_map,
                                                                   Normal_map>;
    using Efficient_ransac = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
    using Plane = CGAL::Shape_detection::Plane<Traits>;
    using Sphere = CGAL::Shape_detection::Sphere<Traits>;
    using Cylinder = CGAL::Shape_detection::Cylinder<Traits>;
    using Cone = CGAL::Shape_detection::Cone<Traits>;
    using Torus = CGAL::Shape_detection::Torus<Traits>;

    Efficient_ransac ransac;
    ransac.set_input(points, Point_map(), Normal_map());
    if (shape_flags & 1) {
      ransac.template add_shape_factory<Plane>();
    }
    if (shape_flags & 2) {
      ransac.template add_shape_factory<Sphere>();
    }
    if (shape_flags & 4) {
      ransac.template add_shape_factory<Cylinder>();
    }
    if (shape_flags & 8) {
      ransac.template add_shape_factory<Cone>();
    }
    if (shape_flags & 16) {
      ransac.template add_shape_factory<Torus>();
    }

    Efficient_ransac::Parameters params;
    if (probability > 0.0 && probability < 1.0) {
      params.probability = probability;
    }
    if (min_points > 0) {
      params.min_points = std::size_t(min_points);
    }
    if (epsilon > 0.0) {
      params.epsilon = epsilon;
    }
    if (cluster_epsilon > 0.0) {
      params.cluster_epsilon = cluster_epsilon;
    }
    if (normal_threshold > 0.0 && normal_threshold <= 1.0) {
      params.normal_threshold = normal_threshold;
    }
    ransac.detect(params);

    result.face_offsets = {0};
    int sid = 0;
    for (const auto &shape_ptr : ransac.shapes()) {
      std::vector<Point_3> inliers;
      inliers.reserve(shape_ptr->indices_of_assigned_points().size());
      for (const std::size_t idx : shape_ptr->indices_of_assigned_points()) {
        if (idx < points.size()) {
          inliers.push_back(std::get<0>(points[idx]));
        }
      }
      if (inliers.size() < 3) {
        sid++;
        continue;
      }

      if (auto plane = std::dynamic_pointer_cast<Plane>(shape_ptr)) {
        const Vector_3 n = plane->plane_normal();
        Vector_3 u, v;
        orthonormal_frame(n, u, v);
        std::vector<Point_2> pts2;
        pts2.reserve(inliers.size());
        for (const Point_3 &p : inliers) {
          const Point_3 q = plane->projection(p);
          const Vector_3 d(q.x() - inliers.front().x(),
                           q.y() - inliers.front().y(),
                           q.z() - inliers.front().z());
          pts2.emplace_back(d * u, d * v);
        }
        std::vector<Point_2> hull;
        CGAL::convex_hull_2(pts2.begin(), pts2.end(), std::back_inserter(hull));
        if (hull.size() < 3) {
          sid++;
          continue;
        }
        const Point_3 o = plane->projection(inliers.front());
        const int vbase = result.verts_num();
        for (const Point_2 &q : hull) {
          const double x = CGAL::to_double(o.x() + q.x() * u.x() + q.y() * v.x());
          const double y = CGAL::to_double(o.y() + q.x() * u.y() + q.y() * v.y());
          const double z = CGAL::to_double(o.z() + q.x() * u.z() + q.y() * v.z());
          result.positions.push_back(float(x));
          result.positions.push_back(float(y));
          result.positions.push_back(float(z));
          result.corner_verts.push_back(vbase + int(result.verts_num() - vbase - 1));
        }
        result.face_offsets.push_back(int(result.corner_verts.size()));
        result.face_tag.push_back(sid);
      }
      else if (auto sphere = std::dynamic_pointer_cast<Sphere>(shape_ptr)) {
        const Point_3 c = sphere->center();
        const float r = float(CGAL::to_double(sphere->radius()));
        append_uv_sphere_onto(float(CGAL::to_double(c.x())),
                              float(CGAL::to_double(c.y())),
                              float(CGAL::to_double(c.z())),
                              r,
                              segments,
                              sid,
                              result);
      }
      else if (auto cyl = std::dynamic_pointer_cast<Cylinder>(shape_ptr)) {
        const Line_3 axis = cyl->axis();
        const Vector_3 dir = axis.to_vector();
        const double dlen = std::sqrt(CGAL::to_double(dir.squared_length()));
        if (dlen < 1e-18) {
          sid++;
          continue;
        }
        const Vector_3 unit(CGAL::to_double(dir.x()) / dlen,
                            CGAL::to_double(dir.y()) / dlen,
                            CGAL::to_double(dir.z()) / dlen);
        const Point_3 p0 = axis.point();
        double tmin = 1e300, tmax = -1e300;
        for (const Point_3 &p : inliers) {
          const Vector_3 d(p.x() - p0.x(), p.y() - p0.y(), p.z() - p0.z());
          const double t = CGAL::to_double(d * unit);
          tmin = std::min(tmin, t);
          tmax = std::max(tmax, t);
        }
        if (!(tmax > tmin)) {
          tmax = tmin + 1e-3;
        }
        const Point_3 a(CGAL::to_double(p0.x()) + tmin * CGAL::to_double(unit.x()),
                        CGAL::to_double(p0.y()) + tmin * CGAL::to_double(unit.y()),
                        CGAL::to_double(p0.z()) + tmin * CGAL::to_double(unit.z()));
        const Point_3 b(CGAL::to_double(p0.x()) + tmax * CGAL::to_double(unit.x()),
                        CGAL::to_double(p0.y()) + tmax * CGAL::to_double(unit.y()),
                        CGAL::to_double(p0.z()) + tmax * CGAL::to_double(unit.z()));
        append_cylinder(a, b, CGAL::to_double(cyl->radius()), segments, sid, true, result);
      }
      else if (auto cone = std::dynamic_pointer_cast<Cone>(shape_ptr)) {
        const Point_3 apex = cone->apex();
        Vector_3 axis = cone->axis();
        const double alen = std::sqrt(CGAL::to_double(axis.squared_length()));
        if (alen < 1e-18) {
          sid++;
          continue;
        }
        axis = Vector_3(CGAL::to_double(axis.x()) / alen,
                        CGAL::to_double(axis.y()) / alen,
                        CGAL::to_double(axis.z()) / alen);
        const double ang = CGAL::to_double(cone->angle());
        const double tang = std::tan(ang);
        double tmin = 1e300, tmax = -1e300;
        for (const Point_3 &p : inliers) {
          const Vector_3 d(p.x() - apex.x(), p.y() - apex.y(), p.z() - apex.z());
          const double t = CGAL::to_double(d * axis);
          tmin = std::min(tmin, t);
          tmax = std::max(tmax, t);
        }
        tmin = std::max(tmin, 1e-4);
        if (!(tmax > tmin)) {
          tmax = tmin + 1e-3;
        }
        const Point_3 a(CGAL::to_double(apex.x()) + tmin * CGAL::to_double(axis.x()),
                        CGAL::to_double(apex.y()) + tmin * CGAL::to_double(axis.y()),
                        CGAL::to_double(apex.z()) + tmin * CGAL::to_double(axis.z()));
        const Point_3 b(CGAL::to_double(apex.x()) + tmax * CGAL::to_double(axis.x()),
                        CGAL::to_double(apex.y()) + tmax * CGAL::to_double(axis.y()),
                        CGAL::to_double(apex.z()) + tmax * CGAL::to_double(axis.z()));
        append_cone_frustum(a, tmin * tang, b, tmax * tang, segments, sid, result);
      }
      else if (auto torus = std::dynamic_pointer_cast<Torus>(shape_ptr)) {
        const Point_3 c = torus->center();
        Vector_3 axis = torus->axis();
        const double alen = std::sqrt(CGAL::to_double(axis.squared_length()));
        if (alen < 1e-18) {
          sid++;
          continue;
        }
        axis = Vector_3(CGAL::to_double(axis.x()) / alen,
                        CGAL::to_double(axis.y()) / alen,
                        CGAL::to_double(axis.z()) / alen);
        Vector_3 u, v;
        orthonormal_frame(axis, u, v);
        const double R = CGAL::to_double(torus->major_radius());
        const double r = CGAL::to_double(torus->minor_radius());
        const int rings = segments;
        const int sides = std::max(8, segments / 2);
        if (result.face_offsets.empty()) {
          result.face_offsets = {0};
        }
        const int vbase = result.verts_num();
        for (int i = 0; i < rings; i++) {
          const double a0 = 2.0 * M_PI * double(i) / double(rings);
          const Vector_3 radial(std::cos(a0) * CGAL::to_double(u.x()) +
                                    std::sin(a0) * CGAL::to_double(v.x()),
                                std::cos(a0) * CGAL::to_double(u.y()) +
                                    std::sin(a0) * CGAL::to_double(v.y()),
                                std::cos(a0) * CGAL::to_double(u.z()) +
                                    std::sin(a0) * CGAL::to_double(v.z()));
          const Point_3 ring_c(CGAL::to_double(c.x()) + R * CGAL::to_double(radial.x()),
                               CGAL::to_double(c.y()) + R * CGAL::to_double(radial.y()),
                               CGAL::to_double(c.z()) + R * CGAL::to_double(radial.z()));
          for (int j = 0; j < sides; j++) {
            const double a1 = 2.0 * M_PI * double(j) / double(sides);
            const double cr = std::cos(a1);
            const double sr = std::sin(a1);
            result.positions.push_back(float(CGAL::to_double(ring_c.x()) +
                                             r * (cr * CGAL::to_double(radial.x()) +
                                                  sr * CGAL::to_double(axis.x()))));
            result.positions.push_back(float(CGAL::to_double(ring_c.y()) +
                                             r * (cr * CGAL::to_double(radial.y()) +
                                                  sr * CGAL::to_double(axis.y()))));
            result.positions.push_back(float(CGAL::to_double(ring_c.z()) +
                                             r * (cr * CGAL::to_double(radial.z()) +
                                                  sr * CGAL::to_double(axis.z()))));
          }
        }
        for (int i = 0; i < rings; i++) {
          const int i1 = (i + 1) % rings;
          for (int j = 0; j < sides; j++) {
            const int j1 = (j + 1) % sides;
            const int a = vbase + i * sides + j;
            const int b = vbase + i1 * sides + j;
            const int c0 = vbase + i1 * sides + j1;
            const int d = vbase + i * sides + j1;
            result.corner_verts.push_back(a);
            result.corner_verts.push_back(b);
            result.corner_verts.push_back(c0);
            result.face_offsets.push_back(int(result.corner_verts.size()));
            result.face_tag.push_back(sid);
            result.corner_verts.push_back(a);
            result.corner_verts.push_back(c0);
            result.corner_verts.push_back(d);
            result.face_offsets.push_back(int(result.corner_verts.size()));
            result.face_tag.push_back(sid);
          }
        }
        result.ok = result.faces_num() > 0;
      }
      sid++;
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "RANSAC found no extractable primitives";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "RANSAC Primitives failed";
  }
  return result;
}

MeshResult points_voronoi_slice_3(const float *positions,
                                  int n,
                                  float ox,
                                  float oy,
                                  float oz,
                                  float nx,
                                  float ny,
                                  float nz,
                                  float clip_margin)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<Point_3> pts;
    std::string error;
    if (!load_points(positions, n, pts, error) || pts.size() < 3) {
      result.error = "Voronoi Slice 3D needs at least 3 points";
      return result;
    }
    Vector_3 normal(nx, ny, nz);
    const double nlen = std::sqrt(CGAL::to_double(normal.squared_length()));
    if (nlen < 1e-12) {
      result.error = "Voronoi Slice 3D needs a non-zero plane normal";
      return result;
    }
    normal = Vector_3(CGAL::to_double(normal.x()) / nlen,
                      CGAL::to_double(normal.y()) / nlen,
                      CGAL::to_double(normal.z()) / nlen);
    const Point_3 origin(ox, oy, oz);
    Vector_3 u, v;
    orthonormal_frame(normal, u, v);

    using Gt = CGAL::Voronoi_intersection_2_traits_3<Kernel>;
    using Rt = CGAL::Regular_triangulation_2<Gt>;
    Gt traits(origin, normal);
    Rt rt(traits);
    auto p2wp = rt.geom_traits().construct_weighted_point_2_object();
    for (const Point_3 &p : pts) {
      rt.insert(p2wp(p));
    }
    if (rt.dimension() < 2) {
      result.error = "Voronoi Slice 3D degenerate (sites almost collinear on the plane)";
      return result;
    }

    auto to_uv = [&](const Point_3 &p, double &uu, double &vv) {
      const Vector_3 d(p.x() - origin.x(), p.y() - origin.y(), p.z() - origin.z());
      uu = CGAL::to_double(d * u);
      vv = CGAL::to_double(d * v);
    };
    auto from_uv = [&](double uu, double vv) -> Point_3 {
      return Point_3(CGAL::to_double(origin.x()) + uu * CGAL::to_double(u.x()) +
                         vv * CGAL::to_double(v.x()),
                     CGAL::to_double(origin.y()) + uu * CGAL::to_double(u.y()) +
                         vv * CGAL::to_double(v.y()),
                     CGAL::to_double(origin.z()) + uu * CGAL::to_double(u.z()) +
                         vv * CGAL::to_double(v.z()));
    };

    double minx = 1e300, miny = 1e300, maxx = -1e300, maxy = -1e300;
    for (const Point_3 &p : pts) {
      double uu, vv;
      to_uv(p, uu, vv);
      minx = std::min(minx, uu);
      miny = std::min(miny, vv);
      maxx = std::max(maxx, uu);
      maxy = std::max(maxy, vv);
    }
    const double m = std::max(double(clip_margin), 0.0);
    minx -= m;
    miny -= m;
    maxx += m;
    maxy += m;
    if (!(maxx > minx) || !(maxy > miny)) {
      result.error = "Voronoi Slice 3D degenerate bbox";
      return result;
    }

    auto clip_poly = [&](std::vector<Point_2> poly) -> std::vector<Point_2> {
      const double box[4][3] = {
          {1, 0, -minx}, {-1, 0, maxx}, {0, 1, -miny}, {0, -1, maxy}};
      for (int s = 0; s < 4; s++) {
        std::vector<Point_2> next;
        if (poly.empty()) {
          break;
        }
        auto inside = [&](const Point_2 &q) {
          return box[s][0] * CGAL::to_double(q.x()) + box[s][1] * CGAL::to_double(q.y()) +
                     box[s][2] <=
                 1e-12;
        };
        auto isect = [&](const Point_2 &a, const Point_2 &b) {
          const double da = box[s][0] * CGAL::to_double(a.x()) +
                            box[s][1] * CGAL::to_double(a.y()) + box[s][2];
          const double db = box[s][0] * CGAL::to_double(b.x()) +
                            box[s][1] * CGAL::to_double(b.y()) + box[s][2];
          const double t = da / (da - db);
          return Point_2(CGAL::to_double(a.x()) + t * (CGAL::to_double(b.x()) - CGAL::to_double(a.x())),
                         CGAL::to_double(a.y()) + t * (CGAL::to_double(b.y()) - CGAL::to_double(a.y())));
        };
        Point_2 prev = poly.back();
        bool prev_in = inside(prev);
        for (const Point_2 &cur : poly) {
          const bool cur_in = inside(cur);
          if (cur_in) {
            if (!prev_in) {
              next.push_back(isect(prev, cur));
            }
            next.push_back(cur);
          }
          else if (prev_in) {
            next.push_back(isect(prev, cur));
          }
          prev = cur;
          prev_in = cur_in;
        }
        poly.swap(next);
      }
      return poly;
    };

    result.face_offsets = {0};
    int tag = 0;
    for (auto vit = rt.finite_vertices_begin(); vit != rt.finite_vertices_end(); ++vit) {
      std::vector<Point_2> cand;
      auto fc = rt.incident_faces(vit);
      auto done = fc;
      bool unbounded = false;
      do {
        if (rt.is_infinite(fc)) {
          unbounded = true;
        }
        else {
          const Point_3 c = rt.dual(fc);
          double uu, vv;
          to_uv(c, uu, vv);
          cand.emplace_back(uu, vv);
        }
      } while (++fc != done);
      if (unbounded) {
        /* Close unbounded cells against the plane bbox. */
        const Point_2 corners[4] = {Point_2(minx, miny),
                                    Point_2(maxx, miny),
                                    Point_2(maxx, maxy),
                                    Point_2(minx, maxy)};
        for (const Point_2 &q : corners) {
          cand.push_back(q);
        }
      }
      if (cand.size() < 3) {
        tag++;
        continue;
      }
      std::vector<Point_2> hull;
      CGAL::convex_hull_2(cand.begin(), cand.end(), std::back_inserter(hull));
      hull = clip_poly(std::move(hull));
      if (hull.size() < 3) {
        tag++;
        continue;
      }
      const int vbase = result.verts_num();
      for (const Point_2 &q : hull) {
        const Point_3 p = from_uv(CGAL::to_double(q.x()), CGAL::to_double(q.y()));
        result.positions.push_back(float(CGAL::to_double(p.x())));
        result.positions.push_back(float(CGAL::to_double(p.y())));
        result.positions.push_back(float(CGAL::to_double(p.z())));
        result.corner_verts.push_back(vbase + int(result.verts_num() - vbase - 1));
      }
      result.face_offsets.push_back(int(result.corner_verts.size()));
      result.face_tag.push_back(tag);
      tag++;
    }
    result.ok = result.faces_num() > 0;
    if (!result.ok) {
      result.error = "Voronoi Slice 3D produced no cells";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Voronoi Slice 3D failed";
  }
  return result;
}

MeshResult mesh_convex_offset_3(const MeshIn &mesh, double offset)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    Surface_mesh sm;
    std::string error;
    if (!mesh_in_to_surface_mesh(mesh, sm, error, true) || sm.number_of_faces() == 0) {
      result.error = error.empty() ? "Convex Offset 3D needs a mesh with faces" : error;
      return result;
    }
    triangulate_faces_keep_ids(sm);
    std::vector<Plane_3> planes;
    collect_planes_inward(sm, planes);
    if (planes.size() < 4) {
      result.error = "Convex Offset 3D needs ≥4 non-degenerate faces";
      return result;
    }
    Point_3 origin = inward_guess(sm);
    int neg = 0;
    for (const Plane_3 &pl : planes) {
      if (pl.has_on_negative_side(origin)) {
        neg++;
      }
    }
    if (neg * 2 < int(planes.size())) {
      for (Plane_3 &pl : planes) {
        pl = pl.opposite();
      }
    }
    origin = push_inside(origin, planes);
    std::vector<Plane_3> off = offset_planes(planes, offset);
    if (!halfspace_to_result(off, origin, result)) {
      if (result.error.empty()) {
        result.error = "Convex Offset 3D empty (offset too large inward)";
      }
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Convex Offset 3D failed";
  }
  return result;
}

MeshResult mesh_volume_components(const MeshIn &mesh)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    Surface_mesh sm;
    std::string error;
    if (!mesh_in_to_surface_mesh(mesh, sm, error, true) || sm.number_of_faces() == 0) {
      result.error = error.empty() ? "Volume Components needs a mesh with faces" : error;
      return result;
    }
    triangulate_faces_keep_ids(sm);
    if (!CGAL::is_closed(sm)) {
      result.error = "Volume Components needs a closed triangle mesh";
      return result;
    }
    auto vol_map = sm.add_property_map<Surface_mesh::Face_index, std::size_t>("f:vol", 0).first;
    const std::size_t nvol = PMP::volume_connected_components(sm, vol_map);
    result = surface_mesh_to_result(sm);
    if (!result.ok) {
      result.error = "Volume Components export failed";
      return result;
    }
    result.face_tag.assign(size_t(result.faces_num()), 0);
    int fi = 0;
    for (const auto f : sm.faces()) {
      if (fi < result.faces_num()) {
        result.face_tag[size_t(fi)] = int(vol_map[f]);
      }
      fi++;
    }
    result.alpha_used = double(nvol);
    result.ok = result.faces_num() > 0;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Volume Components failed";
  }
  return result;
}

MeshResult mesh_inscribed_sphere_3(const MeshIn &mesh, int segments)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    Surface_mesh sm;
    std::string error;
    if (!mesh_in_to_surface_mesh(mesh, sm, error, true) || sm.number_of_faces() == 0) {
      result.error = error.empty() ? "Inscribed Sphere 3D needs a mesh with faces" : error;
      return result;
    }
    triangulate_faces_keep_ids(sm);
    std::vector<Plane_3> planes;
    collect_planes_inward(sm, planes);
    if (planes.size() < 4) {
      result.error = "Inscribed Sphere 3D needs ≥4 non-degenerate faces";
      return result;
    }
    /* Convex hull centroid is always inside a full-dimensional hull. */
    Point_3 origin = vertex_centroid(sm);
    int neg = 0;
    for (const Plane_3 &pl : planes) {
      if (pl.has_on_negative_side(origin)) {
        neg++;
      }
    }
    if (neg * 2 < int(planes.size())) {
      for (Plane_3 &pl : planes) {
        pl = pl.opposite();
      }
    }
    origin = push_inside(origin, planes);
    if (!strictly_inside_all(origin, planes)) {
      origin = inward_guess(sm);
      origin = push_inside(origin, planes);
    }
    if (!strictly_inside_all(origin, planes)) {
      result.error = "Inscribed Sphere 3D: points are coplanar or hull has no volume";
      return result;
    }

    double minx = 1e300, miny = 1e300, minz = 1e300;
    double maxx = -1e300, maxy = -1e300, maxz = -1e300;
    for (const auto v : sm.vertices()) {
      const Point_3 &p = sm.point(v);
      minx = std::min(minx, CGAL::to_double(p.x()));
      miny = std::min(miny, CGAL::to_double(p.y()));
      minz = std::min(minz, CGAL::to_double(p.z()));
      maxx = std::max(maxx, CGAL::to_double(p.x()));
      maxy = std::max(maxy, CGAL::to_double(p.y()));
      maxz = std::max(maxz, CGAL::to_double(p.z()));
    }
    double lo = 0.0;
    double hi = 0.5 * std::sqrt((maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny) +
                                (maxz - minz) * (maxz - minz));
    Point_3 best_c = origin;
    double best_r = 0.0;
    for (int it = 0; it < 28; it++) {
      const double mid = 0.5 * (lo + hi);
      std::vector<Plane_3> inset = offset_planes(planes, -mid);
      MeshResult tmp;
      Point_3 c = origin;
      if (halfspace_to_result(inset, origin, tmp, &c)) {
        best_r = mid;
        best_c = c;
        lo = mid;
      }
      else {
        hi = mid;
      }
    }
    if (best_r <= 1e-12) {
      result.error = "Inscribed Sphere 3D radius collapsed (non-convex or degenerate)";
      return result;
    }
    result.center[0] = float(CGAL::to_double(best_c.x()));
    result.center[1] = float(CGAL::to_double(best_c.y()));
    result.center[2] = float(CGAL::to_double(best_c.z()));
    result.radius = float(best_r);
    append_uv_sphere(
        result.center[0], result.center[1], result.center[2], result.radius, segments, result);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Inscribed Sphere 3D failed";
  }
  return result;
}

MeshResult points_min_cylinder_3(const float *positions, int n, int segments)
{
  MeshResult result;
  CgalThrowGuard guard;
  try {
    std::vector<Point_3> pts;
    std::string error;
    if (!load_points(positions, n, pts, error) || pts.size() < 2) {
      result.error = "Min Cylinder 3D needs at least 2 points";
      return result;
    }
    Line_3 line;
    Point_3 centroid;
    CGAL::linear_least_squares_fitting_3(
        pts.begin(), pts.end(), line, centroid, CGAL::Dimension_tag<0>());
    Vector_3 dir = line.to_vector();
    const double dlen = std::sqrt(CGAL::to_double(dir.squared_length()));
    if (dlen < 1e-18) {
      result.error = "Min Cylinder 3D: degenerate axis";
      return result;
    }
    dir = Vector_3(CGAL::to_double(dir.x()) / dlen,
                   CGAL::to_double(dir.y()) / dlen,
                   CGAL::to_double(dir.z()) / dlen);
    Vector_3 u, v;
    orthonormal_frame(dir, u, v);

    using Traits = CGAL::Min_circle_2_traits_2<Kernel>;
    using Min_circle = CGAL::Min_circle_2<Traits>;
    std::vector<Point_2> pts2;
    pts2.reserve(pts.size());
    double tmin = 1e300, tmax = -1e300;
    for (const Point_3 &p : pts) {
      const Vector_3 d(p.x() - centroid.x(), p.y() - centroid.y(), p.z() - centroid.z());
      const double t = CGAL::to_double(d * dir);
      tmin = std::min(tmin, t);
      tmax = std::max(tmax, t);
      pts2.emplace_back(d * u, d * v);
    }
    if (!(tmax > tmin)) {
      tmax = tmin + 1e-4;
    }
    Min_circle mc(pts2.begin(), pts2.end(), true);
    const auto circ = mc.circle();
    const double r = std::sqrt(CGAL::to_double(circ.squared_radius()));
    const Point_2 c2 = circ.center();
    const Point_3 axis_c(CGAL::to_double(centroid.x()) + CGAL::to_double(c2.x()) * CGAL::to_double(u.x()) +
                             CGAL::to_double(c2.y()) * CGAL::to_double(v.x()),
                         CGAL::to_double(centroid.y()) + CGAL::to_double(c2.x()) * CGAL::to_double(u.y()) +
                             CGAL::to_double(c2.y()) * CGAL::to_double(v.y()),
                         CGAL::to_double(centroid.z()) + CGAL::to_double(c2.x()) * CGAL::to_double(u.z()) +
                             CGAL::to_double(c2.y()) * CGAL::to_double(v.z()));
    const Point_3 a(CGAL::to_double(axis_c.x()) + tmin * CGAL::to_double(dir.x()),
                    CGAL::to_double(axis_c.y()) + tmin * CGAL::to_double(dir.y()),
                    CGAL::to_double(axis_c.z()) + tmin * CGAL::to_double(dir.z()));
    const Point_3 b(CGAL::to_double(axis_c.x()) + tmax * CGAL::to_double(dir.x()),
                    CGAL::to_double(axis_c.y()) + tmax * CGAL::to_double(dir.y()),
                    CGAL::to_double(axis_c.z()) + tmax * CGAL::to_double(dir.z()));
    result.center[0] = float(0.5 * (CGAL::to_double(a.x()) + CGAL::to_double(b.x())));
    result.center[1] = float(0.5 * (CGAL::to_double(a.y()) + CGAL::to_double(b.y())));
    result.center[2] = float(0.5 * (CGAL::to_double(a.z()) + CGAL::to_double(b.z())));
    result.radius = float(r);
    result.face_offsets = {0};
    append_cylinder(a, b, r, segments, 0, true, result);
    if (!result.ok) {
      result.error = "Min Cylinder 3D produced no mesh";
    }
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Min Cylinder 3D failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
