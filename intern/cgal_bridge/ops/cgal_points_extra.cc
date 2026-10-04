/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#include <CGAL/Advancing_front_surface_reconstruction.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Min_sphere_of_points_d_traits_3.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_spheres_d_traits_3.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/optimal_bounding_box.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

namespace blender::cgal_bridge {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point = K::Point_3;

/**
 * Post-process raw reconstruction triangles so they behave more like Alpha Wrap:
 * repair soup, remove degenerates/dupes, consistent orientation, compact unused verts,
 * rebuild as manifold mesh when possible, orient closed volumes outward.
 */
static void finalize_reconstruction_triangles(TriangleMeshResult &result)
{
  if (!result.ok || result.positions.empty() || result.corner_verts.size() < 3) {
    return;
  }

  const int verts_num = result.verts_num();
  std::vector<Point> points;
  points.reserve(size_t(verts_num));
  for (int i = 0; i < verts_num; i++) {
    points.emplace_back(result.positions[size_t(i) * 3 + 0],
                        result.positions[size_t(i) * 3 + 1],
                        result.positions[size_t(i) * 3 + 2]);
  }

  std::vector<std::vector<std::size_t>> faces;
  faces.reserve(result.corner_verts.size() / 3);
  std::set<std::array<std::size_t, 3>> unique;
  auto emit_tri = [&](std::size_t a, std::size_t b, std::size_t c) {
    if (a == b || b == c || a == c) {
      return;
    }
    if (a >= points.size() || b >= points.size() || c >= points.size()) {
      return;
    }
    /* Skip near-degenerate triangles. */
    const Point &p0 = points[a];
    const Point &p1 = points[b];
    const Point &p2 = points[c];
    const auto n = CGAL::cross_product(p1 - p0, p2 - p0);
    if (n.squared_length() <= 1e-24) {
      return;
    }
    std::array<std::size_t, 3> key = {a, b, c};
    std::sort(key.begin(), key.end());
    if (!unique.insert(key).second) {
      return;
    }
    faces.push_back({a, b, c});
  };

  if (result.face_offsets.empty()) {
    for (size_t i = 0; i + 2 < result.corner_verts.size(); i += 3) {
      emit_tri(std::size_t(result.corner_verts[i]),
               std::size_t(result.corner_verts[i + 1]),
               std::size_t(result.corner_verts[i + 2]));
    }
  }
  else {
    for (int f = 0; f + 1 < int(result.face_offsets.size()); f++) {
      const int b0 = result.face_offsets[size_t(f)];
      const int b1 = result.face_offsets[size_t(f + 1)];
      if (b1 - b0 < 3) {
        continue;
      }
      const std::size_t i0 = std::size_t(result.corner_verts[size_t(b0)]);
      for (int c = b0 + 1; c + 1 < b1; c++) {
        emit_tri(i0,
                 std::size_t(result.corner_verts[size_t(c)]),
                 std::size_t(result.corner_verts[size_t(c + 1)]));
      }
    }
  }

  if (faces.empty()) {
    result.ok = false;
    result.error = "Reconstruction produced only degenerate faces";
    result.positions.clear();
    result.corner_verts.clear();
    result.face_offsets.clear();
    return;
  }

  try {
    PMP::repair_polygon_soup(points, faces);
  }
  catch (...) {
  }
  try {
    PMP::orient_polygon_soup(points, faces);
  }
  catch (...) {
  }

  /* Compact to used vertices. */
  std::vector<int> old_to_new(points.size(), -1);
  std::vector<Point> compact_pts;
  compact_pts.reserve(points.size());
  for (auto &poly : faces) {
    for (std::size_t &vi : poly) {
      if (vi >= points.size()) {
        continue;
      }
      if (old_to_new[vi] < 0) {
        old_to_new[vi] = int(compact_pts.size());
        compact_pts.push_back(points[vi]);
      }
      vi = std::size_t(old_to_new[vi]);
    }
  }
  points.swap(compact_pts);

  Surface_mesh sm;
  bool rebuilt = false;
  try {
    PMP::polygon_soup_to_polygon_mesh(points, faces, sm);
    rebuilt = sm.number_of_faces() > 0;
  }
  catch (...) {
    rebuilt = false;
  }

  if (rebuilt) {
    try {
      if (CGAL::is_closed(sm)) {
        PMP::orient_to_bound_a_volume(sm);
      }
    }
    catch (...) {
    }
    MeshResult cleaned = surface_mesh_to_result(sm);
    if (cleaned.ok) {
      result = std::move(cleaned);
      return;
    }
  }

  /* Soup fallback: write oriented triangles (no unused verts). */
  result.positions.clear();
  result.corner_verts.clear();
  result.face_offsets.clear();
  result.positions.reserve(points.size() * 3);
  for (const Point &p : points) {
    result.positions.push_back(float(CGAL::to_double(p.x())));
    result.positions.push_back(float(CGAL::to_double(p.y())));
    result.positions.push_back(float(CGAL::to_double(p.z())));
  }
  result.corner_verts.reserve(faces.size() * 3);
  for (const auto &poly : faces) {
    if (poly.size() < 3) {
      continue;
    }
    /* Fan triangulate any n-gons from repair. */
    for (size_t i = 1; i + 1 < poly.size(); i++) {
      result.corner_verts.push_back(int(poly[0]));
      result.corner_verts.push_back(int(poly[i]));
      result.corner_verts.push_back(int(poly[i + 1]));
    }
  }
  result.ok = !result.corner_verts.empty();
  if (!result.ok) {
    result.error = "Reconstruction post-process failed";
  }
}

/**
 * Build a box mesh from CGAL oriented_bounding_box's 8 corners.
 *
 * CGAL order (see Optimal_bounding_box/oriented_bounding_box.h + make_hexahedron):
 *   0 (xmin,ymin,zmin)  1 (xmax,ymin,zmin)  2 (xmax,ymax,zmin)  3 (xmin,ymax,zmin)
 *   4 (xmin,ymax,zmax)  5 (xmin,ymin,zmax)  6 (xmax,ymin,zmax)  7 (xmax,ymax,zmax)
 *
 * Vertical pairs: 0-5, 1-6, 2-7, 3-4  (NOT 0-4).
 * Faces match CGAL::make_hexahedron connectivity (6 quads).
 */
static void append_box_mesh(const std::array<Point, 8> &c, TriangleMeshResult &result)
{
  result.positions.clear();
  result.corner_verts.clear();
  result.face_offsets.clear();
  result.error.clear();
  for (int i = 0; i < 8; i++) {
    result.positions.push_back(float(CGAL::to_double(c[i].x())));
    result.positions.push_back(float(CGAL::to_double(c[i].y())));
    result.positions.push_back(float(CGAL::to_double(c[i].z())));
  }
  /* Same face table as make_hexahedron: bottom (0,3,2,1), top (4,5,6,7), then sides. */
  static const int quads[6][4] = {
      {0, 3, 2, 1}, /* bottom (zmin) */
      {4, 5, 6, 7}, /* top (zmax), CGAL make_hexahedron order */
      {0, 1, 6, 5}, /* ymin side */
      {1, 2, 7, 6}, /* xmax side */
      {2, 3, 4, 7}, /* ymax side */
      {3, 0, 5, 4}, /* xmin side */
  };
  result.face_offsets.push_back(0);
  for (int f = 0; f < 6; f++) {
    for (int k = 0; k < 4; k++) {
      result.corner_verts.push_back(quads[f][k]);
    }
    result.face_offsets.push_back(int(result.corner_verts.size()));
  }
  result.ok = true;
}

static void append_uv_sphere(float cx,
                             float cy,
                             float cz,
                             float radius,
                             int segments,
                             TriangleMeshResult &result)
{
  segments = std::clamp(segments, 8, 64);
  const int rings = segments / 2;
  result.positions.clear();
  result.corner_verts.clear();

  /* poles + rings */
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
  /* top fan */
  for (int j = 0; j < segments; j++) {
    const int a = 1 + j;
    const int b = 1 + (j + 1) % segments;
    result.corner_verts.push_back(0);
    result.corner_verts.push_back(a);
    result.corner_verts.push_back(b);
  }
  /* middle quads as tris */
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
  /* bottom fan */
  const int last_ring = 1 + (rings - 2) * segments;
  for (int j = 0; j < segments; j++) {
    const int a = last_ring + j;
    const int b = last_ring + (j + 1) % segments;
    result.corner_verts.push_back(south);
    result.corner_verts.push_back(b);
    result.corner_verts.push_back(a);
  }
  result.ok = true;
}

TriangleMeshResult alpha_wrap_points(const float *positions,
                                     int points_num,
                                     double alpha,
                                     double offset)
{
  TriangleMeshResult result;
  if (!positions || points_num < 4) {
    result.error = "Alpha Wrap needs at least 4 points";
    return result;
  }
  try {
    std::vector<Point> pts;
    pts.reserve(size_t(points_num));
    for (int i = 0; i < points_num; i++) {
      pts.emplace_back(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
    }
    alpha = std::max(alpha, 1e-8);
    if (!(offset > 0.0)) {
      offset = alpha / 30.0;
    }
    CGAL::Surface_mesh<Point> wrap;
    CGAL::alpha_wrap_3(pts, alpha, offset, wrap);

    result.positions.reserve(wrap.number_of_vertices() * 3);
    std::map<CGAL::Surface_mesh<Point>::Vertex_index, int> v2i;
    for (const auto v : wrap.vertices()) {
      const Point &p = wrap.point(v);
      v2i[v] = int(result.positions.size() / 3);
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
    }
    for (const auto f : wrap.faces()) {
      std::vector<int> fv;
      for (const auto v : vertices_around_face(wrap.halfedge(f), wrap)) {
        fv.push_back(v2i[v]);
      }
      if (fv.size() >= 3) {
        for (size_t i = 1; i + 1 < fv.size(); i++) {
          result.corner_verts.push_back(fv[0]);
          result.corner_verts.push_back(fv[i]);
          result.corner_verts.push_back(fv[i + 1]);
        }
      }
    }
    if (result.corner_verts.empty()) {
      result.error = "Alpha Wrap produced empty mesh";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL alpha_wrap_3 (points) failed";
  }
  return result;
}

TriangleMeshResult advancing_front_surface(const float *positions,
                                           int points_num,
                                           double radius_ratio_bound,
                                           double beta)
{
  TriangleMeshResult result;
  if (!positions || points_num < 4) {
    result.error = "Advancing Front needs at least 4 points";
    return result;
  }
  try {
    std::vector<Point> pts;
    pts.reserve(size_t(points_num));
    for (int i = 0; i < points_num; i++) {
      pts.emplace_back(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
    }
    /* Tighter defaults than raw CGAL: fewer slivers / inverted patches. */
    radius_ratio_bound = std::max(radius_ratio_bound, 1.0);
    if (!(radius_ratio_bound > 1.0)) {
      radius_ratio_bound = 5.0;
    }
    beta = std::clamp(beta, 0.01, 1.5707963267948966); /* ~0..pi/2 */

    std::vector<std::array<std::size_t, 3>> facets;
    CGAL::advancing_front_surface_reconstruction(
        pts.begin(), pts.end(), std::back_inserter(facets), radius_ratio_bound, beta);

    result.positions.resize(size_t(points_num) * 3);
    for (int i = 0; i < points_num; i++) {
      result.positions[size_t(i) * 3 + 0] = positions[i * 3 + 0];
      result.positions[size_t(i) * 3 + 1] = positions[i * 3 + 1];
      result.positions[size_t(i) * 3 + 2] = positions[i * 3 + 2];
    }
    result.corner_verts.reserve(facets.size() * 3);
    for (const auto &f : facets) {
      result.corner_verts.push_back(int(f[0]));
      result.corner_verts.push_back(int(f[1]));
      result.corner_verts.push_back(int(f[2]));
    }
    if (result.corner_verts.empty()) {
      result.error = "Advancing Front produced no facets (try denser points or larger Radius Ratio)";
      return result;
    }
    result.ok = true;
    /* Orient, dedupe, drop unused points, prefer closed outward volume. */
    finalize_reconstruction_triangles(result);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL advancing_front_surface_reconstruction failed";
  }
  return result;
}

TriangleMeshResult delaunay_3d(const float *positions, int points_num, bool separate_tets)
{
  TriangleMeshResult result;
  if (!positions || points_num < 4) {
    result.error = "Delaunay 3D needs at least 4 points";
    return result;
  }
  try {
    using Dt = CGAL::Delaunay_triangulation_3<K>;
    std::vector<Point> pts;
    pts.reserve(size_t(points_num));
    for (int i = 0; i < points_num; i++) {
      pts.emplace_back(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
    }
    Dt dt(pts.begin(), pts.end());
    if (dt.dimension() < 3) {
      result.error = "Points are coplanar or degenerate for 3D Delaunay";
      return result;
    }

    if (separate_tets) {
      /* Each finite tet is an independent shell: 4 unique verts + 4 triangles.
       * Vertices are not shared across tets (can explode/separate in GN). */
      const std::size_t ncells = dt.number_of_finite_cells();
      result.positions.reserve(ncells * 4 * 3);
      result.corner_verts.reserve(ncells * 4 * 3);
      for (auto cit = dt.finite_cells_begin(); cit != dt.finite_cells_end(); ++cit) {
        const int base = int(result.positions.size() / 3);
        for (int v = 0; v < 4; v++) {
          const Point &p = cit->vertex(v)->point();
          result.positions.push_back(float(CGAL::to_double(p.x())));
          result.positions.push_back(float(CGAL::to_double(p.y())));
          result.positions.push_back(float(CGAL::to_double(p.z())));
        }
        /* 4 faces opposite each vertex; order (i+1,i+2,i+3) keeps outward-ish winding. */
        for (int i = 0; i < 4; i++) {
          result.corner_verts.push_back(base + ((i + 1) & 3));
          result.corner_verts.push_back(base + ((i + 2) & 3));
          result.corner_verts.push_back(base + ((i + 3) & 3));
        }
      }
    }
    else {
      /* Welded mesh: one vertex per unique Delaunay vertex (shared across tets). */
      std::map<Dt::Vertex_handle, int> v2i;
      result.positions.reserve(size_t(dt.number_of_vertices()) * 3);
      for (auto vit = dt.finite_vertices_begin(); vit != dt.finite_vertices_end(); ++vit) {
        const Point &p = vit->point();
        v2i[vit] = int(result.positions.size() / 3);
        result.positions.push_back(float(CGAL::to_double(p.x())));
        result.positions.push_back(float(CGAL::to_double(p.y())));
        result.positions.push_back(float(CGAL::to_double(p.z())));
      }
      for (auto cit = dt.finite_cells_begin(); cit != dt.finite_cells_end(); ++cit) {
        for (int i = 0; i < 4; i++) {
          const int i0 = v2i[cit->vertex((i + 1) & 3)];
          const int i1 = v2i[cit->vertex((i + 2) & 3)];
          const int i2 = v2i[cit->vertex((i + 3) & 3)];
          result.corner_verts.push_back(i0);
          result.corner_verts.push_back(i1);
          result.corner_verts.push_back(i2);
        }
      }
    }
    if (result.corner_verts.empty()) {
      result.error = "Delaunay 3D produced no cells";
      return result;
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL Delaunay_triangulation_3 failed";
  }
  return result;
}

TriangleMeshResult min_sphere(const float *positions, int points_num, int sphere_segments)
{
  TriangleMeshResult result;
  if (!positions || points_num < 2) {
    result.error = "Min Sphere needs at least 2 points";
    return result;
  }
  try {
    typedef CGAL::Min_sphere_of_points_d_traits_3<K, double> Traits;
    typedef CGAL::Min_sphere_of_spheres_d<Traits> Min_sphere;
    std::vector<Point> pts;
    pts.reserve(size_t(points_num));
    for (int i = 0; i < points_num; i++) {
      pts.emplace_back(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
    }
    Min_sphere ms(pts.begin(), pts.end());
    auto c_it = ms.center_cartesian_begin();
    result.center[0] = float(*c_it++);
    result.center[1] = float(*c_it++);
    result.center[2] = float(*c_it);
    result.radius = float(ms.radius());
    append_uv_sphere(result.center[0],
                     result.center[1],
                     result.center[2],
                     result.radius,
                     sphere_segments,
                     result);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL Min_sphere failed";
  }
  return result;
}

TriangleMeshResult min_sphere_of_spheres(const float *positions,
                                         const float *radii,
                                         int n,
                                         int sphere_segments)
{
  TriangleMeshResult result;
  if (!positions || n < 1) {
    result.error = "Min Sphere of Spheres needs at least 1 ball";
    return result;
  }
  try {
    typedef CGAL::Min_sphere_of_spheres_d_traits_3<K, double> Traits;
    typedef CGAL::Min_sphere_of_spheres_d<Traits> Min_sphere;
    typedef Traits::Sphere Sphere;
    std::vector<Sphere> spheres;
    spheres.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      const double r = double(std::max(radii ? radii[i] : 0.0f, 0.0f));
      spheres.emplace_back(Point(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]), r);
    }
    Min_sphere ms(spheres.begin(), spheres.end());
    auto c_it = ms.center_cartesian_begin();
    result.center[0] = float(*c_it++);
    result.center[1] = float(*c_it++);
    result.center[2] = float(*c_it);
    result.radius = float(ms.radius());
    append_uv_sphere(result.center[0],
                     result.center[1],
                     result.center[2],
                     result.radius,
                     sphere_segments,
                     result);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL Min_sphere_of_spheres failed";
  }
  return result;
}

TriangleMeshResult optimal_bbox(const float *positions, int points_num)
{
  TriangleMeshResult result;
  if (!positions || points_num < 2) {
    result.error = "Optimal BBox needs at least 2 points";
    return result;
  }
  try {
    std::vector<Point> pts;
    pts.reserve(size_t(points_num));
    for (int i = 0; i < points_num; i++) {
      pts.emplace_back(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
    }
    /*
     * Corners are in make_hexahedron order (see oriented_bounding_box.h).
     * Prefer CGAL::make_hexahedron for exact face connectivity, then export quads.
     */
    std::array<Point, 8> obb_points;
    CGAL::oriented_bounding_box(pts, obb_points);
    Surface_mesh box_sm;
    CGAL::make_hexahedron(obb_points[0],
                          obb_points[1],
                          obb_points[2],
                          obb_points[3],
                          obb_points[4],
                          obb_points[5],
                          obb_points[6],
                          obb_points[7],
                          box_sm);
    result = surface_mesh_to_result(box_sm);
    if (result.ok && result.faces_num() == 6 && result.verts_num() == 8) {
      return result;
    }
    /* Explicit quad table matching the same corner order (stable fallback). */
    append_box_mesh(obb_points, result);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL oriented_bounding_box failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
