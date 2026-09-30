/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedral_envelope.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/cluster_point_set.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/region_growing.h>
#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

using K = Kernel;
using Point = K::Point_3;
using Vector = K::Vector_3;
using Pwn = std::pair<Point, Vector>;

static std::vector<Point> to_points(const float *xyz, int n)
{
  std::vector<Point> pts;
  pts.reserve(size_t(n));
  for (int i = 0; i < n; i++) {
    pts.emplace_back(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]);
  }
  return pts;
}

bool points_pca_estimate_normals(const float *in_xyz,
                                 int n,
                                 int k,
                                 float *out_nxyz,
                                 std::string &error)
{
  if (!in_xyz || !out_nxyz || n < 3) {
    error = "PCA Estimate Normals needs >=3 points";
    return false;
  }
  k = std::max(6, k);
  try {
    std::vector<Pwn> points;
    points.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      points.emplace_back(Point(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]),
                          Vector(0, 0, 0));
    }
    CGAL::pca_estimate_normals<CGAL::Sequential_tag>(
        points,
        k,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
            .normal_map(CGAL::Second_of_pair_property_map<Pwn>()));
    CGAL::mst_orient_normals(
        points,
        k,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
            .normal_map(CGAL::Second_of_pair_property_map<Pwn>()));
    for (int i = 0; i < n; i++) {
      out_nxyz[i * 3 + 0] = float(CGAL::to_double(points[size_t(i)].second.x()));
      out_nxyz[i * 3 + 1] = float(CGAL::to_double(points[size_t(i)].second.y()));
      out_nxyz[i * 3 + 2] = float(CGAL::to_double(points[size_t(i)].second.z()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "pca_estimate_normals failed";
    return false;
  }
}

bool points_mst_orient_normals(const float *in_xyz,
                               const float *in_nxyz,
                               int n,
                               int k,
                               float *out_nxyz,
                               std::string &error)
{
  if (!in_xyz || !in_nxyz || !out_nxyz || n < 3) {
    error = "MST Orient Normals needs positions and normals";
    return false;
  }
  k = std::max(6, k);
  try {
    std::vector<Pwn> points;
    points.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      Vector nrm(in_nxyz[i * 3], in_nxyz[i * 3 + 1], in_nxyz[i * 3 + 2]);
      if (nrm.squared_length() < 1e-20) {
        nrm = Vector(0, 0, 1);
      }
      points.emplace_back(Point(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]), nrm);
    }
    CGAL::mst_orient_normals(
        points,
        k,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
            .normal_map(CGAL::Second_of_pair_property_map<Pwn>()));
    for (int i = 0; i < n; i++) {
      out_nxyz[i * 3 + 0] = float(CGAL::to_double(points[size_t(i)].second.x()));
      out_nxyz[i * 3 + 1] = float(CGAL::to_double(points[size_t(i)].second.y()));
      out_nxyz[i * 3 + 2] = float(CGAL::to_double(points[size_t(i)].second.z()));
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "mst_orient_normals failed";
    return false;
  }
}

bool points_radial_orient_normals(const float *in_xyz,
                                  const float *in_nxyz,
                                  int n,
                                  float *out_nxyz,
                                  std::string &error)
{
  /* Radial orient: flip n so n · (p - centroid) >= 0 (same idea as CGAL radial_orient_normals). */
  if (!in_xyz || !in_nxyz || !out_nxyz || n < 1) {
    error = "Radial Orient Normals needs positions and normals";
    return false;
  }
  double cx = 0, cy = 0, cz = 0;
  for (int i = 0; i < n; i++) {
    cx += in_xyz[i * 3 + 0];
    cy += in_xyz[i * 3 + 1];
    cz += in_xyz[i * 3 + 2];
  }
  const double inv = 1.0 / double(n);
  cx *= inv;
  cy *= inv;
  cz *= inv;
  for (int i = 0; i < n; i++) {
    double nx = in_nxyz[i * 3 + 0];
    double ny = in_nxyz[i * 3 + 1];
    double nz = in_nxyz[i * 3 + 2];
    const double rx = in_xyz[i * 3 + 0] - cx;
    const double ry = in_xyz[i * 3 + 1] - cy;
    const double rz = in_xyz[i * 3 + 2] - cz;
    if (nx * rx + ny * ry + nz * rz < 0.0) {
      nx = -nx;
      ny = -ny;
      nz = -nz;
    }
    out_nxyz[i * 3 + 0] = float(nx);
    out_nxyz[i * 3 + 1] = float(ny);
    out_nxyz[i * 3 + 2] = float(nz);
  }
  return true;
}

bool points_cluster(const float *in_xyz,
                    int n,
                    double neighbor_radius,
                    std::vector<int> &out_cluster,
                    int &out_cluster_count,
                    std::string &error)
{
  if (!in_xyz || n < 1) {
    error = "Cluster Point Set needs points";
    return false;
  }
  try {
    using Point_set = CGAL::Point_set_3<Point>;
    Point_set point_set;
    point_set.reserve(std::size_t(n));
    for (int i = 0; i < n; i++) {
      point_set.insert(Point(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]));
    }
    Point_set::Property_map<std::size_t> labels =
        point_set.add_property_map<std::size_t>("cluster", 0).first;
    std::size_t n_clusters = 0;
    if (neighbor_radius > 0.0) {
      n_clusters = CGAL::cluster_point_set(
          point_set, labels, CGAL::parameters::neighbor_radius(neighbor_radius));
    }
    else {
      n_clusters = CGAL::cluster_point_set(point_set, labels);
    }
    out_cluster.resize(size_t(n));
    int i = 0;
    for (const auto &idx : point_set) {
      out_cluster[size_t(i++)] = int(labels[idx]);
    }
    out_cluster_count = int(n_clusters);
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "cluster_point_set failed";
    return false;
  }
}

bool mesh_polyhedral_envelope_contains(const MeshIn &mesh,
                                       double epsilon,
                                       const float *query_xyz,
                                       int query_n,
                                       std::vector<int8_t> &out_inside,
                                       std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  if (sm.number_of_faces() == 0 || query_n <= 0 || !query_xyz) {
    error = "Empty mesh or query";
    return false;
  }
  if (!(epsilon > 0.0)) {
    error = "Epsilon must be positive";
    return false;
  }
  try {
    CGAL::Polyhedral_envelope<K> envelope(sm, epsilon);
    out_inside.resize(size_t(query_n));
    for (int i = 0; i < query_n; i++) {
      Point q(query_xyz[i * 3], query_xyz[i * 3 + 1], query_xyz[i * 3 + 2]);
      out_inside[size_t(i)] = envelope(q) ? int8_t(1) : int8_t(0);
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "Polyhedral envelope failed";
    return false;
  }
}

bool mesh_vertex_normals(const MeshIn &mesh, std::vector<float> &out_nxyz, std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  try {
    auto vnormals = sm.add_property_map<Surface_mesh::Vertex_index, Vector>("v:n", Vector(0, 0, 0))
                        .first;
    PMP::compute_vertex_normals(sm, vnormals);
    out_nxyz.assign(size_t(mesh.verts_num) * 3, 0.0f);
    int i = 0;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v) || i >= mesh.verts_num) {
        continue;
      }
      out_nxyz[size_t(i) * 3 + 0] = float(CGAL::to_double(vnormals[v].x()));
      out_nxyz[size_t(i) * 3 + 1] = float(CGAL::to_double(vnormals[v].y()));
      out_nxyz[size_t(i) * 3 + 2] = float(CGAL::to_double(vnormals[v].z()));
      i++;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "compute_vertex_normals failed";
    return false;
  }
}

bool mesh_face_normals(const MeshIn &mesh, std::vector<float> &out_nxyz, std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  try {
    auto fnormals = sm.add_property_map<Surface_mesh::Face_index, Vector>("f:n", Vector(0, 0, 0))
                        .first;
    PMP::compute_face_normals(sm, fnormals);
    out_nxyz.assign(size_t(mesh.faces_num) * 3, 0.0f);
    /* Map by original face id stamped on surface mesh. */
    auto fsp = sm.property_map<Surface_mesh::Face_index, int>("f:src");
    int fi = 0;
    for (const auto f : sm.faces()) {
      if (sm.is_removed(f)) {
        continue;
      }
      int idx = fi;
      if (fsp.has_value() && (*fsp)[f] >= 0 && (*fsp)[f] < mesh.faces_num) {
        idx = (*fsp)[f];
      }
      if (idx >= 0 && idx < mesh.faces_num) {
        out_nxyz[size_t(idx) * 3 + 0] = float(CGAL::to_double(fnormals[f].x()));
        out_nxyz[size_t(idx) * 3 + 1] = float(CGAL::to_double(fnormals[f].y()));
        out_nxyz[size_t(idx) * 3 + 2] = float(CGAL::to_double(fnormals[f].z()));
      }
      fi++;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "compute_face_normals failed";
    return false;
  }
}

bool mesh_face_aspect_ratio(const MeshIn &mesh,
                            std::vector<float> &out_aspect,
                            std::string &error)
{
  /* Aspect = longest edge / shortest edge on each face (1 = equilateral/square-ish). */
  if (mesh.faces_num <= 0 || !mesh.positions || !mesh.corner_verts) {
    error = "Empty mesh";
    return false;
  }
  out_aspect.assign(size_t(mesh.faces_num), 1.0f);
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
    const int n = end - begin;
    if (n < 3) {
      continue;
    }
    double min_l = 1e300, max_l = 0.0;
    for (int c = 0; c < n; c++) {
      const int i0 = mesh.corner_verts[begin + c];
      const int i1 = mesh.corner_verts[begin + ((c + 1) % n)];
      if (i0 < 0 || i1 < 0 || i0 >= mesh.verts_num || i1 >= mesh.verts_num) {
        continue;
      }
      const float *a = mesh.positions + i0 * 3;
      const float *b = mesh.positions + i1 * 3;
      const double dx = double(b[0] - a[0]);
      const double dy = double(b[1] - a[1]);
      const double dz = double(b[2] - a[2]);
      const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
      min_l = std::min(min_l, len);
      max_l = std::max(max_l, len);
    }
    if (min_l > 1e-30 && max_l > 0.0) {
      out_aspect[size_t(f)] = float(max_l / min_l);
    }
    else {
      out_aspect[size_t(f)] = 0.0f;
    }
  }
  return true;
}

bool mesh_vertex_valence(const MeshIn &mesh, std::vector<int> &out_valence, std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  try {
    out_valence.assign(size_t(mesh.verts_num), 0);
    auto vsp = sm.property_map<Surface_mesh::Vertex_index, int>("v:src0");
    int i = 0;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      int idx = i;
      if (vsp.has_value() && (*vsp)[v] >= 0 && (*vsp)[v] < mesh.verts_num) {
        idx = (*vsp)[v];
      }
      if (idx >= 0 && idx < mesh.verts_num) {
        out_valence[size_t(idx)] = int(sm.degree(v));
      }
      i++;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "vertex valence failed";
    return false;
  }
}

bool mesh_mean_edge_length(const MeshIn &mesh, std::vector<float> &out_len, std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  try {
    out_len.assign(size_t(mesh.verts_num), 0.0f);
    std::vector<double> sum(size_t(mesh.verts_num), 0.0);
    std::vector<int> cnt(size_t(mesh.verts_num), 0);
    auto vsp = sm.property_map<Surface_mesh::Vertex_index, int>("v:src0");
    auto map_v = [&](Surface_mesh::Vertex_index v) -> int {
      if (vsp.has_value() && (*vsp)[v] >= 0 && (*vsp)[v] < mesh.verts_num) {
        return (*vsp)[v];
      }
      return -1;
    };
    for (const auto e : sm.edges()) {
      if (sm.is_removed(e)) {
        continue;
      }
      const auto h = sm.halfedge(e);
      const auto va = sm.source(h);
      const auto vb = sm.target(h);
      const double len = std::sqrt(CGAL::to_double(CGAL::squared_distance(sm.point(va), sm.point(vb))));
      const int ia = map_v(va);
      const int ib = map_v(vb);
      if (ia >= 0) {
        sum[size_t(ia)] += len;
        cnt[size_t(ia)]++;
      }
      if (ib >= 0) {
        sum[size_t(ib)] += len;
        cnt[size_t(ib)]++;
      }
    }
    for (int i = 0; i < mesh.verts_num; i++) {
      out_len[size_t(i)] = cnt[size_t(i)] > 0 ? float(sum[size_t(i)] / double(cnt[size_t(i)])) :
                                                0.0f;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "mean edge length failed";
    return false;
  }
}

bool mesh_border_vertices(const MeshIn &mesh, std::vector<int8_t> &out_border, std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  try {
    out_border.assign(size_t(mesh.verts_num), 0);
    auto vsp = sm.property_map<Surface_mesh::Vertex_index, int>("v:src0");
    int i = 0;
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      int idx = i;
      if (vsp.has_value() && (*vsp)[v] >= 0 && (*vsp)[v] < mesh.verts_num) {
        idx = (*vsp)[v];
      }
      if (idx >= 0 && idx < mesh.verts_num) {
        out_border[size_t(idx)] = CGAL::is_border(v, sm) ? int8_t(1) : int8_t(0);
      }
      i++;
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "border vertices failed";
    return false;
  }
}

bool mesh_face_quality(const MeshIn &mesh, std::vector<float> &out_quality, std::string &error)
{
  /* Quality = minimum interior angle in degrees (higher is better equilateral). */
  if (mesh.faces_num <= 0 || !mesh.positions || !mesh.corner_verts) {
    error = "Empty mesh";
    return false;
  }
  out_quality.assign(size_t(mesh.faces_num), 0.0f);
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
    if (end - begin < 3) {
      continue;
    }
    double min_deg = 180.0;
    const int n = end - begin;
    for (int c = 0; c < n; c++) {
      const int i0 = mesh.corner_verts[begin + c];
      const int i1 = mesh.corner_verts[begin + ((c + 1) % n)];
      const int i2 = mesh.corner_verts[begin + ((c + n - 1) % n)];
      if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= mesh.verts_num || i1 >= mesh.verts_num ||
          i2 >= mesh.verts_num)
      {
        continue;
      }
      const double *p = reinterpret_cast<const double *>(nullptr);
      (void)p;
      const float *a = mesh.positions + i0 * 3;
      const float *b = mesh.positions + i1 * 3;
      const float *cpos = mesh.positions + i2 * 3;
      const double bax = a[0] - b[0], bay = a[1] - b[1], baz = a[2] - b[2];
      const double bcx = cpos[0] - b[0], bcy = cpos[1] - b[1], bcz = cpos[2] - b[2];
      const double na = std::sqrt(bax * bax + bay * bay + baz * baz);
      const double nc = std::sqrt(bcx * bcx + bcy * bcy + bcz * bcz);
      if (na < 1e-20 || nc < 1e-20) {
        min_deg = 0.0;
        continue;
      }
      double cosang = (bax * bcx + bay * bcy + baz * bcz) / (na * nc);
      cosang = std::clamp(cosang, -1.0, 1.0);
      const double deg = std::acos(cosang) * 180.0 / 3.14159265358979323846;
      min_deg = std::min(min_deg, deg);
    }
    out_quality[size_t(f)] = float(min_deg);
  }
  return true;
}

bool mesh_region_growing(const MeshIn &mesh,
                         double max_distance,
                         double max_accepted_angle_deg,
                         int min_region_size,
                         std::vector<int> &out_region,
                         int &out_region_count,
                         std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  try {
    auto region_map =
        sm.add_property_map<Surface_mesh::Face_index, std::size_t>("f:region", std::size_t(-1))
            .first;
    max_distance = std::max(0.0, max_distance);
    max_accepted_angle_deg = std::clamp(max_accepted_angle_deg, 0.0, 90.0);
    min_region_size = std::max(1, min_region_size);

    const std::size_t n_regions = PMP::region_growing_of_planes_on_faces(
        sm,
        region_map,
        CGAL::parameters::maximum_distance(max_distance)
            .maximum_angle(max_accepted_angle_deg)
            .minimum_region_size(std::size_t(min_region_size)));

    out_region.assign(size_t(mesh.faces_num), -1);
    auto fsp = sm.property_map<Surface_mesh::Face_index, int>("f:src");
    for (const auto f : sm.faces()) {
      if (sm.is_removed(f)) {
        continue;
      }
      int idx = -1;
      if (fsp.has_value()) {
        idx = (*fsp)[f];
      }
      if (idx >= 0 && idx < mesh.faces_num) {
        const std::size_t rid = region_map[f];
        out_region[size_t(idx)] = (rid == std::size_t(-1)) ? -1 : int(rid);
      }
    }
    out_region_count = int(n_regions);
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "region_growing_of_planes_on_faces failed";
    return false;
  }
}

WireResult mesh_surface_shortest_path(const MeshIn &mesh,
                                      const float source_xyz[3],
                                      const float target_xyz[3])
{
  WireResult result;
  Surface_mesh sm;
  std::string error;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    result.error = error.empty() ? "Shortest path input failed" : error;
    return result;
  }
  triangulate_faces_keep_ids(sm);
  if (sm.number_of_faces() == 0) {
    result.error = "Empty mesh";
    return result;
  }
  try {
    /* Snap source/target to nearest vertices. */
    Surface_mesh::Vertex_index src_v = *sm.vertices().begin();
    Surface_mesh::Vertex_index dst_v = src_v;
    double best_s = 1e300, best_t = 1e300;
    const Point sp(source_xyz[0], source_xyz[1], source_xyz[2]);
    const Point tp(target_xyz[0], target_xyz[1], target_xyz[2]);
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      const double ds = CGAL::to_double(CGAL::squared_distance(sm.point(v), sp));
      const double dt = CGAL::to_double(CGAL::squared_distance(sm.point(v), tp));
      if (ds < best_s) {
        best_s = ds;
        src_v = v;
      }
      if (dt < best_t) {
        best_t = dt;
        dst_v = v;
      }
    }

    using Traits = CGAL::Surface_mesh_shortest_path_traits<K, Surface_mesh>;
    using Shortest_paths = CGAL::Surface_mesh_shortest_path<Traits>;
    Shortest_paths shortest_paths(sm);
    shortest_paths.add_source_point(src_v);

    std::vector<Point> path_points;
    shortest_paths.shortest_path_points_to_source_points(dst_v, std::back_inserter(path_points));
    if (path_points.size() < 2) {
      /* Degenerate: single vertex path. */
      path_points.clear();
      path_points.push_back(sm.point(src_v));
      path_points.push_back(sm.point(dst_v));
    }

    result.positions.reserve(path_points.size() * 3);
    for (const Point &p : path_points) {
      result.positions.push_back(float(CGAL::to_double(p.x())));
      result.positions.push_back(float(CGAL::to_double(p.y())));
      result.positions.push_back(float(CGAL::to_double(p.z())));
    }
    const int n = int(path_points.size());
    result.edge_v0.reserve(size_t(n - 1));
    result.edge_v1.reserve(size_t(n - 1));
    for (int i = 0; i < n - 1; i++) {
      result.edge_v0.push_back(i);
      result.edge_v1.push_back(i + 1);
    }
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "Surface_mesh_shortest_path failed";
  }
  return result;
}

bool mesh_locate_points(const MeshIn &mesh,
                        const float *query_xyz,
                        int query_n,
                        std::vector<int> &out_face,
                        std::vector<float> &out_bary,
                        std::string &error)
{
  Surface_mesh sm;
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  if (sm.number_of_faces() == 0 || !query_xyz || query_n <= 0) {
    error = "Empty mesh or query";
    return false;
  }
  try {
    using Primitive = CGAL::AABB_face_graph_triangle_primitive<Surface_mesh>;
    using Traits = CGAL::AABB_traits_3<K, Primitive>;
    using Tree = CGAL::AABB_tree<Traits>;
    Tree tree(faces(sm).first, faces(sm).second, sm);
    tree.accelerate_distance_queries();

    auto fsp = sm.property_map<Surface_mesh::Face_index, int>("f:src");
    out_face.assign(size_t(query_n), -1);
    out_bary.assign(size_t(query_n) * 3, 0.0f);

    for (int i = 0; i < query_n; i++) {
      Point q(query_xyz[i * 3], query_xyz[i * 3 + 1], query_xyz[i * 3 + 2]);
      auto p_and_id = tree.closest_point_and_primitive(q);
      const Point &cp = p_and_id.first;
      const auto f = p_and_id.second;
      int face_idx = -1;
      if (fsp.has_value() && (*fsp)[f] >= 0) {
        face_idx = (*fsp)[f];
      }
      out_face[size_t(i)] = face_idx;

      /* Barycentric of closest point in the hit triangle. */
      auto h = sm.halfedge(f);
      const Point &p0 = sm.point(sm.source(h));
      const Point &p1 = sm.point(sm.target(h));
      const Point &p2 = sm.point(sm.target(sm.next(h)));
      const Vector v0 = p1 - p0;
      const Vector v1 = p2 - p0;
      const Vector v2 = cp - p0;
      const double d00 = CGAL::to_double(v0 * v0);
      const double d01 = CGAL::to_double(v0 * v1);
      const double d11 = CGAL::to_double(v1 * v1);
      const double d20 = CGAL::to_double(v2 * v0);
      const double d21 = CGAL::to_double(v2 * v1);
      const double denom = d00 * d11 - d01 * d01;
      double v = 0.0, w = 0.0;
      if (std::abs(denom) > 1e-30) {
        v = (d11 * d20 - d01 * d21) / denom;
        w = (d00 * d21 - d01 * d20) / denom;
      }
      double u = 1.0 - v - w;
      out_bary[size_t(i) * 3 + 0] = float(u);
      out_bary[size_t(i) * 3 + 1] = float(v);
      out_bary[size_t(i) * 3 + 2] = float(w);
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "mesh_locate_points failed";
    return false;
  }
}

}  // namespace blender::cgal_bridge
