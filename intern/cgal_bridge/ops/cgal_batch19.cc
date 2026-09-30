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
#include <CGAL/estimate_scale.h>
#include <CGAL/linear_least_squares_fitting_3.h>
#include <CGAL/scanline_orient_normals.h>
#include <CGAL/property_map.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

using K = Kernel;
using Point = K::Point_3;
using Vector = K::Vector_3;
using Plane = K::Plane_3;
using Pwn = std::pair<Point, Vector>;

bool mesh_edge_lengths(const float *positions,
                       int verts_num,
                       const int *edge_verts,
                       int edges_num,
                       std::vector<float> &out_len,
                       std::string &error)
{
  if (!positions || verts_num <= 0 || !edge_verts || edges_num <= 0) {
    error = "Edge Length needs a mesh with edges";
    return false;
  }
  out_len.resize(size_t(edges_num));
  for (int e = 0; e < edges_num; e++) {
    const int v0 = edge_verts[e * 2 + 0];
    const int v1 = edge_verts[e * 2 + 1];
    if (v0 < 0 || v1 < 0 || v0 >= verts_num || v1 >= verts_num) {
      out_len[size_t(e)] = 0.0f;
      continue;
    }
    const float dx = positions[v0 * 3 + 0] - positions[v1 * 3 + 0];
    const float dy = positions[v0 * 3 + 1] - positions[v1 * 3 + 1];
    const float dz = positions[v0 * 3 + 2] - positions[v1 * 3 + 2];
    out_len[size_t(e)] = float(std::sqrt(double(dx * dx + dy * dy + dz * dz)));
  }
  return true;
}

bool mesh_face_perimeters(const MeshIn &mesh, std::vector<float> &out_perim, std::string &error)
{
  if (mesh.faces_num <= 0 || mesh.corners_num < 3 || !mesh.corner_verts || !mesh.face_offsets) {
    error = "Face Perimeter needs a mesh with faces";
    return false;
  }
  out_perim.assign(size_t(mesh.faces_num), 0.0f);
  for (int f = 0; f < mesh.faces_num; f++) {
    const int start = mesh.face_offsets[f];
    const int end = mesh.face_offsets[f + 1];
    if (end - start < 3) {
      continue;
    }
    double perim = 0.0;
    for (int c = start; c < end; c++) {
      const int v0 = mesh.corner_verts[c];
      const int v1 = mesh.corner_verts[(c + 1 < end) ? (c + 1) : start];
      if (v0 < 0 || v1 < 0 || v0 >= mesh.verts_num || v1 >= mesh.verts_num) {
        continue;
      }
      const double dx = double(mesh.positions[v0 * 3 + 0] - mesh.positions[v1 * 3 + 0]);
      const double dy = double(mesh.positions[v0 * 3 + 1] - mesh.positions[v1 * 3 + 1]);
      const double dz = double(mesh.positions[v0 * 3 + 2] - mesh.positions[v1 * 3 + 2]);
      perim += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    out_perim[size_t(f)] = float(perim);
  }
  return true;
}

bool points_centroid(const float *in_xyz, int n, float out_xyz[3], std::string &error)
{
  if (!in_xyz || n < 1 || !out_xyz) {
    error = "Points Centroid needs at least 1 point";
    return false;
  }
  double cx = 0, cy = 0, cz = 0;
  for (int i = 0; i < n; i++) {
    cx += in_xyz[i * 3 + 0];
    cy += in_xyz[i * 3 + 1];
    cz += in_xyz[i * 3 + 2];
  }
  const double inv = 1.0 / double(n);
  out_xyz[0] = float(cx * inv);
  out_xyz[1] = float(cy * inv);
  out_xyz[2] = float(cz * inv);
  return true;
}

bool points_fit_plane(const float *in_xyz,
                      int n,
                      float out_center[3],
                      float out_normal[3],
                      float &out_half_extent,
                      std::string &error)
{
  if (!in_xyz || n < 3 || !out_center || !out_normal) {
    error = "Fit Plane needs at least 3 points";
    return false;
  }
  try {
    std::vector<Point> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]);
    }
    Plane plane;
    Point centroid;
    const double quality = CGAL::linear_least_squares_fitting_3(
        pts.begin(), pts.end(), plane, centroid, CGAL::Dimension_tag<0>());
    (void)quality;
    Vector nrm = plane.orthogonal_vector();
    const double len = std::sqrt(CGAL::to_double(nrm.squared_length()));
    if (!(len > 1e-18)) {
      error = "Fit Plane produced a degenerate normal";
      return false;
    }
    nrm = nrm / len;
    out_center[0] = float(CGAL::to_double(centroid.x()));
    out_center[1] = float(CGAL::to_double(centroid.y()));
    out_center[2] = float(CGAL::to_double(centroid.z()));
    out_normal[0] = float(CGAL::to_double(nrm.x()));
    out_normal[1] = float(CGAL::to_double(nrm.y()));
    out_normal[2] = float(CGAL::to_double(nrm.z()));

    /* Half-extent of projected points for a useful quad size. */
    Vector n = nrm;
    Vector t1;
    if (std::abs(CGAL::to_double(n.z())) < 0.9) {
      t1 = CGAL::cross_product(n, Vector(0, 0, 1));
    }
    else {
      t1 = CGAL::cross_product(n, Vector(0, 1, 0));
    }
    const double t1len = std::sqrt(CGAL::to_double(t1.squared_length()));
    t1 = t1 / t1len;
    Vector t2 = CGAL::cross_product(n, t1);
    double max_r = 0.0;
    for (const Point &p : pts) {
      Vector d(centroid, p);
      const double u = CGAL::to_double(d * t1);
      const double v = CGAL::to_double(d * t2);
      max_r = std::max(max_r, std::sqrt(u * u + v * v));
    }
    out_half_extent = float(std::max(max_r, 1e-4));
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "linear_least_squares_fitting_3 failed";
    return false;
  }
}

bool points_neighbor_scale(const float *in_xyz, int n, int &out_k, std::string &error)
{
  if (!in_xyz || n < 6) {
    error = "Neighbor Scale needs at least 6 points";
    return false;
  }
  try {
    std::vector<Point> pts;
    pts.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      pts.emplace_back(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]);
    }
    const std::size_t k = CGAL::estimate_global_k_neighbor_scale(pts);
    out_k = int(std::max<std::size_t>(1, k));
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "estimate_global_k_neighbor_scale failed";
    return false;
  }
}

bool points_scanline_orient_normals(const float *in_xyz,
                                    const float *in_nxyz,
                                    int n,
                                    float *out_nxyz,
                                    std::string &error)
{
  if (!in_xyz || !in_nxyz || !out_nxyz || n < 3) {
    error = "Scanline Orient Normals needs positions and normals";
    return false;
  }
  try {
    std::vector<Pwn> points;
    points.reserve(size_t(n));
    for (int i = 0; i < n; i++) {
      Vector nrm(in_nxyz[i * 3], in_nxyz[i * 3 + 1], in_nxyz[i * 3 + 2]);
      const double len = std::sqrt(CGAL::to_double(nrm.squared_length()));
      if (!(len > 1e-12)) {
        error = "Scanline Orient Normals requires non-zero input normals";
        return false;
      }
      nrm = nrm / len;
      points.emplace_back(Point(in_xyz[i * 3], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]), nrm);
    }
    CGAL::scanline_orient_normals(
        points,
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
    error = "scanline_orient_normals failed";
    return false;
  }
}

}  // namespace blender::cgal_bridge
