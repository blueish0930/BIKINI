/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Polygon_mesh_processing/angle_and_area_smoothing.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/extrude.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/merge_border_vertices.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/random_perturbation.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/remesh_planar_patches.h>
#include <CGAL/Polygon_mesh_processing/repair_self_intersections.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/tangential_relaxation.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

static bool load_any(const MeshIn &mesh, Surface_mesh &sm, std::string &error, bool tri)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  if (tri) {
    triangulate_faces_keep_ids(sm);
  }
  return sm.number_of_faces() > 0;
}

MeshResult mesh_detect_features(const MeshIn &mesh, double angle_deg)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, false)) {
    result.error = error.empty() ? "Detect features input failed" : error;
    return result;
  }
  try {
    angle_deg = std::clamp(angle_deg, 0.0, 180.0);
    auto ecm = sm.add_property_map<Surface_mesh::Edge_index, bool>(PROP_E_CONSTRAINED, false).first;
    PMP::detect_sharp_edges(sm, angle_deg, ecm);
    /* Output same mesh geometry; seams carry sharp feature edges. */
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL detect_sharp_edges failed";
  }
  return result;
}

MeshResult mesh_angle_area_smooth(const MeshIn &mesh, int iterations, bool use_safety)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, true)) {
    result.error = error.empty() ? "Angle/area smooth input failed" : error;
    return result;
  }
  try {
    iterations = std::max(iterations, 1);
    PMP::angle_and_area_smoothing(
        sm,
        CGAL::parameters::number_of_iterations(static_cast<unsigned int>(iterations))
            .use_safety_constraints(use_safety));
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL angle_and_area_smoothing failed";
  }
  return result;
}

MeshResult mesh_tangential_relaxation(const MeshIn &mesh, int iterations)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, true)) {
    result.error = error.empty() ? "Tangential relaxation input failed" : error;
    return result;
  }
  try {
    iterations = std::max(iterations, 1);
    PMP::tangential_relaxation(
        sm, CGAL::parameters::number_of_iterations(static_cast<unsigned int>(iterations)));
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL tangential_relaxation failed";
  }
  return result;
}

MeshResult mesh_extrude(
    const MeshIn &mesh, float dist, float dx, float dy, float dz, bool along_normals)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, false)) {
    result.error = error.empty() ? "Extrude input failed" : error;
    return result;
  }
  try {
    if (CGAL::is_closed(sm)) {
      result.error = "Extrude needs an open mesh (has a boundary)";
      return result;
    }
    Surface_mesh out;
    if (along_normals) {
      auto vnormals =
          sm.add_property_map<Surface_mesh::Vertex_index, Kernel::Vector_3>("v:n").first;
      PMP::compute_vertex_normals(sm, vnormals);
      auto bot = [&](Surface_mesh::Vertex_index vin, Surface_mesh::Vertex_index vout) {
        out.point(vout) = sm.point(vin);
      };
      auto top = [&](Surface_mesh::Vertex_index vin, Surface_mesh::Vertex_index vout) {
        const Kernel::Vector_3 n = vnormals[vin];
        const double len2 = CGAL::to_double(n.squared_length());
        if (len2 > 1e-30) {
          const double inv = double(dist) / std::sqrt(len2);
          out.point(vout) = sm.point(vin) + n * inv;
        }
        else {
          out.point(vout) = sm.point(vin);
        }
      };
      PMP::extrude_mesh(sm, out, bot, top);
    }
    else {
      double lx = double(dx), ly = double(dy), lz = double(dz);
      double len = std::sqrt(lx * lx + ly * ly + lz * lz);
      if (!(len > 1e-30)) {
        lx = 0;
        ly = 0;
        lz = 1;
        len = 1;
      }
      const double s = double(dist) / len;
      const Kernel::Vector_3 dir(lx * s, ly * s, lz * s);
      PMP::extrude_mesh(sm, out, dir);
    }
    result = surface_mesh_to_result(out);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL extrude_mesh failed";
  }
  return result;
}

MeshResult mesh_smooth_angle_area(const MeshIn &mesh, int iterations, bool use_area, bool use_angle)
{
  /* Alias used by some node names; same as angle_area_smooth with flags. */
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, true)) {
    result.error = error.empty() ? "Smooth angle/area input failed" : error;
    return result;
  }
  try {
    iterations = std::max(iterations, 1);
    PMP::angle_and_area_smoothing(
        sm,
        CGAL::parameters::number_of_iterations(static_cast<unsigned int>(iterations))
            .use_area_smoothing(use_area)
            .use_angle_smoothing(use_angle));
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL angle_and_area_smoothing failed";
  }
  return result;
}

MeshResult mesh_remesh_planar_patches(const MeshIn &mesh, double cos_angle, double max_dist)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, true)) {
    result.error = error.empty() ? "Remesh planar patches input failed" : error;
    return result;
  }
  try {
    cos_angle = std::clamp(cos_angle, 0.0, 1.0);
    max_dist = std::max(max_dist, 0.0);
    Surface_mesh out;
    PMP::remesh_planar_patches(sm,
                               out,
                               CGAL::parameters::cosine_of_maximum_angle(cos_angle)
                                   .maximum_distance(max_dist));
    result = surface_mesh_to_result(out);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL remesh_planar_patches failed";
  }
  return result;
}

MeshResult mesh_random_perturbation(const MeshIn &mesh, double max_move, bool rel)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, false)) {
    result.error = error.empty() ? "Random perturbation input failed" : error;
    return result;
  }
  try {
    max_move = std::max(max_move, 0.0);
    PMP::random_perturbation(sm, max_move, CGAL::parameters::do_project(rel));
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL random_perturbation failed";
  }
  return result;
}

MeshResult mesh_triangulate(const MeshIn &mesh)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, true)) {
    result.error = error.empty() ? "Triangulate input failed" : error;
    return result;
  }
  result = surface_mesh_to_result(sm);
  return result;
}

/**
 * Write oriented polygon soup back as MeshResult, keeping 1:1 vertex indices and
 * original face ids so Blender can preserve attributes via mesh_flip_faces /
 * interpolate maps.
 */
static void soup_to_result_with_maps(const std::vector<Point_3> &points,
                                     const std::vector<std::vector<std::size_t>> &faces,
                                     const std::vector<int> &face_src_ids,
                                     MeshResult &result)
{
  result.positions.clear();
  result.positions.reserve(points.size() * 3);
  for (const Point_3 &p : points) {
    result.positions.push_back(float(CGAL::to_double(p.x())));
    result.positions.push_back(float(CGAL::to_double(p.y())));
    result.positions.push_back(float(CGAL::to_double(p.z())));
  }
  result.vert_src0.resize(points.size());
  result.vert_src1.resize(points.size(), -1);
  result.vert_factor.resize(points.size(), 0.0f);
  for (int i = 0; i < int(points.size()); i++) {
    result.vert_src0[size_t(i)] = i;
  }

  result.corner_verts.clear();
  result.face_offsets.clear();
  result.face_src.clear();
  result.face_src_mesh.clear();
  result.face_offsets.push_back(0);
  for (size_t fi = 0; fi < faces.size(); fi++) {
    const auto &poly = faces[fi];
    if (poly.size() < 3) {
      continue;
    }
    for (const std::size_t vi : poly) {
      result.corner_verts.push_back(int(vi));
    }
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_src.push_back(fi < face_src_ids.size() ? face_src_ids[fi] : int(fi));
    result.face_src_mesh.push_back(0);
  }
  result.ok = !result.corner_verts.empty();
  if (!result.ok) {
    result.error = "Orient produced no faces";
  }
}

/** Signed volume of a closed (or nearly closed) polygon soup; sign = outward-ness. */
static double soup_signed_volume(const std::vector<Point_3> &points,
                                 const std::vector<std::vector<std::size_t>> &faces)
{
  double vol = 0.0;
  for (const auto &poly : faces) {
    if (poly.size() < 3) {
      continue;
    }
    const Point_3 &p0 = points[poly[0]];
    for (size_t i = 1; i + 1 < poly.size(); i++) {
      const Point_3 &p1 = points[poly[i]];
      const Point_3 &p2 = points[poly[i + 1]];
      /* 6 * tet volume of (origin, p0, p1, p2). */
      vol += CGAL::to_double(p0.x()) *
                 (CGAL::to_double(p1.y()) * CGAL::to_double(p2.z()) -
                  CGAL::to_double(p1.z()) * CGAL::to_double(p2.y())) -
             CGAL::to_double(p0.y()) *
                 (CGAL::to_double(p1.x()) * CGAL::to_double(p2.z()) -
                  CGAL::to_double(p1.z()) * CGAL::to_double(p2.x())) +
             CGAL::to_double(p0.z()) *
                 (CGAL::to_double(p1.x()) * CGAL::to_double(p2.y()) -
                  CGAL::to_double(p1.y()) * CGAL::to_double(p2.x()));
    }
  }
  return vol / 6.0;
}

MeshResult mesh_orient_outward(const MeshIn &mesh)
{
  MeshResult result;
  if (!mesh.positions || mesh.verts_num <= 0 || !mesh.corner_verts || mesh.faces_num <= 0) {
    result.error = "Orient input empty";
    return result;
  }

  try {
    /*
     * Soup path only (no halfedge import): random face flips break Surface_mesh
     * add_face. Keep original vertex indices so attributes can be preserved.
     */
    std::vector<Point_3> points;
    points.reserve(size_t(mesh.verts_num));
    for (int i = 0; i < mesh.verts_num; i++) {
      const float *p = mesh.positions + i * 3;
      points.emplace_back(p[0], p[1], p[2]);
    }

    std::vector<std::vector<std::size_t>> faces;
    std::vector<int> face_src_ids;
    faces.reserve(size_t(mesh.faces_num));
    face_src_ids.reserve(size_t(mesh.faces_num));
    for (int f = 0; f < mesh.faces_num; f++) {
      int begin = 0;
      int end = 0;
      if (mesh.face_offsets) {
        begin = mesh.face_offsets[f];
        end = mesh.face_offsets[f + 1];
      }
      else {
        begin = f * 3;
        end = begin + 3;
      }
      if (end - begin < 3 || begin < 0 || end > mesh.corners_num) {
        continue;
      }
      std::vector<std::size_t> poly;
      poly.reserve(size_t(end - begin));
      for (int c = begin; c < end; c++) {
        const int vi = mesh.corner_verts[c];
        if (vi < 0 || vi >= mesh.verts_num) {
          poly.clear();
          break;
        }
        if (!poly.empty() && poly.back() == std::size_t(vi)) {
          continue;
        }
        poly.push_back(std::size_t(vi));
      }
      if (poly.size() >= 3 && poly.front() == poly.back()) {
        poly.pop_back();
      }
      if (poly.size() >= 3) {
        faces.push_back(std::move(poly));
        face_src_ids.push_back(f);
      }
    }
    if (faces.empty()) {
      result.error = "No faces to orient";
      return result;
    }

    try {
      PMP::orient_polygon_soup(points, faces);
    }
    catch (...) {
      /* Keep soup as-is. */
    }

    /* Prefer outward volume sign without rebuilding (preserves 1:1 topology). */
    try {
      if (soup_signed_volume(points, faces) < 0.0) {
        for (auto &poly : faces) {
          std::reverse(poly.begin(), poly.end());
        }
      }
    }
    catch (...) {
    }

    soup_to_result_with_maps(points, faces, face_src_ids, result);
  }
  catch (const std::exception &e) {
    result.error = std::string("CGAL orient failed: ") + e.what();
  }
  catch (...) {
    result.error = "CGAL orient failed (non-manifold / degenerate geometry)";
  }
  return result;
}

bool mesh_does_self_intersect(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_any(mesh, sm, error, true)) {
    return false;
  }
  try {
    return PMP::does_self_intersect(sm);
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "CGAL does_self_intersect failed";
    return false;
  }
}

MeshResult mesh_repair_self_intersections(const MeshIn &mesh)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, true)) {
    result.error = error.empty() ? "Repair self-intersections input failed" : error;
    return result;
  }
  try {
    PMP::experimental::remove_self_intersections(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL remove_self_intersections failed";
  }
  return result;
}

MeshResult mesh_split_long_edges(const MeshIn &mesh, double max_length)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, true)) {
    result.error = error.empty() ? "Split long edges input failed" : error;
    return result;
  }
  try {
    max_length = std::max(max_length, 1e-12);
    PMP::split_long_edges(edges(sm), max_length, sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL split_long_edges failed";
  }
  return result;
}

MeshResult mesh_connected_component_keep(const MeshIn &mesh, int component_index)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, false)) {
    result.error = error.empty() ? "Connected components input failed" : error;
    return result;
  }
  try {
    auto fccmap = sm.add_property_map<Surface_mesh::Face_index, std::size_t>("f:cc", 0).first;
    const std::size_t n = PMP::connected_components(sm, fccmap);
    if (n == 0) {
      result.error = "No connected components";
      return result;
    }
    component_index = std::clamp(component_index, 0, int(n) - 1);
    std::vector<std::size_t> keep = {std::size_t(component_index)};
    PMP::keep_connected_components(sm, keep, fccmap);
    sm.collect_garbage();
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL connected_components failed";
  }
  return result;
}

double mesh_volume(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_any(mesh, sm, error, true)) {
    return 0.0;
  }
  try {
    if (!CGAL::is_closed(sm)) {
      error = "Volume requires a closed mesh";
      return 0.0;
    }
    return CGAL::to_double(PMP::volume(sm));
  }
  catch (const std::exception &e) {
    error = e.what();
    return 0.0;
  }
  catch (...) {
    error = "CGAL volume failed";
    return 0.0;
  }
}

double mesh_area(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_any(mesh, sm, error, false)) {
    return 0.0;
  }
  try {
    return CGAL::to_double(PMP::area(sm));
  }
  catch (const std::exception &e) {
    error = e.what();
    return 0.0;
  }
  catch (...) {
    error = "CGAL area failed";
    return 0.0;
  }
}

MeshResult mesh_merge_border_vertices(const MeshIn &mesh, double dist)
{
  MeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_any(mesh, sm, error, false)) {
    result.error = error.empty() ? "Merge border vertices input failed" : error;
    return result;
  }
  try {
    (void)dist;
    PMP::merge_duplicated_vertices_in_boundary_cycles(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL merge border vertices failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
