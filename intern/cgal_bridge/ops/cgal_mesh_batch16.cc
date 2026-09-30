/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_fail_guard.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/interpolated_corrected_curvatures.h>
#include <CGAL/Polygon_mesh_processing/manifoldness.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/intersections.h>
#include <CGAL/Intersections_3/Triangle_3_Triangle_3.h>
#include <CGAL/mesh_segmentation.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <variant>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

using K = Kernel;

static bool load_any(const MeshIn &mesh, Surface_mesh &sm, std::string &error, bool stamp = true)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, stamp)) {
    return false;
  }
  return sm.number_of_faces() > 0;
}

static bool load_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error)
{
  if (!load_any(mesh, sm, error, true)) {
    return false;
  }
  triangulate_faces_keep_ids(sm);
  return sm.number_of_faces() > 0;
}

bool mesh_is_manifold(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_any(mesh, sm, error, false)) {
    return false;
  }
  try {
    for (const auto v : sm.vertices()) {
      if (sm.is_removed(v)) {
        continue;
      }
      if (PMP::is_non_manifold_vertex(v, sm)) {
        return false;
      }
    }
    /* Halfedge meshes represent at most two faces per edge; non-manifold
     * configurations show up as non-manifold vertices above. */
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "is_manifold failed";
    return false;
  }
}

int mesh_euler_characteristic(const MeshIn &mesh, std::string &error)
{
  Surface_mesh sm;
  if (!load_any(mesh, sm, error, false)) {
    return 0;
  }
  try {
    const int v = int(sm.number_of_vertices());
    const int e = int(sm.number_of_edges());
    const int f = int(sm.number_of_faces());
    return v - e + f;
  }
  catch (...) {
    error = "euler characteristic failed";
    return 0;
  }
}

bool mesh_face_component_ids(const MeshIn &mesh, std::vector<int> &out_ids, std::string &error)
{
  Surface_mesh sm;
  if (!load_any(mesh, sm, error, true)) {
    return false;
  }
  try {
    auto fcc = sm.add_property_map<Surface_mesh::Face_index, std::size_t>("f:cc", 0).first;
    PMP::connected_components(sm, fcc);
    out_ids.assign(size_t(mesh.faces_num), 0);
    auto fsrc = sm.property_map<Surface_mesh::Face_index, int>(PROP_F_SRC);
    /* Prefer mapping via f:src when present (stable original face index). */
    if (fsrc.has_value()) {
      for (const auto f : sm.faces()) {
        if (sm.is_removed(f)) {
          continue;
        }
        const int src = (*fsrc)[f];
        if (src >= 0 && src < mesh.faces_num) {
          out_ids[size_t(src)] = int(fcc[f]);
        }
      }
    }
    else {
      int i = 0;
      for (const auto f : sm.faces()) {
        if (sm.is_removed(f)) {
          continue;
        }
        if (i < mesh.faces_num) {
          out_ids[size_t(i)] = int(fcc[f]);
        }
        i++;
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "face component ids failed";
    return false;
  }
}

bool mesh_principal_curvatures(const MeshIn &mesh,
                               std::vector<float> &out_kmin,
                               std::vector<float> &out_kmax,
                               std::vector<float> &out_dmin,
                               std::vector<float> &out_dmax,
                               std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  try {
    using PC = PMP::Principal_curvatures_and_directions<K>;
    auto pmap = sm.add_property_map<Surface_mesh::Vertex_index, PC>("v:pc", PC()).first;
    PMP::interpolated_corrected_curvatures(
        sm, CGAL::parameters::vertex_principal_curvatures_and_directions_map(pmap));
    out_kmin.assign(size_t(mesh.verts_num), 0.0f);
    out_kmax.assign(size_t(mesh.verts_num), 0.0f);
    out_dmin.assign(size_t(mesh.verts_num) * 3, 0.0f);
    out_dmax.assign(size_t(mesh.verts_num) * 3, 0.0f);

    auto write_v = [&](int src, const PC &pc) {
      if (src < 0 || src >= mesh.verts_num) {
        return;
      }
      out_kmin[size_t(src)] = float(CGAL::to_double(pc.min_curvature));
      out_kmax[size_t(src)] = float(CGAL::to_double(pc.max_curvature));
      out_dmin[size_t(src) * 3 + 0] = float(CGAL::to_double(pc.min_direction.x()));
      out_dmin[size_t(src) * 3 + 1] = float(CGAL::to_double(pc.min_direction.y()));
      out_dmin[size_t(src) * 3 + 2] = float(CGAL::to_double(pc.min_direction.z()));
      out_dmax[size_t(src) * 3 + 0] = float(CGAL::to_double(pc.max_direction.x()));
      out_dmax[size_t(src) * 3 + 1] = float(CGAL::to_double(pc.max_direction.y()));
      out_dmax[size_t(src) * 3 + 2] = float(CGAL::to_double(pc.max_direction.z()));
    };

    auto vsrc = sm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
    if (vsrc.has_value()) {
      for (const auto v : sm.vertices()) {
        if (sm.is_removed(v)) {
          continue;
        }
        write_v((*vsrc)[v], pmap[v]);
      }
    }
    else {
      int i = 0;
      for (const auto v : sm.vertices()) {
        if (sm.is_removed(v)) {
          continue;
        }
        write_v(i, pmap[v]);
        i++;
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "principal curvatures failed";
    return false;
  }
}

bool mesh_shape_diameter(const MeshIn &mesh, std::vector<float> &out_sdf, std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  if (!CGAL::is_closed(sm)) {
    error = "Shape Diameter needs a closed mesh";
    return false;
  }
  try {
    auto sdf = sm.add_property_map<Surface_mesh::Face_index, double>("f:sdf", 0.0).first;
    CGAL::sdf_values(sm, sdf);

    out_sdf.assign(size_t(mesh.faces_num), 0.0f);
    std::vector<int> counts(size_t(mesh.faces_num), 0);
    auto fsrc = sm.property_map<Surface_mesh::Face_index, int>(PROP_F_SRC);
    if (fsrc.has_value()) {
      for (const auto f : sm.faces()) {
        if (sm.is_removed(f)) {
          continue;
        }
        const int src = (*fsrc)[f];
        if (src >= 0 && src < mesh.faces_num) {
          out_sdf[size_t(src)] += float(sdf[f]);
          counts[size_t(src)]++;
        }
      }
      for (int i = 0; i < mesh.faces_num; i++) {
        if (counts[size_t(i)] > 0) {
          out_sdf[size_t(i)] /= float(counts[size_t(i)]);
        }
      }
    }
    else {
      int i = 0;
      for (const auto f : sm.faces()) {
        if (sm.is_removed(f)) {
          continue;
        }
        if (i < mesh.faces_num) {
          out_sdf[size_t(i)] = float(sdf[f]);
        }
        i++;
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "shape diameter failed";
    return false;
  }
}

bool mesh_mark_self_intersect_faces(const MeshIn &mesh,
                                    std::vector<int8_t> &out_hit,
                                    std::string &error)
{
  Surface_mesh sm;
  if (!load_tri(mesh, sm, error)) {
    return false;
  }
  CgalThrowGuard guard;
  try {
    using Face = Surface_mesh::Face_index;
    using Triangle_3 = K::Triangle_3;
    using Segment_3 = K::Segment_3;
    using Vector_3 = K::Vector_3;

    std::vector<std::pair<Face, Face>> pairs;
    PMP::self_intersections(sm, std::back_inserter(pairs));

    out_hit.assign(size_t(mesh.faces_num), 0);
    auto fsrc = sm.property_map<Face, int>(PROP_F_SRC);

    auto mark = [&](Face f) {
      if (sm.is_removed(f)) {
        return;
      }
      if (fsrc.has_value()) {
        const int src = (*fsrc)[f];
        if (src >= 0 && src < mesh.faces_num) {
          out_hit[size_t(src)] = 1;
        }
        return;
      }
      const int idx = int(f);
      if (idx >= 0 && idx < mesh.faces_num) {
        out_hit[size_t(idx)] = 1;
      }
    };

    auto face_tri = [&](Face f) -> Triangle_3 {
      std::array<Point_3, 3> p;
      int k = 0;
      for (auto h : CGAL::halfedges_around_face(sm.halfedge(f), sm)) {
        if (k < 3) {
          p[size_t(k++)] = sm.point(sm.target(h));
        }
      }
      return Triangle_3(p[0], p[1], p[2]);
    };

    /* Barycentric interior of a triangle (boundary / vertex hits stay out). */
    auto point_in_tri_interior = [](const Point_3 &p,
                                    const Point_3 &a,
                                    const Point_3 &b,
                                    const Point_3 &c) -> bool {
      constexpr double eps = 1.0e-5;
      const Vector_3 v0 = b - a;
      const Vector_3 v1 = c - a;
      const Vector_3 v2 = p - a;
      const double d00 = CGAL::to_double(v0 * v0);
      const double d01 = CGAL::to_double(v0 * v1);
      const double d11 = CGAL::to_double(v1 * v1);
      const double d20 = CGAL::to_double(v2 * v0);
      const double d21 = CGAL::to_double(v2 * v1);
      const double denom = d00 * d11 - d01 * d01;
      if (denom * denom < 1.0e-30 * (d00 * d11 + 1.0e-30)) {
        return false;
      }
      const double v = (d11 * d20 - d01 * d21) / denom;
      const double w = (d00 * d21 - d01 * d20) / denom;
      const double u = 1.0 - v - w;
      return (u > eps) && (v > eps) && (w > eps);
    };

    auto segment_hits_interior = [&](const Segment_3 &s, const Triangle_3 &t) -> bool {
      if (CGAL::to_double(s.squared_length()) <= 1.0e-24) {
        return false;
      }
      const Point_3 a = t.vertex(0);
      const Point_3 b = t.vertex(1);
      const Point_3 c = t.vertex(2);
      const Point_3 src = s.source();
      const Point_3 dst = s.target();
      const Point_3 samples[3] = {
          CGAL::midpoint(src, dst),
          Point_3(src.x() * 0.75 + dst.x() * 0.25,
                  src.y() * 0.75 + dst.y() * 0.25,
                  src.z() * 0.75 + dst.z() * 0.25),
          Point_3(src.x() * 0.25 + dst.x() * 0.75,
                  src.y() * 0.25 + dst.y() * 0.75,
                  src.z() * 0.25 + dst.z() * 0.75),
      };
      for (const Point_3 &q : samples) {
        if (point_in_tri_interior(q, a, b, c)) {
          return true;
        }
      }
      return false;
    };

    auto share_edge = [&](Face a, Face b) -> bool {
      for (auto h : CGAL::halfedges_around_face(sm.halfedge(a), sm)) {
        if (sm.face(sm.opposite(h)) == b) {
          return true;
        }
      }
      return false;
    };

    /* PMP reports vertex-touch, coplanar contact, and one-sided T-junctions.
     * Self Intersect = surfaces actually crossing:
     *  - Segment through BOTH triangle interiors (mutual pierce)
     *  - coplanar area overlap of mesh-adjacent faces (bow-tie / fold-over)
     * Skip Point, boundary-only Segment, and non-adjacent coplanar overlap
     * (two cubes' parallel walls, flush coincident faces). Those are Inside
     * or duplicate-face issues, not a crossing. */
    for (const auto &pr : pairs) {
      if (pr.first == pr.second || sm.is_removed(pr.first) || sm.is_removed(pr.second)) {
        continue;
      }
      const Triangle_3 t1 = face_tri(pr.first);
      const Triangle_3 t2 = face_tri(pr.second);
      if (t1.is_degenerate() || t2.is_degenerate()) {
        continue;
      }
      const auto hit = CGAL::intersection(t1, t2);
      if (!hit) {
        continue;
      }
      if (std::get_if<Point_3>(&*hit)) {
        continue;
      }
      if (const Segment_3 *seg = std::get_if<Segment_3>(&*hit)) {
        if (segment_hits_interior(*seg, t1) && segment_hits_interior(*seg, t2)) {
          mark(pr.first);
          mark(pr.second);
        }
        continue;
      }
      if (std::get_if<Triangle_3>(&*hit) || std::get_if<std::vector<Point_3>>(&*hit)) {
        if (share_edge(pr.first, pr.second)) {
          mark(pr.first);
          mark(pr.second);
        }
      }
    }
    return true;
  }
  catch (const std::exception &e) {
    error = e.what();
    return false;
  }
  catch (...) {
    error = "self intersection mark failed";
    return false;
  }
}

}  // namespace blender::cgal_bridge
