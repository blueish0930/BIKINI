/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Eigen_solver_traits.h>
#include <CGAL/Eigen_sparse_matrix.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/connected_components.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/fair.h>
#include <CGAL/Polygon_mesh_processing/internal/fair_impl.h>
#include <CGAL/Weights/uniform_weights.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Subdivision_method_3.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

static bool load_sm(const MeshIn &mesh, Surface_mesh &sm, std::string &error, bool need_triangles)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  if (need_triangles) {
    triangulate_faces_keep_ids(sm);
  }
  return sm.number_of_faces() > 0;
}

TriangleMeshResult mesh_fair(const MeshIn &mesh, int continuity)
{
  TriangleMeshResult result;
  Surface_mesh sm;
  std::string error;
  /*
   * Keep original polygons (no triangulate). Cotangent-on-kinked-tris is what
   * made C1 look like a slightly-better soap film. The geometry-module Fair
   * node uses BKE k-harmonic + original k-ring pins; this path is a uniform-weight
   * fallback for any remaining bridge callers.
   */
  if (!load_sm(mesh, sm, error, false)) {
    result.error = error.empty() ? "Fair input failed" : error;
    return result;
  }
  try {
    continuity = std::clamp(continuity, 0, 2);
    std::vector<Surface_mesh::Vertex_index> region;
    for (const auto v : sm.vertices()) {
      if (!CGAL::is_border(v, sm)) {
        region.push_back(v);
      }
    }
    if (region.empty()) {
      result.error = "Fair needs interior vertices (all vertices are on boundary)";
      return result;
    }
    if (region.size() >= sm.number_of_vertices()) {
      result.error = "Fair needs some fixed (boundary) vertices; mesh is closed with no border";
      /* Still try with a random fixed set: skip (fair all shrinks to origin). */
      return result;
    }

    /* Public PMP::fair requires triangles; call the impl on the polygon mesh. */
    using UniformW = CGAL::Weights::Uniform_weight<Surface_mesh>;
    using Solver = CGAL::Eigen_solver_traits<Eigen::SparseLU<
        CGAL::Eigen_sparse_matrix<double>::EigenType, Eigen::COLAMDOrdering<int>>>;
    auto vpmap = sm.points();
    UniformW weights;
    Solver solver;
    CGAL::Polygon_mesh_processing::internal::Fair_Polyhedron_3<Surface_mesh,
                                                               Solver,
                                                               UniformW,
                                                               decltype(vpmap)>
        fair_fn(sm, vpmap, weights);
    const bool ok = fair_fn.fair(region, solver, static_cast<unsigned int>(continuity));
    if (!ok) {
      result.error = "CGAL fair failed (insufficient boundary constraints)";
      return result;
    }
    /*
     * Fair only moves vertex positions. Always restore input topology so
     * attributes are preserved via positions_only path (not rebuilt mesh).
     * triangulate_faces does not add vertices, so counts must match.
     */
    if (int(sm.number_of_vertices()) != mesh.verts_num) {
      result.error = "Fair changed vertex count unexpectedly";
      return result;
    }
    if (!surface_mesh_positions_to_buffer(sm, result.positions)) {
      result.error = "Fair produced empty positions";
      return result;
    }
    const int cn = mesh.corners_num > 0 ? mesh.corners_num :
                                          (mesh.face_offsets ? 0 : mesh.faces_num * 3);
    if (cn > 0) {
      result.corner_verts.assign(mesh.corner_verts, mesh.corner_verts + cn);
    }
    if (mesh.face_offsets) {
      result.face_offsets.assign(mesh.face_offsets, mesh.face_offsets + mesh.faces_num + 1);
    }
    /* Identity maps so any attr path can still gather 1:1. */
    result.vert_src0.resize(size_t(mesh.verts_num));
    result.vert_src1.assign(size_t(mesh.verts_num), -1);
    result.vert_factor.assign(size_t(mesh.verts_num), 0.0f);
    for (int i = 0; i < mesh.verts_num; i++) {
      result.vert_src0[size_t(i)] = i;
    }
    result.face_src.resize(size_t(mesh.faces_num));
    for (int i = 0; i < mesh.faces_num; i++) {
      result.face_src[size_t(i)] = i;
    }
    result.positions_only = true;
    result.ok = true;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL fair failed";
  }
  return result;
}

TriangleMeshResult mesh_refine(const MeshIn &mesh, double density_factor)
{
  /**
   * Smooth densification: CGAL PMP::refine only mid-splits edges and creates
   * skinny spikes on curved meshes. Use isotropic remeshing with target edge
   * length = mean_edge / density instead.
   */
  TriangleMeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_sm(mesh, sm, error, true)) {
    result.error = error.empty() ? "Refine input failed" : error;
    return result;
  }
  try {
    density_factor = std::clamp(density_factor, 1.0, 16.0);
    double edge_sum = 0.0;
    int edge_n = 0;
    for (const auto e : sm.edges()) {
      const auto h = sm.halfedge(e);
      const Point_3 &a = sm.point(sm.source(h));
      const Point_3 &b = sm.point(sm.target(h));
      const double dx = CGAL::to_double(a.x() - b.x());
      const double dy = CGAL::to_double(a.y() - b.y());
      const double dz = CGAL::to_double(a.z() - b.z());
      edge_sum += std::sqrt(dx * dx + dy * dy + dz * dz);
      edge_n++;
    }
    const double mean_edge = (edge_n > 0) ? (edge_sum / double(edge_n)) : 0.1;
    const double target = std::max(mean_edge / density_factor, mean_edge * 0.05);
    const int iterations = 5;
    PMP::isotropic_remeshing(faces(sm),
                             target,
                             sm,
                             CGAL::parameters::number_of_iterations(iterations)
                                 .protect_constraints(false));
    /* Remesh rebuilds topology; no reliable parent maps — leave maps empty. */
    result = surface_mesh_to_result(sm);
    result.face_src.clear();
    result.face_src_mesh.clear();
    result.vert_src0.clear();
    result.vert_src1.clear();
    result.vert_factor.clear();
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL refine (isotropic densify) failed";
  }
  return result;
}

TriangleMeshResult mesh_clip_plane(const MeshIn &mesh,
                                   float px,
                                   float py,
                                   float pz,
                                   float nx,
                                   float ny,
                                   float nz)
{
  TriangleMeshResult result;
  Surface_mesh sm;
  std::string error;
  /* Clip works on polygon meshes; keep n-gons when possible. */
  if (!load_sm(mesh, sm, error, false)) {
    result.error = error.empty() ? "Clip input failed" : error;
    return result;
  }
  try {
    double len = std::sqrt(double(nx) * nx + double(ny) * ny + double(nz) * nz);
    if (!(len > 1e-20)) {
      result.error = "Clip plane normal is zero";
      return result;
    }
    nx = float(nx / len);
    ny = float(ny / len);
    nz = float(nz / len);
    /* CGAL clip keeps the *negative* side of the plane (opposite its normal).
     * Flip normal so we keep the half-space in the direction of the user normal. */
    Kernel::Plane_3 clip_plane(Kernel::Point_3(px, py, pz), Kernel::Vector_3(-nx, -ny, -nz));
    const bool ok = PMP::clip(sm, clip_plane, CGAL::parameters::clip_volume(false));
    if (!ok) {
      result.error = "CGAL clip failed";
      return result;
    }
    fill_missing_vert_maps_from_edges(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL clip failed";
  }
  return result;
}

TriangleMeshResult mesh_subdivision(const MeshIn &mesh, int mode, int steps)
{
  TriangleMeshResult result;
  Surface_mesh sm;
  std::string error;
  /* Loop / Sqrt3 require triangles; Catmull-Clark / DooSabin keep general polygons. */
  const bool need_tri = (mode == 0 || mode == 3);
  if (!load_sm(mesh, sm, error, need_tri)) {
    result.error = error.empty() ? "Subdivision input failed" : error;
    return result;
  }
  try {
    steps = std::clamp(steps, 1, 5);
    const int orig_verts = mesh.verts_num;
    const int orig_faces = mesh.faces_num;
    switch (mode) {
      case 1:
        CGAL::Subdivision_method_3::CatmullClark_subdivision(
            sm, CGAL::parameters::number_of_iterations(steps));
        break;
      case 2:
        CGAL::Subdivision_method_3::DooSabin_subdivision(
            sm, CGAL::parameters::number_of_iterations(steps));
        break;
      case 3:
        CGAL::Subdivision_method_3::Sqrt3_subdivision(
            sm, CGAL::parameters::number_of_iterations(steps));
        break;
      default:
        CGAL::Subdivision_method_3::Loop_subdivision(
            sm, CGAL::parameters::number_of_iterations(steps));
        break;
    }
    /* Topology-aware vert maps for point attribute interpolation. */
    assign_subdiv_vert_maps(sm, sm, orig_verts, orig_faces);
    /* Face attrs: each output face inherits from the nearest original face by
     * corner-vertex ownership (most frequent original face among corners). */
    {
      auto fsp = sm.property_map<Surface_mesh::Face_index, int>(PROP_F_SRC);
      auto v0p = sm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
      if (fsp.has_value() && v0p.has_value()) {
        for (const auto f : sm.faces()) {
          if (sm.is_removed(f)) {
            continue;
          }
          /* Majority vote of corner vert_src0 among valid originals.
           * For pure edge-lerp verts use both endpoints as votes. */
          std::map<int, int> votes;
          auto v1p = sm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC1);
          for (const auto v : vertices_around_face(sm.halfedge(f), sm)) {
            const int a = (*v0p)[v];
            if (a >= 0) {
              votes[a]++;
            }
            if (v1p.has_value()) {
              const int b = (*v1p)[v];
              if (b >= 0) {
                votes[b]++;
              }
            }
          }
          /* Map vertex votes to original faces that contained those verts. */
          std::map<int, int> face_votes;
          if (!votes.empty() && mesh.corner_verts && mesh.faces_num > 0) {
            for (int of = 0; of < mesh.faces_num; of++) {
              int begin = 0, end = 0;
              if (mesh.face_offsets) {
                begin = mesh.face_offsets[of];
                end = mesh.face_offsets[of + 1];
              }
              else {
                begin = of * 3;
                end = begin + 3;
              }
              int score = 0;
              for (int c = begin; c < end; c++) {
                const int vi = mesh.corner_verts[c];
                auto it = votes.find(vi);
                if (it != votes.end()) {
                  score += it->second;
                }
              }
              if (score > 0) {
                face_votes[of] = score;
              }
            }
          }
          int best_f = -1;
          int best_s = -1;
          for (const auto &kv : face_votes) {
            if (kv.second > best_s) {
              best_s = kv.second;
              best_f = kv.first;
            }
          }
          (*fsp)[f] = best_f >= 0 ? best_f : -1;
        }
      }
    }
    result = surface_mesh_to_result(sm);
    (void)orig_faces;
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL subdivision failed";
  }
  return result;
}

TriangleMeshResult mesh_repair(const MeshIn &mesh)
{
  TriangleMeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_sm(mesh, sm, error, false)) {
    result.error = error.empty() ? "Repair input failed" : error;
    return result;
  }
  try {
    /* remove_degenerate_faces needs triangles; do not destroy n-gons for it. */
    if (CGAL::is_triangle_mesh(sm)) {
      PMP::remove_degenerate_faces(sm);
    }
    PMP::remove_isolated_vertices(sm);
    PMP::stitch_borders(sm);
    sm.collect_garbage();
    fill_missing_vert_maps_from_edges(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL repair failed";
  }
  return result;
}

TriangleMeshResult mesh_keep_largest_component(const MeshIn &mesh)
{
  TriangleMeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_sm(mesh, sm, error, false)) {
    result.error = error.empty() ? "Keep largest input failed" : error;
    return result;
  }
  try {
    PMP::keep_largest_connected_components(sm, 1);
    sm.collect_garbage();
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL keep_largest_connected_components failed";
  }
  return result;
}

TriangleMeshResult mesh_alpha_wrap(const MeshIn &mesh, double alpha, double offset)
{
  TriangleMeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_sm(mesh, sm, error, true)) {
    result.error = error.empty() ? "Alpha wrap input failed" : error;
    return result;
  }
  try {
    alpha = std::max(alpha, 1e-8);
    if (!(offset > 0.0)) {
      offset = alpha / 30.0;
    }
    Surface_mesh wrap;
    CGAL::alpha_wrap_3(sm, alpha, offset, wrap);
    result = surface_mesh_to_result(wrap);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL alpha_wrap_3 failed";
  }
  return result;
}

TriangleMeshResult mesh_corefine(const MeshIn &a, const MeshIn &b)
{
  TriangleMeshResult result;
  Surface_mesh sm_a, sm_b;
  std::string error;
  /*
   * Corefinement needs triangles for intersection. Track face ids through the
   * visitor, mark intersection edges constrained, then reverse-triangulate by
   * original face id (not coplanar merge).
   */
  if (!load_sm(a, sm_a, error, true)) {
    result.error = "Mesh A: " + (error.empty() ? "failed" : error);
    return result;
  }
  if (!load_sm(b, sm_b, error, true)) {
    result.error = "Mesh B: " + (error.empty() ? "failed" : error);
    return result;
  }
  try {
    /* Stamp mesh B face origin as mesh 1 before corefine. */
    {
      auto pm = sm_b.property_map<Surface_mesh::Face_index, int>(PROP_F_MESH);
      if (pm.has_value()) {
        for (const auto f : sm_b.faces()) {
          if (!sm_b.is_removed(f)) {
            (*pm)[f] = 1;
          }
        }
      }
    }

    auto ecm_a = sm_a.add_property_map<Surface_mesh::Edge_index, bool>(PROP_E_CONSTRAINED, false)
                     .first;
    auto ecm_b = sm_b.add_property_map<Surface_mesh::Edge_index, bool>(PROP_E_CONSTRAINED, false)
                     .first;

    /* Reuse the same face/vert id visitor as boolean (defined in mesh_ops).
     * Corefine only needs face-id propagation during splits; use lightweight
     * local visitor mirroring BooleanIdVisitor face callbacks. */
    struct CorefineIdVisitor : public PMP::Corefinement::Default_visitor<Surface_mesh> {
      int cur_face_src = -1;
      int cur_face_mesh = 0;
      int split_v0 = -1;
      int split_v1 = -1;

      static int get_fsrc(const Surface_mesh &tm, face_descriptor f)
      {
        auto p = tm.property_map<Surface_mesh::Face_index, int>(PROP_F_SRC);
        return p.has_value() ? (*p)[f] : -1;
      }
      static int get_fmesh(const Surface_mesh &tm, face_descriptor f)
      {
        auto p = tm.property_map<Surface_mesh::Face_index, int>(PROP_F_MESH);
        return p.has_value() ? (*p)[f] : 0;
      }
      static void set_face(Surface_mesh &tm, face_descriptor f, int src, int mesh_id)
      {
        Surface_mesh::Property_map<Surface_mesh::Face_index, int> ps, pm;
        auto ops = tm.property_map<Surface_mesh::Face_index, int>(PROP_F_SRC);
        auto opm = tm.property_map<Surface_mesh::Face_index, int>(PROP_F_MESH);
        ps = ops.has_value() ?
                 ops.value() :
                 tm.add_property_map<Surface_mesh::Face_index, int>(PROP_F_SRC, -1).first;
        pm = opm.has_value() ?
                 opm.value() :
                 tm.add_property_map<Surface_mesh::Face_index, int>(PROP_F_MESH, 0).first;
        ps[f] = src;
        pm[f] = mesh_id;
      }
      static int get_vsrc0(const Surface_mesh &tm, vertex_descriptor v)
      {
        auto p = tm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
        return p.has_value() ? (*p)[v] : -1;
      }
      static void set_vert(Surface_mesh &tm, vertex_descriptor v, int a, int b, float t)
      {
        Surface_mesh::Property_map<Surface_mesh::Vertex_index, int> p0, p1;
        Surface_mesh::Property_map<Surface_mesh::Vertex_index, float> pf;
        auto o0 = tm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
        auto o1 = tm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC1);
        auto of = tm.property_map<Surface_mesh::Vertex_index, float>(PROP_V_FACTOR);
        p0 = o0.has_value() ?
                 o0.value() :
                 tm.add_property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0, -1).first;
        p1 = o1.has_value() ?
                 o1.value() :
                 tm.add_property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC1, -1).first;
        pf = of.has_value() ?
                 of.value() :
                 tm.add_property_map<Surface_mesh::Vertex_index, float>(PROP_V_FACTOR, 0.0f).first;
        p0[v] = a;
        p1[v] = b;
        pf[v] = t;
      }

      void before_subface_creations(face_descriptor f_old, Surface_mesh &tm)
      {
        cur_face_src = get_fsrc(tm, f_old);
        cur_face_mesh = get_fmesh(tm, f_old);
      }
      void after_subface_created(face_descriptor f_new, Surface_mesh &tm)
      {
        set_face(tm, f_new, cur_face_src, cur_face_mesh);
      }
      void before_edge_split(halfedge_descriptor h, Surface_mesh &tm)
      {
        split_v0 = get_vsrc0(tm, tm.source(h));
        split_v1 = get_vsrc0(tm, tm.target(h));
      }
      void edge_split(halfedge_descriptor hnew, Surface_mesh &tm)
      {
        const vertex_descriptor v = tm.target(hnew);
        if (split_v0 >= 0 && split_v1 >= 0) {
          set_vert(tm, v, split_v0, split_v1, 0.5f);
        }
      }
    };

    CorefineIdVisitor visitor;
    PMP::corefine(sm_a,
                  sm_b,
                  CGAL::parameters::visitor(visitor).edge_is_constrained_map(ecm_a),
                  CGAL::parameters::edge_is_constrained_map(ecm_b));

    fill_missing_vert_maps_from_edges(sm_a);
    detriangulate_by_original_face(sm_a);
    result = surface_mesh_to_result(sm_a);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL corefine failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
