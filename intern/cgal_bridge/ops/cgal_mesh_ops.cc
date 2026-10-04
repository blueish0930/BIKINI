/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_bridge.hh"
#include "cgal_mesh_io.hh"
#include "cgal_types.hh"

/* Must be defined before any CGAL header that branches on Eigen. */
#ifndef CGAL_EIGEN3_ENABLED
#  define CGAL_EIGEN3_ENABLED 1
#endif

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Eigen_solver_traits.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;
namespace SMS = CGAL::Surface_mesh_simplification;

namespace blender::cgal_bridge {

static bool load_sm_tri(const MeshIn &mesh, Surface_mesh &sm, std::string &error, int mesh_id = 0)
{
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true)) {
    return false;
  }
  if (mesh_id != 0) {
    auto pm = sm.property_map<Surface_mesh::Face_index, int>(PROP_F_MESH);
    if (pm.has_value()) {
      for (const auto f : sm.faces()) {
        if (!sm.is_removed(f)) {
          (*pm)[f] = mesh_id;
        }
      }
    }
  }
  triangulate_faces_keep_ids(sm);
  return sm.number_of_faces() > 0;
}

/** Propagate face/vert topology ids through CGAL corefinement boolean. */
struct BooleanIdVisitor : public PMP::Corefinement::Default_visitor<Surface_mesh> {
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
  static void set_face(Surface_mesh &tm, face_descriptor f, int src, int mesh)
  {
    auto ps = tm.property_map<Surface_mesh::Face_index, int>(PROP_F_SRC);
    auto pm = tm.property_map<Surface_mesh::Face_index, int>(PROP_F_MESH);
    if (!ps.has_value()) {
      auto a = tm.add_property_map<Surface_mesh::Face_index, int>(PROP_F_SRC, -1);
      ps = a.first;
    }
    if (!pm.has_value()) {
      auto a = tm.add_property_map<Surface_mesh::Face_index, int>(PROP_F_MESH, 0);
      pm = a.first;
    }
    (*ps)[f] = src;
    (*pm)[f] = mesh;
  }
  static void set_vert_copy(Surface_mesh &tm, vertex_descriptor v, int src0)
  {
    auto p0 = tm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
    auto p1 = tm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC1);
    auto pf = tm.property_map<Surface_mesh::Vertex_index, float>(PROP_V_FACTOR);
    if (!p0.has_value()) {
      auto a = tm.add_property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0, -1);
      p0 = a.first;
    }
    if (!p1.has_value()) {
      auto a = tm.add_property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC1, -1);
      p1 = a.first;
    }
    if (!pf.has_value()) {
      auto a = tm.add_property_map<Surface_mesh::Vertex_index, float>(PROP_V_FACTOR, 0.0f);
      pf = a.first;
    }
    (*p0)[v] = src0;
    (*p1)[v] = -1;
    (*pf)[v] = 0.0f;
  }
  static int get_vsrc0(const Surface_mesh &tm, vertex_descriptor v)
  {
    auto p = tm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
    return p.has_value() ? (*p)[v] : -1;
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
  void after_face_copy(face_descriptor f_old,
                       const Surface_mesh &tm_src,
                       face_descriptor f_new,
                       Surface_mesh &tm_out)
  {
    set_face(tm_out, f_new, get_fsrc(tm_src, f_old), get_fmesh(tm_src, f_old));
  }
  void after_vertex_copy(vertex_descriptor v_src,
                         const Surface_mesh &tm_src,
                         vertex_descriptor v_tgt,
                         Surface_mesh &tm_tgt)
  {
    set_vert_copy(tm_tgt, v_tgt, get_vsrc0(tm_src, v_src));
  }
  void before_edge_split(halfedge_descriptor h, Surface_mesh &tm)
  {
    split_v0 = get_vsrc0(tm, tm.source(h));
    split_v1 = get_vsrc0(tm, tm.target(h));
  }
  void edge_split(halfedge_descriptor hnew, Surface_mesh &tm)
  {
    /* hnew points toward the newly inserted vertex. */
    const vertex_descriptor v = tm.target(hnew);
    auto p0 = tm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
    auto p1 = tm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC1);
    auto pf = tm.property_map<Surface_mesh::Vertex_index, float>(PROP_V_FACTOR);
    if (!p0.has_value()) {
      return;
    }
    if (split_v0 >= 0 && split_v1 >= 0) {
      (*p0)[v] = split_v0;
      if (p1.has_value()) {
        (*p1)[v] = split_v1;
      }
      if (pf.has_value()) {
        (*pf)[v] = 0.5f;
      }
    }
  }
};

TriangleMeshResult mesh_simplify(const MeshIn &mesh, double stop_ratio)
{
  TriangleMeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_sm_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Simplify input failed" : error;
    return result;
  }
  try {
    stop_ratio = std::max(0.01, std::min(stop_ratio, 1.0));
    SMS::Count_ratio_stop_predicate<Surface_mesh> stop(stop_ratio);
    SMS::edge_collapse(sm, stop);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL mesh simplify failed";
  }
  return result;
}

TriangleMeshResult mesh_isotropic_remesh(const MeshIn &mesh,
                                         double target_edge_length,
                                         int iterations)
{
  TriangleMeshResult result;
  Surface_mesh sm;
  std::string error;
  if (!load_sm_tri(mesh, sm, error)) {
    result.error = error.empty() ? "Remesh input failed" : error;
    return result;
  }
  try {
    target_edge_length = std::max(target_edge_length, 1e-8);
    iterations = std::max(iterations, 1);
    PMP::isotropic_remeshing(faces(sm),
                             target_edge_length,
                             sm,
                             CGAL::parameters::number_of_iterations(iterations));
    /* Full remesh: no parent topology maps. */
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
    result.error = "CGAL isotropic remesh failed";
  }
  return result;
}

TriangleMeshResult mesh_smooth_shape(const MeshIn &mesh,
                                     double time,
                                     int iterations,
                                     bool do_scale)
{
  /*
   * Dedicated Simple_cartesian<double> mesh for MCF.
   * EPICK + Eigen float path was unreliable (no-op or noise). CGAL examples use
   * double Cartesian kernels for this algorithm.
   */
  using MCF_K = CGAL::Simple_cartesian<double>;
  using MCF_Point = MCF_K::Point_3;
  using MCF_Mesh = CGAL::Surface_mesh<MCF_Point>;

  TriangleMeshResult result;
  if (!mesh.positions || mesh.verts_num < 4 || !mesh.corner_verts || mesh.faces_num <= 0) {
    result.error = "Need a non-empty mesh with at least 4 vertices";
    return result;
  }
  if (!(time > 0.0)) {
    result.error = "Mean curvature flow requires Time > 0. Try 0.01 .. 0.1.";
    return result;
  }
  time = std::min(std::max(time, 1e-6), 1.0);
  iterations = std::max(iterations, 1);

  try {
    MCF_Mesh sm;
    std::vector<MCF_Mesh::Vertex_index> vmap;
    vmap.reserve(size_t(mesh.verts_num));

    /* Centroid + mean edge length for scale-robust MCF. */
    double cx = 0, cy = 0, cz = 0;
    for (int i = 0; i < mesh.verts_num; i++) {
      cx += double(mesh.positions[i * 3 + 0]);
      cy += double(mesh.positions[i * 3 + 1]);
      cz += double(mesh.positions[i * 3 + 2]);
    }
    const double inv_n = 1.0 / double(mesh.verts_num);
    cx *= inv_n;
    cy *= inv_n;
    cz *= inv_n;

    double edge_sum = 0;
    int edge_count = 0;
    auto add_len = [&](int a, int b) {
      const double dx = double(mesh.positions[a * 3 + 0] - mesh.positions[b * 3 + 0]);
      const double dy = double(mesh.positions[a * 3 + 1] - mesh.positions[b * 3 + 1]);
      const double dz = double(mesh.positions[a * 3 + 2] - mesh.positions[b * 3 + 2]);
      edge_sum += std::sqrt(dx * dx + dy * dy + dz * dz);
      edge_count++;
    };
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
      for (int c = begin; c < end; c++) {
        const int a = mesh.corner_verts[c];
        const int b = mesh.corner_verts[c + 1 < end ? c + 1 : begin];
        add_len(a, b);
      }
    }
    const double mean_edge = (edge_count > 0) ? (edge_sum / double(edge_count)) : 1.0;
    const double target_edge = 0.1;
    const double s = (mean_edge > 1e-30) ? (mean_edge / target_edge) : 1.0;
    const double inv_s = 1.0 / s;

    for (int i = 0; i < mesh.verts_num; i++) {
      const double x = (double(mesh.positions[i * 3 + 0]) - cx) * inv_s;
      const double y = (double(mesh.positions[i * 3 + 1]) - cy) * inv_s;
      const double z = (double(mesh.positions[i * 3 + 2]) - cz) * inv_s;
      vmap.push_back(sm.add_vertex(MCF_Point(x, y, z)));
    }
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
      std::vector<MCF_Mesh::Vertex_index> fvs;
      for (int c = begin; c < end; c++) {
        fvs.push_back(vmap[size_t(mesh.corner_verts[c])]);
      }
      if (fvs.size() >= 3) {
        sm.add_face(fvs);
      }
    }
    if (sm.number_of_faces() == 0) {
      result.error = "No valid faces for mean curvature flow";
      return result;
    }
    /* MCF requires a triangle mesh. */
    if (!CGAL::is_triangle_mesh(sm)) {
      PMP::triangulate_faces(sm);
    }

    /*
     * Use a direct sparse Cholesky solver instead of the default
     * BiCGSTAB+IncompleteLUT, which often silently fails (zero motion)
     * on denser meshes at moderate time steps.
     */
    using Eigen_matrix = typename CGAL::Eigen_sparse_matrix<double>::EigenType;
    using Eigen_solver = Eigen::SimplicialLDLT<Eigen_matrix>;
    using Solver_traits = CGAL::Eigen_solver_traits<Eigen_solver>;
    Solver_traits solver;

    /* CGAL PMP mean curvature flow (not Laplacian blur). */
    PMP::smooth_shape(
        sm,
        time,
        CGAL::parameters::number_of_iterations(static_cast<unsigned int>(iterations))
            .do_scale(do_scale)
            .sparse_linear_solver(solver));

    /* Restore scale/center. Vertex count must match input (triangulate_faces adds no verts). */
    if (int(sm.number_of_vertices()) != mesh.verts_num) {
      result.error = "MCF changed vertex count unexpectedly";
      return result;
    }
    result.positions.resize(size_t(mesh.verts_num) * 3);
    int vi = 0;
    for (const auto v : sm.vertices()) {
      const MCF_Point &p = sm.point(v);
      result.positions[size_t(vi) * 3 + 0] = float(p.x() * s + cx);
      result.positions[size_t(vi) * 3 + 1] = float(p.y() * s + cy);
      result.positions[size_t(vi) * 3 + 2] = float(p.z() * s + cz);
      vi++;
    }
    /* Preserve original face topology (n-gons), only positions change. */
    const int cn = mesh.corners_num > 0 ? mesh.corners_num :
                                          (mesh.face_offsets ? 0 : mesh.faces_num * 3);
    if (cn > 0) {
      result.corner_verts.assign(mesh.corner_verts, mesh.corner_verts + cn);
    }
    if (mesh.face_offsets) {
      result.face_offsets.assign(mesh.face_offsets, mesh.face_offsets + mesh.faces_num + 1);
    }
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
    result.error = "CGAL smooth_shape (mean curvature flow) failed";
  }
  return result;
}

TriangleMeshResult mesh_boolean(const MeshIn &a, const MeshIn &b, int op)
{
  TriangleMeshResult result;
  Surface_mesh sm_a, sm_b;
  std::string error;
  /*
   * CGAL corefinement requires pure triangle meshes for the intersection
   * algorithm (CGAL_precondition). Triangulate only for that step (keeping
   * face/vert ids), then join coplanar faces so exported n-gons stay where
   * possible. Attribute maps travel with the visitor.
   */
  if (!load_sm_tri(a, sm_a, error, 0)) {
    result.error = "Mesh A: " + (error.empty() ? "failed" : error);
    return result;
  }
  if (!load_sm_tri(b, sm_b, error, 1)) {
    result.error = "Mesh B: " + (error.empty() ? "failed" : error);
    return result;
  }
  try {
    Surface_mesh out;
    BooleanIdVisitor visitor;

    /* Constrained edge maps: CGAL marks intersection/seam edges true. */
    auto ecm_a = sm_a.add_property_map<Surface_mesh::Edge_index, bool>(PROP_E_CONSTRAINED, false)
                     .first;
    auto ecm_b = sm_b.add_property_map<Surface_mesh::Edge_index, bool>(PROP_E_CONSTRAINED, false)
                     .first;
    auto ecm_out = out.add_property_map<Surface_mesh::Edge_index, bool>(PROP_E_CONSTRAINED, false)
                       .first;

    const auto np1 = CGAL::parameters::visitor(visitor).edge_is_constrained_map(ecm_a);
    const auto np2 = CGAL::parameters::edge_is_constrained_map(ecm_b);
    const auto np_out = CGAL::parameters::edge_is_constrained_map(ecm_out);

    bool ok = false;
    switch (op) {
      case 1:
        ok = PMP::corefine_and_compute_difference(sm_a, sm_b, out, np1, np2, np_out);
        break;
      case 2:
        ok = PMP::corefine_and_compute_intersection(sm_a, sm_b, out, np1, np2, np_out);
        break;
      default:
        ok = PMP::corefine_and_compute_union(sm_a, sm_b, out, np1, np2, np_out);
        break;
    }
    if (!ok) {
      result.error = "CGAL boolean corefinement failed (non-manifold or degenerate)";
      return result;
    }
    fill_missing_vert_maps_from_edges(out);
    /* Restore n-gons by reverse-triangulating within each original face id.
     * Never join across constrained (intersection) edges. No coplanar merge. */
    detriangulate_by_original_face(out);
    result = surface_mesh_to_result(out);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL mesh boolean failed";
  }
  return result;
}

TriangleMeshResult mesh_hole_fill(const MeshIn &mesh)
{
  TriangleMeshResult result;
  Surface_mesh sm;
  std::string error;
  /* Keep original n-gons; only the filled hole patches are triangulated. */
  if (!mesh_in_to_surface_mesh(mesh, sm, error, true) || sm.number_of_faces() == 0) {
    result.error = error.empty() ? "Hole fill input failed" : error;
    return result;
  }
  try {
    std::vector<Surface_mesh::Halfedge_index> border_loops;
    CGAL::extract_boundary_cycles(sm, std::back_inserter(border_loops));
    for (const auto h : border_loops) {
      std::vector<Surface_mesh::Face_index> patch_faces;
      std::vector<Surface_mesh::Vertex_index> patch_vertices;
      PMP::triangulate_and_refine_hole(sm,
                                       h,
                                       std::back_inserter(patch_faces),
                                       std::back_inserter(patch_vertices));
    }
    fill_missing_vert_maps_from_edges(sm);
    result = surface_mesh_to_result(sm);
  }
  catch (const std::exception &e) {
    result.error = e.what();
  }
  catch (...) {
    result.error = "CGAL hole fill failed";
  }
  return result;
}

}  // namespace blender::cgal_bridge
