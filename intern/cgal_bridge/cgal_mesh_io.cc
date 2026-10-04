/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "cgal_mesh_io.hh"

#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/boost/graph/Euler_operations.h>
#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

namespace PMP = CGAL::Polygon_mesh_processing;

namespace blender::cgal_bridge {

using V_src0 = Surface_mesh::Property_map<Surface_mesh::Vertex_index, int>;
using V_src1 = Surface_mesh::Property_map<Surface_mesh::Vertex_index, int>;
using V_factor = Surface_mesh::Property_map<Surface_mesh::Vertex_index, float>;
using F_src = Surface_mesh::Property_map<Surface_mesh::Face_index, int>;
using F_mesh = Surface_mesh::Property_map<Surface_mesh::Face_index, int>;

static bool ensure_vert_maps(Surface_mesh &sm, V_src0 &s0, V_src1 &s1, V_factor &sf)
{
  bool created = false;
  auto p0 = sm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
  if (!p0.has_value()) {
    auto added = sm.add_property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0, -1);
    s0 = added.first;
    created = true;
  }
  else {
    s0 = p0.value();
  }
  auto p1 = sm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC1);
  if (!p1.has_value()) {
    auto added = sm.add_property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC1, -1);
    s1 = added.first;
  }
  else {
    s1 = p1.value();
  }
  auto pf = sm.property_map<Surface_mesh::Vertex_index, float>(PROP_V_FACTOR);
  if (!pf.has_value()) {
    auto added = sm.add_property_map<Surface_mesh::Vertex_index, float>(PROP_V_FACTOR, 0.0f);
    sf = added.first;
  }
  else {
    sf = pf.value();
  }
  return created;
}

static bool ensure_face_maps(Surface_mesh &sm, F_src &fs, F_mesh &fm)
{
  auto p = sm.property_map<Surface_mesh::Face_index, int>(PROP_F_SRC);
  if (!p.has_value()) {
    auto added = sm.add_property_map<Surface_mesh::Face_index, int>(PROP_F_SRC, -1);
    fs = added.first;
  }
  else {
    fs = p.value();
  }
  auto pm = sm.property_map<Surface_mesh::Face_index, int>(PROP_F_MESH);
  if (!pm.has_value()) {
    auto added = sm.add_property_map<Surface_mesh::Face_index, int>(PROP_F_MESH, 0);
    fm = added.first;
  }
  else {
    fm = pm.value();
  }
  return true;
}

static bool face_normal(const Surface_mesh &sm,
                        Surface_mesh::Face_index f,
                        double &nx,
                        double &ny,
                        double &nz)
{
  nx = ny = nz = 0.0;
  const auto h0 = sm.halfedge(f);
  if (h0 == Surface_mesh::null_halfedge()) {
    return false;
  }
  auto h = h0;
  do {
    const Point_3 &p = sm.point(sm.source(h));
    const Point_3 &q = sm.point(sm.target(h));
    const double px = CGAL::to_double(p.x());
    const double py = CGAL::to_double(p.y());
    const double pz = CGAL::to_double(p.z());
    const double qx = CGAL::to_double(q.x());
    const double qy = CGAL::to_double(q.y());
    const double qz = CGAL::to_double(q.z());
    nx += (py - qy) * (pz + qz);
    ny += (pz - qz) * (px + qx);
    nz += (px - qx) * (py + qy);
    h = sm.next(h);
  } while (h != h0);
  const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
  if (!(len > 1e-30)) {
    return false;
  }
  nx /= len;
  ny /= len;
  nz /= len;
  return true;
}

void detriangulate_by_original_face(
    Surface_mesh &sm, const std::function<bool(Surface_mesh::Edge_index)> *is_constrained)
{
  /**
   * Reverse local triangulation: join adjacent faces that share the same
   * original parent (f:src, f:mesh). Never join across constrained edges
   * (boolean/corefine intersection seams). No coplanarity test.
   */
  if (sm.number_of_faces() == 0) {
    return;
  }

  auto fsp = sm.property_map<Surface_mesh::Face_index, int>(PROP_F_SRC);
  if (!fsp.has_value()) {
    return;
  }
  F_src fsrc = fsp.value();
  F_mesh fmesh;
  ensure_face_maps(sm, fsrc, fmesh);

  auto econst = sm.property_map<Surface_mesh::Edge_index, bool>(PROP_E_CONSTRAINED);

  bool changed = true;
  int guard = 0;
  const int max_passes = int(sm.number_of_edges()) + 8;
  while (changed && guard++ < max_passes) {
    changed = false;
    std::vector<Surface_mesh::Halfedge_index> candidates;
    candidates.reserve(sm.number_of_edges());

    for (const auto e : sm.edges()) {
      if (sm.is_removed(e) || sm.is_border(e)) {
        continue;
      }
      if (is_constrained && (*is_constrained)(e)) {
        continue;
      }
      if (econst.has_value() && (*econst)[e]) {
        continue;
      }
      const auto h = sm.halfedge(e);
      const auto f1 = sm.face(h);
      const auto f2 = sm.face(sm.opposite(h));
      if (f1 == Surface_mesh::null_face() || f2 == Surface_mesh::null_face() || f1 == f2) {
        continue;
      }
      const int a = fsrc[f1];
      const int b = fsrc[f2];
      /* Both must map to the same original face of the same input mesh. */
      if (a < 0 || a != b || fmesh[f1] != fmesh[f2]) {
        continue;
      }
      candidates.push_back(h);
    }

    for (const auto h : candidates) {
      if (sm.is_removed(sm.edge(h)) || sm.is_border(sm.edge(h))) {
        continue;
      }
      const auto e = sm.edge(h);
      if (is_constrained && (*is_constrained)(e)) {
        continue;
      }
      if (econst.has_value() && (*econst)[e]) {
        continue;
      }
      const auto f1 = sm.face(h);
      const auto f2 = sm.face(sm.opposite(h));
      if (f1 == Surface_mesh::null_face() || f2 == Surface_mesh::null_face() || f1 == f2) {
        continue;
      }
      if (fsrc[f1] < 0 || fsrc[f1] != fsrc[f2] || fmesh[f1] != fmesh[f2]) {
        continue;
      }
      const int keep_src = fsrc[f1];
      const int keep_mesh = fmesh[f1];
      try {
        CGAL::Euler::join_face(h, sm);
        if (!sm.is_removed(f1)) {
          fsrc[f1] = keep_src;
          fmesh[f1] = keep_mesh;
        }
        changed = true;
      }
      catch (...) {
      }
    }
  }
  sm.collect_garbage();
}

struct TriangulateIdVisitor : public PMP::Triangulate_faces::Default_visitor<Surface_mesh> {
  F_src fsrc;
  F_mesh fmesh;
  int cur_src = -1;
  int cur_mesh = 0;

  void before_subface_creations(face_descriptor f_old)
  {
    cur_src = fsrc[f_old];
    cur_mesh = fmesh[f_old];
  }
  void after_subface_created(face_descriptor f_new)
  {
    fsrc[f_new] = cur_src;
    fmesh[f_new] = cur_mesh;
  }
  void after_subface_creations() {}
};

void triangulate_faces_keep_ids(Surface_mesh &sm)
{
  if (CGAL::is_triangle_mesh(sm)) {
    return;
  }
  F_src fsrc;
  F_mesh fmesh;
  ensure_face_maps(sm, fsrc, fmesh);
  TriangulateIdVisitor visitor;
  visitor.fsrc = fsrc;
  visitor.fmesh = fmesh;
  PMP::triangulate_faces(sm, CGAL::parameters::visitor(visitor));
}

void fill_missing_vert_maps_from_edges(Surface_mesh &sm)
{
  V_src0 s0;
  V_src1 s1;
  V_factor sf;
  ensure_vert_maps(sm, s0, s1, sf);

  for (const auto v : sm.vertices()) {
    if (sm.is_removed(v) || s0[v] >= 0) {
      continue;
    }
    /* Find two incident verts that map to original endpoints. */
    int a = -1, b = -1;
    for (const auto h : CGAL::halfedges_around_target(sm.halfedge(v), sm)) {
      const auto u = sm.source(h);
      if (s0[u] >= 0 && s1[u] < 0) {
        if (a < 0) {
          a = s0[u];
        }
        else if (b < 0 && s0[u] != a) {
          b = s0[u];
        }
      }
    }
    if (a >= 0 && b >= 0) {
      /* Parameterize by distance along chord between the two original-mapped neighbors. */
      Point_3 pa, pb;
      bool have_a = false, have_b = false;
      for (const auto u : sm.vertices()) {
        if (sm.is_removed(u)) {
          continue;
        }
        if (s0[u] == a && s1[u] < 0) {
          pa = sm.point(u);
          have_a = true;
        }
        if (s0[u] == b && s1[u] < 0) {
          pb = sm.point(u);
          have_b = true;
        }
      }
      float t = 0.5f;
      if (have_a && have_b) {
        const Point_3 &p = sm.point(v);
        const double ax = CGAL::to_double(pa.x()), ay = CGAL::to_double(pa.y()),
                     az = CGAL::to_double(pa.z());
        const double bx = CGAL::to_double(pb.x()), by = CGAL::to_double(pb.y()),
                     bz = CGAL::to_double(pb.z());
        const double px = CGAL::to_double(p.x()), py = CGAL::to_double(p.y()),
                     pz = CGAL::to_double(p.z());
        const double abx = bx - ax, aby = by - ay, abz = bz - az;
        const double apx = px - ax, apy = py - ay, apz = pz - az;
        const double ab2 = abx * abx + aby * aby + abz * abz;
        if (ab2 > 1e-30) {
          t = float(std::clamp((apx * abx + apy * aby + apz * abz) / ab2, 0.0, 1.0));
        }
      }
      s0[v] = a;
      s1[v] = b;
      sf[v] = t;
    }
  }
}

bool mesh_in_to_surface_mesh(const MeshIn &in,
                             Surface_mesh &sm,
                             std::string &error,
                             bool stamp_topology_ids)
{
  sm.clear();
  if (!in.positions || in.verts_num <= 0 || !in.corner_verts || in.faces_num <= 0) {
    error = "Empty mesh";
    return false;
  }

  V_src0 vs0;
  V_src1 vs1;
  V_factor vsf;
  F_src fsrc;
  F_mesh fmesh;
  if (stamp_topology_ids) {
    ensure_vert_maps(sm, vs0, vs1, vsf);
    ensure_face_maps(sm, fsrc, fmesh);
  }

  std::vector<Surface_mesh::Vertex_index> vmap;
  vmap.reserve(size_t(in.verts_num));
  for (int i = 0; i < in.verts_num; i++) {
    const float *p = in.positions + i * 3;
    const auto v = sm.add_vertex(Point_3(p[0], p[1], p[2]));
    vmap.push_back(v);
    if (stamp_topology_ids) {
      vs0[v] = i;
      vs1[v] = -1;
      vsf[v] = 0.0f;
    }
  }

  for (int f = 0; f < in.faces_num; f++) {
    int begin = 0;
    int end = 0;
    if (in.face_offsets) {
      begin = in.face_offsets[f];
      end = in.face_offsets[f + 1];
    }
    else {
      begin = f * 3;
      end = begin + 3;
    }
    if (end - begin < 3 || begin < 0 || end > in.corners_num) {
      error = "Invalid face offsets";
      return false;
    }

    std::vector<Surface_mesh::Vertex_index> face_vs;
    face_vs.reserve(size_t(end - begin));
    for (int c = begin; c < end; c++) {
      const int vi = in.corner_verts[c];
      if (vi < 0 || vi >= in.verts_num) {
        error = "Invalid corner index";
        return false;
      }
      if (!face_vs.empty() && face_vs.back() == vmap[size_t(vi)]) {
        continue;
      }
      face_vs.push_back(vmap[size_t(vi)]);
    }
    if (face_vs.size() >= 3 && face_vs.front() == face_vs.back()) {
      face_vs.pop_back();
    }
    if (face_vs.size() < 3) {
      continue;
    }
    const auto fh = sm.add_face(face_vs);
    if (stamp_topology_ids && fh != Surface_mesh::null_face()) {
      fsrc[fh] = f;
      fmesh[fh] = 0;
    }
  }

  if (sm.number_of_faces() == 0) {
    error = "No valid faces after conversion";
    return false;
  }
  return true;
}

MeshResult surface_mesh_to_result(const Surface_mesh &sm)
{
  MeshResult result;
  if (sm.is_empty()) {
    result.error = "Empty Surface_mesh";
    return result;
  }

  auto v0p = sm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC0);
  auto v1p = sm.property_map<Surface_mesh::Vertex_index, int>(PROP_V_SRC1);
  auto vfp = sm.property_map<Surface_mesh::Vertex_index, float>(PROP_V_FACTOR);
  auto fsp = sm.property_map<Surface_mesh::Face_index, int>(PROP_F_SRC);
  auto fmp = sm.property_map<Surface_mesh::Face_index, int>(PROP_F_MESH);
  const bool has_v = v0p.has_value();
  const bool has_f = fsp.has_value();

  std::map<Surface_mesh::Vertex_index, int> v2i;
  result.positions.reserve(sm.number_of_vertices() * 3);
  if (has_v) {
    result.vert_src0.reserve(sm.number_of_vertices());
    result.vert_src1.reserve(sm.number_of_vertices());
    result.vert_factor.reserve(sm.number_of_vertices());
  }

  for (const auto v : sm.vertices()) {
    if (sm.is_removed(v)) {
      continue;
    }
    const Point_3 &p = sm.point(v);
    v2i[v] = int(result.positions.size() / 3);
    result.positions.push_back(float(CGAL::to_double(p.x())));
    result.positions.push_back(float(CGAL::to_double(p.y())));
    result.positions.push_back(float(CGAL::to_double(p.z())));
    if (has_v) {
      result.vert_src0.push_back((*v0p)[v]);
      result.vert_src1.push_back(v1p.has_value() ? (*v1p)[v] : -1);
      result.vert_factor.push_back(vfp.has_value() ? (*vfp)[v] : 0.0f);
    }
  }

  result.face_offsets.clear();
  result.face_offsets.push_back(0);
  result.corner_verts.reserve(sm.number_of_halfedges());
  bool all_tri = true;
  bool any_mesh_b = false;

  for (const auto f : sm.faces()) {
    if (sm.is_removed(f)) {
      continue;
    }
    std::vector<int> face_verts;
    for (const auto v : vertices_around_face(sm.halfedge(f), sm)) {
      auto it = v2i.find(v);
      if (it == v2i.end()) {
        face_verts.clear();
        break;
      }
      face_verts.push_back(it->second);
    }
    if (face_verts.size() < 3) {
      continue;
    }
    if (face_verts.size() != 3) {
      all_tri = false;
    }
    for (const int vi : face_verts) {
      result.corner_verts.push_back(vi);
    }
    result.face_offsets.push_back(int(result.corner_verts.size()));
    if (has_f) {
      result.face_src.push_back((*fsp)[f]);
      const int mid = fmp.has_value() ? (*fmp)[f] : 0;
      result.face_src_mesh.push_back(int8_t(mid));
      if (mid != 0) {
        any_mesh_b = true;
      }
    }
  }

  if (result.positions.empty() || result.corner_verts.empty()) {
    result.error = "No faces produced";
    return result;
  }

  if (all_tri) {
    result.face_offsets.clear();
  }
  if (!any_mesh_b) {
    result.face_src_mesh.clear();
  }

  /* Corefine/boolean seams: constrained edges as endpoint pairs in result indices. */
  auto econst = sm.property_map<Surface_mesh::Edge_index, bool>(PROP_E_CONSTRAINED);
  if (econst.has_value()) {
    result.seam_vert_a.reserve(sm.number_of_edges());
    result.seam_vert_b.reserve(sm.number_of_edges());
    for (const auto e : sm.edges()) {
      if (sm.is_removed(e) || !(*econst)[e]) {
        continue;
      }
      const auto h = sm.halfedge(e);
      const auto va = sm.source(h);
      const auto vb = sm.target(h);
      auto ia = v2i.find(va);
      auto ib = v2i.find(vb);
      if (ia == v2i.end() || ib == v2i.end()) {
        continue;
      }
      result.seam_vert_a.push_back(ia->second);
      result.seam_vert_b.push_back(ib->second);
    }
  }

  result.ok = true;
  return result;
}

MeshResult surface_mesh_to_triangles(const Surface_mesh &sm)
{
  MeshResult poly = surface_mesh_to_result(sm);
  if (!poly.ok || poly.face_offsets.empty()) {
    return poly;
  }
  MeshResult tri;
  tri.positions = std::move(poly.positions);
  tri.vert_src0 = std::move(poly.vert_src0);
  tri.vert_src1 = std::move(poly.vert_src1);
  tri.vert_factor = std::move(poly.vert_factor);
  tri.corner_verts.reserve(poly.corner_verts.size());
  for (int f = 0; f + 1 < int(poly.face_offsets.size()); f++) {
    const int begin = poly.face_offsets[f];
    const int end = poly.face_offsets[f + 1];
    if (end - begin < 3) {
      continue;
    }
    const int i0 = poly.corner_verts[begin];
    const int src_f = (f < int(poly.face_src.size())) ? poly.face_src[f] : -1;
    const int8_t src_m = (f < int(poly.face_src_mesh.size())) ? poly.face_src_mesh[f] : int8_t(0);
    for (int c = begin + 1; c + 1 < end; c++) {
      tri.corner_verts.push_back(i0);
      tri.corner_verts.push_back(poly.corner_verts[c]);
      tri.corner_verts.push_back(poly.corner_verts[c + 1]);
      if (src_f >= 0 || !poly.face_src.empty()) {
        tri.face_src.push_back(src_f);
      }
      if (!poly.face_src_mesh.empty()) {
        tri.face_src_mesh.push_back(src_m);
      }
    }
  }
  if (tri.corner_verts.empty()) {
    tri.error = "No triangles after fan";
    return tri;
  }
  tri.ok = true;
  return tri;
}

bool surface_mesh_positions_to_buffer(const Surface_mesh &sm, std::vector<float> &positions)
{
  positions.clear();
  positions.reserve(sm.number_of_vertices() * 3);
  for (const auto v : sm.vertices()) {
    if (sm.is_removed(v)) {
      continue;
    }
    const Point_3 &p = sm.point(v);
    positions.push_back(float(CGAL::to_double(p.x())));
    positions.push_back(float(CGAL::to_double(p.y())));
    positions.push_back(float(CGAL::to_double(p.z())));
  }
  return !positions.empty();
}

void assign_subdiv_vert_maps(Surface_mesh & /*sm_before*/,
                             Surface_mesh &sm_after,
                             int original_verts_num,
                             int /*original_faces_num*/)
{
  /**
   * CGAL subdivision keeps original vertices (updated in place) and appends
   * new edge/face points. Stamp original verts as pure copies; for new verts
   * build edge-lerp maps from incident original-mapped neighbors.
   */
  V_src0 s0;
  V_src1 s1;
  V_factor sf;
  ensure_vert_maps(sm_after, s0, s1, sf);

  /* First pass: verts that already have maps (from pre-subdiv stamp) keep them.
   * Original vertices that lost maps: match by stable iteration order of the
   * first original_verts_num non-removed verts when they still carry src. */
  int stamped = 0;
  for (const auto v : sm_after.vertices()) {
    if (sm_after.is_removed(v)) {
      continue;
    }
    if (s0[v] >= 0) {
      stamped++;
    }
  }

  if (stamped == 0 && original_verts_num > 0) {
    /* Fallback: first N verts in index order are treated as original. */
    int i = 0;
    for (const auto v : sm_after.vertices()) {
      if (sm_after.is_removed(v)) {
        continue;
      }
      if (i < original_verts_num) {
        s0[v] = i;
        s1[v] = -1;
        sf[v] = 0.0f;
      }
      i++;
    }
  }

  fill_missing_vert_maps_from_edges(sm_after);
}

}  // namespace blender::cgal_bridge
