/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Voronoi Fracture
 *
 * **No Blender GEO boolean** (Manifold / Float / MeshArr) — too slow and too picky.
 *
 * Neighbors: Voro++ exact Voronoi adjacency planes (sparse, correct).
 * Clip: `mesh_clip_by_plane` (triangle Sutherland–Hodgman, open/non-manifold OK).
 * Solid: after each plane clip, fill only boundary holes (planar cut) via BMO
 *        holes_fill — not mesh∩mesh boolean, not N× full Bisect rebuild logic.
 */

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "BLI_array.hh"
#include "BLI_bounds.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"

#include "GEO_join_geometries.hh"
#include "GEO_mesh_clip_plane.hh"

#include "bmesh.hh"
#include "bmesh_tools.hh"

#include "MEM_guardedalloc.h"

#include "node_geometry_util.hh"

#ifdef WITH_VORO_PLUSPLUS
#  include "voro++.hh"
#endif

namespace blender::nodes::node_geo_voronoi_fracture_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .is_default_link_socket()
      .description("Mesh to fracture (closed solid recommended for Inside)");
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Fractured mesh (joined Voronoi cells)");

  auto &points_in = b.add_input<decl::Geometry>("Points"_ustr)
                        .only_realized_data()
                        .supported_type(
                            {GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
                        .description("Voronoi sites (point cloud or mesh vertices)");
  b.add_input<decl::Float>("Gap"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .evaluated_geometry_field({points_in.index()})
      .description("Per-site wall shrink toward the site");
  b.add_input<decl::Bool>("Inside"_ustr)
      .default_value(true)
      .description(
          "Solid: plane-clip + fill cut caps (no Blender boolean library). "
          "Off: open shells only (non-manifold OK)");
  b.add_input<decl::Int>("Max Neighbors"_ustr)
      .default_value(0)
      .min(0)
      .description("Unused (Voro++ uses exact neighbors). Kept for compatibility; leave 0");
}

static Span<float3> positions_from_points(const GeometrySet &points_geometry)
{
  if (const PointCloud *pointcloud = points_geometry.get_pointcloud()) {
    return pointcloud->positions();
  }
  if (const Mesh *mesh = points_geometry.get_mesh()) {
    return mesh->vert_positions();
  }
  return {};
}

static Array<float> evaluate_gap_on_points(const GeometrySet &points_geometry,
                                           bke::SocketValueVariant &gap_value,
                                           const int sites_num)
{
  Array<float> gaps(sites_num, 0.0f);
  if (sites_num == 0) {
    return gaps;
  }
  if (!gap_value.is_context_dependent_field()) {
    gap_value.convert_to_single();
  }
  if (gap_value.is_single()) {
    gaps.fill(math::max(gap_value.copy_as<float>(), 0.0f));
    return gaps;
  }
  if (!gap_value.is_field()) {
    return gaps;
  }
  const Field<float> gap_field = gap_value.copy_as<Field<float>>();
  auto materialize = [&](const fn::FieldContext &ctx) -> bool {
    fn::FieldEvaluator evaluator{ctx, sites_num};
    evaluator.add(gap_field);
    evaluator.evaluate();
    const VArray<float> values = evaluator.get_evaluated<float>(0);
    if (values.size() != sites_num) {
      return false;
    }
    values.materialize(gaps.as_mutable_span());
    for (float &g : gaps) {
      g = math::max(g, 0.0f);
    }
    return true;
  };
  if (const PointCloud *pointcloud = points_geometry.get_pointcloud()) {
    if (pointcloud->totpoint == sites_num &&
        materialize(bke::PointCloudFieldContext{*pointcloud}))
    {
      return gaps;
    }
  }
  if (const Mesh *mesh = points_geometry.get_mesh()) {
    if (mesh->verts_num == sites_num &&
        materialize(bke::MeshFieldContext{*mesh, AttrDomain::Point}))
    {
      return gaps;
    }
  }
  return gaps;
}

struct ClipPlane {
  float3 position;
  float3 normal; /* away from site */
};

/* Hidden point attr: original / propagated outer shading normals through clip. */
static constexpr StringRefNull vf_point_normal_id = ".vf_point_normal";

static void write_cell_index(Mesh &mesh, const int site_i)
{
  if (mesh.faces_num == 0) {
    return;
  }
  bke::SpanAttributeWriter<int> cell_index =
      mesh.attributes_for_write().lookup_or_add_for_write_only_span<int>("cell_index",
                                                                         AttrDomain::Face);
  cell_index.span.fill(site_i);
  cell_index.finish();
}

static void write_face_side_attrs(Mesh &mesh, const bool all_outside)
{
  if (mesh.faces_num == 0) {
    return;
  }
  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  bke::SpanAttributeWriter<bool> outside =
      attributes.lookup_or_add_for_write_only_span<bool>("outside", AttrDomain::Face);
  bke::SpanAttributeWriter<bool> inside =
      attributes.lookup_or_add_for_write_only_span<bool>("inside", AttrDomain::Face);
  outside.span.fill(all_outside);
  inside.span.fill(!all_outside);
  outside.finish();
  inside.finish();
}

/**
 * Bake evaluated vertex normals onto points so mesh_clip can propagate them
 * (original verts keep exact normals; cut verts get interpolated).
 */
static void bake_point_normals(Mesh &mesh)
{
  if (mesh.verts_num == 0) {
    return;
  }
  const Span<float3> nors = mesh.vert_normals();
  bke::SpanAttributeWriter<float3> writer =
      mesh.attributes_for_write().lookup_or_add_for_write_only_span<float3>(vf_point_normal_id,
                                                                            AttrDomain::Point);
  if (!writer) {
    return;
  }
  writer.span.copy_from(nors);
  writer.finish();
}

/**
 * Cap faces from hole-fill lie on the cutting plane. Tag them inside so outer
 * surface corners can keep original normals while caps use face normals.
 * Never clears a previously-tagged inside face.
 */
static void tag_cut_cap_faces(Mesh &mesh,
                              const float3 &plane_position,
                              const float3 &plane_normal)
{
  if (mesh.faces_num == 0) {
    return;
  }
  constexpr float align_min = 0.92f;
  float dist_eps = 1.5e-4f;
  if (const std::optional<Bounds<float3>> bounds = mesh.bounds_min_max()) {
    dist_eps = math::max(dist_eps, 1.0e-5f * math::length(bounds->size()));
  }

  const Span<float3> positions = mesh.vert_positions();
  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<float3> face_normals = mesh.face_normals();

  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  bke::SpanAttributeWriter<bool> outside = attributes.lookup_or_add_for_write_span<bool>(
      "outside", AttrDomain::Face);
  bke::SpanAttributeWriter<bool> inside = attributes.lookup_or_add_for_write_span<bool>(
      "inside", AttrDomain::Face);
  if (!outside || !inside) {
    if (outside) {
      outside.finish();
    }
    if (inside) {
      inside.finish();
    }
    return;
  }

  for (const int face_i : faces.index_range()) {
    if (inside.span[face_i]) {
      continue; /* Keep previous caps. */
    }
    bool all_on_plane = true;
    for (const int vert : corner_verts.slice(faces[face_i])) {
      if (math::abs(math::dot(positions[vert] - plane_position, plane_normal)) > dist_eps) {
        all_on_plane = false;
        break;
      }
    }
    if (!all_on_plane) {
      continue;
    }
    if (math::abs(math::dot(face_normals[face_i], plane_normal)) < align_min) {
      continue;
    }
    outside.span[face_i] = false;
    inside.span[face_i] = true;
  }
  outside.finish();
  inside.finish();
}

/**
 * Restore shading: outer faces use propagated point normals; inner (cut) faces
 * use flat face normals. Avoids averaging outer + wall normals at shared verts.
 */
static void apply_preserved_outer_normals(Mesh &mesh)
{
  if (mesh.verts_num == 0 || mesh.faces_num == 0 || mesh.corners_num == 0) {
    return;
  }

  bke::AttributeAccessor attributes = mesh.attributes();
  const bke::AttributeReader<float3> point_nors_reader = attributes.lookup<float3>(
      vf_point_normal_id, AttrDomain::Point);
  const bool has_point_nors = point_nors_reader &&
                              point_nors_reader.varray.size() == mesh.verts_num;
  const VArraySpan<float3> point_nors = has_point_nors ? VArraySpan(point_nors_reader.varray) :
                                                         VArraySpan<float3>();

  const bke::AttributeReader<bool> outside_reader = attributes.lookup<bool>("outside",
                                                                            AttrDomain::Face);
  const bke::AttributeReader<bool> inside_reader = attributes.lookup<bool>("inside",
                                                                           AttrDomain::Face);
  const bool has_outside = outside_reader && outside_reader.varray.size() == mesh.faces_num;
  const bool has_inside = inside_reader && inside_reader.varray.size() == mesh.faces_num;

  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<float3> face_normals = mesh.face_normals();
  /* Only used if bake was lost after BMesh convert. */
  const Span<float3> auto_vert_nors = mesh.vert_normals();

  Array<float3> corner_normals(mesh.corners_num);
  for (const int face_i : faces.index_range()) {
    bool is_outside = true;
    if (has_inside && inside_reader.varray[face_i]) {
      is_outside = false;
    }
    else if (has_outside) {
      is_outside = outside_reader.varray[face_i];
    }
    const float3 face_n = face_normals[face_i];
    for (const int corner : faces[face_i]) {
      if (!is_outside) {
        corner_normals[corner] = face_n;
        continue;
      }
      const int vi = corner_verts[corner];
      float3 n = has_point_nors ? point_nors[vi] : auto_vert_nors[vi];
      if (math::length_squared(n) <= 1.0e-20f) {
        n = face_n;
      }
      corner_normals[corner] = n;
    }
  }

  bke::mesh_set_custom_normals(mesh, corner_normals);

  /* Drop temporary bake so join/export stays clean. */
  mesh.attributes_for_write().remove(vf_point_normal_id);
}

static int classify_aabb(const Bounds<float3> &bounds,
                         const float3 &plane_position,
                         const float3 &plane_normal)
{
  const float3 center = bounds.center();
  const float3 half_extent = bounds.size() * 0.5f;
  const float center_distance = math::dot(center - plane_position, plane_normal);
  const float radius = math::dot(math::abs(plane_normal), half_extent);
  if (center_distance + radius <= 0.0f) {
    return 1; /* fully kept */
  }
  if (center_distance - radius > 0.0f) {
    return -1; /* discard */
  }
  return 0;
}

static Mesh *fracture_open_clip(const Mesh &src_mesh,
                                const Bounds<float3> &mesh_bounds,
                                const Span<ClipPlane> planes,
                                const bke::AttributeFilter &attribute_filter)
{
  Mesh *owned = nullptr;
  Bounds<float3> piece_bounds = mesh_bounds;
  for (const ClipPlane &plane : planes) {
    const Mesh &current = owned ? *owned : src_mesh;
    const int c = classify_aabb(piece_bounds, plane.position, plane.normal);
    if (c > 0) {
      continue;
    }
    if (c < 0) {
      if (owned) {
        BKE_id_free(nullptr, owned);
      }
      return nullptr;
    }
    /* keep_below = non-positive side = site side */
    Mesh *clipped = geometry::mesh_clip_by_plane(
        current, plane.position, plane.normal, false, attribute_filter);
    if (owned) {
      BKE_id_free(nullptr, owned);
    }
    owned = clipped;
    if (owned == nullptr || owned->verts_num == 0) {
      if (owned) {
        BKE_id_free(nullptr, owned);
      }
      return nullptr;
    }
    if (const std::optional<Bounds<float3>> b = owned->bounds_min_max()) {
      piece_bounds = *b;
    }
  }
  if (owned == nullptr) {
    owned = BKE_mesh_copy_for_eval(src_mesh);
  }
  return owned;
}

/**
 * Fill open boundary loops after a plane clip (cut is planar).
 * Uses BMesh only as a hole-filler — never GEO_mesh_boolean / Manifold.
 * Cap faces on the plane are tagged inside so outer point normals stay intact.
 */
static Mesh *fill_boundary_holes(Mesh *mesh,
                                 const float3 &plane_position,
                                 const float3 &plane_normal)
{
  if (mesh == nullptr || mesh->verts_num == 0) {
    return mesh;
  }
  BMeshCreateParams create_params{};
  create_params.use_toolflags = true;
  BMeshFromMeshParams from_params{};
  from_params.calc_face_normal = true;
  /* Do not recompute vertex normals from topology — outer shading uses baked attrs. */
  from_params.calc_vert_normal = false;
  BMesh *bm = BKE_mesh_to_bmesh_ex(mesh, &create_params, &from_params);
  if (bm == nullptr) {
    return mesh;
  }

  BM_mesh_elem_hflag_disable_all(bm, BM_EDGE | BM_FACE, BM_ELEM_TAG, false);
  bool need = false;
  BMEdge *e;
  BMIter eiter;
  BM_ITER_MESH (e, &eiter, bm, BM_EDGES_OF_MESH) {
    if (BM_edge_is_boundary(e)) {
      BM_elem_flag_enable(e, BM_ELEM_TAG);
      need = true;
    }
  }
  if (need) {
    const int op_flag = (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE);
    BMOperator bmop_holes;
    BMO_op_initf(bm, &bmop_holes, op_flag, "holes_fill edges=%he sides=%i", BM_ELEM_TAG, 0);
    BMO_op_exec(bm, &bmop_holes);
    BMO_op_finish(bm, &bmop_holes);
  }

  Mesh *out = BKE_mesh_from_bmesh_for_eval_nomain(bm, nullptr, mesh);
  BM_mesh_free(bm);
  BKE_id_free(nullptr, mesh);
  if (out) {
    /* holes_fill may copy outside=true onto caps via face_attribute_fill — re-tag. */
    tag_cut_cap_faces(*out, plane_position, plane_normal);
  }
  return out;
}

/**
 * Solid cell: successive half-space clips (mesh_clip) + hole fill each cut.
 * No Blender boolean solvers. Planes from Voro++ only.
 */
static Mesh *fracture_solid_clip_fill(const Mesh &src_mesh,
                                      const Bounds<float3> &mesh_bounds,
                                      const Span<ClipPlane> planes,
                                      const bke::AttributeFilter &attribute_filter)
{
  if (src_mesh.verts_num == 0 || src_mesh.faces_num == 0) {
    return nullptr;
  }
  Mesh *owned = nullptr;
  Bounds<float3> piece_bounds = mesh_bounds;
  for (const ClipPlane &plane : planes) {
    const Mesh &current = owned ? *owned : src_mesh;
    const int c = classify_aabb(piece_bounds, plane.position, plane.normal);
    if (c > 0) {
      continue;
    }
    if (c < 0) {
      if (owned) {
        BKE_id_free(nullptr, owned);
      }
      return nullptr;
    }
    Mesh *clipped = geometry::mesh_clip_by_plane(
        current, plane.position, plane.normal, false, attribute_filter);
    if (owned) {
      BKE_id_free(nullptr, owned);
    }
    owned = clipped;
    if (owned == nullptr || owned->verts_num == 0) {
      if (owned) {
        BKE_id_free(nullptr, owned);
      }
      return nullptr;
    }
    owned = fill_boundary_holes(owned, plane.position, plane.normal);
    if (owned == nullptr || owned->faces_num == 0) {
      if (owned) {
        BKE_id_free(nullptr, owned);
      }
      return nullptr;
    }
    if (const std::optional<Bounds<float3>> b = owned->bounds_min_max()) {
      piece_bounds = *b;
    }
  }
  if (owned == nullptr) {
    owned = BKE_mesh_copy_for_eval(src_mesh);
  }
  return owned;
}

#ifdef WITH_VORO_PLUSPLUS

struct VoroPacket {
  bool valid = false;
  Vector<ClipPlane> planes;
};

static bool apply_gap(voro::voronoicell_neighbor &cell,
                      const Span<float3> sites,
                      const int site_i,
                      const float gap)
{
  if (gap <= 0.0f) {
    return true;
  }
  const float3 site = sites[site_i];
  for (const int j : sites.index_range()) {
    if (j == site_i) {
      continue;
    }
    const float3 d = sites[j] - site;
    const double dx = double(d.x), dy = double(d.y), dz = double(d.z);
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len <= 1.0e-14) {
      continue;
    }
    const double rsq = len * len - 2.0 * double(gap) * len;
    if (rsq <= 0.0) {
      return false;
    }
    if (!cell.nplane(dx, dy, dz, rsq, j)) {
      return false;
    }
  }
  return true;
}

static void extract_planes(voro::voronoicell_neighbor &cell,
                           const Span<float3> sites,
                           const int site_i,
                           const float gap,
                           Vector<ClipPlane> &r_planes)
{
  r_planes.clear();
  std::vector<int> neigh;
  cell.neighbors(neigh);
  const float3 site = sites[site_i];
  VectorSet<int> seen;
  for (const int nid : neigh) {
    if (nid < 0 || nid >= sites.size() || nid == site_i || !seen.add(nid)) {
      continue;
    }
    float nlen;
    const float3 n = math::normalize_and_get_length(sites[nid] - site, nlen);
    if (nlen <= 1.0e-12f) {
      continue;
    }
    const float3 mid = (site + sites[nid]) * 0.5f;
    r_planes.append({mid - n * gap, n});
  }
}

static Array<VoroPacket> compute_voro_packets(const Span<float3> sites,
                                              const Span<float> gaps,
                                              const Bounds<float3> &mesh_bounds)
{
  const int n = sites.size();
  Array<VoroPacket> packets(n);
  if (n == 0) {
    return packets;
  }

  float3 bmin = mesh_bounds.min;
  float3 bmax = mesh_bounds.max;
  for (const float3 &s : sites) {
    bmin = math::min(bmin, s);
    bmax = math::max(bmax, s);
  }
  const float pad = math::max(1.0e-4f, 1.0e-3f * math::length(bmax - bmin));
  bmin -= float3(pad);
  bmax += float3(pad);

  const float3 extent = bmax - bmin;
  const float emax = math::max(extent.x, math::max(extent.y, extent.z));
  int blocks = math::max(1, int(std::cbrt(std::max(1.0, double(n) / 5.0)) + 0.5));
  int nx = blocks, ny = blocks, nz = blocks;
  if (emax > 1.0e-12f) {
    nx = math::max(1, int(blocks * extent.x / emax + 0.5f));
    ny = math::max(1, int(blocks * extent.y / emax + 0.5f));
    nz = math::max(1, int(blocks * extent.z / emax + 0.5f));
  }

  voro::container con(double(bmin.x),
                      double(bmax.x),
                      double(bmin.y),
                      double(bmax.y),
                      double(bmin.z),
                      double(bmax.z),
                      nx,
                      ny,
                      nz,
                      false,
                      false,
                      false,
                      8);
  for (const int i : sites.index_range()) {
    con.put(i, double(sites[i].x), double(sites[i].y), double(sites[i].z));
  }

  voro::c_loop_all cl(con);
  voro::voronoicell_neighbor cell;
  if (cl.start()) {
    do {
      if (!con.compute_cell(cell, cl)) {
        continue;
      }
      const int pid = cl.pid();
      if (pid < 0 || pid >= n) {
        continue;
      }
      if (!apply_gap(cell, sites, pid, gaps[pid])) {
        continue;
      }
      VoroPacket &pk = packets[pid];
      extract_planes(cell, sites, pid, gaps[pid], pk.planes);
      pk.valid = !pk.planes.is_empty() || n == 1;
    } while (cl.inc());
  }
  return packets;
}

static Mesh *fracture_one_voro(const Mesh &src_mesh,
                               const Bounds<float3> &mesh_bounds,
                               const Span<float3> sites,
                               const int site_i,
                               const VoroPacket &packet,
                               const bool fill_inside,
                               const bke::AttributeFilter &attribute_filter)
{
  if (sites.size() == 1) {
    Mesh *copy = BKE_mesh_copy_for_eval(src_mesh);
    if (copy) {
      write_cell_index(*copy, site_i);
      write_face_side_attrs(*copy, true);
      /* Single cell is the original shell — drop bake, keep native normals. */
      copy->attributes_for_write().remove(vf_point_normal_id);
    }
    return copy;
  }
  if (!packet.valid) {
    return nullptr;
  }

  Mesh *owned = nullptr;
  if (fill_inside) {
    /* Half-space clip + cap fill. Never Blender GEO boolean / Manifold. */
    owned = fracture_solid_clip_fill(
        src_mesh, mesh_bounds, packet.planes, attribute_filter);
  }
  else {
    owned = fracture_open_clip(src_mesh, mesh_bounds, packet.planes, attribute_filter);
    if (owned) {
      write_face_side_attrs(*owned, true);
    }
  }
  if (owned) {
    write_cell_index(*owned, site_i);
    /* Outer points keep baked/original normals; cut walls get flat face normals. */
    apply_preserved_outer_normals(*owned);
  }
  return owned;
}

#endif /* WITH_VORO_PLUSPLUS */

static void node_geo_exec(GeoNodeExecParams params)
{
#ifndef WITH_VORO_PLUSPLUS
  params.error_message_add(
      NodeWarningType::Error,
      TIP_("Voronoi Fracture requires Voro++ (WITH_VORO_PLUSPLUS=ON)"));
  params.set_default_remaining_outputs();
  return;
#else
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry"_ustr);
  geometry_set.keep_only({GeometryComponent::Type::Mesh, GeometryComponent::Type::Edit});

  const GeometrySet points_geometry = params.extract_input<GeometrySet>("Points"_ustr);
  bke::SocketValueVariant gap_value = params.extract_input<bke::SocketValueVariant>("Gap"_ustr);
  const bool fill_inside = params.extract_input<bool>("Inside"_ustr);
  (void)params.extract_input<int>("Max Neighbors"_ustr);

  const Mesh *src_mesh = geometry_set.get_mesh();
  const Span<float3> sites = positions_from_points(points_geometry);
  if (src_mesh == nullptr || src_mesh->verts_num == 0 || sites.is_empty()) {
    params.set_output("Geometry"_ustr, GeometrySet());
    return;
  }
  const std::optional<Bounds<float3>> mesh_bounds_opt = src_mesh->bounds_min_max();
  if (!mesh_bounds_opt) {
    params.set_output("Geometry"_ustr, GeometrySet());
    return;
  }
  const Bounds<float3> mesh_bounds = *mesh_bounds_opt;
  const int sites_num = sites.size();
  const Array<float> gaps = evaluate_gap_on_points(points_geometry, gap_value, sites_num);
  const NodeAttributeFilter &attribute_filter = params.get_attribute_filter("Geometry"_ustr);

  /* Bake outer vertex normals once; clip propagates them to each cell. */
  Mesh *src_prepared = BKE_mesh_copy_for_eval(*src_mesh);
  bake_point_normals(*src_prepared);
  write_face_side_attrs(*src_prepared, true);

  /* 1) Voro++ exact neighbor planes only. */
  Array<VoroPacket> packets = compute_voro_packets(sites, gaps, mesh_bounds);

  /* 2) Per-site half-space clip (+ cap fill if solid). Parallel. */
  Array<Mesh *> cell_meshes(sites_num, nullptr);
  threading::parallel_for(IndexRange(sites_num), 1, [&](const IndexRange range) {
    for (const int site_i : range) {
      cell_meshes[site_i] = fracture_one_voro(*src_prepared,
                                              mesh_bounds,
                                              sites,
                                              site_i,
                                              packets[site_i],
                                              fill_inside,
                                              attribute_filter);
    }
  });

  BKE_id_free(nullptr, src_prepared);

  Vector<GeometrySet> pieces;
  pieces.reserve(sites_num);
  for (const int site_i : IndexRange(sites_num)) {
    Mesh *cell = cell_meshes[site_i];
    if (cell == nullptr || cell->verts_num == 0) {
      if (cell) {
        BKE_id_free(nullptr, cell);
      }
      continue;
    }
    pieces.append(GeometrySet::from_mesh(cell));
  }
  if (pieces.is_empty()) {
    params.set_output("Geometry"_ustr, GeometrySet());
    return;
  }

  const std::array types = {GeometryComponent::Type::Mesh};
  GeometrySet result = geometry::join_geometries(
      pieces, attribute_filter, std::make_optional<Span<GeometryComponent::Type>>(types));
  result.set_name(geometry_set.name());
  result.copy_bundle_from(geometry_set);
  params.set_output("Geometry"_ustr, std::move(result));
#endif
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeVoronoiFracture"_ustr, GEO_NODE_VORONOI_FRACTURE);
  ntype.ui_name = "Voronoi Fracture";
  ntype.ui_description =
      "Voronoi fracture: Voro++ planes + mesh half-space clip "
      "(no Blender boolean library); optional solid cap fill";
  ntype.enum_name_legacy = "VORONOI_FRACTURE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_voronoi_fracture_cc
