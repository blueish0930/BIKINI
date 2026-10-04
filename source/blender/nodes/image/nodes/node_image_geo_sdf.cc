/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Image Process: Geo SDF
 *
 * Geometry XY in [0,1] maps 1:1 onto the texture grid (no world projection).
 *
 * - Mesh that is a 3D solid: Geometry Clip at z=0, separate clip-boundary edges.
 *   Closed loops are filled (signed SDF); open chains are unsigned.
 * - Planar mesh: rasterize faces in XY (signed).
 * - Curves: project to XY; cyclic curves are filled (signed), open are unsigned.
 * - Point cloud and instances: treated as points in XY (unsigned).
 *
 * GPU: hardware raster occupancy → 1+JFA → UV-space distance.
 */

#include <cfloat>
#include <limits>
#include <utility>

#include "BLI_array.hh"
#include "BLI_delaunay_2d.hh"
#include "BLI_index_range.hh"
#include "BLI_math_base.hh"
#include "BLI_math_base_c.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "BLT_translation.hh"

#include "COM_algorithm_jump_flooding.hh"
#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "GEO_mesh_clip_plane.hh"

#include "GPU_batch.hh"
#include "GPU_framebuffer.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"
#include "GPU_vertex_buffer.hh"
#include "GPU_vertex_format.hh"

#include "NOD_image_points.hh"
#include "NOD_socket_declarations_geometry.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_geo_sdf_cc {

using namespace blender::compositor;
using namespace blender::nodes::image_points;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description(
          "Geometry from Import Geo. XY in [0,1] maps to the texture. "
          "3D solids are sliced at z=0; points/instances/curves use XY");

  b.add_output<decl::Float>("Distance"_ustr)
      .structure_type(StructureType::Dynamic)
      .description(
          "Signed distance to the occupancy mask boundary in unit-square UV ([0,1]²). "
          "Image diagonal is sqrt(2). Negative inside the mask");
  b.add_output<decl::Float>("Mask"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("1 inside filled regions (or on unsigned features), 0 elsewhere");
}

static bool bary_2d(const float2 p, const float2 a, const float2 b, const float2 c, float3 &r_w)
{
  const float2 v0 = b - a;
  const float2 v1 = c - a;
  const float2 v2 = p - a;
  const float den = v0.x * v1.y - v1.x * v0.y;
  if (math::abs(den) < 1e-12f) {
    return false;
  }
  const float inv = 1.0f / den;
  const float w1 = (v2.x * v1.y - v1.x * v2.y) * inv;
  const float w2 = (v0.x * v2.y - v2.x * v0.y) * inv;
  const float w0 = 1.0f - w1 - w2;
  if (w0 < -1e-5f || w1 < -1e-5f || w2 < -1e-5f) {
    return false;
  }
  r_w = float3(w0, w1, w2);
  return true;
}

static float2 uv_to_ndc(const float2 uv)
{
  return uv * 2.0f - float2(1.0f);
}

/* -------------------------------------------------------------------- */
/*  Primitive collection (UV [0,1] space)                               */
/* -------------------------------------------------------------------- */

struct GeoSDFPrims {
  Vector<float2> fill_tris;
  Vector<float2> segs;
  Vector<float2> points;

  bool has_fill() const
  {
    return !fill_tris.is_empty();
  }
  bool is_empty() const
  {
    return fill_tris.is_empty() && segs.is_empty() && points.is_empty();
  }
};

static void append_tri(Vector<float2> &tris, const float2 a, const float2 b, const float2 c)
{
  tris.append(a);
  tris.append(b);
  tris.append(c);
}

static void append_filled_loop_xy(const Span<float2> loop, Vector<float2> &fill_tris)
{
  const int n = int(loop.size());
  if (n < 3) {
    return;
  }

  Array<double2> verts(n);
  Array<int> face_verts(n);
  Array<int> offs(2);
  offs[0] = 0;
  offs[1] = n;
  for (int i = 0; i < n; i++) {
    verts[i] = double2(double(loop[i].x), double(loop[i].y));
    face_verts[i] = i;
  }

  meshintersect::CDT_input<double> input;
  input.vert = verts;
  input.face_offsets = OffsetIndices<int>(offs.as_span());
  input.face_vert_indices = face_verts;
  const meshintersect::CDT_result<double> result = meshintersect::delaunay_2d_calc(
      input, CDT_INSIDE);

  const int64_t before = fill_tris.size();
  for (const Vector<int> &face : result.face) {
    if (face.size() < 3) {
      continue;
    }
    const float2 a(float(result.vert[face[0]].x), float(result.vert[face[0]].y));
    for (int i = 1; i + 1 < face.size(); i++) {
      append_tri(fill_tris,
                 a,
                 float2(float(result.vert[face[i]].x), float(result.vert[face[i]].y)),
                 float2(float(result.vert[face[i + 1]].x), float(result.vert[face[i + 1]].y)));
    }
  }
  if (fill_tris.size() == before) {
    const float2 a = loop[0];
    for (int i = 1; i + 1 < n; i++) {
      append_tri(fill_tris, a, loop[i], loop[i + 1]);
    }
  }
}

static void append_mesh_faces_xy(const Mesh &mesh, Vector<float2> &fill_tris)
{
  const Span<float3> pos = mesh.vert_positions();
  const Span<int> cv = mesh.corner_verts();
  const Span<int3> tris = mesh.corner_tris();
  fill_tris.reserve(fill_tris.size() + tris.size() * 3);
  for (const int3 t : tris) {
    append_tri(fill_tris,
               float2(pos[cv[t[0]]].x, pos[cv[t[0]]].y),
               float2(pos[cv[t[1]]].x, pos[cv[t[1]]].y),
               float2(pos[cv[t[2]]].x, pos[cv[t[2]]].y));
  }
}

static bool mesh_is_3d_solid(const Mesh &mesh)
{
  if (mesh.faces_num == 0 || mesh.verts_num == 0) {
    return false;
  }
  const Span<float3> pos = mesh.vert_positions();
  float zmin = FLT_MAX;
  float zmax = -FLT_MAX;
  for (const float3 &p : pos) {
    zmin = math::min(zmin, p.z);
    zmax = math::max(zmax, p.z);
  }
  return (zmax - zmin) > 1.0e-4f;
}

static void walk_contour_polylines(const Span<int2> contour_edges,
                                   const int verts_num,
                                   Vector<Vector<int>> &r_closed,
                                   Vector<Vector<int>> &r_open)
{
  if (contour_edges.is_empty()) {
    return;
  }

  struct Adj {
    int vert;
    int edge_i;
  };
  Vector<Vector<Adj>> adj(verts_num);
  for (const int i : contour_edges.index_range()) {
    const int2 e = contour_edges[i];
    adj[e.x].append({e.y, i});
    adj[e.y].append({e.x, i});
  }

  Array<int> degree(verts_num, 0);
  for (const int2 e : contour_edges) {
    degree[e.x]++;
    degree[e.y]++;
  }

  Array<bool> used(contour_edges.size(), false);

  auto take_unused = [&](const int v, const int prev) -> int {
    for (const Adj &a : adj[v]) {
      if (!used[a.edge_i] && a.vert != prev) {
        used[a.edge_i] = true;
        return a.vert;
      }
    }
    return -1;
  };

  auto walk_from = [&](const int start, const bool prefer_closed) {
    Vector<int> chain;
    chain.append(start);
    int prev = -1;
    int cur = start;
    while (true) {
      const int next = take_unused(cur, prev);
      if (next < 0) {
        break;
      }
      chain.append(next);
      prev = cur;
      cur = next;
      if (prefer_closed && next == start && chain.size() > 2) {
        break;
      }
    }
    if (chain.size() >= 2 && chain.first() == chain.last()) {
      chain.pop_last();
      if (chain.size() >= 3) {
        r_closed.append(std::move(chain));
        return;
      }
    }
    if (chain.size() >= 2) {
      r_open.append(std::move(chain));
    }
  };

  /* Open chains first (degree-1 starts). */
  for (int v = 0; v < verts_num; v++) {
    bool has_unused = false;
    for (const Adj &a : adj[v]) {
      if (!used[a.edge_i]) {
        has_unused = true;
        break;
      }
    }
    if (!has_unused) {
      continue;
    }
    if (degree[v] == 1) {
      walk_from(v, false);
    }
  }
  /* Remaining closed loops. */
  for (int v = 0; v < verts_num; v++) {
    bool has_unused = false;
    for (const Adj &a : adj[v]) {
      if (!used[a.edge_i]) {
        has_unused = true;
        break;
      }
    }
    if (has_unused) {
      walk_from(v, true);
    }
  }
}

static void collect_clip_contour_edges(const Mesh &mesh,
                                       const Span<bool> clip_boundary,
                                       Vector<int2> &r_edges)
{
  const Span<int2> edges = mesh.edges();
  const Span<int> corner_edges = mesh.corner_edges();
  const OffsetIndices faces = mesh.faces();
  Array<int> face_count(edges.size(), 0);
  for (const int face_i : faces.index_range()) {
    for (const int corner : faces[face_i]) {
      face_count[corner_edges[corner]]++;
    }
  }
  for (const int i : edges.index_range()) {
    const int2 e = edges[i];
    if (face_count[i] == 1 && clip_boundary[e.x] && clip_boundary[e.y]) {
      r_edges.append(e);
    }
  }
}

static void add_mesh_prims(const Mesh &mesh, GeoSDFPrims &prims)
{
  if (mesh.verts_num == 0) {
    return;
  }
  const Span<float3> pos = mesh.vert_positions();

  if (mesh.faces_num == 0) {
    if (mesh.edges_num > 0) {
      for (const int2 e : mesh.edges()) {
        prims.segs.append(float2(pos[e.x].x, pos[e.x].y));
        prims.segs.append(float2(pos[e.y].x, pos[e.y].y));
      }
    }
    else {
      for (const float3 &p : pos) {
        prims.points.append(float2(p.x, p.y));
      }
    }
    return;
  }

  if (!mesh_is_3d_solid(mesh)) {
    append_mesh_faces_xy(mesh, prims.fill_tris);
    return;
  }

  /* 3D solid: Geometry Clip at z=0, then separate clip-boundary edges. */
  geometry::MeshPlaneClipResult clipped = geometry::mesh_clip_by_plane_both(
      mesh, float3(0.0f, 0.0f, 0.0f), float3(0.0f, 0.0f, 1.0f), true, false);
  if (clipped.below) {
    BKE_id_free(nullptr, clipped.below);
  }
  Mesh *slice = clipped.above;
  if (!slice) {
    return;
  }

  Vector<int2> contour;
  if (clipped.above_clip_boundary.size() == slice->verts_num) {
    collect_clip_contour_edges(*slice, clipped.above_clip_boundary, contour);
  }

  Vector<Vector<int>> closed_loops;
  Vector<Vector<int>> open_chains;
  walk_contour_polylines(contour, slice->verts_num, closed_loops, open_chains);

  const Span<float3> spos = slice->vert_positions();
  for (const Vector<int> &loop : closed_loops) {
    Vector<float2> xy;
    xy.reserve(loop.size());
    for (const int v : loop) {
      xy.append(float2(spos[v].x, spos[v].y));
    }
    append_filled_loop_xy(xy, prims.fill_tris);
  }
  for (const Vector<int> &chain : open_chains) {
    for (int i = 0; i + 1 < chain.size(); i++) {
      prims.segs.append(float2(spos[chain[i]].x, spos[chain[i]].y));
      prims.segs.append(float2(spos[chain[i + 1]].x, spos[chain[i + 1]].y));
    }
  }

  BKE_id_free(nullptr, slice);
}

static void add_curves_prims(const bke::CurvesGeometry &curves, GeoSDFPrims &prims)
{
  const OffsetIndices points_by_curve = curves.evaluated_points_by_curve();
  const Span<float3> positions = curves.evaluated_positions();
  const VArray<bool> cyclic = curves.cyclic();
  for (const int curve_i : points_by_curve.index_range()) {
    const IndexRange pts = points_by_curve[curve_i];
    if (pts.size() < 2) {
      if (pts.size() == 1) {
        prims.points.append(float2(positions[pts[0]].x, positions[pts[0]].y));
      }
      continue;
    }
    const bool is_cyclic = cyclic[curve_i] && pts.size() >= 3;
    if (is_cyclic) {
      Vector<float2> loop;
      loop.reserve(pts.size());
      for (const int i : pts) {
        loop.append(float2(positions[i].x, positions[i].y));
      }
      append_filled_loop_xy(loop, prims.fill_tris);
    }
    else {
      const int segs = bke::curves::segments_num(int(pts.size()), cyclic[curve_i]);
      for (int i = 0; i < segs; i++) {
        const int a = pts[i];
        const int b = pts[(i + 1) % pts.size()];
        prims.segs.append(float2(positions[a].x, positions[a].y));
        prims.segs.append(float2(positions[b].x, positions[b].y));
      }
    }
  }
}

static void collect_prims(const bke::GeometrySet &geometry, GeoSDFPrims &prims)
{
  if (const Mesh *mesh = geometry.get_mesh()) {
    add_mesh_prims(*mesh, prims);
  }
  if (const Curves *curves_id = geometry.get_curves()) {
    add_curves_prims(curves_id->geometry.wrap(), prims);
  }
  if (const PointCloud *points = geometry.get_pointcloud()) {
    for (const float3 &p : points->positions()) {
      prims.points.append(float2(p.x, p.y));
    }
  }
  if (const bke::Instances *instances = geometry.get_instances()) {
    for (const float4x4 &tf : instances->transforms()) {
      const float3 loc = tf.location();
      prims.points.append(float2(loc.x, loc.y));
    }
  }
}

static void append_seg_quad_ndc(const float2 a_uv,
                                const float2 b_uv,
                                const float2 half_px_ndc,
                                Vector<float2> &ndc_tris)
{
  const float2 a = uv_to_ndc(a_uv);
  const float2 b = uv_to_ndc(b_uv);
  float2 dir = b - a;
  const float len2 = math::length_squared(dir);
  float2 n;
  if (len2 < 1e-20f) {
    n = float2(0.0f, 1.0f);
  }
  else {
    dir *= (1.0f / math::sqrt(len2));
    n = float2(-dir.y, dir.x);
  }
  const float w = math::max(half_px_ndc.x, half_px_ndc.y);
  const float2 o = n * w;
  append_tri(ndc_tris, a + o, a - o, b + o);
  append_tri(ndc_tris, a - o, b - o, b + o);
}

static void append_point_diamond_ndc(const float2 p_uv,
                                     const float2 half_px_ndc,
                                     Vector<float2> &ndc_tris)
{
  const float2 p = uv_to_ndc(p_uv);
  const float2 rx(half_px_ndc.x, 0.0f);
  const float2 ry(0.0f, half_px_ndc.y);
  append_tri(ndc_tris, p + rx, p + ry, p - rx);
  append_tri(ndc_tris, p + ry, p - rx, p - ry);
}

static Vector<float2> prims_to_ndc_tris(const GeoSDFPrims &prims, const int2 res)
{
  Vector<float2> ndc;
  ndc.reserve(prims.fill_tris.size() + prims.segs.size() * 3 + prims.points.size() * 6);
  for (const float2 &uv : prims.fill_tris) {
    ndc.append(uv_to_ndc(uv));
  }
  const float2 half_px_ndc(1.0f / float(res.x), 1.0f / float(res.y));
  for (int64_t i = 0; i + 1 < prims.segs.size(); i += 2) {
    append_seg_quad_ndc(prims.segs[i], prims.segs[i + 1], half_px_ndc, ndc);
  }
  for (const float2 &p : prims.points) {
    append_point_diamond_ndc(p, half_px_ndc, ndc);
  }
  return ndc;
}

/* -------------------------------------------------------------------- */
/*  COM evaluation                                                      */
/* -------------------------------------------------------------------- */

class GeoSDFOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    Result &dist_out = this->get_result("Distance");
    const bool need_dist = dist_out.should_compute();
    const bool need_mask = this->has_result("Mask") && this->get_result("Mask").should_compute();
    if (!need_dist && !need_mask) {
      return;
    }

    const Domain domain = this->compute_domain();
    const int2 res = math::max(domain.data_size, int2(1));
    const float2 pixel_size(1.0f / float(res.x), 1.0f / float(res.y));

    const bke::GeometrySet *imported = nullptr;
    if (this->has_input("Geometry")) {
      Result &geo_in = this->get_input("Geometry");
      if (geo_in.type() == ResultType::String && geo_in.is_single_value()) {
        const std::string key = geo_in.get_single_value_default<std::string>();
        if (PointsGeometryCache *cache = active_cache()) {
          imported = cache->lookup(key);
        }
      }
    }

    GeoSDFPrims prims;
    if (imported) {
      collect_prims(*imported, prims);
    }
    const Vector<float2> ndc_tris = prims_to_ndc_tris(prims, res);

    if (this->context().use_gpu() &&
        GPU_shader_create_info_get("compositor_geo_sdf_raster") != nullptr)
    {
      if (gpu::Shader *raster = this->context().get_shader("compositor_geo_sdf_raster")) {
        if (this->execute_gpu(raster, ndc_tris, domain, need_dist, need_mask))
        {
          return;
        }
      }
    }

    this->execute_cpu(ndc_tris, domain, res, pixel_size, need_dist, need_mask);
  }

 private:
  bool execute_gpu(gpu::Shader *raster_shader,
                   const Span<float2> ndc_tris,
                   const Domain &domain,
                   const bool need_dist,
                   const bool need_mask)
  {
    Result occupancy = this->context().create_result(ResultType::Color);
    occupancy.allocate_texture(domain, true, ResultStorageType::GPUImage);
    if (!occupancy.gpu_texture()) {
      occupancy.release();
      return false;
    }

    gpu::FrameBuffer *fb = nullptr;
    GPU_framebuffer_ensure_config(&fb,
                                  {
                                      GPU_ATTACHMENT_NONE,
                                      GPU_ATTACHMENT_TEXTURE(occupancy.gpu_texture()),
                                  });
    GPU_framebuffer_bind(fb);
    GPU_framebuffer_clear_color(fb, double4(0.0, 0.0, 0.0, 0.0));

    if (!ndc_tris.is_empty()) {
      GPUVertFormat format = {0};
      const uint a_ndc = GPU_vertformat_attr_add(
          &format, "pos_ndc", gpu::VertAttrType::SFLOAT_32_32);

      gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
      GPU_vertbuf_data_alloc(*vbo, uint(ndc_tris.size()));
      GPU_vertbuf_attr_fill(vbo, a_ndc, ndc_tris.data());

      gpu::Batch *batch = GPU_batch_create_ex(GPU_PRIM_TRIS, vbo, nullptr, GPU_BATCH_OWNS_VBO);

      const GPUDepthTest prev_depth = GPU_depth_test_get();
      const GPUFaceCullTest prev_cull = GPU_face_culling_get();
      const GPUBlend prev_blend = GPU_blend_get();
      GPU_depth_test(GPU_DEPTH_NONE);
      GPU_face_culling(GPU_CULL_NONE);
      GPU_blend(GPU_BLEND_NONE);

      GPU_batch_set_shader(batch, raster_shader);
      GPU_batch_draw(batch);
      GPU_shader_unbind();

      GPU_depth_test(prev_depth);
      GPU_face_culling(prev_cull);
      GPU_blend(prev_blend);
      GPU_batch_discard(batch);
    }

    GPU_framebuffer_restore();
    GPU_framebuffer_free(fb);
    GPU_memory_barrier(GPUBarrier(GPU_BARRIER_FRAMEBUFFER | GPU_BARRIER_TEXTURE_FETCH |
                                  GPU_BARRIER_SHADER_IMAGE_ACCESS));

    Result mask = this->color_to_bool(occupancy);
    occupancy.release();
    if (!mask.is_allocated() || mask.is_single_value()) {
      mask.release();
      return false;
    }

    if (need_mask) {
      this->bool_to_float_gpu(mask, this->get_result("Mask"));
    }

    if (need_dist) {
      if (!this->compute_distance_from_mask_gpu(mask)) {
        mask.release();
        return false;
      }
    }

    mask.release();
    return true;
  }

  Result color_to_bool(const Result &occupancy)
  {
    Result mask = this->context().create_result(ResultType::Bool);
    gpu::Shader *shader = this->context().get_shader("compositor_convert_color_to_bool");
    if (!shader) {
      mask.allocate_invalid();
      return mask;
    }
    GPU_shader_bind(shader);
    const float luma[3] = {1.0f, 0.0f, 0.0f};
    GPU_shader_uniform_3fv(shader, "luminance_coefficients_u", luma);
    occupancy.bind_as_texture(shader, "input_tx");
    mask.allocate_texture(occupancy.domain());
    mask.bind_as_image(shader, "output_img");
    compute_dispatch_threads_at_least(shader, occupancy.domain().data_size);
    occupancy.unbind_as_texture();
    mask.unbind_as_image();
    GPU_shader_unbind();
    GPU_memory_barrier(GPUBarrier(GPU_BARRIER_SHADER_IMAGE_ACCESS | GPU_BARRIER_TEXTURE_FETCH));
    return mask;
  }

  void bool_to_float_gpu(const Result &mask, Result &out)
  {
    gpu::Shader *shader = this->context().get_shader("compositor_convert_bool_to_float");
    if (!shader) {
      out.allocate_invalid();
      return;
    }
    GPU_shader_bind(shader);
    mask.bind_as_texture(shader, "input_tx");
    out.allocate_texture(mask.domain());
    out.bind_as_image(shader, "output_img");
    compute_dispatch_threads_at_least(shader, mask.domain().data_size);
    mask.unbind_as_texture();
    out.unbind_as_image();
    GPU_shader_unbind();
  }

  bool compute_distance_from_mask_gpu(const Result &mask)
  {
    /* Identical to Mask To SDF: boundary seeds → 1+JFA → unit-square UV distance. */
    gpu::Shader *boundary_shader = this->context().get_shader(
        "compositor_mask_to_sdf_compute_boundary", ResultPrecision::Half);
    gpu::Shader *distance_shader = this->context().get_shader(
        "compositor_mask_to_sdf_compute_distance");
    gpu::Shader *jfa_shader = this->context().get_shader("compositor_jump_flooding",
                                                         ResultPrecision::Half);
    if (!boundary_shader || !distance_shader || !jfa_shader) {
      return false;
    }

    GPU_memory_barrier(GPUBarrier(GPU_BARRIER_SHADER_IMAGE_ACCESS | GPU_BARRIER_TEXTURE_FETCH));

    Result boundary = this->context().create_result(ResultType::Int2, ResultPrecision::Half);
    GPU_shader_bind(boundary_shader);
    mask.bind_as_texture(boundary_shader, "mask_tx");
    boundary.allocate_texture(mask.domain());
    boundary.bind_as_image(boundary_shader, "boundary_img");
    compute_dispatch_threads_at_least(boundary_shader, mask.domain().data_size);
    mask.unbind_as_texture();
    boundary.unbind_as_image();
    GPU_shader_unbind();
    GPU_memory_barrier(GPUBarrier(GPU_BARRIER_SHADER_IMAGE_ACCESS | GPU_BARRIER_TEXTURE_FETCH));

    Result flooded = this->context().create_result(ResultType::Int2, ResultPrecision::Half);
    jump_flooding(this->context(), boundary, flooded);
    boundary.release();
    if (!flooded.is_allocated() || flooded.is_single_value()) {
      flooded.release();
      return false;
    }
    GPU_memory_barrier(GPUBarrier(GPU_BARRIER_SHADER_IMAGE_ACCESS | GPU_BARRIER_TEXTURE_FETCH));

    Result &dist_out = this->get_result("Distance");
    GPU_shader_bind(distance_shader);
    mask.bind_as_texture(distance_shader, "mask_tx");
    flooded.bind_as_texture(distance_shader, "flooded_boundary_tx");
    dist_out.allocate_texture(mask.domain());
    dist_out.bind_as_image(distance_shader, "distance_img");
    compute_dispatch_threads_at_least(distance_shader, mask.domain().data_size);
    mask.unbind_as_texture();
    flooded.unbind_as_texture();
    dist_out.unbind_as_image();
    GPU_shader_unbind();

    flooded.release();
    return true;
  }

  void execute_cpu(const Span<float2> ndc_tris,
                   const Domain &domain,
                   const int2 res,
                   const float2 pixel_size,
                   const bool need_dist,
                   const bool need_mask)
  {
    Result mask = this->context().create_result(ResultType::Bool);
    mask.allocate_texture(domain, false, ResultStorageType::CPUImage);
    parallel_for(res, [&](const int2 texel) { mask.store_pixel(texel, false); });

    const float2 res_f(float(res.x), float(res.y));
    for (int64_t i = 0; i + 2 < ndc_tris.size(); i += 3) {
      const float2 p0 = (ndc_tris[i] * 0.5f + 0.5f) * res_f;
      const float2 p1 = (ndc_tris[i + 1] * 0.5f + 0.5f) * res_f;
      const float2 p2 = (ndc_tris[i + 2] * 0.5f + 0.5f) * res_f;
      this->raster_tri_occupancy(mask, res, p0, p1, p2);
    }

    if (need_mask) {
      Result &out = this->get_result("Mask");
      Result cpu = this->context().create_result(ResultType::Float);
      cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
      parallel_for(res, [&](const int2 texel) {
        cpu.store_pixel(texel, mask.load_pixel<bool>(texel) ? 1.0f : 0.0f);
      });
      this->publish_cpu_or_upload(cpu, out);
    }

    if (need_dist) {
      Result boundary = this->compute_boundary_cpu(mask);
      Result flooded = this->context().create_result(ResultType::Int2, ResultPrecision::Half);
      flooded.allocate_texture(domain, false, ResultStorageType::CPUImage);
      this->jump_flooding_cpu(boundary, flooded);
      boundary.release();

      Result cpu = this->context().create_result(ResultType::Float);
      cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
      this->store_distance_cpu(mask, flooded, cpu, pixel_size);
      this->publish_cpu_or_upload(cpu, this->get_result("Distance"));
      flooded.release();
    }

    mask.release();
  }

  void raster_tri_occupancy(
      Result &mask, const int2 res, const float2 p0, const float2 p1, const float2 p2)
  {
    const float2 lo = math::min(math::min(p0, p1), p2);
    const float2 hi = math::max(math::max(p0, p1), p2);
    const int2 b0 = math::clamp(int2(lo), int2(0), res - 1);
    const int2 b1 = math::clamp(int2(math::ceil(hi)), int2(0), res - 1);
    for (int y = b0.y; y <= b1.y; y++) {
      for (int x = b0.x; x <= b1.x; x++) {
        float3 w;
        if (!bary_2d(float2(x + 0.5f, y + 0.5f), p0, p1, p2, w)) {
          continue;
        }
        mask.store_pixel(int2(x, y), true);
      }
    }
  }

  void store_distance_cpu(const Result &mask,
                          const Result &flooded,
                          Result &dist_cpu,
                          const float2 pixel_size)
  {
    const int2 size = mask.domain().data_size;
    const float fallback = math::sqrt(2.0f);
    parallel_for(size, [&](const int2 texel) {
      const bool is_inside = mask.load_pixel<bool>(texel);
      const int2 closest = flooded.load_pixel<int2>(texel);
      float d;
      if (closest.x < 0) {
        d = fallback;
      }
      else {
        d = math::length((float2(closest) - float2(texel)) * pixel_size);
      }
      dist_cpu.store_pixel(texel, is_inside ? -d : d);
    });
  }

  void jump_flooding_pass_cpu(const Result &input, Result &output, const int step_size)
  {
    parallel_for(input.domain().data_size, [&](const int2 texel) {
      int2 closest_seed_texel = int2(0);
      float minimum_squared_distance = std::numeric_limits<float>::max();
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          const int2 offset = int2(i, j) * step_size;
          const int2 jump_flooding_value = input.load_pixel_fallback(
              texel + offset, JUMP_FLOODING_NON_FLOODED_VALUE);
          if (jump_flooding_value == JUMP_FLOODING_NON_FLOODED_VALUE) {
            continue;
          }
          const float squared_distance = math::distance_squared(float2(jump_flooding_value),
                                                                float2(texel));
          if (squared_distance < minimum_squared_distance) {
            minimum_squared_distance = squared_distance;
            closest_seed_texel = jump_flooding_value;
          }
        }
      }
      const bool flooding_happened = minimum_squared_distance !=
                                     std::numeric_limits<float>::max();
      output.store_pixel(texel,
                         encode_jump_flooding_value(closest_seed_texel, flooding_happened));
    });
  }

  void jump_flooding_cpu(Result &input, Result &output)
  {
    Result a = this->context().create_result(ResultType::Int2, ResultPrecision::Half);
    a.allocate_texture(input.domain(), false, ResultStorageType::CPUImage);
    this->jump_flooding_pass_cpu(input, a, 1);

    Result b = this->context().create_result(ResultType::Int2, ResultPrecision::Half);
    b.allocate_texture(input.domain(), false, ResultStorageType::CPUImage);

    Result *from = &a;
    Result *to = &b;
    const int max_size = math::max(input.domain().data_size.x, input.domain().data_size.y);
    int step_size = power_of_2_max_i(max_size) / 2;
    while (step_size != 0) {
      this->jump_flooding_pass_cpu(*from, *to, step_size);
      std::swap(from, to);
      step_size /= 2;
    }

    parallel_for(input.domain().data_size, [&](const int2 texel) {
      output.store_pixel(texel, from->load_pixel<int2>(texel));
    });
    a.release();
    b.release();
  }

  Result compute_boundary_cpu(const Result &mask)
  {
    Result boundary = this->context().create_result(ResultType::Int2, ResultPrecision::Half);
    boundary.allocate_texture(mask.domain(), false, ResultStorageType::CPUImage);
    const int2 size = mask.domain().data_size;
    parallel_for(size, [&](const int2 texel) {
      bool has_unmasked_neighbors = false;
      for (int j = -1; j <= 1; j++) {
        for (int i = -1; i <= 1; i++) {
          const int2 offset(i, j);
          if (offset == int2(0)) {
            continue;
          }
          if (!mask.load_pixel_extended<bool>(texel + offset)) {
            has_unmasked_neighbors = true;
            break;
          }
        }
        if (has_unmasked_neighbors) {
          break;
        }
      }
      const bool is_masked = mask.load_pixel<bool>(texel);
      const bool is_boundary = is_masked && has_unmasked_neighbors;
      boundary.store_pixel(texel, initialize_jump_flooding_value(texel, is_boundary));
    });
    return boundary;
  }

  void publish_cpu_or_upload(Result &cpu, Result &out)
  {
    if (this->context().use_gpu()) {
      Result gpu = cpu.upload_to_gpu(true);
      out.share_data(gpu);
      gpu.release();
    }
    else {
      out.share_data(cpu);
    }
    cpu.release();
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new GeoSDFOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeGeoSDF"_ustr, IMG_NODE_GEO_SDF);
  ntype.ui_name = "Geo SDF";
  ntype.ui_description =
      "Compute a per-pixel SDF on the 0–1 XY texture grid. 3D solids are sliced at z=0; "
      "closed contours are filled (signed), open curves and points are unsigned";
  ntype.nclass = NODE_CLASS_TEXTURE;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_geo_sdf_cc
