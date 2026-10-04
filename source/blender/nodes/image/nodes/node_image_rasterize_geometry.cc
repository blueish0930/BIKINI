/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Image Process: Rasterize Geometry
 * Outputs Position (Vector), Normal (Vector), Alpha (Float), and Attributes (Bundle).
 * Primary path: GPU hardware triangle rasterization (VBO + depth + MRT).
 * CPU path is fallback only when GPU is unavailable. */

#include <cctype>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>

#include "BLI_color.hh"
#include "BLI_hash.hh"
#include "BLI_map.hh"
#include "BLI_set.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"
#include "BLI_string_utf8.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"

#include "MEM_guardedalloc.h"

#include "BKE_anonymous_attribute_id.hh"
#include "BKE_attribute.hh"
#include "BKE_attribute_legacy_convert.hh"
#include "BKE_context.hh"
#include "BKE_global.hh"
#include "BKE_layer.hh"
#include "BKE_main.hh"
#include "BKE_mesh.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_object.hh"
#include "BKE_scene.hh"

#include "BLT_translation.hh"

#include "COM_algorithm_extract_alpha.hh"
#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "GPU_batch.hh"
#include "GPU_framebuffer.hh"
#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"
#include "GPU_vertex_buffer.hh"
#include "GPU_vertex_format.hh"

#include "ED_screen.hh"
#include "ED_undo.hh"

#include "RE_pipeline.h"
#include "render_types.h"

#include "NOD_geo_bundle.hh"
#include "NOD_geometry_nodes_bundle.hh"
#include "NOD_image_points.hh"
#include "NOD_socket_declarations.hh"
#include "NOD_socket_declarations_geometry.hh"
#include "NOD_socket_items.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_rasterize_geometry_cc {

using namespace blender::compositor;
using namespace blender::nodes::image_points;

enum class RasterizeMode : int { World = 0, UV = 1 };

static const EnumPropertyItem mode_items[] = {
    {int(RasterizeMode::World),
     "WORLD",
     0,
     N_("World Space"),
     N_("Orthographic projection along a world-space ray")},
    {int(RasterizeMode::UV),
     "UV",
     0,
     N_("UV Space"),
     N_("Unwrap the mesh onto the image using a UV map")},
    {0, nullptr, 0, nullptr, nullptr},
};

/* -------------------------------------------------------------------- */
/*  Attribute-name helpers                                              */
/* -------------------------------------------------------------------- */

static Vector<std::string> parse_attribute_list(const StringRef text)
{
  Vector<std::string> names;
  std::string cur;
  auto flush = [&]() {
    size_t s = 0, e = cur.size();
    while (s < e && std::isspace(uchar(cur[s]))) { s++; }
    while (e > s && std::isspace(uchar(cur[e - 1]))) { e--; }
    if (e > s) {
      const std::string n = cur.substr(s, e - s);
      if (!ELEM(StringRef(n), "p", "n", "a", "Type", "Bundle",
                "position", "normal", "alpha"))
      { names.append(n); }
    }
    cur.clear();
  };
  for (char c : text) {
    if (c == ',' || std::isspace(uchar(c))) { flush(); }
    else { cur.push_back(c); }
  }
  flush();
  return names;
}

static std::string attributes_string_from_node(const bNode &node)
{
  for (const bNodeSocket *s : node.input_sockets()) {
    if (!s || !s->is_available() || StringRef(s->name) != "Attributes") { continue; }
    if (s->type == SOCK_STRING) {
      return s->default_value_typed<bNodeSocketValueString>()->value;
    }
    return {};
  }
  return {};
}

static const Object *object_from_node(const bNode &node)
{
  for (const bNodeSocket *s : node.input_sockets()) {
    if (s && s->is_available() && s->type == SOCK_OBJECT && StringRef(s->name) == "Object") {
      return s->default_value_typed<bNodeSocketValueObject>()->value;
    }
  }
  return nullptr;
}

static bool geometry_input_is_linked(const bNode &node)
{
  /* File-load / declare can run before topology cache wires owner_tree. */
  if (!node.runtime || !node.runtime->owner_tree || !node.runtime->owner_tree->runtime) {
    return false;
  }
  node.runtime->owner_tree->ensure_topology_cache();
  for (const bNodeSocket *s : node.input_sockets()) {
    if (!s || !s->is_available() || s->type != SOCK_GEOMETRY) {
      continue;
    }
    for (const bNodeLink *link : s->directly_linked_links()) {
      if (link && link->is_used() && !link->is_muted()) {
        return true;
      }
    }
  }
  return false;
}

/** Last cooked Import Geo / Import Points geometry linked into Rasterize Geometry. */
static const bke::GeometrySet *imported_geometry_from_node(const bNode &node)
{
  /* owner_tree() requires a live topology cache; during liblink it is still null
   * and crashed V1 on read (ACCESS_VIOLATION 0x2E8). */
  if (!node.runtime || !node.runtime->owner_tree || !node.runtime->owner_tree->runtime) {
    return nullptr;
  }
  node.runtime->owner_tree->ensure_topology_cache();
  for (const bNodeSocket *input : node.input_sockets()) {
    if (!input || input->type != SOCK_GEOMETRY || !input->is_available()) {
      continue;
    }
    for (const bNodeLink *link : input->directly_linked_links()) {
      if (!link || !link->is_used() || !link->fromnode) {
        continue;
      }
      const bNode &from = *link->fromnode;
      if (!from.id || GS(from.id->name) != ID_NT) {
        continue;
      }
      if (!from.is_type("ImageNodeImportGeo"_ustr) &&
          !from.is_type("ImageNodeImportPoints"_ustr))
      {
        continue;
      }
      const bNodeTree &geo_tree = *reinterpret_cast<const bNodeTree *>(from.id);
      if (geo_tree.type != NTREE_GEOMETRY) {
        continue;
      }
      if (const bke::GeometrySet *last = lookup_last_import_geo_geometry(geo_tree)) {
        return last;
      }
      if (const bke::GeometrySet *last = lookup_last_import_points_geometry(geo_tree)) {
        return last;
      }
    }
  }
  return nullptr;
}

/* -------------------------------------------------------------------- */
/*  Attribute‑type mapping (Float / Vector / Color)                     */
/* -------------------------------------------------------------------- */

static eNodeSocketDatatype socket_type_for_attrtype(const bke::AttrType type)
{
  switch (type) {
    case bke::AttrType::Float:
    case bke::AttrType::Int8:
    case bke::AttrType::Int32:
    case bke::AttrType::Bool:   return SOCK_FLOAT;
    case bke::AttrType::Float2:
    case bke::AttrType::Float3:  return SOCK_VECTOR;
    default:                     return SOCK_RGBA; /* Color, Float4, Quat, Matrix */
  }
}

static std::optional<bke::AttrType> attrtype_from_mesh(const Mesh *mesh, const StringRef name)
{
  if (!mesh) {
    return std::nullopt;
  }
  const auto meta = mesh->attributes().lookup_meta_data(name);
  return meta ? std::optional(meta->data_type) : std::nullopt;
}

static std::optional<bke::AttrType> attrtype_from_object(const Object *obj,
                                                          const StringRef name,
                                                          Depsgraph *dg = nullptr)
{
  if (!obj || obj->type != OB_MESH) { return std::nullopt; }
  auto find = [&](const Mesh *m) -> std::optional<bke::AttrType> {
    if (!m) { return std::nullopt; }
    const auto meta = m->attributes().lookup_meta_data(name);
    return meta ? std::optional(meta->data_type) : std::nullopt;
  };
  /* Use evaluated mesh from depsgraph when available (necessary for GN attributes). */
  if (dg) {
    if (const Object *o_eval = DEG_get_evaluated(dg, const_cast<Object *>(obj))) {
      if (const Mesh *m = BKE_object_get_evaluated_mesh_no_subsurf_unchecked(o_eval)) {
        if (auto t = find(m)) { return t; }
      }
      if (o_eval->runtime->geometry_set_eval) {
        if (const Mesh *m = o_eval->runtime->geometry_set_eval->get_mesh()) {
          if (auto t = find(m)) { return t; }
        }
      }
    }
  }
  if (obj->runtime->geometry_set_eval) {
    if (const Mesh *m = obj->runtime->geometry_set_eval->get_mesh()) {
      if (auto t = find(m)) { return t; }
    }
  }
  if (const Mesh *m = BKE_object_get_evaluated_mesh_no_subsurf_unchecked(obj)) {
    if (auto t = find(m)) { return t; }
  }
  return find(BKE_object_get_original_mesh(const_cast<Object *>(obj)));
}

/* -------------------------------------------------------------------- */
/*  Per‑triangle attribute sampling                                     */
/* -------------------------------------------------------------------- */

static float4 col_from(float v)      { return float4(v, v, v, 1.0f); }
static float4 col_from(const float3 &v) { return float4(v.x, v.y, v.z, 1.0f); }

/** Cached mesh attribute reader — lookup once per cook, not once per triangle. */
struct AttrSampleSrc {
  std::string name;
  bke::GAttributeReader reader;
  bke::AttrDomain domain = bke::AttrDomain::Point;
  const CPPType *type = nullptr;
  bool valid = false;
};

static Vector<AttrSampleSrc> build_attr_sample_srcs(const Mesh &mesh,
                                                    const Span<std::string> attr_names)
{
  Vector<AttrSampleSrc> srcs;
  srcs.reserve(attr_names.size());
  const bke::AttributeAccessor attrs = mesh.attributes();
  for (const std::string &name : attr_names) {
    AttrSampleSrc src;
    src.name = name;
    src.reader = attrs.lookup(name);
    if (src.reader) {
      src.domain = src.reader.domain;
      src.type = &src.reader.varray.type();
      src.valid = true;
    }
    srcs.append(std::move(src));
  }
  return srcs;
}

static void sample_attribute_on_tri_cached(const AttrSampleSrc &src,
                                           const Mesh &mesh,
                                           const int3 tri,
                                           const int face_i,
                                           float4 out_vals[3])
{
  out_vals[0] = out_vals[1] = out_vals[2] = float4(0);
  if (!src.valid || src.type == nullptr) {
    return;
  }
  const CPPType &ty = *src.type;
  const Span<int> cv = mesh.corner_verts();
  const GVArray &varray = src.reader.varray;

  auto read = [&](int idx, float4 &o) {
    if (ty.is<float>()) {
      o = col_from(varray.get<float>(idx));
    }
    else if (ty.is<int>()) {
      o = col_from(float(varray.get<int>(idx)));
    }
    else if (ty.is<bool>()) {
      o = col_from(varray.get<bool>(idx) ? 1.0f : 0.0f);
    }
    else if (ty.is<float2>()) {
      const float2 v = varray.get<float2>(idx);
      o = float4(v.x, v.y, 0, 1);
    }
    else if (ty.is<float3>()) {
      o = col_from(varray.get<float3>(idx));
    }
    else if (ty.is<float4>()) {
      o = varray.get<float4>(idx);
    }
    else if (ty.is<ColorGeometry4f>()) {
      const ColorGeometry4f c = varray.get<ColorGeometry4f>(idx);
      o = float4(c.r, c.g, c.b, c.a);
    }
    else if (ty.is<ColorGeometry4b>()) {
      const ColorGeometry4b c = varray.get<ColorGeometry4b>(idx);
      o = float4(c.r / 255.f, c.g / 255.f, c.b / 255.f, c.a / 255.f);
    }
    else if (ty.is<int8_t>()) {
      o = col_from(float(varray.get<int8_t>(idx)));
    }
    else if (ty.is<math::Quaternion>()) {
      const math::Quaternion q = varray.get<math::Quaternion>(idx);
      o = float4(q.x, q.y, q.z, q.w);
    }
  };

  switch (src.domain) {
    case bke::AttrDomain::Point:
      for (const int i : {0, 1, 2}) {
        read(cv[tri[i]], out_vals[i]);
      }
      break;
    case bke::AttrDomain::Corner:
      for (const int i : {0, 1, 2}) {
        read(tri[i], out_vals[i]);
      }
      break;
    case bke::AttrDomain::Face:
      if (face_i >= 0) {
        float4 c;
        read(face_i, c);
        out_vals[0] = out_vals[1] = out_vals[2] = c;
      }
      break;
    default:
      break;
  }
}

static void sample_attribute_on_tri(const Mesh &mesh,
                                    const StringRef name,
                                    const int3 tri,
                                    const int face_i,
                                    float4 out_vals[3])
{
  AttrSampleSrc src;
  src.name = name;
  src.reader = mesh.attributes().lookup(name);
  if (src.reader) {
    src.domain = src.reader.domain;
    src.type = &src.reader.varray.type();
    src.valid = true;
  }
  sample_attribute_on_tri_cached(src, mesh, tri, face_i, out_vals);
}

/* -------------------------------------------------------------------- */
/*  Raster buffers                                                      */
/* -------------------------------------------------------------------- */

struct RasterBuffers {
  int2 size = int2(1);
  Array<float>  depth;
  Array<float3> position;
  Array<float3> normal;
  Array<float>  alpha;
  Vector<std::string> attr_names;
  Vector<Array<float4>> attrs;

  void allocate(const int2 res, const Span<std::string> names) {
    size = math::max(res, int2(1));
    int64_t n = int64_t(size.x) * int64_t(size.y);
    depth.reinitialize(n);       depth.fill(std::numeric_limits<float>::infinity());
    position.reinitialize(n);    position.fill(float3(0));
    normal.reinitialize(n);      normal.fill(float3(0));
    alpha.reinitialize(n);       alpha.fill(0);
    attr_names.clear(); attr_names.extend(names);
    attrs.reinitialize(names.size());
    for (auto &a : attrs) { a.reinitialize(n); a.fill(float4(0)); }
  }

  int64_t idx(int x, int y) const { return int64_t(y) * int64_t(size.x) + int64_t(x); }

  void write(int x, int y, float z, const float3 &pos, const float3 &nor,
             const Span<float4> av)
  {
    if (x < 0 || y < 0 || x >= size.x || y >= size.y) { return; }
    const int64_t i = idx(x, y);
    if (z < 0.0f || !(z < depth[i])) { return; }
    depth[i] = z;
    position[i] = pos;
    normal[i] = math::normalize(nor);
    alpha[i] = 1.0f;
    for (int a = 0; a < av.size(); a++) { attrs[a][i] = av[a]; }
  }
};

/* -------------------------------------------------------------------- */
/*  Projection helpers                                                  */
/* -------------------------------------------------------------------- */

static void ortho_basis(const float3 n, float3 &r_t, float3 &r_b)
{
  const float3 z = math::normalize(n);
  float3 y(0, 0, 1);
  y = y - math::dot(y, z) * z;
  if (math::length_squared(y) < 1e-12f) { y = float3(0, 1, 0); y -= math::dot(y, z) * z; }
  y = math::normalize(y);
  r_t = math::cross(z, y);
  r_b = y;
}

static bool bary_2d(const float2 p, const float2 a, const float2 b, const float2 c, float3 &r_w)
{
  const float2 v0 = b - a, v1 = c - a, v2 = p - a;
  const float den = v0.x * v1.y - v1.x * v0.y;
  if (math::abs(den) < 1e-12f) { return false; }
  const float inv = 1.0f / den;
  const float w1 = (v2.x * v1.y - v1.x * v2.y) * inv;
  const float w2 = (v0.x * v2.y - v2.x * v0.y) * inv;
  const float w0 = 1.0f - w1 - w2;
  if (w0 < -1e-5f || w1 < -1e-5f || w2 < -1e-5f) { return false; }
  r_w = float3(w0, w1, w2);
  return true;
}

static void raster_tri(RasterBuffers &buf,
                       const float2 p0, const float2 p1, const float2 p2,
                       const float3 pos0, const float3 pos1, const float3 pos2,
                       const float3 n0, const float3 n1, const float3 n2,
                       float z0, float z1, float z2,
                       const Span<float4> a0, const Span<float4> a1, const Span<float4> a2)
{
  const int ac = a0.size();
  float2 lo = math::min(math::min(p0, p1), p2);
  float2 hi = math::max(math::max(p0, p1), p2);
  int2 b0 = math::clamp(int2(lo), int2(0), buf.size - 1);
  int2 b1 = math::clamp(int2(math::ceil(hi)), int2(0), buf.size - 1);

  Array<float4> attrs(ac);
  for (int y = b0.y; y <= b1.y; y++) {
    for (int x = b0.x; x <= b1.x; x++) {
      float3 w;
      if (!bary_2d(float2(x + 0.5f, y + 0.5f), p0, p1, p2, w)) { continue; }
      float3 pos = pos0 * w.x + pos1 * w.y + pos2 * w.z;
      float3 nor = n0 * w.x + n1 * w.y + n2 * w.z;
      if (math::length_squared(nor) < 1e-12f) { nor = float3(0, 0, 1); }
      float z = z0 * w.x + z1 * w.y + z2 * w.z;
      for (int a = 0; a < ac; a++) {
        attrs[a] = float4(a0[a].x * w.x + a1[a].x * w.y + a2[a].x * w.z,
                          a0[a].y * w.x + a1[a].y * w.y + a2[a].y * w.z,
                          a0[a].z * w.x + a1[a].z * w.y + a2[a].z * w.z,
                          a0[a].w * w.x + a1[a].w * w.y + a2[a].w * w.z);
      }
      buf.write(x, y, z, pos, nor, attrs);
    }
  }
}

/* -------------------------------------------------------------------- */
/*  Evaluated‑mesh helper                                               */
/* -------------------------------------------------------------------- */

static std::pair<const Mesh *, const Object *>
get_eval_mesh(const Object *obj, Depsgraph *dg)
{
  if (!obj || obj->type != OB_MESH) { return {nullptr, obj}; }
  const Object *o_eval = obj;
  if (dg) { o_eval = DEG_get_evaluated(dg, const_cast<Object *>(obj)); }
  if (!o_eval) { o_eval = obj; }
  if (o_eval->runtime->geometry_set_eval) {
    if (const Mesh *m = o_eval->runtime->geometry_set_eval->get_mesh()) { return {m, o_eval}; }
  }
  if (const Mesh *m = BKE_object_get_evaluated_mesh_no_subsurf_unchecked(o_eval)) {
    return {m, o_eval};
  }
  if (Mesh *m = BKE_object_get_evaluated_mesh(o_eval)) { return {m, o_eval}; }
  return {BKE_object_get_original_mesh(const_cast<Object *>(obj)), obj};
}

/**
 * Mesh used for UV / Attributes dropdowns: prefer evaluated (GN attributes),
 * fall back to original mesh when depsgraph is unavailable.
 */
static const Mesh *mesh_for_ui(const bContext *C, const Object *object)
{
  if (!object || object->type != OB_MESH) {
    return nullptr;
  }
  if (C) {
    if (Depsgraph *dg = CTX_data_depsgraph_pointer(C)) {
      auto [mesh, /*o_eval*/ _] = get_eval_mesh(object, dg);
      if (mesh) {
        return mesh;
      }
    }
  }
  if (object->runtime->geometry_set_eval) {
    if (const Mesh *m = object->runtime->geometry_set_eval->get_mesh()) {
      return m;
    }
  }
  return BKE_object_get_original_mesh(const_cast<Object *>(object));
}

/** True for Face Corner vector-like attributes usable as UV / 2D coords. */
static bool is_corner_vector_attr(const bke::AttrType type, const bke::AttrDomain domain)
{
  if (domain != bke::AttrDomain::Corner) {
    return false;
  }
  return ELEM(type,
              bke::AttrType::Float2,
              bke::AttrType::Float3,
              bke::AttrType::Int16_2D,
              bke::AttrType::Int32_2D);
}

/* -------------------------------------------------------------------- */
/*  Projected triangle vertices (shared CPU prep for GPU/CPU raster)  */
/* -------------------------------------------------------------------- */

/** Max user attributes packed into the hardware raster MRT (matches shader). */
static constexpr int RASTER_GPU_ATTR_SLOTS = 4;

struct ProjectedVert {
  float3 ndc; /* xy NDC [-1,1], z depth [0,1] */
  float3 world_pos;
  float3 normal;
  float4 attrs[RASTER_GPU_ATTR_SLOTS];
};

/**
 * Build triangle list in projected space. O(tris) — cheap compared to O(pixels) CPU fill.
 * Returns empty if mesh/UV unavailable.
 */
static Vector<ProjectedVert> build_projected_verts(const Mesh *mesh,
                                                   const float4x4 &object_to_world,
                                                   RasterizeMode mode,
                                                   float3 plane_pos,
                                                   float3 plane_dir,
                                                   float plane_size,
                                                   const StringRef uv_map,
                                                   int2 resolution,
                                                   const Span<std::string> attr_names)
{
  Vector<ProjectedVert> out;
  if (!mesh || mesh->verts_num == 0 || mesh->faces_num == 0) {
    return out;
  }

  const float4x4 o2w = object_to_world;
  bool inv_ok = false;
  const float3x3 nm = math::transpose(math::invert(float3x3(o2w), inv_ok));
  UNUSED_VARS(inv_ok);

  const Span<float3> lpos = mesh->vert_positions();
  const Span<float3> cnorm = mesh->corner_normals();
  const Span<int> cv = mesh->corner_verts();
  const Span<int3> tris = mesh->corner_tris();
  const Span<int> tfaces = mesh->corner_tri_faces();

  Array<float3> wpos(mesh->verts_num);
  threading::parallel_for(IndexRange(mesh->verts_num), 1024, [&](const IndexRange r) {
    for (const int i : r) {
      wpos[i] = math::transform_point(o2w, lpos[i]);
    }
  });

  Array<float2> uv_storage;
  VArraySpan<float2> uvs;
  bool have_uv = false;
  if (mode == RasterizeMode::UV) {
    const bke::AttributeAccessor attrs = mesh->attributes();
    std::string uvn_own;
    StringRefNull uvn;
    if (!uv_map.is_empty()) {
      uvn_own = uv_map;
      uvn = uvn_own;
    }
    else {
      uvn = mesh->active_or_default_uv_map_name();
    }
    if (!uvn.is_empty()) {
      if (auto rd = attrs.lookup<float2>(uvn, bke::AttrDomain::Corner)) {
        uvs = *rd;
        have_uv = true;
      }
      else if (auto rd3 = attrs.lookup<float3>(uvn, bke::AttrDomain::Corner)) {
        const VArraySpan<float3> v3 = *rd3;
        uv_storage.reinitialize(v3.size());
        for (const int64_t i : v3.index_range()) {
          uv_storage[i] = float2(v3[i].x, v3[i].y);
        }
        uvs = VArraySpan(VArray<float2>::from_span(uv_storage.as_span()));
        have_uv = true;
      }
      else if (auto rd_i2 = attrs.lookup<int2>(uvn, bke::AttrDomain::Corner)) {
        const VArraySpan<int2> v2 = *rd_i2;
        uv_storage.reinitialize(v2.size());
        for (const int64_t i : v2.index_range()) {
          uv_storage[i] = float2(float(v2[i].x), float(v2[i].y));
        }
        uvs = VArraySpan(VArray<float2>::from_span(uv_storage.as_span()));
        have_uv = true;
      }
    }
    if (!have_uv) {
      return out;
    }
  }

  float3 tang, bitang, pn;
  const float hs = math::max(plane_size, 1e-6f) * 0.5f;
  if (mode == RasterizeMode::World) {
    pn = math::normalize(plane_dir);
    if (math::length_squared(pn) < 1e-12f) {
      pn = float3(0, 0, 1);
    }
    ortho_basis(pn, tang, bitang);
  }

  /* Depth scale: map positive plane distance into [0,1] for the hardware depth buffer. */
  const float z_range = math::max(plane_size * 4.0f, 1e-3f);
  const int ac = int(attr_names.size());
  const int ac_gpu = math::min(ac, RASTER_GPU_ATTR_SLOTS);
  /* Lookup each attribute once — not once per triangle (was a major CPU cost). */
  const Vector<AttrSampleSrc> attr_srcs = build_attr_sample_srcs(*mesh, attr_names);
  const float2 res_f(float(math::max(resolution.x, 1)), float(math::max(resolution.y, 1)));
  const float inv_res_x = 1.0f / res_f.x;
  const float inv_res_y = 1.0f / res_f.y;
  const float inv_z_range = 1.0f / z_range;

  out.reserve(tris.size() * 3);
  for (const int ti : tris.index_range()) {
    const int3 t = tris[ti];
    const int v0 = cv[t[0]], v1 = cv[t[1]], v2 = cv[t[2]];
    const float3 p0 = wpos[v0], p1 = wpos[v1], p2 = wpos[v2];
    float3 n0 = nm * cnorm[t[0]], n1 = nm * cnorm[t[1]], n2 = nm * cnorm[t[2]];

    float2 s0, s1, s2;
    float z0, z1, z2;
    if (mode == RasterizeMode::World) {
      auto proj = [&](const float3 p, float2 &s, float &z) {
        const float3 d = p - plane_pos;
        z = math::dot(d, pn);
        s.x = ((math::dot(d, tang) / hs) * 0.5f + 0.5f) * res_f.x;
        s.y = ((math::dot(d, bitang) / hs) * 0.5f + 0.5f) * res_f.y;
      };
      proj(p0, s0, z0);
      proj(p1, s1, z1);
      proj(p2, s2, z2);
    }
    else {
      s0 = float2(uvs[t[0]].x * res_f.x, uvs[t[0]].y * res_f.y);
      s1 = float2(uvs[t[1]].x * res_f.x, uvs[t[1]].y * res_f.y);
      s2 = float2(uvs[t[2]].x * res_f.x, uvs[t[2]].y * res_f.y);
      z0 = p0.z;
      z1 = p1.z;
      z2 = p2.z;
    }

    /* Skip triangles fully behind the plane (matches CPU write() z < 0 reject). */
    if (z0 < 0.0f && z1 < 0.0f && z2 < 0.0f) {
      continue;
    }

    const int fi = tfaces[ti];
    float4 a0[RASTER_GPU_ATTR_SLOTS] = {}, a1[RASTER_GPU_ATTR_SLOTS] = {},
           a2[RASTER_GPU_ATTR_SLOTS] = {};
    for (int a = 0; a < ac_gpu; a++) {
      float4 vv[3];
      sample_attribute_on_tri_cached(attr_srcs[a], *mesh, t, fi, vv);
      a0[a] = vv[0];
      a1[a] = vv[1];
      a2[a] = vv[2];
    }

    auto emit = [&](const float2 s, const float z, const float3 p, const float3 n, const float4 *av) {
      ProjectedVert v;
      v.ndc.x = s.x * inv_res_x * 2.0f - 1.0f;
      v.ndc.y = s.y * inv_res_y * 2.0f - 1.0f;
      v.ndc.z = (z < 0.0f) ? 1.0f : math::clamp(z * inv_z_range, 0.0f, 1.0f);
      v.world_pos = p;
      v.normal = n;
      for (int a = 0; a < RASTER_GPU_ATTR_SLOTS; a++) {
        v.attrs[a] = av[a];
      }
      out.append(v);
    };
    emit(s0, z0, p0, n0, a0);
    emit(s1, z1, p1, n1, a1);
    emit(s2, z2, p2, n2, a2);
  }
  return out;
}

/* -------------------------------------------------------------------- */
/*  Main rasterize (CPU fallback)                                       */
/* -------------------------------------------------------------------- */

static void rasterize_geometry(const Mesh *mesh,
                               const float4x4 &object_to_world,
                               RasterizeMode mode, float3 plane_pos, float3 plane_dir,
                               float plane_size, const StringRef uv_map,
                               int2 resolution, const Span<std::string> attr_names,
                               RasterBuffers &buf)
{
  buf.allocate(resolution, attr_names);
  if (!mesh || mesh->verts_num == 0 || mesh->faces_num == 0) { return; }

  const float4x4 o2w = object_to_world;
  bool inv_ok; const float3x3 nm = math::transpose(math::invert(float3x3(o2w), inv_ok));
  UNUSED_VARS(inv_ok);

  const Span<float3> lpos = mesh->vert_positions();
  const Span<float3> cnorm = mesh->corner_normals();
  const Span<int> cv = mesh->corner_verts();
  const Span<int3> tris = mesh->corner_tris();
  const Span<int> tfaces = mesh->corner_tri_faces();

  Array<float3> wpos(mesh->verts_num);
  threading::parallel_for(IndexRange(mesh->verts_num), 1024, [&](IndexRange r) {
    for (int i : r) { wpos[i] = math::transform_point(o2w, lpos[i]); }
  });

  /* UV mode: Face Corner vector attribute (traditional UV Map = Float2 corner, or GN
   * Store Named Attribute as Float2/Float3 on Face Corner). */
  Array<float2> uv_storage;
  VArraySpan<float2> uvs;
  bool have_uv = false;
  if (mode == RasterizeMode::UV) {
    const bke::AttributeAccessor attrs = mesh->attributes();
    std::string uvn_own;
    StringRefNull uvn;
    if (!uv_map.is_empty()) {
      uvn_own = uv_map;
      uvn = uvn_own;
    }
    else {
      uvn = mesh->active_or_default_uv_map_name();
    }
    if (!uvn.is_empty()) {
      if (auto rd = attrs.lookup<float2>(uvn, bke::AttrDomain::Corner)) {
        uvs = *rd;
        have_uv = true;
      }
      else if (auto rd3 = attrs.lookup<float3>(uvn, bke::AttrDomain::Corner)) {
        const VArraySpan<float3> v3 = *rd3;
        uv_storage.reinitialize(v3.size());
        for (const int64_t i : v3.index_range()) {
          uv_storage[i] = float2(v3[i].x, v3[i].y);
        }
        uvs = VArraySpan(VArray<float2>::from_span(uv_storage.as_span()));
        have_uv = true;
      }
      else if (auto rd_i2 = attrs.lookup<int2>(uvn, bke::AttrDomain::Corner)) {
        const VArraySpan<int2> v2 = *rd_i2;
        uv_storage.reinitialize(v2.size());
        for (const int64_t i : v2.index_range()) {
          uv_storage[i] = float2(float(v2[i].x), float(v2[i].y));
        }
        uvs = VArraySpan(VArray<float2>::from_span(uv_storage.as_span()));
        have_uv = true;
      }
    }
    if (!have_uv) {
      return;
    }
  }

  float3 tang, bitang, pn;
  float hs = math::max(plane_size, 1e-6f) * 0.5f;
  if (mode == RasterizeMode::World) {
    pn = math::normalize(plane_dir);
    if (math::length_squared(pn) < 1e-12f) { pn = float3(0, 0, 1); }
    ortho_basis(pn, tang, bitang);
  }

  const int ac = int(attr_names.size());
  const Vector<AttrSampleSrc> attr_srcs = build_attr_sample_srcs(*mesh, attr_names);
  for (int ti : tris.index_range()) {
    int3 t = tris[ti];
    int v0 = cv[t[0]], v1 = cv[t[1]], v2 = cv[t[2]];
    float3 p0 = wpos[v0], p1 = wpos[v1], p2 = wpos[v2];
    float3 n0 = nm * cnorm[t[0]], n1 = nm * cnorm[t[1]], n2 = nm * cnorm[t[2]];

    float2 s0, s1, s2; float z0, z1, z2;
    if (mode == RasterizeMode::World) {
      auto proj = [&](float3 p, float2 &s, float &z) {
        float3 d = p - plane_pos;
        z = math::dot(d, pn);
        s.x = ((math::dot(d, tang) / hs) * 0.5f + 0.5f) * float(buf.size.x);
        s.y = ((math::dot(d, bitang) / hs) * 0.5f + 0.5f) * float(buf.size.y);
      };
      proj(p0, s0, z0); proj(p1, s1, z1); proj(p2, s2, z2);
    } else {
      s0 = float2(uvs[t[0]].x * buf.size.x, uvs[t[0]].y * buf.size.y);
      s1 = float2(uvs[t[1]].x * buf.size.x, uvs[t[1]].y * buf.size.y);
      s2 = float2(uvs[t[2]].x * buf.size.x, uvs[t[2]].y * buf.size.y);
      z0 = p0.z; z1 = p1.z; z2 = p2.z;
    }

    int fi = tfaces[ti];
    Array<float4> a0(ac), a1(ac), a2(ac);
    for (int a = 0; a < ac; a++) {
      float4 vv[3];
      sample_attribute_on_tri_cached(attr_srcs[a], *mesh, t, fi, vv);
      a0[a] = vv[0]; a1[a] = vv[1]; a2[a] = vv[2];
    }
    raster_tri(buf, s0, s1, s2, p0, p1, p2, n0, n1, n2, z0, z1, z2, a0, a1, a2);
  }
}

/* -------------------------------------------------------------------- */
/*  UV Map / Attributes string UI                                       */
/*  UV: SearchMenu (Face Corner vectors on evaluated mesh).             */
/*  Attributes: dropdown multi-select checklist (Menu multi look),      */
/*  storage remains a comma-separated String — not Menu type.           */
/*  Scoped to this node only via custom_draw — never is_attribute_name. */
/* -------------------------------------------------------------------- */

/** Skip internal/hidden mesh attributes (leading '.' and common builtins). */
static bool attr_name_selectable(const StringRef name)
{
  if (name.is_empty() || name.startswith(".")) {
    return false;
  }
  if (bke::attribute_name_is_anonymous(name)) {
    return false;
  }
  return !ELEM(name, "position", "normal", "id", "Type", "Bundle");
}

static std::string csv_from_selected(const Span<std::string> names)
{
  std::string csv;
  for (const int i : names.index_range()) {
    if (i > 0) {
      csv += ", ";
    }
    csv += names[i];
  }
  return csv;
}

static constexpr int ATTR_LIST_MAX = 128;

/** Shared name-list payload for UV single-select and Attributes multi-select popups. */
struct AttrListPopupArgs {
  int32_t node_id = 0;
  /** "UV Map" or "Attributes". */
  char socket_id[64] = {};
  bool multi_select = false;
  int count = 0;
  int flags[ATTR_LIST_MAX] = {};
  char names[ATTR_LIST_MAX][64] = {};
};

/**
 * Session-stable storage for the open multi-select popup.
 *
 * uiDefBlockButN owns a heap AttrListPopupArgs and frees it when the outer button is
 * destroyed. NC_NODE notifiers (and area redraws) rebuild the node layout while the
 * popup is still open; button update-from-old then frees the args that the checkboxes
 * still point at. Subsequent toggles write UAF / zeroed flags, so only the last click
 * appears to stick after the popup closes.
 *
 * Checkboxes and writebacks always use this live copy for the lifetime of one open.
 */
static AttrListPopupArgs g_attr_list_popup_live;
/** Multi-select: defer NC_NODE|NA_EDITED until the popup closes (one cook, not per checkbox). */
static bool g_attr_list_eval_pending = false;
static int g_attr_list_eval_pending_node_id = 0;

static void attr_list_fill_from_mesh(AttrListPopupArgs &args,
                                     const Mesh *mesh,
                                     const bool corner_vectors_only)
{
  args.count = 0;
  if (!mesh) {
    return;
  }
  mesh->attributes().foreach_attribute([&](const bke::AttributeIter &iter) {
    if (args.count >= ATTR_LIST_MAX) {
      return;
    }
    if (!attr_name_selectable(iter.name)) {
      return;
    }
    if (corner_vectors_only && !is_corner_vector_attr(iter.data_type, iter.domain)) {
      return;
    }
    const int i = args.count++;
    BLI_strncpy(args.names[i], std::string(iter.name).c_str(), sizeof(args.names[i]));
    args.flags[i] = 0;
  });
}

/** Redraw outer dropdown label after the multi-select popup is no longer holding live flags.
 * Prefer not to call this on every multi-toggle (it rebuilds the node layout). */
static void attr_list_tag_ui_refresh(bContext &C, const bool redraw_area)
{
  if (ARegion *popup = CTX_wm_region_popup(&C)) {
    ED_region_tag_redraw(popup);
  }
  /* uiDefBlockBut popups open with can_refresh=false so region_popup is often unset. */
  if (bScreen *screen = CTX_wm_screen(&C)) {
    for (ARegion *region = static_cast<ARegion *>(screen->regionbase.first()); region;
         region = region->next)
    {
      if (region->regiontype == RGN_TYPE_TEMPORARY) {
        ED_region_tag_redraw(region);
      }
    }
  }
  if (redraw_area) {
    if (ARegion *region = CTX_wm_region(&C)) {
      ED_region_tag_redraw(region);
    }
    if (ScrArea *area = CTX_wm_area(&C)) {
      ED_area_tag_redraw(area);
    }
  }
}

static void attr_list_write_socket(bContext &C, AttrListPopupArgs &args)
{
  SpaceNode *snode = CTX_wm_space_node(&C);
  if (!snode || !snode->edittree) {
    return;
  }
  bNodeTree *ntree = snode->edittree;
  bNode *node = ntree->node_by_id(args.node_id);
  if (!node) {
    return;
  }
  bNodeSocket *sock = bke::node_find_enabled_input_socket(*node, args.socket_id);
  if (!sock || sock->type != SOCK_STRING) {
    return;
  }
  std::string value;
  if (args.multi_select) {
    Vector<std::string> selected;
    for (int i = 0; i < args.count; i++) {
      if (args.flags[i]) {
        selected.append(args.names[i]);
      }
    }
    value = csv_from_selected(selected);
  }
  else {
    for (int i = 0; i < args.count; i++) {
      if (args.flags[i]) {
        value = args.names[i];
        break;
      }
    }
  }
  auto *val = sock->default_value_typed<bNodeSocketValueString>();
  if (!val) {
    return;
  }
  BLI_strncpy_utf8(val->value, value.c_str(), sizeof(val->value));
  BKE_ntree_update_tag_node_property(ntree, node);
  if (args.multi_select) {
    /* Do NOT fire NA_EDITED per checkbox — that re-declares sockets + full Image Process cook
     * (~250ms) on every click. Commit once when the outer layout redraws after popup closes. */
    g_attr_list_eval_pending = true;
    g_attr_list_eval_pending_node_id = args.node_id;
    attr_list_tag_ui_refresh(C, false);
  }
  else {
    WM_main_add_notifier(NC_NODE | NA_EDITED, &ntree->id);
    attr_list_tag_ui_refresh(C, true);
  }
}

static ui::Block *attr_list_popup(bContext *C, ARegion *region, void *args_v)
{
  using namespace ui;
  /* Promote button-owned args into session-stable storage before any checkbox poin is set. */
  AttrListPopupArgs *src = static_cast<AttrListPopupArgs *>(args_v);
  if (src != &g_attr_list_popup_live) {
    g_attr_list_popup_live = *src;
  }
  AttrListPopupArgs *args = &g_attr_list_popup_live;

  /* Re-sync flags from current socket string once when the popup is created. */
  {
    SpaceNode *snode = CTX_wm_space_node(C);
    if (snode && snode->edittree) {
      if (bNode *node = snode->edittree->node_by_id(args->node_id)) {
        if (bNodeSocket *sock = bke::node_find_enabled_input_socket(*node, args->socket_id)) {
          if (sock->type == SOCK_STRING) {
            const char *cur = sock->default_value_typed<bNodeSocketValueString>()->value;
            if (args->multi_select) {
              const Vector<std::string> selected = parse_attribute_list(cur);
              for (int i = 0; i < args->count; i++) {
                args->flags[i] = selected.contains(std::string(args->names[i])) ? 1 : 0;
              }
            }
            else {
              for (int i = 0; i < args->count; i++) {
                args->flags[i] = (StringRef(cur) == args->names[i]) ? 1 : 0;
              }
            }
          }
        }
      }
    }
  }

  Block *block = block_begin(C, region, __func__, EmbossType::Emboss);
  block_flag_enable(block, BLOCK_KEEP_OPEN);

  Layout &layout = block_layout(block,
                                LayoutDirection::Vertical,
                                LayoutType::Panel,
                                0,
                                0,
                                UI_UNIT_X * 10,
                                UI_UNIT_Y,
                                0,
                                style_get())
                       .column(true);

  if (args->count == 0) {
    layout.label(IFACE_("No attributes on evaluated mesh"), ICON_INFO);
  }
  else if (args->multi_select) {
    for (int i = 0; i < args->count; i++) {
      Button *but = uiDefButV(block,
                              ButtonType::Checkbox,
                              args->names[i],
                              0,
                              0,
                              short(UI_UNIT_X * 10),
                              UI_UNIT_Y,
                              &args->flags[i],
                              0.0f,
                              0.0f,
                              std::nullopt);
      button_flag_enable(but, BUT_DRAG_LOCK);
      /* Capture args pointer to the live session storage (not the freeable button argN). */
      button_func_set(but, [args](bContext &C) {
        attr_list_write_socket(C, *args);
        ED_undo_push(&C, "Select Rasterize Attributes");
      });
    }
  }
  else {
    layout.label(IFACE_("Internal error"), ICON_ERROR);
  }

  block_bounds_set_normal(block, int(0.3f * U.widget_unit));
  block_direction_set(block, UI_DIR_DOWN);
  return block;
}

/**
 * UV Map: same SearchMenu UX as traditional UV Map / Named Attribute (single select),
 * but items = all Face Corner vector attributes on the *evaluated* mesh (GN UVs included).
 * Node-local custom_draw only — does not use Geometry Nodes attribute/portal search.
 */
struct UvCornerSearchData {
  int32_t node_id;
  int count;
  char names[ATTR_LIST_MAX][64];
};
static_assert(std::is_trivially_destructible_v<UvCornerSearchData>);

static void uv_corner_search_refill(const bContext *C, UvCornerSearchData &data)
{
  data.count = 0;
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return;
  }
  bNode *node = snode->edittree->node_by_id(data.node_id);
  if (!node) {
    return;
  }
  const Mesh *mesh = nullptr;
  if (const bke::GeometrySet *geo = imported_geometry_from_node(*node)) {
    mesh = geo->get_mesh();
  }
  if (!mesh) {
    mesh = mesh_for_ui(C, object_from_node(*node));
  }
  if (!mesh) {
    return;
  }
  mesh->attributes().foreach_attribute([&](const bke::AttributeIter &iter) {
    if (data.count >= ATTR_LIST_MAX) {
      return;
    }
    if (!attr_name_selectable(iter.name)) {
      return;
    }
    if (!is_corner_vector_attr(iter.data_type, iter.domain)) {
      return;
    }
    const int i = data.count++;
    BLI_strncpy(data.names[i], std::string(iter.name).c_str(), sizeof(data.names[i]));
  });
}

static void uv_corner_search_update(const bContext *C,
                                    void *arg,
                                    const char *str,
                                    ui::SearchItems *items,
                                    const bool is_first)
{
  using namespace ui;
  /* Always fill items — including while the timeline is playing.
   * Returning empty here makes button_search_refresh set BUT_REDALERT on the UV Map
   * SearchMenu (classic single-select validates drawstr against the item list). */
  UvCornerSearchData *data = static_cast<UvCornerSearchData *>(arg);
  uv_corner_search_refill(C, *data);

  const char *search = (is_first || str == nullptr) ? "" : str;
  if (search[0] == '\0' && !is_first) {
    search_item_add(items, "", nullptr, ICON_X, 0, 0);
  }
  bool any = false;
  for (int i = 0; i < data->count; i++) {
    if (search[0] != '\0' && BLI_strcasestr(data->names[i], search) == nullptr) {
      continue;
    }
    if (!search_item_add(items, data->names[i], data->names[i], ICON_GROUP_UVS, 0, 0)) {
      break;
    }
    any = true;
  }
  /* Fallback: keep the current socket string valid for red-alert checks even if the
   * mesh/attrs temporarily fail to resolve (e.g. object not evaluated yet this frame).
   * button_search_refresh flags BUT_REDALERT when totitem==0 or drawstr is missing. */
  if (!any && search[0] != '\0') {
    search_item_add(items, search, nullptr, ICON_GROUP_UVS, 0, 0);
  }
}

static void uv_corner_search_exec(bContext *C, void *data_v, void *item_v)
{
  /* Allow picking UV during playback; mesh list is always refreshed above. */
  UvCornerSearchData *data = static_cast<UvCornerSearchData *>(data_v);
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return;
  }
  bNode *node = snode->edittree->node_by_id(data->node_id);
  if (!node) {
    return;
  }
  bNodeSocket *sock = bke::node_find_enabled_input_socket(*node, "UV Map");
  if (!sock || sock->type != SOCK_STRING) {
    return;
  }
  auto *val = sock->default_value_typed<bNodeSocketValueString>();
  if (item_v) {
    BLI_strncpy_utf8(val->value, static_cast<const char *>(item_v), sizeof(val->value));
  }
  /* String RNA button already holds the value when suggestions=false; still write for safety. */
  BKE_ntree_update_tag_node_property(snode->edittree, node);
  /* Do not full-area redraw here — that rebuilds the SearchMenu and leaves a ghost list open. */
  WM_main_add_notifier(NC_NODE | NA_EDITED, &snode->edittree->id);
  ED_undo_push(C, "Assign UV Attribute");
}

static void draw_uv_map_socket(CustomSocketDrawParams &params)
{
  using namespace ui;
  params.layout.alignment_set(LayoutAlign::Expand);

  Block *block = params.layout.block();
  Button *but = uiDefIconTextButR(block,
                                  ButtonType::SearchMenu,
                                  ICON_NONE,
                                  "",
                                  0,
                                  0,
                                  short(10 * UI_UNIT_X),
                                  UI_UNIT_Y,
                                  &params.socket_ptr,
                                  "default_value",
                                  0,
                                  "");
  button_placeholder_set(but, IFACE_("UV / Corner Vector"));

  UvCornerSearchData *data = MEM_new<UvCornerSearchData>(__func__);
  memset(data, 0, sizeof(*data));
  data->node_id = params.node.identifier;
  uv_corner_search_refill(&params.C, *data);

  /* false = classic single-select: pick one item → value commits and popup closes.
   * true (suggestions) keeps the search field open like free-form autocomplete. */
  button_func_search_set_results_are_suggestions(but, false);
  button_func_search_set(but,
                         nullptr,
                         uv_corner_search_update,
                         data,
                         true,
                         nullptr,
                         uv_corner_search_exec,
                         nullptr);
}

static void draw_attributes_socket(CustomSocketDrawParams &params)
{
  using namespace ui;
  params.layout.alignment_set(LayoutAlign::Expand);

  /* Popup closed → outer layout rebuilds here: flush one deferred full cook. */
  if (g_attr_list_eval_pending && g_attr_list_eval_pending_node_id == params.node.identifier) {
    g_attr_list_eval_pending = false;
    g_attr_list_eval_pending_node_id = 0;
    if (bNodeTree *ntree = const_cast<bNodeTree *>(&params.node.owner_tree())) {
      WM_main_add_notifier(NC_NODE | NA_EDITED, &ntree->id);
    }
  }

  const Object *object = object_from_node(params.node);
  const Mesh *imported_mesh = nullptr;
  if (const bke::GeometrySet *geo = imported_geometry_from_node(params.node)) {
    imported_mesh = geo->get_mesh();
  }
  bNodeSocketValueString *str_val = params.socket.default_value_typed<bNodeSocketValueString>();
  if (!str_val) {
    params.draw_standard(params.layout);
    return;
  }

  const Vector<std::string> selected = parse_attribute_list(str_val->value);
  std::string summary = selected.is_empty() ? IFACE_("(None)") : csv_from_selected(selected);

  AttrListPopupArgs *args = MEM_new<AttrListPopupArgs>(__func__);
  args->node_id = params.node.identifier;
  BLI_strncpy(args->socket_id, "Attributes", sizeof(args->socket_id));
  args->multi_select = true;
  const Mesh *mesh = imported_mesh ? imported_mesh : mesh_for_ui(&params.C, object);
  attr_list_fill_from_mesh(*args, mesh, false);
  for (int i = 0; i < args->count; i++) {
    args->flags[i] = selected.contains(std::string(args->names[i])) ? 1 : 0;
  }

  Block *block = params.layout.block();
  block_align_begin(block);
  uiDefBlockButN(block,
                 attr_list_popup,
                 args,
                 summary,
                 0,
                 0,
                 short(UI_UNIT_X * 10),
                 UI_UNIT_Y,
                 TIP_("Attributes on evaluated mesh (includes Geometry Nodes)"),
                 but_func_argN_free<AttrListPopupArgs>,
                 but_func_argN_copy<AttrListPopupArgs>);
  block_align_end(block);
}

/* -------------------------------------------------------------------- */
/*  Node declaration                                                    */
/* -------------------------------------------------------------------- */

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Menu>("Mode"_ustr)
      .default_value(RasterizeMode::World)
      .static_items(mode_items)
      .description("World Space: orthographic ray through the mesh. UV Space: unwrap with a UV map");
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description(
          "Mesh from Import Geo. When connected, takes priority over Object. "
          "Instances are realized by Import Geo");
  b.add_input<decl::Object>("Object"_ustr).description(
      "Scene object to rasterize when Geometry is unconnected");
  b.add_input<decl::Vector>("Ray Origin"_ustr)
      .default_value({0, 0, 0})
      .subtype(PROP_TRANSLATION)
      .usage_by_menu("Mode"_ustr, int(RasterizeMode::World))
      .description("World-space origin of the orthographic projection");
  b.add_input<decl::Vector>("Ray Direction"_ustr)
      .default_value({0, 0, 1})
      .usage_by_menu("Mode"_ustr, int(RasterizeMode::World))
      .description("World-space ray direction (points toward the mesh)");
  b.add_input<decl::Float>("Size"_ustr)
      .default_value(2.0f)
      .min(0)
      .usage_by_menu("Mode"_ustr, int(RasterizeMode::World))
      .description("World-space width of the rasterized view");
  b.add_input<decl::String>("UV Map"_ustr)
      .default_value("")
      .usage_by_menu("Mode"_ustr, int(RasterizeMode::UV))
      .custom_draw(draw_uv_map_socket)
      .description("UV map name (Face Corner vectors on evaluated mesh)");
  b.add_input<decl::String>("Attributes"_ustr)
      .default_value("")
      .custom_draw(draw_attributes_socket)
      .description(
          "Comma-separated attribute names. Multi-select checklist from Object mesh attributes");

  b.add_output<decl::Vector>("Position"_ustr)
      .dimensions(3)
      .structure_type(StructureType::Dynamic)
      .description("World-space position of the closest hit");
  b.add_output<decl::Vector>("Normal"_ustr)
      .dimensions(3)
      .structure_type(StructureType::Dynamic)
      .description("World-space shading normal of the closest hit");
  b.add_output<decl::Float>("Alpha"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Coverage (1 = hit, 0 = miss)");
  b.add_output<decl::Bundle>("Attributes"_ustr)
      .structure_type(StructureType::Single)
      .description("Rasterized mesh attributes as a Bundle");

  /* Hidden typed outputs for COM + SeparateBundle auto-sync. */
  Vector<std::string> attr_names;
  const Object *object = nullptr;
  const Mesh *imported_mesh = nullptr;
  if (const bNode *node = b.node_or_null()) {
    attr_names = parse_attribute_list(attributes_string_from_node(*node));
    object = object_from_node(*node);
    if (const bke::GeometrySet *geo = imported_geometry_from_node(*node)) {
      imported_mesh = geo->get_mesh();
    }
  }
  for (const std::string &name : attr_names) {
    eNodeSocketDatatype st = SOCK_RGBA;
    if (std::optional<bke::AttrType> at = attrtype_from_mesh(imported_mesh, name)) {
      st = socket_type_for_attrtype(*at);
    }
    else if (std::optional<bke::AttrType> at = attrtype_from_object(object, name)) {
      st = socket_type_for_attrtype(*at);
    }
    switch (st) {
      case SOCK_FLOAT:
        b.add_output<decl::Float>(UString(name)).structure_type(StructureType::Dynamic); break;
      case SOCK_VECTOR:
        b.add_output<decl::Vector>(UString(name)).dimensions(3).structure_type(StructureType::Dynamic); break;
      default:
        b.add_output<decl::Color>(UString(name)).structure_type(StructureType::Dynamic); break;
    }
  }
}

/* -------------------------------------------------------------------- */
/*  Separate‑Bundle auto‑sync                                           */
/* -------------------------------------------------------------------- */

static void auto_sync_separate_bundles(bNodeTree &ntree, bNode &node,
                                       const Span<std::string> names,
                                       const Object *object,
                                       const Mesh *imported_mesh = nullptr,
                                       Depsgraph *dg = nullptr)
{
  ntree.ensure_topology_cache();
  bNodeSocket *bundle_out = nullptr;
  for (bNodeSocket *s : node.output_sockets()) {
    if (s->type == SOCK_BUNDLE) { bundle_out = s; break; }
  }
  if (!bundle_out) { return; }

  for (bNodeLink *lnk : ntree.all_links()) {
    if (lnk->fromsock != bundle_out || !lnk->tonode) { continue; }
    bNode &ton = *lnk->tonode;
    if (!ton.is_type("NodeSeparateBundle"_ustr)) { continue; }
    if (!ton.storage) {
      continue;
    }

    auto &stor = *static_cast<NodeSeparateBundle *>(ton.storage);
    bool match = (stor.items_num == int(names.size()));
    if (match) {
      for (int i : names.index_range()) {
        eNodeSocketDatatype exp = SOCK_RGBA;
        if (auto at = attrtype_from_object(object, names[i], dg)) {
          exp = socket_type_for_attrtype(*at);
        }
        if (!stor.items[i].name || StringRef(stor.items[i].name) != names[i] ||
            stor.items[i].socket_type != exp)
        { match = false; break; }
      }
    }
    if (match) { continue; }

    Map<std::string, int> old_id;
    for (int i : IndexRange(stor.items_num)) {
      if (stor.items[i].name) {
        old_id.add_overwrite(stor.items[i].name, stor.items[i].identifier);
      }
    }
    socket_items::clear<SeparateBundleItemsAccessor>(ton);
    for (const std::string &nm : names) {
      eNodeSocketDatatype st = SOCK_RGBA;
      if (auto at = attrtype_from_mesh(imported_mesh, nm)) {
        st = socket_type_for_attrtype(*at);
      }
      else if (auto at = attrtype_from_object(object, nm, dg)) {
        st = socket_type_for_attrtype(*at);
      }
      auto &it = *socket_items::add_item_with_socket_type_and_name<
          SeparateBundleItemsAccessor>(
          ntree, ton, st, nm.c_str(),
          st == SOCK_VECTOR ? std::optional(3) : std::nullopt);
      it.structure_type = NodeSocketInterfaceStructureType::Dynamic;
      if (auto id = old_id.lookup_try(nm)) { it.identifier = *id; }
    }
    BKE_ntree_update_tag_node_property(&ntree, &ton);
  }
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  if (!ntree || !node) {
    return;
  }
  const Object *object = object_from_node(*node);
  const Mesh *imported_mesh = nullptr;
  if (const bke::GeometrySet *geo = imported_geometry_from_node(*node)) {
    imported_mesh = geo->get_mesh();
  }
  const Vector<std::string> names = parse_attribute_list(attributes_string_from_node(*node));

  /* Separate Bundle item list only — do not create a temporary depsgraph here.
   * Full scene graph updates during attribute multi-select re-entered Image Process
   * evaluation and crashed (闪退). Types fall back to Color when unknown; re-eval fixes. */
  auto_sync_separate_bundles(*ntree, *node, names, object, imported_mesh);

  /* Hide dynamic attribute outputs (they exist via declare + declaration rebuild).
   * Avoid manual node_add_socket / node_remove_socket here — that races with declare()
   * and the multi-select popup free path. */
  for (bNodeSocket *s : node->output_sockets()) {
    if (!s) {
      continue;
    }
    if (s->type == SOCK_BUNDLE) {
      s->flag &= ~SOCK_HIDDEN;
      continue;
    }
    if (StringRef(s->identifier) == "Position" || StringRef(s->identifier) == "Normal" ||
        StringRef(s->identifier) == "Alpha")
    {
      s->flag &= ~SOCK_HIDDEN;
    }
    else {
      s->flag |= SOCK_HIDDEN;
    }
  }
}

/* -------------------------------------------------------------------- */
/*  COM evaluation                                                      */
/* -------------------------------------------------------------------- */

/* Cache last raster so static meshes are not re-rasterized every Image Process cook.
 * Key covers object identity, mesh topology, mode/params, resolution, attributes. */
struct RasterizeResultCache {
  uint64_t key = 0;
  RasterBuffers buf;
};
static Map<int32_t, RasterizeResultCache> &rasterize_result_caches()
{
  static Map<int32_t, RasterizeResultCache> map;
  return map;
}

static uint64_t rasterize_cache_key(const Object *object,
                                    const Mesh *mesh,
                                    const float4x4 &object_to_world,
                                    Depsgraph *dg,
                                    RasterizeMode mode,
                                    const float3 &plane_pos,
                                    const float3 &plane_dir,
                                    float plane_size,
                                    const StringRef uv_map,
                                    const int2 res,
                                    const Span<std::string> attr_names)
{
  uint64_t h = get_default_hash(int(mode), res.x, res.y);
  h = get_default_hash(h, plane_pos.x, plane_pos.y, plane_pos.z);
  h = get_default_hash(h, plane_dir.x, plane_dir.y, plane_dir.z, plane_size);
  h = get_default_hash(h, uv_map);
  for (const std::string &n : attr_names) {
    h = get_default_hash(h, n);
  }
  h = get_default_hash(h, object_to_world[3].x, object_to_world[3].y, object_to_world[3].z);
  h = get_default_hash(h, object_to_world[0].x, object_to_world[1].y, object_to_world[2].z);
  const Mesh *use_mesh = mesh;
  if (!use_mesh && object) {
    h = get_default_hash(h, object->id.session_uid);
    auto [eval_mesh, obj_xf] = get_eval_mesh(object, dg);
    use_mesh = eval_mesh;
    if (obj_xf && obj_xf != object) {
      const float4x4 &eo2w = obj_xf->object_to_world();
      h = get_default_hash(h, eo2w[3].x, eo2w[3].y, eo2w[3].z);
    }
  }
  if (use_mesh) {
    h = get_default_hash(h, use_mesh->verts_num, use_mesh->faces_num, use_mesh->corners_num);
    h = get_default_hash(h, uintptr_t(use_mesh));
    const Span<float3> pos = use_mesh->vert_positions();
    if (!pos.is_empty()) {
      h = get_default_hash(h, pos[0].x, pos[0].y, pos[0].z);
      h = get_default_hash(h, pos[pos.size() / 2].x, pos.last().x, pos.last().z);
    }
  }
  else {
    h = get_default_hash(h, uint64_t(0xdead));
  }
  return h;
}

class RasterizeGeometryOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    RasterizeMode mode = RasterizeMode(
        get_input("Mode").get_single_value_default<MenuValue>().value);
    const Object *object = get_input("Object").get_single_value_default<Object *>();
    float3 plane_pos = get_input("Ray Origin").get_single_value_default<float3>();
    float3 plane_dir = get_input("Ray Direction").get_single_value_default<float3>();
    float plane_size = get_input("Size").get_single_value_default<float>();
    std::string uv_map = get_input("UV Map").get_single_value_default<std::string>();
    std::string attrs_str = get_input("Attributes").get_single_value_default<std::string>();

    const Domain dom = context().get_compositing_domain();
    int2 res = math::max(int2(1), dom.data_size);
    const Vector<std::string> attr_names = parse_attribute_list(attrs_str);

    Depsgraph *dg = this->get_depsgraph_readonly();

    const bool geo_linked = geometry_input_is_linked(this->node());
    const bke::GeometrySet *imported = nullptr;
    if (geo_linked && this->has_input("Geometry")) {
      Result &geo_in = this->get_input("Geometry");
      if (geo_in.type() == ResultType::String && geo_in.is_single_value()) {
        const std::string key = geo_in.get_single_value_default<std::string>();
        if (PointsGeometryCache *cache = active_cache()) {
          imported = cache->lookup(key);
        }
      }
    }

    const Mesh *mesh = nullptr;
    float4x4 object_to_world = float4x4::identity();
    if (geo_linked) {
      mesh = imported ? imported->get_mesh() : nullptr;
    }
    else {
      auto [eval_mesh, obj_xf] = get_eval_mesh(object, dg);
      mesh = eval_mesh;
      if (obj_xf) {
        object_to_world = obj_xf->object_to_world();
      }
    }

    if (context().use_gpu() &&
        GPU_shader_create_info_get("compositor_rasterize_geometry") != nullptr)
    {
      if (gpu::Shader *shader = context().get_shader("compositor_rasterize_geometry")) {
        if (this->execute_gpu(shader,
                              object,
                              mesh,
                              object_to_world,
                              dg,
                              mode,
                              plane_pos,
                              plane_dir,
                              plane_size,
                              uv_map,
                              res,
                              attr_names))
        {
          return;
        }
      }
    }
    this->execute_cpu(object,
                      mesh,
                      object_to_world,
                      dg,
                      mode,
                      plane_pos,
                      plane_dir,
                      plane_size,
                      uv_map,
                      res,
                      attr_names);
  }

 private:
  Depsgraph *get_depsgraph_readonly()
  {
    /* Prefer an already-evaluated depsgraph. Never re-tag geometry every cook. */
    const Scene &scene = context().get_scene();
    Render *render = RE_GetSceneRender(&scene);
    if (render && render->pipeline_depsgraph) {
      return render->pipeline_depsgraph;
    }
    Main &main = const_cast<Main &>(context().get_main());
    ViewLayer *vl = BKE_view_layer_default_view(&const_cast<Scene &>(scene));
    if (!vl) {
      return nullptr;
    }
    Depsgraph *dg = BKE_scene_get_depsgraph(&const_cast<Scene &>(scene), vl);
    if (!dg) {
      dg = BKE_scene_ensure_depsgraph(&main, &const_cast<Scene &>(scene), vl);
      if (dg) {
        BKE_scene_graph_evaluated_ensure(dg, &main);
      }
    }
    return dg;
  }

  const bNodeSocket *find_out(const StringRef id) const
  {
    for (const bNodeSocket *s : node().output_sockets()) {
      if (s->identifier == id) {
        return s;
      }
    }
    return nullptr;
  }

  /**
   * Attr names actually consumed by Bundle → Separate Bundle / Get Bundle Item.
   * Empty = no specific consumer found (caller may fall back conservatively).
   */
  static Vector<std::string> collect_bundle_consumer_attr_names(const bNode &raster_node)
  {
    Vector<std::string> names;
    Set<std::string> seen;
    auto add_name = [&](const char *n) {
      if (!n || n[0] == '\0') {
        return;
      }
      std::string s(n);
      if (seen.add(s)) {
        names.append(std::move(s));
      }
    };
    for (const bNodeSocket *sock : raster_node.output_sockets()) {
      if (!sock || sock->type != SOCK_BUNDLE) {
        continue;
      }
      for (const bNodeLink *link : sock->directly_linked_links()) {
        if (!link || !link->tonode) {
          continue;
        }
        const bNode &ton = *link->tonode;
        if (ton.is_type("NodeSeparateBundle"_ustr) && ton.storage) {
          const auto &st = *static_cast<const NodeSeparateBundle *>(ton.storage);
          for (int i = 0; i < st.items_num; i++) {
            add_name(st.items[i].name);
          }
        }
        else if (ton.is_type("NodeGetBundleItem"_ustr)) {
          /* Path is a string socket (may be nested "a/b" — stamp maps use leaf). */
          if (const bNodeSocket *path_sock = bke::node_find_enabled_input_socket(
                  const_cast<bNode &>(ton), "Path"))
          {
            if (path_sock->type == SOCK_STRING) {
              const char *path = path_sock->default_value_typed<bNodeSocketValueString>()->value;
              if (path && path[0] != '\0') {
                const char *leaf = path;
                for (const char *p = path; *p; p++) {
                  if (*p == '/') {
                    leaf = p + 1;
                  }
                }
                add_name(leaf);
              }
            }
          }
        }
      }
    }
    return names;
  }

  /** Hardware rasterize into MRT Result textures. Returns false → CPU fallback. */
  bool execute_gpu(gpu::Shader *shader,
                   const Object *object,
                   const Mesh *mesh,
                   const float4x4 &object_to_world,
                   Depsgraph *dg,
                   RasterizeMode mode,
                   const float3 plane_pos,
                   const float3 plane_dir,
                   float plane_size,
                   const StringRef uv_map,
                   const int2 res,
                   const Span<std::string> attr_names)
  {
    UNUSED_VARS(object, dg);
    const Domain rdom(res);
    const int attr_n_all = math::min(int(attr_names.size()), RASTER_GPU_ATTR_SLOTS);

    const bool need_pos = this->should_compute_out("Position");
    const bool need_nor = this->should_compute_out("Normal");
    const bool need_alpha = this->should_compute_out("Alpha");
    /* Bundle → Separate Bundle reads StampAttrMapsCache, not hidden typed sockets. */
    const bool need_bundle = this->should_compute_out("Attributes");

    /* Only raster/download attrs that are actually needed:
     * - typed socket should_compute, or
     * - named by a linked Separate Bundle / Get Bundle Item when Bundle is used.
     * Previously need_bundle forced ALL selected attrs → multi-download every cook. */
    const Vector<std::string> bundle_wanted =
        need_bundle ? collect_bundle_consumer_attr_names(this->node()) : Vector<std::string>{};
    const bool bundle_wants_all = need_bundle && bundle_wanted.is_empty();

    Vector<std::string> active_attrs;
    active_attrs.reserve(attr_n_all);
    for (int i = 0; i < attr_n_all; i++) {
      const bool typed = this->should_compute_attr(attr_names[i]);
      bool via_bundle = false;
      if (need_bundle) {
        if (bundle_wants_all) {
          via_bundle = true;
        }
        else {
          for (const std::string &w : bundle_wanted) {
            if (w == attr_names[i]) {
              via_bundle = true;
              break;
            }
          }
        }
      }
      if (typed || via_bundle) {
        active_attrs.append(attr_names[i]);
      }
    }
    const int attr_n = math::min(int(active_attrs.size()), RASTER_GPU_ATTR_SLOTS);
    const bool any_attr = attr_n > 0;
    /* StampAttrMaps only when Bundle consumers need CPU maps. */
    const bool need_stamp_maps = need_bundle && any_attr;

    /* MRT requires equal-sized attachments. Use texture pool (from_pool=true) to avoid
     * realloc cost every cook. Fixed shader has 7 color outs — fill unused with pool textures. */
    Result pos_tex = context().create_result(ResultType::Float3);
    Result nor_tex = context().create_result(ResultType::Float3);
    Result alpha_color = context().create_result(ResultType::Color);
    Result attr_tex[RASTER_GPU_ATTR_SLOTS] = {context().create_result(ResultType::Color),
                                              context().create_result(ResultType::Color),
                                              context().create_result(ResultType::Color),
                                              context().create_result(ResultType::Color)};
    pos_tex.allocate_texture(rdom, true, ResultStorageType::GPUImage);
    nor_tex.allocate_texture(rdom, true, ResultStorageType::GPUImage);
    alpha_color.allocate_texture(rdom, true, ResultStorageType::GPUImage);
    for (int i = 0; i < RASTER_GPU_ATTR_SLOTS; i++) {
      attr_tex[i].allocate_texture(rdom, true, ResultStorageType::GPUImage);
    }

    gpu::Texture *depth_tx = GPU_texture_create_2d(__func__,
                                                   res.x,
                                                   res.y,
                                                   1,
                                                   gpu::TextureFormat::SFLOAT_32_DEPTH,
                                                   GPU_TEXTURE_USAGE_ATTACHMENT,
                                                   nullptr);
    if (!depth_tx || !pos_tex.gpu_texture() || !nor_tex.gpu_texture() ||
        !alpha_color.gpu_texture())
    {
      if (depth_tx) {
        GPU_texture_free(depth_tx);
      }
      pos_tex.release();
      nor_tex.release();
      alpha_color.release();
      for (int i = 0; i < RASTER_GPU_ATTR_SLOTS; i++) {
        attr_tex[i].release();
      }
      return false;
    }

    /* Only sample active attrs into VBO (empty when previewing Position only). */
    const Vector<ProjectedVert> verts = build_projected_verts(mesh,
                                                              object_to_world,
                                                              mode,
                                                              plane_pos,
                                                              plane_dir,
                                                              plane_size,
                                                              uv_map,
                                                              res,
                                                              active_attrs);

    gpu::FrameBuffer *fb = nullptr;
    GPU_framebuffer_ensure_config(
        &fb,
        {
            GPU_ATTACHMENT_TEXTURE(depth_tx),
            GPU_ATTACHMENT_TEXTURE(pos_tex.gpu_texture()),
            GPU_ATTACHMENT_TEXTURE(nor_tex.gpu_texture()),
            GPU_ATTACHMENT_TEXTURE(alpha_color.gpu_texture()),
            GPU_ATTACHMENT_TEXTURE(attr_tex[0].gpu_texture()),
            GPU_ATTACHMENT_TEXTURE(attr_tex[1].gpu_texture()),
            GPU_ATTACHMENT_TEXTURE(attr_tex[2].gpu_texture()),
            GPU_ATTACHMENT_TEXTURE(attr_tex[3].gpu_texture()),
        });
    GPU_framebuffer_bind(fb);
    const double4 clear0(0.0, 0.0, 0.0, 0.0);
    GPU_framebuffer_clear_color_depth(fb, clear0, 1.0f);

    if (!verts.is_empty()) {
      GPUVertFormat format = {0};
      const uint a_ndc = GPU_vertformat_attr_add(
          &format, "pos_ndc", gpu::VertAttrType::SFLOAT_32_32_32);
      const uint a_wpos = GPU_vertformat_attr_add(
          &format, "world_pos", gpu::VertAttrType::SFLOAT_32_32_32);
      const uint a_nor = GPU_vertformat_attr_add(
          &format, "normal", gpu::VertAttrType::SFLOAT_32_32_32);
      const uint a_a0 = GPU_vertformat_attr_add(
          &format, "attr0", gpu::VertAttrType::SFLOAT_32_32_32_32);
      const uint a_a1 = GPU_vertformat_attr_add(
          &format, "attr1", gpu::VertAttrType::SFLOAT_32_32_32_32);
      const uint a_a2 = GPU_vertformat_attr_add(
          &format, "attr2", gpu::VertAttrType::SFLOAT_32_32_32_32);
      const uint a_a3 = GPU_vertformat_attr_add(
          &format, "attr3", gpu::VertAttrType::SFLOAT_32_32_32_32);

      const int n = int(verts.size());
      /* Bulk SoA fill — GPU_vertbuf_attr_set-per-vertex was a major CPU bottleneck. */
      Array<float3> ndc(n), wpos(n), nor(n);
      Array<float4> a0(n), a1(n), a2(n), a3(n);
      threading::parallel_for(IndexRange(n), 4096, [&](const IndexRange range) {
        for (const int i : range) {
          const ProjectedVert &v = verts[i];
          ndc[i] = v.ndc;
          wpos[i] = v.world_pos;
          nor[i] = v.normal;
          a0[i] = v.attrs[0];
          a1[i] = v.attrs[1];
          a2[i] = v.attrs[2];
          a3[i] = v.attrs[3];
        }
      });

      gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
      GPU_vertbuf_data_alloc(*vbo, uint(n));
      GPU_vertbuf_attr_fill(vbo, a_ndc, ndc.data());
      GPU_vertbuf_attr_fill(vbo, a_wpos, wpos.data());
      GPU_vertbuf_attr_fill(vbo, a_nor, nor.data());
      GPU_vertbuf_attr_fill(vbo, a_a0, a0.data());
      GPU_vertbuf_attr_fill(vbo, a_a1, a1.data());
      GPU_vertbuf_attr_fill(vbo, a_a2, a2.data());
      GPU_vertbuf_attr_fill(vbo, a_a3, a3.data());

      gpu::Batch *batch = GPU_batch_create_ex(
          GPU_PRIM_TRIS, vbo, nullptr, GPU_BATCH_OWNS_VBO);

      const GPUDepthTest prev_depth = GPU_depth_test_get();
      const bool prev_depth_mask = GPU_depth_mask_get();
      GPU_depth_test(GPU_DEPTH_LESS);
      GPU_depth_mask(true);
      GPU_face_culling(GPU_CULL_NONE);
      GPU_blend(GPU_BLEND_NONE);

      GPU_batch_set_shader(batch, shader);
      GPU_batch_draw(batch);

      GPU_depth_test(prev_depth);
      GPU_depth_mask(prev_depth_mask);

      GPU_batch_discard(batch);
    }

    GPU_framebuffer_restore();
    GPU_framebuffer_free(fb);
    GPU_texture_free(depth_tx);

    /* Publish Position / Normal / Alpha. */
    if (need_pos) {
      Result &out = get_result("Position");
      out.share_data(pos_tex);
    }
    if (need_nor) {
      Result &out = get_result("Normal");
      out.share_data(nor_tex);
    }
    if (need_alpha) {
      Result &out = get_result("Alpha");
      extract_alpha(context(), alpha_color, out);
    }

    /* Publish attrs for Bundle consumers.
     * Prefer GPU texture cache (share_data path, same cost class as Position) — CPU
     * StampAttrMaps download was ~450ms vs ~100ms Position. */
    const std::string key = stamp_node_cache_key(node());
    if (need_stamp_maps) {
      if (StampAttrGpuCache *gpu_cache = active_stamp_attr_gpu_cache()) {
        StampAttrGpuMaps gpu_maps;
        gpu_maps.size = res;
        for (int i = 0; i < attr_n; i++) {
          if (!attr_tex[i].is_allocated() || !attr_tex[i].gpu_texture()) {
            continue;
          }
          StampAttrGpuTexture entry;
          entry.texture = attr_tex[i].gpu_texture();
          /* Hold a sharing ref so attr_tex.release() does not free the texture mid-cook. */
          entry.sharing_info = attr_tex[i].sharing_info();
          entry.size = res;
          gpu_maps.maps.add_overwrite(active_attrs[i], std::move(entry));
        }
        if (!gpu_maps.maps.is_empty()) {
          gpu_cache->store(key, std::move(gpu_maps));
        }
      }
      else if (StampAttrMapsCache *stamp_cache = active_stamp_attr_cache()) {
        /* CPU fallback when GPU attr cache unavailable. */
        StampAttrMaps stamp_maps;
        stamp_maps.size = res;
        for (int i = 0; i < attr_n; i++) {
          Result cpu = attr_tex[i].download_to_cpu();
          Array<float4> rgba(int64_t(res.x) * int64_t(res.y));
          parallel_for(res, [&](const int2 t) {
            const Color c = cpu.load_pixel_zero<Color>(t);
            const float4 v(c.r, c.g, c.b, c.a);
            rgba[int64_t(t.y) * res.x + t.x] = float4(v.xyz() * v.w, v.w);
          });
          stamp_maps.maps.add_overwrite(active_attrs[i], std::move(rgba));
          cpu.release();
        }
        stamp_cache->store(key, std::move(stamp_maps));
      }
    }

    /* Hidden typed outputs (directly linked): always GPU share / convert — no download. */
    for (int i = 0; i < attr_n; i++) {
      if (!this->should_compute_attr(active_attrs[i]) || !this->has_result(active_attrs[i])) {
        continue;
      }
      Result &out = get_result(active_attrs[i]);
      if (out.type() == ResultType::Color) {
        /* a≈1 for most mesh attrs → premul identity; share Color MRT. */
        out.share_data(attr_tex[i]);
      }
      else if (out.type() == ResultType::Float3 || out.type() == ResultType::Float4) {
        this->color_to_vector_gpu(attr_tex[i], out);
      }
      else if (out.type() == ResultType::Float) {
        this->color_to_float_gpu(attr_tex[i], out);
      }
      else {
        out.share_data(attr_tex[i]);
      }
    }

    if (need_bundle) {
      Result &out = get_result("Attributes");
      out.allocate_single_value();
      if (out.type() == ResultType::String) {
        out.set_single_value(key);
      }
    }

    pos_tex.release();
    nor_tex.release();
    alpha_color.release();
    for (int i = 0; i < RASTER_GPU_ATTR_SLOTS; i++) {
      attr_tex[i].release();
    }
    return true;
  }

  /** Premultiplied float4 map → typed Result (same rules as SeparateBundleImageOperation). */
  void publish_attr_from_premul_map(Result &out, const Array<float4> &pixels, const int2 size)
  {
    const ResultType out_type = out.type();
    Result cpu = context().create_result(out_type);
    cpu.allocate_texture(Domain(size), false, ResultStorageType::CPUImage);
    parallel_for(size, [&](const int2 texel) {
      const float4 v = pixels[int64_t(texel.y) * size.x + texel.x];
      const float inv_a = (v.w > 1e-8f) ? (1.0f / v.w) : 0.0f;
      switch (out_type) {
        case ResultType::Float:
          cpu.store_pixel(texel, v.x * inv_a);
          break;
        case ResultType::Float2:
          cpu.store_pixel(texel, float2(v.x, v.y) * inv_a);
          break;
        case ResultType::Float3:
          cpu.store_pixel(texel, float3(v.x, v.y, v.z) * inv_a);
          break;
        case ResultType::Float4:
          cpu.store_pixel(texel, float4(v.xyz() * inv_a, v.w));
          break;
        case ResultType::Color:
        default:
          cpu.store_pixel(texel, Color(v.x, v.y, v.z, v.w));
          break;
      }
    });
    if (context().use_gpu()) {
      Result g = cpu.upload_to_gpu(true);
      out.share_data(g);
      g.release();
    }
    else {
      out.share_data(cpu);
    }
    cpu.release();
  }

  void publish_color_premul_from_gpu(Result &src_color, Result &out)
  {
    Result cpu = src_color.download_to_cpu();
    Result dst = context().create_result(ResultType::Color);
    dst.allocate_texture(src_color.domain(), false, ResultStorageType::CPUImage);
    parallel_for(src_color.domain().data_size, [&](const int2 t) {
      const Color c = cpu.load_pixel_zero<Color>(t);
      dst.store_pixel(t, Color(c.r * c.a, c.g * c.a, c.b * c.a, c.a));
    });
    if (context().use_gpu()) {
      Result g = dst.upload_to_gpu(true);
      out.share_data(g);
      g.release();
    }
    else {
      out.share_data(dst);
    }
    dst.release();
    cpu.release();
  }

  bool should_compute_out(const StringRef id)
  {
    const bNodeSocket *sock = find_out(id);
    if (!sock || !sock->is_available() || !this->has_result(id)) {
      return false;
    }
    return this->get_result(id).should_compute();
  }

  bool should_compute_attr(const StringRef name)
  {
    if (!this->has_result(name)) {
      return false;
    }
    const bNodeSocket *sock = find_out(name);
    if (!sock || !sock->is_available()) {
      return false;
    }
    return this->get_result(name).should_compute();
  }

  void color_to_float_gpu(const Result &color_in, Result &float_out)
  {
    /* Prefer .r (scalar attrs store value in rgb via col_from). Use convert with R-only luma. */
    gpu::Shader *shader = context().get_shader("compositor_convert_color_to_float");
    if (!shader) {
      float_out.allocate_invalid();
      return;
    }
    GPU_shader_bind(shader);
    const float luma[3] = {1.0f, 0.0f, 0.0f};
    GPU_shader_uniform_3fv(shader, "luminance_coefficients_u", luma);
    color_in.bind_as_texture(shader, "input_tx");
    float_out.allocate_texture(color_in.domain());
    float_out.bind_as_image(shader, "output_img");
    compute_dispatch_threads_at_least(shader, color_in.domain().data_size);
    color_in.unbind_as_texture();
    float_out.unbind_as_image();
    GPU_shader_unbind();
  }

  void color_to_vector_gpu(const Result &color_in, Result &vec_out)
  {
    const char *info = (vec_out.type() == ResultType::Float2) ?
                           "compositor_convert_color_to_float2" :
                       (vec_out.type() == ResultType::Float4) ?
                           "compositor_convert_color_to_float4" :
                           "compositor_convert_color_to_float3";
    gpu::Shader *shader = context().get_shader(info);
    if (!shader) {
      /* Fallback: share as Color if convert missing (should not happen). */
      vec_out.share_data(color_in);
      return;
    }
    GPU_shader_bind(shader);
    color_in.bind_as_texture(shader, "input_tx");
    vec_out.allocate_texture(color_in.domain());
    vec_out.bind_as_image(shader, "output_img");
    compute_dispatch_threads_at_least(shader, color_in.domain().data_size);
    color_in.unbind_as_texture();
    vec_out.unbind_as_image();
    GPU_shader_unbind();
  }

  void execute_cpu(const Object *object,
                   const Mesh *mesh,
                   const float4x4 &object_to_world,
                   Depsgraph *dg,
                   RasterizeMode mode,
                   const float3 plane_pos,
                   const float3 plane_dir,
                   float plane_size,
                   const StringRef uv_map,
                   const int2 res,
                   const Span<std::string> attr_names)
  {
    /* Same active-attr filter as GPU: avoid rasterizing every multi-selected attr when
     * only Position or one Separate Bundle item is consumed. */
    const bool need_bundle = this->should_compute_out("Attributes");
    const Vector<std::string> bundle_wanted =
        need_bundle ? collect_bundle_consumer_attr_names(this->node()) : Vector<std::string>{};
    const bool bundle_wants_all = need_bundle && bundle_wanted.is_empty();
    Vector<std::string> active_attrs;
    active_attrs.reserve(attr_names.size());
    for (const std::string &name : attr_names) {
      const bool typed = this->should_compute_attr(name);
      bool via_bundle = false;
      if (need_bundle) {
        if (bundle_wants_all) {
          via_bundle = true;
        }
        else {
          for (const std::string &w : bundle_wanted) {
            if (w == name) {
              via_bundle = true;
              break;
            }
          }
        }
      }
      if (typed || via_bundle) {
        active_attrs.append(name);
      }
    }

    const uint64_t cache_key = rasterize_cache_key(object,
                                                   mesh,
                                                   object_to_world,
                                                   dg,
                                                   mode,
                                                   plane_pos,
                                                   plane_dir,
                                                   plane_size,
                                                   uv_map,
                                                   res,
                                                   active_attrs);
    RasterizeResultCache &r_cache = rasterize_result_caches().lookup_or_add_default(
        this->node().identifier);

    const bool cache_hit = (r_cache.key == cache_key && r_cache.buf.size == res &&
                            !r_cache.buf.alpha.is_empty());
    if (!cache_hit) {
      rasterize_geometry(mesh,
                         object_to_world,
                         mode,
                         plane_pos,
                         plane_dir,
                         plane_size,
                         uv_map,
                         res,
                         active_attrs,
                         r_cache.buf);
      r_cache.key = cache_key;
    }
    const RasterBuffers &buf = r_cache.buf;
    const Domain rdom(buf.size);

    if (const bNodeSocket *sock = find_out("Position")) {
      if (sock->is_available()) {
        Result &out = get_result("Position");
        if (out.should_compute()) {
          Result cpu = context().create_result(ResultType::Float3);
          cpu.allocate_texture(rdom, false, ResultStorageType::CPUImage);
          parallel_for(buf.size, [&](int2 t) {
            cpu.store_pixel(t, buf.position[buf.idx(t.x, t.y)]);
          });
          if (context().use_gpu()) {
            Result g = cpu.upload_to_gpu(true);
            out.share_data(g);
            g.release();
          }
          else {
            out.share_data(cpu);
          }
          cpu.release();
        }
      }
    }
    if (const bNodeSocket *sock = find_out("Normal")) {
      if (sock->is_available()) {
        Result &out = get_result("Normal");
        if (out.should_compute()) {
          Result cpu = context().create_result(ResultType::Float3);
          cpu.allocate_texture(rdom, false, ResultStorageType::CPUImage);
          parallel_for(buf.size, [&](int2 t) {
            cpu.store_pixel(t, buf.normal[buf.idx(t.x, t.y)]);
          });
          if (context().use_gpu()) {
            Result g = cpu.upload_to_gpu(true);
            out.share_data(g);
            g.release();
          }
          else {
            out.share_data(cpu);
          }
          cpu.release();
        }
      }
    }
    if (const bNodeSocket *sock = find_out("Alpha")) {
      if (sock->is_available()) {
        Result &out = get_result("Alpha");
        if (out.should_compute()) {
          Result cpu = context().create_result(ResultType::Float);
          cpu.allocate_texture(rdom, false, ResultStorageType::CPUImage);
          parallel_for(buf.size, [&](int2 t) {
            cpu.store_pixel(t, buf.alpha[buf.idx(t.x, t.y)]);
          });
          if (context().use_gpu()) {
            Result g = cpu.upload_to_gpu(true);
            out.share_data(g);
            g.release();
          }
          else {
            out.share_data(cpu);
          }
          cpu.release();
        }
      }
    }
    for (int i : buf.attr_names.index_range()) {
      const bNodeSocket *sock = find_out(buf.attr_names[i]);
      if (!sock || !sock->is_available() || !this->has_result(buf.attr_names[i])) {
        continue;
      }
      Result &out = get_result(buf.attr_names[i]);
      if (!out.should_compute() || i >= buf.attrs.size()) {
        continue;
      }
      ResultType rt = out.type();
      Result cpu = context().create_result(rt);
      cpu.allocate_texture(rdom, false, ResultStorageType::CPUImage);
      parallel_for(buf.size, [&](int2 t) {
        float4 v = buf.attrs[i][buf.idx(t.x, t.y)];
        switch (rt) {
          case ResultType::Float:
            cpu.store_pixel(t, v.x);
            break;
          case ResultType::Float3:
            cpu.store_pixel(t, v.xyz());
            break;
          case ResultType::Color:
            cpu.store_pixel(t, Color(v.x * v.w, v.y * v.w, v.z * v.w, v.w));
            break;
          default:
            cpu.store_pixel(t, Color(v.x * v.w, v.y * v.w, v.z * v.w, v.w));
            break;
        }
      });
      if (context().use_gpu()) {
        Result g = cpu.upload_to_gpu(true);
        out.share_data(g);
        g.release();
      }
      else {
        out.share_data(cpu);
      }
      cpu.release();
    }
    std::string key = stamp_node_cache_key(node());
    if (need_bundle) {
      if (StampAttrMapsCache *stamp_cache = active_stamp_attr_cache()) {
        StampAttrMaps maps;
        maps.size = buf.size;
        for (int i : buf.attr_names.index_range()) {
          if (i >= buf.attrs.size()) {
            continue;
          }
          Array<float4> rgba(buf.attrs[i].size());
          for (int64_t p : buf.attrs[i].index_range()) {
            float4 v = buf.attrs[i][p];
            rgba[p] = float4(v.xyz() * v.w, v.w);
          }
          maps.maps.add_overwrite(buf.attr_names[i], std::move(rgba));
        }
        stamp_cache->store(key, std::move(maps));
      }
    }
    if (const bNodeSocket *sock = find_out("Attributes")) {
      if (sock->is_available()) {
        Result &out = get_result("Attributes");
        if (out.should_compute()) {
          out.allocate_single_value();
          if (out.type() == ResultType::String) {
            out.set_single_value(key);
          }
        }
      }
    }
  }
};

static NodeOperation *get_compositor_operation(Context &ctx, const bNode &node)
{
  return new RasterizeGeometryOperation(ctx, node);
}

static void node_register()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeRasterizeGeometry"_ustr);
  ntype.ui_name = "Rasterize Geometry";
  ntype.ui_description =
      "Rasterize mesh to image-space textures (GPU hardware raster + depth). "
      "Geometry from Import Geo takes priority over Object. "
      "Outputs Position, Normal, Alpha, and a Bundle of attributes.";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.updatefunc = node_update;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_rasterize_geometry_cc
