/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_map.hh"
#include "BLI_math_vector.hh"
#include "BLI_set.hh"
#include "BLI_task.hh"

#include "DNA_mesh_types.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "GEO_mesh_loop_subdivide.hh"

namespace blender::geometry {
namespace {

struct PointMask {
  Vector<int, 8> indices;
  Vector<float, 8> weights;
};

struct EdgeInfo {
  int opposite[2] = {-1, -1};
  int face_count = 0;
};

static int2 ordered_edge(const int a, const int b)
{
  return a < b ? int2(a, b) : int2(b, a);
}

static void add_mask_weight(PointMask &mask, const int index, const float weight)
{
  for (const int i : mask.indices.index_range()) {
    if (mask.indices[i] == index) {
      mask.weights[i] += weight;
      return;
    }
  }
  mask.indices.append(index);
  mask.weights.append(weight);
}

static int nearest_source(const PointMask &mask)
{
  int best = 0;
  for (const int i : mask.indices.index_range().drop_front(1)) {
    if (mask.weights[i] > mask.weights[best] ||
        (mask.weights[i] == mask.weights[best] && mask.indices[i] < mask.indices[best]))
    {
      best = i;
    }
  }
  return mask.indices[best];
}

static void mix_mask_value(const GVArray &src, const PointMask &mask, void *dst)
{
  const CPPType &type = src.type();
  bool mixed = false;
  bke::attribute_math::to_static_type(type, [&]<typename T>() {
    if constexpr (!std::is_void_v<bke::attribute_math::DefaultMixer<T>>) {
      bke::attribute_math::DefaultMixer<T> mixer({static_cast<T *>(dst), 1});
      for (const int i : mask.indices.index_range()) {
        T value;
        src.get(mask.indices[i], &value);
        mixer.mix_in(0, value, mask.weights[i]);
      }
      mixer.finalize();
      mixed = true;
    }
  });
  if (!mixed) {
    src.get(nearest_source(mask), dst);
  }
}

static void fill_point_domain(const Mesh &src_mesh,
                              const Span<PointMask> masks,
                              const bke::AttributeFilter &attribute_filter,
                              Mesh &dst_mesh)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();
  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != bke::AttrDomain::Point || iter.name == "position" ||
        attribute_filter.allow_skip(iter.name))
    {
      return;
    }
    const GVArray src = *iter.get();
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        iter.name, bke::AttrDomain::Point, iter.data_type);
    if (!dst) {
      return;
    }
    for (const int i : masks.index_range()) {
      mix_mask_value(src, masks[i], dst.span[i]);
    }
    dst.finish();
  });
}

static void fill_edge_domain(const Mesh &src_mesh,
                             const Span<int> dst_to_src_edge,
                             const bke::AttributeFilter &attribute_filter,
                             Mesh &dst_mesh)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();
  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != bke::AttrDomain::Edge || iter.name == ".edge_verts" ||
        attribute_filter.allow_skip(iter.name))
    {
      return;
    }
    const GVArray src = *iter.get();
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        iter.name, bke::AttrDomain::Edge, iter.data_type);
    if (!dst) {
      return;
    }
    for (const int i : dst_to_src_edge.index_range()) {
      src.get(dst_to_src_edge[i], dst.span[i]);
    }
    dst.finish();
  });
}

static void fill_face_domain(const Mesh &src_mesh,
                             const Span<int> dst_to_src_face,
                             const bke::AttributeFilter &attribute_filter,
                             Mesh &dst_mesh)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();
  for (auto &attribute : bke::retrieve_attributes_for_transfer(
           src_attributes, dst_attributes, {bke::AttrDomain::Face}, attribute_filter))
  {
    bke::attribute_math::gather(attribute.src, dst_to_src_face, attribute.dst.span);
    attribute.dst.finish();
  }
}

static void fill_corner_domain(const Mesh &src_mesh,
                               const Span<PointMask> corner_masks,
                               const bke::AttributeFilter &attribute_filter,
                               Mesh &dst_mesh)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();
  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != bke::AttrDomain::Corner ||
        ELEM(iter.name, ".corner_vert", ".corner_edge") ||
        attribute_filter.allow_skip(iter.name))
    {
      return;
    }
    const GVArray src = *iter.get();
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        iter.name, bke::AttrDomain::Corner, iter.data_type);
    if (!dst) {
      return;
    }
    for (const int i : corner_masks.index_range()) {
      mix_mask_value(src, corner_masks[i], dst.span[i]);
    }
    dst.finish();
  });
}

static Mesh *subdivide_once(const Mesh &src_mesh,
                            const bke::AttributeFilter &attribute_filter)
{
  const Span<float3> src_positions = src_mesh.vert_positions();
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();

  Map<int2, int> edge_by_verts;
  edge_by_verts.reserve(src_mesh.edges_num);
  for (const int edge_i : src_edges.index_range()) {
    edge_by_verts.add_overwrite(ordered_edge(src_edges[edge_i][0], src_edges[edge_i][1]), edge_i);
  }

  Array<EdgeInfo> edge_info(src_mesh.edges_num);
  for (const int face_i : src_faces.index_range()) {
    const IndexRange face = src_faces[face_i];
    BLI_assert(face.size() == 3);
    for (const int local : IndexRange(3)) {
      const int edge = src_corner_edges[face[local]];
      EdgeInfo &info = edge_info[edge];
      if (info.face_count < 2) {
        info.opposite[info.face_count] = src_corner_verts[face[(local + 2) % 3]];
      }
      info.face_count++;
    }
  }

  Array<int> vert_to_edge_offsets;
  Array<int> vert_to_edge_indices;
  const GroupedSpan<int> vert_to_edge = bke::mesh::build_vert_to_edge_map(
      src_edges, src_mesh.verts_num, vert_to_edge_offsets, vert_to_edge_indices);

  Array<PointMask> point_masks(src_mesh.verts_num + src_mesh.edges_num);
  for (const int vert : IndexRange(src_mesh.verts_num)) {
    Vector<int, 8> boundary_neighbors;
    for (const int edge : vert_to_edge[vert]) {
      if (edge_info[edge].face_count != 2) {
        boundary_neighbors.append(bke::mesh::edge_other_vert(src_edges[edge], vert));
      }
    }
    PointMask &mask = point_masks[vert];
    if (boundary_neighbors.size() == 2) {
      add_mask_weight(mask, vert, 0.75f);
      add_mask_weight(mask, boundary_neighbors[0], 0.125f);
      add_mask_weight(mask, boundary_neighbors[1], 0.125f);
    }
    else if (boundary_neighbors.size() > 2 || vert_to_edge[vert].is_empty()) {
      add_mask_weight(mask, vert, 1.0f);
    }
    else {
      const int valence = vert_to_edge[vert].size();
      const float beta = valence == 3 ? 3.0f / 16.0f : 3.0f / (8.0f * float(valence));
      add_mask_weight(mask, vert, 1.0f - float(valence) * beta);
      for (const int edge : vert_to_edge[vert]) {
        add_mask_weight(mask, bke::mesh::edge_other_vert(src_edges[edge], vert), beta);
      }
    }
  }
  for (const int edge : src_edges.index_range()) {
    PointMask &mask = point_masks[src_mesh.verts_num + edge];
    if (edge_info[edge].face_count == 2) {
      add_mask_weight(mask, src_edges[edge][0], 3.0f / 8.0f);
      add_mask_weight(mask, src_edges[edge][1], 3.0f / 8.0f);
      add_mask_weight(mask, edge_info[edge].opposite[0], 1.0f / 8.0f);
      add_mask_weight(mask, edge_info[edge].opposite[1], 1.0f / 8.0f);
    }
    else {
      add_mask_weight(mask, src_edges[edge][0], 0.5f);
      add_mask_weight(mask, src_edges[edge][1], 0.5f);
    }
  }

  Vector<int3> dst_tris;
  Vector<int> dst_to_src_face;
  Vector<PointMask> corner_masks;
  dst_tris.reserve(src_mesh.faces_num * 4);
  dst_to_src_face.reserve(src_mesh.faces_num * 4);
  corner_masks.reserve(src_mesh.faces_num * 12);

  for (const int face_i : src_faces.index_range()) {
    const IndexRange face = src_faces[face_i];
    const int v0 = src_corner_verts[face[0]];
    const int v1 = src_corner_verts[face[1]];
    const int v2 = src_corner_verts[face[2]];
    const int e01 = edge_by_verts.lookup(ordered_edge(v0, v1));
    const int e12 = edge_by_verts.lookup(ordered_edge(v1, v2));
    const int e20 = edge_by_verts.lookup(ordered_edge(v2, v0));
    const int m01 = src_mesh.verts_num + e01;
    const int m12 = src_mesh.verts_num + e12;
    const int m20 = src_mesh.verts_num + e20;

    dst_tris.append({v0, m01, m20});
    dst_tris.append({v1, m12, m01});
    dst_tris.append({v2, m20, m12});
    dst_tris.append({m01, m12, m20});
    for ([[maybe_unused]] const int i : IndexRange(4)) {
      dst_to_src_face.append(face_i);
    }

    PointMask c0, c1, c2, c01, c12, c20;
    add_mask_weight(c0, face[0], 1.0f);
    add_mask_weight(c1, face[1], 1.0f);
    add_mask_weight(c2, face[2], 1.0f);
    add_mask_weight(c01, face[0], 0.5f);
    add_mask_weight(c01, face[1], 0.5f);
    add_mask_weight(c12, face[1], 0.5f);
    add_mask_weight(c12, face[2], 0.5f);
    add_mask_weight(c20, face[2], 0.5f);
    add_mask_weight(c20, face[0], 0.5f);
    corner_masks.append(c0);
    corner_masks.append(c01);
    corner_masks.append(c20);
    corner_masks.append(c1);
    corner_masks.append(c12);
    corner_masks.append(c01);
    corner_masks.append(c2);
    corner_masks.append(c20);
    corner_masks.append(c12);
    corner_masks.append(c01);
    corner_masks.append(c12);
    corner_masks.append(c20);
  }

  Set<int2> surface_edges;
  surface_edges.reserve(dst_tris.size() * 3);
  for (const int3 &tri : dst_tris) {
    surface_edges.add(ordered_edge(tri[0], tri[1]));
    surface_edges.add(ordered_edge(tri[1], tri[2]));
    surface_edges.add(ordered_edge(tri[2], tri[0]));
  }

  Vector<int2> dst_edges;
  Vector<int> dst_to_src_edge;
  dst_edges.reserve(src_mesh.edges_num * 2 + surface_edges.size());
  dst_to_src_edge.reserve(src_mesh.edges_num * 2 + surface_edges.size());
  Map<int2, int> dst_edge_by_verts;
  auto add_edge = [&](const int2 edge, const int src_edge) {
    const int2 key = ordered_edge(edge[0], edge[1]);
    if (dst_edge_by_verts.contains(key)) {
      return;
    }
    dst_edge_by_verts.add(key, dst_edges.size());
    dst_edges.append(edge);
    dst_to_src_edge.append(src_edge);
  };
  for (const int edge : src_edges.index_range()) {
    const int midpoint = src_mesh.verts_num + edge;
    add_edge({src_edges[edge][0], midpoint}, edge);
    add_edge({midpoint, src_edges[edge][1]}, edge);
  }
  for (const int3 &tri : dst_tris) {
    const int2 tri_edges[3] = {{tri[0], tri[1]}, {tri[1], tri[2]}, {tri[2], tri[0]}};
    for (const int2 edge : tri_edges) {
      if (dst_edge_by_verts.contains(ordered_edge(edge[0], edge[1]))) {
        continue;
      }
      int src_edge = -1;
      if (edge[0] >= src_mesh.verts_num && edge[1] >= src_mesh.verts_num) {
        const int coarse_a = edge[0] - src_mesh.verts_num;
        const int coarse_b = edge[1] - src_mesh.verts_num;
        src_edge = std::min(coarse_a, coarse_b);
      }
      else {
        const int midpoint = std::max(edge[0], edge[1]) - src_mesh.verts_num;
        src_edge = std::max(midpoint, 0);
      }
      add_edge(edge, src_edge);
    }
  }

  Mesh *dst_mesh = bke::mesh_new_no_attributes(
      point_masks.size(), dst_edges.size(), dst_tris.size(), dst_tris.size() * 3);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();
  dst_attributes.add<float3>(
      "position", bke::AttrDomain::Point, bke::AttributeInitConstruct());
  dst_attributes.add<int2>(
      ".edge_verts", bke::AttrDomain::Edge, bke::AttributeInitConstruct());
  dst_attributes.add<int>(
      ".corner_vert", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  dst_attributes.add<int>(
      ".corner_edge", bke::AttrDomain::Corner, bke::AttributeInitConstruct());

  MutableSpan<float3> dst_positions = dst_mesh->vert_positions_for_write();
  for (const int i : point_masks.index_range()) {
    float3 position(0.0f);
    const PointMask &mask = point_masks[i];
    for (const int j : mask.indices.index_range()) {
      position += src_positions[mask.indices[j]] * mask.weights[j];
    }
    dst_positions[i] = position;
  }
  dst_mesh->edges_for_write().copy_from(dst_edges);
  offset_indices::fill_constant_group_size(3, 0, dst_mesh->face_offsets_for_write());
  MutableSpan<int> dst_corner_verts = dst_mesh->corner_verts_for_write();
  MutableSpan<int> dst_corner_edges = dst_mesh->corner_edges_for_write();
  for (const int face_i : dst_tris.index_range()) {
    const int3 tri = dst_tris[face_i];
    for (const int local : IndexRange(3)) {
      const int corner = face_i * 3 + local;
      const int v1 = tri[local];
      const int v2 = tri[(local + 1) % 3];
      dst_corner_verts[corner] = v1;
      dst_corner_edges[corner] = dst_edge_by_verts.lookup(ordered_edge(v1, v2));
    }
  }

  fill_point_domain(src_mesh, point_masks, attribute_filter, *dst_mesh);
  fill_edge_domain(src_mesh, dst_to_src_edge, attribute_filter, *dst_mesh);
  fill_face_domain(src_mesh, dst_to_src_face, attribute_filter, *dst_mesh);
  fill_corner_domain(src_mesh, corner_masks, attribute_filter, *dst_mesh);
  dst_mesh->tag_topology_changed();
  return dst_mesh;
}

}  // namespace

Mesh *mesh_loop_subdivide(const Mesh &src_mesh,
                          const int level,
                          const bke::AttributeFilter &attribute_filter)
{
  if (level <= 0) {
    return BKE_mesh_copy_for_eval(src_mesh);
  }
  Mesh *result = subdivide_once(src_mesh, attribute_filter);
  for ([[maybe_unused]] const int i : IndexRange(1, level - 1)) {
    Mesh *next = subdivide_once(*result, attribute_filter);
    BKE_id_free(nullptr, result);
    result = next;
  }
  return result;
}

}  // namespace blender::geometry