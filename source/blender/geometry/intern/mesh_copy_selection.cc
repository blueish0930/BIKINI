/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo
 */

#include <algorithm>

#include "DNA_object_types.h"

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask.hh"
#include "BLI_listbase.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"
#include "PRF_profile.hh"

#include "BKE_attribute.hh"
#include "BKE_deform.hh"
#include "BKE_mesh.hh"

#include "GEO_mesh_copy_selection.hh"
#include "GEO_mesh_selection.hh"

namespace blender::geometry {

static void remap_verts(const OffsetIndices<int> src_faces,
                        const OffsetIndices<int> dst_faces,
                        const int src_verts_num,
                        const IndexMask &vert_mask,
                        const IndexMask &edge_mask,
                        const IndexMask &face_mask,
                        const Span<int2> src_edges,
                        const Span<int> src_corner_verts,
                        MutableSpan<int2> dst_edges,
                        MutableSpan<int> dst_corner_verts)
{
  Array<int> map(src_verts_num);
  index_mask::build_reverse_map<int>(vert_mask, map);
  threading::parallel_invoke(
      vert_mask.size() > 1024,
      [&]() {
        face_mask.foreach_index(
            [&](const int64_t src_i, const int64_t dst_i) {
              const IndexRange src_face = src_faces[src_i];
              const IndexRange dst_face = dst_faces[dst_i];
              for (const int i : src_face.index_range()) {
                dst_corner_verts[dst_face[i]] = map[src_corner_verts[src_face[i]]];
              }
            },
            exec_mode::grain_size(512));
      },
      [&]() {
        edge_mask.foreach_index(
            [&](const int64_t src_i, const int64_t dst_i) {
              dst_edges[dst_i][0] = map[src_edges[src_i][0]];
              dst_edges[dst_i][1] = map[src_edges[src_i][1]];
            },
            exec_mode::grain_size(512));
      });
}

static void remap_edges(const OffsetIndices<int> src_faces,
                        const OffsetIndices<int> dst_faces,
                        const int src_edges_num,
                        const IndexMask &edge_mask,
                        const IndexMask &face_mask,
                        const Span<int> src_corner_edges,
                        MutableSpan<int> dst_corner_edges)
{
  Array<int> map(src_edges_num);
  index_mask::build_reverse_map<int>(edge_mask, map);
  face_mask.foreach_index(
      [&](const int64_t src_i, const int64_t dst_i) {
        const IndexRange src_face = src_faces[src_i];
        const IndexRange dst_face = dst_faces[dst_i];
        for (const int i : src_face.index_range()) {
          dst_corner_edges[dst_face[i]] = map[src_corner_edges[src_face[i]]];
        }
      },
      exec_mode::grain_size(512));
}

static void copy_loose_vert_hint(const Mesh &src, Mesh &dst)
{
  const auto &src_cache = src.runtime->loose_verts_cache;
  if (src_cache.is_cached() && src_cache.data().mask.is_empty()) {
    dst.tag_loose_verts_none();
  }
}

static void copy_loose_edge_hint(const Mesh &src, Mesh &dst)
{
  const auto &src_cache = src.runtime->loose_edges_cache;
  if (src_cache.is_cached() && src_cache.data().mask.is_empty()) {
    dst.tag_loose_edges_none();
  }
}

static void copy_overlapping_hint(const Mesh &src, Mesh &dst)
{
  if (src.no_overlapping_topology()) {
    dst.tag_overlapping_none();
  }
}

/** Gather vertex group data and array attributes in separate loops. */
static void gather_vert_attributes(const Mesh &mesh_src,
                                   const bke::AttributeFilter &attribute_filter,
                                   const IndexMask &vert_mask,
                                   Mesh &mesh_dst)
{
  Set<std::string> vertex_group_names;
  for (bDeformGroup &group : mesh_src.vertex_group_names) {
    vertex_group_names.add(group.name);
  }

  const Span<MDeformVert> src = mesh_src.deform_verts();
  if (!vertex_group_names.is_empty() && !src.is_empty()) {
    MutableSpan<MDeformVert> dst = mesh_dst.deform_verts_for_write();
    bke::gather_deform_verts(src, vert_mask, dst);
  }

  bke::gather_attributes(mesh_src.attributes(),
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         bke::attribute_filter_with_skip_ref(attribute_filter, vertex_group_names),
                         vert_mask,
                         mesh_dst.attributes_for_write());
}

std::optional<Mesh *> mesh_copy_selection(const Mesh &src_mesh,
                                          const VArray<bool> &selection,
                                          const bke::AttrDomain selection_domain,
                                          const bke::AttributeFilter &attribute_filter)
{
  PRF_scope(ProfileCategory::Default);
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();

  if (selection.is_empty()) {
    return std::nullopt;
  }
  if (const std::optional<bool> single = selection.get_if_single()) {
    return *single ? std::nullopt : std::make_optional<Mesh *>(nullptr);
  }

  threading::EnumerableThreadSpecific<IndexMaskMemory> memory;
  IndexMask vert_mask;
  IndexMask edge_mask;
  IndexMask face_mask;
  switch (selection_domain) {
    case bke::AttrDomain::Point: {
      const VArraySpan<bool> span(selection);
      threading::parallel_invoke(
          src_mesh.verts_num > 1024,
          [&]() { vert_mask = IndexMask::from_bools(span, memory.local()); },
          [&]() { edge_mask = edge_selection_from_vert(src_edges, span, memory.local()); },
          [&]() {
            face_mask = face_selection_from_vert(
                src_faces, src_corner_verts, span, memory.local());
          });
      break;
    }
    case bke::AttrDomain::Edge: {
      const VArraySpan<bool> span(selection);
      threading::parallel_invoke(
          src_edges.size() > 1024,
          [&]() {
            edge_mask = IndexMask::from_bools(span, memory.local());
            vert_mask = vert_selection_from_edge(
                src_edges, edge_mask, src_mesh.verts_num, memory.local());
          },
          [&]() {
            face_mask = face_selection_from_edge(
                src_faces, src_corner_edges, span, memory.local());
          });
      break;
    }
    case bke::AttrDomain::Face: {
      const VArraySpan<bool> span(selection);
      face_mask = IndexMask::from_bools(span, memory.local());
      threading::parallel_invoke(
          face_mask.size() > 1024,
          [&]() {
            vert_mask = vert_selection_from_face(
                src_faces, face_mask, src_corner_verts, src_mesh.verts_num, memory.local());
          },
          [&]() {
            edge_mask = edge_selection_from_face(
                src_faces, face_mask, src_corner_edges, src_mesh.edges_num, memory.local());
          });
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }

  if (vert_mask.is_empty()) {
    return nullptr;
  }
  const bool same_verts = vert_mask.size() == src_mesh.verts_num;
  const bool same_edges = edge_mask.size() == src_mesh.edges_num;
  const bool same_faces = face_mask.size() == src_mesh.faces_num;
  if (same_verts && same_edges && same_faces) {
    return std::nullopt;
  }

  Mesh *dst_mesh = bke::mesh_new_no_attributes(
      vert_mask.size(), edge_mask.size(), face_mask.size(), 0);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();
  dst_attributes.add<int2>(".edge_verts", bke::AttrDomain::Edge, bke::AttributeInitConstruct());
  MutableSpan<int2> dst_edges = dst_mesh->edges_for_write();

  const OffsetIndices<int> dst_faces = offset_indices::gather_selected_offsets(
      src_faces, face_mask, dst_mesh->face_offsets_for_write());
  dst_mesh->corners_num = dst_faces.total_size();
  dst_attributes.add<int>(".corner_vert", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  dst_attributes.add<int>(".corner_edge", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  MutableSpan<int> dst_corner_verts = dst_mesh->corner_verts_for_write();
  MutableSpan<int> dst_corner_edges = dst_mesh->corner_edges_for_write();

  threading::parallel_invoke(
      vert_mask.size() > 1024,
      [&]() {
        remap_verts(src_faces,
                    dst_faces,
                    src_mesh.verts_num,
                    vert_mask,
                    edge_mask,
                    face_mask,
                    src_edges,
                    src_corner_verts,
                    dst_edges,
                    dst_corner_verts);
      },
      [&]() {
        remap_edges(src_faces,
                    dst_faces,
                    src_edges.size(),
                    edge_mask,
                    face_mask,
                    src_corner_edges,
                    dst_corner_edges);
      },
      [&]() {
        gather_vert_attributes(src_mesh, attribute_filter, vert_mask, *dst_mesh);
        bke::gather_attributes(
            src_attributes,
            bke::AttrDomain::Edge,
            bke::AttrDomain::Edge,
            bke::attribute_filter_with_skip_ref(attribute_filter, {".edge_verts"}),
            edge_mask,
            dst_attributes);
        bke::gather_attributes(src_attributes,
                               bke::AttrDomain::Face,
                               bke::AttrDomain::Face,
                               attribute_filter,
                               face_mask,
                               dst_attributes);
        bke::gather_attributes_group_to_group(
            src_attributes,
            bke::AttrDomain::Corner,
            bke::AttrDomain::Corner,
            bke::attribute_filter_with_skip_ref(attribute_filter,
                                                {".corner_edge", ".corner_vert"}),
            src_faces,
            dst_faces,
            face_mask,
            dst_attributes);
      });

  if (selection_domain == bke::AttrDomain::Edge) {
    copy_loose_vert_hint(src_mesh, *dst_mesh);
  }
  else if (selection_domain == bke::AttrDomain::Face) {
    copy_loose_vert_hint(src_mesh, *dst_mesh);
    copy_loose_edge_hint(src_mesh, *dst_mesh);
  }
  copy_overlapping_hint(src_mesh, *dst_mesh);

  return dst_mesh;
}

namespace {

/**
 * Maps from indices in the source mesh to indices in the group that is being copied. Entries are
 * -1 outside of that group and are reset after it, so the arrays are only filled once per thread.
 */
struct FaceGroupCopyData {
  Array<int> vert_map;
  Array<int> edge_map;
  Vector<int> verts;
  Vector<int> edges;
};

}  // namespace

static Mesh *copy_face_group(const Mesh &src_mesh,
                             const IndexMask &face_mask,
                             const bke::AttributeFilter &attribute_filter,
                             FaceGroupCopyData &data)
{
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();

  data.verts.clear();
  data.edges.clear();
  face_mask.foreach_index([&](const int64_t face_i) {
    for (const int corner : src_faces[face_i]) {
      const int vert = src_corner_verts[corner];
      if (data.vert_map[vert] == -1) {
        data.vert_map[vert] = 0;
        data.verts.append(vert);
      }
      const int edge = src_corner_edges[corner];
      if (data.edge_map[edge] == -1) {
        data.edge_map[edge] = 0;
        data.edges.append(edge);
      }
    }
  });
  /* Keep the order of the source mesh, like #mesh_copy_selection does. */
  std::sort(data.verts.begin(), data.verts.end());
  std::sort(data.edges.begin(), data.edges.end());
  for (const int i : data.verts.index_range()) {
    data.vert_map[data.verts[i]] = i;
  }
  for (const int i : data.edges.index_range()) {
    data.edge_map[data.edges[i]] = i;
  }

  IndexMaskMemory memory;
  const IndexMask vert_mask = IndexMask::from_indices<int>(data.verts, memory);
  const IndexMask edge_mask = IndexMask::from_indices<int>(data.edges, memory);

  Mesh *dst_mesh = bke::mesh_new_no_attributes(
      vert_mask.size(), edge_mask.size(), face_mask.size(), 0);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();
  dst_attributes.add<int2>(".edge_verts", bke::AttrDomain::Edge, bke::AttributeInitConstruct());
  MutableSpan<int2> dst_edges = dst_mesh->edges_for_write();

  const OffsetIndices<int> dst_faces = offset_indices::gather_selected_offsets(
      src_faces, face_mask, dst_mesh->face_offsets_for_write());
  dst_mesh->corners_num = dst_faces.total_size();
  dst_attributes.add<int>(".corner_vert", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  dst_attributes.add<int>(".corner_edge", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  MutableSpan<int> dst_corner_verts = dst_mesh->corner_verts_for_write();
  MutableSpan<int> dst_corner_edges = dst_mesh->corner_edges_for_write();

  face_mask.foreach_index([&](const int64_t src_i, const int64_t dst_i) {
    const IndexRange src_face = src_faces[src_i];
    const IndexRange dst_face = dst_faces[dst_i];
    for (const int i : src_face.index_range()) {
      dst_corner_verts[dst_face[i]] = data.vert_map[src_corner_verts[src_face[i]]];
      dst_corner_edges[dst_face[i]] = data.edge_map[src_corner_edges[src_face[i]]];
    }
  });
  for (const int i : data.edges.index_range()) {
    const int2 src_edge = src_edges[data.edges[i]];
    dst_edges[i] = int2(data.vert_map[src_edge[0]], data.vert_map[src_edge[1]]);
  }

  gather_vert_attributes(src_mesh, attribute_filter, vert_mask, *dst_mesh);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Edge,
                         bke::AttrDomain::Edge,
                         bke::attribute_filter_with_skip_ref(attribute_filter, {".edge_verts"}),
                         edge_mask,
                         dst_attributes);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Face,
                         bke::AttrDomain::Face,
                         attribute_filter,
                         face_mask,
                         dst_attributes);
  bke::gather_attributes_group_to_group(
      src_attributes,
      bke::AttrDomain::Corner,
      bke::AttrDomain::Corner,
      bke::attribute_filter_with_skip_ref(attribute_filter, {".corner_edge", ".corner_vert"}),
      src_faces,
      dst_faces,
      face_mask,
      dst_attributes);

  copy_loose_vert_hint(src_mesh, *dst_mesh);
  copy_loose_edge_hint(src_mesh, *dst_mesh);
  copy_overlapping_hint(src_mesh, *dst_mesh);

  for (const int vert : data.verts) {
    data.vert_map[vert] = -1;
  }
  for (const int edge : data.edges) {
    data.edge_map[edge] = -1;
  }
  return dst_mesh;
}

Array<Mesh *> mesh_copy_face_groups(const Mesh &src_mesh,
                                    const Span<IndexMask> face_groups,
                                    const bke::AttributeFilter &attribute_filter)
{
  PRF_scope(ProfileCategory::Default);
  Array<Mesh *> dst_meshes(face_groups.size(), nullptr);
  if (face_groups.is_empty()) {
    return dst_meshes;
  }

  threading::EnumerableThreadSpecific<FaceGroupCopyData> data_per_thread{[&]() {
    FaceGroupCopyData data;
    data.vert_map = Array<int>(src_mesh.verts_num, -1);
    data.edge_map = Array<int>(src_mesh.edges_num, -1);
    return data;
  }};

  threading::parallel_for(
      face_groups.index_range(),
      4096,
      [&](const IndexRange range) {
        /* Need task isolation because of the thread local maps. */
        threading::isolate_task([&]() {
          FaceGroupCopyData &data = data_per_thread.local();
          for (const int group_i : range) {
            dst_meshes[group_i] = copy_face_group(
                src_mesh, face_groups[group_i], attribute_filter, data);
          }
        });
      },
      threading::individual_task_sizes(
          [&](const int64_t group_i) { return face_groups[group_i].size(); }));

  return dst_meshes;
}

std::optional<Mesh *> mesh_copy_selection_keep_verts(const Mesh &src_mesh,
                                                     const VArray<bool> &selection,
                                                     const bke::AttrDomain selection_domain,
                                                     const bke::AttributeFilter &attribute_filter)
{
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();

  if (selection.is_empty()) {
    return std::nullopt;
  }

  threading::EnumerableThreadSpecific<IndexMaskMemory> memory;
  IndexMask edge_mask;
  IndexMask face_mask;
  switch (selection_domain) {
    case bke::AttrDomain::Point: {
      const VArraySpan<bool> span(selection);
      threading::parallel_invoke(
          src_edges.size() > 1024,
          [&]() { edge_mask = edge_selection_from_vert(src_edges, span, memory.local()); },
          [&]() {
            face_mask = face_selection_from_vert(
                src_faces, src_corner_verts, span, memory.local());
          });
      break;
    }
    case bke::AttrDomain::Edge: {
      const VArraySpan<bool> span(selection);
      threading::parallel_invoke(
          src_edges.size() > 1024,
          [&]() { edge_mask = IndexMask::from_bools(span, memory.local()); },
          [&]() {
            face_mask = face_selection_from_edge(
                src_faces, src_corner_edges, span, memory.local());
          });
      break;
    }
    case bke::AttrDomain::Face: {
      const VArraySpan<bool> span(selection);
      face_mask = IndexMask::from_bools(span, memory.local());
      edge_mask = edge_selection_from_face(
          src_faces, face_mask, src_corner_edges, src_edges.size(), memory.local());
      break;
    }
    default:
      BLI_assert_unreachable();
      break;
  }

  const bool same_edges = edge_mask.size() == src_mesh.edges_num;
  const bool same_faces = face_mask.size() == src_mesh.faces_num;
  if (same_edges && same_faces) {
    return std::nullopt;
  }

  Mesh *dst_mesh = bke::mesh_new_no_attributes(
      src_mesh.verts_num, edge_mask.size(), face_mask.size(), 0);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  const OffsetIndices<int> dst_faces = offset_indices::gather_selected_offsets(
      src_faces, face_mask, dst_mesh->face_offsets_for_write());
  dst_mesh->corners_num = dst_faces.total_size();
  dst_attributes.add<int>(".corner_edge", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  MutableSpan<int> dst_corner_edges = dst_mesh->corner_edges_for_write();

  threading::parallel_invoke(
      [&]() {
        remap_edges(src_faces,
                    dst_faces,
                    src_edges.size(),
                    edge_mask,
                    face_mask,
                    src_corner_edges,
                    dst_corner_edges);
      },
      [&]() {
        bke::copy_attributes(src_attributes,
                             bke::AttrDomain::Point,
                             bke::AttrDomain::Point,
                             attribute_filter,
                             dst_attributes);
        bke::gather_attributes(src_attributes,
                               bke::AttrDomain::Edge,
                               bke::AttrDomain::Edge,
                               attribute_filter,
                               edge_mask,
                               dst_attributes);
        bke::gather_attributes(src_attributes,
                               bke::AttrDomain::Face,
                               bke::AttrDomain::Face,
                               attribute_filter,
                               face_mask,
                               dst_attributes);
        bke::gather_attributes_group_to_group(
            src_attributes,
            bke::AttrDomain::Corner,
            bke::AttrDomain::Corner,
            bke::attribute_filter_with_skip_ref(attribute_filter, {".corner_edge"}),
            src_faces,
            dst_faces,
            face_mask,
            dst_attributes);
      });

  /* Positions are not changed by the operation, so the bounds are the same. */
  dst_mesh->runtime->bounds_cache = src_mesh.runtime->bounds_cache;
  if (selection_domain == bke::AttrDomain::Face) {
    copy_loose_edge_hint(src_mesh, *dst_mesh);
  }
  copy_overlapping_hint(src_mesh, *dst_mesh);

  return dst_mesh;
}

std::optional<Mesh *> mesh_copy_selection_keep_edges(const Mesh &src_mesh,
                                                     const VArray<bool> &selection,
                                                     const bke::AttrDomain selection_domain,
                                                     const bke::AttributeFilter &attribute_filter)
{
  const OffsetIndices src_faces = src_mesh.faces();
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();

  if (selection.is_empty()) {
    return std::nullopt;
  }

  IndexMaskMemory memory;
  IndexMask face_mask;
  switch (selection_domain) {
    case bke::AttrDomain::Point:
      face_mask = face_selection_from_vert(
          src_faces, src_mesh.corner_verts(), VArraySpan(selection), memory);
      break;
    case bke::AttrDomain::Edge:
      face_mask = face_selection_from_edge(
          src_faces, src_mesh.corner_edges(), VArraySpan(selection), memory);
      break;
    case bke::AttrDomain::Face:
      face_mask = IndexMask::from_bools(selection, memory);
      break;
    default:
      BLI_assert_unreachable();
      break;
  }

  const bool same_faces = face_mask.size() == src_mesh.faces_num;
  if (same_faces) {
    return std::nullopt;
  }

  Mesh *dst_mesh = bke::mesh_new_no_attributes(
      src_mesh.verts_num, src_mesh.edges_num, face_mask.size(), 0);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  const OffsetIndices<int> dst_faces = offset_indices::gather_selected_offsets(
      src_faces, face_mask, dst_mesh->face_offsets_for_write());
  dst_mesh->corners_num = dst_faces.total_size();
  dst_attributes.add<int>(".corner_vert", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  dst_attributes.add<int>(".corner_edge", bke::AttrDomain::Corner, bke::AttributeInitConstruct());

  bke::copy_attributes(src_attributes,
                       bke::AttrDomain::Point,
                       bke::AttrDomain::Point,
                       attribute_filter,
                       dst_attributes);
  bke::copy_attributes(src_attributes,
                       bke::AttrDomain::Edge,
                       bke::AttrDomain::Edge,
                       attribute_filter,
                       dst_attributes);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Face,
                         bke::AttrDomain::Face,
                         attribute_filter,
                         face_mask,
                         dst_attributes);
  bke::gather_attributes_group_to_group(src_attributes,
                                        bke::AttrDomain::Corner,
                                        bke::AttrDomain::Corner,
                                        attribute_filter,
                                        src_faces,
                                        dst_faces,
                                        face_mask,
                                        dst_attributes);

  /* Positions are not changed by the operation, so the bounds are the same. */
  dst_mesh->runtime->bounds_cache = src_mesh.runtime->bounds_cache;
  copy_loose_vert_hint(src_mesh, *dst_mesh);
  copy_overlapping_hint(src_mesh, *dst_mesh);
  return dst_mesh;
}

}  // namespace blender::geometry
