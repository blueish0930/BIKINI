/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo
 */

#include <algorithm>

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"

#include "BLI_array_utils.hh"
#include "BLI_atomic_disjoint_set.hh"
#include "BLI_disjoint_set.hh"
#include "BLI_index_mask.hh"
#include "BLI_listbase.hh"
#include "BLI_math_vector.hh"
#include "BLI_offset_indices.hh"
#include "BLI_ordered_edge.hh"
#include "BLI_set.hh"
#include "BLI_task.hh"
#include "BLI_vector_set.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_filters.hh"
#include "BKE_attribute_math.hh"
#include "BKE_customdata.hh"
#include "BKE_deform.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.hh"

#include "GEO_mesh_dissolve.hh"
#include "GEO_randomize.hh"

namespace blender::geometry {

static int count_kept_verts(const Span<int> verts, const Span<bool> verts_to_dissolve)
{
  return std::count_if(
      verts.begin(), verts.end(), [&](const int vert) { return !verts_to_dissolve[vert]; });
}

/**
 * Faces need at least three corners, so some of the vertices of a face have to be kept if too
 * many of them are selected.
 */
static void ensure_min_face_sizes(const OffsetIndices<int> faces,
                                  const Span<int> corner_verts,
                                  MutableSpan<bool> verts_to_dissolve)
{
  static constexpr int min_face_size = 3;

  IndexMaskMemory memory;
  const IndexMask small_faces = IndexMask::from_predicate(
      faces.index_range(),
      memory,
      [&](const int face_i) {
        return count_kept_verts(corner_verts.slice(faces[face_i]), verts_to_dissolve) <
               min_face_size;
      },
      exec_mode::grain_size(4096));

  /* Not in parallel, because faces can share vertices and the result has to be deterministic. */
  Vector<int, 32> candidates;
  small_faces.foreach_index([&](const int face_i) {
    const Span<int> face_verts = corner_verts.slice(faces[face_i]);
    candidates.clear();
    for (const int vert : face_verts) {
      if (verts_to_dissolve[vert]) {
        candidates.append(vert);
      }
    }
    const int missing_verts = min_face_size - int(face_verts.size() - candidates.size());
    /* Spread the kept vertices over the face to keep as much of its shape as possible. */
    for (const int i : IndexRange(std::max(missing_verts, 0))) {
      verts_to_dissolve[candidates[i * candidates.size() / missing_verts]] = false;
    }
  });
}

/**
 * A closed loop of loose edges does not have a face that forces some of its vertices to be kept.
 * Dissolving all of the vertices (or all but one) would leave an edge without two different
 * vertices, so a triangle is kept in this case.
 */
static void ensure_min_loose_loop_sizes(const Span<int2> edges,
                                        const GroupedSpan<int> vert_to_edge_map,
                                        const IndexMask &candidates,
                                        MutableSpan<bool> verts_to_dissolve)
{
  static constexpr int min_loop_size = 3;

  AtomicDisjointSet edge_sets(edges.size());
  candidates.foreach_index(
      [&](const int vert) {
        if (verts_to_dissolve[vert]) {
          const Span<int> vert_edges = vert_to_edge_map[vert];
          edge_sets.join(vert_edges[0], vert_edges[1]);
        }
      },
      exec_mode::grain_size(4096));

  Array<int> edge_groups(edges.size());
  const int groups_num = edge_sets.calc_reduced_ids(edge_groups);

  /* The kept vertices at the ends of every chain of joined edges. */
  Array<int2> chain_ends(groups_num, int2(-1));
  for (const int vert : vert_to_edge_map.index_range()) {
    if (verts_to_dissolve[vert]) {
      continue;
    }
    for (const int edge : vert_to_edge_map[vert]) {
      int2 &ends = chain_ends[edge_groups[edge]];
      (ends[0] == -1 ? ends[0] : ends[1]) = vert;
    }
  }

  const auto group_of_vert = [&](const int vert) {
    return edge_groups[vert_to_edge_map[vert][0]];
  };
  const auto is_loop = [&](const int group) {
    const int2 ends = chain_ends[group];
    return ends[0] == -1 || ends[0] == ends[1];
  };

  Array<int> dissolved_num(groups_num, 0);
  bool any_loop = false;
  candidates.foreach_index([&](const int vert) {
    if (verts_to_dissolve[vert]) {
      const int group = group_of_vert(vert);
      dissolved_num[group]++;
      any_loop |= is_loop(group);
    }
  });
  if (!any_loop) {
    return;
  }

  Array<int> visited_num(groups_num, 0);
  candidates.foreach_index([&](const int vert) {
    if (!verts_to_dissolve[vert]) {
      return;
    }
    const int group = group_of_vert(vert);
    if (!is_loop(group)) {
      return;
    }
    const int kept_num = chain_ends[group][0] == -1 ? 0 : 1;
    const int missing_verts = std::min(min_loop_size - kept_num, dissolved_num[group]);
    /* Spread the kept vertices over the loop. */
    const int i = visited_num[group]++;
    if (i * missing_verts / dissolved_num[group] != (i + 1) * missing_verts / dissolved_num[group])
    {
      verts_to_dissolve[vert] = false;
    }
  });
}

/**
 * Join the two edges of every dissolved vertex.
 *
 * \param r_old_to_new_edges: The result edge that every source edge becomes a part of.
 * \return The result edges, still with the vertex indices of the source mesh.
 */
static VectorSet<OrderedEdge> dissolved_edges_for_verts(const Span<int2> src_edges,
                                                        const GroupedSpan<int> vert_to_edge_map,
                                                        const IndexMask &verts_to_dissolve,
                                                        const IndexMask &kept_verts,
                                                        MutableSpan<int> r_old_to_new_edges)
{
  AtomicDisjointSet edge_sets(src_edges.size());
  verts_to_dissolve.foreach_index(
      [&](const int vert) {
        const Span<int> vert_edges = vert_to_edge_map[vert];
        BLI_assert(vert_edges.size() == 2);
        edge_sets.join(vert_edges[0], vert_edges[1]);
      },
      exec_mode::grain_size(4096));

  const int joined_edges_num = edge_sets.calc_reduced_ids(r_old_to_new_edges);

  Array<int2> joined_edges(joined_edges_num, int2(-1));
  kept_verts.foreach_index([&](const int vert) {
    for (const int edge : vert_to_edge_map[vert]) {
      int2 &joined_edge = joined_edges[r_old_to_new_edges[edge]];
      if (joined_edge[0] == -1) {
        joined_edge[0] = vert;
      }
      else {
        BLI_assert(joined_edge[1] == -1);
        joined_edge[1] = vert;
      }
    }
  });
  BLI_assert(!joined_edges.as_span().cast<int>().contains(-1));

  /* Dissolving vertices can result in edges that connect the same vertices. */
  VectorSet<OrderedEdge> unique_edges;
  unique_edges.reserve(joined_edges.size());
  for (const int2 edge : joined_edges) {
    unique_edges.add(edge);
  }

  threading::parallel_for(src_edges.index_range(), 2048, [&](const IndexRange range) {
    for (const int edge : range) {
      r_old_to_new_edges[edge] = unique_edges.index_of(joined_edges[r_old_to_new_edges[edge]]);
    }
  });

  return unique_edges;
}

struct FaceKey {
  int face;
  int first_vert;
};

struct FaceHash {
  uint64_t operator()(const FaceKey &value) const
  {
    return get_default_hash(value.first_vert);
  }
};

struct FacesEquality {
  GroupedSpan<int> sorted_face_verts;
  bool operator()(const FaceKey &a, const FaceKey &b) const
  {
    return sorted_face_verts[a.face] == sorted_face_verts[b.face];
  }
};

/**
 * Dissolving vertices can result in faces that use the same vertices. Only one of them is kept.
 * Faces that were equal already before are not changed.
 *
 * \param src_faces: Faces of the source mesh.
 * \param faces: The same faces without the corners of the dissolved vertices.
 * \param r_face_offsets: Offsets of the result faces.
 * \param r_old_to_new_faces: Index of the result face for every face in \a faces.
 * \param r_old_to_new_corners: Index of the result corner for every corner in \a faces.
 * \return The index of the face in \a faces that every result face is made from.
 */
static Array<int> deduplicate_faces(const OffsetIndices<int> src_faces,
                                    const GroupedSpan<int> faces,
                                    Array<int> &r_face_offsets,
                                    MutableSpan<int> r_old_to_new_faces,
                                    MutableSpan<int> r_old_to_new_corners)
{
  const auto use_all_faces = [&]() {
    r_face_offsets.reinitialize(faces.size() + 1);
    if (faces.is_empty()) {
      r_face_offsets.first() = 0;
    }
    else {
      r_face_offsets.as_mutable_span().copy_from(faces.offsets.data());
    }
    array_utils::fill_index_range<int>(r_old_to_new_faces);
    array_utils::fill_index_range<int>(r_old_to_new_corners);
    Array<int> unique_faces(faces.size());
    array_utils::fill_index_range<int>(unique_faces);
    return unique_faces;
  };

  /* For every face, the corners in the face ordered by their vertex. */
  Array<int> sorted_corners(faces.data.size());
  Array<int> sorted_verts(faces.data.size());
  threading::parallel_for(
      faces.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int face_i : range) {
          const IndexRange face = faces.offsets[face_i];
          const Span<int> face_verts = faces.data.slice(face);
          MutableSpan<int> corners = sorted_corners.as_mutable_span().slice(face);
          array_utils::fill_index_range<int>(corners);
          std::sort(corners.begin(), corners.end(), [&](const int a, const int b) {
            return face_verts[a] < face_verts[b];
          });
          array_utils::gather(
              face_verts, corners.as_span(), sorted_verts.as_mutable_span().slice(face));
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return faces.offsets[range].size(); }));

  const GroupedSpan<int> sorted_face_verts(faces.offsets, sorted_verts);
  VectorSet<FaceKey, 0, DefaultProbingStrategy, FaceHash, FacesEquality> face_set(
      FaceHash{}, FacesEquality{sorted_face_verts});
  face_set.reserve(faces.size());
  /* Index of the group of equal faces for every face. */
  Array<int> face_groups(faces.size());
  for (const int face_i : faces.index_range()) {
    face_groups[face_i] = int(
        face_set.index_of_or_add(FaceKey{face_i, sorted_face_verts[face_i].first()}));
  }
  if (face_set.size() == faces.size()) {
    return use_all_faces();
  }

  /* Only groups with a face that lost some of its vertices are joined. */
  Array<bool> group_changed(face_set.size(), false);
  for (const int face_i : faces.index_range()) {
    if (faces.offsets[face_i].size() != src_faces[face_i].size()) {
      group_changed[face_groups[face_i]] = true;
    }
  }
  bool any_face_removed = false;
  for (const int face_i : faces.index_range()) {
    const int group = face_groups[face_i];
    any_face_removed |= group_changed[group] && face_set[group].face != face_i;
  }
  if (!any_face_removed) {
    return use_all_faces();
  }

  /* The first face of every group is the one that is kept. */
  Vector<int> unique_faces;
  unique_faces.reserve(faces.size());
  for (const int face_i : faces.index_range()) {
    const int group = face_groups[face_i];
    const int first_face = face_set[group].face;
    if (first_face == face_i || !group_changed[group]) {
      r_old_to_new_faces[face_i] = int(unique_faces.append_and_get_index(face_i));
    }
    else {
      r_old_to_new_faces[face_i] = r_old_to_new_faces[first_face];
    }
  }

  r_face_offsets.reinitialize(unique_faces.size() + 1);
  for (const int i : unique_faces.index_range()) {
    r_face_offsets[i] = faces.offsets[unique_faces[i]].size();
  }
  const OffsetIndices<int> dst_faces = offset_indices::accumulate_counts_to_offsets(
      r_face_offsets);

  threading::parallel_for(faces.index_range(), 4096, [&](const IndexRange range) {
    for (const int face_i : range) {
      const IndexRange face = faces.offsets[face_i];
      const int dst_face_i = r_old_to_new_faces[face_i];
      const IndexRange dst_face = dst_faces[dst_face_i];
      MutableSpan<int> dst_corners = r_old_to_new_corners.slice(face);
      const int first_face = unique_faces[dst_face_i];
      if (first_face == face_i) {
        array_utils::fill_index_range<int>(dst_corners, int(dst_face.start()));
        continue;
      }
      /* Corners that use the same vertex are at the same position in the sorted corners. */
      const Span<int> corners = sorted_corners.as_span().slice(face);
      const Span<int> first_face_corners = sorted_corners.as_span().slice(
          faces.offsets[first_face]);
      for (const int i : corners.index_range()) {
        dst_corners[corners[i]] = int(dst_face.start()) + first_face_corners[i];
      }
    }
  });

  return Array<int>(unique_faces.as_span());
}

static bool attribute_type_supports_mixing(const bke::AttrType type)
{
  return !ELEM(type, bke::AttrType::String, bke::AttrType::WrangleArray);
}

/**
 * Create the attributes of the result elements, every one of them is made from the group of
 * source elements in \a dst_to_src_map.
 */
static void propagate_attributes(const bke::AttributeAccessor src_attributes,
                                 const bke::AttrDomain domain,
                                 const bke::AttributeFilter &attribute_filter,
                                 const GroupedSpan<int> dst_to_src_map,
                                 bke::MutableAttributeAccessor dst_attributes)
{
  Array<int> first_indices(dst_to_src_map.size());
  threading::parallel_for(dst_to_src_map.index_range(), 8192, [&](const IndexRange range) {
    for (const int i : range) {
      first_indices[i] = dst_to_src_map[i].first();
    }
  });
  bke::gather_attributes(
      src_attributes, domain, domain, attribute_filter, first_indices.as_span(), dst_attributes);

  IndexMaskMemory memory;
  const IndexMask mixed = IndexMask::from_predicate(
      dst_to_src_map.index_range(),
      memory,
      [&](const int i) { return dst_to_src_map.offsets[i].size() > 1; },
      exec_mode::grain_size(4096));
  if (mixed.is_empty()) {
    return;
  }

  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != domain) {
      return;
    }
    if (!attribute_type_supports_mixing(iter.data_type)) {
      return;
    }
    /* Identifiers and indices have no meaningful average, keep the first value. */
    if (iter.name == "id" || iter.name == "material_index") {
      return;
    }
    if (attribute_filter.allow_skip(iter.name)) {
      return;
    }
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_for_write_span(iter.name);
    if (!dst) {
      return;
    }
    const bke::GAttributeReader src = iter.get();
    bke::attribute_math::to_static_type(iter.data_type, [&]<typename T>() {
      const VArraySpan<T> src_span(src.varray.typed<T>());
      bke::attribute_math::DefaultMixer<T> mixer(dst.span.typed<T>(), mixed);
      mixed.foreach_index(
          [&](const int dst_i) {
            for (const int src_i : dst_to_src_map[dst_i]) {
              mixer.mix_in(dst_i, src_span[src_i]);
            }
          },
          exec_mode::grain_size(1024));
      mixer.finalize(mixed);
    });
    dst.finish();
  });
}

/** Gather vertex group data and array attributes in separate loops. */
static void gather_vert_attributes(const Mesh &src_mesh,
                                   const bke::AttributeFilter &attribute_filter,
                                   const IndexMask &vert_mask,
                                   Mesh &dst_mesh)
{
  Set<std::string> vertex_group_names;
  for (bDeformGroup &group : src_mesh.vertex_group_names) {
    vertex_group_names.add(group.name);
  }

  const Span<MDeformVert> src = src_mesh.deform_verts();
  if (!vertex_group_names.is_empty() && !src.is_empty()) {
    MutableSpan<MDeformVert> dst = dst_mesh.deform_verts_for_write();
    bke::gather_deform_verts(src, vert_mask, dst);
  }

  bke::gather_attributes(src_mesh.attributes(),
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         bke::attribute_filter_with_skip_ref(attribute_filter, vertex_group_names),
                         vert_mask,
                         dst_mesh.attributes_for_write());
}

static void gather_orig_indices(const CustomData &src_data,
                                const int src_num,
                                const Span<int> dst_to_src,
                                CustomData &dst_data)
{
  if (!CustomData_has_layer(&src_data, CD_ORIGINDEX)) {
    return;
  }
  const Span src(static_cast<const int *>(CustomData_get_layer(&src_data, CD_ORIGINDEX)), src_num);
  MutableSpan dst(static_cast<int *>(CustomData_add_layer(
                      &dst_data, CD_ORIGINDEX, CD_CONSTRUCT, dst_to_src.size())),
                  dst_to_src.size());
  array_utils::gather(src, dst_to_src, dst);
}

/**
 * Remove the selected vertices that are connected to exactly two edges and join these two edges
 * into one. Vertices that are necessary to keep faces with at least three corners are not
 * removed.
 *
 * \param angle_threshold: Vertices are kept if the direction changes by more than this angle
 * from one of their edges to the other.
 * \return #std::nullopt if the mesh is unchanged.
 */
static std::optional<Mesh *> dissolve_boundary_verts(const Mesh &src_mesh,
                                                     const IndexMask &verts_mask,
                                                     const float angle_threshold,
                                                     const bke::AttributeFilter &attribute_filter)
{
  const Span<float3> src_positions = src_mesh.vert_positions();
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices<int> src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();

  Array<int> vert_to_edge_offsets;
  Array<int> vert_to_edge_indices;
  const GroupedSpan<int> vert_to_edge_map = bke::mesh::build_vert_to_edge_map(
      src_edges, src_mesh.verts_num, vert_to_edge_offsets, vert_to_edge_indices);

  /* Avoid visible differences because of the limited accuracy of the angle. */
  const float angle_epsilon = 1e-4f;
  const bool use_angle_threshold = angle_threshold < float(M_PI) - angle_epsilon;

  IndexMaskMemory memory;
  const IndexMask boundary_verts = IndexMask::from_predicate(
      verts_mask,
      memory,
      [&](const int vert) {
        const Span<int> vert_edges = vert_to_edge_map[vert];
        if (vert_edges.size() != 2) {
          return false;
        }
        if (!use_angle_threshold) {
          return true;
        }
        const float3 &position = src_positions[vert];
        const float3 direction_a = math::normalize(
            position - src_positions[bke::mesh::edge_other_vert(src_edges[vert_edges[0]], vert)]);
        const float3 direction_b = math::normalize(
            src_positions[bke::mesh::edge_other_vert(src_edges[vert_edges[1]], vert)] - position);
        const float angle = std::acos(
            std::clamp(math::dot(direction_a, direction_b), -1.0f, 1.0f));
        return angle <= angle_threshold + angle_epsilon;
      },
      exec_mode::grain_size(4096));
  if (boundary_verts.is_empty()) {
    return std::nullopt;
  }

  /* The selection of boundary vertices is reduced further to keep the mesh valid. */
  Array<bool> verts_to_dissolve_selection(src_mesh.verts_num, false);
  boundary_verts.to_bools(verts_to_dissolve_selection);
  ensure_min_face_sizes(src_faces, src_corner_verts, verts_to_dissolve_selection);
  if (!src_mesh.loose_edges().is_empty()) {
    ensure_min_loose_loop_sizes(
        src_edges, vert_to_edge_map, boundary_verts, verts_to_dissolve_selection);
  }

  const IndexMask verts_to_dissolve = IndexMask::from_bools(
      boundary_verts, verts_to_dissolve_selection, memory);
  if (verts_to_dissolve.is_empty()) {
    return std::nullopt;
  }
  const IndexMask kept_verts = verts_to_dissolve.complement(IndexMask(src_mesh.verts_num), memory);
  const IndexMask kept_corners = IndexMask::from_predicate(
      IndexMask(src_mesh.corners_num),
      memory,
      [&](const int corner) { return !verts_to_dissolve_selection[src_corner_verts[corner]]; },
      exec_mode::grain_size(4096));

  /* The source faces without the dissolved vertices. */
  Array<int> face_offsets(src_mesh.faces_num + 1);
  threading::parallel_for(
      src_faces.index_range(),
      4096,
      [&](const IndexRange range) {
        for (const int face_i : range) {
          face_offsets[face_i] = count_kept_verts(src_corner_verts.slice(src_faces[face_i]),
                                                  verts_to_dissolve_selection);
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return src_faces[range].size(); }));
  const OffsetIndices<int> faces = offset_indices::accumulate_counts_to_offsets(face_offsets);
  BLI_assert(faces.total_size() == kept_corners.size());

  Array<int> corner_verts(kept_corners.size());
  array_utils::gather(src_corner_verts, kept_corners, corner_verts.as_mutable_span());

  Array<int> old_to_new_edges(src_mesh.edges_num);
  VectorSet<OrderedEdge> unique_edges;

  Array<int> old_to_new_faces(src_mesh.faces_num);
  Array<int> old_to_new_corners(kept_corners.size());
  Array<int> dst_face_offsets;
  Array<int> unique_faces;

  threading::parallel_invoke(
      src_mesh.edges_num + src_mesh.faces_num > 1000,
      [&]() {
        unique_edges = dissolved_edges_for_verts(
            src_edges, vert_to_edge_map, verts_to_dissolve, kept_verts, old_to_new_edges);
      },
      [&]() {
        unique_faces = deduplicate_faces(src_faces,
                                         {faces, corner_verts},
                                         dst_face_offsets,
                                         old_to_new_faces,
                                         old_to_new_corners);
      });

  const OffsetIndices<int> dst_faces(dst_face_offsets);
  const int dst_edges_num = int(unique_edges.size());
  const int dst_faces_num = int(unique_faces.size());
  const int dst_corners_num = dst_faces.total_size();

  Mesh *dst_mesh = BKE_mesh_new_nomain(
      int(kept_verts.size()), dst_edges_num, dst_faces_num, dst_corners_num);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);

  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  Array<int> new_to_old_edge_offsets;
  Array<int> new_to_old_edge_indices;
  const GroupedSpan<int> new_to_old_edge_map = offset_indices::build_groups_from_indices(
      old_to_new_edges, dst_edges_num, new_to_old_edge_offsets, new_to_old_edge_indices);

  Array<int> new_to_old_face_offsets;
  Array<int> new_to_old_face_indices;
  const GroupedSpan<int> new_to_old_face_map = offset_indices::build_groups_from_indices(
      old_to_new_faces, dst_faces_num, new_to_old_face_offsets, new_to_old_face_indices);

  Array<int> new_to_old_corner_offsets;
  Array<int> new_to_old_corner_indices;
  const GroupedSpan<int> new_to_old_corner_map = offset_indices::build_groups_from_indices(
      old_to_new_corners, dst_corners_num, new_to_old_corner_offsets, new_to_old_corner_indices);
  /* Corners are grouped by their index in the faces without the dissolved vertices, but
   * attributes are propagated from the corners of the source mesh. */
  Array<int> kept_corner_indices(kept_corners.size());
  kept_corners.to_indices<int>(kept_corner_indices);
  array_utils::gather(kept_corner_indices.as_span(),
                      new_to_old_corner_indices.as_span(),
                      new_to_old_corner_indices.as_mutable_span());

  gather_vert_attributes(src_mesh, attribute_filter, kept_verts, *dst_mesh);
  propagate_attributes(src_attributes,
                       bke::AttrDomain::Edge,
                       bke::attribute_filter_with_skip_ref(attribute_filter, {".edge_verts"}),
                       new_to_old_edge_map,
                       dst_attributes);
  propagate_attributes(src_attributes,
                       bke::AttrDomain::Face,
                       attribute_filter,
                       new_to_old_face_map,
                       dst_attributes);
  propagate_attributes(
      src_attributes,
      bke::AttrDomain::Corner,
      bke::attribute_filter_with_skip_ref(attribute_filter, {".corner_vert", ".corner_edge"}),
      new_to_old_corner_map,
      dst_attributes);

  Array<int> new_verts_remap(kept_verts.min_array_size(), -1);
  index_mask::build_reverse_map<int>(kept_verts, new_verts_remap);

  MutableSpan<int2> dst_edges = dst_mesh->edges_for_write();
  threading::parallel_for(dst_edges.index_range(), 4096, [&](const IndexRange range) {
    for (const int edge : range) {
      const OrderedEdge &unique_edge = unique_edges[edge];
      dst_edges[edge] = int2(new_verts_remap[unique_edge.v_low],
                             new_verts_remap[unique_edge.v_high]);
    }
  });
  BLI_assert(!dst_edges.as_span().cast<int>().contains(-1));

  if (dst_faces_num > 0) {
    dst_mesh->face_offsets_for_write().copy_from(dst_face_offsets);
  }

  MutableSpan<int> dst_corner_verts = dst_mesh->corner_verts_for_write();
  MutableSpan<int> dst_corner_edges = dst_mesh->corner_edges_for_write();
  threading::parallel_for(
      IndexRange(dst_faces_num),
      2048,
      [&](const IndexRange range) {
        for (const int face_i : range) {
          const IndexRange dst_face = dst_faces[face_i];
          const Span<int> face_verts = corner_verts.as_span().slice(faces[unique_faces[face_i]]);
          MutableSpan<int> face_corner_verts = dst_corner_verts.slice(dst_face);
          MutableSpan<int> face_corner_edges = dst_corner_edges.slice(dst_face);
          for (const int i : face_verts.index_range()) {
            const int vert = face_verts[i];
            const int next_vert = face_verts[(i + 1) % face_verts.size()];
            face_corner_verts[i] = new_verts_remap[vert];
            face_corner_edges[i] = int(unique_edges.index_of(OrderedEdge(vert, next_vert)));
          }
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return dst_faces[range].size(); }));
  BLI_assert(!dst_corner_verts.as_span().contains(-1));

  Array<int> kept_vert_indices(kept_verts.size());
  kept_verts.to_indices<int>(kept_vert_indices);
  gather_orig_indices(
      src_mesh.vert_data, src_mesh.verts_num, kept_vert_indices, dst_mesh->vert_data);
  Array<int> first_old_edges(dst_edges_num);
  for (const int edge : first_old_edges.index_range()) {
    first_old_edges[edge] = new_to_old_edge_map[edge].first();
  }
  gather_orig_indices(
      src_mesh.edge_data, src_mesh.edges_num, first_old_edges, dst_mesh->edge_data);
  gather_orig_indices(src_mesh.face_data, src_mesh.faces_num, unique_faces, dst_mesh->face_data);

  BLI_assert(bke::mesh_is_valid(*dst_mesh));

  debug_randomize_vert_order(dst_mesh);
  debug_randomize_edge_order(dst_mesh);
  debug_randomize_face_order(dst_mesh);

  return dst_mesh;
}

/* -------------------------------------------------------------------- */
/** \name Dissolve Edges
 * \{ */

/**
 * The faces of a mesh stored as linked lists of corners, which makes it possible to join
 * neighboring faces without moving data. Every corner stands for its vertex and for the edge to
 * the next corner of the face.
 */
class FaceJoiner {
 public:
  enum class Result : int8_t {
    Joined,
    /** Joining might become possible after other faces were joined. */
    Retry,
    Never,
  };

 private:
  Span<int> corner_verts_;
  Span<int> corner_edges_;
  Span<int> corner_to_face_;
  GroupedSpan<int> vert_to_corner_map_;
  GroupedSpan<int> edge_to_corner_map_;

  Array<int> next_;
  Array<int> prev_;
  Array<bool> corner_removed_;
  Array<bool> edge_removed_;

  DisjointSet<int> face_sets_;
  /* The following is stored for the root of every set of joined faces. */
  Array<int> face_sizes_;
  Array<int> face_corners_;
  Array<int> first_faces_;

 public:
  FaceJoiner(const Mesh &mesh)
      : corner_verts_(mesh.corner_verts()),
        corner_edges_(mesh.corner_edges()),
        corner_to_face_(mesh.corner_to_face_map()),
        vert_to_corner_map_(mesh.vert_to_corner_map()),
        edge_to_corner_map_(mesh.edge_to_corner_map()),
        next_(mesh.corners_num),
        prev_(mesh.corners_num),
        corner_removed_(mesh.corners_num, false),
        edge_removed_(mesh.edges_num, false),
        face_sets_(mesh.faces_num),
        face_sizes_(mesh.faces_num),
        face_corners_(mesh.faces_num),
        first_faces_(mesh.faces_num)
  {
    const OffsetIndices<int> faces = mesh.faces();
    threading::parallel_for(faces.index_range(), 4096, [&](const IndexRange range) {
      for (const int face_i : range) {
        const IndexRange face = faces[face_i];
        for (const int corner : face) {
          next_[corner] = bke::mesh::face_corner_next(face, corner);
          prev_[corner] = bke::mesh::face_corner_prev(face, corner);
        }
        face_sizes_[face_i] = int(face.size());
        face_corners_[face_i] = int(face.start());
        first_faces_[face_i] = face_i;
      }
    });
  }

  /**
   * Join the two faces that use the edge. Everything that the faces have in common is removed,
   * which can be more than the one edge. The faces are not joined if that would result in a face
   * with a hole or in a face that uses a vertex more than once.
   */
  Result try_join(const int edge)
  {
    if (edge_removed_[edge]) {
      return Result::Never;
    }
    const Span<int> edge_corners = edge_to_corner_map_[edge];
    if (edge_corners.size() != 2) {
      return Result::Never;
    }
    const int corner_a = edge_corners[0];
    const int corner_b = edge_corners[1];
    const int face_a = face_sets_.find_root(corner_to_face_[corner_a]);
    const int face_b = face_sets_.find_root(corner_to_face_[corner_b]);
    if (face_a == face_b) {
      return Result::Never;
    }
    if (corner_verts_[corner_a] != corner_verts_[next_[corner_b]] ||
        corner_verts_[next_[corner_a]] != corner_verts_[corner_b])
    {
      /* The faces have a different winding. */
      return Result::Never;
    }

    /* The chain of neighboring edges that are used by both faces. It goes into opposite
     * directions in the two faces. */
    const int max_chain_size = std::min(face_sizes_[face_a], face_sizes_[face_b]);
    int chain_size = 1;
    int a_first = corner_a;
    int a_last = corner_a;
    int b_first = corner_b;
    int b_last = corner_b;
    while (chain_size < max_chain_size) {
      const int a = next_[a_last];
      const int b = prev_[b_first];
      if (corner_edges_[a] != corner_edges_[b]) {
        break;
      }
      a_last = a;
      b_first = b;
      chain_size++;
    }
    while (chain_size < max_chain_size) {
      const int a = prev_[a_first];
      const int b = next_[b_last];
      if (corner_edges_[a] != corner_edges_[b]) {
        break;
      }
      a_first = a;
      b_last = b;
      chain_size++;
    }

    const int rest_a = face_sizes_[face_a] - chain_size;
    const int rest_b = face_sizes_[face_b] - chain_size;
    if (rest_a < 1 || rest_b < 1 || rest_a + rest_b < 3) {
      return Result::Retry;
    }
    for (int corner = a_first, i = 0; i < chain_size; corner = next_[corner], i++) {
      if (edge_to_corner_map_[corner_edges_[corner]].size() != 2) {
        /* Other faces use the edge too. */
        return Result::Retry;
      }
    }

    /* The corners next to the chain, #a_next and #b_next are the corners of the vertices at the
     * ends of the chain in the joined face. */
    const int a_prev = prev_[a_first];
    const int a_next = next_[a_last];
    const int b_prev = prev_[b_first];
    const int b_next = next_[b_last];

    /* The faces must not have anything else in common, otherwise a vertex would be used multiple
     * times by the joined face. Only the smaller face has to be checked for that. */
    if (rest_a <= rest_b) {
      for (int corner = next_[a_next], i = 1; i < rest_a; corner = next_[corner], i++) {
        if (this->face_uses_vert(face_b, corner_verts_[corner])) {
          return Result::Retry;
        }
      }
    }
    else {
      for (int corner = next_[b_next], i = 1; i < rest_b; corner = next_[corner], i++) {
        if (this->face_uses_vert(face_a, corner_verts_[corner])) {
          return Result::Retry;
        }
      }
    }

    if (this->is_back_side_of_other_face(a_next, rest_a, b_next, rest_b)) {
      return Result::Retry;
    }

    for (int corner = a_first, i = 0; i < chain_size; corner = next_[corner], i++) {
      corner_removed_[corner] = true;
      edge_removed_[corner_edges_[corner]] = true;
    }
    for (int corner = b_first, i = 0; i < chain_size; corner = next_[corner], i++) {
      corner_removed_[corner] = true;
    }
    next_[a_prev] = b_next;
    prev_[b_next] = a_prev;
    next_[b_prev] = a_next;
    prev_[a_next] = b_prev;

    const int first_face = std::min(first_faces_[face_a], first_faces_[face_b]);
    const int root = face_sets_.join(face_a, face_b);
    face_sizes_[root] = rest_a + rest_b;
    face_corners_[root] = a_next;
    first_faces_[root] = first_face;
    return Result::Joined;
  }

  Span<bool> edge_removed() const
  {
    return edge_removed_;
  }

  /** The face with the smallest index of the faces that were joined with the given face. */
  int first_face(const int face)
  {
    return first_faces_[face_sets_.find_root(face)];
  }

  int face_size(const int face)
  {
    return face_sizes_[face_sets_.find_root(face)];
  }

  /** Gather the corners of the source mesh that the joined face is made of. */
  void face_corners(const int face, MutableSpan<int> r_corners) const
  {
    int corner = face_corners_[face_sets_.find_root(face)];
    for (int &r_corner : r_corners) {
      r_corner = corner;
      corner = next_[corner];
    }
  }

 private:
  /** The face at the other side of the edge of the corner, or -1 if there is not exactly one. */
  int neighbor_face(const int corner)
  {
    const Span<int> edge_corners = edge_to_corner_map_[corner_edges_[corner]];
    if (edge_corners.size() != 2) {
      return -1;
    }
    const int other_corner = edge_corners[0] == corner ? edge_corners[1] : edge_corners[0];
    return face_sets_.find_root(corner_to_face_[other_corner]);
  }

  /**
   * Joining all faces of a closed surface but one results in a face that uses exactly the same
   * edges as the remaining face, which is not better than having two faces at the same place.
   * The arguments are the corners that the joined face would be made of.
   */
  bool is_back_side_of_other_face(const int a_start,
                                  const int a_size,
                                  const int b_start,
                                  const int b_size)
  {
    const int other_face = this->neighbor_face(a_start);
    if (other_face == -1 || face_sizes_[other_face] != a_size + b_size) {
      return false;
    }
    for (int corner = a_start, i = 0; i < a_size; corner = next_[corner], i++) {
      if (this->neighbor_face(corner) != other_face) {
        return false;
      }
    }
    for (int corner = b_start, i = 0; i < b_size; corner = next_[corner], i++) {
      if (this->neighbor_face(corner) != other_face) {
        return false;
      }
    }
    return true;
  }

  bool face_uses_vert(const int face_root, const int vert)
  {
    for (const int corner : vert_to_corner_map_[vert]) {
      if (!corner_removed_[corner] && face_sets_.find_root(corner_to_face_[corner]) == face_root) {
        return true;
      }
    }
    return false;
  }
};

std::optional<Mesh *> dissolve_edges(const Mesh &src_mesh,
                                     const IndexMask &edge_mask,
                                     const bool dissolve_verts,
                                     const float vert_angle_threshold,
                                     const bke::AttributeFilter &attribute_filter)
{
  if (edge_mask.is_empty() || src_mesh.faces_num < 2) {
    return std::nullopt;
  }
  const Span<int2> src_edges = src_mesh.edges();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();

  /* Join faces one edge after the other, that way as many edges as possible are dissolved even
   * if not all of the faces can be joined. This is repeated because joining faces can make it
   * possible to join them with others that they could not be joined with before. */
  FaceJoiner joiner(src_mesh);
  Vector<int> edges_to_join(edge_mask.size());
  edge_mask.to_indices<int>(edges_to_join);
  bool any_joined = false;
  while (!edges_to_join.is_empty()) {
    Vector<int> edges_to_retry;
    bool joined = false;
    for (const int edge : edges_to_join) {
      switch (joiner.try_join(edge)) {
        case FaceJoiner::Result::Joined:
          joined = true;
          break;
        case FaceJoiner::Result::Retry:
          edges_to_retry.append(edge);
          break;
        case FaceJoiner::Result::Never:
          break;
      }
    }
    if (!joined) {
      break;
    }
    any_joined = true;
    edges_to_join = std::move(edges_to_retry);
  }
  if (!any_joined) {
    return std::nullopt;
  }

  IndexMaskMemory memory;
  const Span<bool> edge_removed = joiner.edge_removed();
  const IndexMask kept_edges = IndexMask::from_bools_inverse(edge_removed, memory);

  /* Vertices that were only used by dissolved edges are removed too. */
  Array<bool> vert_of_removed_edge(src_mesh.verts_num, false);
  for (const int edge : src_edges.index_range()) {
    if (edge_removed[edge]) {
      vert_of_removed_edge[src_edges[edge][0]] = true;
      vert_of_removed_edge[src_edges[edge][1]] = true;
    }
  }
  Array<bool> vert_removed(vert_of_removed_edge.as_span());
  kept_edges.foreach_index([&](const int edge) {
    vert_removed[src_edges[edge][0]] = false;
    vert_removed[src_edges[edge][1]] = false;
  });
  const IndexMask kept_verts = IndexMask::from_bools_inverse(vert_removed, memory);

  Array<int> vert_remap(src_mesh.verts_num, -1);
  index_mask::build_reverse_map<int>(kept_verts, vert_remap);
  Array<int> edge_remap(src_mesh.edges_num, -1);
  index_mask::build_reverse_map<int>(kept_edges, edge_remap);

  /* Every group of joined faces becomes one face, at the place of its first face. */
  Array<int> old_to_new_faces(src_mesh.faces_num);
  Vector<int> first_faces;
  first_faces.reserve(src_mesh.faces_num);
  for (const int face : IndexRange(src_mesh.faces_num)) {
    const int first_face = joiner.first_face(face);
    if (first_face == face) {
      old_to_new_faces[face] = int(first_faces.append_and_get_index(face));
    }
    else {
      old_to_new_faces[face] = old_to_new_faces[first_face];
    }
  }
  const int dst_faces_num = int(first_faces.size());

  Array<int> dst_face_offsets(dst_faces_num + 1);
  for (const int face : first_faces.index_range()) {
    dst_face_offsets[face] = joiner.face_size(first_faces[face]);
  }
  const OffsetIndices<int> dst_faces = offset_indices::accumulate_counts_to_offsets(
      dst_face_offsets);
  const int dst_corners_num = dst_faces.total_size();

  Array<int> dst_to_src_corners(dst_corners_num);
  threading::parallel_for(
      IndexRange(dst_faces_num),
      2048,
      [&](const IndexRange range) {
        for (const int face : range) {
          joiner.face_corners(first_faces[face],
                              dst_to_src_corners.as_mutable_span().slice(dst_faces[face]));
        }
      },
      threading::accumulated_task_sizes(
          [&](const IndexRange range) { return dst_faces[range].size(); }));

  Mesh *dst_mesh = BKE_mesh_new_nomain(
      int(kept_verts.size()), int(kept_edges.size()), dst_faces_num, dst_corners_num);
  BKE_mesh_copy_parameters_for_eval(dst_mesh, &src_mesh);

  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh->attributes_for_write();

  gather_vert_attributes(src_mesh, attribute_filter, kept_verts, *dst_mesh);
  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Edge,
                         bke::AttrDomain::Edge,
                         bke::attribute_filter_with_skip_ref(attribute_filter, {".edge_verts"}),
                         kept_edges,
                         dst_attributes);

  Array<int> new_to_old_face_offsets;
  Array<int> new_to_old_face_indices;
  const GroupedSpan<int> new_to_old_face_map = offset_indices::build_groups_from_indices(
      old_to_new_faces, dst_faces_num, new_to_old_face_offsets, new_to_old_face_indices);
  propagate_attributes(src_attributes,
                       bke::AttrDomain::Face,
                       attribute_filter,
                       new_to_old_face_map,
                       dst_attributes);
  bke::gather_attributes(
      src_attributes,
      bke::AttrDomain::Corner,
      bke::AttrDomain::Corner,
      bke::attribute_filter_with_skip_ref(attribute_filter, {".corner_vert", ".corner_edge"}),
      dst_to_src_corners.as_span(),
      dst_attributes);

  MutableSpan<int2> dst_edges = dst_mesh->edges_for_write();
  kept_edges.foreach_index(
      [&](const int src_edge, const int dst_edge) {
        dst_edges[dst_edge] = int2(vert_remap[src_edges[src_edge][0]],
                                   vert_remap[src_edges[src_edge][1]]);
      },
      exec_mode::grain_size(4096));
  BLI_assert(!dst_edges.as_span().cast<int>().contains(-1));

  dst_mesh->face_offsets_for_write().copy_from(dst_face_offsets);
  MutableSpan<int> dst_corner_verts = dst_mesh->corner_verts_for_write();
  MutableSpan<int> dst_corner_edges = dst_mesh->corner_edges_for_write();
  threading::parallel_for(dst_to_src_corners.index_range(), 4096, [&](const IndexRange range) {
    for (const int corner : range) {
      const int src_corner = dst_to_src_corners[corner];
      dst_corner_verts[corner] = vert_remap[src_corner_verts[src_corner]];
      dst_corner_edges[corner] = edge_remap[src_corner_edges[src_corner]];
    }
  });
  BLI_assert(!dst_corner_verts.as_span().contains(-1));
  BLI_assert(!dst_corner_edges.as_span().contains(-1));

  Array<int> kept_vert_indices(kept_verts.size());
  kept_verts.to_indices<int>(kept_vert_indices);
  gather_orig_indices(
      src_mesh.vert_data, src_mesh.verts_num, kept_vert_indices, dst_mesh->vert_data);
  Array<int> kept_edge_indices(kept_edges.size());
  kept_edges.to_indices<int>(kept_edge_indices);
  gather_orig_indices(
      src_mesh.edge_data, src_mesh.edges_num, kept_edge_indices, dst_mesh->edge_data);
  gather_orig_indices(src_mesh.face_data, src_mesh.faces_num, first_faces, dst_mesh->face_data);

  BLI_assert(bke::mesh_is_valid(*dst_mesh));

  if (dissolve_verts) {
    /* Vertices in between the dissolved edges often only connect two edges now. */
    Array<bool> dissolve_selection(kept_verts.size());
    array_utils::gather(
        vert_of_removed_edge.as_span(), kept_verts, dissolve_selection.as_mutable_span());
    const IndexMask verts_to_dissolve = IndexMask::from_bools(dissolve_selection, memory);
    if (std::optional<Mesh *> result = dissolve_boundary_verts(
            *dst_mesh, verts_to_dissolve, vert_angle_threshold, attribute_filter))
    {
      BKE_id_free(nullptr, dst_mesh);
      return *result;
    }
  }

  debug_randomize_vert_order(dst_mesh);
  debug_randomize_edge_order(dst_mesh);
  debug_randomize_face_order(dst_mesh);

  return dst_mesh;
}

/** \} */

}  // namespace blender::geometry
