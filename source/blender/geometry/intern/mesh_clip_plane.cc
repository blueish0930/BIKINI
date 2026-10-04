/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_mesh_clip_plane.hh"

#include "DNA_object_types.h"

#include "BLI_array.hh"
#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_vector.hh"
#include "BLI_offset_indices.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_deform.hh"
#include "BKE_mesh.hh"

#include "DNA_meshdata_types.h"

namespace blender::geometry {
namespace {

constexpr float plane_eps = 1.0e-6f;

struct PointSource {
  int index_a = 0;
  int index_b = 0;
  float factor = 0.0f;

  bool is_original() const
  {
    return index_a == index_b;
  }
};

struct FaceBuild {
  int src_face = 0;
  Vector<PointSource, 8> points;
};

struct EdgeBuild {
  int src_edge = -1;
  std::array<PointSource, 2> points;
};

struct SideBuild {
  Vector<int> original_faces;
  Vector<FaceBuild> clipped_faces;
  Vector<EdgeBuild> loose_edges;
  Vector<PointSource> isolated_points;
};

static PointSource source_point(const int index)
{
  return {index, index, 0.0f};
}

static PointSource intersection_point(const int index_a,
                                      const int index_b,
                                      const float distance_a,
                                      const float distance_b)
{
  const float denom = distance_a - distance_b;
  float factor = 0.0f;
  if (std::abs(denom) > plane_eps) {
    factor = std::clamp(distance_a / denom, 0.0f, 1.0f);
  }
  if (factor <= plane_eps) {
    return source_point(index_a);
  }
  if (factor >= 1.0f - plane_eps) {
    return source_point(index_b);
  }
  return {index_a, index_b, factor};
}

static bool source_equal(const PointSource &a, const PointSource &b)
{
  if (a.is_original() && b.is_original()) {
    return a.index_a == b.index_a;
  }
  return a.index_a == b.index_a && a.index_b == b.index_b &&
         std::abs(a.factor - b.factor) <= plane_eps;
}

static void append_unique(Vector<PointSource, 8> &points, const PointSource &point)
{
  if (points.is_empty() || !source_equal(points.last(), point)) {
    points.append(point);
  }
}

static bool is_inside(const float distance, const bool keep_above)
{
  return keep_above ? distance >= -plane_eps : distance <= plane_eps;
}

static Vector<PointSource, 8> clip_polygon(const Span<int> corner_verts,
                                           const Span<float> distances,
                                           const bool keep_above)
{
  Vector<PointSource, 8> output;
  if (corner_verts.is_empty()) {
    return output;
  }

  const int n = corner_verts.size();
  for (const int i : IndexRange(n)) {
    const int prev_i = (i + n - 1) % n;
    const int vert_prev = corner_verts[prev_i];
    const int vert_curr = corner_verts[i];
    const float d_prev = distances[vert_prev];
    const float d_curr = distances[vert_curr];
    const bool inside_prev = is_inside(d_prev, keep_above);
    const bool inside_curr = is_inside(d_curr, keep_above);

    if (inside_curr) {
      if (!inside_prev) {
        append_unique(output, intersection_point(vert_prev, vert_curr, d_prev, d_curr));
      }
      append_unique(output, source_point(vert_curr));
    }
    else if (inside_prev) {
      append_unique(output, intersection_point(vert_prev, vert_curr, d_prev, d_curr));
    }
  }

  if (output.size() >= 2 && source_equal(output.first(), output.last())) {
    output.pop_last();
  }
  return output;
}


static int2 ordered_edge(const int a, const int b)
{
  return a < b ? int2(a, b) : int2(b, a);
}

static float3 evaluate_position(const Span<float3> positions, const PointSource &source)
{
  if (source.is_original()) {
    return positions[source.index_a];
  }
  return math::interpolate(positions[source.index_a], positions[source.index_b], source.factor);
}

static void mix_attribute_value(const GVArray &src,
                                const bke::AttrType data_type,
                                const PointSource &source,
                                void *dst)
{
  if (source.is_original()) {
    src.get(source.index_a, dst);
    return;
  }
  if (data_type == bke::AttrType::String) {
    src.get(source.factor < 0.5f ? source.index_a : source.index_b, dst);
    return;
  }

  const CPPType &type = src.type();
  BUFFER_FOR_CPP_TYPE_VALUE(type, value_a);
  BUFFER_FOR_CPP_TYPE_VALUE(type, value_b);
  src.get(source.index_a, value_a);
  src.get(source.index_b, value_b);

  bool mixed = false;
  bke::attribute_math::to_static_type(type, [&]<typename T>() {
    if constexpr (!std::is_void_v<bke::attribute_math::DefaultMixer<T>>) {
      *static_cast<T *>(dst) = bke::attribute_math::mix2(
          source.factor, *static_cast<const T *>(value_a), *static_cast<const T *>(value_b));
      mixed = true;
    }
  });
  if (!mixed) {
    /* Other non-mixable types take the nearest endpoint. */
    src.get(source.factor < 0.5f ? source.index_a : source.index_b, dst);
  }
}

static void fill_point_attributes(const Mesh &src_mesh,
                                  const Span<PointSource> point_sources,
                                  const bke::AttributeFilter &attribute_filter,
                                  Mesh &dst_mesh)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();

  Set<std::string> vertex_group_names;
  for (const bDeformGroup &group : src_mesh.vertex_group_names) {
    vertex_group_names.add(group.name);
  }

  const Span<MDeformVert> src_dverts = src_mesh.deform_verts();
  if (!vertex_group_names.is_empty() && !src_dverts.is_empty()) {
    MutableSpan<MDeformVert> dst_dverts = dst_mesh.deform_verts_for_write();
    for (const int i : point_sources.index_range()) {
      const PointSource &source = point_sources[i];
      const int src_index = source.is_original() ?
                                source.index_a :
                                (source.factor < 0.5f ? source.index_a : source.index_b);
      BKE_defvert_copy(&dst_dverts[i], &src_dverts[src_index]);
    }
  }

  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != bke::AttrDomain::Point) {
      return;
    }
    if (ELEM(iter.name, "position") || attribute_filter.allow_skip(iter.name) ||
        vertex_group_names.contains(iter.name))
    {
      return;
    }
    const GVArray src = *iter.get();
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        iter.name, bke::AttrDomain::Point, iter.data_type);
    if (!dst) {
      return;
    }
    for (const int i : point_sources.index_range()) {
      mix_attribute_value(src, iter.data_type, point_sources[i], dst.span[i]);
    }
    dst.finish();
  });
}

static void fill_face_attributes(const Mesh &src_mesh,
                                 const Span<int> src_faces,
                                 const bke::AttributeFilter &attribute_filter,
                                 Mesh &dst_mesh)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();
  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != bke::AttrDomain::Face || attribute_filter.allow_skip(iter.name)) {
      return;
    }
    const GVArraySpan src = *iter.get();
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        iter.name, bke::AttrDomain::Face, iter.data_type);
    if (!dst) {
      return;
    }
    bke::attribute_math::gather(src, src_faces, dst.span);
    dst.finish();
  });
}

static std::optional<int2> original_edge_for_output_edge(const PointSource &a,
                                                         const PointSource &b)
{
  if (a.is_original() && b.is_original()) {
    return a.index_a != b.index_a ? std::optional<int2>(ordered_edge(a.index_a, b.index_a)) :
                                   std::nullopt;
  }
  if (!a.is_original() && !b.is_original()) {
    const int2 edge_a = ordered_edge(a.index_a, a.index_b);
    const int2 edge_b = ordered_edge(b.index_a, b.index_b);
    return edge_a == edge_b ? std::optional<int2>(edge_a) : std::nullopt;
  }
  const PointSource &intersection = a.is_original() ? b : a;
  const int original = a.is_original() ? a.index_a : b.index_a;
  return ELEM(original, intersection.index_a, intersection.index_b) ?
             std::optional<int2>(ordered_edge(intersection.index_a, intersection.index_b)) :
             std::nullopt;
}

static void fill_edge_attributes_from_map(const Mesh &src_mesh,
                                          const Span<int> src_edge_by_dst_edge,
                                          const bke::AttributeFilter &attribute_filter,
                                          Mesh &dst_mesh)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();
  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != bke::AttrDomain::Edge || attribute_filter.allow_skip(iter.name) ||
        iter.name == ".edge_verts")
    {
      return;
    }
    const GVArray src = *iter.get();
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        iter.name, bke::AttrDomain::Edge, iter.data_type);
    if (!dst) {
      return;
    }
    const CPPType &type = src.type();
    for (const int edge_i : src_edge_by_dst_edge.index_range()) {
      const int src_edge = src_edge_by_dst_edge[edge_i];
      if (src_edge >= 0) {
        src.get(src_edge, dst.span[edge_i]);
      }
      else {
        type.value_initialize(dst.span[edge_i]);
      }
    }
    dst.finish();
  });
}

static void fill_edge_attributes(const Mesh &src_mesh,
                                 const Span<PointSource> point_sources,
                                 const bke::AttributeFilter &attribute_filter,
                                 Mesh &dst_mesh)
{
  Map<int2, int> src_edge_by_verts;
  for (const int edge_i : src_mesh.edges().index_range()) {
    const int2 edge = src_mesh.edges()[edge_i];
    src_edge_by_verts.add(ordered_edge(edge[0], edge[1]), edge_i);
  }
  Array<int> edge_map(dst_mesh.edges_num, -1);
  for (const int edge_i : dst_mesh.edges().index_range()) {
    const int2 edge = dst_mesh.edges()[edge_i];
    if (const std::optional<int2> original_edge = original_edge_for_output_edge(
            point_sources[edge[0]], point_sources[edge[1]]))
    {
      edge_map[edge_i] = src_edge_by_verts.lookup_default(*original_edge, -1);
    }
  }
  fill_edge_attributes_from_map(src_mesh, edge_map, attribute_filter, dst_mesh);
}

static void fill_corner_attributes(const Mesh &src_mesh,
                                   const SideBuild &side,
                                   const bke::AttributeFilter &attribute_filter,
                                   Mesh &dst_mesh)
{
  const bke::AttributeAccessor src_attributes = src_mesh.attributes();
  const OffsetIndices src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  bke::MutableAttributeAccessor dst_attributes = dst_mesh.attributes_for_write();

  src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.domain != bke::AttrDomain::Corner || attribute_filter.allow_skip(iter.name) ||
        ELEM(iter.name, ".corner_vert", ".corner_edge"))
    {
      return;
    }
    const GVArray src = *iter.get();
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(
        iter.name, bke::AttrDomain::Corner, iter.data_type);
    if (!dst) {
      return;
    }

    const CPPType &type = src.type();
    int dst_corner = 0;
    for (const int src_face_i : side.original_faces) {
      for (const int src_corner : src_faces[src_face_i]) {
        src.get(src_corner, dst.span[dst_corner++]);
      }
    }
    for (const FaceBuild &face : side.clipped_faces) {
      const IndexRange src_face = src_faces[face.src_face];
      const Span<int> face_verts = src_corner_verts.slice(src_face);
      for (const PointSource &point : face.points) {
        if (point.is_original()) {
          const int local = face_verts.first_index_try(point.index_a);
          if (local != -1) {
            src.get(src_face[local], dst.span[dst_corner]);
          }
          else {
            type.value_initialize(dst.span[dst_corner]);
          }
        }
        else {
          int corner_a = -1;
          int corner_b = -1;
          for (const int local : face_verts.index_range()) {
            const int next = (local + 1) % face_verts.size();
            if (face_verts[local] == point.index_a && face_verts[next] == point.index_b) {
              corner_a = src_face[local];
              corner_b = src_face[next];
              break;
            }
            if (face_verts[local] == point.index_b && face_verts[next] == point.index_a) {
              corner_a = src_face[next];
              corner_b = src_face[local];
              break;
            }
          }
          if (corner_a == -1) {
            type.value_initialize(dst.span[dst_corner]);
          }
          else {
            PointSource corner_source = point;
            corner_source.index_a = corner_a;
            corner_source.index_b = corner_b;
            mix_attribute_value(src, iter.data_type, corner_source, dst.span[dst_corner]);
          }
        }
        dst_corner++;
      }
    }
    dst.finish();
  });
}

}  // namespace

static Mesh *build_side_mesh(const Mesh &src_mesh,
                             const SideBuild &side,
                             const bke::AttributeFilter &attribute_filter,
                             const Span<float> distances = {},
                             Array<bool> *r_clip_boundary = nullptr)
{
  if (side.original_faces.is_empty() && side.clipped_faces.is_empty() &&
      side.loose_edges.is_empty() && side.isolated_points.is_empty())
  {
    return nullptr;
  }

  Array<int> original_to_output(src_mesh.verts_num, -1);
  Map<int2, int> intersection_to_output;
  Vector<PointSource> point_sources;
  point_sources.reserve(src_mesh.verts_num);
  auto ensure_point = [&](const PointSource &source) -> int {
    if (source.is_original()) {
      int &existing = original_to_output[source.index_a];
      if (existing != -1) {
        return existing;
      }
      existing = point_sources.size();
      point_sources.append(source);
      return existing;
    }
    const int2 key = ordered_edge(source.index_a, source.index_b);
    if (const int *existing = intersection_to_output.lookup_ptr(key)) {
      return *existing;
    }
    PointSource canonical = source;
    if (source.index_a > source.index_b) {
      canonical = {source.index_b, source.index_a, 1.0f - source.factor};
    }
    const int index = point_sources.size();
    point_sources.append(canonical);
    intersection_to_output.add(key, index);
    return index;
  };

  const int faces_num = side.original_faces.size() + side.clipped_faces.size();
  Array<int> face_offsets(faces_num + 1);
  const OffsetIndices src_faces = src_mesh.faces();
  for (const int i : side.original_faces.index_range()) {
    face_offsets[i] = src_faces[side.original_faces[i]].size();
  }
  for (const int i : side.clipped_faces.index_range()) {
    face_offsets[side.original_faces.size() + i] = side.clipped_faces[i].points.size();
  }
  const OffsetIndices faces = offset_indices::accumulate_counts_to_offsets(face_offsets);
  const int corners_num = face_offsets.last();
  Array<int> corner_verts(corners_num);
  Array<int> src_face_map(faces_num);
  for (const int i : side.original_faces.index_range()) {
    const int src_face = side.original_faces[i];
    src_face_map[i] = src_face;
    MutableSpan<int> dst_verts = corner_verts.as_mutable_span().slice(faces[i]);
    const Span<int> src_verts = src_mesh.corner_verts().slice(src_faces[src_face]);
    for (const int j : src_verts.index_range()) {
      dst_verts[j] = ensure_point(source_point(src_verts[j]));
    }
  }
  for (const int i : side.clipped_faces.index_range()) {
    const int dst_face = side.original_faces.size() + i;
    const FaceBuild &face = side.clipped_faces[i];
    src_face_map[dst_face] = face.src_face;
    MutableSpan<int> dst_verts = corner_verts.as_mutable_span().slice(faces[dst_face]);
    for (const int j : face.points.index_range()) {
      dst_verts[j] = ensure_point(face.points[j]);
    }
  }

  Vector<int2> output_edges;
  Vector<int> src_edge_by_dst_edge;
  Array<int> dst_edge_by_src_edge(src_mesh.edges_num, -1);
  Map<int2, int> new_edge_by_verts;
  auto ensure_edge = [&](const int vert_a, const int vert_b, const int src_edge) -> int {
    if (src_edge >= 0) {
      int &dst_edge = dst_edge_by_src_edge[src_edge];
      if (dst_edge != -1) {
        return dst_edge;
      }
      dst_edge = output_edges.size();
      output_edges.append(int2(vert_a, vert_b));
      src_edge_by_dst_edge.append(src_edge);
      return dst_edge;
    }
    const int2 key = ordered_edge(vert_a, vert_b);
    if (const int *dst_edge = new_edge_by_verts.lookup_ptr(key)) {
      return *dst_edge;
    }
    const int dst_edge = output_edges.size();
    output_edges.append(int2(vert_a, vert_b));
    src_edge_by_dst_edge.append(-1);
    new_edge_by_verts.add(key, dst_edge);
    return dst_edge;
  };

  Array<int> corner_edges(corners_num);
  const Span<int> src_corner_edges = src_mesh.corner_edges();
  for (const int i : side.original_faces.index_range()) {
    const IndexRange dst_face = faces[i];
    const IndexRange src_face = src_faces[side.original_faces[i]];
    for (const int j : dst_face.index_range()) {
      const int next = (j + 1) % dst_face.size();
      corner_edges[dst_face[j]] = ensure_edge(corner_verts[dst_face[j]],
                                              corner_verts[dst_face[next]],
                                              src_corner_edges[src_face[j]]);
    }
  }
  const Span<int2> src_edges = src_mesh.edges();
  for (const int i : side.clipped_faces.index_range()) {
    const int dst_face_i = side.original_faces.size() + i;
    const IndexRange dst_face = faces[dst_face_i];
    const FaceBuild &face = side.clipped_faces[i];
    const IndexRange src_face = src_faces[face.src_face];
    for (const int j : dst_face.index_range()) {
      const int next = (j + 1) % dst_face.size();
      int src_edge = -1;
      if (const std::optional<int2> original_edge = original_edge_for_output_edge(
              face.points[j], face.points[next]))
      {
        for (const int src_corner : src_face) {
          const int candidate = src_corner_edges[src_corner];
          if (ordered_edge(src_edges[candidate][0], src_edges[candidate][1]) == *original_edge) {
            src_edge = candidate;
            break;
          }
        }
      }
      corner_edges[dst_face[j]] = ensure_edge(
          corner_verts[dst_face[j]], corner_verts[dst_face[next]], src_edge);
    }
  }
  for (const EdgeBuild &edge : side.loose_edges) {
    ensure_edge(ensure_point(edge.points[0]), ensure_point(edge.points[1]), edge.src_edge);
  }
  for (const PointSource &point : side.isolated_points) {
    ensure_point(point);
  }

  Mesh *mesh = bke::mesh_new_no_attributes(
      point_sources.size(), output_edges.size(), faces_num, corners_num);
  BKE_mesh_copy_parameters_for_eval(mesh, &src_mesh);
  mesh->face_offsets_for_write().copy_from(face_offsets);
  mesh->corners_num = corners_num;

  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
  attributes.add<float3>("position", bke::AttrDomain::Point, bke::AttributeInitConstruct());
  attributes.add<int>(".corner_vert", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  attributes.add<int>(".corner_edge", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  if (!output_edges.is_empty()) {
    attributes.add<int2>(".edge_verts", bke::AttrDomain::Edge, bke::AttributeInitConstruct());
    mesh->edges_for_write().copy_from(output_edges);
  }

  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  const Span<float3> src_positions = src_mesh.vert_positions();
  for (const int i : point_sources.index_range()) {
    positions[i] = evaluate_position(src_positions, point_sources[i]);
  }
  mesh->corner_verts_for_write().copy_from(corner_verts);
  mesh->corner_edges_for_write().copy_from(corner_edges);
  fill_point_attributes(src_mesh, point_sources, attribute_filter, *mesh);
  fill_edge_attributes_from_map(src_mesh, src_edge_by_dst_edge, attribute_filter, *mesh);
  fill_face_attributes(src_mesh, src_face_map, attribute_filter, *mesh);
  fill_corner_attributes(src_mesh, side, attribute_filter, *mesh);
  if (r_clip_boundary != nullptr) {
    r_clip_boundary->reinitialize(point_sources.size());
    for (const int i : point_sources.index_range()) {
      const PointSource &source = point_sources[i];
      bool on_cut = !source.is_original();
      if (!on_cut && source.is_original() && !distances.is_empty()) {
        on_cut = std::abs(distances[source.index_a]) <= plane_eps;
      }
      (*r_clip_boundary)[i] = on_cut;
    }
  }
  mesh->tag_topology_changed();
  return mesh;
}

MeshPlaneClipResult mesh_clip_by_plane_both(const Mesh &src_mesh,
                                            const float3 &plane_position,
                                            const float3 &plane_normal,
                                            const bool need_above,
                                            const bool need_below,
                                            const bke::AttributeFilter &above_filter,
                                            const bke::AttributeFilter &below_filter)
{
  if (src_mesh.verts_num == 0 || (!need_above && !need_below)) {
    return {};
  }

  const Span<float3> positions = src_mesh.vert_positions();
  const Span<int2> edges = src_mesh.edges();
  const OffsetIndices faces = src_mesh.faces();
  const Span<int> corner_verts = src_mesh.corner_verts();
  const Span<int> corner_edges = src_mesh.corner_edges();
  Array<float> distances(src_mesh.verts_num);
  threading::parallel_for(positions.index_range(), 4096, [&](const IndexRange range) {
    for (const int vert : range) {
      const float distance = math::dot(positions[vert] - plane_position, plane_normal);
      distances[vert] = std::abs(distance) <= plane_eps ? 0.0f : distance;
    }
  });

  SideBuild above;
  SideBuild below;
  above.original_faces.reserve(need_above ? src_mesh.faces_num : 0);
  below.original_faces.reserve(need_below ? src_mesh.faces_num : 0);
  for (const int face_i : faces.index_range()) {
    const Span<int> verts = corner_verts.slice(faces[face_i]);
    bool all_above = true;
    bool all_below = true;
    for (const int vert : verts) {
      all_above &= distances[vert] >= -plane_eps;
      all_below &= distances[vert] <= plane_eps;
    }
    if (need_above && all_above) {
      above.original_faces.append(face_i);
    }
    else if (need_above && !all_below) {
      Vector<PointSource, 8> clipped = clip_polygon(verts, distances, true);
      if (clipped.size() >= 3) {
        above.clipped_faces.append({face_i, std::move(clipped)});
      }
    }
    if (need_below && all_below) {
      below.original_faces.append(face_i);
    }
    else if (need_below && !all_above) {
      Vector<PointSource, 8> clipped = clip_polygon(verts, distances, false);
      if (clipped.size() >= 3) {
        below.clipped_faces.append({face_i, std::move(clipped)});
      }
    }
  }

  Array<bool> edge_used_by_face(src_mesh.edges_num, false);
  for (const int face_i : faces.index_range()) {
    for (const int edge : corner_edges.slice(faces[face_i])) {
      edge_used_by_face[edge] = true;
    }
  }
  auto append_loose_edge = [&](SideBuild &side,
                               const int src_edge,
                               const int2 edge,
                               const bool keep_above) {
    const bool in0 = is_inside(distances[edge[0]], keep_above);
    const bool in1 = is_inside(distances[edge[1]], keep_above);
    if (in0 && in1) {
      side.loose_edges.append({src_edge, {source_point(edge[0]), source_point(edge[1])}});
    }
    else if (in0 != in1) {
      const PointSource intersection = intersection_point(
          edge[0], edge[1], distances[edge[0]], distances[edge[1]]);
      side.loose_edges.append(
          {src_edge,
           {in0 ? source_point(edge[0]) : intersection,
            in0 ? intersection : source_point(edge[1])}});
    }
  };
  for (const int edge_i : edges.index_range()) {
    if (!edge_used_by_face[edge_i]) {
      if (need_above) {
        append_loose_edge(above, edge_i, edges[edge_i], true);
      }
      if (need_below) {
        append_loose_edge(below, edge_i, edges[edge_i], false);
      }
    }
  }

  Array<bool> vert_used(src_mesh.verts_num, false);
  for (const int2 edge : edges) {
    vert_used[edge[0]] = true;
    vert_used[edge[1]] = true;
  }
  for (const int face_i : faces.index_range()) {
    for (const int vert : corner_verts.slice(faces[face_i])) {
      vert_used[vert] = true;
    }
  }
  for (const int vert : IndexRange(src_mesh.verts_num)) {
    if (!vert_used[vert]) {
      if (need_above && is_inside(distances[vert], true)) {
        above.isolated_points.append(source_point(vert));
      }
      if (need_below && is_inside(distances[vert], false)) {
        below.isolated_points.append(source_point(vert));
      }
    }
  }

  MeshPlaneClipResult result;
  threading::parallel_invoke(
      need_above && need_below,
      [&] {
        result.above = build_side_mesh(
            src_mesh, above, above_filter, distances, &result.above_clip_boundary);
      },
      [&] {
        result.below = build_side_mesh(
            src_mesh, below, below_filter, distances, &result.below_clip_boundary);
      });
  return result;
}

Mesh *mesh_clip_by_plane(const Mesh &src_mesh,
                         const float3 &plane_position,
                         const float3 &plane_normal,
                         const bool keep_above,
                         const bke::AttributeFilter &attribute_filter)
{
  if (src_mesh.verts_num == 0) {
    return nullptr;
  }

  const Span<float3> src_positions = src_mesh.vert_positions();
  const Span<int2> src_edges = src_mesh.edges();
  const OffsetIndices src_faces = src_mesh.faces();
  const Span<int> src_corner_verts = src_mesh.corner_verts();
  const Span<int> src_corner_edges = src_mesh.corner_edges();

  Array<float> distances(src_mesh.verts_num);
  threading::parallel_for(src_positions.index_range(), 4096, [&](const IndexRange range) {
    for (const int vert : range) {
      float distance = math::dot(src_positions[vert] - plane_position, plane_normal);
      if (std::abs(distance) <= plane_eps) {
        distance = 0.0f;
      }
      distances[vert] = distance;
    }
  });

  Vector<FaceBuild> out_faces;
  out_faces.reserve(src_mesh.faces_num);
  for (const int face_i : src_faces.index_range()) {
    Vector<PointSource, 8> clipped = clip_polygon(
        src_corner_verts.slice(src_faces[face_i]), distances, keep_above);
    if (clipped.size() >= 3) {
      out_faces.append({face_i, std::move(clipped)});
    }
  }

  Vector<std::array<PointSource, 2>> out_loose_edges;
  if (src_mesh.edges_num > 0) {
    Array<bool> edge_used_by_face(src_mesh.edges_num, false);
    for (const int face_i : src_faces.index_range()) {
      for (const int edge : src_corner_edges.slice(src_faces[face_i])) {
        edge_used_by_face[edge] = true;
      }
    }
    for (const int edge_i : src_edges.index_range()) {
      if (edge_used_by_face[edge_i]) {
        continue;
      }
      const int2 edge = src_edges[edge_i];
      const float d0 = distances[edge[0]];
      const float d1 = distances[edge[1]];
      const bool in0 = is_inside(d0, keep_above);
      const bool in1 = is_inside(d1, keep_above);
      if (in0 && in1) {
        out_loose_edges.append({source_point(edge[0]), source_point(edge[1])});
      }
      else if (in0 != in1) {
        const PointSource isect = intersection_point(edge[0], edge[1], d0, d1);
        out_loose_edges.append(
            {in0 ? source_point(edge[0]) : isect, in0 ? isect : source_point(edge[1])});
      }
    }
  }

  Vector<PointSource> out_isolated_points;
  {
    Array<bool> vert_used(src_mesh.verts_num, false);
    for (const int2 edge : src_edges) {
      vert_used[edge[0]] = true;
      vert_used[edge[1]] = true;
    }
    for (const int face_i : src_faces.index_range()) {
      for (const int vert : src_corner_verts.slice(src_faces[face_i])) {
        vert_used[vert] = true;
      }
    }
    for (const int vert : IndexRange(src_mesh.verts_num)) {
      if (!vert_used[vert] && is_inside(distances[vert], keep_above)) {
        out_isolated_points.append(source_point(vert));
      }
    }
  }

  if (out_faces.is_empty() && out_loose_edges.is_empty() && out_isolated_points.is_empty()) {
    return nullptr;
  }

  Map<int, int> original_to_output;
  Map<int2, int> intersection_to_output;
  Vector<PointSource> point_sources;
  point_sources.reserve(src_mesh.verts_num);

  auto ensure_point = [&](const PointSource &source) -> int {
    if (source.is_original()) {
      if (const int *existing = original_to_output.lookup_ptr(source.index_a)) {
        return *existing;
      }
      const int index = point_sources.size();
      point_sources.append(source);
      original_to_output.add(source.index_a, index);
      return index;
    }
    const int2 key = ordered_edge(source.index_a, source.index_b);
    if (const int *existing = intersection_to_output.lookup_ptr(key)) {
      return *existing;
    }
    PointSource canonical = source;
    if (source.index_a > source.index_b) {
      canonical = {source.index_b, source.index_a, 1.0f - source.factor};
    }
    const int index = point_sources.size();
    point_sources.append(canonical);
    intersection_to_output.add(key, index);
    return index;
  };

  const int faces_num = out_faces.size();
  Array<int> face_offsets(faces_num + 1);
  int corners_num = 0;
  for (const int face_i : IndexRange(faces_num)) {
    face_offsets[face_i] = out_faces[face_i].points.size();
    corners_num += face_offsets[face_i];
  }
  const OffsetIndices faces = offset_indices::accumulate_counts_to_offsets(face_offsets);

  Array<int> corner_verts(corners_num);
  Array<int> src_face_map(faces_num);
  for (const int face_i : IndexRange(faces_num)) {
    const FaceBuild &face = out_faces[face_i];
    src_face_map[face_i] = face.src_face;
    MutableSpan<int> face_corner_verts = corner_verts.as_mutable_span().slice(faces[face_i]);
    for (const int i : face.points.index_range()) {
      face_corner_verts[i] = ensure_point(face.points[i]);
    }
  }

  Array<int2> loose_edges(out_loose_edges.size());
  for (const int i : out_loose_edges.index_range()) {
    loose_edges[i] = int2(ensure_point(out_loose_edges[i][0]), ensure_point(out_loose_edges[i][1]));
  }
  for (const PointSource &point : out_isolated_points) {
    ensure_point(point);
  }

  Mesh *mesh = bke::mesh_new_no_attributes(
      point_sources.size(), loose_edges.size(), faces_num, corners_num);
  BKE_mesh_copy_parameters_for_eval(mesh, &src_mesh);

  mesh->face_offsets_for_write().copy_from(face_offsets);
  mesh->corners_num = corners_num;

  bke::MutableAttributeAccessor attributes = mesh->attributes_for_write();
  attributes.add<float3>("position", bke::AttrDomain::Point, bke::AttributeInitConstruct());
  attributes.add<int>(".corner_vert", bke::AttrDomain::Corner, bke::AttributeInitConstruct());
  if (!loose_edges.is_empty()) {
    attributes.add<int2>(".edge_verts", bke::AttrDomain::Edge, bke::AttributeInitConstruct());
    mesh->edges_for_write().copy_from(loose_edges);
  }

  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  for (const int i : point_sources.index_range()) {
    positions[i] = evaluate_position(src_positions, point_sources[i]);
  }
  mesh->corner_verts_for_write().copy_from(corner_verts);

  bke::mesh_calc_edges(*mesh, !loose_edges.is_empty(), false);

  fill_point_attributes(src_mesh, point_sources, attribute_filter, *mesh);
  fill_edge_attributes(src_mesh, point_sources, attribute_filter, *mesh);
  fill_face_attributes(src_mesh, src_face_map, attribute_filter, *mesh);
  SideBuild corner_side;
  corner_side.clipped_faces = std::move(out_faces);
  fill_corner_attributes(src_mesh, corner_side, attribute_filter, *mesh);

  mesh->tag_topology_changed();
  return mesh;
}

}  // namespace blender::geometry
