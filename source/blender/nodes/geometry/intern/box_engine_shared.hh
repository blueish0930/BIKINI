/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Shared helpers for Box Engine Set Rigid Body + Solver. */

#pragma once

#include "BKE_attribute.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "DNA_pointcloud_types.h"

#include "BLI_array.hh"
#include "BLI_disjoint_set.hh"
#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_vector.hh"
#include "BLI_virtual_array.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BLI_set.hh"

#include "GEO_realize_instances.hh"

#include <algorithm>
#include <cstring>
#include <fmt/format.h>
#include <string>

namespace blender::nodes::box_engine {

using bke::GeometrySet;

constexpr StringRefNull attr_body_name = "body_name";
constexpr StringRefNull attr_compound_proxy = "box_compound_proxy";
constexpr StringRefNull attr_compound_island = "box_compound_island";
/** Instance-domain flag: this instance is a convex hull piece, not a rigid body. */
constexpr StringRefNull attr_compound_hull = "box_compound_hull";

inline void mark_mesh_compound_proxy(Mesh &mesh)
{
  bke::SpanAttributeWriter<bool> writer =
      mesh.attributes_for_write().lookup_or_add_for_write_span<bool>(attr_compound_proxy,
                                                                    bke::AttrDomain::Point);
  if (writer) {
    writer.span.fill(true);
    writer.finish();
  }
}

inline bool mesh_is_compound_proxy(const Mesh &mesh)
{
  return mesh.attributes().contains(attr_compound_proxy);
}

/**
 * Realize nested instances (Object / Collection / Instance-of-instances) so a Mesh can be read.
 * Caller must keep the returned GeometrySet alive while using the Mesh*.
 */
inline GeometrySet collision_geometry_from_reference(const bke::InstanceReference &ref)
{
  GeometrySet geo;
  ref.to_geometry_set(geo);
  if (geo.has_instances()) {
    geometry::RealizeInstancesOptions options;
    options.realize_instance_attributes = false;
    return geometry::realize_instances(std::move(geo), options).geometry;
  }
  return geo;
}

/**
 * Blender Mesh Island (edge connectivity) plus face-only joins.
 * Isolated unused verts are dropped. Islands with fewer than 3 verts are dropped.
 */
inline void fill_islands_from_mesh(const Mesh &mesh, Vector<Vector<float3>> &r_islands)
{
  r_islands.clear();
  if (mesh.verts_num == 0) {
    return;
  }

  DisjointSet<> ds(mesh.verts_num);
  Array<bool> used(mesh.verts_num, false);

  for (const int2 e : mesh.edges()) {
    if (e[0] < 0 || e[1] < 0 || e[0] >= mesh.verts_num || e[1] >= mesh.verts_num) {
      continue;
    }
    ds.join(e[0], e[1]);
    used[e[0]] = true;
    used[e[1]] = true;
  }

  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    if (face.size() < 2) {
      continue;
    }
    const int v0 = corner_verts[face[0]];
    if (v0 < 0 || v0 >= mesh.verts_num) {
      continue;
    }
    used[v0] = true;
    for (const int c : face.drop_front(1)) {
      const int v = corner_verts[c];
      if (v < 0 || v >= mesh.verts_num) {
        continue;
      }
      ds.join(v0, v);
      used[v] = true;
    }
  }

  Map<int, int> root_to_island;
  const Span<float3> positions = mesh.vert_positions();
  for (const int v : IndexRange(mesh.verts_num)) {
    if (!used[v]) {
      continue;
    }
    const int root = int(ds.find_root(v));
    int *island_i = root_to_island.lookup_ptr(root);
    if (!island_i) {
      const int new_i = r_islands.append_and_get_index({});
      root_to_island.add(root, new_i);
      island_i = root_to_island.lookup_ptr(root);
    }
    r_islands[*island_i].append(positions[v]);
  }

  Vector<Vector<float3>> filtered;
  filtered.reserve(r_islands.size());
  for (Vector<float3> &island : r_islands) {
    if (island.size() >= 3) {
      filtered.append(std::move(island));
    }
  }
  r_islands = std::move(filtered);
}

inline void append_transformed_islands(const Mesh &mesh,
                                       const float4x4 &tf,
                                       Vector<Vector<float3>> &r_islands)
{
  Vector<Vector<float3>> local;
  fill_islands_from_mesh(mesh, local);
  for (Vector<float3> &island : local) {
    for (float3 &p : island) {
      p = math::transform_point(tf, p);
    }
    r_islands.append(std::move(island));
  }
}

/**
 * Collect convex-hull source islands from a GeometrySet.
 * Instances: each instance (and mesh-islands inside it) becomes one or more islands,
 * with verts transformed into the GeometrySet's space.
 * A top-level mesh is only used when there are no instances (avoid double-counting
 * a mesh that was wrapped into an instance).
 */
inline void collect_islands_from_geometry(const GeometrySet &geo, Vector<Vector<float3>> &r_islands)
{
  r_islands.clear();
  if (const bke::Instances *instances = geo.get_instances()) {
    if (instances->instances_num() > 0) {
      const Span<float4x4> transforms = instances->transforms();
      const Span<int> handles = instances->reference_handles();
      const Span<bke::InstanceReference> refs = instances->references();
      for (const int i : transforms.index_range()) {
        const int handle = handles[i];
        if (handle < 0 || handle >= refs.size()) {
          continue;
        }
        const GeometrySet extracted = collision_geometry_from_reference(refs[handle]);
        if (const Mesh *mesh = extracted.get_mesh()) {
          append_transformed_islands(*mesh, transforms[i], r_islands);
        }
      }
      return;
    }
  }
  if (const Mesh *mesh = geo.get_mesh()) {
    append_transformed_islands(*mesh, float4x4::identity(), r_islands);
  }
}

/** Mesh with a polyline per island + island-id, so connectivity fallback still works. */
inline Mesh *bake_compound_islands(const Span<Vector<float3>> islands)
{
  int total = 0;
  int edges_num = 0;
  for (const Vector<float3> &island : islands) {
    total += island.size();
    if (island.size() >= 2) {
      edges_num += island.size() - 1;
    }
  }
  if (total == 0) {
    return nullptr;
  }

  Mesh *mesh = BKE_mesh_new_nomain(total, edges_num, 0, 0);
  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  MutableSpan<int2> edges = mesh->edges_for_write();
  bke::SpanAttributeWriter<int> id_w = mesh->attributes_for_write().lookup_or_add_for_write_span<int>(
      attr_compound_island, bke::AttrDomain::Point);
  int offset = 0;
  int e = 0;
  for (const int i : islands.index_range()) {
    const int start = offset;
    for (const float3 &p : islands[i]) {
      positions[offset] = p;
      if (id_w) {
        id_w.span[offset] = i;
      }
      offset++;
    }
    for (int k = 0; k + 1 < islands[i].size(); k++) {
      edges[e++] = int2(start + k, start + k + 1);
    }
  }
  if (id_w) {
    id_w.finish();
  }
  mesh->tag_positions_changed();
  mesh->tag_topology_changed();
  mark_mesh_compound_proxy(*mesh);
  return mesh;
}

inline PointCloud *bake_compound_points(const Span<Vector<float3>> islands)
{
  int total = 0;
  for (const Vector<float3> &island : islands) {
    total += island.size();
  }
  if (total == 0) {
    return nullptr;
  }
  PointCloud *points = BKE_pointcloud_new_nomain(PointCloudType::Points, total);
  MutableSpan<float3> positions = points->positions_for_write();
  bke::MutableAttributeAccessor attributes = points->attributes_for_write();
  bke::SpanAttributeWriter<int> id_w = attributes.lookup_or_add_for_write_span<int>(
      attr_compound_island, bke::AttrDomain::Point);
  bke::SpanAttributeWriter<bool> mark_w = attributes.lookup_or_add_for_write_span<bool>(
      attr_compound_proxy, bke::AttrDomain::Point);
  int offset = 0;
  for (const int i : islands.index_range()) {
    for (const float3 &p : islands[i]) {
      positions[offset] = p;
      if (id_w) {
        id_w.span[offset] = i;
      }
      if (mark_w) {
        mark_w.span[offset] = true;
      }
      offset++;
    }
  }
  if (id_w) {
    id_w.finish();
  }
  if (mark_w) {
    mark_w.finish();
  }
  return points;
}

inline void parse_island_ids(const Span<float3> positions,
                             const VArray<int> &ids,
                             Vector<Vector<float3>> &r_islands)
{
  r_islands.clear();
  if (positions.is_empty() || ids.is_empty() || ids[0] < 0) {
    return;
  }
  Map<int, int> id_to;
  for (const int v : positions.index_range()) {
    const int id = ids[v];
    int *slot = id_to.lookup_ptr(id);
    if (!slot) {
      const int ni = r_islands.append_and_get_index({});
      id_to.add(id, ni);
      slot = id_to.lookup_ptr(id);
    }
    r_islands[*slot].append(positions[v]);
  }
}

inline void parse_baked_islands(const Mesh &proxy, Vector<Vector<float3>> &r_islands)
{
  r_islands.clear();
  if (proxy.verts_num == 0) {
    return;
  }
  const VArray<int> ids = *proxy.attributes().lookup_or_default<int>(
      attr_compound_island, bke::AttrDomain::Point, -1);
  parse_island_ids(proxy.vert_positions(), ids, r_islands);
  if (r_islands.size() >= 2) {
    return;
  }
  fill_islands_from_mesh(proxy, r_islands);
}

inline void parse_baked_points(const PointCloud &points, Vector<Vector<float3>> &r_islands)
{
  r_islands.clear();
  if (points.totpoint == 0) {
    return;
  }
  const VArray<int> ids = *points.attributes().lookup_or_default<int>(
      attr_compound_island, bke::AttrDomain::Point, -1);
  parse_island_ids(points.positions(), ids, r_islands);
}

inline std::string mstring_to_std(const MStringProperty &p)
{
  const int n = std::clamp(int(p.s_len), 0, 127);
  if (n <= 0) {
    return {};
  }
  return std::string(p.s, p.s + n);
}

inline void std_to_mstring(MStringProperty &p, const StringRef src)
{
  const int n = std::min(int(src.size()), 127);
  if (n > 0) {
    memcpy(p.s, src.data(), size_t(n));
  }
  p.s[n] = '\0';
  p.s_len = char(n);
}

/**
 * Write unique instance/point `body_name` only where it is missing.
 * Existing non-empty names are left untouched.
 * `prefix` is used as-is if free, otherwise prefix.001, prefix.002, …
 */
inline void ensure_unique_body_names(bke::MutableAttributeAccessor attributes,
                                     const bke::AttrDomain domain,
                                     const int size,
                                     const StringRef prefix,
                                     const VArray<bool> *skip = nullptr)
{
  if (size <= 0) {
    return;
  }
  bke::SpanAttributeWriter<MStringProperty> names =
      attributes.lookup_or_add_for_write_span<MStringProperty>(attr_body_name, domain);
  if (!names) {
    return;
  }
  Set<std::string> used;
  for (const int i : IndexRange(size)) {
    const std::string existing = mstring_to_std(names.span[i]);
    if (!existing.empty()) {
      used.add(existing);
    }
  }
  const std::string base = prefix.is_empty() ? std::string("Body") : std::string(prefix);
  auto next_from_prefix = [&]() -> std::string {
    if (!used.contains(base)) {
      used.add(base);
      return base;
    }
    for (int serial = 1; serial < 1000000; serial++) {
      const std::string candidate = fmt::format("{}.{:03d}", base, serial);
      if (!used.contains(candidate)) {
        used.add(candidate);
        return candidate;
      }
    }
    return fmt::format("{}.X", base);
  };
  for (const int i : IndexRange(size)) {
    if (skip && (*skip)[i]) {
      continue;
    }
    if (names.span[i].s_len > 0) {
      continue;
    }
    std_to_mstring(names.span[i], next_from_prefix());
  }
  names.finish();
}

inline void collect_islands_from_instance_indices(const bke::Instances &instances,
                                                  const Span<int> indices,
                                                  Vector<Vector<float3>> &r_islands)
{
  r_islands.clear();
  const Span<float4x4> transforms = instances.transforms();
  const Span<int> handles = instances.reference_handles();
  const Span<bke::InstanceReference> refs = instances.references();
  for (const int i : indices) {
    if (i < 0 || i >= transforms.size()) {
      continue;
    }
    const int handle = handles[i];
    if (handle < 0 || handle >= refs.size()) {
      continue;
    }
    const GeometrySet extracted = collision_geometry_from_reference(refs[handle]);
    if (const Mesh *mesh = extracted.get_mesh()) {
      append_transformed_islands(*mesh, transforms[i], r_islands);
    }
  }
}

/**
 * Baked proxy first. Never fall back to "every instance in the GeometrySet" —
 * Join Geometry with falling boxes would turn those boxes into extra hulls and explode.
 */
inline void resolve_compound_islands(const GeometrySet &geo,
                                     const Span<int> compound_instance_indices,
                                     Vector<Vector<float3>> &r_islands)
{
  r_islands.clear();
  if (const Mesh *mesh = geo.get_mesh()) {
    if (mesh_is_compound_proxy(*mesh) || mesh->attributes().contains(attr_compound_island)) {
      parse_baked_islands(*mesh, r_islands);
    }
  }
  if (r_islands.size() < 2) {
    if (const PointCloud *points = geo.get_pointcloud()) {
      if (points->attributes().contains(attr_compound_proxy)) {
        /* Only points stamped as the compound bake — ignore joined leftovers. */
        const VArray<bool> mark = *points->attributes().lookup_or_default<bool>(
            attr_compound_proxy, bke::AttrDomain::Point, false);
        const VArray<int> ids = *points->attributes().lookup_or_default<int>(
            attr_compound_island, bke::AttrDomain::Point, -1);
        Vector<float3> kept_pos;
        Vector<int> kept_ids;
        const Span<float3> positions = points->positions();
        for (const int i : positions.index_range()) {
          if (!mark[i] || ids[i] < 0) {
            continue;
          }
          kept_pos.append(positions[i]);
          kept_ids.append(ids[i]);
        }
        if (!kept_pos.is_empty()) {
          parse_island_ids(kept_pos, VArray<int>::from_span(kept_ids.as_span()), r_islands);
        }
      }
    }
  }
  if (r_islands.size() < 2 && geo.get_instances() && !compound_instance_indices.is_empty()) {
    collect_islands_from_instance_indices(
        *geo.get_instances(), compound_instance_indices, r_islands);
  }
}

/** True if any edge is used by only one face (open box, cup, tube, …). */
inline bool mesh_has_boundary_edges(const Mesh &mesh)
{
  if (mesh.edges_num <= 0 || mesh.faces_num <= 0) {
    return false;
  }
  Array<int> users(mesh.edges_num, 0);
  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_edges = mesh.corner_edges();
  for (const int fi : faces.index_range()) {
    for (const int e : corner_edges.slice(faces[fi])) {
      if (uint(e) < uint(mesh.edges_num)) {
        users[e]++;
      }
    }
  }
  for (const int c : users) {
    if (c == 1) {
      return true;
    }
  }
  return false;
}

inline void fill_triangle_mesh_from_mesh(const Mesh &mesh,
                                         Vector<float3> &r_verts,
                                         Vector<int> &r_indices)
{
  r_verts.extend(mesh.vert_positions());
  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    if (face.size() < 3) {
      continue;
    }
    const int v0 = corner_verts[face[0]];
    for (int k = 1; k + 1 < face.size(); k++) {
      r_indices.append(v0);
      r_indices.append(corner_verts[face[k]]);
      r_indices.append(corner_verts[face[k + 1]]);
    }
  }
}

}  // namespace blender::nodes::box_engine
