/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Shared attribute helpers for deformable solver nodes. */

#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>

#include "BKE_attribute.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "GEO_realize_instances.hh"

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

namespace blender::nodes::deformable_solver {

using bke::GeometrySet;

inline VArray<float> get_float(const bke::AttributeAccessor &attributes,
                               const StringRef name,
                               const bke::AttrDomain domain,
                               const float fallback)
{
  return *attributes.lookup_or_default<float>(name, domain, fallback);
}

inline VArray<float3> get_float3(const bke::AttributeAccessor &attributes,
                                 const StringRef name,
                                 const bke::AttrDomain domain,
                                 const float3 fallback)
{
  return *attributes.lookup_or_default<float3>(name, domain, fallback);
}

inline VArray<bool> get_bool(const bke::AttributeAccessor &attributes,
                             const StringRef name,
                             const bke::AttrDomain domain,
                             const bool fallback)
{
  return *attributes.lookup_or_default<bool>(name, domain, fallback);
}

inline void write_float3(bke::MutableAttributeAccessor attributes,
                         const StringRef name,
                         const bke::AttrDomain domain,
                         const Span<float3> values)
{
  bke::SpanAttributeWriter<float3> writer = attributes.lookup_or_add_for_write_span<float3>(
      name, domain);
  if (!writer) {
    return;
  }
  const int n = std::min(int(values.size()), int(writer.span.size()));
  for (int i = 0; i < n; i++) {
    writer.span[i] = values[i];
  }
  writer.finish();
}

inline void write_float(bke::MutableAttributeAccessor attributes,
                        const StringRef name,
                        const bke::AttrDomain domain,
                        const Span<float> values)
{
  bke::SpanAttributeWriter<float> writer = attributes.lookup_or_add_for_write_span<float>(name,
                                                                                          domain);
  if (!writer) {
    return;
  }
  const int n = std::min(int(values.size()), int(writer.span.size()));
  for (int i = 0; i < n; i++) {
    writer.span[i] = values[i];
  }
  writer.finish();
}

inline void collect_triangles(const Mesh &mesh, Vector<int> &r_tris)
{
  r_tris.clear();
  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    if (face.size() < 3) {
      continue;
    }
    const int v0 = corner_verts[face[0]];
    for (int k = 1; k + 1 < face.size(); k++) {
      r_tris.append(v0);
      r_tris.append(corner_verts[face[k]]);
      r_tris.append(corner_verts[face[k + 1]]);
    }
  }
}

inline void collect_collider_tris(const GeometrySet &geo,
                                  Vector<float3> &r_verts,
                                  Vector<int> &r_indices)
{
  r_verts.clear();
  r_indices.clear();

  auto add_mesh = [&](const Mesh &mesh, const float4x4 &tf) {
    const int base = r_verts.size();
    for (const float3 &p : mesh.vert_positions()) {
      r_verts.append(math::transform_point(tf, p));
    }
    Vector<int> local;
    collect_triangles(mesh, local);
    for (const int i : local) {
      r_indices.append(base + i);
    }
  };

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
        GeometrySet extracted;
        refs[handle].to_geometry_set(extracted);
        if (extracted.has_instances()) {
          geometry::RealizeInstancesOptions options;
          options.realize_instance_attributes = false;
          extracted = geometry::realize_instances(std::move(extracted), options).geometry;
        }
        if (const Mesh *mesh = extracted.get_mesh()) {
          add_mesh(*mesh, transforms[i]);
        }
      }
    }
  }
  if (const Mesh *mesh = geo.get_mesh()) {
    if (r_indices.is_empty()) {
      add_mesh(*mesh, float4x4::identity());
    }
  }
}

/** One AABB per connected mesh island. Box tanks become solid volumes, not thin shells. */
inline void collect_collider_aabbs(const GeometrySet &geo,
                                   Vector<float3> &r_min,
                                   Vector<float3> &r_max)
{
  r_min.clear();
  r_max.clear();

  auto add_mesh_islands = [&](const Mesh &mesh, const float4x4 &tf) {
    const Span<float3> positions = mesh.vert_positions();
    const Span<int2> edges = mesh.edges();
    if (positions.is_empty()) {
      return;
    }
    Array<int> parent(positions.size());
    for (const int i : positions.index_range()) {
      parent[i] = i;
    }
    auto find = [&](auto &&self, int i) -> int {
      if (parent[i] != i) {
        parent[i] = self(self, parent[i]);
      }
      return parent[i];
    };
    auto unite = [&](int a, int b) {
      a = find(find, a);
      b = find(find, b);
      if (a != b) {
        parent[b] = a;
      }
    };
    for (const int2 e : edges) {
      if (e[0] >= 0 && e[1] >= 0 && e[0] < positions.size() && e[1] < positions.size()) {
        unite(e[0], e[1]);
      }
    }
    Array<int> root_to_box(positions.size(), -1);
    for (const int i : positions.index_range()) {
      const int r = find(find, i);
      const float3 p = math::transform_point(tf, positions[i]);
      int box = root_to_box[r];
      if (box < 0) {
        box = r_min.size();
        root_to_box[r] = box;
        r_min.append(p);
        r_max.append(p);
      }
      else {
        r_min[box] = math::min(r_min[box], p);
        r_max[box] = math::max(r_max[box], p);
      }
    }
  };

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
        GeometrySet extracted;
        refs[handle].to_geometry_set(extracted);
        if (extracted.has_instances()) {
          geometry::RealizeInstancesOptions options;
          options.realize_instance_attributes = false;
          extracted = geometry::realize_instances(std::move(extracted), options).geometry;
        }
        if (const Mesh *mesh = extracted.get_mesh()) {
          add_mesh_islands(*mesh, transforms[i]);
        }
      }
    }
  }
  if (const Mesh *mesh = geo.get_mesh()) {
    if (r_min.is_empty()) {
      add_mesh_islands(*mesh, float4x4::identity());
    }
  }
}

inline float stiffness_to_compliance(const float stiffness)
{
  const float s = std::clamp(stiffness, 0.0f, 1.0f);
  if (s >= 0.999f) {
    return 0.0f;
  }
  if (s <= 0.0f) {
    return 0.05f;
  }
  return std::pow(10.0f, -1.0f - 4.0f * s);
}

inline void collect_cross_bends(const Mesh &mesh, Vector<int2> &r_bends)
{
  r_bends.clear();
  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  const Span<int> corner_edges = mesh.corner_edges();
  Array<int> opp_a(mesh.edges_num, -1);
  Array<int> opp_b(mesh.edges_num, -1);
  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    if (face.size() < 3) {
      continue;
    }
    const int n = int(face.size());
    for (int local = 0; local < n; local++) {
      const int e = corner_edges[face[local]];
      if (e < 0 || e >= mesh.edges_num) {
        continue;
      }
      const int opp = corner_verts[face[(local + 2) % n]];
      if (opp_a[e] < 0) {
        opp_a[e] = opp;
      }
      else if (opp_b[e] < 0 && opp != opp_a[e]) {
        opp_b[e] = opp;
      }
    }
  }
  for (const int e : IndexRange(mesh.edges_num)) {
    if (opp_a[e] >= 0 && opp_b[e] >= 0 && opp_a[e] != opp_b[e]) {
      r_bends.append(int2(opp_a[e], opp_b[e]));
    }
  }
}

/** Gravity comes from Set Gravity (attribute), never a solver default. */
inline float3 read_gravity(const bke::AttributeAccessor &attributes, const bke::AttrDomain domain)
{
  const VArray<float3> g = get_float3(attributes, "gravity", domain, float3(0.0f));
  return g.is_empty() ? float3(0.0f) : g[0];
}

inline float median_positive(Vector<float> values)
{
  Vector<float> kept;
  kept.reserve(values.size());
  for (const float v : values) {
    if (v > 1e-8f && std::isfinite(v)) {
      kept.append(v);
    }
  }
  if (kept.is_empty()) {
    return 0.0f;
  }
  std::sort(kept.begin(), kept.end());
  return kept[kept.size() / 2];
}

inline float median_edge_length(const Span<float3> positions, const Span<int2> edges)
{
  Vector<float> lens;
  lens.reserve(edges.size());
  for (const int2 e : edges) {
    if (e[0] < 0 || e[1] < 0 || e[0] >= positions.size() || e[1] >= positions.size()) {
      continue;
    }
    lens.append(math::distance(positions[e[0]], positions[e[1]]));
  }
  return median_positive(std::move(lens));
}

inline float median_point_spacing(const Span<float3> positions)
{
  if (positions.size() < 2) {
    return 0.1f;
  }
  const int n = int(positions.size());
  const int samples = std::min(n, 64);
  Vector<float> nearest;
  nearest.reserve(samples);
  const int stride = std::max(1, n / samples);
  for (int i = 0; i < n; i += stride) {
    float best = 1.0e9f;
    for (int j = 0; j < n; j++) {
      if (j == i) {
        continue;
      }
      const float d = math::distance(positions[i], positions[j]);
      if (d > 1e-8f && d < best) {
        best = d;
      }
    }
    if (best < 1.0e8f) {
      nearest.append(best);
    }
  }
  const float mid = median_positive(std::move(nearest));
  return mid > 1e-8f ? mid : 0.1f;
}

inline float clamp_cloth_thickness(const float thickness, const float median_rest)
{
  if (median_rest <= 1e-8f) {
    return std::max(thickness, 1e-4f);
  }
  return std::clamp(thickness, 1e-4f, 0.32f * median_rest);
}

/**
 * Compact cores are process-global and not thread-safe.
 *
 * Box2D / Box3D keep a static world table (`b2_worlds[]` / `b3_worlds[]`).
 * CreateWorld scans `inUse` with no lock. Geometry Nodes + the depsgraph can
 * evaluate several objects in parallel (TBB), so two Box2D hosts on Play
 * race the same slot and crash in b2CreateContact.
 *
 * Hold the matching mutex around CreateWorld / Step / DestroyWorld. Different
 * engines may run at the same time; two worlds of the same engine may not.
 */
inline std::mutex &box2d_engine_mutex()
{
  static std::mutex mutex;
  return mutex;
}
inline std::mutex &box3d_engine_mutex()
{
  static std::mutex mutex;
  return mutex;
}
inline std::mutex &jolt_engine_mutex()
{
  static std::mutex mutex;
  return mutex;
}

}  // namespace blender::nodes::deformable_solver
