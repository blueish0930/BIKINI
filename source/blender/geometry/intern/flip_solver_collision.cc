/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "flip_solver_private.hh"

#include <algorithm>
#include <cmath>
#include <limits>

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"

#include "BLI_kdopbvh.hh"
#include "BLI_map.hh"
#include "BLI_math_geom_c.hh"
#include "BLI_math_vector.hh"

#include "DNA_mesh_types.h"

namespace blender::geometry {

namespace {

struct ColliderBvhData {
  Span<float3> positions;
  Span<int3> triangles;
};

static bool finite_float3(const float3 value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

static float length_squared(const float3 value)
{
  return value.x * value.x + value.y * value.y + value.z * value.z;
}

static void nearest_triangle_callback(void *userdata,
                                      const int index,
                                      const float co[3],
                                      BVHTreeNearest *nearest)
{
  const ColliderBvhData &data = *static_cast<const ColliderBvhData *>(userdata);
  const int3 tri = data.triangles[index];
  float closest[3];
  closest_on_tri_to_point_v3(
      closest, co, data.positions[tri.x], data.positions[tri.y], data.positions[tri.z]);
  const float3 delta = float3(closest) - float3(co);
  const float distance_sq = length_squared(delta);
  if (distance_sq >= nearest->dist_sq) {
    return;
  }
  nearest->index = index;
  nearest->dist_sq = distance_sq;
  copy_v3_v3(nearest->co, closest);
  normal_tri_v3(nearest->no, data.positions[tri.x], data.positions[tri.y], data.positions[tri.z]);
}

static int ray_intersection_count(const BVHTree &tree,
                                  const ColliderBvhData &data,
                                  const float3 origin,
                                  const float3 direction)
{
  int intersections = 0;
  BLI_bvhtree_ray_cast_all_cpp(
      tree,
      origin,
      direction,
      0.0f,
      BVH_RAYCAST_DIST_MAX,
      [&](const int index, const BVHTreeRay &ray, BVHTreeRayHit & /*hit*/) {
        const int3 tri = data.triangles[index];
        float distance;
        if (isect_ray_tri_epsilon_v3(ray.origin,
                                     ray.direction,
                                     data.positions[tri.x],
                                     data.positions[tri.y],
                                     data.positions[tri.z],
                                     &distance,
                                     nullptr,
                                     1.0e-7f) &&
            distance > 1.0e-6f)
        {
          intersections++;
        }
      });
  return intersections;
}

static void set_static_fallback(const Mesh &mesh, PreparedFlipCollider &r_prepared)
{
  r_prepared.previous_positions.as_mutable_span().copy_from(mesh.vert_positions());
  r_prepared.vertex_velocities.fill(float3(0.0f));
  r_prepared.max_speed = 0.0f;
  r_prepared.animated = false;
}

static bool topology_matches_by_index(const Mesh &current, const Mesh &previous)
{
  if (current.verts_num != previous.verts_num || current.faces_num != previous.faces_num ||
      current.corners_num != previous.corners_num ||
      current.corner_tris().size() != previous.corner_tris().size())
  {
    return false;
  }
  return std::equal(current.corner_verts().begin(),
                    current.corner_verts().end(),
                    previous.corner_verts().begin()) &&
         std::equal(current.corner_tris().begin(),
                    current.corner_tris().end(),
                    previous.corner_tris().begin());
}

static bool map_previous_positions_by_id(const Mesh &current,
                                         const Mesh &previous,
                                         MutableSpan<float3> r_previous_positions)
{
  const bke::AttributeAccessor current_attributes = current.attributes();
  const bke::AttributeAccessor previous_attributes = previous.attributes();
  const bke::AttributeReader<int> current_ids = current_attributes.lookup<int>(
      "id", bke::AttrDomain::Point);
  const bke::AttributeReader<int> previous_ids = previous_attributes.lookup<int>(
      "id", bke::AttrDomain::Point);
  if (!current_ids || !previous_ids || current_ids.varray.size() != current.verts_num ||
      previous_ids.varray.size() != previous.verts_num)
  {
    return false;
  }

  Map<int, int> previous_index_by_id;
  previous_index_by_id.reserve(previous.verts_num);
  for (const int i : IndexRange(previous.verts_num)) {
    const int id = previous_ids.varray[i];
    if (previous_index_by_id.contains(id)) {
      return false;
    }
    previous_index_by_id.add_new(id, i);
  }
  const Span<float3> previous_positions = previous.vert_positions();
  for (const int i : IndexRange(current.verts_num)) {
    const int *previous_index = previous_index_by_id.lookup_ptr(current_ids.varray[i]);
    if (previous_index == nullptr) {
      return false;
    }
    r_previous_positions[i] = previous_positions[*previous_index];
  }
  return true;
}

}  // namespace

FlipColliderQuery::~FlipColliderQuery()
{
  if (tree_ != nullptr) {
    BLI_bvhtree_free(tree_);
  }
}

bool FlipColliderQuery::build(const PreparedFlipCollider &prepared,
                              const float alpha,
                              std::string &r_error)
{
  if (tree_ != nullptr) {
    BLI_bvhtree_free(tree_);
    tree_ = nullptr;
  }
  prepared_ = &prepared;
  if (!prepared.enabled || prepared.triangles.is_empty()) {
    return true;
  }

  positions_.reinitialize(prepared.current_positions.size());
  const float clamped_alpha = std::clamp(alpha, 0.0f, 1.0f);
  for (const int i : positions_.index_range()) {
    positions_[i] = math::interpolate(
        prepared.previous_positions[i], prepared.current_positions[i], clamped_alpha);
  }

  tree_ = BLI_bvhtree_new(prepared.triangles.size(), 0.0f, 4, 6);
  if (tree_ == nullptr) {
    r_error = "Unable to allocate FLIP collider BVH";
    return false;
  }
  for (const int i : prepared.triangles.index_range()) {
    const int3 tri = prepared.triangles[i];
    const float3 points[3] = {positions_[tri.x], positions_[tri.y], positions_[tri.z]};
    BLI_bvhtree_insert(tree_, i, points[0], 3);
  }
  BLI_bvhtree_balance(tree_);

  bounds_min_ = positions_.first();
  bounds_max_ = positions_.first();
  for (const float3 position : positions_.as_span().drop_front(1)) {
    bounds_min_.x = std::min(bounds_min_.x, position.x);
    bounds_min_.y = std::min(bounds_min_.y, position.y);
    bounds_min_.z = std::min(bounds_min_.z, position.z);
    bounds_max_.x = std::max(bounds_max_.x, position.x);
    bounds_max_.y = std::max(bounds_max_.y, position.y);
    bounds_max_.z = std::max(bounds_max_.z, position.z);
  }
  bounds_min_ -= float3(prepared.thickness);
  bounds_max_ += float3(prepared.thickness);
  return true;
}

FlipColliderSample FlipColliderQuery::sample(const float3 &position) const
{
  FlipColliderSample result;
  if (tree_ == nullptr || prepared_ == nullptr) {
    return result;
  }

  ColliderBvhData data{positions_, prepared_->triangles};
  BVHTreeNearest nearest;
  nearest.index = -1;
  nearest.dist_sq = std::numeric_limits<float>::max();
  copy_v3_v3(nearest.co, position);
  BLI_bvhtree_find_nearest(tree_, position, &nearest, nearest_triangle_callback, &data);
  if (nearest.index < 0) {
    return result;
  }

  const float3 ray_direction = math::normalize(float3(1.0f, 0.0137f, 0.0079f));
  const bool inside = (ray_intersection_count(*tree_, data, position, ray_direction) & 1) != 0;
  const int3 tri = prepared_->triangles[nearest.index];
  float barycentric[3];
  float closest[3];
  closest_on_tri_to_point_v3(
      closest, barycentric, position, positions_[tri.x], positions_[tri.y], positions_[tri.z]);
  result.closest_position = float3(closest);
  const float unsigned_distance = std::sqrt(std::max(nearest.dist_sq, 0.0f));
  result.phi = (inside ? -unsigned_distance : unsigned_distance) - prepared_->thickness;

  float3 normal = inside ? result.closest_position - position : position - result.closest_position;
  if (length_squared(normal) > 1.0e-20f) {
    result.normal = math::normalize(normal);
  }
  else {
    normal_tri_v3(result.normal, positions_[tri.x], positions_[tri.y], positions_[tri.z]);
  }
  result.velocity = prepared_->vertex_velocities[tri.x] * barycentric[0] +
                    prepared_->vertex_velocities[tri.y] * barycentric[1] +
                    prepared_->vertex_velocities[tri.z] * barycentric[2];
  result.valid = true;
  return result;
}

bool FlipColliderQuery::is_built() const
{
  return tree_ != nullptr;
}

const float3 &FlipColliderQuery::bounds_min() const
{
  return bounds_min_;
}

const float3 &FlipColliderQuery::bounds_max() const
{
  return bounds_max_;
}

bool prepare_flip_collider(const FlipColliderInput &input,
                           const float frame_dt,
                           PreparedFlipCollider &r_prepared,
                           FlipSolverStats *stats,
                           std::string &r_error)
{
  r_prepared = PreparedFlipCollider();
  r_prepared.thickness = std::max(input.settings.thickness, 0.0f);
  r_prepared.friction = std::clamp(input.settings.friction, 0.0f, 1.0f);
  if (input.settings.mode == FlipCollisionMode::Disabled) {
    return true;
  }
  if (input.current_mesh == nullptr || input.current_mesh->verts_num == 0 ||
      input.current_mesh->faces_num == 0)
  {
    if (stats != nullptr) {
      stats->collider_missing = true;
    }
    return true;
  }

  const Mesh &current = *input.current_mesh;
  r_prepared.current_positions.reinitialize(current.verts_num);
  r_prepared.current_positions.as_mutable_span().copy_from(current.vert_positions());
  r_prepared.previous_positions.reinitialize(current.verts_num);
  r_prepared.previous_positions.as_mutable_span().copy_from(current.vert_positions());
  r_prepared.vertex_velocities.reinitialize(current.verts_num);
  r_prepared.vertex_velocities.fill(float3(0.0f));
  r_prepared.triangles.reinitialize(current.corner_tris().size());
  const Span<int> corner_verts = current.corner_verts();
  for (const int i : current.corner_tris().index_range()) {
    const int3 corners = current.corner_tris()[i];
    r_prepared.triangles[i] = int3(
        corner_verts[corners.x], corner_verts[corners.y], corner_verts[corners.z]);
  }
  for (const float3 position : r_prepared.current_positions) {
    if (!finite_float3(position)) {
      if (stats != nullptr) {
        stats->collider_non_finite = true;
      }
      r_error = "FLIP collider contains non-finite positions";
      return false;
    }
  }
  r_prepared.enabled = !r_prepared.triangles.is_empty();
  if (stats != nullptr) {
    stats->collider_triangles = r_prepared.triangles.size();
  }

  if (input.settings.mode == FlipCollisionMode::Animated && frame_dt > 0.0f) {
    if (input.settings.motion_source == FlipColliderMotionSource::PreviousGeometry) {
      if (input.previous_mesh == nullptr) {
        if (stats != nullptr) {
          stats->collider_previous_missing = true;
        }
        set_static_fallback(current, r_prepared);
      }
      else {
        const Mesh &previous = *input.previous_mesh;
        bool topology_matches = current.verts_num == previous.verts_num &&
                                current.corner_tris().size() == previous.corner_tris().size();
        if (topology_matches) {
          const bool mapped_by_id = map_previous_positions_by_id(
              current, previous, r_prepared.previous_positions);
          if (!mapped_by_id) {
            topology_matches = topology_matches_by_index(current, previous);
            if (topology_matches) {
              r_prepared.previous_positions.as_mutable_span().copy_from(previous.vert_positions());
            }
          }
        }
        if (!topology_matches) {
          if (stats != nullptr) {
            stats->collider_topology_mismatch = true;
          }
          set_static_fallback(current, r_prepared);
        }
        else {
          r_prepared.animated = true;
        }
      }
    }
    else {
      const bke::AttributeAccessor attributes = current.attributes();
      const std::optional<bke::AttributeMetaData> meta = attributes.lookup_meta_data(
          input.velocity_attribute);
      if (!meta) {
        if (stats != nullptr) {
          stats->collider_velocity_missing = true;
        }
        set_static_fallback(current, r_prepared);
      }
      else if (meta->domain != bke::AttrDomain::Point || meta->data_type != bke::AttrType::Float3)
      {
        if (stats != nullptr) {
          stats->collider_velocity_invalid_type = true;
        }
        set_static_fallback(current, r_prepared);
      }
      else {
        const VArray<float3> velocities = *attributes.lookup<float3>(input.velocity_attribute,
                                                                     bke::AttrDomain::Point);
        velocities.materialize(r_prepared.vertex_velocities.as_mutable_span());
        for (const int i : r_prepared.current_positions.index_range()) {
          r_prepared.previous_positions[i] = r_prepared.current_positions[i] -
                                             frame_dt * r_prepared.vertex_velocities[i];
        }
        r_prepared.animated = true;
      }
    }
  }

  if (r_prepared.animated &&
      input.settings.motion_source == FlipColliderMotionSource::PreviousGeometry)
  {
    for (const int i : r_prepared.current_positions.index_range()) {
      r_prepared.vertex_velocities[i] = (r_prepared.current_positions[i] -
                                         r_prepared.previous_positions[i]) /
                                        frame_dt;
    }
  }
  for (const float3 velocity : r_prepared.vertex_velocities) {
    if (!finite_float3(velocity)) {
      if (stats != nullptr) {
        stats->collider_non_finite = true;
      }
      r_error = "FLIP collider contains non-finite velocity";
      return false;
    }
    r_prepared.max_speed = std::max(r_prepared.max_speed, std::sqrt(length_squared(velocity)));
  }
  if (stats != nullptr) {
    stats->collider_max_speed = r_prepared.max_speed;
  }
  return true;
}

bool resolve_flip_particle_collision(const FlipColliderQuery &query,
                                     const float3 &old_position,
                                     float3 &position,
                                     float3 &velocity,
                                     const float margin,
                                     const float friction)
{
  if (!query.is_built()) {
    return false;
  }
  const float safe_margin = std::max(margin, 0.0f);
  const FlipColliderSample old_sample = query.sample(old_position);
  FlipColliderSample sample = query.sample(position);
  if (!sample.valid) {
    return false;
  }

  if (old_sample.valid && old_sample.phi > safe_margin && sample.phi < safe_margin) {
    float low = 0.0f;
    float high = 1.0f;
    for ([[maybe_unused]] const int iteration : IndexRange(8)) {
      const float middle = 0.5f * (low + high);
      const FlipColliderSample middle_sample = query.sample(
          math::interpolate(old_position, position, middle));
      if (middle_sample.valid && middle_sample.phi < safe_margin) {
        high = middle;
      }
      else {
        low = middle;
      }
    }
    position = math::interpolate(old_position, position, high);
    sample = query.sample(position);
  }
  if (sample.phi >= safe_margin) {
    return false;
  }

  position += (safe_margin - sample.phi + std::max(1.0e-6f, safe_margin * 0.01f)) * sample.normal;
  float3 relative_velocity = velocity - sample.velocity;
  const float inward_speed = math::dot(relative_velocity, sample.normal);
  if (inward_speed < 0.0f) {
    relative_velocity -= inward_speed * sample.normal;
  }
  const float normal_speed = std::max(math::dot(relative_velocity, sample.normal), 0.0f);
  const float3 normal_velocity = normal_speed * sample.normal;
  const float3 tangential_velocity = relative_velocity - normal_velocity;
  relative_velocity = normal_velocity + tangential_velocity * (1.0f - friction);
  velocity = sample.velocity + relative_velocity;
  return true;
}

}  // namespace blender::geometry
