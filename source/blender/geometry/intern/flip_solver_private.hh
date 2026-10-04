/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <cstdint>
#include <string>

#include "BLI_array.hh"
#include "BLI_math_vector_types.hh"

#include "GEO_flip_solver.hh"

namespace blender {
struct BVHTree;
}

namespace blender::geometry {

struct PreparedFlipCollider {
  Array<float3> previous_positions;
  Array<float3> current_positions;
  Array<float3> vertex_velocities;
  Array<int3> triangles;

  float3 bounds_min = float3(0.0f);
  float3 bounds_max = float3(0.0f);
  float max_speed = 0.0f;
  float thickness = 0.0f;
  float friction = 0.0f;
  bool enabled = false;
  bool animated = false;
};

struct FlipColliderSample {
  float phi = 1.0e30f;
  float3 normal = float3(0.0f, 0.0f, 1.0f);
  float3 velocity = float3(0.0f);
  float3 closest_position = float3(0.0f);
  bool valid = false;
};

class FlipColliderQuery {
 private:
  BVHTree *tree_ = nullptr;
  const PreparedFlipCollider *prepared_ = nullptr;
  Array<float3> positions_;
  float3 bounds_min_ = float3(0.0f);
  float3 bounds_max_ = float3(0.0f);

 public:
  FlipColliderQuery() = default;
  FlipColliderQuery(const FlipColliderQuery &) = delete;
  FlipColliderQuery &operator=(const FlipColliderQuery &) = delete;
  ~FlipColliderQuery();

  bool build(const PreparedFlipCollider &prepared, float alpha, std::string &r_error);
  FlipColliderSample sample(const float3 &position) const;

  bool is_built() const;
  const float3 &bounds_min() const;
  const float3 &bounds_max() const;
  uint64_t cache_key() const;
};

bool prepare_flip_collider(const FlipColliderInput &input,
                           float frame_dt,
                           PreparedFlipCollider &r_prepared,
                           FlipSolverStats *stats,
                           std::string &r_error);

bool resolve_flip_particle_collision(const FlipColliderQuery &query,
                                     const float3 &old_position,
                                     float3 &position,
                                     float3 &velocity,
                                     float margin,
                                     float friction);

}  // namespace blender::geometry
