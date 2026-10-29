/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "gpu_shader_math_rotation_conversion.bsl.hh"
#include "gpu_shader_math_vector.bsl.hh"
#include "gpu_shader_math_vector_compare.bsl.hh"
#include "gpu_shader_math_vector_safe.bsl.hh"

float3 axes_to_rotation_orthogonal(float3 v)
{
  /* Match the deterministic fallback used by Geometry Nodes on the CPU. */
  if (v.x != -v.y) {
    return float3(-v.y, v.x, 0.0f);
  }
  if (v.x != -v.z) {
    return float3(-v.z, 0.0f, v.x);
  }
  return float3(0.0f, -v.z, v.y);
}

[[node]]
void axes_to_rotation(float3 primary_in,
                      float3 secondary_in,
                      float primary_idx_f,
                      float secondary_idx_f,
                      float tertiary_idx_f,
                      float tertiary_factor,
                      float4 &rotation)
{
  float3 primary = normalize_fallback(primary_in, float3(0.0f));
  float3 secondary = secondary_in;
  float3 tertiary;

  const bool primary_is_non_zero = !is_zero(primary);
  const bool secondary_is_non_zero = !is_zero(secondary);

  if (primary_is_non_zero && secondary_is_non_zero) {
    tertiary = cross(primary, secondary);
    /* GPU fused operations can leave a tiny cross product for parallel inputs. */
    if (dot(tertiary, tertiary) <= 1e-12f * dot(secondary, secondary)) {
      tertiary = axes_to_rotation_orthogonal(primary);
    }
    tertiary = normalize(tertiary);
    secondary = cross(tertiary, primary);
  }
  else if (primary_is_non_zero) {
    secondary = axes_to_rotation_orthogonal(primary);
    secondary = normalize(secondary);
    tertiary = cross(primary, secondary);
  }
  else if (secondary_is_non_zero) {
    secondary = normalize(secondary);
    primary = axes_to_rotation_orthogonal(secondary);
    primary = normalize(primary);
    tertiary = cross(primary, secondary);
  }
  else {
    rotation = float4(1.0f, 0.0f, 0.0f, 0.0f);
    return;
  }

  const int primary_axis = int(primary_idx_f);
  const int secondary_axis = int(secondary_idx_f);
  const int tertiary_axis = int(tertiary_idx_f);

  float3x3 mat;
  mat[primary_axis] = primary;
  mat[secondary_axis] = secondary;
  mat[tertiary_axis] = tertiary_factor * tertiary;

  rotation = to_quaternion(mat).as_float4();
}

[[node]]
void axes_to_rotation_identity(float3 /*primary_in*/, float3 /*secondary_in*/, float4 &rotation)
{
  rotation = float4(1.0f, 0.0f, 0.0f, 0.0f);
}
