/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "gpu_shader_math_rotation_conversion.bsl.hh"

[[node]]
void rotation_to_axis_angle(float4 rotation, float3 &axis, float &angle)
{
  /* Match Geometry Nodes, including both quaternion signs and the near-identity fallback. */
  const float sine = length(rotation.yzw);
  if (sine < 0.0005f) {
    axis = float3(0.0f, rotation.x < 0.0f ? -1.0f : 1.0f, 0.0f);
    angle = 0.0f;
  }
  else {
    axis = rotation.yzw / sine;
    angle = 2.0f * atan(sine, rotation.x);
  }
}
