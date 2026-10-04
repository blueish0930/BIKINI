/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Solid cells get collider velocity; liquid/air keep simulated velocity. */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_apply_obstacle)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }
  float solid = texture_load(solid_tx, texel).x;
  float2 vel = texture_load(velocity_tx, texel).xy;
  if (solid > 0.5f) {
    vel = texture_load(collider_vel_tx, texel).xy;
  }
  imageStore(output_img, texel, float4(vel, 0.0f, 1.0f));
}
