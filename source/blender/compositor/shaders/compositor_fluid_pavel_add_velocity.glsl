/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Add an external force field into collocated velocity:
 *   out = velocity + force * force_scale
 * Used every frame so Velocity socket acts like continuous mouse/splat force
 * (not only cold-start seed).
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_add_velocity)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }
  float2 vel = texture_load(velocity_tx, texel).xy;
  float2 force = texture_load(force_tx, texel).xy;
  vel += force * force_scale;
  vel = clamp(vel, float2(-80.0f), float2(80.0f));
  imageStore(output_img, texel, float4(vel, 0.0f, 1.0f));
}
