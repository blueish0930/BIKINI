/* SPDX-FileCopyrightText: 2017 Pavel Dobryakov
 * SPDX-FileCopyrightText: 2026 Blender Authors (compute adaptation)
 *
 * SPDX-License-Identifier: MIT
 *
 * Velocity self-advection: same advectionShader with uSource = uVelocity.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_advect_velocity)

#include "gpu_shader_compositor_texture_utilities.glsl"
#include "webgl_fluid_pavel_common.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  float2 uv = pavel_uv(texel, size);
  float2 ts = pavel_texel_size(size);
  float2 velocity = pavel_bilerp(velocity_tx, uv, ts).xy;
  velocity = clamp(velocity, float2(-80.0f), float2(80.0f));
  float2 coord = uv - timestep * velocity * ts;
  float2 value = pavel_bilerp(velocity_tx, coord, ts).xy;
  float decay = 1.0f + dissipation * timestep;
  value = value / max(decay, 1e-4f);
  value = clamp(value, float2(-80.0f), float2(80.0f));
  if (any(notEqual(value, value))) {
    value = float2(0.0f);
  }
  imageStore(output_img, texel, float4(value, 0.0f, 1.0f));
}
