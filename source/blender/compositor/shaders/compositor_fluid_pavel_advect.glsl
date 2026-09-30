/* SPDX-FileCopyrightText: 2017 Pavel Dobryakov
 * SPDX-FileCopyrightText: 2026 Blender Authors (compute adaptation)
 *
 * SPDX-License-Identifier: MIT
 *
 * From WebGL-Fluid-Simulation/script.js advectionShader (MANUAL_FILTERING path).
 * Always uses bilerp so behaviour matches NEAREST FBO without hardware bilinear.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_advect)

#include "gpu_shader_compositor_texture_utilities.glsl"
#include "webgl_fluid_pavel_common.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 out_size = imageSize(output_img);
  if (any(greaterThanEqual(texel, out_size))) { return; }

  float2 uv = pavel_uv(texel, out_size);
  int2 vel_size = texture_size(velocity_tx);
  float2 vel_ts = pavel_texel_size(vel_size);
  int2 src_size = texture_size(source_tx);
  float2 dye_ts = pavel_texel_size(src_size);

  float2 velocity = pavel_bilerp(velocity_tx, uv, vel_ts).xy;
  velocity = clamp(velocity, float2(-80.0f), float2(80.0f));
  float2 coord = uv - timestep * velocity * vel_ts;

  float3 dye = pavel_bilerp(source_tx, coord, dye_ts).rgb;
  float decay = 1.0f + dissipation * timestep;
  dye = max(dye, float3(0.0f)) / max(decay, 1e-4f);
  dye = min(dye, float3(1.0f));
  if (any(notEqual(dye, dye))) { dye = float3(0.0f); }
  float dens = max(dye.r, max(dye.g, dye.b));
  float a = dens > 1e-4f ? 1.0f : 0.0f;
  imageStore(output_img, texel, float4(dye, a));
}
