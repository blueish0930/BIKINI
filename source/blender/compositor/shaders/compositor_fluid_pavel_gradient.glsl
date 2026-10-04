/* SPDX-FileCopyrightText: 2017 Pavel Dobryakov
 * SPDX-FileCopyrightText: 2026 Blender Authors (compute adaptation)
 *
 * SPDX-License-Identifier: MIT
 *
 * From WebGL-Fluid-Simulation/script.js gradientSubtractShader.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_gradient)

#include "gpu_shader_compositor_texture_utilities.glsl"
#include "webgl_fluid_pavel_common.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) { return; }

  float2 uv = pavel_uv(texel, size);
  float2 ts = pavel_texel_size(size);
  float2 vL = uv - float2(ts.x, 0.0f);
  float2 vR = uv + float2(ts.x, 0.0f);
  float2 vT = uv + float2(0.0f, ts.y);
  float2 vB = uv - float2(0.0f, ts.y);

  float L = pavel_sample(pressure_tx, vL).x;
  float R = pavel_sample(pressure_tx, vR).x;
  float T = pavel_sample(pressure_tx, vT).x;
  float B = pavel_sample(pressure_tx, vB).x;
  float2 velocity = pavel_sample(velocity_tx, uv).xy;
  velocity.xy -= float2(R - L, T - B);
  imageStore(output_img, texel, float4(velocity, 0.0f, 1.0f));
}
