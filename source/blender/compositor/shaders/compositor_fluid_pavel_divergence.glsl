/* SPDX-FileCopyrightText: 2017 Pavel Dobryakov
 * SPDX-FileCopyrightText: 2026 Blender Authors (compute adaptation)
 *
 * SPDX-License-Identifier: MIT
 *
 * From WebGL-Fluid-Simulation/script.js divergenceShader (verbatim equations).
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_divergence)

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

  float L = pavel_sample(velocity_tx, vL).x;
  float R = pavel_sample(velocity_tx, vR).x;
  float T = pavel_sample(velocity_tx, vT).y;
  float B = pavel_sample(velocity_tx, vB).y;
  float2 C = pavel_sample(velocity_tx, uv).xy;

  if (vL.x < 0.0f) { L = -C.x; }
  if (vR.x > 1.0f) { R = -C.x; }
  if (vT.y > 1.0f) { T = -C.y; }
  if (vB.y < 0.0f) { B = -C.y; }

  float div = 0.5f * (R - L + T - B);
  imageStore(output_img, texel, float4(div, 0.0f, 0.0f, 1.0f));
}
