/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Scalar version of Pavel's advection shader for the Temperature extension. */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_advect_scalar)

#include "gpu_shader_compositor_texture_utilities.glsl"

float2 sample_velocity(sampler2D tx, float2 uv)
{
  int2 size = texture_size(tx);
  float2 p = clamp(uv * float2(size) - 0.5f, float2(0.0f), float2(size - int2(1)));
  int2 i0 = int2(floor(p));
  int2 i1 = min(i0 + int2(1), size - int2(1));
  float2 f = fract(p);
  float2 a = texture_load(tx, i0).xy;
  float2 b = texture_load(tx, int2(i1.x, i0.y)).xy;
  float2 c = texture_load(tx, int2(i0.x, i1.y)).xy;
  float2 d = texture_load(tx, i1).xy;
  return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

float sample_scalar(sampler2D tx, float2 uv)
{
  int2 size = texture_size(tx);
  float2 p = clamp(uv * float2(size) - 0.5f, float2(0.0f), float2(size - int2(1)));
  int2 i0 = int2(floor(p));
  int2 i1 = min(i0 + int2(1), size - int2(1));
  float2 f = fract(p);
  float a = texture_load(tx, i0).x;
  float b = texture_load(tx, int2(i1.x, i0.y)).x;
  float c = texture_load(tx, int2(i0.x, i1.y)).x;
  float d = texture_load(tx, i1).x;
  return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) { return; }
  float2 uv = (float2(texel) + 0.5f) / float2(size);
  int2 velocity_size = texture_size(velocity_tx);
  float2 velocity = sample_velocity(velocity_tx, uv);
  float2 coord = uv - timestep * velocity / float2(velocity_size);
  float value = sample_scalar(source_tx, coord) / (1.0f + dissipation * timestep);
  imageStore(output_img, texel, float4(value, 0.0f, 0.0f, 1.0f));
}
