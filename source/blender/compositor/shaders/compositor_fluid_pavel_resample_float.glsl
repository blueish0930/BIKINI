/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_resample_float)

#include "gpu_shader_compositor_texture_utilities.glsl"

float sample_bilinear_clamped(sampler2D tx, float2 uv)
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
  if (any(greaterThanEqual(texel, size))) {
    return;
  }
  float2 uv = (float2(texel) + 0.5f) / float2(size);
  imageStore(output_img, texel, float4(sample_bilinear_clamped(input_tx, uv) * value_scale, 0.0f, 0.0f, 1.0f));
}
