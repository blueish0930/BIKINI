/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Add a bilinearly prolonged pressure correction to the fine level.
 *
 * Coordinate convention matches full-weighting restriction (texel*2):
 *   coarse cell i covers fine cells (2i, 2i+1).
 * Continuous coarse sample for fine texel t:
 *   p = (t + 0.5) * 0.5 - 0.5 = t/2 - 0.25
 *
 * Using domain UV * coarse_size is WRONG when fine size is odd or
 * 2*coarse != fine (orphan right/top columns) — that stretches the
 * correction and corrupts the high-index half of the domain.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_prolong)

#include "gpu_shader_compositor_texture_utilities.glsl"

float sample_bilinear_index(sampler2D tx, float2 p)
{
  int2 size = texture_size(tx);
  float2 pc = clamp(p, float2(0.0f), float2(size - int2(1)));
  int2 i0 = int2(floor(pc));
  int2 i1 = min(i0 + int2(1), size - int2(1));
  float2 f = pc - float2(i0);
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
  /* Half-index mapping aligned with restrict's fine(texel*2). */
  float2 p = (float2(texel) + 0.5f) * 0.5f - 0.5f;
  float value = texture_load(fine_tx, texel).x + sample_bilinear_index(coarse_tx, p);
  imageStore(output_img, texel, float4(value, 0.0f, 0.0f, 1.0f));
}
