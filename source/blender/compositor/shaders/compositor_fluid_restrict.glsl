/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Full-weighting restriction: coarse(i,j) = avg of 2×2 fine block.
 * coarse size = imageSize(output); fine is ~2×.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_restrict)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 csize = imageSize(output_img);
  if (any(greaterThanEqual(texel, csize))) {
    return;
  }

  int2 fsize = texture_size(fine_tx);
  int2 i0 = min(texel * 2, fsize - int2(1));
  int2 i1 = min(i0 + int2(1), fsize - int2(1));

  float a = texture_load(fine_tx, i0).x;
  float b = texture_load(fine_tx, int2(i1.x, i0.y)).x;
  float c = texture_load(fine_tx, int2(i0.x, i1.y)).x;
  float d = texture_load(fine_tx, i1).x;
  float v = 0.25f * (a + b + c + d);
  imageStore(output_img, texel, float4(v, 0.0f, 0.0f, 1.0f));
}
