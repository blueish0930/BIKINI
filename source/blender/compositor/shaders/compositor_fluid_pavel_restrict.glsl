/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Full-weighting restriction for the pressure correction equation. */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_restrict)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 coarse_size = imageSize(output_img);
  if (any(greaterThanEqual(texel, coarse_size))) {
    return;
  }
  int2 fine_size = texture_size(fine_tx);
  int2 p0 = min(texel * 2, fine_size - int2(1));
  int2 p1 = min(p0 + int2(1), fine_size - int2(1));
  float value = 0.25f * (texture_load(fine_tx, p0).x +
                         texture_load(fine_tx, int2(p1.x, p0.y)).x +
                         texture_load(fine_tx, int2(p0.x, p1.y)).x +
                         texture_load(fine_tx, p1).x);
  /* The rediscretized coarse Laplacian has four times the fine-grid h² scale. */
  imageStore(output_img, texel, float4(value * rhs_scale, 0.0f, 0.0f, 1.0f));
}
