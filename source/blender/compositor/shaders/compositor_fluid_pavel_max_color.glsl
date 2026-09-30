/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Per-channel max blend of dye with an external color source.
 * Lets Color socket keep "painting" into the fluid every frame.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_max_color)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }
  float4 a = texture_load(dye_tx, texel);
  float4 b = texture_load(source_tx, texel);
  float4 outv = max(a, b * source_scale);
  outv.a = clamp(max(outv.r, max(outv.g, outv.b)), 0.0f, 1.0f);
  imageStore(output_img, texel, outv);
}
