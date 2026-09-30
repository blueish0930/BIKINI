/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Gems Ch.38 Listing 38-3 — divergence of velocity field.
 * div = halfdx * ((wR.x - wL.x) + (wT.y - wB.y))
 * halfdx = 0.5 / dx
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_divergence)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  if (texel.x == 0 || texel.y == 0 || texel.x == size.x - 1 || texel.y == size.y - 1) {
    imageStore(output_img, texel, float4(0.0f));
    return;
  }

  int2 L = texel - int2(1, 0);
  int2 R = texel + int2(1, 0);
  int2 B = texel - int2(0, 1);
  int2 T = texel + int2(0, 1);

  float2 wL = texture_load(velocity_tx, L).xy;
  float2 wR = texture_load(velocity_tx, R).xy;
  float2 wB = texture_load(velocity_tx, B).xy;
  float2 wT = texture_load(velocity_tx, T).xy;

  float div = halfdx * ((wR.x - wL.x) + (wT.y - wB.y));
  imageStore(output_img, texel, float4(div, 0.0f, 0.0f, 1.0f));
}
