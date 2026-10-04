/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Gems Ch.38 Listing 38-4 — subtract pressure gradient from intermediate velocity.
 * uNew.xy = w.xy - halfdx * float2(pR-pL, pT-pB)
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_gradient)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  if (texel.x == 0 || texel.y == 0 || texel.x == size.x - 1 || texel.y == size.y - 1) {
    imageStore(output_img, texel, texture_load(velocity_tx, texel));
    return;
  }

  float pL = texture_load(pressure_tx, texel - int2(1, 0)).x;
  float pR = texture_load(pressure_tx, texel + int2(1, 0)).x;
  float pB = texture_load(pressure_tx, texel - int2(0, 1)).x;
  float pT = texture_load(pressure_tx, texel + int2(0, 1)).x;

  float4 uNew = texture_load(velocity_tx, texel);
  uNew.xy -= halfdx * float2(pR - pL, pT - pB);
  imageStore(output_img, texel, uNew);
}
