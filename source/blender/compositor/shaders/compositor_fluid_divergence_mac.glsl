/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * MAC divergence on cell centers (Δx=1):
 *   div(i,j) = u(i+1,j) - u(i,j) + v(i,j+1) - v(i,j)
 * u: (W+1)×H, v: W×(H+1), output: W×H
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_divergence_mac)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img); /* W×H cells */
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  if (texel.x == 0 || texel.y == 0 || texel.x == size.x - 1 || texel.y == size.y - 1) {
    imageStore(output_img, texel, float4(0.0f));
    return;
  }

  /* u face left of cell (i,j) is u_tx[i,j]; right is u_tx[i+1,j]. */
  float uL = texture_load(u_tx, texel).x;
  float uR = texture_load(u_tx, texel + int2(1, 0)).x;
  float vB = texture_load(v_tx, texel).x;
  float vT = texture_load(v_tx, texel + int2(0, 1)).x;

  float div = (uR - uL) + (vT - vB);
  imageStore(output_img, texel, float4(div, 0.0f, 0.0f, 1.0f));
}
