/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * MAC gradient subtract (Δx=1).
 * component=0: u faces (W+1)×H: u(i,j) -= p(i,j) - p(i-1,j)  (i=1..W-1)
 * component=1: v faces W×(H+1): v(i,j) -= p(i,j) - p(i,j-1)  (j=1..H-1)
 * Domain walls leave edge faces unchanged (BC pass zeros them).
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_gradient_mac)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  float vel = texture_load(vel_tx, texel).x;
  int2 psize = texture_size(pressure_tx);

  if (component == 0) {
    /* u faces: i ∈ [0, W], j ∈ [0, H) */
    if (texel.x == 0 || texel.x == size.x - 1) {
      imageStore(output_img, texel, float4(vel, 0.0f, 0.0f, 1.0f));
      return;
    }
    float pR = texture_load(pressure_tx, clamp(texel, int2(0), psize - int2(1))).x;
    float pL = texture_load(pressure_tx, clamp(texel - int2(1, 0), int2(0), psize - int2(1))).x;
    vel -= (pR - pL);
  }
  else {
    /* v faces: i ∈ [0, W), j ∈ [0, H] */
    if (texel.y == 0 || texel.y == size.y - 1) {
      imageStore(output_img, texel, float4(vel, 0.0f, 0.0f, 1.0f));
      return;
    }
    float pT = texture_load(pressure_tx, clamp(texel, int2(0), psize - int2(1))).x;
    float pB = texture_load(pressure_tx, clamp(texel - int2(0, 1), int2(0), psize - int2(1))).x;
    vel -= (pT - pB);
  }

  vel = clamp(vel, -40.0f, 40.0f);
  imageStore(output_img, texel, float4(vel, 0.0f, 0.0f, 1.0f));
}
