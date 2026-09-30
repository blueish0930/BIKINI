/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Geometric Multigrid residual for Poisson pressure (dx = 1).
 * Discrete equation: (xL+xR+xB+xT - 4*xC) = bC
 * residual r = b - A x = bC - (xL+xR+xB+xT - 4*xC)
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_residual)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  float xC = texture_load(x_tx, texel).x;
  float xL = texture_load(x_tx, texel - int2(1, 0)).x;
  float xR = texture_load(x_tx, texel + int2(1, 0)).x;
  float xB = texture_load(x_tx, texel - int2(0, 1)).x;
  float xT = texture_load(x_tx, texel + int2(0, 1)).x;
  float bC = texture_load(b_tx, texel).x;

  float Ax = (xL + xR + xB + xT - 4.0f * xC);
  float r = bC - Ax;
  imageStore(output_img, texel, float4(r, 0.0f, 0.0f, 1.0f));
}
