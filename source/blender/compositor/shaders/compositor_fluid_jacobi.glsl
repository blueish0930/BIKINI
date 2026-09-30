/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Gems Ch.38 Listing 38-2 — Jacobi iteration for Poisson equations (Eq. 16).
 * xNew = (xL + xR + xB + xT + alpha * bC) * rBeta
 *
 * Pressure:  alpha = -dx², rBeta = 1/4, x=p, b=div
 * Diffusion: alpha = dx²/(ν·dt), rBeta = 1/(4+alpha), x=u, b=u
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_jacobi)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  int2 L = texel - int2(1, 0);
  int2 R = texel + int2(1, 0);
  int2 B = texel - int2(0, 1);
  int2 T = texel + int2(0, 1);

  float4 xL = texture_load(x_tx, L);
  float4 xR = texture_load(x_tx, R);
  float4 xB = texture_load(x_tx, B);
  float4 xT = texture_load(x_tx, T);
  float4 bC = texture_load(b_tx, texel);

  float4 xNew = (xL + xR + xB + xT + alpha * bC) * rBeta;
  imageStore(output_img, texel, xNew);
}
