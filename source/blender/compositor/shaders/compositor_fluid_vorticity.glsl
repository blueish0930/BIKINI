/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Gems Ch.38 §38.5.1 — vorticity confinement force (optional extension).
 * ω = ∇ × u (2D scalar), N = ∇|ω| / |∇|ω||, f = ε (N × ω)
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_vorticity)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  float2 vC = texture_load(velocity_tx, texel).xy;
  if (curl_strength == 0.0f || texel.x == 0 || texel.y == 0 || texel.x == size.x - 1 ||
      texel.y == size.y - 1)
  {
    imageStore(output_img, texel, float4(vC, 0.0f, 1.0f));
    return;
  }

  int2 L = texel - int2(1, 0);
  int2 R = texel + int2(1, 0);
  int2 B = texel - int2(0, 1);
  int2 T = texel + int2(0, 1);

  float2 vL = texture_load(velocity_tx, L).xy;
  float2 vR = texture_load(velocity_tx, R).xy;
  float2 vB = texture_load(velocity_tx, B).xy;
  float2 vT = texture_load(velocity_tx, T).xy;

  /* ω = ∂v/∂x − ∂u/∂y with half differences (dx spacing). */
  float inv_2dx = 0.5f / max(dx, 1e-6f);
  float curlC = inv_2dx * ((vR.y - vL.y) - (vT.x - vB.x));

  float curlL = inv_2dx * ((vC.y - texture_load(velocity_tx, L - int2(1, 0)).y) -
                           (texture_load(velocity_tx, int2(L.x, T.y)).x -
                            texture_load(velocity_tx, int2(L.x, B.y)).x));
  float curlR = inv_2dx * ((texture_load(velocity_tx, R + int2(1, 0)).y - vC.y) -
                           (texture_load(velocity_tx, int2(R.x, T.y)).x -
                            texture_load(velocity_tx, int2(R.x, B.y)).x));
  float curlB = inv_2dx * ((texture_load(velocity_tx, int2(R.x, B.y)).y -
                            texture_load(velocity_tx, int2(L.x, B.y)).y) -
                           (vC.x - texture_load(velocity_tx, B - int2(0, 1)).x));
  float curlT = inv_2dx * ((texture_load(velocity_tx, int2(R.x, T.y)).y -
                            texture_load(velocity_tx, int2(L.x, T.y)).y) -
                           (texture_load(velocity_tx, T + int2(0, 1)).x - vC.x));

  /* Simplified: use abs(curl) gradient from neighbors of curlC. */
  float etaL = abs(inv_2dx * ((vC.y - vL.y) - (vT.x - vB.x)));
  float etaR = abs(inv_2dx * ((vR.y - vC.y) - (vT.x - vB.x)));
  float etaB = abs(inv_2dx * ((vR.y - vL.y) - (vC.x - vB.x)));
  float etaT = abs(inv_2dx * ((vR.y - vL.y) - (vT.x - vC.x)));
  float2 N = float2(etaR - etaL, etaT - etaB);
  float len = length(N) + 1e-5f;
  N /= len;

  /* f = ε (N × ω) in 2D: (N_y * ω, −N_x * ω) with sign convention. */
  float2 force = curl_strength * float2(N.y * curlC, -N.x * curlC);
  float2 out_v = vC + force * timestep;
  imageStore(output_img, texel, float4(out_v, 0.0f, 1.0f));
}
