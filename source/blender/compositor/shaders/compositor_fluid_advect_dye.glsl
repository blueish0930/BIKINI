/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Dye advection at cell centers using MAC face velocities.
 * Cell (i,j) velocity = average of surrounding faces.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_advect_dye)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  if (texel.x == 0 || texel.y == 0 || texel.x == size.x - 1 || texel.y == size.y - 1) {
    imageStore(output_img, texel, texture_load(source_tx, texel));
    return;
  }

  float uL = texture_load(u_tx, texel).x;
  float uR = texture_load(u_tx, texel + int2(1, 0)).x;
  float vB = texture_load(v_tx, texel).x;
  float vT = texture_load(v_tx, texel + int2(0, 1)).x;
  float2 vel = 0.5f * float2(uL + uR, vB + vT);

  float2 pos = float2(texel) + 0.5f - timestep * vel;
  pos = clamp(pos, float2(0.5f), float2(size) - float2(0.5f));

  float2 p0 = pos - 0.5f;
  int2 i0 = int2(floor(p0));
  float2 f = fract(p0);
  int2 i1 = min(i0 + int2(1), size - int2(1));
  i0 = clamp(i0, int2(0), size - int2(1));

  float4 a = texture_load(source_tx, i0);
  float4 b = texture_load(source_tx, int2(i1.x, i0.y));
  float4 c = texture_load(source_tx, int2(i0.x, i1.y));
  float4 d = texture_load(source_tx, i1);
  float4 s = mix(mix(a, b, f.x), mix(c, d, f.x), f.y);

  float decay = 1.0f + dissipation * timestep;
  imageStore(output_img, texel, s / decay);
}
