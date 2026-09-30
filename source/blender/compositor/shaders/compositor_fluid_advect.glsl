/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Gems Ch.38 Listing 38-1 — semi-Lagrangian advection (Eq. 13).
 * pos = coords - timestep * rdx * velocity; bilinear sample source.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_advect)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  /* Interior only; 1-cell perimeter reserved for boundary ghosts (Ch.38 §38.3.2). */
  if (texel.x == 0 || texel.y == 0 || texel.x == size.x - 1 || texel.y == size.y - 1) {
    imageStore(output_img, texel, texture_load(source_tx, texel));
    return;
  }

  float2 coords = float2(texel) + 0.5f;
  float2 vel = texture_load(velocity_tx, texel).xy;
  /* Listing 38-1: follow velocity field back in time. */
  float2 pos = coords - timestep * rdx * vel;

  /* Clamp to domain (texture CLAMP_TO_EDGE equivalent). */
  pos = clamp(pos, float2(0.5f), float2(size) - float2(0.5f));

  /* Manual bilinear (float textures may not auto-filter). */
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

  /* Optional dye/velocity dissipation (not in Harris listing; product feature). */
  float decay = 1.0f + dissipation * timestep;
  imageStore(output_img, texel, s / decay);
}
