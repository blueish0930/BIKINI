/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Semi-Lagrangian advection of MAC u faces (texture size (W+1)×H).
 * Face position (i, j+0.5); velocity (u, v_avg) back-traces.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_advect_u)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 usize = imageSize(output_img); /* (W+1)×H */
  if (any(greaterThanEqual(texel, usize))) {
    return;
  }

  /* Domain walls fixed by BC pass. */
  if (texel.x == 0 || texel.x == usize.x - 1) {
    imageStore(output_img, texel, float4(0.0f));
    return;
  }

  int2 vsize = texture_size(v_tx); /* W×(H+1) */
  float u = texture_load(u_tx, texel).x;
  /* Average v to this vertical face. */
  int2 v0 = clamp(int2(texel.x - 1, texel.y), int2(0), vsize - int2(1));
  int2 v1 = clamp(int2(texel.x, texel.y), int2(0), vsize - int2(1));
  int2 v2 = clamp(int2(texel.x - 1, texel.y + 1), int2(0), vsize - int2(1));
  int2 v3 = clamp(int2(texel.x, texel.y + 1), int2(0), vsize - int2(1));
  float vv = 0.25f * (texture_load(v_tx, v0).x + texture_load(v_tx, v1).x +
                      texture_load(v_tx, v2).x + texture_load(v_tx, v3).x);

  float2 pos = float2(texel) + float2(0.0f, 0.5f) - timestep * float2(u, vv);
  pos.x = clamp(pos.x, 0.0f, float(usize.x - 1));
  pos.y = clamp(pos.y, 0.0f, float(usize.y - 1));

  float2 p0 = pos;
  int2 i0 = int2(floor(p0));
  float2 f = fract(p0);
  int2 i1 = min(i0 + int2(1), usize - int2(1));
  i0 = clamp(i0, int2(0), usize - int2(1));

  float a = texture_load(u_tx, i0).x;
  float b = texture_load(u_tx, int2(i1.x, i0.y)).x;
  float c = texture_load(u_tx, int2(i0.x, i1.y)).x;
  float d = texture_load(u_tx, i1).x;
  float s = mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
  s = clamp(s, -40.0f, 40.0f);
  imageStore(output_img, texel, float4(s, 0.0f, 0.0f, 1.0f));
}
