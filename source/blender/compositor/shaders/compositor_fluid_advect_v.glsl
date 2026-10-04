/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Semi-Lagrangian advection of MAC v faces (texture size W×(H+1)).
 * Face position (i+0.5, j); velocity (u_avg, v) back-traces.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_advect_v)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 vsize = imageSize(output_img); /* W×(H+1) */
  if (any(greaterThanEqual(texel, vsize))) {
    return;
  }

  if (texel.y == 0 || texel.y == vsize.y - 1) {
    imageStore(output_img, texel, float4(0.0f));
    return;
  }

  int2 usize = texture_size(u_tx); /* (W+1)×H */
  float vv = texture_load(v_tx, texel).x;
  int2 u0 = clamp(int2(texel.x, texel.y - 1), int2(0), usize - int2(1));
  int2 u1 = clamp(int2(texel.x + 1, texel.y - 1), int2(0), usize - int2(1));
  int2 u2 = clamp(int2(texel.x, texel.y), int2(0), usize - int2(1));
  int2 u3 = clamp(int2(texel.x + 1, texel.y), int2(0), usize - int2(1));
  float uu = 0.25f * (texture_load(u_tx, u0).x + texture_load(u_tx, u1).x +
                      texture_load(u_tx, u2).x + texture_load(u_tx, u3).x);

  float2 pos = float2(texel) + float2(0.5f, 0.0f) - timestep * float2(uu, vv);
  pos.x = clamp(pos.x, 0.0f, float(vsize.x - 1));
  pos.y = clamp(pos.y, 0.0f, float(vsize.y - 1));

  float2 p0 = pos;
  int2 i0 = int2(floor(p0));
  float2 f = fract(p0);
  int2 i1 = min(i0 + int2(1), vsize - int2(1));
  i0 = clamp(i0, int2(0), vsize - int2(1));

  float a = texture_load(v_tx, i0).x;
  float b = texture_load(v_tx, int2(i1.x, i0.y)).x;
  float c = texture_load(v_tx, int2(i0.x, i1.y)).x;
  float d = texture_load(v_tx, i1).x;
  float s = mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
  s = clamp(s, -40.0f, 40.0f);
  imageStore(output_img, texel, float4(s, 0.0f, 0.0f, 1.0f));
}
