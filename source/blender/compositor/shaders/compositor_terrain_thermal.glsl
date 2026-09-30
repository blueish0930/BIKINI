/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_thermal)

#include "gpu_shader_compositor_texture_utilities.glsl"
#include "gpu_shader_compositor_terrain_lib.glsl"

float thermal_delta(float h, float hn, float lim)
{
  float delta = 0.0f;
  float dh_to = h - hn;
  if (dh_to > lim) {
    delta -= (dh_to - lim) * 0.12f;
  }
  float dh_from = hn - h;
  if (dh_from > lim) {
    delta += (dh_from - lim) * 0.12f;
  }
  return delta;
}

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(texel, domain_size))) {
    return;
  }

  float4 f = texture_load(field_tx, texel);
  float h = f.r;
  float mask = clamp(terrain_luma(texture_load(mask_tx, texel)), 0.0f, 1.0f);
  float rest = tan(radians(clamp(talus, 1.0f, 85.0f))) / float(max(domain_size.x, 1));
  float delta = 0.0f;
  delta += thermal_delta(h, texture_load(field_tx, texel + int2(-1, 0), f).r, rest);
  delta += thermal_delta(h, texture_load(field_tx, texel + int2(1, 0), f).r, rest);
  delta += thermal_delta(h, texture_load(field_tx, texel + int2(0, -1), f).r, rest);
  delta += thermal_delta(h, texture_load(field_tx, texel + int2(0, 1), f).r, rest);
  if (corners != 0) {
    float limd = rest * 1.4142135f;
    delta += thermal_delta(h, texture_load(field_tx, texel + int2(-1, -1), f).r, limd);
    delta += thermal_delta(h, texture_load(field_tx, texel + int2(1, -1), f).r, limd);
    delta += thermal_delta(h, texture_load(field_tx, texel + int2(-1, 1), f).r, limd);
    delta += thermal_delta(h, texture_load(field_tx, texel + int2(1, 1), f).r, limd);
  }
  h = max(h + delta * strength * mask, 0.0f);
  imageStore(field_img, texel, float4(h, f.g, f.b, f.a));
}
