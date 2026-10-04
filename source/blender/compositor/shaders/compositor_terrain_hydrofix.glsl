/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_hydrofix)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(texel, domain_size))) {
    return;
  }

  float4 f = texture_load(field_tx, texel);
  float h = f.r;
  float mn = h;
  mn = min(mn, texture_load(field_tx, texel + int2(-1, 0), f).r);
  mn = min(mn, texture_load(field_tx, texel + int2(1, 0), f).r);
  mn = min(mn, texture_load(field_tx, texel + int2(0, -1), f).r);
  mn = min(mn, texture_load(field_tx, texel + int2(0, 1), f).r);
  mn = min(mn, texture_load(field_tx, texel + int2(-1, -1), f).r);
  mn = min(mn, texture_load(field_tx, texel + int2(1, -1), f).r);
  mn = min(mn, texture_load(field_tx, texel + int2(-1, 1), f).r);
  mn = min(mn, texture_load(field_tx, texel + int2(1, 1), f).r);
  float drain = slope / float(max(domain_size.x, 1));
  if (h + 1.0e-6f < mn) {
    h = mix(h, mn - drain, clamp(amount, 0.0f, 1.0f));
  }
  else {
    h = mix(h, min(h, mn + drain * 4.0f), clamp(amount, 0.0f, 1.0f) * 0.15f);
  }
  imageStore(field_img, texel, float4(h, f.g, f.b, f.a));
}
