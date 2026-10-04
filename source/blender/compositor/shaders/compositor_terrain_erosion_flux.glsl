/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Mei-style pipe-model outflow flux. Field: R=height G=water B=sediment A=wear. */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_erosion_flux)

#include "gpu_shader_compositor_texture_utilities.glsl"

float flux_dir(float h, float water, float prev, float hn, float wn, float step, float g)
{
  float dh = (h + water) - (hn + wn);
  return max(0.0f, prev + step * g * dh);
}

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(texel, domain_size))) {
    return;
  }

  float4 f = texture_load(field_tx, texel);
  float water = max(f.g, 0.0f);
  float4 prev = texture_load(flux_tx, texel);
  float4 fl = texture_load(field_tx, texel + int2(-1, 0), f);
  float4 fr = texture_load(field_tx, texel + int2(1, 0), f);
  float4 fd = texture_load(field_tx, texel + int2(0, -1), f);
  float4 fu = texture_load(field_tx, texel + int2(0, 1), f);

  float g = max(gravity, 0.01f);
  float step = max(dt, 1.0e-4f);
  float ol = flux_dir(f.r, water, prev.x, fl.r, max(fl.g, 0.0f), step, g);
  float orr = flux_dir(f.r, water, prev.y, fr.r, max(fr.g, 0.0f), step, g);
  float od = flux_dir(f.r, water, prev.z, fd.r, max(fd.g, 0.0f), step, g);
  float ou = flux_dir(f.r, water, prev.w, fu.r, max(fu.g, 0.0f), step, g);
  float sum = ol + orr + od + ou;
  float scale = 1.0f;
  if (sum * step > water && sum > 1.0e-8f) {
    scale = water / (sum * step);
  }
  imageStore(flux_img, texel, float4(ol, orr, od, ou) * scale);
}
