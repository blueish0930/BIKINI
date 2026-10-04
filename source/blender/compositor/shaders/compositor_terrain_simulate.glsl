/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_simulate)

#include "gpu_shader_compositor_texture_utilities.glsl"
#include "gpu_shader_compositor_terrain_lib.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(texel, domain_size))) {
    return;
  }

  float4 hc = texture_load(height_tx, texel);
  float h = terrain_luma(hc);
  float mask_in = clamp(terrain_luma(texture_load(mask_tx, texel)), 0.0f, 1.0f);
  float hl = terrain_luma(texture_load(height_tx, texel + int2(-1, 0), hc));
  float hr = terrain_luma(texture_load(height_tx, texel + int2(1, 0), hc));
  float hd = terrain_luma(texture_load(height_tx, texel + int2(0, -1), hc));
  float hu = terrain_luma(texture_load(height_tx, texel + int2(0, 1), hc));
  float2 grad = float2(hr - hl, hu - hd) * 0.5f;
  float slope = length(grad) * float(domain_size.x) * 0.5f;
  float2 uv = (float2(texel) + 0.5f) / float2(domain_size);
  float nse = terrain_fbm(uv * 8.0f + seed, 4, 2.0f, 0.5f);

  float h_out = h;
  float mask_out = 0.0f;
  float str = strength * mask_in;

  if (sim_type == 0 || sim_type == 1 || sim_type == 2) {
    /* Snow / Snowfield / Dusting. */
    float alt = smoothstep(height_min, height_max, h);
    float slope_f = 1.0f - smoothstep(slope_max * 0.35f, max(slope_max, 0.001f), slope);
    float cover = coverage * alt * slope_f * (0.65f + nse * 0.35f);
    cover *= 1.0f - melt * (1.0f - alt);
    if (sim_type == 2) {
      cover *= 0.35f;
    }
    if (sim_type == 1) {
      cover = mix(cover, coverage * alt, 0.55f);
    }
    mask_out = clamp(cover, 0.0f, 1.0f);
    h_out = h + mask_out * str * 0.08f;
  }
  else if (sim_type == 3) {
    /* Rivers: carve along local downhill. */
    float mn = min(min(hl, hr), min(hd, hu));
    float carve = max(h - mn, 0.0f);
    float flow = pow(clamp(carve * 6.0f + (1.0f - nse) * 0.15f, 0.0f, 1.0f), 1.4f);
    h_out = h - flow * str * 0.04f;
    mask_out = flow;
  }
  else if (sim_type == 4) {
    /* Sea + coastal. */
    float water = 1.0f - smoothstep(sea_level, sea_level + 0.02f, h);
    float beach = smoothstep(sea_level - 0.04f, sea_level, h) *
                  (1.0f - smoothstep(sea_level, sea_level + 0.06f, h));
    if (h < sea_level) {
      h_out = mix(h, sea_level - 0.004f, str * 0.35f);
    }
    else {
      float cliff = smoothstep(sea_level, sea_level + 0.08f, h) * str * 0.02f;
      h_out = h - beach * str * 0.015f - cliff * max(slope, 0.0f) * 0.02f;
    }
    mask_out = water;
  }
  else if (sim_type == 5) {
    /* Lake: fill local basins toward sea_level. */
    float mn = min(min(hl, hr), min(hd, hu));
    if (h < sea_level && h <= mn + 1.0e-4f) {
      h_out = mix(h, min(sea_level, mn + 0.002f), str);
      mask_out = 1.0f;
    }
    else if (h < sea_level) {
      mask_out = smoothstep(sea_level, sea_level - 0.05f, h);
    }
  }
  else if (sim_type == 6) {
    /* Sediments in concavities. */
    float lap = (hl + hr + hd + hu) * 0.25f - h;
    float fill = max(-lap, 0.0f);
    float settle = (1.0f - smoothstep(0.0f, slope_max, slope));
    mask_out = clamp(fill * 8.0f * settle, 0.0f, 1.0f);
    h_out = h + mask_out * str * 0.06f;
  }
  else if (sim_type == 7) {
    /* Crumble from edges / high curvature. */
    float lap = h - (hl + hr + hd + hu) * 0.25f;
    float edge = max(lap, 0.0f);
    h_out = h - edge * str * 0.35f;
    mask_out = clamp(edge * 6.0f, 0.0f, 1.0f);
  }
  else if (sim_type == 8) {
    /* Glacier: high-altitude flow downhill. */
    float alt = smoothstep(height_min, height_max, h);
    float flow = max(slope, 0.0f) * alt;
    float2 dir = -normalize(grad + float2(1.0e-5f));
    float2 src_uv = uv - dir * 0.01f * str;
    float src = terrain_luma(textureLod(height_tx, src_uv, 0.0f));
    h_out = mix(h, max(h - flow * str * 0.02f, src * 0.98f), alt);
    mask_out = alt * flow;
  }
  else {
    /* Anastomosis: branching extraction. */
    float v = terrain_voronoi(uv * 6.0f + seed);
    float branch = pow(1.0f - clamp(v * 1.8f, 0.0f, 1.0f), 2.2f);
    float follow = 1.0f - smoothstep(0.0f, slope_max, slope);
    mask_out = branch * mix(0.4f, 1.0f, follow);
    h_out = h - mask_out * str * 0.05f;
  }

  imageStore(height_img, texel, float4(h_out, h_out, h_out, 1.0f));
  imageStore(mask_img, texel, float4(mask_out, mask_out, mask_out, 1.0f));
}
