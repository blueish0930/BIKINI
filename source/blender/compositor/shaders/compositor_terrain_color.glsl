/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_color)

#include "gpu_shader_compositor_texture_utilities.glsl"
#include "gpu_shader_compositor_terrain_lib.glsl"

float3 terrain_palette(float h, float slope, float wet)
{
  float3 water = water_color.rgb;
  float3 sand = sand_color.rgb;
  float3 grass = grass_color.rgb;
  float3 rock = rock_color.rgb;
  float3 snow = snow_color.rgb;
  float3 c = mix(sand, grass, smoothstep(sea_level, sea_level + 0.18f, h));
  c = mix(c, rock, smoothstep(0.35f, 0.72f, h) * smoothstep(0.15f, 1.2f, slope));
  c = mix(c, snow, smoothstep(snow_level - 0.08f, snow_level + 0.05f, h) *
                       (1.0f - smoothstep(0.8f, 2.2f, slope)));
  if (h < sea_level) {
    c = mix(water, sand, smoothstep(sea_level - 0.08f, sea_level, h));
  }
  c = mix(c, water * 0.65f + c * 0.35f, wet * 0.35f);
  return c;
}

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(texel, domain_size))) {
    return;
  }

  float4 hc = texture_load(height_tx, texel);
  float h = terrain_luma(hc);
  float hl = terrain_luma(texture_load(height_tx, texel + int2(-1, 0), hc));
  float hr = terrain_luma(texture_load(height_tx, texel + int2(1, 0), hc));
  float hd = terrain_luma(texture_load(height_tx, texel + int2(0, -1), hc));
  float hu = terrain_luma(texture_load(height_tx, texel + int2(0, 1), hc));
  float slope = length(float2(hr - hl, hu - hd)) * float(domain_size.x) * 0.5f;
  float wet = texture_load(flow_tx, texel).b;
  float2 uv = (float2(texel) + 0.5f) / float2(domain_size);
  float nse = terrain_fbm(uv * 12.0f, 4, 2.0f, 0.5f);

  float3 c = terrain_palette(h, slope, wet);
  if (color_type == 1) { /* Weathering */
    float dirt = (1.0f - smoothstep(0.0f, 1.1f, slope)) * (0.5f + nse * 0.5f);
    c = mix(c, c * float3(0.55f, 0.42f, 0.28f), dirt * weathering);
  }
  else if (color_type == 2) { /* SuperColor: extra flow/peak variation */
    float peaks = max(h - (hl + hr + hd + hu) * 0.25f, 0.0f) * 12.0f;
    c = mix(c, rock_color.rgb, clamp(peaks, 0.0f, 1.0f) * 0.45f);
    c = mix(c, sand_color.rgb, clamp(wet, 0.0f, 1.0f) * 0.25f);
    c *= 0.85f + nse * 0.3f;
  }
  else if (color_type == 3) { /* WaterColor */
    float water_m = 1.0f - smoothstep(sea_level - 0.02f, sea_level + 0.01f, h);
    float deep = 1.0f - smoothstep(0.0f, sea_level, h);
    c = mix(hc.rgb, mix(water_color.rgb, water_color.rgb * 0.35f, deep), water_m);
  }

  imageStore(color_img, texel, float4(c, 1.0f));
}
