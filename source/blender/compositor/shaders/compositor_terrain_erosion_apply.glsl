/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Integrate flux, erode/deposit, advect sediment, rain + evaporate. */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_erosion_apply)

#include "gpu_shader_compositor_texture_utilities.glsl"
#include "gpu_shader_compositor_terrain_lib.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(texel, domain_size))) {
    return;
  }

  float4 f = texture_load(field_tx, texel);
  float height = f.r;
  float water = max(f.g, 0.0f);
  float sediment = max(f.b, 0.0f);
  float wear = f.a;
  float mask = clamp(terrain_luma(texture_load(mask_tx, texel)), 0.0f, 1.0f);

  float4 flux = texture_load(flux_tx, texel);
  float4 flux_l = texture_load(flux_tx, texel + int2(-1, 0), float4(0.0f));
  float4 flux_r = texture_load(flux_tx, texel + int2(1, 0), float4(0.0f));
  float4 flux_d = texture_load(flux_tx, texel + int2(0, -1), float4(0.0f));
  float4 flux_u = texture_load(flux_tx, texel + int2(0, 1), float4(0.0f));

  /* Inflow: neighbor's outflow toward this cell. L outflow R, R outflow L, D outflow U, U outflow D. */
  float inflow = flux_l.y + flux_r.x + flux_d.w + flux_u.z;
  float outflow = flux.x + flux.y + flux.z + flux.w;
  float step = max(dt, 1.0e-4f);
  water = max(water + (inflow - outflow) * step, 0.0f);

  float2 vel = float2(flux_l.y - flux.x + flux.y - flux_r.x,
                      flux_d.w - flux.z + flux.w - flux_u.z);
  float speed = length(vel);

  float hl = texture_load(field_tx, texel + int2(-1, 0), f).r;
  float hr = texture_load(field_tx, texel + int2(1, 0), f).r;
  float hd = texture_load(field_tx, texel + int2(0, -1), f).r;
  float hu = texture_load(field_tx, texel + int2(0, 1), f).r;
  float2 grad = float2(hr - hl, hu - hd) * 0.5f;
  float slope = length(grad);

  float rain_mul = 1.0f;
  if (orographic > 1.0e-5f) {
    float2 ndir = normalize(rain_dir + float2(1.0e-5f));
    float windward = max(dot(normalize(grad + float2(1.0e-5f)), ndir), 0.0f);
    rain_mul = mix(1.0f, 1.0f + windward * 2.0f, orographic);
  }
  water += rain * rain_mul * mask * step;

  float cap = capacity * max(speed, 0.02f) * (0.15f + slope);
  float erode = solubility * max(cap - sediment, 0.0f) * mask;
  float deposit_amt = deposition * max(sediment - cap, 0.0f);
  float cut = downcutting * water * slope * mask;
  float dh = (deposit_amt - erode - cut) * step;
  height = max(height + dh, 0.0f);
  sediment = max(sediment - deposit_amt + erode, 0.0f);
  wear += (erode + cut) * step;

  /* Semi-Lagrangian sediment transport. */
  float2 uv = (float2(texel) + 0.5f - vel * step * 8.0f) / float2(domain_size);
  float sed_src = textureLod(field_tx, uv, 0.0f).b;
  sediment = mix(sediment, max(sed_src, 0.0f), 0.65f);

  water *= max(1.0f - evaporation * step, 0.0f);

  /* Light thermal mix inside hydraulic when thermal > 0 (Combined). */
  if (thermal > 1.0e-5f) {
    float mn = min(min(hl, hr), min(hd, hu));
    float mx = max(max(hl, hr), max(hd, hu));
    float rest = tan(radians(clamp(talus, 1.0f, 80.0f))) / float(max(domain_size.x, 1));
    if (height - mn > rest) {
      height = mix(height, height - (height - mn - rest) * 0.25f, thermal * mask);
    }
    else if (mx - height > rest * 0.25f && height < mx) {
      height = mix(height, height + (mx - height) * 0.05f, thermal * 0.35f * mask);
    }
  }

  imageStore(field_img, texel, float4(height, water, sediment, wear));
}
