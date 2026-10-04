/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_outputs)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(texel, domain_size))) {
    return;
  }
  float4 f = texture_load(field_tx, texel);
  float4 flux = texture_load(flux_tx, texel);
  float2 vel = float2(flux.y - flux.x, flux.w - flux.z);
  float mag = length(vel);
  float h = f.r;
  imageStore(height_img, texel, float4(h, h, h, 1.0f));
  imageStore(flow_img, texel, float4(vel.x * 0.5f + 0.5f, vel.y * 0.5f + 0.5f, mag, 1.0f));
  imageStore(sediment_img, texel, float4(f.b, f.b, f.b, 1.0f));
  imageStore(wear_img, texel, float4(f.a, f.a, f.a, 1.0f));
  imageStore(water_img, texel, float4(f.g, f.g, f.g, 1.0f));
}
