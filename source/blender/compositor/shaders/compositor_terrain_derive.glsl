/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_derive)

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
  float hl = terrain_luma(texture_load(height_tx, texel + int2(-1, 0), hc));
  float hr = terrain_luma(texture_load(height_tx, texel + int2(1, 0), hc));
  float hd = terrain_luma(texture_load(height_tx, texel + int2(0, -1), hc));
  float hu = terrain_luma(texture_load(height_tx, texel + int2(0, 1), hc));
  float2 grad = float2(hr - hl, hu - hd) * 0.5f * float(domain_size.x);
  float slope = length(grad);
  float lap = (hl + hr + hd + hu) * 0.25f - h;
  float v = 0.0f;

  if (derive_type == 0) { /* Slope */
    v = slope;
  }
  else if (derive_type == 1) { /* Curvature (convex positive) */
    v = max(-lap, 0.0f) * float(domain_size.x);
  }
  else if (derive_type == 2) { /* Flow (downhill magnitude) */
    v = max(h - min(min(hl, hr), min(hd, hu)), 0.0f) * float(domain_size.x);
  }
  else if (derive_type == 3) { /* Height range */
    v = h;
  }
  else if (derive_type == 4) { /* Peaks */
    v = max(-lap, 0.0f) * float(domain_size.x) * (0.4f + h);
  }
  else if (derive_type == 5) { /* Occlusion (cavity) */
    v = max(lap, 0.0f) * float(domain_size.x);
  }
  else if (derive_type == 6) { /* Angle / aspect */
    float ang = atan(grad.y, grad.x);
    float az = radians(azimuth);
    float diff = abs(atan(sin(ang - az), cos(ang - az)));
    v = 1.0f - smoothstep(0.0f, 1.2f, diff);
  }
  else { /* Soil: concave + low slope */
    float settle = 1.0f - smoothstep(0.0f, 1.5f, slope);
    v = max(lap, 0.0f) * float(domain_size.x) * settle;
  }

  float lo = min(min_v, max_v);
  float hi = max(min_v, max_v);
  float t = smoothstep(lo, hi + 1.0e-5f, v);
  if (softness > 1.0e-5f) {
    t = mix(t, smoothstep(lo - softness, hi + softness, v), 0.65f);
  }
  imageStore(mask_img, texel, float4(t, t, t, 1.0f));
}
