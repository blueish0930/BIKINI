/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_modify)

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
  float2 uv = (float2(texel) + 0.5f) / float2(domain_size);
  float nse = terrain_fbm(uv * 10.0f + seed, 4, 2.0f, 0.5f);
  float guide = terrain_luma(texture_load(guide_tx, texel));
  float hl = terrain_luma(texture_load(height_tx, texel + int2(-1, 0), hc));
  float hr = terrain_luma(texture_load(height_tx, texel + int2(1, 0), hc));
  float hd = terrain_luma(texture_load(height_tx, texel + int2(0, -1), hc));
  float hu = terrain_luma(texture_load(height_tx, texel + int2(0, 1), hc));
  float2 grad = float2(hr - hl, hu - hd);
  float h_out = h;

  if (modify_type == 0) { /* Terrace */
    float st = max(steps, 2.0f);
    float j = (nse - 0.5f) * (1.0f - sharpness) * 0.15f;
    float t = floor((h + j) * st) / st;
    h_out = mix(h, t, amount);
  }
  else if (modify_type == 1) { /* Stratify */
    float layer = sin(h * max(steps, 2.0f) * 6.2831f + nse * 3.0f);
    h_out = h + layer * amount * 0.04f * (0.4f + length(grad) * 8.0f);
  }
  else if (modify_type == 2 || modify_type == 3) { /* Warp / SlopeWarp */
    float2 dir;
    if (modify_type == 3) {
      dir = normalize(grad + float2(1.0e-5f));
    }
    else {
      dir = float2(nse * 2.0f - 1.0f, terrain_fbm(uv * 10.0f + 4.2f + seed, 4, 2.0f, 0.5f) * 2.0f - 1.0f);
    }
    float2 src = uv + dir * amount * 0.12f * (0.35f + guide);
    h_out = terrain_luma(textureLod(height_tx, src, 0.0f));
  }
  else if (modify_type == 4) { /* Fold */
    float k = max(steps, 2.0f);
    float fold = abs(fract(h * k + nse * 0.2f) - 0.5f) * 2.0f;
    h_out = mix(h, mix(h, fold, 0.65f), amount);
  }
  else if (modify_type == 5) { /* Thermal shaper */
    float lap = (hl + hr + hd + hu) * 0.25f - h;
    h_out = h + lap * amount * 1.8f;
  }
  else if (modify_type == 6) { /* Steps */
    float st = max(steps, 2.0f);
    h_out = mix(h, floor(h * st + 0.0001f) / st, amount);
  }
  else if (modify_type == 7) { /* Sandstone */
    float band = sin((h + nse * 0.05f) * max(steps, 3.0f) * 12.566f);
    float hard = smoothstep(0.0f, 0.4f, abs(band));
    h_out = h + band * amount * 0.03f * hard;
  }
  else if (modify_type == 8) { /* Clamp */
    h_out = mix(h, clamp(h, min_h, max_h), amount);
  }
  else { /* Recurve */
    float lap = (hl + hr + hd + hu) * 0.25f - h;
    h_out = h - lap * amount * 2.2f;
  }

  imageStore(height_img, texel, float4(h_out, h_out, h_out, 1.0f));
}
