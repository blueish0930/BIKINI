/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_combine)

#include "gpu_shader_compositor_texture_utilities.glsl"
#include "gpu_shader_compositor_terrain_lib.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(texel, domain_size))) {
    return;
  }
  float a = terrain_luma(texture_load(a_tx, texel));
  float b = terrain_luma(texture_load(b_tx, texel));
  float m = clamp(terrain_luma(texture_load(mask_tx, texel)), 0.0f, 1.0f) * factor;
  float r = a;
  if (combine_type == 0) {
    r = mix(a, b, m);
  }
  else if (combine_type == 1) {
    r = mix(a, a + b, m);
  }
  else if (combine_type == 2) {
    r = mix(a, a - b, m);
  }
  else if (combine_type == 3) {
    r = mix(a, a * b, m);
  }
  else if (combine_type == 4) {
    r = mix(a, max(a, b), m);
  }
  else if (combine_type == 5) {
    r = mix(a, min(a, b), m);
  }
  else if (combine_type == 6) {
    r = mix(a, 1.0f - (1.0f - a) * (1.0f - b), m);
  }
  else if (combine_type == 7) {
    float ov = (a < 0.5f) ? (2.0f * a * b) : (1.0f - 2.0f * (1.0f - a) * (1.0f - b));
    r = mix(a, ov, m);
  }
  else {
    r = mix(a, sqrt(a * a + b * b), m);
  }
  imageStore(height_img, texel, float4(r, r, r, 1.0f));
}
