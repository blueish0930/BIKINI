/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_terrain_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_terrain_primitive)

#include "gpu_shader_compositor_texture_utilities.glsl"
#include "gpu_shader_compositor_terrain_lib.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(texel, domain_size))) {
    return;
  }

  float2 p;
  if (vector_is_single != 0) {
    p.x = ((float(texel.x) + 0.5f) / float(domain_size.x)) * 2.0f - 1.0f;
    p.y = ((float(texel.y) + 0.5f) / float(domain_size.y)) * 2.0f - 1.0f;
  }
  else {
    p = texture_load(vector_tx, texel).xy;
  }

  float h = terrain_primitive_eval(
                p, primitive_type, scale, detail, roughness, distortion, lacunarity, seed, offset) *
            height;
  imageStore(height_img, texel, float4(h, h, h, 1.0f));
}
