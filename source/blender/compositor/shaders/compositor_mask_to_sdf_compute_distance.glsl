/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_mask_to_sdf_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_mask_to_sdf_compute_distance)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  const int2 texel = int2(gl_GlobalInvocationID.xy);
  const int2 size = texture_size(mask_tx);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  const bool is_inside_mask = bool(texture_load(mask_tx, texel).x);
  const int2 closest_boundary_texel = texture_load(flooded_boundary_tx, texel).xy;

  /* Unit square [0,1]²: one texel is (1/width, 1/height), image diagonal is sqrt(2). */
  const float2 uv_pixel = float2(1.0f / float(max(size.x, 1)), 1.0f / float(max(size.y, 1)));
  float distance_to_boundary;
  if (closest_boundary_texel.x < 0) {
    distance_to_boundary = sqrt(2.0f);
  }
  else {
    distance_to_boundary = length((float2(closest_boundary_texel) - float2(texel)) * uv_pixel);
  }
  const float signed_distance = is_inside_mask ? -distance_to_boundary : distance_to_boundary;

  imageStore(distance_img, texel, float4(signed_distance));
}
