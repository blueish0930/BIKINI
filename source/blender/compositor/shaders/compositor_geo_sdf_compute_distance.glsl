/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_geo_sdf_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_geo_sdf_compute_distance)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  const int2 texel = int2(gl_GlobalInvocationID.xy);

  const bool is_inside_mask = bool(texture_load(mask_tx, texel).x);
  const int2 closest_boundary_texel = texture_load(flooded_boundary_tx, texel).xy;

  float distance_to_boundary;
  if (closest_boundary_texel.x < 0) {
    /* No seed (empty or fully covered window). Use the window diagonal in world units. */
    const int2 size = texture_size(mask_tx);
    distance_to_boundary = length(float2(size) * pixel_size);
  }
  else {
    const float2 delta_world = (float2(closest_boundary_texel) - float2(texel)) * pixel_size;
    distance_to_boundary = length(delta_world);
  }

  float signed_distance = distance_to_boundary;
  if (signed_mode != 0 && is_inside_mask) {
    signed_distance = -signed_distance;
  }
  imageStore(distance_img, texel, float4(signed_distance));
}
