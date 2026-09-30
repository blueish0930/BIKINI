/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Domain edge BC for collocated velocity: hardcoded bounce (zero normal). */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_boundary)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) { return; }
  float2 vel = texture_load(velocity_tx, texel).xy;
  const int max_x = size.x - 1;
  const int max_y = size.y - 1;

  /* Bounce: zero normal component on the wall cell. */
  if (texel.x == 0)  { vel.x = 0.0f; }
  if (texel.x == max_x) { vel.x = 0.0f; }
  if (texel.y == 0) { vel.y = 0.0f; }
  if (texel.y == max_y) { vel.y = 0.0f; }

  imageStore(output_img, texel, float4(vel, 0.0f, 1.0f));
}
