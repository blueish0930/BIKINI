/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Gems Ch.38 Listing 38-5 — boundary ghost cells.
 * bv = scale * sample(interior neighbor)
 * scale = -1 → no-slip velocity (Eq. 17)
 * scale = +1 → pure Neumann pressure (Eq. 18)
 *
 * Interior cells are copied through unchanged.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_boundary)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  const bool left = texel.x == 0;
  const bool right = texel.x == size.x - 1;
  const bool bottom = texel.y == 0;
  const bool top = texel.y == size.y - 1;

  if (!left && !right && !bottom && !top) {
    imageStore(output_img, texel, texture_load(field_tx, texel));
    return;
  }

  /* Offset toward interior (Listing 38-5 / Fig 38-6). */
  int2 offset = int2(0);
  if (left) {
    offset.x = 1;
  }
  else if (right) {
    offset.x = -1;
  }
  if (bottom) {
    offset.y = 1;
  }
  else if (top) {
    offset.y = -1;
  }

  /* Corners: average both interior directions with scale. */
  if ((left || right) && (bottom || top)) {
    int2 ox = int2(left ? 1 : -1, 0);
    int2 oy = int2(0, bottom ? 1 : -1);
    float4 a = scale * texture_load(field_tx, texel + ox);
    float4 b = scale * texture_load(field_tx, texel + oy);
    imageStore(output_img, texel, 0.5f * (a + b));
    return;
  }

  imageStore(output_img, texel, scale * texture_load(field_tx, texel + offset));
}
