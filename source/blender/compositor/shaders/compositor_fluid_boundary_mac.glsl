/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * MAC domain boundary conditions (solid box).
 * field_kind 0: pressure — pure Neumann (ghost = interior)
 * field_kind 1: u faces  — no-slip: left/right walls u=0; top/bottom copy interior
 * field_kind 2: v faces  — no-slip: bottom/top walls v=0; left/right copy interior
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_boundary_mac)

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
  const bool interior = !left && !right && !bottom && !top;

  if (interior) {
    imageStore(output_img, texel, texture_load(field_tx, texel));
    return;
  }

  if (field_kind == 0) {
    /* Pressure Neumann: copy from nearest interior. */
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
    if ((left || right) && (bottom || top)) {
      int2 ox = int2(left ? 1 : -1, 0);
      int2 oy = int2(0, bottom ? 1 : -1);
      float a = texture_load(field_tx, texel + ox).x;
      float b = texture_load(field_tx, texel + oy).x;
      imageStore(output_img, texel, float4(0.5f * (a + b), 0.0f, 0.0f, 1.0f));
    }
    else {
      imageStore(output_img, texel, float4(texture_load(field_tx, texel + offset).x, 0.0f, 0.0f, 1.0f));
    }
    return;
  }

  if (field_kind == 1) {
    /* u faces (W+1)×H: solid vertical walls at i=0 and i=W → u=0. */
    if (left || right) {
      imageStore(output_img, texel, float4(0.0f));
      return;
    }
    /* Horizontal domain edges: copy from interior face. */
    int2 n = texel;
    if (bottom) {
      n.y = 1;
    }
    if (top) {
      n.y = size.y - 2;
    }
    imageStore(output_img, texel, float4(texture_load(field_tx, n).x, 0.0f, 0.0f, 1.0f));
    return;
  }

  /* field_kind == 2: v faces W×(H+1): solid horizontal walls j=0, j=H → v=0. */
  if (bottom || top) {
    imageStore(output_img, texel, float4(0.0f));
    return;
  }
  int2 n = texel;
  if (left) {
    n.x = 1;
  }
  if (right) {
    n.x = size.x - 2;
  }
  imageStore(output_img, texel, float4(texture_load(field_tx, n).x, 0.0f, 0.0f, 1.0f));
}
