/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Bilinear prolongation of coarse correction, added to fine field:
 * out = fine + prolong(coarse)
 *
 * Uses half-index coordinates matching full-weighting restriction
 * (coarse i ← avg fine[2i, 2i+1]), not domain-UV * coarse_size.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_prolong)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 fsize = imageSize(output_img);
  if (any(greaterThanEqual(texel, fsize))) {
    return;
  }

  float base = texture_load(fine_tx, texel).x;

  if (texel.x == 0 || texel.y == 0 || texel.x == fsize.x - 1 || texel.y == fsize.y - 1) {
    imageStore(output_img, texel, float4(base, 0.0f, 0.0f, 1.0f));
    return;
  }

  int2 csize = texture_size(coarse_tx);
  /* Align with restrict: fine t maps to continuous coarse (t+0.5)/2 - 0.5. */
  float fx = (float(texel.x) + 0.5f) * 0.5f - 0.5f;
  float fy = (float(texel.y) + 0.5f) * 0.5f - 0.5f;
  int i0 = clamp(int(floor(fx)), 0, csize.x - 1);
  int j0 = clamp(int(floor(fy)), 0, csize.y - 1);
  int i1 = min(i0 + 1, csize.x - 1);
  int j1 = min(j0 + 1, csize.y - 1);
  float tx = clamp(fx - float(i0), 0.0f, 1.0f);
  float ty = clamp(fy - float(j0), 0.0f, 1.0f);

  float a = texture_load(coarse_tx, int2(i0, j0)).x;
  float b = texture_load(coarse_tx, int2(i1, j0)).x;
  float c = texture_load(coarse_tx, int2(i0, j1)).x;
  float d = texture_load(coarse_tx, int2(i1, j1)).x;
  float corr = mix(mix(a, b, tx), mix(c, d, tx), ty);

  imageStore(output_img, texel, float4(base + corr, 0.0f, 0.0f, 1.0f));
}
