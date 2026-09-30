/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Multigrid residual for Pavel's clamp-to-edge pressure stencil. */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_residual)

#include "gpu_shader_compositor_texture_utilities.glsl"

float scalar_at(sampler2D tx, int2 p, int2 size)
{
  return texture_load(tx, clamp(p, int2(0), size - int2(1))).x;
}

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }
  float C = scalar_at(pressure_tx, texel, size);
  float L = scalar_at(pressure_tx, texel - int2(1, 0), size);
  float R = scalar_at(pressure_tx, texel + int2(1, 0), size);
  float B = scalar_at(pressure_tx, texel - int2(0, 1), size);
  float T = scalar_at(pressure_tx, texel + int2(0, 1), size);
  float rhs = texture_load(rhs_tx, texel).x;
  imageStore(output_img, texel, float4(rhs - (L + R + B + T - 4.0f * C), 0.0f, 0.0f, 1.0f));
}
