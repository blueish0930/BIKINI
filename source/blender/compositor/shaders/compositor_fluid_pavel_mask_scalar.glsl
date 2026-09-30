/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Mask a scalar field using solid/air classification.
 * mode:
 *   0 = zero on solid or air (Poisson RHS / liquid-only divergence)
 *   1 = zero on air only (free-surface pressure p=0)
 *   2 = zero on solid only
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_mask_scalar)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }
  float solid = texture_load(solid_tx, texel).x;
  float air = texture_load(air_tx, texel).x;
  float value = texture_load(input_tx, texel).x;
  bool is_solid = solid > 0.5f;
  bool is_air = (!is_solid) && (air > 0.5f);
  if (mode == 0 && (is_solid || is_air)) {
    value = 0.0f;
  }
  else if (mode == 1 && is_air) {
    value = 0.0f;
  }
  else if (mode == 2 && is_solid) {
    value = 0.0f;
  }
  imageStore(output_img, texel, float4(value, 0.0f, 0.0f, 1.0f));
}
