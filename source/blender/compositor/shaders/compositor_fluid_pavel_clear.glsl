/* SPDX-FileCopyrightText: 2017 Pavel Dobryakov
 * SPDX-FileCopyrightText: 2026 Blender Authors (compute adaptation)
 *
 * SPDX-License-Identifier: MIT
 *
 * From WebGL-Fluid-Simulation/script.js clearShader:
 *   gl_FragColor = value * texture2D(uTexture, vUv);
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_pavel_clear)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }
  imageStore(output_img, texel, texture_load(input_tx, texel) * value);
}
