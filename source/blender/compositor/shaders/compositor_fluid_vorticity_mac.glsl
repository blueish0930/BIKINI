/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Optional vorticity confinement on MAC faces (mild).
 * component 0: u faces; 1: v faces.
 */

#include "infos/compositor_fluid_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fluid_vorticity_mac)

#include "gpu_shader_compositor_texture_utilities.glsl"

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  int2 size = imageSize(output_img);
  if (any(greaterThanEqual(texel, size))) {
    return;
  }

  if (component == 0) {
    float u = texture_load(u_tx, texel).x;
    if (curl_strength == 0.0f || texel.x <= 0 || texel.x >= size.x - 1 || texel.y <= 0 ||
        texel.y >= size.y - 1)
    {
      imageStore(output_img, texel, float4(u, 0.0f, 0.0f, 1.0f));
      return;
    }
    int2 usize = texture_size(u_tx);
    int2 vsize = texture_size(v_tx);
    int2 c = clamp(texel, int2(1), min(usize, vsize) - int2(2));
    float vR = texture_load(v_tx, clamp(c + int2(1, 0), int2(0), vsize - int2(1))).x;
    float vL = texture_load(v_tx, clamp(c - int2(1, 0), int2(0), vsize - int2(1))).x;
    float uT = texture_load(u_tx, clamp(c + int2(0, 1), int2(0), usize - int2(1))).x;
    float uB = texture_load(u_tx, clamp(c - int2(0, 1), int2(0), usize - int2(1))).x;
    float omega = 0.5f * ((vR - vL) - (uT - uB));
    float force = curl_strength * omega * 0.05f;
    imageStore(output_img, texel, float4(u + force * timestep, 0.0f, 0.0f, 1.0f));
  }
  else {
    float v = texture_load(v_tx, texel).x;
    if (curl_strength == 0.0f || texel.y <= 0 || texel.y >= size.y - 1 || texel.x <= 0 ||
        texel.x >= size.x - 1)
    {
      imageStore(output_img, texel, float4(v, 0.0f, 0.0f, 1.0f));
      return;
    }
    int2 usize = texture_size(u_tx);
    int2 vsize = texture_size(v_tx);
    int2 c = clamp(texel, int2(1), min(usize, vsize) - int2(2));
    float vR = texture_load(v_tx, clamp(c + int2(1, 0), int2(0), vsize - int2(1))).x;
    float vL = texture_load(v_tx, clamp(c - int2(1, 0), int2(0), vsize - int2(1))).x;
    float uT = texture_load(u_tx, clamp(c + int2(0, 1), int2(0), usize - int2(1))).x;
    float uB = texture_load(u_tx, clamp(c - int2(0, 1), int2(0), usize - int2(1))).x;
    float omega = 0.5f * ((vR - vL) - (uT - uB));
    float force = -curl_strength * omega * 0.05f;
    imageStore(output_img, texel, float4(v + force * timestep, 0.0f, 0.0f, 1.0f));
  }
}
