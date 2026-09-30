/* SPDX-FileCopyrightText: 2017 Pavel Dobryakov (WebGL-Fluid-Simulation)
 * SPDX-FileCopyrightText: 2026 Blender Authors (compute adaptation)
 *
 * SPDX-License-Identifier: MIT
 *
 * Shared helpers for the compute port of
 * https://github.com/PavelDoGreat/WebGL-Fluid-Simulation
 *
 * Original baseVertexShader emits:
 *   vUv = aPosition * 0.5 + 0.5;
 *   vL = vUv - vec2(texelSize.x, 0);
 *   vR = vUv + vec2(texelSize.x, 0);
 *   vT = vUv + vec2(0, texelSize.y);
 *   vB = vUv - vec2(0, texelSize.y);
 * with texelSize = 1/resolution.
 */

#pragma once

float2 pavel_uv(int2 texel, int2 size)
{
  return (float2(texel) + 0.5f) / float2(size);
}

float2 pavel_texel_size(int2 size)
{
  return 1.0f / float2(size);
}

/* Manual bilinear sample matching advectionShader bilerp (tsize = 1/size). */
float4 pavel_bilerp(sampler2D sam, float2 uv, float2 tsize)
{
  float2 st = uv / tsize - 0.5f;
  float2 iuv = floor(st);
  float2 fuv = fract(st);
  float4 a = texture_load(sam, int2(iuv + float2(0.5f, 0.5f)));
  float4 b = texture_load(sam, int2(iuv + float2(1.5f, 0.5f)));
  float4 c = texture_load(sam, int2(iuv + float2(0.5f, 1.5f)));
  float4 d = texture_load(sam, int2(iuv + float2(1.5f, 1.5f)));
  /* Clamp loads already in texture_load. */
  return mix(mix(a, b, fuv.x), mix(c, d, fuv.x), fuv.y);
}

float4 pavel_sample(sampler2D sam, float2 uv)
{
  int2 size = texture_size(sam);
  float2 p = clamp(uv * float2(size) - 0.5f, float2(0.0f), float2(size - int2(1)));
  int2 i0 = int2(floor(p));
  int2 i1 = min(i0 + int2(1), size - int2(1));
  float2 f = fract(p);
  float4 a = texture_load(sam, i0);
  float4 b = texture_load(sam, int2(i1.x, i0.y));
  float4 c = texture_load(sam, int2(i0.x, i1.y));
  float4 d = texture_load(sam, i1);
  return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}
