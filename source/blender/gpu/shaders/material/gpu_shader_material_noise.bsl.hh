/* SPDX-FileCopyrightText: 2019-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "gpu_shader_common_hash.bsl.hh"
#include "gpu_shader_math_vector_safe.bsl.hh"

[[force_inline]] void floor_frac(float x, int &x_int, float &x_fract)
{
  float x_floor = floor(x);
  x_int = int(x_floor);
  x_fract = x - x_floor;
}

#ifndef GPU_SHADER_PERIODIC_WRAP
#define GPU_SHADER_PERIODIC_WRAP
int wrap_period_int(int x, int period)
{
  if (period <= 0) {
    return x;
  }
  /* GLSL `%` is undefined for negative operands. Floor division is well-defined
   * and matches C++ `((x % p) + p) % p` (e.g. cell -1 with period 4 -> 3). */
  return x - period * int(floor(float(x) / float(period)));
}

int period_cells(float period)
{
  if (period < 0.5f) {
    return 0;
  }
  return max(int(floor(period + 0.5f)), 2);
}

int wrap_cell(int cell, float period)
{
  return wrap_period_int(cell, period_cells(period));
}

int2 wrap_cell(int2 cell, float2 period)
{
  return int2(wrap_cell(cell.x, period.x), wrap_cell(cell.y, period.y));
}

int3 wrap_cell(int3 cell, float3 period)
{
  return int3(wrap_cell(cell.x, period.x), wrap_cell(cell.y, period.y), wrap_cell(cell.z, period.z));
}

int4 wrap_cell(int4 cell, float4 period)
{
  return int4(wrap_cell(cell.x, period.x),
              wrap_cell(cell.y, period.y),
              wrap_cell(cell.z, period.z),
              wrap_cell(cell.w, period.w));
}

float wrap_cell(float cell, float period)
{
  return float(wrap_cell(int(floor(cell)), period));
}

/* Reduce a coordinate into [0, period). period_cells <= 0 leaves the value unchanged. */
float wrap_coord(float x, float period)
{
  int p = period_cells(period);
  if (p <= 0) {
    return x;
  }
  return x - float(p) * floor(x / float(p));
}
float2 wrap_coord(float2 x, float2 period)
{
  return float2(wrap_coord(x.x, period.x), wrap_coord(x.y, period.y));
}
float3 wrap_coord(float3 x, float3 period)
{
  return float3(wrap_coord(x.x, period.x), wrap_coord(x.y, period.y), wrap_coord(x.z, period.z));
}
float4 wrap_coord(float4 x, float4 period)
{
  return float4(wrap_coord(x.x, period.x),
                wrap_coord(x.y, period.y),
                wrap_coord(x.z, period.z),
                wrap_coord(x.w, period.w));
}
float wrap_coord(float x, float4 period)
{
  return wrap_coord(x, period.x);
}
float2 wrap_coord(float2 x, float4 period)
{
  return wrap_coord(x, period.xy);
}
float3 wrap_coord(float3 x, float4 period)
{
  return wrap_coord(x, period.xyz);
}

/* 1D Voronoi stores the feature in Position.w while the period lives in period.x. */
float4 wrap_voronoi_position(float4 pos, float4 period)
{
  pos.x = wrap_coord(pos.x, period.x);
  pos.y = wrap_coord(pos.y, period.y);
  pos.z = wrap_coord(pos.z, period.z);
  pos.w = wrap_coord(pos.w, period_cells(period.w) > 0 ? period.w : period.x);
  return pos;
}

float quantize_period(float p)
{
  return float(period_cells(p));
}
float2 quantize_period(float2 p)
{
  return float2(quantize_period(p.x), quantize_period(p.y));
}
float3 quantize_period(float3 p)
{
  return float3(quantize_period(p.x), quantize_period(p.y), quantize_period(p.z));
}
float4 quantize_period(float4 p)
{
  return float4(
      quantize_period(p.x), quantize_period(p.y), quantize_period(p.z), quantize_period(p.w));
}

bool period_active(float p)
{
  return period_cells(p) > 0;
}
bool period_active(float2 p)
{
  return period_cells(p.x) > 0 || period_cells(p.y) > 0;
}
bool period_active(float3 p)
{
  return period_cells(p.x) > 0 || period_cells(p.y) > 0 || period_cells(p.z) > 0;
}
bool period_active(float4 p)
{
  return period_cells(p.x) > 0 || period_cells(p.y) > 0 || period_cells(p.z) > 0 ||
         period_cells(p.w) > 0;
}

/* Integer seed offsets keep floor(p + T + off) = floor(p + off) + T for integer tile size. */
float tile_offset(float off, float per)
{
  return period_active(per) ? floor(off) : off;
}
float2 tile_offset(float2 off, float2 per)
{
  return period_active(per) ? floor(off) : off;
}
float3 tile_offset(float3 off, float3 per)
{
  return period_active(per) ? floor(off) : off;
}
float4 tile_offset(float4 off, float4 per)
{
  return period_active(per) ? floor(off) : off;
}
#endif

/* Bilinear Interpolation:
 *
 * v2          v3
 *  @ + + + + @       y
 *  +         +       ^
 *  +         +       |
 *  +         +       |
 *  @ + + + + @       @------> x
 * v0          v1
 */
float bi_mix(float v0, float v1, float v2, float v3, float x, float y)
{
  float x1 = 1.0f - x;
  return (1.0f - y) * (v0 * x1 + v1 * x) + y * (v2 * x1 + v3 * x);
}

/* Trilinear Interpolation:
 *
 *   v6               v7
 *     @ + + + + + + @
 *     +\            +\
 *     + \           + \
 *     +  \          +  \
 *     +   \ v4      +   \ v5
 *     +    @ + + + +++ + @          z
 *     +    +        +    +      y   ^
 *  v2 @ + +++ + + + @ v3 +       \  |
 *      \   +         \   +        \ |
 *       \  +          \  +         \|
 *        \ +           \ +          +---------> x
 *         \+            \+
 *          @ + + + + + + @
 *        v0               v1
 */
float tri_mix(float v0,
              float v1,
              float v2,
              float v3,
              float v4,
              float v5,
              float v6,
              float v7,
              float x,
              float y,
              float z)
{
  float x1 = 1.0f - x;
  float y1 = 1.0f - y;
  float z1 = 1.0f - z;
  return z1 * (y1 * (v0 * x1 + v1 * x) + y * (v2 * x1 + v3 * x)) +
         z * (y1 * (v4 * x1 + v5 * x) + y * (v6 * x1 + v7 * x));
}

float quad_mix(float v0,
               float v1,
               float v2,
               float v3,
               float v4,
               float v5,
               float v6,
               float v7,
               float v8,
               float v9,
               float v10,
               float v11,
               float v12,
               float v13,
               float v14,
               float v15,
               float x,
               float y,
               float z,
               float w)
{
  return mix(tri_mix(v0, v1, v2, v3, v4, v5, v6, v7, x, y, z),
             tri_mix(v8, v9, v10, v11, v12, v13, v14, v15, x, y, z),
             w);
}

float fade(float t)
{
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float negate_if(float value, uint condition)
{
  return (condition != 0u) ? -value : value;
}

float noise_grad(uint hash, float x)
{
  uint h = hash & 15u;
  float g = 1u + (h & 7u);
  return negate_if(g, h & 8u) * x;
}

float noise_grad(uint hash, float x, float y)
{
  uint h = hash & 7u;
  float u = h < 4u ? x : y;
  float v = 2.0f * (h < 4u ? y : x);
  return negate_if(u, h & 1u) + negate_if(v, h & 2u);
}

float noise_grad(uint hash, float x, float y, float z)
{
  uint h = hash & 15u;
  float u = h < 8u ? x : y;
  float vt = ((h == 12u) || (h == 14u)) ? x : z;
  float v = h < 4u ? y : vt;
  return negate_if(u, h & 1u) + negate_if(v, h & 2u);
}

float noise_grad(uint hash, float x, float y, float z, float w)
{
  uint h = hash & 31u;
  float u = h < 24u ? x : y;
  float v = h < 16u ? y : z;
  float s = h < 8u ? z : w;
  return negate_if(u, h & 1u) + negate_if(v, h & 2u) + negate_if(s, h & 4u);
}

float noise_perlin(float x, float period)
{
  int X;
  float fx;

  floor_frac(x, X, fx);
  int X0 = wrap_cell(X, period);
  int X1 = wrap_cell(X + 1, period);

  float u = fade(fx);

  float r = mix(noise_grad(hash_int(X0), fx), noise_grad(hash_int(X1), fx - 1.0f), u);

  return r;
}

float noise_perlin(float x)
{
  return noise_perlin(x, 0.0f);
}

float noise_perlin(float2 vec, float2 period)
{
  int X, Y;
  float fx, fy;

  floor_frac(vec.x, X, fx);
  floor_frac(vec.y, Y, fy);
  int X0 = wrap_cell(X, period.x);
  int X1 = wrap_cell(X + 1, period.x);
  int Y0 = wrap_cell(Y, period.y);
  int Y1 = wrap_cell(Y + 1, period.y);

  float u = fade(fx);
  float v = fade(fy);

  float r = bi_mix(noise_grad(hash_int2(X0, Y0), fx, fy),
                   noise_grad(hash_int2(X1, Y0), fx - 1.0f, fy),
                   noise_grad(hash_int2(X0, Y1), fx, fy - 1.0f),
                   noise_grad(hash_int2(X1, Y1), fx - 1.0f, fy - 1.0f),
                   u,
                   v);

  return r;
}

float noise_perlin(float2 vec)
{
  return noise_perlin(vec, float2(0.0f));
}

float noise_perlin(float3 vec, float3 period)
{
  int X, Y, Z;
  float fx, fy, fz;

  floor_frac(vec.x, X, fx);
  floor_frac(vec.y, Y, fy);
  floor_frac(vec.z, Z, fz);
  int X0 = wrap_cell(X, period.x);
  int X1 = wrap_cell(X + 1, period.x);
  int Y0 = wrap_cell(Y, period.y);
  int Y1 = wrap_cell(Y + 1, period.y);
  int Z0 = wrap_cell(Z, period.z);
  int Z1 = wrap_cell(Z + 1, period.z);

  float u = fade(fx);
  float v = fade(fy);
  float w = fade(fz);

  float r = tri_mix(noise_grad(hash_int3(X0, Y0, Z0), fx, fy, fz),
                    noise_grad(hash_int3(X1, Y0, Z0), fx - 1, fy, fz),
                    noise_grad(hash_int3(X0, Y1, Z0), fx, fy - 1, fz),
                    noise_grad(hash_int3(X1, Y1, Z0), fx - 1, fy - 1, fz),
                    noise_grad(hash_int3(X0, Y0, Z1), fx, fy, fz - 1),
                    noise_grad(hash_int3(X1, Y0, Z1), fx - 1, fy, fz - 1),
                    noise_grad(hash_int3(X0, Y1, Z1), fx, fy - 1, fz - 1),
                    noise_grad(hash_int3(X1, Y1, Z1), fx - 1, fy - 1, fz - 1),
                    u,
                    v,
                    w);

  return r;
}

float noise_perlin(float3 vec)
{
  return noise_perlin(vec, float3(0.0f));
}

float noise_perlin(float4 vec, float4 period)
{
  int X, Y, Z, W;
  float fx, fy, fz, fw;

  floor_frac(vec.x, X, fx);
  floor_frac(vec.y, Y, fy);
  floor_frac(vec.z, Z, fz);
  floor_frac(vec.w, W, fw);
  int X0 = wrap_cell(X, period.x);
  int X1 = wrap_cell(X + 1, period.x);
  int Y0 = wrap_cell(Y, period.y);
  int Y1 = wrap_cell(Y + 1, period.y);
  int Z0 = wrap_cell(Z, period.z);
  int Z1 = wrap_cell(Z + 1, period.z);
  int W0 = wrap_cell(W, period.w);
  int W1 = wrap_cell(W + 1, period.w);

  float u = fade(fx);
  float v = fade(fy);
  float t = fade(fz);
  float s = fade(fw);

  float r = quad_mix(
      noise_grad(hash_int4(X0, Y0, Z0, W0), fx, fy, fz, fw),
      noise_grad(hash_int4(X1, Y0, Z0, W0), fx - 1.0f, fy, fz, fw),
      noise_grad(hash_int4(X0, Y1, Z0, W0), fx, fy - 1.0f, fz, fw),
      noise_grad(hash_int4(X1, Y1, Z0, W0), fx - 1.0f, fy - 1.0f, fz, fw),
      noise_grad(hash_int4(X0, Y0, Z1, W0), fx, fy, fz - 1.0f, fw),
      noise_grad(hash_int4(X1, Y0, Z1, W0), fx - 1.0f, fy, fz - 1.0f, fw),
      noise_grad(hash_int4(X0, Y1, Z1, W0), fx, fy - 1.0f, fz - 1.0f, fw),
      noise_grad(hash_int4(X1, Y1, Z1, W0), fx - 1.0f, fy - 1.0f, fz - 1.0f, fw),
      noise_grad(hash_int4(X0, Y0, Z0, W1), fx, fy, fz, fw - 1.0f),
      noise_grad(hash_int4(X1, Y0, Z0, W1), fx - 1.0f, fy, fz, fw - 1.0f),
      noise_grad(hash_int4(X0, Y1, Z0, W1), fx, fy - 1.0f, fz, fw - 1.0f),
      noise_grad(hash_int4(X1, Y1, Z0, W1), fx - 1.0f, fy - 1.0f, fz, fw - 1.0f),
      noise_grad(hash_int4(X0, Y0, Z1, W1), fx, fy, fz - 1.0f, fw - 1.0f),
      noise_grad(hash_int4(X1, Y0, Z1, W1), fx - 1.0f, fy, fz - 1.0f, fw - 1.0f),
      noise_grad(hash_int4(X0, Y1, Z1, W1), fx, fy - 1.0f, fz - 1.0f, fw - 1.0f),
      noise_grad(
          hash_int4(X1, Y1, Z1, W1), fx - 1.0f, fy - 1.0f, fz - 1.0f, fw - 1.0f),
      u,
      v,
      t,
      s);

  return r;
}

float noise_perlin(float4 vec)
{
  return noise_perlin(vec, float4(0.0f));
}

/* Remap the output of noise to a predictable range [-1, 1].
 * The scale values were computed experimentally by the OSL developers.
 */
float noise_scale1(float result)
{
  return 0.2500f * result;
}

float noise_scale2(float result)
{
  return 0.6616f * result;
}

float noise_scale3(float result)
{
  return 0.9820f * result;
}

float noise_scale4(float result)
{
  return 0.8344f * result;
}

/* Safe Signed And Unsigned Noise */

float snoise(float p)
{
  float precision_correction = 0.5f * float(abs(p) >= 1000000.0f);
  /* Repeat Perlin noise texture every 100000.0 on each axis to prevent floating point
   * representation issues. */
  p = compatible_mod(p, 100000.0f) + precision_correction;

  return noise_scale1(noise_perlin(p));
}

float snoise(float p, float period)
{
  if (period_cells(period) <= 0) {
    return snoise(p);
  }
  return noise_scale1(noise_perlin(p, period));
}

float noise(float p)
{
  return 0.5f * snoise(p) + 0.5f;
}

float snoise(float2 p)
{
  float2 precision_correction = 0.5f * float2(float(abs(p.x) >= 1000000.0f),
                                              float(abs(p.y) >= 1000000.0f));
  /* Repeat Perlin noise texture every 100000.0 on each axis to prevent floating point
   * representation issues. This causes discontinuities every 100000.0f, however at such scales
   * this usually shouldn't be noticeable. */
  p = compatible_mod(p, 100000.0f) + precision_correction;

  return noise_scale2(noise_perlin(p));
}

float snoise(float2 p, float2 period)
{
  if (period_cells(period.x) <= 0 && period_cells(period.y) <= 0) {
    return snoise(p);
  }
  return noise_scale2(noise_perlin(p, period));
}

float noise(float2 p)
{
  return 0.5f * snoise(p) + 0.5f;
}

float snoise(float3 p)
{
  float3 precision_correction = 0.5f * float3(float(abs(p.x) >= 1000000.0f),
                                              float(abs(p.y) >= 1000000.0f),
                                              float(abs(p.z) >= 1000000.0f));
  /* Repeat Perlin noise texture every 100000.0 on each axis to prevent floating point
   * representation issues. This causes discontinuities every 100000.0f, however at such scales
   * this usually shouldn't be noticeable. */
  p = compatible_mod(p, 100000.0f) + precision_correction;

  return noise_scale3(noise_perlin(p));
}

float snoise(float3 p, float3 period)
{
  if (period_cells(period.x) <= 0 && period_cells(period.y) <= 0 && period_cells(period.z) <= 0) {
    return snoise(p);
  }
  return noise_scale3(noise_perlin(p, period));
}

float noise(float3 p)
{
  return 0.5f * snoise(p) + 0.5f;
}

float snoise(float4 p)
{
  float4 precision_correction = 0.5f * float4(float(abs(p.x) >= 1000000.0f),
                                              float(abs(p.y) >= 1000000.0f),
                                              float(abs(p.z) >= 1000000.0f),
                                              float(abs(p.w) >= 1000000.0f));
  /* Repeat Perlin noise texture every 100000.0 on each axis to prevent floating point
   * representation issues. This causes discontinuities every 100000.0f, however at such scales
   * this usually shouldn't be noticeable. */
  p = compatible_mod(p, 100000.0f) + precision_correction;

  return noise_scale4(noise_perlin(p));
}

float snoise(float4 p, float4 period)
{
  if (period_cells(period.x) <= 0 && period_cells(period.y) <= 0 && period_cells(period.z) <= 0 &&
      period_cells(period.w) <= 0)
  {
    return snoise(p);
  }
  return noise_scale4(noise_perlin(p, period));
}

float noise(float4 p)
{
  return 0.5f * snoise(p) + 0.5f;
}
