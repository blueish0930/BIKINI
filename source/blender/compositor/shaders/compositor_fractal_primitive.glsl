/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* 2D Mandelbrot / Julia (IQ smooth iteration + polar z^p). Full-domain GPU path. */

#include "infos/compositor_fractal_primitive_infos.hh"

COMPUTE_SHADER_CREATE_INFO(compositor_fractal_primitive)

#include "gpu_shader_compositor_texture_utilities.glsl"

float2 c_mul(float2 a, float2 b)
{
  return float2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}

float2 c_pow(float2 z, float p)
{
  float r = length(z);
  if (r < 1.0e-20f) {
    return float2(0.0f);
  }
  float a = atan(z.y, z.x);
  float rp = pow(r, p);
  return float2(rp * cos(a * p), rp * sin(a * p));
}

float de_mandelbrot_2d(float2 c, int iters, float bailout, float pwr)
{
  float2 z = float2(0.0f);
  float2 dz = float2(1.0f, 0.0f);
  float escape2 = min(max(bailout, 1.01f) * max(bailout, 1.01f), 1.0e12f);
  for (int i = 0; i < iters; i++) {
    /* dz = p * z^{p-1} * dz + 1 */
    float2 zp = c_pow(z, pwr - 1.0f);
    dz = c_mul(float2(pwr, 0.0f), c_mul(zp, dz)) + float2(1.0f, 0.0f);
    z = c_pow(z, pwr) + c;
    float m2 = dot(z, z);
    if (m2 > escape2) {
      float r = sqrt(m2);
      float dr = length(dz);
      return 0.5f * log(max(r, 1.0e-20f)) * r / max(dr, 1.0e-20f);
    }
  }
  return 0.0f;
}

float de_julia_2d(float2 z0, float2 c, int iters, float bailout, float pwr)
{
  float2 z = z0;
  float2 dz = float2(1.0f, 0.0f);
  float escape2 = min(max(bailout, 1.01f) * max(bailout, 1.01f), 1.0e12f);
  for (int i = 0; i < iters; i++) {
    float2 zp = c_pow(z, pwr - 1.0f);
    dz = c_mul(float2(pwr, 0.0f), c_mul(zp, dz));
    z = c_pow(z, pwr) + c;
    float m2 = dot(z, z);
    if (m2 > escape2) {
      float r = sqrt(m2);
      float dr = length(dz);
      return 0.5f * log(max(r, 1.0e-20f)) * r / max(dr, 1.0e-20f);
    }
  }
  return 0.0f;
}

float smooth_iter_mandelbrot(float2 c, int iters, float B, float pwr)
{
  float2 z = float2(0.0f);
  float Bb = max(B, 2.0f);
  float B2 = Bb * Bb;
  float log_p = log(max(pwr, 1.0f));
  float n = 0.0f;
  for (int i = 0; i < iters; i++) {
    z = c_pow(z, pwr) + c;
    float m2 = dot(z, z);
    if (m2 > B2) {
      float lz = 0.5f * log(max(m2, 4.0f));
      return n - log(lz / log(Bb)) / log_p;
    }
    n += 1.0f;
  }
  return float(iters);
}

float smooth_iter_julia(float2 z0, float2 c, int iters, float B, float pwr)
{
  float2 z = z0;
  float Bb = max(B, 2.0f);
  float B2 = Bb * Bb;
  float log_p = log(max(pwr, 1.0f));
  float n = 0.0f;
  for (int i = 0; i < iters; i++) {
    z = c_pow(z, pwr) + c;
    float m2 = dot(z, z);
    if (m2 > B2) {
      float lz = 0.5f * log(max(m2, 4.0f));
      return n - log(lz / log(Bb)) / log_p;
    }
    n += 1.0f;
  }
  return float(iters);
}

float3 smooth_iter_to_color(float iter, float max_iter)
{
  if (iter >= max_iter - 0.5f) {
    return float3(0.02f, 0.02f, 0.05f);
  }
  float t = iter * 0.05f;
  return float3(0.5f + 0.5f * cos(6.2831f * (t + 0.0f)),
                0.5f + 0.5f * cos(6.2831f * (t + 0.15f)),
                0.5f + 0.5f * cos(6.2831f * (t + 0.35f)));
}

void main()
{
  int2 texel = int2(gl_GlobalInvocationID.xy);
  if (any(greaterThanEqual(texel, domain_size))) {
    return;
  }

  float2 p;
  if (vector_is_single != 0) {
    p.x = (((float(texel.x) + 0.5f) / float(domain_size.x)) * 2.0f - 1.0f) * scale;
    p.y = (((float(texel.y) + 0.5f) / float(domain_size.y)) * 2.0f - 1.0f) * scale;
  }
  else {
    float3 v = texture_load(vector_tx, texel).xyz;
    p = v.xy;
  }

  int iters = max(iterations, 1);
  float pwr = max(power, 1.0f);
  float B = max(bailout, 1.01f);

  float d;
  float it_smooth;
  if (fractal_kind == 1) {
    d = de_julia_2d(p, julia_c, iters, B, pwr);
    it_smooth = smooth_iter_julia(p, julia_c, iters, B, pwr);
  }
  else {
    d = de_mandelbrot_2d(p, iters, B, pwr);
    it_smooth = smooth_iter_mandelbrot(p, iters, B, pwr);
  }

  float3 col = smooth_iter_to_color(it_smooth, float(iters));
  imageStore(color_img, texel, float4(col, 1.0f));
  imageStore(distance_img, texel, float4(d));
}
