/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* IQ 2D fractal DE + smooth iteration:
 *   https://iquilezles.org/articles/distancefractals/
 *   https://iquilezles.org/articles/msetsmooth/
 *
 * kind: 0 = Mandelbrot, 1 = Julia
 */

#include "gpu_shader_material_transform_utils.bsl.hh"

float2 c_mul_iq(float2 a, float2 b)
{
  return float2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}

float2 c_pow_iq(float2 z, float p)
{
  float r = length(z);
  if (r < 1e-20f) {
    return float2(0.0f);
  }
  float a = atan(z.y, z.x);
  float rp = pow(r, p);
  return float2(rp * cos(a * p), rp * sin(a * p));
}

float fractal_escape2_iq(float bailout)
{
  float b = max(bailout, 1.01f);
  return min(b * b, 1.0e12f);
}

/* Mandelbrot / multibrot DE: z' = p z^{p-1} z' + 1 */
float de_mandelbrot_2d_iq(float3 pos, float power, float iterations, float bailout)
{
  float2 c = pos.xy;
  float2 z = float2(0.0f);
  float2 dz = float2(0.0f);
  float pwr = max(power, 1.0f);
  int iters = int(max(iterations, 1.0f));
  float escape2 = fractal_escape2_iq(bailout);
  float m2 = 0.0f;
  for (int i = 0; i < 256; i++) {
    if (i >= iters) {
      break;
    }
    float2 zp1 = c_pow_iq(z, pwr - 1.0f);
    dz = c_mul_iq(zp1, dz) * pwr + float2(1.0f, 0.0f);
    z = c_pow_iq(z, pwr) + c;
    m2 = dot(z, z);
    if (!(m2 < escape2)) {
      break;
    }
  }
  if (!(m2 > 1.0f)) {
    return 0.0f;
  }
  float dz2 = max(dot(dz, dz), 1e-20f);
  return sqrt(m2 / dz2) * (0.5f * log(m2)) / pwr;
}

/* Julia DE: z' = p z^{p-1} z' */
float de_julia_2d_iq(float3 pos, float2 jc, float power, float iterations, float bailout)
{
  float2 z = pos.xy;
  float2 dz = float2(1.0f, 0.0f);
  float pwr = max(power, 1.0f);
  int iters = int(max(iterations, 1.0f));
  float escape2 = fractal_escape2_iq(bailout);
  float m2 = dot(z, z);
  for (int i = 0; i < 256; i++) {
    if (i >= iters) {
      break;
    }
    float2 zp1 = c_pow_iq(z, pwr - 1.0f);
    dz = c_mul_iq(zp1, dz) * pwr;
    z = c_pow_iq(z, pwr) + jc;
    m2 = dot(z, z);
    if (!(m2 < escape2)) {
      break;
    }
  }
  if (!(m2 > 1.0f)) {
    return 0.0f;
  }
  float dz2 = max(dot(dz, dz), 1e-20f);
  return sqrt(m2 / dz2) * (0.5f * log(m2)) / pwr;
}

float smooth_iter_mandelbrot_iq(float2 c, float power, float iterations, float bailout)
{
  float pwr = max(power, 1.0f);
  float B = max(bailout, 2.0f);
  float2 z = float2(0.0f);
  float n = 0.0f;
  int iters = int(max(iterations, 1.0f));
  float B2 = B * B;
  float log_p = log(pwr);
  for (int i = 0; i < 256; i++) {
    if (i >= iters) {
      break;
    }
    z = c_pow_iq(z, pwr) + c;
    float m2 = dot(z, z);
    if (m2 > B2) {
      float lz = 0.5f * log(max(m2, 4.0f));
      return n - log(lz / log(B)) / log_p;
    }
    n += 1.0f;
  }
  return float(iters);
}

float smooth_iter_julia_iq(float2 z0, float2 c, float power, float iterations, float bailout)
{
  float pwr = max(power, 1.0f);
  float B = max(bailout, 2.0f);
  float2 z = z0;
  float n = 0.0f;
  int iters = int(max(iterations, 1.0f));
  float B2 = B * B;
  float log_p = log(pwr);
  for (int i = 0; i < 256; i++) {
    if (i >= iters) {
      break;
    }
    z = c_pow_iq(z, pwr) + c;
    float m2 = dot(z, z);
    if (m2 > B2) {
      float lz = 0.5f * log(max(m2, 4.0f));
      return n - log(lz / log(B)) / log_p;
    }
    n += 1.0f;
  }
  return float(iters);
}

float3 smooth_iter_to_color_iq(float iter, float max_iter)
{
  if (iter >= max_iter - 0.5f) {
    return float3(0.02f, 0.02f, 0.05f);
  }
  float t = iter * 0.05f;
  return float3(0.5f + 0.5f * cos(6.2831f * (t + 0.0f)),
                0.5f + 0.5f * cos(6.2831f * (t + 0.15f)),
                0.5f + 0.5f * cos(6.2831f * (t + 0.35f)));
}

[[node]]
void node_fractal_primitive(float3 vector,
                            float scale,
                            float power,
                            float iterations,
                            float bailout,
                            float3 julia_c,
                            float kind,
                            float4 &color,
                            float &distance)
{
  float3 p = vector * max(scale, 1e-6f);
  int k = int(kind + 0.5f);
  if (k == 1) {
    distance = de_julia_2d_iq(p, julia_c.xy, power, iterations, bailout);
    float it = smooth_iter_julia_iq(p.xy, julia_c.xy, power, iterations, bailout);
    color = float4(smooth_iter_to_color_iq(it, iterations), 1.0f);
  }
  else {
    distance = de_mandelbrot_2d_iq(p, power, iterations, bailout);
    float it = smooth_iter_mandelbrot_iq(p.xy, power, iterations, bailout);
    color = float4(smooth_iter_to_color_iq(it, iterations), 1.0f);
  }
}
