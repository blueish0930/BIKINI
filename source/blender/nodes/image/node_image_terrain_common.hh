/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"

#include "GPU_shader.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "DNA_node_types.h"

#include <cmath>

namespace blender::nodes::terrain {

using Color = compositor::Color;

inline float luma(const Color &c)
{
  return c.r * 0.2126f + c.g * 0.7152f + c.b * 0.0722f;
}

inline Color gray(const float h)
{
  return Color(h, h, h, 1.0f);
}

inline float smoothstep(const float edge0, const float edge1, const float x)
{
  const float t = math::clamp((x - edge0) / math::max(edge1 - edge0, 1.0e-8f), 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

inline float read_float(const compositor::Result &r, const float fallback)
{
  if (r.is_single_value()) {
    return r.get_single_value_default<float>();
  }
  if (r.is_allocated()) {
    return r.load_pixel<float, true>(int2(0));
  }
  return fallback;
}

inline float3 read_float3(const compositor::Result &r, const float3 fallback)
{
  if (r.is_single_value()) {
    return r.get_single_value_default<float3>();
  }
  if (r.is_allocated()) {
    return r.load_pixel<float3, true>(int2(0));
  }
  return fallback;
}

inline Color read_color(const compositor::Result &r, const Color fallback)
{
  if (r.is_single_value()) {
    return r.get_single_value_default<Color>();
  }
  if (r.is_allocated()) {
    return r.load_pixel<Color, true>(int2(0));
  }
  return fallback;
}

inline void gpu_barrier()
{
  GPU_memory_barrier(GPU_BARRIER_SHADER_IMAGE_ACCESS | GPU_BARRIER_TEXTURE_FETCH |
                     GPU_BARRIER_TEXTURE_UPDATE);
}

inline gpu::Shader *try_shader(compositor::Context &context, const char *name)
{
  if (!context.use_gpu()) {
    return nullptr;
  }
  if (GPU_shader_create_info_get(name) == nullptr) {
    return nullptr;
  }
  return context.get_shader(name);
}

inline compositor::Result *ensure_cpu(compositor::Context &context,
                                        compositor::Result &input,
                                        compositor::Result &storage)
{
  if (context.use_gpu() && input.is_allocated() && !input.is_single_value() &&
      input.is_stored_on_gpu())
  {
    storage = input.download_to_cpu();
    return &storage;
  }
  return &input;
}

inline void share_cpu_to_output(compositor::Context &context,
                                compositor::Result &output,
                                compositor::Result &cpu)
{
  if (context.use_gpu()) {
    compositor::Result gpu = cpu.upload_to_gpu(true);
    output.share_data(gpu);
    gpu.release();
  }
  else {
    output.share_data(cpu);
  }
  cpu.release();
}

inline float hash12(const float2 p)
{
  float3 p3 = math::fract(float3(p.x, p.y, p.x) * 0.1031f);
  p3 += math::dot(p3, float3(p3.y, p3.z, p3.x) + 33.33f);
  return math::fract((p3.x + p3.y) * p3.z);
}

inline float2 hash22(const float2 p)
{
  float3 p3 = math::fract(float3(p.x, p.y, p.x) * float3(0.1031f, 0.1030f, 0.0973f));
  p3 += math::dot(p3, float3(p3.y, p3.z, p3.x) + 33.33f);
  return math::fract(float2((p3.x + p3.y) * p3.z, (p3.x + p3.z) * p3.y));
}

inline float perlin(const float2 p)
{
  const float2 i = float2(math::floor(p.x), math::floor(p.y));
  const float2 f = p - i;
  const float2 u = f * f * f * (f * (f * 6.0f - 15.0f) + 10.0f);
  const float2 g00 = hash22(i) * 2.0f - 1.0f;
  const float2 g10 = hash22(i + float2(1.0f, 0.0f)) * 2.0f - 1.0f;
  const float2 g01 = hash22(i + float2(0.0f, 1.0f)) * 2.0f - 1.0f;
  const float2 g11 = hash22(i + float2(1.0f, 1.0f)) * 2.0f - 1.0f;
  const float n00 = math::dot(g00, f);
  const float n10 = math::dot(g10, f - float2(1.0f, 0.0f));
  const float n01 = math::dot(g01, f - float2(0.0f, 1.0f));
  const float n11 = math::dot(g11, f - float2(1.0f, 1.0f));
  return math::interpolate(math::interpolate(n00, n10, u.x),
                           math::interpolate(n01, n11, u.x),
                           u.y);
}

inline float remap01(const float h, const float lo, const float hi)
{
  return math::clamp((h - lo) / math::max(hi - lo, 1.0e-6f), 0.0f, 1.0f);
}

inline float fbm(float2 p, int octaves, const float lacunarity, const float gain)
{
  float sum = 0.0f;
  float amp = 0.5f;
  float norm = 0.0f;
  const int n = math::clamp(octaves, 1, 12);
  for (int i = 0; i < n; i++) {
    sum += amp * perlin(p);
    norm += amp;
    p *= lacunarity;
    amp *= gain;
  }
  return (sum / math::max(norm, 1.0e-6f)) * 0.5f + 0.5f;
}

inline float ridged_mf(float2 p,
                       const int octaves,
                       const float H,
                       const float lacunarity,
                       const float offset,
                       const float gain)
{
  float signal = offset - math::abs(perlin(p));
  signal *= signal;
  float result = signal;
  float frequency = 1.0f;
  const int n = math::clamp(octaves, 1, 12);
  for (int i = 1; i < n; i++) {
    p *= lacunarity;
    const float weight = math::clamp(signal * gain, 0.0f, 1.0f);
    frequency *= lacunarity;
    signal = offset - math::abs(perlin(p));
    signal *= signal;
    signal *= weight;
    result += signal * std::pow(frequency, -H);
  }
  return result;
}

inline float hetero(
    float2 p, const int octaves, const float H, const float lacunarity, const float offset)
{
  float frequency = 1.0f;
  float result = perlin(p) * 0.5f + 0.5f + offset;
  p *= lacunarity;
  const int n = math::clamp(octaves, 1, 12);
  for (int i = 1; i < n; i++) {
    const float increment = (perlin(p) * 0.5f + offset) * std::pow(frequency, -H) * result;
    result += increment;
    p *= lacunarity;
    frequency *= lacunarity;
  }
  return result;
}

inline float ridged(float2 p, int octaves, const float lacunarity, const float gain)
{
  const float H = math::interpolate(1.15f, 0.42f, math::clamp(gain, 0.0f, 1.0f));
  return remap01(ridged_mf(p, octaves, H, lacunarity, 1.0f, 2.05f), 0.0f, 1.55f);
}

inline float2 warp(float2 q, const float distortion)
{
  if (distortion < 1.0e-5f) {
    return q;
  }
  for (int i = 0; i < 3; i++) {
    const float amp = distortion / std::exp2(float(i));
    const float2 w = float2(fbm(q + float2(17.13f, 9.27f), 4, 2.02f, 0.5f),
                             fbm(q + float2(5.2f, 1.3f), 4, 2.02f, 0.5f));
    q += (w * 2.0f - 1.0f) * amp;
  }
  return q;
}

inline float noise2(const float2 p)
{
  return perlin(p) * 0.5f + 0.5f;
}

inline float voronoi(const float2 p)
{
  const float2 i = float2(math::floor(p.x), math::floor(p.y));
  const float2 f = p - i;
  float min_d = 8.0f;
  for (int y = -1; y <= 1; y++) {
    for (int x = -1; x <= 1; x++) {
      const float2 g = float2(float(x), float(y));
      const float2 o = hash22(i + g);
      const float2 r = g + o - f;
      min_d = math::min(min_d, math::dot(r, r));
    }
  }
  return math::sqrt(min_d);
}

inline float cell(const float2 p)
{
  const float2 i = float2(math::floor(p.x), math::floor(p.y));
  const float2 f = p - i;
  float min_d = 8.0f;
  float min_d2 = 8.0f;
  for (int y = -1; y <= 1; y++) {
    for (int x = -1; x <= 1; x++) {
      const float2 g = float2(float(x), float(y));
      const float2 o = hash22(i + g);
      const float2 r = g + o - f;
      const float d = math::length(r);
      if (d < min_d) {
        min_d2 = min_d;
        min_d = d;
      }
      else {
        min_d2 = math::min(min_d2, d);
      }
    }
  }
  return min_d2 - min_d;
}

inline float primitive_eval(const float2 p,
                            const int kind,
                            const float scale,
                            const float amp,
                            const int oct,
                            const float gain,
                            const float distort,
                            const float lac,
                            const float seed,
                            const float2 offset)
{
  const float s = math::max(scale, 0.001f);
  float2 q = p * s + offset * s + float2(seed * 17.13f, seed * 9.27f);
  q = warp(q, distort);
  const float H = math::interpolate(1.15f, 0.42f, math::clamp(gain, 0.0f, 1.0f));
  const float d_radial = math::length(p + offset);
  float h = 0.0f;

  switch (kind) {
    case NODE_TERRAIN_PRIM_MOUNTAIN: {
      float r = ridged_mf(q, oct, H, lac, 1.0f, 2.05f);
      r = remap01(r, 0.05f, 1.45f);
      h = std::pow(r, 1.28f);
      break;
    }
    case NODE_TERRAIN_PRIM_MOUNTAINOUS: {
      float ht = hetero(q * 0.72f, oct, math::interpolate(0.95f, 0.55f, gain), lac, 0.65f);
      ht = remap01(ht, 0.2f, 2.4f);
      float r = ridged_mf(q * 1.05f, math::max(oct - 2, 2), H, lac, 1.0f, 1.8f);
      r = remap01(r, 0.0f, 1.4f);
      h = math::interpolate(ht, r, 0.22f);
      break;
    }
    case NODE_TERRAIN_PRIM_MOUNTAIN_RANGE: {
      float ridges = 0.0f;
      for (int i = 0; i < 3; i++) {
        const float x = q.x - float(i - 1) * 0.85f;
        const float envelope = std::exp(-x * x * 1.65f);
        const float r = remap01(ridged_mf(float2(x * 0.35f, q.y), oct, H, lac, 1.0f, 2.0f),
                                 0.0f,
                                 1.5f);
        ridges = math::max(ridges, envelope * r);
      }
      const float drape = remap01(hetero(q * 0.4f, 4, 0.85f, lac, 0.4f), 0.1f, 1.8f);
      h = math::interpolate(ridges, math::interpolate(ridges, drape, 0.18f), 0.35f);
      break;
    }
    case NODE_TERRAIN_PRIM_RIDGE: {
      const float crest = std::exp(-q.x * q.x * 2.35f);
      float r = remap01(ridged_mf(q * 1.15f, oct, H, lac, 1.0f, 2.0f), 0.0f, 1.45f);
      h = math::clamp(crest * math::interpolate(0.55f, 1.05f, r), 0.0f, 1.0f);
      break;
    }
    case NODE_TERRAIN_PRIM_CANYON: {
      float high = remap01(
          hetero(q * 0.55f, oct, math::interpolate(0.9f, 0.5f, gain), lac, 0.7f), 0.25f, 2.2f);
      const float channel = math::abs(q.x) + fbm(q * 0.45f, 3, 2.0f, 0.5f) * 0.28f;
      float cut = remap01(
          ridged_mf(float2(channel * 1.4f, q.y), oct, H, lac, 1.0f, 2.1f), 0.0f, 1.4f);
      const float mask = 1.0f - smoothstep(0.15f, 1.15f, math::abs(q.x));
      h = math::max(high - cut * 0.62f * mask, 0.0f);
      break;
    }
    case NODE_TERRAIN_PRIM_CRATER: {
      const float d = d_radial;
      const float bowl = smoothstep(0.72f, 0.10f, d);
      const float rim = smoothstep(0.20f, 0.34f, d) * (1.0f - smoothstep(0.38f, 0.58f, d));
      const float floor_n = fbm(q * 3.2f, 3, 2.0f, 0.5f) * 0.08f;
      h = math::clamp(bowl * 0.16f + rim * 0.62f + floor_n * bowl, 0.0f, 1.0f);
      break;
    }
    case NODE_TERRAIN_PRIM_DUNES: {
      float2 qd = q;
      qd.x += fbm(q * 0.75f, 3, 2.0f, 0.5f) * 1.35f;
      const float phase = math::fract(qd.x * 0.52f + std::sin(qd.y * 0.35f) * 0.12f);
      const float profile = (phase < 0.72f) ? std::pow(phase / 0.72f, 1.55f) :
                                                (1.0f - (phase - 0.72f) / 0.28f);
      const float wind = fbm(q * 0.38f, 4, 2.0f, gain);
      h = math::interpolate(profile, profile * math::interpolate(0.65f, 1.0f, wind), 0.28f);
      break;
    }
    case NODE_TERRAIN_PRIM_ISLAND: {
      const float mass = 1.0f - smoothstep(0.12f, 0.95f, d_radial);
      float r = remap01(ridged_mf(q, oct, H, lac, 1.0f, 2.0f), 0.0f, 1.45f);
      h = mass * math::interpolate(0.22f, 1.0f, r);
      break;
    }
    case NODE_TERRAIN_PRIM_VOLCANO: {
      const float cone = std::pow(math::max(1.0f - d_radial * 1.12f, 0.0f), 1.28f);
      const float caldera = 1.0f - smoothstep(0.0f, 0.11f, d_radial);
      const float rim = smoothstep(0.08f, 0.16f, d_radial) *
                         (1.0f - smoothstep(0.18f, 0.28f, d_radial));
      const float r = fbm(q * 1.55f, oct, lac, gain);
      h = math::clamp(math::max(cone - caldera * 0.32f, 0.0f) *
                              math::interpolate(0.72f, 1.08f, r) +
                          rim * 0.18f,
                      0.0f,
                      1.0f);
      break;
    }
    case NODE_TERRAIN_PRIM_SLUMP: {
      float ht = remap01(
          hetero(q * 0.58f, oct, math::interpolate(0.85f, 0.5f, gain), lac, 0.55f), 0.15f, 2.1f);
      const float s = fbm(q * 1.35f + 3.1f, 3, 2.0f, 0.5f);
      h = math::interpolate(ht, smoothstep(0.18f, 0.88f, ht), 0.32f) *
          math::interpolate(0.88f, 1.08f, s);
      break;
    }
    case NODE_TERRAIN_PRIM_PLATES: {
      float n = remap01(
          hetero(q * 0.48f, oct, math::interpolate(0.9f, 0.5f, gain), lac, 0.6f), 0.2f, 2.15f);
      const float levels = 5.0f;
      const float plate = std::floor(n * levels) / levels;
      const float edge = math::abs(n * levels - std::round(n * levels));
      const float cliff = 1.0f - smoothstep(0.0f, 0.16f, edge);
      h = math::interpolate(plate, n, 1.0f - cliff * 0.85f);
      break;
    }
    case NODE_TERRAIN_PRIM_CRACKS: {
      float base = remap01(hetero(q * 0.5f, 4, 0.8f, lac, 0.55f), 0.2f, 1.9f);
      const float c = cell(q * 1.85f);
      const float carve = std::pow(math::clamp(c * 1.7f, 0.0f, 1.0f), 0.55f);
      h = math::max(base - carve * 0.55f, 0.0f);
      break;
    }
    default: {
      float r = remap01(ridged_mf(q, oct, H, lac, 1.0f, 2.0f), 0.0f, 1.5f);
      const float dx = fbm(q + float2(0.04f, 0.0f), 3, 2.0f, 0.5f) -
                        fbm(q - float2(0.04f, 0.0f), 3, 2.0f, 0.5f);
      const float dy = fbm(q + float2(0.0f, 0.04f), 3, 2.0f, 0.5f) -
                        fbm(q - float2(0.0f, 0.04f), 3, 2.0f, 0.5f);
      const float slope = math::clamp(math::length(float2(dx, dy)) * 6.0f, 0.0f, 1.0f);
      const float c = cell(q * 2.15f);
      h = math::interpolate(r, r * (0.5f + c), slope * 0.55f);
      break;
    }
  }
  return math::clamp(h, 0.0f, 1.0f) * amp;
}

}  // namespace blender::nodes::terrain
