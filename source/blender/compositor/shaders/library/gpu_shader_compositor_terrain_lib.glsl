/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Gaea-style terrain library: gradient noise, ridged multifractal, hetero, warp.
 * Must stay in lockstep with node_image_terrain_common.hh. */

#pragma once

float terrain_luma(float4 c)
{
  return c.r * 0.2126f + c.g * 0.7152f + c.b * 0.0722f;
}

float terrain_hash12(float2 p)
{
  float3 p3 = fract(float3(p.xyx) * 0.1031f);
  p3 += dot(p3, p3.yzx + 33.33f);
  return fract((p3.x + p3.y) * p3.z);
}

float2 terrain_hash22(float2 p)
{
  float3 p3 = fract(float3(p.xyx) * float3(0.1031f, 0.1030f, 0.0973f));
  p3 += dot(p3, p3.yzx + 33.33f);
  return fract((p3.xx + p3.yz) * p3.zy);
}

float terrain_perlin(float2 p)
{
  float2 i = floor(p);
  float2 f = fract(p);
  float2 u = f * f * f * (f * (f * 6.0f - 15.0f) + 10.0f);
  float2 g00 = terrain_hash22(i) * 2.0f - 1.0f;
  float2 g10 = terrain_hash22(i + float2(1.0f, 0.0f)) * 2.0f - 1.0f;
  float2 g01 = terrain_hash22(i + float2(0.0f, 1.0f)) * 2.0f - 1.0f;
  float2 g11 = terrain_hash22(i + float2(1.0f, 1.0f)) * 2.0f - 1.0f;
  float n00 = dot(g00, f);
  float n10 = dot(g10, f - float2(1.0f, 0.0f));
  float n01 = dot(g01, f - float2(0.0f, 1.0f));
  float n11 = dot(g11, f - float2(1.0f, 1.0f));
  return mix(mix(n00, n10, u.x), mix(n01, n11, u.x), u.y);
}

float terrain_noise(float2 p)
{
  return terrain_perlin(p) * 0.5f + 0.5f;
}

float terrain_remap01(float h, float lo, float hi)
{
  return clamp((h - lo) / max(hi - lo, 1.0e-6f), 0.0f, 1.0f);
}

float terrain_fbm(float2 p, int octaves, float lacunarity, float gain)
{
  float sum = 0.0f;
  float amp = 0.5f;
  float norm = 0.0f;
  int n = clamp(octaves, 1, 12);
  for (int i = 0; i < n; i++) {
    sum += amp * terrain_perlin(p);
    norm += amp;
    p *= lacunarity;
    amp *= gain;
  }
  return (sum / max(norm, 1.0e-6f)) * 0.5f + 0.5f;
}

float terrain_ridged_mf(
    float2 p, int octaves, float H, float lacunarity, float offset, float gain)
{
  float signal = offset - abs(terrain_perlin(p));
  signal *= signal;
  float result = signal;
  float frequency = 1.0f;
  int n = clamp(octaves, 1, 12);
  for (int i = 1; i < n; i++) {
    p *= lacunarity;
    float weight = clamp(signal * gain, 0.0f, 1.0f);
    frequency *= lacunarity;
    signal = offset - abs(terrain_perlin(p));
    signal *= signal;
    signal *= weight;
    result += signal * pow(frequency, -H);
  }
  return result;
}

float terrain_hetero(float2 p, int octaves, float H, float lacunarity, float offset)
{
  float frequency = 1.0f;
  float result = terrain_perlin(p) * 0.5f + 0.5f + offset;
  p *= lacunarity;
  int n = clamp(octaves, 1, 12);
  for (int i = 1; i < n; i++) {
    float increment = (terrain_perlin(p) * 0.5f + offset) * pow(frequency, -H) * result;
    result += increment;
    p *= lacunarity;
    frequency *= lacunarity;
  }
  return result;
}

float terrain_ridged(float2 p, int octaves, float lacunarity, float gain)
{
  float H = mix(1.15f, 0.42f, clamp(gain, 0.0f, 1.0f));
  return terrain_remap01(terrain_ridged_mf(p, octaves, H, lacunarity, 1.0f, 2.05f), 0.0f, 1.55f);
}

float2 terrain_warp(float2 q, float distortion)
{
  if (distortion < 1.0e-5f) {
    return q;
  }
  for (int i = 0; i < 3; i++) {
    float amp = distortion / exp2(float(i));
    float2 w = float2(terrain_fbm(q + float2(17.13f, 9.27f), 4, 2.02f, 0.5f),
                      terrain_fbm(q + float2(5.2f, 1.3f), 4, 2.02f, 0.5f));
    q += (w * 2.0f - 1.0f) * amp;
  }
  return q;
}

float terrain_voronoi(float2 p)
{
  float2 i = floor(p);
  float2 f = fract(p);
  float min_d = 8.0f;
  for (int y = -1; y <= 1; y++) {
    for (int x = -1; x <= 1; x++) {
      float2 g = float2(float(x), float(y));
      float2 o = terrain_hash22(i + g);
      float2 r = g + o - f;
      min_d = min(min_d, dot(r, r));
    }
  }
  return sqrt(min_d);
}

float terrain_cell(float2 p)
{
  float2 i = floor(p);
  float2 f = fract(p);
  float min_d = 8.0f;
  float min_d2 = 8.0f;
  for (int y = -1; y <= 1; y++) {
    for (int x = -1; x <= 1; x++) {
      float2 g = float2(float(x), float(y));
      float2 o = terrain_hash22(i + g);
      float2 r = g + o - f;
      float d = length(r);
      if (d < min_d) {
        min_d2 = min_d;
        min_d = d;
      }
      else {
        min_d2 = min(min_d2, d);
      }
    }
  }
  return min_d2 - min_d;
}

float terrain_primitive_eval(float2 p,
                               int kind,
                               float scale,
                               float detail,
                               float roughness,
                               float distortion,
                               float lacunarity,
                               float seed,
                               float2 offset)
{
  float2 q = p * max(scale, 0.001f) + offset * max(scale, 0.001f) +
             float2(seed * 17.13f, seed * 9.27f);
  q = terrain_warp(q, distortion);
  int oct = int(clamp(detail, 1.0f, 12.0f));
  float g = clamp(roughness, 0.0f, 1.0f);
  float lac = max(lacunarity, 1.01f);
  float H = mix(1.15f, 0.42f, g);
  float d_radial = length(p + offset);

  if (kind == 0) { /* Mountain: warp + ridged MF + peak sharpen. */
    float r = terrain_ridged_mf(q, oct, H, lac, 1.0f, 2.05f);
    r = terrain_remap01(r, 0.05f, 1.45f);
    return pow(r, 1.28f);
  }
  if (kind == 12) { /* Mountainous: hetero + light ridged. */
    float h = terrain_hetero(q * 0.72f, oct, mix(0.95f, 0.55f, g), lac, 0.65f);
    h = terrain_remap01(h, 0.2f, 2.4f);
    float r = terrain_ridged_mf(q * 1.05f, max(oct - 2, 2), H, lac, 1.0f, 1.8f);
    r = terrain_remap01(r, 0.0f, 1.4f);
    return mix(h, r, 0.22f);
  }
  if (kind == 1) { /* Mountain range: parallel ridges. */
    float ridges = 0.0f;
    for (int i = 0; i < 3; i++) {
      float x = q.x - float(i - 1) * 0.85f;
      float envelope = exp(-x * x * 1.65f);
      float r = terrain_ridged_mf(float2(x * 0.35f, q.y), oct, H, lac, 1.0f, 2.0f);
      ridges = max(ridges, envelope * terrain_remap01(r, 0.0f, 1.5f));
    }
    float drape = terrain_hetero(q * 0.4f, 4, 0.85f, lac, 0.4f);
    return mix(ridges, mix(ridges, terrain_remap01(drape, 0.1f, 1.8f), 0.18f), 0.35f);
  }
  if (kind == 2) { /* Ridge: crest axis + gaussian falloff. */
    float crest = exp(-q.x * q.x * 2.35f);
    float r = terrain_ridged_mf(q * 1.15f, oct, H, lac, 1.0f, 2.0f);
    r = terrain_remap01(r, 0.0f, 1.45f);
    return clamp(crest * mix(0.55f, 1.05f, r), 0.0f, 1.0f);
  }
  if (kind == 3) { /* Canyon: hetero highlands minus directional incision. */
    float high = terrain_hetero(q * 0.55f, oct, mix(0.9f, 0.5f, g), lac, 0.7f);
    high = terrain_remap01(high, 0.25f, 2.2f);
    float channel = abs(q.x) + terrain_fbm(q * 0.45f, 3, 2.0f, 0.5f) * 0.28f;
    float cut = terrain_ridged_mf(float2(channel * 1.4f, q.y), oct, H, lac, 1.0f, 2.1f);
    cut = terrain_remap01(cut, 0.0f, 1.4f);
    float mask = 1.0f - smoothstep(0.15f, 1.15f, abs(q.x));
    return max(high - cut * 0.62f * mask, 0.0f);
  }
  if (kind == 4) { /* Crater. */
    float d = d_radial;
    float bowl = smoothstep(0.72f, 0.10f, d);
    float rim = smoothstep(0.20f, 0.34f, d) * (1.0f - smoothstep(0.38f, 0.58f, d));
    float floor_n = terrain_fbm(q * 3.2f, 3, 2.0f, 0.5f) * 0.08f;
    return clamp(bowl * 0.16f + rim * 0.62f + floor_n * bowl, 0.0f, 1.0f);
  }
  if (kind == 5) { /* Dunes: asymmetric lee. */
    float2 qd = q;
    qd.x += terrain_fbm(q * 0.75f, 3, 2.0f, 0.5f) * 1.35f;
    float phase = fract(qd.x * 0.52f + sin(qd.y * 0.35f) * 0.12f);
    float profile = (phase < 0.72f) ? pow(phase / 0.72f, 1.55f) :
                                        (1.0f - (phase - 0.72f) / 0.28f);
    float wind = terrain_fbm(q * 0.38f, 4, 2.0f, g);
    return mix(profile, profile * mix(0.65f, 1.0f, wind), 0.28f);
  }
  if (kind == 6) { /* Island. */
    float mass = 1.0f - smoothstep(0.12f, 0.95f, d_radial);
    float r = terrain_ridged_mf(q, oct, H, lac, 1.0f, 2.0f);
    r = terrain_remap01(r, 0.0f, 1.45f);
    return mass * mix(0.22f, 1.0f, r);
  }
  if (kind == 7) { /* Volcano: cone + caldera. */
    float cone = pow(max(1.0f - d_radial * 1.12f, 0.0f), 1.28f);
    float caldera = 1.0f - smoothstep(0.0f, 0.11f, d_radial);
    float rim = smoothstep(0.08f, 0.16f, d_radial) * (1.0f - smoothstep(0.18f, 0.28f, d_radial));
    float r = terrain_fbm(q * 1.55f, oct, lac, g);
    return clamp(max(cone - caldera * 0.32f, 0.0f) * mix(0.72f, 1.08f, r) + rim * 0.18f,
                 0.0f,
                 1.0f);
  }
  if (kind == 8) { /* Slump: hetero hills. */
    float h = terrain_hetero(q * 0.58f, oct, mix(0.85f, 0.5f, g), lac, 0.55f);
    h = terrain_remap01(h, 0.15f, 2.1f);
    float s = terrain_fbm(q * 1.35f + 3.1f, 3, 2.0f, 0.5f);
    return mix(h, smoothstep(0.18f, 0.88f, h), 0.32f) * mix(0.88f, 1.08f, s);
  }
  if (kind == 9) { /* Plates: terraces + cliffs. */
    float n = terrain_hetero(q * 0.48f, oct, mix(0.9f, 0.5f, g), lac, 0.6f);
    n = terrain_remap01(n, 0.2f, 2.15f);
    float levels = 5.0f;
    float plate = floor(n * levels) / levels;
    float edge = abs(n * levels - round(n * levels));
    float cliff = 1.0f - smoothstep(0.0f, 0.16f, edge);
    return mix(plate, n, 1.0f - cliff * 0.85f);
  }
  if (kind == 10) { /* Cracks: carve from plateau. */
    float base = terrain_hetero(q * 0.5f, 4, 0.8f, lac, 0.55f);
    base = terrain_remap01(base, 0.2f, 1.9f);
    float c = terrain_cell(q * 1.85f);
    float carve = pow(clamp(c * 1.7f, 0.0f, 1.0f), 0.55f);
    return max(base - carve * 0.55f, 0.0f);
  }
  /* Rugged: slope-aware breakup. */
  float r = terrain_ridged_mf(q, oct, H, lac, 1.0f, 2.0f);
  r = terrain_remap01(r, 0.0f, 1.5f);
  float dx = terrain_fbm(q + float2(0.04f, 0.0f), 3, 2.0f, 0.5f) -
              terrain_fbm(q - float2(0.04f, 0.0f), 3, 2.0f, 0.5f);
  float dy = terrain_fbm(q + float2(0.0f, 0.04f), 3, 2.0f, 0.5f) -
              terrain_fbm(q - float2(0.0f, 0.04f), 3, 2.0f, 0.5f);
  float slope = clamp(length(float2(dx, dy)) * 6.0f, 0.0f, 1.0f);
  float c = terrain_cell(q * 2.15f);
  return mix(r, r * (0.5f + c), slope * 0.55f);
}
