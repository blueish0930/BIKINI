/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Signed-distance / fractal DE math — Inigo Quilez exact formulas.
 *
 * References (MIT code snippets):
 *   https://iquilezles.org/articles/distfunctions/      (3D SDF)
 *   https://iquilezles.org/articles/distfunctions2d/    (2D SDF)
 *   https://iquilezles.org/articles/distancefractals/   (2D fractal DE)
 *   https://iquilezles.org/articles/msetsmooth/         (smooth iteration)
 *   https://iquilezles.org/articles/mandelbulb/         (Mandelbulb)
 *
 * Shared by Shader Editor and Image Process nodes. Header-only.
 */

#pragma once

#include <algorithm>
#include <cmath>

namespace blender::nodes::sdf_math {

struct float3 {
  float x, y, z;
  float3() : x(0), y(0), z(0) {}
  explicit float3(float v) : x(v), y(v), z(v) {}
  float3(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct float2 {
  float x, y;
  float2() : x(0), y(0) {}
  float2(float x, float y) : x(x), y(y) {}
};

inline float3 operator+(const float3 &a, const float3 &b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline float3 operator-(const float3 &a, const float3 &b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline float3 operator*(const float3 &a, const float s)
{
  return {a.x * s, a.y * s, a.z * s};
}
inline float3 operator*(const float s, const float3 &a)
{
  return a * s;
}
inline float2 operator+(const float2 &a, const float2 &b)
{
  return {a.x + b.x, a.y + b.y};
}
inline float2 operator-(const float2 &a, const float2 &b)
{
  return {a.x - b.x, a.y - b.y};
}
inline float2 operator*(const float2 &a, const float s)
{
  return {a.x * s, a.y * s};
}
inline float2 operator*(const float s, const float2 &a)
{
  return a * s;
}
inline float2 operator*(const float2 &a, const float2 &b)
{
  return {a.x * b.x, a.y * b.y};
}
inline float3 operator*(const float3 &a, const float3 &b)
{
  return {a.x * b.x, a.y * b.y, a.z * b.z};
}

inline float length(const float3 &p)
{
  return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}
inline float length(const float2 &p)
{
  return std::sqrt(p.x * p.x + p.y * p.y);
}
inline float dot(const float3 &a, const float3 &b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline float dot(const float2 &a, const float2 &b)
{
  return a.x * b.x + a.y * b.y;
}
inline float dot2(const float2 &v)
{
  return dot(v, v);
}
inline float dot2(const float3 &v)
{
  return dot(v, v);
}
inline float3 abs3(const float3 &p)
{
  return {std::fabs(p.x), std::fabs(p.y), std::fabs(p.z)};
}
inline float2 abs2(const float2 &p)
{
  return {std::fabs(p.x), std::fabs(p.y)};
}
inline float3 max3(const float3 &a, const float3 &b)
{
  return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}
inline float3 min3(const float3 &a, const float3 &b)
{
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
inline float2 max2(const float2 &a, const float2 &b)
{
  return {std::max(a.x, b.x), std::max(a.y, b.y)};
}
inline float2 min2(const float2 &a, const float2 &b)
{
  return {std::min(a.x, b.x), std::min(a.y, b.y)};
}
inline float clamp_f(const float v, const float lo, const float hi)
{
  return std::max(lo, std::min(hi, v));
}
inline float sign_f(const float v)
{
  return (v > 0.0f) ? 1.0f : ((v < 0.0f) ? -1.0f : 0.0f);
}
/* GLSL `mod` (always non-negative for y>0). `std::fmod` keeps the sign of x,
 * which breaks IQ's `mod(atan2(p.x,p.y), 2*an)` on the left half of a star. */
inline float glsl_mod(const float x, const float y)
{
  return x - y * std::floor(x / y);
}
inline float2 float2_from_xy(const float3 &p)
{
  return {p.x, p.y};
}

/* -------------------------------------------------------------------- */
/** \name 3D SDF primitives (exact unless noted) — distfunctions
 * \{ */

inline float sd_sphere(const float3 &p, const float r)
{
  return length(p) - r;
}

inline float sd_box(const float3 &p, const float3 &b)
{
  const float3 q = abs3(p) - b;
  return length(max3(q, float3(0.0f))) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
}

inline float sd_round_box(const float3 &p, const float3 &b, const float r)
{
  const float3 q = abs3(p) - b + float3(r);
  return length(max3(q, float3(0.0f))) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f) - r;
}

inline float sd_box_frame(const float3 &p, const float3 &b, const float e)
{
  float3 pp = abs3(p) - b;
  float3 q = abs3(pp + float3(e)) - float3(e);
  const float d1 = length(max3(float3(pp.x, q.y, q.z), float3(0.0f))) +
                   std::min(std::max(pp.x, std::max(q.y, q.z)), 0.0f);
  const float d2 = length(max3(float3(q.x, pp.y, q.z), float3(0.0f))) +
                   std::min(std::max(q.x, std::max(pp.y, q.z)), 0.0f);
  const float d3 = length(max3(float3(q.x, q.y, pp.z), float3(0.0f))) +
                   std::min(std::max(q.x, std::max(q.y, pp.z)), 0.0f);
  return std::min(std::min(d1, d2), d3);
}

inline float sd_torus(const float3 &p, const float major_r, const float minor_r)
{
  const float2 q(length(float2(p.x, p.z)) - major_r, p.y);
  return length(q) - minor_r;
}

/** Capped torus: sc = (sin, cos) of half-aperture, ra major, rb minor. */
inline float sd_capped_torus(const float3 &p, const float2 &sc, const float ra, const float rb)
{
  float3 pp = p;
  pp.x = std::fabs(pp.x);
  const float k = (sc.y * pp.x > sc.x * pp.y) ? (pp.x * sc.x + pp.y * sc.y) : length(float2(pp.x, pp.y));
  return std::sqrt(dot(pp, pp) + ra * ra - 2.0f * ra * k) - rb;
}

inline float sd_link(const float3 &p, const float le, const float r1, const float r2)
{
  const float3 q(p.x, std::max(std::fabs(p.y) - le, 0.0f), p.z);
  return length(float2(length(float2(q.x, q.y)) - r1, q.z)) - r2;
}

/** Infinite cylinder along Y: c.xy = center in XZ, c.z = radius. */
inline float sd_infinite_cylinder(const float3 &p, const float2 &c_xz, const float r)
{
  return length(float2(p.x - c_xz.x, p.z - c_xz.y)) - r;
}

/** Finite cone: c = (sin,cos) angle, h height (exact). Base on +Y? IQ uses h along -Y tip. */
inline float sd_cone(const float3 &p, const float2 &c, const float h)
{
  const float2 q = float2(c.x / std::max(c.y, 1e-8f), -1.0f) * h;
  const float2 w(length(float2(p.x, p.z)), p.y);
  const float2 a = w - q * clamp_f(dot(w, q) / std::max(dot(q, q), 1e-20f), 0.0f, 1.0f);
  const float2 b = w - q * float2(clamp_f(w.x / std::max(q.x, 1e-20f), 0.0f, 1.0f), 1.0f);
  const float k = sign_f(q.y);
  const float d = std::min(dot(a, a), dot(b, b));
  const float s = std::max(k * (w.x * q.y - w.y * q.x), k * (w.y - q.y));
  return std::sqrt(d) * sign_f(s);
}

inline float sd_infinite_cone(const float3 &p, const float2 &c)
{
  const float2 q(length(float2(p.x, p.z)), -p.y);
  const float d = length(q - c * std::max(dot(q, c), 0.0f));
  return d * ((q.x * c.y - q.y * c.x < 0.0f) ? -1.0f : 1.0f);
}

inline float sd_plane(const float3 &p, const float3 &n, const float h)
{
  return dot(p, n) + h;
}

inline float sd_hex_prism(const float3 &p, const float2 &h)
{
  const float3 k(-0.8660254f, 0.5f, 0.57735f);
  float3 pp = abs3(p);
  const float m = 2.0f * std::min(k.x * pp.x + k.y * pp.y, 0.0f);
  pp.x -= m * k.x;
  pp.y -= m * k.y;
  const float2 d(length(float2(pp.x - clamp_f(pp.x, -k.z * h.x, k.z * h.x), pp.y - h.x)) *
                     sign_f(pp.y - h.x),
                 pp.z - h.y);
  return std::min(std::max(d.x, d.y), 0.0f) + length(max3(float3(d.x, d.y, 0.0f), float3(0.0f)));
}

/** Vertical capsule: total height h, radius r, centered (endpoints ±h/2). */
inline float sd_vertical_capsule(const float3 &p, const float h, const float r)
{
  const float half = std::max(h, 0.0f) * 0.5f;
  float3 q = p;
  q.y -= clamp_f(q.y, -half, half);
  return length(q) - r;
}

/** IQ vertical capsule: from y=0 to y=h. */
inline float sd_vertical_capsule_iq(const float3 &p, const float h, const float r)
{
  float3 q = p;
  q.y -= clamp_f(q.y, 0.0f, h);
  return length(q) - r;
}

inline float sd_capped_cylinder(const float3 &p, const float r, const float h)
{
  const float2 d(std::fabs(length(float2(p.x, p.z))) - r, std::fabs(p.y) - h);
  return std::min(std::max(d.x, d.y), 0.0f) + length(max3(float3(d.x, d.y, 0.0f), float3(0.0f)));
}

inline float sd_rounded_cylinder(const float3 &p, const float ra, const float rb, const float h)
{
  const float2 d(length(float2(p.x, p.z)) - ra + rb, std::fabs(p.y) - h + rb);
  return std::min(std::max(d.x, d.y), 0.0f) + length(max3(float3(d.x, d.y, 0.0f), float3(0.0f))) -
         rb;
}

inline float sd_capped_cone(const float3 &p, const float h, const float r1, const float r2)
{
  const float2 q(length(float2(p.x, p.z)), p.y);
  const float2 k1(r2, h);
  const float2 k2(r2 - r1, 2.0f * h);
  const float2 ca(q.x - std::min(q.x, (q.y < 0.0f) ? r1 : r2), std::fabs(q.y) - h);
  const float k2_len2 = dot(k2, k2);
  const float t = clamp_f(dot(float2(k1.x - q.x, k1.y - q.y), k2) / std::max(k2_len2, 1e-20f),
                          0.0f,
                          1.0f);
  const float2 cb = q - k1 + k2 * t;
  const float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * std::sqrt(std::min(dot(ca, ca), dot(cb, cb)));
}

/** Solid angle: c=(sin,cos) of angle, ra radius. */
inline float sd_solid_angle(const float3 &p, const float2 &c, const float ra)
{
  const float2 q(length(float2(p.x, p.z)), p.y);
  const float l = length(q) - ra;
  const float m = length(q - c * clamp_f(dot(q, c), 0.0f, ra));
  return std::max(l, m * sign_f(c.y * q.x - c.x * q.y));
}

inline float sd_cut_sphere(const float3 &p, const float r, const float h)
{
  const float w = std::sqrt(std::max(r * r - h * h, 0.0f));
  const float2 q(length(float2(p.x, p.z)), p.y);
  const float s = std::max((h - r) * q.x * q.x + w * w * (h + r - 2.0f * q.y), h * q.x - w * q.y);
  return (s < 0.0f) ? (length(q) - r) : ((q.x < w) ? (h - q.y) : length(q - float2(w, h)));
}

inline float sd_cut_hollow_sphere(const float3 &p, const float r, const float h, const float t)
{
  const float w = std::sqrt(std::max(r * r - h * h, 0.0f));
  const float2 q(length(float2(p.x, p.z)), p.y);
  return ((h * q.x < w * q.y) ? length(q - float2(w, h)) : std::fabs(length(q) - r)) - t;
}

inline float sd_death_star(const float3 &p2, const float ra, const float rb, const float d)
{
  const float a = (ra * ra - rb * rb + d * d) / (2.0f * std::max(d, 1e-8f));
  const float b = std::sqrt(std::max(ra * ra - a * a, 0.0f));
  const float2 p(p2.x, length(float2(p2.y, p2.z)));
  if (p.x * b - p.y * a > d * std::max(b - p.y, 0.0f)) {
    return length(p - float2(a, b));
  }
  return std::max(length(p) - ra, -(length(p - float2(d, 0.0f)) - rb));
}

inline float sd_round_cone(const float3 &p, const float r1, const float r2, const float h)
{
  const float b = (r1 - r2) / std::max(h, 1e-8f);
  const float a = std::sqrt(std::max(1.0f - b * b, 0.0f));
  const float2 q(length(float2(p.x, p.z)), p.y);
  const float k = dot(q, float2(-b, a));
  if (k < 0.0f) {
    return length(q) - r1;
  }
  if (k > a * h) {
    return length(q - float2(0.0f, h)) - r2;
  }
  return dot(q, float2(a, b)) - r1;
}

inline float sd_rhombus_3d(const float3 &p, const float la, const float lb, const float h, const float ra)
{
  float3 pp = abs3(p);
  const float f = clamp_f((la * pp.x - lb * pp.z + lb * lb) / std::max(la * la + lb * lb, 1e-20f),
                          0.0f,
                          1.0f);
  const float2 w = float2(pp.x, pp.z) - float2(la, lb) * float2(f, 1.0f - f);
  const float2 q(length(w) * sign_f(w.x) - ra, pp.y - h);
  return std::min(std::max(q.x, q.y), 0.0f) + length(max3(float3(q.x, q.y, 0.0f), float3(0.0f)));
}

inline float sd_octahedron(const float3 &p, const float s)
{
  float3 pp = abs3(p);
  const float m = pp.x + pp.y + pp.z - s;
  float3 q;
  if (3.0f * pp.x < m) {
    q = pp;
  }
  else if (3.0f * pp.y < m) {
    q = float3(pp.y, pp.z, pp.x);
  }
  else if (3.0f * pp.z < m) {
    q = float3(pp.z, pp.x, pp.y);
  }
  else {
    return m * 0.57735027f;
  }
  const float k = clamp_f(0.5f * (q.z - q.y + s), 0.0f, s);
  return length(float3(q.x, q.y - s + k, q.z - k));
}

inline float sd_pyramid(const float3 &p, const float h)
{
  const float m2 = h * h + 0.25f;
  float3 pp = p;
  pp.x = std::fabs(pp.x);
  pp.z = std::fabs(pp.z);
  if (pp.z > pp.x) {
    const float t = pp.x;
    pp.x = pp.z;
    pp.z = t;
  }
  pp.x -= 0.5f;
  pp.z -= 0.5f;
  const float3 q(pp.z, h * pp.y - 0.5f * pp.x, h * pp.x + 0.5f * pp.y);
  const float s = std::max(-q.x, 0.0f);
  const float t = clamp_f((q.y - 0.5f * q.z) / (m2 + 0.25f), 0.0f, 1.0f);
  const float a = m2 * (q.x + s) * (q.x + s) + q.y * q.y;
  const float b = m2 * (q.x + 0.5f * t) * (q.x + 0.5f * t) + (q.y - m2 * t) * (q.y - m2 * t);
  const float d2 = std::min(q.y, -q.x * m2 - q.y * 0.5f) > 0.0f ? 0.0f : std::min(a, b);
  return std::sqrt((d2 + q.z * q.z) / m2) * sign_f(std::max(q.z, -pp.y));
}

/** Ellipsoid — lower bound (IQ). */
inline float sd_ellipsoid(const float3 &p, const float3 &r)
{
  const float k0 = length(float3(p.x / r.x, p.y / r.y, p.z / r.z));
  const float k1 = length(float3(p.x / (r.x * r.x), p.y / (r.y * r.y), p.z / (r.z * r.z)));
  return k0 * (k0 - 1.0f) / std::max(k1, 1e-20f);
}

/** Triangular prism — lower bound (IQ). */
inline float sd_tri_prism(const float3 &p, const float2 &h)
{
  const float3 q = abs3(p);
  return std::max(q.z - h.y, std::max(q.x * 0.866025f + p.y * 0.5f, -p.y) - h.x * 0.5f);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name 2D SDF primitives (exact) — distfunctions2d
 * \{ */

inline float sd_circle(const float2 &p, const float r)
{
  return length(p) - r;
}

inline float sd_box_2d(const float2 &p, const float2 &b)
{
  const float2 d = abs2(p) - b;
  return length(max2(d, float2(0.0f, 0.0f))) + std::min(std::max(d.x, d.y), 0.0f);
}

/** Rounded box: r.xy = (tr,br) if p.x>0 else (tl,bl) — simplified uniform corner via r.x */
inline float sd_rounded_box_2d(const float2 &p, const float2 &b, const float r)
{
  /* Uniform corner radius (full vec4 version uses per-corner; socket gives one r). */
  float2 rr(r, r);
  /* Match IQ vec4 path with all corners equal. */
  float2 q = abs2(p) - b + float2(r, r);
  return std::min(std::max(q.x, q.y), 0.0f) + length(max2(q, float2(0.0f, 0.0f))) - r;
}

inline float sd_chamfer_box_2d(const float2 &p, const float2 &b, const float chamfer)
{
  float2 pp = abs2(p) - b;
  if (pp.y > pp.x) {
    const float t = pp.x;
    pp.x = pp.y;
    pp.y = t;
  }
  pp.y += chamfer;
  const float k = 1.0f - std::sqrt(2.0f);
  if (pp.y < 0.0f && pp.y + pp.x * k < 0.0f) {
    return pp.x;
  }
  if (pp.x < pp.y) {
    return (pp.x + pp.y) * std::sqrt(0.5f);
  }
  return length(pp);
}

inline float sd_segment_2d(const float2 &p, const float2 &a, const float2 &b)
{
  const float2 pa = p - a;
  const float2 ba = b - a;
  const float h = clamp_f(dot(pa, ba) / std::max(dot(ba, ba), 1e-20f), 0.0f, 1.0f);
  return length(pa - ba * h);
}

inline float sd_rhombus_2d(const float2 &p, const float2 &b)
{
  float2 bb = b;
  bb.y = -bb.y;
  float2 pp = abs2(p);
  const float h = clamp_f((dot(bb, pp) + bb.y * bb.y) / std::max(dot(bb, bb), 1e-20f), 0.0f, 1.0f);
  pp = pp - bb * float2(h, h - 1.0f);
  return length(pp) * sign_f(pp.x);
}

inline float sd_trapezoid_2d(const float2 &p, const float r1, const float r2, const float he)
{
  const float2 k1(r2, he);
  const float2 k2(r2 - r1, 2.0f * he);
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  const float2 ca(pp.x - std::min(pp.x, (pp.y < 0.0f) ? r1 : r2), std::fabs(pp.y) - he);
  const float t = clamp_f(dot(k1 - pp, k2) / std::max(dot(k2, k2), 1e-20f), 0.0f, 1.0f);
  const float2 cb = pp - k1 + k2 * t;
  const float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * std::sqrt(std::min(dot2(ca), dot2(cb)));
}

inline float sd_parallelogram_2d(const float2 &p, const float wi, const float he, const float sk)
{
  const float2 e(sk, he);
  float2 pp = (p.y < 0.0f) ? float2(-p.x, -p.y) : p;
  float2 w = pp - e;
  w.x -= clamp_f(w.x, -wi, wi);
  float2 d(dot(w, w), -w.y);
  const float s = pp.x * e.y - pp.y * e.x;
  pp = (s < 0.0f) ? float2(-pp.x, -pp.y) : pp;
  float2 v = pp - float2(wi, 0.0f);
  v = v - e * clamp_f(dot(v, e) / std::max(dot(e, e), 1e-20f), -1.0f, 1.0f);
  d = min2(d, float2(dot(v, v), wi * he - std::fabs(s)));
  return std::sqrt(d.x) * sign_f(-d.y);
}

inline float sd_equilateral_triangle_2d(const float2 &p, const float r)
{
  const float k = std::sqrt(3.0f);
  float2 pp = p;
  pp.x = std::fabs(pp.x) - r;
  pp.y = pp.y + r / k;
  if (pp.x + k * pp.y > 0.0f) {
    pp = float2(pp.x - k * pp.y, -k * pp.x - pp.y) * 0.5f;
  }
  pp.x -= clamp_f(pp.x, -2.0f * r, 0.0f);
  return -length(pp) * sign_f(pp.y);
}

inline float sd_triangle_isosceles_2d(const float2 &p, const float2 &q)
{
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  const float2 a = pp - q * clamp_f(dot(pp, q) / std::max(dot(q, q), 1e-20f), 0.0f, 1.0f);
  const float2 b = pp - q * float2(clamp_f(pp.x / std::max(q.x, 1e-20f), 0.0f, 1.0f), 1.0f);
  const float s = -sign_f(q.y);
  const float2 d = min2(float2(dot(a, a), s * (pp.x * q.y - pp.y * q.x)),
                        float2(dot(b, b), s * (pp.y - q.y)));
  return -std::sqrt(d.x) * sign_f(d.y);
}

inline float sd_triangle_2d(const float2 &p, const float2 &p0, const float2 &p1, const float2 &p2)
{
  const float2 e0 = p1 - p0, e1 = p2 - p1, e2 = p0 - p2;
  const float2 v0 = p - p0, v1 = p - p1, v2 = p - p2;
  const float2 pq0 = v0 - e0 * clamp_f(dot(v0, e0) / std::max(dot(e0, e0), 1e-20f), 0.0f, 1.0f);
  const float2 pq1 = v1 - e1 * clamp_f(dot(v1, e1) / std::max(dot(e1, e1), 1e-20f), 0.0f, 1.0f);
  const float2 pq2 = v2 - e2 * clamp_f(dot(v2, e2) / std::max(dot(e2, e2), 1e-20f), 0.0f, 1.0f);
  const float s = sign_f(e0.x * e2.y - e0.y * e2.x);
  float2 d = min2(float2(dot(pq0, pq0), s * (v0.x * e0.y - v0.y * e0.x)),
                  float2(dot(pq1, pq1), s * (v1.x * e1.y - v1.y * e1.x)));
  d = min2(d, float2(dot(pq2, pq2), s * (v2.x * e2.y - v2.y * e2.x)));
  return -std::sqrt(d.x) * sign_f(d.y);
}

inline float sd_uneven_capsule_2d(const float2 &p, const float r1, const float r2, const float h)
{
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  const float b = (r1 - r2) / std::max(h, 1e-8f);
  const float a = std::sqrt(std::max(1.0f - b * b, 0.0f));
  const float k = dot(pp, float2(-b, a));
  if (k < 0.0f) {
    return length(pp) - r1;
  }
  if (k > a * h) {
    return length(pp - float2(0.0f, h)) - r2;
  }
  return dot(pp, float2(a, b)) - r1;
}

inline float sd_pentagon_2d(const float2 &p, const float r)
{
  const float3 k(0.809016994f, 0.587785252f, 0.726542528f);
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  {
    const float2 n(-k.x, k.y);
    pp = pp - n * (2.0f * std::min(dot(n, pp), 0.0f));
  }
  {
    const float2 n(k.x, k.y);
    pp = pp - n * (2.0f * std::min(dot(n, pp), 0.0f));
  }
  pp = pp - float2(clamp_f(pp.x, -r * k.z, r * k.z), r);
  return length(pp) * sign_f(pp.y);
}

inline float sd_hexagon_2d(const float2 &p, const float r)
{
  const float3 k(-0.866025404f, 0.5f, 0.577350269f);
  float2 pp = abs2(p);
  pp = pp - float2(k.x, k.y) * (2.0f * std::min(dot(float2(k.x, k.y), pp), 0.0f));
  pp = pp - float2(clamp_f(pp.x, -k.z * r, k.z * r), r);
  return length(pp) * sign_f(pp.y);
}

inline float sd_octogon_2d(const float2 &p, const float r)
{
  const float3 k(-0.9238795325f, 0.3826834323f, 0.4142135623f);
  float2 pp = abs2(p);
  pp = pp - float2(k.x, k.y) * (2.0f * std::min(dot(float2(k.x, k.y), pp), 0.0f));
  pp = pp - float2(-k.x, k.y) * (2.0f * std::min(dot(float2(-k.x, k.y), pp), 0.0f));
  pp = pp - float2(clamp_f(pp.x, -k.z * r, k.z * r), r);
  return length(pp) * sign_f(pp.y);
}

inline float sd_hexagram_2d(const float2 &p, const float r)
{
  /* k = (-0.5, 0.8660254038, 0.5773502692, 1.7320508076) */
  float2 pp = abs2(p);
  pp = pp - float2(-0.5f, 0.8660254038f) *
                (2.0f * std::min(dot(float2(-0.5f, 0.8660254038f), pp), 0.0f));
  pp = pp - float2(0.8660254038f, -0.5f) *
                (2.0f * std::min(dot(float2(0.8660254038f, -0.5f), pp), 0.0f));
  pp = pp - float2(clamp_f(pp.x, r * 0.5773502692f, r * 1.7320508076f), r);
  return length(pp) * sign_f(pp.y);
}

inline float sd_star_2d(const float2 &p, const float r, const int n, const float m)
{
  const float an = 3.14159265f / float(std::max(n, 2));
  /* IQ: m interpolates n-gon (m=n) → pointy star (m→2+). m must be >2. */
  const float en = 3.14159265f / std::max(m, 2.0001f);
  const float2 acs(std::cos(an), std::sin(an));
  const float2 ecs(std::cos(en), std::sin(en));
  const float an2 = 2.0f * an;
  float bn = glsl_mod(std::atan2(p.x, p.y), an2) - an;
  float2 pp = float2(std::cos(bn), std::fabs(std::sin(bn))) * length(p);
  pp = pp - acs * r;
  pp = pp + ecs * clamp_f(-dot(pp, ecs), 0.0f, r * acs.y / std::max(ecs.y, 1e-8f));
  return length(pp) * sign_f(pp.x);
}

/** Pie: c = (sin, cos) of aperture half-angle. */
inline float sd_pie_2d(const float2 &p, const float2 &c, const float r)
{
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  const float l = length(pp) - r;
  const float m = length(pp - c * clamp_f(dot(pp, c), 0.0f, r));
  return std::max(l, m * sign_f(c.y * pp.x - c.x * pp.y));
}

inline float sd_cut_disk_2d(const float2 &p, const float r, const float h)
{
  const float w = std::sqrt(std::max(r * r - h * h, 0.0f));
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  const float s = std::max((h - r) * pp.x * pp.x + w * w * (h + r - 2.0f * pp.y),
                           h * pp.x - w * pp.y);
  return (s < 0.0f) ? (length(pp) - r) : ((pp.x < w) ? (h - pp.y) : length(pp - float2(w, h)));
}

/** Arc: sc=(sin,cos) aperture, ra radius, rb thickness. */
inline float sd_arc_2d(const float2 &p, const float2 &sc, const float ra, const float rb)
{
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  return ((sc.y * pp.x > sc.x * pp.y) ? length(pp - sc * ra) : std::fabs(length(pp) - ra)) - rb;
}

/** Ring: n=(sin,cos) of aperture, r radius, th thickness. */
inline float sd_ring_2d(const float2 &p, const float2 &n, const float r, const float th)
{
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  /* mat2(n.x,n.y,-n.y,n.x) * p */
  pp = float2(n.x * pp.x + n.y * pp.y, -n.y * pp.x + n.x * pp.y);
  return std::max(std::fabs(length(pp) - r) - th * 0.5f,
                  length(float2(pp.x, std::max(0.0f, std::fabs(r - pp.y) - th * 0.5f))) *
                      sign_f(pp.x));
}

inline float sd_horseshoe_2d(const float2 &p, const float2 &c, const float r, const float2 &w)
{
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  const float l = length(pp);
  /* mat2(-c.x, c.y, c.y, c.x) * p */
  pp = float2(-c.x * pp.x + c.y * pp.y, c.y * pp.x + c.x * pp.y);
  pp = float2((pp.y > 0.0f || pp.x > 0.0f) ? pp.x : l * sign_f(-c.x),
              (pp.x > 0.0f) ? pp.y : l);
  pp = float2(pp.x, std::fabs(pp.y - r)) - w;
  return length(max2(pp, float2(0.0f, 0.0f))) + std::min(0.0f, std::max(pp.x, pp.y));
}

inline float sd_vesica_2d(const float2 &p, const float w, const float h)
{
  const float d = 0.5f * (w * w - h * h) / std::max(h, 1e-8f);
  float2 pp = abs2(p);
  const float3 c = (w * pp.y < d * (pp.x - w)) ? float3(0.0f, w, 0.0f) : float3(-d, 0.0f, d + h);
  return length(pp - float2(c.y, c.x)) - c.z;
}

inline float sd_moon_2d(const float2 &p, const float d, const float ra, const float rb)
{
  float2 pp = p;
  pp.y = std::fabs(pp.y);
  const float a = (ra * ra - rb * rb + d * d) / (2.0f * std::max(d, 1e-8f));
  const float b = std::sqrt(std::max(ra * ra - a * a, 0.0f));
  if (d * (pp.x * b - pp.y * a) > d * d * std::max(b - pp.y, 0.0f)) {
    return length(pp - float2(a, b));
  }
  return std::max(length(pp) - ra, -(length(pp - float2(d, 0.0f)) - rb));
}

inline float sd_rounded_cross_2d(const float2 &p, const float h)
{
  const float k = 0.5f * (h + 1.0f / std::max(h, 1e-8f));
  float2 pp = abs2(p);
  if (pp.x < 1.0f && pp.y < pp.x * (k - h) + h) {
    return k - std::sqrt(dot2(pp - float2(1.0f, k)));
  }
  return std::sqrt(std::min(dot2(pp - float2(0.0f, h)), dot2(pp - float2(1.0f, 0.0f))));
}

inline float sd_heart_2d(const float2 &p)
{
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  if (pp.y + pp.x > 1.0f) {
    return std::sqrt(dot2(pp - float2(0.25f, 0.75f))) - std::sqrt(2.0f) / 4.0f;
  }
  return std::sqrt(std::min(dot2(pp - float2(0.0f, 1.0f)),
                            dot2(pp - float2(0.5f, 0.5f) * std::max(pp.x + pp.y, 0.0f)))) *
         sign_f(pp.x - pp.y);
}

inline float sd_cross_2d(const float2 &p, const float2 &b, const float r)
{
  float2 pp = abs2(p);
  if (pp.y > pp.x) {
    const float t = pp.x;
    pp.x = pp.y;
    pp.y = t;
  }
  const float2 q = pp - b;
  const float k = std::max(q.y, q.x);
  const float2 w = (k > 0.0f) ? q : float2(b.y - pp.x, -k);
  return sign_f(k) * length(max2(w, float2(0.0f, 0.0f))) + r;
}

inline float sd_rounded_x_2d(const float2 &p, const float w, const float r)
{
  float2 pp = abs2(p);
  return length(pp - float2(1.0f, 1.0f) * (std::min(pp.x + pp.y, w) * 0.5f)) - r;
}

inline float sd_ellipse_2d(const float2 &p, const float2 &ab)
{
  float2 pp = abs2(p);
  float2 abb = ab;
  if (pp.x > pp.y) {
    const float t = pp.x;
    pp.x = pp.y;
    pp.y = t;
    const float u = abb.x;
    abb.x = abb.y;
    abb.y = u;
  }
  const float l = abb.y * abb.y - abb.x * abb.x;
  if (std::fabs(l) < 1e-12f) {
    return length(pp) - abb.x;
  }
  const float m = abb.x * pp.x / l;
  const float m2 = m * m;
  const float n = abb.y * pp.y / l;
  const float n2 = n * n;
  const float c = (m2 + n2 - 1.0f) / 3.0f;
  const float c3 = c * c * c;
  const float q = c3 + m2 * n2 * 2.0f;
  const float d = c3 + m2 * n2;
  const float g = m + m * n2;
  float co;
  if (d < 0.0f) {
    const float h = std::acos(clamp_f(q / c3, -1.0f, 1.0f)) / 3.0f;
    const float s = std::cos(h);
    const float t = std::sin(h) * std::sqrt(3.0f);
    const float rx = std::sqrt(std::max(-c * (s + t + 2.0f) + m2, 0.0f));
    const float ry = std::sqrt(std::max(-c * (s - t + 2.0f) + m2, 0.0f));
    co = (ry + sign_f(l) * rx + std::fabs(g) / std::max(rx * ry, 1e-20f) - m) / 2.0f;
  }
  else {
    const float h = 2.0f * m * n * std::sqrt(std::max(d, 0.0f));
    const float s = sign_f(q + h) * std::pow(std::fabs(q + h), 1.0f / 3.0f);
    const float u = sign_f(q - h) * std::pow(std::fabs(q - h), 1.0f / 3.0f);
    const float rx = -s - u - c * 4.0f + 2.0f * m2;
    const float ry = (s - u) * std::sqrt(3.0f);
    const float rm = std::sqrt(rx * rx + ry * ry);
    co = (ry / std::sqrt(std::max(rm - rx, 1e-20f)) + 2.0f * g / std::max(rm, 1e-20f) - m) / 2.0f;
  }
  const float2 r = float2(abb.x * co, abb.y * std::sqrt(std::max(1.0f - co * co, 0.0f)));
  return length(r - pp) * sign_f(pp.y - r.y);
}

inline float sd_oriented_box_2d(const float2 &p, const float2 &a, const float2 &b, const float th)
{
  const float l = length(b - a);
  const float2 d = (b - a) * (1.0f / std::max(l, 1e-20f));
  float2 q = p - (a + b) * 0.5f;
  /* mat2(d.x,-d.y,d.y,d.x) * q */
  q = float2(d.x * q.x - d.y * q.y, d.y * q.x + d.x * q.y);
  q = abs2(q) - float2(l, th) * 0.5f;
  return length(max2(q, float2(0.0f, 0.0f))) + std::min(std::max(q.x, q.y), 0.0f);
}

inline float sd_tunnel_2d(const float2 &p, const float2 &wh)
{
  float2 pp = p;
  pp.x = std::fabs(pp.x);
  pp.y = -pp.y;
  float2 q = pp - wh;
  const float d1 = dot2(float2(std::max(q.x, 0.0f), q.y));
  q.x = (pp.y > 0.0f) ? q.x : (length(pp) - wh.x);
  const float d2 = dot2(float2(q.x, std::max(q.y, 0.0f)));
  const float d = std::sqrt(std::min(d1, d2));
  return (std::max(q.x, q.y) < 0.0f) ? -d : d;
}

inline float sd_stairs_2d(const float2 &p, const float2 &wh, const float n)
{
  const float2 ba = wh * n;
  float d = std::min(dot2(p - float2(clamp_f(p.x, 0.0f, ba.x), 0.0f)),
                     dot2(p - float2(ba.x, clamp_f(p.y, 0.0f, ba.y))));
  float s = sign_f(std::max(-p.y, p.x - ba.x));
  const float dia = length(wh);
  /* rotate by wh */
  float2 pp = float2(wh.x * p.x - wh.y * p.y, wh.y * p.x + wh.x * p.y) * (1.0f / std::max(dia, 1e-20f));
  const float id = clamp_f(std::round(pp.x / std::max(dia, 1e-20f)), 0.0f, n - 1.0f);
  pp.x = pp.x - id * dia;
  pp = float2(wh.x * pp.x + wh.y * pp.y, -wh.y * pp.x + wh.x * pp.y) * (1.0f / std::max(dia, 1e-20f));
  const float hh = wh.y * 0.5f;
  pp.y -= hh;
  if (pp.y > hh * sign_f(pp.x)) {
    s = 1.0f;
  }
  pp = (id < 0.5f || pp.x > 0.0f) ? pp : float2(-pp.x, -pp.y);
  d = std::min(d, dot2(pp - float2(0.0f, clamp_f(pp.y, -hh, hh))));
  d = std::min(d, dot2(pp - float2(clamp_f(pp.x, 0.0f, wh.x), hh)));
  return std::sqrt(d) * s;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Fractal distance estimators (IQ)
 * \{ */

/** Complex multiply. */
inline float2 c_mul(const float2 &a, const float2 &b)
{
  return {a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x};
}

/** Complex power z^p via polar form (multibrot / multi-Julia). */
inline float2 c_pow(const float2 &z, const float p)
{
  const float r = length(z);
  if (r < 1e-20f) {
    return float2(0.0f, 0.0f);
  }
  const float a = std::atan2(z.y, z.x);
  const float rp = std::pow(r, p);
  return float2(rp * std::cos(a * p), rp * std::sin(a * p));
}

/**
 * Escape radius for DE / smooth-iter.
 * User bailout is honored; clamped only for float32 safety after squaring.
 */
inline float fractal_escape2(const float bailout)
{
  const float b = std::max(bailout, 1.01f);
  return std::min(b * b, 1.0e12f);
}

/**
 * 2D Julia DE — IQ distancefractals with complex derivative, degree `power`.
 * f(z)=z^p+c,  z' = p z^{p-1} z'  (Julia, no +1).
 * d = 0.5 * |z| * log(|z|) / |z'|
 */
inline float de_julia_2d(const float2 &z0,
                         const float2 &c,
                         const int iterations,
                         const float bailout,
                         const float power = 2.0f)
{
  float2 z = z0;
  float2 dz(1.0f, 0.0f);
  const int iters = std::max(iterations, 1);
  const float pwr = std::max(power, 1.0f);
  const float escape2 = fractal_escape2(bailout);
  float m2 = dot(z, z);

  for (int i = 0; i < iters; i++) {
    /* z' = p * z^{p-1} * z' */
    const float2 zp1 = c_pow(z, pwr - 1.0f);
    dz = c_mul(zp1, dz) * pwr;
    z = c_pow(z, pwr) + c;
    m2 = dot(z, z);
    if (!(m2 < escape2)) {
      break;
    }
  }
  if (!(m2 > 1.0f) || !std::isfinite(m2)) {
    return 0.0f;
  }
  const float dz2 = std::max(dot(dz, dz), 1e-20f);
  if (!std::isfinite(dz2)) {
    return 0.0f;
  }
  /* For degree p: d = |z| log|z| / (|z'| * p) wait — IQ general uses 1/p in potential.
   * Traditional p=2: 0.5 * |z| log|z| / |dz| with log(|z|^2)=2log|z| → 0.5*log(m2).
   * General: |z| log|z| / (|dz| * p)  ==  0.5 * log(m2) * sqrt(m2) / (|dz| * p) */
  return std::sqrt(m2 / dz2) * (0.5f * std::log(m2)) / pwr;
}

/**
 * 2D Mandelbrot / multibrot DE — IQ: z' = p z^{p-1} z' + 1, z0=0.
 */
inline float de_mandelbrot_2d(const float2 &c,
                              const int iterations,
                              const float bailout,
                              const float power = 2.0f)
{
  float2 z(0.0f, 0.0f);
  float2 dz(0.0f, 0.0f);
  const int iters = std::max(iterations, 1);
  const float pwr = std::max(power, 1.0f);
  const float escape2 = fractal_escape2(bailout);
  float m2 = 0.0f;

  for (int i = 0; i < iters; i++) {
    const float2 zp1 = c_pow(z, pwr - 1.0f);
    dz = c_mul(zp1, dz) * pwr + float2(1.0f, 0.0f);
    z = c_pow(z, pwr) + c;
    m2 = dot(z, z);
    if (!(m2 < escape2)) {
      break;
    }
  }
  if (!(m2 > 1.0f) || !std::isfinite(m2)) {
    return 0.0f;
  }
  const float dz2 = std::max(dot(dz, dz), 1e-20f);
  if (!std::isfinite(dz2)) {
    return 0.0f;
  }
  return std::sqrt(m2 / dz2) * (0.5f * std::log(m2)) / pwr;
}

/**
 * Smooth iteration count (IQ msetsmooth), generalized to degree `power`.
 * Escape radius `B` = bailout (user-controlled).
 */
inline float smooth_iter_mandelbrot(const float2 &c,
                                    const int iterations,
                                    const float B = 256.0f,
                                    const float power = 2.0f)
{
  float2 z(0.0f, 0.0f);
  const int iters = std::max(iterations, 1);
  const float pwr = std::max(power, 1.0f);
  const float Bb = std::max(B, 2.0f);
  const float B2 = Bb * Bb;
  const float log_p = std::log(pwr);
  float n = 0.0f;
  for (int i = 0; i < iters; i++) {
    z = c_pow(z, pwr) + c;
    const float m2 = dot(z, z);
    if (m2 > B2) {
      /* n - log(log(|z|)/log(B)) / log(p) */
      const float lz = 0.5f * std::log(std::max(m2, 4.0f));
      return n - std::log(lz / std::log(Bb)) / log_p;
    }
    n += 1.0f;
  }
  return float(iters);
}

inline float smooth_iter_julia(const float2 &z0,
                               const float2 &c,
                               const int iterations,
                               const float B = 256.0f,
                               const float power = 2.0f)
{
  float2 z = z0;
  const int iters = std::max(iterations, 1);
  const float pwr = std::max(power, 1.0f);
  const float Bb = std::max(B, 2.0f);
  const float B2 = Bb * Bb;
  const float log_p = std::log(pwr);
  float n = 0.0f;
  for (int i = 0; i < iters; i++) {
    z = c_pow(z, pwr) + c;
    const float m2 = dot(z, z);
    if (m2 > B2) {
      const float lz = 0.5f * std::log(std::max(m2, 4.0f));
      return n - std::log(lz / std::log(Bb)) / log_p;
    }
    n += 1.0f;
  }
  return float(iters);
}

/** Mandelbulb DE (IQ / common spherical power). */
inline float de_mandelbulb(const float3 &pos,
                           const float power,
                           const int iterations,
                           const float bailout)
{
  float3 z = pos;
  float dr = 1.0f;
  float r = 0.0f;
  const float pwr = std::max(power, 1.0f);
  const int iters = std::max(iterations, 1);
  const float escape = std::max(bailout, 1.0f);

  for (int i = 0; i < iters; i++) {
    r = length(z);
    if (r > escape) {
      break;
    }
    const float theta = std::acos(clamp_f(z.z / std::max(r, 1e-20f), -1.0f, 1.0f));
    const float phi = std::atan2(z.y, z.x);
    dr = std::pow(r, pwr - 1.0f) * pwr * dr + 1.0f;
    const float zr = std::pow(r, pwr);
    const float new_theta = theta * pwr;
    const float new_phi = phi * pwr;
    z = float3(std::sin(new_theta) * std::cos(new_phi),
               std::sin(new_phi) * std::sin(new_theta),
               std::cos(new_theta)) *
        zr;
    z = z + pos;
  }
  r = std::max(r, 1e-20f);
  return 0.5f * std::log(r) * r / std::max(dr, 1e-20f);
}

/** Mandelbox DE (box fold + sphere fold). */
inline float de_mandelbox(const float3 &pos,
                          const float scale,
                          const int iterations,
                          const float bailout)
{
  float3 z = pos;
  float dr = 1.0f;
  const float s = (std::fabs(scale) < 1e-3f) ? -2.0f : scale;
  const int iters = std::max(iterations, 1);
  const float escape = std::max(bailout, 2.0f);
  const float fixed_radius2 = 1.0f;
  const float min_radius2 = 0.25f;

  for (int i = 0; i < iters; i++) {
    z.x = clamp_f(z.x, -1.0f, 1.0f) * 2.0f - z.x;
    z.y = clamp_f(z.y, -1.0f, 1.0f) * 2.0f - z.y;
    z.z = clamp_f(z.z, -1.0f, 1.0f) * 2.0f - z.z;

    const float r2 = dot(z, z);
    if (r2 < min_radius2) {
      const float t = fixed_radius2 / min_radius2;
      z = z * t;
      dr *= t;
    }
    else if (r2 < fixed_radius2) {
      const float t = fixed_radius2 / r2;
      z = z * t;
      dr *= t;
    }

    z = z * s + pos;
    dr = dr * std::fabs(s) + 1.0f;

    if (length(z) > escape * 4.0f) {
      break;
    }
  }
  return length(z) / std::max(std::fabs(dr), 1e-20f);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Shape / fractal kind dispatch
 * \{ */

/**
 * Shape catalog. Values 0–11 preserved for existing files.
 * 3D: 0–25, 2D: 32–63.
 */
enum class SDFShapeType : int {
  /* --- 3D (distfunctions) --- */
  Sphere = 0,
  Box = 1,
  Torus = 2,
  RoundBox = 3,
  BoxFrame = 4,
  Capsule = 5,
  CappedCylinder = 6,
  CappedCone = 7,
  Plane = 8,
  HexPrism = 9,
  Octahedron = 10,
  Pyramid = 11,
  CappedTorus = 12,
  Link = 13,
  InfiniteCylinder = 14,
  Cone = 15,
  RoundedCylinder = 16,
  SolidAngle = 17,
  CutSphere = 18,
  CutHollowSphere = 19,
  DeathStar = 20,
  RoundCone = 21,
  Rhombus3D = 22,
  Ellipsoid = 23,
  TriPrism = 24,
  InfiniteCone = 25,

  /* --- 2D (distfunctions2d) --- */
  Circle2D = 32,
  Box2D = 33,
  RoundedBox2D = 34,
  ChamferBox2D = 35,
  Segment2D = 36,
  Rhombus2D = 37,
  Trapezoid2D = 38,
  Parallelogram2D = 39,
  EquilateralTriangle2D = 40,
  IsoscelesTriangle2D = 41,
  Triangle2D = 42,
  UnevenCapsule2D = 43,
  Pentagon2D = 44,
  Hexagon2D = 45,
  Octogon2D = 46,
  Hexagram2D = 47,
  Star2D = 48,
  Pie2D = 49,
  CutDisk2D = 50,
  Arc2D = 51,
  Ring2D = 52,
  Horseshoe2D = 53,
  Vesica2D = 54,
  Moon2D = 55,
  Heart2D = 56,
  Cross2D = 57,
  RoundedX2D = 58,
  Ellipse2D = 59,
  OrientedBox2D = 60,
  Tunnel2D = 61,
  Stairs2D = 62,
  RoundedCross2D = 63,

  Count = 64,
};

enum class FractalPrimitiveType : int {
  Mandelbrot2D = 0,
  Julia2D = 1,
  Count = 2,
};

/**
 * Unified parameters. Unused fields are ignored per shape.
 *
 * Common mapping:
 * - radius / size / height / minor_radius / roundness / top_radius / offset: classic
 * - point_a, point_b, point_c: segment / triangle / oriented box endpoints
 * - angle: half-aperture radians (cone, pie, arc, solid angle, ring, horseshoe, capped torus)
 * - thickness: arc tube / ring / cut-hollow / cross pad
 * - count: star n / stairs steps
 * - factor: star m / chamfer / death-star separation / parallelogram skew / vesica height
 */
struct SDFShapeParams {
  float radius = 0.5f;
  float3 size = float3(0.5f, 0.5f, 0.5f);
  float height = 0.5f;
  float minor_radius = 0.15f;
  float roundness = 0.1f;
  float top_radius = 0.1f;
  float offset = 0.0f;
  float3 point_a = float3(-0.5f, 0.0f, 0.0f);
  float3 point_b = float3(0.5f, 0.0f, 0.0f);
  float3 point_c = float3(0.0f, 0.5f, 0.0f);
  float angle = 1.047198f;
  float thickness = 0.1f;
  float count = 5.0f;
  float factor = 3.0f;
};

inline float2 sc_from_angle(const float angle)
{
  /* Half-aperture. IQ pie/arc/cone/horseshoe crease if this exceeds π. */
  const float a = clamp_f(angle, 1.0e-4f, 3.14159265f - 1.0e-4f);
  return {std::sin(a), std::cos(a)};
}

inline float sdf_shape_distance(const SDFShapeType shape,
                                const float3 &p,
                                const SDFShapeParams &params)
{
  const float2 p2 = float2_from_xy(p);
  const float2 sc = sc_from_angle(params.angle);

  switch (shape) {
    /* ---- 3D ---- */
    case SDFShapeType::Box:
      return sd_box(p, params.size);
    case SDFShapeType::Torus:
      return sd_torus(p, params.radius, params.minor_radius);
    case SDFShapeType::RoundBox:
      return sd_round_box(p, params.size, params.roundness);
    case SDFShapeType::BoxFrame:
      return sd_box_frame(p, params.size, params.roundness);
    case SDFShapeType::Capsule:
      return sd_vertical_capsule(p, std::max(params.height, 0.0f), params.radius);
    case SDFShapeType::CappedCylinder:
      return sd_capped_cylinder(p, params.radius, std::max(params.height, 0.0f));
    case SDFShapeType::CappedCone:
      return sd_capped_cone(p,
                            std::max(params.height, 1e-4f),
                            params.radius,
                            std::max(params.top_radius, 0.0f));
    case SDFShapeType::Plane:
      return sd_plane(p, float3(0.0f, 1.0f, 0.0f), params.offset);
    case SDFShapeType::HexPrism:
      return sd_hex_prism(
          p, float2(std::max(params.radius, 1e-4f), std::max(params.height, 1e-4f)));
    case SDFShapeType::Octahedron:
      return sd_octahedron(p, std::max(params.radius, 1e-4f));
    case SDFShapeType::Pyramid:
      return sd_pyramid(p, std::max(params.height, 1e-4f));
    case SDFShapeType::CappedTorus:
      return sd_capped_torus(p, sc, params.radius, params.minor_radius);
    case SDFShapeType::Link:
      return sd_link(p, params.height * 0.5f, params.radius, params.minor_radius);
    case SDFShapeType::InfiniteCylinder:
      return sd_infinite_cylinder(p, float2(0.0f, 0.0f), params.radius);
    case SDFShapeType::Cone:
      return sd_cone(p, sc, std::max(params.height, 1e-4f));
    case SDFShapeType::RoundedCylinder:
      return sd_rounded_cylinder(
          p, params.radius, params.roundness, std::max(params.height, 0.0f));
    case SDFShapeType::SolidAngle:
      return sd_solid_angle(p, sc, params.radius);
    case SDFShapeType::CutSphere:
      return sd_cut_sphere(
          p, params.radius, clamp_f(params.height, -params.radius * 0.99f, params.radius * 0.99f));
    case SDFShapeType::CutHollowSphere:
      return sd_cut_hollow_sphere(
          p,
          params.radius,
          clamp_f(params.height, -params.radius * 0.99f, params.radius * 0.99f),
          params.thickness);
    case SDFShapeType::DeathStar:
      return sd_death_star(
          p,
          params.radius,
          params.minor_radius,
          clamp_f(params.factor, 1e-4f, params.radius + params.minor_radius));
    case SDFShapeType::RoundCone:
      return sd_round_cone(
          p, params.radius, params.top_radius, std::max(params.height, 1e-4f));
    case SDFShapeType::Rhombus3D:
      return sd_rhombus_3d(p, params.size.x, params.size.z, params.height, params.roundness);
    case SDFShapeType::Ellipsoid:
      return sd_ellipsoid(p, float3(std::max(params.size.x, 1e-4f),
                                    std::max(params.size.y, 1e-4f),
                                    std::max(params.size.z, 1e-4f)));
    case SDFShapeType::TriPrism:
      return sd_tri_prism(
          p, float2(std::max(params.radius, 1e-4f), std::max(params.height, 1e-4f)));
    case SDFShapeType::InfiniteCone:
      return sd_infinite_cone(p, sc);

    /* ---- 2D ---- */
    case SDFShapeType::Circle2D:
      return sd_circle(p2, params.radius);
    case SDFShapeType::Box2D:
      return sd_box_2d(p2, float2(params.size.x, params.size.y));
    case SDFShapeType::RoundedBox2D:
      return sd_rounded_box_2d(p2, float2(params.size.x, params.size.y), params.roundness);
    case SDFShapeType::ChamferBox2D:
      return sd_chamfer_box_2d(
          p2,
          float2(params.size.x, params.size.y),
          clamp_f(params.factor, 0.0f, std::min(params.size.x, params.size.y)));
    case SDFShapeType::Segment2D:
      return sd_segment_2d(p2, float2(params.point_a.x, params.point_a.y),
                           float2(params.point_b.x, params.point_b.y));
    case SDFShapeType::Rhombus2D:
      return sd_rhombus_2d(p2, float2(params.size.x, params.size.y));
    case SDFShapeType::Trapezoid2D:
      return sd_trapezoid_2d(p2, params.radius, params.top_radius, params.height);
    case SDFShapeType::Parallelogram2D:
      return sd_parallelogram_2d(p2, params.size.x, params.height, params.factor);
    case SDFShapeType::EquilateralTriangle2D:
      return sd_equilateral_triangle_2d(p2, params.radius);
    case SDFShapeType::IsoscelesTriangle2D:
      return sd_triangle_isosceles_2d(p2, float2(params.size.x, params.height));
    case SDFShapeType::Triangle2D:
      return sd_triangle_2d(p2,
                            float2(params.point_a.x, params.point_a.y),
                            float2(params.point_b.x, params.point_b.y),
                            float2(params.point_c.x, params.point_c.y));
    case SDFShapeType::UnevenCapsule2D:
      return sd_uneven_capsule_2d(p2, params.radius, params.top_radius, params.height);
    case SDFShapeType::Pentagon2D:
      return sd_pentagon_2d(p2, params.radius);
    case SDFShapeType::Hexagon2D:
      return sd_hexagon_2d(p2, params.radius);
    case SDFShapeType::Octogon2D:
      return sd_octogon_2d(p2, params.radius);
    case SDFShapeType::Hexagram2D:
      return sd_hexagram_2d(p2, params.radius);
    case SDFShapeType::Star2D:
      return sd_star_2d(p2, params.radius, int(std::max(params.count, 2.0f)), params.factor);
    case SDFShapeType::Pie2D:
      return sd_pie_2d(p2, sc, params.radius);
    case SDFShapeType::CutDisk2D:
      return sd_cut_disk_2d(
          p2, params.radius, clamp_f(params.height, -params.radius * 0.99f, params.radius * 0.99f));
    case SDFShapeType::Arc2D:
      return sd_arc_2d(p2, sc, params.radius, params.thickness);
    case SDFShapeType::Ring2D:
      return sd_ring_2d(p2, sc, params.radius, params.thickness);
    case SDFShapeType::Horseshoe2D:
      return sd_horseshoe_2d(
          p2, sc, params.radius, float2(params.size.x, params.thickness));
    case SDFShapeType::Vesica2D:
      return sd_vesica_2d(
          p2, params.radius, clamp_f(params.factor, 1e-4f, params.radius * 0.95f));
    case SDFShapeType::Moon2D:
      return sd_moon_2d(p2,
                        clamp_f(params.factor, 1e-4f, params.radius + params.minor_radius),
                        params.radius,
                        params.minor_radius);
    case SDFShapeType::Heart2D:
      return sd_heart_2d(p2 * (1.0f / std::max(params.radius, 1e-4f))) * params.radius;
    case SDFShapeType::Cross2D:
      return sd_cross_2d(p2, float2(params.size.x, params.size.y), params.thickness);
    case SDFShapeType::RoundedX2D:
      return sd_rounded_x_2d(p2, params.radius, params.thickness);
    case SDFShapeType::Ellipse2D:
      return sd_ellipse_2d(p2, float2(std::max(params.size.x, 1e-4f), std::max(params.size.y, 1e-4f)));
    case SDFShapeType::OrientedBox2D:
      return sd_oriented_box_2d(p2,
                                float2(params.point_a.x, params.point_a.y),
                                float2(params.point_b.x, params.point_b.y),
                                params.thickness);
    case SDFShapeType::Tunnel2D:
      return sd_tunnel_2d(p2, float2(params.size.x, params.size.y));
    case SDFShapeType::Stairs2D:
      return sd_stairs_2d(p2, float2(params.size.x, params.size.y), std::max(params.count, 1.0f));
    case SDFShapeType::RoundedCross2D:
      return sd_rounded_cross_2d(p2, std::max(params.height, 1e-4f));

    case SDFShapeType::Sphere:
    default:
      return sd_sphere(p, params.radius);
  }
}

/** Legacy 3-param bridge. */
inline float sdf_shape_distance(const SDFShapeType shape,
                                const float3 &p,
                                const float radius,
                                const float3 &size,
                                const float thickness)
{
  SDFShapeParams params;
  params.radius = radius;
  params.size = size;
  params.height = thickness;
  params.minor_radius = thickness;
  params.roundness = thickness;
  params.top_radius = size.x;
  params.offset = radius;
  params.thickness = thickness;
  return sdf_shape_distance(shape, p, params);
}

inline bool sdf_is_2d(const SDFShapeType s)
{
  return int(s) >= int(SDFShapeType::Circle2D);
}

inline bool sdf_uses_radius(const SDFShapeType s)
{
  switch (s) {
    case SDFShapeType::Sphere:
    case SDFShapeType::Torus:
    case SDFShapeType::Capsule:
    case SDFShapeType::CappedCylinder:
    case SDFShapeType::CappedCone:
    case SDFShapeType::HexPrism:
    case SDFShapeType::Octahedron:
    case SDFShapeType::CappedTorus:
    case SDFShapeType::Link:
    case SDFShapeType::InfiniteCylinder:
    case SDFShapeType::RoundedCylinder:
    case SDFShapeType::SolidAngle:
    case SDFShapeType::CutSphere:
    case SDFShapeType::CutHollowSphere:
    case SDFShapeType::DeathStar:
    case SDFShapeType::RoundCone:
    case SDFShapeType::TriPrism:
    case SDFShapeType::Circle2D:
    case SDFShapeType::Trapezoid2D:
    case SDFShapeType::EquilateralTriangle2D:
    case SDFShapeType::UnevenCapsule2D:
    case SDFShapeType::Pentagon2D:
    case SDFShapeType::Hexagon2D:
    case SDFShapeType::Octogon2D:
    case SDFShapeType::Hexagram2D:
    case SDFShapeType::Star2D:
    case SDFShapeType::Pie2D:
    case SDFShapeType::CutDisk2D:
    case SDFShapeType::Arc2D:
    case SDFShapeType::Ring2D:
    case SDFShapeType::Horseshoe2D:
    case SDFShapeType::Vesica2D:
    case SDFShapeType::Moon2D:
    case SDFShapeType::Heart2D:
    case SDFShapeType::RoundedX2D:
      return true;
    default:
      return false;
  }
}
inline bool sdf_uses_size(const SDFShapeType s)
{
  switch (s) {
    case SDFShapeType::Box:
    case SDFShapeType::RoundBox:
    case SDFShapeType::BoxFrame:
    case SDFShapeType::Rhombus3D:
    case SDFShapeType::Ellipsoid:
    case SDFShapeType::Box2D:
    case SDFShapeType::RoundedBox2D:
    case SDFShapeType::ChamferBox2D:
    case SDFShapeType::Rhombus2D:
    case SDFShapeType::Parallelogram2D:
    case SDFShapeType::IsoscelesTriangle2D:
    case SDFShapeType::Horseshoe2D:
    case SDFShapeType::Cross2D:
    case SDFShapeType::Ellipse2D:
    case SDFShapeType::Tunnel2D:
    case SDFShapeType::Stairs2D:
      return true;
    default:
      return false;
  }
}
inline bool sdf_uses_height(const SDFShapeType s)
{
  switch (s) {
    case SDFShapeType::Capsule:
    case SDFShapeType::CappedCylinder:
    case SDFShapeType::CappedCone:
    case SDFShapeType::HexPrism:
    case SDFShapeType::Pyramid:
    case SDFShapeType::Link:
    case SDFShapeType::Cone:
    case SDFShapeType::RoundedCylinder:
    case SDFShapeType::CutSphere:
    case SDFShapeType::CutHollowSphere:
    case SDFShapeType::RoundCone:
    case SDFShapeType::Rhombus3D:
    case SDFShapeType::TriPrism:
    case SDFShapeType::Trapezoid2D:
    case SDFShapeType::Parallelogram2D:
    case SDFShapeType::IsoscelesTriangle2D:
    case SDFShapeType::UnevenCapsule2D:
    case SDFShapeType::CutDisk2D:
    case SDFShapeType::RoundedCross2D:
      return true;
    default:
      return false;
  }
}
inline bool sdf_uses_minor_radius(const SDFShapeType s)
{
  return s == SDFShapeType::Torus || s == SDFShapeType::CappedTorus || s == SDFShapeType::Link ||
         s == SDFShapeType::DeathStar || s == SDFShapeType::Moon2D;
}
inline bool sdf_uses_roundness(const SDFShapeType s)
{
  return s == SDFShapeType::RoundBox || s == SDFShapeType::BoxFrame ||
         s == SDFShapeType::RoundedCylinder || s == SDFShapeType::Rhombus3D ||
         s == SDFShapeType::RoundedBox2D;
}
inline bool sdf_uses_top_radius(const SDFShapeType s)
{
  return s == SDFShapeType::CappedCone || s == SDFShapeType::RoundCone ||
         s == SDFShapeType::Trapezoid2D || s == SDFShapeType::UnevenCapsule2D;
}
inline bool sdf_uses_offset(const SDFShapeType s)
{
  return s == SDFShapeType::Plane;
}
inline bool sdf_uses_point_a(const SDFShapeType s)
{
  return s == SDFShapeType::Segment2D || s == SDFShapeType::Triangle2D ||
         s == SDFShapeType::OrientedBox2D;
}
inline bool sdf_uses_point_b(const SDFShapeType s)
{
  return sdf_uses_point_a(s);
}
inline bool sdf_uses_point_c(const SDFShapeType s)
{
  return s == SDFShapeType::Triangle2D;
}
inline bool sdf_uses_angle(const SDFShapeType s)
{
  return s == SDFShapeType::CappedTorus || s == SDFShapeType::Cone ||
         s == SDFShapeType::SolidAngle || s == SDFShapeType::InfiniteCone ||
         s == SDFShapeType::Pie2D || s == SDFShapeType::Arc2D || s == SDFShapeType::Ring2D ||
         s == SDFShapeType::Horseshoe2D;
}
inline bool sdf_uses_thickness(const SDFShapeType s)
{
  return s == SDFShapeType::CutHollowSphere || s == SDFShapeType::Arc2D ||
         s == SDFShapeType::Ring2D || s == SDFShapeType::Horseshoe2D || s == SDFShapeType::Cross2D ||
         s == SDFShapeType::RoundedX2D || s == SDFShapeType::OrientedBox2D;
}
inline bool sdf_uses_count(const SDFShapeType s)
{
  return s == SDFShapeType::Star2D || s == SDFShapeType::Stairs2D;
}
inline bool sdf_uses_factor(const SDFShapeType s)
{
  return s == SDFShapeType::DeathStar || s == SDFShapeType::ChamferBox2D ||
         s == SDFShapeType::Parallelogram2D || s == SDFShapeType::Star2D ||
         s == SDFShapeType::Vesica2D || s == SDFShapeType::Moon2D;
}

inline float fractal_primitive_distance(const FractalPrimitiveType kind,
                                        const float3 &p,
                                        const float power,
                                        const int iterations,
                                        const float bailout,
                                        const float2 &julia_c = float2(-0.8f, 0.156f))
{
  switch (kind) {
    case FractalPrimitiveType::Julia2D:
      return de_julia_2d(float2(p.x, p.y), julia_c, iterations, bailout, power);
    case FractalPrimitiveType::Mandelbrot2D:
    default:
      return de_mandelbrot_2d(float2(p.x, p.y), iterations, bailout, power);
  }
}

/** Map signed distance to a high-contrast RGB color for viewers / EEVEE. */
inline float3 distance_to_color(const float d, const float viz_scale = 2.0f)
{
  const float a = std::fabs(d) * std::max(viz_scale, 1e-3f);
  const float band = 0.5f + 0.5f * std::cos(40.0f * d);
  float3 col = (d < 0.0f) ? float3(0.2f, 0.55f, 0.95f) : float3(0.95f, 0.55f, 0.2f);
  const float fade = 1.0f - std::exp(-4.0f * a);
  col = col * fade;
  col = col * (0.75f + 0.25f * band);
  const float edge = std::exp(-80.0f * d * d);
  col = col * (1.0f - 0.55f * edge) + float3(1.0f) * (0.55f * edge);
  return col;
}

/**
 * Color from smooth iteration (2D Mandelbrot/Julia) — IQ continuous banding.
 * Interior (iter >= max) is dark.
 */
inline float3 smooth_iter_to_color(const float iter, const float max_iter)
{
  if (iter >= max_iter - 0.5f) {
    return float3(0.02f, 0.02f, 0.05f);
  }
  const float t = iter * 0.05f;
  return float3(0.5f + 0.5f * std::cos(6.2831f * (t + 0.0f)),
                0.5f + 0.5f * std::cos(6.2831f * (t + 0.15f)),
                0.5f + 0.5f * std::cos(6.2831f * (t + 0.35f)));
}

template<typename Vec3> inline float3 to_sdf_float3(const Vec3 &v)
{
  return float3(v.x, v.y, v.z);
}

inline constexpr int sdf_shape_count()
{
  return int(SDFShapeType::Count);
}
inline constexpr int fractal_primitive_count()
{
  return int(FractalPrimitiveType::Count);
}

/** Valid shape indices (skips gaps between 3D and 2D blocks). */
inline bool sdf_shape_index_valid(const int s)
{
  return (s >= 0 && s <= int(SDFShapeType::InfiniteCone)) ||
         (s >= int(SDFShapeType::Circle2D) && s <= int(SDFShapeType::RoundedCross2D));
}

/** \} */

}  // namespace blender::nodes::sdf_math
