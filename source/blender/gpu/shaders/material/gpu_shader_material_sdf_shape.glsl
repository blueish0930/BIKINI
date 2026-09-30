/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* IQ exact SDFs:
 *   https://iquilezles.org/articles/distfunctions/
 *   https://iquilezles.org/articles/distfunctions2d/
 */

#include "gpu_shader_material_transform_utils.bsl.hh"

float sd_sphere_iq(float3 p, float r)
{
  return length(p) - r;
}
float sd_box_iq(float3 p, float3 b)
{
  float3 q = abs(p) - b;
  return length(max(q, float3(0.0f))) + min(max(q.x, max(q.y, q.z)), 0.0f);
}
float sd_torus_iq(float3 p, float major_r, float minor_r)
{
  float2 q = float2(length(p.xz) - major_r, p.y);
  return length(q) - minor_r;
}
float sd_round_box_iq(float3 p, float3 b, float r)
{
  float3 q = abs(p) - b + r;
  return length(max(q, float3(0.0f))) + min(max(q.x, max(q.y, q.z)), 0.0f) - r;
}
float sd_box_frame_iq(float3 p, float3 b, float e)
{
  p = abs(p) - b;
  float3 q = abs(p + e) - e;
  return min(min(length(max(float3(p.x, q.y, q.z), float3(0.0f))) +
                     min(max(p.x, max(q.y, q.z)), 0.0f),
                 length(max(float3(q.x, p.y, q.z), float3(0.0f))) +
                     min(max(q.x, max(p.y, q.z)), 0.0f)),
             length(max(float3(q.x, q.y, p.z), float3(0.0f))) +
                 min(max(q.x, max(q.y, p.z)), 0.0f));
}
float sd_vertical_capsule_iq(float3 p, float h, float r)
{
  float half_h = max(h, 0.0f) * 0.5f;
  p.y -= clamp(p.y, -half_h, half_h);
  return length(p) - r;
}
float sd_capped_cylinder_iq(float3 p, float r, float h)
{
  float2 d = abs(float2(length(p.xz), p.y)) - float2(r, h);
  return min(max(d.x, d.y), 0.0f) + length(max(d, float2(0.0f)));
}
float sd_capped_cone_iq(float3 p, float h, float r1, float r2)
{
  float2 q = float2(length(p.xz), p.y);
  float2 k1 = float2(r2, h);
  float2 k2 = float2(r2 - r1, 2.0f * h);
  float2 ca = float2(q.x - min(q.x, (q.y < 0.0f) ? r1 : r2), abs(q.y) - h);
  float2 cb = q - k1 + k2 * clamp(dot(k1 - q, k2) / max(dot(k2, k2), 1e-20f), 0.0f, 1.0f);
  float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * sqrt(min(dot(ca, ca), dot(cb, cb)));
}
float sd_plane_iq(float3 p, float h)
{
  return p.y + h;
}
float sd_hex_prism_iq(float3 p, float2 h)
{
  float3 k = float3(-0.8660254f, 0.5f, 0.57735f);
  p = abs(p);
  p.xy -= 2.0f * min(dot(k.xy, p.xy), 0.0f) * k.xy;
  float2 d = float2(length(float2(p.x - clamp(p.x, -k.z * h.x, k.z * h.x), p.y - h.x)) *
                        sign(p.y - h.x),
                    p.z - h.y);
  return min(max(d.x, d.y), 0.0f) + length(max(d, float2(0.0f)));
}
float sd_octahedron_iq(float3 p, float s)
{
  p = abs(p);
  float m = p.x + p.y + p.z - s;
  float3 q;
  if (3.0f * p.x < m) {
    q = p.xyz;
  }
  else if (3.0f * p.y < m) {
    q = p.yzx;
  }
  else if (3.0f * p.z < m) {
    q = p.zxy;
  }
  else {
    return m * 0.57735027f;
  }
  float k = clamp(0.5f * (q.z - q.y + s), 0.0f, s);
  return length(float3(q.x, q.y - s + k, q.z - k));
}
float sd_pyramid_iq(float3 p, float h)
{
  float m2 = h * h + 0.25f;
  p.xz = abs(p.xz);
  p.xz = (p.z > p.x) ? p.zx : p.xz;
  p.xz -= 0.5f;
  float3 q = float3(p.z, h * p.y - 0.5f * p.x, h * p.x + 0.5f * p.y);
  float s = max(-q.x, 0.0f);
  float t = clamp((q.y - 0.5f * q.z) / (m2 + 0.25f), 0.0f, 1.0f);
  float a = m2 * (q.x + s) * (q.x + s) + q.y * q.y;
  float b = m2 * (q.x + 0.5f * t) * (q.x + 0.5f * t) + (q.y - m2 * t) * (q.y - m2 * t);
  float d2 = min(q.y, -q.x * m2 - q.y * 0.5f) > 0.0f ? 0.0f : min(a, b);
  return sqrt((d2 + q.z * q.z) / m2) * sign(max(q.z, -p.y));
}
float sd_capped_torus_iq(float3 p, float2 sc, float ra, float rb)
{
  p.x = abs(p.x);
  float k = (sc.y * p.x > sc.x * p.y) ? dot(p.xy, sc) : length(p.xy);
  return sqrt(dot(p, p) + ra * ra - 2.0f * ra * k) - rb;
}
float sd_link_iq(float3 p, float le, float r1, float r2)
{
  float3 q = float3(p.x, max(abs(p.y) - le, 0.0f), p.z);
  return length(float2(length(q.xy) - r1, q.z)) - r2;
}
float sd_infinite_cylinder_iq(float3 p, float r)
{
  return length(p.xz) - r;
}
float sd_cone_iq(float3 p, float2 c, float h)
{
  float2 q = h * float2(c.x / max(c.y, 1e-8f), -1.0f);
  float2 w = float2(length(p.xz), p.y);
  float2 a = w - q * clamp(dot(w, q) / max(dot(q, q), 1e-20f), 0.0f, 1.0f);
  float2 b = w - q * float2(clamp(w.x / max(q.x, 1e-20f), 0.0f, 1.0f), 1.0f);
  float k = sign(q.y);
  float d = min(dot(a, a), dot(b, b));
  float s = max(k * (w.x * q.y - w.y * q.x), k * (w.y - q.y));
  return sqrt(d) * sign(s);
}
float sd_infinite_cone_iq(float3 p, float2 c)
{
  float2 q = float2(length(p.xz), -p.y);
  float d = length(q - c * max(dot(q, c), 0.0f));
  return d * ((q.x * c.y - q.y * c.x < 0.0f) ? -1.0f : 1.0f);
}
float sd_rounded_cylinder_iq(float3 p, float ra, float rb, float h)
{
  float2 d = float2(length(p.xz) - ra + rb, abs(p.y) - h + rb);
  return min(max(d.x, d.y), 0.0f) + length(max(d, float2(0.0f))) - rb;
}
float sd_solid_angle_iq(float3 p, float2 c, float ra)
{
  float2 q = float2(length(p.xz), p.y);
  float l = length(q) - ra;
  float m = length(q - c * clamp(dot(q, c), 0.0f, ra));
  return max(l, m * sign(c.y * q.x - c.x * q.y));
}
float sd_cut_sphere_iq(float3 p, float r, float h)
{
  float w = sqrt(max(r * r - h * h, 0.0f));
  float2 q = float2(length(p.xz), p.y);
  float s = max((h - r) * q.x * q.x + w * w * (h + r - 2.0f * q.y), h * q.x - w * q.y);
  return (s < 0.0f) ? length(q) - r : ((q.x < w) ? h - q.y : length(q - float2(w, h)));
}
float sd_cut_hollow_sphere_iq(float3 p, float r, float h, float t)
{
  float w = sqrt(max(r * r - h * h, 0.0f));
  float2 q = float2(length(p.xz), p.y);
  return ((h * q.x < w * q.y) ? length(q - float2(w, h)) : abs(length(q) - r)) - t;
}
float sd_death_star_iq(float3 p2, float ra, float rb, float d)
{
  float a = (ra * ra - rb * rb + d * d) / (2.0f * max(d, 1e-8f));
  float b = sqrt(max(ra * ra - a * a, 0.0f));
  float2 p = float2(p2.x, length(p2.yz));
  if (p.x * b - p.y * a > d * max(b - p.y, 0.0f)) {
    return length(p - float2(a, b));
  }
  return max(length(p) - ra, -(length(p - float2(d, 0.0f)) - rb));
}
float sd_round_cone_iq(float3 p, float r1, float r2, float h)
{
  float b = (r1 - r2) / max(h, 1e-8f);
  float a = sqrt(max(1.0f - b * b, 0.0f));
  float2 q = float2(length(p.xz), p.y);
  float k = dot(q, float2(-b, a));
  if (k < 0.0f) {
    return length(q) - r1;
  }
  if (k > a * h) {
    return length(q - float2(0.0f, h)) - r2;
  }
  return dot(q, float2(a, b)) - r1;
}
float sd_rhombus_3d_iq(float3 p, float la, float lb, float h, float ra)
{
  p = abs(p);
  float f = clamp((la * p.x - lb * p.z + lb * lb) / max(la * la + lb * lb, 1e-20f), 0.0f, 1.0f);
  float2 w = p.xz - float2(la, lb) * float2(f, 1.0f - f);
  float2 q = float2(length(w) * sign(w.x) - ra, p.y - h);
  return min(max(q.x, q.y), 0.0f) + length(max(q, float2(0.0f)));
}
float sd_ellipsoid_iq(float3 p, float3 r)
{
  float k0 = length(p / r);
  float k1 = length(p / (r * r));
  return k0 * (k0 - 1.0f) / max(k1, 1e-20f);
}
float sd_tri_prism_iq(float3 p, float2 h)
{
  float3 q = abs(p);
  return max(q.z - h.y, max(q.x * 0.866025f + p.y * 0.5f, -p.y) - h.x * 0.5f);
}

/* ---- 2D ---- */
float sd_circle_2d_iq(float2 p, float r)
{
  return length(p) - r;
}
float sd_box_2d_iq(float2 p, float2 b)
{
  float2 d = abs(p) - b;
  return length(max(d, float2(0.0f))) + min(max(d.x, d.y), 0.0f);
}
float sd_rounded_box_2d_iq(float2 p, float2 b, float r)
{
  float2 q = abs(p) - b + r;
  return min(max(q.x, q.y), 0.0f) + length(max(q, float2(0.0f))) - r;
}
float sd_chamfer_box_2d_iq(float2 p, float2 b, float chamfer)
{
  p = abs(p) - b;
  p = (p.y > p.x) ? p.yx : p.xy;
  p.y += chamfer;
  float k = 1.0f - sqrt(2.0f);
  if (p.y < 0.0f && p.y + p.x * k < 0.0f) {
    return p.x;
  }
  if (p.x < p.y) {
    return (p.x + p.y) * sqrt(0.5f);
  }
  return length(p);
}
float sd_segment_2d_iq(float2 p, float2 a, float2 b)
{
  float2 pa = p - a, ba = b - a;
  float h = clamp(dot(pa, ba) / max(dot(ba, ba), 1e-20f), 0.0f, 1.0f);
  return length(pa - ba * h);
}
float sd_rhombus_2d_iq(float2 p, float2 b)
{
  b.y = -b.y;
  p = abs(p);
  float h = clamp((dot(b, p) + b.y * b.y) / max(dot(b, b), 1e-20f), 0.0f, 1.0f);
  p -= b * float2(h, h - 1.0f);
  return length(p) * sign(p.x);
}
float sd_trapezoid_2d_iq(float2 p, float r1, float r2, float he)
{
  float2 k1 = float2(r2, he);
  float2 k2 = float2(r2 - r1, 2.0f * he);
  p.x = abs(p.x);
  float2 ca = float2(p.x - min(p.x, (p.y < 0.0f) ? r1 : r2), abs(p.y) - he);
  float2 cb = p - k1 + k2 * clamp(dot(k1 - p, k2) / max(dot(k2, k2), 1e-20f), 0.0f, 1.0f);
  float s = (cb.x < 0.0f && ca.y < 0.0f) ? -1.0f : 1.0f;
  return s * sqrt(min(dot(ca, ca), dot(cb, cb)));
}
float sd_parallelogram_2d_iq(float2 p, float wi, float he, float sk)
{
  float2 e = float2(sk, he);
  p = (p.y < 0.0f) ? -p : p;
  float2 w = p - e;
  w.x -= clamp(w.x, -wi, wi);
  float2 d = float2(dot(w, w), -w.y);
  float s = p.x * e.y - p.y * e.x;
  p = (s < 0.0f) ? -p : p;
  float2 v = p - float2(wi, 0.0f);
  v -= e * clamp(dot(v, e) / max(dot(e, e), 1e-20f), -1.0f, 1.0f);
  d = min(d, float2(dot(v, v), wi * he - abs(s)));
  return sqrt(d.x) * sign(-d.y);
}
float sd_equilateral_triangle_2d_iq(float2 p, float r)
{
  float k = sqrt(3.0f);
  p.x = abs(p.x) - r;
  p.y = p.y + r / k;
  if (p.x + k * p.y > 0.0f) {
    p = float2(p.x - k * p.y, -k * p.x - p.y) / 2.0f;
  }
  p.x -= clamp(p.x, -2.0f * r, 0.0f);
  return -length(p) * sign(p.y);
}
float sd_triangle_isosceles_2d_iq(float2 p, float2 q)
{
  p.x = abs(p.x);
  float2 a = p - q * clamp(dot(p, q) / max(dot(q, q), 1e-20f), 0.0f, 1.0f);
  float2 b = p - q * float2(clamp(p.x / max(q.x, 1e-20f), 0.0f, 1.0f), 1.0f);
  float s = -sign(q.y);
  float2 d = min(float2(dot(a, a), s * (p.x * q.y - p.y * q.x)),
                 float2(dot(b, b), s * (p.y - q.y)));
  return -sqrt(d.x) * sign(d.y);
}
float sd_triangle_2d_iq(float2 p, float2 p0, float2 p1, float2 p2)
{
  float2 e0 = p1 - p0, e1 = p2 - p1, e2 = p0 - p2;
  float2 v0 = p - p0, v1 = p - p1, v2 = p - p2;
  float2 pq0 = v0 - e0 * clamp(dot(v0, e0) / max(dot(e0, e0), 1e-20f), 0.0f, 1.0f);
  float2 pq1 = v1 - e1 * clamp(dot(v1, e1) / max(dot(e1, e1), 1e-20f), 0.0f, 1.0f);
  float2 pq2 = v2 - e2 * clamp(dot(v2, e2) / max(dot(e2, e2), 1e-20f), 0.0f, 1.0f);
  float s = sign(e0.x * e2.y - e0.y * e2.x);
  float2 d = min(min(float2(dot(pq0, pq0), s * (v0.x * e0.y - v0.y * e0.x)),
                     float2(dot(pq1, pq1), s * (v1.x * e1.y - v1.y * e1.x))),
                 float2(dot(pq2, pq2), s * (v2.x * e2.y - v2.y * e2.x)));
  return -sqrt(d.x) * sign(d.y);
}
float sd_uneven_capsule_2d_iq(float2 p, float r1, float r2, float h)
{
  p.x = abs(p.x);
  float b = (r1 - r2) / max(h, 1e-8f);
  float a = sqrt(max(1.0f - b * b, 0.0f));
  float k = dot(p, float2(-b, a));
  if (k < 0.0f) {
    return length(p) - r1;
  }
  if (k > a * h) {
    return length(p - float2(0.0f, h)) - r2;
  }
  return dot(p, float2(a, b)) - r1;
}
float sd_pentagon_2d_iq(float2 p, float r)
{
  float3 k = float3(0.809016994f, 0.587785252f, 0.726542528f);
  p.x = abs(p.x);
  p -= 2.0f * min(dot(float2(-k.x, k.y), p), 0.0f) * float2(-k.x, k.y);
  p -= 2.0f * min(dot(float2(k.x, k.y), p), 0.0f) * float2(k.x, k.y);
  p -= float2(clamp(p.x, -r * k.z, r * k.z), r);
  return length(p) * sign(p.y);
}
float sd_hexagon_2d_iq(float2 p, float r)
{
  float3 k = float3(-0.866025404f, 0.5f, 0.577350269f);
  p = abs(p);
  p -= 2.0f * min(dot(k.xy, p), 0.0f) * k.xy;
  p -= float2(clamp(p.x, -k.z * r, k.z * r), r);
  return length(p) * sign(p.y);
}
float sd_octogon_2d_iq(float2 p, float r)
{
  float3 k = float3(-0.9238795325f, 0.3826834323f, 0.4142135623f);
  p = abs(p);
  p -= 2.0f * min(dot(float2(k.x, k.y), p), 0.0f) * float2(k.x, k.y);
  p -= 2.0f * min(dot(float2(-k.x, k.y), p), 0.0f) * float2(-k.x, k.y);
  p -= float2(clamp(p.x, -k.z * r, k.z * r), r);
  return length(p) * sign(p.y);
}
float sd_hexagram_2d_iq(float2 p, float r)
{
  float4 k = float4(-0.5f, 0.8660254038f, 0.5773502692f, 1.7320508076f);
  p = abs(p);
  p -= 2.0f * min(dot(k.xy, p), 0.0f) * k.xy;
  p -= 2.0f * min(dot(k.yx, p), 0.0f) * k.yx;
  p -= float2(clamp(p.x, r * k.z, r * k.w), r);
  return length(p) * sign(p.y);
}
float sd_star_2d_iq(float2 p, float r, float n, float m)
{
  float an = 3.14159265f / max(n, 2.0f);
  float en = 3.14159265f / max(m, 2.0001f);
  float2 acs = float2(cos(an), sin(an));
  float2 ecs = float2(cos(en), sin(en));
  /* GLSL mod is x - y*floor(x/y). Do not use fmod: negative atan2 (left
   * half-plane) would keep a negative remainder and crease the star. */
  float an2 = 2.0f * an;
  float bn = atan(p.x, p.y);
  bn = bn - an2 * floor(bn / an2) - an;
  p = length(p) * float2(cos(bn), abs(sin(bn)));
  p -= r * acs;
  p += ecs * clamp(-dot(p, ecs), 0.0f, r * acs.y / max(ecs.y, 1e-8f));
  return length(p) * sign(p.x);
}
float sd_pie_2d_iq(float2 p, float2 c, float r)
{
  p.x = abs(p.x);
  float l = length(p) - r;
  float m = length(p - c * clamp(dot(p, c), 0.0f, r));
  return max(l, m * sign(c.y * p.x - c.x * p.y));
}
float sd_cut_disk_2d_iq(float2 p, float r, float h)
{
  float w = sqrt(max(r * r - h * h, 0.0f));
  p.x = abs(p.x);
  float s = max((h - r) * p.x * p.x + w * w * (h + r - 2.0f * p.y), h * p.x - w * p.y);
  return (s < 0.0f) ? length(p) - r : ((p.x < w) ? h - p.y : length(p - float2(w, h)));
}
float sd_arc_2d_iq(float2 p, float2 sc, float ra, float rb)
{
  p.x = abs(p.x);
  return ((sc.y * p.x > sc.x * p.y) ? length(p - sc * ra) : abs(length(p) - ra)) - rb;
}
float sd_ring_2d_iq(float2 p, float2 n, float r, float th)
{
  p.x = abs(p.x);
  p = float2(n.x * p.x + n.y * p.y, -n.y * p.x + n.x * p.y);
  return max(abs(length(p) - r) - th * 0.5f,
             length(float2(p.x, max(0.0f, abs(r - p.y) - th * 0.5f))) * sign(p.x));
}
float sd_horseshoe_2d_iq(float2 p, float2 c, float r, float2 w)
{
  p.x = abs(p.x);
  float l = length(p);
  p = float2(-c.x * p.x + c.y * p.y, c.y * p.x + c.x * p.y);
  p = float2((p.y > 0.0f || p.x > 0.0f) ? p.x : l * sign(-c.x), (p.x > 0.0f) ? p.y : l);
  p = float2(p.x, abs(p.y - r)) - w;
  return length(max(p, float2(0.0f))) + min(0.0f, max(p.x, p.y));
}
float sd_vesica_2d_iq(float2 p, float w, float h)
{
  float d = 0.5f * (w * w - h * h) / max(h, 1e-8f);
  p = abs(p);
  float3 c = (w * p.y < d * (p.x - w)) ? float3(0.0f, w, 0.0f) : float3(-d, 0.0f, d + h);
  return length(p - c.yx) - c.z;
}
float sd_moon_2d_iq(float2 p, float d, float ra, float rb)
{
  p.y = abs(p.y);
  float a = (ra * ra - rb * rb + d * d) / (2.0f * max(d, 1e-8f));
  float b = sqrt(max(ra * ra - a * a, 0.0f));
  if (d * (p.x * b - p.y * a) > d * d * max(b - p.y, 0.0f)) {
    return length(p - float2(a, b));
  }
  return max(length(p) - ra, -(length(p - float2(d, 0.0f)) - rb));
}
float sd_heart_2d_iq(float2 p)
{
  p.x = abs(p.x);
  if (p.y + p.x > 1.0f) {
    return sqrt(dot(p - float2(0.25f, 0.75f), p - float2(0.25f, 0.75f))) - sqrt(2.0f) / 4.0f;
  }
  return sqrt(min(dot(p - float2(0.0f, 1.0f), p - float2(0.0f, 1.0f)),
                  dot(p - 0.5f * max(p.x + p.y, 0.0f), p - 0.5f * max(p.x + p.y, 0.0f)))) *
         sign(p.x - p.y);
}
float sd_cross_2d_iq(float2 p, float2 b, float r)
{
  p = abs(p);
  p = (p.y > p.x) ? p.yx : p.xy;
  float2 q = p - b;
  float k = max(q.y, q.x);
  float2 w = (k > 0.0f) ? q : float2(b.y - p.x, -k);
  return sign(k) * length(max(w, float2(0.0f))) + r;
}
float sd_rounded_x_2d_iq(float2 p, float w, float r)
{
  p = abs(p);
  return length(p - min(p.x + p.y, w) * 0.5f) - r;
}
float sd_ellipse_2d_iq(float2 p, float2 ab)
{
  p = abs(p);
  if (p.x > p.y) {
    p = p.yx;
    ab = ab.yx;
  }
  float l = ab.y * ab.y - ab.x * ab.x;
  if (abs(l) < 1e-12f) {
    return length(p) - ab.x;
  }
  float m = ab.x * p.x / l;
  float m2 = m * m;
  float n = ab.y * p.y / l;
  float n2 = n * n;
  float c = (m2 + n2 - 1.0f) / 3.0f;
  float c3 = c * c * c;
  float q = c3 + m2 * n2 * 2.0f;
  float d = c3 + m2 * n2;
  float g = m + m * n2;
  float co;
  if (d < 0.0f) {
    float h = acos(clamp(q / c3, -1.0f, 1.0f)) / 3.0f;
    float s = cos(h);
    float t = sin(h) * sqrt(3.0f);
    float rx = sqrt(max(-c * (s + t + 2.0f) + m2, 0.0f));
    float ry = sqrt(max(-c * (s - t + 2.0f) + m2, 0.0f));
    co = (ry + sign(l) * rx + abs(g) / max(rx * ry, 1e-20f) - m) / 2.0f;
  }
  else {
    float h = 2.0f * m * n * sqrt(max(d, 0.0f));
    float s = sign(q + h) * pow(abs(q + h), 1.0f / 3.0f);
    float u = sign(q - h) * pow(abs(q - h), 1.0f / 3.0f);
    float rx = -s - u - c * 4.0f + 2.0f * m2;
    float ry = (s - u) * sqrt(3.0f);
    float rm = sqrt(rx * rx + ry * ry);
    co = (ry / sqrt(max(rm - rx, 1e-20f)) + 2.0f * g / max(rm, 1e-20f) - m) / 2.0f;
  }
  float2 r = ab * float2(co, sqrt(max(1.0f - co * co, 0.0f)));
  return length(r - p) * sign(p.y - r.y);
}
float sd_oriented_box_2d_iq(float2 p, float2 a, float2 b, float th)
{
  float l = length(b - a);
  float2 d = (b - a) / max(l, 1e-20f);
  float2 q = p - (a + b) * 0.5f;
  q = float2(d.x * q.x - d.y * q.y, d.y * q.x + d.x * q.y);
  q = abs(q) - float2(l, th) * 0.5f;
  return length(max(q, float2(0.0f))) + min(max(q.x, q.y), 0.0f);
}
float sd_tunnel_2d_iq(float2 p, float2 wh)
{
  p.x = abs(p.x);
  p.y = -p.y;
  float2 q = p - wh;
  float d1 = dot(float2(max(q.x, 0.0f), q.y), float2(max(q.x, 0.0f), q.y));
  q.x = (p.y > 0.0f) ? q.x : length(p) - wh.x;
  float d2 = dot(float2(q.x, max(q.y, 0.0f)), float2(q.x, max(q.y, 0.0f)));
  float d = sqrt(min(d1, d2));
  return (max(q.x, q.y) < 0.0f) ? -d : d;
}
float sd_stairs_2d_iq(float2 p, float2 wh, float n)
{
  float2 ba = wh * n;
  float d = min(dot(p - float2(clamp(p.x, 0.0f, ba.x), 0.0f),
                    p - float2(clamp(p.x, 0.0f, ba.x), 0.0f)),
                dot(p - float2(ba.x, clamp(p.y, 0.0f, ba.y)),
                    p - float2(ba.x, clamp(p.y, 0.0f, ba.y))));
  float s = sign(max(-p.y, p.x - ba.x));
  float dia = length(wh);
  p = float2(wh.x * p.x - wh.y * p.y, wh.y * p.x + wh.x * p.y) / max(dia, 1e-20f);
  float id = clamp(round(p.x / max(dia, 1e-20f)), 0.0f, n - 1.0f);
  p.x = p.x - id * dia;
  p = float2(wh.x * p.x + wh.y * p.y, -wh.y * p.x + wh.x * p.y) / max(dia, 1e-20f);
  float hh = wh.y / 2.0f;
  p.y -= hh;
  if (p.y > hh * sign(p.x)) {
    s = 1.0f;
  }
  p = (id < 0.5f || p.x > 0.0f) ? p : -p;
  d = min(d, dot(p - float2(0.0f, clamp(p.y, -hh, hh)), p - float2(0.0f, clamp(p.y, -hh, hh))));
  d = min(d, dot(p - float2(clamp(p.x, 0.0f, wh.x), hh), p - float2(clamp(p.x, 0.0f, wh.x), hh)));
  return sqrt(d) * s;
}
float sd_rounded_cross_2d_iq(float2 p, float h)
{
  float k = 0.5f * (h + 1.0f / max(h, 1e-8f));
  p = abs(p);
  return (p.x < 1.0f && p.y < p.x * (k - h) + h) ?
             k - sqrt(dot(p - float2(1.0f, k), p - float2(1.0f, k))) :
             sqrt(min(dot(p - float2(0.0f, h), p - float2(0.0f, h)),
                      dot(p - float2(1.0f, 0.0f), p - float2(1.0f, 0.0f))));
}

float2 sc_from_angle_iq(float angle)
{
  float a = clamp(angle, 1.0e-4f, 3.14159265f - 1.0e-4f);
  return float2(sin(a), cos(a));
}

float evaluate_sdf_shape(float3 vector,
                         float radius,
                         float3 size,
                         float height,
                         float minor_radius,
                         float roundness,
                         float top_radius,
                         float offset,
                         float3 point_a,
                         float3 point_b,
                         float3 point_c,
                         float angle,
                         float thickness,
                         float count,
                         float factor,
                         float shape)
{
  int s = int(shape + 0.5f);
  float2 p2 = vector.xy;
  float2 sc = sc_from_angle_iq(angle);

  if (s == 1) {
    return sd_box_iq(vector, size);
  }
  if (s == 2) {
    return sd_torus_iq(vector, radius, minor_radius);
  }
  if (s == 3) {
    return sd_round_box_iq(vector, size, roundness);
  }
  if (s == 4) {
    return sd_box_frame_iq(vector, size, roundness);
  }
  if (s == 5) {
    return sd_vertical_capsule_iq(vector, max(height, 0.0f), radius);
  }
  if (s == 6) {
    return sd_capped_cylinder_iq(vector, radius, max(height, 0.0f));
  }
  if (s == 7) {
    return sd_capped_cone_iq(vector, max(height, 1e-4f), radius, max(top_radius, 0.0f));
  }
  if (s == 8) {
    return sd_plane_iq(vector, offset);
  }
  if (s == 9) {
    return sd_hex_prism_iq(vector, float2(max(radius, 1e-4f), max(height, 1e-4f)));
  }
  if (s == 10) {
    return sd_octahedron_iq(vector, max(radius, 1e-4f));
  }
  if (s == 11) {
    return sd_pyramid_iq(vector, max(height, 1e-4f));
  }
  if (s == 12) {
    return sd_capped_torus_iq(vector, sc, radius, minor_radius);
  }
  if (s == 13) {
    return sd_link_iq(vector, height * 0.5f, radius, minor_radius);
  }
  if (s == 14) {
    return sd_infinite_cylinder_iq(vector, radius);
  }
  if (s == 15) {
    return sd_cone_iq(vector, sc, max(height, 1e-4f));
  }
  if (s == 16) {
    return sd_rounded_cylinder_iq(vector, radius, roundness, max(height, 0.0f));
  }
  if (s == 17) {
    return sd_solid_angle_iq(vector, sc, radius);
  }
  if (s == 18) {
    return sd_cut_sphere_iq(vector, radius, clamp(height, -radius * 0.99f, radius * 0.99f));
  }
  if (s == 19) {
    return sd_cut_hollow_sphere_iq(
        vector, radius, clamp(height, -radius * 0.99f, radius * 0.99f), thickness);
  }
  if (s == 20) {
    return sd_death_star_iq(
        vector, radius, minor_radius, clamp(factor, 1e-4f, radius + minor_radius));
  }
  if (s == 21) {
    return sd_round_cone_iq(vector, radius, top_radius, max(height, 1e-4f));
  }
  if (s == 22) {
    return sd_rhombus_3d_iq(vector, size.x, size.z, height, roundness);
  }
  if (s == 23) {
    return sd_ellipsoid_iq(vector, max(size, float3(1e-4f)));
  }
  if (s == 24) {
    return sd_tri_prism_iq(vector, float2(max(radius, 1e-4f), max(height, 1e-4f)));
  }
  if (s == 25) {
    return sd_infinite_cone_iq(vector, sc);
  }

  /* 2D */
  if (s == 32) {
    return sd_circle_2d_iq(p2, radius);
  }
  if (s == 33) {
    return sd_box_2d_iq(p2, size.xy);
  }
  if (s == 34) {
    return sd_rounded_box_2d_iq(p2, size.xy, roundness);
  }
  if (s == 35) {
    return sd_chamfer_box_2d_iq(p2, size.xy, clamp(factor, 0.0f, min(size.x, size.y)));
  }
  if (s == 36) {
    return sd_segment_2d_iq(p2, point_a.xy, point_b.xy);
  }
  if (s == 37) {
    return sd_rhombus_2d_iq(p2, size.xy);
  }
  if (s == 38) {
    return sd_trapezoid_2d_iq(p2, radius, top_radius, height);
  }
  if (s == 39) {
    return sd_parallelogram_2d_iq(p2, size.x, height, factor);
  }
  if (s == 40) {
    return sd_equilateral_triangle_2d_iq(p2, radius);
  }
  if (s == 41) {
    return sd_triangle_isosceles_2d_iq(p2, float2(size.x, height));
  }
  if (s == 42) {
    return sd_triangle_2d_iq(p2, point_a.xy, point_b.xy, point_c.xy);
  }
  if (s == 43) {
    return sd_uneven_capsule_2d_iq(p2, radius, top_radius, height);
  }
  if (s == 44) {
    return sd_pentagon_2d_iq(p2, radius);
  }
  if (s == 45) {
    return sd_hexagon_2d_iq(p2, radius);
  }
  if (s == 46) {
    return sd_octogon_2d_iq(p2, radius);
  }
  if (s == 47) {
    return sd_hexagram_2d_iq(p2, radius);
  }
  if (s == 48) {
    return sd_star_2d_iq(p2, radius, max(count, 2.0f), factor);
  }
  if (s == 49) {
    return sd_pie_2d_iq(p2, sc, radius);
  }
  if (s == 50) {
    return sd_cut_disk_2d_iq(p2, radius, clamp(height, -radius * 0.99f, radius * 0.99f));
  }
  if (s == 51) {
    return sd_arc_2d_iq(p2, sc, radius, thickness);
  }
  if (s == 52) {
    return sd_ring_2d_iq(p2, sc, radius, thickness);
  }
  if (s == 53) {
    return sd_horseshoe_2d_iq(p2, sc, radius, float2(size.x, thickness));
  }
  if (s == 54) {
    return sd_vesica_2d_iq(p2, radius, clamp(factor, 1e-4f, radius * 0.95f));
  }
  if (s == 55) {
    return sd_moon_2d_iq(p2, clamp(factor, 1e-4f, radius + minor_radius), radius, minor_radius);
  }
  if (s == 56) {
    return sd_heart_2d_iq(p2 / max(radius, 1e-4f)) * radius;
  }
  if (s == 57) {
    return sd_cross_2d_iq(p2, size.xy, thickness);
  }
  if (s == 58) {
    return sd_rounded_x_2d_iq(p2, radius, thickness);
  }
  if (s == 59) {
    return sd_ellipse_2d_iq(p2, max(size.xy, float2(1e-4f)));
  }
  if (s == 60) {
    return sd_oriented_box_2d_iq(p2, point_a.xy, point_b.xy, thickness);
  }
  if (s == 61) {
    return sd_tunnel_2d_iq(p2, size.xy);
  }
  if (s == 62) {
    return sd_stairs_2d_iq(p2, size.xy, max(count, 1.0f));
  }
  if (s == 63) {
    return sd_rounded_cross_2d_iq(p2, max(height, 1e-4f));
  }
  return sd_sphere_iq(vector, radius);
}

[[node]]
void node_sdf_shape(float3 vector,
                    float scale,
                    float radius,
                    float3 size,
                    float height,
                    float minor_radius,
                    float roundness,
                    float top_radius,
                    float offset,
                    float3 point_a,
                    float3 point_b,
                    float3 point_c,
                    float angle,
                    float thickness,
                    float count,
                    float factor,
                    float shape,
                    float &distance)
{
  /* shape >= 1000: Vector unlinked → Texture Coordinate Object (local P).
   * C++ packs: shape_id + (unlinked ? 1000 : 0). */
  float shape_id = shape;
  float3 coord = vector;
  if (shape >= 999.5f) {
    shape_id = shape - 1000.0f;
    point_transform_world_to_object(g_data.P, coord);
  }
  float3 p = coord * max(scale, 1e-6f);
  distance = evaluate_sdf_shape(p,
                                radius,
                                size,
                                height,
                                minor_radius,
                                roundness,
                                top_radius,
                                offset,
                                point_a,
                                point_b,
                                point_c,
                                angle,
                                thickness,
                                count,
                                factor,
                                shape_id);
}
