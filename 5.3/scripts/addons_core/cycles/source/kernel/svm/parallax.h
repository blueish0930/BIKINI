/* SPDX-FileCopyrightText: 2026 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/geom/attribute.h"
#include "kernel/geom/object.h"
#include "kernel/geom/primitive.h"
#include "kernel/svm/image.h"
#include "kernel/svm/node_types.h"
#include "kernel/svm/util.h"

#include "util/math_float2.h"
#include "util/math_float3.h"
#include "util/types_dual.h"

CCL_NAMESPACE_BEGIN

ccl_device_inline float svm_parallax_channel(const float4 t, const uint8_t channel)
{
  if (channel == 0) {
    return t.x;
  }
  if (channel == 1) {
    return t.y;
  }
  if (channel == 2) {
    return t.z;
  }
  if (channel == 3) {
    return t.w;
  }
  if (channel == 4) {
    return (t.x + t.y + t.z) * (1.0f / 3.0f);
  }
  return dot(make_float3(t), make_float3(0.2126f, 0.7152f, 0.0722f));
}

ccl_device_inline float svm_parallax_sample(KernelGlobals kg,
                                            ccl_private ShaderData *sd,
                                            const int id,
                                            const float2 uv,
                                            const dual2 grad,
                                            const uint8_t channel)
{
  const float4 t = svm_image_texture(kg, sd, id, dual2(uv, grad.dx, grad.dy), 0);
  return svm_parallax_channel(t, channel);
}

ccl_device_inline void svm_parallax_store(ccl_private float *stack,
                                          const ccl_global SVMNodeParallaxOcclusion &node,
                                          const float3 out_vector,
                                          const float out_height,
                                          const float out_shadow,
                                          const float out_alpha)
{
  if (stack_valid(node.out_vector)) {
    stack_store_float3(stack, node.out_vector, out_vector);
  }
  if (stack_valid(node.out_height)) {
    stack_store_float(stack, node.out_height, out_height);
  }
  if (stack_valid(node.out_shadow)) {
    stack_store_float(stack, node.out_shadow, out_shadow);
  }
  if (stack_valid(node.out_alpha)) {
    stack_store_float(stack, node.out_alpha, out_alpha);
  }
}

ccl_device_inline float3 svm_spom_eval_P(const float u,
                                         const float v,
                                         const float w,
                                         const float3 A,
                                         const float3 B,
                                         const float3 C,
                                         const float3 Na,
                                         const float3 Nb,
                                         const float3 Nc,
                                         const float H)
{
  const float b0 = 1.0f - u - v;
  return b0 * (A + (w * H) * Na) + u * (B + (w * H) * Nb) + v * (C + (w * H) * Nc);
}

ccl_device_inline float3 svm_spom_newton_uvw(const float3 P,
                                             const float3 A,
                                             const float3 B,
                                             const float3 C,
                                             const float3 Na,
                                             const float3 Nb,
                                             const float3 Nc,
                                             const float H,
                                             const float3 uvw0,
                                             const int iters)
{
  float3 uvw = uvw0;
  for (int i = 0; i < 8; i++) {
    if (i >= iters) {
      break;
    }
    const float u = uvw.x;
    const float v = uvw.y;
    const float w = uvw.z;
    const float3 F = svm_spom_eval_P(u, v, w, A, B, C, Na, Nb, Nc, H) - P;
    if (dot(F, F) < 1e-12f) {
      break;
    }
    const float3 du = (B - A) + (w * H) * (Nb - Na);
    const float3 dv = (C - A) + (w * H) * (Nc - Na);
    const float3 dw = H * ((1.0f - u - v) * Na + u * Nb + v * Nc);
    const float3 c1c2 = cross(dv, dw);
    const float det = dot(du, c1c2);
    if (fabsf(det) < 1e-12f) {
      break;
    }
    const float inv = 1.0f / det;
    uvw.x = u - clamp(dot(c1c2, F) * inv, -0.5f, 0.5f);
    uvw.y = v - clamp(dot(cross(dw, du), F) * inv, -0.5f, 0.5f);
    uvw.z = w - clamp(dot(cross(du, dv), F) * inv, -0.5f, 0.5f);
  }
  return uvw;
}

ccl_device_inline float3 svm_spom_init_uvw(const float3 P,
                                           const float3 A,
                                           const float3 B,
                                           const float3 C,
                                           const float3 Na,
                                           const float3 Nb,
                                           const float3 Nc,
                                           const float H)
{
  float3 Navg = Na + Nb + Nc;
  const float nlen = len(Navg);
  if (nlen > 1e-8f) {
    Navg = Navg / nlen;
  }
  else {
    Navg = make_float3(0.0f, 0.0f, 1.0f);
  }
  const float3 e1 = B - A;
  const float3 e2 = C - A;
  const float w = dot(P - A, Navg) / max(H, 1e-8f);
  const float3 q = (P - Navg * (w * H)) - A;
  const float d00 = dot(e1, e1);
  const float d01 = dot(e1, e2);
  const float d11 = dot(e2, e2);
  const float d20 = dot(q, e1);
  const float d21 = dot(q, e2);
  const float den = d00 * d11 - d01 * d01;
  float u = 1.0f / 3.0f;
  float v = 1.0f / 3.0f;
  if (fabsf(den) > 1e-20f) {
    u = (d11 * d20 - d01 * d21) / den;
    v = (d00 * d21 - d01 * d20) / den;
  }
  return make_float3(u, v, w);
}

ccl_device_inline float3 svm_spom_clamp_uvw(const float3 uvw)
{
  float u = uvw.x;
  float v = uvw.y;
  float w = uvw.z;
  if (u < 0.0f) {
    v += u * 0.5f;
    u = 0.0f;
  }
  if (v < 0.0f) {
    u += v * 0.5f;
    v = 0.0f;
  }
  const float s = u + v;
  if (s > 1.0f && s > 1e-8f) {
    u /= s;
    v /= s;
  }
  w = clamp(w, 0.0f, 1.0f);
  return make_float3(max(u, 0.0f), max(v, 0.0f), w);
}

ccl_device_inline float3 svm_spom_invert_uvw(const float3 P,
                                             const float3 A,
                                             const float3 B,
                                             const float3 C,
                                             const float3 Na,
                                             const float3 Nb,
                                             const float3 Nc,
                                             const float H,
                                             const float3 uvw0,
                                             const int iters)
{
  if (dot(Na, Nb) > 0.995f && dot(Nb, Nc) > 0.995f && dot(Nc, Na) > 0.995f) {
    const float3 e1 = B - A;
    const float3 e2 = C - A;
    const float3 e3 = Na * H;
    const float3 p = P - A;
    const float3 c1 = cross(e2, e3);
    const float det = dot(e1, c1);
    if (fabsf(det) > 1e-12f) {
      const float inv = 1.0f / det;
      return svm_spom_clamp_uvw(make_float3(
          dot(c1, p) * inv, dot(cross(e3, e1), p) * inv, dot(cross(e1, e2), p) * inv));
    }
  }
  float3 uvw = uvw0;
  if (dot(uvw, uvw) < 1e-20f) {
    uvw = svm_spom_init_uvw(P, A, B, C, Na, Nb, Nc, H);
  }
  return svm_spom_clamp_uvw(svm_spom_newton_uvw(P, A, B, C, Na, Nb, Nc, H, uvw, iters));
}

ccl_device_inline float2 svm_spom_uv_from_uvw(const float3 uvw,
                                              const float3 uv0,
                                              const float3 uv1,
                                              const float3 uv2)
{
  const float u = uvw.x;
  const float v = uvw.y;
  const float b0 = 1.0f - u - v;
  return make_float2(uv0.x, uv0.y) * b0 + make_float2(uv1.x, uv1.y) * u +
         make_float2(uv2.x, uv2.y) * v;
}

ccl_device_inline float svm_parallax_depth(const float height,
                                           const float midlevel,
                                           const uint8_t invert)
{
  float h = height;
  if (invert) {
    h = 1.0f - h;
  }
  const float ml = max(midlevel, 1e-5f);
  return clamp((ml - h) / ml, 0.0f, 1.0f);
}

ccl_device_noinline void svm_node_parallax_occlusion(
    KernelGlobals kg,
    ccl_private ShaderData *sd,
    ccl_private float *stack,
    const ccl_global SVMNodeParallaxOcclusion &node)
{
  const float3 vector = stack_load(stack, node.vector);
  float2 uv = make_float2(vector.x, vector.y);
  const dual2 uv_grad = dual2(uv);
  float3 out_vector = vector;
  float out_height = 0.0f;
  float out_shadow = 1.0f;
  float out_alpha = 1.0f;

  if (node.id < 0) {
    svm_parallax_store(stack, node, out_vector, out_height, out_shadow, out_alpha);
    return;
  }

  const float scale = max(stack_load(stack, node.scale), 0.0f);
  const float midlevel = stack_load(stack, node.midlevel);
  if (scale < 1e-8f) {
    out_height = svm_parallax_sample(kg, sd, node.id, uv, uv_grad, node.channel);
    svm_parallax_store(stack, node, out_vector, out_height, out_shadow, out_alpha);
    return;
  }

  if (node.mode >= 4) {
    const AttributeDescriptor d0 = find_attribute(kg, sd, node.attr_a);
    const AttributeDescriptor d1 = find_attribute(kg, sd, node.attr_b);
    const AttributeDescriptor d2 = find_attribute(kg, sd, node.attr_c);
    const AttributeDescriptor d3 = find_attribute(kg, sd, node.attr_na);
    const AttributeDescriptor d4 = find_attribute(kg, sd, node.attr_nb);
    const AttributeDescriptor d5 = find_attribute(kg, sd, node.attr_nc);
    if (!(is_attribute_found(d0) && is_attribute_found(d1) && is_attribute_found(d2) &&
          is_attribute_found(d3) && is_attribute_found(d4) && is_attribute_found(d5)))
    {
      svm_parallax_store(stack, node, out_vector, out_height, out_shadow, 1.0f);
      return;
    }
    const float4 p0 = primitive_surface_attribute<float4>(kg, sd, d0);
    const float4 p1 = primitive_surface_attribute<float4>(kg, sd, d1);
    const float4 p2 = primitive_surface_attribute<float4>(kg, sd, d2);
    const float4 p3 = primitive_surface_attribute<float4>(kg, sd, d3);
    const float4 p4 = primitive_surface_attribute<float4>(kg, sd, d4);
    const float4 p5 = primitive_surface_attribute<float4>(kg, sd, d5);
    const float3 A = make_float3(p0.x, p0.y, p0.z);
    const float3 B = make_float3(p1.x, p1.y, p1.z);
    const float3 C = make_float3(p2.x, p2.y, p2.z);
    float3 Na = safe_normalize(make_float3(p3.x, p3.y, p3.z));
    float3 Nb = safe_normalize(make_float3(p4.x, p4.y, p4.z));
    float3 Nc = safe_normalize(make_float3(p5.x, p5.y, p5.z));
    const float3 uv0 = make_float3(p0.w, p1.w, 0.0f);
    const float3 uv1 = make_float3(p2.w, p3.w, 0.0f);
    const float3 uv2 = make_float3(p4.w, p5.w, 0.0f);
    float3 P = sd->P;
    object_inverse_position_transform(kg, sd, &P);
    float3 V = stack_load(stack, node.incoming);
    if (dot(V, V) < 1e-12f) {
      V = sd->wi;
    }
    object_inverse_dir_transform(kg, sd, &V);
    V = safe_normalize(V);
    const float3 dir0 = -V;
    const float H = max(scale, 1e-5f);
    const int nsteps = clamp(int(stack_load(stack, node.samples)), 4, 64);
    const int nrefine = clamp(int(stack_load(stack, node.refine)), 0, 16);
    const int nnewton = 6;
    const float3 Navg = safe_normalize(Na + Nb + Nc);
    float3 march_dir = dir0;
    float3 uvw = svm_spom_invert_uvw(P, A, B, C, Na, Nb, Nc, H, zero_float3(), nnewton);
    const float3 uvw_probe = svm_spom_invert_uvw(
        P + march_dir * (1e-3f * H), A, B, C, Na, Nb, Nc, H, uvw, nnewton);
    if ((uvw_probe.z - uvw.z) > 0.0f) {
      march_dir = -march_dir;
    }
    const float ndot = max(fabsf(dot(march_dir, Navg)), 0.08f);
    const float step_len = (H / float(nsteps)) / ndot;
    float3 Pcur = P + march_dir * (1e-4f * H);
    uvw = svm_spom_invert_uvw(Pcur, A, B, C, Na, Nb, Nc, H, uvw, nnewton);
    float3 P_lo = Pcur;
    float3 P_hi = Pcur;
    bool hit = false;
    bool had_air = false;
    float2 hit_uv = uv;
    float hit_h = 0.0f;
    for (int i = 0; i < 64; i++) {
      if (i >= nsteps) {
        break;
      }
      uvw = svm_spom_invert_uvw(Pcur, A, B, C, Na, Nb, Nc, H, uvw, nnewton);
      if (uvw.z < -0.02f) {
        break;
      }
      const float2 suv = svm_spom_uv_from_uvw(uvw, uv0, uv1, uv2);
      float hsamp = svm_parallax_sample(kg, sd, node.id, suv, uv_grad, node.channel);
      if (node.invert) {
        hsamp = 1.0f - hsamp;
      }
      hsamp = clamp(hsamp - midlevel, 0.0f, 1.0f);
      if (uvw.z <= hsamp) {
        hit = true;
        hit_uv = suv;
        hit_h = hsamp;
        P_hi = Pcur;
        if (!had_air) {
          P_lo = Pcur - march_dir * step_len;
        }
        break;
      }
      had_air = true;
      P_lo = Pcur;
      Pcur = Pcur + march_dir * step_len;
    }
    if (hit) {
      for (int r = 0; r < 16; r++) {
        if (r >= nrefine) {
          break;
        }
        const float3 Pmid = 0.5f * (P_lo + P_hi);
        uvw = svm_spom_invert_uvw(Pmid, A, B, C, Na, Nb, Nc, H, uvw, nnewton);
        if (uvw.z < -0.02f) {
          P_lo = Pmid;
          continue;
        }
        const float2 suv = svm_spom_uv_from_uvw(uvw, uv0, uv1, uv2);
        float hsamp = svm_parallax_sample(kg, sd, node.id, suv, uv_grad, node.channel);
        if (node.invert) {
          hsamp = 1.0f - hsamp;
        }
        hsamp = clamp(hsamp - midlevel, 0.0f, 1.0f);
        if (uvw.z <= hsamp) {
          P_hi = Pmid;
          hit_uv = suv;
          hit_h = hsamp;
        }
        else {
          P_lo = Pmid;
        }
      }
      out_vector = make_float3(hit_uv.x, hit_uv.y, vector.z);
      out_height = hit_h;
      out_alpha = 1.0f;
    }
    else {
      out_alpha = 0.0f;
    }
    svm_parallax_store(stack, node, out_vector, out_height, out_shadow, out_alpha);
    return;
  }

  float3 N = stack_load(stack, node.normal);
  if (dot(N, N) < 1e-12f) {
    N = sd->N;
  }
  N = safe_normalize(N);

  float3 V = stack_load(stack, node.incoming);
  if (dot(V, V) < 1e-12f) {
    V = sd->wi;
  }
  V = safe_normalize(V);

  /* Geometric TBN (dPdu/dPdv). Screen-space UV derivatives are skipped on purpose:
   * they require SVM need_derivatives and make Cycles viewport unusable. */
  float3 T = sd->dPdu;
  float3 B = sd->dPdv;
  T = T - N * dot(N, T);
  if (len_squared(T) < 1e-16f) {
    float3 Btmp;
    make_orthonormals(N, &T, &Btmp);
  }
  else {
    T = normalize(T);
  }
  B = B - N * dot(N, B);
  if (len_squared(B) < 1e-16f) {
    B = cross(N, T);
  }
  else {
    B = normalize(B);
    if (dot(B, cross(N, T)) < 0.0f) {
      B = -B;
    }
  }

  float3 Vts = make_float3(dot(V, T), dot(V, B), dot(V, N));
  if (sd->runtime_flag & SR_BACKFACING) {
    Vts.z = -Vts.z;
  }
  const float vlen = len(Vts);
  if (vlen < 1e-8f) {
    out_height = svm_parallax_sample(kg, sd, node.id, uv, uv_grad, node.channel);
    svm_parallax_store(stack, node, out_vector, out_height, out_shadow, out_alpha);
    return;
  }
  Vts /= vlen;
  const float vz = max(fabsf(Vts.z), 0.02f);
  const float2 parallax_dir = make_float2(Vts.x, Vts.y) / vz * scale;

  int nsteps = clamp(int(stack_load(stack, node.samples)), 1, 32);
  const float ndotv = clamp(fabsf(Vts.z), 0.0f, 1.0f);
  nsteps = clamp(
      int(float(nsteps) * (1.0f - ndotv) + max(float(nsteps) * 0.25f, 4.0f) * ndotv), 1, 32);

  float2 hit_uv = uv;
  float hit_depth = 0.0f;

  if (node.mode == 0) {
    const float h = svm_parallax_sample(kg, sd, node.id, uv, uv_grad, node.channel);
    hit_depth = svm_parallax_depth(h, midlevel, node.invert);
    hit_uv = uv - parallax_dir * hit_depth;
  }
  else {
    const float layer_depth = 1.0f / float(nsteps);
    const float2 delta_uv = parallax_dir / float(nsteps);
    float current_layer = 0.0f;
    float2 curr_uv = uv;
    float curr_h = svm_parallax_depth(
        svm_parallax_sample(kg, sd, node.id, curr_uv, uv_grad, node.channel), midlevel, node.invert);
    for (int i = 0; i < 32; i++) {
      if (i >= nsteps || current_layer >= curr_h) {
        break;
      }
      curr_uv = curr_uv - delta_uv;
      current_layer += layer_depth;
      curr_h = svm_parallax_depth(
          svm_parallax_sample(kg, sd, node.id, curr_uv, uv_grad, node.channel), midlevel, node.invert);
    }
    hit_uv = curr_uv;
    hit_depth = current_layer;

    if (node.mode >= 2) {
      const float2 prev_uv = curr_uv + delta_uv;
      const float after = curr_h - current_layer;
      const float before =
          svm_parallax_depth(svm_parallax_sample(kg, sd, node.id, prev_uv, uv_grad, node.channel),
                             midlevel,
                             node.invert) -
          current_layer + layer_depth;
      const float denom = after - before;
      const float w = (fabsf(denom) > 1e-8f) ? clamp(after / denom, 0.0f, 1.0f) : 0.0f;
      hit_uv = curr_uv * (1.0f - w) + prev_uv * w;
      hit_depth = current_layer * (1.0f - w) + (current_layer - layer_depth) * w;

      const int nrefine = clamp(int(stack_load(stack, node.refine)), 0, 8);
      float2 lo_uv = prev_uv;
      float2 hi_uv = curr_uv;
      float lo_layer = current_layer - layer_depth;
      float hi_layer = current_layer;
      for (int r = 0; r < 8; r++) {
        if (r >= nrefine) {
          break;
        }
        const float2 mid_uv = 0.5f * (lo_uv + hi_uv);
        const float mid_layer = 0.5f * (lo_layer + hi_layer);
        const float mid_h = svm_parallax_depth(
            svm_parallax_sample(kg, sd, node.id, mid_uv, uv_grad, node.channel), midlevel, node.invert);
        if (mid_h > mid_layer) {
          lo_uv = mid_uv;
          lo_layer = mid_layer;
        }
        else {
          hi_uv = mid_uv;
          hi_layer = mid_layer;
        }
      }
      if (nrefine > 0) {
        hit_uv = hi_uv;
        hit_depth = hi_layer;
      }
    }
  }

  if (node.clip && (hit_uv.x < 0.0f || hit_uv.x > 1.0f || hit_uv.y < 0.0f || hit_uv.y > 1.0f)) {
    hit_uv = uv;
    hit_depth = 0.0f;
  }

  out_vector = make_float3(hit_uv.x, hit_uv.y, vector.z);
  out_height = svm_parallax_sample(kg, sd, node.id, hit_uv, uv_grad, node.channel);

  if (node.mode >= 3) {
    const float3 light = stack_load(stack, node.light);
    if (len_squared(light) > 1e-12f) {
      const float3 L = safe_normalize(light);
      float3 Lts = make_float3(dot(L, T), dot(L, B), dot(L, N));
      if (Lts.z <= 1e-4f) {
        out_shadow = 0.0f;
      }
      else {
        const float lz = max(Lts.z, 0.02f);
        const float2 light_dir = make_float2(Lts.x, Lts.y) / lz * scale;
        const int ssteps = clamp(max(nsteps / 2, 4), 1, 16);
        const float2 sdelta = light_dir / float(ssteps);
        const float slayer = 1.0f / float(ssteps);
        float2 suv = hit_uv;
        float sdepth = hit_depth;
        const float bias = 0.02f;
        out_shadow = 1.0f;
        for (int s = 0; s < 16; s++) {
          if (s >= ssteps) {
            break;
          }
          suv = suv + sdelta;
          sdepth -= slayer;
          if (sdepth <= 0.0f) {
            break;
          }
          const float d = svm_parallax_depth(
              svm_parallax_sample(kg, sd, node.id, suv, uv_grad, node.channel), midlevel, node.invert);
          if (d < sdepth - bias) {
            out_shadow = 0.0f;
            break;
          }
        }
      }
    }
  }

  svm_parallax_store(stack, node, out_vector, out_height, out_shadow, out_alpha);
}

CCL_NAMESPACE_END
