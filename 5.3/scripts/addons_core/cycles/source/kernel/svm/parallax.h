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
