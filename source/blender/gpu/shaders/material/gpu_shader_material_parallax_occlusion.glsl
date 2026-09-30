/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "gpu_shader_codegen_lib.glsl"
#include "gpu_shader_material_interface.bsl.hh"

/* Parallax Occlusion Mapping.
 * Height map: white = high (towards camera). Invert treats it as a depth map.
 * Midlevel is the geometric surface; values below it inset. Scale is UV-space height.
 *
 * Modes (params.x): 0 Offset, 1 Steep, 2 POM, 3 POM + self-shadow.
 * params.y invert, params.z clip to 0–1, params.w height channel.
 */

float parallax_sample_height(sampler2D ima, float2 uv, float2 dx, float2 dy, float channel)
{
#ifdef GPU_FRAGMENT_SHADER
  float4 t = textureGrad(ima, uv, dx, dy);
#else
  float4 t = texture(ima, uv);
#endif
  int ch = int(channel + 0.5f);
  if (ch <= 0) {
    return t.x;
  }
  if (ch == 1) {
    return t.y;
  }
  if (ch == 2) {
    return t.z;
  }
  if (ch == 3) {
    return t.w;
  }
  if (ch == 4) {
    return (t.x + t.y + t.z) * (1.0f / 3.0f);
  }
  return dot(t.xyz, float3(0.2126f, 0.7152f, 0.0722f));
}

float parallax_depth_from_height(float height, float midlevel, float invert)
{
  float h = height;
  if (invert > 0.5f) {
    h = 1.0f - h;
  }
  float ml = max(midlevel, 1e-5f);
  return saturate((ml - h) / ml);
}

[[node]]
void node_parallax_occlusion(float3 vector,
                             float scale,
                             float midlevel,
                             float samples,
                             float refine,
                             float3 normal,
                             float3 incoming,
                             float3 light,
                             sampler2D ima,
                             float4 params,
                             [[resource_table]] KernelGlobals &kg,
                             const ShadingData &sd,
                             float3 &out_vector,
                             float &out_height,
                             float &out_shadow,
                             float &out_alpha)
{
  out_vector = vector;
  out_height = midlevel;
  out_shadow = 1.0f;
  out_alpha = 1.0f;

#ifndef GPU_FRAGMENT_SHADER
  return;
#else

  float2 uv = vector.xy;
  float scl = max(scale, 0.0f);
  if (scl < 1e-8f) {
    return;
  }

  float3 N = (dot(normal, normal) < 1e-12f) ? float3(sd.N) : normalize(normal);
  float3 V = incoming;
  if (dot(V, V) < 1e-12f) {
    V = coordinate_impl(kg, sd, sd.P, N).incoming;
  }
  V = normalize(V);

  float3 dPdx = gpu_dfdx(sd.P) * derivative_scale_get(kg);
  float3 dPdy = gpu_dfdy(sd.P) * derivative_scale_get(kg);
  float2 dUVdx = gpu_dfdx(uv);
  float2 dUVdy = gpu_dfdy(uv);

  float det = dUVdx.x * dUVdy.y - dUVdx.y * dUVdy.x;
  if (abs(det) < 1e-12f) {
    return;
  }

  float3 T = (dPdx * dUVdy.y - dPdy * dUVdx.y) / det;
  float3 B = (dPdy * dUVdx.x - dPdx * dUVdy.x) / det;
  T = T - N * dot(N, T);
  float tlen = length(T);
  if (tlen < 1e-8f) {
    return;
  }
  T /= tlen;
  float3 Bn = cross(N, T);
  if (dot(Bn, B) < 0.0f) {
    Bn = -Bn;
  }

  float3 Vts = float3(dot(V, T), dot(V, Bn), dot(V, N));
  if (!FrontFacing) {
    Vts.z = -Vts.z;
  }
  float vlen = length(Vts);
  if (vlen < 1e-8f) {
    return;
  }
  Vts /= vlen;

  float vz = max(abs(Vts.z), 0.02f);
  float2 parallax_dir = Vts.xy / vz * scl;

  float mode = params.x;
  float invert = params.y;
  float clip = params.z;
  float channel = params.w;

  int nsteps = int(clamp(samples, 1.0f, 64.0f));
  float ndotv = saturate(abs(Vts.z));
  nsteps = int(mix(float(nsteps), max(float(nsteps) * 0.25f, 4.0f), ndotv));
  nsteps = clamp(nsteps, 1, 64);

  float2 dx = dUVdx * texture_lod_bias_get(kg);
  float2 dy = dUVdy * texture_lod_bias_get(kg);

  float2 hit_uv = uv;
  float hit_depth = 0.0f;
  float hit_height = parallax_sample_height(ima, uv, dx, dy, channel);

  if (mode < 0.5f) {
    /* Cheap offset parallax (one sample). */
    hit_depth = parallax_depth_from_height(hit_height, midlevel, invert);
    hit_uv = uv - parallax_dir * hit_depth;
  }
  else {
    float layer_depth = 1.0f / float(nsteps);
    float2 delta_uv = parallax_dir / float(nsteps);
    float current_layer = 0.0f;
    float2 curr_uv = uv;
    float curr_h = parallax_depth_from_height(
        parallax_sample_height(ima, curr_uv, dx, dy, channel), midlevel, invert);

    for (int i = 0; i < 64; i++) {
      if (i >= nsteps) {
        break;
      }
      if (current_layer >= curr_h) {
        break;
      }
      curr_uv -= delta_uv;
      current_layer += layer_depth;
      curr_h = parallax_depth_from_height(
          parallax_sample_height(ima, curr_uv, dx, dy, channel), midlevel, invert);
    }

    hit_uv = curr_uv;
    hit_depth = current_layer;

    if (mode >= 1.5f) {
      /* POM: interpolate between the last two layers. */
      float2 prev_uv = curr_uv + delta_uv;
      float after = curr_h - current_layer;
      float before = parallax_depth_from_height(
                         parallax_sample_height(ima, prev_uv, dx, dy, channel), midlevel, invert) -
                     current_layer + layer_depth;
      float denom = after - before;
      float w = (abs(denom) > 1e-8f) ? saturate(after / denom) : 0.0f;
      hit_uv = mix(curr_uv, prev_uv, w);
      hit_depth = mix(current_layer, current_layer - layer_depth, w);

      int nrefine = int(clamp(refine, 0.0f, 8.0f));
      float2 lo_uv = prev_uv;
      float2 hi_uv = curr_uv;
      float lo_layer = current_layer - layer_depth;
      float hi_layer = current_layer;
      for (int r = 0; r < 8; r++) {
        if (r >= nrefine) {
          break;
        }
        float2 mid_uv = 0.5f * (lo_uv + hi_uv);
        float mid_layer = 0.5f * (lo_layer + hi_layer);
        float mid_h = parallax_depth_from_height(
            parallax_sample_height(ima, mid_uv, dx, dy, channel), midlevel, invert);
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

  if (clip > 0.5f) {
    if (hit_uv.x < 0.0f || hit_uv.x > 1.0f || hit_uv.y < 0.0f || hit_uv.y > 1.0f) {
      hit_uv = uv;
      hit_depth = 0.0f;
    }
  }

  out_vector = float3(hit_uv, vector.z);
  out_height = parallax_sample_height(ima, hit_uv, dx, dy, channel);

  /* Self-shadow: march from the hit towards the light in the height field. */
  if (mode >= 2.5f && dot(light, light) > 1e-12f) {
    float3 L = normalize(light);
    float3 Lts = float3(dot(L, T), dot(L, Bn), dot(L, N));
    if (!FrontFacing) {
      Lts.z = -Lts.z;
    }
    if (Lts.z <= 1e-4f) {
      out_shadow = 0.0f;
    }
    else {
      float lz = max(Lts.z, 0.02f);
      float2 light_dir = Lts.xy / lz * scl;
      int ssteps = max(nsteps / 2, 4);
      float2 sdelta = light_dir / float(ssteps);
      float slayer = 1.0f / float(ssteps);
      float2 suv = hit_uv;
      float sdepth = hit_depth;
      float sh = 1.0f;
      const float bias = 0.02f;
      for (int s = 0; s < 64; s++) {
        if (s >= ssteps) {
          break;
        }
        suv += sdelta;
        sdepth -= slayer;
        if (sdepth <= 0.0f) {
          break;
        }
        float d = parallax_depth_from_height(
            parallax_sample_height(ima, suv, dx, dy, channel), midlevel, invert);
        if (d < sdepth - bias) {
          sh = 0.0f;
          break;
        }
      }
      out_shadow = sh;
    }
  }
#endif
}
