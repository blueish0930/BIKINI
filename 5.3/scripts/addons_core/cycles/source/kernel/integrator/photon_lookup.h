/* SPDX-FileCopyrightText: 2026 CyclesPlus
 * SPDX-FileCopyrightText: 2026 BIKINI
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* === CyclesPlus: Personal Photon Lookup Begin === */
/* Photon caustics: kernel-side photon gather. Reads the photon grid from the
 * standard kernel data arrays, so it runs unchanged on CPU and GPU devices.
 * Grid parameters live in kernel_data.integrator (updated per progressive
 * batch). */

#pragma once

#include "kernel/closure/volume.h"
#include "kernel/integrator/photon_grid.h"
#include "kernel/integrator/photon_profile_types.h"
#include "kernel/integrator/state.h"
#include "util/hash.h"

CCL_NAMESPACE_BEGIN

ccl_device_inline ccl_global uint64_t *photon_volume_profile_select(
    const IntegratorState state)
{
#ifdef __KERNEL_CUDA__
  if (kernel_integrator_state.photon_volume_profile != nullptr) {
    const uint hash = hash_uint2(INTEGRATOR_STATE(state, path, render_pixel_index),
                                 INTEGRATOR_STATE(state, path, sample));
    if ((hash & PHOTON_PROFILE_SAMPLE_MASK) == 0) {
      const uint shard = (hash >> 10) & (PHOTON_PROFILE_SHARDS - 1);
      return kernel_integrator_state.photon_volume_profile +
             shard * PHOTON_PROFILE_NUM_COUNTERS;
    }
  }
#endif
  return nullptr;
}

ccl_device_inline void photon_volume_profile_add(ccl_global uint64_t *profile,
                                                 const int counter,
                                                 const uint64_t count = 1)
{
#ifdef __KERNEL_CUDA__
  if (profile != nullptr && count != 0) {
    atomicAdd((unsigned long long *)(profile + counter), (unsigned long long)count);
  }
#endif
}

/* Epanechnikov-weighted flux sum from photons within the gather radius,
 * plus the raw photon count for the SPPM statistics update. `r2` is the
 * per-pixel SPPM radius; it starts at the grid cell size and only ever
 * shrinks. Normalization (disc area, generation average) happens in the
 * caller. */
/* Optionally the same sum split by light group. `r_group_sum` points at
 * `group_count` accumulators covering groups [group_base, group_base+count),
 * which the caller clears; every deposit carries its emitting light's group in
 * flux.w. The split rides along inside the ONE traversal that was going to
 * happen anyway - gathering per group separately would multiply the cost of
 * the most expensive per-pixel step in the renderer. Photons outside the
 * window (and ungrouped ones, group -1) still count towards the total, so the
 * caller can always compare the parts against the whole. */
ccl_device float3 photon_grid_gather(KernelGlobals kg,
                                     const float3 P,
                                     const float3 N,
                                     const float r2,
                                     ccl_private int *r_num_gathered,
                                     ccl_private float *r_centroid_offset,
                                     ccl_private float3 *r_group_sum = nullptr,
                                     const int group_base = 0,
                                     const int group_count = 0)
{
  *r_num_gathered = 0;
  *r_centroid_offset = 0.0f;

  /* Keep the density estimate finite when an extreme scene scale or
   * progressive shrink produces a subnormal radius. */
  const float gather_r2 = photon_safe_radius_squared(r2);

  if (kernel_data.integrator.photon_num == 0) {
    return make_float3(0.0f, 0.0f, 0.0f);
  }

  const float inv_cell = kernel_data.integrator.photon_inv_cell;
  const uint table_size = (uint)kernel_data.integrator.photon_table_size;

  const int cx = (int)floorf(P.x * inv_cell);
  const int cy = (int)floorf(P.y * inv_cell);
  const int cz = (int)floorf(P.z * inv_cell);

  /* The per-pixel radius never exceeds the cell size, so the 3x3x3 cell
   * neighborhood always covers the gather disc. */
  const int range = 1;

  float3 sum = make_float3(0.0f, 0.0f, 0.0f);
  float3 weighted_offset = make_float3(0.0f, 0.0f, 0.0f);
  float weight_total = 0.0f;
  int num = 0;

  for (int dz = -range; dz <= range; dz++) {
    for (int dy = -range; dy <= range; dy++) {
      for (int dx = -range; dx <= range; dx++) {
        const uint b = photon_grid_hash(cx + dx, cy + dy, cz + dz, table_size);
        const int p0 = kernel_data_fetch(photon_cell_start, (int)b);
        const int p1 = kernel_data_fetch(photon_cell_start, (int)b + 1);
        for (int p = p0; p < p1; p++) {
          const float4 pp = kernel_data_fetch(photon_pos, p);
          const float3 d = make_float3(pp.x - P.x, pp.y - P.y, pp.z - P.z);
          const float d2 = dot(d, d);
          if (d2 >= gather_r2) {
            continue;
          }
          const float4 flux = kernel_data_fetch(photon_flux, p);
          if (photon_deposit_is_volume(flux.w)) {
            continue;
          }
          /* Reject photons that are radially close but on another surface:
           * distance along the normal must stay well under the radius. */
          const float dn = dot(d, N);
          if (dn * dn > 0.1f * gather_r2) {
            continue;
          }
          /* Reject photons deposited on an opposite-facing surface. */
          if (dot(photon_unpack_normal(pp.w), N) <= 0.0f) {
            continue;
          }
          const float w = 1.0f - d2 / gather_r2; /* Epanechnikov */
          const float3 contribution = w * make_float3(flux.x, flux.y, flux.z);
          sum += contribution;
          if (r_group_sum != nullptr) {
            const int g = (int)floorf(flux.w) - group_base;
            if (g >= 0 && g < group_count) {
              r_group_sum[g] += contribution;
            }
          }
          weighted_offset += w * d;
          weight_total += w;
          num++;
        }
      }
    }
  }

  *r_num_gathered = num;
  if (weight_total > 0.0f) {
    /* Weighted centroid offset relative to the radius: ~0 for a uniform
     * photon field, large when the disc straddles an illumination boundary
     * (waterline, shadow edge) and all photons sit on one side. */
    *r_centroid_offset = len(weighted_offset / weight_total) / sqrtf(gather_r2);
  }
  return sum;
}

/* Beam x beam, 2D blur: Jarosz et al. 2011, Comprehensive Theory, Eq. 29.
 * Integrate both transmittances over the finite cylinder intersection and
 * normalize by its cross section. No 1/sin(theta) singularity is needed. */
ccl_device float3 photon_grid_beam_integral(KernelGlobals kg,
                                             const float3 ray_P,
                                             const float3 ray_D,
                                             const float3 inv_d,
                                             const float tmin,
                                            const float tmax,
                                            const float3 sigma_t,
                                            const float3 sigma_s,
                                            const float volume_g,
                                            const float r2,
                                            const ccl_private ShaderData *sd = nullptr,
                                            const bool skip_root_bounds = false,
                                            ccl_global uint64_t *profile = nullptr)
{
  if (kernel_data.integrator.photon_volume_beam_num == 0 ||
      kernel_data.integrator.photon_volume_beam_node_num == 0 || tmax <= tmin || r2 <= 0.0f)
  {
    return make_float3(0.0f, 0.0f, 0.0f);
  }

  /* === CyclesPlus: Personal Volume Beam Radius Begin === */
  const float radius_scale = max(kernel_data.integrator.photon_volume_beam_radius_scale,
                                 1.0f);
  const float radius2 = r2 * radius_scale * radius_scale;
  /* The Epanechnikov kernel remains normalized by this expanded support area,
   * so widening blends neighboring beams without multiplying total energy. */
  /* === CyclesPlus: Personal Volume Beam Radius End === */
  const float gg = clamp(volume_g, -0.999f, 0.999f);
  const float3 camera_sigma_t = max(sigma_t, make_float3(0.0f));
  /* Neutral media share the same scalar attenuation and integral across RGB. */
  const bool reuse_neutral = camera_sigma_t.x == camera_sigma_t.y &&
                             camera_sigma_t.x == camera_sigma_t.z;
  float3 result = make_float3(0.0f, 0.0f, 0.0f);
  int stack[64];
  int stack_size = 0;
  stack[stack_size++] = 0;
  const float3 ro = ray_P + ray_D * tmin;
  uint nodes = 0, rejects = 0, leaves = 0, candidates = 0, hits = 0, phases = 0;
  /* === CyclesPlus: Beam Integral Reuse Counter Begin === */
  uint integral_reuses = 0;
  /* === CyclesPlus: Beam Integral Reuse Counter End === */
  while (stack_size > 0) {
    if (profile) {
      nodes++;
    }
    const int node_index = stack[--stack_size];
    const KernelPhotonBeamNode node = kernel_data_fetch(photon_volume_beam_nodes, node_index);
    const float3 bmin = node.bmin;
    const float3 bmax = node.bmax;
    const float3 t0 = (bmin - ro) * inv_d;
    const float3 t1 = (bmax - ro) * inv_d;
    const float tn = max(max(min(t0.x, t1.x), min(t0.y, t1.y)), min(t0.z, t1.z));
    const float tf = min(min(max(t0.x, t1.x), max(t0.y, t1.y)), max(t0.z, t1.z));
    /* Refracted camera paths may enter the tree through a numerically stale
     * root interval. Child AABBs and the exact beam test remain authoritative. */
    if ((!skip_root_bounds || node_index != 0) &&
        (tf < max(tn, 0.0f) || tn > (tmax - tmin))) {
      if (profile) {
        rejects++;
      }
      continue;
    }

    if (node.left < 0) {
      if (profile) {
        leaves++;
        candidates += node.right;
      }
      const int first = -node.left - 1;
      for (int i = 0; i < node.right; i++) {
        const int beam = first + i;
        const float4 a4 = kernel_data_fetch(photon_volume_beam_start, beam);
        const float4 b4 = kernel_data_fetch(photon_volume_beam_end, beam);
        const float3 a = make_float3(a4.x, a4.y, a4.z);
        const float3 b = make_float3(b4.x, b4.y, b4.z);
        const float3 v = b - a;
        const float c = dot(v, v);
        if (c <= 1e-20f) {
          photon_volume_profile_add(profile, PHOTON_PROFILE_DEGENERATE_BEAMS);
          continue;
        }
        const float beam_length = b4.w;
        const float3 beam_dir = v / beam_length;
        const float3 w = ro - a;
        const float along = dot(w, beam_dir);
        const float cosine = clamp(dot(ray_D, beam_dir), -1.0f, 1.0f);
        const float3 radial_o = w - along * beam_dir;
        const float3 radial_d = ray_D - cosine * beam_dir;
        const float qa = dot(radial_d, radial_d);
        float near_t = 0.0f, far_t = tmax - tmin;
        if (qa > 1e-12f) {
          const float center_t = -dot(radial_o, radial_d) / qa;
          const float3 closest = radial_o + center_t * radial_d;
          const float radial2 = dot(closest, closest);
          if (radial2 >= radius2) {
            photon_volume_profile_add(profile, PHOTON_PROFILE_RADIAL_REJECTS);
            continue;
          }
          const float half_span = sqrtf((radius2 - radial2) / qa);
          near_t = max(near_t, center_t - half_span);
          far_t = min(far_t, center_t + half_span);
        }
        else if (dot(radial_o, radial_o) >= radius2) {
          photon_volume_profile_add(profile, PHOTON_PROFILE_RADIAL_REJECTS);
          continue;
        }
        if (fabsf(cosine) > 1e-7f) {
          const float cap0 = -along / cosine;
          const float cap1 = (beam_length - along) / cosine;
          near_t = max(near_t, min(cap0, cap1));
          far_t = min(far_t, max(cap0, cap1));
        }
        else if (along < 0.0f || along > beam_length) {
          photon_volume_profile_add(profile, PHOTON_PROFILE_CAP_REJECTS);
          continue;
        }
        if (far_t <= near_t) {
          photon_volume_profile_add(profile, PHOTON_PROFILE_INTERVAL_REJECTS);
          continue;
        }
        if (profile) {
          hits++;
        }

        /* Flux and beam extinction are only needed after the geometric test. */
        const float4 f4 = kernel_data_fetch(photon_volume_beam_flux, beam);
        const float4 sigma4 = kernel_data_fetch(photon_volume_beam_sigma, beam);
        const float3 beam_sigma_t = max(make_float3(sigma4.x, sigma4.y, sigma4.z),
                                        make_float3(0.0f));

        /* Integrate from the endpoint with lower optical depth. Opposing rays
         * can have a negative combined rate; clamping it breaks subdivision. */
        const float span = far_t - near_t;
        const float beam_t = clamp(along + cosine * near_t, 0.0f, beam_length);
        const float beam_far_t = clamp(along + cosine * far_t, 0.0f, beam_length);
        const float3 rate = fabs(camera_sigma_t + beam_sigma_t * cosine);
        const float3 depth_near = camera_sigma_t * near_t + beam_sigma_t * (beam_t + a4.w);
        const float3 depth_far = camera_sigma_t * far_t + beam_sigma_t * (beam_far_t + a4.w);
        /* === CyclesPlus: Beam Integral Reuse Begin === */
        float3 attenuation;
        float3 integral;
        if (reuse_neutral && beam_sigma_t.x == beam_sigma_t.y &&
            beam_sigma_t.x == beam_sigma_t.z) {
          /* Exact channel equality only. Preserve the original scalar operation order,
           * small-rate branch and expm1 stability; colored flux/scattering stays RGB. */
          attenuation = make_float3(expf(-min(depth_near.x, depth_far.x)));
          integral = make_float3(
              fabsf(rate.x * span) < 1e-4f ? span * (1.0f - 0.5f * rate.x * span) :
                                           -expm1f(-rate.x * span) / rate.x);
          if (profile) {
            integral_reuses++;
          }
        }
        else {
          attenuation = exp(-min(depth_near, depth_far));
          integral = make_float3(
            fabsf(rate.x * span) < 1e-4f ? span * (1.0f - 0.5f * rate.x * span) :
                                         -expm1f(-rate.x * span) / rate.x,
            fabsf(rate.y * span) < 1e-4f ? span * (1.0f - 0.5f * rate.y * span) :
                                         -expm1f(-rate.y * span) / rate.y,
            fabsf(rate.z * span) < 1e-4f ? span * (1.0f - 0.5f * rate.z * span) :
                                         -expm1f(-rate.z * span) / rate.z);
        }
        /* === CyclesPlus: Beam Integral Reuse End === */
        /* Use a normalized Epanechnikov footprint across the beam cross
         * section. Simpson sampling along the camera interval preserves the
         * unit integral while replacing the hard cylinder edge that exposes
         * individual photon beams as laser-like strips. */
        const float tmid = 0.5f * (near_t + far_t);
        const float q0 = dot(radial_o + radial_d * near_t, radial_o + radial_d * near_t);
        const float qm = dot(radial_o + radial_d * tmid, radial_o + radial_d * tmid);
        const float q1 = dot(radial_o + radial_d * far_t, radial_o + radial_d * far_t);
        const float kernel0 = 2.0f * fmaxf(0.0f, 1.0f - q0 / radius2);
        const float kernelm = 2.0f * fmaxf(0.0f, 1.0f - qm / radius2);
        const float kernel1 = 2.0f * fmaxf(0.0f, 1.0f - q1 / radius2);
        const float radial_kernel = (kernel0 + 4.0f * kernelm + kernel1) / 6.0f;
        float3 scattering;
        if (sd != nullptr) {
          /* Actual mixed closures include texture weights and colored scattering. */
          Spectrum phase_sum = zero_spectrum();
          for (int ci = 0; ci < sd->num_closure; ci++) {
            const ccl_private ShaderClosure *sc = &sd->closure[ci];
            if (CLOSURE_IS_VOLUME_SCATTER(sc->type)) {
              if (profile) {
                phases++;
              }
              float pdf;
              phase_sum += sc->weight * volume_phase_eval(
                  sd, (const ccl_private ShaderVolumeClosure *)sc, -beam_dir, &pdf);
            }
          }
          scattering = spectrum_to_rgb(phase_sum);
        }
        else {
          const float cos_theta = -cosine;
          const float denom_phase = 1.0f + gg * gg - 2.0f * gg * cos_theta;
          const float phase = (1.0f - gg * gg) /
                              (4.0f * M_PI_F * denom_phase * sqrtf(max(denom_phase, 1e-12f)));
          scattering = phase * sigma_s;
        }
        result += (scattering / (M_PI_F * radius2)) * radial_kernel * attenuation * integral *
                  make_float3(f4.x, f4.y, f4.z);
      }
    }
    else {
      if (stack_size + 2 <= 64) {
        stack[stack_size++] = node.left;
        stack[stack_size++] = node.right;
      }
      else {
        photon_volume_profile_add(profile, PHOTON_PROFILE_STACK_OVERFLOWS);
      }
    }
  }
  photon_volume_profile_add(profile, PHOTON_PROFILE_QUERIES);
  photon_volume_profile_add(profile, PHOTON_PROFILE_EMPTY_QUERIES, hits == 0);
  photon_volume_profile_add(profile, PHOTON_PROFILE_NODES, nodes);
  photon_volume_profile_add(profile, PHOTON_PROFILE_NODE_REJECTS, rejects);
  photon_volume_profile_add(profile, PHOTON_PROFILE_LEAVES, leaves);
  photon_volume_profile_add(profile, PHOTON_PROFILE_CANDIDATES, candidates);
  photon_volume_profile_add(profile, PHOTON_PROFILE_HITS, hits);
  photon_volume_profile_add(profile, PHOTON_PROFILE_HG_EVALS, sd == nullptr ? hits : 0);
  photon_volume_profile_add(profile, PHOTON_PROFILE_CLOSURE_EVALS, phases);
  /* === CyclesPlus: Beam Integral Reuse Counter Flush Begin === */
  photon_volume_profile_add(profile, PHOTON_PROFILE_INTEGRAL_REUSES, integral_reuses);
  /* === CyclesPlus: Beam Integral Reuse Counter Flush End === */
  return result;
}

CCL_NAMESPACE_END
/* === CyclesPlus: Personal Photon Lookup End === */
