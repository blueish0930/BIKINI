/* SPDX-FileCopyrightText: 2026 CyclesPlus
 * SPDX-FileCopyrightText: 2026 BIKINI
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* === CyclesPlus: Personal Photon Passes Begin === */
/* Photon caustics: per-pixel SPPM statistics (stochastic progressive photon
 * mapping, Hachisuka/Jensen 2009).
 *
 * The path kernel only records one measurement point per pixel and sample
 * (film_write_photon_hitpoint). The photon gather runs as a separate film
 * kernel exactly once per published photon generation, between render works
 * (film_photon_gather_pixel): it looks up the photon batch at the pixel's own
 * gather radius, then updates the pixel statistics in place
 *
 *   N'   = N + alpha * M
 *   r'^2 = r^2 * (N + alpha * M) / (N + M)
 *   tau' = (tau + f_r * Phi) * r'^2 / r^2
 *
 * so the radius only ever shrinks and the estimate converges in place instead
 * of relying on film averaging over independent batches. The shrink stops at
 * a radius floor of r0/4; from there on the estimate is a plain average of
 * constant-radius gathers (see the floor comment in the update below).
 *
 * The film shows the current SPPM state through a delta write into the
 * combined pass: with S samples accumulated, the pass holds S * E for the
 * current estimate E, so writing S * E_new - (previous write) makes the
 * running film average display E exactly. The last written sum lives in the
 * state pass, which also makes the write self-correcting. */

#pragma once

#include "kernel/film/write.h"
#include "kernel/integrator/photon_lookup.h"

CCL_NAMESPACE_BEGIN

/* Radius shrink rate. Same alpha as classic progressive photon mapping. */
#define PHOTON_SPPM_ALPHA 0.7f

/* The radius only starts shrinking once this many (alpha-weighted) photons
 * back the local density estimate. Densely lit caustic areas pass this in
 * one generation; sparsely lit receivers (a floaty catching water-surface
 * reflections) would otherwise shrink by ~0.7 per generation regardless of
 * density until the disc is tiny and single photons explode into huge
 * colored speckles - visibly worsening as the render progresses. */
#define PHOTON_SPPM_MIN_PHOTONS_FOR_SHRINK 16.0f

/* NOTE: an earlier version adaptively GREW sparse pixels' discs (up to 16x
 * the start area). Measurably useless against its target (speckle - solved
 * by per-gather tau averaging instead) while widening the boundary-leak
 * band at waterlines fourfold. Removed. */

/* The tau average weighs every gather by the current generation's size
 * relative to a full batch, in 1/16th steps (interactive sessions ramp the
 * batch size up after scene changes). A full-batch gather adds this much
 * weight; the estimate divides by accumulated weight / this. For all-full
 * batches (final renders) the estimator is identical to plain per-gather
 * averaging. */
#define PHOTON_SPPM_WEIGHT_FULL 16.0f

/* Below this accumulated per-pixel gather weight (the equivalent of 32
 * full-batch gathers) the caustic estimate counts as immature: its delta is
 * kept out of the adaptive sampling aux buffer, so the convergence error
 * sees the full caustic and the sampler cannot retire the pixel yet.
 * Retired pixels freeze their SPPM state, so this bounds the noise that can
 * get frozen in. Pixels without caustics are unaffected. */
#define PHOTON_SPPM_MIN_WEIGHT (32.0f * PHOTON_SPPM_WEIGHT_FULL)

/* Octahedral normal pack/unpack lives in kernel/integrator/photon_grid.h,
 * shared with the host-side photon tracer (deposit normals). */

/* The hitpoint marker (component 3 of the hitpoint pass) packs two things:
 * the integer part is the pixel's accumulated gather weight (in 1/16th-batch
 * units, see PHOTON_SPPM_WEIGHT_FULL), a fractional 0.5 flags a fresh
 * measurement point since the last gather. */

/* Clear the fresh flag when a pixel starts a new sample, so at gather time
 * the hitpoint reflects the most recent sample's outcome (hit or miss) and
 * partially covered pixels get coverage-proportional caustics. Converged
 * pixels (adaptive sampling) stop sampling, keep no fresh flag and are
 * frozen by the gather - re-consuming a stale hitpoint would converge to
 * one lens/AA sample instead of the pixel average (blocky patches with
 * depth of field). */
ccl_device_inline void film_clear_photon_hitpoint(KernelGlobals kg,
                                                  ConstIntegratorState state,
                                                  ccl_global float *ccl_restrict render_buffer)
{
  if (kernel_data.film.pass_photon_hitpoint == PASS_UNUSED) {
    return;
  }
  ccl_global float *buffer = film_pass_pixel_render_buffer(kg, state, render_buffer);
  ccl_global float *marker = buffer + kernel_data.film.pass_photon_hitpoint + 3;
  *marker = floorf(*marker);
}

/* Record the SPPM measurement point: the first diffuse hit of this sample
 * that qualifies for photon caustics. `weight` premultiplies throughput and
 * diffuse_albedo / pi, so the gather turns irradiance into pixel radiance
 * without shader access. */
ccl_device_inline void film_write_photon_hitpoint(KernelGlobals kg,
                                                  ConstIntegratorState state,
                                                  const float3 P,
                                                  const float3 N,
                                                  const Spectrum weight,
                                                  ccl_global float *ccl_restrict render_buffer)
{
  if (kernel_data.film.pass_photon_hitpoint == PASS_UNUSED) {
    return;
  }
  /* Only the work's designated writer sample may touch the hitpoint (see
   * PATH_RAY_PHOTON_HITPOINT_WRITER) - concurrent same-pixel samples on GPU
   * otherwise tear the P/normal/weight triplet. */
  if (!(INTEGRATOR_STATE(state, path, flag) & PATH_RAY_PHOTON_HITPOINT_WRITER)) {
    return;
  }
  ccl_global float *buffer = film_pass_pixel_render_buffer(kg, state, render_buffer);

  ccl_global float *hit = buffer + kernel_data.film.pass_photon_hitpoint;
  hit[0] = P.x;
  hit[1] = P.y;
  hit[2] = P.z;
  hit[3] = floorf(hit[3]) + 0.5f; /* Keep G, set the fresh flag. */

  /* Clamp the weight like Cycles clamps indirect light: measurement points
   * reached through rough transmission or SSS chains carry f/pdf throughput
   * spikes far above 1, and one such weight multiplies the full photon flux
   * of every following generation - persistent colored confetti on such
   * materials. Ordinary weights (albedo/pi scale) stay untouched. */
  float3 w = spectrum_to_rgb(weight);
  const float clamp_val = kernel_data.integrator.sample_clamp_indirect;
  if (clamp_val > 0.0f) {
    const float m = fmaxf(w.x, fmaxf(w.y, w.z));
    if (m > clamp_val) {
      w *= clamp_val / m;
    }
  }

  ccl_global float *wgt = buffer + kernel_data.film.pass_photon_weight;
  wgt[0] = w.x;
  wgt[1] = w.y;
  wgt[2] = w.z;
  wgt[3] = photon_pack_normal(N);
}

/* Whether adaptive sampling may retire this pixel. Retirement freezes the
 * per-pixel SPPM state, so before the photon statistics are trustworthy it
 * must not happen: pixels whose caustic simply has not ARRIVED yet (the
 * interactive batch ramp starts with small generations) would freeze
 * caustic-less and turn into dark patches/tiles. Pixels that already carry
 * a caustic are protected separately by the immature-delta mirroring below;
 * this guard covers the zero-caustic-so-far case, which is only decidable
 * once generations reached their steady-state size (then "no photons seen"
 * really means "no caustic here", like the old full-first-batch contract). */
ccl_device_inline bool film_photon_adaptive_can_retire(KernelGlobals kg,
                                                       const ccl_global float *buffer)
{
  if (!kernel_data.integrator.use_photon_caustics ||
      kernel_data.film.pass_photon_hitpoint == PASS_UNUSED)
  {
    return true;
  }
  if (kernel_data.integrator.photon_batch_steady) {
    return true;
  }
  const float marker = buffer[kernel_data.film.pass_photon_hitpoint + 3];
  return floorf(marker) >= PHOTON_SPPM_MIN_WEIGHT;
}

/* Mirror the caustic delta into the user-visible caustics pass.
 *
 * The caustic reaches the image as a DELTA into the combined pass (see the
 * header note), and it is deliberately not added to any of the light passes -
 * so without this pass Diffuse + Glossy + Transmission + ... no longer sum to
 * Combined, and the difference is exactly the caustic. Feeding the same delta
 * here restores that decomposition and lets compositing grade the caustics on
 * their own: Combined - Caustics is the render without them.
 *
 * Same delta, same `last_written` bookkeeping, so the two passes cannot drift
 * apart. Free when the pass is not enabled. */
ccl_device_inline void film_write_photon_caustics_pass(KernelGlobals kg,
                                                       ccl_global float *buffer,
                                                       const float3 target,
                                                       const float3 last_written)
{
  if (kernel_data.film.pass_caustics == PASS_UNUSED) {
    return;
  }
  ccl_global float *caustics = buffer + kernel_data.film.pass_caustics;
  caustics[0] += target.x - last_written.x;
  caustics[1] += target.y - last_written.y;
  caustics[2] += target.z - last_written.z;
}

/* Per-light-group caustics (CyclesPlus): fold one gather's flux into each
 * group's running tau.
 *
 * `corr` is every scalar the TOTAL contribution was corrected by (boundary
 * suppression, clamp, batch weight) and `ratio` is the SPPM radius rescale the
 * total tau just received. Both are handed down rather than recomputed per
 * group, which is what makes the split exact: the recursion
 * tau <- (tau + contribution) * ratio is linear in tau, so applying the same
 * factors to every share keeps sum(tau_g) equal to tau by construction. A
 * group cannot drift on its own - if the sum test ever fails, something real
 * is broken rather than merely imprecise. */
ccl_device_inline void film_photon_update_tau_groups(KernelGlobals kg,
                                                     ccl_global float *buffer,
                                                     const ccl_private float3 *group_flux,
                                                     const int base,
                                                     const int count,
                                                     const float3 weight,
                                                     const float corr,
                                                     const float ratio)
{
  ccl_global float *tau_g = buffer + kernel_data.film.pass_photon_tau_group;
  for (int i = 0; i < count; i++) {
    const float3 c = group_flux[i] * weight * corr;
    const int o = 3 * (base + i);
    tau_g[o + 0] = (tau_g[o + 0] + c.x) * ratio;
    tau_g[o + 1] = (tau_g[o + 1] + c.y) * ratio;
    tau_g[o + 2] = (tau_g[o + 2] + c.z) * ratio;
  }
}

/* Write each group's share of the caustic. `scale` turns a group's tau into
 * the same quantity the total's `target` holds - including the sample count,
 * the intensity control and, where display smoothing is active, the ratio the
 * total was smoothed by, so that the groups follow the total exactly.
 *
 * Written absolutely, not as a delta: nothing else writes these passes, so
 * there is no foreign running sum to correct against. That also saves the
 * three floats per pixel and group a `last_written` would have cost - at 4K
 * and a handful of groups, real memory. */
ccl_device_inline void film_write_photon_caustics_groups(KernelGlobals kg,
                                                         ccl_global float *buffer,
                                                         const float3 scale)
{
  const int num_groups = kernel_data.film.num_caustics_lightgroups;
  if (num_groups == 0 || kernel_data.film.pass_caustics_lightgroup == PASS_UNUSED ||
      kernel_data.film.pass_photon_tau_group == PASS_UNUSED)
  {
    return;
  }
  ccl_global float *out = buffer + kernel_data.film.pass_caustics_lightgroup;
  const ccl_global float *tau_g = buffer + kernel_data.film.pass_photon_tau_group;
  for (int g = 0; g < num_groups; g++) {
    out[3 * g + 0] = tau_g[3 * g + 0] * scale.x;
    out[3 * g + 1] = tau_g[3 * g + 1] * scale.y;
    out[3 * g + 2] = tau_g[3 * g + 2] * scale.z;
  }
}

/* One SPPM update for one pixel. Runs in the film photon gather kernel,
 * single thread per pixel and never concurrently with path kernels.
 *
 * `consume` != 0 folds the current photon generation into the statistics
 * (exactly once per published generation); `consume` == 0 only rewrites the
 * display delta so the estimate keeps its strength as the sample count
 * grows between generations.
 *
 * A generation only counts for a pixel with a FRESH measurement point (a
 * sample ran since the last gather). Pixels the adaptive sampler retired
 * are frozen: they keep the unbiased average accumulated while they were
 * active. Re-consuming their stale hitpoint instead would keep shrinking
 * the radius and converge to one AA/lens sample of the caustic rather than
 * the pixel average - visible as blocky patches that get worse the longer
 * the render runs. Sampled pixels whose path found no measurement point
 * count the generation with zero contribution, which bakes coverage into
 * the estimate the same way sample averaging does.
 *
 * `num_samples` is the fallback when there is no per-pixel sample count
 * pass; it must match the value the pass accessor / denoiser divides the
 * combined pass by. Temporal DLSS zeros the film every work and passes 1
 * here (get_num_samples_for_buffer): writing the growing rendered-sample
 * count would bake N*E into a 1spp buffer and the NGX history would keep
 * adding it. */
ccl_device void film_photon_gather_pixel(KernelGlobals kg,
                                         ccl_global float *ccl_restrict render_buffer,
                                         const int x,
                                         const int y,
                                         const int offset,
                                         const int stride,
                                         const int num_samples,
                                         const int consume)
{
  if (kernel_data.film.pass_photon_hitpoint == PASS_UNUSED ||
      kernel_data.film.pass_combined == PASS_UNUSED)
  {
    return;
  }

  /* No photon grid uploaded (yet): do not count a generation. The photon
   * map rebuilds after scene changes; counting the grid-less gathers as
   * zero-contribution generations would dilute the estimate and make
   * caustics start too dark and heal only asymptotically. */
  if (kernel_data.integrator.photon_num == 0) {
    return;
  }

  const int render_pixel_index = offset + x + y * stride;
  ccl_global float *buffer = render_buffer +
                             (uint64_t)render_pixel_index * kernel_data.film.pass_stride;

  ccl_global float *tau_pass = buffer + kernel_data.film.pass_photon_tau;
  ccl_global float *state_pass = buffer + kernel_data.film.pass_photon_state;
  ccl_global float *hit = buffer + kernel_data.film.pass_photon_hitpoint;

  const float r0 = kernel_data.integrator.photon_radius;
  float r2 = (state_pass[0] > 0.0f) ? state_pass[0] : r0 * r0;
  float3 tau = make_float3(tau_pass[0], tau_pass[1], tau_pass[2]);
  float num_photons = tau_pass[3];

  /* Per-pixel accumulated gather weight (integer, in 1/16th-batch units)
   * and fresh flag, packed in the marker. */
  float gather_weight = floorf(hit[3]);
  const bool fresh = (hit[3] - gather_weight) != 0.0f;
  const bool was_mature = gather_weight >= PHOTON_SPPM_MIN_WEIGHT;
  const float batch_weight = (kernel_data.integrator.photon_heuristic_mask & 4) ?
                                 (float)kernel_data.integrator.photon_batch_weight :
                                 PHOTON_SPPM_WEIGHT_FULL;

  /* Tau accumulates one estimate per GATHER (every render work), not only
   * per consumed photon generation: through curved specular surfaces the
   * measurement point jumps chaotically between samples, and averaging
   * only per generation leaves persistent colored speckles there. Re-using
   * the same photons at a fresh measurement point adds correlated but
   * spatially independent information, exactly what those pixels lack.
   * Photon-count and radius statistics still update only on `consume`
   * (once per photon generation). */
  if (fresh) {
    ccl_global float *wgt = buffer + kernel_data.film.pass_photon_weight;
    const float3 weight = make_float3(wgt[0], wgt[1], wgt[2]);
    if (!is_zero(weight)) {
      const float3 P = make_float3(hit[0], hit[1], hit[2]);
      const float3 N = photon_unpack_normal(wgt[3]);
      int gathered = 0;
      float centroid_offset = 0.0f;

      /* Per-light-group caustics (CyclesPlus). The split rides along in the
       * traversal that happens anyway; only scenes with more groups than fit
       * in one window pay for a second pass over the grid. */
      const int num_groups = kernel_data.film.num_caustics_lightgroups;
      const bool split_groups = (num_groups > 0 &&
                                 kernel_data.film.pass_photon_tau_group != PASS_UNUSED);
      const int first_chunk = (num_groups < PHOTON_LIGHTGROUP_CHUNK) ? num_groups :
                                                                       PHOTON_LIGHTGROUP_CHUNK;
      float3 group_flux[PHOTON_LIGHTGROUP_CHUNK];
      if (split_groups) {
        for (int i = 0; i < PHOTON_LIGHTGROUP_CHUNK; i++) {
          group_flux[i] = make_float3(0.0f, 0.0f, 0.0f);
        }
      }
      /* The radius the gather actually used: the SPPM update below shrinks r2,
       * and any follow-up chunk has to see the same disc as this one. */
      const float r2_gather = r2;

      const float3 flux = photon_grid_gather(kg,
                                             P,
                                             N,
                                             r2,
                                             &gathered,
                                             &centroid_offset,
                                             split_groups ? group_flux : nullptr,
                                             0,
                                             first_chunk);
      if (gathered > 0) {
        float3 contribution = flux * weight;
        /* Everything the total contribution gets scaled by, collected so the
         * group shares can receive exactly the same treatment. The statements
         * on `contribution` are deliberately left as they were - the whole
         * reference battery is pinned to their rounding. */
        float group_corr = 1.0f;

        /* Boundary-leak suppression: when the photons cluster far to one
         * side of the disc (beyond ~2 sigma of a uniform field), the disc
         * straddles an illumination discontinuity - e.g. above-waterline
         * points sucking in the bright submerged deposits of a floating
         * object. Only applied with enough photons for the statistics. */
        if ((kernel_data.integrator.photon_heuristic_mask & 1) && gathered >= 8) {
          const float sigma = 0.35f / sqrtf((float)gathered);
          const float excess = fmaxf(0.0f, centroid_offset - 2.0f * sigma);
          const float suppress = fmaxf(0.0f, 1.0f - 3.0f * excess);
          contribution *= suppress;
          group_corr *= suppress;
        }

        /* Clamp this estimate like Cycles clamps indirect light samples,
         * so a single extreme gather cannot poison tau for the rest of
         * the render.
         *
         * The threshold has to account for how many photons back the disc.
         * A gather holding one photon reports radiance ~1/M times the true
         * mean (M = expected photons per disc) - not because it caught a
         * firefly, but because that IS the shape of a sparse density
         * estimate. Clamping it anyway clips the signal instead of an
         * outlier, and since the clipped excess is simply dropped, the
         * caustic loses energy in exactly the regions that have the fewest
         * photons. Measured on a daylight interior (window glass as caster,
         * gather radius 1.7mm at Detail 10): the clamp removed 52% of the
         * caustic on a dark counter and 32% on the floor, and the loss
         * scaled with the Detail slider - so a sharpness control silently
         * changed brightness.
         *
         * Allowing the threshold to rise as the disc empties keeps the
         * firefly brake where the density is trustworthy and stops it from
         * eating the estimate where it is not. The reference count is the
         * one this file already uses for "enough photons to trust the
         * density"; at or above it the clamp is bit-identical to before. */
        const float clamp_val = kernel_data.integrator.sample_clamp_indirect;
        if ((kernel_data.integrator.photon_heuristic_mask & 2) && clamp_val > 0.0f) {
          const float to_radiance = 2.0f / (M_PI_F * r2);
          const float m = fmaxf(contribution.x, fmaxf(contribution.y, contribution.z)) *
                          to_radiance;
          const float sparse = PHOTON_SPPM_MIN_PHOTONS_FOR_SHRINK /
                               fmaxf((float)gathered, 1.0f);
          const float clamp_eff = clamp_val * fmaxf(sparse, 1.0f);
          if (m > clamp_eff) {
            const float limit = clamp_eff / m;
            contribution *= limit;
            group_corr *= limit;
          }
        }

        /* Weigh this gather by the generation's relative size, so full
         * generations dominate the average over grainy ramp-up previews. */
        const float batch_scale = batch_weight / PHOTON_SPPM_WEIGHT_FULL;
        contribution *= batch_scale;
        group_corr *= batch_scale;

        /* The radius rescale the total tau receives below; the groups get the
         * identical factor, which is what keeps them summing to the total. */
        float tau_ratio = 1.0f;

        if (consume) {
          const float new_photons = num_photons + PHOTON_SPPM_ALPHA * (float)gathered;
          if (new_photons >= PHOTON_SPPM_MIN_PHOTONS_FOR_SHRINK) {
            /* The radius never shrinks below (r0/4)^2. The alpha schedule was
             * calibrated against ~10s CPU generations; GPU generations land
             * every few hundred ms, so an unbounded schedule racks up hundreds
             * of shrink steps within minutes and collapses the discs into
             * sub-pixel filaments whose gathers mostly miss - the caustic
             * visibly fades the longer the render runs. At the floor the
             * estimate continues as plain tau averaging at constant radius
             * (no ratio rescale), which is time-stable; r0 carries the Detail
             * control, so the floor scales with it. */
            const float r2_floor = (kernel_data.integrator.photon_heuristic_mask & 8) ?
                                       r0 * r0 * (1.0f / 16.0f) :
                                       0.0f;
            if (r2 > r2_floor) {
              const float ratio = fmaxf(new_photons / (num_photons + (float)gathered),
                                        r2_floor / r2);
              tau = (tau + contribution) * ratio;
              r2 *= ratio;
              tau_ratio = ratio;
            }
            else {
              tau += contribution;
            }
          }
          else {
            /* Too sparse to trust the density: average at constant radius. */
            tau += contribution;
          }
          num_photons = new_photons;
        }
        else {
          tau += contribution;
        }

        /* Same flux, same corrections, same ratio - split by light group. */
        if (split_groups) {
          film_photon_update_tau_groups(
              kg, buffer, group_flux, 0, first_chunk, weight, group_corr, tau_ratio);

          /* More groups than fit in one window: walk the grid again per extra
           * window, at the radius this gather used. Rare, and it costs only
           * the scenes that ask for it - nothing is dropped for being late. */
          for (int base = first_chunk; base < num_groups; base += PHOTON_LIGHTGROUP_CHUNK) {
            const int left = num_groups - base;
            const int count = (left < PHOTON_LIGHTGROUP_CHUNK) ? left : PHOTON_LIGHTGROUP_CHUNK;
            for (int i = 0; i < PHOTON_LIGHTGROUP_CHUNK; i++) {
              group_flux[i] = make_float3(0.0f, 0.0f, 0.0f);
            }
            int extra_gathered = 0;
            float extra_offset = 0.0f;
            photon_grid_gather(
                kg, P, N, r2_gather, &extra_gathered, &extra_offset, group_flux, base, count);
            film_photon_update_tau_groups(
                kg, buffer, group_flux, base, count, weight, group_corr, tau_ratio);
          }
        }
      }
    }
    gather_weight += batch_weight;
    hit[3] = gather_weight; /* Gathered: clear the fresh flag. */
  }
  else {
    /* No fresh measurement point. If the pixel is still being sampled, its
     * last sample found no qualifying hit: count the gather with zero
     * contribution (coverage). Retired pixels stay frozen. */
    bool retired = false;
    if (kernel_data.film.pass_adaptive_aux_buffer != PASS_UNUSED) {
      retired = buffer[kernel_data.film.pass_adaptive_aux_buffer + 3] != 0.0f;
    }
    if (!retired) {
      gather_weight += batch_weight;
      hit[3] = gather_weight;
    }
  }

  /* Persist the statistics first: the display path may be deferred to the
   * smoothing kernel, which reconstructs the raw estimate from them. */
  tau_pass[0] = tau.x;
  tau_pass[1] = tau.y;
  tau_pass[2] = tau.z;
  tau_pass[3] = num_photons;
  state_pass[0] = r2;

  if (kernel_data.integrator.photon_heuristic_mask & 16) {
    /* Display smoothing enabled: FILM_PHOTON_SMOOTH writes the combined
     * delta and the aux mirror after this sweep. Flag a maturity TRANSITION
     * for it as marker fraction 0.25 (the writer's fresh flag uses 0.5). */
    const bool mature_now = gather_weight >= PHOTON_SPPM_MIN_WEIGHT;
    if (mature_now && !was_mature) {
      hit[3] = gather_weight + 0.25f;
    }
    return;
  }

  /* Current SPPM radiance estimate: the Epanechnikov disc integral is
   * pi * r^2 / 2, tau is the weighted average over this pixel's gathers. */
  const float tau_to_radiance = 2.0f * PHOTON_SPPM_WEIGHT_FULL *
                                kernel_data.integrator.photon_intensity /
                                (max(gather_weight, 1.0f) * M_PI_F * r2);
  const float3 estimate = tau * tau_to_radiance;

  /* Delta write so the film's running average displays the estimate.
   * `samples` is the count THIS buffer currently holds (1 after a temporal
   * DLSS reset, N when Cycles is accumulating), not the session's total. */
  float samples = (float)num_samples;
  if (kernel_data.film.pass_sample_count != PASS_UNUSED) {
    samples = (float)__float_as_uint(buffer[kernel_data.film.pass_sample_count]);
  }
  const float3 target = estimate * samples;
  const float3 last_written = make_float3(state_pass[1], state_pass[2], state_pass[3]);

  ccl_global float *combined = buffer + kernel_data.film.pass_combined;
  combined[0] += target.x - last_written.x;
  combined[1] += target.y - last_written.y;
  combined[2] += target.z - last_written.z;

  film_write_photon_caustics_pass(kg, buffer, target, last_written);

  /* The groups go through the very same normalisation, so they add back up to
   * the pass above. No smoothing ratio here - this branch is the unsmoothed
   * one. */
  const float group_scale = tau_to_radiance * samples;
  film_write_photon_caustics_groups(
      kg, buffer, make_float3(group_scale, group_scale, group_scale));

  /* Mirror the delta into the adaptive sampling aux buffer (it estimates the
   * same mean from half the samples), so a MATURE caustic cancels out of the
   * convergence error instead of keeping the pixel forever active. Immature
   * pixels are deliberately left unmirrored: the error then sees the full
   * caustic and the sampler cannot retire them before the estimate is worth
   * freezing. The first mature update writes the full target once, bringing
   * the aux caustic in sync so later deltas keep both sides identical. */
  if (kernel_data.film.pass_adaptive_aux_buffer != PASS_UNUSED) {
    const bool mature = gather_weight >= PHOTON_SPPM_MIN_WEIGHT;
    if (mature) {
      ccl_global float *aux = buffer + kernel_data.film.pass_adaptive_aux_buffer;
      if (!was_mature) {
        aux[0] += target.x;
        aux[1] += target.y;
        aux[2] += target.z;
      }
      else {
        aux[0] += target.x - last_written.x;
        aux[1] += target.y - last_written.y;
        aux[2] += target.z - last_written.z;
      }
    }
  }

  state_pass[1] = target.x;
  state_pass[2] = target.y;
  state_pass[3] = target.z;
}

/* Display smoothing (CyclesPlus step 2): maturity-faded cross-bilateral
 * filter over the RAW SPPM estimates, reconstructed per neighbor from the
 * statistics passes - no extra storage. Display-only: tau/N/r2 are not
 * touched, so the converged image is exactly the unfiltered one (the blend
 * weight lambda hits zero at the same maturity that allows adaptive
 * retirement). Runs as its own film kernel after the gather sweep. */
ccl_device void film_photon_smooth_pixel(KernelGlobals kg,
                                         ccl_global float *ccl_restrict render_buffer,
                                         const int x,
                                         const int y,
                                         const int sx,
                                         const int sy,
                                         const int sw,
                                         const int sh,
                                         const int offset,
                                         const int stride,
                                         const int num_samples)
{
  if (kernel_data.film.pass_photon_hitpoint == PASS_UNUSED ||
      kernel_data.film.pass_combined == PASS_UNUSED ||
      kernel_data.integrator.photon_num == 0 ||
      !(kernel_data.integrator.photon_heuristic_mask & 16))
  {
    return;
  }

  const uint64_t pass_stride = kernel_data.film.pass_stride;
  ccl_global float *buffer = render_buffer +
                             (uint64_t)(offset + x + y * stride) * pass_stride;
  ccl_global float *hit = buffer + kernel_data.film.pass_photon_hitpoint;
  ccl_global float *tau_pass = buffer + kernel_data.film.pass_photon_tau;
  ccl_global float *state_pass = buffer + kernel_data.film.pass_photon_state;
  ccl_global float *wgt = buffer + kernel_data.film.pass_photon_weight;

  const float marker = hit[3];
  float gather_weight = floorf(marker);
  const float frac = marker - gather_weight;
  const bool transition = (frac > 0.2f && frac < 0.3f);
  if (transition) {
    hit[3] = gather_weight; /* consume the transition flag */
  }
  if (gather_weight < 1.0f) {
    return; /* no statistics yet */
  }

  const float r0 = kernel_data.integrator.photon_radius;
  const float r2_c = (state_pass[0] > 0.0f) ? state_pass[0] : r0 * r0;
  const float tau_to_radiance = 2.0f * PHOTON_SPPM_WEIGHT_FULL /
                                (max(gather_weight, 1.0f) * M_PI_F * r2_c);
  const float3 raw = make_float3(tau_pass[0], tau_pass[1], tau_pass[2]) * tau_to_radiance;

  const bool mature = gather_weight >= PHOTON_SPPM_MIN_WEIGHT;
  const float lambda = mature ? 0.0f : 1.0f - gather_weight / PHOTON_SPPM_MIN_WEIGHT;

  /* Per-light-group caustics (CyclesPlus): the groups are filtered in the very
   * same loop, with the very same weights. The filter is linear in the values
   * it blends, so sum(shown_g) == shown exactly - the split survives smoothing
   * for the same reason it survives the SPPM recursion.
   *
   * Deriving the groups from the total instead (scale each by shown/raw) looks
   * cheaper and is wrong: where a pixel has no photons of its own, raw is zero
   * and there is no proportion to distribute the smoothed value by. That left
   * the groups short by up to 6% of the peak along caustic edges, exactly
   * where the filter does its work. Only immature pixels pay for this - at
   * lambda 0 there is nothing to filter and the groups are their raw share. */
  const int num_groups = kernel_data.film.num_caustics_lightgroups;
  const bool split_groups = (num_groups > 0 &&
                             kernel_data.film.pass_photon_tau_group != PASS_UNUSED &&
                             kernel_data.film.pass_caustics_lightgroup != PASS_UNUSED);
  /* Same sample count as the gather delta (buffer spp, 1 under temporal DLSS). */
  float samples = (float)num_samples;
  if (kernel_data.film.pass_sample_count != PASS_UNUSED) {
    samples = (float)__float_as_uint(buffer[kernel_data.film.pass_sample_count]);
  }
  /* Two different starting points, so two scales - mixing them up applies
   * tau_to_radiance twice and the groups come out ~100x too bright. The
   * filtered path below starts from a RADIANCE (raw_g already carries
   * tau_to_radiance); the mature path starts from tau itself. */
  const float radiance_to_target = samples * kernel_data.integrator.photon_intensity;
  const float tau_to_target = tau_to_radiance * radiance_to_target;

  float3 shown = raw;
  if (lambda > 0.0f) {
    const float3 P_c = make_float3(hit[0], hit[1], hit[2]);
    const float3 N_c = photon_unpack_normal(wgt[3]);
    const float inv_r02 = 1.0f / max(r0 * r0, 1e-12f);

    /* One pass over the neighbourhood per group window; the total is only
     * accumulated on the first. Scenes with few groups - all of them, in
     * practice - run this exactly once. */
    const int num_chunks = split_groups ? (num_groups + PHOTON_LIGHTGROUP_CHUNK - 1) /
                                              PHOTON_LIGHTGROUP_CHUNK :
                                          1;
    for (int chunk = 0; chunk < num_chunks; chunk++) {
      const int gbase = chunk * PHOTON_LIGHTGROUP_CHUNK;
      const int gcount = split_groups ? min(num_groups - gbase, PHOTON_LIGHTGROUP_CHUNK) : 0;

      float3 acc = make_float3(0.0f, 0.0f, 0.0f);
      float3 acc_g[PHOTON_LIGHTGROUP_CHUNK];
      for (int i = 0; i < PHOTON_LIGHTGROUP_CHUNK; i++) {
        acc_g[i] = make_float3(0.0f, 0.0f, 0.0f);
      }
      float wsum = 0.0f;

      for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < sx || nx >= sx + sw || ny < sy || ny >= sy + sh) {
            continue;
          }
          const ccl_global float *nbuf = render_buffer +
                                         (uint64_t)(offset + nx + ny * stride) * pass_stride;
          const ccl_global float *nhit = nbuf + kernel_data.film.pass_photon_hitpoint;
          const float ngw = floorf(nhit[3]);
          if (ngw < 1.0f) {
            continue;
          }
          const ccl_global float *ntau = nbuf + kernel_data.film.pass_photon_tau;
          const ccl_global float *nstate = nbuf + kernel_data.film.pass_photon_state;
          const ccl_global float *nwgt = nbuf + kernel_data.film.pass_photon_weight;
          const float nr2 = (nstate[0] > 0.0f) ? nstate[0] : r0 * r0;
          const float nk = 2.0f * PHOTON_SPPM_WEIGHT_FULL / (max(ngw, 1.0f) * M_PI_F * nr2);
          const float3 nraw = make_float3(ntau[0], ntau[1], ntau[2]) * nk;
          const float3 dP = make_float3(nhit[0], nhit[1], nhit[2]) - P_c;
          const float ndot = max(dot(N_c, photon_unpack_normal(nwgt[3])), 0.0f);
          const float w = expf(-dot(dP, dP) * inv_r02) * ndot * ndot * ndot * ndot *
                          min(ngw * (1.0f / 16.0f), 1.0f);
          if (chunk == 0) {
            acc += nraw * w;
          }
          if (gcount > 0) {
            const ccl_global float *ntau_g = nbuf + kernel_data.film.pass_photon_tau_group;
            const float gw = nk * w;
            for (int i = 0; i < gcount; i++) {
              const int o = 3 * (gbase + i);
              acc_g[i] += make_float3(ntau_g[o + 0], ntau_g[o + 1], ntau_g[o + 2]) * gw;
            }
          }
          wsum += w;
        }
      }

      if (chunk == 0 && wsum > 1e-6f) {
        shown = raw + (acc * (1.0f / wsum) - raw) * lambda;
      }
      if (gcount > 0) {
        ccl_global float *out = buffer + kernel_data.film.pass_caustics_lightgroup;
        const ccl_global float *tau_g = buffer + kernel_data.film.pass_photon_tau_group;
        const float inv_wsum = (wsum > 1e-6f) ? 1.0f / wsum : 0.0f;
        for (int i = 0; i < gcount; i++) {
          const int o = 3 * (gbase + i);
          const float3 raw_g = make_float3(tau_g[o + 0], tau_g[o + 1], tau_g[o + 2]) *
                               tau_to_radiance;
          const float3 shown_g = (wsum > 1e-6f) ?
                                     raw_g + (acc_g[i] * inv_wsum - raw_g) * lambda :
                                     raw_g;
          out[o + 0] = shown_g.x * radiance_to_target;
          out[o + 1] = shown_g.y * radiance_to_target;
          out[o + 2] = shown_g.z * radiance_to_target;
        }
      }
    }
  }
  else if (split_groups) {
    /* Mature: nothing to filter, each group is simply its own share. */
    film_write_photon_caustics_groups(
        kg, buffer, make_float3(tau_to_target, tau_to_target, tau_to_target));
  }

  /* Artistic multiplier last, so it scales the filtered result exactly like
   * the unfiltered one (both branches feed `shown`). */
  const float3 target = shown * (samples * kernel_data.integrator.photon_intensity);
  const float3 last_written = make_float3(state_pass[1], state_pass[2], state_pass[3]);

  ccl_global float *combined = buffer + kernel_data.film.pass_combined;
  combined[0] += target.x - last_written.x;
  combined[1] += target.y - last_written.y;
  combined[2] += target.z - last_written.z;

  film_write_photon_caustics_pass(kg, buffer, target, last_written);

  if (kernel_data.film.pass_adaptive_aux_buffer != PASS_UNUSED && mature) {
    ccl_global float *aux = buffer + kernel_data.film.pass_adaptive_aux_buffer;
    if (transition) {
      aux[0] += target.x;
      aux[1] += target.y;
      aux[2] += target.z;
    }
    else {
      aux[0] += target.x - last_written.x;
      aux[1] += target.y - last_written.y;
      aux[2] += target.z - last_written.z;
    }
  }

  state_pass[1] = target.x;
  state_pass[2] = target.y;
  state_pass[3] = target.z;
}

CCL_NAMESPACE_END
/* === CyclesPlus: Personal Photon Passes End === */
