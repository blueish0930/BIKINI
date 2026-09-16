/* SPDX-FileCopyrightText: 2026 CyclesPlus
 * SPDX-FileCopyrightText: 2026 BIKINI
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* === CyclesPlus: Personal Photon Trace Begin === */
/* Photon caustics: device-portable photon tracing (one thread = one photon).
 *
 * Replaces the host-side tracer: photons walk the REAL scene BVH via
 * scene_intersect (no separate photon BVH, instancing and motion handled by
 * the render kernels' own machinery), classify surfaces through a small
 * per-shader material table extracted on the host, and write deposits into a
 * device buffer through an atomic cursor. The same code runs on CPU and GPU
 * devices.
 *
 * The walk itself matches the previous host tracer exactly: specular
 * reflect/refract with Fresnel, GGX via VNDF for rough reflection, clearcoat
 * layer, thin-wall translucent pass-through, per-event transmission tint and
 * a power cutoff RELATIVE to the photon's initial power. */

#pragma once

#include "kernel/globals.h"

#include "kernel/bvh/bvh.h"
#include "kernel/geom/object.h"
#include "kernel/geom/shader_data.h"
#include "kernel/geom/triangle.h"
#include "kernel/integrator/photon_grid.h"
#include "kernel/integrator/photon_trace_types.h"
#include "kernel/integrator/state.h"
#include "kernel/integrator/surface_shader.h"
#include "kernel/util/colorspace.h"
/* kernel_ies_interp: photons are aimed host-side, so an IES lamp's
 * distribution has to be applied here rather than by the shader. */
#include "kernel/util/ies.h"
#include "kernel/util/differential.h"

#include "util/atomic.h"
#include "util/hash.h"

CCL_NAMESPACE_BEGIN

/* Deposit output: pos.w packs the surface normal (octahedral), flux.w carries
 * the emitting light's group index (-1 = none) so the gather can split the
 * caustic per light group. `counter` (single uint) is bumped atomically; slots
 * beyond `capacity` are counted but not written, so the host can detect
 * overflow and retry with a bigger buffer.
 *
 * NOTE for anyone bisecting with CYCLESPLUS_PHOTON_GPU_DEBUG: debug mode 5
 * reuses flux.w for its bounce index, so with light groups enabled that mode
 * files deposits under nonsense groups. It already deposits at every hit and
 * produces a deliberately wrong image, so this changes nothing about its
 * purpose - but do not read per-group passes while it is on. */

/* ------------------------------------------------------------------ rng */

/* PCG32, seeded per photon: deterministic across devices and thread counts. */
struct PhotonTraceRNG {
  uint64_t state, inc;
};

ccl_device_inline uint photon_rng_next(ccl_private PhotonTraceRNG *rng)
{
  const uint64_t old = rng->state;
  rng->state = old * 6364136223846793005ULL + rng->inc;
  const uint32_t xs = (uint32_t)(((old >> 18u) ^ old) >> 27u);
  const uint32_t rot = (uint32_t)(old >> 59u);
  return (xs >> rot) | (xs << ((-(int)rot) & 31));
}

ccl_device_inline void photon_rng_seed(ccl_private PhotonTraceRNG *rng,
                                       const uint64_t seed,
                                       const uint64_t stream)
{
  rng->state = 0;
  rng->inc = (stream << 1u) | 1u;
  photon_rng_next(rng);
  rng->state += seed;
  photon_rng_next(rng);
}

ccl_device_inline float photon_rng_uniform(ccl_private PhotonTraceRNG *rng)
{
  return (photon_rng_next(rng) >> 8) * (1.0f / 16777216.0f);
}

ccl_device_inline float photon_rng_range(ccl_private PhotonTraceRNG *rng,
                                         const float a,
                                         const float b)
{
  return a + (b - a) * photon_rng_uniform(rng);
}

ccl_device_inline float3 photon_rng_unit(ccl_private PhotonTraceRNG *rng)
{
  for (;;) {
    const float3 v = make_float3(photon_rng_range(rng, -1.0f, 1.0f),
                                 photon_rng_range(rng, -1.0f, 1.0f),
                                 photon_rng_range(rng, -1.0f, 1.0f));
    const float l2 = dot(v, v);
    if (l2 > 1e-8f && l2 <= 1.0f) {
      return v * (1.0f / sqrtf(l2));
    }
  }
}

/* ------------------------------------------------------------- sampling */

ccl_device_inline float3 photon_reflect_dir(const float3 d, const float3 n)
{
  return d - n * (2.0f * dot(d, n));
}

/* GGX half-vector via the visible normal distribution (Heitz 2018), same
 * math as the host tracer. Roughness semantics like Cycles: alpha=rough^2. */
ccl_device float3 photon_ggx_scatter_reflect(ccl_private PhotonTraceRNG *rng,
                                             const float3 d,
                                             const float3 ns,
                                             const float rough)
{
  const float alpha = fmaxf(rough * rough, 1e-4f);

  const float3 up = fabsf(ns.z) < 0.99f ? make_float3(0.0f, 0.0f, 1.0f) :
                                          make_float3(1.0f, 0.0f, 0.0f);
  const float3 t1 = normalize(cross(up, ns));
  const float3 t2 = cross(ns, t1);
  const float3 wi = -d;
  const float3 v_local = make_float3(dot(wi, t1), dot(wi, t2), dot(wi, ns));

  for (int attempt = 0; attempt < 4; attempt++) {
    const float3 vh = normalize(make_float3(alpha * v_local.x, alpha * v_local.y, v_local.z));
    const float lensq = vh.x * vh.x + vh.y * vh.y;
    const float3 T1 = lensq > 1e-9f ? make_float3(-vh.y, vh.x, 0.0f) * (1.0f / sqrtf(lensq)) :
                                      make_float3(1.0f, 0.0f, 0.0f);
    const float3 T2 = cross(vh, T1);
    const float u1 = photon_rng_uniform(rng);
    const float u2 = photon_rng_uniform(rng);
    const float r = sqrtf(u1);
    const float phi = 2.0f * M_PI_F * u2;
    const float p1 = r * cosf(phi);
    float p2 = r * sinf(phi);
    const float s = 0.5f * (1.0f + vh.z);
    p2 = (1.0f - s) * sqrtf(fmaxf(0.0f, 1.0f - p1 * p1)) + s * p2;
    const float pz = sqrtf(fmaxf(0.0f, 1.0f - p1 * p1 - p2 * p2));
    const float3 nh = T1 * p1 + T2 * p2 + vh * pz;
    const float3 h_local = normalize(
        make_float3(alpha * nh.x, alpha * nh.y, fmaxf(0.0f, nh.z)));
    const float3 h = t1 * h_local.x + t2 * h_local.y + ns * h_local.z;
    const float3 refl = photon_reflect_dir(d, h);
    if (dot(refl, ns) > 0.0f) {
      return refl;
    }
  }
  return photon_reflect_dir(d, ns);
}

ccl_device_inline bool photon_refract_dir(const float3 d,
                                          const float3 n,
                                          const float eta,
                                          ccl_private float3 *out)
{
  const float ci = -dot(d, n);
  const float s2 = eta * eta * (1.0f - ci * ci);
  if (s2 > 1.0f) {
    return false;
  }
  *out = d * eta + n * (eta * ci - sqrtf(1.0f - s2));
  return true;
}

ccl_device_inline float photon_fresnel_schlick(const float ci, const float n1, const float n2)
{
  float r0 = (n1 - n2) / (n1 + n2);
  r0 *= r0;
  const float m = 1.0f - fabsf(ci);
  const float m2 = m * m;
  return r0 + (1.0f - r0) * m2 * m2 * m;
}

/* Same Cauchy parameterization as bsdf_glass_ior(), kept local to the photon
 * walk because this path does not construct ShaderData for classified glass. */
ccl_device_inline float photon_glass_ior(const float ior,
                                         const float inv_abbe,
                                         const float wavelength)
{
  if (inv_abbe == 0.0f) {
    return ior;
  }
  constexpr float lambda_d = 0.5876f;
  constexpr float lambda_C = 0.6563f;
  constexpr float lambda_F = 0.4861f;
  constexpr float fac = 1.0f /
                        (1.0f / (lambda_F * lambda_F) - 1.0f / (lambda_C * lambda_C));
  constexpr float inv_lambda_d_sq = 1.0f / (lambda_d * lambda_d);
  const float B = (ior - 1.0f) * inv_abbe * fac;
  const float A = ior - B * inv_lambda_d_sq;
  return A + B / sqr(wavelength);
}

/* --------------------------------------------------------------- surface */

/* Position, geometric and shading normal, and shader index for a triangle
 * intersection - without a full shader_setup. Handles instancing. */
ccl_device_inline bool photon_surface_from_isect(KernelGlobals kg,
                                                 const ccl_private Ray *ray,
                                                 const ccl_private Intersection *isect,
                                                 ccl_private float3 *r_P,
                                                 ccl_private float3 *r_Ng,
                                                 ccl_private float3 *r_N,
                                                 ccl_private int *r_shader)
{
  if (!(isect->type & PRIMITIVE_TRIANGLE)) {
    return false; /* hair/points do not participate in the photon walk */
  }

  *r_P = ray->P + ray->D * isect->t;

  float3 P_local, Ng;
  int shader;
  triangle_point_normal(kg, isect->object, isect->prim, isect->u, isect->v, &P_local, &Ng, &shader);
  /* tri_shader carries flag bits (SHADER_SMOOTH_NORMAL etc.) in the high
   * bits - mask them off before using the value as a table index. */
  *r_shader = shader & SHADER_MASK;

  const int object_flag = kernel_data_fetch(object_flag, isect->object);
  float3 N = triangle_smooth_normal(kg, Ng, isect->object, object_flag, isect->prim, isect->u, isect->v);

  if (!(object_flag & SD_OBJECT_TRANSFORM_APPLIED)) {
    /* Instanced object: normals were computed in object space. */
    const Transform itfm = object_fetch_transform(kg, isect->object, OBJECT_INVERSE_TRANSFORM);
    Ng = normalize(transform_direction_transposed(&itfm, Ng));
    N = normalize(transform_direction_transposed(&itfm, N));
  }
  else {
    Ng = normalize(Ng);
    N = normalize(N);
  }

  /* Match the host tracer: shading normal never flips against Ng. */
  if (dot(N, Ng) < 0.0f) {
    N = -N;
  }

  *r_Ng = Ng;
  *r_N = N;
  return true;
}

/* --------------------------------------------------------------- deposit */

/* Blender light linking: may this light's photons illuminate this object?
 * Tested at the DEPOSIT, against the receiving surface - path tracing runs
 * the same test at its shading point (light_link_receiver_nee returns
 * sd->object), so a caustic lands exactly where direct light would. Free
 * when the scene uses no light linking, and skipped when the emitter has no
 * object (host-built virtual lights). */
ccl_device_inline bool photon_light_link_match(KernelGlobals kg,
                                               const int receiver,
                                               const int emitter)
{
#ifdef __LIGHT_LINKING__
  if (!(kernel_data.kernel_features & KERNEL_FEATURE_LIGHT_LINKING) ||
      emitter == OBJECT_NONE || receiver == OBJECT_NONE)
  {
    return true;
  }
  const uint64_t set_membership = kernel_data_fetch(objects, emitter).light_set_membership;
  const uint receiver_set = kernel_data_fetch(objects, receiver).receiver_light_set;
  return ((uint64_t(1) << uint64_t(receiver_set)) & set_membership) != 0;
#else
  (void)kg;
  (void)receiver;
  (void)emitter;
  return true;
#endif
}

/* Write one deposit and report the guiding yields. Returns the deposited
 * luminance (feeds the pilot's light-share measurement). */
ccl_device_inline float photon_deposit_write(const float3 P,
                                              const float3 Ng,
                                              const float3 d,
                                              const float3 power,
                                              const float3 beam_start,
                                              const float3 beam_sigma,
                                              ccl_global float4 *out_pos,
                                              ccl_global float4 *out_beam_start,
                                              ccl_global float4 *out_flux,
                                              ccl_global float4 *out_beam_sigma,
                                             ccl_global uint *out_counter,
                                             const int out_capacity,
                                             ccl_global float *target_yield,
                                             const int target_index,
                                             const bool volume,
                                             const float lightgroup)
{
  const float3 n_dep = (dot(d, Ng) < 0.0f) ? Ng : -Ng;
  const uint slot = atomic_fetch_and_add_uint32(out_counter, 1);
  if (slot < (uint)out_capacity) {
    out_pos[slot] = make_float4(P.x, P.y, P.z, photon_pack_normal(n_dep));
    out_beam_start[slot] = make_float4(beam_start.x, beam_start.y, beam_start.z, 0.0f);
    out_beam_sigma[slot] = make_float4(beam_sigma.x, beam_sigma.y, beam_sigma.z, 0.0f);
    /* w carries the emitting light's group (-1 = none), so the gather can
     * split the caustic per light group. Free: the slot was unused. */
    out_flux[slot] = make_float4(power.x, power.y, power.z,
                                 lightgroup + (volume ? 0.5f : 0.0f));
  }
  const float lum = 0.2126f * power.x + 0.7152f * power.y + 0.0722f * power.z;
  if (target_yield != nullptr) {
    atomic_add_and_fetch_float(&target_yield[target_index], lum);
  }
  return lum;
}

/* ------------------------------------------------------------------ walk */

/* Trace one photon and (maybe) write one deposit. Returns luminance of the
 * deposited flux (for the pilot's light-share measurement).
 *
 * debug_mode (env CYCLESPLUS_PHOTON_GPU_DEBUG, runtime-switchable so the
 * fault site can be bisected without kernel rebuilds):
 *   0 = full walk
 *   1 = emission only, deposit at the emission origin (buffers/RNG/lights)
 *   2 = + one scene_intersect, deposit at the hit point (BVH traversal)
 *   3 = + surface/material lookup, deposit with material color (tables)
 *   5 = full walk, deposit at every hit with the bounce index (histogram)
 *   6 = accurate-mode post-eval probe (see the accurate block below)
 *
 * NOTE: mode 7 (full walk + real shader eval at EVERY hit) priced accurate
 * mode before it was built and has been removed. It declared a full
 * ShaderData - MAX_CLOSURE (64) closures, ~10KB - which sized this kernel's
 * stack frame even though the mode never runs in a shipping render, while
 * the accurate walk below deliberately uses the 4-closure
 * ShaderDataCausticsStorage for exactly that reason. Its measurement
 * (1.36-1.44x slower photons on shader-evaluating materials) is recorded;
 * re-deriving it would only need this branch back temporarily. */
ccl_device float photon_trace_single(KernelGlobals kg,
                                     const int photon_index,
                                     const uint64_t batch_seed,
                                     const int max_bounces,
                                     const int debug_mode,
                                     const ccl_global PhotonTraceLight *lights,
                                     const int num_lights,
                                     const ccl_global PhotonTraceTarget *targets,
                                      const int num_targets,
                                      const ccl_global PhotonTraceMaterial *materials,
                                      ccl_global float4 *out_beam_start,
                                      ccl_global float4 *out_pos,
                                     ccl_global float4 *out_flux,
                                      ccl_global float4 *out_beam_sigma,
                                     ccl_global uint *out_counter,
                                     const int out_capacity,
                                     ccl_global float *target_yield)
{
  /* Empty tables must never dereference: num_targets == 0 would read
   * targets[0] of a zero-size (null) buffer below - the light floor of 256
   * photons per light guarantees launches even with zero caster targets. */
  if (num_lights <= 0 || num_targets <= 0) {
    return 0.0f;
  }

  /* Find this photon's light (ranges are few and sorted: linear scan). */
  int li = 0;
  while (li + 1 < num_lights && photon_index >= lights[li].index_end) {
    li++;
  }
  const PhotonTraceLight L = lights[li];
  if (photon_index < L.index_start || photon_index >= L.index_end) {
    return 0.0f;
  }
  const int local_index = photon_index - L.index_start;

  PhotonTraceRNG rng;
  photon_rng_seed(&rng, batch_seed + 7919ULL * (uint64_t)li, (uint64_t)local_index);

  /* One wavelength is shared by every interaction of this photon. This is
   * the same importance-sampled wavelength distribution used by Cycles' own
   * dispersion path; non-dispersive materials pay only the RNG cost. */
  float wavelength = 0.5876f;
  float wavelength_probability = 1.0f;
#ifdef __SPECTRAL__
  PhotonTraceRNG wavelength_rng;
  photon_rng_seed(&wavelength_rng, batch_seed ^ 0xD15C3A5EULL, (uint64_t)photon_index);
  const float rand_wavelength = photon_rng_uniform(&wavelength_rng);
  wavelength = sample_wavelength(rand_wavelength, &wavelength_probability);
#endif
  bool wavelength_weighted = false;

  /* Roulette only the beam records, never the surface walk. Inverse inclusion
   * probability preserves expected flux while bounding the volume workload. */
  const float beam_probability = fminf(1.0f, 65536.0f / lights[num_lights - 1].index_end);
  PhotonTraceRNG beam_rng;
  photon_rng_seed(&beam_rng, batch_seed ^ 0xBEA641ULL, (uint64_t)photon_index);
  const bool store_beams = photon_rng_uniform(&beam_rng) < beam_probability;

  /* Pick a caster target (binary search over cumulative weights). */
  const float r_pick = photon_rng_uniform(&rng);
  int lo = 0, hi = num_targets - 1;
  while (lo < hi) {
    const int mid = (lo + hi) / 2;
    if (r_pick <= targets[mid].wcum) {
      hi = mid;
    }
    else {
      lo = mid + 1;
    }
  }
  const PhotonTraceTarget T = targets[lo];
  const float prev_wcum = (lo > 0) ? targets[lo - 1].wcum : 0.0f;
  const float p_pick = fmaxf(T.wcum - prev_wcum, 1e-9f);
  const float3 Tc = make_float3(T.c_r.x, T.c_r.y, T.c_r.z);
  const float Tr = T.c_r.w;

  /* Emit. */
  float3 o, d;
  float flux;
  float selected_pdf;
  const float3 L_axis = make_float3(L.axis.x, L.axis.y, L.axis.z);
  if (L.type == 0) { /* sun */
    if (L.shape == 1) {
      /* World median-cut cell: uniform direction over the cell's angular
       * rect (pos=(phi0, dphi, cos0), axis.w=dcos - see eval_world_lights).
       * The old centroid-cone concentrated studio-HDRI softbox panels into
       * discrete blobs (square ghost patches on glossy surfaces). */
      const float phi = L.pos.x + photon_rng_uniform(&rng) * L.pos.y;
      const float ct = L.pos.z + photon_rng_uniform(&rng) * L.axis.w;
      const float st = sqrtf(fmaxf(0.0f, 1.0f - ct * ct));
      d = -make_float3(st * cosf(phi), st * sinf(phi), ct);
    }
    else {
      d = L_axis;
      if (L.p0 > 1e-5f) {
        d = normalize(d + photon_rng_unit(&rng) * tanf(L.p0));
      }
    }
    const float3 u = photon_rng_unit(&rng);
    const float3 t1 = normalize(
        cross(d, fabsf(dot(d, u)) < 0.99f ? u : make_float3(1.0f, 0.0f, 0.0f)));
    const float3 t2 = cross(d, t1);
    /* Uniform aim-disc sample. A directional 8x8 histogram over this disc
     * (guiding stage 1b) was tried and REMOVED: measured zero-to-negative
     * benefit on flamingo, a car scene and a synthetic concentration scene -
     * the per-photon random frame makes the angular dimension pure noise. */
    const float u1 = photon_rng_uniform(&rng);
    const float u2 = photon_rng_uniform(&rng);
    const float rr = Tr * sqrtf(u1);
    const float ang = u2 * 2.0f * M_PI_F;
    /* Launch from outside the whole scene (host-computed per-target
     * distance from the scene bounding sphere): exterior occluders block
     * the photon exactly like they block path-traced rays. 0 = old-bundle
     * fallback to the ~50 m heuristic. */
    const float sd = (T.start_dist > 0.0f) ? T.start_dist : (Tr * 20.0f + 10.0f);
    o = Tc - d * sd + t1 * (rr * cosf(ang)) + t2 * (rr * sinf(ang));
    flux = M_PI_F * Tr * Tr / ((float)L.n_total * p_pick);
    selected_pdf = p_pick / (M_PI_F * Tr * Tr);
  }
  else if (L.type == 1 || L.type == 2) { /* point / spot */
    o = make_float3(L.pos.x, L.pos.y, L.pos.z);
    if (L.p0 > 1e-6f) {
      o = o + photon_rng_unit(&rng) * (L.p0 * photon_rng_uniform(&rng));
    }
    /* UNIFORM-CONE direction sampling toward the target cap. The previous
     * point-in-sphere trick oversampled the sphere center (longer chords)
     * while every photon carried constant flux - hits were systematically
     * overestimated (~1.7x measured against converged path tracing). The
     * sun is immune (parallel rays, uniform area sampling), which is why
     * the validated sun/HDRI scenes never showed it. */
    float3 ca = Tc - o;
    const float dist = len(ca);
    if (dist < 1e-6f) {
      return 0.0f;
    }
    ca = ca * (1.0f / dist);
    const float st = fminf(1.0f, Tr / fmaxf(dist, Tr));
    const float cos_max = sqrtf(fmaxf(0.0f, 1.0f - st * st));
    const float ct = 1.0f - photon_rng_uniform(&rng) * (1.0f - cos_max);
    const float sn = sqrtf(fmaxf(0.0f, 1.0f - ct * ct));
    const float ph = 2.0f * M_PI_F * photon_rng_uniform(&rng);
    const float3 up = fabsf(ca.z) < 0.99f ? make_float3(0.0f, 0.0f, 1.0f) :
                                            make_float3(1.0f, 0.0f, 0.0f);
    const float3 t1 = normalize(cross(up, ca));
    const float3 t2 = cross(ca, t1);
    d = t1 * (sn * cosf(ph)) + t2 * (sn * sinf(ph)) + ca * ct;
    if (L.type == 2 && dot(d, L_axis) < L.spot_cos) {
      return 0.0f; /* outside the spot cone: zero radiance there */
    }
    const float omega = 2.0f * M_PI_F * (1.0f - cos_max);
    flux = (1.0f / (4.0f * M_PI_F)) * omega / ((float)L.n_total * p_pick);
    selected_pdf = p_pick / omega;
  }
  else { /* area */
    const float sx = L.axis.w, sy = L.extra.x;
    const float area = sx * sy * (L.shape ? M_PI_F / 4.0f : 1.0f);
    float lx, ly;
    if (L.shape) {
      const float rr = 0.5f * sqrtf(photon_rng_uniform(&rng));
      const float ang = photon_rng_uniform(&rng) * 2.0f * M_PI_F;
      lx = rr * cosf(ang) * sx;
      ly = rr * sinf(ang) * sy;
    }
    else {
      lx = (photon_rng_uniform(&rng) - 0.5f) * sx;
      ly = (photon_rng_uniform(&rng) - 0.5f) * sy;
    }
    const float3 zax = L_axis;
    const float3 up = fabsf(zax.z) < 0.99f ? make_float3(0.0f, 0.0f, 1.0f) :
                                             make_float3(1.0f, 0.0f, 0.0f);
    const float3 xax = normalize(cross(up, zax));
    const float3 yax = cross(zax, xax);
    o = make_float3(L.pos.x, L.pos.y, L.pos.z) + xax * lx + yax * ly;
    /* A zero-spread area light is a collimated emitter. Do not aim its
     * photons at every caster target: that creates caustics outside the
     * actual rectangular beam. */
    if (L.tan_half_spread == 0.0f) {
      d = zax;
      flux = (L.normalize ? 1.0f : area) /
             ((float)L.n_total * p_pick);
      selected_pdf = p_pick;
    }
    else {
    /* Sample the actual emission cone around the light normal. */
    const float3 ca = zax;
    const float cos_max = 1.0f / sqrtf(1.0f + sqr(L.tan_half_spread));
    const float ct = 1.0f - photon_rng_uniform(&rng) * (1.0f - cos_max);
    const float sn = sqrtf(fmaxf(0.0f, 1.0f - ct * ct));
    const float ph = 2.0f * M_PI_F * photon_rng_uniform(&rng);
    const float3 cup = fabsf(ca.z) < 0.99f ? make_float3(0.0f, 0.0f, 1.0f) :
                                             make_float3(1.0f, 0.0f, 0.0f);
    const float3 ct1 = normalize(cross(cup, ca));
    const float3 ct2 = cross(ca, ct1);
    d = ct1 * (sn * cosf(ph)) + ct2 * (sn * sinf(ph)) + ca * ct;
    const float cl = dot(d, zax);
    if (cl <= 0.0f) {
      return 0.0f; /* behind the emitting side: zero radiance there */
    }
    const float omega = 2.0f * M_PI_F * (1.0f - cos_max);
    const float tan_a = sqrtf(fmaxf(0.0f, 1.0f - cl * cl)) / fmaxf(cl, 1e-8f);
    const float spread_attenuation = (L.tan_half_spread == FLT_MAX) ? 1.0f :
        fmaxf((L.tan_half_spread - tan_a) * L.normalize_spread, 0.0f);
    flux = (1.0f / (M_PI_F * fmaxf(area, 1e-8f))) * area * cl * omega *
           spread_attenuation /
           ((float)L.n_total * p_pick);
    selected_pdf = p_pick / omega;
    }
  }

  /* Balance the selected proposal against every overlapping target. Without
   * this, a ray covered by two target domains contributes its energy twice. */
  if (num_targets > 1) {
    float mixture_pdf = selected_pdf;
    float previous = 0.0f;
    for (int i = 0; i < num_targets; i++) {
      const PhotonTraceTarget other = targets[i];
      const float probability = other.wcum - previous;
      previous = other.wcum;
      if (i != lo) {
        mixture_pdf += probability *
                       photon_target_pdf(make_float3(other.c_r.x, other.c_r.y, other.c_r.z),
                                         other.c_r.w, o, d, L.type == 0);
      }
    }
    flux *= selected_pdf / mixture_pdf;
  }

  float3 power = make_float3(L.color.x, L.color.y, L.color.z) * flux;

  /* IES profile: a measured lamp does not emit the same in every direction,
   * and since photons are aimed host-side the shader never gets to shape
   * them - an IES lamp lit the scene as if the file were not loaded (tester
   * report 2026-08-25). Weight each photon by the profile in ITS direction,
   * the same lookup and the same angle convention svm_node_ies uses, so the
   * caustic is shaped by the same data the render is.
   *
   * The direction is the photon's own, expressed in the lamp's frame. */
  const int ies_slot = (int)L.extra.z;
  if (ies_slot >= 0) {
    const float3 zax = L_axis;
    const float3 up = fabsf(zax.z) < 0.99f ? make_float3(0.0f, 0.0f, 1.0f) :
                                             make_float3(1.0f, 0.0f, 0.0f);
    const float3 xax = normalize(cross(up, zax));
    const float3 yax = cross(zax, xax);
    const float3 dl = make_float3(dot(d, xax), dot(d, yax), dot(d, zax));
    const float v_angle = safe_acosf(dl.z);
    const float h_angle = atan2f(dl.x, dl.y) + M_PI_F;
    power = power * fmaxf(kernel_ies_interp(kg, ies_slot, h_angle, v_angle), 0.0f);
  }
  const float power_cutoff = 1e-5f * fmaxf(power.x, fmaxf(power.y, power.z));
  int spec = 0;
  /* Accurate mode can deposit on several mixed surfaces along one walk; the
   * pilot wants the total deposited luminance. */
  float lum_total = 0.0f;

  if (debug_mode == 1) {
    /* Emission only: deposit at the origin, no BVH. */
    const uint slot = atomic_fetch_and_add_uint32(out_counter, 1);
    if (slot < (uint)out_capacity) {
      out_pos[slot] = make_float4(o.x, o.y, o.z, photon_pack_normal(-d));
      out_beam_sigma[slot] = make_float4(0.0f);
      out_flux[slot] = make_float4(power.x, power.y, power.z, 0.0f);
    }
    return 0.0f;
  }

  /* Beer-Lambert state of the medium the photon travels in (set on
   * refracting INTO a glass with volume absorption, cleared on exit).
   *
   * vol_object records WHICH object put us in that medium, because drinks
   * are modelled with the liquid poked into the glass wall so no air gap
   * shows. A photon then crosses the glass's inner wall while still inside
   * the liquid, and clearing the medium on any exit threw the liquid's
   * colour away for the rest of the path - the tester's pink drink cast a
   * colourless caustic until he scaled the liquid down out of the wall
   * (2026-08-25). Only the object that set the medium may clear it.
   *
   * One slot, not a stack: registers in this raygen are precious (an
   * oversized frame is what crashed OptiX in the v2.1 era). That covers
   * clear glass around a tinted liquid, which is the case artists build.
   * A tinted glass AND a tinted liquid would lose the glass's tint after
   * leaving the liquid - a real stack is the fix if that ever shows up. */
  float3 vol_sigma = make_float3(L.initial_volume_sigma.x,
                                 L.initial_volume_sigma.y,
                                 L.initial_volume_sigma.z);
  float3 vol_sigma_s = make_float3(L.initial_volume_scatter.x,
                                   L.initial_volume_scatter.y,
                                   L.initial_volume_scatter.z);
  int vol_object = __float_as_int(L.initial_volume_sigma.w);
  for (int bounce = 0; bounce < max_bounces; bounce++) {
    /* Ray guard: RT-core traversal of a non-finite or degenerate ray is
     * undefined behavior - observed as a hard device hang on OptiX after
     * minutes of animation (context dead, cold boot needed), while the
     * software BVH on CUDA shrugs NaN rays off as misses (which is why
     * only OptiX ever died). Several scatter paths can go degenerate in
     * rare RNG corners (normalize(d + j) with |j| ~ |d|, normalize(wo)
     * after bsdf_sample, jittered sun frames), and at 15M photons per
     * generation a one-in-a-billion corner fires every few minutes. Kill
     * the photon instead of tracing it. */
    const float d_len2 = dot(d, d);
    if (!isfinite_safe(d_len2) || d_len2 < 0.25f || d_len2 > 4.0f || !isfinite_safe(o)) {
      return lum_total;
    }
    Ray ray;
    ray.P = o;
    ray.D = d;
    ray.tmin = 1e-4f;
    ray.tmax = FLT_MAX;
    ray.time = 0.5f;
    ray.self.prim = PRIM_NONE;
    ray.self.object = OBJECT_NONE;
    ray.self.light_prim = PRIM_NONE;
    ray.self.light_object = OBJECT_NONE;
    /* A small angular differential (~1 mrad, camera-pixel scale) so the
     * accurate block's shader eval has a neighborhood: the SVM Bump node
     * derives its height gradient from sd->dP, which shader_setup_from_ray
     * transfers from these ray differentials. With zero differentials a
     * bump/normal-mapped caster casts the caustic of a SMOOTH surface
     * (measured 12.08.: force-accurate bump A/B bit-identical). The fast
     * path never reads them, and shaders without bump sections are
     * unaffected (gradient of nothing stays nothing). */
    ray.dP = differential_zero_compact();
    ray.dD = 1e-3f;

    Intersection isect;
    /* Shadow visibility: objects that cast no shadows must not block or
     * receive photons (helper planes etc.), same rule as path tracing.
     * Deliberately ONLY the transparent-shadow bit: shadow-casting objects
     * carry both shadow bits, so the filtering is identical - but on OptiX
     * a ray visibility containing SHADOW_OPAQUE sets
     * OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT, which returns the FIRST-found
     * hit instead of the closest (fine for occlusion queries, fatally wrong
     * for a photon walk - deposits landed on the wrong surfaces). */
    if (!scene_intersect(kg, &ray, PATH_RAY_VISIBILITY_SHADOW_TRANSPARENT, &isect)) {
      return lum_total;
    }

    if (kernel_data.integrator.use_photon_volume_caustics && spec > 0 &&
        vol_sigma_s.x <= 0.0f && vol_sigma_s.y <= 0.0f && vol_sigma_s.z <= 0.0f)
    {
      float3 boundary_P, boundary_Ng, boundary_N;
      int boundary_shader;
      if (photon_surface_from_isect(
              kg, &ray, &isect, &boundary_P, &boundary_Ng, &boundary_N, &boundary_shader))
      {
        const PhotonTraceMaterial boundary = materials[boundary_shader];
        if (boundary.kind == PHOTON_MAT_VOLUME && dot(d, boundary_N) >= 0.0f) {
          vol_sigma = make_float3(boundary.volume_sigma.x,
                                  boundary.volume_sigma.y,
                                  boundary.volume_sigma.z);
          vol_sigma_s = make_float3(boundary.volume_scatter.x,
                                    boundary.volume_scatter.y,
                                    boundary.volume_scatter.z);
          vol_object = isect.object;
        }
      }
    }

    if (kernel_data.integrator.use_photon_volume_caustics &&
        (vol_sigma_s.x > 0.0f || vol_sigma_s.y > 0.0f || vol_sigma_s.z > 0.0f)) {
      /* Store the complete in-medium segment. The old stochastic point
       * deposit carried a scattering-event weight and could not represent a
       * continuous beam. The volume gather applies sigma_s and phase at the
       * query point, so the stored flux is the incident beam power. */
      const float3 beam_end = ray.P + ray.D * isect.t;
      if (store_beams && spec > 0 && photon_light_link_match(kg, vol_object, L.emitter_object)) {
        lum_total += photon_deposit_write(beam_end,
                                          -d,
                                          d,
                                          power / beam_probability,
                                          ray.P,
                                          vol_sigma,
                                          out_pos,
                                          out_beam_start,
                                          out_flux,
                                          out_beam_sigma,
                                          out_counter,
                                          out_capacity,
                                          target_yield,
                                          lo,
                                          true,
                                          L.extra.y);
      }
    }

    if (vol_sigma.x > 0.0f || vol_sigma.y > 0.0f || vol_sigma.z > 0.0f) {
      power = power * make_float3(expf(-vol_sigma.x * isect.t),
                                  expf(-vol_sigma.y * isect.t),
                                  expf(-vol_sigma.z * isect.t));
    }

    if (debug_mode == 2) {
      /* BVH only: deposit at the hit point. */
      const float3 hitP = ray.P + ray.D * isect.t;
      const uint slot = atomic_fetch_and_add_uint32(out_counter, 1);
      if (slot < (uint)out_capacity) {
        out_pos[slot] = make_float4(hitP.x, hitP.y, hitP.z, photon_pack_normal(-d));
        out_beam_start[slot] = make_float4(ray.P.x, ray.P.y, ray.P.z, 0.0f);
        /* -1, not 0: an ungrouped deposit, otherwise this diagnostic would
         * quietly pile up in whichever light group happens to be first. */
        out_flux[slot] = make_float4(power.x, power.y, power.z, -1.0f);
      }
      return 0.0f;
    }

    if (debug_mode == 5) {
      /* Full walk, but deposit at EVERY hit with the bounce index in flux.w
       * (the host logs a bounce histogram) - shows at which segment paths
       * die when backends disagree. */
      const float3 hitP = ray.P + ray.D * isect.t;
      const uint slot = atomic_fetch_and_add_uint32(out_counter, 1);
      if (slot < (uint)out_capacity) {
        out_pos[slot] = make_float4(hitP.x, hitP.y, hitP.z, photon_pack_normal(-d));
        out_beam_start[slot] = make_float4(ray.P.x, ray.P.y, ray.P.z, 0.0f);
        out_flux[slot] = make_float4(power.x, power.y, power.z, (float)bounce);
      }
    }

    float3 P, Ng, N;
    int shader;
    if (!photon_surface_from_isect(kg, &ray, &isect, &P, &Ng, &N, &shader)) {
      /* Non-triangle geometry: pass straight through. */
      o = ray.P + ray.D * (isect.t + 1e-4f);
      continue;
    }

    const PhotonTraceMaterial m = materials[shader];
    if (debug_mode == 3) {
      /* Surface/material lookup: deposit tinted by the material color. */
      const uint slot = atomic_fetch_and_add_uint32(out_counter, 1);
      if (slot < (uint)out_capacity) {
        out_pos[slot] = make_float4(P.x, P.y, P.z, photon_pack_normal(Ng));
        out_flux[slot] = make_float4(power.x * m.color.x, power.y * m.color.y,
                                     power.z * m.color.z, 0.0f);
      }
      return 0.0f;
    }

    if (m.kind == PHOTON_MAT_SKIP) {
      o = P + d * 1e-4f;
      continue;
    }

    /* Alpha transparency (Principled Alpha < 1, carried in color.w): pass
     * straight through untouched with the transparent share - Cycles mixes a
     * WHITE Transparent BSDF for alpha, and a transparent hop is not a
     * caustic-forming event (no spec++, LABEL_TRANSPARENT semantics).
     * Without this an invisible pane is a photon wall: a caster behind
     * alpha-0 glass lost its caustic (user find 2026-08-01). Accurate
     * materials sample the real transparent closure instead. */
    if (m.accurate == 0.0f && m.color.w > 0.0f && photon_rng_uniform(&rng) < m.color.w) {
      o = P + d * 1e-4f;
      continue;
    }

    if (m.accurate != 0.0f) {
      /* Accurate mode (Weg B): run the REAL surface shader at this hit and
       * sample the actual BSDF instead of the classified profile. Only
       * materials the classifier could not capture exactly pay this price.
       * Caustics-sized closure storage (like MNEE): a full ShaderData is
       * ~10KB of per-thread stack, which blows up the OptiX continuation
       * stack allocation at photon launch sizes (measured: instant OOM +
       * illegal address on the very first launch). 4 closure slots cover
       * the caustic-relevant lobes. */
      ShaderDataCausticsStorage sd_storage;
      ccl_private ShaderData *sd = AS_SHADER_DATA(&sd_storage);
      shader_setup_from_ray(kg, sd, &ray, &isect);
#ifdef __SPECTRAL__
      sd->rand_wavelength = rand_wavelength;
#endif
      ConstIntegratorBakeState state;
      surface_shader_eval<KERNEL_FEATURE_NODE_MASK_SURFACE &
                          ~(KERNEL_FEATURE_NODE_RAYTRACE | KERNEL_FEATURE_NODE_LIGHT_PATH |
                            KERNEL_FEATURE_NODE_AOV)>(
          kg, state, sd, nullptr, PATH_RAY_VISIBILITY_CAMERA, PATH_RAY_FLAG_NONE, true);

      if (debug_mode == 6) {
        /* Post-eval probe: why do photons die in the accurate block?
         * flux = (cache_miss, num_closure, bsdf weight sum, shader index). */
        float w_dbg = 0.0f;
        for (int ci = 0; ci < sd->num_closure; ci++) {
          const ccl_private ShaderClosure *dsc = &sd->closure[ci];
          if (CLOSURE_IS_BSDF_OR_BSSRDF(dsc->type)) {
            w_dbg += dsc->sample_weight;
          }
        }
        const uint slot = atomic_fetch_and_add_uint32(out_counter, 1);
        if (slot < (uint)out_capacity) {
          out_pos[slot] = make_float4(P.x, P.y, P.z, photon_pack_normal(Ng));
          out_flux[slot] = make_float4((sd->runtime_flag & SR_CACHE_MISS) ? 1.0f : 0.0f,
                                       (float)sd->num_closure,
                                       w_dbg,
                                       (float)shader);
        }
        return lum_total;
      }

      if (!(sd->runtime_flag & SR_CACHE_MISS)) {
        float w_all = 0.0f, w_recv = 0.0f;
        for (int ci = 0; ci < sd->num_closure; ci++) {
          const ccl_private ShaderClosure *sc = &sd->closure[ci];
          if (!CLOSURE_IS_BSDF_OR_BSSRDF(sc->type)) {
            continue;
          }
          w_all += sc->sample_weight;
          if (CLOSURE_IS_BSDF_DIFFUSE(sc->type) || CLOSURE_IS_BSSRDF(sc->type)) {
            w_recv += sc->sample_weight;
          }
        }
        if (w_all <= 0.0f) {
          return lum_total; /* emission/holdout only: photon absorbed */
        }
        /* A receiving (diffuse-ish) component collects the incident caustic
         * flux. The gather applies the measurement point's own BSDF, so the
         * deposit carries the FULL incident power, like the fast receiver. */
        if (spec > 0 && w_recv > 1e-4f * w_all &&
            photon_light_link_match(kg, isect.object, L.emitter_object))
        {
          lum_total += photon_deposit_write(P,
                                            Ng,
                                            d,
                                            power,
                                            P,
                                            make_float3(0.0f),
                                            out_pos,
                                            out_beam_start,
                                            out_flux,
                                            out_beam_sigma,
                                            out_counter,
                                            out_capacity,
                                            target_yield,
                                            lo,
                                            false,
                                            L.extra.y);
        }
        /* Continue through ONE closure, picked exactly like Cycles itself
         * (proportional to sample_weight over all BSDFs). A diffuse pick is
         * absorption; dividing the continuation by the pick probability
         * makes this an exact Russian-roulette estimator. */
        float3 rand_bsdf = make_float3(photon_rng_uniform(&rng),
                                       photon_rng_uniform(&rng),
                                       photon_rng_uniform(&rng));
        const ccl_private ShaderClosure *sc = surface_shader_bsdf_bssrdf_pick(sd, &rand_bsdf);
        if (sc == nullptr || !CLOSURE_IS_BSDF(sc->type) || CLOSURE_IS_BSDF_DIFFUSE(sc->type)) {
          return lum_total; /* absorbed (diffuse/BSSRDF): no caustic chain */
        }
        Spectrum bsdf_eval_value = zero_spectrum();
        float3 wo;
        float pdf = 0.0f;
        float2 sampled_roughness;
        float sampled_eta;
        const int label = bsdf_sample(
            kg, sd, sc, rand_bsdf, &bsdf_eval_value, &wo, &pdf, &sampled_roughness, &sampled_eta);
        if (pdf <= 0.0f || (label & LABEL_DIFFUSE)) {
          return lum_total;
        }
        const float p_pick = fmaxf(sc->sample_weight / w_all, 1e-6f);
        float3 thr = spectrum_to_rgb(bsdf_eval_value * sc->weight) / (pdf * p_pick);
        /* Adjoint correction: eval/pdf is built for camera paths; traced
         * from the light it carries an extra eta^2 per refraction (the
         * half-vector Jacobian is not symmetric). Divide it back out so the
         * accurate walk matches path tracing - same convention as the fast
         * walk's (eta_in/eta_out)^2, sampled_eta is 1 for non-refractions. */
        if (sampled_eta != 1.0f) {
          thr = thr / (sampled_eta * sampled_eta);
        }
#ifdef __SPECTRAL__
        /* Keep the incident deposit unchanged; weight the continuing path once. */
        if (!wavelength_weighted && (sd->runtime_flag & SR_BSDF_HAS_DISPERSION)) {
          power *= wavelength_to_rgb_d65(kg, wavelength) /
                   fmaxf(wavelength_probability, 1e-6f);
          wavelength_weighted = true;
        }
#endif
        power = power * make_float3(fmaxf(thr.x, 0.0f), fmaxf(thr.y, 0.0f), fmaxf(thr.z, 0.0f));
        d = normalize(wo);
        if ((label & LABEL_TRANSMIT) && !(label & LABEL_TRANSPARENT)) {
          /* Real refraction: entering from the front face puts the photon
           * inside the medium, leaving through the back face clears it -
           * but only the object that set the medium may clear it, or a
           * liquid poked into a glass wall loses its tint when the photon
           * crosses that wall (see the vol_object note above). */
          if (sd->runtime_flag & SR_BACKFACING) {
            if (isect.object == vol_object) {
              vol_sigma = make_float3(0.0f, 0.0f, 0.0f);
              vol_sigma_s = make_float3(0.0f, 0.0f, 0.0f);
              vol_object = OBJECT_NONE;
            }
          }
          else {
            const float3 s = make_float3(
                m.volume_sigma.x, m.volume_sigma.y, m.volume_sigma.z);
            if (s.x > 0.0f || s.y > 0.0f || s.z > 0.0f) {
              vol_sigma = s;
              vol_sigma_s = make_float3(m.volume_scatter.x,
                                        m.volume_scatter.y,
                                        m.volume_scatter.z);
              vol_object = isect.object;
            }
          }
        }
        if (!(label & LABEL_TRANSPARENT)) {
          spec++; /* transparent pass-through is not a caustic-forming event */
        }
        if (fmaxf(power.x, fmaxf(power.y, power.z)) < power_cutoff) {
          return lum_total;
        }
        o = P;
        continue;
      }
      /* Texture cache miss: fall back to the profile walk for this hit. */
    }

    const bool entering = dot(d, N) < 0.0f;
    const float3 ns = entering ? N : -N;

    /* Clearcoat reflects its Fresnel share before the base interacts. */
    if (m.coat > 0.0f) {
      const float ci_coat = -dot(d, ns);
      const float fr_coat = fminf(m.coat, 1.0f) *
                            photon_fresnel_schlick(ci_coat, 1.0f, m.coat_ior);
      if (photon_rng_uniform(&rng) < fr_coat) {
        d = (m.coat_rough > 0.001f) ? photon_ggx_scatter_reflect(&rng, d, ns, m.coat_rough) :
                                      photon_reflect_dir(d, ns);
        spec++;
        o = P;
        continue;
      }
    }

    if (m.kind == PHOTON_MAT_VOLUME) {
      if (!kernel_data.integrator.use_photon_volume_caustics) {
        o = P + d * 1e-4f;
        continue;
      }
      if (dot(d, N) < 0.0f) {
        vol_sigma = make_float3(m.volume_sigma.x, m.volume_sigma.y, m.volume_sigma.z);
        vol_sigma_s = make_float3(m.volume_scatter.x, m.volume_scatter.y, m.volume_scatter.z);
        vol_object = isect.object;
      }
      else if (isect.object == vol_object) {
        vol_sigma = make_float3(0.0f, 0.0f, 0.0f);
        vol_sigma_s = make_float3(0.0f, 0.0f, 0.0f);
        vol_object = OBJECT_NONE;
      }
      o = P + d * 1e-4f;
      continue;
    }

    if (m.kind == PHOTON_MAT_RECEIVER) {
      /* Thin-wall translucent pass-through (tinted per event).
       * volume_sigma.w = pass_mode (see host PhotonMaterial): 2 = demoted
       * glass, only the Fresnel-transmitted share passes (real panes
       * reflect); != 1 = the hop does not arm a deposit, path tracing owns
       * that material's caustics (per-material gate). */
      const int pass_mode = (int)m.volume_sigma.w;
      float pass_p = m.transmission;
      if (pass_mode == 2) {
        pass_p *= 1.0f - photon_fresnel_schlick(-dot(d, ns), 1.0f, m.ior);
      }
      if (pass_p > 0.0f && photon_rng_uniform(&rng) < pass_p) {
        power = power * make_float3(m.color.x, m.color.y, m.color.z);
        if (m.rough > 0.001f) {
          const float3 j = photon_rng_unit(&rng) * (m.rough * 2.0f);
          d = normalize(d + j);
        }
        if (pass_mode == 1) {
          spec++;
        }
        if (fmaxf(power.x, fmaxf(power.y, power.z)) < power_cutoff) {
          return lum_total;
        }
        o = P;
        continue;
      }
      if (spec > 0 && photon_light_link_match(kg, isect.object, L.emitter_object)) {
        /* Target guiding (CyclesPlus Stufe 1a): the deposit helper reports
         * the yield per aim target, so the host can adapt the target
         * weights (wcum) for the next generation. The pick PDF is the
         * host's own table, so the estimator stays exact for ANY weights
         * (flux already divides by p_pick above). */
        return lum_total + photon_deposit_write(P,
                                                Ng,
                                                 d,
                                                power,
                                                P,
                                                make_float3(0.0f),
                                                out_pos,
                                                out_beam_start,
                                                out_flux,
                                                out_beam_sigma,
                                                out_counter,
                                                out_capacity,
                                                target_yield,
                                                lo,
                                                false,
                                                L.extra.y);
      }
      return lum_total;
    }

    if (m.kind == PHOTON_MAT_METAL) {
      d = (m.rough > 0.001f) ? photon_ggx_scatter_reflect(&rng, d, ns, m.rough) :
                               photon_reflect_dir(d, ns);
      power = power * make_float3(m.color.x, m.color.y, m.color.z);
    }
    else { /* glass */
      const float glass_ior = photon_glass_ior(m.ior, m.dispersion_inv_abbe, wavelength);
      const float eta1 = entering ? 1.0f : glass_ior;
      const float eta2 = entering ? glass_ior : 1.0f;
      if (!wavelength_weighted && m.dispersion_inv_abbe != 0.0f) {
#ifdef __SPECTRAL__
        power = power * (wavelength_to_rgb_d65(kg, wavelength) /
                         fmaxf(wavelength_probability, 1e-6f));
#endif
        wavelength_weighted = true;
      }
      const float ci = -dot(d, ns);
      const float fr = photon_fresnel_schlick(ci, eta1, eta2);
      float3 refr;
      if (!photon_refract_dir(d, ns, eta1 / eta2, &refr) || photon_rng_uniform(&rng) < fr) {
        d = (m.rough > 0.001f) ? photon_ggx_scatter_reflect(&rng, d, ns, m.rough) :
                                 photon_reflect_dir(d, ns);
      }
      else {
        /* Tint per refraction event, matching Cycles' transmission.
         * Cycles' BSDFs omit the eta^2 radiance rescale at refractions, so a
         * power-conserving photon walk renders eta^2 (1.77x for water) hotter
         * than path tracing wherever a photon enters a medium and deposits
         * without leaving it (pool floors!). Follow the PT convention
         * instead: scale power by (eta_in/eta_out)^2 per event. Solid glass
         * (enter+exit) cancels exactly, which is why sphere scenes never
         * showed it. Measured: white slab, underwater camera - PT 0.1779,
         * uncorrected photons 0.3020, physical E*(1-F) 0.3074. */
        d = refr;
        const float eta_scale = (eta1 / eta2) * (eta1 / eta2);
        power = power * (eta_scale * make_float3(fmaxf(m.color.x, 0.0f),
                                                 fmaxf(m.color.y, 0.0f),
                                                 fmaxf(m.color.z, 0.0f)));
        /* See the vol_object note at the top of the walk: a medium is only
         * cleared by the object that set it, so crossing out of the glass
         * wall does not strip the liquid the photon is still inside. */
        if (entering) {
          const float3 s = make_float3(
              m.volume_sigma.x, m.volume_sigma.y, m.volume_sigma.z);
          if (s.x > 0.0f || s.y > 0.0f || s.z > 0.0f) {
            vol_sigma = s;
            vol_sigma_s = make_float3(m.volume_scatter.x,
                                      m.volume_scatter.y,
                                      m.volume_scatter.z);
            vol_object = isect.object;
          }
        }
        else if (isect.object == vol_object) {
          vol_sigma = make_float3(0.0f, 0.0f, 0.0f);
          vol_sigma_s = make_float3(0.0f, 0.0f, 0.0f);
          vol_object = OBJECT_NONE;
        }
        if (m.rough > 0.001f) {
          const float3 j = photon_rng_unit(&rng) * (m.rough * m.rough * 2.0f);
          d = normalize(d + j);
        }
      }
    }
    spec++;
    if (fmaxf(power.x, fmaxf(power.y, power.z)) < power_cutoff) {
      return lum_total;
    }
    o = P;
  }
  return lum_total;
}

CCL_NAMESPACE_END
/* === CyclesPlus: Personal Photon Trace End === */
