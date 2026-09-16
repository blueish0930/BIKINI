/* SPDX-FileCopyrightText: 2026 CyclesPlus
 * SPDX-FileCopyrightText: 2026 BIKINI
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* === CyclesPlus: Personal Photon Trace Types Begin === */
/* Photon caustics: PODs shared between the host batch scheduler
 * (integrator/photon_map.cpp) and the device photon tracing kernel
 * (kernel/integrator/photon_trace.h). Host-safe: no kernel includes. */

#pragma once

#include "util/math.h"
#include "util/types.h"

CCL_NAMESPACE_BEGIN

enum PhotonTraceMaterialKind {
  PHOTON_MAT_RECEIVER = 0,
  PHOTON_MAT_GLASS = 1,
  PHOTON_MAT_METAL = 2,
  PHOTON_MAT_SKIP = 3,
  PHOTON_MAT_VOLUME = 4,
};

/* Per-shader photon material, extracted host-side from the shader graphs
 * (indexed by the intersection's shader index). 16-byte aligned PODs so the
 * layout matches between host and device. */
struct PhotonTraceMaterial {
  int kind;
  float ior;
  float rough;
  float transmission; /* receivers: thin-wall pass-through probability */

  /* Inverse Abbe number used by the official Principled dispersion model.
   * Zero keeps the legacy non-dispersive photon path. */
  float dispersion_inv_abbe;

  float coat;
  float coat_rough;
  float coat_ior;
  /* != 0: the classifier could not capture this material exactly - the
   * photon walk runs the REAL surface shader at its hits (accurate mode). */
  float accurate;

  /* xyz: per-event tint. w: alpha-transparent pass probability (photons
   * pass straight through untouched, not a caustic event). */
  float4 color;
  /* xyz: Beer-Lambert absorption coefficient of the interior medium
   * (Volume Absorption node: density * (1 - color)); zero = clear. The
   * walk attenuates power over the distance travelled INSIDE the medium
   * (volume-tinted pool water, colored solid glass).
   * w: pass_mode - 1 = photon-owned (receiver pass-through arms deposits),
   * 0 = demoted (path tracing owns the caustics, no spec++), 2 = demoted
   * glass (Fresnel-share pass-through, no spec++). */
  float4 volume_sigma;
  /* xyz: scattering coefficient of the interior medium; w: HG anisotropy. */
  float4 volume_scatter;
};

/* Emission source. Photon indices are partitioned into per-light ranges on
 * the host (budget from the pilot measurement); each light normalizes its
 * flux by its own range size, exactly like the host tracer. */
struct PhotonTraceLight {
  int type; /* 0 sun, 1 point, 2 spot, 3 area */
  int shape;
  float p0; /* sun: half angle | point/spot: radius */
  float spot_cos;

  int index_start; /* [start, end) photon indices of this light */
  int index_end;
  int n_total; /* flux normalization for this light's range */
  /* Object index of the emitter, for Blender's light linking (OBJECT_NONE
   * when unknown). The deposit is tested against the receiving object with
   * exactly the rule path tracing uses at its shading points. */
  int emitter_object;

  float4 color;
  float4 pos;
  float4 axis;  /* xyz = emission direction, w = sx (area) */
  /* x = sy (area)
   * y = light group index, or -1. Every deposit carries the group of the
   *     light that emitted it, so the gather can split the caustic the way
   *     Cycles splits Combined into Combined_<group> passes. */
  float4 extra;
  float tan_half_spread; /* area-light emission cone; zero means parallel */
  float normalize_spread; /* Cycles area-light spread normalization */
  int normalize; /* match Cycles light power normalization */
  float4 initial_volume_sigma; /* xyz: medium at emission; w: bit-packed object index */
  float4 initial_volume_scatter; /* xyz: scattering medium at emission */
};

/* Caster target (bounding sphere), cumulative weight for picking. */
struct PhotonTraceTarget {
  float4 c_r; /* xyz = center, w = radius */
  float wcum; /* cumulative pick weight in (0, 1] */
  /* Sun/world photons launch this far in front of the target so exterior
   * occluders block them exactly like they block the path tracer's rays.
   * Host-computed from the scene bounding sphere (|Tc - scene center| +
   * scene radius + margin); 0 means "unknown", the walks fall back to the
   * old ~50 m heuristic. */
  float start_dist;
  float pad1, pad2;
};

/* Density of an emitted ray under one target's disc (sun/world) or cone proposal.
 * Shared by the host and device tracers for overlapping-target MIS. */
ccl_device_inline float photon_target_pdf(const float3 center,
                                         const float radius,
                                         const float3 origin,
                                         const float3 direction,
                                         const bool directional)
{
  const float3 delta = center - origin;
  if (directional) {
    const float3 radial = delta - direction * dot(delta, direction);
    return dot(radial, radial) <= radius * radius ?
               1.0f / (M_PI_F * radius * radius) :
               0.0f;
  }
  const float distance = len(delta);
  if (distance < 1e-6f) {
    return 0.0f;
  }
  const float sin_max = fminf(1.0f, radius / fmaxf(distance, radius));
  const float cos_max = sqrtf(fmaxf(0.0f, 1.0f - sin_max * sin_max));
  if (cos_max >= 1.0f || dot(delta / distance, direction) < cos_max) {
    return 0.0f;
  }
  return 1.0f / (2.0f * M_PI_F * (1.0f - cos_max));
}

/* Batch seed, shared so host and kernel derive identical RNG streams. */
ccl_device_inline uint64_t photon_trace_batch_seed(const uint64_t k)
{
  return 0x9E3779B97F4A7C15ULL + k * 0x2545F4914F6CDD1DULL;
}

CCL_NAMESPACE_END
/* === CyclesPlus: Personal Photon Trace Types End === */
