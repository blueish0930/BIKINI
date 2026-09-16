/* SPDX-FileCopyrightText: 2026 CyclesPlus
 * SPDX-FileCopyrightText: 2026 BIKINI
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* === CyclesPlus: Personal Photon Grid Begin === */
/* Photon caustics: shared definitions between the host-side builder
 * (integrator/photon_map.cpp) and the kernel lookup
 * (kernel/integrator/photon_lookup.h).
 *
 * Photon records are sorted by spatial-hash cell. `cell_start` holds
 * table_size + 1 prefix offsets; bucket b spans
 * [cell_start[b], cell_start[b + 1]). Hash collisions merge distant cells
 * into one bucket, which only costs lookup time - the per-photon distance
 * test keeps the estimate correct. */

#pragma once

#include "util/math.h"
#include "util/types.h"
#include "kernel/types.h"

CCL_NAMESPACE_BEGIN

/* Host-side view of one finished photon batch. The arrays are uploaded to
 * the render device as the kernel data arrays photon_pos / photon_flux /
 * photon_cell_start (see PathTrace::set_photon_grid). */
struct PhotonGrid {
  const float4 *pos = nullptr;  /* xyz = position, w = packed deposit normal */
  const float4 *beam_start = nullptr; /* xyz = volume beam segment start */
  const float4 *beam_sigma = nullptr;
  const float4 *flux = nullptr; /* xyz = flux in W, w = emitting light's group */
  const int *cell_start = nullptr;
  int num_photons = 0;
  uint table_size = 0; /* power of two */
  float radius = 0.0f; /* gather radius == cell size */
  float inv_cell = 0.0f;
  const float4 *volume_beam_start = nullptr;
  const float4 *volume_beam_end = nullptr;
  const float4 *volume_beam_flux = nullptr;
  const float4 *volume_beam_sigma = nullptr;
  const KernelPhotonBeamNode *volume_beam_nodes = nullptr;
  int num_volume_beams = 0;
  int num_volume_beam_nodes = 0;
  uint64_t generation = 0; /* bumped on every published batch */
  /* Device-resident batch (GPU tracer + GPU binning): `pos`/`flux` are null,
   * the unsorted deposits live in VRAM at the addresses below and are
   * scattered into the render device's photon arrays by the PhotonMap
   * (`owner`) when the grid is bound (PathTrace::set_photon_grid). Only
   * `cell_start` (the host prefix table) crosses PCIe. */
  int device_resident = 0;
  uint64_t dep_pos_device = 0;
  uint64_t dep_beam_start_device = 0;
  uint64_t dep_flux_device = 0;
  uint64_t dep_beam_sigma_device = 0;
  void *owner = nullptr; /* PhotonMap*, host-side only */
  /* Per kernel shader slot: does the photon map cast from this shader?
   * Uploaded as the kernel data array photon_shader_caster. */
  const uint8_t *shader_caster = nullptr;
  int num_shader_caster = 0;
  /* Batch size relative to a full batch, in 1/16th steps (1..16). Weighs
   * this generation in the per-pixel SPPM tau average. */
  int weight16 = 16;
  /* Whether the batch ramp reached its steady-state size: only then may
   * adaptive sampling retire pixels that saw no photons yet (before that,
   * "no photons" may just mean "the ramp is still small"). */
  int steady = 1;
};

/* ccl_device_inline resolves to static inline on the host and to a __device__
 * function in GPU kernels, so this compiles in both contexts. */
ccl_device_inline uint photon_grid_hash(const int x, const int y, const int z, const uint table_size)
{
  const uint h = ((uint)x * 73856093u) ^ ((uint)y * 19349663u) ^ ((uint)z * 83492791u);
  return h & (table_size - 1);
}

ccl_device_inline float photon_safe_radius(const float radius)
{
  return (isfinite_safe(radius) && radius > 1e-10f) ? radius : 1e-10f;
}

ccl_device_inline float photon_safe_radius_squared(const float radius_squared)
{
  return (isfinite_safe(radius_squared) && radius_squared > 1e-20f) ? radius_squared : 1e-20f;
}

/* Octahedral normal packing: two signed 16 bit components in one float's bit
 * pattern. Used for photon deposit normals (pos.w) and the film measurement
 * point normal; plenty of precision for orientation tests. */

ccl_device_inline float photon_pack_normal(const float3 n)
{
  const float inv_l1 = 1.0f / max(fabsf(n.x) + fabsf(n.y) + fabsf(n.z), 1e-12f);
  float u = n.x * inv_l1;
  float v = n.y * inv_l1;
  if (n.z < 0.0f) {
    const float uo = u;
    u = (1.0f - fabsf(v)) * (uo >= 0.0f ? 1.0f : -1.0f);
    v = (1.0f - fabsf(uo)) * (v >= 0.0f ? 1.0f : -1.0f);
  }
  const uint iu = (uint)((clamp(u, -1.0f, 1.0f) * 0.5f + 0.5f) * 65535.0f + 0.5f);
  const uint iv = (uint)((clamp(v, -1.0f, 1.0f) * 0.5f + 0.5f) * 65535.0f + 0.5f);
  return __uint_as_float((iu << 16) | iv);
}

ccl_device_inline float3 photon_unpack_normal(const float packed)
{
  const uint bits = __float_as_uint(packed);
  const float u = ((bits >> 16) & 0xFFFFu) * (2.0f / 65535.0f) - 1.0f;
  const float v = (bits & 0xFFFFu) * (2.0f / 65535.0f) - 1.0f;
  float3 n = make_float3(u, v, 1.0f - fabsf(u) - fabsf(v));
  if (n.z < 0.0f) {
    const float nx = n.x;
    n.x = (1.0f - fabsf(n.y)) * (nx >= 0.0f ? 1.0f : -1.0f);
    n.y = (1.0f - fabsf(nx)) * (n.y >= 0.0f ? 1.0f : -1.0f);
  }
  return normalize(n);
}

/* Fractional light-group marker distinguishes beams from surface deposits. */
ccl_device_inline bool photon_deposit_is_volume(const float packed_lightgroup)
{
  const float fraction = packed_lightgroup - floorf(packed_lightgroup);
  return fraction >= 0.375f && fraction <= 0.625f;
}

CCL_NAMESPACE_END
/* === CyclesPlus: Personal Photon Grid End === */
