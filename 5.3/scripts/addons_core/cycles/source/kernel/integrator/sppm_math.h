/* SPDX-FileCopyrightText: 2026 BIKINI
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* === BIKINI SPPM Begin === */

#pragma once

/* Hachisuka, Ogaki, Jensen, SIGGRAPH Asia 2008 (PPM) and
 * Hachisuka & Jensen, SIGGRAPH Asia 2009 (SPPM).
 *
 * Per measurement point / pixel:
 *   N'   = N + α M
 *   r'²  = r² · (N + α M) / (N + M)
 *   τ'   = (τ + Φ) · (r'/r)²
 * Displayed radiance (constant 2D kernel):
 *   L = τ / (π r² N_e)
 */

#include "util/math.h"

#ifndef ccl_private
#  define ccl_private
#endif
#ifndef ccl_device_inline
#  define ccl_device_inline static inline
#endif

CCL_NAMESPACE_BEGIN

struct SPPMStat {
  float N;
  float r2;
  float tau_x;
  float tau_y;
  float tau_z;
};

/* SDS / caustic deposit: at least one specular event, then a front-facing
 * diffuse receiver. Photons that never hit glass/metal do not deposit. */
ccl_device_inline bool sppm_should_deposit(const int specular_events,
                                           const bool is_receiver,
                                           const float cos_in)
{
  return specular_events >= 1 && is_receiver && cos_in > 0.0f;
}

ccl_device_inline bool sppm_refract(const float3 d,
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

ccl_device_inline void sppm_update(ccl_private SPPMStat *s,
                                   const float M,
                                   const float3 Phi,
                                   const float alpha)
{
  if (!(M > 0.0f)) {
    return;
  }
  if (!(alpha > 0.0f && alpha < 1.0f) || !(s->N >= 0.0f) || !(s->r2 > 0.0f)) {
    s->tau_x += Phi.x;
    s->tau_y += Phi.y;
    s->tau_z += Phi.z;
    s->N += M;
    return;
  }
  const float N = s->N;
  const float Np = N + alpha * M;
  const float scale = Np / (N + M);
  s->tau_x = (s->tau_x + Phi.x) * scale;
  s->tau_y = (s->tau_y + Phi.y) * scale;
  s->tau_z = (s->tau_z + Phi.z) * scale;
  s->r2 *= scale;
  s->N = Np;
}

ccl_device_inline float3 sppm_radiance(const SPPMStat s, const float N_emitted)
{
  const float denom = M_PI_F * s.r2 * fmaxf(N_emitted, 1.0f);
  if (!(denom > 0.0f) || !isfinite_safe(denom)) {
    return zero_float3();
  }
  return make_float3(s.tau_x / denom, s.tau_y / denom, s.tau_z / denom);
}

CCL_NAMESPACE_END

/* === BIKINI SPPM End === */
