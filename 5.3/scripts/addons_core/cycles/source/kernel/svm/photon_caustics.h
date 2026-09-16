/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

/* === CyclesPlus: Personal Photon SVM Helpers Begin === */
#pragma once

/* CyclesPlus: the photon map ADDS to Cycles' caustics, it does not replace
 * them.
 *
 * It used to replace them. Both estimators describe the same light, so
 * running both counts it twice - suppressing the path-traced side looked
 * like the obviously correct thing to do.
 *
 * It is not, and an interior water shot showed why (tester report
 * 2026-08-25). Cycles suppresses caustics by refusing to build the specular
 * closure once a path has gone diffuse, which is far broader than "the
 * caustic": a bathtub floor looks up THROUGH the water surface to collect
 * bounced wall light, and with that closure gone it sees black. The photon
 * map never replaced that light - it only carries what arrives over an
 * unbroken specular chain straight from a lamp. So we were switching off
 * something we do not deliver. Measured on that scene: 16% of lit pixels
 * dropped to a sixth of their brightness, the water going flat black.
 *
 * Outdoors this hid for months: sunlight dominates a pool, so the part that
 * went missing was small (the documented pool3 "-12% broad" note is the same
 * effect). Interiors live on bounced light, so there it takes nearly
 * everything.
 *
 * The double counting that motivated the suppression turns out to be minor,
 * because the paths a photon map is good at are exactly the ones the path
 * tracer almost never finds. Measured against ground truth (PT @4096sp, no
 * clamp, no glossy filter) across a hard sun, a 20-degree soft sun and a
 * large area light: adding lands 2-3.5% above truth, replacing 1-1.5% below
 * it. Both are close; only one of them turns interiors black.
 *
 * This is also what the renderers artists come from do - Corona runs its
 * caustics solver alongside the path tracer rather than in place of it.
 *
 * CYCLESPLUS_PHOTON_REPLACE_PT=1 restores the old behaviour for support. */

CCL_NAMESPACE_BEGIN

/* Does the photon map REPLACE this material's path-traced caustics? Only
 * under the support hatch; normally nothing is replaced. */
ccl_device_inline bool photon_caustics_owned(KernelGlobals kg,
                                             const ccl_private ShaderData *sd)
{
  if (!kernel_data.integrator.photon_replace_pt) {
    return false;
  }
  if (!kernel_data.integrator.use_photon_caustics) {
    return false;
  }
  if (!kernel_data.integrator.photon_casters_selected) {
    /* Every material may cast; the host classifier picks the ones that can. */
    return true;
  }
  return kernel_data_fetch(shaders, (sd->shader & SHADER_MASK)).photon_cast != 0;
}

ccl_device_inline bool photon_caustics_reflective(KernelGlobals kg,
                                                  const ccl_private ShaderData *sd)
{
  return kernel_data.integrator.caustics_reflective && !photon_caustics_owned(kg, sd);
}

ccl_device_inline bool photon_caustics_refractive(KernelGlobals kg,
                                                  const ccl_private ShaderData *sd)
{
  return kernel_data.integrator.caustics_refractive && !photon_caustics_owned(kg, sd);
}

/* Does the photon map deliver the caustics this specular surface throws?
 * Same question the material gate asks, without the replace hatch. */
ccl_device_inline bool photon_caustic_caster(KernelGlobals kg,
                                             const ccl_private ShaderData *sd)
{
  if (!kernel_data.integrator.use_photon_caustics) {
    return false;
  }

  /* The host classifier decides this, not the closure that happens to be
   * sampled here: a glossy plastic throws a specular bounce in the kernel
   * while the classifier rightly files it as a receiver, and the photon walk
   * never carries light off it. Handing such a bounce to the photon map would
   * take light away with nothing to put back - the interior failure again.
   * No verdict (table not uploaded yet, unknown slot) means "not ours". */
  const int slot = sd->shader & SHADER_MASK;
  if (slot < 0 || slot >= kernel_data.integrator.photon_shader_count) {
    return false;
  }
  if (kernel_data_fetch(photon_shader_caster, slot) == 0) {
    return false;
  }

  /* In "selected materials" mode the artist has the last word. */
  if (kernel_data.integrator.photon_casters_selected) {
    return kernel_data_fetch(shaders, slot).photon_cast != 0;
  }
  return true;
}

/* Is the light about to reach this path already carried by the photon map?
 *
 * This is the partition that replaces the old scene-wide suppression. That one
 * refused to BUILD the specular closure once a path had gone diffuse, which
 * also took away light the photon map never delivers - a floor looking up
 * through water to collect bounced wall light - and interiors went black.
 *
 * Here nothing is suppressed at the closure. The path is built and traced as
 * usual, and only the one contribution the photon map already accounts for is
 * skipped, at the moment it would be added. A path that goes on to another
 * diffuse surface instead of reaching a light keeps everything it had. */
ccl_device_inline bool photon_caustic_path_owned(KernelGlobals kg, const uint32_t path_flag)
{
  return (kernel_data.integrator.photon_partition_pt & 1) != 0 &&
         kernel_data.integrator.photon_num > 0 &&
         (path_flag & PATH_RAY_PHOTON_CAUSTIC_CHAIN) != 0;
}

CCL_NAMESPACE_END
/* === CyclesPlus: Personal Photon SVM Helpers End === */
