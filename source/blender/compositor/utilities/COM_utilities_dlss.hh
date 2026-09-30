/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_context.hh"
#include "COM_result.hh"

namespace blender::compositor {

/* True when CUDA + NGX Ray Reconstruction can run on this machine. */
bool is_dlss_available();

/* Last failure from probe/init/evaluate. Never null. */
const char *dlss_last_error();

/* Denoise with NVIDIA DLSS Ray Reconstruction (DLSS 4.5). Color must already be a CPU image.
 * Optional guide buffers may be null or single-value; missing guides are filled with defaults.
 * Output is allocated as a CPU Color image at the input resolution (no upscaling). */
bool denoise_with_dlss(Context &context,
                       const Result &color,
                       const Result *albedo,
                       const Result *normal,
                       const Result *depth,
                       const Result *specular_albedo,
                       const Result *roughness,
                       const Result *motion,
                       const Result *specular_motion,
                       Result &output);

}  // namespace blender::compositor
