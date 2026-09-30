/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <cstdint>

#include "COM_context.hh"
#include "COM_result.hh"

namespace blender::compositor {

/* True when nvngx_dlssnr.dll is present and an NVIDIA D3D12 adapter exists. */
bool is_dlssnr_available();

/* Last failure, valid until the next probe/apply. Never null. */
const char *dlssnr_last_error();

struct DLSSNRParams {
  float intensity = 1.0f;
  float structure = 1.0f;
  float tone = 1.0f;
  float skin = -1.0f;
  int style = 2;
  bool reset = false;
  uint64_t view_key = 0;
  bool auto_mask = false;
  bool ui_correction = false;
  bool clamp_input = true;
};

/* Neural appearance pass. Color must already be a CPU image. Optional guides may be null or
 * single-value. Output is allocated as a CPU Color image at the input resolution. */
bool apply_dlssnr(Context &context,
                  const Result &color,
                  const Result *depth,
                  const Result *motion,
                  const Result *mask,
                  const Result *albedo,
                  const Result *normal,
                  const DLSSNRParams &params,
                  Result &output);

}  // namespace blender::compositor
