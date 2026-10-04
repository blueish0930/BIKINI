/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string>

#include "BLI_index_mask_fwd.hh"

#include "vex_program.hh"

namespace blender::nodes::vex {

/**
 * Run the per-element program as a compute shader. Attributes live in SSBOs for the whole
 * domain; `point(0, name, i)` is an indexed load. When #Program::array_passes is greater than
 * one, the same shader is dispatched that many times with ping-pong buffers so each pass
 * reads the previous pass's writes. Returns false to fall back to the CPU VM.
 */
bool gpu_try_run(const Program &program,
                 VMEnv &env,
                 const blender::IndexMask &mask,
                 int domain_size,
                 std::string &r_error);

}  // namespace blender::nodes::vex
