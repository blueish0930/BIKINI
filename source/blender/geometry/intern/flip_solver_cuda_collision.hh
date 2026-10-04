/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/* Plain host pointers used only during a CUDA substep upload. The owning MacGrid lives in the
 * FLIP solver; the CUDA runner never retains these pointers after the callback returns. */
struct FlipCudaColliderFields {
  bool dirty = false;
  /** The fields are fully sampled (not skipped because the liquid is far away). */
  bool complete = false;
  const float *cell_phi = nullptr;
  /** Outward collider normal at each cell center (3 floats per cell), may be null. */
  const float *cell_normal = nullptr;
  const unsigned char *blocked[3] = {nullptr, nullptr, nullptr};
  const float *velocity[3] = {nullptr, nullptr, nullptr};
};
