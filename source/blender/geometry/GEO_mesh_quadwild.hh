/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/**
 * \file
 * Geometry Nodes entry for QRemeshify (ksami) / QuadWild-BiMDF.
 *
 * Three stages (node Mode menu):
 *   Remesh — full pipeline → pure quads
 *   Field  — stop after remeshAndField; write 4-RoSy face attributes
 *   Layout — stop after trace; edge-only separatrix from .patch + seam / boundary attrs
 *
 * Implementation: headless Blender runs bundled qremeshify_bridge.py which
 * imports the QRemeshify package (no reimplemented remesh math in C++).
 */

#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include <string>

namespace blender {

struct Mesh;

namespace geometry {

/** Pipeline stop / product mode. */
enum class QuadWildMode {
  Remesh = 0,
  Field = 1,
  Layout = 2,
};

struct QuadWildOptions {
  QuadWildMode mode = QuadWildMode::Remesh;

  /**
   * Sharp feature dihedral angle in degrees.
   * &lt; 0 disables sharp features (organic).
   */
  float sharp_feature_threshold_deg = 35.0f;
  /** QR-stage alpha (Remesh only). */
  float alpha = 0.005f;
  /** QR-stage scaleFact (Remesh only). */
  float scale_fact = 1.0f;
  /** Preprocess remesh before field (all modes). */
  bool do_remesh = true;
  /** Smoothed quadrangulation (Remesh only). */
  bool smooth_output = true;
  bool align_singularities = true;
  std::string satsuma_config;
  std::string flow_config;

  /** Field mode: face attributes `{prefix}0`…`{prefix}3`. */
  std::string field_attribute_prefix = "qw_dir";

  /** Layout mode: mark edge seam flag on patch boundaries. */
  bool mark_seams = true;
  /** Layout mode: optional bool edge attribute name (empty = skip). */
  std::string layout_boundary_attribute = "qw_layout_boundary";

  bool apply_rot_scale = false;
  float4x4 rot_scale = float4x4::identity();

  /** Extra sharp edges as input vertex pairs (0-based). Used even if Sharp Angle is disabled. */
  Vector<int2> hard_edges;
};

struct QuadWildResultInfo {
  bool success = false;
  std::string message;
  std::string package_dir;
};

Mesh *mesh_quadwild(const Mesh &src_mesh,
                    const QuadWildOptions &options,
                    QuadWildResultInfo *r_info = nullptr);

std::string quadwild_package_dir_find();

}  // namespace geometry
}  // namespace blender
