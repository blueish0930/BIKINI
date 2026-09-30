/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup geo
 *
 * Isotropic / adaptive triangle remeshing via pmp-library
 * (Botsch & Kobbelt style — Houdini Remesh SOP family).
 */

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

namespace blender {

struct Mesh;

namespace geometry {

enum class TriangleRemeshMode {
  Uniform = 0,
  Adaptive = 1,
};

struct TriangleRemeshOptions {
  TriangleRemeshMode mode = TriangleRemeshMode::Uniform;
  /** Uniform target edge length. Must be > 0 in uniform mode. */
  float edge_length = 0.1f;
  /** Adaptive bounds and approximation error. */
  float min_edge_length = 0.05f;
  float max_edge_length = 0.2f;
  float approx_error = 0.01f;
  /** Split / collapse / flip / smooth cycles (pmp default 10). */
  int iterations = 10;
  /** Project relaxed vertices back onto the input surface. */
  bool use_projection = true;
  /**
   * Dihedral crease angle in radians. Edges with larger angles are locked.
   * When ≤ 0, crease detection is disabled.
   */
  float crease_angle = 0.0f; /* 0 disables crease locking. */
  /** Lock open boundary edges and vertices as features (may still resample). */
  bool protect_boundaries = true;
  /**
   * Freeze the open-boundary vertex ring: do not split or collapse
   * open-boundary edges (no vertices inserted or deleted on the boundary).
   */
  bool preserve_boundary_topology = true;
};

/**
 * Remesh a triangle mesh with pmp-library isotropic remeshing.
 *
 * \param src_mesh Must contain only triangular faces (caller triangulates if needed).
 * \param hard_verts Input vertex indices that must stay (feature vertices).
 * \param hard_edges Input vertex pairs that must stay (feature edges).
 * \return A new mesh (caller owns), or an empty mesh on failure.
 *
 * Attributes are not propagated; the output is a pure remesh of positions and topology.
 */
Mesh *mesh_triangle_remesh(const Mesh &src_mesh,
                           const TriangleRemeshOptions &options,
                           Span<int> hard_verts = {},
                           Span<int2> hard_edges = {});

}  // namespace geometry
}  // namespace blender
