/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup geo
 *
 * Instant Field-Aligned Meshes remeshing via the official Instant Meshes core
 * (Jakob et al., SIGGRAPH Asia 2015 / wjakob/instant-meshes).
 */

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

namespace blender {

struct Mesh;

namespace geometry {

/** Output mesh element family driven by RoSy / PoSy symmetry order. */
enum class InstantMeshesMode {
  /** 4-RoSy + 4-PoSy → pure or dominant quads. */
  Quad = 0,
  /** 6-RoSy + 3-PoSy → isotropic triangle mesh (official Instant Meshes convention). */
  Triangle = 1,
};

struct InstantMeshesOptions {
  InstantMeshesMode mode = InstantMeshesMode::Quad;
  /**
   * Target edge length ρ of the output lattice.
   * When ≤ 0, face_count / vertex_count is used, or auto (~1/16 input verts).
   */
  float edge_length = 0.0f;
  /** Desired face count; ignored if ≤ 0. Mutually exclusive with edge_length / vertex_count. */
  int face_count = 0;
  /** Desired vertex count; ignored if ≤ 0. */
  int vertex_count = 0;
  /**
   * Dihedral angle (radians) for crease normals.
   * When < 0, creases are disabled (smooth normals, official CLI default).
   */
  float crease_angle = 0.523598775598f; /* 30° */
  /** Nonlinear Gauss–Seidel iterations per multiresolution level (paper / official = 6). */
  int iterations_per_level = 6;
  /** Extract-time smoothing + ray reprojection steps (official default 2). */
  int smooth_iterations = 2;
  /**
   * When true (default), use extrinsic smoothness energies so fields snap to geometric features.
   */
  bool extrinsic = true;
  /** Constrain orientation/position fields along open boundary loops. */
  bool align_to_boundaries = false;
  /**
   * After extraction, convert to pure quads (Catmull–Clark style face/edge split).
   * Official default is pure (not dominant). Ignored in triangle mode.
   */
  bool pure_quad = true;
  /** Prefer deterministic (slower) algorithms. */
  bool deterministic = false;
};

/**
 * Remesh a triangle mesh with official Instant Meshes.
 *
 * \param src_mesh Must contain only triangular faces (caller triangulates if needed).
 * \return A new mesh (caller owns), or an empty mesh if extraction yields no faces.
 *
 * Attributes are not propagated; the output is a pure remesh of positions and topology.
 */
Mesh *mesh_instant_meshes(const Mesh &src_mesh,
                          const InstantMeshesOptions &options,
                          Span<int> hard_verts = {},
                          Span<int2> hard_edges = {});

}  // namespace geometry
}  // namespace blender
