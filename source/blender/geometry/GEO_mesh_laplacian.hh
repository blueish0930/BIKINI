/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

namespace blender {

struct Mesh;

namespace geometry {

/** Weight formula for the discrete mesh Laplacian. */
enum class MeshLaplacianWeightMode {
  /** Graph Laplacian: off-diagonals −1 for each unique edge, diagonal = degree. */
  Uniform = 0,
  /** Cotangent Laplacian (triangle meshes); off-diagonals accumulate −½ cot of opposite angles. */
  Cotangent = 1,
  /** Caller-provided matrix entries, see #MeshLaplacianOptions::custom_weights. */
  Custom = 2,
};

struct MeshLaplacianOptions {
  MeshLaplacianWeightMode mode = MeshLaplacianWeightMode::Cotangent;
  /**
   * Entries for #MeshLaplacianWeightMode::Custom, all three spans have the same length. The value
   * at (row, col) is used as it is: the sign is not changed and no diagonal is derived, so the
   * caller provides the diagonal too. Entries that share (row, col) accumulate; entries with
   * indices outside of the vertex range are skipped.
   */
  Span<float> custom_weights;
  Span<int> custom_rows;
  Span<int> custom_cols;
  /** When true, assemble I+tL, or M+tML when #use_mass is also true. */
  bool build_diffusion = false;
  /** Scalar t used when #build_diffusion is true. */
  float diffusion_t = 1.0f;
  /**
   * When true, multiply by the lumped diagonal mass M (face area / 3 per incident triangle).
   * Without diffusion this yields M L; with diffusion it yields M + t M L.
   */
  bool use_mass = false;
};

/**
 * Sparse COO storage for a vertex×vertex matrix: only nonzero entries.
 * Parallel arrays #weights / #rows / #cols are always the same length.
 */
struct MeshLaplacianCOO {
  Vector<float> weights;
  Vector<int> rows;
  Vector<int> cols;
};

/**
 * Build a discrete Laplacian (or diffusion) matrix for a triangle mesh in sparse COO form.
 *
 * \param positions Vertex positions.
 * \param corner_verts Flattened triangle corners, length = faces_num * 3.
 * \param faces_num Number of triangles.
 * \param options Weight mode, diffusion, and mass flags.
 *
 * Non-triangle faces must be triangulated by the caller. Exact-zero weights are dropped.
 * Indices are vertex indices in [0, positions.size()).
 * In custom mode the triangles are only used for the mass, so there may be none.
 */
MeshLaplacianCOO mesh_laplacian_build(Span<float3> positions,
                                      Span<int> corner_verts,
                                      int faces_num,
                                      const MeshLaplacianOptions &options);

/**
 * Convenience wrapper that reads positions and topology from a triangle #Mesh.
 * Faces that are not triangles are ignored (caller should triangulate first).
 */
MeshLaplacianCOO mesh_laplacian_from_mesh(const Mesh &mesh, const MeshLaplacianOptions &options);

}  // namespace geometry
}  // namespace blender
