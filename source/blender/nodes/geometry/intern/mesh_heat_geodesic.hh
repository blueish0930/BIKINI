/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * Keenan Crane et al. heat method for geodesic distances on triangle meshes.
 *
 * Pipeline (Geodesics in Heat, Crane/Weischedel/Wardetzky):
 *  1. Short-time heat: (M + t L) u = δ with t = h² (mean edge length squared)
 *  2. Unit vector field X = −∇u / |∇u| on faces
 *  3. Integrated divergence b = ∇·X at vertices
 *  4. Poisson solve L φ = b with sources pinned to 0
 *
 * Always uses the cotangent Laplacian + sparse Linear Solver (LLT).
 */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"

#include <string>

namespace blender::nodes::mesh_heat_geodesic {

struct HeatGeodesicResult {
  bool success = false;
  std::string message;
  /** Per-vertex geodesic distance; 0 on sources when successful. */
  Vector<float> distance;
  /** Effective diffusion time t = h². */
  float time_used = 0.0f;
  /** Mean edge length h. */
  float mean_edge = 0.0f;
};

/**
 * Compute approximate geodesic distances from source vertices via the heat method.
 *
 * \param positions Vertex positions (length n).
 * \param corner_verts Flattened triangle corners, length faces_num * 3.
 * \param faces_num Number of triangles.
 * \param is_source Length n; true marks heat sources / distance zeros.
 */
HeatGeodesicResult compute(Span<float3> positions,
                           Span<int> corner_verts,
                           int faces_num,
                           Span<bool> is_source);

}  // namespace blender::nodes::mesh_heat_geodesic
