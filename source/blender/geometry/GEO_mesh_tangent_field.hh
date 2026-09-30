/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup geo
 *
 * Smooth N-RoSy tangent fields via the Directional library (power fields),
 * with Houdini-like alignment to curvature, boundary, and optional guide vectors.
 *
 * Paper/software: Amir Vaxman et al., Directional — SGP 2021 Software Award
 *   https://github.com/avaxman/Directional
 * Method: directional::power_field + power_to_raw (N tangents per vertex).
 *
 * Attribute names: `{prefix}0` … `{prefix}{N-1}` on the point domain.
 */

#include <string>

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

namespace blender {

struct Mesh;

namespace geometry {

struct TangentFieldOptions {
  /**
   * N-RoSy order. Clamped by the Geometry Node (typically 1…12).
   * Odd N (3, 5, …) is valid for Directional power fields.
   */
  int n = 4;
  /**
   * Attribute name prefix. Direction k is stored as `{prefix}{k}`.
   * Example: "dir" → dir0, dir1, …
   */
  std::string attribute_prefix = "dir";
  /** Normalize each raw direction to unit length. */
  bool normalize = true;

  /**
   * Balance smoothness vs alignment constraints (Houdini "Alignment Weight").
   * 0 = pure smooth eigenmode (gauge-fixed); 1 = strongly follow constraints.
   */
  float alignment_weight = 0.45f;

  /**
   * Align to principal curvature directions (N ≥ 2 only; ignored for N=1).
   * Weight is relative among alignment channels (curvature / boundary / guide).
   */
  float curvature_weight = 1.0f;
  /** Extra rotation of curvature directions in the tangent plane (radians). */
  float curvature_rotation = 0.0f;

  /**
   * Align to open-boundary edge tangents.
   * Strongly improves strip-like fields on open meshes.
   */
  float boundary_weight = 1.0f;
  /** Extra rotation of boundary tangents in the tangent plane (radians). */
  float boundary_rotation = 0.0f;

  /**
   * Soft-align to a per-point guide vector field (zero length = no guide at that point).
   * Size must equal mesh.verts_num when used.
   */
  float guide_weight = 0.0f;
  Span<float3> guide_vectors;
};

/**
 * Compute a smooth N-RoSy power field on a triangle mesh (Directional library)
 * and store N tangent vectors per vertex as named attributes.
 *
 * \return N on success, 0 on failure.
 */
int mesh_tangent_field_store_attributes(Mesh &mesh, const TangentFieldOptions &options);

}  // namespace geometry
}  // namespace blender
