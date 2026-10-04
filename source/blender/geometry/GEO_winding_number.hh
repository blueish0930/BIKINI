/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Generalized winding number of a triangle soup at query points
 * (Jacobson et al. 2013; Barill et al. 2018).
 *
 * Closed, consistently oriented solids evaluate to ~1 inside and ~0 outside.
 * Open, self-intersecting, and non-manifold meshes still produce a continuous
 * insidedness (fractional or |w| > 1 at overlaps).
 */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

#include <memory>
#include <string>

namespace blender {

struct Mesh;

namespace geometry {

enum class WindingNumberMethod {
  Fast = 0,
  Exact = 1,
};

/**
 * Build once, then evaluate at arbitrary query positions.
 * \param accuracy Far-field opening-angle factor for Fast (typical 2). Ignored for Exact.
 */
class MeshWindingNumber {
 public:
  MeshWindingNumber();
  ~MeshWindingNumber();
  MeshWindingNumber(MeshWindingNumber &&) noexcept;
  MeshWindingNumber &operator=(MeshWindingNumber &&) noexcept;
  MeshWindingNumber(const MeshWindingNumber &) = delete;
  MeshWindingNumber &operator=(const MeshWindingNumber &) = delete;

  bool build(const Mesh &mesh,
             float accuracy,
             WindingNumberMethod method,
             std::string &r_error);
  bool is_valid() const;
  float eval(const float3 &p) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace geometry
}  // namespace blender
