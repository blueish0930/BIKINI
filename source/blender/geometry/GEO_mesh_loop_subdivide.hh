/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_attribute.hh"

namespace blender {

struct Mesh;

namespace geometry {

/**
 * Apply uniform Loop subdivision to a triangle mesh.
 *
 * Every level splits each triangle into four triangles. Point and corner attributes are
 * interpolated with the same masks as positions; edge and face attributes are propagated from
 * their source elements. String and other non-mixable values use deterministic nearest-source
 * propagation.
 *
 * The input must contain only triangular faces. Loose geometry is preserved and subdivided along
 * with the surface edges.
 */
Mesh *mesh_loop_subdivide(const Mesh &src_mesh,
                          int level,
                          const bke::AttributeFilter &attribute_filter);

}  // namespace geometry
}  // namespace blender