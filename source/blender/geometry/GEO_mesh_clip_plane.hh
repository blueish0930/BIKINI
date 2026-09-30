/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_array.hh"
#include "BLI_math_vector_types.hh"

#include "BKE_attribute_filter.hh"

namespace blender {

struct Mesh;

namespace geometry {

struct MeshPlaneClipResult {
  Mesh *above = nullptr;
  Mesh *below = nullptr;
  /** Point-domain: vertices created on the clip plane (and original verts on the plane). */
  Array<bool> above_clip_boundary;
  Array<bool> below_clip_boundary;
};

/**
 * Clip a mesh by a half-space defined by a plane.
 *
 * This is a surface half-space clip (Sutherland–Hodgman style face clipping), not a solid
 * boolean. It does not require manifold / watertight input, does not fill the cut with a cap,
 * and works for open meshes, non-manifold edges, and ordinary polygonal surfaces.
 *
 * The kept side is the half-space that includes the plane itself:
 * - keep_above: points with signed distance >= 0
 * - keep_below: points with signed distance <= 0
 * where signed distance is `dot(position - plane_position, plane_normal)`.
 *
 * Attributes are propagated:
 * - numeric point/corner attributes are linearly interpolated at edge intersections
 * - non-mixable string attributes use a deterministic nearest-endpoint rule
 * - face attributes are copied from the source face
 */
Mesh *mesh_clip_by_plane(const Mesh &src_mesh,
                         const float3 &plane_position,
                         const float3 &plane_normal,
                         bool keep_above,
                         const bke::AttributeFilter &attribute_filter = {});

/** Calculate both requested sides in one classification pass. */
MeshPlaneClipResult mesh_clip_by_plane_both(
    const Mesh &src_mesh,
    const float3 &plane_position,
    const float3 &plane_normal,
    bool need_above,
    bool need_below,
    const bke::AttributeFilter &above_filter = {},
    const bke::AttributeFilter &below_filter = {});

}  // namespace geometry
}  // namespace blender
