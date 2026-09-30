/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "cgal_bridge.hh"
#include "cgal_types.hh"

#include <functional>

namespace blender::cgal_bridge {

/** Property map names stamped onto Surface_mesh for attribute topology. */
inline constexpr const char *PROP_V_SRC0 = "v:src0";
inline constexpr const char *PROP_V_SRC1 = "v:src1";
inline constexpr const char *PROP_V_FACTOR = "v:factor";
inline constexpr const char *PROP_F_SRC = "f:src";
inline constexpr const char *PROP_F_MESH = "f:mesh";

/** Import n-gons or triangles into CGAL Surface_mesh. Optionally stamp topology ids. */
bool mesh_in_to_surface_mesh(const MeshIn &in,
                             Surface_mesh &sm,
                             std::string &error,
                             bool stamp_topology_ids = true);

/**
 * Export Surface_mesh faces as-is (triangles and n-gons).
 * Does NOT fan-triangulate n-gons.
 * If topology property maps exist, fills MeshResult maps for attribute interpolation.
 * If e:constrained exists, fills seam_vert_a/b as endpoint pairs in result vertex index.
 */
MeshResult surface_mesh_to_result(const Surface_mesh &sm);

/** Same as surface_mesh_to_result but force fan-triangulation of every face. */
MeshResult surface_mesh_to_triangles(const Surface_mesh &sm);

/** Positions only, same vertex iteration order as mesh_in_to_surface_mesh. */
bool surface_mesh_positions_to_buffer(const Surface_mesh &sm, std::vector<float> &positions);

/**
 * Inverse of local triangulation using original-face ids (f:src, f:mesh).
 * Joins adjacent faces that share the same original parent across edges that
 * are NOT marked constrained (intersection / seam). Does NOT use coplanarity.
 *
 * \param is_constrained if non-null, edges where *is_constrained(e)==true are never joined.
 */
void detriangulate_by_original_face(
    Surface_mesh &sm,
    const std::function<bool(Surface_mesh::Edge_index)> *is_constrained = nullptr);

/** Ensure triangle mesh while propagating f:src / f:mesh onto new sub-faces. */
void triangulate_faces_keep_ids(Surface_mesh &sm);

/** Edge property: true if edge lies on a boolean/corefine intersection seam. */
inline constexpr const char *PROP_E_CONSTRAINED = "e:constrained";

/**
 * After algorithms that introduce verts without maps, fill edge-lerp maps for
 * unmapped verts from two original endpoints along an incident edge, when possible.
 */
void fill_missing_vert_maps_from_edges(Surface_mesh &sm);

/**
 * For subdivision: match new verts to edge midpoints / face centroids of the
 * pre-subdiv mesh (exact geometric parents), write edge-lerp / face-average maps.
 * Call BEFORE subdiv with stamped ids; AFTER subdiv with the same original
 * positions/connectivity snapshot is handled inside mesh_subdivision.
 */
void assign_subdiv_vert_maps(Surface_mesh &sm_before,
                             Surface_mesh &sm_after,
                             int original_verts_num,
                             int original_faces_num);

}  // namespace blender::cgal_bridge
