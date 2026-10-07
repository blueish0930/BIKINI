/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>

#include "BLI_index_mask_fwd.hh"

#include "BKE_attribute_filter.hh"

struct Mesh;

namespace blender::geometry {

/**
 * Remove the selected edges and join the faces at both of their sides into one face. Only edges
 * that are used by exactly two faces are dissolved. Edges are kept if dissolving them would
 * result in a face with a hole or in a face that uses a vertex more than once.
 *
 * \param dissolve_verts: Also remove the vertices of the dissolved edges that are connected to
 * exactly two edges afterwards, joining these two edges into one.
 * \param vert_angle_threshold: These vertices are kept if the direction changes by more than
 * this angle from one of their edges to the other.
 * \return #std::nullopt if the mesh is unchanged.
 */
std::optional<Mesh *> dissolve_edges(const Mesh &src_mesh,
                                     const IndexMask &edge_mask,
                                     bool dissolve_verts,
                                     float vert_angle_threshold,
                                     const bke::AttributeFilter &attribute_filter = {});

}  // namespace blender::geometry
