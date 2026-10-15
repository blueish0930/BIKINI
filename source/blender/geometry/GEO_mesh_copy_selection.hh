/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo
 */

#pragma once

#include <optional>

#include "BLI_array.hh"
#include "BLI_index_mask_fwd.hh"
#include "BLI_span.hh"
#include "BLI_virtual_array.hh"

#include "BKE_attribute_filter.hh"

namespace blender {

struct Mesh;
namespace fn {
template<typename T> class Field;
}
namespace bke {
enum class AttrDomain : int8_t;
}  // namespace bke

namespace geometry {

std::optional<Mesh *> mesh_copy_selection(const Mesh &src_mesh,
                                          const VArray<bool> &selection,
                                          bke::AttrDomain selection_domain,
                                          const bke::AttributeFilter &attribute_filter = {});

std::optional<Mesh *> mesh_copy_selection_keep_verts(
    const Mesh &src_mesh,
    const VArray<bool> &selection,
    bke::AttrDomain selection_domain,
    const bke::AttributeFilter &attribute_filter = {});

std::optional<Mesh *> mesh_copy_selection_keep_edges(
    const Mesh &mesh,
    const VArray<bool> &selection,
    bke::AttrDomain selection_domain,
    const bke::AttributeFilter &attribute_filter = {});

/**
 * Copy the faces of every group into a mesh of its own, together with the vertices and edges
 * they use. Unlike calling #mesh_copy_selection once per group, the cost of a group depends on
 * its own size and not on the size of the whole mesh.
 */
Array<Mesh *> mesh_copy_face_groups(const Mesh &src_mesh,
                                    Span<IndexMask> face_groups,
                                    const bke::AttributeFilter &attribute_filter = {});

}  // namespace geometry
}  // namespace blender
