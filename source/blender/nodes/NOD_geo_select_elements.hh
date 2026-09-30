/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 *
 * Helpers for the Geometry Nodes "Select Elements" node: geometry cache for
 * interactive viewport edit, and writing selection indices back to node storage.
 */

#pragma once

#include <optional>

#include "BKE_geometry_set.hh"

#include "DNA_node_types.h"

namespace blender::nodes {

/** Request that a Select Elements node is force-evaluated as a side-effect. */
struct SelectElementsForceEvalRequest {
  /** #bNodeTree.id.session_uid of the root geometry node group on the modifier. */
  uint root_tree_session_uid = 0;
  /**
   * When true, #node_identifier is a top-level node in the root tree.
   * When false, #nested_node_ref_id is a #bNestedNodeRef.id path into the root tree.
   */
  bool is_root_level = true;
  int32_t node_identifier = 0;
  int nested_node_ref_id = 0;
};

void select_elements_set_force_eval_request(const SelectElementsForceEvalRequest &request);
void select_elements_clear_force_eval_request();
std::optional<SelectElementsForceEvalRequest> select_elements_get_force_eval_request();

/** Cache the last evaluated input geometry for interactive selection edit. */
void select_elements_cache_geometry(const bNodeTree &tree,
                                    const bNode &node,
                                    bke::GeometrySet geometry);

/** Copy of cached geometry, if any (thread-safe). */
std::optional<bke::GeometrySet> select_elements_copy_cached_geometry(const bNodeTree &tree,
                                                                     const bNode &node);

/** Drop cached geometry for one node (or ignore if absent). */
void select_elements_clear_cache(const bNodeTree &tree, const bNode &node);

/**
 * Drop every cached GeometrySet. Must run before GPU teardown on exit so Mesh
 * batch caches are not freed after Vulkan/OpenGL is already gone.
 */
void select_elements_free_all_caches();

/**
 * Replace stored selection on the node.
 * \param domain AttrDomain of the selection (Point/Edge/Face or Curve).
 * \param indices Unique element indices on that domain.
 */
void select_elements_set_indices(bNode &node, bke::AttrDomain domain, Span<int> indices);

/** True when the node is a Select Elements node. */
bool is_select_elements_node(const bNode &node);

}  // namespace blender::nodes
