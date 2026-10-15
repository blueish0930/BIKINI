/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 *
 * Helpers for Geometry Nodes "Edit Elements": cache cooked input geometry and
 * write the interactively edited component back to the node (#bNode.id).
 */

#pragma once

#include <optional>

#include "BKE_geometry_set.hh"

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

#include "DNA_node_types.h"

struct ID;

namespace blender::nodes {

/** Same force-eval request shape as Select Elements (root tree side-effect). */
struct EditElementsForceEvalRequest {
  uint root_tree_session_uid = 0;
  bool is_root_level = true;
  int32_t node_identifier = 0;
  int nested_node_ref_id = 0;
};

void edit_elements_set_force_eval_request(const EditElementsForceEvalRequest &request);
void edit_elements_clear_force_eval_request();
std::optional<EditElementsForceEvalRequest> edit_elements_get_force_eval_request();

void edit_elements_cache_geometry(const bNodeTree &tree,
                                  const bNode &node,
                                  bke::GeometrySet geometry);
std::optional<bke::GeometrySet> edit_elements_copy_cached_geometry(const bNodeTree &tree,
                                                                   const bNode &node);
void edit_elements_clear_cache(const bNodeTree &tree, const bNode &node);

/**
 * Drop every cached GeometrySet. Must run before GPU teardown on exit so Mesh
 * batch caches are not freed after Vulkan/OpenGL is already gone.
 */
void edit_elements_free_all_caches();

/**
 * Store the edited component on the node (#bNode.id): a #Mesh, #Curves or #PointCloud. Edited
 * instances are a #PointCloud with one point per instance, see #elements_instances_to_points.
 * Clears any legacy position array. Takes a user ref on the data-block, null removes the result.
 */
void edit_elements_set_result(bNode &node,
                              ID *result,
                              bke::GeometryComponent::Type component_type);

/** Data-block with the edited component stored on the node, or null. */
const ID *edit_elements_get_result(const bNode &node);

/** Component that the stored result replaces, only meaningful when there is a result. */
bke::GeometryComponent::Type edit_elements_result_component_type(const bNode &node);

/** @deprecated Legacy position-only storage. Prefer #edit_elements_set_result. */
void edit_elements_set_positions(bNode &node, Span<float3> positions);

bool is_edit_elements_node(const bNode &node);

}  // namespace blender::nodes
