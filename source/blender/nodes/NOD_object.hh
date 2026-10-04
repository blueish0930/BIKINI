/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_node.hh"

struct Main;
struct Scene;
struct ViewLayer;
struct bContext;
struct bNodeTree;
struct wmOperatorType;

namespace blender {

extern bke::bNodeTreeType *ntreeType_Object;

void register_node_tree_type_obj();
void register_object_nodes();
void register_node_type_obj_custom_group(bke::bNodeType *ntype);

/** Seed a new empty ObjectNodeTree with Object → Set Transform → Object Output. */
void node_tree_object_default_init(const bContext *C, bNodeTree *ntree);

/**
 * Evaluate an Object Editor tree and apply object property, slot,
 * parent, create and delete operations to the scene. Set Modifiers edits the
 * live Object.modifiers stack and is not rewritten during evaluation.
 */
bool ntreeObjectNodesEvaluate(Main &bmain, Scene &scene, ViewLayer &view_layer, bNodeTree &ntree);

/** Evaluate every ObjectNodeTree in #bmain (file-load / recook). */
void ntreeObjectNodesEvaluateAll(Main &bmain, Scene *scene = nullptr);

void ntreeObjectNodesRegisterLoadHooks();

}  // namespace blender
