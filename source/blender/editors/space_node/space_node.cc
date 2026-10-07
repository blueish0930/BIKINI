/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 */

#include "AS_asset_representation.hh"

#include <cstring>
#include <limits>
#include <ostream>

#include "BKE_node_socket_value.hh"
#include "BLI_listbase.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector_set.hh"

#include "DNA_ID.h"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_material_types.h"
#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "MEM_guardedalloc.h"

#include "BKE_anim_data.hh"
#include "BKE_annotations.h"
#include "BKE_asset.hh"
#include "BKE_attribute.hh"
#include "BKE_camera.h"
#include "BKE_collection.hh"
#include "BKE_compositor.hh"
#include "BKE_compute_context_cache.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_context.hh"
#include "BKE_global.hh"
#include "BKE_idprop.hh"
#include "BKE_layer.hh"
#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_lib_remap.hh"
#include "BKE_main.hh"
#include "BKE_material.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"
#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_object.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_node_tree_zones.hh"
#include "BKE_scene.hh"
#include "BKE_screen.hh"
#include "BKE_world.h"

#include "BLI_math_geom_c.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_matrix.hh"

#include "DNA_camera_types.h"
#include "DNA_layer_types.h"
#include "DNA_light_types.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_scene_types.h"
#include "DNA_view3d_types.h"
#include "DNA_world_types.h"

#include "BLI_vector.hh"

#include "DRW_engine.hh"

#include "BLT_translation.hh"

#include "ED_asset_shelf.hh"
#include "ED_image.hh"
#include "ED_node.hh"
#include "ED_node_preview.hh"
#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_view3d_offscreen.hh"

#include "GEO_mesh_primitive_grid.hh"

#include "RE_pipeline.h"

#include "UI_view2d.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_build.hh"
#include "DEG_depsgraph_query.hh"

#include "BLO_read_write.hh"

#include "DNA_object_enums.h"
#include "DNA_view3d_enums.h"

#include "GPU_framebuffer.hh"
#include "GPU_texture.hh"
#include "GPU_viewport.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "NOD_image.hh"
#include "NOD_trace_values.hh"

#include "COM_undefined_node_operation.hh"

#include "io_utils.hh"

#include "node_intern.hh" /* own include */

namespace blender {

/* Session memory: last node-editor tree type and last Image Process group. */
static char g_last_node_tree_idname[64] = "";
static int g_last_image_node_group_session_uid = 0;

static void remember_node_editor_tree(const SpaceNode &snode)
{
  if (snode.tree_idname[0]) {
    STRNCPY_UTF8(g_last_node_tree_idname, snode.tree_idname);
  }
  /* Prefer explicit Image Process picks / roots even when currently nested in Geometry (Import
   * Points) or about to switch tree type away. */
  if (snode.selected_node_group && snode.selected_node_group->type == NTREE_IMAGE) {
    g_last_image_node_group_session_uid = int(snode.selected_node_group->id.session_uid);
  }
  else if (snode.nodetree && snode.nodetree->type == NTREE_IMAGE) {
    g_last_image_node_group_session_uid = int(snode.nodetree->id.session_uid);
  }
  else if (snode.edittree && snode.edittree->type == NTREE_IMAGE) {
    g_last_image_node_group_session_uid = int(snode.edittree->id.session_uid);
  }
  else if (STREQ(snode.tree_idname, "ImageNodeTree")) {
    /* User unlinked the active Image group (header X). Clear session memory so redraw
     * does not re-stick the previous tree via ED_node_image_process_group_restore. */
    g_last_image_node_group_session_uid = 0;
  }
}

void ED_node_image_process_group_remember(const SpaceNode *snode)
{
  if (snode) {
    remember_node_editor_tree(*snode);
  }
}

static bNodeTree *find_image_node_group_by_session_uid(Main *bmain, const int session_uid)
{
  if (!bmain || session_uid == 0) {
    return nullptr;
  }
  for (bNodeTree &ntree : bmain->nodetrees) {
    if (ntree.type == NTREE_IMAGE && int(ntree.id.session_uid) == session_uid) {
      return &ntree;
    }
  }
  return nullptr;
}

bNodeTree *ED_node_image_process_group_restore(Main *bmain)
{
  return find_image_node_group_by_session_uid(bmain, g_last_image_node_group_session_uid);
}

/* ******************** tree path ********************* */

static void tag_guide_geometry_eval(SpaceNode & /*snode*/)
{
  /* Entering/exiting a node group only changes the editor path. Do not recook the modifier. */
}

void ED_node_tree_start(ARegion *region, SpaceNode *snode, bNodeTree *ntree, ID *id, ID *from)
{
  for (bNodeTreePath &path : snode->treepath.items_mutable()) {
    MEM_delete(&path);
  }
  snode->treepath.clear_no_delete();

  if (ntree) {
    bNodeTreePath *path = MEM_new<bNodeTreePath>("node tree path");
    path->nodetree = ntree;
    path->parent_key = bke::NODE_INSTANCE_KEY_BASE;

    /* Set initial view center from node tree. */
    copy_v2_v2(path->view_center, ntree->view_center);
    path->view_width = ntree->view_width;

    if (region) {
      /* Leave the zoom level unchanged if it hasn't been set before. */
      if (ntree->view_width != 0.0f) {
        ui::view2d_size_x_set(&region->v2d, ntree->view_width);
      }
      ui::view2d_center_set(&region->v2d, ntree->view_center[0], ntree->view_center[1]);
    }

    if (id) {
      STRNCPY_UTF8(path->display_name, id->name + 2);
    }

    BLI_addtail(&snode->treepath, path);

    if (ntree->type != NTREE_GEOMETRY) {
      /* This can probably be removed for all node tree types. It mainly exists because it was not
       * possible to store id references in custom properties. Also see #36024. I don't want to
       * remove it for all tree types in bcon3 though. */
      id_us_ensure_real(&ntree->id);
    }
  }

  /* update current tree */
  snode->nodetree = snode->edittree = ntree;
  snode->id = id;
  snode->from = from;

  if (ntree && ELEM(ntree->type, NTREE_IMAGE, NTREE_OBJECT)) {
    snode->selected_node_group = ntree;
  }
  else if (STREQ(snode->tree_idname, "ImageNodeTree")) {
    /* Explicit clear (unlink / empty start): drop Image group pointer so template_ID X sticks. */
    snode->selected_node_group = nullptr;
  }

  remember_node_editor_tree(*snode);

  ED_node_set_active_viewer_key(snode);
  snode->runtime->node_can_sync_states.clear();

  tag_guide_geometry_eval(*snode);

  WM_main_add_notifier(NC_SCENE | ND_NODES, nullptr);
}

void ED_node_tree_push(ARegion *region, SpaceNode *snode, bNodeTree *ntree, bNode *gnode)
{
  bNodeTreePath *path = MEM_new<bNodeTreePath>("node tree path");
  bNodeTreePath *prev_path = snode->treepath.last();
  path->nodetree = ntree;
  if (gnode) {
    if (prev_path) {
      path->parent_key = bke::node_instance_key(prev_path->parent_key, prev_path->nodetree, gnode);
    }
    else {
      path->parent_key = bke::NODE_INSTANCE_KEY_BASE;
    }

    STRNCPY_UTF8(path->node_name, gnode->name);
    STRNCPY_UTF8(path->display_name, gnode->name);
  }
  else {
    path->parent_key = bke::NODE_INSTANCE_KEY_BASE;
  }

  /* Set initial view center from node tree. */
  copy_v2_v2(path->view_center, ntree->view_center);
  path->view_width = ntree->view_width;
  if (region) {
    if (ntree->view_width != 0.0f) {
      ui::view2d_size_x_set(&region->v2d, ntree->view_width);
    }
    ui::view2d_center_set(&region->v2d, ntree->view_center[0], ntree->view_center[1]);
  }

  BLI_addtail(&snode->treepath, path);

  id_us_ensure_real(&ntree->id);

  /* update current tree
   * NOTE: Do NOT change snode->tree_idname here. Import Points nests a GeometryNodeTree under
   * Image Process — the editor must stay Image Process; only edittree/path change. Menus use
   * edit_tree.bl_idname when nested (see node_add menus). */
  snode->edittree = ntree;

  /* Force Image Process editor type when nesting Geometry under an Image root (switching must not
   * flip to Geometry Nodes / object modifier UI). */
  if (snode->nodetree && snode->nodetree->type == NTREE_IMAGE && ntree &&
      ntree->type == NTREE_GEOMETRY && ntreeType_Image)
  {
    STRNCPY_UTF8(snode->tree_idname, ntreeType_Image->idname.c_str());
  }

  ED_node_set_active_viewer_key(snode);
  snode->runtime->node_can_sync_states.clear();

  tag_guide_geometry_eval(*snode);

  WM_main_add_notifier(NC_SCENE | ND_NODES, nullptr);
}

void ED_node_tree_pop(ARegion *region, SpaceNode *snode)
{
  bNodeTreePath *path = snode->treepath.last();

  /* don't remove root */
  if (path == snode->treepath.first_) {
    return;
  }

  BLI_remlink(&snode->treepath, path);
  MEM_delete(path);

  /* update current tree */
  path = snode->treepath.last();
  snode->edittree = path->nodetree;

  /* Set view center and zoom from node tree path. */
  if (region) {
    if (path->view_width != 0.0f) {
      ui::view2d_size_x_set(&region->v2d, path->view_width);
    }
    ui::view2d_center_set(&region->v2d, path->view_center[0], path->view_center[1]);
  }

  ED_node_set_active_viewer_key(snode);
  snode->runtime->node_can_sync_states.clear();

  tag_guide_geometry_eval(*snode);

  WM_main_add_notifier(NC_SCENE | ND_NODES, nullptr);
}

int ED_node_tree_depth(SpaceNode *snode)
{
  return snode->treepath.count();
}

bNodeTree *ED_node_tree_get(SpaceNode *snode, int level)
{
  bNodeTreePath *path;
  int i;
  for (path = snode->treepath.last(), i = 0; path; path = path->prev, i++) {
    if (i == level) {
      return path->nodetree;
    }
  }
  return nullptr;
}

int ED_node_tree_path_length(SpaceNode *snode)
{
  int length = 0;

  for (const auto [i, path] : snode->treepath.enumerate()) {
    length += strlen(path.display_name);
    if (i > 0) {
      length += 1; /* for separator char */
    }
  }
  return length;
}

void ED_node_tree_path_get(SpaceNode *snode, char *value)
{

#ifndef NDEBUG
  const char *value_orig = value;
#endif
  /* Note that the caller ensures there is enough space available. */
  for (const auto [i, path] : snode->treepath.enumerate()) {
    const int len = strlen(path.display_name);
    if (i != 0) {
      *value++ = '/';
    }
    memcpy(value, path.display_name, len);
    value += len;
  }
  *value = '\0';
  BLI_assert(ptrdiff_t(ED_node_tree_path_length(snode)) == ptrdiff_t(value - value_orig));
}

void ED_node_set_active_viewer_key(SpaceNode *snode)
{
  bNodeTreePath *path = snode->treepath.last();
  if (snode->nodetree && path) {
    /* A change in active viewer may result in the change of the output node used by the
     * compositor, so we need to get notified about such changes. */
    if (snode->nodetree->active_viewer_key.value != path->parent_key.value &&
        snode->nodetree->type == NTREE_COMPOSIT)
    {
      DEG_id_tag_update(&snode->nodetree->id, ID_RECALC_NTREE_OUTPUT);
      WM_main_add_notifier(NC_NODE, nullptr);
    }
    /* Image Process: keep backdrop key in sync when path changes (enter/exit nest). */
    if (snode->nodetree->type == NTREE_IMAGE &&
        snode->nodetree->active_viewer_key.value != path->parent_key.value)
    {
      if (snode->runtime) {
        snode->runtime->image_eval_dirty = true;
      }
      WM_main_add_notifier(NC_NODE | NA_EDITED, snode->nodetree);
    }

    snode->nodetree->active_viewer_key = path->parent_key;
  }
}

void ED_node_cursor_location_get(const SpaceNode *snode, float value[2])
{
  copy_v2_v2(value, snode->runtime->cursor);
}

void ED_node_cursor_location_set(SpaceNode *snode, const float value[2])
{
  copy_v2_v2(snode->runtime->cursor, value);
}

namespace ed::space_node {

float2 space_node_group_offset(const SpaceNode &snode)
{
  const bNodeTreePath *path = snode.treepath.last();

  if (path && path->prev) {
    return float2(path->view_center) - float2(path->prev->view_center);
  }
  return float2(0);
}

std::optional<nodes::FoundNestedNodeID> find_nested_node_id_in_root(const SpaceNode &snode,
                                                                    const bNode &query_node)
{
  BLI_assert(snode.edittree->runtime->nodes_by_id.contains(const_cast<bNode *>(&query_node)));
  bke::ComputeContextCache compute_context_cache;
  const ComputeContext *compute_context = compute_context_for_edittree_node(
      snode, compute_context_cache, query_node);
  if (!compute_context) {
    return {};
  }
  return find_nested_node_id_in_root(*snode.nodetree, compute_context, query_node.identifier);
}

std::optional<nodes::FoundNestedNodeID> find_nested_node_id_in_root(
    const bNodeTree &root_tree, const ComputeContext *compute_context, const int node_id)
{
  nodes::FoundNestedNodeID found;
  Vector<int> node_ids;
  for (const ComputeContext *context = compute_context; context != nullptr;
       context = context->parent())
  {
    if (const auto *node_context = dynamic_cast<const bke::GroupNodeComputeContext *>(context)) {
      node_ids.append(node_context->node_id());
    }
    else if (dynamic_cast<const bke::RepeatZoneComputeContext *>(context) != nullptr) {
      found.is_in_loop = true;
    }
    else if (dynamic_cast<const bke::SimulationZoneComputeContext *>(context) != nullptr) {
      found.is_in_simulation = true;
    }
    else if (dynamic_cast<const bke::ForeachGeometryElementZoneComputeContext *>(context) !=
             nullptr)
    {
      found.is_in_loop = true;
    }
    else if (dynamic_cast<const bke::EvaluateClosureComputeContext *>(context) != nullptr) {
      found.is_in_closure = true;
    }
  }
  std::reverse(node_ids.begin(), node_ids.end());
  node_ids.append(node_id);
  const bNestedNodeRef *nested_node_ref = root_tree.nested_node_ref_from_node_id_path(node_ids);
  if (nested_node_ref == nullptr) {
    return std::nullopt;
  }
  found.id = nested_node_ref->id;
  return found;
}

static bool node_group_matches_tree(const bNodeTree *group, const bNodeTree *tree)
{
  if (!group || !tree) {
    return false;
  }
  if (group == tree) {
    return true;
  }
  /* Modifier eval / pin can hold CoW tree vs editor original — always compare originals. */
  return DEG_get_original(group) == DEG_get_original(tree);
}

static const NodesModifierData *find_nodes_modifier_on_object(const Object &object,
                                                             const bNodeTree *tree)
{
  if (!tree) {
    return nullptr;
  }
  const NodesModifierData *matching_any = nullptr;
  const NodesModifierData *matching_active = nullptr;
  for (const ModifierData &md : object.modifiers) {
    if (md.type != eModifierType_Nodes) {
      continue;
    }
    const NodesModifierData *nmd = reinterpret_cast<const NodesModifierData *>(&md);
    if (!node_group_matches_tree(nmd->node_group, tree)) {
      continue;
    }
    if (!matching_any) {
      matching_any = nmd;
    }
    if (md.flag & eModifierFlag_Active) {
      matching_active = nmd;
    }
  }
  return matching_active ? matching_active : matching_any;
}

Object *get_space_editor_object(const bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);

  if (snode && snode->from) {
    if (snode->from->id_type() == ID_OB) {
      return id_cast<Object *>(snode->from);
    }
  }

  return static_cast<Object *>(CTX_data_pointer_get(C, "object").data);
}

std::optional<ObjectAndModifier> get_geometry_nodes_modifier_for_node_editor(
    const SpaceNode &snode)
{
  if (snode.node_tree_sub_type != SNODE_GEOMETRY_MODIFIER) {
    return std::nullopt;
  }
  if (snode.id == nullptr) {
    return std::nullopt;
  }
  if (GS(snode.id->name) != ID_OB) {
    return std::nullopt;
  }
  const Object *object = reinterpret_cast<Object *>(snode.id);
  const NodesModifierData *used_modifier = find_nodes_modifier_on_object(*object, snode.nodetree);
  if (used_modifier == nullptr) {
    return std::nullopt;
  }
  return ObjectAndModifier{object, used_modifier};
}

std::optional<ObjectAndModifier> find_object_and_nodes_modifier_for_geometry_edit(
    const bContext *C, const SpaceNode &snode)
{
  /* 1) Normal node-editor modifier context (pinned / active object + subtype). */
  if (std::optional<ObjectAndModifier> om = get_geometry_nodes_modifier_for_node_editor(snode)) {
    return om;
  }

  const bNodeTree *tree = snode.nodetree ? snode.nodetree : snode.edittree;
  if (!tree || tree->type != NTREE_GEOMETRY) {
    return std::nullopt;
  }

  auto try_object = [&](const Object *object) -> std::optional<ObjectAndModifier> {
    if (!object) {
      return std::nullopt;
    }
    if (const NodesModifierData *nmd = find_nodes_modifier_on_object(*object, tree)) {
      return ObjectAndModifier{object, nmd};
    }
    return std::nullopt;
  };

  /* 2) Object stored on the space (even if subtype is wrong after pin/temp-object). */
  if (snode.id && GS(snode.id->name) == ID_OB) {
    if (std::optional<ObjectAndModifier> om = try_object(
            reinterpret_cast<const Object *>(snode.id)))
    {
      return om;
    }
  }

  if (!C) {
    return std::nullopt;
  }

  /* 3) Active object. */
  if (std::optional<ObjectAndModifier> om = try_object(CTX_data_active_object(C))) {
    return om;
  }

  /* 4) Selected objects. */
  {
    CTX_DATA_BEGIN (C, Object *, object, selected_objects) {
      if (std::optional<ObjectAndModifier> om = try_object(object)) {
        return om;
      }
    }
    CTX_DATA_END;
  }

  /* 5) Any object in the view layer that uses this node group. */
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (bmain && scene && view_layer) {
    BKE_view_layer_synced_ensure(*bmain, scene, view_layer);
    for (Base &base : *BKE_view_layer_object_bases_get(view_layer)) {
      if (std::optional<ObjectAndModifier> om = try_object(base.object)) {
        return om;
      }
    }
  }

  return std::nullopt;
}

bool node_editor_is_for_geometry_nodes_modifier(const SpaceNode &snode,
                                                const Object &object,
                                                const NodesModifierData &nmd)
{
  const std::optional<ObjectAndModifier> object_and_modifier =
      get_geometry_nodes_modifier_for_node_editor(snode);
  if (!object_and_modifier) {
    return false;
  }
  const Object *object_orig = DEG_is_original(&object) ? &object : DEG_get_original(&object);
  if (object_and_modifier->object != object_orig) {
    return false;
  }
  return object_and_modifier->nmd->modifier.persistent_uid == nmd.modifier.persistent_uid;
}

struct SceneAndCompositorEffect {
  const Scene *scene = nullptr;
  const SceneCompositorEffect *effect = nullptr;
};

static std::optional<SceneAndCompositorEffect> get_scene_compositor_effect_for_node_editor(
    const SpaceNode &space_node)
{
  if (space_node.node_tree_sub_type != SNODE_COMPOSITOR_SCENE) {
    return std::nullopt;
  }

  if (!space_node.id) {
    return std::nullopt;
  }

  if (GS(space_node.id->name) != ID_SCE) {
    return std::nullopt;
  }

  const Scene *scene = id_cast<Scene *>(space_node.id);
  if (space_node.flag & SNODE_PIN) {
    for (const SceneCompositorEffect &effect : scene->compositor_effects) {
      if (effect.node_group == space_node.nodetree) {
        return SceneAndCompositorEffect(scene, &effect);
      }
    }
    return std::nullopt;
  }

  const SceneCompositorEffect *active_effect = bke::compositor::get_active_effect(*scene);
  if (!active_effect) {
    return std::nullopt;
  }
  return SceneAndCompositorEffect(scene, active_effect);
}

const ComputeContext *compute_context_for_zone(const bke::bNodeTreeZone &zone,
                                               bke::ComputeContextCache &compute_context_cache,
                                               const ComputeContext *parent_compute_context)
{
  const bNode *output_node_ptr = zone.output_node();
  if (!output_node_ptr) {
    return nullptr;
  }
  const bNode &output_node = *output_node_ptr;
  switch (output_node.type_legacy) {
    case GEO_NODE_SIMULATION_OUTPUT: {
      return &compute_context_cache.for_simulation_zone(parent_compute_context, output_node);
    }
    case GEO_NODE_REPEAT_OUTPUT: {
      const auto &storage = *static_cast<const NodeGeometryRepeatOutput *>(output_node.storage);
      return &compute_context_cache.for_repeat_zone(
          parent_compute_context, output_node, storage.inspection_index);
    }
    case GEO_NODE_FOREACH_GEOMETRY_ELEMENT_OUTPUT: {
      const auto &storage = *static_cast<const NodeGeometryForeachGeometryElementOutput *>(
          output_node.storage);
      return &compute_context_cache.for_foreach_geometry_element_zone(
          parent_compute_context, output_node, storage.inspection_index);
    }
    case NODE_CLOSURE_OUTPUT: {
      nodes::ClosureSourceLocation source_location{};
      const bNodeTree &tree = output_node.owner_tree();
      source_location.tree = &tree;
      source_location.closure_output_node_id = output_node.identifier;
      source_location.compute_context_hash = parent_compute_context ?
                                                 parent_compute_context->hash() :
                                                 ComputeContextHash{};
      source_location.compute_context = parent_compute_context;
      return compute_context_for_closure_evaluation(parent_compute_context,
                                                    output_node.output_socket(0),
                                                    compute_context_cache,
                                                    source_location);
    }
  }
  return nullptr;
}

const ComputeContext *compute_context_for_zones(const Span<const bke::bNodeTreeZone *> zones,
                                                bke::ComputeContextCache &compute_context_cache,
                                                const ComputeContext *parent_compute_context)
{
  const ComputeContext *current = parent_compute_context;
  for (const bke::bNodeTreeZone *zone : zones) {
    current = compute_context_for_zone(*zone, compute_context_cache, current);
    if (!current) {
      return nullptr;
    }
  }
  return current;
}

static std::optional<const ComputeContext *> compute_context_for_tree_path(
    const SpaceNode &snode,
    bke::ComputeContextCache &compute_context_cache,
    const ComputeContext *parent_compute_context)
{
  const ComputeContext *current = parent_compute_context;
  Vector<const bNodeTreePath *> tree_path;
  for (const bNodeTreePath &item : snode.treepath) {
    tree_path.append(&item);
  }
  if (tree_path.is_empty()) {
    return current;
  }

  for (const int i : tree_path.index_range().drop_back(1)) {
    bNodeTree *tree = tree_path[i]->nodetree;
    const char *group_node_name = tree_path[i + 1]->node_name;
    const bNode *group_node = bke::node_find_node_by_name(*tree, group_node_name);
    if (group_node == nullptr) {
      return std::nullopt;
    }
    const bke::bNodeTreeZones *tree_zones = tree->zones();
    if (tree_zones == nullptr) {
      return std::nullopt;
    }
    const Vector<const bke::bNodeTreeZone *> zone_stack = tree_zones->get_zones_to_enter_from_root(
        tree_zones->get_zone_by_node(group_node->identifier));
    current = compute_context_for_zones(zone_stack, compute_context_cache, current);
    if (!current) {
      return std::nullopt;
    }
    current = &compute_context_cache.for_group_node(current, group_node->identifier, tree);
  }
  return current;
}

static const ComputeContext *get_node_editor_root_compute_context(
    const SpaceNode &snode, bke::ComputeContextCache &compute_context_cache)
{
  if (snode.nodetree->type == NTREE_GEOMETRY) {
    switch (SpaceNodeGeometryNodesType(snode.node_tree_sub_type)) {
      case SNODE_GEOMETRY_MODIFIER: {
        std::optional<ed::space_node::ObjectAndModifier> object_and_modifier =
            ed::space_node::get_geometry_nodes_modifier_for_node_editor(snode);
        if (!object_and_modifier) {
          return nullptr;
        }
        const bke::DataBlockComputeContext &object_context = compute_context_cache.for_data_block(
            nullptr, object_and_modifier->object->id);
        return &compute_context_cache.for_geometry_nodes_modifier(&object_context,
                                                                  *object_and_modifier->nmd);
      }
      case SNODE_GEOMETRY_TOOL: {
        return &compute_context_cache.for_operator(nullptr);
      }
    }
    return nullptr;
  }
  if (snode.nodetree->type == NTREE_COMPOSIT) {
    switch (SpaceNodeCompositorNodesType(snode.node_tree_sub_type)) {
      case SNODE_COMPOSITOR_SCENE: {
        std::optional<SceneAndCompositorEffect> scene_and_effect =
            ed::space_node::get_scene_compositor_effect_for_node_editor(snode);
        if (!scene_and_effect) {
          return nullptr;
        }
        const bke::DataBlockComputeContext &scene_context = compute_context_cache.for_data_block(
            nullptr, scene_and_effect->scene->id);
        return &compute_context_cache.for_scene_compositor_effect(&scene_context,
                                                                  *scene_and_effect->effect);
      }
      case SNODE_COMPOSITOR_SEQUENCER: {
        return nullptr;
      }
    }
    return nullptr;
  }
  if (snode.nodetree->type == NTREE_SHADER) {
    return &compute_context_cache.for_shader(nullptr, snode.nodetree);
  }
  /* Image Process: evaluation logs under Scene (ImageNodesContext uses scene.id), same as
   * Compositor. Nested Import Points Geometry keeps its own datablock context. */
  if (snode.nodetree->type == NTREE_IMAGE) {
    if (snode.edittree && snode.edittree->type == NTREE_GEOMETRY) {
      return &compute_context_cache.for_data_block(nullptr, snode.edittree->id);
    }
    if (snode.id && GS(snode.id->name) == ID_SCE) {
      return &compute_context_cache.for_data_block(nullptr, *snode.id);
    }
    /* Fallback when SpaceNode has no scene yet. */
    return &compute_context_cache.for_data_block(nullptr, snode.nodetree->id);
  }
  return nullptr;
}

class RecursiveInspectionComputeContext : public ComputeContext {
  ComputeContextHash hash_;

 public:
  RecursiveInspectionComputeContext(const ComputeContext *parent, const ComputeContextHash hash)
      : ComputeContext(parent), hash_(hash)
  {
  }

 private:
  ComputeContextHash compute_hash() const override
  {
    return hash_;
  }

  void print_current_in_line(std::ostream &stream) const override
  {
    stream << "Recursive inspection";
  }
};

[[nodiscard]] const ComputeContext *compute_context_for_edittree_base(
    const SpaceNode &snode, bke::ComputeContextCache &compute_context_cache)
{
  if (!snode.edittree) {
    return nullptr;
  }
  if (!ELEM(snode.edittree->type, NTREE_GEOMETRY, NTREE_SHADER, NTREE_COMPOSIT, NTREE_IMAGE)) {
    return nullptr;
  }
  /* Import Points nest under Image Process: compute context is the Geometry datablock directly
   * (see get_node_editor_root_compute_context) — do not walk Image→ImportPoints as a group path. */
  if (snode.nodetree && snode.nodetree->type == NTREE_IMAGE &&
      snode.edittree->type == NTREE_GEOMETRY)
  {
    return get_node_editor_root_compute_context(snode, compute_context_cache);
  }
  const ComputeContext *root_context = get_node_editor_root_compute_context(snode,
                                                                            compute_context_cache);
  if (!root_context) {
    return nullptr;
  }
  const ComputeContext *edittree_context =
      compute_context_for_tree_path(snode, compute_context_cache, root_context).value_or(nullptr);
  return edittree_context;
}

/** Finds a node without the topology cache, this may run on depsgraph worker threads. */
static const bNode *find_node_by_identifier(const bNodeTree &tree, const int32_t identifier)
{
  for (const bNode &node : tree.nodes) {
    if (node.identifier == identifier) {
      return &node;
    }
  }
  return nullptr;
}

const ComputeContext *compute_context_for_recursive_inspection_path(
    const bNodeTree &tree,
    const Span<RecursiveInspectionStep> path,
    bke::ComputeContextCache &compute_context_cache,
    const ComputeContext *base)
{
  if (path.is_empty()) {
    return nullptr;
  }
  const bNodeTree *current_tree = &tree;
  const ComputeContext *context = base;
  for (const RecursiveInspectionStep &step : path) {
    switch (step.type) {
      case RecursiveInspectionStep::Type::GroupNode: {
        const bNode *node = find_node_by_identifier(*current_tree, step.node_id);
        if (node == nullptr || !node->is_group() || node->id == nullptr ||
            GS(node->id->name) != ID_NT)
        {
          return nullptr;
        }
        context = &compute_context_cache.for_group_node(context, step.node_id, current_tree);
        current_tree = reinterpret_cast<const bNodeTree *>(node->id);
        break;
      }
      case RecursiveInspectionStep::Type::SimulationZone:
        context = &compute_context_cache.for_simulation_zone(context, step.node_id);
        break;
      case RecursiveInspectionStep::Type::RepeatZone:
        context = &compute_context_cache.for_repeat_zone(context, step.node_id, step.index);
        break;
      case RecursiveInspectionStep::Type::ForeachZone:
        context = &compute_context_cache.for_foreach_geometry_element_zone(
            context, step.node_id, step.index);
        break;
    }
  }
  return context;
}

[[nodiscard]] const ComputeContext *compute_context_for_edittree(
    const SpaceNode &snode, bke::ComputeContextCache &compute_context_cache)
{
  const ComputeContext *base = compute_context_for_edittree_base(snode, compute_context_cache);
  if (base == nullptr || snode.runtime == nullptr || snode.edittree == nullptr) {
    return base;
  }
  if (snode.edittree->type != NTREE_GEOMETRY ||
      (snode.edittree->flag & NTREE_GEOMETRY_RECURSIVE) == 0)
  {
    return base;
  }
  if (!snode.runtime->recursive_inspection_hash.has_value()) {
    return base;
  }
  if (snode.runtime->recursive_inspection_tree_uid != snode.edittree->id.session_uid) {
    return base;
  }
  if (*snode.runtime->recursive_inspection_hash == base->hash()) {
    return base;
  }
  if (const ComputeContext *context = compute_context_for_recursive_inspection_path(
          *snode.edittree, snode.runtime->recursive_inspection_path, compute_context_cache, base))
  {
    if (context->hash() == *snode.runtime->recursive_inspection_hash) {
      return context;
    }
  }
  /* Values can still be looked up by hash, but viewer paths cannot be built from this. */
  return &compute_context_cache.for_any_uncached<RecursiveInspectionComputeContext>(
      base, *snode.runtime->recursive_inspection_hash);
}

const ComputeContext *compute_context_for_edittree_socket(
    const SpaceNode &snode,
    bke::ComputeContextCache &compute_context_cache,
    const bNodeSocket &socket)
{
  const ComputeContext *context = compute_context_for_edittree(snode, compute_context_cache);
  if (!context) {
    return nullptr;
  }
  const bke::bNodeTreeZones *zones = snode.edittree->zones();
  if (!zones) {
    return nullptr;
  }
  const bke::bNodeTreeZone *zone = zones->get_zone_by_socket(socket);
  const Vector<const bke::bNodeTreeZone *> zone_stack = zones->get_zones_to_enter_from_root(zone);
  return compute_context_for_zones(zone_stack, compute_context_cache, context);
}

const ComputeContext *compute_context_for_edittree_node(
    const SpaceNode &snode, bke::ComputeContextCache &compute_context_cache, const bNode &node)
{
  const ComputeContext *context = compute_context_for_edittree(snode, compute_context_cache);
  if (!context) {
    return nullptr;
  }
  const bke::bNodeTreeZones *zones = snode.edittree->zones();
  if (!zones) {
    return nullptr;
  }
  const bke::bNodeTreeZone *zone = zones->get_zone_by_node(node.identifier);
  const Vector<const bke::bNodeTreeZone *> zone_stack = zones->get_zones_to_enter_from_root(zone);
  return compute_context_for_zones(zone_stack, compute_context_cache, context);
}

/* ******************** default callbacks for node space ***************** */

static SpaceLink *node_create(const ScrArea * /*area*/, const Scene * /*scene*/)
{
  SpaceNode *snode = MEM_new<SpaceNode>(__func__);
  snode->runtime = MEM_new<SpaceNode_Runtime>(__func__);
  snode->spacetype = SPACE_NODE;

  snode->flag = SNODE_SHOW_GPENCIL | SNODE_USE_ALPHA;
  snode->overlay.flag = (SN_OVERLAY_SHOW_OVERLAYS | SN_OVERLAY_SHOW_WIRE_COLORS |
                         SN_OVERLAY_SHOW_PATH | SN_OVERLAY_SHOW_PREVIEWS |
                         SN_OVERLAY_SHOW_RENDER_REGION | SN_OVERLAY_SHOW_ORIGINAL_LABELS);
  snode->overlay.link_style = SN_OVERLAY_LINK_BEZIER;

  /* backdrop */
  snode->zoom = 1.0f;
  snode->overlay.passepartout_alpha = 0.5f;
  snode->overlay.coordinate_alpha = 0.5f;
  /* Image Process texture resolution default. */
  snode->image_resolution[0] = 1024;
  snode->image_resolution[1] = 1024;

  /* Inherit last used tree type / Image Process group when opening a new node editor. */
  if (g_last_node_tree_idname[0] && bke::node_tree_type_find(g_last_node_tree_idname)) {
    STRNCPY_UTF8(snode->tree_idname, g_last_node_tree_idname);
  }
  else {
    /* select the first tree type for valid type */
    for (const bke::bNodeTreeType *treetype : bke::node_tree_types_get()) {
      STRNCPY_UTF8(snode->tree_idname, treetype->idname.c_str());
      break;
    }
  }
  if (STREQ(snode->tree_idname, "ImageNodeTree")) {
    snode->flag |= SNODE_BACKDRAW;
  }

  /* header */
  ARegion *region = BKE_area_region_new();

  BLI_addtail(&snode->regionbase, region);
  region->regiontype = RGN_TYPE_HEADER;
  region->alignment = (U.uiflag & USER_HEADER_BOTTOM) ? RGN_ALIGN_BOTTOM : RGN_ALIGN_TOP;

  /* asset shelf */
  region = BKE_area_region_new();

  BLI_addtail(&snode->regionbase, region);
  region->regiontype = RGN_TYPE_ASSET_SHELF;
  region->alignment = RGN_ALIGN_BOTTOM;
  region->flag |= RGN_FLAG_HIDDEN;

  /* asset shelf header */
  region = BKE_area_region_new();

  BLI_addtail(&snode->regionbase, region);
  region->regiontype = RGN_TYPE_ASSET_SHELF_HEADER;
  region->alignment = RGN_ALIGN_BOTTOM | RGN_ALIGN_HIDE_WITH_PREV;

  /* buttons/list view */
  region = BKE_area_region_new();

  BLI_addtail(&snode->regionbase, region);
  region->regiontype = RGN_TYPE_UI;
  region->alignment = RGN_ALIGN_RIGHT;

  /* toolbar */
  region = BKE_area_region_new();

  BLI_addtail(&snode->regionbase, region);
  region->regiontype = RGN_TYPE_TOOLS;
  region->alignment = RGN_ALIGN_LEFT;

  region->flag = RGN_FLAG_HIDDEN;

  /* main region */
  region = BKE_area_region_new();

  BLI_addtail(&snode->regionbase, region);
  region->regiontype = RGN_TYPE_WINDOW;

  region->v2d.tot.xmin = -12.8f * U.widget_unit;
  region->v2d.tot.ymin = -12.8f * U.widget_unit;
  region->v2d.tot.xmax = 38.4f * U.widget_unit;
  region->v2d.tot.ymax = 38.4f * U.widget_unit;

  region->v2d.cur = region->v2d.tot;

  region->v2d.min[0] = 1.0f;
  region->v2d.min[1] = 1.0f;

  region->v2d.max[0] = 32000.0f;
  region->v2d.max[1] = 32000.0f;

  region->v2d.minzoom = 0.05f;
  region->v2d.maxzoom = 2.31f;

  region->v2d.scroll = (V2D_SCROLL_RIGHT | V2D_SCROLL_BOTTOM);
  region->v2d.keepzoom = V2D_LIMITZOOM | V2D_KEEPASPECT;
  region->v2d.keeptot = eView2D_KeepTot{};

  return reinterpret_cast<SpaceLink *>(snode);
}

static void node_free(SpaceLink *sl)
{
  SpaceNode *snode = reinterpret_cast<SpaceNode *>(sl);
  snode->treepath.free_no_destruct();
  MEM_delete(snode->runtime);
}

/* spacetype; init callback */
static void node_init(wmWindowManager * /*wm*/, ScrArea * /*area*/) {}

static void node_exit(wmWindowManager *wm, ScrArea *area)
{
  SpaceNode *snode = area->spacedata.first_as<SpaceNode>();
  free_previews(*wm, *snode);
}

static bool any_node_uses_id(const bNodeTree *ntree, const ID *id)
{
  if (ELEM(nullptr, ntree, id)) {
    return false;
  }
  for (const bNode *node : ntree->all_nodes()) {
    if (node->id == id) {
      return true;
    }
  }
  return false;
}

/**
 * Tag the space to recalculate the current tree.
 *
 * This will do `snode_set_context()` which takes care of setting an active tree. This will be done
 * in the area refresh callback.
 *
 * IMPORTANT: for Image Process / GPU Texture Editor this must NOT set #image_eval_dirty.
 * Generic notifiers (NC_SCENE, NC_SPACE, compositor ND_NODES with nullptr, material updates from
 * Image Output, etc.) used to thrash dirty every frame while the editor was open — one multipass
 * cook freezes the whole app, while closing the editor (no cook) looked "realtime" with the last
 * Image Output. Real graph edits call #node_area_tag_image_process_dirty instead.
 */
static void node_area_tag_tree_recalc(SpaceNode *snode, ScrArea *area)
{
  ED_area_tag_refresh(area);
}

/** Mark GPU Texture Editor tree dirty and schedule a recook on next area refresh. */
static void node_area_tag_image_process_dirty(SpaceNode *snode, ScrArea *area)
{
  if (snode && snode->runtime) {
    snode->runtime->image_eval_dirty = true;
  }
  ED_area_tag_refresh(area);
}

static bool image_node_is_time_dependent_type(const bNode &node)
{
  return node.is_type("GeometryNodeSimulationInput"_ustr) ||
         node.is_type("GeometryNodeSimulationOutput"_ustr) ||
         node.is_type("ImageNodeFluidSimInput"_ustr) ||
         node.is_type("ImageNodeFluidSimOutput"_ustr) ||
         node.is_type("ImageNodeSceneTime"_ustr) ||
         node.is_type("ImageNodeSceneFrame"_ustr) || /* alias of Scene Time */
         node.is_type("ImageNodeShaderToy"_ustr) ||  /* iTime from Scene Time */
         node.is_type("ImageNodeCameraView"_ustr) || /* per-frame camera offscreen */
         node.is_type("GeometryNodeInputSceneTime"_ustr) ||
         node.is_type("CompositorNodeSceneTime"_ustr) ||
         node.is_type("CompositorNodeTime"_ustr) ||
         node.is_type("CompositorNodeMovieClip"_ustr) ||
         node.is_type("CompositorNodeTrackPos"_ustr) ||
         node.is_type("CompositorNodeStabilize"_ustr) ||
         node.is_type("CompositorNodePlaneTrackDeform"_ustr) ||
         node.is_type("CompositorNodeMovieDistortion"_ustr) ||
         node.is_type("CompositorNodeKeyingScreen"_ustr);
}

/** True when walking backward from `terminal` hits a time-dependent node (or animated tree). */
static bool image_terminal_demand_needs_frame(const bNodeTree &ntree, const bNode &terminal)
{
  ntree.ensure_topology_cache();
  if (BKE_animdata_id_is_animated(&ntree.id)) {
    return true;
  }

  Stack<const bNode *> stack;
  Set<const bNode *> visited;
  stack.push(&terminal);

  while (!stack.is_empty()) {
    const bNode *node = stack.pop();
    if (!visited.add(node) || node->is_muted()) {
      continue;
    }

    if (image_node_is_time_dependent_type(*node)) {
      return true;
    }

    if (node->is_group() && node->id) {
      /* Group body treated as opaque time dependency if the group tree is animated or has
       * time nodes anywhere — conservative for nested demos. */
      const bNodeTree &group = *reinterpret_cast<const bNodeTree *>(node->id);
      if (BKE_animdata_id_is_animated(&group.id)) {
        return true;
      }
      for (const bNode *gn : group.all_nodes()) {
        if (!gn->is_muted() && image_node_is_time_dependent_type(*gn)) {
          return true;
        }
      }
    }

    for (const bNodeSocket *input : node->input_sockets()) {
      if (!input->is_available()) {
        continue;
      }
      for (const bNodeSocket *from : input->logically_linked_sockets()) {
        stack.push(&from->owner_node());
      }
    }
  }

  return false;
}

static bool image_node_is_viewer_type(const bNode &node)
{
  return node.is_type("ImageNodeViewer"_ustr) || node.is_type("CompositorNodeViewer"_ustr);
}

static bool image_node_is_file_output_type(const bNode &node)
{
  return node.is_type("ImageNodeFileOutput"_ustr) ||
         node.is_type("CompositorNodeOutputFile"_ustr);
}

static bool image_node_is_demand_terminal(const bNode &node)
{
  if (node.is_muted()) {
    return false;
  }
  if (!(image_node_is_viewer_type(node) || image_node_is_file_output_type(node) ||
        node.is_type("ImageNodeBakeImage"_ustr)))
  {
    return false;
  }
  /* Active Viewer only (same as scheduler). */
  if (image_node_is_viewer_type(node) && (node.flag & NODE_DO_OUTPUT) == 0) {
    return false;
  }
  return true;
}

/** True when walking backward from `terminal` hits any of `dirty_ids`. */
static bool image_terminal_demand_contains_dirty(const bNodeTree &ntree,
                                                const bNode &terminal,
                                                const Span<int32_t> dirty_ids)
{
  if (dirty_ids.is_empty()) {
    return false;
  }
  Set<int32_t> dirty;
  dirty.reserve(dirty_ids.size());
  for (const int32_t id : dirty_ids) {
    dirty.add(id);
  }
  if (dirty.contains(terminal.identifier)) {
    return true;
  }

  ntree.ensure_topology_cache();
  Stack<const bNode *> stack;
  Set<const bNode *> visited;
  stack.push(&terminal);

  while (!stack.is_empty()) {
    const bNode *node = stack.pop();
    if (!visited.add(node)) {
      continue;
    }
    /* Walk through muted relays so mute / unmute still demand-reaches terminals. */
    if (dirty.contains(node->identifier)) {
      return true;
    }
    for (const bNodeSocket *input : node->input_sockets()) {
      /* Prefer logical links even when the input is usage-hidden (mode menus): a dirty
       * upstream must still invalidate the terminal that reads it when available. */
      for (const bNodeSocket *from : input->logically_linked_sockets()) {
        stack.push(&from->owner_node());
      }
    }
    /* Zone borders / stale topology cache: also walk DNA links. */
    for (const bNodeLink &link : ntree.links) {
      if (link.tonode == node && link.fromnode) {
        stack.push(link.fromnode);
      }
    }
  }
  return false;
}

/**
 * Forward search: from dirty nodes, walk output links to find demand terminals.
 * This matches the user expectation "only re-evaluate nodes linked forward from the edit".
 */
static Vector<int> image_tree_dirty_cook_terminals_forward(const bNodeTree &ntree,
                                                          const Span<int32_t> dirty_ids)
{
  Vector<int> ids;
  if (dirty_ids.is_empty()) {
    return ids;
  }

  ntree.ensure_topology_cache();
  Set<int32_t> dirty;
  dirty.reserve(dirty_ids.size());
  for (const int32_t id : dirty_ids) {
    dirty.add(id);
  }

  Stack<const bNode *> stack;
  Set<const bNode *> visited;
  for (const bNode *node : ntree.all_nodes()) {
    if (dirty.contains(node->identifier)) {
      stack.push(node);
    }
  }

  VectorSet<int> terminal_ids;
  while (!stack.is_empty()) {
    const bNode *node = stack.pop();
    if (!visited.add(node)) {
      continue;
    }
    if (image_node_is_demand_terminal(*node)) {
      terminal_ids.add(node->identifier);
    }
    /* Always walk outputs even when muted: mute changes what flows to
     * downstream terminals (M toggle must recook Viewer / Image Output). */
    for (const bNodeSocket *output : node->output_sockets()) {
      for (const bNodeSocket *to : output->logically_linked_sockets()) {
        stack.push(&to->owner_node());
      }
    }
    for (const bNodeLink &link : ntree.links) {
      if (link.fromnode == node && link.tonode) {
        stack.push(link.tonode);
      }
    }
  }

  for (const int id : terminal_ids) {
    ids.append(id);
  }
  return ids;
}

/**
 * Terminals (Viewer / Image Output) whose demand path needs the scene frame.
 * Used for partial frame cooks so Fluid preview does not re-evaluate independent modules.
 */
static Vector<int> image_tree_frame_cook_terminals(const bNodeTree &ntree)
{
  ntree.ensure_topology_cache();
  Vector<int> ids;
  for (const bNode *node : ntree.all_nodes()) {
    if (!image_node_is_demand_terminal(*node)) {
      continue;
    }
    if (image_terminal_demand_needs_frame(ntree, *node)) {
      ids.append(node->identifier);
    }
  }
  return ids;
}

/**
 * Terminals whose demand graph includes any of the dirty node identifiers.
 * Used so editing one independent Image Output branch does not re-run Repeat/Sim multipass
 * on another branch. Combines forward (from dirty) and backward (from terminals) search.
 */
static Vector<int> image_tree_dirty_cook_terminals(const bNodeTree &ntree,
                                                  const Span<int32_t> dirty_ids)
{
  /* Prefer forward isolation from the edited node(s). */
  Vector<int> ids = image_tree_dirty_cook_terminals_forward(ntree, dirty_ids);
  if (!ids.is_empty()) {
    return ids;
  }

  /* Backward fallback (same graph, different walk). */
  ntree.ensure_topology_cache();
  for (const bNode *node : ntree.all_nodes()) {
    if (!image_node_is_demand_terminal(*node)) {
      continue;
    }
    if (image_terminal_demand_contains_dirty(ntree, *node, dirty_ids)) {
      ids.append(node->identifier);
    }
  }
  return ids;
}

/** Active Viewer terminals only — never schedules independent Image Output / Repeat. */
static Vector<int> image_tree_active_viewer_terminals(const bNodeTree &ntree)
{
  ntree.ensure_topology_cache();
  Vector<int> ids;
  for (const bNode *node : ntree.all_nodes()) {
    if (!image_node_is_viewer_type(*node) || node->is_muted()) {
      continue;
    }
    if ((node->flag & NODE_DO_OUTPUT) == 0) {
      continue;
    }
    ids.append(node->identifier);
  }
  return ids;
}

/** Ensure File Outputs that demand-reach dirty nodes are scheduled (dedupe). */
static void image_tree_merge_dirty_file_outputs(const bNodeTree &ntree,
                                               const Span<int32_t> dirty_ids,
                                               Vector<int> &cook_terminals)
{
  VectorSet<int> set;
  for (const int id : cook_terminals) {
    set.add(id);
  }
  ntree.ensure_topology_cache();
  for (const bNode *node : ntree.all_nodes()) {
    if (node->is_muted()) {
      continue;
    }
    if (!image_node_is_file_output_type(*node)) {
      continue;
    }
    if (set.contains(node->identifier)) {
      continue;
    }
    /* Dirty Image Output itself must recook even with no remaining input links
     * (disconnect should clear the Image datablock, not keep the last pixels). */
    bool include = false;
    for (const int32_t id : dirty_ids) {
      if (id == node->identifier) {
        include = true;
        break;
      }
    }
    if (!include) {
      include = dirty_ids.is_empty() ||
                image_terminal_demand_contains_dirty(ntree, *node, dirty_ids);
    }
    if (include) {
      set.add(node->identifier);
    }
  }
  cook_terminals.clear();
  for (const int id : set) {
    cook_terminals.append(id);
  }
}

/** True when any demand terminal needs the scene frame. */
static bool image_tree_demand_needs_frame(const bNodeTree &ntree)
{
  return !image_tree_frame_cook_terminals(ntree).is_empty() ||
         BKE_animdata_id_is_animated(&ntree.id);
}

/** True when a notifier reference is this Image Process root / edit / selected group tree. */
static bool image_process_notifier_targets_tree(const SpaceNode *snode, const void *reference)
{
  if (!snode || !reference) {
    return false;
  }
  const ID *ref = static_cast<const ID *>(reference);
  if (GS(ref->name) != ID_NT) {
    return false;
  }
  if (snode->nodetree && ref == &snode->nodetree->id) {
    return true;
  }
  if (snode->edittree && ref == &snode->edittree->id) {
    return true;
  }
  if (snode->selected_node_group && ref == &snode->selected_node_group->id) {
    return true;
  }
  return false;
}

static void node_area_listener(const wmSpaceTypeListenerParams *params)
{
  ScrArea *area = params->area;
  const wmNotifier *wmn = params->notifier;

  /* NOTE: #ED_area_tag_refresh will re-execute compositor. */
  SpaceNode *snode = area->spacedata.first_as<SpaceNode>();
  /* shaderfrom is only used for new shading nodes, otherwise all shaders are from objects */
  short shader_type = snode->shaderfrom;

  /* preview renders */
  switch (wmn->category) {
    case NC_SCENE:
      switch (wmn->data) {
        case ND_NODES: {
          /* Image Process: only when the notifier targets our tree (not compositor nullptr). */
          if (ED_node_is_image(snode)) {
            if (image_process_notifier_targets_tree(snode, wmn->reference)) {
              node_area_tag_image_process_dirty(snode, area);
            }
          }
          else {
            node_area_tag_tree_recalc(snode, area);
          }
          break;
        }
        case ND_FRAME:
          /* Image Process: refresh only — node_area_refresh does a *partial* frame cook for
           * terminals whose demand path needs time (Fluid/Sim), not a full-tree dirty. */
          if (ED_node_is_image(snode)) {
            if (snode->nodetree && image_tree_demand_needs_frame(*snode->nodetree)) {
              ED_area_tag_refresh(area);
            }
          }
          else {
            node_area_tag_tree_recalc(snode, area);
          }
          break;
        case ND_COMPO_RESULT: {
          ED_area_tag_redraw(area);
          /* Backdrop image offset is calculated during compositing so gizmos need to be updated
           * afterwards. */
          const ARegion *region = BKE_area_find_region_type(area, RGN_TYPE_WINDOW);
          WM_gizmomap_tag_refresh(region->runtime->gizmo_map);
          break;
        }
      }
      break;

    /* future: add ID checks? */
    case NC_MATERIAL:
      if (ED_node_is_shader(snode)) {
        if (wmn->data == ND_SHADING) {
          node_area_tag_tree_recalc(snode, area);
        }
        else if (wmn->data == ND_SHADING_DRAW) {
          node_area_tag_tree_recalc(snode, area);
        }
        else if (wmn->data == ND_SHADING_LINKS) {
          node_area_tag_tree_recalc(snode, area);
        }
      }
      break;
    case NC_TEXTURE:
      if (ED_node_is_shader(snode) || ED_node_is_texture(snode)) {
        if (wmn->data == ND_NODES) {
          node_area_tag_tree_recalc(snode, area);
        }
      }
      break;
    case NC_WORLD:
      if (ED_node_is_shader(snode) && shader_type == SNODE_SHADER_WORLD) {
        node_area_tag_tree_recalc(snode, area);
      }
      break;
    case NC_OBJECT:
      if (ED_node_is_shader(snode)) {
        if (wmn->data == ND_OB_SHADING) {
          node_area_tag_tree_recalc(snode, area);
        }
      }
      else if (ED_node_is_geometry(snode) || ED_node_is_object(snode)) {
        if (wmn->data == ND_MODIFIER) {
          /* Rather strict check: only redraw when the reference matches current editor's ID, */
          if (ED_node_is_geometry(snode) &&
              (wmn->reference == snode->id || snode->id == nullptr))
          {
            node_area_tag_tree_recalc(snode, area);
          }
          /* Redraw context path or modifier dependent information. */
          ED_area_tag_redraw(area);
        }
      }
      break;
    case NC_SPACE:
      if (wmn->data == ND_SPACE_NODE) {
        /* Space settings / path: refresh context, but do not force Image Process recook. */
        if (ED_node_is_image(snode)) {
          ED_area_tag_refresh(area);
          ED_area_tag_redraw(area);
        }
        else {
          node_area_tag_tree_recalc(snode, area);
        }
      }
      else if (wmn->data == ND_SPACE_NODE_VIEW) {
        ED_area_tag_redraw(area);
      }
      break;
    case NC_NODE:
      if (wmn->action == NA_EDITED) {
        if (ED_node_is_image(snode)) {
          /* Match the edited node-tree id only. snode->id is the Scene for Image Process — matching
           * it (or nullptr) re-dirtied on every unrelated NC_NODE and recooked multipass forever
           * while the editor stayed open. Socket/node RNA updates pass the tree id via
           * BKE_main_ensure_invariants. */
          if (image_process_notifier_targets_tree(snode, wmn->reference)) {
            node_area_tag_image_process_dirty(snode, area);
          }
          else {
            ED_area_tag_redraw(area);
          }
        }
        else if (ELEM(wmn->reference, snode->nodetree, snode->id, nullptr) ||
                 snode->id == nullptr)
        {
          node_area_tag_tree_recalc(snode, area);
        }
      }
      else if (wmn->action == NA_SELECTED) {
        ED_area_tag_redraw(area);
      }
      break;
    case NC_SCREEN:
      switch (wmn->data) {
        case ND_ANIMPLAY:
        case ND_ANIMATION_PLAYBACK:
          /* Same as ND_FRAME: partial frame cook via refresh, not full dirty. */
          if (ED_node_is_image(snode)) {
            if (snode->nodetree && image_tree_demand_needs_frame(*snode->nodetree)) {
              ED_area_tag_refresh(area);
            }
          }
          else {
            node_area_tag_tree_recalc(snode, area);
          }
          break;
      }
      break;
    case NC_MASK:
      if (wmn->action == NA_EDITED) {
        if (snode->nodetree && snode->nodetree->type == NTREE_COMPOSIT) {
          node_area_tag_tree_recalc(snode, area);
        }
      }
      break;

    case NC_IMAGE:
      if (wmn->action == NA_EDITED) {
        if (ED_node_is_compositor(snode)) {
          /* Without this check drawing on an image could become very slow when the compositor is
           * open. */
          if (any_node_uses_id(snode->nodetree, static_cast<ID *>(wmn->reference))) {
            node_area_tag_tree_recalc(snode, area);
          }
        }
        else if (ED_node_is_image(snode)) {
          /* Image Output writes during cook send NC_IMAGE — redraw only, never recook (that
           * was a feedback loop: cook → write → dirty → cook…). */
          ED_area_tag_redraw(area);
        }
        else {
          ED_area_tag_redraw(area);
        }
      }
      break;

    case NC_MOVIECLIP:
      if (wmn->action == NA_EDITED) {
        if (ED_node_is_compositor(snode)) {
          if (any_node_uses_id(snode->nodetree, static_cast<ID *>(wmn->reference))) {
            node_area_tag_tree_recalc(snode, area);
          }
        }
      }
      break;

    case NC_LINESTYLE:
      if (ED_node_is_shader(snode) && shader_type == SNODE_SHADER_LINESTYLE) {
        node_area_tag_tree_recalc(snode, area);
      }
      break;
    case NC_WM:
      if (wmn->data == ND_UNDO) {
        if (ED_node_is_image(snode)) {
          node_area_tag_image_process_dirty(snode, area);
        }
        else {
          node_area_tag_tree_recalc(snode, area);
        }
      }
      break;
    case NC_GPENCIL:
      if (ELEM(wmn->action, NA_EDITED, NA_SELECTED)) {
        ED_area_tag_redraw(area);
      }
      break;
  }
}

static bool image_tree_needs_evaluation(const bNodeTree &ntree)
{
  for (const bNode *node : ntree.all_nodes()) {
    if (node->is_type("ImageNodeViewer"_ustr) || node->is_type("ImageNodeBakeImage"_ustr) ||
        node->is_type("ImageNodeFileOutput"_ustr))
    {
      return true;
    }
  }
  return false;
}

static void node_area_refresh(const bContext *C, ScrArea *area)
{
  snode_set_context(*C);

  /* Image Process editor: evaluate so Viewer backdrop updates
   * (same role as the compositor job for CompositorNodeTree). Always run when the tree has
   * Viewer/Bake nodes — not only when backdrop is enabled. */
  SpaceNode *snode = CTX_wm_space_node(C);
  if (snode && ED_node_is_image(snode) && snode->nodetree && snode->runtime) {
    if ((snode->flag & SNODE_BACKDRAW) || image_tree_needs_evaluation(*snode->nodetree)) {
      Main *bmain = CTX_data_main(C);
      Scene *scene = CTX_data_scene(C);
      if (bmain && scene) {
        const int cfra = scene->r.cfra;
        const bool frame_changed = snode->runtime->image_eval_frame != cfra;
        const bool full_dirty = snode->runtime->image_eval_dirty;
        bNodeTree &ntree = *snode->nodetree;

        /* Demand isolation (Houdini-style):
         * - Viewer preview (Ctrl-Shift / activate Viewer) → active Viewer only.
         *   Image Output is never scheduled (pixels unchanged; GPU\to CPU is expensive).
         * - Property edit → only terminals that demand-reach the dirty node(s).
         *   Independent Repeat→Image Output is NOT scheduled.
         * - Link/topology change or first cook → all demand terminals (or partial when ids known).
         * - Frame-only → time-dependent terminals only. */
        Vector<int> cook_terminals;
        bool use_terminal_filter = false;

        if (full_dirty) {
          ImageProcessTreeDirtyState &dirty = image_process_tree_dirty_state(ntree.id.session_uid);
          /* Merge edit-tree dirty (nested group property edits). */
          if (snode->edittree && snode->edittree != &ntree) {
            ImageProcessTreeDirtyState &edit_dirty = image_process_tree_dirty_state(
                snode->edittree->id.session_uid);
            if (edit_dirty.topology_dirty) {
              dirty.topology_dirty = true;
            }
            if (edit_dirty.viewer_preview_only) {
              dirty.viewer_preview_only = true;
            }
            for (const int32_t id : edit_dirty.dirty_node_ids) {
              dirty.dirty_node_ids.add(id);
            }
          }

          const bool topology = dirty.topology_dirty;
          const bool first_cook = snode->runtime->image_eval_frame ==
                                  std::numeric_limits<int>::min();

          /* Drop ids of nodes that no longer exist (deleted). Partial cook from a ghost
           * id finds no terminals and used to fall back to Viewer-only, leaving Image
           * Output stale. Topology with no live ids → full cook. */
          VectorSet<int32_t> live_dirty_ids;
          {
            ntree.ensure_topology_cache();
            Set<int32_t> existing;
            existing.reserve(ntree.all_nodes().size());
            for (const bNode *node : ntree.all_nodes()) {
              existing.add(node->identifier);
            }
            for (const int32_t id : dirty.dirty_node_ids) {
              if (existing.contains(id)) {
                live_dirty_ids.add(id);
              }
            }
            /* Nested group property edits store ids from edittree. Map those to the
             * group node in the root tree so Image Output on the root still recooks. */
            if (snode->edittree && snode->edittree != &ntree) {
              for (const bNode *node : ntree.all_nodes()) {
                if (node->is_group() && node->id == &snode->edittree->id) {
                  live_dirty_ids.add(node->identifier);
                }
              }
            }
          }
          Span<int32_t> dirty_ids = live_dirty_ids.as_span();

          /* Ctrl-Shift preview / activate Viewer: only the active Viewer terminal.
           * Even when the previewed node also feeds Image Output, do not re-write Image
           * datablocks — that path downloads full textures and freezes the UI. */
          if (!first_cook && dirty.viewer_preview_only) {
            cook_terminals = image_tree_active_viewer_terminals(ntree);
            if (cook_terminals.is_empty()) {
              image_process_tree_dirty_state_clear(ntree.id.session_uid);
              if (snode->edittree) {
                image_process_tree_dirty_state_clear(snode->edittree->id.session_uid);
              }
              snode->runtime->image_eval_dirty = false;
              if (frame_changed) {
                snode->runtime->image_eval_frame = cfra;
              }
              return;
            }
            use_terminal_filter = true;
          }
          /* Known dirty nodes → partial cook of only demand-reaching terminals.
           * Applies to property edits AND structural edits on a known node (e.g. Rasterize
           * attribute socket rebuild). Never re-run an independent Repeat→Image Output
           * branch when the dirty node is only upstream of Viewer. */
          else {
            VectorSet<int32_t> fallback_ids;
            if (!first_cook && dirty_ids.is_empty() && !topology) {
              /* Tag path missed owner_node: fall back to active node only. */
              if (const bNode *active = bke::node_get_active(
                      snode->edittree ? *snode->edittree : ntree))
              {
                fallback_ids.add(active->identifier);
                dirty_ids = fallback_ids.as_span();
              }
            }
            if (!first_cook && !dirty_ids.is_empty()) {
              cook_terminals = image_tree_dirty_cook_terminals(ntree, dirty_ids);
              /* Include remaining File Output terminals (e.g. Compositor Output File) that
               * demand the dirty nodes. Do not fall back to the active Viewer when the
               * dirty nodes are unused: Ctrl+G of an unconnected branch would recook. */
              image_tree_merge_dirty_file_outputs(ntree, dirty_ids, cook_terminals);
              if (cook_terminals.is_empty()) {
                /* No Viewer / Bake / File Output depends on the dirty nodes — nothing to show. */
                image_process_tree_dirty_state_clear(ntree.id.session_uid);
                if (snode->edittree) {
                  image_process_tree_dirty_state_clear(snode->edittree->id.session_uid);
                }
                snode->runtime->image_eval_dirty = false;
                if (frame_changed) {
                  snode->runtime->image_eval_frame = cfra;
                }
                return;
              }
              use_terminal_filter = true;
            }
            /* else: first cook, or topology with unknown scope (no dirty ids) → full cook. */
          }
        }
        else {
          if (!frame_changed) {
            return;
          }
          cook_terminals = image_tree_frame_cook_terminals(ntree);
          if (cook_terminals.is_empty()) {
            snode->runtime->image_eval_frame = cfra;
            return;
          }
          use_terminal_filter = true;
        }

        const int2 resolution = int2(math::max(1, snode->image_resolution[0]),
                                     math::max(1, snode->image_resolution[1]));
        bool used_gpu = false;

        if (use_terminal_filter) {
          compositor::image_process_set_terminal_filter(cook_terminals);
        }
        else {
          compositor::image_process_clear_terminal_filter();
        }

        ntreeImageNodesEvaluate(*bmain, *scene, ntree, &used_gpu, resolution);
        compositor::image_process_clear_terminal_filter();

        image_process_tree_dirty_state_clear(ntree.id.session_uid);
        if (snode->edittree && snode->edittree != &ntree) {
          image_process_tree_dirty_state_clear(snode->edittree->id.session_uid);
        }

        snode->runtime->image_eval_frame = cfra;
        /* ALWAYS clear dirty after a cook attempt (no multipass GPU-miss thrash). */
        snode->runtime->image_eval_dirty = false;
        snode->runtime->image_eval_gpu_misses = used_gpu ?
                                                    0 :
                                                    (snode->runtime->image_eval_gpu_misses + 1);
        /* Do not tag full area redraw or NC_IMAGE here — write_viewer already marks the
         * Viewer ImBuf dirty; extra notifiers cause a free/redraw flash loop. */
        ED_region_tag_redraw(BKE_area_find_region_type(area, RGN_TYPE_WINDOW));
      }
    }
  }
}

static SpaceLink *node_duplicate(SpaceLink *sl)
{
  SpaceNode *snode = reinterpret_cast<SpaceNode *>(sl);
  SpaceNode *snoden = MEM_dupalloc(snode);

  BLI_duplicatelist(&snoden->treepath, &snode->treepath);

  snoden->runtime = MEM_new<SpaceNode_Runtime>(__func__);

  /* NOTE: no need to set node tree user counts,
   * the editor only keeps at least 1 (id_us_ensure_real),
   * which is already done by the original SpaceNode.
   */

  return reinterpret_cast<SpaceLink *>(snoden);
}

/* add handlers, stuff you only do once or on area/region changes */
static void node_buttons_region_init(wmWindowManager *wm, ARegion *region)
{
  wmKeyMap *keymap;

  ED_region_panels_init(wm, region);

  keymap = WM_keymap_ensure(wm->runtime->defaultconf, "Node Generic", SPACE_NODE, RGN_TYPE_WINDOW);
  WM_event_add_keymap_handler(&region->runtime->handlers, keymap);
}

static void node_buttons_region_draw(const bContext *C, ARegion *region)
{
  ED_region_panels(C, region);
}

/* add handlers, stuff you only do once or on area/region changes */
static void node_toolbar_region_init(wmWindowManager *wm, ARegion *region)
{
  wmKeyMap *keymap;

  ED_region_panels_init(wm, region);

  keymap = WM_keymap_ensure(wm->runtime->defaultconf, "Node Generic", SPACE_NODE, RGN_TYPE_WINDOW);
  WM_event_add_keymap_handler(&region->runtime->handlers, keymap);
}

static void node_toolbar_region_draw(const bContext *C, ARegion *region)
{
  ED_region_panels(C, region);
}

static void node_cursor(wmWindow *win, ScrArea *area, ARegion *region)
{
  SpaceNode *snode = area->spacedata.first_as<SpaceNode>();

  /* convert mouse coordinates to v2d space */
  ui::view2d_region_to_view(&region->v2d,
                            win->runtime->eventstate->xy[0] - region->winrct.xmin,
                            win->runtime->eventstate->xy[1] - region->winrct.ymin,
                            &snode->runtime->cursor[0],
                            &snode->runtime->cursor[1]);

  /* here snode->runtime->cursor is used to detect the node edge for sizing */
  node_set_cursor(*win, *region, *snode, snode->runtime->cursor);

  /* XXX snode->runtime->cursor is in placing new nodes space */
  snode->runtime->cursor[0] /= UI_SCALE_FAC;
  snode->runtime->cursor[1] /= UI_SCALE_FAC;
}

static bool node_region_is_recursion_tree(const wmWindowManager *wm, const ARegion *region)
{
  if (wm == nullptr || region == nullptr) {
    return false;
  }
  for (const wmWindow &win : wm->windows) {
    const bScreen *screen = WM_window_get_active_screen(&win);
    if (screen == nullptr) {
      continue;
    }
    for (const ScrArea &area : screen->areabase) {
      if (area.spacetype != SPACE_NODE) {
        continue;
      }
      for (const ARegion &reg : area.regionbase) {
        if (&reg != region) {
          continue;
        }
        const SpaceNode *snode = area.spacedata.first_as<SpaceNode>();
        return snode->runtime && snode->runtime->recursion_tree_window;
      }
    }
  }
  return false;
}

/* Initialize main region, setting handlers. */
static void node_main_region_init(wmWindowManager *wm, ARegion *region)
{
  wmKeyMap *keymap;
  ListBaseT<wmDropBox> *lb;

  view2d_region_reinit(&region->v2d, ui::V2D_COMMONVIEW_CUSTOM, region->winx, region->winy);

  if (node_region_is_recursion_tree(wm, region)) {
    return;
  }

  /* Keep type-menu Shift+CLICK ahead of node.select (CLICK strips pass-through). */
  node_socket_type_menu_ensure_keymap_priority(wm);
  node_minimap_ensure_keymap_priority(wm);

  /* own keymaps */
  keymap = WM_keymap_ensure(wm->runtime->defaultconf, "Node Generic", SPACE_NODE, RGN_TYPE_WINDOW);
  WM_event_add_keymap_handler(&region->runtime->handlers, keymap);

  keymap = WM_keymap_ensure(wm->runtime->defaultconf, "Node Editor", SPACE_NODE, RGN_TYPE_WINDOW);
  WM_event_add_keymap_handler_v2d_mask(&region->runtime->handlers, keymap);

  WM_event_remove_ui_handler(
      &region->runtime->handlers, node_minimap_ui_handler, nullptr, nullptr, false);
  WM_event_add_ui_handler(nullptr,
                          &region->runtime->handlers,
                          node_minimap_ui_handler,
                          nullptr,
                          nullptr,
                          eWM_EventHandlerFlag(0));

  /* add drop boxes */
  lb = WM_dropboxmap_find("Node Editor", SPACE_NODE, RGN_TYPE_WINDOW);

  WM_event_add_dropbox_handler(&region->runtime->handlers, lb);

  /* The backdrop image gizmo needs to change together with the view. So always refresh gizmos on
   * region size changes. */
  WM_gizmomap_tag_refresh(region->runtime->gizmo_map);
}

static void node_main_region_draw(const bContext *C, ARegion *region)
{
  const SpaceNode *snode = CTX_wm_space_node(C);
  if (snode && snode->runtime && snode->runtime->recursion_tree_window) {
    node_recursive_tree_draw(*C, *region);
    return;
  }
  node_draw_space(*C, *region);
}

/* ************* dropboxes ************* */

static bool node_group_drop_poll(bContext *C, wmDrag *drag, const wmEvent * /*event*/)
{
  SpaceNode *snode = CTX_wm_space_node(C);

  if (snode == nullptr) {
    return false;
  }

  if (snode->edittree == nullptr) {
    return false;
  }

  if (!WM_drag_is_ID_type(drag, ID_NT)) {
    return false;
  }

  if (drag->type == WM_DRAG_ID) {
    const bNodeTree *node_tree = reinterpret_cast<const bNodeTree *>(
        WM_drag_get_local_ID(drag, ID_NT));
    if (!node_tree) {
      return false;
    }
    return node_tree->type == snode->edittree->type;
  }

  if (drag->type == WM_DRAG_ASSET) {
    const wmDragAsset *asset_data = WM_drag_get_asset_data(drag, ID_NT);
    if (!asset_data) {
      return false;
    }
    const AssetMetaData *metadata = &asset_data->asset->get_metadata();
    const IDProperty *tree_type = BKE_asset_metadata_idprop_find(metadata, "type");
    if (!tree_type || IDP_int_get(tree_type) != snode->edittree->type) {
      return false;
    }
  }

  return true;
}

static bool node_object_drop_poll(bContext *C, wmDrag *drag, const wmEvent * /*event*/)
{
  return WM_drag_is_ID_type(drag, ID_OB) && !ui::button_active_drop_name(C);
}

static bool node_collection_drop_poll(bContext *C, wmDrag *drag, const wmEvent * /*event*/)
{
  return WM_drag_is_ID_type(drag, ID_GR) && !ui::button_active_drop_name(C);
}

static bool node_id_im_drop_poll(bContext * /*C*/, wmDrag *drag, const wmEvent * /*event*/)
{
  return WM_drag_is_ID_type(drag, ID_IM);
}

static bool node_mask_drop_poll(bContext * /*C*/, wmDrag *drag, const wmEvent * /*event*/)
{
  return WM_drag_is_ID_type(drag, ID_MSK);
}

static bool node_material_drop_poll(bContext *C, wmDrag *drag, const wmEvent * /*event*/)
{
  return WM_drag_is_ID_type(drag, ID_MA) && !ui::button_active_drop_name(C);
}

static bool node_color_drop_poll(bContext *C, wmDrag *drag, const wmEvent * /*event*/)
{
  return (drag->type == WM_DRAG_COLOR) && !ui::button_active_drop_color(C);
}

static bool node_import_file_drop_poll(bContext *C, wmDrag *drag, const wmEvent * /*event*/)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode) {
    return false;
  }
  if (!snode->edittree) {
    return false;
  }
  if (!ELEM(snode->edittree->type, NTREE_GEOMETRY, NTREE_COMPOSIT)) {
    return false;
  }
  if (drag->type != WM_DRAG_PATH) {
    return false;
  }
  const bool is_geometry_tree = snode->edittree->type == NTREE_GEOMETRY;
  const Span<std::string> paths = WM_drag_get_paths(drag);
  for (const StringRef path : paths) {
    if (path.endswith(".txt")) {
      return true;
    }
    if (is_geometry_tree &&
        (path.endswith(".csv") || path.endswith(".obj") || path.endswith(".ply") ||
         path.endswith(".stl") || path.endswith(".vdb") || path.endswith(".spz")))
    {
      return true;
    }
  }
  return false;
}

static bool node_interface_item_drop_poll_base(bContext *C,
                                               wmDrag *drag,
                                               const wmEvent * /*event*/)
{
  if (drag->type != WM_DRAG_NODE_TREE_INTERFACE) {
    return false;
  }
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return false;
  }
  const bNodeTree *target_ntree = snode->edittree;

  auto *drag_data = static_cast<bke::node_interface::bNodeTreeInterfaceItemReference *>(
      drag->poin);

  /* Drag only onto node editors of the same node tree. */
  const bNodeTree *source_ntree = drag_data->tree;
  if (target_ntree != source_ntree) {
    return false;
  }

  return true;
}

static bool node_interface_item_drop_poll_single(bContext *C, wmDrag *drag, const wmEvent *event)
{
  return !(event->modifier & KM_CTRL) && node_interface_item_drop_poll_base(C, drag, event);
}

static bool node_interface_item_drop_poll_all(bContext *C, wmDrag *drag, const wmEvent *event)
{
  return event->modifier & KM_CTRL && node_interface_item_drop_poll_base(C, drag, event);
}

static void node_group_drop_copy(bContext *C, wmDrag *drag, wmDropBox *drop)
{
  ID *id = WM_drag_get_local_ID_or_import_from_asset(C, drag, 0);
  if (id) {
    RNA_int_set(drop->ptr, "session_uid", int(id->session_uid));
  }

  RNA_boolean_set(drop->ptr, "show_datablock_in_node", (drag->type != WM_DRAG_ASSET));
}

static void node_id_drop_copy(bContext *C, wmDrag *drag, wmDropBox *drop)
{
  ID *id = WM_drag_get_local_ID_or_import_from_asset(C, drag, 0);
  if (id) {
    RNA_int_set(drop->ptr, "session_uid", int(id->session_uid));
  }
}

static void node_id_im_drop_copy(bContext *C, wmDrag *drag, wmDropBox *drop)
{
  ID *id = WM_drag_get_local_ID_or_import_from_asset(C, drag, 0);
  if (id) {
    RNA_int_set(drop->ptr, "session_uid", int(id->session_uid));
    RNA_struct_property_unset(drop->ptr, "filepath");
    return;
  }
}

static void node_import_file_drop_copy(bContext * /*C*/, wmDrag *drag, wmDropBox *drop)
{
  io::paths_to_operator_properties(drop->ptr, WM_drag_get_paths(drag));
}

static void node_interface_item_drop_copy_single(bContext * /*C*/,
                                                 wmDrag * /*drag*/,
                                                 wmDropBox *drop)
{
  RNA_boolean_set(drop->ptr, "only_selected_sockets", true);
  RNA_boolean_set(drop->ptr, "all_panel_contents", false);
}

static void node_interface_item_drop_copy_all(bContext * /*C*/, wmDrag * /*drag*/, wmDropBox *drop)
{
  RNA_boolean_set(drop->ptr, "only_selected_sockets", true);
  RNA_boolean_set(drop->ptr, "all_panel_contents", true);
}

static std::string node_interface_item_drop_tooltip_single(bContext * /*C*/,
                                                           wmDrag *drag,
                                                           const int /*xy*/[2],
                                                           wmDropBox * /*drop*/)
{
  auto *drag_data = static_cast<bke::node_interface::bNodeTreeInterfaceItemReference *>(
      drag->poin);
  bool has_selected_header_toggle = false;

  for (int i = 0; i < drag_data->items_count; i++) {
    const bNodeTreeInterfaceItem *item = drag_data->items[i];
    if (const auto *panel = bke::node_interface::get_item_as<bNodeTreeInterfacePanel>(item)) {
      if (panel->header_toggle_socket()) {
        has_selected_header_toggle = true;
        break;
      }
    }
  }

  if (has_selected_header_toggle) {
    return TIP_("Add Group Input (hold Ctrl to include sockets under panel toggle)");
  }
  return TIP_("Add Group Input");
}

static std::string node_interface_item_drop_tooltip_all(bContext * /*C*/,
                                                        wmDrag * /*drag*/,
                                                        const int /*xy*/[2],
                                                        wmDropBox * /*drop*/)
{
  return TIP_("Add Group Input");
}

/* this region dropbox definition */
static void node_dropboxes()
{
  ListBaseT<wmDropBox> *lb = WM_dropboxmap_find("Node Editor", SPACE_NODE, RGN_TYPE_WINDOW);

  WM_dropbox_add(lb,
                 "NODE_OT_add_object",
                 node_object_drop_poll,
                 node_id_drop_copy,
                 WM_drag_free_imported_drag_ID,
                 nullptr);
  WM_dropbox_add(lb,
                 "NODE_OT_add_collection",
                 node_collection_drop_poll,
                 node_id_drop_copy,
                 WM_drag_free_imported_drag_ID,
                 nullptr);
  WM_dropbox_add(lb,
                 "NODE_OT_add_group",
                 node_group_drop_poll,
                 node_group_drop_copy,
                 WM_drag_free_imported_drag_ID,
                 nullptr);
  WM_dropbox_add(lb,
                 "NODE_OT_add_image",
                 node_id_im_drop_poll,
                 node_id_im_drop_copy,
                 WM_drag_free_imported_drag_ID,
                 nullptr);
  WM_dropbox_add(lb,
                 "NODE_OT_add_mask",
                 node_mask_drop_poll,
                 node_id_drop_copy,
                 WM_drag_free_imported_drag_ID,
                 nullptr);
  WM_dropbox_add(lb,
                 "NODE_OT_add_material",
                 node_material_drop_poll,
                 node_id_drop_copy,
                 WM_drag_free_imported_drag_ID,
                 nullptr);
  WM_dropbox_add(
      lb, "NODE_OT_add_color", node_color_drop_poll, ui::drop_color_copy, nullptr, nullptr);
  WM_dropbox_add(lb,
                 "NODE_OT_add_import_node",
                 node_import_file_drop_poll,
                 node_import_file_drop_copy,
                 nullptr,
                 nullptr);
  WM_dropbox_add(lb,
                 "NODE_OT_add_group_input_node",
                 node_interface_item_drop_poll_single,
                 node_interface_item_drop_copy_single,
                 nullptr,
                 node_interface_item_drop_tooltip_single);
  WM_dropbox_add(lb,
                 "NODE_OT_add_group_input_node",
                 node_interface_item_drop_poll_all,
                 node_interface_item_drop_copy_all,
                 nullptr,
                 node_interface_item_drop_tooltip_all);
}

/* ************* end drop *********** */

/* add handlers, stuff you only do once or on area/region changes */
static void node_header_region_init(wmWindowManager * /*wm*/, ARegion *region)
{
  ED_region_header_init(region);
}

static void node_header_region_draw(const bContext *C, ARegion *region)
{
  /* find and set the context */
  snode_set_context(*C);

  ED_region_header(C, region);
}

/* used for header + main region */
static void node_region_listener(const wmRegionListenerParams *params)
{
  ARegion *region = params->region;
  const wmNotifier *wmn = params->notifier;
  wmGizmoMap *gzmap = region->runtime->gizmo_map;

  /* context changes */
  switch (wmn->category) {
    case NC_SPACE:
      switch (wmn->data) {
        case ND_SPACE_NODE:
          ED_region_tag_redraw(region);
          break;
        case ND_SPACE_NODE_VIEW:
          WM_gizmomap_tag_refresh(gzmap);
          break;
      }
      break;
    case NC_ANIMATION:
      if (wmn->data == ND_NLA_ACTCHANGE) {
        ED_region_tag_redraw(region);
      }
      break;
    case NC_SCREEN:
      if (wmn->data == ND_LAYOUTSET || wmn->action == NA_EDITED) {
        WM_gizmomap_tag_refresh(gzmap);
      }
      switch (wmn->data) {
        case ND_ANIMPLAY:
        case ND_LAYER:
          ED_region_tag_redraw(region);
          break;
      }
      break;
    case NC_WM:
      if (wmn->data == ND_JOB) {
        ED_region_tag_redraw(region);
      }
      break;
    case NC_SCENE:
      ED_region_tag_redraw(region);
      if (wmn->data == ND_RENDER_RESULT) {
        WM_gizmomap_tag_refresh(gzmap);
      }
      break;
    case NC_NODE:
      ED_region_tag_redraw(region);
      if (ELEM(wmn->action, NA_EDITED, NA_SELECTED)) {
        WM_gizmomap_tag_refresh(gzmap);
      }
      break;
    case NC_MATERIAL:
    case NC_TEXTURE:
    case NC_WORLD:
    case NC_LINESTYLE:
      ED_region_tag_redraw(region);
      break;
    case NC_OBJECT:
      if (ELEM(wmn->data, ND_OB_SHADING, ND_TRANSFORM, ND_MODIFIER)) {
        ED_region_tag_redraw(region);
      }
      break;
    case NC_ID:
      if (ELEM(wmn->action, NA_RENAME, NA_EDITED)) {
        ED_region_tag_redraw(region);
      }
      break;
    case NC_GPENCIL:
      if (wmn->action == NA_EDITED) {
        ED_region_tag_redraw(region);
      }
      else if (wmn->data & ND_GPENCIL_EDITMODE) {
        ED_region_tag_redraw(region);
      }
      break;
    case NC_VIEWER_PATH:
      ED_region_tag_redraw(region);
      break;
  }
}

}  // namespace ed::space_node

/* Outside of blender namespace to avoid Python documentation build error with `ctypes`. */
extern "C" {
const char *node_context_dir[] = {
    "selected_nodes", "active_node", "light", "material", "world", nullptr};
};

namespace ed::space_node {

static int /*eContextResult*/ node_context(const bContext *C,
                                           const char *member,
                                           bContextDataResult *result)
{
  SpaceNode *snode = CTX_wm_space_node(C);

  if (CTX_data_dir(member)) {
    CTX_data_dir_set(result, node_context_dir);
    return CTX_RESULT_OK;
  }
  if (CTX_data_equals(member, "selected_nodes")) {
    if (snode->edittree) {
      for (bNode *node : snode->edittree->all_nodes()) {
        if (node->is_selected()) {
          PointerRNA ptr = RNA_pointer_create_id_subdata(snode->edittree->id, RNA_Node, node);
          CTX_data_list_add_ptr(result, &ptr);
        }
      }
    }
    CTX_data_type_set(result, ContextDataType::Collection);
    return CTX_RESULT_OK;
  }
  if (CTX_data_equals(member, "active_node")) {
    if (snode->edittree) {
      bNode *node = bke::node_get_active(*snode->edittree);
      PointerRNA ptr = RNA_pointer_create_id_subdata(snode->edittree->id, RNA_Node, node);
      CTX_data_pointer_set_ptr(result, &ptr);
    }

    CTX_data_type_set(result, ContextDataType::Pointer);
    return CTX_RESULT_OK;
  }
  if (CTX_data_equals(member, "material")) {
    if (snode->id && GS(snode->id->name) == ID_MA) {
      CTX_data_id_pointer_set(result, snode->id);
    }
    return CTX_RESULT_OK;
  }
  if (CTX_data_equals(member, "light")) {
    if (snode->id && GS(snode->id->name) == ID_LA) {
      CTX_data_id_pointer_set(result, snode->id);
    }
    return CTX_RESULT_OK;
  }
  if (CTX_data_equals(member, "world")) {
    if (snode->id && GS(snode->id->name) == ID_WO) {
      CTX_data_id_pointer_set(result, snode->id);
    }
    return CTX_RESULT_OK;
  }
  if (CTX_data_equals(member, "edit_image")) {
    if (snode->edittree != nullptr) {
      if (bNode *node = bke::node_get_active(*snode->edittree)) {
        if (ELEM(node->type_legacy, SH_NODE_TEX_IMAGE, SH_NODE_TEX_ENVIRONMENT)) {
          Image *image = id_cast<Image *>(node->id);
          if (!image && node->type_legacy == SH_NODE_TEX_IMAGE) {
            if (const bNodeSocket *sock = bke::node_find_socket(*node, SOCK_IN, "Image"_ustr)) {
              if (const auto *val = sock->default_value_typed<bNodeSocketValueImage>()) {
                image = val->value;
              }
            }
          }
          if (image) {
            CTX_data_id_pointer_set(result, &image->id);
          }
          return CTX_RESULT_OK;
        }
      }
    }
  }
  return CTX_RESULT_MEMBER_NOT_FOUND;
}

static void node_widgets()
{
  /* Create the widget-map for the area here. */
  wmGizmoMapType_Params params{SPACE_NODE, RGN_TYPE_WINDOW};
  wmGizmoMapType *gzmap_type = WM_gizmomaptype_ensure(&params);
  WM_gizmogrouptype_append_and_link(gzmap_type, NODE_GGT_backdrop_transform);
  WM_gizmogrouptype_append_and_link(gzmap_type, NODE_GGT_backdrop_crop);
  WM_gizmogrouptype_append_and_link(gzmap_type, NODE_GGT_backdrop_glare);
  WM_gizmogrouptype_append_and_link(gzmap_type, NODE_GGT_backdrop_corner_pin);
  WM_gizmogrouptype_append_and_link(gzmap_type, NODE_GGT_backdrop_box_mask);
  WM_gizmogrouptype_append_and_link(gzmap_type, NODE_GGT_backdrop_ellipse_mask);
  WM_gizmogrouptype_append_and_link(gzmap_type, NODE_GGT_backdrop_split);
  WM_gizmogrouptype_append_and_link(gzmap_type, NODE_GGT_compositor_translate);
}

static void node_id_remap(ID *old_id, ID *new_id, SpaceNode *snode)
{
  if (snode->id == old_id) {
    /* nasty DNA logic for SpaceNode:
     * ideally should be handled by editor code, but would be bad level call
     */
    snode->treepath.free_no_destruct();

    /* XXX Untested in case new_id != nullptr... */
    snode->id = new_id;
    snode->from = nullptr;
    snode->nodetree = nullptr;
    snode->edittree = nullptr;
  }
  else if (GS(old_id->name) == ID_OB) {
    if (snode->from == old_id) {
      if (new_id == nullptr) {
        snode->flag &= ~SNODE_PIN;
      }
      snode->from = new_id;
    }
  }
  else if (GS(old_id->name) == ID_GD_LEGACY) {
    if (id_cast<ID *>(snode->gpd) == old_id) {
      snode->gpd = id_cast<bGPdata *>(new_id);
      id_us_min(old_id);
      id_us_plus(new_id);
    }
  }
  else if (GS(old_id->name) == ID_NT) {

    if (snode->selected_node_group) {
      if (&snode->selected_node_group->id == old_id) {
        snode->selected_node_group = reinterpret_cast<bNodeTree *>(new_id);
      }
    }

    bNodeTreePath *path, *path_next;

    for (path = snode->treepath.first(); path; path = path->next) {
      if (id_cast<ID *>(path->nodetree) == old_id) {
        path->nodetree = id_cast<bNodeTree *>(new_id);
        id_us_ensure_real(new_id);
      }
      if (path == snode->treepath.first_) {
        /* first nodetree in path is same as snode->nodetree */
        snode->nodetree = path->nodetree;
      }
      if (path->nodetree == nullptr) {
        break;
      }
    }

    /* remaining path entries are invalid, remove */
    for (; path; path = path_next) {
      path_next = path->next;

      BLI_remlink(&snode->treepath, path);
      MEM_delete(path);
    }

    /* edittree is just the last in the path,
     * set this directly since the path may have been shortened above */
    if (snode->treepath.last()) {
      path = snode->treepath.last();
      snode->edittree = path->nodetree;
      ED_node_set_active_viewer_key(snode);
    }
    else {
      snode->edittree = nullptr;
    }
  }
}

static void node_id_remap(ScrArea * /*area*/,
                          SpaceLink *slink,
                          const bke::id::IDRemapper &mappings)
{
  /* Although we should be able to perform all the mappings in a single go this lead to issues when
   * running the python test cases. Somehow the nodetree/edittree weren't updated to the new
   * pointers that generated a SEGFAULT.
   *
   * To move forward we should perhaps remove snode->edittree and snode->nodetree as they are just
   * copies of pointers. All usages should be calling a function that will receive the appropriate
   * instance.
   *
   * We could also move a remap address at a time to use the IDRemapper as that should get closer
   * to cleaner code. See {D13615} for more information about this topic.
   */
  mappings.iter([&](ID *old_id, ID *new_id) {
    node_id_remap(old_id, new_id, reinterpret_cast<SpaceNode *>(slink));
  });
}

static void node_foreach_id(SpaceLink *space_link, LibraryForeachIDData *data)
{
  SpaceNode *snode = reinterpret_cast<SpaceNode *>(space_link);
  const int data_flags = BKE_lib_query_foreachid_process_flags_get(data);
  const bool is_readonly = (data_flags & IDWALK_READONLY) != 0;
  const bool allow_pointer_access = (data_flags & IDWALK_NO_ORIG_POINTERS_ACCESS) == 0;
  bool is_embedded_nodetree = snode->id != nullptr && allow_pointer_access &&
                              bke::node_tree_from_id(snode->id) == snode->nodetree;

  BKE_LIB_FOREACHID_PROCESS_ID(data, snode->id, IDWALK_CB_DIRECT_WEAK_LINK);
  BKE_LIB_FOREACHID_PROCESS_ID(data, snode->from, IDWALK_CB_DIRECT_WEAK_LINK);

  bNodeTreePath *path = snode->treepath.first();
  BLI_assert(path == nullptr || path->nodetree == snode->nodetree);

  if (is_embedded_nodetree) {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, snode->nodetree, IDWALK_CB_EMBEDDED_NOT_OWNING);
    if (path != nullptr) {
      BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, path->nodetree, IDWALK_CB_EMBEDDED_NOT_OWNING);
    }

    /* Embedded ID pointers are not remapped (besides exceptions), ensure it still matches
     * actual data. Note that `snode->id` was already processed (and therefore potentially
     * remapped) above. */
    if (!is_readonly) {
      snode->nodetree = (snode->id == nullptr) ? nullptr : bke::node_tree_from_id(snode->id);
      if (path != nullptr) {
        path->nodetree = snode->nodetree;
      }
    }
  }
  else {
    BKE_LIB_FOREACHID_PROCESS_IDSUPER(
        data, snode->nodetree, IDWALK_CB_USER_ONE | IDWALK_CB_DIRECT_WEAK_LINK);
    if (path != nullptr) {
      BKE_LIB_FOREACHID_PROCESS_IDSUPER(
          data, path->nodetree, IDWALK_CB_USER_ONE | IDWALK_CB_DIRECT_WEAK_LINK);
    }
  }

  BKE_LIB_FOREACHID_PROCESS_IDSUPER(
      data, snode->selected_node_group, IDWALK_CB_USER_ONE | IDWALK_CB_DIRECT_WEAK_LINK);

  /* Both `snode->id` and `snode->nodetree` have been remapped now, so their data can be
   * accessed. */
  BLI_assert(snode->id == nullptr || snode->nodetree == nullptr ||
             (snode->nodetree->id.flag & ID_FLAG_EMBEDDED_DATA) == 0 ||
             snode->nodetree == bke::node_tree_from_id(snode->id));

  /* This is mainly here for readfile case ('lib_link' process), as in such case there is no access
   * to original data allowed, so no way to know whether the SpaceNode nodetree pointer is an
   * embedded one or not. */
  if (!is_readonly && snode->id && !snode->nodetree) {
    /* LuxCore materials store an independent tree on Material.luxcore_nodetree.
     * node_tree_from_id(material) is the Cycles shader tree — stuffing that into
     * this editor crashes as soon as the user clicks the material ID. */
    if (!STREQ(snode->tree_idname, "LuxCoreMaterialNodeTree")) {
      is_embedded_nodetree = true;
      snode->nodetree = bke::node_tree_from_id(snode->id);
      if (path != nullptr) {
        path->nodetree = snode->nodetree;
      }
    }
  }

  if (path != nullptr) {
    for (path = path->next; path != nullptr; path = path->next) {
      BLI_assert(path->nodetree != nullptr);
      if (allow_pointer_access) {
        BLI_assert((path->nodetree->id.flag & ID_FLAG_EMBEDDED_DATA) == 0);
      }

      BKE_LIB_FOREACHID_PROCESS_IDSUPER(
          data, path->nodetree, IDWALK_CB_USER_ONE | IDWALK_CB_DIRECT_WEAK_LINK);

      if (path->nodetree == nullptr) {
        BLI_assert(!is_readonly);
        /* Remaining path entries are invalid, remove them. */
        for (bNodeTreePath *path_next; path; path = path_next) {
          path_next = path->next;
          BLI_remlink(&snode->treepath, path);
          MEM_delete(path);
        }
        break;
      }
    }
  }
  BLI_assert(path == nullptr);

  if (!is_readonly) {
    /* `edittree` is just the last in the path, set this directly since the path may have
     * been shortened above. */
    if (snode->treepath.last() != nullptr) {
      path = snode->treepath.last();
      snode->edittree = path->nodetree;
    }
    else {
      snode->edittree = nullptr;
    }
  }
  else {
    /* Only process this pointer in readonly case, otherwise could lead to a bad
     * double-remapping e.g. */
    if (is_embedded_nodetree && snode->edittree == snode->nodetree) {
      BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, snode->edittree, IDWALK_CB_EMBEDDED_NOT_OWNING);
    }
    else {
      BKE_LIB_FOREACHID_PROCESS_IDSUPER(data, snode->edittree, IDWALK_CB_DIRECT_WEAK_LINK);
    }
  }
}

static int node_space_subtype_get(ScrArea *area)
{
  SpaceNode *snode = area->spacedata.first_as<SpaceNode>();
  return rna_node_tree_idname_to_enum(snode->tree_idname);
}

static void node_space_subtype_set(ScrArea *area, int value)
{
  SpaceNode *snode = area->spacedata.first_as<SpaceNode>();
  ED_node_set_tree_type(snode, rna_node_tree_type_from_enum(value));
}

static void node_space_subtype_item_extend(bContext *C, EnumPropertyItem **item, int *totitem)
{
  bool free;
  const EnumPropertyItem *item_src = RNA_enum_node_tree_types_itemf_impl(C, &free);
  RNA_enum_items_add(item, totitem, item_src);
  if (free) {
    MEM_delete(item_src);
  }
}

static StringRefNull node_space_name_get(const ScrArea *area)
{
  SpaceNode *snode = area->spacedata.first_as<SpaceNode>();
  bke::bNodeTreeType *tree_type = bke::node_tree_type_find(snode->tree_idname);
  if (tree_type == nullptr) {
    return IFACE_("Node Editor");
  }
  return tree_type->ui_name;
}

static int node_space_icon_get(const ScrArea *area)
{
  SpaceNode *snode = area->spacedata.first_as<SpaceNode>();
  bke::bNodeTreeType *tree_type = bke::node_tree_type_find(snode->tree_idname);
  if (tree_type == nullptr) {
    return ICON_NODETREE;
  }
  return tree_type->ui_icon;
}

static void node_space_blend_read_data(BlendDataReader *reader, SpaceLink *sl)
{
  SpaceNode *snode = reinterpret_cast<SpaceNode *>(sl);

  if (snode->gpd) {
    BLO_read_struct(reader, bGPdata, &snode->gpd);
    BKE_annotations_blend_read_data(reader, snode->gpd);
  }

  BLO_read_struct_list(reader, bNodeTreePath, &snode->treepath);
  snode->edittree = nullptr;
  snode->runtime = MEM_new<SpaceNode_Runtime>(__func__);
}

static void node_space_blend_write(BlendWriter *writer, SpaceLink *sl)
{
  SpaceNode *snode = reinterpret_cast<SpaceNode *>(sl);
  writer->write_struct_cast<SpaceNode>(snode, [](BlendStructWriter<SpaceNode> &struct_writer) {
    struct_writer.shallow_data.runtime = nullptr;
  });

  for (bNodeTreePath &path : snode->treepath) {
    writer->write_struct(&path);
  }
}

static void node_asset_shelf_region_init(wmWindowManager *wm, ARegion *region)
{
  using namespace blender::ed;
  wmKeyMap *keymap = WM_keymap_ensure(
      wm->runtime->defaultconf, "Node Generic", SPACE_NODE, RGN_TYPE_WINDOW);
  WM_event_add_keymap_handler(&region->runtime->handlers, keymap);

  asset::shelf::region_init(wm, region);
}

/**
 * Persistent offscreen + GPUViewport for Camera View.
 * Reuse avoids create/destroy every cook; still uses proven ImBuf float readback (stable color).
 *
 * IMPORTANT: Do NOT free GPU resources in a static destructor — on Blender exit the GPU/DRW
 * context is already torn down and free() crashes. Leak at process exit is fine.
 */
struct ImageProcessCameraViewCache {
  GPUOffScreen *ofs = nullptr;
  GPUViewport *viewport = nullptr;
  int width = 0;
  int height = 0;

  void free_gpu_runtime()
  {
    /* Only call while a GPU context is active (resize path). */
    if (viewport) {
      GPU_viewport_free(viewport);
      viewport = nullptr;
    }
    if (ofs) {
      GPU_offscreen_free(ofs);
      ofs = nullptr;
    }
    width = height = 0;
  }

  bool ensure(const int w, const int h, char err_out[256])
  {
    if (ofs && viewport && width == w && height == h) {
      return true;
    }
    free_gpu_runtime();
    /* Match ImBuf float path / Full precision Color (avoids half↔float reinterpret noise). */
    ofs = GPU_offscreen_create(w,
                               h,
                               true,
                               gpu::TextureFormat::SFLOAT_32_32_32_32,
                               GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_HOST_READ,
                               false,
                               err_out);
    if (!ofs) {
      return false;
    }
    viewport = GPU_viewport_create();
    if (!viewport) {
      GPU_offscreen_free(ofs);
      ofs = nullptr;
      if (err_out) {
        BLI_strncpy(err_out, "Camera View: failed to create GPU viewport", 256);
      }
      return false;
    }
    width = w;
    height = h;
    return true;
  }
};

static ImageProcessCameraViewCache &image_process_camera_view_cache()
{
  static ImageProcessCameraViewCache cache;
  return cache;
}

/** Cached wrapper around #ED_view3d_draw_offscreen_imbuf_simple (same pixels, reused GPU objs). */
static ImBuf *image_process_camera_view_draw_cached(Depsgraph *depsgraph,
                                                    Scene *scene,
                                                    View3DShading *shading_override,
                                                    eDrawType drawtype,
                                                    Object *camera,
                                                    int width,
                                                    int height,
                                                    ImBufFlags imbuf_flags,
                                                    eV3DOffscreenDrawFlag draw_flags,
                                                    int alpha_mode,
                                                    const char *viewname,
                                                    GPUOffScreen * /*ofs_unused*/,
                                                    GPUViewport * /*viewport_unused*/,
                                                    char err_out[256])
{
  ImageProcessCameraViewCache &cache = image_process_camera_view_cache();
  if (!cache.ensure(width, height, err_out)) {
    return nullptr;
  }
  return ED_view3d_draw_offscreen_imbuf_simple(depsgraph,
                                               scene,
                                               shading_override,
                                               drawtype,
                                               camera,
                                               width,
                                               height,
                                               imbuf_flags,
                                               draw_flags,
                                               alpha_mode,
                                               viewname,
                                               cache.ofs,
                                               cache.viewport,
                                               err_out);
}

/**
 * Isolated UV plane + ortho camera for Render Material.
 *
 * Material is copied into this Main (embedded nodetree + Color→Surface wrapped as Emission).
 * GPU offscreen is created per cook (nullptr ofs/viewport) so LookdevWorld is destroyed while
 * the ImBuf cache is still alive — do not persist GPUViewport across WM_exit.
 */
struct ImageProcessRenderMaterialScratch {
  Main *bmain = nullptr;
  Scene *scene = nullptr;
  ViewLayer *view_layer = nullptr;
  Object *plane = nullptr;
  Object *camera = nullptr;
  Object *sun = nullptr;
  Mesh *mesh = nullptr;
  World *world = nullptr;
  Material *mat_copy = nullptr;
  Material *assigned_src = nullptr;
  Depsgraph *depsgraph = nullptr;
  Render *render = nullptr;
  uint64_t shader_key = 0;
  int cache_w = 0;
  int cache_h = 0;
  Vector<float> cache_pixels;
};

static ImageProcessRenderMaterialScratch &image_process_render_material_scratch()
{
  static ImageProcessRenderMaterialScratch scratch;
  return scratch;
}

static uint64_t hash_mix(uint64_t h, uint64_t v)
{
  h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
  return h;
}

static uint64_t hash_float(uint64_t h, const float v)
{
  uint32_t bits = 0;
  memcpy(&bits, &v, sizeof(bits));
  return hash_mix(h, bits);
}

static uint64_t hash_material_shader(const Material *ma, const int cfra)
{
  uint64_t h = uint64_t(ma->id.session_uid);
  h = hash_mix(h, uint64_t(uint32_t(cfra)));
  h = hash_mix(h, uint64_t(ma->use_nodes));
  h = hash_float(h, ma->r);
  h = hash_float(h, ma->g);
  h = hash_float(h, ma->b);
  h = hash_float(h, ma->roughness);
  h = hash_float(h, ma->metallic);
  if (!ma->nodetree) {
    return h;
  }
  bNodeTree &ntree = *ma->nodetree;
  ntree.ensure_topology_cache();
  h = hash_mix(h, uint64_t(ntree.all_nodes().size()));
  h = hash_mix(h, uint64_t(ntree.all_links().size()));
  for (const bNode *node : ntree.all_nodes()) {
    h = hash_mix(h, uint64_t(node->type_legacy));
    h = hash_mix(h, uint64_t(uint32_t(node->custom1)));
    h = hash_mix(h, uint64_t(uint32_t(node->custom2)));
    if (node->id) {
      h = hash_mix(h, uint64_t(node->id->session_uid));
    }
    for (const bNodeSocket *sock : node->input_sockets()) {
      if (!sock->default_value) {
        continue;
      }
      switch (eNodeSocketDatatype(sock->type)) {
        case SOCK_FLOAT:
          h = hash_float(h, sock->default_value_typed<bNodeSocketValueFloat>()->value);
          break;
        case SOCK_INT:
          h = hash_mix(h, uint64_t(uint32_t(sock->default_value_typed<bNodeSocketValueInt>()->value)));
          break;
        case SOCK_BOOLEAN:
          h = hash_mix(h, uint64_t(sock->default_value_typed<bNodeSocketValueBoolean>()->value));
          break;
        case SOCK_VECTOR: {
          const float *v = sock->default_value_typed<bNodeSocketValueVector>()->value;
          h = hash_float(h, v[0]);
          h = hash_float(h, v[1]);
          h = hash_float(h, v[2]);
          break;
        }
        case SOCK_RGBA: {
          const float *v = sock->default_value_typed<bNodeSocketValueRGBA>()->value;
          h = hash_float(h, v[0]);
          h = hash_float(h, v[1]);
          h = hash_float(h, v[2]);
          h = hash_float(h, v[3]);
          break;
        }
        default:
          break;
      }
    }
  }
  return h;
}

static void wrap_non_shader_surface(Main *bmain, Material *mat)
{
  bNodeTree *ntree = mat->nodetree;
  if (!ntree || !mat->use_nodes) {
    return;
  }
  ntree->ensure_topology_cache();
  Vector<bNodeLink *> to_wrap;
  for (bNodeLink *link : ntree->all_links()) {
    if (link->tosock && link->tosock->type == SOCK_SHADER && link->fromsock &&
        link->fromsock->type != SOCK_SHADER && link->tonode &&
        link->tonode->type_legacy == SH_NODE_OUTPUT_MATERIAL)
    {
      to_wrap.append(link);
    }
  }
  if (to_wrap.is_empty()) {
    return;
  }
  for (bNodeLink *link : to_wrap) {
    bNode *fromnode = link->fromnode;
    bNodeSocket *fromsock = link->fromsock;
    bNode *tonode = link->tonode;
    bNodeSocket *tosock = link->tosock;
    bke::node_remove_link(ntree, *link);
    bNode *emission = bke::node_add_static_node(nullptr, *ntree, SH_NODE_EMISSION);
    bNodeSocket *color_in = bke::node_find_socket(*emission, SOCK_IN, "Color"_ustr);
    bNodeSocket *str_in = bke::node_find_socket(*emission, SOCK_IN, "Strength"_ustr);
    bNodeSocket *em_out = bke::node_find_socket(*emission, SOCK_OUT, "Emission"_ustr);
    if (str_in) {
      str_in->default_value_typed<bNodeSocketValueFloat>()->value = 1.0f;
    }
    if (fromnode && fromsock && color_in && em_out && tonode && tosock) {
      bke::node_add_link(*ntree, *fromnode, *fromsock, *emission, *color_in);
      bke::node_add_link(*ntree, *emission, *em_out, *tonode, *tosock);
    }
  }
  BKE_ntree_update_after_single_tree_change(*bmain, *ntree);
}

static void replace_material_nodetree(Main *bmain, Material *dst, const Material *src)
{
  dst->use_nodes = src->use_nodes;
  dst->r = src->r;
  dst->g = src->g;
  dst->b = src->b;
  dst->a = src->a;
  dst->roughness = src->roughness;
  dst->metallic = src->metallic;
  dst->spec = src->spec;
  if (dst->nodetree) {
    bke::node_tree_free_embedded_tree(dst->nodetree);
    MEM_delete(dst->nodetree);
    dst->nodetree = nullptr;
  }
  if (src->nodetree) {
    dst->nodetree = bke::node_tree_copy_tree_ex(*src->nodetree, nullptr, false);
    dst->nodetree->owner_id = &dst->id;
  }
  wrap_non_shader_surface(bmain, dst);
}

static Mesh *render_material_make_uv_plane_mesh(Main *bmain)
{
  Mesh *mesh_dst = BKE_mesh_add(bmain, "Plane");
  /* 2x2 verts, size 2: a single quad on XY in [-1, 1] with UV 0-1. */
  Mesh *mesh_src = geometry::create_grid_mesh(2, 2, 2.0f, 2.0f, "UVMap");
  BKE_mesh_nomain_to_mesh(mesh_src, mesh_dst, nullptr);
  mesh_dst->uv_maps_active_set("UVMap");
  mesh_dst->uv_maps_default_set("UVMap");
  return mesh_dst;
}

static void render_material_sync_object_matrix(Object *ob)
{
  if (!ob || !ob->runtime) {
    return;
  }
  float mat[4][4];
  BKE_object_to_mat4(ob, mat);
  copy_m4_m4(ob->runtime->object_to_world.ptr(), mat);
  invert_m4_m4(ob->runtime->world_to_object.ptr(), mat);
}

static bool render_material_ensure_scene(ImageProcessRenderMaterialScratch &s)
{
  if (s.bmain && s.scene && s.view_layer && s.plane && s.camera && s.depsgraph) {
    return true;
  }

  s.bmain = BKE_main_new();
  s.scene = BKE_scene_add(s.bmain, "RenderMaterial");
  s.view_layer = static_cast<ViewLayer *>(s.scene->view_layers.first());
  if (!s.view_layer) {
    return false;
  }

  STRNCPY_UTF8(s.scene->r.engine, RE_engine_id_BLENDER_EEVEE);
  s.scene->r.alphamode = R_ALPHAPREMUL;
  s.scene->display.shading.type = OB_MATERIAL;

  s.world = BKE_world_add(s.bmain, "World");
  s.world->horr = s.world->horg = s.world->horb = 0.0f;
  s.world->sun_threshold = 0.0f;
  s.world->flag &= ~WO_USE_SUN_SHADOW;
  if (s.world->nodetree) {
    bNodeTree &ntree = *s.world->nodetree;
    bNode *background = bke::node_add_static_node(nullptr, ntree, SH_NODE_BACKGROUND);
    bNode *output = bke::node_add_static_node(nullptr, ntree, SH_NODE_OUTPUT_WORLD);
    if (background && output) {
      bNodeSocket *bg_out = bke::node_find_socket(*background, SOCK_OUT, "Background"_ustr);
      bNodeSocket *surf_in = bke::node_find_socket(*output, SOCK_IN, "Surface"_ustr);
      bNodeSocket *col_in = bke::node_find_socket(*background, SOCK_IN, "Color"_ustr);
      bNodeSocket *str_in = bke::node_find_socket(*background, SOCK_IN, "Strength"_ustr);
      if (bg_out && surf_in) {
        bke::node_add_link(ntree, *background, *bg_out, *output, *surf_in);
      }
      if (col_in) {
        float *col = col_in->default_value_typed<bNodeSocketValueRGBA>()->value;
        col[0] = col[1] = col[2] = 0.0f;
        col[3] = 1.0f;
      }
      if (str_in) {
        str_in->default_value_typed<bNodeSocketValueFloat>()->value = 0.0f;
      }
    }
    BKE_ntree_update_after_single_tree_change(*s.bmain, ntree);
  }
  s.scene->world = s.world;

  s.mesh = render_material_make_uv_plane_mesh(s.bmain);
  s.plane = BKE_object_add_for_data(
      s.bmain, s.scene, s.view_layer, OB_MESH, "Plane", &s.mesh->id, true);
  s.plane->color[0] = s.plane->color[1] = s.plane->color[2] = s.plane->color[3] = 1.0f;
  s.plane->loc[0] = s.plane->loc[1] = s.plane->loc[2] = 0.0f;
  s.plane->rot[0] = s.plane->rot[1] = s.plane->rot[2] = 0.0f;
  s.plane->scale[0] = s.plane->scale[1] = s.plane->scale[2] = 1.0f;
  s.plane->rotmode = ROT_MODE_EUL;

  s.camera = BKE_object_add(s.bmain, s.scene, s.view_layer, OB_CAMERA, "Camera");
  Camera *cam = id_cast<Camera *>(s.camera->data);
  s.camera->loc[0] = 0.0f;
  s.camera->loc[1] = 0.0f;
  s.camera->loc[2] = 1.0f;
  s.camera->rot[0] = s.camera->rot[1] = s.camera->rot[2] = 0.0f;
  s.camera->rotmode = ROT_MODE_EUL;
  cam->type = CAM_ORTHO;
  cam->ortho_scale = 2.0f;
  cam->sensor_fit = CAMERA_SENSOR_FIT_AUTO;
  cam->clip_start = 0.01f;
  cam->clip_end = 100.0f;
  cam->flag &= ~CAM_SHOWPASSEPARTOUT;
  s.scene->camera = s.camera;

  s.sun = BKE_object_add(s.bmain, s.scene, s.view_layer, OB_LAMP, "Sun");
  Light *la = id_cast<Light *>(s.sun->data);
  la->type = LA_SUN;
  la->energy = 4.0f;
  la->mode &= ~LA_SHADOW;
  s.sun->loc[0] = 0.0f;
  s.sun->loc[1] = 0.0f;
  s.sun->loc[2] = 5.0f;
  s.sun->rot[0] = s.sun->rot[1] = s.sun->rot[2] = 0.0f;
  s.sun->rotmode = ROT_MODE_EUL;

  BKE_view_layer_synced_ensure(*s.bmain, s.scene, s.view_layer);
  for (Base &base : *BKE_view_layer_object_bases_get(s.view_layer)) {
    base.flag |= BASE_ENABLED_AND_MAYBE_VISIBLE_IN_VIEWPORT | BASE_ENABLED_VIEWPORT |
                 BASE_ENABLED_RENDER;
  }
  s.depsgraph = DEG_graph_new(s.bmain, s.scene, s.view_layer, DAG_EVAL_VIEWPORT);
  DEG_graph_build_from_view_layer(s.depsgraph);
  BKE_scene_graph_update_tagged(s.depsgraph, s.bmain);
  return true;
}

static bool render_material_sync(ImageProcessRenderMaterialScratch &s,
                                 Material *src,
                                 const uint64_t shader_key)
{
  const bool new_id = (s.assigned_src != src) || (s.mat_copy == nullptr);
  if (new_id) {
    if (s.mesh && s.mesh->mat && s.mesh->totcol > 0) {
      s.mesh->mat[0] = nullptr;
    }
    if (s.mat_copy) {
      BKE_id_free(s.bmain, s.mat_copy);
      s.mat_copy = nullptr;
    }
    s.mat_copy = id_cast<Material *>(BKE_id_copy_ex(
        s.bmain, &src->id, nullptr, LIB_ID_CREATE_LOCAL | LIB_ID_COPY_NO_ANIMDATA));
    if (!s.mat_copy) {
      s.assigned_src = nullptr;
      return false;
    }
    wrap_non_shader_surface(s.bmain, s.mat_copy);
    BKE_object_material_assign_single_obdata(s.bmain, s.plane, s.mat_copy, 1);
    s.plane->actcol = 1;
    s.assigned_src = src;
    DEG_graph_tag_relations_update(s.depsgraph);
  }
  else if (s.shader_key != shader_key) {
    replace_material_nodetree(s.bmain, s.mat_copy, src);
    DEG_id_tag_update(&s.mat_copy->id, ID_RECALC_SHADING);
  }
  s.shader_key = shader_key;
  return s.mat_copy != nullptr;
}

/** Engine preview of the UV plane. Not the 3D Viewport lookdev (that showed studio HDRI). */
static ImBuf *render_material_engine_preview(ImageProcessRenderMaterialScratch &s,
                                             const int width,
                                             const int height,
                                             char err_out[256])
{
  render_material_sync_object_matrix(s.plane);
  render_material_sync_object_matrix(s.camera);
  render_material_sync_object_matrix(s.sun);

  s.scene->r.xsch = width;
  s.scene->r.ysch = height;
  s.scene->r.size = 100;
  s.scene->r.alphamode = R_ALPHAPREMUL;
  s.scene->r.scemode |= R_BUTS_PREVIEW;
  s.scene->r.scemode &= ~(R_MATNODE_PREVIEW | R_TEXNODE_PREVIEW | R_NO_IMAGE_LOAD);
  STRNCPY_UTF8(s.scene->r.engine, RE_engine_id_BLENDER_EEVEE);
  s.scene->display.render_aa = SCE_DISPLAY_AA_SAMPLES_8;

  if (!s.render) {
    s.render = RE_NewRender(&s);
  }
  RE_ClearResult(s.render);
  RE_PreviewRender(s.render, s.bmain, s.scene);

  RenderResult rr;
  RE_AcquireResultImage(s.render, &rr, 0);
  ImBuf *ibuf = nullptr;
  if (rr.ibuf) {
    ibuf = IMB_dupImBuf(rr.ibuf);
  }
  RE_ReleaseResultImage(s.render);

  if (ibuf && !ibuf->float_buffer.data && ibuf->byte_buffer.data) {
    IMB_float_from_byte(ibuf);
  }
  if (!ibuf || !ibuf->float_buffer.data) {
    if (ibuf) {
      IMB_freeImBuf(ibuf);
    }
    if (err_out) {
      BLI_strncpy(err_out, "Render Material: engine preview produced no image", 256);
    }
    return nullptr;
  }
  return ibuf;
}

static ImBuf *image_process_render_material_draw(Main * /*user_bmain*/,
                                                 Scene *user_scene,
                                                 Material *material,
                                                 int width,
                                                 int height,
                                                 char err_out[256])
{
  if (!material) {
    if (err_out) {
      BLI_strncpy(err_out, "Render Material: no material", 256);
    }
    return nullptr;
  }
  width = math::max(width, 1);
  height = math::max(height, 1);

  ImageProcessRenderMaterialScratch &s = image_process_render_material_scratch();
  if (!render_material_ensure_scene(s)) {
    if (err_out) {
      BLI_strncpy(err_out, "Render Material: failed to create preview scene", 256);
    }
    return nullptr;
  }

  const int cfra = user_scene ? user_scene->r.cfra : 0;
  if (user_scene) {
    s.scene->r.cfra = user_scene->r.cfra;
  }
  s.scene->r.alphamode = R_ALPHAPREMUL;
  s.scene->display.shading.type = OB_MATERIAL;

  const uint64_t shader_key = hash_material_shader(material, cfra);
  if (s.shader_key == shader_key && s.cache_w == width && s.cache_h == height &&
      !s.cache_pixels.is_empty())
  {
    ImBuf *cached = IMB_allocImBuf(uint(width), uint(height), ImBufFlags::FloatData);
    if (cached && cached->float_data_for_write()) {
      memcpy(cached->float_data_for_write(),
             s.cache_pixels.data(),
             size_t(width) * size_t(height) * 4 * sizeof(float));
      return cached;
    }
    if (cached) {
      IMB_freeImBuf(cached);
    }
  }

  s.plane->loc[0] = s.plane->loc[1] = s.plane->loc[2] = 0.0f;
  s.plane->rot[0] = s.plane->rot[1] = s.plane->rot[2] = 0.0f;
  s.plane->scale[0] = s.plane->scale[1] = s.plane->scale[2] = 1.0f;
  s.camera->loc[0] = 0.0f;
  s.camera->loc[1] = 0.0f;
  s.camera->loc[2] = 1.0f;
  s.camera->rot[0] = s.camera->rot[1] = s.camera->rot[2] = 0.0f;
  Camera *cam = id_cast<Camera *>(s.camera->data);
  cam->type = CAM_ORTHO;
  cam->ortho_scale = 2.0f;

  if (!render_material_sync(s, material, shader_key)) {
    if (err_out) {
      BLI_strncpy(err_out, "Render Material: failed to copy material", 256);
    }
    return nullptr;
  }

  DEG_id_tag_update(&s.plane->id, ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY | ID_RECALC_SHADING);
  DEG_id_tag_update(&s.camera->id, ID_RECALC_TRANSFORM | ID_RECALC_PARAMETERS);
  if (s.sun) {
    DEG_id_tag_update(&s.sun->id, ID_RECALC_TRANSFORM);
  }
  if (s.mat_copy) {
    DEG_id_tag_update(&s.mat_copy->id, ID_RECALC_SHADING);
  }
  BKE_scene_graph_update_tagged(s.depsgraph, s.bmain);

  ImBuf *ibuf = render_material_engine_preview(s, width, height, err_out);
  if (ibuf && ibuf->float_buffer.data) {
    const int channels = (ibuf->channels > 0) ? ibuf->channels : 4;
    if (ibuf->x == width && ibuf->y == height && channels == 4) {
      s.cache_w = width;
      s.cache_h = height;
      s.cache_pixels.reinitialize(size_t(width) * size_t(height) * 4);
      memcpy(s.cache_pixels.data(),
             ibuf->float_buffer.data,
             s.cache_pixels.size() * sizeof(float));
    }
  }
  return ibuf;
}

static void render_material_scratch_free()
{
  ImageProcessRenderMaterialScratch &s = image_process_render_material_scratch();
  s.cache_pixels.clear();
  s.cache_w = s.cache_h = 0;
  s.shader_key = 0;
  if (s.render) {
    RE_FreeRender(s.render);
    s.render = nullptr;
  }
  if (s.mesh && s.mesh->mat && s.mesh->totcol > 0) {
    s.mesh->mat[0] = nullptr;
  }
  s.mat_copy = nullptr;
  s.assigned_src = nullptr;
  if (s.depsgraph) {
    DEG_graph_free(s.depsgraph);
    s.depsgraph = nullptr;
  }
  if (s.bmain) {
    BKE_main_free(s.bmain);
    s.bmain = nullptr;
  }
  s.scene = nullptr;
  s.view_layer = nullptr;
  s.plane = nullptr;
  s.camera = nullptr;
  s.sun = nullptr;
  s.world = nullptr;
  s.mesh = nullptr;
}

}  // namespace ed::space_node

void ED_spacetype_node()
{
  using namespace blender::ed;
  using namespace blender::ed::space_node;

  /* Image Process Camera View: same viewport offscreen API, with resource reuse. */
  image_process_view3d_fn = image_process_camera_view_draw_cached;
  image_process_camera_view_render_gpu_fn = nullptr;
  image_process_render_material_fn = image_process_render_material_draw;
  image_process_render_material_free_fn = render_material_scratch_free;

  std::unique_ptr<SpaceType> st = std::make_unique<SpaceType>();
  ARegionType *art;

  st->spaceid = SPACE_NODE;
  STRNCPY_UTF8(st->name, "Node");

  st->create = node_create;
  st->free = node_free;
  st->init = node_init;
  st->exit = node_exit;
  st->duplicate = node_duplicate;
  st->operatortypes = node_operatortypes;
  st->keymap = node_keymap;
  st->listener = node_area_listener;
  st->refresh = node_area_refresh;
  st->context = node_context;
  st->dropboxes = node_dropboxes;
  st->gizmos = node_widgets;
  st->id_remap = node_id_remap;
  st->foreach_id = node_foreach_id;
  st->space_subtype_item_extend = node_space_subtype_item_extend;
  st->space_subtype_get = node_space_subtype_get;
  st->space_subtype_set = node_space_subtype_set;
  st->space_name_get = node_space_name_get;
  st->space_icon_get = node_space_icon_get;
  st->blend_read_data = node_space_blend_read_data;
  st->blend_read_after_liblink = nullptr;
  st->blend_write = node_space_blend_write;

  /* regions: main window */
  art = MEM_new_zeroed<ARegionType>("spacetype node region");
  art->regionid = RGN_TYPE_WINDOW;
  art->init = node_main_region_init;
  art->draw = node_main_region_draw;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_GIZMO | ED_KEYMAP_TOOL | ED_KEYMAP_VIEW2D |
                    ED_KEYMAP_FRAMES | ED_KEYMAP_GPENCIL;
  art->listener = node_region_listener;
  art->cursor = node_cursor;
  art->event_cursor = true;
  art->clip_gizmo_events_by_ui = true;
  art->lock = REGION_DRAW_LOCK_ALL;

  BLI_addhead(&st->regiontypes, art);

  /* regions: header */
  art = MEM_new_zeroed<ARegionType>("spacetype node region");
  art->regionid = RGN_TYPE_HEADER;
  art->prefsizey = HEADERY;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_VIEW2D | ED_KEYMAP_FRAMES | ED_KEYMAP_HEADER;
  art->listener = node_region_listener;
  art->init = node_header_region_init;
  art->draw = node_header_region_draw;

  BLI_addhead(&st->regiontypes, art);

  /* regions: asset shelf */
  art = MEM_new_zeroed<ARegionType>("spacetype node asset shelf region");
  art->regionid = RGN_TYPE_ASSET_SHELF;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_ASSET_SHELF | ED_KEYMAP_FRAMES;
  art->duplicate = asset::shelf::region_duplicate;
  art->free = asset::shelf::region_free;
  art->on_poll_success = asset::shelf::region_on_poll_success;
  art->listener = asset::shelf::region_listen;
  art->message_subscribe = asset::shelf::region_message_subscribe;
  art->poll = asset::shelf::regions_poll;
  art->snap_size = asset::shelf::region_snap;
  art->on_user_resize = asset::shelf::region_on_user_resize;
  art->context = asset::shelf::context;
  art->init = node_asset_shelf_region_init;
  art->layout = asset::shelf::region_layout;
  art->draw = asset::shelf::region_draw;
  BLI_addhead(&st->regiontypes, art);

  /* regions: asset shelf header */
  art = MEM_new_zeroed<ARegionType>("spacetype node asset shelf header region");
  art->regionid = RGN_TYPE_ASSET_SHELF_HEADER;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_ASSET_SHELF | ED_KEYMAP_VIEW2D | ED_KEYMAP_FOOTER;
  art->init = asset::shelf::header_region_init;
  art->poll = asset::shelf::regions_poll;
  art->draw = asset::shelf::header_region;
  art->listener = asset::shelf::header_region_listen;
  art->context = asset::shelf::context;
  BLI_addhead(&st->regiontypes, art);
  asset::shelf::types_register(art, SPACE_NODE);

  /* regions: list-view/buttons */
  art = MEM_new_zeroed<ARegionType>("spacetype node region");
  art->regionid = RGN_TYPE_UI;
  art->flag = ARegionTypeFlag::UsePanelCategoriesSearch;
  art->prefsizex = UI_SIDEBAR_PANEL_WIDTH;
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_FRAMES;
  art->listener = node_region_listener;
  art->message_subscribe = ED_area_do_mgs_subscribe_for_tool_ui;
  art->init = node_buttons_region_init;
  art->snap_size = ED_region_generic_panel_region_snap_size;
  art->draw = node_buttons_region_draw;
  BLI_addhead(&st->regiontypes, art);

  node_tree_interface_panel_register(art);
  node_modal_keymap_panel_register(art);

  /* regions: toolbar */
  art = MEM_new_zeroed<ARegionType>("spacetype view3d tools region");
  art->regionid = RGN_TYPE_TOOLS;
  art->prefsizex = int(UI_TOOLBAR_WIDTH);
  art->prefsizey = 50; /* XXX */
  art->keymapflag = ED_KEYMAP_UI | ED_KEYMAP_FRAMES;
  art->listener = node_region_listener;
  art->message_subscribe = ED_region_generic_tools_region_message_subscribe;
  art->snap_size = ED_region_generic_tools_region_snap_size;
  art->init = node_toolbar_region_init;
  art->draw = node_toolbar_region_draw;
  BLI_addhead(&st->regiontypes, art);

  WM_menutype_add(MEM_new<MenuType>(__func__, catalog_assets_menu_type()));
  WM_menutype_add(MEM_new<MenuType>(__func__, unassigned_assets_menu_type()));
  WM_menutype_add(MEM_new<MenuType>(__func__, add_root_catalogs_menu_type()));
  WM_menutype_add(MEM_new<MenuType>(__func__, swap_root_catalogs_menu_type()));

  BKE_spacetype_register(std::move(st));
}

}  // namespace blender
