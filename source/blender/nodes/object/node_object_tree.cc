/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#include "DNA_node_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"

#include "BKE_context.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "UI_resources.hh"

#include "node_common.h"

#include "RNA_prototypes.hh"

#include "NOD_object.hh"
#include "node_object_util.hh"

namespace blender {

bke::bNodeTreeType *ntreeType_Object;

static void object_get_from_context(const bContext *C,
                                    bke::bNodeTreeType * /*treetype*/,
                                    bNodeTree **r_ntree,
                                    ID **r_id,
                                    ID **r_from)
{
  Scene *scene = CTX_data_scene(C);
  ID *scene_id = scene ? &scene->id : nullptr;

  const SpaceNode *snode = CTX_wm_space_node(C);
  if (snode && snode->selected_node_group && snode->selected_node_group->type == NTREE_OBJECT) {
    *r_ntree = snode->selected_node_group;
    *r_id = scene_id;
    *r_from = nullptr;
    return;
  }

  if (snode && snode->nodetree && snode->nodetree->type == NTREE_OBJECT &&
      snode->selected_node_group == snode->nodetree)
  {
    *r_ntree = snode->nodetree;
    *r_id = scene_id;
    *r_from = nullptr;
    return;
  }

  *r_ntree = nullptr;
  *r_id = nullptr;
  *r_from = nullptr;
}

static void foreach_nodeclass(void *calldata, bke::bNodeClassCallback func)
{
  func(calldata, NODE_CLASS_INPUT, N_("Input"));
  func(calldata, NODE_CLASS_CONVERTER, N_("Object"));
  func(calldata, NODE_CLASS_OUTPUT, N_("Output"));
  func(calldata, NODE_CLASS_GROUP, N_("Group"));
  func(calldata, NODE_CLASS_LAYOUT, N_("Layout"));
}

static void update(bNodeTree *ntree)
{
  /* Never evaluate here. typeinfo->update runs in the middle of BKE_ntree_update while
   * topology is being rebuilt. Creating/deleting objects or calling ensure_topology_cache
   * from this callback crashes. Evaluation is deferred until BKE_ntree_update finishes. */
  bke::node_tree_set_output(*ntree);
  ntree_update_reroute_nodes(ntree);
}

static bool object_node_tree_socket_type_valid(bke::bNodeTreeType * /*ntreetype*/,
                                               bke::bNodeSocketType *socket_type)
{
  if (socket_type->type == SOCK_CUSTOM) {
    return true;
  }
  return bke::node_is_static_socket_type(*socket_type) && ELEM(socket_type->type,
                                                               SOCK_FLOAT,
                                                               SOCK_INT,
                                                               SOCK_BOOLEAN,
                                                               SOCK_VECTOR,
                                                               SOCK_RGBA,
                                                               SOCK_ROTATION,
                                                               SOCK_MATRIX,
                                                               SOCK_STRING,
                                                               SOCK_OBJECT,
                                                               SOCK_MATERIAL,
                                                               SOCK_COLLECTION,
                                                               SOCK_MENU,
                                                               SOCK_COLOR_RAMP,
                                                               SOCK_CURVE);
}

static bool object_validate_link(eNodeSocketDatatype from_type, eNodeSocketDatatype to_type)
{
  if (from_type == SOCK_CUSTOM || to_type == SOCK_CUSTOM) {
    return true;
  }
  if (ELEM(from_type, SOCK_OBJECT, SOCK_MATERIAL, SOCK_COLLECTION, SOCK_STRING) ||
      ELEM(to_type, SOCK_OBJECT, SOCK_MATERIAL, SOCK_COLLECTION, SOCK_STRING))
  {
    return from_type == to_type;
  }
  if (ELEM(from_type, SOCK_FLOAT, SOCK_VECTOR, SOCK_RGBA, SOCK_BOOLEAN, SOCK_INT) &&
      ELEM(to_type, SOCK_FLOAT, SOCK_VECTOR, SOCK_RGBA, SOCK_BOOLEAN, SOCK_INT))
  {
    return true;
  }
  return from_type == to_type;
}

void register_node_tree_type_obj()
{
  bke::bNodeTreeType *tt = ntreeType_Object = MEM_new<bke::bNodeTreeType>(__func__);

  tt->type = NTREE_OBJECT;
  tt->idname = "ObjectNodeTree"_ustr;
  tt->group_idname = "ObjectNodeGroup"_ustr;
  tt->ui_name = N_("Object Editor");
  tt->ui_icon = ICON_OUTLINER_OB_MESH;
  tt->ui_description = N_(
      "Edit object properties, transforms, visibility, material slots, modifiers and hierarchy");
  tt->asset_catalog_path_prefix = "Object Editor";

  tt->foreach_nodeclass = foreach_nodeclass;
  tt->update = update;
  tt->get_from_context = object_get_from_context;
  tt->validate_link = object_validate_link;
  tt->valid_socket_type = object_node_tree_socket_type_valid;

  tt->rna_ext.srna = RNA_ObjectNodeTree;

  bke::node_tree_type_add(*tt);

  ntreeObjectNodesRegisterLoadHooks();
}

}  // namespace blender
