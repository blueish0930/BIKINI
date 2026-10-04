/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* IMAGE_NODES_MVP: ImageNodeTree registration (distinct from CompositorNodeTree). */

/** \file
 * \ingroup nodes
 */

#include "DNA_node_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"

#include "BKE_context.hh"
#include "BKE_main.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "UI_resources.hh"

#include "node_common.h"

#include "RNA_prototypes.hh"

#include "NOD_image.hh"
#include "node_image_util.hh"

namespace blender {

bke::bNodeTreeType *ntreeType_Image;

static void image_get_from_context(const bContext *C,
                                   bke::bNodeTreeType * /*treetype*/,
                                   bNodeTree **r_ntree,
                                   ID **r_id,
                                   ID **r_from)
{
  /* Scene is the compute-context root for Image Process evaluation/timings (matches
   * ImageNodesContext). Tree remains an independent data-block; id points at the scene. */
  Scene *scene = CTX_data_scene(C);
  ID *scene_id = scene ? &scene->id : nullptr;

  /* Image trees are independent data-blocks. selected_node_group is the UI source of truth
   * (template_ID). Do not fall back to nodetree/edittree/first-in-Main after the user unlinks
   * with the header X — that used to immediately re-select the previous group. */
  const SpaceNode *snode = CTX_wm_space_node(C);
  if (snode && snode->selected_node_group && snode->selected_node_group->type == NTREE_IMAGE) {
    *r_ntree = snode->selected_node_group;
    *r_id = scene_id;
    *r_from = nullptr;
    return;
  }

  /* Only use nodetree when it already is the selected root (nested Import Points path keeps
   * the same root while edittree is Geometry). Never invent a tree when selection is empty. */
  if (snode && snode->nodetree && snode->nodetree->type == NTREE_IMAGE &&
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
  func(calldata, NODE_CLASS_OUTPUT, N_("Output"));
  func(calldata, NODE_CLASS_OP_COLOR, N_("Color"));
  func(calldata, NODE_CLASS_OP_FILTER, N_("Filter"));
  func(calldata, NODE_CLASS_TEXTURE, N_("Texture"));
  func(calldata, NODE_CLASS_DISTORT, N_("Transform"));
  func(calldata, NODE_CLASS_CONVERTER, N_("Utilities"));
  func(calldata, NODE_CLASS_GROUP, N_("Group"));
  func(calldata, NODE_CLASS_LAYOUT, N_("Layout"));
}

static void update(bNodeTree *ntree)
{
  bke::node_tree_set_output(*ntree);
  ntree_update_reroute_nodes(ntree);
}

static bool image_node_tree_socket_type_valid(bke::bNodeTreeType * /*ntreetype*/,
                                              bke::bNodeSocketType *socket_type)
{
  /* Virtual extend sockets (zone item "+") are SOCK_CUSTOM / NodeSocketVirtual. */
  if (socket_type->type == SOCK_CUSTOM) {
    return true;
  }
  return bke::node_is_static_socket_type(*socket_type) && ELEM(socket_type->type,
                                                               SOCK_FLOAT,
                                                               SOCK_INT,
                                                               SOCK_BOOLEAN,
                                                               SOCK_VECTOR,
                                                               SOCK_INT_VECTOR,
                                                               SOCK_RGBA,
                                                               SOCK_MATRIX,
                                                               SOCK_MENU,
                                                               SOCK_ROTATION,
                                                               SOCK_STRING,
                                                               SOCK_OBJECT,
                                                               /* Geometry pipeline (Import Points / Import Geo / Point Stamp). */
                                                               SOCK_GEOMETRY,
                                                               /* Zones / bundles / closures. */
                                                               SOCK_BUNDLE,
                                                               SOCK_CLOSURE,
                                                               SOCK_COLOR_RAMP,
                                                               SOCK_CURVE);
}

static bool image_validate_link(eNodeSocketDatatype from_type, eNodeSocketDatatype to_type)
{
  /* Allow links to/from virtual extend sockets so Simulation/Repeat/Fluid "+" can add items. */
  if (from_type == SOCK_CUSTOM || to_type == SOCK_CUSTOM) {
    return true;
  }
  /* Geometry (Import Points → Point Stamp) must not accept Color/Float/etc. — wrong links
   * previously crashed evaluation instead of being rejected at connect time. */
  if (from_type == SOCK_GEOMETRY || to_type == SOCK_GEOMETRY) {
    return from_type == to_type;
  }
  if (ELEM(from_type, SOCK_BUNDLE, SOCK_CLOSURE) || ELEM(to_type, SOCK_BUNDLE, SOCK_CLOSURE)) {
    return from_type == to_type;
  }
  if (ELEM(from_type, SOCK_FLOAT, SOCK_VECTOR, SOCK_INT_VECTOR, SOCK_RGBA, SOCK_BOOLEAN, SOCK_INT) &&
      ELEM(to_type, SOCK_FLOAT, SOCK_VECTOR, SOCK_INT_VECTOR, SOCK_RGBA, SOCK_BOOLEAN, SOCK_INT))
  {
    return true;
  }
  return from_type == to_type;
}

void register_node_tree_type_img()
{
  bke::bNodeTreeType *tt = ntreeType_Image = MEM_new<bke::bNodeTreeType>(__func__);

  tt->type = NTREE_IMAGE;
  tt->idname = "ImageNodeTree"_ustr;
  tt->group_idname = "ImageNodeGroup"_ustr;
  /* Distinct from the Image Editor space; this is a GPU texture node graph. */
  tt->ui_name = N_("GPU Texture Editor");
  /* Same icon as the legacy Texture node editor. */
  tt->ui_icon = ICON_NODE_TEXTURE;
  tt->ui_description = N_(
      "GPU-accelerated texture node graph (filters, transforms, and image processing)");
  tt->asset_catalog_path_prefix = "GPU Texture Editor";

  tt->foreach_nodeclass = foreach_nodeclass;
  tt->update = update;
  tt->get_from_context = image_get_from_context;
  tt->validate_link = image_validate_link;
  tt->valid_socket_type = image_node_tree_socket_type_valid;

  tt->rna_ext.srna = RNA_ImageNodeTree;

  bke::node_tree_type_add(*tt);

  /* After opening a .blend, re-evaluate all Image Process trees so Image Output textures
   * (not stored as baked pixels in the file for generated paths) are repopulated. */
  ntreeImageNodesRegisterLoadHooks();
}

}  // namespace blender
