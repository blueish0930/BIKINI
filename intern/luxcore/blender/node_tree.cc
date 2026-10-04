/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Independent LuxCore material node tree. Appears as its own Node Editor subtype. */

#include "nodes.hh"

#include "DNA_layer_types.h"
#include "DNA_material_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"


#include "MEM_guardedalloc.h"

#include "BLI_listbase.hh"
#include "BLI_string.hh"
#include "BLI_utildefines.hh"

#include "BLT_translation.hh"

#include "BKE_context.hh"
#include "BKE_layer.hh"
#include "BKE_main.hh"
#include "BKE_material.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_scene.hh"

#include "RNA_access.hh"

#include "UI_resources.hh"

#include "node_common.h"

namespace blender {

static bool lux_tree_poll(const bContext *C, bke::bNodeTreeType * /*treetype*/)
{
  const Scene *scene = CTX_data_scene(C);
  return scene && scene->r.engine[0] && STREQ(scene->r.engine, "LUXCORE");
}

static void lux_get_from_context(const bContext *C,
                                 bke::bNodeTreeType * /*treetype*/,
                                 bNodeTree **r_ntree,
                                 ID **r_id,
                                 ID **r_from)
{
  /* Callers (snode_set_context) zero these first, but foreach_id / pin paths
   * do not. Never leave garbage, and never pair a Material id with a null
   * tree: SpaceNode would then treat it as embedded and swap in Cycles'
   * ma->nodetree (node_tree_from_id), which silently kills Blender. */
  *r_ntree = nullptr;
  *r_id = nullptr;
  *r_from = nullptr;

  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  if (bmain == nullptr || scene == nullptr || view_layer == nullptr) {
    return;
  }
  BKE_view_layer_synced_ensure(*bmain, scene, view_layer);
  Object *ob = BKE_view_layer_active_object_get(view_layer);
  if (ob == nullptr) {
    return;
  }
  Material *ma = BKE_object_material_get(ob, ob->actcol);
  if (ma == nullptr) {
    return;
  }
  /* Owner is the Material so the header ID picker matches the shader editor.
   * The tree itself is independent (not node_tree_from_id(ma), which is Cycles).
   * SpaceNode foreach_id must not steal ma->nodetree when luxcore_nodetree is
   * null — that path is guarded in space_node.cc. */
  *r_from = &ob->id;
  *r_id = &ma->id;
  *r_ntree = ma->luxcore_nodetree;
}

static void lux_foreach_nodeclass(void *calldata, bke::bNodeClassCallback func)
{
  func(calldata, NODE_CLASS_INPUT, N_("Input"));
  func(calldata, NODE_CLASS_OUTPUT, N_("Output"));
  func(calldata, NODE_CLASS_SHADER, N_("Material"));
  func(calldata, NODE_CLASS_TEXTURE, N_("Texture"));
  func(calldata, NODE_CLASS_CONVERTER, N_("Converter"));
  func(calldata, NODE_CLASS_LAYOUT, N_("Layout"));
}

static void lux_update(bNodeTree *ntree)
{
  if (ntree == nullptr) {
    return;
  }
  bke::node_tree_set_output(*ntree);
  ntree_update_reroute_nodes(ntree);
}

static bool lux_validate_link(eNodeSocketDatatype from, eNodeSocketDatatype to)
{
  if (from == SOCK_SHADER) {
    return to == SOCK_SHADER;
  }
  return from == to || ELEM(from, SOCK_FLOAT, SOCK_VECTOR, SOCK_RGBA) ||
         ELEM(to, SOCK_FLOAT, SOCK_VECTOR, SOCK_RGBA);
}

static bool lux_socket_type_valid(bke::bNodeTreeType * /*ntreetype*/,
                                  bke::bNodeSocketType *socket_type)
{
  return bke::node_is_static_socket_type(*socket_type) &&
         ELEM(socket_type->type, SOCK_FLOAT, SOCK_INT, SOCK_VECTOR, SOCK_RGBA, SOCK_SHADER);
}

void LUX_node_tree_register()
{
  bke::bNodeTreeType *tt = MEM_new<bke::bNodeTreeType>(__func__);
  tt->type = NTREE_CUSTOM;
  tt->idname = "LuxCoreMaterialNodeTree"_ustr;
  tt->ui_name = N_("LuxCore Node Editor");
  tt->ui_icon = ICON_NODE_MATERIAL;
  tt->ui_description = N_("Edit LuxCore materials in a dedicated node tree");

  tt->foreach_nodeclass = lux_foreach_nodeclass;
  tt->poll = lux_tree_poll;
  tt->get_from_context = lux_get_from_context;
  tt->update = lux_update;
  tt->validate_link = lux_validate_link;
  tt->valid_socket_type = lux_socket_type_valid;
  tt->no_group_interface = 1;

  tt->rna_ext.srna = RNA_struct_find("LuxCoreMaterialNodeTree");
  if (tt->rna_ext.srna) {
    RNA_struct_blender_type_set(tt->rna_ext.srna, tt);
  }

  bke::node_tree_type_add(*tt);
}

}  // namespace blender
