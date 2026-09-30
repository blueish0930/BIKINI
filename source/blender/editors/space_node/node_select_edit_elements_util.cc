/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup ed_node
 *
 * Shared Enter geometry resolve for Select Elements / Edit Elements.
 *
 * Primary path (Visual Geometry to Objects on an isolated modifier output):
 * 1. Wire this node's Geometry output to the Geometry Nodes modifier Group Output
 *    (and parent group outputs when the node sits in a nested group).
 * 2. Temporarily disable every modifier below that Geometry Nodes modifier,
 *    keeping modifiers above it.
 * 3. Evaluate, copy the object's visual geometry, then restore links + modifiers.
 * Fallback: node input cache, then original mesh datablock.
 */

#include "BKE_context.hh"
#include "BKE_curves.h"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_geometry_set_instances.hh"
#include "BKE_main.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"

#include "BLI_listbase.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "DNA_curves_types.h"
#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"
#include "DNA_space_types.h"

#include "ED_node.hh"

#include "GEO_realize_instances.hh"

#include "NOD_geo_edit_elements.hh"
#include "NOD_geo_select_elements.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

static const bNodeSocket *find_geometry_input_socket(const bNode &node)
{
  for (const bNodeSocket *sock : node.input_sockets()) {
    if (sock->is_available() && sock->type == SOCK_GEOMETRY) {
      return sock;
    }
  }
  return nullptr;
}

static std::optional<bke::GeometrySet> single_component_geometry(bke::GeometrySet geometry)
{
  if (geometry.is_empty()) {
    return std::nullopt;
  }
  if (geometry.has_instances()) {
    geometry::RealizeInstancesOptions options;
    geometry = geometry::realize_instances(std::move(geometry), options).geometry;
  }

  const bool has_mesh = geometry.has_mesh() && geometry.get_mesh()->verts_num > 0;
  const bool has_curves = geometry.has_curves() && geometry.get_curves()->geometry.point_num > 0;
  const bool has_points = geometry.has_pointcloud() && geometry.get_pointcloud()->totpoint > 0;

  /* Prefer mesh; otherwise the single non-empty type. */
  if (has_mesh) {
    return bke::GeometrySet::from_mesh(BKE_mesh_copy_for_eval(*geometry.get_mesh()));
  }
  if (has_curves && !has_points) {
    return bke::GeometrySet::from_curves(BKE_curves_copy_for_eval(geometry.get_curves()));
  }
  if (has_points && !has_curves) {
    return std::nullopt;
  }
  return std::nullopt;
}

static std::optional<bke::GeometrySet> copy_node_input_cache(const bNodeTree &tree,
                                                              const bNode &node)
{
  if (nodes::is_select_elements_node(node)) {
    return nodes::select_elements_copy_cached_geometry(tree, node);
  }
  if (nodes::is_edit_elements_node(node)) {
    return nodes::edit_elements_copy_cached_geometry(tree, node);
  }
  return std::nullopt;
}

static bNodeSocket *find_first_geometry_socket(bNode &node, const eNodeSocketInOut in_out)
{
  ListBaseT<bNodeSocket> &sockets = (in_out == SOCK_OUT) ? node.outputs : node.inputs;
  for (bNodeSocket &socket : sockets) {
    if (socket.is_available() && socket.type == SOCK_GEOMETRY) {
      return &socket;
    }
  }
  return nullptr;
}

static bNode *find_active_group_output(bNodeTree &tree)
{
  bNode *fallback = nullptr;
  for (bNode &node : tree.nodes) {
    if (!node.is_group_output()) {
      continue;
    }
    if (node.flag & NODE_DO_OUTPUT) {
      return &node;
    }
    if (fallback == nullptr) {
      fallback = &node;
    }
  }
  return fallback;
}

/**
 * Temporarily isolate a node as the Geometry Nodes modifier output, then restore.
 *
 * - Rewire Geometry → Group Output along the editor tree path.
 * - Disable modifiers after the current Geometry Nodes modifier.
 */
struct IsolatedModifierOutput {
  struct SavedLink {
    bNodeTree *tree = nullptr;
    bNode *fromnode = nullptr;
    bNodeSocket *fromsock = nullptr;
    bNode *tonode = nullptr;
    bNodeSocket *tosock = nullptr;
    eNodeLink_Flag flag = {};
    int multi_input_sort_id = 0;
  };

  Main *bmain = nullptr;
  Object *object = nullptr;
  Vector<SavedLink> removed_links;
  Vector<SavedLink> added_links;
  Vector<ModifierData *> disabled_modifiers;
  VectorSet<bNodeTree *> modified_trees;
  bool restored = false;

  IsolatedModifierOutput(Main *bmain, Object *object) : bmain(bmain), object(object) {}

  ~IsolatedModifierOutput()
  {
    restore();
  }

  IsolatedModifierOutput(const IsolatedModifierOutput &) = delete;
  IsolatedModifierOutput &operator=(const IsolatedModifierOutput &) = delete;
  IsolatedModifierOutput(IsolatedModifierOutput &&) = delete;
  IsolatedModifierOutput &operator=(IsolatedModifierOutput &&) = delete;

  void restore()
  {
    if (restored) {
      return;
    }
    restored = true;

    for (const SavedLink &added : added_links) {
      if (added.tree == nullptr || added.fromsock == nullptr || added.tosock == nullptr) {
        continue;
      }
      Vector<bNodeLink *> matches;
      for (bNodeLink &link : added.tree->links) {
        if (link.fromsock == added.fromsock && link.tosock == added.tosock) {
          matches.append(&link);
        }
      }
      for (bNodeLink *link : matches) {
        bke::node_remove_link(added.tree, *link);
      }
      modified_trees.add(added.tree);
    }

    for (const SavedLink &saved : removed_links) {
      if (saved.tree == nullptr || saved.fromnode == nullptr || saved.fromsock == nullptr ||
          saved.tonode == nullptr || saved.tosock == nullptr)
      {
        continue;
      }
      saved.tree->ensure_topology_cache();
      bNodeLink &link = bke::node_add_link(
          *saved.tree, *saved.fromnode, *saved.fromsock, *saved.tonode, *saved.tosock);
      link.flag = saved.flag;
      link.multi_input_sort_id = saved.multi_input_sort_id;
      modified_trees.add(saved.tree);
    }

    for (ModifierData *md : disabled_modifiers) {
      md->mode &= ~eModifierMode_DisableTemporary;
    }

    tag_and_update_trees();
    if (object) {
      DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
    }
  }

  void tag_and_update_trees()
  {
    if (!bmain || modified_trees.is_empty()) {
      return;
    }
    Vector<bNodeTree *> trees;
    for (bNodeTree *tree : modified_trees) {
      trees.append(tree);
    }
    BKE_ntree_update(*bmain, trees.as_span());
    for (bNodeTree *tree : trees) {
      DEG_id_tag_update(&tree->id, ID_RECALC_NTREE_OUTPUT | ID_RECALC_SYNC_TO_EVAL);
    }
  }

  bool connect_geometry_to_group_output(bNodeTree &tree, bNode &from_node)
  {
    bNode *output = find_active_group_output(tree);
    if (output == nullptr) {
      return false;
    }
    bNodeSocket *from_sock = find_first_geometry_socket(from_node, SOCK_OUT);
    bNodeSocket *to_sock = find_first_geometry_socket(*output, SOCK_IN);
    if (from_sock == nullptr || to_sock == nullptr) {
      return false;
    }

    bool already_connected = false;
    Vector<bNodeLink *> to_remove;
    for (bNodeLink &link : tree.links) {
      if (link.tosock != to_sock) {
        continue;
      }
      if (link.fromsock == from_sock && link.fromnode == &from_node && !link.is_muted()) {
        already_connected = true;
        continue;
      }
      to_remove.append(&link);
    }

    for (bNodeLink *link : to_remove) {
      SavedLink saved;
      saved.tree = &tree;
      saved.fromnode = link->fromnode;
      saved.fromsock = link->fromsock;
      saved.tonode = link->tonode;
      saved.tosock = link->tosock;
      saved.flag = link->flag;
      saved.multi_input_sort_id = link->multi_input_sort_id;
      removed_links.append(saved);
      bke::node_remove_link(&tree, *link);
    }

    if (!already_connected) {
      tree.ensure_topology_cache();
      bke::node_add_link(tree, from_node, *from_sock, *output, *to_sock);
      SavedLink added;
      added.tree = &tree;
      added.fromnode = &from_node;
      added.fromsock = from_sock;
      added.tonode = output;
      added.tosock = to_sock;
      added_links.append(added);
    }

    modified_trees.add(&tree);
    return true;
  }

  bool isolate_node_as_modifier_output(SpaceNode &snode, bNode &node)
  {
    if (!connect_geometry_to_group_output(node.owner_tree(), node)) {
      return false;
    }

    Vector<bNodeTreePath *> path;
    for (bNodeTreePath &item : snode.treepath) {
      path.append(&item);
    }
    if (path.size() <= 1 || path.last()->nodetree != &node.owner_tree()) {
      return true;
    }

    for (int i = int(path.size()) - 1; i > 0; --i) {
      bNodeTree *parent_tree = path[i - 1]->nodetree;
      if (parent_tree == nullptr) {
        return false;
      }
      bNode *group_node = bke::node_find_node_by_name(*parent_tree, path[i]->node_name);
      if (group_node == nullptr) {
        return false;
      }
      if (!connect_geometry_to_group_output(*parent_tree, *group_node)) {
        return false;
      }
    }
    return true;
  }

  void disable_modifiers_after(const NodesModifierData &nmd)
  {
    if (object == nullptr) {
      return;
    }
    bool seen_nodes_modifier = false;
    for (ModifierData &md : object->modifiers) {
      if (&md == &nmd.modifier) {
        seen_nodes_modifier = true;
        continue;
      }
      if (!seen_nodes_modifier) {
        continue;
      }
      if (md.mode & eModifierMode_DisableTemporary) {
        continue;
      }
      md.mode |= eModifierMode_DisableTemporary;
      disabled_modifiers.append(&md);
    }
  }
};

std::optional<bke::GeometrySet> resolve_visual_geometry_for_elements_node(
    bContext *C, SpaceNode &snode, bNode &node, ReportList *reports, Object **r_transform_from)
{
  *r_transform_from = nullptr;

  const std::optional<ObjectAndModifier> object_and_modifier =
      find_object_and_nodes_modifier_for_geometry_edit(C, snode);
  if (!object_and_modifier) {
    BKE_report(reports,
               RPT_ERROR,
               "No object with a Geometry Nodes modifier using this tree "
               "(select the object / open the tree from the modifier)");
    return std::nullopt;
  }

  Object *object = const_cast<Object *>(object_and_modifier->object);
  const NodesModifierData *nmd = object_and_modifier->nmd;
  *r_transform_from = object;

  if (!nmd->node_group) {
    BKE_report(reports, RPT_ERROR, "No root node tree on modifier");
    return std::nullopt;
  }
  if (!(nmd->modifier.mode & eModifierMode_Realtime)) {
    BKE_report(reports, RPT_ERROR, "Enable Geometry Nodes modifier viewport visibility");
    return std::nullopt;
  }

  bNodeTree &edit_tree = *snode.edittree;
  edit_tree.ensure_topology_cache();

  const bNodeSocket *geometry_in = find_geometry_input_socket(node);
  if (!geometry_in) {
    BKE_report(reports, RPT_ERROR, "Node has no Geometry input");
    return std::nullopt;
  }
  if (!geometry_in->is_logically_linked()) {
    BKE_report(reports,
               RPT_ERROR,
               "Geometry input is not connected — connect the previous node's Geometry output");
    return std::nullopt;
  }

  Main *bmain = CTX_data_main(C);
  Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);

  std::optional<bke::GeometrySet> result;

  /* Isolate this node as the modifier output, drop later modifiers, then evaluate visual geo.
   * Copy the mesh before restoring so we don't keep pointers into evaluated data. */
  if (bmain && depsgraph) {
    IsolatedModifierOutput isolation(bmain, object);
    const bool wired = isolation.isolate_node_as_modifier_output(snode, node);
    isolation.disable_modifiers_after(*nmd);
    isolation.tag_and_update_trees();
    DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
    DEG_id_tag_update(&nmd->node_group->id, ID_RECALC_NTREE_OUTPUT);
    DEG_id_tag_update(&edit_tree.id, ID_RECALC_NTREE_OUTPUT);
    BKE_scene_graph_update_tagged(depsgraph, bmain);

    Object *object_eval = DEG_get_evaluated(depsgraph, object);
    if (object_eval) {
      bke::GeometrySet eval_geo = bke::object_get_evaluated_geometry_set(*object_eval);
      if (!eval_geo.is_empty()) {
        result = single_component_geometry(std::move(eval_geo));
        if (result) {
          BKE_report(reports,
                     RPT_INFO,
                     wired ? "Using visual geometry at this node "
                             "(wired to modifier output, later modifiers disabled)" :
                             "Using visual geometry with later modifiers disabled");
        }
      }
    }
    /* Destructor restores original links and modifier flags. */
  }

  if (!result) {
    std::optional<bke::GeometrySet> raw = copy_node_input_cache(edit_tree, node);
    if (raw && !raw->is_empty()) {
      result = single_component_geometry(std::move(*raw));
      if (result) {
        BKE_report(reports, RPT_INFO, "Using previous-node geometry (node input cache)");
      }
    }
  }

  if (!result && object->type == OB_MESH && object->data) {
    const Mesh *mesh_orig = reinterpret_cast<const Mesh *>(object->data);
    if (mesh_orig->verts_num > 0) {
      result = bke::GeometrySet::from_mesh(BKE_mesh_copy_for_eval(*mesh_orig));
      BKE_report(reports, RPT_INFO, "Using original object mesh (no evaluated geometry yet)");
    }
  }

  if (!result) {
    BKE_report(reports,
               RPT_ERROR,
               "Could not get visual geometry. Ensure the Geometry Nodes modifier is enabled, "
               "the previous node outputs a mesh, and the object is visible in the viewport");
    return std::nullopt;
  }

  if (!result->has_mesh() && !result->has_curves()) {
    BKE_report(reports,
               RPT_ERROR,
               "Geometry has no usable mesh (or single curve). "
               "Connect one mesh-producing node");
    return std::nullopt;
  }

  if (const Mesh *m = result->get_mesh()) {
    BKE_reportf(reports,
                RPT_INFO,
                "Visual geometry: %d verts, %d edges, %d faces",
                m->verts_num,
                m->edges_num,
                m->faces_num);
  }
  else if (const Curves *c = result->get_curves()) {
    BKE_reportf(reports, RPT_INFO, "Visual geometry: curves (%d points)", c->geometry.point_num);
  }

  return result;
}

}  // namespace blender::ed::space_node
