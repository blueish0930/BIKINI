/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup ed_node
 *
 * Edit Elements node → free Edit Mode on a temporary object for one component of the geometry:
 * mesh, curves, point cloud, or the instances as one point each.
 * Esc/Enter/Tab writes the full result into the node (#bNode.id).
 */

#include "BKE_compute_contexts.hh"
#include "BKE_context.hh"
#include "BKE_curves.h"
#include "BKE_geometry_set.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_main_invariants.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_object.hh"
#include "BKE_pointcloud.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"

#include "GEO_realize_instances.hh"

#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_geometry_edit.hh"
#include "ED_node.hh"
#include "ED_object.hh"
#include "ED_screen.hh"

#include "NOD_geo_edit_elements.hh"
#include "NOD_geo_elements_component.hh"
#include "NOD_geometry_nodes_execute.hh"
#include "NOD_geometry_nodes_lazy_function.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_prototypes.hh"

#include <climits>

#include "WM_api.hh"
#include "WM_types.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

struct EditElementsSessionBridge {
  bNodeTree *ntree = nullptr;
  int32_t node_identifier = 0;
  bool active = false;
};

static EditElementsSessionBridge &edit_bridge()
{
  static EditElementsSessionBridge b;
  return b;
}

static bNode *find_node_by_identifier(bNodeTree &ntree, const int32_t identifier)
{
  for (bNode *node : ntree.all_nodes()) {
    if (node->identifier == identifier) {
      return node;
    }
  }
  return nullptr;
}

static bNode *find_single_selected_edit_elements_node(bNodeTree &ntree)
{
  VectorSet<bNode *> selected = get_selected_nodes(ntree);
  if (selected.size() != 1) {
    return nullptr;
  }
  bNode *node = *selected.begin();
  if (!nodes::is_edit_elements_node(*node)) {
    return nullptr;
  }
  return node;
}

/**
 * Fill the session parameters with the component to edit. A stored result is edited further,
 * otherwise the visual geometry at the node is the start (input cache → object visual geo, like
 * Visual Geometry to Objects).
 */
static bool resolve_edit_geometry(bContext *C,
                                  SpaceNode &snode,
                                  bNode &node,
                                  ReportList *reports,
                                  geometry_edit::SessionParams &r_params,
                                  ElementsSessionData &r_data)
{
  using ComponentType = bke::GeometryComponent::Type;
  const ID *stored = nodes::edit_elements_get_result(node);

  /* The geometry at the node, that is with the stored result when there is one. Without a
   * result it is what is edited, with one it only shows the rest of the geometry. */
  Object *transform_from = nullptr;
  std::optional<bke::GeometrySet> geometry = resolve_visual_geometry_for_elements_node(
      C, snode, node, stored ? nullptr : reports, &transform_from);
  if (transform_from == nullptr) {
    if (const std::optional<ObjectAndModifier> object_and_modifier =
            find_object_and_nodes_modifier_for_geometry_edit(C, snode))
    {
      transform_from = const_cast<Object *>(object_and_modifier->object);
    }
  }
  r_params.host_object = transform_from;
  if (transform_from) {
    r_params.object_to_world = transform_from->object_to_world();
  }

  if (stored) {
    /* Continue with exactly what is stored, also when the node could not be evaluated. */
    const ComponentType component_type = nodes::edit_elements_result_component_type(node);
    bke::GeometrySet edit_geometry = geometry ? *geometry : bke::GeometrySet();
    const PointCloud *instance_points = nullptr;
    switch (component_type) {
      case ComponentType::Mesh:
        edit_geometry.replace_mesh(
            BKE_mesh_copy_for_eval(*reinterpret_cast<const Mesh *>(stored)));
        break;
      case ComponentType::Curve:
        edit_geometry.replace_curves(
            BKE_curves_copy_for_eval(reinterpret_cast<const Curves *>(stored)));
        break;
      case ComponentType::PointCloud:
        edit_geometry.replace_pointcloud(
            BKE_pointcloud_copy_for_eval(reinterpret_cast<const PointCloud *>(stored)));
        break;
      case ComponentType::Instance:
        /* The points keep the instance they come from, so they are not made again. */
        instance_points = reinterpret_cast<const PointCloud *>(stored);
        break;
      default:
        break;
    }
    if (!elements_session_params_from_geometry(
            edit_geometry, component_type, instance_points, false, r_params, r_data))
    {
      BKE_report(reports, RPT_ERROR, "The stored edit result of the node is empty, reset it");
      return false;
    }
    return true;
  }

  if (!geometry) {
    return false;
  }
  const NodeGeometryEditElements &storage = *static_cast<const NodeGeometryEditElements *>(
      node.storage);
  const std::optional<ComponentType> component_type = nodes::elements_component_pick(
      *geometry, ComponentType(storage.component_type));
  if (!component_type ||
      !elements_session_params_from_geometry(
          *geometry, *component_type, nullptr, false, r_params, r_data))
  {
    BKE_report(reports,
               RPT_ERROR,
               "The Geometry input has no mesh, curves, point cloud or instances to edit");
    return false;
  }
  return true;
}

static bool commit_to_node_from_bridge(bContext *C)
{
  EditElementsSessionBridge b_copy = edit_bridge();
  edit_bridge() = {};

  if (!b_copy.ntree || b_copy.node_identifier == 0) {
    if (geometry_edit::session_is_active()) {
      geometry_edit::session_cancel(C);
    }
    return false;
  }

  /* The session forgets its component when it is committed. */
  const bke::GeometryComponent::Type component_type = geometry_edit::session_component_type();
  ID *result = nullptr;
  if (!geometry_edit::session_commit_result(C, &result) || !result) {
    return false;
  }

  Main *bmain = CTX_data_main(C);
  bNode *node = find_node_by_identifier(*b_copy.ntree, b_copy.node_identifier);
  if (!node || !nodes::is_edit_elements_node(*node) || !bmain) {
    id_us_min(result);
    if (bmain) {
      BKE_id_delete(bmain, result);
    }
    return false;
  }

  /* Replace any previous edit result / legacy positions with the free-edit result. The node
   * takes its own user, the one from the commit is not needed anymore. */
  ID *old_result = node->id;
  nodes::edit_elements_set_result(*node, result, component_type);
  id_us_min(result);
  if (old_result && old_result != result && old_result->us == 0) {
    BKE_id_delete(bmain, old_result);
  }

  bNodeTree &tree = node->owner_tree();
  BKE_ntree_update_tag_node_property(&tree, node);
  BKE_main_ensure_invariants(*bmain, tree.id);
  DEG_id_tag_update(&tree.id, ID_RECALC_NTREE_OUTPUT);
  DEG_id_tag_update(result, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_NODE | NA_EDITED, &tree);
  WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, nullptr);
  return true;
}

static wmOperatorStatus edit_elements_edit_invoke(bContext *C,
                                                  wmOperator *op,
                                                  const wmEvent * /*event*/)
{
  if (geometry_edit::session_is_active() || edit_bridge().active) {
    BKE_report(op->reports, RPT_WARNING, "Already in a Geometry Edit session");
    return OPERATOR_CANCELLED;
  }

  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || !ED_node_is_geometry(snode)) {
    return OPERATOR_CANCELLED;
  }

  bNodeTree &ntree = *snode->edittree;
  bNode *node = find_single_selected_edit_elements_node(ntree);
  if (!node) {
    return OPERATOR_PASS_THROUGH | OPERATOR_CANCELLED;
  }

  geometry_edit::SessionParams params;
  ElementsSessionData session_data;
  if (!resolve_edit_geometry(C, *snode, *node, op->reports, params, session_data)) {
    return OPERATOR_CANCELLED;
  }
  params.ntree = &ntree;
  params.node_identifier = node->identifier;
  params.kind = geometry_edit::SessionKind::MeshEdit;

  if (!geometry_edit::session_begin(C, params, op->reports)) {
    return OPERATOR_CANCELLED;
  }

  edit_bridge().active = true;
  edit_bridge().ntree = &ntree;
  edit_bridge().node_identifier = node->identifier;

  if (params.component_type == bke::GeometryComponent::Type::Instance) {
    WM_global_report(RPT_INFO,
                     "Edit Elements: one point per instance — move, duplicate or delete them. "
                     "Esc / Enter / Tab saves the instances into the node");
  }
  else {
    WM_global_report(RPT_INFO,
                     "Edit Elements: free Edit Mode — all tools allowed. "
                     "Esc / Enter / Tab saves the result into the node");
  }

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus edit_elements_edit_modal(bContext *C,
                                                 wmOperator * /*op*/,
                                                 const wmEvent *event)
{
  if (!geometry_edit::session_is_active() ||
      geometry_edit::session_kind() != geometry_edit::SessionKind::MeshEdit)
  {
    edit_bridge() = {};
    return OPERATOR_CANCELLED;
  }

  /* Confirm: Esc / Enter / Tab. RMB must PASS_THROUGH for native Edit Mode
   * (select / context menu); do not cancel the session on right-click. */
  if (event->val == KM_PRESS &&
      ELEM(event->type, EVT_ESCKEY, EVT_RETKEY, EVT_PADENTER, EVT_TABKEY))
  {
    commit_to_node_from_bridge(C);
    return OPERATOR_FINISHED;
  }

  /* Full Edit Mode: pass every tool / operator through (no topology filter). */
  return OPERATOR_PASS_THROUGH;
}

static void edit_elements_edit_cancel(bContext *C, wmOperator * /*op*/)
{
  if (geometry_edit::session_is_active()) {
    geometry_edit::session_cancel(C);
  }
  edit_bridge() = {};
}

static bool edit_elements_edit_poll(bContext *C)
{
  if (geometry_edit::session_is_active() || edit_bridge().active) {
    return false;
  }
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || !ED_node_is_geometry(snode)) {
    return false;
  }
  return find_single_selected_edit_elements_node(*snode->edittree) != nullptr;
}

void NODE_OT_edit_elements_edit(wmOperatorType *ot)
{
  ot->name = "Edit Elements";
  ot->idname = "NODE_OT_edit_elements_edit";
  ot->description =
      "Enter free Edit Mode on a temporary copy of the cooked input geometry: its mesh, "
      "curves, point cloud, or instances as one point each. All edit tools are allowed "
      "(extrude, knife, delete, …). "
      "Esc / Enter / Tab writes the full result into the Edit Elements node";

  ot->invoke = edit_elements_edit_invoke;
  ot->modal = edit_elements_edit_modal;
  ot->cancel = edit_elements_edit_cancel;
  ot->poll = edit_elements_edit_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** Resolve target from node_id (set by body UI) or selection fallback. */
static bNode *find_edit_elements_node_for_op(bNodeTree &ntree, const wmOperator *op)
{
  const int node_id = RNA_int_get(op->ptr, "node_id");
  if (node_id != 0) {
    if (bNode *node = ntree.node_by_id(node_id)) {
      if (nodes::is_edit_elements_node(*node)) {
        return node;
      }
    }
  }
  if (bNode *node = find_single_selected_edit_elements_node(ntree)) {
    return node;
  }
  if (bNode *active = bke::node_get_active(ntree)) {
    if (nodes::is_edit_elements_node(*active)) {
      return active;
    }
  }
  return nullptr;
}

static wmOperatorStatus edit_elements_reset_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return OPERATOR_CANCELLED;
  }
  bNode *node = find_edit_elements_node_for_op(*snode->edittree, op);
  if (!node) {
    BKE_report(op->reports, RPT_ERROR, "Select an Edit Elements node");
    return OPERATOR_CANCELLED;
  }

  ID *old_result = node->id;
  /* Also clears legacy positions. */
  nodes::edit_elements_set_result(*node, nullptr, bke::GeometryComponent::Type::Mesh);
  if (old_result && old_result->us == 0) {
    BKE_id_delete(CTX_data_main(C), old_result);
  }
  nodes::edit_elements_clear_cache(*snode->edittree, *node);
  if (snode->edittree) {
    nodes::edit_elements_clear_cache(*DEG_get_original(snode->edittree), *node);
  }

  Main *bmain = CTX_data_main(C);
  BKE_ntree_update_tag_node_property(snode->edittree, node);
  BKE_main_ensure_invariants(*bmain, snode->edittree->id);
  DEG_id_tag_update(&snode->edittree->id, ID_RECALC_NTREE_OUTPUT);
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, snode->edittree);
  WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, nullptr);
  return OPERATOR_FINISHED;
}

void NODE_OT_edit_elements_reset(wmOperatorType *ot)
{
  ot->name = "Reset Edit Elements";
  ot->idname = "NODE_OT_edit_elements_reset";
  ot->description = "Clear stored edit result / positions and runtime geometry cache";

  ot->exec = edit_elements_reset_exec;
  /* No poll so draw_buttons always paints the Reset button; exec validates the node. */
  ot->poll = nullptr;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* Written by node body UI so the button always targets its own node. */
  PropertyRNA *prop = RNA_def_int(ot->srna,
                                  "node_id",
                                  0,
                                  INT_MIN,
                                  INT_MAX,
                                  "Node",
                                  "Edit Elements node identifier (0 = use selection)",
                                  INT_MIN,
                                  INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

}  // namespace blender::ed::space_node
