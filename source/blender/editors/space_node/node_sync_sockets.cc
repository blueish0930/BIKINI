/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 */

#include <fmt/format.h>

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_enums.h"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "BKE_context.hh"
#include "BKE_main_invariants.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_wm_runtime.hh"

#include "BLI_utildefines.hh"

#include "BLT_translation.hh"

#include "DEG_depsgraph.hh"

#include "ED_node.hh"
#include "ED_screen.hh"

#include "NOD_geometry_nodes_output_cache.hh"
#include "NOD_sync_sockets.hh"

#include "UI_interface_icons.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

static Vector<bNode *> get_nodes_to_sync(bContext &C, PointerRNA *ptr)
{
  SpaceNode &snode = *CTX_wm_space_node(&C);
  if (!snode.edittree) {
    return {};
  }

  std::optional<std::string> node_name;
  PropertyRNA *node_name_prop = RNA_struct_find_property(ptr, "node_name");
  if (RNA_property_is_set(ptr, node_name_prop)) {
    node_name = RNA_property_string_get(ptr, node_name_prop);
  }

  Vector<bNode *> nodes_to_sync;
  bNodeTree &tree = *snode.edittree;
  for (bNode *node : tree.all_nodes()) {
    if (node_name.has_value()) {
      if (node->name != node_name) {
        continue;
      }
    }
    else {
      if (!node->is_selected()) {
        continue;
      }
    }
    nodes_to_sync.append(node);
  }
  return nodes_to_sync;
}

static wmOperatorStatus sockets_sync_exec(bContext *C, wmOperator *op)
{
  Main &bmain = *CTX_data_main(C);
  SpaceNode &snode = *CTX_wm_space_node(C);
  if (!snode.edittree) {
    return OPERATOR_CANCELLED;
  }
  Vector<bNode *> nodes_to_sync = get_nodes_to_sync(*C, op->ptr);
  if (nodes_to_sync.is_empty()) {
    return OPERATOR_CANCELLED;
  }
  for (bNode *node : nodes_to_sync) {
    nodes::sync_node(*C, *node, op->reports);
  }
  BKE_main_ensure_invariants(bmain, snode.edittree->id);
  return OPERATOR_FINISHED;
}

static std::string sockets_sync_get_description(bContext *C, wmOperatorType *ot, PointerRNA *ptr)
{
  Vector<bNode *> nodes_to_sync = get_nodes_to_sync(*C, ptr);
  if (nodes_to_sync.size() != 1) {
    return TIP_(ot->description);
  }
  const bNode &node = *nodes_to_sync.first();
  std::string description = nodes::sync_node_description_get(*C, node);
  if (description.empty()) {
    return TIP_(ot->description);
  }
  return description;
}

void NODE_OT_sockets_sync(wmOperatorType *ot)
{
  ot->name = "Sync Sockets";
  ot->idname = "NODE_OT_sockets_sync";
  ot->description = "Update sockets to match what is actually used";
  ot->get_description = sockets_sync_get_description;

  ot->poll = ED_operator_node_editable;
  ot->exec = sockets_sync_exec;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  PropertyRNA *prop;
  prop = RNA_def_string(ot->srna, "node_name", nullptr, 0, "Node Name", nullptr);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
}

Map<int, bool> &node_can_sync_cache_get(SpaceNode &snode)
{
  return snode.runtime->node_can_sync_states;
}

static bNode *output_cache_operator_node(const bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (snode == nullptr || snode->edittree == nullptr) {
    return nullptr;
  }
  if (snode->edittree->type != NTREE_GEOMETRY) {
    return nullptr;
  }
  const std::string node_name = RNA_string_get(op->ptr, "node_name");
  return bke::node_find_node_by_name(*snode->edittree, node_name);
}

static wmOperatorStatus output_cache_force_eval_invoke(bContext *C,
                                                       wmOperator *op,
                                                       const wmEvent * /*event*/)
{
  return WM_operator_confirm_ex(
      C,
      op,
      IFACE_("Force Re-evaluate"),
      TIP_("Discard this node's cached output and recook?\n"
           "The old cache is freed first so it does not keep using memory."),
      IFACE_("Re-evaluate"),
      ui::AlertIcon::Question,
      false);
}

static wmOperatorStatus output_cache_force_eval_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  bNode *node = output_cache_operator_node(C, op);
  if (snode == nullptr || node == nullptr) {
    return OPERATOR_CANCELLED;
  }
  bNodeTree &tree = *snode->edittree;

  nodes::geometry_nodes_output_cache_discard_node(&tree, node);
  nodes::geometry_nodes_output_cache_tag_node(&tree, node);
  nodes::geometry_nodes_output_cache_propagate(tree);

  DEG_id_tag_update(&tree.id, ID_RECALC_NTREE_OUTPUT);
  if (std::optional<ObjectAndModifier> om = get_geometry_nodes_modifier_for_node_editor(*snode)) {
    DEG_id_tag_update(&const_cast<Object *>(om->object)->id, ID_RECALC_GEOMETRY);
  }
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, &tree);
  WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, nullptr);
  return OPERATOR_FINISHED;
}

void NODE_OT_output_cache_force_eval(wmOperatorType *ot)
{
  ot->name = "Force Re-evaluate Cached Node";
  ot->idname = "NODE_OT_output_cache_force_eval";
  ot->description = "Discard this node's cached output and evaluate it again";

  ot->poll = ED_operator_node_editable;
  ot->invoke = output_cache_force_eval_invoke;
  ot->exec = output_cache_force_eval_exec;

  PropertyRNA *prop;
  prop = RNA_def_string(ot->srna, "node_name", nullptr, 0, "Node Name", nullptr);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
}

static wmOperatorStatus output_cache_toggle_exec(bContext *C, wmOperator *op);

static bool output_cache_toggle_skip_confirm(const bContext *C, const wmEvent *event)
{
  if (event && (event->modifier & (KM_CTRL | KM_OSKEY))) {
    return true;
  }
  /* UI buttons invoke the operator without the click event; read the live window state. */
  if (const wmWindow *win = CTX_wm_window(C)) {
    if (win->runtime && win->runtime->eventstate) {
      return (win->runtime->eventstate->modifier & (KM_CTRL | KM_OSKEY)) != 0;
    }
  }
  return false;
}

static wmOperatorStatus output_cache_toggle_invoke(bContext *C,
                                                   wmOperator *op,
                                                   const wmEvent *event)
{
  bNode *node = output_cache_operator_node(C, op);
  if (node == nullptr) {
    return OPERATOR_CANCELLED;
  }
  if (output_cache_toggle_skip_confirm(C, event)) {
    return output_cache_toggle_exec(C, op);
  }
  if (node->flag & NODE_OUTPUT_CACHE) {
    return WM_operator_confirm_ex(
        C,
        op,
        IFACE_("Disable Output Cache"),
        TIP_("Stop caching this node's output?\nThe stored data will be freed."),
        IFACE_("Disable"),
        ui::AlertIcon::Question,
        false);
  }
  return WM_operator_confirm_ex(
      C,
      op,
      IFACE_("Enable Output Cache"),
      TIP_("Cache this node's output in memory?\n"
           "Uses extra RAM — only enable on expensive nodes."),
      IFACE_("Enable"),
      ui::AlertIcon::Warning,
      false);
}

static wmOperatorStatus output_cache_toggle_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  bNode *node = output_cache_operator_node(C, op);
  if (snode == nullptr || node == nullptr) {
    return OPERATOR_CANCELLED;
  }
  bNodeTree &tree = *snode->edittree;
  const bool enable = (node->flag & NODE_OUTPUT_CACHE) == 0;

  nodes::geometry_nodes_output_cache_discard_node(&tree, node);
  SET_FLAG_FROM_TEST(node->flag, enable, NODE_OUTPUT_CACHE);
  BKE_ntree_update_tag_node_property(&tree, node);

  if (enable) {
    nodes::geometry_nodes_output_cache_tag_node(&tree, node);
    nodes::geometry_nodes_output_cache_propagate(tree);
    DEG_id_tag_update(&tree.id, ID_RECALC_NTREE_OUTPUT);
    if (std::optional<ObjectAndModifier> om = get_geometry_nodes_modifier_for_node_editor(*snode))
    {
      DEG_id_tag_update(&const_cast<Object *>(om->object)->id, ID_RECALC_GEOMETRY);
    }
  }

  BKE_main_ensure_invariants(*CTX_data_main(C), tree.id);
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, &tree);
  WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, nullptr);
  return OPERATOR_FINISHED;
}

void NODE_OT_output_cache_toggle(wmOperatorType *ot)
{
  ot->name = "Toggle Output Cache";
  ot->idname = "NODE_OT_output_cache_toggle";
  ot->description =
      "Enable or disable in-memory caching of this node's output. Uses extra memory. "
      "Ctrl-click skips the confirmation";

  ot->poll = ED_operator_node_editable;
  ot->invoke = output_cache_toggle_invoke;
  ot->exec = output_cache_toggle_exec;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  PropertyRNA *prop;
  prop = RNA_def_string(ot->srna, "node_name", nullptr, 0, "Node Name", nullptr);
  RNA_def_property_flag(prop, PROP_SKIP_SAVE);
}

}  // namespace blender::ed::space_node
