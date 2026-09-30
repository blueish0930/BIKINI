/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup ed_node
 *
 * Select Elements node → selection session (Houdini-style path).
 *
 * - Resolve cooked Geometry from the node's Geometry input (upstream cook).
 * - Spawn a temporary mesh object and enter **native Mesh Edit Mode**.
 * - Selection-only: topology changes are undone; no modeling.
 * - Esc / Enter / Tab confirms and writes indices to the node.
 */

#include "BKE_compute_contexts.hh"
#include "BKE_context.hh"
#include "BKE_curves.h"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_layer.hh"
#include "BKE_main.hh"
#include "BKE_main_invariants.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_object.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"
#include "BKE_screen.hh"

#include "GEO_realize_instances.hh"

#include "DEG_depsgraph_build.hh"

#include <cstdlib>

#include "BLI_listbase.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_geometry_edit.hh"
#include "ED_node.hh"
#include "ED_object.hh"
#include "ED_screen.hh"
#include "ED_select_utils.hh"

#include "BKE_screen.hh"

#include "DNA_space_types.h"
#include "DNA_userdef_types.h"

#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include <algorithm>
#include <cmath>

#include "MEM_guardedalloc.h"

#include "NOD_geo_select_elements.hh"
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

/* -------------------------------------------------------------------- */
/** \name Helpers
 * \{ */

static bNode *find_node_by_identifier(bNodeTree &ntree, const int32_t identifier)
{
  for (bNode *node : ntree.all_nodes()) {
    if (node->identifier == identifier) {
      return node;
    }
  }
  return nullptr;
}

static bNode *find_single_selected_select_elements_node(bNodeTree &ntree)
{
  VectorSet<bNode *> selected = get_selected_nodes(ntree);
  if (selected.size() != 1) {
    return nullptr;
  }
  bNode *node = *selected.begin();
  if (!nodes::is_select_elements_node(*node)) {
    return nullptr;
  }
  return node;
}

/**
 * Upstream visual geometry (input cache → object visual geo → original mesh).
 * Enter session needs a mesh component.
 */
static std::optional<bke::GeometrySet> resolve_edit_geometry(bContext *C,
                                                             SpaceNode &snode,
                                                             bNode &node,
                                                             ReportList *reports,
                                                             Object **r_transform_from)
{
  std::optional<bke::GeometrySet> geo = resolve_visual_geometry_for_elements_node(
      C, snode, node, reports, r_transform_from);
  if (!geo) {
    return std::nullopt;
  }
  if (!geo->has_mesh() || geo->get_mesh()->verts_num == 0) {
    BKE_report(reports,
               RPT_ERROR,
               "Select Elements Enter needs a mesh (one geometry type). "
               "Connect a mesh-producing previous node");
    return std::nullopt;
  }
  return geo;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Enter Geometry Edit Session from Select Elements
 * \{ */

/** Maps Geometry Edit session back to the Select Elements node for commit. */
struct SelectElementsSessionBridge {
  bNodeTree *ntree = nullptr;
  int32_t node_identifier = 0;
  bool active = false;
};

static SelectElementsSessionBridge &bridge()
{
  static SelectElementsSessionBridge b;
  return b;
}

static bool commit_to_node_from_bridge(bContext *C)
{
  SelectElementsSessionBridge b_copy = bridge();
  bridge() = {};

  if (!b_copy.ntree || b_copy.node_identifier == 0) {
    if (geometry_edit::session_is_active()) {
      geometry_edit::session_cancel(C);
    }
    return false;
  }

  bke::AttrDomain domain = bke::AttrDomain::Point;
  Vector<int> indices;
  if (!geometry_edit::session_commit_selection(C, domain, indices)) {
    return false;
  }

  bNode *node = find_node_by_identifier(*b_copy.ntree, b_copy.node_identifier);
  if (!node || !nodes::is_select_elements_node(*node)) {
    return false;
  }

  nodes::select_elements_set_indices(*node, domain, indices);

  Main *bmain = CTX_data_main(C);
  if (!bmain) {
    return false;
  }
  bNodeTree &tree = node->owner_tree();
  BKE_ntree_update_tag_node_property(&tree, node);
  BKE_main_ensure_invariants(*bmain, tree.id);
  DEG_id_tag_update(&tree.id, ID_RECALC_NTREE_OUTPUT);
  WM_main_add_notifier(NC_NODE | NA_EDITED, &tree);
  WM_main_add_notifier(NC_OBJECT | ND_MODIFIER, nullptr);
  return true;
}

static wmOperatorStatus select_elements_edit_invoke(bContext *C,
                                                    wmOperator *op,
                                                    const wmEvent * /*event*/)
{
  if (geometry_edit::session_is_active() || bridge().active) {
    BKE_report(op->reports, RPT_WARNING, "Already in Select Elements session");
    return OPERATOR_CANCELLED;
  }

  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || !ED_node_is_geometry(snode)) {
    return OPERATOR_CANCELLED;
  }

  bNodeTree &ntree = *snode->edittree;
  bNode *node = find_single_selected_select_elements_node(ntree);
  if (!node) {
    return OPERATOR_PASS_THROUGH | OPERATOR_CANCELLED;
  }

  Object *transform_from = nullptr;
  std::optional<bke::GeometrySet> geometry_opt = resolve_edit_geometry(
      C, *snode, *node, op->reports, &transform_from);
  if (!geometry_opt) {
    return OPERATOR_CANCELLED;
  }
  const bke::GeometrySet &geometry = *geometry_opt;

  const Mesh *mesh = geometry.get_mesh();
  if (!mesh) {
    BKE_report(op->reports,
               RPT_ERROR,
               "Select Elements input has no mesh "
               "(connect Geometry from previous node's cook output)");
    return OPERATOR_CANCELLED;
  }

  const NodeGeometrySelectElements &storage =
      *static_cast<const NodeGeometrySelectElements *>(node->storage);
  const bke::AttrDomain domain = bke::AttrDomain(storage.domain);
  const Span<int> indices(storage.indices, storage.indices_num);

  float4x4 object_to_world = float4x4::identity();
  if (transform_from) {
    object_to_world = transform_from->object_to_world();
  }

  if (!geometry_edit::session_begin(C,
                                    &ntree,
                                    node->identifier,
                                    transform_from,
                                    *mesh,
                                    object_to_world,
                                    domain,
                                    indices,
                                    geometry_edit::SessionKind::SelectOnly,
                                    op->reports))
  {
    return OPERATOR_CANCELLED;
  }

  bridge().active = true;
  bridge().ntree = &ntree;
  bridge().node_identifier = node->identifier;

  WM_global_report(RPT_INFO,
                   "Select Elements: selection only (gizmos off). "
                   "Mesh select tools · 1/2/3 domain · Esc/Enter/Tab confirm");

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

/**
 * Modal keeps the session alive while native Mesh Edit keymaps do selection.
 * - PASS_THROUGH for navigation + selection tools.
 * - Block modeling keys (G/R/S/E/X/…).
 * - Undo topology changes if something still slipped through.
 * - Esc / Enter / Tab commits indices to the node.
 */
static wmOperatorStatus select_elements_edit_modal(bContext *C,
                                                   wmOperator * /*op*/,
                                                   const wmEvent *event)
{
  if (!geometry_edit::session_is_active()) {
    bridge() = {};
    return OPERATOR_CANCELLED;
  }

  /* Confirm: Esc / Enter / Tab (leave edit and write selection).
   * Do not treat RMB as cancel — pass it through for native Edit Mode select / context menu. */
  if (event->val == KM_PRESS &&
      ELEM(event->type, EVT_ESCKEY, EVT_RETKEY, EVT_PADENTER, EVT_TABKEY))
  {
    if (!geometry_edit::session_enforce_constraints(C)) {
      geometry_edit::session_cancel(C);
      bridge() = {};
      return OPERATOR_CANCELLED;
    }
    commit_to_node_from_bridge(C);
    return OPERATOR_FINISHED;
  }

  if (event->val == KM_PRESS) {
    /* Domain shortcuts (also pass native 1/2/3 via toolsettings when we set domain). */
    if (event->type == EVT_ONEKEY && (event->modifier & (KM_CTRL | KM_SHIFT | KM_ALT)) == 0) {
      geometry_edit::session_set_domain(bke::AttrDomain::Point);
      if (Scene *scene = CTX_data_scene(C)) {
        scene->toolsettings->selectmode = SCE_SELECT_VERTEX;
      }
      return OPERATOR_RUNNING_MODAL;
    }
    if (event->type == EVT_TWOKEY && (event->modifier & (KM_CTRL | KM_SHIFT | KM_ALT)) == 0) {
      geometry_edit::session_set_domain(bke::AttrDomain::Edge);
      if (Scene *scene = CTX_data_scene(C)) {
        scene->toolsettings->selectmode = SCE_SELECT_EDGE;
      }
      return OPERATOR_RUNNING_MODAL;
    }
    if (event->type == EVT_THREEKEY && (event->modifier & (KM_CTRL | KM_SHIFT | KM_ALT)) == 0) {
      geometry_edit::session_set_domain(bke::AttrDomain::Face);
      if (Scene *scene = CTX_data_scene(C)) {
        scene->toolsettings->selectmode = SCE_SELECT_FACE;
      }
      return OPERATOR_RUNNING_MODAL;
    }

    /* Block modeling / transform / delete — selection session only. */
    const bool no_mod = (event->modifier & (KM_CTRL | KM_SHIFT | KM_ALT | KM_OSKEY)) == 0;
    const bool ctrl = (event->modifier & KM_CTRL) != 0;
    const bool block_model = no_mod &&
                             (ELEM(event->type,
                                   EVT_GKEY,
                                   EVT_RKEY,
                                   EVT_SKEY,
                                   EVT_EKEY,
                                   EVT_XKEY,
                                   EVT_YKEY,
                                   EVT_ZKEY,
                                   EVT_FKEY,
                                   EVT_IKEY,
                                   EVT_KKEY,
                                   EVT_PKEY,
                                   EVT_VKEY) ||
                              ELEM(event->type,
                                   EVT_DKEY,
                                   EVT_UKEY,
                                   EVT_WKEY,
                                   EVT_HKEY,
                                   EVT_JKEY,
                                   EVT_NKEY,
                                   EVT_MKEY,
                                   EVT_OKEY,
                                   EVT_TKEY,
                                   EVT_DELKEY));
    if (block_model || (ctrl && ELEM(event->type, EVT_BKEY, EVT_VKEY, EVT_XKEY, EVT_DKEY, EVT_TKEY)))
    {
      WM_global_report(RPT_WARNING, "Select Elements: selection only");
      return OPERATOR_RUNNING_MODAL;
    }
  }

  /* After other operators: reject topology or position moves (gizmo/G). */
  if (event->val == KM_RELEASE || event->type == MOUSEMOVE) {
    if (!geometry_edit::session_topology_ok() || !geometry_edit::session_positions_ok()) {
      if (!geometry_edit::session_enforce_constraints(C)) {
        geometry_edit::session_cancel(C);
        bridge() = {};
        return OPERATOR_CANCELLED;
      }
    }
  }

  /* Let native mesh select + view navigation run. */
  return OPERATOR_PASS_THROUGH;
}

static void select_elements_edit_cancel(bContext *C, wmOperator * /*op*/)
{
  if (geometry_edit::session_is_active()) {
    geometry_edit::session_cancel(C);
  }
  bridge() = {};
}

static bool select_elements_edit_poll(bContext *C)
{
  if (geometry_edit::session_is_active() || bridge().active) {
    return false;
  }
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || !ED_node_is_geometry(snode)) {
    return false;
  }
  return find_single_selected_select_elements_node(*snode->edittree) != nullptr;
}

void NODE_OT_select_elements_edit(wmOperatorType *ot)
{
  ot->name = "Edit Select Elements";
  ot->idname = "NODE_OT_select_elements_edit";
  ot->description =
      "Enter native Edit Mode on the Select Elements node's cooked input mesh "
      "(selection only). Esc / Enter / Tab confirms selection into the node";

  ot->invoke = select_elements_edit_invoke;
  ot->modal = select_elements_edit_modal;
  ot->cancel = select_elements_edit_cancel;
  ot->poll = select_elements_edit_poll;

  /* Not BLOCKING: native select/nav need events. */
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Scripting helper: set indices without viewport session
 * \{ */

static Vector<int> parse_indices_csv(const char *csv)
{
  Vector<int> indices;
  if (!csv || csv[0] == '\0') {
    return indices;
  }
  const char *p = csv;
  while (*p) {
    while (*p == ' ' || *p == ',' || *p == ';') {
      p++;
    }
    if (*p == '\0') {
      break;
    }
    char *end = nullptr;
    const long value = std::strtol(p, &end, 10);
    if (end == p) {
      break;
    }
    indices.append(int(value));
    p = end;
  }
  return indices;
}

static wmOperatorStatus select_elements_set_indices_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return OPERATOR_CANCELLED;
  }
  bNode *node = find_single_selected_select_elements_node(*snode->edittree);
  if (!node) {
    BKE_report(op->reports, RPT_ERROR, "Select a single Select Elements node");
    return OPERATOR_CANCELLED;
  }

  const int domain_i = RNA_enum_get(op->ptr, "domain");
  const std::string csv = RNA_string_get(op->ptr, "indices");
  Vector<int> indices = parse_indices_csv(csv.c_str());

  nodes::select_elements_set_indices(*node, bke::AttrDomain(domain_i), indices);

  Main *bmain = CTX_data_main(C);
  BKE_ntree_update_tag_node_property(snode->edittree, node);
  BKE_main_ensure_invariants(*bmain, snode->edittree->id);
  DEG_id_tag_update(&snode->edittree->id, ID_RECALC_NTREE_OUTPUT);
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, snode->edittree);
  WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, nullptr);
  return OPERATOR_FINISHED;
}

static bool select_elements_set_indices_poll(bContext *C)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || !ED_node_is_geometry(snode)) {
    return false;
  }
  return find_single_selected_select_elements_node(*snode->edittree) != nullptr;
}

void NODE_OT_select_elements_set_indices(wmOperatorType *ot)
{
  ot->name = "Set Select Elements Indices";
  ot->idname = "NODE_OT_select_elements_set_indices";
  ot->description = "Set Select Elements node indices from a CSV string (scripting/tests)";

  ot->exec = select_elements_set_indices_exec;
  ot->poll = select_elements_set_indices_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  static const EnumPropertyItem domain_items[] = {
      {int(bke::AttrDomain::Point), "POINT", 0, "Point", ""},
      {int(bke::AttrDomain::Edge), "EDGE", 0, "Edge", ""},
      {int(bke::AttrDomain::Face), "FACE", 0, "Face", ""},
      {int(bke::AttrDomain::Curve), "CURVE", 0, "Spline", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };
  RNA_def_enum(ot->srna, "domain", domain_items, int(bke::AttrDomain::Point), "Domain", "");
  /* Non-empty default + positive maxlen avoids rna.define ERROR on empty string. */
  RNA_def_string(ot->srna, "indices", " ", 1024, "Indices", "Comma-separated element indices");
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Reset stored selection + runtime cache
 * \{ */

/** Resolve target from node_id (set by body UI) or selection fallback. */
static bNode *find_select_elements_node_for_op(bNodeTree &ntree, const wmOperator *op)
{
  const int node_id = RNA_int_get(op->ptr, "node_id");
  if (node_id != 0) {
    if (bNode *node = ntree.node_by_id(node_id)) {
      if (nodes::is_select_elements_node(*node)) {
        return node;
      }
    }
  }
  if (bNode *node = find_single_selected_select_elements_node(ntree)) {
    return node;
  }
  if (bNode *active = bke::node_get_active(ntree)) {
    if (nodes::is_select_elements_node(*active)) {
      return active;
    }
  }
  return nullptr;
}

static wmOperatorStatus select_elements_reset_exec(bContext *C, wmOperator *op)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree) {
    return OPERATOR_CANCELLED;
  }
  bNode *node = find_select_elements_node_for_op(*snode->edittree, op);
  if (!node) {
    BKE_report(op->reports, RPT_ERROR, "Select a Select Elements node");
    return OPERATOR_CANCELLED;
  }

  nodes::select_elements_set_indices(*node, bke::AttrDomain::Point, {});
  nodes::select_elements_clear_cache(*snode->edittree, *node);
  if (snode->edittree) {
    nodes::select_elements_clear_cache(*DEG_get_original(snode->edittree), *node);
  }

  Main *bmain = CTX_data_main(C);
  BKE_ntree_update_tag_node_property(snode->edittree, node);
  BKE_main_ensure_invariants(*bmain, snode->edittree->id);
  DEG_id_tag_update(&snode->edittree->id, ID_RECALC_NTREE_OUTPUT);
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, snode->edittree);
  WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, nullptr);
  return OPERATOR_FINISHED;
}

void NODE_OT_select_elements_reset(wmOperatorType *ot)
{
  ot->name = "Reset Select Elements";
  ot->idname = "NODE_OT_select_elements_reset";
  ot->description = "Clear stored selection indices and runtime geometry cache";

  ot->exec = select_elements_reset_exec;
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
                                  "Select Elements node identifier (0 = use selection)",
                                  INT_MIN,
                                  INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

/** \} */

}  // namespace blender::ed::space_node
