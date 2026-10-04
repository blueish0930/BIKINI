/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 *
 * Image Process Paint: force-eval Color input, bake canvas, Image Paint; Esc applies.
 */

#include "BKE_context.hh"
#include "BKE_image.hh"
#include "BKE_main.hh"
#include "BKE_main_invariants.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_tree_update.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"

#include "BLI_listbase.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_vector.hh"

#include "DNA_image_types.h"
#include "DNA_node_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "DEG_depsgraph.hh"

#include "ED_image.hh"
#include "ED_node.hh"
#include "ED_node_c.hh"
#include "ED_screen.hh"

#include "COM_undefined_node_operation.hh"

#include "NOD_image.hh"
#include "NOD_image_paint.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "node_intern.hh"

namespace blender::ed::space_node {

struct PaintEditSession {
  bNodeTree *ntree = nullptr;
  int node_identifier = 0;
  ScrArea *area = nullptr;
  Image *image = nullptr;
};

static PaintEditSession &session()
{
  static PaintEditSession s;
  return s;
}

static void session_clear()
{
  session() = {};
}

static bNode *find_single_selected_paint_node(bNodeTree &ntree)
{
  bNode *found = nullptr;
  for (bNode *node : ntree.all_nodes()) {
    if (!(node->flag & NODE_SELECT)) {
      continue;
    }
    if (!nodes::is_paint_node(*node)) {
      /* Other selected node types are ok — only require exactly one Paint among selection. */
      continue;
    }
    if (found) {
      return nullptr;
    }
    found = node;
  }
  return found;
}

static int2 image_process_resolution(const SpaceNode *snode)
{
  if (snode) {
    return int2(math::max(1, snode->image_resolution[0]),
                math::max(1, snode->image_resolution[1]));
  }
  return int2(1024, 1024);
}

static void recook_image_tree(bContext *C, bNodeTree *ntree, const int2 resolution)
{
  if (!C || !ntree || ntree->type != NTREE_IMAGE) {
    return;
  }
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  if (!bmain || !scene) {
    return;
  }
  bool used_gpu = false;
  ntreeImageNodesEvaluate(*bmain, *scene, *ntree, &used_gpu, resolution);
  /* Do not broadcast NC_NODE|NA_EDITED here — that re-dirties the whole Image Process tree
   * and can re-enter a full cook (Rasterize/Fluid) right after Paint finishes. */
  WM_main_add_notifier(NC_SPACE | ND_SPACE_NODE_VIEW, nullptr);
}

static bool bake_canvas_from_input(bContext *C,
                                   SpaceNode *snode,
                                   bNodeTree &ntree,
                                   bNode &node,
                                   Image &ima,
                                   ReportList *reports)
{
  if (nodes::image_paint_uses_canvas(node)) {
    return true;
  }

  const int2 resolution = image_process_resolution(snode);
  Vector<float> rgba;
  int2 size = resolution;

  /* Fast path: reuse Color bake from the last tree cook — do not re-evaluate (avoids
   * Rasterize/Fluid/sim fill on Enter Paint). */
  if (nodes::image_paint_get_cached_input(ntree, node.identifier, rgba, size)) {
    if (size.x >= 1 && size.y >= 1 && rgba.size() >= size_t(size.x) * size.y * 4) {
      if (!nodes::image_paint_write_canvas(ima, rgba, size)) {
        if (reports) {
          BKE_report(reports, RPT_ERROR, "Failed to bake Color input into paint canvas");
        }
        return false;
      }
      return true;
    }
  }

  /* Color unlinked → solid socket default, no tree evaluate. */
  ntree.ensure_topology_cache();
  const bNodeSocket *color_sock = bke::node_find_enabled_input_socket(node, "Color");
  const bool color_linked = color_sock && color_sock->is_logically_linked();
  if (!color_linked) {
    size = resolution;
    size.x = math::clamp(size.x, 1, 8192);
    size.y = math::clamp(size.y, 1, 8192);
    rgba.reinitialize(size_t(size.x) * size.y * 4);
    float4 c(0.0f, 0.0f, 0.0f, 1.0f);
    if (color_sock && color_sock->type == SOCK_RGBA) {
      const float *v = color_sock->default_value_typed<bNodeSocketValueRGBA>()->value;
      c = float4(v[0], v[1], v[2], v[3]);
    }
    for (int64_t i = 0; i < int64_t(size.x) * size.y; i++) {
      rgba[i * 4 + 0] = c.x;
      rgba[i * 4 + 1] = c.y;
      rgba[i * 4 + 2] = c.z;
      rgba[i * 4 + 3] = c.w;
    }
  }
  else {
    Main *bmain = CTX_data_main(C);
    Scene *scene = CTX_data_scene(C);
    if (bmain && scene) {
      /* Exclusive Paint schedule — never full tree (Rasterize etc.). Also skips sim fill. */
      compositor::image_process_clear_terminal_filter();
      compositor::image_process_set_paint_force_schedule(ntree.id.session_uid, node.identifier);
      bool used_gpu = false;
      ntreeImageNodesEvaluate(*bmain, *scene, ntree, &used_gpu, resolution);
      compositor::image_process_clear_paint_force_schedule();
    }

    if (!nodes::image_paint_get_cached_input(ntree, node.identifier, rgba, size)) {
      size = resolution;
      size.x = math::clamp(size.x, 1, 8192);
      size.y = math::clamp(size.y, 1, 8192);
      rgba.reinitialize(size_t(size.x) * size.y * 4);
      rgba.fill(0.0f);
      for (int64_t i = 0; i < int64_t(size.x) * size.y; i++) {
        rgba[i * 4 + 3] = 1.0f;
      }
      if (reports) {
        BKE_report(reports,
                   RPT_WARNING,
                   "Paint: Color input empty after force evaluate — blank canvas");
      }
    }
  }

  if (size.x < 1 || size.y < 1 || rgba.size() < size_t(size.x) * size.y * 4) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Paint bake: invalid pixel buffer");
    }
    return false;
  }

  if (!nodes::image_paint_write_canvas(ima, rgba, size)) {
    if (reports) {
      BKE_report(reports, RPT_ERROR, "Failed to bake Color input into paint canvas");
    }
    return false;
  }
  return true;
}

static bNode *find_paint_node_for_edit(bNodeTree &ntree, const wmOperator *op)
{
  const int node_id = RNA_int_get(op->ptr, "node_id");
  if (node_id != 0) {
    if (bNode *node = ntree.node_by_id(node_id)) {
      if (nodes::is_paint_node(*node)) {
        return node;
      }
    }
  }
  if (bNode *node = find_single_selected_paint_node(ntree)) {
    return node;
  }
  if (bNode *active = bke::node_get_active(ntree)) {
    if (nodes::is_paint_node(*active)) {
      return active;
    }
  }
  return nullptr;
}

static bool session_area_still_valid(bContext *C)
{
  ScrArea *area = session().area;
  if (!area || !C) {
    return false;
  }
  bScreen *screen = CTX_wm_screen(C);
  if (!screen) {
    return false;
  }
  for (ScrArea *sa = static_cast<ScrArea *>(screen->areabase.first()); sa; sa = sa->next) {
    if (sa == area) {
      return true;
    }
  }
  return false;
}

static wmOperatorStatus paint_edit_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  /* Always clear stale force-schedule from a previous failed Edit. */
  compositor::image_process_clear_paint_force_schedule();

  /* Recover stuck session (crash / cancelled space switch left area pointer). */
  if (session().area != nullptr) {
    if (session_area_still_valid(C)) {
      BKE_report(op->reports, RPT_WARNING, "Already in Paint edit session — press Esc to finish");
      return OPERATOR_CANCELLED;
    }
    session_clear();
  }

  SpaceNode *snode = CTX_wm_space_node(C);
  ScrArea *area = CTX_wm_area(C);
  if (!snode || !snode->edittree || !ED_node_is_image(snode) || !area) {
    return OPERATOR_CANCELLED;
  }

  bNodeTree &ntree = *snode->edittree;
  /* Prefer root Image tree for force-schedule session_uid matching. */
  bNodeTree *root_tree = snode->nodetree ? snode->nodetree : &ntree;
  if (root_tree->type != NTREE_IMAGE) {
    root_tree = &ntree;
  }

  bNode *node = find_paint_node_for_edit(ntree, op);
  if (!node) {
    BKE_report(op->reports, RPT_ERROR, "Select a Paint node");
    return OPERATOR_CANCELLED;
  }

  /* Paint must live on the tree we evaluate (root for Showcase). Nested-group Paint:
   * force-schedule only searches the evaluated tree; fall back to blank if missing. */
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  if (!bmain || !scene) {
    return OPERATOR_CANCELLED;
  }

  const int2 resolution = image_process_resolution(snode);
  Image *ima = nodes::image_paint_ensure_image(*bmain, *node, resolution, op->reports);
  if (!ima) {
    return OPERATOR_CANCELLED;
  }

  /* Bake Color into canvas — prefer cache / blank; never full-tree cook. */
  if (!bake_canvas_from_input(C, snode, *root_tree, *node, *ima, op->reports)) {
    compositor::image_process_clear_paint_force_schedule();
    return OPERATOR_CANCELLED;
  }
  compositor::image_process_clear_paint_force_schedule();

  /* Switch this area to Image Editor for painting. */
  ED_area_newspace(C, area, SPACE_IMAGE, true);

  SpaceImage *sima = CTX_wm_space_image(C);
  if (!sima && area->spacetype == SPACE_IMAGE) {
    sima = area->spacedata.first_as<SpaceImage>();
  }
  if (!sima) {
    BKE_report(op->reports, RPT_ERROR, "Failed to open Image Editor for Paint");
    if (area) {
      ED_area_prevspace(C, area);
    }
    return OPERATOR_CANCELLED;
  }

  ED_space_image_set(bmain, sima, ima, false);
  sima->mode = SI_MODE_PAINT;
  sima->flag |= SI_PREVSPACE;
  sima->link_flag |= SPACE_FLAG_TYPE_TEMPORARY;

  if (scene->toolsettings) {
    scene->toolsettings->imapaint.mode = IMAGEPAINT_MODE_IMAGE;
    scene->toolsettings->imapaint.canvas = ima;
  }
  if (wmWindowManager *wm = CTX_wm_manager(C)) {
    ED_space_image_paint_update(bmain, wm, scene);
  }

  session().ntree = root_tree;
  session().node_identifier = node->identifier;
  session().area = area;
  session().image = ima;

  WM_event_add_modal_handler(C, op);
  ED_area_tag_redraw(area);
  WM_main_add_notifier(NC_SPACE | ND_SPACE_IMAGE, nullptr);
  return OPERATOR_RUNNING_MODAL;
}

static void paint_edit_finish(bContext *C, bool restore_space)
{
  PaintEditSession &s = session();
  ScrArea *area = s.area;
  bNodeTree *ntree = s.ntree;
  Image *image = s.image;
  const int node_id = s.node_identifier;

  /* Clear session first so a re-entrant finish cannot loop. */
  session_clear();
  compositor::image_process_clear_paint_force_schedule();

  if (restore_space && area && C) {
    if (area->spacetype == SPACE_IMAGE) {
      if (SpaceImage *sima = area->spacedata.first_as<SpaceImage>()) {
        sima->flag &= ~SI_PREVSPACE;
      }
      ED_area_prevspace(C, area);
    }
  }

  if (ntree && node_id != 0) {
    ntree->ensure_topology_cache();
    if (bNode *node = ntree->node_by_id(node_id)) {
      if (nodes::is_paint_node(*node)) {
        nodes::image_paint_set_use_canvas(*node, true);
        BKE_ntree_update_tag_node_property(ntree, node);
      }
    }
  }

  if (image) {
    DEG_id_tag_update(&image->id, 0);
    WM_main_add_notifier(NC_IMAGE | NA_EDITED, image);
  }

  if (ntree && C) {
    int2 resolution(1024, 1024);
    if (SpaceNode *snode = CTX_wm_space_node(C)) {
      resolution = image_process_resolution(snode);
    }
    recook_image_tree(C, ntree, resolution);
  }
}

static wmOperatorStatus paint_edit_modal(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  if (session().area == nullptr) {
    return OPERATOR_CANCELLED;
  }
  if (event->val == KM_PRESS &&
      ELEM(event->type, EVT_ESCKEY, EVT_RETKEY, EVT_PADENTER, EVT_TABKEY))
  {
    paint_edit_finish(C, true);
    return OPERATOR_FINISHED;
  }
  return OPERATOR_PASS_THROUGH;
}

static void paint_edit_cancel(bContext *C, wmOperator * /*op*/)
{
  paint_edit_finish(C, true);
}

static bool paint_edit_poll(bContext *C)
{
  /* Modal session: keep poll true after area switches to Image Editor (re-checks can cancel
   * the modal if we returned false here while session is active). */
  if (session().area != nullptr) {
    return true;
  }
  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || !ED_node_is_image(snode)) {
    return false;
  }
  /* Button passes node_id — allow poll when any Paint exists, or selection/active. */
  if (find_single_selected_paint_node(*snode->edittree)) {
    return true;
  }
  if (bNode *active = bke::node_get_active(*snode->edittree)) {
    if (nodes::is_paint_node(*active)) {
      return true;
    }
  }
  for (bNode *node : snode->edittree->all_nodes()) {
    if ((node->flag & NODE_SELECT) && nodes::is_paint_node(*node)) {
      return true;
    }
  }
  /* Edit button on node body sets node_id; poll must pass without selection. */
  for (bNode *node : snode->edittree->all_nodes()) {
    if (nodes::is_paint_node(*node)) {
      return true;
    }
  }
  return false;
}

void NODE_OT_paint_edit(wmOperatorType *ot)
{
  ot->name = "Edit Paint";
  ot->idname = "NODE_OT_paint_edit";
  ot->description =
      "Bake Color input and paint in the Image Editor; Esc applies and returns";

  ot->invoke = paint_edit_invoke;
  ot->modal = paint_edit_modal;
  ot->cancel = paint_edit_cancel;
  ot->poll = paint_edit_poll;
  /* No OPTYPE_UNDO: switching Space type mid-undo push is fragile and caused crashes. */
  ot->flag = OPTYPE_REGISTER;

  PropertyRNA *prop = RNA_def_int(ot->srna,
                                  "node_id",
                                  0,
                                  INT_MIN,
                                  INT_MAX,
                                  "Node",
                                  "Paint node identifier (0 = use selection)",
                                  INT_MIN,
                                  INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

static bNode *find_paint_node_for_op(bNodeTree &ntree, const wmOperator *op)
{
  return find_paint_node_for_edit(ntree, op);
}

static wmOperatorStatus paint_reset_exec(bContext *C, wmOperator *op)
{
  compositor::image_process_clear_paint_force_schedule();

  SpaceNode *snode = CTX_wm_space_node(C);
  if (!snode || !snode->edittree || !ED_node_is_image(snode)) {
    return OPERATOR_CANCELLED;
  }
  bNode *node = find_paint_node_for_op(*snode->edittree, op);
  if (!node) {
    BKE_report(op->reports, RPT_ERROR, "Select a Paint node");
    return OPERATOR_CANCELLED;
  }

  nodes::image_paint_set_use_canvas(*node, false);

  Main *bmain = CTX_data_main(C);
  BKE_ntree_update_tag_node_property(snode->edittree, node);
  if (bmain) {
    BKE_main_ensure_invariants(*bmain, snode->edittree->id);
  }
  DEG_id_tag_update(&snode->edittree->id, ID_RECALC_NTREE_OUTPUT);

  recook_image_tree(C, snode->edittree, image_process_resolution(snode));
  WM_event_add_notifier(C, NC_NODE | NA_EDITED, snode->edittree);
  return OPERATOR_FINISHED;
}

void NODE_OT_paint_reset(wmOperatorType *ot)
{
  ot->name = "Reset Paint";
  ot->idname = "NODE_OT_paint_reset";
  ot->description = "Clear painted canvas and pass the Color input through again";

  ot->exec = paint_reset_exec;
  ot->poll = paint_edit_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  PropertyRNA *prop = RNA_def_int(ot->srna,
                                  "node_id",
                                  0,
                                  INT_MIN,
                                  INT_MAX,
                                  "Node",
                                  "Paint node identifier (0 = use selection)",
                                  INT_MIN,
                                  INT_MAX);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

}  // namespace blender::ed::space_node
