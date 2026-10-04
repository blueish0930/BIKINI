/* SPDX-FileCopyrightText: 2008 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spnode
 */

#include "DNA_node_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "BLI_listbase.hh"
#include "BLI_math_base.hh"
#include "BLI_rect.hh"
#include "BLI_string.hh"
#include "BLI_utildefines.hh"

#include "BKE_context.hh"
#include "BKE_image.hh"
#include "BKE_node_runtime.hh"
#include "BKE_screen.hh"
#include "BKE_wm_runtime.hh"

#include "ED_image.hh"
#include "ED_node.hh" /* own include */
#include "ED_screen.hh"
#include "ED_space_api.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include "WM_keymap.hh"
#include "WM_types.hh"

#include "UI_view2d.hh"

#include "MEM_guardedalloc.h"

#include "IMB_colormanagement.hh"
#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "node_intern.hh" /* own include */

namespace blender {

namespace ed::space_node {

/* -------------------------------------------------------------------- */
/** \name Local Functions
 * \{ */

static bool space_node_active_view_poll(bContext *C)
{
  if (!ED_operator_node_active(C)) {
    return false;
  }
  const ARegion *region = CTX_wm_region(C);
  if (!(region && region->regiontype == RGN_TYPE_WINDOW)) {
    return false;
  }
  return true;
}

static bool space_node_composite_active_view_poll(bContext *C)
{
  const ARegion *region = CTX_wm_region(C);
  if (!(region && region->regiontype == RGN_TYPE_WINDOW)) {
    return false;
  }
  if (composite_node_active(C)) {
    return true;
  }
  /* Image editor Viewer backdrop uses the same move/zoom/fit operators. */
  if (ED_operator_node_active(C)) {
    const SpaceNode *snode = CTX_wm_space_node(C);
    if (snode && ED_node_is_image(snode)) {
      return true;
    }
  }
  return false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name View All Operator
 * \{ */

bool space_node_view_flag(
    bContext &C, SpaceNode &snode, ARegion &region, const int node_flag, const int smooth_viewtx)
{
  const float oldwidth = BLI_rctf_size_x(&region.v2d.cur);
  const float oldheight = BLI_rctf_size_y(&region.v2d.cur);

  const float old_aspect = oldwidth / oldheight;

  rctf cur_new;
  BLI_rctf_init_minmax(&cur_new);

  int tot = 0;
  if (snode.edittree) {
    for (const bNode *node : snode.edittree->all_nodes()) {
      if ((node->flag & node_flag) == node_flag) {
        BLI_rctf_union(&cur_new, &node->runtime->draw_bounds);
        tot++;
      }
    }
  }

  if (tot == 0) {
    return false;
  }

  const float width = BLI_rctf_size_x(&cur_new);
  const float height = BLI_rctf_size_y(&cur_new);
  const float new_aspect = width / height;

  if (old_aspect < new_aspect) {
    const float height_new = width / old_aspect;
    cur_new.ymin = cur_new.ymin - height_new / 4.0f;
    cur_new.ymax = cur_new.ymax + height_new / 4.0f;
  }
  else {
    const float width_new = height * old_aspect;
    cur_new.xmin = cur_new.xmin - width_new / 4.0f;
    cur_new.xmax = cur_new.xmax + width_new / 4.0f;
  }

  /* add some padding */
  BLI_rctf_scale(&cur_new, 1.1f);

  ui::view2d_smooth_view(&C, &region, &cur_new, smooth_viewtx);

  return true;
}

static wmOperatorStatus node_view_all_exec(bContext *C, wmOperator *op)
{
  ARegion *region = CTX_wm_region(C);
  SpaceNode *snode = CTX_wm_space_node(C);
  const int smooth_viewtx = WM_operator_smooth_viewtx_get(op);

  /* is this really needed? */
  snode->xof = 0;
  snode->yof = 0;

  if (space_node_view_flag(*C, *snode, *region, 0, smooth_viewtx)) {
    return OPERATOR_FINISHED;
  }
  return OPERATOR_CANCELLED;
}

void NODE_OT_view_all(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Frame All";
  ot->idname = "NODE_OT_view_all";
  ot->description = "Resize view so you can see all nodes";

  /* API callbacks. */
  ot->exec = node_view_all_exec;
  ot->poll = space_node_active_view_poll;

  /* flags */
  ot->flag = 0;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name View Selected Operator
 * \{ */

static wmOperatorStatus node_view_selected_exec(bContext *C, wmOperator *op)
{
  ARegion *region = CTX_wm_region(C);
  SpaceNode *snode = CTX_wm_space_node(C);
  const int smooth_viewtx = WM_operator_smooth_viewtx_get(op);

  if (space_node_view_flag(*C, *snode, *region, NODE_SELECT, smooth_viewtx)) {
    return OPERATOR_FINISHED;
  }
  return OPERATOR_CANCELLED;
}

void NODE_OT_view_selected(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Frame Selected";
  ot->idname = "NODE_OT_view_selected";
  ot->description = "Resize view so you can see selected nodes";

  /* API callbacks. */
  ot->exec = node_view_selected_exec;
  ot->poll = space_node_active_view_poll;

  /* flags */
  ot->flag = 0;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Minimap Navigate Operator
 * \{ */

static void node_minimap_apply_center(bContext *C, const float2 &world)
{
  ARegion *region = CTX_wm_region(C);
  ui::view2d_center_set(&region->v2d, world.x, world.y);
  ED_region_tag_redraw(region);
}

static void node_minimap_zoom_range(SpaceNode &snode,
                                    ARegion &region,
                                    const int mval[2],
                                    const bool zoom_in)
{
  const NodeMinimapLayout layout = node_minimap_layout_get(region, snode);
  float2 world;
  if (!node_minimap_mouse_to_world(layout, mval, true, world)) {
    return;
  }
  const float world_w = math::max(BLI_rctf_size_x(&layout.world), 1e-4f);
  const float world_h = math::max(BLI_rctf_size_y(&layout.world), 1e-4f);
  const float u = (world.x - layout.world.xmin) / world_w;
  const float v = (world.y - layout.world.ymin) / world_h;

  SpaceNode_Runtime &runtime = *snode.runtime;
  if (!runtime.thumbnail_view_custom) {
    runtime.thumbnail_center = {BLI_rctf_cent_x(&layout.fit), BLI_rctf_cent_y(&layout.fit)};
    runtime.thumbnail_zoom = 1.0f;
    runtime.thumbnail_view_custom = true;
  }

  const float factor = zoom_in ? 1.6f : (1.0f / 1.6f);
  const float new_zoom = math::clamp(runtime.thumbnail_zoom * factor, 1.0f, 64.0f);
  if (new_zoom <= 1.001f) {
    runtime.thumbnail_zoom = 1.0f;
    runtime.thumbnail_view_custom = false;
  }
  else {
    runtime.thumbnail_zoom = new_zoom;
    const float new_w = BLI_rctf_size_x(&layout.fit) / new_zoom;
    const float new_h = BLI_rctf_size_y(&layout.fit) / new_zoom;
    runtime.thumbnail_center.x = world.x - (u - 0.5f) * new_w;
    runtime.thumbnail_center.y = world.y - (v - 0.5f) * new_h;
  }
  ED_region_tag_redraw(&region);
}

static wmOperatorStatus node_minimap_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  if (snode == nullptr || region == nullptr) {
    return OPERATOR_PASS_THROUGH;
  }
  const NodeMinimapLayout layout = node_minimap_layout_get(*region, *snode);
  float2 world;
  if (!node_minimap_mouse_to_world(layout, event->mval, false, world)) {
    return OPERATOR_PASS_THROUGH;
  }
  if (ELEM(event->type, WHEELUPMOUSE, WHEELDOWNMOUSE, WHEELINMOUSE, WHEELOUTMOUSE)) {
    const bool zoom_in = ELEM(event->type, WHEELUPMOUSE, WHEELINMOUSE);
    node_minimap_zoom_range(*snode, *region, event->mval, zoom_in);
    return OPERATOR_FINISHED;
  }
  if (event->type == LEFTMOUSE && event->val == KM_PRESS) {
    node_minimap_apply_center(C, world);
    WM_event_add_modal_handler(C, op);
    return OPERATOR_RUNNING_MODAL;
  }
  return OPERATOR_PASS_THROUGH;
}

static wmOperatorStatus node_minimap_modal(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  if (snode == nullptr || region == nullptr) {
    return OPERATOR_CANCELLED;
  }
  switch (event->type) {
    case MOUSEMOVE: {
      const NodeMinimapLayout layout = node_minimap_layout_get(*region, *snode);
      float2 world;
      if (node_minimap_mouse_to_world(layout, event->mval, true, world)) {
        node_minimap_apply_center(C, world);
      }
      return OPERATOR_RUNNING_MODAL;
    }
    case LEFTMOUSE:
      if (event->val == KM_RELEASE) {
        return OPERATOR_FINISHED;
      }
      break;
    case EVT_ESCKEY:
      if (event->val == KM_PRESS) {
        return OPERATOR_CANCELLED;
      }
      break;
    default:
      break;
  }
  return OPERATOR_RUNNING_MODAL;
}

int node_minimap_ui_handler(bContext *C, const wmEvent *event, void * /*userdata*/)
{
  if (event->val != KM_PRESS) {
    return WM_UI_HANDLER_CONTINUE;
  }
  if (!ELEM(event->type,
            LEFTMOUSE,
            WHEELUPMOUSE,
            WHEELDOWNMOUSE,
            WHEELINMOUSE,
            WHEELOUTMOUSE))
  {
    return WM_UI_HANDLER_CONTINUE;
  }
  /* Ctrl+Wheel adjusts hovered number buttons; never steal it for minimap zoom. */
  if (ELEM(event->type, WHEELUPMOUSE, WHEELDOWNMOUSE, WHEELINMOUSE, WHEELOUTMOUSE) &&
      (event->modifier & KM_CTRL))
  {
    return WM_UI_HANDLER_CONTINUE;
  }

  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  if (snode == nullptr || region == nullptr || region->regiontype != RGN_TYPE_WINDOW) {
    return WM_UI_HANDLER_CONTINUE;
  }
  const NodeMinimapLayout layout = node_minimap_layout_get(*region, *snode);
  if (!layout.visible ||
      !BLI_rctf_isect_pt(&layout.widget, float(event->mval[0]), float(event->mval[1])))
  {
    return WM_UI_HANDLER_CONTINUE;
  }

  const wmOperatorStatus result = WM_operator_name_call(
      C, "NODE_OT_minimap_navigate", wm::OpCallContext::InvokeDefault, nullptr, event);
  if (result == OPERATOR_PASS_THROUGH) {
    return WM_UI_HANDLER_CONTINUE;
  }
  return WM_UI_HANDLER_BREAK;
}

void NODE_OT_minimap_navigate(wmOperatorType *ot)
{
  ot->name = "Navigate Thumbnail";
  ot->idname = "NODE_OT_minimap_navigate";
  ot->description =
      "Click to jump the editor to a location; scroll to zoom the thumbnail's visible range";

  ot->invoke = node_minimap_invoke;
  ot->modal = node_minimap_modal;
  ot->poll = space_node_active_view_poll;
  ot->flag = OPTYPE_BLOCKING;
}

void node_minimap_ensure_keymap_priority(wmWindowManager *wm)
{
  if (wm == nullptr) {
    return;
  }
  auto ensure_item = [](wmKeyMap *keymap, const short type) {
    if (keymap == nullptr) {
      return;
    }
    wmKeyMapItem *found = nullptr;
    for (wmKeyMapItem &kmi : keymap->items) {
      if (STREQ(kmi.idname, "NODE_OT_minimap_navigate") && kmi.type == type &&
          kmi.val == KM_PRESS)
      {
        found = &kmi;
        break;
      }
    }
    if (found == nullptr) {
      KeyMapItem_Params params{};
      params.type = type;
      params.value = KM_PRESS;
      params.modifier = 0;
      params.keymodifier = 0;
      params.direction = KM_ANY;
      found = WM_keymap_add_item(keymap, "NODE_OT_minimap_navigate", &params);
    }
    if (found != nullptr) {
      BLI_remlink(&keymap->items, found);
      BLI_addhead(&keymap->items, found);
    }
  };
  auto ensure_all = [&](wmKeyMap *keymap) {
    ensure_item(keymap, WHEELUPMOUSE);
    ensure_item(keymap, WHEELDOWNMOUSE);
    ensure_item(keymap, LEFTMOUSE);
  };
  for (wmKeyConfig &keyconf : wm->runtime->keyconfigs) {
    ensure_all(WM_keymap_list_find(&keyconf.keymaps, "Node Editor", SPACE_NODE, RGN_TYPE_WINDOW));
  }
  if (wm->runtime->defaultconf) {
    ensure_all(WM_keymap_list_find(
        &wm->runtime->defaultconf->keymaps, "Node Editor", SPACE_NODE, RGN_TYPE_WINDOW));
  }
  if (wm->runtime->userconf) {
    ensure_all(WM_keymap_list_find(
        &wm->runtime->userconf->keymaps, "Node Editor", SPACE_NODE, RGN_TYPE_WINDOW));
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Background Image Operators
 * \{ */

struct NodeViewMove {
  int2 mvalo;
  int xmin, ymin, xmax, ymax;
  /** Original Offset for cancel. */
  float xof_orig, yof_orig;
};

static wmOperatorStatus snode_bg_viewmove_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  NodeViewMove *nvm = static_cast<NodeViewMove *>(op->customdata);

  switch (event->type) {
    case MOUSEMOVE:

      snode->xof -= (nvm->mvalo.x - event->mval[0]);
      snode->yof -= (nvm->mvalo.y - event->mval[1]);
      nvm->mvalo.x = event->mval[0];
      nvm->mvalo.y = event->mval[1];

      /* prevent dragging image outside of the window and losing it! */
      CLAMP(snode->xof, nvm->xmin, nvm->xmax);
      CLAMP(snode->yof, nvm->ymin, nvm->ymax);

      ED_region_tag_redraw(region);
      WM_main_add_notifier(NC_NODE | ND_DISPLAY, nullptr);
      WM_main_add_notifier(NC_SPACE | ND_SPACE_NODE_VIEW, nullptr);

      break;

    case LEFTMOUSE:
    case MIDDLEMOUSE:
      if (event->val == KM_RELEASE) {
        MEM_delete(nvm);
        op->customdata = nullptr;
        return OPERATOR_FINISHED;
      }
      break;
    case EVT_ESCKEY:
    case RIGHTMOUSE:
      snode->xof = nvm->xof_orig;
      snode->yof = nvm->yof_orig;
      ED_region_tag_redraw(region);
      WM_main_add_notifier(NC_NODE | ND_DISPLAY, nullptr);
      WM_main_add_notifier(NC_SPACE | ND_SPACE_NODE_VIEW, nullptr);

      MEM_delete(nvm);
      op->customdata = nullptr;

      return OPERATOR_CANCELLED;
    default: {
      break;
    }
  }

  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus snode_bg_viewmove_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  Main *bmain = CTX_data_main(C);
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  NodeViewMove *nvm;
  Image *ima;
  ImBuf *ibuf;
  const float pad = 32.0f; /* Better be bigger than scroll-bars. */

  void *lock;

  ima = BKE_image_ensure_viewer(bmain, IMA_TYPE_COMPOSITE, "Viewer Node");
  ibuf = BKE_image_acquire_ibuf(ima, nullptr, &lock);

  if (ibuf == nullptr) {
    BKE_image_release_ibuf(ima, ibuf, lock);
    return OPERATOR_CANCELLED;
  }

  nvm = MEM_new_zeroed<NodeViewMove>(__func__);
  op->customdata = nvm;
  nvm->mvalo.x = event->mval[0];
  nvm->mvalo.y = event->mval[1];

  nvm->xmin = -(region->winx / 2) - (ibuf->x * (0.5f * snode->zoom)) + pad;
  nvm->xmax = (region->winx / 2) + (ibuf->x * (0.5f * snode->zoom)) - pad;
  nvm->ymin = -(region->winy / 2) - (ibuf->y * (0.5f * snode->zoom)) + pad;
  nvm->ymax = (region->winy / 2) + (ibuf->y * (0.5f * snode->zoom)) - pad;

  nvm->xof_orig = snode->xof;
  nvm->yof_orig = snode->yof;

  BKE_image_release_ibuf(ima, ibuf, lock);

  /* add modal handler */
  WM_event_add_modal_handler(C, op);

  return OPERATOR_RUNNING_MODAL;
}

static void snode_bg_viewmove_cancel(bContext * /*C*/, wmOperator *op)
{
  NodeViewMove *nvm = static_cast<NodeViewMove *>(op->customdata);
  MEM_delete(nvm);
  op->customdata = nullptr;
}

void NODE_OT_backimage_move(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Background Image Move";
  ot->description = "Move node backdrop";
  ot->idname = "NODE_OT_backimage_move";

  /* API callbacks. */
  ot->invoke = snode_bg_viewmove_invoke;
  ot->modal = snode_bg_viewmove_modal;
  ot->poll = space_node_composite_active_view_poll;
  ot->cancel = snode_bg_viewmove_cancel;

  /* flags */
  ot->flag = OPTYPE_BLOCKING | OPTYPE_GRAB_CURSOR_XY;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Background Image Zoom
 * \{ */

/* Simple struct for background image zoom data */
struct backImageZoomData {
  float factor;
  float2 offset;
};

static void backimage_zoom_init(bContext *C, wmOperator *op)
{
  BLI_assert(space_node_composite_active_view_poll(C));

  SpaceNode *snode = CTX_wm_space_node(C);

  backImageZoomData *zdata = MEM_new_zeroed<backImageZoomData>(__func__);
  op->customdata = zdata;

  zdata->factor = RNA_float_get(op->ptr, "factor");
  zdata->offset = {snode->xof, snode->yof}; /* Current img offset for execute. */
}

static wmOperatorStatus backimage_zoom_exec(bContext *C, wmOperator *op)
{
  ARegion *region = CTX_wm_region(C);
  SpaceNode *snode = CTX_wm_space_node(C);

  /* If executed without invoking, initialize the customdata here. */
  if (op->customdata == nullptr) {
    backimage_zoom_init(C, op);
  }

  backImageZoomData *zdata = static_cast<backImageZoomData *>(op->customdata);

  snode->zoom *= zdata->factor;
  snode->xof = zdata->offset.x;
  snode->yof = zdata->offset.y;

  ED_region_tag_redraw(region);
  WM_main_add_notifier(NC_NODE | ND_DISPLAY, nullptr);
  WM_main_add_notifier(NC_SPACE | ND_SPACE_NODE_VIEW, nullptr);

  MEM_SAFE_DELETE(zdata);
  op->customdata = nullptr;

  return OPERATOR_FINISHED;
}

static wmOperatorStatus backimage_zoom_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  ARegion *region = CTX_wm_region(C);
  SpaceNode *snode = CTX_wm_space_node(C);

  backimage_zoom_init(C, op);
  backImageZoomData *zdata = static_cast<backImageZoomData *>(op->customdata);

  /* If the not set in the preference, no need to do all the checks below. */
  if ((U.uiflag & USER_ZOOM_TO_MOUSEPOS) == 0) {
    return backimage_zoom_exec(C, op);
  }

  if (RNA_boolean_get(op->ptr, "use_mouse_pos")) {
    float fac = RNA_float_get(op->ptr, "factor");

    float2 img_center = {snode->xof + region->winx / 2.0f, snode->yof + region->winy / 2.0f};

    float2 img_offset = {snode->xof + (event->mval[0] - img_center.x) * (1 - fac),
                         snode->yof + (event->mval[1] - img_center.y) * (1 - fac)};

    zdata->offset = {img_offset.x, img_offset.y};
  }

  return backimage_zoom_exec(C, op);
}

void NODE_OT_backimage_zoom(wmOperatorType *ot)
{

  /* identifiers */
  ot->name = "Background Image Zoom";
  ot->idname = "NODE_OT_backimage_zoom";
  ot->description = "Zoom in/out the background image";

  /* API callbacks. */
  ot->invoke = backimage_zoom_invoke;
  ot->exec = backimage_zoom_exec;
  ot->poll = space_node_composite_active_view_poll;

  /* flags */
  ot->flag = OPTYPE_BLOCKING;

  /* internal */
  RNA_def_float(ot->srna, "factor", 1.2f, 0.0f, 10.0f, "Factor", "", 0.0f, 10.0f);
  RNA_def_boolean(ot->srna, "use_mouse_pos", true, "Use Mouse Position", "Zoom to mouse position");
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Background Image Fit
 * \{ */

static wmOperatorStatus backimage_fit_exec(bContext *C, wmOperator * /*op*/)
{
  Main *bmain = CTX_data_main(C);
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);

  Image *ima;
  ImBuf *ibuf;

  const float pad = 32.0f;

  void *lock;

  float facx, facy;

  ima = BKE_image_ensure_viewer(bmain, IMA_TYPE_COMPOSITE, "Viewer Node");
  ibuf = BKE_image_acquire_ibuf(ima, nullptr, &lock);

  if ((ibuf == nullptr) || (ibuf->x == 0) || (ibuf->y == 0)) {
    BKE_image_release_ibuf(ima, ibuf, lock);
    return OPERATOR_CANCELLED;
  }

  facx = 1.0f * (region->sizex - pad) / (ibuf->x * snode->zoom);
  facy = 1.0f * (region->sizey - pad) / (ibuf->y * snode->zoom);

  BKE_image_release_ibuf(ima, ibuf, lock);

  snode->zoom *= min_ff(facx, facy) * UI_SCALE_FAC;

  snode->xof = 0;
  snode->yof = 0;

  ED_region_tag_redraw(region);
  WM_main_add_notifier(NC_NODE | ND_DISPLAY, nullptr);
  WM_main_add_notifier(NC_SPACE | ND_SPACE_NODE_VIEW, nullptr);

  return OPERATOR_FINISHED;
}

void NODE_OT_backimage_fit(wmOperatorType *ot)
{

  /* identifiers */
  ot->name = "Background Image Fit";
  ot->idname = "NODE_OT_backimage_fit";
  ot->description = "Fit the background image to the view";

  /* API callbacks. */
  ot->exec = backimage_fit_exec;
  ot->poll = space_node_composite_active_view_poll;

  /* flags */
  ot->flag = OPTYPE_BLOCKING;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Sample Backdrop Operator
 * \{ */

struct ImageSampleInfo {
  ARegionType *art;
  void *draw_handle;
  int x, y;
  int channels;

  uchar col[4];
  float colf[4];
  float linearcol[4];

  int draw;
  int color_manage;
};

static void sample_draw(const bContext *C, ARegion *region, void *arg_info)
{
  Scene *scene = CTX_data_scene(C);
  ImageSampleInfo *info = static_cast<ImageSampleInfo *>(arg_info);

  if (info->draw) {
    ED_image_draw_info(scene,
                       region,
                       info->color_manage,
                       false,
                       info->channels,
                       info->x,
                       info->y,
                       info->col,
                       info->colf,
                       info->linearcol);
  }
}

}  // namespace ed::space_node

bool ED_space_node_get_position(
    Main *bmain, SpaceNode *snode, ARegion *region, const int mval[2], float fpos[2])
{
  if (!ED_node_uses_viewer_backdrop(snode)) {
    return false;
  }

  void *lock;
  Image *ima = BKE_image_ensure_viewer(bmain, IMA_TYPE_COMPOSITE, "Viewer Node");
  ImBuf *ibuf = BKE_image_acquire_ibuf(ima, nullptr, &lock);
  if (!ibuf) {
    BKE_image_release_ibuf(ima, ibuf, lock);
    return false;
  }

  /* map the mouse coords to the backdrop image space */
  float bufx = ibuf->x * snode->zoom;
  float bufy = ibuf->y * snode->zoom;
  fpos[0] = (bufx > 0.0f ? (float(mval[0]) - 0.5f * region->winx - snode->xof) / bufx + 0.5f :
                           0.0f);
  fpos[1] = (bufy > 0.0f ? (float(mval[1]) - 0.5f * region->winy - snode->yof) / bufy + 0.5f :
                           0.0f);

  BKE_image_release_ibuf(ima, ibuf, lock);
  return true;
}

bool ED_space_node_color_sample(
    Main *bmain, SpaceNode *snode, ARegion *region, const int mval[2], float r_col[3])
{
  void *lock;
  Image *ima;
  ImBuf *ibuf;
  float fx, fy, bufx, bufy;
  bool ret = false;

  if (!ED_node_uses_viewer_backdrop(snode)) {
    /* Use viewer image for color sampling only with Viewer backdrop enabled. */
    return false;
  }

  ima = BKE_image_ensure_viewer(bmain, IMA_TYPE_COMPOSITE, "Viewer Node");
  ibuf = BKE_image_acquire_ibuf(ima, nullptr, &lock);
  if (!ibuf) {
    return false;
  }

  /* map the mouse coords to the backdrop image space */
  bufx = ibuf->x * snode->zoom;
  bufy = ibuf->y * snode->zoom;
  fx = (bufx > 0.0f ? (float(mval[0]) - 0.5f * region->winx - snode->xof) / bufx + 0.5f : 0.0f);
  fy = (bufy > 0.0f ? (float(mval[1]) - 0.5f * region->winy - snode->yof) / bufy + 0.5f : 0.0f);

  if (fx >= 0.0f && fy >= 0.0f && fx < 1.0f && fy < 1.0f) {
    const float *fp;
    const uchar *cp;
    int x = int(fx * ibuf->x), y = int(fy * ibuf->y);

    CLAMP(x, 0, ibuf->x - 1);
    CLAMP(y, 0, ibuf->y - 1);

    if (const float *float_data = ibuf->float_data()) {
      fp = (float_data + (ibuf->channels) * (y * ibuf->x + x));
      copy_v3_v3(r_col, fp);
      ret = true;
    }
    else if (const uchar *byte_data = ibuf->byte_data()) {
      cp = byte_data + 4 * (y * ibuf->x + x);
      rgb_uchar_to_float(r_col, cp);
      IMB_colormanagement_colorspace_to_scene_linear_v3(r_col, ibuf->byte_buffer.colorspace);
      ret = true;
    }
  }

  BKE_image_release_ibuf(ima, ibuf, lock);

  return ret;
}

namespace ed::space_node {

static void sample_apply(bContext *C, wmOperator *op, const wmEvent *event)
{
  Main *bmain = CTX_data_main(C);
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  ImageSampleInfo *info = static_cast<ImageSampleInfo *>(op->customdata);
  void *lock;
  Image *ima;
  ImBuf *ibuf;
  float fx, fy, bufx, bufy;

  ima = BKE_image_ensure_viewer(bmain, IMA_TYPE_COMPOSITE, "Viewer Node");
  ibuf = BKE_image_acquire_ibuf(ima, nullptr, &lock);
  if (!ibuf) {
    info->draw = 0;
    return;
  }

  if (!ibuf->byte_data()) {
    IMB_byte_from_float(ibuf);
  }

  /* map the mouse coords to the backdrop image space */
  bufx = ibuf->x * snode->zoom;
  bufy = ibuf->y * snode->zoom;
  fx = (bufx > 0.0f ? (float(event->mval[0]) - 0.5f * region->winx - snode->xof) / bufx + 0.5f :
                      0.0f);
  fy = (bufy > 0.0f ? (float(event->mval[1]) - 0.5f * region->winy - snode->yof) / bufy + 0.5f :
                      0.0f);

  if (fx >= 0.0f && fy >= 0.0f && fx < 1.0f && fy < 1.0f) {
    const float *fp;
    const uchar *cp;
    int x = int(fx * ibuf->x), y = int(fy * ibuf->y);

    CLAMP(x, 0, ibuf->x - 1);
    CLAMP(y, 0, ibuf->y - 1);

    info->x = x;
    info->y = y;
    info->draw = 1;
    info->channels = ibuf->channels;

    if (const uchar *byte_data = ibuf->byte_data()) {
      cp = byte_data + 4 * (y * ibuf->x + x);

      info->col[0] = cp[0];
      info->col[1] = cp[1];
      info->col[2] = cp[2];
      info->col[3] = cp[3];

      info->colf[0] = float(cp[0]) / 255.0f;
      info->colf[1] = float(cp[1]) / 255.0f;
      info->colf[2] = float(cp[2]) / 255.0f;
      info->colf[3] = float(cp[3]) / 255.0f;

      copy_v4_v4(info->linearcol, info->colf);
      IMB_colormanagement_colorspace_to_scene_linear_v4(
          info->linearcol, false, ibuf->byte_buffer.colorspace);

      info->color_manage = true;
    }
    if (const float *float_data = ibuf->float_data()) {
      fp = float_data + (ibuf->channels) * (y * ibuf->x + x);

      info->colf[0] = fp[0];
      info->colf[1] = fp[1];
      info->colf[2] = fp[2];
      info->colf[3] = fp[3];

      info->color_manage = true;
    }

    ED_node_sample_set(info->colf);
  }
  else {
    info->draw = 0;
    ED_node_sample_set(nullptr);
  }

  BKE_image_release_ibuf(ima, ibuf, lock);

  ED_area_tag_redraw(CTX_wm_area(C));
}

static void sample_exit(bContext *C, wmOperator *op)
{
  ImageSampleInfo *info = static_cast<ImageSampleInfo *>(op->customdata);

  ED_node_sample_set(nullptr);
  ED_region_draw_cb_exit(info->art, info->draw_handle);
  ED_area_tag_redraw(CTX_wm_area(C));
  MEM_delete(info);
}

static wmOperatorStatus sample_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  SpaceNode *snode = CTX_wm_space_node(C);
  ARegion *region = CTX_wm_region(C);
  ImageSampleInfo *info;

  /* Don't handle events intended for nodes (which rely on click/drag distinction).
   * which this operator would use since sampling is normally activated on press, see: #98191. */
  if (node_or_socket_isect_event(*C, *event)) {
    return OPERATOR_PASS_THROUGH;
  }

  if (!ED_node_uses_viewer_backdrop(snode)) {
    return OPERATOR_CANCELLED;
  }

  info = MEM_new_zeroed<ImageSampleInfo>("ImageSampleInfo");
  info->art = region->runtime->type;
  info->draw_handle = ED_region_draw_cb_activate(
      region->runtime->type, sample_draw, info, REGION_DRAW_POST_PIXEL);
  op->customdata = info;

  sample_apply(C, op, event);

  WM_event_add_modal_handler(C, op);

  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus sample_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  switch (event->type) {
    case LEFTMOUSE:
    case RIGHTMOUSE: /* XXX hardcoded */
      if (event->val == KM_RELEASE) {
        sample_exit(C, op);
        return OPERATOR_CANCELLED;
      }
      break;
    case MOUSEMOVE:
      sample_apply(C, op, event);
      break;
    default: {
      break;
    }
  }

  return OPERATOR_RUNNING_MODAL;
}

static void sample_cancel(bContext *C, wmOperator *op)
{
  sample_exit(C, op);
}

void NODE_OT_backimage_sample(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Backimage Sample";
  ot->idname = "NODE_OT_backimage_sample";
  ot->description = "Use mouse to sample background image";

  /* API callbacks. */
  ot->invoke = sample_invoke;
  ot->modal = sample_modal;
  ot->cancel = sample_cancel;
  ot->poll = space_node_active_view_poll;

  /* flags */
  ot->flag = OPTYPE_BLOCKING;
}

/** \} */

}  // namespace ed::space_node

}  // namespace blender
