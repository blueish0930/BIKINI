/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spimage
 */

#include <algorithm>

#include "BKE_context.hh"
#include "BKE_image.hh"
#include "BKE_image_paint_layers.hh"
#include "BKE_lib_id.hh"
#include "BKE_main.hh"
#include "BKE_paint.hh"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "BLI_listbase.hh"
#include "BLI_path_utils.hh"
#include "BLI_rect.hh"
#include "BLI_utildefines.hh"

#include "BLT_translation.hh"

#include "DEG_depsgraph.hh"

#include "DNA_image_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_enums.h"
#include "DNA_space_types.h"
#include "DNA_userdef_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_image.hh"
#include "ED_paint.hh"
#include "ED_screen.hh"
#include "ED_undo.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_prototypes.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"
#include "UI_view2d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "image_intern.hh"

namespace blender {

static Image *paint_layer_image_from_context(const bContext *C)
{
  if (Image *ima = CTX_data_edit_image(C)) {
    return ima;
  }
  Object *ob = CTX_data_active_object(C);
  Scene *scene = CTX_data_scene(C);
  if (ob && scene && scene->toolsettings) {
    if (std::optional<CanvasImageData> canvas = BKE_paint_canvas_image_get(
            scene->toolsettings->imapaint, *ob))
    {
      return canvas->first;
    }
  }
  return nullptr;
}

static ImageUser paint_layer_iuser_from_context(const bContext *C, Image *ima)
{
  ImageUser iuser{};
  BKE_imageuser_default(&iuser);
  if (SpaceImage *sima = CTX_wm_space_image(C)) {
    iuser = sima->iuser;
  }
  else if (ima && ima->tiles.first()) {
    iuser.tile = ima->tiles.first()->tile_number;
  }
  return iuser;
}

static bool paint_layer_poll(bContext *C)
{
  Image *ima = paint_layer_image_from_context(C);
  return ima && BKE_image_paint_layers_supported(*ima);
}

static wmOperatorStatus paint_layer_add_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  ED_paint_proj_gpu_overlay_commit();
  if (BKE_image_paint_layers_add(*ima, &iuser, op->reports) == nullptr) {
    return OPERATOR_CANCELLED;
  }
  /* An empty layer does not change pixels. Skip the full-image composite and
   * the GPU texture reload — those are what hitch at 8K/12K. */
  if (ima->runtime == nullptr || !ima->runtime->paint_layers_composite_valid) {
    BKE_image_paint_layers_refresh_display(ima, &iuser);
    WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  }
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_add(wmOperatorType *ot)
{
  ot->name = "Add Paint Layer";
  ot->idname = "IMAGE_OT_paint_layer_add";
  ot->description = "Add a new texture paint layer";
  ot->exec = paint_layer_add_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static wmOperatorStatus paint_layer_remove_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  ED_paint_proj_gpu_overlay_commit();
  BKE_image_paint_layers_pack_dirty(*ima);
  ED_undo_push(C, "Remove Paint Layer");
  if (!BKE_image_paint_layers_remove_active(*ima, &iuser, op->reports)) {
    return OPERATOR_CANCELLED;
  }
  BKE_image_paint_layers_refresh_display(ima, &iuser);
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_remove(wmOperatorType *ot)
{
  ot->name = "Remove Paint Layer";
  ot->idname = "IMAGE_OT_paint_layer_remove";
  ot->description = "Remove the active texture paint layer";
  ot->exec = paint_layer_remove_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static wmOperatorStatus paint_layer_move_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  const int direction = RNA_enum_get(op->ptr, "direction");
  ED_paint_proj_gpu_overlay_commit();
  if (!BKE_image_paint_layers_move(*ima, direction)) {
    return OPERATOR_CANCELLED;
  }
  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  BKE_image_paint_layers_refresh_display(ima, &iuser);
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_move(wmOperatorType *ot)
{
  static const EnumPropertyItem direction_items[] = {
      {-1, "UP", 0, "Up", ""},
      {1, "DOWN", 0, "Down", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };
  ot->name = "Move Paint Layer";
  ot->idname = "IMAGE_OT_paint_layer_move";
  ot->description = "Move the active paint layer up or down";
  ot->exec = paint_layer_move_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
  RNA_def_enum(ot->srna, "direction", direction_items, -1, "Direction", "");
}

static wmOperatorStatus paint_layer_duplicate_exec(bContext *C, wmOperator * /*op*/)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  ED_paint_proj_gpu_overlay_commit();
  if (BKE_image_paint_layers_duplicate_active(*ima, &iuser) == nullptr) {
    return OPERATOR_CANCELLED;
  }
  BKE_image_paint_layers_refresh_display(ima, &iuser);
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_duplicate(wmOperatorType *ot)
{
  ot->name = "Duplicate Paint Layer";
  ot->idname = "IMAGE_OT_paint_layer_duplicate";
  ot->description = "Duplicate the active texture paint layer";
  ot->exec = paint_layer_duplicate_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static wmOperatorStatus paint_layer_merge_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  ED_paint_proj_gpu_overlay_commit();
  BKE_image_paint_layers_pack_dirty(*ima);
  ED_undo_push(C, "Merge Paint Layer");
  if (!BKE_image_paint_layers_merge_down(*ima, &iuser, op->reports)) {
    return OPERATOR_CANCELLED;
  }
  BKE_image_paint_layers_refresh_display(ima, &iuser);
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_merge_down(wmOperatorType *ot)
{
  ot->name = "Merge Paint Layer Down";
  ot->idname = "IMAGE_OT_paint_layer_merge_down";
  ot->description = "Merge the active layer into the layer below";
  ot->exec = paint_layer_merge_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static wmOperatorStatus paint_layer_flatten_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  ED_paint_proj_gpu_overlay_commit();
  BKE_image_paint_layers_pack_dirty(*ima);
  ED_undo_push(C, "Flatten Paint Layers");
  if (!BKE_image_paint_layers_flatten(*ima, &iuser, op->reports)) {
    return OPERATOR_CANCELLED;
  }
  BKE_image_paint_layers_refresh_display(ima, &iuser);
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_flatten(wmOperatorType *ot)
{
  ot->name = "Flatten Paint Layers";
  ot->idname = "IMAGE_OT_paint_layer_flatten";
  ot->description = "Flatten all paint layers into the background";
  ot->exec = paint_layer_flatten_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static wmOperatorStatus paint_layer_set_active_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  const int index = RNA_int_get(op->ptr, "index");
  if (ima->active_paint_layer_index == index) {
    ImagePaintLayer *layer = BKE_image_paint_layers_active(*ima);
    if (layer && (layer->flag & IMA_PAINT_LAYER_PAINT_MASK)) {
      BKE_image_paint_layers_set_active(*ima, index);
    }
    else {
      BKE_image_paint_layers_set_active(*ima, -1);
    }
  }
  else {
    BKE_image_paint_layers_set_active(*ima, index);
  }
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_set_active(wmOperatorType *ot)
{
  ot->name = "Set Active Paint Layer";
  ot->idname = "IMAGE_OT_paint_layer_set_active";
  ot->description = "Select a texture paint layer";
  ot->exec = paint_layer_set_active_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
  RNA_def_int(ot->srna, "index", 0, 0, INT_MAX, "Index", "", 0, INT_MAX);
}

static void paint_layer_activate_from_op(Image &ima, wmOperator *op)
{
  if (!RNA_struct_find_property(op->ptr, "layer_index")) {
    return;
  }
  const int layer_index = RNA_int_get(op->ptr, "layer_index");
  if (layer_index >= 0) {
    BKE_image_paint_layers_set_active(ima, layer_index);
  }
}

static wmOperatorStatus paint_layer_mask_set_active_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  const int layer_index = RNA_int_get(op->ptr, "layer_index");
  const int mask_index = RNA_int_get(op->ptr, "index");
  if (layer_index >= 0 && ima->active_paint_layer_index != layer_index) {
    BKE_image_paint_layers_set_active(*ima, layer_index);
  }
  ImagePaintLayer *layer = BKE_image_paint_layers_active(*ima);
  if (layer && (layer->flag & IMA_PAINT_LAYER_PAINT_MASK) &&
      layer->active_mask_index == mask_index)
  {
    layer->flag = eImagePaintLayer_Flag(layer->flag & ~IMA_PAINT_LAYER_PAINT_MASK);
  }
  else {
    BKE_image_paint_layers_set_active_mask(*ima, mask_index);
  }
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_mask_set_active(wmOperatorType *ot)
{
  ot->name = "Set Active Paint Mask";
  ot->idname = "IMAGE_OT_paint_layer_mask_set_active";
  ot->description = "Select a paint layer mask for painting";
  ot->exec = paint_layer_mask_set_active_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
  RNA_def_int(ot->srna, "index", 0, 0, INT_MAX, "Index", "", 0, INT_MAX);
  RNA_def_int(ot->srna, "layer_index", -1, -1, INT_MAX, "Layer", "", -1, INT_MAX);
}

static wmOperatorStatus paint_layer_mask_add_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  paint_layer_activate_from_op(*ima, op);
  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  ED_paint_proj_gpu_overlay_commit();
  if (BKE_image_paint_layers_mask_add(*ima, &iuser, op->reports) == nullptr) {
    return OPERATOR_CANCELLED;
  }
  BKE_image_paint_layers_refresh_display(ima, &iuser);
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_mask_add(wmOperatorType *ot)
{
  ot->name = "Add Paint Mask";
  ot->idname = "IMAGE_OT_paint_layer_mask_add";
  ot->description = "Add a mask under the active paint layer";
  ot->exec = paint_layer_mask_add_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
  RNA_def_int(ot->srna, "layer_index", -1, -1, INT_MAX, "Layer", "", -1, INT_MAX);
}

static wmOperatorStatus paint_layer_mask_remove_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  paint_layer_activate_from_op(*ima, op);
  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  ED_paint_proj_gpu_overlay_commit();
  if (!BKE_image_paint_layers_mask_remove_active(*ima, op->reports)) {
    return OPERATOR_CANCELLED;
  }
  BKE_image_paint_layers_refresh_display(ima, &iuser);
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_mask_remove(wmOperatorType *ot)
{
  ot->name = "Remove Paint Mask";
  ot->idname = "IMAGE_OT_paint_layer_mask_remove";
  ot->description = "Remove the active mask from the active paint layer";
  ot->exec = paint_layer_mask_remove_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
  RNA_def_int(ot->srna, "layer_index", -1, -1, INT_MAX, "Layer", "", -1, INT_MAX);
}

static wmOperatorStatus paint_layer_mask_move_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  paint_layer_activate_from_op(*ima, op);
  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  ED_paint_proj_gpu_overlay_commit();
  if (!BKE_image_paint_layers_mask_move(*ima, RNA_enum_get(op->ptr, "direction"))) {
    return OPERATOR_CANCELLED;
  }
  BKE_image_paint_layers_refresh_display(ima, &iuser);
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layer_mask_move(wmOperatorType *ot)
{
  static const EnumPropertyItem direction_items[] = {
      {-1, "UP", 0, "Up", ""},
      {1, "DOWN", 0, "Down", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };
  ot->name = "Move Paint Mask";
  ot->idname = "IMAGE_OT_paint_layer_mask_move";
  ot->description = "Move the active mask up or down";
  ot->exec = paint_layer_mask_move_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
  RNA_def_enum(ot->srna, "direction", direction_items, -1, "Direction", "");
  RNA_def_int(ot->srna, "layer_index", -1, -1, INT_MAX, "Layer", "", -1, INT_MAX);
}

static wmOperatorStatus paint_layer_open_image_exec(bContext *C, wmOperator *op)
{
  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    return OPERATOR_CANCELLED;
  }
  char filepath[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", filepath);
  Image *src = BKE_image_load_exists(CTX_data_main(C), filepath);
  if (src == nullptr) {
    BKE_reportf(op->reports, RPT_ERROR, "Cannot load image '%s'", filepath);
    return OPERATOR_CANCELLED;
  }
  const bool as_mask = RNA_boolean_get(op->ptr, "as_mask");
  const int layer_index = RNA_int_get(op->ptr, "layer_index");
  const int mask_index = RNA_int_get(op->ptr, "mask_index");
  if (layer_index >= 0) {
    BKE_image_paint_layers_set_active(*ima, layer_index);
  }
  if (as_mask) {
    if (mask_index >= 0) {
      BKE_image_paint_layers_set_active_mask(*ima, mask_index);
    }
    ImagePaintLayer *layer = BKE_image_paint_layers_active(*ima);
    ImagePaintMask *mask = layer ? BKE_image_paint_layers_mask_active(*layer) : nullptr;
    if (mask == nullptr) {
      BKE_report(op->reports, RPT_ERROR, "No active mask");
      return OPERATOR_CANCELLED;
    }
    BKE_image_paint_layers_mask_set_source(*mask, src);
    mask->flag = eImagePaintMask_Flag(mask->flag | IMA_PAINT_MASK_USE_IMAGE);
  }
  else {
    ImagePaintLayer *layer = BKE_image_paint_layers_active(*ima);
    if (layer == nullptr) {
      BKE_report(op->reports, RPT_ERROR, "No active paint layer");
      return OPERATOR_CANCELLED;
    }
    BKE_image_paint_layers_set_source(*layer, src);
    layer->flag = eImagePaintLayer_Flag(layer->flag | IMA_PAINT_LAYER_USE_IMAGE);
  }
  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  BKE_image_paint_layers_refresh_display(ima, &iuser);
  WM_event_add_notifier(C, NC_IMAGE | ND_DISPLAY, ima);
  WM_event_add_notifier(C, NC_OBJECT | ND_DRAW, nullptr);
  return OPERATOR_FINISHED;
}

static wmOperatorStatus paint_layer_open_image_invoke(bContext *C,
                                                      wmOperator *op,
                                                      const wmEvent *event)
{
  if (RNA_struct_property_is_set(op->ptr, "filepath")) {
    return paint_layer_open_image_exec(C, op);
  }
  return WM_operator_filesel(C, op, event);
}

void IMAGE_OT_paint_layer_open_image(wmOperatorType *ot)
{
  ot->name = "Open Image for Layer";
  ot->idname = "IMAGE_OT_paint_layer_open_image";
  ot->description = "Load an image as the active layer or mask input";
  ot->exec = paint_layer_open_image_exec;
  ot->invoke = paint_layer_open_image_invoke;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER | FILE_TYPE_IMAGE,
                                 FILE_SPECIAL,
                                 FILE_OPENFILE,
                                 WM_FILESEL_FILEPATH | WM_FILESEL_RELPATH,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);
  RNA_def_boolean(ot->srna, "as_mask", false, "As Mask", "Assign the image to the active mask");
  RNA_def_int(ot->srna, "layer_index", -1, -1, INT_MAX, "Layer", "", -1, INT_MAX);
  RNA_def_int(ot->srna, "mask_index", -1, -1, INT_MAX, "Mask", "", -1, INT_MAX);
}

static void paint_layers_window_draw_contents(const bContext *C, ui::Layout &layout)
{
  layout.use_property_decorate_set(false);

  if (wmWindow *win = CTX_wm_window(C)) {
    PointerRNA win_ptr = RNA_pointer_create_discrete(nullptr, RNA_Window, win);
    ui::Layout &pinrow = layout.row(true);
    pinrow.prop(&win_ptr, "stay_on_top", ui::ITEM_R_TOGGLE, IFACE_("Pin on Top"), ICON_PINNED);
  }

  Image *ima = paint_layer_image_from_context(C);
  if (ima == nullptr) {
    if (SpaceImage *sima = CTX_wm_space_image(C)) {
      ima = sima->image;
    }
  }
  if (ima == nullptr) {
    layout.label(IFACE_("No paint image"), ICON_INFO);
    return;
  }

  ImageUser iuser = paint_layer_iuser_from_context(C, ima);
  BKE_image_paint_layers_ensure_default(*ima, &iuser);

  PointerRNA ima_ptr = RNA_id_pointer_create(&ima->id);
  ui::Layout &row = layout.row(false);
  ui::Layout &col = row.column(true);

  int layer_index = 0;
  for (ImagePaintLayer &layer : ima->paint_layers) {
    const bool is_active = (layer_index == ima->active_paint_layer_index);
    PointerRNA layer_ptr = RNA_pointer_create_with_parent(ima_ptr, RNA_ImagePaintLayer, &layer);

    ui::Layout &box = col.box();
    ui::Layout &header = box.row(true);
    header.prop(&layer_ptr, "hide", ui::ITEM_R_ICON_ONLY, "", ICON_NONE);

    PointerRNA sel = header.op("image.paint_layer_set_active",
                               "",
                               is_active ? ICON_RADIOBUT_ON : ICON_RADIOBUT_OFF,
                               wm::OpCallContext::InvokeDefault,
                               is_active ? ui::ITEM_O_DEPRESS : UI_ITEM_NONE);
    if (sel.data) {
      RNA_int_set(&sel, "index", layer_index);
    }

    const bool show_masks = (layer.flag & IMA_PAINT_LAYER_SHOW_MASKS) != 0;
    header.prop(&layer_ptr,
                "show_masks",
                ui::ITEM_R_ICON_ONLY,
                "",
                show_masks ? ICON_TRIA_DOWN : ICON_TRIA_RIGHT);
    header.prop(&layer_ptr, "name", UI_ITEM_NONE, "", ICON_NONE);
    header.prop(&layer_ptr, "blend_mode", UI_ITEM_NONE, "", ICON_NONE);
    header.prop(&layer_ptr, "opacity", UI_ITEM_NONE, "", ICON_NONE);
    header.prop(&layer_ptr, "lock", ui::ITEM_R_ICON_ONLY, "", ICON_NONE);

    if (show_masks) {
      ui::Layout &sub = box.column(true);
      sub.use_property_split_set(true);
      sub.use_property_decorate_set(false);

      ui::Layout &idrow = sub.row(true);
      idrow.prop(&layer_ptr, "use_image", UI_ITEM_NONE, "", ICON_NONE);
      ui::Layout &imgrow = idrow.row(true);
      imgrow.enabled_set((layer.flag & IMA_PAINT_LAYER_USE_IMAGE) != 0);
      imgrow.prop(&layer_ptr, "source_image", UI_ITEM_NONE, IFACE_("Image"), ICON_NONE);
      PointerRNA open_layer = imgrow.op("image.paint_layer_open_image", "", ICON_FILEBROWSER);
      if (open_layer.data) {
        RNA_boolean_set(&open_layer, "as_mask", false);
        RNA_int_set(&open_layer, "layer_index", layer_index);
      }

      sub.separator(0.3f);
      sub.label(IFACE_("Masks"), ICON_NONE);

      int mask_index = 0;
      for (ImagePaintMask &mask : layer.masks) {
        const bool mask_active = is_active && (layer.flag & IMA_PAINT_LAYER_PAINT_MASK) &&
                                 (mask_index == layer.active_mask_index);
        PointerRNA mask_ptr = RNA_pointer_create_with_parent(
            layer_ptr, RNA_ImagePaintMask, &mask);

        ui::Layout &mbox = sub.box();
        ui::Layout &mrow = mbox.row(true);
        mrow.prop(&mask_ptr, "hide", ui::ITEM_R_ICON_ONLY, "", ICON_NONE);
        PointerRNA mop = mrow.op("image.paint_layer_mask_set_active",
                                 "",
                                 mask_active ? ICON_RADIOBUT_ON : ICON_RADIOBUT_OFF,
                                 wm::OpCallContext::InvokeDefault,
                                 mask_active ? ui::ITEM_O_DEPRESS : UI_ITEM_NONE);
        if (mop.data) {
          RNA_int_set(&mop, "index", mask_index);
          RNA_int_set(&mop, "layer_index", layer_index);
        }
        mrow.prop(&mask_ptr, "name", UI_ITEM_NONE, "", ICON_NONE);
        mrow.prop(&mask_ptr, "blend_mode", UI_ITEM_NONE, "", ICON_NONE);
        mrow.prop(&mask_ptr, "opacity", UI_ITEM_NONE, "", ICON_NONE);
        mrow.prop(&mask_ptr, "invert", ui::ITEM_R_ICON_ONLY, "", ICON_NONE);
        mrow.prop(&mask_ptr, "lock", ui::ITEM_R_ICON_ONLY, "", ICON_NONE);

        ui::Layout &msrc = mbox.row(true);
        msrc.prop(&mask_ptr, "use_image", UI_ITEM_NONE, "", ICON_NONE);
        ui::Layout &mimg = msrc.row(true);
        mimg.enabled_set((mask.flag & IMA_PAINT_MASK_USE_IMAGE) != 0);
        mimg.prop(&mask_ptr, "source_image", UI_ITEM_NONE, IFACE_("Image"), ICON_NONE);
        PointerRNA open_mask = mimg.op("image.paint_layer_open_image", "", ICON_FILEBROWSER);
        if (open_mask.data) {
          RNA_boolean_set(&open_mask, "as_mask", true);
          RNA_int_set(&open_mask, "layer_index", layer_index);
          RNA_int_set(&open_mask, "mask_index", mask_index);
        }
        mask_index++;
      }

      ui::Layout &brow = sub.row(true);
      PointerRNA add_mask = brow.op("image.paint_layer_mask_add", IFACE_("Mask"), ICON_ADD);
      if (add_mask.data) {
        RNA_int_set(&add_mask, "layer_index", layer_index);
      }
      PointerRNA rem_mask = brow.op("image.paint_layer_mask_remove", "", ICON_REMOVE);
      if (rem_mask.data) {
        RNA_int_set(&rem_mask, "layer_index", layer_index);
      }
      PointerRNA up_mask = brow.op("image.paint_layer_mask_move", "", ICON_TRIA_UP);
      if (up_mask.data) {
        RNA_enum_set(&up_mask, "direction", -1);
        RNA_int_set(&up_mask, "layer_index", layer_index);
      }
      PointerRNA down_mask = brow.op("image.paint_layer_mask_move", "", ICON_TRIA_DOWN);
      if (down_mask.data) {
        RNA_enum_set(&down_mask, "direction", 1);
        RNA_int_set(&down_mask, "layer_index", layer_index);
      }
      if (layer.flag & IMA_PAINT_LAYER_PAINT_MASK) {
        sub.label(IFACE_("Painting mask"), ICON_MOD_MASK);
      }
    }
    layer_index++;
  }

  ui::Layout &tools = row.column(true);
  tools.op("image.paint_layer_add", "", ICON_ADD);
  tools.op("image.paint_layer_remove", "", ICON_REMOVE);
  tools.separator();
  PointerRNA up = tools.op("image.paint_layer_move", "", ICON_TRIA_UP);
  if (up.data) {
    RNA_enum_set(&up, "direction", -1);
  }
  PointerRNA down = tools.op("image.paint_layer_move", "", ICON_TRIA_DOWN);
  if (down.data) {
    RNA_enum_set(&down, "direction", 1);
  }
  tools.separator();
  tools.op("image.paint_layer_duplicate", "", ICON_DUPLICATE);
  tools.op("image.paint_layer_merge_down", "", ICON_DOWNARROW_HLT);
  layout.op("image.paint_layer_flatten", IFACE_("Flatten Layers"), ICON_NONE);
}

void ED_image_paint_layers_window_layout(const bContext *C, ARegion *region)
{
  ui::view2d_region_reinit(&region->v2d, ui::V2D_COMMONVIEW_PANELS_UI, region->winx, region->winy);
  region->v2d.scroll = V2D_SCROLL_RIGHT | V2D_SCROLL_VERTICAL_HIDE;
  region->v2d.keepofs |= V2D_LOCKOFS_X | V2D_KEEPOFS_Y;
  region->v2d.keepofs &= ~(V2D_LOCKOFS_Y | V2D_KEEPOFS_X);

  ui::Block *block = ui::block_begin(C, region, __func__, ui::EmbossType::Emboss);
  const uiStyle *style = ui::style_get_dpi();
  const int pad = int(UI_SCALE_FAC * 6.0f);
  ui::Layout &layout = ui::block_layout(block,
                                        ui::LayoutDirection::Vertical,
                                        ui::LayoutType::Panel,
                                        pad,
                                        -pad,
                                        std::max(region->winx - 2 * pad, int(UI_UNIT_X * 8)),
                                        0,
                                        0,
                                        style);
  paint_layers_window_draw_contents(C, layout);
  const int layout_height = ui::block_layout_resolve(block).y;
  ui::view2d_totRect_set(&region->v2d, region->winx - 1, layout_height - pad);
  ui::view2d_curRect_validate(&region->v2d);
  ui::view2d_view_ortho(&region->v2d);
  ui::blocklist_update_window_matrix(C, &region->runtime->uiblocks);
  ui::block_end(C, block);
}

void ED_image_paint_layers_window_draw(const bContext *C, ARegion *region)
{
  ui::theme::theme_set(SPACE_PROPERTIES, RGN_TYPE_WINDOW);
  ED_region_clear(C, region, TH_BACK);
  ui::view2d_view_ortho(&region->v2d);
  ui::blocklist_update_window_matrix(C, &region->runtime->uiblocks);
  ui::blocklist_draw(C, &region->runtime->uiblocks);
  ui::view2d_view_restore(C);
  ui::view2d_scrollers_draw(&region->v2d, nullptr);
}

static bool paint_layers_area_is_window(const ScrArea &area)
{
  if (area.spacetype != SPACE_PROPERTIES) {
    return false;
  }
  const SpaceProperties *sbuts = area.spacedata.first_as<SpaceProperties>();
  return sbuts && (sbuts->flag & SB_PAINT_LAYERS_WINDOW);
}

static wmWindow *paint_layers_window_find(wmWindowManager *wm)
{
  if (wm == nullptr) {
    return nullptr;
  }
  for (wmWindow &win : wm->windows) {
    const bScreen *screen = WM_window_get_active_screen(&win);
    if (screen == nullptr) {
      continue;
    }
    for (const ScrArea &area : screen->areabase) {
      if (paint_layers_area_is_window(area)) {
        return &win;
      }
    }
  }
  return nullptr;
}

static void paint_layers_window_hide_chrome(ScrArea *area)
{
  for (ARegion &region : area->regionbase) {
    if (region.regiontype == RGN_TYPE_WINDOW) {
      region.flag &= ~(RGN_FLAG_HIDDEN | RGN_FLAG_HIDDEN_BY_USER);
      continue;
    }
    region.flag |= RGN_FLAG_HIDDEN | RGN_FLAG_HIDDEN_BY_USER;
  }
}

static wmOperatorStatus paint_layers_float_exec(bContext *C, wmOperator *op)
{
  wmWindowManager *wm = CTX_wm_manager(C);
  if (wmWindow *existing = paint_layers_window_find(wm)) {
    WM_window_set_topmost(existing, true);
    WM_window_title_set(existing, "Paint Layers");
    return OPERATOR_FINISHED;
  }

  wmWindow *parent = CTX_wm_window(C);
  if (parent == nullptr) {
    return OPERATOR_CANCELLED;
  }
  const int width = std::max(int(420 * UI_SCALE_FAC), 280);
  const int height = std::max(int(620 * UI_SCALE_FAC), 360);
  const rcti rect = {0, width, 0, height};

  /* Properties editor has no image viewport, so the layer list is not drawn into a black canvas. */
  wmWindow *win = WM_window_open(
      C, "Paint Layers", &rect, SPACE_PROPERTIES, true, false, true, WIN_ALIGN_PARENT_CENTER, nullptr, nullptr);
  if (win == nullptr) {
    BKE_report(op->reports, RPT_ERROR, "Failed to open the paint layer window");
    return OPERATOR_CANCELLED;
  }

  bScreen *screen = WM_window_get_active_screen(win);
  ScrArea *area = screen ? screen->areabase.first() : nullptr;
  if (area == nullptr || area->spacetype != SPACE_PROPERTIES) {
    BKE_report(op->reports, RPT_ERROR, "Failed to open the paint layer window");
    return OPERATOR_CANCELLED;
  }
  if (SpaceProperties *sbuts = area->spacedata.first_as<SpaceProperties>()) {
    sbuts->flag |= SB_PAINT_LAYERS_WINDOW;
  }
  paint_layers_window_hide_chrome(area);
  for (ARegion &region : area->regionbase) {
    if (region.regiontype != RGN_TYPE_WINDOW) {
      ED_region_visibility_change_update(C, area, &region);
    }
  }
  ED_area_tag_redraw(area);
  WM_window_title_set(win, "Paint Layers");
  WM_window_set_topmost(win, true);
  return OPERATOR_FINISHED;
}

void IMAGE_OT_paint_layers_float(wmOperatorType *ot)
{
  ot->name = "Float Paint Layers";
  ot->idname = "IMAGE_OT_paint_layers_float";
  ot->description =
      "Open the paint layers in a window that stays above the main window while painting";
  ot->exec = paint_layers_float_exec;
  ot->poll = paint_layer_poll;
  ot->flag = OPTYPE_REGISTER;
}

}  // namespace blender
