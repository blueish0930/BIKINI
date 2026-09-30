/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "BKE_colorband.hh"
#include "BKE_context.hh"
#include "BKE_library.hh"

#include "BLI_listbase.hh"
#include "BLI_math_base_c.hh"
#include "BLI_math_color_c.hh"
#include "BLI_rect.hh"
#include "BLI_string_ref.hh"

#include "BLT_translation.hh"

#include "DNA_texture_types.h"

#include "ED_screen.hh"
#include "ED_undo.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "UI_interface_layout.hh"
#include "interface_intern.hh"
#include "interface_templates_intern.hh"

namespace blender::ui {

#include "interface_template_color_ramp_presets.hh"

static void colorband_retarget_element_buttons(Block *block, ColorBand *coba);
static void colorband_tag_redraw(bContext &C, Block *block);

static void colorband_preset_build(ColorBand &coba, const float (*srgb)[3], const int stops)
{
  const int count = std::min(stops, int(MAXCOLORBAND));
  coba.tot = short(count);
  coba.cur = 0;
  coba.color_mode = COLBAND_BLEND_RGB;
  coba.ipotype = COLBAND_INTERP_LINEAR;
  coba.ipotype_hue = COLBAND_HUE_NEAR;
  const float step = (count <= 1) ? 0.0f : 1.0f / float(count - 1);
  for (int i = 0; i < count; i++) {
    float linear[3];
    srgb_to_linearrgb_v3_v3(linear, srgb[i]);
    coba.data[i].r = linear[0];
    coba.data[i].g = linear[1];
    coba.data[i].b = linear[2];
    coba.data[i].a = 1.0f;
    coba.data[i].pos = i * step;
    coba.data[i].cur = i;
  }
}

static void colorband_preset_copy(ColorBand &dst, const ColorBand &src)
{
  dst.tot = src.tot;
  dst.cur = 0;
  dst.color_mode = src.color_mode;
  dst.ipotype = src.ipotype;
  dst.ipotype_hue = src.ipotype_hue;
  for (int i = 0; i < src.tot; i++) {
    dst.data[i] = src.data[i];
  }
}

static ColorBand *colorband_preset_bands()
{
  constexpr int count = int(sizeof(color_ramp_presets) / sizeof(*color_ramp_presets));
  static ColorBand bands[count];
  static bool ready = false;
  if (!ready) {
    for (int i = 0; i < count; i++) {
      const ColorRampPreset &preset = color_ramp_presets[i];
      colorband_preset_build(bands[i], preset.srgb, preset.stops);
    }
    ready = true;
  }
  return bands;
}

static void colorband_preset_row(Layout &layout,
                                 Block *block,
                                 ColorBand *coba,
                                 const RNAUpdateCb &cb,
                                 const int begin,
                                 const int end)
{
  ColorBand *bands = colorband_preset_bands();
  const int n = std::max(end - begin, 1);
  const int width = std::max(int(UI_UNIT_X * 2),
                             layout.width() > 0 ? layout.width() / n : int(UI_UNIT_X * 3));
  layout.row(true);
  for (int i = begin; i < end; i++) {
    const ColorRampPreset &preset = color_ramp_presets[i];
    Button *bt = uiDefBut(block,
                          ButtonType::ColorBand,
                          "",
                          0,
                          0,
                          short(width),
                          short(UI_UNIT_Y),
                          &bands[i],
                          -1.0f,
                          0.0f,
                          TIP_(preset.description));
    /* hardmin < 0 marks a swatch: click applies, drag does not edit the card. */
    bt->drawflag &= ~BUT_ICON_LEFT;
    button_func_set(bt, [coba, src = &bands[i], cb, block](bContext &C) {
      colorband_preset_copy(*coba, *src);
      colorband_retarget_element_buttons(block, coba);
      rna_update_cb(C, cb);
      ED_undo_push(&C, "Color Ramp Preset");
      colorband_tag_redraw(C, block);
    });
  }
}

static void colorband_presets_layout(Layout &layout,
                                     Block *block,
                                     ColorBand *coba,
                                     const RNAUpdateCb &cb)
{
  constexpr int count = int(sizeof(color_ramp_presets) / sizeof(*color_ramp_presets));
  int begin = 0;
  while (begin < count) {
    const int group = color_ramp_presets[begin].group;
    int end = begin + 1;
    while (end < count && color_ramp_presets[end].group == group) {
      end++;
    }
    layout.label(IFACE_(group == 0 ? N_("Debug") : N_("Visualization")), ICON_NONE);
    /* Debug is five cards. Visualization is split into rows of four. */
    const int per_row = (group == 0) ? 5 : 4;
    for (int row_begin = begin; row_begin < end; row_begin += per_row) {
      colorband_preset_row(
          layout, block, coba, cb, row_begin, std::min(row_begin + per_row, end));
    }
    begin = end;
  }
}

static void colorband_flip(bContext *C, ColorBand *coba)
{
  CBData data_tmp[MAXCOLORBAND];

  for (int a = 0; a < coba->tot; a++) {
    data_tmp[a] = coba->data[coba->tot - (a + 1)];
  }
  for (int a = 0; a < coba->tot; a++) {
    data_tmp[a].pos = 1.0f - data_tmp[a].pos;
    coba->data[a] = data_tmp[a];
  }

  /* May as well flip the `cur`. */
  coba->cur = coba->tot - (coba->cur + 1);

  ED_undo_push(C, "Flip Color Ramp");
}

static void colorband_distribute(bContext *C, ColorBand *coba, bool evenly)
{
  if (coba->tot > 1) {
    const int tot = evenly ? coba->tot - 1 : coba->tot;
    const float gap = 1.0f / tot;
    float pos = 0.0f;
    for (int a = 0; a < coba->tot; a++) {
      coba->data[a].pos = pos;
      pos += gap;
    }
    const char *undo_str = evenly ? CTX_N_(BLT_I18NCONTEXT_OPERATOR_DEFAULT,
                                           "Distribute Stops Evenly") :
                                    CTX_N_(BLT_I18NCONTEXT_OPERATOR_DEFAULT,
                                           "Distribute Stops from Left");
    ED_undo_push(C, undo_str);
  }
}

static void colorramp_disabled_tip_func(bContext & /*C*/,
                                        TooltipData &tip,
                                        Button *but,
                                        void * /*space*/)
{
  const bool has_tip = !but->tip.is_empty();
  if (has_tip) {
    tooltip_text_field_add(tip, but->tip + ".", {}, TIP_STYLE_HEADER, TIP_LC_NORMAL, false);
  }
  tooltip_text_field_add(tip,
                         TIP_("Disabled: Color ramp only has one stop"),
                         {},
                         TIP_STYLE_NORMAL,
                         TIP_LC_ALERT,
                         has_tip);
};

static Block *colorband_tools_fn(bContext *C, ARegion *region, void *cb_v)
{
  RNAUpdateCb &cb = *static_cast<RNAUpdateCb *>(cb_v);
  const uiStyle *style = style_get_dpi();
  PointerRNA coba_ptr = RNA_property_pointer_get(&cb.ptr, cb.prop);
  ColorBand *coba = static_cast<ColorBand *>(coba_ptr.data);
  short yco = 0;
  const short menuwidth = 10 * UI_UNIT_X;

  Block *block = block_begin(C, region, __func__, EmbossType::Pulldown);

  Layout &layout = block_layout(block,
                                LayoutDirection::Vertical,
                                LayoutType::Menu,
                                0,
                                0,
                                UI_MENU_WIDTH_MIN,
                                0,
                                UI_MENU_PADDING,
                                style);
  block_layout_set_current(block, &layout);
  {
    layout.context_ptr_set("color_ramp", &coba_ptr);
  }

  /* We could move these to operators,
   * although this isn't important unless we want to assign key shortcuts to them. */
  {
    Button *but = uiDefIconTextBut(block,
                                   ButtonType::ButMenu,
                                   ICON_ARROW_LEFTRIGHT,
                                   IFACE_("Flip Color Ramp"),
                                   0,
                                   yco -= UI_UNIT_Y,
                                   menuwidth,
                                   UI_UNIT_Y,
                                   nullptr,
                                   "");
    button_func_set(but, [coba, cb](bContext &C) {
      colorband_flip(&C, coba);
      ED_region_tag_redraw(CTX_wm_region(&C));
      rna_update_cb(C, cb);
    });
  }
  {
    Button *but = uiDefIconTextBut(block,
                                   ButtonType::ButMenu,
                                   ICON_BLANK1,
                                   IFACE_("Distribute Stops from Left"),
                                   0,
                                   yco -= UI_UNIT_Y,
                                   menuwidth,
                                   UI_UNIT_Y,
                                   nullptr,
                                   "");
    button_func_set(but, [coba, cb](bContext &C) {
      colorband_distribute(&C, coba, false);
      ED_region_tag_redraw(CTX_wm_region(&C));
      rna_update_cb(C, cb);
    });

    if (coba->tot < 2) {
      button_flag_enable(but, BUT_DISABLED);
      button_func_tooltip_custom_set(but, colorramp_disabled_tip_func, nullptr, nullptr);
    }
  }
  {
    Button *but = uiDefIconTextBut(block,
                                   ButtonType::ButMenu,
                                   ICON_BLANK1,
                                   IFACE_("Distribute Stops Evenly"),
                                   0,
                                   yco -= UI_UNIT_Y,
                                   menuwidth,
                                   UI_UNIT_Y,
                                   nullptr,
                                   "");
    button_func_set(but, [coba, cb](bContext &C) {
      colorband_distribute(&C, coba, true);
      ED_region_tag_redraw(CTX_wm_region(&C));
      rna_update_cb(C, cb);
    });

    if (coba->tot < 2) {
      button_flag_enable(but, BUT_DISABLED);
      button_func_tooltip_custom_set(but, colorramp_disabled_tip_func, nullptr, nullptr);
    }
  }

  layout.separator();

  layout.op("UI_OT_eyedropper_colorramp", IFACE_("Eyedropper"), ICON_EYEDROPPER);

  layout.separator();

  {
    Button *but = uiDefIconTextBut(block,
                                   ButtonType::ButMenu,
                                   ICON_LOOP_BACK,
                                   IFACE_("Reset Color Ramp"),
                                   0,
                                   yco -= UI_UNIT_Y,
                                   menuwidth,
                                   UI_UNIT_Y,
                                   nullptr,
                                   "");
    button_func_set(but, [coba, cb](bContext &C) {
      BKE_colorband_init(coba, true);
      ED_undo_push(&C, "Reset Color Ramp");
      ED_region_tag_redraw(CTX_wm_region(&C));
      rna_update_cb(C, cb);
    });
  }

  block_direction_set(block, UI_DIR_DOWN);
  block_bounds_set_text(block, 3.0f * UI_UNIT_X);

  return block;
}

static void colorband_add(bContext &C, const RNAUpdateCb &cb, ColorBand &coba)
{
  float pos = 0.5f;

  if (coba.tot > 1) {
    if (coba.cur > 0) {
      pos = (coba.data[coba.cur - 1].pos + coba.data[coba.cur].pos) * 0.5f;
    }
    else {
      pos = (coba.data[coba.cur + 1].pos + coba.data[coba.cur].pos) * 0.5f;
    }
  }

  if (BKE_colorband_element_add(&coba, pos)) {
    rna_update_cb(C, cb);
    ED_undo_push(&C, "Add Color Ramp Stop");
  }
}

static bool colorband_data_contains(const ColorBand *coba, const void *data)
{
  if (coba == nullptr || data == nullptr) {
    return false;
  }
  const CBData *elem = static_cast<const CBData *>(data);
  const CBData *begin = coba->data;
  const CBData *end = coba->data + (sizeof(coba->data) / sizeof(coba->data[0]));
  return elem >= begin && elem < end;
}

/** Position and color buttons are bound once. Point them at the stop that is selected now. */
static void colorband_retarget_element_buttons(Block *block, ColorBand *coba)
{
  if (block == nullptr || coba == nullptr || coba->tot <= 0) {
    return;
  }
  coba->cur = clamp_i(coba->cur, 0, coba->tot - 1);
  CBData *active = &coba->data[coba->cur];
  for (Button &but : block->buttons()) {
    if (colorband_data_contains(coba, but.rnapoin.data)) {
      but.rnapoin.data = active;
    }
  }
}

static void colorband_tag_redraw(bContext &C, Block *block)
{
  ARegion *region = (block && block->handle) ? block->handle->region : nullptr;
  if (region == nullptr) {
    region = CTX_wm_region_popup(&C);
  }
  if (region == nullptr) {
    region = CTX_wm_region(&C);
  }
  if (region) {
    ED_region_tag_redraw(region);
  }
}

static void colorband_update_cb(bContext *C, void *bt_v, void *coba_v)
{
  Button *bt = static_cast<Button *>(bt_v);
  ColorBand *coba = static_cast<ColorBand *>(coba_v);

  /* Sneaky update here, we need to sort the color-band points to be in order,
   * however the RNA pointer then is wrong, so we update it */
  BKE_colorband_update_sort(coba);
  colorband_retarget_element_buttons(bt->block, coba);
  if (C != nullptr) {
    colorband_tag_redraw(*C, bt->block);
  }
}

static void colorband_buttons_layout(Layout &layout,
                                     Block *block,
                                     ColorBand *coba,
                                     const rctf *butr,
                                     const RNAUpdateCb &cb,
                                     int expand,
                                     bool presets)
{
  Button *bt;
  const float unit = BLI_rctf_size_x(butr) / 14.0f;
  const float xs = butr->xmin;
  const float ys = butr->ymin;

  PointerRNA ptr = RNA_pointer_create_discrete(cb.ptr.owner_id, RNA_ColorRamp, coba);

  Layout *split = &layout.split(0.4f, false);

  block_emboss_set(block, EmbossType::None);
  block_align_begin(block);
  Layout *row = &split->row(false);

  bt = uiDefIconTextBut(block,
                        ButtonType::But,
                        ICON_ADD,
                        "",
                        0,
                        0,
                        2.0f * unit,
                        UI_UNIT_Y,
                        nullptr,
                        TIP_("Add a new color stop to the color ramp"));
  button_func_set(bt, [coba, cb](bContext &C) { colorband_add(C, cb, *coba); });

  bt = uiDefIconTextBut(block,
                        ButtonType::But,
                        ICON_REMOVE,
                        "",
                        xs + 2.0f * unit,
                        ys + UI_UNIT_Y,
                        2.0f * unit,
                        UI_UNIT_Y,
                        nullptr,
                        TIP_("Delete the active position"));
  button_func_set(bt, [coba, cb](bContext &C) {
    if (BKE_colorband_element_remove(coba, coba->cur)) {
      rna_update_cb(C, cb);
      ED_undo_push(&C, "Delete Color Ramp Stop");
    }
  });
  if (coba->tot < 2) {
    button_flag_enable(bt, BUT_DISABLED);
    button_func_tooltip_custom_set(bt, colorramp_disabled_tip_func, nullptr, nullptr);
  }

  RNAUpdateCb *tools_cb = MEM_new<RNAUpdateCb>(__func__, cb);
  bt = uiDefIconBlockBut(block,
                         colorband_tools_fn,
                         tools_cb,
                         ICON_DOWNARROW_HLT,
                         xs + 4.0f * unit,
                         ys + UI_UNIT_Y,
                         2.0f * unit,
                         UI_UNIT_Y,
                         TIP_("Tools"));
  /* Pass ownership of `tools_cb` to the button. */
  button_funcN_set(
      bt,
      [](bContext *, void *, void *) {},
      tools_cb,
      nullptr,
      but_func_argN_free<RNAUpdateCb>,
      but_func_argN_copy<RNAUpdateCb>);

  block_align_end(block);
  block_emboss_set(block, EmbossType::Emboss);

  row = &split->row(false);

  block_align_begin(block);
  row->prop(&ptr, "color_mode", UI_ITEM_NONE, "", ICON_NONE);
  if (ELEM(coba->color_mode, COLBAND_BLEND_HSV, COLBAND_BLEND_HSL)) {
    row->prop(&ptr, "hue_interpolation", UI_ITEM_NONE, "", ICON_NONE);
  }
  else { /* COLBAND_BLEND_RGB */
    row->prop(&ptr, "interpolation", UI_ITEM_NONE, "", ICON_NONE);
  }
  block_align_end(block);

  row = &layout.row(false);

  bt = uiDefBut(
      block, ButtonType::ColorBand, "", xs, ys, BLI_rctf_size_x(butr), UI_UNIT_Y, coba, 0, 0, "");
  bt->rnapoin = cb.ptr;
  bt->rnaprop = cb.prop;
  button_func_set(bt, [cb, coba, block](bContext &C) {
    rna_update_cb(C, cb);
    colorband_retarget_element_buttons(block, coba);
    colorband_tag_redraw(C, block);
  });

  row = &layout.row(false);

  if (coba->tot) {
    coba->cur = clamp_i(coba->cur, 0, coba->tot - 1);
    CBData *cbd = coba->data + coba->cur;

    ptr = RNA_pointer_create_discrete(cb.ptr.owner_id, RNA_ColorRampElement, cbd);

    if (!expand) {
      split = &layout.split(0.3f, false);

      row = &split->row(false);
      bt = uiDefButV(block,
                     ButtonType::Num,
                     "",
                     0,
                     0,
                     5.0f * UI_UNIT_X,
                     UI_UNIT_Y,
                     &coba->cur,
                     0.0,
                     float(std::max(0, coba->tot - 1)),
                     TIP_("Choose active color stop"));
      button_number_step_size_set(bt, 1);

      row = &split->row(false);
      row->prop(&ptr, "position", UI_ITEM_NONE, IFACE_("Pos"), ICON_NONE);

      row = &layout.row(false);
      row->prop(&ptr, "color", UI_ITEM_NONE, "", ICON_NONE);
    }
    else {
      split = &layout.split(0.5f, false);
      Layout &subsplit = split->split(0.35f, false);

      row = &subsplit.row(false);
      bt = uiDefButV(block,
                     ButtonType::Num,
                     "",
                     0,
                     0,
                     5.0f * UI_UNIT_X,
                     UI_UNIT_Y,
                     &coba->cur,
                     0.0,
                     float(std::max(0, coba->tot - 1)),
                     TIP_("Choose active color stop"));
      button_number_step_size_set(bt, 1);

      row = &subsplit.row(false);
      row->prop(&ptr, "position", ITEM_R_SLIDER, IFACE_("Pos"), ICON_NONE);

      row = &split->row(false);
      row->prop(&ptr, "color", UI_ITEM_NONE, "", ICON_NONE);
    }

    /* Some special (rather awkward) treatment to update UI state on certain property changes. */
    for (Button &but : block->buttons() | std::views::reverse) {
      if (but.rnapoin.data != ptr.data) {
        continue;
      }
      if (!but.rnaprop) {
        continue;
      }

      const char *prop_identifier = RNA_property_identifier(but.rnaprop);
      if (STREQ(prop_identifier, "position")) {
        button_func_set(&but, colorband_update_cb, &but, coba);
      }

      if (STREQ(prop_identifier, "color")) {
        button_func_set(&but, [cb](bContext &C) { rna_update_cb(C, cb); });
      }
    }

    /* `bt` is the active-stop index. Rebind color and position after it changes. */
    button_func_set(bt, [cb, coba, block](bContext &C) {
      rna_update_cb(C, cb);
      colorband_retarget_element_buttons(block, coba);
      colorband_tag_redraw(C, block);
    });
  }

  if (presets) {
    colorband_presets_layout(layout, block, coba, cb);
  }
}

void template_color_ramp(Layout *layout,
                         PointerRNA *ptr,
                         const StringRefNull propname,
                         bool expand,
                         bool presets)
{
  PropertyRNA *prop = RNA_struct_find_property(ptr, propname.c_str());

  if (!prop || RNA_property_type(prop) != PROP_POINTER) {
    return;
  }

  const PointerRNA cptr = RNA_property_pointer_get(ptr, prop);
  if (!cptr || !RNA_struct_is_a(cptr.type, RNA_ColorRamp)) {
    return;
  }

  rctf rect;
  rect.xmin = 0;
  rect.xmax = 10.0f * UI_UNIT_X;
  rect.ymin = 0;
  rect.ymax = 19.5f * UI_UNIT_X;

  Block *block = layout->absolute().block();

  ID *id = cptr.owner_id;
  block_lock_set(block, (id && !ID_IS_EDITABLE(id)), ERROR_LIBDATA_MESSAGE);

  colorband_buttons_layout(*layout,
                           block,
                           static_cast<ColorBand *>(cptr.data),
                           &rect,
                           RNAUpdateCb{*ptr, prop},
                           expand,
                           presets);

  block_lock_clear(block);
}

}  // namespace blender::ui
