/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup buttons
 */

#include <algorithm>
#include <cmath>
#include <fmt/format.h>

#include "BKE_screen.hh"

#include "BLF_api.hh"

#include "BLI_listbase.hh"
#include "BLI_listbase_iterator.hh"
#include "BLI_rect.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"

#include "DNA_screen_types.h"
#include "DNA_theme_types.h"
#include "DNA_userdef_types.h"

#include "RNA_access.hh"

#include "UI_interface_c.hh"

#include "interface_intern.hh"
#include "interface_textbox.hh"

namespace blender::ui {

static int textbox_code_cell_w(int fontid);

float textbox_code_font_scale(const ButtonTextBox *textbox)
{
  if (textbox && textbox->state && textbox->state->font_scale > 0.05f) {
    return std::clamp(textbox->state->font_scale, 0.5f, 4.0f);
  }
  return 1.0f;
}

static int textbox_blf_fontid(const ButtonTextBox *textbox, const uiFontStyle &fstyle)
{
  if ((textbox->flag & BUT_CODE_EDITOR) && blf_mono_font >= 0) {
    const float px = textbox->last_code_font_px > 0.5f ?
                         textbox->last_code_font_px :
                         std::max(fstyle.points * UI_SCALE_FAC * textbox_code_font_scale(textbox),
                                  1.0f);
    BLF_size(blf_mono_font, px);
    return blf_mono_font;
  }
  return fstyle.uifont_id;
}

void textbox_code_sync_font(ButtonTextBox *textbox, const int text_w_px, const int text_h_px)
{
  const float aspect = std::max(textbox->block ? textbox->block->aspect : 1.0f, 0.0001f);
  const uiFontStyle &widget = style_get()->widget;
  /* Same zoom law as every other node-editor widget: points * UI_SCALE / aspect.
   * Deriving size from row pixels fought view2d (padding / scrollbar rounding)
   * and made glyphs drift off the code box while zooming. */
  const float font_px = std::max(
      1.0f, widget.points * UI_SCALE_FAC / aspect * textbox_code_font_scale(textbox));
  textbox->last_code_font_px = font_px;
  if (blf_mono_font >= 0) {
    BLF_size(blf_mono_font, font_px);
  }
  const CodeLineMetrics m = textbox_code_metrics(blf_mono_font,
                                                 std::max(textbox->last_total_lines, 1));
  const int gutter = textbox_code_gutter_for_width(m, text_w_px);
  textbox->last_code_gutter = gutter;
  textbox->last_code_cell_w = (blf_mono_font >= 0) ? textbox_code_cell_w(blf_mono_font) : 0;
  textbox->last_code_visible_w = std::max(8, text_w_px - gutter);
  /* Line spacing tracks the font, not the widget packed into visible_lines slots. */
  textbox->last_code_line_h = std::max(1.0f, m.line_h);
  textbox->last_code_rows_fit = std::max(
      1, int(std::floor(float(std::max(text_h_px, 1)) / textbox->last_code_line_h)));
}

void invalidate_text_wrap_cache(const ARegion &region)
{
  for (Block &block : region.runtime->uiblocks) {
    block.text_wrap_cache.clear();
    block.markdown_layout_cache.clear();
    for (Button &button : block.buttons()) {
      if (button.type == ButtonType::TextBox) {
        auto &textbox = static_cast<ButtonTextBox &>(button);
        textbox.wrap_cache.reset();
        textbox.placeholder_wrap_cache.reset();
      }
      if (button.type == ButtonType::Label) {
        auto &label = static_cast<ButtonLabel &>(button);
        label.wrap_cache.reset();
        label.markdown_cache.reset();
      }
    }
  }
}

void textbox_add_scroll(ButtonTextBox *textbox, int step)
{
  textbox_wrap_lines(textbox);
  textbox->line_scroll_set(textbox->line_scroll() + step);
}

void textbox_add_col_scroll(ButtonTextBox *textbox, int pixels)
{
  textbox_wrap_lines(textbox);
  textbox->col_scroll_set(textbox->col_scroll() + pixels);
}

float textbox_hscroll_height()
{
  return UI_UNIT_Y * 0.45f;
}

float textbox_vscroll_width(const ButtonTextBox *textbox)
{
  const float pad = float(button_text_padding(textbox));
  const float aspect = (textbox && textbox->block) ? std::max(textbox->block->aspect, 0.0001f) :
                                                     1.0f;
  if (textbox && (textbox->flag & BUT_CODE_EDITOR)) {
    /* ~12px at default widget size, so the thumb is actually grab-able. */
    return std::max(pad, 0.65f * UI_UNIT_X / aspect);
  }
  return pad;
}

void textbox_scroll_win_rects(const ButtonTextBox *textbox,
                              const rctf *win_rect,
                              const float aspect,
                              rctf *r_vscroll,
                              rctf *r_hscroll,
                              rctf *r_grip)
{
  const float aspect_safe = std::max(aspect, 0.0001f);
  const float pad = 2.0f / aspect_safe;
  const float vscroll_w = textbox_vscroll_width(textbox);
  const float grip_h = textbox_grip_height() / aspect_safe;
  const bool is_code = (textbox->flag & BUT_CODE_EDITOR) != 0;
  const float grip_w = is_code ? std::max(vscroll_w, 0.9f * UI_UNIT_X / aspect_safe) : vscroll_w;
  const float grip_hit_h = is_code ? std::max(grip_h, 0.75f * UI_UNIT_Y / aspect_safe) : grip_h;
  const float hscroll_h = textbox_hscroll_height() / aspect_safe;
  /* Code editors need a wide slop: the caret otherwise steals the first press. */
  const float hit_extra = is_code ? (0.5f * UI_UNIT_X / aspect_safe) : (4.0f / aspect_safe);

  if (r_grip) {
    *r_grip = *win_rect;
    r_grip->xmin = r_grip->xmax - grip_w - pad;
    r_grip->ymax = r_grip->ymin + grip_hit_h;
  }
  if (r_hscroll) {
    *r_hscroll = *win_rect;
    r_hscroll->xmin += pad;
    r_hscroll->xmax -= pad + grip_w;
    r_hscroll->ymin += pad;
    r_hscroll->ymax = r_hscroll->ymin + std::max(4.0f, hscroll_h);
    /* Extra pixels toward the text so the drawn thumb is easy to grab. */
    r_hscroll->ymax += hit_extra;
  }
  if (r_vscroll) {
    *r_vscroll = *win_rect;
    r_vscroll->xmax -= pad;
    r_vscroll->xmin = r_vscroll->xmax - vscroll_w;
    r_vscroll->ymin += pad + grip_h;
    r_vscroll->ymax -= pad;
    r_vscroll->xmin -= hit_extra;
  }
}

int textbox_scroll_hit_test(const ButtonTextBox *textbox,
                            const rctf *win_rect,
                            const float aspect,
                            const int xy[2])
{
  rctf vscroll, hscroll, grip;
  textbox_scroll_win_rects(textbox, win_rect, aspect, &vscroll, &hscroll, &grip);
  const bool is_code = (textbox->flag & BUT_CODE_EDITOR) != 0;
  if (BLI_rctf_isect_pt(&grip, xy[0], xy[1])) {
    return TEXTBOX_SCROLL_HIT_GRIP;
  }
  if (is_code && BLI_rctf_isect_pt(&hscroll, xy[0], xy[1])) {
    return TEXTBOX_SCROLL_HIT_H;
  }
  if ((is_code || textbox->last_total_lines > textbox->drawn_rows()) &&
      BLI_rctf_isect_pt(&vscroll, xy[0], xy[1]))
  {
    return TEXTBOX_SCROLL_HIT_V;
  }
  return TEXTBOX_SCROLL_HIT_NONE;
}

static int textbox_code_cell_w(const int fontid)
{
  return std::max(1, int(std::lround(double(BLF_fixed_width(fontid)))));
}

static int textbox_code_char_columns(const char *s)
{
  if (s == nullptr || s[0] == '\0' || s[0] == '\n' || s[0] == '\r') {
    return 0;
  }
  if (s[0] == '\t') {
    return 2;
  }
  return std::max(1, BLI_str_utf8_char_width_safe(s));
}

int textbox_code_x_from_offset(const int fontid,
                               const StringRef line,
                               const int offset,
                               const int cell_w)
{
  const int n = std::clamp(offset, 0, int(line.size()));
  if (n <= 0 || line.is_empty()) {
    return 0;
  }
  const int cell = cell_w > 0 ? cell_w : textbox_code_cell_w(fontid);
  int cols = 0;
  int i = 0;
  const char *s = line.data();
  while (i < n) {
    const int sz = std::max(1, BLI_str_utf8_size_safe(s + i));
    cols += textbox_code_char_columns(s + i);
    i += sz;
  }
  return cols * cell;
}

int textbox_code_offset_from_x(const int fontid,
                               const StringRef line,
                               const float x,
                               const int cell_w)
{
  const int cell = cell_w > 0 ? cell_w : textbox_code_cell_w(fontid);
  if (line.is_empty() || cell <= 0 || x <= 0.0f) {
    return 0;
  }
  /* Caret slots are the voids between characters (spaces count). Pick the
   * nearest gap: before the first glyph, between each pair, and after the last. */
  const int n = int(line.size());
  const char *s = line.data();
  int best_off = 0;
  float best_dist = std::abs(x);
  int cols = 0;
  int i = 0;
  while (i < n) {
    const int sz = std::max(1, BLI_str_utf8_size_safe(s + i));
    const int w = textbox_code_char_columns(s + i);
    if (w > 0) {
      cols += w;
      const float gap_x = float(cols * cell);
      const float d = std::abs(x - gap_x);
      if (d < best_dist) {
        best_dist = d;
        best_off = i + sz;
      }
    }
    i += sz;
  }
  return best_off;
}

CodeLineMetrics textbox_code_metrics(const int fontid, const int total_lines)
{
  CodeLineMetrics m;
  const int asc = BLF_ascender(fontid);
  const int desc = BLF_descender(fontid);
  m.descender = desc;
  m.em = std::max(1.0f, float(asc - desc));
  /* Tight em-box plus a little leading so the caret hugs the glyphs instead of
   * sitting in FreeType's extra line-gap above the letters. */
  m.pad = std::max(1.0f, 0.18f * m.em);
  m.line_h = m.em + m.pad;
  int digits = 1;
  for (int n = std::max(total_lines, 1); n >= 10; n /= 10) {
    digits++;
  }
  digits = std::clamp(digits, 2, 6);
  /* Gutter follows the font (em), not a fixed UI_SCALE pixel pad — otherwise zooming
   * the node editor leaves a huge black strip and hides the left of the code. */
  m.gutter = textbox_code_cell_w(fontid) * digits +
             int(std::lround(std::max(m.em * 0.4f, 1.0f)));
  return m;
}

void textbox_scroll_to_cursor(ButtonTextBox *textbox)
{
  const Vector<StringRef> lines = textbox_wrap_lines(textbox);
  int but_pos = textbox->pos;
#ifdef WITH_INPUT_IME
  /* Include the ime composition string when scrolling to the cursor. */
  const wmIMEData *ime_data = button_ime_data_get(textbox);
  if (ime_data && !ime_data->composite.empty() && ime_data->cursor_pos != -1) {
    but_pos += ime_data->cursor_pos;
  }
#endif
  const int line_cursor = textbox_wrapped_line_index_from_char_offset(lines, but_pos);
  const int visible_bounds[] = {textbox->line_scroll(),
                                textbox->line_scroll() + textbox->drawn_rows()};
  if (visible_bounds[0] > line_cursor) {
    textbox_add_scroll(textbox, line_cursor - visible_bounds[0]);
  }
  else if (line_cursor >= visible_bounds[1]) {
    textbox_add_scroll(textbox, line_cursor - visible_bounds[1] + 1);
  }

  if ((textbox->flag & BUT_CODE_EDITOR) && line_cursor >= 0 && line_cursor < lines.size()) {
    uiFontStyle fstyle = style_get()->widget;
    fontscale(&fstyle.points, textbox->block->aspect);
    fontstyle_set(&fstyle);
    const int fontid = textbox_blf_fontid(textbox, fstyle);
    const int caret_px = textbox_code_x_from_offset(
        fontid,
        lines[line_cursor],
        std::max(0, but_pos - int(lines[line_cursor].begin() - lines[0].begin())),
        textbox->last_code_cell_w);
    const int visible_w = textbox->last_code_visible_w > 0 ? textbox->last_code_visible_w : 8;
    const int sx = textbox->col_scroll();
    if (caret_px < sx) {
      textbox->col_scroll_set(caret_px - 8);
    }
    else if (caret_px > sx + visible_w) {
      textbox->col_scroll_set(caret_px - visible_w + 16);
    }
  }
}

void textbox_textedit_set_cursor_pos(ButtonTextBox *textbox,
                                     const ARegion *region,
                                     const float2 xy)
{
  const float aspect = textbox->block->aspect;
  const Vector<StringRef> lines = textbox_wrap_lines(textbox);
  uiFontStyle fstyle = style_get()->widget;
  fontscale(&fstyle.points, aspect);
  fontstyle_set(&fstyle);
  const int fontid = textbox_blf_fontid(textbox, fstyle);
  const bool is_code = (textbox->flag & BUT_CODE_EDITOR) && blf_mono_font >= 0;
  const int text_pad = (textbox->drawflag & BUT_NO_TEXT_PADDING) ? 0 :
                                                                  button_text_padding(textbox);

  int position = 0;
  if (is_code) {
    /* Hit-test in the same region-pixel space the last draw used for glyphs.
     * Re-deriving gutter/padding here drifted by ~1 cell vs the line-number strip. */
    rcti rect = rect_to_pixelrect(region, textbox->block, &textbox->rect);
    const int scrollbar_pad = int(std::lround(2.0f / aspect));
    const int vscroll_w = std::max(text_pad, int(std::lround(textbox_vscroll_width(textbox))));
    rect.xmin += text_pad;
    rect.xmax = std::max(rect.xmin, rect.xmax - vscroll_w - scrollbar_pad);
    rect.ymax -= int(textbox_vertical_padding() / aspect);
    rect.ymin += int(textbox_vertical_padding() / aspect);
    rect.ymin += int(textbox_hscroll_height() / aspect);
    if (textbox->last_code_font_px > 0.5f) {
      BLF_size(blf_mono_font, textbox->last_code_font_px);
    }
    const int gutter = textbox->last_code_gutter > 0 ?
                           textbox->last_code_gutter :
                           textbox_code_gutter_for_width(
                               textbox_code_metrics(fontid, textbox->last_total_lines),
                               BLI_rcti_size_x(&rect));
    const float rx = xy.x - float(region->winrct.xmin);
    const float ry = xy.y - float(region->winrct.ymin);
    const float origin_x = textbox->last_code_hit_valid ? float(textbox->last_code_origin_x) :
                                                          float(rect.xmin + gutter);
    const float origin_y = textbox->last_code_hit_valid ? float(textbox->last_code_origin_y) :
                                                          float(rect.ymax);
    const float line_h = textbox->last_code_line_h > 0.5f ?
                             textbox->last_code_line_h :
                             textbox_row_height_px(rect, textbox->drawn_rows());
    int line_under_mouse = textbox->line_scroll() +
                           int(std::floor((origin_y - ry) / std::max(line_h, 1.0f)));
    line_under_mouse = std::clamp<int>(line_under_mouse,
                                       textbox->line_scroll(),
                                       textbox->line_scroll() + textbox->drawn_rows() - 1);
    line_under_mouse = std::clamp<int>(line_under_mouse, 0, int(lines.size()) - 1);
    const StringRef line = lines[line_under_mouse];
    const int hscroll = textbox->col_scroll();
    const float local_x = (rx - origin_x) + float(hscroll);
    const int offset = textbox_code_offset_from_x(
        fontid, line, local_x, textbox->last_code_cell_w);
    position = int(line.begin() - lines[0].data()) + offset;
  }
  else {
    /* Don't include grip bounds when selecting text with the mouse. */
    float2 start = {textbox->rect.xmin, textbox->rect.ymin};
    float2 end = {textbox->rect.xmax, textbox->rect.ymax};

    block_to_window_fl(region, textbox->block, &start.x, &start.y);
    block_to_window_fl(region, textbox->block, &end.x, &end.y);

    start.y += textbox_vertical_padding() / aspect;
    end.y -= textbox_vertical_padding() / aspect;
    start.x += text_pad;
    end.x -= text_pad;

    const float line_h = std::max(
        1.0f, (end.y - start.y) / float(std::max(textbox->visible_lines(), 1)));
    int line_under_mouse = textbox->line_scroll() + int((end.y - xy.y) / line_h);
    line_under_mouse = std::clamp<int>(line_under_mouse,
                                       textbox->line_scroll(),
                                       textbox->line_scroll() + textbox->visible_lines() - 1);
    line_under_mouse = std::clamp<int>(line_under_mouse, 0, int(lines.size()) - 1);

    const StringRef line = lines[line_under_mouse];
    const int offset = BLF_str_offset_from_cursor_position(
        fontid, line.data(), line.size(), int(xy.x - start.x));
    position = int(line.begin() - lines[0].data()) + offset;
  }
#ifdef WITH_INPUT_IME
  /* Text-box text wrap includes the IME composition string, remove the IME string pad from the
   * selection. */
  const wmIMEData *ime_data = button_ime_data_get(textbox);
  if (ime_data && position > int(textbox->pos)) {
    position = std::max<int>(int(textbox->pos), position - int(ime_data->composite.size()));
  }
#endif
  textbox->pos = position;
  /* Do not scroll to cursor now, wait the HandleButtonData::text_select_auto_scroll timer or the
   * #LEFTMOUSE release event for scrolling to the cursor. */
}

int textbox_wrapped_line_index_from_char_offset(Span<StringRef> lines, int offset)
{
  const char *dest = lines.first().begin() + offset;
  int i = 0;
  for (const StringRef line : lines) {
    if (line.begin() > dest) {
      i = i - 1;
      break;
    }
    i++;
  }
  i = std::clamp<int>(i, 0, lines.size() - 1);
  return i;
}

void textbox_jump_line(ButtonTextBox *textbox,
                       eStrCursorJumpDirection direction,
                       const bool select)
{
  button_update(textbox);
  if (textbox->selend == textbox->selsta) {
    textbox->selsta = textbox->selend = textbox->pos;
  }
  const Vector<StringRef> lines = textbox_wrap_lines(textbox);
  const char *str = lines.first().begin();
  const bool append_selection = textbox->selend == textbox->pos;
  const int line_cursor = textbox_wrapped_line_index_from_char_offset(lines, textbox->pos);
  const int col = std::max(0, textbox->pos - int(lines[line_cursor].begin() - str));
  uiFontStyle fstyle = style_get()->widget;
  fontscale(&fstyle.points, textbox->block->aspect);
  fontstyle_set(&fstyle);
  const int fontid = textbox_blf_fontid(textbox, fstyle);
  const bool is_code = (textbox->flag & BUT_CODE_EDITOR) && blf_mono_font >= 0;
  StringRef dest_line = nullptr;
  if (direction == STRCUR_DIR_NEXT) {
    if (line_cursor == lines.size() - 1) {
      textbox->pos = lines.last().end() - str;
    }
    else {
      dest_line = lines[line_cursor + 1];
    }
  }
  else {
    if (line_cursor == 0) {
      textbox->pos = 0;
    }
    else {
      dest_line = lines[line_cursor - 1];
    }
  }
  if (dest_line.data()) {
    int dest_off = 0;
    if (is_code) {
      /* Monospace: keep the same character column. Pixel round-trip via
       * BLF_width_to_strlen is off-by-one and walked the caret left each time. */
      dest_off = std::min(col, int(dest_line.size()));
    }
    else {
      const int offset = BLF_str_offset_to_cursor(fontid,
                                                  lines[line_cursor].begin(),
                                                  lines[line_cursor].size(),
                                                  col,
                                                  0);
      dest_off = int(BLF_str_offset_from_cursor_position(
          fontid, dest_line.data(), dest_line.size(), offset));
    }
    textbox->pos = dest_line.begin() - str + dest_off;
  }
  if (!select) {
    textbox->selsta = textbox->selend = textbox->pos;
    return;
  }

  if (append_selection) {
    textbox->selend = textbox->pos;
  }
  else {
    textbox->selsta = textbox->pos;
  }
  if (textbox->selend < textbox->selsta) {
    std::swap(textbox->selend, textbox->selsta);
  }
}

Vector<StringRef> textbox_wrap_lines(ButtonTextBox *textbox)
{
  uiFontStyle fstyle = style_get()->widget;
  const float aspect = textbox->block->aspect;
  const int width = std::max<int>(std::ceil(BLI_rctf_size_x(&textbox->rect) -
                                            2.0f * UI_TEXT_MARGIN_X * float(U.widget_unit) - 2.0f),
                                  0) /
                    aspect;
  StringRef text = textbox->drawstr;
#ifdef WITH_INPUT_IME
  std::string ime_text;
  const wmIMEData *ime_data = button_ime_data_get(textbox);
  if (ime_data && !ime_data->composite.empty()) {
    StringRef edit_str = textbox->editstr;
    StringRef l = edit_str.is_empty() ? StringRef("") : edit_str.substr(0, textbox->pos);
    StringRef r = edit_str.is_empty() ? StringRef("") : edit_str.substr(textbox->pos);
    ime_text = fmt::format("{}{}{}", l, ime_data->composite, r);
    text = ime_text;
  }
  else
#endif
      if (textbox->editstr)
  {
    text = textbox->editstr;
  }
  if (!textbox->wrap_cache) {
    textbox->wrap_cache = std::make_unique<TextWrapCache>();
  }
  TextWrapCache &cache = *textbox->wrap_cache;
  const float font_scale = textbox_code_font_scale(textbox);
  float code_font_px = textbox->last_code_font_px;
  if (textbox->flag & BUT_CODE_EDITOR) {
    const int pad = button_text_padding(textbox);
    const int w_px = std::max(
        8, int(std::lround((BLI_rctf_size_x(&textbox->rect) - 2.0f * pad) / aspect)));
    const int h_px = std::max(8, int(std::lround(BLI_rctf_size_y(&textbox->rect) / aspect)));
    const int vpad = int(std::lround(textbox_vertical_padding() / aspect)) * 2;
    const int hscroll_h = int(std::lround(textbox_hscroll_height() / aspect));
    /* The number of rows that fit was measured by the last draw on the real pixel rectangle.
     * The estimate made here can be one row more, and scrolling to the caret would then leave
     * the line that is being typed just below the visible rows. */
    const int drawn_rows_fit = textbox->last_code_rows_fit;
    textbox_code_sync_font(textbox, w_px, std::max(1, h_px - vpad - hscroll_h));
    if (drawn_rows_fit > 0) {
      textbox->last_code_rows_fit = std::min(textbox->last_code_rows_fit, drawn_rows_fit);
    }
    code_font_px = textbox->last_code_font_px;
  }
  if (cache.aspect == aspect && cache.wrap_width == width && text == cache.text &&
      cache.font_scale == font_scale && cache.code_font_px == code_font_px)
  {
    textbox->last_total_lines = cache.wrapped_lines.size();
    return cache.wrapped_lines;
  }
  cache.text = text;
  text = cache.text;
  cache.wrap_width = width;
  cache.aspect = aspect;
  cache.font_scale = font_scale;
  cache.code_font_px = code_font_px;

  if (textbox->flag & BUT_CODE_EDITOR) {
    fontscale(&fstyle.points, aspect);
    fontstyle_set(&fstyle);
    const int fontid = textbox_blf_fontid(textbox, fstyle);
    Vector<StringRef> lines;
    const char *p = text.data();
    const char *end = p + text.size();
    const char *start = p;
    for (; p != end; p++) {
      if (*p == '\n') {
        lines.append(StringRef(start, p));
        start = p + 1;
      }
    }
    lines.append(StringRef(start, end));
    /* A source that ends with `\n` already produced an empty last line via the
     * append above (start == end). A second empty line parks the caret below
     * the row the user is on until they type a character. */
    if (lines.is_empty()) {
      lines.append(text);
    }
    int max_px = 0;
    for (const StringRef line : lines) {
      max_px = std::max(max_px, int(BLF_width(fontid, line.data(), line.size())));
    }
    /* Extra em so the last glyph isn't flush against the vertical scrollbar. */
    const CodeLineMetrics m = textbox_code_metrics(fontid, int(lines.size()));
    textbox->last_max_line_px = max_px + int(std::lround(m.em));
    textbox->last_total_lines = int(lines.size());
    cache.wrapped_lines = std::move(lines);
    return cache.wrapped_lines;
  }

  fontscale(&fstyle.points, aspect);
  fontstyle_set(&fstyle);
  Vector<StringRef> lines = BLF_string_wrap(
      fstyle.uifont_id, text, width, BLFWrapMode::HardLimit | BLFWrapMode::Typographical);
  if (lines.is_empty()) {
    lines.append(text);
  }
  /* Add empty trailing line to put cursor in a new line. */
  if (text.endswith("\n")) {
    lines.append(StringRef(text.end(), text.end()));
  }
  textbox->last_total_lines = lines.size();

  /* WORKAROUND: Text-box event handling and drawing requires lines to not include line breaks,
   * but sometimes text wrap adds them and other times not. */
  for (int i : lines.index_range()) {
    if (lines[i].endswith("\n")) {
      lines[i] = lines[i].drop_suffix(1);
    }
  }

  cache.wrapped_lines = lines;

  return lines;
}

Vector<StringRef> textbox_wrap_placeholder(ButtonTextBox *textbox)
{
  BLI_assert(textbox->placeholder);
  uiFontStyle fstyle = style_get()->widget;
  const float aspect = textbox->block->aspect;
  const int width = std::max<int>(std::ceil(BLI_rctf_size_x(&textbox->rect) -
                                            2.0f * UI_TEXT_MARGIN_X * float(U.widget_unit) - 2.0f),
                                  0) /
                    aspect;
  StringRef text = textbox->placeholder;

  if (!textbox->placeholder_wrap_cache) {
    textbox->placeholder_wrap_cache = std::make_unique<TextWrapCache>();
  }
  TextWrapCache &cache = *textbox->placeholder_wrap_cache;
  if (cache.aspect == aspect && cache.wrap_width == width && text == cache.text) {
    return cache.wrapped_lines;
  }
  cache.text = text;
  cache.wrap_width = width;
  cache.aspect = aspect;

  fontscale(&fstyle.points, aspect);
  fontstyle_set(&fstyle);

  cache.wrapped_lines = BLF_string_wrap(
      fstyle.uifont_id, cache.text, width, BLFWrapMode::HardLimit | BLFWrapMode::Typographical);

  return cache.wrapped_lines;
}

float textbox_grip_height()
{
  return UI_UNIT_Y * 0.55f;
}

void ButtonTextBox::line_scroll_set(int line_scroll)
{
  this->state->scroll = line_scroll;
  /* Clamp stored value. */
  this->state->scroll = this->line_scroll();
}

void ButtonTextBox::col_scroll_set(int col_scroll)
{
  this->state->scroll_x = col_scroll;
  this->state->scroll_x = this->col_scroll();
}

float textbox_vertical_padding()
{
  /* Allow aligning text buttons with single line text-box buttons. */
  return float(UI_UNIT_Y - fontstyle_height_max(UI_FSTYLE_WIDGET)) / 2.0f;
}

TextboxState *textbox_ensure_state(ARegion *region,
                                   StringRefNull idname,
                                   const int initial_visible_lines)
{
  for (uiTextboxStateLink &link : region->textbox_states) {
    if (link.idname == idname) {
      return &link.state;
    }
  }
  uiTextboxStateLink *link = MEM_new<uiTextboxStateLink>(__func__);
  link->idname = BLI_strdupn(idname.data(), idname.size());
  link->state.visible_lines = std::max(initial_visible_lines, textbox_minimum_visible_lines);
  BLI_addtail(&region->textbox_states, link);
  return &link->state;
}

int ButtonTextBox::line_scroll() const
{
  const int max_scroll = std::max(this->last_total_lines - this->drawn_rows(), 0);
  return std::clamp(this->state->scroll, 0, max_scroll);
}

int ButtonTextBox::col_scroll() const
{
  const int visible_w = this->last_code_visible_w > 0 ? this->last_code_visible_w : 8;
  const int max_scroll = std::max(this->last_max_line_px - visible_w, 0);
  return std::clamp(this->state->scroll_x, 0, max_scroll);
}

int ButtonTextBox::visible_lines() const
{
  return std::max<int>(this->state->visible_lines, textbox_minimum_visible_lines);
}

int ButtonTextBox::drawn_rows() const
{
  if ((this->flag & BUT_CODE_EDITOR) && this->last_code_rows_fit > 0) {
    return this->last_code_rows_fit;
  }
  return this->visible_lines();
}

int textbox_but_height(const TextboxState &state)
{
  const int visible_lines = std::max(state.visible_lines, textbox_minimum_visible_lines);
  const float line_height = fontstyle_height_max(UI_FSTYLE_WIDGET);
  return std::max<int>(
      UI_UNIT_Y, int(std::round(line_height * visible_lines + textbox_vertical_padding() * 2.0f)));
}

Button *uiDefButTextBoxR(Block *block,
                         PointerRNA *ptr,
                         StringRefNull propname,
                         TextboxState *state,
                         const int x,
                         const int y,
                         const short width)
{
  BLI_assert(state);

  PropertyRNA *prop = RNA_struct_find_property_check(*ptr, propname.c_str(), PROP_STRING);
  if (!prop) {
    RNA_warning(
        "string property not found: %s.%s", RNA_struct_identifier(ptr->type), propname.c_str());
    return nullptr;
  }

  state->visible_lines = std::max(state->visible_lines, textbox_minimum_visible_lines);

  Button *but = uiDefButR_prop(block,
                               ButtonType::TextBox,
                               RNA_property_ui_name(prop),
                               x,
                               y,
                               width,
                               textbox_but_height(*state),
                               ptr,
                               prop,
                               0,
                               0,
                               0,
                               std::nullopt);

  ButtonTextBox *textbox = static_cast<ButtonTextBox *>(but);
  textbox->state = state;

  if (RNA_property_flag(prop) & PROP_TEXTEDIT_UPDATE) {
    button_flag_enable(but, BUT_TEXTEDIT_UPDATE);
  }

  return but;
}

}  // namespace blender::ui
