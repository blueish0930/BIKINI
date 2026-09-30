/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup buttons
 */

#pragma once

#include <algorithm>

#include "BLI_math_vector_types.hh"
#include "BLI_rect.hh"
#include "BLI_span.hh"
#include "BLI_string_cursor_utf8.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

namespace blender {
struct ARegion;
struct TextboxState;

namespace ui {

struct ButtonTextBox;

constexpr int textbox_minimum_visible_lines = 1;

void textbox_add_scroll(ButtonTextBox *textbox, int step);
void textbox_add_col_scroll(ButtonTextBox *textbox, int pixels);

/** Scroll the textbox to make the text cursor visible. */
void textbox_scroll_to_cursor(ButtonTextBox *textbox);

float textbox_hscroll_height();
/** Visual + hit width of the vertical scrollbar (wider for code editors). */
float textbox_vscroll_width(const ButtonTextBox *textbox);

enum {
  TEXTBOX_SCROLL_HIT_NONE = 0,
  TEXTBOX_SCROLL_HIT_V = 1,
  TEXTBOX_SCROLL_HIT_H = 2,
  TEXTBOX_SCROLL_HIT_GRIP = 3,
};

/** Window-space track rects matching the always-on code-editor scrollbars. */
void textbox_scroll_win_rects(const ButtonTextBox *textbox,
                              const rctf *win_rect,
                              float aspect,
                              rctf *r_vscroll,
                              rctf *r_hscroll,
                              rctf *r_grip);

int textbox_scroll_hit_test(const ButtonTextBox *textbox,
                            const rctf *win_rect,
                            float aspect,
                            const int xy[2]);

/** Pixel x of a byte offset in a code-editor line (advance width, not glyph ink).
 * \param cell_w: Monospace cell from the last draw; 0 queries the font. */
int textbox_code_x_from_offset(int fontid, StringRef line, int offset, int cell_w = 0);
/** Byte offset of a caret gap nearest to pixel x.
 * Finds the two closest characters (spaces count) and returns the void between them.
 * \param cell_w: Monospace cell from the last draw; 0 queries the font. */
int textbox_code_offset_from_x(int fontid, StringRef line, float x, int cell_w = 0);

struct CodeLineMetrics {
  float em = 16.0f;
  float pad = 4.0f;
  float line_h = 20.0f;
  int descender = 0;
  int gutter = 0;
};
CodeLineMetrics textbox_code_metrics(int fontid, int total_lines);

inline int textbox_code_gutter_for_width(const CodeLineMetrics &m, const int widget_w)
{
  return std::min(m.gutter, std::max(8, widget_w / 4));
}

/** One wrapped row in widget pixels so line spacing tracks node-editor zoom. */
inline float textbox_row_height_px(const rcti &rect, const int visible_lines)
{
  return std::max(1.0f, float(BLI_rcti_size_y(&rect)) / float(std::max(visible_lines, 1)));
}

/** Extra font scale from #TextboxState (Ctrl-+ / wheel). 0 and missing count as 1. */
float textbox_code_font_scale(const ButtonTextBox *textbox);

/** Size the code font from row height and store visible text width (gutter subtracted). */
void textbox_code_sync_font(ButtonTextBox *textbox, int text_w_px, int text_h_px);

/** Moves the text cursor under the `xy` point. */
void textbox_textedit_set_cursor_pos(ButtonTextBox *textbox,
                                     const ARegion *region,
                                     const float2 xy);

/** Returns the index of the line which contains the string offset. */
int textbox_wrapped_line_index_from_char_offset(Span<StringRef> lines, int offset);

/**
 * Moves te cursor in the textbox one line up/down and tries to maintain the horizontal offset in
 * pixels from the current line.
 */
void textbox_jump_line(ButtonTextBox *textbox,
                       eStrCursorJumpDirection direction,
                       const bool select);

/**
 * Wraps input text into lines, if the button is in text editing mode it will insert and wrap
 * ongoing IME composition string if available. This may override active font style.
 */
Vector<StringRef> textbox_wrap_lines(ButtonTextBox *textbox);

Vector<StringRef> textbox_wrap_placeholder(ButtonTextBox *textbox);

float textbox_grip_height();

/* Top/Bottom padding for text in a text-box. */
float textbox_vertical_padding();

TextboxState *textbox_ensure_state(ARegion *region,
                                   StringRefNull idname,
                                   const int initial_visible_lines = 3);

}  // namespace ui
}  // namespace blender
