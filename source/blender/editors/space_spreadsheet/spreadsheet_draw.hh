/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_bit_span.hh"
#include "DNA_space_types.h"

namespace blender {

struct ARegion;
struct bContext;
namespace ui {
struct Block;
}

namespace ed::spreadsheet {

struct CellDrawParams {
  ui::Block *block;
  int xmin, ymin;
  int width, height;
};

class SpreadsheetDrawer {
 public:
  int left_column_width;
  int top_row_height;
  int row_height;
  int tot_rows = 0;
  int tot_columns = 0;

  /** Sort indicator state for the index column header triangle. */
  eSpreadsheetSortDirection index_sort_direction = SPREADSHEET_SORT_NONE;
  bool index_sort_active = false;

  /** Domain-index bits for selected rows. Empty means nothing selected. */
  BoundedBitSpan selected_domain_rows;
  /** Domain index of the active row, or -1. */
  int active_domain_index = -1;

  SpreadsheetDrawer();
  virtual ~SpreadsheetDrawer();

  virtual void draw_top_row_cell(int column_index, const CellDrawParams &params) const;

  virtual void draw_left_column_cell(int row_index, const CellDrawParams &params) const;

  /** Map a visible spreadsheet row to the geometry domain index. */
  virtual int domain_index(int row_index) const;

  virtual void draw_content_cell(int row_index,
                                 int column_index,
                                 const CellDrawParams &params) const;

  virtual int column_width(int column_index) const;
};

void draw_spreadsheet_in_region(const bContext *C,
                                ARegion *region,
                                const SpreadsheetDrawer &drawer);

}  // namespace ed::spreadsheet
}  // namespace blender
