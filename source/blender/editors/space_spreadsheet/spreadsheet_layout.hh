/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_space_types.h"

#include "BLI_index_mask.hh"
#include "BLI_span.hh"
#include "BLI_string_ref.hh"

#include "spreadsheet_column_values.hh"
#include "spreadsheet_draw.hh"

namespace blender::ed::spreadsheet {

/* Layout information for a single column. */
struct ColumnLayout {
  const ColumnValues *values;
  int width;
};

/* Layout information for the entire spreadsheet. */
struct SpreadsheetLayout {
  Vector<ColumnLayout> columns;
  /**
   * Filtered domain indices (always sorted unique — valid #IndexMask).
   * Used for display when #display_row_indices is empty.
   */
  IndexMask filtered_row_indices;
  /**
   * Optional display order of domain indices after value/index sort.
   * Empty means use #filtered_row_indices order (preserves O(1) full-range path).
   * Non-empty may be unsorted (cannot be an #IndexMask).
   */
  Span<int> display_row_indices;
  int index_column_width = 100;

  /** Digits after the decimal point for float / matrix / array preview. */
  int decimal_precision = 3;

  /** Active sort for header triangle indicators. */
  eSpreadsheetSortDirection sort_direction = SPREADSHEET_SORT_NONE;
  /**
   * Column display name being sorted, or empty when sorting by index / none.
   */
  StringRef sort_column_name;

  int64_t tot_rows() const
  {
    if (!display_row_indices.is_empty()) {
      return display_row_indices.size();
    }
    return filtered_row_indices.size();
  }

  /** Map a visible spreadsheet row to the domain index. */
  int domain_index(const int64_t row_index) const
  {
    if (!display_row_indices.is_empty()) {
      return display_row_indices[row_index];
    }
    return int(filtered_row_indices[row_index]);
  }
};

std::unique_ptr<SpreadsheetDrawer> spreadsheet_drawer_from_layout(
    const SpreadsheetLayout &spreadsheet_layout);

}  // namespace blender::ed::spreadsheet
