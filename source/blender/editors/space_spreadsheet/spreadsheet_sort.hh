/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_index_mask.hh"
#include "BLI_resource_scope.hh"
#include "BLI_span.hh"

#include "DNA_space_types.h"

#include "spreadsheet_layout.hh"

namespace blender::ed::spreadsheet {

/**
 * Build a display-order permutation of \a filtered_rows according to the table's sort settings.
 *
 * \return Empty span when no sort is active (caller should use #IndexMask order), or a span of
 * domain indices owned by \a scope when sorting is applied.
 */
Span<int> spreadsheet_sort_rows(const SpreadsheetTable &table,
                                const SpreadsheetLayout &spreadsheet_layout,
                                const IndexMask &filtered_rows,
                                ResourceScope &scope);

}  // namespace blender::ed::spreadsheet
