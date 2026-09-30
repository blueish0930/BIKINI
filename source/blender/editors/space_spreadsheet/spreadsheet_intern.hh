/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <algorithm>
#include <optional>

#include "BKE_geometry_set.hh"

#include "BKE_node_socket_value.hh"
#include "BLI_bit_vector.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"
#include "DNA_space_types.h"

namespace blender {

struct ARegionType;
struct Depsgraph;
struct Object;
struct SpaceSpreadsheet;
struct ARegion;
struct SpreadsheetColumn;
struct bContext;
namespace nodes {
class Bundle;
}
namespace nodes::eval_log {
class ViewerNodeLog;
}

/** Display digits after the decimal point (0–9). Older files with 0 stored → 3. */
inline int spreadsheet_decimal_precision(const SpaceSpreadsheet &ss)
{
  if (ss.decimal_precision == 0) {
    return 3;
  }
  return std::clamp(int(ss.decimal_precision) - 1, 0, 9);
}

#define SPREADSHEET_EDGE_ACTION_ZONE (UI_UNIT_X * 0.3f)
/** Width of the sort triangle hit zone / icon in column headers. */
#define SPREADSHEET_SORT_TRIANGLE_WIDTH (UI_UNIT_X * 0.85f)

namespace ed::spreadsheet {

class DataSource;

struct ReorderColumnVisualizationData {
  int old_index = 0;
  int new_index = 0;
  int current_offset_x_px = 0;
};

struct SpaceSpreadsheet_Runtime {
 public:
  int visible_rows = 0;
  int tot_rows = 0;
  int tot_columns = 0;
  int top_row_height = 0;
  int left_column_width = 0;
  int row_height = 0;

  /**
   * Per-domain-index selection. Size matches #tot_rows when a table is displayed.
   */
  BitVector<> selected_rows;
  int selected_count = 0;
  /** Domain index of the active (last clicked) row, or -1. */
  int active_row = -1;
  /** Domain index used as the Shift-click range anchor, or -1. */
  int anchor_row = -1;
  /**
   * Display-row -> domain-index. Empty means identity (display row equals domain index).
   */
  Vector<int> visible_domain_indices;

  std::optional<ReorderColumnVisualizationData> reorder_column_visualization_data;

  SpaceSpreadsheet_Runtime() = default;

  SpaceSpreadsheet_Runtime(const SpaceSpreadsheet_Runtime &other)
      : visible_rows(other.visible_rows),
        tot_rows(other.tot_rows),
        tot_columns(other.tot_columns),
        top_row_height(other.top_row_height),
        left_column_width(other.left_column_width),
        row_height(other.row_height),
        selected_rows(other.selected_rows),
        selected_count(other.selected_count),
        active_row(other.active_row),
        anchor_row(other.anchor_row),
        visible_domain_indices(other.visible_domain_indices)
  {
  }
};

void spreadsheet_selection_ensure_size(SpaceSpreadsheet_Runtime &runtime, int tot_rows);
void spreadsheet_selection_update_count(SpaceSpreadsheet_Runtime &runtime);
int spreadsheet_display_row_to_domain(const SpaceSpreadsheet_Runtime &runtime, int display_row);
std::optional<int> spreadsheet_find_hovered_display_row(const SpaceSpreadsheet &sspreadsheet,
                                                        const ARegion &region,
                                                        const int2 &cursor_re);

/**
 * True when the column should be shown given the attribute filter.
 * Include tokens list names to keep; `^` / `!` tokens subtract from that set.
 */
bool spreadsheet_attribute_is_visible(const SpaceSpreadsheet &sspreadsheet,
                                      StringRef id_name,
                                      StringRef display_name);

void spreadsheet_operatortypes();
Object *spreadsheet_get_object_eval(const SpaceSpreadsheet *sspreadsheet,
                                    const Depsgraph *depsgraph);

const nodes::eval_log::ViewerNodeLog *viewer_node_log_lookup(const SpaceSpreadsheet &sspreadsheet);

bke::SocketValueVariant root_display_data_get(const SpaceSpreadsheet *sspreadsheet,
                                              Object *object_eval);
std::optional<bke::GeometrySet> root_geometry_set_get(const SpaceSpreadsheet *sspreadsheet,
                                                      Object *object_eval);

void spreadsheet_data_set_region_panels_register(ARegionType &region_type);

/** Find the column edge that the cursor is hovering in the header row. */
SpreadsheetColumn *find_hovered_column_header_edge(SpaceSpreadsheet &sspreadsheet,
                                                   ARegion &region,
                                                   const int2 &cursor_re);

/** Find the column that the cursor is hovering in the header row. */
SpreadsheetColumn *find_hovered_column_header(SpaceSpreadsheet &sspreadsheet,
                                              ARegion &region,
                                              const int2 &cursor_re);

/** Find the column edge that the cursor is hovering. */
SpreadsheetColumn *find_hovered_column_edge(SpaceSpreadsheet &sspreadsheet,
                                            ARegion &region,
                                            const int2 &cursor_re);

/** Find the column that the cursor is hovering. */
SpreadsheetColumn *find_hovered_column(SpaceSpreadsheet &sspreadsheet,
                                       ARegion &region,
                                       const int2 &cursor_re);

/** True when the cursor is over the index-column header corner. */
bool is_hovering_index_header_corner(const SpaceSpreadsheet &sspreadsheet,
                                     const ARegion &region,
                                     const int2 &cursor_re);

/**
 * Get the data that is currently displayed in the spreadsheet.
 */
std::unique_ptr<DataSource> get_data_source(const bContext &C);

/**
 * Get the ID of the table that should be displayed. This is used to look up the table from
 * #SpaceSpreadsheet::tables.
 */
const SpreadsheetTableID *get_active_table_id(const SpaceSpreadsheet &sspreadsheet);

}  // namespace ed::spreadsheet
}  // namespace blender
