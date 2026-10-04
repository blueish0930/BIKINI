/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fmt/format.h>

#include "DNA_array_utils.hh"
#include "DNA_meshdata_types.h"
#include "DNA_space_types.h"
#include "DNA_userdef_types.h"

#include "ED_screen.hh"
#include "ED_select_utils.hh"
#include "ED_spreadsheet.hh"

#include "MEM_guardedalloc.h"

#include "BLI_color.hh"
#include "BLI_cpp_type.hh"
#include "BLI_index_range.hh"
#include "BLI_listbase.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_rect.hh"
#include "BLI_string.hh"
#include "BLI_string_utf8.hh"
#include "BLI_vector.hh"

#include "BKE_context.hh"
#include "BKE_wrangle_array.hh"

#include "BLF_api.hh"

#include "BLT_translation.hh"

#include "GPU_immediate.hh"
#include "GPU_state.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "UI_interface.hh"
#include "UI_interface_c.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"
#include "UI_view2d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "spreadsheet_column.hh"
#include "spreadsheet_data_source.hh"
#include "spreadsheet_intern.hh"
#include "spreadsheet_row_filter.hh"

namespace blender::ed::spreadsheet {

void spreadsheet_selection_update_count(SpaceSpreadsheet_Runtime &runtime)
{
  int count = 0;
  for (const int64_t i : runtime.selected_rows.index_range()) {
    if (runtime.selected_rows[i]) {
      count++;
    }
  }
  runtime.selected_count = count;
}

void spreadsheet_selection_ensure_size(SpaceSpreadsheet_Runtime &runtime, const int tot_rows)
{
  if (runtime.selected_rows.size() == tot_rows) {
    return;
  }
  runtime.selected_rows.resize(tot_rows, false);
  runtime.selected_rows.fill(false);
  runtime.active_row = -1;
  runtime.anchor_row = -1;
  runtime.selected_count = 0;
}

int spreadsheet_display_row_to_domain(const SpaceSpreadsheet_Runtime &runtime,
                                      const int display_row)
{
  if (display_row < 0 || display_row >= runtime.visible_rows) {
    return -1;
  }
  if (runtime.visible_domain_indices.is_empty()) {
    return display_row;
  }
  if (display_row >= runtime.visible_domain_indices.size()) {
    return -1;
  }
  return runtime.visible_domain_indices[display_row];
}

std::optional<int> spreadsheet_find_hovered_display_row(const SpaceSpreadsheet &sspreadsheet,
                                                        const ARegion &region,
                                                        const int2 &cursor_re)
{
  const SpaceSpreadsheet_Runtime &runtime = *sspreadsheet.runtime;
  if (runtime.row_height <= 0 || runtime.top_row_height <= 0 || runtime.visible_rows <= 0) {
    return std::nullopt;
  }
  if (cursor_re.x < 0 || cursor_re.x >= region.winx) {
    return std::nullopt;
  }
  const int content_top = region.winy - runtime.top_row_height;
  if (cursor_re.y >= content_top || cursor_re.y < 0) {
    return std::nullopt;
  }
  const int scroll_offset_y = region.v2d.cur.ymax;
  const int y_from_content_top = content_top - cursor_re.y - scroll_offset_y;
  if (y_from_content_top < 0) {
    return std::nullopt;
  }
  const int row = y_from_content_top / runtime.row_height;
  if (row < 0 || row >= runtime.visible_rows) {
    return std::nullopt;
  }
  return row;
}

static int domain_to_display_row(const SpaceSpreadsheet_Runtime &runtime, const int domain)
{
  if (domain < 0) {
    return -1;
  }
  if (runtime.visible_domain_indices.is_empty()) {
    return (domain < runtime.visible_rows) ? domain : -1;
  }
  for (const int i : runtime.visible_domain_indices.index_range()) {
    if (runtime.visible_domain_indices[i] == domain) {
      return i;
    }
  }
  return -1;
}

static wmOperatorStatus row_filter_add_exec(bContext *C, wmOperator * /*op*/)
{
  SpaceSpreadsheet *sspreadsheet = CTX_wm_space_spreadsheet(C);

  SpreadsheetRowFilter *row_filter = spreadsheet_row_filter_new();
  BLI_addtail(&sspreadsheet->row_filters, row_filter);

  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_SPREADSHEET, sspreadsheet);

  return OPERATOR_FINISHED;
}

static void SPREADSHEET_OT_add_row_filter_rule(wmOperatorType *ot)
{
  ot->name = "Add Row Filter";
  ot->description = "Add a filter to remove rows from the displayed data";
  ot->idname = "SPREADSHEET_OT_add_row_filter_rule";

  ot->exec = row_filter_add_exec;
  ot->poll = ED_operator_spreadsheet_active;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static wmOperatorStatus row_filter_remove_exec(bContext *C, wmOperator *op)
{
  SpaceSpreadsheet *sspreadsheet = CTX_wm_space_spreadsheet(C);

  SpreadsheetRowFilter *row_filter = static_cast<SpreadsheetRowFilter *>(
      BLI_findlink(&sspreadsheet->row_filters, RNA_int_get(op->ptr, "index")));
  if (row_filter == nullptr) {
    return OPERATOR_CANCELLED;
  }

  BLI_remlink(&sspreadsheet->row_filters, row_filter);
  spreadsheet_row_filter_free(row_filter);

  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_SPREADSHEET, sspreadsheet);

  return OPERATOR_FINISHED;
}

static void SPREADSHEET_OT_remove_row_filter_rule(wmOperatorType *ot)
{
  ot->name = "Remove Row Filter";
  ot->description = "Remove a row filter from the rules";
  ot->idname = "SPREADSHEET_OT_remove_row_filter_rule";

  ot->exec = row_filter_remove_exec;
  ot->poll = ED_operator_spreadsheet_active;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  RNA_def_int(ot->srna, "index", 0, 0, INT_MAX, "Index", "", 0, INT_MAX);
}

static wmOperatorStatus select_component_domain_invoke(bContext *C,
                                                       wmOperator *op,
                                                       const wmEvent * /*event*/)
{
  const auto component_type = bke::GeometryComponent::Type(RNA_int_get(op->ptr, "component_type"));
  bke::AttrDomain domain = bke::AttrDomain(RNA_int_get(op->ptr, "attribute_domain_type"));

  SpaceSpreadsheet *sspreadsheet = CTX_wm_space_spreadsheet(C);
  sspreadsheet->geometry_id.geometry_component_type = uint8_t(component_type);
  sspreadsheet->geometry_id.attribute_domain = uint8_t(domain);

  /* Refresh header and main region. */
  WM_main_add_notifier(NC_SPACE | ND_SPACE_SPREADSHEET, nullptr);

  return OPERATOR_FINISHED;
}

static void SPREADSHEET_OT_change_spreadsheet_data_source(wmOperatorType *ot)
{
  ot->name = "Change Visible Data Source";
  ot->description = "Change visible data source in the spreadsheet";
  ot->idname = "SPREADSHEET_OT_change_spreadsheet_data_source";

  ot->invoke = select_component_domain_invoke;
  ot->poll = ED_operator_spreadsheet_active;

  RNA_def_int(ot->srna, "component_type", 0, 0, INT16_MAX, "Component Type", "", 0, INT16_MAX);
  RNA_def_int(ot->srna,
              "attribute_domain_type",
              0,
              0,
              INT16_MAX,
              "Attribute Domain Type",
              "",
              0,
              INT16_MAX);

  ot->flag = OPTYPE_INTERNAL;
}

struct ResizeColumnData {
  SpreadsheetColumn *column = nullptr;
  int2 initial_cursor_re;
  int initial_width_px;
};

static wmOperatorStatus resize_column_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  ARegion &region = *CTX_wm_region(C);
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(C);

  SpreadsheetTable &table = *get_active_table(sspreadsheet);
  ResizeColumnData &data = *static_cast<ResizeColumnData *>(op->customdata);

  auto cancel = [&]() {
    data.column->width = data.initial_width_px / SPREADSHEET_WIDTH_UNIT;
    MEM_delete(&data);
    ED_region_tag_redraw(&region);
    return OPERATOR_CANCELLED;
  };
  auto finish = [&]() {
    table.flag |= SPREADSHEET_TABLE_FLAG_MANUALLY_EDITED;
    MEM_delete(&data);
    ED_region_tag_redraw(&region);
    return OPERATOR_FINISHED;
  };

  const int2 cursor_re{event->mval[0], event->mval[1]};

  switch (event->type) {
    case RIGHTMOUSE:
    case EVT_ESCKEY: {
      return cancel();
    }
    case LEFTMOUSE: {
      return finish();
    }
    case MOUSEMOVE: {
      const int offset = cursor_re.x - data.initial_cursor_re.x;
      const float new_width_px = std::max<float>(SPREADSHEET_WIDTH_UNIT,
                                                 data.initial_width_px + offset);
      data.column->width = new_width_px / SPREADSHEET_WIDTH_UNIT;
      ED_region_tag_redraw(&region);
      return OPERATOR_RUNNING_MODAL;
    }
    default: {
      return OPERATOR_RUNNING_MODAL;
    }
  }
}

static bool is_hovering_header_row(const SpaceSpreadsheet &sspreadsheet,
                                   const ARegion &region,
                                   const int2 &cursor_re)
{
  const int region_height = BLI_rcti_size_y(&region.winrct);
  return cursor_re.y >= region_height - sspreadsheet.runtime->top_row_height &&
         cursor_re.y <= region_height;
}

SpreadsheetColumn *find_hovered_column_edge(SpaceSpreadsheet &sspreadsheet,
                                            ARegion &region,
                                            const int2 &cursor_re)
{
  SpreadsheetTable *table = get_active_table(sspreadsheet);
  if (!table) {
    return nullptr;
  }
  const float cursor_x_view = ui::view2d_region_to_view_x(&region.v2d, cursor_re.x);
  for (SpreadsheetColumn *column : Span{table->columns, table->num_columns}) {
    if (column->flag & SPREADSHEET_COLUMN_FLAG_UNAVAILABLE) {
      continue;
    }
    if (std::abs(cursor_x_view - column->runtime->right_x) < SPREADSHEET_EDGE_ACTION_ZONE) {
      return column;
    }
  }
  return nullptr;
}

SpreadsheetColumn *find_hovered_column(SpaceSpreadsheet &sspreadsheet,
                                       ARegion &region,
                                       const int2 &cursor_re)
{
  SpreadsheetTable *table = get_active_table(sspreadsheet);
  if (!table) {
    return nullptr;
  }
  const float cursor_x_view = ui::view2d_region_to_view_x(&region.v2d, cursor_re.x);
  for (SpreadsheetColumn *column : Span{table->columns, table->num_columns}) {
    if (column->flag & SPREADSHEET_COLUMN_FLAG_UNAVAILABLE) {
      continue;
    }
    if (cursor_x_view > column->runtime->left_x && cursor_x_view <= column->runtime->right_x) {
      return column;
    }
  }
  return nullptr;
}

SpreadsheetColumn *find_hovered_column_header_edge(SpaceSpreadsheet &sspreadsheet,
                                                   ARegion &region,
                                                   const int2 &cursor_re)
{
  if (!is_hovering_header_row(sspreadsheet, region, cursor_re)) {
    return nullptr;
  }
  return find_hovered_column_edge(sspreadsheet, region, cursor_re);
}

SpreadsheetColumn *find_hovered_column_header(SpaceSpreadsheet &sspreadsheet,
                                              ARegion &region,
                                              const int2 &cursor_re)
{
  if (!is_hovering_header_row(sspreadsheet, region, cursor_re)) {
    return nullptr;
  }
  return find_hovered_column(sspreadsheet, region, cursor_re);
}

bool is_hovering_index_header_corner(const SpaceSpreadsheet &sspreadsheet,
                                     const ARegion &region,
                                     const int2 &cursor_re)
{
  if (!is_hovering_header_row(sspreadsheet, region, cursor_re)) {
    return false;
  }
  return cursor_re.x >= 0 && cursor_re.x < sspreadsheet.runtime->left_column_width;
}

static void cycle_table_sort(SpreadsheetTable &table, const char *column_name_or_null_for_index)
{
  const bool is_index = column_name_or_null_for_index == nullptr ||
                        column_name_or_null_for_index[0] == '\0';
  const bool same_target = is_index ? (table.sort_column_name[0] == '\0') :
                                      STREQ(table.sort_column_name, column_name_or_null_for_index);

  if (!same_target || table.sort_direction == SPREADSHEET_SORT_NONE) {
    table.sort_direction = SPREADSHEET_SORT_ASC;
    if (is_index) {
      table.sort_column_name[0] = '\0';
    }
    else {
      STRNCPY_UTF8(table.sort_column_name, column_name_or_null_for_index);
    }
  }
  else if (table.sort_direction == SPREADSHEET_SORT_ASC) {
    table.sort_direction = SPREADSHEET_SORT_DESC;
  }
  else {
    table.sort_direction = SPREADSHEET_SORT_NONE;
    table.sort_column_name[0] = '\0';
  }
  table.flag |= SPREADSHEET_TABLE_FLAG_MANUALLY_EDITED;
}

static wmOperatorStatus sort_column_invoke(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(C);
  ARegion &region = *CTX_wm_region(C);
  SpreadsheetTable *table = get_active_table(sspreadsheet);
  if (!table) {
    return OPERATOR_CANCELLED;
  }

  const int2 cursor_re{event->mval[0], event->mval[1]};

  /* Prefer edge resize over sort when near a column edge. */
  if (find_hovered_column_header_edge(sspreadsheet, region, cursor_re)) {
    return OPERATOR_PASS_THROUGH;
  }

  if (is_hovering_index_header_corner(sspreadsheet, region, cursor_re)) {
    cycle_table_sort(*table, nullptr);
    WM_event_add_notifier(C, NC_SPACE | ND_SPACE_SPREADSHEET, &sspreadsheet);
    ED_region_tag_redraw(&region);
    return OPERATOR_FINISHED;
  }

  /* Click anywhere on the header cell (name + triangle) cycles sort. */
  SpreadsheetColumn *column = find_hovered_column_header(sspreadsheet, region, cursor_re);
  if (!column) {
    return OPERATOR_PASS_THROUGH;
  }

  const char *name = column->display_name ? column->display_name :
                                            (column->id ? column->id->name : "");
  if (name == nullptr || name[0] == '\0') {
    return OPERATOR_CANCELLED;
  }

  cycle_table_sort(*table, name);
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_SPREADSHEET, &sspreadsheet);
  ED_region_tag_redraw(&region);
  return OPERATOR_FINISHED;
}

static void SPREADSHEET_OT_sort_column(wmOperatorType *ot)
{
  ot->name = "Sort Column";
  ot->description =
      "Click a column header to cycle sort order "
      "(ascending, descending, original order)";
  ot->idname = "SPREADSHEET_OT_sort_column";

  ot->invoke = sort_column_invoke;
  ot->poll = ED_operator_spreadsheet_active;
  ot->flag = OPTYPE_INTERNAL;
}

static wmOperatorStatus resize_column_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  ARegion &region = *CTX_wm_region(C);
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(C);

  const int2 cursor_re{event->mval[0], event->mval[1]};
  SpreadsheetColumn *column_to_resize = find_hovered_column_header_edge(
      sspreadsheet, region, cursor_re);
  if (!column_to_resize) {
    return OPERATOR_PASS_THROUGH;
  }

  ResizeColumnData *data = MEM_new<ResizeColumnData>(__func__);
  data->column = column_to_resize;
  data->initial_cursor_re = cursor_re;
  data->initial_width_px = column_to_resize->width * SPREADSHEET_WIDTH_UNIT;
  op->customdata = data;

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static void SPREADSHEET_OT_resize_column(wmOperatorType *ot)
{
  ot->name = "Resize Column";
  ot->description = "Resize a spreadsheet column";
  ot->idname = "SPREADSHEET_OT_resize_column";

  ot->invoke = resize_column_invoke;
  ot->modal = resize_column_modal;
  ot->poll = ED_operator_spreadsheet_active;
  ot->flag = OPTYPE_INTERNAL;
}

static wmOperatorStatus fit_column_invoke(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(C);
  ARegion &region = *CTX_wm_region(C);

  std::unique_ptr<DataSource> data_source = get_data_source(*C);
  if (!data_source) {
    return OPERATOR_CANCELLED;
  }
  const int2 cursor_re{event->mval[0], event->mval[1]};
  SpreadsheetColumn *column = find_hovered_column_header_edge(sspreadsheet, region, cursor_re);
  if (!column) {
    return OPERATOR_PASS_THROUGH;
  }

  std::unique_ptr<ColumnValues> values = data_source->get_column_values(*column->id);
  if (!values) {
    return OPERATOR_CANCELLED;
  }

  SpreadsheetTable &table = *get_active_table(sspreadsheet);
  table.flag |= SPREADSHEET_TABLE_FLAG_MANUALLY_EDITED;

  const float width_px = values->fit_column_width_px();
  column->width = width_px / SPREADSHEET_WIDTH_UNIT;

  ED_region_tag_redraw(&region);
  return OPERATOR_FINISHED;
}

static void SPREADSHEET_OT_fit_column(wmOperatorType *ot)
{
  ot->name = "Fit Column";
  ot->description = "Resize a spreadsheet column to the width of the data";
  ot->idname = "SPREADSHEET_OT_fit_column";

  ot->invoke = fit_column_invoke;
  ot->poll = ED_operator_spreadsheet_active;
  ot->flag = OPTYPE_INTERNAL;
}

struct ReorderColumnData {
  SpreadsheetColumn *column = nullptr;
  int initial_cursor_x_view = 0;
  ui::View2DEdgePanData pan_data{};
};

static std::optional<int> find_first_available_column_index(const SpreadsheetTable &table)
{
  for (int i = 0; i < table.num_columns; i++) {
    if (!(table.columns[i]->flag & SPREADSHEET_COLUMN_FLAG_UNAVAILABLE)) {
      return i;
    }
  }
  return std::nullopt;
}

static std::optional<int> find_last_available_column_index(const SpreadsheetTable &table)
{
  for (int i = table.num_columns - 1; i >= 0; i--) {
    if (!(table.columns[i]->flag & SPREADSHEET_COLUMN_FLAG_UNAVAILABLE)) {
      return i;
    }
  }
  return std::nullopt;
}

static wmOperatorStatus reorder_columns_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(C);
  ARegion &region = *CTX_wm_region(C);

  const int2 cursor_re{event->mval[0], event->mval[1]};

  if (find_hovered_column_edge(sspreadsheet, region, cursor_re)) {
    return OPERATOR_PASS_THROUGH;
  }
  /* Index header is not a reorderable column. */
  if (is_hovering_index_header_corner(sspreadsheet, region, cursor_re)) {
    return OPERATOR_PASS_THROUGH;
  }

  SpreadsheetColumn *column_to_move = find_hovered_column_header(sspreadsheet, region, cursor_re);
  if (!column_to_move) {
    return OPERATOR_PASS_THROUGH;
  }

  WM_cursor_set(CTX_wm_window(C), WM_CURSOR_HAND_CLOSED);

  SpreadsheetTable *table = get_active_table(sspreadsheet);
  const int old_index = Span{table->columns, table->num_columns}.first_index(column_to_move);

  ReorderColumnData *data = MEM_new<ReorderColumnData>(__func__);
  data->column = column_to_move;
  data->initial_cursor_x_view = ui::view2d_region_to_view_x(&region.v2d, cursor_re.x);
  op->customdata = data;

  ReorderColumnVisualizationData &visualization_data =
      sspreadsheet.runtime->reorder_column_visualization_data.emplace();
  visualization_data.old_index = old_index;
  visualization_data.new_index = old_index;
  visualization_data.current_offset_x_px = 0;

  view2d_edge_pan_init(C, &data->pan_data, 0, 0, 1, 26, 0.5f, 0.0f);
  /* Limit to horizontal panning. */
  data->pan_data.limit.xmin = region.v2d.tot.xmin;
  data->pan_data.limit.xmax = region.v2d.tot.xmax;
  data->pan_data.limit.ymin = region.v2d.cur.ymin;
  data->pan_data.limit.ymax = region.v2d.cur.ymax;

  WM_event_add_modal_handler(C, op);
  return OPERATOR_RUNNING_MODAL;
}

static wmOperatorStatus reorder_columns_modal(bContext *C, wmOperator *op, const wmEvent *event)
{
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(C);
  ARegion &region = *CTX_wm_region(C);

  const int2 cursor_re{event->mval[0], event->mval[1]};
  ReorderColumnData &data = *static_cast<ReorderColumnData *>(op->customdata);

  SpreadsheetTable &table = *get_active_table(sspreadsheet);
  Span<SpreadsheetColumn *> columns(table.columns, table.num_columns);

  const int old_index = columns.first_index(data.column);
  int new_index = 0;

  SpreadsheetColumn *hovered_column = find_hovered_column(sspreadsheet, region, cursor_re);
  if (hovered_column) {
    new_index = columns.first_index(hovered_column);
  }
  else {
    if (cursor_re.x > sspreadsheet.runtime->left_column_width) {
      new_index = *find_last_available_column_index(table);
    }
    else {
      new_index = *find_first_available_column_index(table);
    }
  }

  auto cleanup_on_finish = [&]() {
    sspreadsheet.runtime->reorder_column_visualization_data.reset();
    MEM_delete(&data);
    ED_region_tag_redraw(&region);
    WM_cursor_set(CTX_wm_window(C), WM_CURSOR_DEFAULT);
  };

  switch (event->type) {
    case RIGHTMOUSE:
    case EVT_ESCKEY: {
      view2d_edge_pan_cancel(C, &data.pan_data);
      cleanup_on_finish();
      return OPERATOR_CANCELLED;
    }
    case LEFTMOUSE: {
      if (old_index != new_index) {
        dna::array::move_index(table.columns, table.num_columns, old_index, new_index);
      }
      table.flag |= SPREADSHEET_TABLE_FLAG_MANUALLY_EDITED;
      cleanup_on_finish();
      return OPERATOR_FINISHED;
    }
    case MOUSEMOVE: {
      view2d_edge_pan_apply(C, &data.pan_data, event->xy);

      ReorderColumnVisualizationData &visualization_data =
          *sspreadsheet.runtime->reorder_column_visualization_data;
      visualization_data.new_index = new_index;
      visualization_data.current_offset_x_px = ui::view2d_region_to_view_x(&region.v2d,
                                                                           cursor_re.x) -
                                               data.initial_cursor_x_view;
      ED_region_tag_redraw(&region);
      return OPERATOR_RUNNING_MODAL;
    }
    case WHEELLEFTMOUSE:
    case WHEELRIGHTMOUSE: {
      if (BLI_rcti_isect_pt_v(&region.winrct, event->xy)) {
        /* Support scrolling left and right. */
        return OPERATOR_PASS_THROUGH;
      }
      return OPERATOR_RUNNING_MODAL;
    }
    default: {
      return OPERATOR_RUNNING_MODAL;
    }
  }
}

static void SPREADSHEET_OT_reorder_columns(wmOperatorType *ot)
{
  ot->name = "Reorder Columns";
  ot->description = "Change the order of columns";
  ot->idname = "SPREADSHEET_OT_reorder_columns";

  ot->poll = ED_operator_spreadsheet_active;
  ot->invoke = reorder_columns_invoke;
  ot->modal = reorder_columns_modal;
  ot->flag = OPTYPE_INTERNAL;
}

static wmOperatorStatus select_invoke(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(C);
  ARegion &region = *CTX_wm_region(C);
  SpaceSpreadsheet_Runtime &runtime = *sspreadsheet.runtime;

  const int2 cursor_re{event->mval[0], event->mval[1]};
  if (ui::view2d_mouse_in_scrollers(&region, &region.v2d, event->xy)) {
    return OPERATOR_PASS_THROUGH;
  }
  if (is_hovering_header_row(sspreadsheet, region, cursor_re)) {
    return OPERATOR_PASS_THROUGH;
  }
  if (find_hovered_column_header_edge(sspreadsheet, region, cursor_re)) {
    return OPERATOR_PASS_THROUGH;
  }

  /* Read modifiers from the live event. CLICK events (and RNA keymap properties) can drop
   * Shift/Ctrl on Windows; PRESS + event->modifier is reliable. */
  const bool shift = (event->modifier & KM_SHIFT) != 0;
  const bool ctrl = (event->modifier & (KM_CTRL | KM_OSKEY)) != 0;

  spreadsheet_selection_ensure_size(runtime, runtime.tot_rows);

  const std::optional<int> hovered = spreadsheet_find_hovered_display_row(
      sspreadsheet, region, cursor_re);
  if (!hovered.has_value()) {
    if (!shift && !ctrl) {
      runtime.selected_rows.fill(false);
      runtime.active_row = -1;
      runtime.anchor_row = -1;
      spreadsheet_selection_update_count(runtime);
      ED_region_tag_redraw(&region);
      WM_event_add_notifier(C, NC_SPACE | ND_SPACE_SPREADSHEET, &sspreadsheet);
    }
    return OPERATOR_FINISHED;
  }

  const int display_row = *hovered;
  const int domain = spreadsheet_display_row_to_domain(runtime, display_row);
  if (domain < 0 || domain >= runtime.selected_rows.size()) {
    return OPERATOR_CANCELLED;
  }

  if (shift) {
    int anchor_display = domain_to_display_row(runtime, runtime.anchor_row);
    if (anchor_display < 0) {
      anchor_display = display_row;
    }
    const int start = std::min(anchor_display, display_row);
    const int end = std::max(anchor_display, display_row);
    if (!ctrl) {
      runtime.selected_rows.fill(false);
    }
    for (int i = start; i <= end; i++) {
      const int range_domain = spreadsheet_display_row_to_domain(runtime, i);
      if (range_domain >= 0 && range_domain < runtime.selected_rows.size()) {
        runtime.selected_rows[range_domain].set(true);
      }
    }
  }
  else if (ctrl) {
    runtime.selected_rows[domain].set(!bool(runtime.selected_rows[domain]));
    runtime.anchor_row = domain;
  }
  else {
    runtime.selected_rows.fill(false);
    runtime.selected_rows[domain].set(true);
    runtime.anchor_row = domain;
  }

  runtime.active_row = domain;
  spreadsheet_selection_update_count(runtime);
  ED_region_tag_redraw(&region);
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_SPREADSHEET, &sspreadsheet);
  return OPERATOR_FINISHED;
}

static void SPREADSHEET_OT_select(wmOperatorType *ot)
{
  ot->name = "Select Spreadsheet Row";
  ot->description = "Select spreadsheet rows (Shift for range, Ctrl to toggle)";
  ot->idname = "SPREADSHEET_OT_select";

  ot->invoke = select_invoke;
  ot->poll = ED_operator_spreadsheet_active;
  ot->flag = OPTYPE_INTERNAL;

  RNA_def_boolean(ot->srna, "extend", false, "Extend", "Extend the existing selection");
  RNA_def_boolean(ot->srna,
                  "extend_range",
                  false,
                  "Extend Range",
                  "Select a range from the last selected row");
}

static wmOperatorStatus select_all_exec(bContext *C, wmOperator *op)
{
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(C);
  ARegion &region = *CTX_wm_region(C);
  SpaceSpreadsheet_Runtime &runtime = *sspreadsheet.runtime;

  spreadsheet_selection_ensure_size(runtime, runtime.tot_rows);
  if (runtime.selected_rows.is_empty() || runtime.visible_rows <= 0) {
    return OPERATOR_CANCELLED;
  }

  int action = RNA_enum_get(op->ptr, "action");
  if (action == SEL_TOGGLE) {
    bool any_unselected = false;
    for (const int i : IndexRange(runtime.visible_rows)) {
      const int domain = spreadsheet_display_row_to_domain(runtime, i);
      if (domain >= 0 && domain < runtime.selected_rows.size() && !runtime.selected_rows[domain]) {
        any_unselected = true;
        break;
      }
    }
    action = any_unselected ? SEL_SELECT : SEL_DESELECT;
  }

  switch (action) {
    case SEL_SELECT: {
      for (const int i : IndexRange(runtime.visible_rows)) {
        const int domain = spreadsheet_display_row_to_domain(runtime, i);
        if (domain >= 0 && domain < runtime.selected_rows.size()) {
          runtime.selected_rows[domain].set(true);
        }
      }
      break;
    }
    case SEL_DESELECT: {
      runtime.selected_rows.fill(false);
      runtime.active_row = -1;
      runtime.anchor_row = -1;
      break;
    }
    case SEL_INVERT: {
      for (const int i : IndexRange(runtime.visible_rows)) {
        const int domain = spreadsheet_display_row_to_domain(runtime, i);
        if (domain >= 0 && domain < runtime.selected_rows.size()) {
          runtime.selected_rows[domain].set(!bool(runtime.selected_rows[domain]));
        }
      }
      break;
    }
    default:
      break;
  }

  spreadsheet_selection_update_count(runtime);
  ED_region_tag_redraw(&region);
  WM_event_add_notifier(C, NC_SPACE | ND_SPACE_SPREADSHEET, &sspreadsheet);
  return OPERATOR_FINISHED;
}

static void SPREADSHEET_OT_select_all(wmOperatorType *ot)
{
  ot->name = "Select All Spreadsheet Rows";
  ot->description = "Select or deselect all currently visible spreadsheet rows";
  ot->idname = "SPREADSHEET_OT_select_all";

  ot->exec = select_all_exec;
  ot->poll = ED_operator_spreadsheet_active;
  ot->flag = OPTYPE_INTERNAL;

  WM_operator_properties_select_all(ot);
}

struct InspectPopupArg {
  std::string title;
  Vector<float4x4> matrices;
  Vector<std::string> items;
  bool truncated = false;
  int precision = 3;
  bool pinned = false;
  std::string column_id;
  int row = -1;
};

static void inspect_fmt_cell(
    char *buf, const int buf_n, const float value, const int precision, const bool comma)
{
  const int prec = std::clamp(precision, 0, 9);
  if (comma) {
    BLI_snprintf(buf, buf_n, "%.*f,", prec, double(value));
  }
  else {
    BLI_snprintf(buf, buf_n, "%.*f", prec, double(value));
  }
}

static std::string inspect_fmt_float(const float value, const int precision)
{
  char buf[40];
  inspect_fmt_cell(buf, sizeof(buf), value, precision, false);
  return buf;
}

struct InspectMatrixGeom {
  float col_w[4] = {0, 0, 0, 0};
  float row_h = 0.0f;
  float em = 0.0f;
  float thick = 0.0f;
  float serif = 0.0f;
  float pad = 0.0f;
  float gap = 0.0f;
  float width = 0.0f;
  float height = 0.0f;
  float comma_w = 0.0f;
};

static int inspect_fontid()
{
  return blf_mono_font >= 0 ? blf_mono_font : BLF_default();
}

static float inspect_font_px()
{
  const uiStyle *style = ui::style_get_dpi();
  return std::max(1.0f, style->widget.points * UI_SCALE_FAC);
}

static InspectMatrixGeom inspect_measure_matrix(const float4x4 &matrix,
                                                const int fontid,
                                                const int precision)
{
  InspectMatrixGeom g;
  const int desc = BLF_descender(fontid);
  g.em = std::max(1.0f, float(BLF_ascender(fontid) - desc));
  g.row_h = std::max(g.em * 1.16f, g.em + 1.0f * UI_SCALE_FAC);
  g.thick = std::max(2.0f * UI_SCALE_FAC, 0.11f * g.em);
  g.serif = std::max(4.0f * UI_SCALE_FAC, 0.40f * g.em);
  g.pad = std::max(3.0f * UI_SCALE_FAC, 0.22f * g.em);
  g.gap = std::max(5.0f * UI_SCALE_FAC, 0.38f * g.em);
  const float min_w = BLF_width(fontid, "0", 1);
  for (int col = 0; col < 4; col++) {
    float cw = min_w;
    for (int row = 0; row < 4; row++) {
      char cell[40];
      inspect_fmt_cell(cell, sizeof(cell), matrix[col][row], precision, col < 3);
      cw = std::max(cw, BLF_width(fontid, cell, strlen(cell)));
    }
    g.col_w[col] = cw;
  }
  float nums_w = g.gap * 3.0f;
  for (int c = 0; c < 4; c++) {
    nums_w += g.col_w[c];
  }
  g.width = g.thick + g.serif + g.pad + nums_w + g.pad + g.serif + g.thick;
  g.height = g.pad + g.row_h * 4.0f + g.pad;
  g.comma_w = BLF_width(fontid, ",", 1) + g.gap * 0.55f;
  return g;
}

static void inspect_fill_rect(
    const uint pos, const float xmin, const float ymin, const float xmax, const float ymax)
{
  immRectf(pos, xmin, ymin, xmax, ymax);
}

static void inspect_draw_brackets(const uint pos, const rctf &box, const InspectMatrixGeom &g)
{
  /* Tall [ ] that fully enclose the 4×4: vertical bar + inward serifs. */
  const float x0 = box.xmin;
  const float x1 = box.xmin + g.thick + g.serif;
  const float y0 = box.ymin;
  const float y1 = box.ymax;
  inspect_fill_rect(pos, x0, y0, x0 + g.thick, y1);
  inspect_fill_rect(pos, x0, y1 - g.thick, x1, y1);
  inspect_fill_rect(pos, x0, y0, x1, y0 + g.thick);

  const float rx1 = box.xmax;
  const float rx0 = box.xmax - g.thick - g.serif;
  inspect_fill_rect(pos, rx1 - g.thick, y0, rx1, y1);
  inspect_fill_rect(pos, rx0, y1 - g.thick, rx1, y1);
  inspect_fill_rect(pos, rx0, y0, rx1, y0 + g.thick);
}

struct InspectPlacedMatrix {
  rctf box;
  InspectMatrixGeom geom;
  int index = 0;
  bool comma = false;
};

static Vector<InspectPlacedMatrix> inspect_place_matrices(const InspectPopupArg &arg,
                                                          const rcti &rect,
                                                          const int fontid)
{
  Vector<InspectPlacedMatrix> placed;
  const float xmin = float(rect.xmin);
  const float xmax = float(rect.xmax);
  const float ymax = float(rect.ymax);
  const float ymin = float(rect.ymin);
  const float item_gap = 4.0f * UI_SCALE_FAC;
  float x = xmin;
  float y = ymax;
  const int n = int(arg.matrices.size());
  for (int i = 0; i < n; i++) {
    InspectMatrixGeom g = inspect_measure_matrix(arg.matrices[i], fontid, arg.precision);
    const bool comma = (i + 1 < n) || arg.truncated;
    const float need = g.width + (comma ? g.comma_w : 0.0f);
    if (x > xmin && x + need > xmax) {
      x = xmin;
      y -= g.height + item_gap;
    }
    if (y - g.height < ymin) {
      break;
    }
    InspectPlacedMatrix p;
    p.box.xmin = x;
    p.box.xmax = x + g.width;
    p.box.ymax = y;
    p.box.ymin = y - g.height;
    p.geom = g;
    p.index = i;
    p.comma = comma;
    placed.append(p);
    x += need + item_gap;
  }
  return placed;
}

static float inspect_content_height(const InspectPopupArg &arg, const int width_px)
{
  const int fontid = inspect_fontid();
  BLF_size(fontid, inspect_font_px());
  rcti probe;
  probe.xmin = 0;
  probe.xmax = std::max(width_px, 8);
  probe.ymax = 1 << 20;
  probe.ymin = 0;
  float bottom = float(probe.ymax);
  if (!arg.matrices.is_empty()) {
    const Vector<InspectPlacedMatrix> placed = inspect_place_matrices(arg, probe, fontid);
    if (placed.is_empty()) {
      bottom = float(probe.ymax);
    }
    else {
      bottom = placed.last().box.ymin;
    }
  }
  if (!arg.items.is_empty() || arg.truncated) {
    const float em = std::max(1.0f, float(BLF_ascender(fontid) - BLF_descender(fontid)));
    const float row_h = em * 1.2f;
    if (!arg.matrices.is_empty()) {
      bottom -= row_h * 0.4f;
    }
    float x = 0.0f;
    float y = bottom;
    auto emit = [&](const StringRef s) {
      const float w = BLF_width(fontid, s.data(), s.size());
      if (x > 0.0f && x + w > float(width_px)) {
        x = 0.0f;
        y -= row_h;
      }
      x += w;
    };
    for (int i = 0; i < int(arg.items.size()); i++) {
      std::string s = arg.items[i];
      if (i + 1 < int(arg.items.size()) || arg.truncated) {
        s += ", ";
      }
      emit(s);
    }
    if (arg.truncated) {
      emit("…");
    }
    bottom = y - row_h;
  }
  return std::max(8.0f, float(probe.ymax) - bottom);
}

static void inspect_draw_canvas(const InspectPopupArg &arg, const rcti &rect)
{
  const int fontid = inspect_fontid();
  BLF_size(fontid, inspect_font_px());
  uchar col[4];
  ui::theme::get_color_4ubv(TH_TEXT, col);

  const Vector<InspectPlacedMatrix> placed = inspect_place_matrices(arg, rect, fontid);
  if (!placed.is_empty()) {
    GPU_blend(GPU_BLEND_ALPHA);
    const uint pos = GPU_vertformat_attr_add(
        immVertexFormat(), "pos", gpu::VertAttrType::SFLOAT_32_32);
    immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);
    immUniformColor4ubv(col);
    for (const InspectPlacedMatrix &p : placed) {
      inspect_draw_brackets(pos, p.box, p.geom);
    }
    immUnbindProgram();
    GPU_blend(GPU_BLEND_NONE);
  }

  BLF_color4ubv(fontid, col);
  const int desc = BLF_descender(fontid);
  for (const InspectPlacedMatrix &p : placed) {
    const InspectMatrixGeom &g = p.geom;
    const float4x4 &matrix = arg.matrices[p.index];
    float nums_x = p.box.xmin + g.thick + g.serif + g.pad;
    for (int col = 0; col < 4; col++) {
      for (int row = 0; row < 4; row++) {
        char cell[40];
        inspect_fmt_cell(cell, sizeof(cell), matrix[col][row], arg.precision, col < 3);
        const float cw = BLF_width(fontid, cell, strlen(cell));
        const float row_top = p.box.ymax - g.pad - float(row) * g.row_h;
        const float extra = std::max(0.0f, g.row_h - g.em);
        const float baseline = row_top - extra * 0.5f - g.em - float(desc);
        const float tx = nums_x + (g.col_w[col] - cw);
        BLF_position(fontid, tx, baseline, 0.0f);
        BLF_draw(fontid, cell, strlen(cell));
      }
      nums_x += g.col_w[col] + g.gap;
    }
    if (p.comma) {
      const float extra = std::max(0.0f, g.row_h - g.em);
      const float baseline = p.box.ymin + g.pad + extra * 0.5f - float(desc);
      BLF_position(fontid, p.box.xmax + 2.0f * UI_SCALE_FAC, baseline, 0.0f);
      BLF_draw(fontid, ",", 1);
    }
  }
  BLF_batch_draw_flush();

  const bool drew_all_matrices = placed.size() == arg.matrices.size();
  if (arg.items.is_empty() && !arg.truncated && drew_all_matrices) {
    return;
  }

  float y = float(rect.ymax);
  if (!placed.is_empty()) {
    y = placed.last().box.ymin - 4.0f * UI_SCALE_FAC;
  }
  const float em = std::max(1.0f, float(BLF_ascender(fontid) - BLF_descender(fontid)));
  const float row_h = em * 1.2f;
  float x = float(rect.xmin);
  auto draw_token = [&](const StringRef s) -> bool {
    const float w = BLF_width(fontid, s.data(), s.size());
    if (x > float(rect.xmin) && x + w > float(rect.xmax)) {
      x = float(rect.xmin);
      y -= row_h;
    }
    if (y - row_h < float(rect.ymin)) {
      BLF_position(fontid, x, y - em - float(desc), 0.0f);
      BLF_draw(fontid, "…", strlen("…"));
      return false;
    }
    BLF_position(fontid, x, y - em - float(desc), 0.0f);
    BLF_draw(fontid, s.data(), s.size());
    x += w;
    return true;
  };
  if (!drew_all_matrices) {
    draw_token("…");
    BLF_batch_draw_flush();
    return;
  }
  for (int i = 0; i < int(arg.items.size()); i++) {
    std::string s = arg.items[i];
    if (i + 1 < int(arg.items.size()) || arg.truncated) {
      s += ", ";
    }
    if (!draw_token(s)) {
      break;
    }
  }
  if (arg.truncated) {
    draw_token("…");
  }
  BLF_batch_draw_flush();
}

static void inspect_reload_from_context(bContext *C, InspectPopupArg &arg);

static ui::Block *spreadsheet_inspect_popup(bContext *C, ARegion *region, void *arg_v)
{
  InspectPopupArg *arg = static_cast<InspectPopupArg *>(arg_v);
  inspect_reload_from_context(C, *arg);
  const uiStyle *style = ui::style_get_dpi();
  ui::Block *block = ui::block_begin(C, region, "spreadsheet_inspect", ui::EmbossType::Emboss);
  ui::block_flag_enable(block,
                        ui::BLOCK_KEEP_OPEN | ui::BLOCK_LOOP | ui::BLOCK_NO_ACCELERATOR_KEYS);
  if (arg->pinned) {
    ui::block_flag_enable(block, ui::BLOCK_PINNED);
  }
  ui::block_theme_style_set(block, ui::BLOCK_THEME_STYLE_REGULAR);

  const int width = arg->matrices.is_empty() ? int(28 * UI_UNIT_X) : int(34 * UI_UNIT_X);
  const int canvas_w = std::max(8, width - int(1.2f * UI_UNIT_X));
  const int canvas_h = std::clamp(
      int(std::ceil(inspect_content_height(*arg, canvas_w))),
      int(2 * UI_UNIT_Y),
      int(22 * UI_UNIT_Y));

  ui::Layout &layout = ui::block_layout(block,
                                        ui::LayoutDirection::Vertical,
                                        ui::LayoutType::Panel,
                                        0,
                                        0,
                                        width,
                                        int(2 * UI_UNIT_Y),
                                        4,
                                        style);

  ui::Layout &header = layout.row(true);
  header.label(arg->title, ICON_INFO);
  header.separator_spacer();
  header.button("",
                arg->pinned ? ICON_PINNED : ICON_UNPINNED,
                [arg](bContext &C) {
                  arg->pinned = !arg->pinned;
                  if (ARegion *popup = CTX_wm_region_popup(&C)) {
                    ED_region_tag_refresh_ui(popup);
                    ED_region_tag_redraw(popup);
                  }
                },
                arg->pinned ? TIP_("Unpin") : TIP_("Keep this window open"));
  header.button("", ICON_X, [block](bContext & /*C*/) { ui::popup_menu_close(block, false); }, IFACE_("Close"));
  layout.separator();

  ui::uiDefBut(block,
               ui::ButtonType::Extra,
               "",
               0,
               0,
               short(std::min(canvas_w, 32767)),
               short(std::min(canvas_h, 32767)),
               nullptr,
               0.0f,
               0.0f,
               "");
  ui::button_func_drawextra_set(
      block, [arg](const bContext * /*C*/, rcti *rect) { inspect_draw_canvas(*arg, *rect); });

  const int bounds_offset[2] = {0, -UI_UNIT_Y};
  ui::block_bounds_set_popup(block, int(0.4f * U.widget_unit), bounds_offset);
  return block;
}

static void spreadsheet_inspect_popup_free(void *arg_v)
{
  MEM_delete(static_cast<InspectPopupArg *>(arg_v));
}

static void inspect_append_array(InspectPopupArg &arg, const bke::WrangleArrayValue &arr)
{
  const int n = std::min(int(arr.count), int(bke::WrangleArrayValue::max_values));
  arg.truncated = arr.truncated || arr.total > arr.count;
  if (arr.kind == bke::WrangleArrayKind::Matrix) {
    for (int i = 0; i < n; i++) {
      arg.matrices.append(arr.matrix_at(i));
    }
    return;
  }
  if (arr.kind == bke::WrangleArrayKind::Matrix2 || arr.kind == bke::WrangleArrayKind::Matrix3) {
    arg.items.append(arr.to_string());
    return;
  }
  if (arr.kind == bke::WrangleArrayKind::Float3) {
    for (int i = 0; i < n; i++) {
      arg.items.append(fmt::format("({}, {}, {})",
                                   inspect_fmt_float(arr.d.f[i * 3 + 0], arg.precision),
                                   inspect_fmt_float(arr.d.f[i * 3 + 1], arg.precision),
                                   inspect_fmt_float(arr.d.f[i * 3 + 2], arg.precision)));
    }
  }
  else if (arr.kind == bke::WrangleArrayKind::String) {
    for (int i = 0; i < n; i++) {
      arg.items.append(fmt::format("\"{}\"", arr.string_at(i)));
    }
  }
  else if (arr.kind == bke::WrangleArrayKind::Float) {
    for (int i = 0; i < n; i++) {
      arg.items.append(inspect_fmt_float(arr.d.f[i], arg.precision));
    }
  }
  else if (arr.kind == bke::WrangleArrayKind::Ray) {
    for (int i = 0; i < n; i++) {
      arg.items.append(fmt::format("ray({}, {})",
                                   int(arr.d.f[i * bke::WrangleArrayValue::ray_floats]),
                                   inspect_fmt_float(
                                       arr.d.f[i * bke::WrangleArrayValue::ray_floats + 7],
                                       arg.precision)));
    }
  }
  else {
    for (int i = 0; i < n; i++) {
      arg.items.append(fmt::format("{}", arr.d.i[i]));
    }
  }
}

static SpreadsheetColumn *inspect_find_column(SpaceSpreadsheet &sspreadsheet, const char *name);

static void inspect_fill_from_values(InspectPopupArg &arg,
                                     const ColumnValues &values,
                                     const int domain)
{
  arg.matrices.clear();
  arg.items.clear();
  arg.truncated = false;
  arg.title = fmt::format("{}  [{}]", values.name(), domain);
  if (domain < 0 || domain >= values.size()) {
    arg.items.append(IFACE_("(out of range)"));
    return;
  }
  const GVArray &data = values.data();
  if (data.type().is<float4x4>()) {
    arg.matrices.append(data.typed<float4x4>()[domain]);
  }
  else if (data.type().is<bke::WrangleArrayValue>()) {
    inspect_append_array(arg, data.typed<bke::WrangleArrayValue>()[domain]);
  }
  else if (data.type().is<float3>()) {
    const float3 v = data.typed<float3>()[domain];
    arg.items.append(fmt::format("({}, {}, {})",
                                 inspect_fmt_float(v.x, arg.precision),
                                 inspect_fmt_float(v.y, arg.precision),
                                 inspect_fmt_float(v.z, arg.precision)));
  }
  else if (data.type().is<float>()) {
    arg.items.append(inspect_fmt_float(data.typed<float>()[domain], arg.precision));
  }
  else if (data.type().is<int>()) {
    arg.items.append(fmt::format("{}", data.typed<int>()[domain]));
  }
  else if (data.type().is<bool>()) {
    arg.items.append(data.typed<bool>()[domain] ? "true" : "false");
  }
  else if (data.type().is<std::string>()) {
    arg.items.append(data.typed<std::string>()[domain]);
  }
  else if (data.type().is<MStringProperty>()) {
    arg.items.append(data.typed<MStringProperty>()[domain].s);
  }
  else if (data.type().is<math::Quaternion>()) {
    const math::Quaternion q = data.typed<math::Quaternion>()[domain];
    arg.items.append(fmt::format("({}, {}, {}, {})",
                                 inspect_fmt_float(q.x, arg.precision),
                                 inspect_fmt_float(q.y, arg.precision),
                                 inspect_fmt_float(q.z, arg.precision),
                                 inspect_fmt_float(q.w, arg.precision)));
  }
  else if (data.type().is<float4>()) {
    const float4 v = data.typed<float4>()[domain];
    arg.items.append(fmt::format("({}, {}, {}, {})",
                                 inspect_fmt_float(v.x, arg.precision),
                                 inspect_fmt_float(v.y, arg.precision),
                                 inspect_fmt_float(v.z, arg.precision),
                                 inspect_fmt_float(v.w, arg.precision)));
  }
  else if (data.type().is<ColorGeometry4f>()) {
    const ColorGeometry4f c = data.typed<ColorGeometry4f>()[domain];
    arg.items.append(fmt::format("({}, {}, {}, {})",
                                 inspect_fmt_float(c.r, arg.precision),
                                 inspect_fmt_float(c.g, arg.precision),
                                 inspect_fmt_float(c.b, arg.precision),
                                 inspect_fmt_float(c.a, arg.precision)));
  }
  else {
    arg.items.append(IFACE_("(no inspect preview for this type)"));
  }
}

static void inspect_reload_from_context(bContext *C, InspectPopupArg &arg)
{
  if (!C || arg.column_id.empty() || arg.row < 0) {
    return;
  }
  SpaceSpreadsheet *sspreadsheet = CTX_wm_space_spreadsheet(C);
  if (!sspreadsheet) {
    return;
  }
  SpreadsheetColumn *column = inspect_find_column(*sspreadsheet, arg.column_id.c_str());
  if (!column || !column->id) {
    return;
  }
  std::unique_ptr<DataSource> data_source = get_data_source(*C);
  if (!data_source) {
    return;
  }
  std::unique_ptr<ColumnValues> values = data_source->get_column_values(*column->id);
  if (!values) {
    return;
  }
  arg.precision = spreadsheet_decimal_precision(*sspreadsheet);
  inspect_fill_from_values(arg, *values, arg.row);
}

static SpreadsheetColumn *inspect_find_column(SpaceSpreadsheet &sspreadsheet, const char *name)
{
  SpreadsheetTable *table = get_active_table(sspreadsheet);
  if (!table || !name || name[0] == '\0') {
    return nullptr;
  }
  for (SpreadsheetColumn *column : Span{table->columns, table->num_columns}) {
    if (column->id && column->id->name && STREQ(column->id->name, name)) {
      return column;
    }
  }
  return nullptr;
}

static bool inspect_cell_from_event(bContext *C,
                                    const wmEvent *event,
                                    SpreadsheetColumn *&r_column,
                                    int &r_domain)
{
  r_column = nullptr;
  r_domain = -1;
  if (!event) {
    return false;
  }
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(C);
  ARegion &region = *CTX_wm_region(C);
  const int2 cursor_re{event->mval[0], event->mval[1]};
  if (ui::view2d_mouse_in_scrollers(&region, &region.v2d, event->xy)) {
    return false;
  }
  if (is_hovering_header_row(sspreadsheet, region, cursor_re)) {
    return false;
  }
  SpreadsheetColumn *column = find_hovered_column(sspreadsheet, region, cursor_re);
  const std::optional<int> hovered = spreadsheet_find_hovered_display_row(
      sspreadsheet, region, cursor_re);
  if (!column || !column->id || !hovered.has_value()) {
    return false;
  }
  r_column = column;
  r_domain = spreadsheet_display_row_to_domain(*sspreadsheet.runtime, *hovered);
  return r_domain >= 0;
}

static wmOperatorStatus inspect_open(bContext *C, SpreadsheetColumn *column, const int domain)
{
  if (!column || !column->id || domain < 0) {
    return OPERATOR_PASS_THROUGH;
  }
  std::unique_ptr<DataSource> data_source = get_data_source(*C);
  if (!data_source) {
    return OPERATOR_PASS_THROUGH;
  }
  std::unique_ptr<ColumnValues> values = data_source->get_column_values(*column->id);
  if (!values || domain >= values->size()) {
    return OPERATOR_PASS_THROUGH;
  }
  InspectPopupArg *arg = MEM_new<InspectPopupArg>(__func__);
  arg->column_id = column->id->name;
  arg->row = domain;
  arg->precision = spreadsheet_decimal_precision(*CTX_wm_space_spreadsheet(C));
  inspect_fill_from_values(*arg, *values, domain);

  ui::popup_block_invoke(C, spreadsheet_inspect_popup, arg, spreadsheet_inspect_popup_free);
  return OPERATOR_FINISHED;
}

static wmOperatorStatus inspect_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
  SpaceSpreadsheet &sspreadsheet = *CTX_wm_space_spreadsheet(C);
  const int prop_row = RNA_int_get(op->ptr, "row");
  char colname[256];
  RNA_string_get(op->ptr, "column", colname);
  SpreadsheetColumn *column = nullptr;
  int domain = -1;
  if (prop_row >= 0 && colname[0] != '\0') {
    column = inspect_find_column(sspreadsheet, colname);
    domain = prop_row;
  }
  else if (!inspect_cell_from_event(C, event, column, domain)) {
    return OPERATOR_PASS_THROUGH;
  }
  return inspect_open(C, column, domain);
}

static void SPREADSHEET_OT_inspect(wmOperatorType *ot)
{
  ot->name = "Inspect";
  ot->description =
      "Inspect the spreadsheet cell (matrices as compact 4×4 widgets, other values "
      "comma-separated)";
  ot->idname = "SPREADSHEET_OT_inspect";
  ot->invoke = inspect_invoke;
  ot->poll = ED_operator_spreadsheet_active;
  ot->flag = OPTYPE_REGISTER;

  RNA_def_int(ot->srna, "row", -1, -1, INT_MAX, "Row", "Domain index of the cell", -1, INT_MAX);
  RNA_def_string(ot->srna, "column", nullptr, 256, "Column", "Column identifier");
}

static wmOperatorStatus inspect_menu_invoke(bContext *C, wmOperator * /*op*/, const wmEvent *event)
{
  SpreadsheetColumn *column = nullptr;
  int domain = -1;
  if (!inspect_cell_from_event(C, event, column, domain) || !column || !column->id) {
    return OPERATOR_PASS_THROUGH;
  }

  ui::PopupMenu *pup = ui::popup_menu_begin(C, IFACE_("Cell"), ICON_NONE);
  ui::Layout &layout = *ui::popup_menu_layout(pup);
  PointerRNA ptr = layout.op("SPREADSHEET_OT_inspect",
                             IFACE_("Inspect"),
                             ICON_VIEWZOOM,
                             wm::OpCallContext::InvokeDefault,
                             UI_ITEM_NONE);
  RNA_int_set(&ptr, "row", domain);
  RNA_string_set(&ptr, "column", column->id->name);
  ui::popup_menu_end(C, pup);
  return OPERATOR_INTERFACE;
}

static void SPREADSHEET_OT_inspect_menu(wmOperatorType *ot)
{
  ot->name = "Inspect Menu";
  ot->description = "Show a menu with Inspect for the spreadsheet cell under the cursor";
  ot->idname = "SPREADSHEET_OT_inspect_menu";
  ot->invoke = inspect_menu_invoke;
  ot->poll = ED_operator_spreadsheet_active;
  ot->flag = OPTYPE_INTERNAL;
}

void spreadsheet_operatortypes()
{
  WM_operatortype_append(SPREADSHEET_OT_add_row_filter_rule);
  WM_operatortype_append(SPREADSHEET_OT_remove_row_filter_rule);
  WM_operatortype_append(SPREADSHEET_OT_change_spreadsheet_data_source);
  WM_operatortype_append(SPREADSHEET_OT_resize_column);
  WM_operatortype_append(SPREADSHEET_OT_fit_column);
  WM_operatortype_append(SPREADSHEET_OT_reorder_columns);
  WM_operatortype_append(SPREADSHEET_OT_sort_column);
  WM_operatortype_append(SPREADSHEET_OT_select);
  WM_operatortype_append(SPREADSHEET_OT_select_all);
  WM_operatortype_append(SPREADSHEET_OT_inspect);
  WM_operatortype_append(SPREADSHEET_OT_inspect_menu);
}

}  // namespace blender::ed::spreadsheet
