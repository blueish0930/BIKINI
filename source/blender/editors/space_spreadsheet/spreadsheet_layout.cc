/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <iomanip>
#include <sstream>

#include <fmt/format.h>

#include "BLF_api.hh"

#include "BLI_color.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string.hh"

#include "BKE_instances.hh"
#include "BKE_lib_id.hh"
#include "BKE_wrangle_array.hh"

#include "DEG_depsgraph_query.hh"

#include "ED_outliner.hh"

#include "NOD_geometry_nodes_bundle.hh"

#include "spreadsheet_column_values.hh"
#include "spreadsheet_data_source_geometry.hh"
#include "spreadsheet_layout.hh"

#include "DNA_collection_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_sound_types.h"
#include "DNA_vfont_types.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLT_translation.hh"

#include "spreadsheet_intern.hh"

/* Need to do our own padding in some cases because we use low-level ui code to draw the
 * spreadsheet. */
#define CELL_PADDING_X (0.15f * SPREADSHEET_WIDTH_UNIT)

namespace blender::ed::spreadsheet {

static std::string format_preview_float(const float value, const int precision)
{
  return fmt::format("{:.{}f}", double(value), std::clamp(precision, 0, 9));
}

static std::string format_float4x4_cell(const float4x4 &value, const int precision)
{
  const float *f = &value[0][0];
  std::string s = "[";
  for (int i = 0; i < 16; i++) {
    s += format_preview_float(f[i], precision);
    if (i + 1 < 16) {
      s += ", ";
    }
  }
  s += "]";
  return s;
}

static std::string format_wrangle_array_cell(const bke::WrangleArrayValue &arr,
                                             const int precision)
{
  std::string s = "[";
  const int n = int(arr.count);
  auto comma = [&](const int i) {
    if (i + 1 < n) {
      s += ", ";
    }
  };
  if (arr.kind == bke::WrangleArrayKind::Float3) {
    for (int i = 0; i < n; i++) {
      s += '(';
      s += format_preview_float(arr.d.f[i * 3 + 0], precision);
      s += ", ";
      s += format_preview_float(arr.d.f[i * 3 + 1], precision);
      s += ", ";
      s += format_preview_float(arr.d.f[i * 3 + 2], precision);
      s += ')';
      comma(i);
    }
  }
  else if (arr.kind == bke::WrangleArrayKind::String) {
    for (int i = 0; i < n; i++) {
      s += '"';
      s += arr.string_at(i);
      s += '"';
      comma(i);
    }
  }
  else if (arr.kind == bke::WrangleArrayKind::Float) {
    for (int i = 0; i < n; i++) {
      s += format_preview_float(arr.d.f[i], precision);
      comma(i);
    }
  }
  else if (arr.kind == bke::WrangleArrayKind::Matrix) {
    for (int i = 0; i < n; i++) {
      s += format_float4x4_cell(arr.matrix_at(i), precision);
      comma(i);
    }
  }
  else if (arr.kind == bke::WrangleArrayKind::Matrix2) {
    s += '[';
    for (int k = 0; k < 4; k++) {
      s += format_preview_float(arr.d.f[k], precision);
      if (k + 1 < 4) {
        s += ", ";
      }
    }
    s += ']';
  }
  else if (arr.kind == bke::WrangleArrayKind::Matrix3) {
    s += '[';
    for (int k = 0; k < 9; k++) {
      s += format_preview_float(arr.d.f[k], precision);
      if (k + 1 < 9) {
        s += ", ";
      }
    }
    s += ']';
  }
  else if (arr.kind == bke::WrangleArrayKind::Ray) {
    for (int i = 0; i < n; i++) {
      s += "ray(";
      s += fmt::format("{}", int(arr.d.f[i * bke::WrangleArrayValue::ray_floats]));
      s += ", ";
      s += format_preview_float(arr.d.f[i * bke::WrangleArrayValue::ray_floats + 7], precision);
      s += ')';
      comma(i);
    }
  }
  else {
    for (int i = 0; i < n; i++) {
      s += fmt::format("{}", arr.d.i[i]);
      comma(i);
    }
  }
  if (arr.truncated || arr.total > arr.count) {
    s += ", …";
  }
  s += "]";
  return s;
}

class SpreadsheetLayoutDrawer : public SpreadsheetDrawer {
 private:
  const SpreadsheetLayout &spreadsheet_layout_;

  bool is_column_sorted(const StringRef column_name) const
  {
    return spreadsheet_layout_.sort_direction != SPREADSHEET_SORT_NONE &&
           spreadsheet_layout_.sort_column_name == column_name;
  }

  BIFIconID sort_icon_for_column(const StringRef column_name) const
  {
    if (!this->is_column_sorted(column_name)) {
      /* Neutral indicator when this column is not the active sort key. */
      return ICON_TRIA_DOWN;
    }
    if (spreadsheet_layout_.sort_direction == SPREADSHEET_SORT_ASC) {
      return ICON_TRIA_UP;
    }
    return ICON_TRIA_DOWN;
  }

 public:
  SpreadsheetLayoutDrawer(const SpreadsheetLayout &spreadsheet_layout)
      : spreadsheet_layout_(spreadsheet_layout)
  {
    tot_columns = spreadsheet_layout.columns.size();
    tot_rows = int(spreadsheet_layout.tot_rows());
    left_column_width = spreadsheet_layout.index_column_width;
    index_sort_direction = spreadsheet_layout.sort_direction;
    index_sort_active = spreadsheet_layout.sort_direction != SPREADSHEET_SORT_NONE &&
                        spreadsheet_layout.sort_column_name.is_empty();
  }

  int domain_index(int row_index) const final
  {
    if (row_index < 0 || row_index >= tot_rows) {
      return -1;
    }
    return spreadsheet_layout_.domain_index(row_index);
  }

  void draw_top_row_cell(int column_index, const CellDrawParams &params) const final
  {
    const ColumnValues &values = *spreadsheet_layout_.columns[column_index].values;
    const StringRefNull name = values.name();
    const int triangle_width = int(SPREADSHEET_SORT_TRIANGLE_WIDTH);
    const int name_width = std::max(0, params.width - triangle_width);

    std::string description = values.description().is_empty() ?
                                  std::string(name) :
                                  fmt::format("{}\n{}", name, values.description());
    description = fmt::format("{}\n{}",
                              description,
                              TIP_("Click header to cycle sort: ascending, descending, "
                                   "original order"));
    ui::Button *but = uiDefIconTextBut(params.block,
                                       ui::ButtonType::Label,
                                       ICON_NONE,
                                       name,
                                       params.xmin,
                                       params.ymin,
                                       name_width,
                                       params.height,
                                       nullptr,
                                       std::nullopt);
    button_func_tooltip_set(
        but,
        [](bContext * /*C*/, void *arg, StringRef /*tip*/) {
          return *static_cast<std::string *>(arg);
        },
        MEM_new<std::string>(__func__, std::move(description)),
        [](void *arg) { MEM_delete(static_cast<std::string *>(arg)); });
    /* Center-align column headers. */
    button_drawflag_disable(but, ui::BUT_TEXT_LEFT);
    button_drawflag_disable(but, ui::BUT_TEXT_RIGHT);

    /* Sort triangle: status indicator only (click is handled on the whole header cell). */
    const BIFIconID sort_icon = this->sort_icon_for_column(name);
    ui::Button *sort_but = uiDefIconBut(params.block,
                                        ui::ButtonType::Label,
                                        sort_icon,
                                        params.xmin + name_width,
                                        params.ymin,
                                        short(triangle_width),
                                        short(params.height),
                                        nullptr,
                                        0.0f,
                                        0.0f,
                                        std::nullopt);
    if (!this->is_column_sorted(name)) {
      button_flag_enable(sort_but, ui::BUT_INACTIVE);
    }
  }

  void draw_left_column_cell(int row_index, const CellDrawParams &params) const final
  {
    const int real_index = spreadsheet_layout_.domain_index(row_index);
    std::string index_str = std::to_string(real_index);
    ui::Button *but = uiDefIconTextBut(params.block,
                                       ui::ButtonType::Label,
                                       ICON_NONE,
                                       index_str,
                                       params.xmin,
                                       params.ymin,
                                       params.width,
                                       params.height,
                                       nullptr,
                                       std::nullopt);
    /* Right-align indices. */
    button_drawflag_enable(but, ui::BUT_TEXT_RIGHT);
    button_drawflag_disable(but, ui::BUT_TEXT_LEFT);
  }

  void draw_content_cell(int row_index, int column_index, const CellDrawParams &params) const final
  {
    const int real_index = spreadsheet_layout_.domain_index(row_index);
    const ColumnValues &column = *spreadsheet_layout_.columns[column_index].values;
    if (real_index > column.size()) {
      return;
    }

    const GVArray &data = column.data();
    const CPPType &type = data.type();
    BUFFER_FOR_CPP_TYPE_VALUE(type, buffer);
    data.get_to_uninitialized(real_index, buffer);
    this->draw_content_cell_value(GPointer(type, buffer), params, column);
    type.destruct(buffer);
  }

  void draw_content_cell_value(const GPointer value_ptr,
                               const CellDrawParams &params,
                               const ColumnValues &column) const
  {
    const CPPType &type = *value_ptr.type();
    if (type.is<int>()) {
      this->draw_int(params, *value_ptr.get<int>(), column.display_hint());
      return;
    }
    if (type.is<int64_t>()) {
      this->draw_int(params, *value_ptr.get<int64_t>(), column.display_hint());
      return;
    }
    if (type.is<int8_t>()) {
      const int8_t value = *value_ptr.get<int8_t>();
      const std::string value_str = std::to_string(value);
      ui::Button *but = uiDefIconTextBut(params.block,
                                         ui::ButtonType::Label,
                                         ICON_NONE,
                                         value_str,
                                         params.xmin,
                                         params.ymin,
                                         params.width,
                                         params.height,
                                         nullptr,
                                         std::nullopt);
      /* Right-align Integers. */
      button_drawflag_disable(but, ui::BUT_TEXT_LEFT);
      button_drawflag_enable(but, ui::BUT_TEXT_RIGHT);
      return;
    }
    if (type.is<short2>()) {
      const int2 value = int2(*value_ptr.get<short2>());
      this->draw_int_vector(params, Span(&value.x, 2));
      return;
    }
    if (type.is<int2>()) {
      const int2 value = *value_ptr.get<int2>();
      this->draw_int_vector(params, Span(&value.x, 2));
      return;
    }
    if (type.is<int3>()) {
      const int3 value = *value_ptr.get<int3>();
      this->draw_int_vector(params, Span(&value.x, 3));
      return;
    }
    if (type.is<float>()) {
      const float value = *value_ptr.get<float>();
      const std::string value_str = format_preview_float(value,
                                                         spreadsheet_layout_.decimal_precision);
      ui::Button *but = uiDefIconTextBut(params.block,
                                         ui::ButtonType::Label,
                                         ICON_NONE,
                                         value_str,
                                         params.xmin,
                                         params.ymin,
                                         params.width,
                                         params.height,
                                         nullptr,
                                         std::nullopt);
      button_func_tooltip_set(
          but,
          [](bContext * /*C*/, void *argN, const StringRef /*tip*/) {
            return fmt::format("{:f}", *(static_cast<float *>(argN)));
          },
          MEM_new<float>(__func__, value),
          MEM_delete_void);
      /* Right-align Floats. */
      button_drawflag_disable(but, ui::BUT_TEXT_LEFT);
      button_drawflag_enable(but, ui::BUT_TEXT_RIGHT);
      return;
    }
    if (type.is<bool>()) {
      const bool value = *value_ptr.get<bool>();
      const int icon = value ? ICON_CHECKBOX_HLT : ICON_CHECKBOX_DEHLT;
      ui::Button *but = uiDefIconTextBut(params.block,
                                         ui::ButtonType::Label,
                                         icon,
                                         "",
                                         params.xmin,
                                         params.ymin,
                                         params.width,
                                         params.height,
                                         nullptr,
                                         std::nullopt);
      button_drawflag_disable(but, ui::BUT_ICON_LEFT);
      return;
    }
    if (type.is<float2>()) {
      const float2 value = *value_ptr.get<float2>();
      this->draw_float_vector(params, Span(&value.x, 2));
      return;
    }
    if (type.is<float3>()) {
      const float3 value = *value_ptr.get<float3>();
      this->draw_float_vector(params, Span(&value.x, 3));
      return;
    }
    if (type.is<float4>()) {
      const float4 value = *value_ptr.get<float4>();
      this->draw_float_vector(params, Span(&value.x, 4));
      return;
    }
    if (type.is<ColorGeometry4f>()) {
      const ColorGeometry4f value = *value_ptr.get<ColorGeometry4f>();
      this->draw_float_vector(params, Span(&value.r, 4));
      return;
    }
    if (type.is<ColorGeometry4b>()) {
      const ColorGeometry4b value = *value_ptr.get<ColorGeometry4b>();
      this->draw_byte_color(params, value);
      return;
    }
    if (type.is<math::Quaternion>()) {
      const float4 value = float4(*value_ptr.get<math::Quaternion>());
      this->draw_float_vector(params, Span(&value.x, 4));
      return;
    }
    if (type.is<float4x4>()) {
      this->draw_float4x4(params, *value_ptr.get<float4x4>());
      return;
    }
    if (type.is<bke::InstanceReference>()) {
      const bke::InstanceReference value = *value_ptr.get<bke::InstanceReference>();
      const StringRefNull name = value.name().is_empty() ? IFACE_("(Geometry)") : value.name();
      const int icon = get_instance_reference_icon(value);
      uiDefIconTextBut(params.block,
                       ui::ButtonType::Label,
                       icon,
                       name.c_str(),
                       params.xmin,
                       params.ymin,
                       params.width,
                       params.height,
                       nullptr,
                       std::nullopt);
      return;
    }
    if (type.is<bke::WrangleArrayValue>()) {
      const std::string value_str = format_wrangle_array_cell(
          *value_ptr.get<bke::WrangleArrayValue>(), spreadsheet_layout_.decimal_precision);
      this->draw_clipped_bracket_cell(params, value_str);
      return;
    }
    if (type.is<std::string>()) {
      uiDefIconTextBut(params.block,
                       ui::ButtonType::Label,
                       ICON_NONE,
                       *value_ptr.get<std::string>(),
                       params.xmin + CELL_PADDING_X,
                       params.ymin,
                       params.width - 2.0f * CELL_PADDING_X,
                       params.height,
                       nullptr,
                       std::nullopt);
      return;
    }
    if (type.is<MStringProperty>()) {
      MStringProperty *prop = MEM_new_zeroed<MStringProperty>(__func__);
      *prop = *value_ptr.get<MStringProperty>();
      const int length = std::min<int>(uint8_t(prop->s_len), sizeof(prop->s));
      ui::Button *but = uiDefIconTextBut(params.block,
                                         ui::ButtonType::Label,
                                         ICON_NONE,
                                         StringRef(prop->s, length),
                                         params.xmin + CELL_PADDING_X,
                                         params.ymin,
                                         params.width - 2.0f * CELL_PADDING_X,
                                         params.height,
                                         nullptr,
                                         std::nullopt);

      button_func_tooltip_set(
          but,
          [](bContext * /*C*/, void *argN, const StringRef /*tip*/) {
            const MStringProperty &prop = *static_cast<MStringProperty *>(argN);
            const int length = std::min<int>(uint8_t(prop.s_len), sizeof(prop.s));
            return std::string(StringRef(prop.s, length));
          },
          prop,
          MEM_delete_void);
      return;
    }
    if (type.is<nodes::BundleItemValue>()) {
      const nodes::BundleItemValue &value = *value_ptr.get<nodes::BundleItemValue>();
      if (const nodes::BundleItemSocketValue *socket_value =
              std::get_if<nodes::BundleItemSocketValue>(&value.value))
      {
        const bke::SocketValueVariant &value_variant = socket_value->value;
        this->draw_content_cell_value(&value_variant, params, column);
        return;
      }
      this->draw_undrawable(params);
      return;
    }
    if (type.is<Object *>()) {
      this->draw_data_block(params, *value_ptr.get<Object *>());
      return;
    }
    if (type.is<Collection *>()) {
      const Collection *collection = *value_ptr.get<Collection *>();
      /* Using original collection because changing the color tag does not cause the eval copy to
       * be updated. */
      const Collection *orig_collection = DEG_get_original(collection);
      if (orig_collection) {
        int icon = ED_outliner_icon_from_id(orig_collection->id);
        if (orig_collection->color_tag != COLLECTION_COLOR_NONE) {
          icon = int(ICON_COLLECTION_COLOR_01) + int(orig_collection->color_tag);
        }
        this->draw_data_block(params, collection, icon);
        return;
      }
      this->draw_data_block(params, collection);
      return;
    }
    if (type.is<Material *>()) {
      this->draw_data_block(params, *value_ptr.get<Material *>());
      return;
    }
    if (type.is<VFont *>()) {
      this->draw_data_block(params, *value_ptr.get<VFont *>());
      return;
    }
    if (type.is<bSound *>()) {
      this->draw_data_block(params, *value_ptr.get<bSound *>());
      return;
    }
    if (type.is<Image *>()) {
      this->draw_data_block(params, *value_ptr.get<Image *>());
      return;
    }
    if (type.is<bke::SocketValueVariant>()) {
      const bke::SocketValueVariant &value_variant = *value_ptr.get<bke::SocketValueVariant>();
      if (value_variant.is_single()) {
        const GPointer single_value_ptr = value_variant.get();
        this->draw_content_cell_value(single_value_ptr, params, column);
        return;
      }
      this->draw_undrawable(params);
      return;
    }
    this->draw_undrawable(params);
  }

  void draw_float_vector(const CellDrawParams &params, const Span<float> values) const
  {
    BLI_assert(!values.is_empty());
    const float segment_width = float(params.width) / values.size();
    for (const int i : values.index_range()) {
      const float value = values[i];
      const std::string value_str = " " +
                                    format_preview_float(value, spreadsheet_layout_.decimal_precision);
      ui::Button *but = uiDefIconTextBut(params.block,
                                         ui::ButtonType::Label,
                                         ICON_NONE,
                                         value_str,
                                         params.xmin + i * segment_width,
                                         params.ymin,
                                         segment_width,
                                         params.height,
                                         nullptr,
                                         std::nullopt);

      button_func_tooltip_set(
          but,
          [](bContext * /*C*/, void *argN, const StringRef /*tip*/) {
            return fmt::format("{:f}", *(static_cast<float *>(argN)));
          },
          MEM_new<float>(__func__, value),
          MEM_delete_void);
      /* Right-align Floats. */
      button_drawflag_disable(but, ui::BUT_TEXT_LEFT);
      button_drawflag_enable(but, ui::BUT_TEXT_RIGHT);
    }
  }

  void draw_int(const CellDrawParams &params,
                const int64_t value,
                const ColumnValueDisplayHint display_hint) const
  {
    std::string value_str;
    switch (display_hint) {
      case ColumnValueDisplayHint::Bytes: {
        char dst[BLI_STR_FORMAT_INT64_BYTE_UNIT_SIZE];
        BLI_str_format_byte_unit(dst, value, true);
        value_str = dst;
        break;
      }
      default: {
        char dst[BLI_STR_FORMAT_INT64_GROUPED_SIZE];
        BLI_str_format_int64_grouped(dst, value);
        value_str = dst;
        break;
      }
    }
    ui::Button *but = uiDefIconTextBut(params.block,
                                       ui::ButtonType::Label,
                                       ICON_NONE,
                                       value_str,
                                       params.xmin,
                                       params.ymin,
                                       params.width,
                                       params.height,
                                       nullptr,
                                       std::nullopt);
    switch (display_hint) {
      case ColumnValueDisplayHint::Bytes: {
        button_func_tooltip_set(
            but,
            [](bContext * /*C*/, void *argN, const StringRef /*tip*/) {
              char dst[BLI_STR_FORMAT_INT64_GROUPED_SIZE];
              BLI_str_format_int64_grouped(dst, *static_cast<int64_t *>(argN));
              return fmt::format("{} {}", dst, TIP_("bytes"));
            },
            MEM_new<int64_t>(__func__, value),
            MEM_delete_void);
        break;
      }
      default: {
        button_func_tooltip_set(
            but,
            [](bContext * /*C*/, void *argN, const StringRef /*tip*/) {
              return fmt::format("{}", *static_cast<int64_t *>(argN));
            },
            MEM_new<int64_t>(__func__, value),
            MEM_delete_void);
        break;
      }
    }
    /* Right-align Integers. */
    button_drawflag_disable(but, ui::BUT_TEXT_LEFT);
    button_drawflag_enable(but, ui::BUT_TEXT_RIGHT);
  }

  void draw_int_vector(const CellDrawParams &params, const Span<int> values) const
  {
    BLI_assert(!values.is_empty());
    const float segment_width = float(params.width) / values.size();
    for (const int i : values.index_range()) {
      std::stringstream ss;
      const int value = values[i];
      ss << " " << value;
      const std::string value_str = ss.str();
      ui::Button *but = uiDefIconTextBut(params.block,
                                         ui::ButtonType::Label,
                                         ICON_NONE,
                                         value_str,
                                         params.xmin + i * segment_width,
                                         params.ymin,
                                         segment_width,
                                         params.height,
                                         nullptr,
                                         std::nullopt);
      button_func_tooltip_set(
          but,
          [](bContext * /*C*/, void *argN, const StringRef /*tip*/) {
            return fmt::format("{}", *(static_cast<int *>(argN)));
          },
          MEM_new<int>(__func__, value),
          MEM_delete_void);
      /* Right-align Floats. */
      button_drawflag_disable(but, ui::BUT_TEXT_LEFT);
      button_drawflag_enable(but, ui::BUT_TEXT_RIGHT);
    }
  }

  void draw_byte_color(const CellDrawParams &params, const ColorGeometry4b color) const
  {
    const ColorGeometry4f float_color = color::decode(color);
    Span<float> values(&float_color.r, 4);
    const float segment_width = float(params.width) / values.size();
    for (const int i : values.index_range()) {
      const float value = values[i];
      const std::string value_str = " " +
                                    format_preview_float(value, spreadsheet_layout_.decimal_precision);
      ui::Button *but = uiDefIconTextBut(params.block,
                                         ui::ButtonType::Label,
                                         ICON_NONE,
                                         value_str,
                                         params.xmin + i * segment_width,
                                         params.ymin,
                                         segment_width,
                                         params.height,
                                         nullptr,
                                         std::nullopt);
      /* Right-align Floats. */
      button_drawflag_disable(but, ui::BUT_TEXT_LEFT);
      button_drawflag_enable(but, ui::BUT_TEXT_RIGHT);

      /* Tooltip showing raw byte values. Encode values in pointer to avoid memory allocation. */
      button_func_tooltip_set(
          but,
          [](bContext * /*C*/, void *argN, const StringRef /*tip*/) {
            const uint32_t uint_color = POINTER_AS_UINT(argN);
            ColorGeometry4b color = *reinterpret_cast<ColorGeometry4b *>(
                const_cast<uint32_t *>(&uint_color));
            return fmt::format(fmt::runtime(TIP_("Byte Color (sRGB encoded):\n{}  {}  {}  {}")),
                               color.r,
                               color.g,
                               color.b,
                               color.a);
          },
          POINTER_FROM_UINT(*(uint32_t *)&color),
          nullptr);
    }
  }

  /* Arrays: if the dump fits, draw one left-aligned label so ']' sits against the last
   * number. If it does not, CLIP_RIGHT the body and glue ']' immediately after it.
   * Always reserving a right-hand ']' slot clipped the last digit and left a gap. */
  void draw_clipped_bracket_cell(const CellDrawParams &params, const std::string &s) const
  {
    const int pad = std::max(int(CELL_PADDING_X + 0.5f), 1);
    const int x0 = params.xmin + pad;
    const int avail = std::max(params.width - 2 * pad, 8);

    const int fontid = BLF_default();
    BLF_size(fontid, UI_DEFAULT_TEXT_POINTS * UI_SCALE_FAC);
    const float full_px = BLF_width(fontid, s.c_str(), s.size());

    auto make_label = [&](const StringRef str, const int x, const int w, const bool clip_right) {
      ui::Button *but = uiDefIconTextBut(params.block,
                                         ui::ButtonType::Label,
                                         ICON_NONE,
                                         str,
                                         x,
                                         params.ymin,
                                         short(std::max(w, 1)),
                                         params.height,
                                         nullptr,
                                         std::nullopt);
      button_drawflag_enable(but, ui::BUT_TEXT_LEFT);
      button_drawflag_disable(but, ui::BUT_TEXT_RIGHT);
      if (clip_right) {
        button_drawflag_enable(but, ui::BUT_TEXT_CLIP_RIGHT);
      }
    };

    if (full_px <= float(avail) + 0.5f) {
      make_label(s, x0, avail, false);
      return;
    }

    const int close_w = std::max(int(BLF_width(fontid, "]", 1) + 0.99f), 4);
    const int body_w = std::max(avail - close_w, 4);
    std::string body = s;
    if (!body.empty() && body.back() == ']') {
      body.pop_back();
    }
    make_label(body, x0, body_w, true);
    make_label("]", x0 + body_w, close_w, false);
  }

  void draw_float4x4(const CellDrawParams &params, const float4x4 &value) const
  {
    const std::string s = format_float4x4_cell(value, spreadsheet_layout_.decimal_precision);
    this->draw_clipped_bracket_cell(params, s);
  }

  template<typename T>
  void draw_data_block(const CellDrawParams &params,
                       const T *id,
                       const std::optional<int> icon_override = std::nullopt) const
  {
    if (!id) {
      uiDefIconTextBut(params.block,
                       ui::ButtonType::Label,
                       icon_override.value_or(ui::icon_from_idcode(int(T::id_type))),
                       "",
                       params.xmin,
                       params.ymin,
                       params.width,
                       params.height,
                       nullptr,
                       std::nullopt);
      return;
    }
    const int icon = icon_override.value_or(ED_outliner_icon_from_id(id_cast<const ID &>(*id)));
    uiDefIconTextBut(params.block,
                     ui::ButtonType::Label,
                     icon,
                     BKE_id_name(id_cast<const ID &>(*id)),
                     params.xmin,
                     params.ymin,
                     params.width,
                     params.height,
                     nullptr,
                     std::nullopt);
  }

  ui::Button *draw_undrawable(const CellDrawParams &params) const
  {
    ui::Button *but = uiDefIconTextBut(params.block,
                                       ui::ButtonType::Label,
                                       ICON_NONE,
                                       "...",
                                       params.xmin,
                                       params.ymin,
                                       params.width,
                                       params.height,
                                       nullptr,
                                       std::nullopt);
    /* Center alignment. */
    button_drawflag_disable(but, ui::BUT_TEXT_LEFT);
    return but;
  }

  int column_width(int column_index) const final
  {
    return spreadsheet_layout_.columns[column_index].width;
  }
};

template<typename T>
static float estimate_max_column_width(const float min_width,
                                       const int fontid,
                                       const std::optional<int64_t> max_sample_size,
                                       const VArray<T> &data,
                                       FunctionRef<std::string(const T &)> to_string)
{
  if (const std::optional<T> value = data.get_if_single()) {
    const std::string str = to_string(*value);
    return std::max(min_width, BLF_width(fontid, str.c_str(), str.size()));
  }
  const int sample_size = max_sample_size.value_or(data.size());
  float width = min_width;
  for (const int i : data.index_range().take_front(sample_size)) {
    const std::string str = to_string(data[i]);
    const float value_width = BLF_width(fontid, str.c_str(), str.size());
    width = std::max(width, value_width);
  }
  return width;
}

float ColumnValues::fit_column_values_width_px(const std::optional<int64_t> &max_sample_size) const
{
  const int fontid = BLF_default();
  BLF_size(fontid, UI_DEFAULT_TEXT_POINTS * UI_SCALE_FAC);

  auto get_min_width = [&](const float min_width) {
    return max_sample_size.has_value() ? min_width : 0.0f;
  };

  const eSpreadsheetColumnValueType column_type = this->type();
  switch (column_type) {
    case SPREADSHEET_VALUE_TYPE_BOOL: {
      return 2.0f * SPREADSHEET_WIDTH_UNIT;
    }
    case SPREADSHEET_VALUE_TYPE_FLOAT4X4: {
      /* Clip to ~4 numbers; Inspect shows the full matrix. */
      return 12.0f * SPREADSHEET_WIDTH_UNIT;
    }
    case SPREADSHEET_VALUE_TYPE_INT8: {
      return estimate_max_column_width<int8_t>(
          get_min_width(3 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<int8_t>(),
          [](const int value) { return fmt::format("{}", value); });
    }
    case SPREADSHEET_VALUE_TYPE_INT32: {
      return estimate_max_column_width<int>(get_min_width(3 * SPREADSHEET_WIDTH_UNIT),
                                            fontid,
                                            max_sample_size,
                                            data_.typed<int>(),
                                            [](const int value) {
                                              char dst[BLI_STR_FORMAT_INT32_GROUPED_SIZE];
                                              BLI_str_format_int_grouped(dst, value);
                                              return std::string(dst);
                                            });
    }
    case SPREADSHEET_VALUE_TYPE_INT64: {
      const ColumnValueDisplayHint display_hint = this->display_hint();
      return estimate_max_column_width<int64_t>(
          get_min_width(3 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<int64_t>(),
          [display_hint](const int64_t value) {
            switch (display_hint) {
              case ColumnValueDisplayHint::Bytes: {
                char dst[BLI_STR_FORMAT_INT64_BYTE_UNIT_SIZE];
                BLI_str_format_byte_unit(dst, value, true);
                return std::string(dst);
              }
              default: {
                char dst[BLI_STR_FORMAT_INT64_GROUPED_SIZE];
                BLI_str_format_int64_grouped(dst, value);
                return std::string(dst);
              }
            }
          });
    }
    case SPREADSHEET_VALUE_TYPE_FLOAT: {
      return estimate_max_column_width<float>(
          get_min_width(3 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<float>(),
          [](const float value) { return fmt::format("{:.3f}", value); });
    }
    case SPREADSHEET_VALUE_TYPE_INT32_2D: {
      if (data_.type().is<short2>()) {
        return estimate_max_column_width<short2>(
            get_min_width(6 * SPREADSHEET_WIDTH_UNIT),
            fontid,
            max_sample_size,
            data_.typed<short2>(),
            [](const short2 value) { return fmt::format("{}  {}", value.x, value.y); });
      }
      return estimate_max_column_width<int2>(
          get_min_width(6 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<int2>(),
          [](const int2 value) { return fmt::format("{}  {}", value.x, value.y); });
    }
    case SPREADSHEET_VALUE_TYPE_INT32_3D: {
      return estimate_max_column_width<int3>(
          get_min_width(9 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<int3>(),
          [](const int3 value) { return fmt::format("{}  {}  {}", value.x, value.y, value.z); });
    }
    case SPREADSHEET_VALUE_TYPE_FLOAT2: {
      return estimate_max_column_width<float2>(
          get_min_width(6 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<float2>(),
          [](const float2 value) { return fmt::format("{:.3f}  {:.3f}", value.x, value.y); });
    }
    case SPREADSHEET_VALUE_TYPE_FLOAT3: {
      return estimate_max_column_width<float3>(
          get_min_width(9 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<float3>(),
          [](const float3 value) {
            return fmt::format("{:.3f}  {:.3f}  {:.3f}", value.x, value.y, value.z);
          });
    }
    case SPREADSHEET_VALUE_TYPE_FLOAT4: {
      return estimate_max_column_width<float4>(
          get_min_width(12 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<float4>(),
          [](const float4 value) {
            return fmt::format(
                "{:.3f}  {:.3f}  {:.3f}  {:.3f}", value.x, value.y, value.z, value.w);
          });
    }
    case SPREADSHEET_VALUE_TYPE_COLOR: {
      return estimate_max_column_width<ColorGeometry4f>(
          get_min_width(12 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<ColorGeometry4f>(),
          [](const ColorGeometry4f value) {
            return fmt::format(
                "{:.3f}  {:.3f}  {:.3f}  {:.3f}", value.r, value.g, value.b, value.a);
          });
    }
    case SPREADSHEET_VALUE_TYPE_BYTE_COLOR: {
      return estimate_max_column_width<ColorGeometry4b>(
          get_min_width(12 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<ColorGeometry4b>(),
          [](const ColorGeometry4b value) {
            const ColorGeometry4f color = color::decode(value);
            return fmt::format(
                "{:.3f}  {:.3f}  {:.3f}  {:.3f}", color.r, color.g, color.b, color.a);
          });
    }
    case SPREADSHEET_VALUE_TYPE_QUATERNION: {
      return estimate_max_column_width<math::Quaternion>(
          get_min_width(12 * SPREADSHEET_WIDTH_UNIT),
          fontid,
          max_sample_size,
          data_.typed<math::Quaternion>(),
          [](const math::Quaternion value) {
            return fmt::format(
                "{:.3f}  {:.3f}  {:.3f}  {:.3f}", value.x, value.y, value.z, value.w);
          });
    }
    case SPREADSHEET_VALUE_TYPE_INSTANCES: {
      return UI_ICON_SIZE + 0.5f * UI_UNIT_X +
             estimate_max_column_width<bke::InstanceReference>(
                 get_min_width(8 * SPREADSHEET_WIDTH_UNIT),
                 fontid,
                 max_sample_size,
                 data_.typed<bke::InstanceReference>(),
                 [](const bke::InstanceReference &value) {
                   const StringRef name = value.name().is_empty() ? IFACE_("(Geometry)") :
                                                                    value.name();
                   return name;
                 });
    }
    case SPREADSHEET_VALUE_TYPE_STRING: {
      if (data_.type().is<std::string>()) {
        return estimate_max_column_width<std::string>(get_min_width(SPREADSHEET_WIDTH_UNIT),
                                                      fontid,
                                                      max_sample_size,
                                                      data_.typed<std::string>(),
                                                      [](const StringRef value) { return value; });
      }
      if (data_.type().is<bke::WrangleArrayValue>()) {
        /* Clip to ~4 numbers; Inspect shows the full array. */
        return 12.0f * SPREADSHEET_WIDTH_UNIT;
      }
      if (data_.type().is<MStringProperty>()) {
        return estimate_max_column_width<MStringProperty>(
            get_min_width(SPREADSHEET_WIDTH_UNIT),
            fontid,
            max_sample_size,
            data_.typed<MStringProperty>(),
            [](const MStringProperty &value) {
              const int length = std::min<int>(uint8_t(value.s_len), sizeof(value.s));
              return StringRef(value.s, length);
            });
      }
      break;
    }
    case SPREADSHEET_VALUE_TYPE_BUNDLE_ITEM: {
      return 12 * SPREADSHEET_WIDTH_UNIT;
    }
    case SPREADSHEET_VALUE_TYPE_UNKNOWN: {
      break;
    }
  }
  return 2.0f * SPREADSHEET_WIDTH_UNIT;
}

float ColumnValues::fit_column_width_px(const std::optional<int64_t> &max_sample_size) const
{
  const float padding_px = 0.5 * SPREADSHEET_WIDTH_UNIT;
  const float min_width_px = SPREADSHEET_WIDTH_UNIT;

  const float data_width_px = this->fit_column_values_width_px(max_sample_size);

  const int fontid = BLF_default();
  BLF_size(fontid, UI_DEFAULT_TEXT_POINTS * UI_SCALE_FAC);
  const float name_width_px = BLF_width(fontid, name_.data(), name_.size());

  const float width_px = std::max(min_width_px,
                                  padding_px + std::max(data_width_px, name_width_px));
  return width_px;
}

std::unique_ptr<SpreadsheetDrawer> spreadsheet_drawer_from_layout(
    const SpreadsheetLayout &spreadsheet_layout)
{
  return std::make_unique<SpreadsheetLayoutDrawer>(spreadsheet_layout);
}

}  // namespace blender::ed::spreadsheet
