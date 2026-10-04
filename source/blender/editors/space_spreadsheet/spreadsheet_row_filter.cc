/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

#include "BLI_bit_vector.hh"
#include "BLI_color.hh"
#include "BLI_fnmatch.hh"
#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"

#include "DNA_space_types.h"

#include "BKE_instances.hh"

#include "spreadsheet_data_source_geometry.hh"
#include "spreadsheet_intern.hh"
#include "spreadsheet_layout.hh"
#include "spreadsheet_row_filter.hh"

namespace blender::ed::spreadsheet {

template<typename T, typename OperationFn>
static IndexMask apply_filter_operation(const VArray<T> &data,
                                        OperationFn check_fn,
                                        const IndexMask &mask,
                                        LinearAllocator<> &memory)
{
  return IndexMask::from_predicate(
      mask, memory, [&](const int64_t i) { return check_fn(data[i]); });
}

static IndexMask apply_row_filter(const SpreadsheetRowFilter &row_filter,
                                  const Map<StringRef, const ColumnValues *> &columns,
                                  const IndexMask &prev_mask,
                                  LinearAllocator<> &memory)
{
  const ColumnValues &column = *columns.lookup(row_filter.column_name);
  const GVArray &column_data = column.data();
  if (column_data.type().is<float>()) {
    const float value = row_filter.value_float;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        const float threshold = row_filter.threshold;
        return apply_filter_operation(
            column_data.typed<float>(),
            [&](const float cell) { return std::abs(cell - value) < threshold; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<float>(),
            [&](const float cell) { return cell > value; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<float>(),
            [&](const float cell) { return cell < value; },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<bool>()) {
    const bool value = (row_filter.flag & SPREADSHEET_ROW_FILTER_BOOL_VALUE) != 0;
    return apply_filter_operation(
        column_data.typed<bool>(),
        [&](const bool cell) { return cell == value; },
        prev_mask,
        memory);
  }
  else if (column_data.type().is<int8_t>()) {
    const int value = row_filter.value_int;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        return apply_filter_operation(
            column_data.typed<int8_t>(),
            [&](const int cell) { return cell == value; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<int8_t>(),
            [value](const int cell) { return cell > value; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<int8_t>(),
            [&](const int cell) { return cell < value; },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<int>()) {
    const int value = row_filter.value_int;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        return apply_filter_operation(
            column_data.typed<int>(),
            [&](const int cell) { return cell == value; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<int>(),
            [value](const int cell) { return cell > value; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<int>(),
            [&](const int cell) { return cell < value; },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<int64_t>()) {
    const int64_t value = row_filter.value_int;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        return apply_filter_operation(
            column_data.typed<int64_t>(),
            [&](const int64_t cell) { return cell == value; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<int64_t>(),
            [value](const int64_t cell) { return cell > value; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<int64_t>(),
            [&](const int64_t cell) { return cell < value; },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<int2>()) {
    const int2 value = row_filter.value_int2;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        return apply_filter_operation(
            column_data.typed<int2>(),
            [&](const int2 cell) { return cell == value; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<int2>(),
            [&](const int2 cell) { return cell.x > value.x && cell.y > value.y; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<int2>(),
            [&](const int2 cell) { return cell.x < value.x && cell.y < value.y; },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<int3>()) {
    const int3 value = row_filter.value_int3;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        return apply_filter_operation(
            column_data.typed<int3>(),
            [&](const int3 cell) { return cell == value; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<int3>(),
            [&](const int3 cell) {
              return cell.x > value.x && cell.y > value.y && cell.z > value.z;
            },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<int3>(),
            [&](const int3 cell) {
              return cell.x < value.x && cell.y < value.y && cell.z < value.z;
            },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<short2>()) {
    const short2 value = short2(int2(row_filter.value_int2));
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        return apply_filter_operation(
            column_data.typed<short2>(),
            [&](const short2 cell) { return cell == value; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<short2>(),
            [&](const short2 cell) { return cell.x > value.x && cell.y > value.y; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<short2>(),
            [&](const short2 cell) { return cell.x < value.x && cell.y < value.y; },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<float2>()) {
    const float2 value = row_filter.value_float2;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        const float threshold_sq = pow2f(row_filter.threshold);
        return apply_filter_operation(
            column_data.typed<float2>(),
            [&](const float2 cell) { return math::distance_squared(cell, value) <= threshold_sq; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<float2>(),
            [&](const float2 cell) { return cell.x > value.x && cell.y > value.y; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<float2>(),
            [&](const float2 cell) { return cell.x < value.x && cell.y < value.y; },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<float3>()) {
    const float3 value = row_filter.value_float3;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        const float threshold_sq = pow2f(row_filter.threshold);
        return apply_filter_operation(
            column_data.typed<float3>(),
            [&](const float3 cell) { return math::distance_squared(cell, value) <= threshold_sq; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<float3>(),
            [&](const float3 cell) {
              return cell.x > value.x && cell.y > value.y && cell.z > value.z;
            },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<float3>(),
            [&](const float3 cell) {
              return cell.x < value.x && cell.y < value.y && cell.z < value.z;
            },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<float4>()) {
    const float4 value = row_filter.value_float4;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        const float threshold_sq = pow2f(row_filter.threshold);
        return apply_filter_operation(
            column_data.typed<float4>(),
            [&](const float4 cell) { return math::distance_squared(cell, value) <= threshold_sq; },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<float4>(),
            [&](const float4 cell) {
              return cell.x > value.x && cell.y > value.y && cell.z > value.z && cell.w > value.w;
            },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<float4>(),
            [&](const float4 cell) {
              return cell.x < value.x && cell.y < value.y && cell.z < value.z && cell.w < value.w;
            },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<ColorGeometry4f>()) {
    const ColorGeometry4f value = row_filter.value_color;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        const float threshold_sq = pow2f(row_filter.threshold);
        return apply_filter_operation(
            column_data.typed<ColorGeometry4f>(),
            [&](const ColorGeometry4f cell) {
              return math::distance_squared(float4(cell), float4(value)) <= threshold_sq;
            },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<ColorGeometry4f>(),
            [&](const ColorGeometry4f cell) {
              return cell.r > value.r && cell.g > value.g && cell.b > value.b && cell.a > value.a;
            },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<ColorGeometry4f>(),
            [&](const ColorGeometry4f cell) {
              return cell.r < value.r && cell.g < value.g && cell.b < value.b && cell.a < value.a;
            },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<ColorGeometry4b>()) {
    const ColorGeometry4f value = row_filter.value_color;
    switch (row_filter.operation) {
      case SPREADSHEET_ROW_FILTER_EQUAL: {
        const float4 value_floats = {
            float(value.r), float(value.g), float(value.b), float(value.a)};
        const float threshold_sq = pow2f(row_filter.threshold);
        return apply_filter_operation(
            column_data.typed<ColorGeometry4b>(),
            [&](const ColorGeometry4b cell_bytes) {
              const ColorGeometry4f cell = color::decode(cell_bytes);
              const float4 cell_floats = {
                  float(cell.r), float(cell.g), float(cell.b), float(cell.a)};
              return math::distance_squared(value_floats, cell_floats) <= threshold_sq;
            },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_GREATER: {
        return apply_filter_operation(
            column_data.typed<ColorGeometry4b>(),
            [&](const ColorGeometry4b cell_bytes) {
              const ColorGeometry4f cell = color::decode(cell_bytes);
              return cell.r > value.r && cell.g > value.g && cell.b > value.b && cell.a > value.a;
            },
            prev_mask,
            memory);
      }
      case SPREADSHEET_ROW_FILTER_LESS: {
        return apply_filter_operation(
            column_data.typed<ColorGeometry4b>(),
            [&](const ColorGeometry4b cell_bytes) {
              const ColorGeometry4f cell = color::decode(cell_bytes);
              return cell.r < value.r && cell.g < value.g && cell.b < value.b && cell.a < value.a;
            },
            prev_mask,
            memory);
      }
    }
  }
  else if (column_data.type().is<bke::InstanceReference>()) {
    const StringRef value = row_filter.value_string;
    return apply_filter_operation(
        column_data.typed<bke::InstanceReference>(),
        [&](const bke::InstanceReference cell) {
          switch (cell.type()) {
            case bke::InstanceReference::Type::Object: {
              return value == (reinterpret_cast<ID &>(cell.object()).name + 2);
            }
            case bke::InstanceReference::Type::Collection: {
              return value == (reinterpret_cast<ID &>(cell.collection()).name + 2);
            }
            case bke::InstanceReference::Type::GeometrySet: {
              return value == cell.geometry_set().name();
            }
            case bke::InstanceReference::Type::None: {
              return false;
            }
          }
          BLI_assert_unreachable();
          return false;
        },
        prev_mask,
        memory);
  }
  return prev_mask;
}

static bool is_pattern_separator(const char c)
{
  return ELEM(c, ' ', '\t', ',', ';');
}

static Vector<std::string> split_filter_tokens(const StringRef pattern)
{
  Vector<std::string> tokens;
  int64_t i = 0;
  const int64_t n = pattern.size();
  while (i < n) {
    while (i < n && is_pattern_separator(pattern[i])) {
      i++;
    }
    const int64_t start = i;
    while (i < n && !is_pattern_separator(pattern[i])) {
      i++;
    }
    if (i > start) {
      tokens.append(std::string(pattern.substr(start, i - start)));
    }
  }
  return tokens;
}

static bool name_matches_glob(const StringRef name, const StringRef glob)
{
  if (name.is_empty() || glob.is_empty()) {
    return false;
  }
  const std::string name_str(name);
  const std::string glob_str(glob);
  return fnmatch(glob_str.c_str(), name_str.c_str(), 0) == 0;
}

static bool column_matches_glob(const StringRef id_name,
                                const StringRef display_name,
                                const StringRef glob)
{
  return name_matches_glob(id_name, glob) ||
         (!display_name.is_empty() && display_name != id_name &&
          name_matches_glob(display_name, glob));
}

bool spreadsheet_attribute_name_matches_pattern(const StringRef name, const StringRef pattern)
{
  if (name.is_empty() || pattern.is_empty()) {
    return false;
  }
  for (const std::string &token : split_filter_tokens(pattern)) {
    StringRef glob = token;
    if (!glob.is_empty() && (glob[0] == '^' || glob[0] == '!')) {
      glob = glob.drop_prefix(1);
    }
    if (name_matches_glob(name, glob)) {
      return true;
    }
  }
  return false;
}

bool spreadsheet_attribute_is_visible(const SpaceSpreadsheet &sspreadsheet,
                                      const StringRef id_name,
                                      const StringRef display_name)
{
  const StringRef pattern = sspreadsheet.attribute_pattern;
  if (pattern.is_empty()) {
    return true;
  }

  bool any_include = false;
  bool include_match = false;
  bool exclude_match = false;
  for (const std::string &raw : split_filter_tokens(pattern)) {
    StringRef token = raw;
    if (token.is_empty()) {
      continue;
    }
    const bool exclude = token[0] == '^' || token[0] == '!';
    if (exclude) {
      token = token.drop_prefix(1);
      if (token.is_empty()) {
        continue;
      }
    }
    const bool matches = column_matches_glob(id_name, display_name, token);
    if (exclude) {
      exclude_match |= matches;
    }
    else {
      any_include = true;
      include_match |= matches;
    }
  }

  /* No include tokens: start from all columns, then subtract excludes (`^uv*`).
   * Include tokens: only those names, then subtract excludes
   * (`position normal ^normal`). */
  if (!any_include) {
    return !exclude_match;
  }
  return include_match && !exclude_match;
}

enum class RowPatternOp {
  Truthy,
  Eq,
  Ne,
  Gt,
  Lt,
  Ge,
  Le,
};

enum class RowPatternKind {
  All,
  Indices,
  Expr,
};

struct RowPatternToken {
  bool exclude = false;
  RowPatternKind kind = RowPatternKind::Expr;
  int64_t start = 0;
  int64_t end = 0; /* Inclusive. -1 means last domain index. */
  int64_t step = 1;
  std::string attr;
  int component = -1;
  RowPatternOp op = RowPatternOp::Truthy;
  double value = 0.0;
  bool value_is_bool = false;
  bool value_bool = false;
  std::string value_str;
};

static bool parse_nonnegative_int(StringRef &s, int64_t &r_value)
{
  if (s.is_empty() || s[0] < '0' || s[0] > '9') {
    return false;
  }
  int64_t value = 0;
  int64_t i = 0;
  while (i < s.size() && s[i] >= '0' && s[i] <= '9') {
    value = value * 10 + (s[i] - '0');
    i++;
  }
  s = s.drop_prefix(i);
  r_value = value;
  return true;
}

static int component_from_suffix(const StringRef suffix)
{
  if (ELEM(suffix, "x", "r", "0")) {
    return 0;
  }
  if (ELEM(suffix, "y", "g", "1")) {
    return 1;
  }
  if (ELEM(suffix, "z", "b", "2")) {
    return 2;
  }
  if (ELEM(suffix, "w", "a", "3")) {
    return 3;
  }
  return -1;
}

static bool parse_index_token(StringRef s, RowPatternToken &tok)
{
  int64_t start = 0;
  if (!parse_nonnegative_int(s, start)) {
    return false;
  }
  tok.kind = RowPatternKind::Indices;
  tok.start = start;
  tok.end = start;
  tok.step = 1;
  bool had_range = false;
  if (!s.is_empty() && s[0] == '-') {
    s = s.drop_prefix(1);
    int64_t end = 0;
    if (!parse_nonnegative_int(s, end)) {
      return false;
    }
    tok.end = end;
    had_range = true;
  }
  if (!s.is_empty() && s[0] == ':') {
    s = s.drop_prefix(1);
    int64_t step = 0;
    if (!parse_nonnegative_int(s, step) || step <= 0) {
      return false;
    }
    tok.step = step;
    if (!had_range) {
      tok.end = -1;
    }
  }
  return s.is_empty();
}

static bool parse_expr_token(StringRef s, RowPatternToken &tok)
{
  if (!s.is_empty() && s[0] == '@') {
    s = s.drop_prefix(1);
  }
  int64_t name_len = 0;
  while (name_len < s.size()) {
    const char c = s[name_len];
    if (ELEM(c, '=', '!', '>', '<')) {
      break;
    }
    name_len++;
  }
  if (name_len == 0) {
    return false;
  }
  StringRef name = s.substr(0, name_len);
  s = s.drop_prefix(name_len);

  tok.component = -1;
  const int64_t dot = name.rfind('.');
  if (dot > 0) {
    const int component = component_from_suffix(name.substr(dot + 1, name.size() - dot - 1));
    if (component >= 0) {
      tok.component = component;
      name = name.substr(0, dot);
    }
  }
  tok.attr = std::string(name);
  tok.kind = RowPatternKind::Expr;
  tok.op = RowPatternOp::Truthy;

  if (s.is_empty()) {
    return !tok.attr.empty();
  }

  if (s.startswith("==")) {
    tok.op = RowPatternOp::Eq;
    s = s.drop_prefix(2);
  }
  else if (s.startswith("!=")) {
    tok.op = RowPatternOp::Ne;
    s = s.drop_prefix(2);
  }
  else if (s.startswith(">=")) {
    tok.op = RowPatternOp::Ge;
    s = s.drop_prefix(2);
  }
  else if (s.startswith("<=")) {
    tok.op = RowPatternOp::Le;
    s = s.drop_prefix(2);
  }
  else if (s.startswith("=")) {
    tok.op = RowPatternOp::Eq;
    s = s.drop_prefix(1);
  }
  else if (s.startswith(">")) {
    tok.op = RowPatternOp::Gt;
    s = s.drop_prefix(1);
  }
  else if (s.startswith("<")) {
    tok.op = RowPatternOp::Lt;
    s = s.drop_prefix(1);
  }
  else {
    return false;
  }

  if (s.is_empty()) {
    return false;
  }

  if (ELEM(s, "true", "True", "TRUE")) {
    tok.value_is_bool = true;
    tok.value_bool = true;
    tok.value = 1.0;
    return true;
  }
  if (ELEM(s, "false", "False", "FALSE")) {
    tok.value_is_bool = true;
    tok.value_bool = false;
    tok.value = 0.0;
    return true;
  }

  const std::string value_storage(s);
  char *end = nullptr;
  const double parsed = std::strtod(value_storage.c_str(), &end);
  if (end != value_storage.c_str() && end == value_storage.c_str() + value_storage.size()) {
    tok.value = parsed;
    return true;
  }
  tok.value_str = value_storage;
  tok.op = (tok.op == RowPatternOp::Ne) ? RowPatternOp::Ne : RowPatternOp::Eq;
  return true;
}

static bool parse_row_pattern_token(StringRef s, RowPatternToken &tok)
{
  tok = {};
  if (s.is_empty()) {
    return false;
  }
  if (s[0] == '^' || s[0] == '!') {
    tok.exclude = true;
    s = s.drop_prefix(1);
  }
  if (s == "*") {
    tok.kind = RowPatternKind::All;
    return true;
  }
  if (parse_index_token(s, tok)) {
    return true;
  }
  return parse_expr_token(s, tok);
}

static bool eval_numeric(const double cell, const RowPatternOp op, const double value)
{
  switch (op) {
    case RowPatternOp::Truthy:
      return cell != 0.0;
    case RowPatternOp::Eq:
      return std::abs(cell - value) < 1e-6;
    case RowPatternOp::Ne:
      return std::abs(cell - value) >= 1e-6;
    case RowPatternOp::Gt:
      return cell > value;
    case RowPatternOp::Lt:
      return cell < value;
    case RowPatternOp::Ge:
      return cell >= value;
    case RowPatternOp::Le:
      return cell <= value;
  }
  return false;
}

static bool eval_bool_cell(const bool cell, const RowPatternToken &tok)
{
  const bool rhs = tok.value_is_bool ? tok.value_bool : (tok.value != 0.0);
  switch (tok.op) {
    case RowPatternOp::Truthy:
      return cell;
    case RowPatternOp::Eq:
      return cell == rhs;
    case RowPatternOp::Ne:
      return cell != rhs;
    case RowPatternOp::Gt:
    case RowPatternOp::Ge:
      return cell && !rhs;
    case RowPatternOp::Lt:
    case RowPatternOp::Le:
      return !cell && rhs;
  }
  return false;
}

template<typename T, int Size>
static bool eval_vector_cell(const VecBase<T, Size> &cell, const RowPatternToken &tok)
{
  if (tok.component >= 0) {
    if (tok.component >= Size) {
      return false;
    }
    return eval_numeric(double(cell[tok.component]), tok.op, tok.value);
  }
  if (tok.op == RowPatternOp::Truthy) {
    for (int i = 0; i < Size; i++) {
      if (cell[i] != T(0)) {
        return true;
      }
    }
    return false;
  }
  for (int i = 0; i < Size; i++) {
    if (!eval_numeric(double(cell[i]), tok.op, tok.value)) {
      return false;
    }
  }
  return true;
}

static bool eval_expr_on_row(const ColumnValues &column,
                             const int64_t index,
                             const RowPatternToken &tok)
{
  const GVArray &data = column.data();
  if (index < 0 || index >= data.size()) {
    return false;
  }
  if (data.type().is<bool>()) {
    return eval_bool_cell(data.typed<bool>()[index], tok);
  }
  if (data.type().is<int8_t>()) {
    return eval_numeric(double(data.typed<int8_t>()[index]), tok.op, tok.value);
  }
  if (data.type().is<int>()) {
    return eval_numeric(double(data.typed<int>()[index]), tok.op, tok.value);
  }
  if (data.type().is<int64_t>()) {
    return eval_numeric(double(data.typed<int64_t>()[index]), tok.op, tok.value);
  }
  if (data.type().is<float>()) {
    return eval_numeric(double(data.typed<float>()[index]), tok.op, tok.value);
  }
  if (data.type().is<int2>()) {
    return eval_vector_cell(data.typed<int2>()[index], tok);
  }
  if (data.type().is<int3>()) {
    return eval_vector_cell(data.typed<int3>()[index], tok);
  }
  if (data.type().is<short2>()) {
    return eval_vector_cell(data.typed<short2>()[index], tok);
  }
  if (data.type().is<float2>()) {
    return eval_vector_cell(data.typed<float2>()[index], tok);
  }
  if (data.type().is<float3>()) {
    return eval_vector_cell(data.typed<float3>()[index], tok);
  }
  if (data.type().is<float4>()) {
    return eval_vector_cell(data.typed<float4>()[index], tok);
  }
  if (data.type().is<ColorGeometry4f>()) {
    const ColorGeometry4f cell = data.typed<ColorGeometry4f>()[index];
    return eval_vector_cell(float4(cell.r, cell.g, cell.b, cell.a), tok);
  }
  if (data.type().is<ColorGeometry4b>()) {
    const ColorGeometry4f cell = color::decode(data.typed<ColorGeometry4b>()[index]);
    return eval_vector_cell(float4(cell.r, cell.g, cell.b, cell.a), tok);
  }
  if (data.type().is<std::string>()) {
    const std::string &cell = data.typed<std::string>()[index];
    switch (tok.op) {
      case RowPatternOp::Truthy:
        return !cell.empty();
      case RowPatternOp::Eq:
        return cell == tok.value_str;
      case RowPatternOp::Ne:
        return cell != tok.value_str;
      default:
        return false;
    }
  }
  return false;
}

static const ColumnValues *lookup_column_for_pattern(
    Map<StringRef, const ColumnValues *> &columns,
    const DataSource &data_source,
    const StringRef name,
    ResourceScope &scope)
{
  if (const ColumnValues *const *found = columns.lookup_ptr(name)) {
    return *found;
  }
  SpreadsheetColumnID column_id;
  std::string name_storage(name);
  column_id.name = name_storage.data();
  std::unique_ptr<ColumnValues> values = data_source.get_column_values(column_id);
  if (!values) {
    return nullptr;
  }
  const ColumnValues *ptr = scope.add(std::move(values));
  columns.add(ptr->name(), ptr);
  return ptr;
}

static void apply_token_to_bits(const RowPatternToken &tok,
                                const int tot_rows,
                                Map<StringRef, const ColumnValues *> &columns,
                                const DataSource &data_source,
                                ResourceScope &scope,
                                const IndexMask &universe,
                                BitVector<> &bits)
{
  if (tok.kind == RowPatternKind::All) {
    universe.foreach_index([&](const int64_t i) { bits[i].set(); });
    return;
  }
  if (tok.kind == RowPatternKind::Indices) {
    int64_t start = tok.start;
    int64_t end = tok.end < 0 ? tot_rows - 1 : tok.end;
    if (start > end) {
      std::swap(start, end);
    }
    start = std::max<int64_t>(start, 0);
    end = std::min<int64_t>(end, tot_rows - 1);
    if (start > end || tok.step <= 0) {
      return;
    }
    for (int64_t i = start; i <= end; i += tok.step) {
      bits[i].set();
    }
    return;
  }

  const ColumnValues *column = lookup_column_for_pattern(columns, data_source, tok.attr, scope);
  if (!column) {
    return;
  }
  universe.foreach_index([&](const int64_t i) {
    if (eval_expr_on_row(*column, i, tok)) {
      bits[i].set();
    }
  });
}

static IndexMask apply_row_pattern_clause(const StringRef clause,
                                          Map<StringRef, const ColumnValues *> &columns,
                                          const DataSource &data_source,
                                          const IndexMask &prev_mask,
                                          ResourceScope &scope)
{
  const Vector<std::string> raw_tokens = split_filter_tokens(clause);
  if (raw_tokens.is_empty()) {
    return prev_mask;
  }

  Vector<RowPatternToken> tokens;
  for (const std::string &raw : raw_tokens) {
    RowPatternToken tok;
    if (parse_row_pattern_token(raw, tok)) {
      tokens.append(std::move(tok));
    }
  }
  if (tokens.is_empty()) {
    return prev_mask;
  }

  const int tot_rows = data_source.tot_rows();
  if (tot_rows <= 0) {
    return {};
  }

  BitVector<> include_bits(tot_rows, false);
  BitVector<> exclude_bits(tot_rows, false);
  bool any_include = false;
  for (const RowPatternToken &tok : tokens) {
    apply_token_to_bits(tok,
                        tot_rows,
                        columns,
                        data_source,
                        scope,
                        prev_mask,
                        tok.exclude ? exclude_bits : include_bits);
    if (!tok.exclude) {
      any_include = true;
    }
  }

  BitVector<> result_bits(tot_rows, false);
  if (!any_include) {
    prev_mask.foreach_index([&](const int64_t i) {
      if (!exclude_bits[i]) {
        result_bits[i].set();
      }
    });
  }
  else {
    prev_mask.foreach_index([&](const int64_t i) {
      if (include_bits[i] && !exclude_bits[i]) {
        result_bits[i].set();
      }
    });
  }

  return IndexMask::from_bits(prev_mask, result_bits, scope.allocator());
}

static IndexMask apply_row_pattern(const SpaceSpreadsheet &sspreadsheet,
                                   Map<StringRef, const ColumnValues *> &columns,
                                   const DataSource &data_source,
                                   const IndexMask &prev_mask,
                                   ResourceScope &scope)
{
  const StringRef pattern = sspreadsheet.row_pattern;
  if (pattern.is_empty()) {
    return prev_mask;
  }

  /* `&` ANDs clauses: `0-10 & @select`. Spaces/commas inside a clause OR together,
   * `^` / `!` subtract from that clause. */
  IndexMask mask = prev_mask;
  int64_t start = 0;
  const int64_t n = pattern.size();
  for (int64_t i = 0; i <= n; i++) {
    if (i == n || pattern[i] == '&') {
      const StringRef clause = pattern.substr(start, i - start);
      if (!split_filter_tokens(clause).is_empty()) {
        mask = apply_row_pattern_clause(clause, columns, data_source, mask, scope);
      }
      start = i + 1;
    }
  }
  return mask;
}

static bool use_row_filters(const SpaceSpreadsheet &sspreadsheet)
{
  if (!(sspreadsheet.filter_flag & SPREADSHEET_FILTER_ENABLE)) {
    return false;
  }
  if (sspreadsheet.row_filters.is_empty()) {
    return false;
  }
  return true;
}

static bool use_selection_filter(const SpaceSpreadsheet &sspreadsheet,
                                 const DataSource &data_source)
{
  if (!(sspreadsheet.filter_flag & SPREADSHEET_FILTER_SELECTED_ONLY)) {
    return false;
  }
  if (!data_source.has_selection_filter()) {
    return false;
  }
  return true;
}

IndexMask spreadsheet_filter_rows(const SpaceSpreadsheet &sspreadsheet,
                                  const SpreadsheetLayout &spreadsheet_layout,
                                  const DataSource &data_source,
                                  ResourceScope &scope)
{
  const int tot_rows = data_source.tot_rows();

  const bool use_selection = use_selection_filter(sspreadsheet, data_source);
  const bool use_filters = use_row_filters(sspreadsheet);
  const bool use_pattern = sspreadsheet.row_pattern[0] != '\0';

  /* Avoid allocating an array if no row filtering is necessary. */
  if (!(use_filters || use_selection || use_pattern)) {
    return IndexMask(tot_rows);
  }

  IndexMask mask(tot_rows);

  if (use_selection) {
    const GeometryDataSource *geometry_data_source = dynamic_cast<const GeometryDataSource *>(
        &data_source);
    mask = geometry_data_source->apply_selection_filter(scope.allocator());
  }

  Map<StringRef, const ColumnValues *> columns;
  if (use_filters || use_pattern) {
    for (const ColumnLayout &column : spreadsheet_layout.columns) {
      columns.add(column.values->name(), column.values);
    }
  }

  if (use_filters) {
    for (const SpreadsheetRowFilter &row_filter : sspreadsheet.row_filters) {
      if (row_filter.flag & SPREADSHEET_ROW_FILTER_ENABLED) {
        if (!columns.contains(row_filter.column_name)) {
          continue;
        }
        mask = apply_row_filter(row_filter, columns, mask, scope.allocator());
      }
    }
  }

  if (use_pattern) {
    mask = apply_row_pattern(sspreadsheet, columns, data_source, mask, scope);
  }

  return mask;
}

SpreadsheetRowFilter *spreadsheet_row_filter_new()
{
  SpreadsheetRowFilter *row_filter = MEM_new<SpreadsheetRowFilter>(__func__);
  row_filter->flag = (SPREADSHEET_ROW_FILTER_UI_EXPAND | SPREADSHEET_ROW_FILTER_ENABLED);
  row_filter->operation = SPREADSHEET_ROW_FILTER_LESS;
  row_filter->threshold = 0.01f;
  row_filter->column_name[0] = '\0';

  return row_filter;
}

SpreadsheetRowFilter *spreadsheet_row_filter_copy(const SpreadsheetRowFilter *src_row_filter)
{
  SpreadsheetRowFilter *new_filter = spreadsheet_row_filter_new();

  memcpy(new_filter, src_row_filter, sizeof(SpreadsheetRowFilter));
  new_filter->next = nullptr;
  new_filter->prev = nullptr;

  return new_filter;
}

void spreadsheet_row_filter_free(SpreadsheetRowFilter *row_filter)
{
  MEM_SAFE_DELETE(row_filter->value_string);
  MEM_delete(row_filter);
}

}  // namespace blender::ed::spreadsheet
