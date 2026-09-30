/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "BLI_array.hh"
#include "BLI_color.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_string_ref.hh"

#include "BKE_instances.hh"

#include "DNA_meshdata_types.h"
#include "DNA_space_types.h"

#include "spreadsheet_sort.hh"

namespace blender::ed::spreadsheet {

/**
 * Scalar compare for sort keys.
 * \return -1 if a < b, 1 if a > b, 0 if equal.
 * NaNs sort after all finite/infinite values (stable relative among NaNs).
 */
template<typename T>
static int compare_scalar(const T a, const T b)
{
  if constexpr (std::is_floating_point_v<T>) {
    const bool a_nan = std::isnan(a);
    const bool b_nan = std::isnan(b);
    if (a_nan || b_nan) {
      if (a_nan == b_nan) {
        return 0;
      }
      return a_nan ? 1 : -1;
    }
  }
  if (a < b) {
    return -1;
  }
  if (b < a) {
    return 1;
  }
  return 0;
}

/**
 * Lexicographic (recursive) multi-component compare: first unequal component decides.
 * Used for vectors, colors, quaternions, and flattened matrices.
 */
template<typename T>
static int compare_components(const Span<T> a, const Span<T> b)
{
  BLI_assert(a.size() == b.size());
  for (const int64_t i : a.index_range()) {
    if (const int c = compare_scalar(a[i], b[i])) {
      return c;
    }
  }
  return 0;
}

template<typename T, int Size>
static int compare_vec(const VecBase<T, Size> &a, const VecBase<T, Size> &b)
{
  /* Contiguous x,y[,z[,w]] — recursive component-wise order. */
  return compare_components(Span(&a.x, Size), Span(&b.x, Size));
}

static int compare_quaternion(const math::Quaternion &a, const math::Quaternion &b)
{
  /* Same component order as spreadsheet cell display (float4 cast is w,x,y,z). */
  const float af[4] = {a.w, a.x, a.y, a.z};
  const float bf[4] = {b.w, b.x, b.y, b.z};
  return compare_components(Span(af, 4), Span(bf, 4));
}

static int compare_float4x4(const float4x4 &a, const float4x4 &b)
{
  /* Column-major 16 floats; compare as a flat float sequence. */
  return compare_components(Span(a.base_ptr(), 16), Span(b.base_ptr(), 16));
}

static int compare_color4f(const ColorGeometry4f &a, const ColorGeometry4f &b)
{
  return compare_components(Span(&a.r, 4), Span(&b.r, 4));
}

static int compare_color4b(const ColorGeometry4b &a, const ColorGeometry4b &b)
{
  return compare_components(Span(&a.r, 4), Span(&b.r, 4));
}

/** Alphabetical (lexicographic) string compare. */
static int compare_alphabetical(const StringRef a, const StringRef b)
{
  const int64_t n = std::min(a.size(), b.size());
  if (n > 0) {
    const int cmp = std::memcmp(a.data(), b.data(), size_t(n));
    if (cmp < 0) {
      return -1;
    }
    if (cmp > 0) {
      return 1;
    }
  }
  return compare_scalar(a.size(), b.size());
}

static int compare_mstring(const MStringProperty &a, const MStringProperty &b)
{
  /* Explicit length; not necessarily null-terminated. Alphabetical byte order. */
  return compare_alphabetical(StringRef(a.s, uint8_t(a.s_len)), StringRef(b.s, uint8_t(b.s_len)));
}

static int compare_instance(const bke::InstanceReference &a, const bke::InstanceReference &b)
{
  if (const int c = compare_scalar(int(a.type()), int(b.type()))) {
    return c;
  }
  return compare_alphabetical(a.name(), b.name());
}

template<typename T, typename CompareFn>
static void stable_sort_by_varray(MutableSpan<int> indices,
                                  const VArray<T> &data,
                                  const bool ascending,
                                  CompareFn compare_fn)
{
  std::stable_sort(indices.begin(), indices.end(), [&](const int ia, const int ib) {
    const int cmp = compare_fn(data[ia], data[ib]);
    return ascending ? (cmp < 0) : (cmp > 0);
  });
}

template<typename T>
static void stable_sort_by_varray(MutableSpan<int> indices,
                                  const VArray<T> &data,
                                  const bool ascending)
{
  stable_sort_by_varray(indices, data, ascending, [](const T &a, const T &b) {
    return compare_scalar(a, b);
  });
}

static bool sort_indices_by_column(MutableSpan<int> indices,
                                   const ColumnValues &column,
                                   const bool ascending)
{
  const GVArray &data = column.data();
  const CPPType &type = data.type();

  if (type.is<bool>()) {
    stable_sort_by_varray(indices, data.typed<bool>(), ascending);
    return true;
  }
  if (type.is<int8_t>()) {
    stable_sort_by_varray(indices, data.typed<int8_t>(), ascending);
    return true;
  }
  if (type.is<int>()) {
    stable_sort_by_varray(indices, data.typed<int>(), ascending);
    return true;
  }
  if (type.is<int64_t>()) {
    stable_sort_by_varray(indices, data.typed<int64_t>(), ascending);
    return true;
  }
  if (type.is<float>()) {
    stable_sort_by_varray(indices, data.typed<float>(), ascending);
    return true;
  }

  /* Vectors: component-wise recursive (x, then y, then z, then w). */
  if (type.is<short2>()) {
    stable_sort_by_varray(
        indices, data.typed<short2>(), ascending, [](const short2 &a, const short2 &b) {
          return compare_vec(a, b);
        });
    return true;
  }
  if (type.is<int2>()) {
    stable_sort_by_varray(indices, data.typed<int2>(), ascending, [](const int2 &a, const int2 &b) {
      return compare_vec(a, b);
    });
    return true;
  }
  if (type.is<int3>()) {
    stable_sort_by_varray(indices, data.typed<int3>(), ascending, [](const int3 &a, const int3 &b) {
      return compare_vec(a, b);
    });
    return true;
  }
  if (type.is<float2>()) {
    stable_sort_by_varray(
        indices, data.typed<float2>(), ascending, [](const float2 &a, const float2 &b) {
          return compare_vec(a, b);
        });
    return true;
  }
  if (type.is<float3>()) {
    stable_sort_by_varray(
        indices, data.typed<float3>(), ascending, [](const float3 &a, const float3 &b) {
          return compare_vec(a, b);
        });
    return true;
  }
  if (type.is<float4>()) {
    stable_sort_by_varray(
        indices, data.typed<float4>(), ascending, [](const float4 &a, const float4 &b) {
          return compare_vec(a, b);
        });
    return true;
  }

  if (type.is<ColorGeometry4f>()) {
    stable_sort_by_varray(indices,
                          data.typed<ColorGeometry4f>(),
                          ascending,
                          [](const ColorGeometry4f &a, const ColorGeometry4f &b) {
                            return compare_color4f(a, b);
                          });
    return true;
  }
  if (type.is<ColorGeometry4b>()) {
    stable_sort_by_varray(indices,
                          data.typed<ColorGeometry4b>(),
                          ascending,
                          [](const ColorGeometry4b &a, const ColorGeometry4b &b) {
                            return compare_color4b(a, b);
                          });
    return true;
  }

  /* Strings: alphabetical lexicographic order. */
  if (type.is<std::string>()) {
    stable_sort_by_varray(
        indices, data.typed<std::string>(), ascending, [](const std::string &a, const std::string &b) {
          return compare_alphabetical(a, b);
        });
    return true;
  }
  if (type.is<MStringProperty>()) {
    stable_sort_by_varray(indices,
                          data.typed<MStringProperty>(),
                          ascending,
                          [](const MStringProperty &a, const MStringProperty &b) {
                            return compare_mstring(a, b);
                          });
    return true;
  }

  /* Quaternion / matrix: flatten to floats, compare component by component. */
  if (type.is<math::Quaternion>()) {
    stable_sort_by_varray(indices,
                          data.typed<math::Quaternion>(),
                          ascending,
                          [](const math::Quaternion &a, const math::Quaternion &b) {
                            return compare_quaternion(a, b);
                          });
    return true;
  }
  if (type.is<float4x4>()) {
    stable_sort_by_varray(
        indices, data.typed<float4x4>(), ascending, [](const float4x4 &a, const float4x4 &b) {
          return compare_float4x4(a, b);
        });
    return true;
  }

  if (type.is<bke::InstanceReference>()) {
    stable_sort_by_varray(indices,
                          data.typed<bke::InstanceReference>(),
                          ascending,
                          [](const bke::InstanceReference &a, const bke::InstanceReference &b) {
                            return compare_instance(a, b);
                          });
    return true;
  }

  /* Unsupported type: leave order unchanged. */
  return false;
}

Span<int> spreadsheet_sort_rows(const SpreadsheetTable &table,
                                const SpreadsheetLayout &spreadsheet_layout,
                                const IndexMask &filtered_rows,
                                ResourceScope &scope)
{
  const eSpreadsheetSortDirection direction = eSpreadsheetSortDirection(table.sort_direction);
  if (direction == SPREADSHEET_SORT_NONE || filtered_rows.is_empty()) {
    return {};
  }

  const bool ascending = direction == SPREADSHEET_SORT_ASC;
  Array<int> &indices = scope.construct<Array<int>>(filtered_rows.size());
  filtered_rows.to_indices<int>(indices.as_mutable_span());

  const StringRef sort_column_name = table.sort_column_name;
  if (sort_column_name.is_empty()) {
    /* Sort by domain index: filtered IndexMask is already ascending. */
    if (!ascending) {
      std::reverse(indices.begin(), indices.end());
    }
    return indices.as_span();
  }

  const ColumnValues *sort_column = nullptr;
  for (const ColumnLayout &column : spreadsheet_layout.columns) {
    if (column.values->name() == sort_column_name) {
      sort_column = column.values;
      break;
    }
  }
  if (!sort_column) {
    /* Column missing: fall back to filtered order. */
    return {};
  }

  if (!sort_indices_by_column(indices, *sort_column, ascending)) {
    return {};
  }
  return indices.as_span();
}

}  // namespace blender::ed::spreadsheet
