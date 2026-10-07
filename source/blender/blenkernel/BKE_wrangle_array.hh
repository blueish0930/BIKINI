/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 *
 * Packed per-element list used by Geometry Wrangle (`i[]@`, `f[]@`, `v[]@`).
 * Stored as a trivially copyable attribute so AttributeStorage blend I/O stays a flat array.
 * Not exposed on Store Named Attribute / Capture Attribute.
 */

#include <algorithm>
#include <cstring>
#include <string>
#include <type_traits>

#include "BLI_span.hh"
#include "BLI_string_ref.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"

namespace blender::bke {

enum class WrangleArrayKind : uint8_t {
  Int = 0,
  Float = 1,
  Float3 = 2,
  String = 3,
  Matrix = 4,
  /** Single 2x2 matrix (4 floats). Wrangle-only scalar `2@` storage. */
  Matrix2 = 5,
  /** Single 3x3 matrix (9 floats). Wrangle-only scalar `3@` storage. */
  Matrix3 = 6,
  /** Packed ray hits: 8 floats each (face, pos.xyz, n.xyz, dist). */
  Ray = 7,
  /** 2D vectors, the items of `u[]@name` and `vector2 xs[]`. */
  Float2 = 8,
  /** 4D vectors, the items of `q[]@name`, `c[]@name` and `r[]@name`. */
  Float4 = 9,
};

struct WrangleArrayValue {
  static constexpr int max_values = 120;

  uint16_t count = 0;
  uint16_t total = 0;
  WrangleArrayKind kind = WrangleArrayKind::Int;
  uint8_t truncated = 0;
  uint8_t _pad[2] = {};
  union {
    int i[max_values];
    float f[max_values];
  } d{};

  friend bool operator==(const WrangleArrayValue &a, const WrangleArrayValue &b)
  {
    return std::memcmp(&a, &b, sizeof(WrangleArrayValue)) == 0;
  }
  friend bool operator!=(const WrangleArrayValue &a, const WrangleArrayValue &b)
  {
    return !(a == b);
  }

  void clear(const WrangleArrayKind k = WrangleArrayKind::Int)
  {
    *this = {};
    kind = k;
  }

  void set_ints(const Span<int> src)
  {
    clear(WrangleArrayKind::Int);
    total = uint16_t(std::min<int64_t>(src.size(), 65535));
    count = uint16_t(std::min<int64_t>(src.size(), max_values));
    truncated = src.size() > max_values;
    if (count > 0) {
      memcpy(d.i, src.data(), sizeof(int) * count);
    }
  }

  void set_floats(const Span<float> src)
  {
    clear(WrangleArrayKind::Float);
    total = uint16_t(std::min<int64_t>(src.size(), 65535));
    count = uint16_t(std::min<int64_t>(src.size(), max_values));
    truncated = src.size() > max_values;
    if (count > 0) {
      memcpy(d.f, src.data(), sizeof(float) * count);
    }
  }

  void set_float3s(const Span<float3> src)
  {
    clear(WrangleArrayKind::Float3);
    const int max_n = max_values / 3;
    total = uint16_t(std::min<int64_t>(src.size(), 65535));
    count = uint16_t(std::min<int64_t>(src.size(), max_n));
    truncated = src.size() > max_n;
    for (int i = 0; i < count; i++) {
      d.f[i * 3 + 0] = src[i].x;
      d.f[i * 3 + 1] = src[i].y;
      d.f[i * 3 + 2] = src[i].z;
    }
  }

  /** The Wrangle keeps the items of a 2D vector array as 3D vectors, Z is dropped here. */
  void set_float2s(const Span<float3> src)
  {
    clear(WrangleArrayKind::Float2);
    const int max_n = max_values / 2;
    total = uint16_t(std::min<int64_t>(src.size(), 65535));
    count = uint16_t(std::min<int64_t>(src.size(), max_n));
    truncated = src.size() > max_n;
    for (int i = 0; i < count; i++) {
      d.f[i * 2 + 0] = src[i].x;
      d.f[i * 2 + 1] = src[i].y;
    }
  }

  void set_float4s(const Span<float4> src)
  {
    clear(WrangleArrayKind::Float4);
    const int max_n = max_values / 4;
    total = uint16_t(std::min<int64_t>(src.size(), 65535));
    count = uint16_t(std::min<int64_t>(src.size(), max_n));
    truncated = src.size() > max_n;
    for (int i = 0; i < count; i++) {
      d.f[i * 4 + 0] = src[i].x;
      d.f[i * 4 + 1] = src[i].y;
      d.f[i * 4 + 2] = src[i].z;
      d.f[i * 4 + 3] = src[i].w;
    }
  }

  static constexpr int matrix_floats = 16;
  static constexpr int ray_floats = 8;

  void set_matrices(const Span<float4x4> src)
  {
    clear(WrangleArrayKind::Matrix);
    const int max_n = max_values / matrix_floats;
    total = uint16_t(std::min<int64_t>(src.size(), 65535));
    count = uint16_t(std::min<int64_t>(src.size(), max_n));
    truncated = src.size() > max_n;
    for (int i = 0; i < count; i++) {
      memcpy(d.f + i * matrix_floats, &src[i][0][0], sizeof(float) * matrix_floats);
    }
  }

  float4x4 matrix_at(const int index) const
  {
    if (kind != WrangleArrayKind::Matrix || index < 0 || index >= int(count)) {
      return float4x4::identity();
    }
    return float4x4(d.f + index * matrix_floats);
  }

  static constexpr int matrix2_floats = 4;
  static constexpr int matrix3_floats = 9;

  void set_matrix2(const float2x2 &src)
  {
    clear(WrangleArrayKind::Matrix2);
    count = 1;
    total = 1;
    memcpy(d.f, &src[0][0], sizeof(float) * matrix2_floats);
  }

  float2x2 as_matrix2() const
  {
    if (kind != WrangleArrayKind::Matrix2 || count < 1) {
      return float2x2::identity();
    }
    return float2x2(d.f);
  }

  void set_matrix3(const float3x3 &src)
  {
    clear(WrangleArrayKind::Matrix3);
    count = 1;
    total = 1;
    memcpy(d.f, &src[0][0], sizeof(float) * matrix3_floats);
  }

  float3x3 as_matrix3() const
  {
    if (kind != WrangleArrayKind::Matrix3 || count < 1) {
      return float3x3::identity();
    }
    return float3x3(d.f);
  }

  /**
   * `2[]@name` / `3[]@name`: several small matrices, stored like the single one above. The
   * Wrangle keeps the items of a matrix array as 4x4 matrices, their upper left part is stored.
   */
  void set_matrix2s(const Span<float4x4> src)
  {
    clear(WrangleArrayKind::Matrix2);
    const int max_n = max_values / matrix2_floats;
    total = uint16_t(std::min<int64_t>(src.size(), 65535));
    count = uint16_t(std::min<int64_t>(src.size(), max_n));
    truncated = src.size() > max_n;
    for (int i = 0; i < count; i++) {
      for (int col = 0; col < 2; col++) {
        for (int row = 0; row < 2; row++) {
          d.f[i * matrix2_floats + col * 2 + row] = src[i][col][row];
        }
      }
    }
  }

  void set_matrix3s(const Span<float4x4> src)
  {
    clear(WrangleArrayKind::Matrix3);
    const int max_n = max_values / matrix3_floats;
    total = uint16_t(std::min<int64_t>(src.size(), 65535));
    count = uint16_t(std::min<int64_t>(src.size(), max_n));
    truncated = src.size() > max_n;
    for (int i = 0; i < count; i++) {
      for (int col = 0; col < 3; col++) {
        for (int row = 0; row < 3; row++) {
          d.f[i * matrix3_floats + col * 3 + row] = src[i][col][row];
        }
      }
    }
  }

  /** Item of a `2[]@` / `3[]@` array as a 4x4 matrix, identity outside of the stored part. */
  float4x4 small_matrix_at(const int index) const
  {
    float4x4 result = float4x4::identity();
    const int dim = (kind == WrangleArrayKind::Matrix2) ? 2 :
                    (kind == WrangleArrayKind::Matrix3) ? 3 :
                                                          0;
    if (dim == 0 || index < 0 || index >= int(count)) {
      return result;
    }
    for (int col = 0; col < dim; col++) {
      for (int row = 0; row < dim; row++) {
        result[col][row] = d.f[index * dim * dim + col * dim + row];
      }
    }
    return result;
  }

  void set_rays(const int n, const float *src)
  {
    clear(WrangleArrayKind::Ray);
    const int max_n = max_values / ray_floats;
    total = uint16_t(std::min(std::max(n, 0), 65535));
    count = uint16_t(std::min(std::max(n, 0), max_n));
    truncated = n > max_n;
    if (count > 0 && src != nullptr) {
      memcpy(d.f, src, sizeof(float) * count * ray_floats);
    }
  }

  static constexpr int string_buf_size = max_values * int(sizeof(int));

  void set_strings(const Span<std::string> src)
  {
    clear(WrangleArrayKind::String);
    total = uint16_t(std::min<int64_t>(src.size(), 65535));
    uint8_t *buf = reinterpret_cast<uint8_t *>(d.i);
    int used = 0;
    int n = 0;
    for (const std::string &s : src) {
      const int sl = std::min(int(s.size()), 255);
      if (used + 1 + sl > string_buf_size) {
        truncated = 1;
        break;
      }
      buf[used++] = uint8_t(sl);
      if (sl > 0) {
        memcpy(buf + used, s.data(), size_t(sl));
        used += sl;
      }
      n++;
    }
    count = uint16_t(n);
    truncated = truncated || src.size() > n;
  }

  std::string string_at(const int index) const
  {
    if (kind != WrangleArrayKind::String || index < 0 || index >= int(count)) {
      return {};
    }
    const uint8_t *buf = reinterpret_cast<const uint8_t *>(d.i);
    int used = 0;
    for (int k = 0; k < int(count); k++) {
      if (used >= string_buf_size) {
        break;
      }
      const int sl = buf[used++];
      if (k == index) {
        return std::string(reinterpret_cast<const char *>(buf + used), size_t(sl));
      }
      used += sl;
    }
    return {};
  }

  std::string to_string() const
  {
    std::string s = "[";
    const int n = int(count);
    auto comma = [&](const int i) {
      if (i + 1 < n) {
        s += ", ";
      }
    };
    if (kind == WrangleArrayKind::Float3) {
      for (int i = 0; i < n; i++) {
        s += '(';
        s += std::to_string(d.f[i * 3 + 0]);
        s += ", ";
        s += std::to_string(d.f[i * 3 + 1]);
        s += ", ";
        s += std::to_string(d.f[i * 3 + 2]);
        s += ')';
        comma(i);
      }
    }
    else if (kind == WrangleArrayKind::Float2) {
      for (int i = 0; i < n; i++) {
        s += '(';
        s += std::to_string(d.f[i * 2 + 0]);
        s += ", ";
        s += std::to_string(d.f[i * 2 + 1]);
        s += ')';
        comma(i);
      }
    }
    else if (kind == WrangleArrayKind::Float4) {
      for (int i = 0; i < n; i++) {
        s += '(';
        for (int k = 0; k < 4; k++) {
          s += std::to_string(d.f[i * 4 + k]);
          if (k + 1 < 4) {
            s += ", ";
          }
        }
        s += ')';
        comma(i);
      }
    }
    else if (kind == WrangleArrayKind::String) {
      for (int i = 0; i < n; i++) {
        s += '"';
        s += string_at(i);
        s += '"';
        comma(i);
      }
    }
    else if (kind == WrangleArrayKind::Float) {
      for (int i = 0; i < n; i++) {
        s += std::to_string(d.f[i]);
        comma(i);
      }
    }
    else if (kind == WrangleArrayKind::Matrix) {
      for (int i = 0; i < n; i++) {
        s += '[';
        const float4x4 m = matrix_at(i);
        for (int k = 0; k < 16; k++) {
          s += std::to_string((&m[0][0])[k]);
          if (k + 1 < 16) {
            s += ", ";
          }
        }
        s += ']';
        comma(i);
      }
    }
    else if (kind == WrangleArrayKind::Matrix2 || kind == WrangleArrayKind::Matrix3) {
      const int floats = (kind == WrangleArrayKind::Matrix2) ? matrix2_floats : matrix3_floats;
      for (int i = 0; i < n; i++) {
        s += '[';
        for (int k = 0; k < floats; k++) {
          s += std::to_string(d.f[i * floats + k]);
          if (k + 1 < floats) {
            s += ", ";
          }
        }
        s += ']';
        comma(i);
      }
    }
    else if (kind == WrangleArrayKind::Ray) {
      for (int i = 0; i < n; i++) {
        s += "ray(";
        s += std::to_string(int(d.f[i * ray_floats]));
        s += ", ";
        s += std::to_string(d.f[i * ray_floats + 7]);
        s += ')';
        comma(i);
      }
    }
    else {
      for (int i = 0; i < n; i++) {
        s += std::to_string(d.i[i]);
        comma(i);
      }
    }
    if (truncated || total > count) {
      s += ", …";
    }
    s += "]";
    return s;
  }
};

static_assert(std::is_trivially_copyable_v<WrangleArrayValue>);
static_assert(sizeof(WrangleArrayValue) <= 512);

}  // namespace blender::bke
