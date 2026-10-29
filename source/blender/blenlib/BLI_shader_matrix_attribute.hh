/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include "BLI_string_ref.hh"
#include <optional>
#include <string>

namespace blender {

/* Renderer-only column requests. The geometry continues to own one ordinary
 * FLOAT4X4 attribute; no extra attributes are created on the user's geometry.
 */
struct ShaderMatrixAttributeColumn {
  StringRef name;
  int column;
};

inline std::string shader_matrix_attribute_column_name(const StringRef name,
                                                       const int column) {
  return ".m4c" + std::to_string(column) + ":" + std::string(name);
}

inline std::optional<ShaderMatrixAttributeColumn>
shader_matrix_attribute_column(const StringRef request) {
  if (request.size() <= 6 || !request.startswith(".m4c") || request[4] < '0' ||
      request[4] > '3' || request[5] != ':') {
    return std::nullopt;
  }
  return ShaderMatrixAttributeColumn{request.substr(6), request[4] - '0'};
}

enum class ShaderAttributeRequestKind { MatrixColumn, Rotation };

struct ShaderAttributeRequest {
  StringRef name;
  int column;
  ShaderAttributeRequestKind kind;
};

inline std::string shader_rotation_attribute_name(const StringRef name)
{
  return ".q4:" + std::string(name);
}

/* Both requests use a four-component transport buffer. Keep the geometry's original attribute
 * untouched, and explicitly bind a zero buffer for missing data instead of GPU's (0,0,0,1). */
inline std::optional<ShaderAttributeRequest> shader_attribute_request(const StringRef request)
{
  if (const auto column = shader_matrix_attribute_column(request)) {
    return ShaderAttributeRequest{column->name, column->column, ShaderAttributeRequestKind::MatrixColumn};
  }
  if (request.size() > 4 && request.startswith(".q4:")) {
    return ShaderAttributeRequest{request.substr(4), 0, ShaderAttributeRequestKind::Rotation};
  }
  return std::nullopt;
}

} // namespace blender
