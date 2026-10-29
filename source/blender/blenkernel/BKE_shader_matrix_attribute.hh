/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once
#include "BKE_attribute.hh"
#include "BLI_color_types.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_shader_matrix_attribute.hh"

namespace blender::bke {
inline GAttributeReader
lookup_shader_matrix_column(const AttributeAccessor &attributes,
                            const StringRef request) {
  const auto column = shader_matrix_attribute_column(request);
  if (!column) {
    return {};
  }
  const auto source = attributes.lookup<float4x4>(column->name);
  if (!source) {
    return {};
  }
  const VArray<float4x4> matrices = source.varray;
  auto values = VArray<ColorGeometry4f>::from_func(
      matrices.size(), [matrices, c = column->column](const int64_t i) {
        const float4 value = matrices[i][c];
        return ColorGeometry4f(value.x, value.y, value.z, value.w);
      });
  return {std::move(values), source.domain, nullptr};
}
inline bool shader_attribute_exists(const AttributeAccessor &attributes,
                                    const StringRef name) {
  if (const auto request = shader_attribute_request(name)) {
    if (request->kind == ShaderAttributeRequestKind::Rotation) {
      return bool(attributes.lookup<math::Quaternion>(request->name));
    }
    return bool(lookup_shader_matrix_column(attributes, name));
  }
  return attributes.contains(name);
}

inline GAttributeReader lookup_shader_attribute_request(const AttributeAccessor &attributes,
                                                       const StringRef name)
{
  const auto request = shader_attribute_request(name);
  if (!request) {
    return {};
  }
  if (request->kind == ShaderAttributeRequestKind::MatrixColumn) {
    return lookup_shader_matrix_column(attributes, name);
  }
  const auto source = attributes.lookup<math::Quaternion>(request->name);
  if (!source) {
    return {};
  }
  const VArray<math::Quaternion> rotations = source.varray;
  auto values = VArray<ColorGeometry4f>::from_func(rotations.size(), [rotations](const int64_t i) {
    const math::Quaternion q = rotations[i];
    return ColorGeometry4f(q.w, q.x, q.y, q.z);
  });
  return {std::move(values), source.domain, nullptr};
}
} // namespace blender::bke
