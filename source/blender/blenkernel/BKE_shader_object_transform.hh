/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include <string>

#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"

namespace blender {
struct Depsgraph;
struct Object;
namespace bke {
/* Renderer object uniforms, never stored on geometry. Fields 0..3 are the full
 * matrix columns, followed by the quaternion (wxyz), signed scale, and the
 * Geometry Nodes object seed. */
std::string shader_object_transform_attribute_name(const Object *object, int field);
bool lookup_shader_object_transform_attribute(StringRefNull name, const Depsgraph *depsgraph,
                                              float4 &r_value);
} // namespace bke
} // namespace blender
