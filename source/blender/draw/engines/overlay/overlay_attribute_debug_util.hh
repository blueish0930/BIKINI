/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 *
 * Shared helpers for Geometry Nodes Debug overlays: domain positions (Spline / Instance /
 * Layer), and looking up Debug attributes stored on geometry-set instance stacks.
 */

#pragma once

#include <algorithm>
#include <string>

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_duplilist.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"

#include "BLI_array.hh"
#include "BLI_color_types.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_string_ref.hh"

#include "DNA_grease_pencil_types.h"
#include "DNA_object_types.h"

namespace blender::draw::overlay::debug_attr {

/**
 * Materialize positions for drawing text/arrows on the given attribute domain.
 * Uses attribute domain adaptation for Point/Edge/Face/Corner/Curve when possible.
 * Returns empty array when the domain has no positions (e.g. Instance / Layer need other helpers).
 */
inline Array<float3> positions_for_domain(const bke::AttributeAccessor &attributes,
                                          const bke::AttrDomain domain)
{
  const bke::AttributeReader<float3> positions = attributes.lookup<float3>("position", domain);
  if (!positions) {
    return {};
  }
  Array<float3> result(positions.varray.size());
  positions.varray.materialize(result.as_mutable_span());
  return result;
}

/**
 * Explicit curve-center positions for Spline (Curve) domain — more reliable than domain
 * adaptation when the attribute accessor is thin (legacy Curve curve_eval path).
 */
inline Array<float3> curve_domain_positions(const bke::CurvesGeometry &curves)
{
  const int curves_num = curves.curves_num();
  Array<float3> result(curves_num, float3(0.0f));
  if (curves_num == 0) {
    return result;
  }
  const Span<float3> positions = curves.positions();
  const OffsetIndices points_by_curve = curves.points_by_curve();
  for (const int curve_i : curves.curves_range()) {
    const IndexRange points = points_by_curve[curve_i];
    if (points.is_empty()) {
      continue;
    }
    float3 sum(0.0f);
    for (const int point_i : points) {
      sum += positions[point_i];
    }
    result[curve_i] = sum / float(points.size());
  }
  return result;
}

/** Instance origins from instance transforms (object/local space of the component). */
inline Array<float3> instance_domain_positions(const bke::Instances &instances)
{
  const Span<float4x4> transforms = instances.transforms();
  Array<float3> result(transforms.size());
  for (const int64_t i : transforms.index_range()) {
    result[i] = transforms[i].location();
  }
  return result;
}

/** Grease Pencil Layer origins in object space. */
inline Array<float3> layer_domain_positions(const GreasePencil &grease_pencil, const Object &object)
{
  const Span<const bke::greasepencil::Layer *> layers = grease_pencil.layers();
  Array<float3> result(layers.size());
  for (const int i : layers.index_range()) {
    const float4x4 layer_to_object = layers[i]->to_object_space(object);
    result[i] = layer_to_object.location();
  }
  return result;
}

/**
 * Walk the DupliObject geometry-set stack (innermost → outermost) and return the first
 * InstancesComponent that contains \a name, plus the instance index within that component.
 */
inline bool find_instance_attribute(const DupliObject *dupli,
                                    const StringRef name,
                                    bke::GAttributeReader &r_attribute,
                                    int &r_instance_index)
{
  if (dupli == nullptr) {
    return false;
  }
  for (const int i : IndexRange(ARRAY_SIZE(dupli->instance_data))) {
    if (dupli->instance_data[i] == nullptr) {
      continue;
    }
    const bke::Instances *instances = dupli->instance_data[i]->get_instances();
    if (instances == nullptr) {
      continue;
    }
    const bke::AttributeAccessor attributes = instances->attributes();
    bke::GAttributeReader attribute = attributes.lookup(name);
    if (!attribute) {
      continue;
    }
    r_attribute = std::move(attribute);
    r_instance_index = dupli->instance_idx[i];
    return true;
  }
  return false;
}

inline bool instance_has_debug_color(const DupliObject *dupli)
{
  bke::GAttributeReader attribute;
  int index = -1;
  return find_instance_attribute(dupli, ".debug_color", attribute, index);
}

inline bool instance_debug_color_at(const DupliObject *dupli, ColorGeometry4f &r_color)
{
  bke::GAttributeReader attribute;
  int index = -1;
  if (!find_instance_attribute(dupli, ".debug_color", attribute, index)) {
    return false;
  }
  if (index < 0 || index >= attribute.varray.size()) {
    return false;
  }
  /* Convert whatever type is stored to ColorGeometry4f via lookup_or_default path. */
  const bke::Instances *instances = nullptr;
  for (const int i : IndexRange(ARRAY_SIZE(dupli->instance_data))) {
    if (dupli->instance_data[i] == nullptr) {
      continue;
    }
    instances = dupli->instance_data[i]->get_instances();
    if (instances && instances->attributes().contains(".debug_color")) {
      break;
    }
  }
  if (instances == nullptr) {
    return false;
  }
  const bke::AttributeReader<ColorGeometry4f> colors =
      instances->attributes().lookup_or_default<ColorGeometry4f>(
          ".debug_color",
          bke::AttrDomain::Instance,
          ColorGeometry4f(1.0f, 0.0f, 1.0f, 1.0f));
  if (!colors || index >= colors.varray.size()) {
    return false;
  }
  r_color = colors.varray[index];
  return true;
}

inline Vector<std::string> collect_debug_value_names(const bke::AttributeAccessor &attributes)
{
  Vector<std::string> names;
  attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.name.startswith(".debug_") && iter.name.endswith("_value")) {
      names.append(iter.name);
    }
  });
  std::sort(names.begin(), names.end());
  return names;
}

inline std::string layer_prefix_from_value_name(const StringRef name)
{
  if (name.startswith(".debug_") && name.endswith("_value")) {
    return std::string(name.drop_suffix(6));
  }
  return "";
}

/**
 * Collect Debug layers from InstancesComponent (instance domain).
 * Used when drawing text/arrows for every instance from the parent object.
 */
inline bool instances_have_debug_layers(const bke::Instances &instances)
{
  const bke::AttributeAccessor attributes = instances.attributes();
  if (attributes.contains(".debug_color")) {
    return true;
  }
  bool found = false;
  attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
    if (iter.name.startswith(".debug_") && iter.name.endswith("_value")) {
      found = true;
    }
  });
  return found;
}

}  // namespace blender::draw::overlay::debug_attr
