/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 *
 * Geometry components that the "Select Elements" and "Edit Elements" nodes work on.
 */

#pragma once

#include <array>
#include <optional>

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

namespace blender::nodes {

/** The supported components, in the order they are stored in and looked for. */
inline constexpr std::array<bke::GeometryComponent::Type, 4> elements_component_types = {
    bke::GeometryComponent::Type::Mesh,
    bke::GeometryComponent::Type::Curve,
    bke::GeometryComponent::Type::PointCloud,
    bke::GeometryComponent::Type::Instance,
};

/** Index into #elements_component_types, -1 for a component that is not supported. */
inline int elements_component_slot(const bke::GeometryComponent::Type type)
{
  for (const int i : IndexRange(elements_component_types.size())) {
    if (elements_component_types[i] == type) {
      return i;
    }
  }
  return -1;
}

inline bool elements_component_supports_domain(const bke::GeometryComponent::Type type,
                                               const bke::AttrDomain domain)
{
  switch (type) {
    case bke::GeometryComponent::Type::Mesh:
      return ELEM(domain, bke::AttrDomain::Point, bke::AttrDomain::Edge, bke::AttrDomain::Face);
    case bke::GeometryComponent::Type::Curve:
      return ELEM(domain, bke::AttrDomain::Point, bke::AttrDomain::Curve);
    case bke::GeometryComponent::Type::PointCloud:
      return domain == bke::AttrDomain::Point;
    case bke::GeometryComponent::Type::Instance:
      return domain == bke::AttrDomain::Instance;
    default:
      return false;
  }
}

/** The domain itself when the component has it, otherwise the one elements are selected on. */
inline bke::AttrDomain elements_component_sanitize_domain(const bke::GeometryComponent::Type type,
                                                          const bke::AttrDomain domain)
{
  if (elements_component_supports_domain(type, domain)) {
    return domain;
  }
  return type == bke::GeometryComponent::Type::Instance ? bke::AttrDomain::Instance :
                                                          bke::AttrDomain::Point;
}

/** Number of vertices, control points, points or instances. */
inline int elements_component_size(const bke::GeometrySet &geometry,
                                   const bke::GeometryComponent::Type type)
{
  switch (type) {
    case bke::GeometryComponent::Type::Mesh:
      return geometry.has_mesh() ? geometry.get_mesh()->verts_num : 0;
    case bke::GeometryComponent::Type::Curve:
      return geometry.has_curves() ? geometry.get_curves()->geometry.point_num : 0;
    case bke::GeometryComponent::Type::PointCloud:
      return geometry.has_pointcloud() ? geometry.get_pointcloud()->totpoint : 0;
    case bke::GeometryComponent::Type::Instance:
      return geometry.has_instances() ? geometry.get_instances()->instances_num() : 0;
    default:
      return 0;
  }
}

/**
 * The component to edit: the preferred one when the geometry has elements of it, otherwise the
 * first one that has any.
 */
inline std::optional<bke::GeometryComponent::Type> elements_component_pick(
    const bke::GeometrySet &geometry, const bke::GeometryComponent::Type preferred)
{
  if (elements_component_slot(preferred) != -1 && elements_component_size(geometry, preferred) > 0)
  {
    return preferred;
  }
  for (const bke::GeometryComponent::Type type : elements_component_types) {
    if (elements_component_size(geometry, type) > 0) {
      return type;
    }
  }
  return std::nullopt;
}

/**
 * Instances have no edit mode, so they are edited as one point per instance. This attribute on
 * the points is the index of the instance a point stands for.
 */
inline constexpr StringRefNull elements_instance_source_attribute = "instance_source";

/** One point at the origin of every instance. The caller owns the result. */
PointCloud *elements_instances_to_points(const bke::Instances &instances);

/**
 * The instances the points stand for, moved to where the points are. Points without a valid
 * source instance are skipped. The caller owns the result.
 */
bke::Instances *elements_points_to_instances(const bke::Instances &src_instances,
                                             const PointCloud &points);

}  // namespace blender::nodes
