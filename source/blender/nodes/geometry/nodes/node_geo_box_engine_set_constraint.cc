/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Build one joint (or append to an existing constraint point cloud) for Box Engine Solver.
 *
 * Body names match instance-domain `body_name` from Set Rigid Body.
 *
 * Distance: two bodies, rest length = current origin distance (not exposed).
 * Revolute: one body + world Anchor (hinge / pendulum).
 * Weld: two bodies, glued in their current relative pose (no anchor).
 * Prismatic: two bodies, B slides along Axis relative to A (drawer / piston).
 */

#include "BKE_attribute.hh"
#include "BKE_pointcloud.hh"

#include "BLI_array.hh"

#include "DNA_meshdata_types.h"
#include "DNA_pointcloud_types.h"

#include "box_engine_shared.hh"
#include "node_geometry_util.hh"

#include <string>

namespace blender::nodes::node_geo_box_engine_set_constraint_cc {

enum class JointType {
  Distance = 0,
  Revolute = 1,
  Weld = 2,
  Prismatic = 3,
};

static const EnumPropertyItem joint_type_items[] = {
    {int(JointType::Distance),
     "DISTANCE",
     0,
     N_("Distance"),
     N_("Keep the current distance between two body origins")},
    {int(JointType::Revolute),
     "REVOLUTE",
     0,
     N_("Revolute"),
     N_("Hinge one body to a world-space Anchor (door / pendulum)")},
    {int(JointType::Weld),
     "WELD",
     0,
     N_("Weld"),
     N_("Glue two bodies together as they are now")},
    {int(JointType::Prismatic),
     "PRISMATIC",
     0,
     N_("Prismatic"),
     N_("Body B slides along Axis relative to Body A, no relative rotation (piston / drawer)")},
    {0, nullptr, 0, nullptr, nullptr},
};

namespace attr {
constexpr StringRefNull body_a = "body_a";
constexpr StringRefNull body_b = "body_b";
constexpr StringRefNull joint_type = "joint_type";
constexpr StringRefNull axis = "axis";
}  // namespace attr

static bool joint_needs_body_b(const JointType type)
{
  return ELEM(type, JointType::Distance, JointType::Weld, JointType::Prismatic);
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Constraints"_ustr)
      .supported_type(GeometryComponent::Type::PointCloud)
      .description("Optional existing joints (point cloud) to append to");
  b.add_output<decl::Geometry>("Constraints"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Point cloud of joints → plug into Box Engine Solver · Constraints");

  b.add_input<decl::String>("Body A"_ustr)
      .default_value("Body")
      .description("Matches instance/point attribute body_name from Set Rigid Body");
  b.add_input<decl::String>("Body B"_ustr)
      .default_value("Body.001")
      .description("Second body_name. Required for Distance, Weld, and Prismatic")
      .usage_by_menu("Type"_ustr,
                     Array<int>{int(JointType::Distance),
                                int(JointType::Weld),
                                int(JointType::Prismatic)});

  b.add_input<decl::Menu>("Type"_ustr)
      .static_items(joint_type_items)
      .default_value(JointType::Revolute)
      .optional_label()
      .description(
          "Distance=keep spacing; Revolute=world hinge; Weld=glue; Prismatic=slider");

  b.add_input<decl::Vector>("Anchor"_ustr)
      .default_value(float3(0.0f, 0.0f, 0.0f))
      .subtype(PROP_TRANSLATION)
      .description("World-space hinge point for Revolute")
      .usage_by_menu("Type"_ustr, int(JointType::Revolute));

  b.add_input<decl::Vector>("Axis"_ustr)
      .default_value(float3(1.0f, 0.0f, 0.0f))
      .subtype(PROP_XYZ)
      .description("World-space slide direction. Body B moves along this axis")
      .usage_by_menu("Type"_ustr, int(JointType::Prismatic));
}

static void ensure_constraint_attrs(PointCloud &points)
{
  bke::MutableAttributeAccessor attributes = points.attributes_for_write();
  attributes.lookup_or_add_for_write_span<MStringProperty>(attr::body_a, bke::AttrDomain::Point)
      .finish();
  attributes.lookup_or_add_for_write_span<MStringProperty>(attr::body_b, bke::AttrDomain::Point)
      .finish();
  attributes.lookup_or_add_for_write_span<int>(attr::joint_type, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float3>(attr::axis, bke::AttrDomain::Point).finish();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Constraints"_ustr);
  const std::string body_a = params.extract_input<std::string>("Body A"_ustr);
  const std::string body_b_in = params.extract_input<std::string>("Body B"_ustr);
  const JointType joint_type = params.extract_input<JointType>("Type"_ustr);
  const float3 anchor = params.extract_input<float3>("Anchor"_ustr);
  const float3 axis = params.extract_input<float3>("Axis"_ustr);

  if (joint_needs_body_b(joint_type) && body_b_in.empty()) {
    params.error_message_add(
        NodeWarningType::Error,
        TIP_("Distance, Weld, and Prismatic need Body B (a second rigid body name)"));
  }

  const std::string body_b = joint_needs_body_b(joint_type) ? body_b_in : std::string();

  PointCloud *points = geometry.get_pointcloud_for_write();
  if (!points) {
    points = BKE_pointcloud_new_nomain(PointCloudType::Points, 0);
    geometry.replace_pointcloud(points);
  }

  const int old_n = points->totpoint;
  pointcloud_resize(*points, old_n + 1);
  ensure_constraint_attrs(*points);

  const int i = old_n;
  MutableSpan<float3> positions = points->positions_for_write();
  bke::MutableAttributeAccessor attributes = points->attributes_for_write();
  bke::SpanAttributeWriter<MStringProperty> a_w =
      attributes.lookup_or_add_for_write_span<MStringProperty>(attr::body_a, bke::AttrDomain::Point);
  bke::SpanAttributeWriter<MStringProperty> b_w =
      attributes.lookup_or_add_for_write_span<MStringProperty>(attr::body_b, bke::AttrDomain::Point);
  bke::SpanAttributeWriter<int> t_w = attributes.lookup_or_add_for_write_span<int>(
      attr::joint_type, bke::AttrDomain::Point);
  bke::SpanAttributeWriter<float3> axis_w = attributes.lookup_or_add_for_write_span<float3>(
      attr::axis, bke::AttrDomain::Point);

  /* Revolute stores the world hinge on the point; other joints compute frames from bodies. */
  positions[i] = (joint_type == JointType::Revolute) ? anchor : float3(0.0f);
  if (a_w) {
    box_engine::std_to_mstring(a_w.span[i], body_a);
  }
  if (b_w) {
    box_engine::std_to_mstring(b_w.span[i], body_b);
  }
  if (t_w) {
    t_w.span[i] = int(joint_type);
  }
  if (axis_w) {
    axis_w.span[i] = (math::length_squared(axis) > 1e-12f) ? axis : float3(1.0f, 0.0f, 0.0f);
  }

  if (a_w) {
    a_w.finish();
  }
  if (b_w) {
    b_w.finish();
  }
  if (t_w) {
    t_w.finish();
  }
  if (axis_w) {
    axis_w.finish();
  }

  params.set_output("Constraints"_ustr, std::move(geometry));
}

static void register_box_set_constraint()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeBoxEngineSetConstraint"_ustr, GEO_NODE_BOX_ENGINE_SET_CONSTRAINT);
  ntype.ui_name = "Box Set Constraint";
  ntype.ui_description =
      "Add one Box Engine joint. Distance/Weld/Prismatic take two named bodies. "
      "Revolute hinges Body A to a world Anchor";
  ntype.enum_name_legacy = "BOX_ENGINE_SET_CONSTRAINT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(register_box_set_constraint)

}  // namespace blender::nodes::node_geo_box_engine_set_constraint_cc
