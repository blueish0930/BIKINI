/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Append one Jolt constraint (point-cloud record) for Jolt Solver.
 *
 * Types match Jolt's constraint set:
 *   https://jrouwe.github.io/JoltPhysics/index.html#constraints
 *
 * Body names match instance-domain `body_name` from Jolt Set Rigid Body.
 * Empty Body B = world (Body::sFixedToWorld).
 */

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "BLI_array.hh"
#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_pointcloud_types.h"

#include "box_engine_shared.hh"
#include "node_geometry_util.hh"

#include <string>

namespace blender::nodes::node_geo_jolt_set_constraint_cc {

enum class JointType {
  Fixed = 0,
  Distance = 1,
  Point = 2,
  Hinge = 3,
  Cone = 4,
  Slider = 5,
  SwingTwist = 6,
  SixDOF = 7,
  Path = 8,
  Gear = 9,
  RackAndPinion = 10,
  Pulley = 11,
  Vehicle = 12,
};

enum class MotorState {
  Off = 0,
  Velocity = 1,
  Position = 2,
};

static const EnumPropertyItem joint_type_items[] = {
    {int(JointType::Fixed), "FIXED", 0, N_("Fixed"), N_("Weld two bodies (or a body to the world)")},
    {int(JointType::Distance),
     "DISTANCE",
     0,
     N_("Distance"),
     N_("Keep two points a distance apart (optional min/max + spring)")},
    {int(JointType::Point), "POINT", 0, N_("Point"), N_("Ball joint: share one world point")},
    {int(JointType::Hinge), "HINGE", 0, N_("Hinge"), N_("One rotation axis (door / wheel axle)")},
    {int(JointType::Cone), "CONE", 0, N_("Cone"), N_("Point + swing limited to a cone")},
    {int(JointType::Slider),
     "SLIDER",
     0,
     N_("Slider"),
     N_("Prismatic: slide along Axis, no relative rotation")},
    {int(JointType::SwingTwist),
     "SWING_TWIST",
     0,
     N_("Swing Twist"),
     N_("Shoulder-like: cone swing + twist limits")},
    {int(JointType::SixDOF),
     "SIX_DOF",
     0,
     N_("Six DOF"),
     N_("Per-axis translation / rotation limits")},
    {int(JointType::Path),
     "PATH",
     0,
     N_("Path"),
     N_("Slide along a Hermite path (Path geometry or Anchor→Axis)")},
    {int(JointType::Gear),
     "GEAR",
     0,
     N_("Gear"),
     N_("Couple two world hinges (Ratio = teeth_B / teeth_A)")},
    {int(JointType::RackAndPinion),
     "RACK_AND_PINION",
     0,
     N_("Rack and Pinion"),
     N_("Hinge on A + slider on B, geared together")},
    {int(JointType::Pulley),
     "PULLEY",
     0,
     N_("Pulley"),
     N_("Two bodies, two world attachment points (rope / block-and-tackle)")},
    {int(JointType::Vehicle),
     "VEHICLE",
     0,
     N_("Vehicle"),
     N_("Virtual wheels on Body A (Z-up chassis). Motor Target = throttle")},
    {0, nullptr, 0, nullptr, nullptr},
};

static const EnumPropertyItem motor_items[] = {
    {int(MotorState::Off), "OFF", 0, N_("Off"), N_("No motor")},
    {int(MotorState::Velocity), "VELOCITY", 0, N_("Velocity"), N_("Drive to a target speed")},
    {int(MotorState::Position), "POSITION", 0, N_("Position"), N_("Drive to a target angle / offset")},
    {0, nullptr, 0, nullptr, nullptr},
};

namespace attr {
constexpr StringRefNull body_a = "body_a";
constexpr StringRefNull body_b = "body_b";
constexpr StringRefNull joint_type = "joint_type";
constexpr StringRefNull axis = "axis";
constexpr StringRefNull axis2 = "axis2";
constexpr StringRefNull anchor2 = "anchor2";
constexpr StringRefNull min_limit = "min_limit";
constexpr StringRefNull max_limit = "max_limit";
constexpr StringRefNull min_limit2 = "min_limit2";
constexpr StringRefNull max_limit2 = "max_limit2";
constexpr StringRefNull ratio = "ratio";
constexpr StringRefNull frequency = "frequency";
constexpr StringRefNull damping = "damping";
constexpr StringRefNull motor_state = "motor_state";
constexpr StringRefNull motor_target = "motor_target";
constexpr StringRefNull motor_force = "motor_force";
constexpr StringRefNull path_id = "path_id";
constexpr StringRefNull wheel_radius = "wheel_radius";
constexpr StringRefNull wheel_width = "wheel_width";
constexpr StringRefNull suspension_min = "suspension_min";
constexpr StringRefNull suspension_max = "suspension_max";
constexpr StringRefNull max_steer = "max_steer";
constexpr StringRefNull num_wheels = "num_wheels";
}  // namespace attr

static bool uses_body_b(const JointType type)
{
  return ELEM(type,
              JointType::Fixed,
              JointType::Distance,
              JointType::Point,
              JointType::Hinge,
              JointType::Cone,
              JointType::Slider,
              JointType::SwingTwist,
              JointType::SixDOF,
              JointType::Path,
              JointType::Gear,
              JointType::RackAndPinion,
              JointType::Pulley);
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Constraints"_ustr)
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Optional existing joints to append to");
  b.add_output<decl::Geometry>("Constraints"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Point cloud of Jolt joints → Jolt Solver · Constraints");

  b.add_input<decl::String>("Body A"_ustr)
      .default_value("Body")
      .description("Matches instance/point attribute body_name");
  b.add_input<decl::String>("Body B"_ustr)
      .default_value("")
      .description("Second body_name. Empty = world. Unused for Vehicle")
      .usage_by_menu("Type"_ustr,
                     Array<int>{int(JointType::Fixed),
                                int(JointType::Distance),
                                int(JointType::Point),
                                int(JointType::Hinge),
                                int(JointType::Cone),
                                int(JointType::Slider),
                                int(JointType::SwingTwist),
                                int(JointType::SixDOF),
                                int(JointType::Path),
                                int(JointType::Gear),
                                int(JointType::RackAndPinion),
                                int(JointType::Pulley)});

  b.add_input<decl::Menu>("Type"_ustr)
      .static_items(joint_type_items)
      .default_value(JointType::Hinge)
      .optional_label()
      .description("Jolt joint type");

  b.add_input<decl::Vector>("Anchor"_ustr)
      .default_value(float3(0.0f, 0.0f, 0.0f))
      .subtype(PROP_TRANSLATION)
      .description("World-space attachment point");

  b.add_input<decl::Vector>("Axis"_ustr)
      .default_value(float3(0.0f, 0.0f, 1.0f))
      .subtype(PROP_XYZ)
      .description("Primary axis (hinge / slider / twist / gear)")
      .usage_by_menu("Type"_ustr,
                     Array<int>{int(JointType::Hinge),
                                int(JointType::Cone),
                                int(JointType::Slider),
                                int(JointType::SwingTwist),
                                int(JointType::SixDOF),
                                int(JointType::Path),
                                int(JointType::Gear),
                                int(JointType::RackAndPinion),
                                int(JointType::Pulley)});

  b.add_input<decl::Vector>("Axis 2"_ustr)
      .default_value(float3(1.0f, 0.0f, 0.0f))
      .subtype(PROP_XYZ)
      .description("Secondary axis (hinge normal / gear B / rack slider / pulley B)")
      .usage_by_menu("Type"_ustr,
                     Array<int>{int(JointType::Hinge),
                                int(JointType::SwingTwist),
                                int(JointType::SixDOF),
                                int(JointType::Gear),
                                int(JointType::RackAndPinion),
                                int(JointType::Pulley)});

  b.add_input<decl::Vector>("Anchor 2"_ustr)
      .default_value(float3(0.0f, 0.0f, 0.0f))
      .subtype(PROP_TRANSLATION)
      .description("Second world point (pulley body B / gear B hinge)")
      .usage_by_menu("Type"_ustr, Array<int>{int(JointType::Pulley), int(JointType::Gear)});

  b.add_input<decl::Float>("Min Limit"_ustr)
      .default_value(0.0f)
      .description("Hinge/slider min (rad or m). Distance min. Cone unused")
      .usage_by_menu("Type"_ustr,
                     Array<int>{int(JointType::Distance),
                                int(JointType::Hinge),
                                int(JointType::Slider),
                                int(JointType::SwingTwist)});
  b.add_input<decl::Float>("Max Limit"_ustr)
      .default_value(0.0f)
      .description("Hinge/slider max. Cone half-angle (rad). Distance max")
      .usage_by_menu("Type"_ustr,
                     Array<int>{int(JointType::Distance),
                                int(JointType::Hinge),
                                int(JointType::Cone),
                                int(JointType::Slider),
                                int(JointType::SwingTwist)});

  b.add_input<decl::Float>("Ratio"_ustr)
      .default_value(1.0f)
      .description("Gear / rack / pulley ratio")
      .usage_by_menu("Type"_ustr,
                     Array<int>{int(JointType::Gear),
                                int(JointType::RackAndPinion),
                                int(JointType::Pulley)});

  b.add_input<decl::Float>("Frequency"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description("Distance limit spring Hz (0 = hard)")
      .usage_by_menu("Type"_ustr, int(JointType::Distance));
  b.add_input<decl::Float>("Damping"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description("Distance limit spring damping")
      .usage_by_menu("Type"_ustr, int(JointType::Distance));

  b.add_input<decl::Menu>("Motor"_ustr)
      .static_items(motor_items)
      .default_value(MotorState::Off)
      .optional_label()
      .description("Hinge/slider/vehicle motor: off, velocity, or position")
      .usage_by_menu("Type"_ustr,
                     Array<int>{int(JointType::Hinge),
                                int(JointType::Slider),
                                int(JointType::Vehicle)});
  b.add_input<decl::Float>("Motor Target"_ustr)
      .default_value(0.0f)
      .description("Hinge rad or rad/s. Slider m or m/s. Vehicle throttle [-1, 1]")
      .usage_by_menu("Type"_ustr,
                     Array<int>{int(JointType::Hinge),
                                int(JointType::Slider),
                                int(JointType::Vehicle)});
  b.add_input<decl::Float>("Motor Force"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description("Max motor force/torque. Vehicle: engine torque")
      .usage_by_menu("Type"_ustr,
                     Array<int>{int(JointType::Hinge),
                                int(JointType::Slider),
                                int(JointType::Vehicle)});

  b.add_input<decl::Geometry>("Path"_ustr)
      .description("Optional polyline / mesh verts for Path constraint")
      .usage_by_menu("Type"_ustr, int(JointType::Path));

  b.add_input<decl::Float>("Wheel Radius"_ustr)
      .default_value(0.3f)
      .min(0.01f)
      .description("Vehicle wheel radius")
      .usage_by_menu("Type"_ustr, int(JointType::Vehicle));
  b.add_input<decl::Float>("Wheel Width"_ustr)
      .default_value(0.1f)
      .min(0.01f)
      .description("Vehicle wheel width")
      .usage_by_menu("Type"_ustr, int(JointType::Vehicle));
  b.add_input<decl::Float>("Suspension Min"_ustr)
      .default_value(0.2f)
      .min(0.0f)
      .description("Minimum suspension length")
      .usage_by_menu("Type"_ustr, int(JointType::Vehicle));
  b.add_input<decl::Float>("Suspension Max"_ustr)
      .default_value(0.4f)
      .min(0.0f)
      .description("Maximum suspension length")
      .usage_by_menu("Type"_ustr, int(JointType::Vehicle));
  b.add_input<decl::Int>("Wheels"_ustr)
      .default_value(4)
      .min(2)
      .max(4)
      .description("Wheel count (2 or 4)")
      .usage_by_menu("Type"_ustr, int(JointType::Vehicle));
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
  attributes.lookup_or_add_for_write_span<float3>(attr::axis2, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float3>(attr::anchor2, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float>(attr::min_limit, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float>(attr::max_limit, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float>(attr::min_limit2, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float>(attr::max_limit2, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float>(attr::ratio, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float>(attr::frequency, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float>(attr::damping, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<int>(attr::motor_state, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float>(attr::motor_target, bke::AttrDomain::Point)
      .finish();
  attributes.lookup_or_add_for_write_span<float>(attr::motor_force, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<int>(attr::path_id, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float>(attr::wheel_radius, bke::AttrDomain::Point)
      .finish();
  attributes.lookup_or_add_for_write_span<float>(attr::wheel_width, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<float>(attr::suspension_min, bke::AttrDomain::Point)
      .finish();
  attributes.lookup_or_add_for_write_span<float>(attr::suspension_max, bke::AttrDomain::Point)
      .finish();
  attributes.lookup_or_add_for_write_span<float>(attr::max_steer, bke::AttrDomain::Point).finish();
  attributes.lookup_or_add_for_write_span<int>(attr::num_wheels, bke::AttrDomain::Point).finish();
}

static Mesh *append_path_mesh(const Mesh *existing, const Span<float3> verts, const int path_id)
{
  if (verts.size() < 2) {
    return existing ? const_cast<Mesh *>(existing) : nullptr;
  }
  Vector<float3> all;
  Vector<int> ids;
  Vector<int2> edges;
  if (existing && existing->verts_num > 0) {
    all.extend(existing->vert_positions());
    const VArray<int> old_ids = *existing->attributes().lookup_or_default<int>(
        attr::path_id, bke::AttrDomain::Point, -1);
    for (const int i : existing->vert_positions().index_range()) {
      ids.append(old_ids[i]);
    }
    edges.extend(existing->edges());
  }
  const int base = all.size();
  all.extend(verts);
  for ([[maybe_unused]] const int k : verts.index_range()) {
    ids.append(path_id);
  }
  for (int k = 0; k + 1 < verts.size(); k++) {
    edges.append(int2(base + k, base + k + 1));
  }
  Mesh *mesh = BKE_mesh_new_nomain(all.size(), edges.size(), 0, 0);
  mesh->vert_positions_for_write().copy_from(all);
  mesh->edges_for_write().copy_from(edges);
  bke::SpanAttributeWriter<int> id_w =
      mesh->attributes_for_write().lookup_or_add_for_write_span<int>(attr::path_id,
                                                                    bke::AttrDomain::Point);
  if (id_w) {
    id_w.span.copy_from(ids);
    id_w.finish();
  }
  mesh->tag_positions_changed();
  return mesh;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Constraints"_ustr);
  const std::string body_a = params.extract_input<std::string>("Body A"_ustr);
  const std::string body_b_in = params.extract_input<std::string>("Body B"_ustr);
  const JointType joint_type = params.extract_input<JointType>("Type"_ustr);
  const float3 anchor = params.extract_input<float3>("Anchor"_ustr);
  const float3 axis = params.extract_input<float3>("Axis"_ustr);
  const float3 axis2 = params.extract_input<float3>("Axis 2"_ustr);
  const float3 anchor2 = params.extract_input<float3>("Anchor 2"_ustr);
  const float min_limit = params.extract_input<float>("Min Limit"_ustr);
  const float max_limit = params.extract_input<float>("Max Limit"_ustr);
  const float ratio = params.extract_input<float>("Ratio"_ustr);
  const float frequency = params.extract_input<float>("Frequency"_ustr);
  const float damping = params.extract_input<float>("Damping"_ustr);
  const MotorState motor = params.extract_input<MotorState>("Motor"_ustr);
  const float motor_target = params.extract_input<float>("Motor Target"_ustr);
  const float motor_force = params.extract_input<float>("Motor Force"_ustr);
  GeometrySet path_geo = params.extract_input<GeometrySet>("Path"_ustr);
  const float wheel_radius = params.extract_input<float>("Wheel Radius"_ustr);
  const float wheel_width = params.extract_input<float>("Wheel Width"_ustr);
  const float sus_min = params.extract_input<float>("Suspension Min"_ustr);
  const float sus_max = params.extract_input<float>("Suspension Max"_ustr);
  const int num_wheels = params.extract_input<int>("Wheels"_ustr);

  const std::string body_b = uses_body_b(joint_type) ? body_b_in : std::string();

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

  auto write_str = [&](StringRef name, const std::string &value) {
    bke::SpanAttributeWriter<MStringProperty> w =
        attributes.lookup_or_add_for_write_span<MStringProperty>(name, bke::AttrDomain::Point);
    if (w) {
      box_engine::std_to_mstring(w.span[i], value);
      w.finish();
    }
  };
  auto write_i = [&](StringRef name, const int value) {
    bke::SpanAttributeWriter<int> w = attributes.lookup_or_add_for_write_span<int>(
        name, bke::AttrDomain::Point);
    if (w) {
      w.span[i] = value;
      w.finish();
    }
  };
  auto write_f = [&](StringRef name, const float value) {
    bke::SpanAttributeWriter<float> w = attributes.lookup_or_add_for_write_span<float>(
        name, bke::AttrDomain::Point);
    if (w) {
      w.span[i] = value;
      w.finish();
    }
  };
  auto write_v = [&](StringRef name, const float3 value) {
    bke::SpanAttributeWriter<float3> w = attributes.lookup_or_add_for_write_span<float3>(
        name, bke::AttrDomain::Point);
    if (w) {
      w.span[i] = value;
      w.finish();
    }
  };

  positions[i] = anchor;
  write_str(attr::body_a, body_a);
  write_str(attr::body_b, body_b);
  write_i(attr::joint_type, int(joint_type));
  write_v(attr::axis, math::length_squared(axis) > 1e-12f ? axis : float3(0.0f, 0.0f, 1.0f));
  write_v(attr::axis2, math::length_squared(axis2) > 1e-12f ? axis2 : float3(1.0f, 0.0f, 0.0f));
  write_v(attr::anchor2, anchor2);
  write_f(attr::min_limit, min_limit);
  write_f(attr::max_limit, max_limit);
  write_f(attr::min_limit2, 0.0f);
  write_f(attr::max_limit2, 0.0f);
  write_f(attr::ratio, ratio);
  write_f(attr::frequency, frequency);
  write_f(attr::damping, damping);
  write_i(attr::motor_state, int(motor));
  write_f(attr::motor_target, motor_target);
  write_f(attr::motor_force, motor_force);
  write_i(attr::path_id, i);
  write_f(attr::wheel_radius, wheel_radius);
  write_f(attr::wheel_width, wheel_width);
  write_f(attr::suspension_min, sus_min);
  write_f(attr::suspension_max, sus_max);
  write_f(attr::max_steer, 0.0f);
  write_i(attr::num_wheels, num_wheels);

  if (joint_type == JointType::Path) {
    Vector<float3> path_verts;
    if (const Mesh *mesh = path_geo.get_mesh()) {
      path_verts.extend(mesh->vert_positions());
    }
    if (path_verts.size() < 2) {
      const float3 dir = math::length_squared(axis) > 1e-12f ? math::normalize(axis) :
                                                               float3(0.0f, 0.0f, 1.0f);
      const float len = max_limit > 0.0f ? max_limit : 1.0f;
      path_verts.append(anchor);
      path_verts.append(anchor + dir * len);
    }
    Mesh *path_mesh = append_path_mesh(geometry.get_mesh(), path_verts, i);
    if (path_mesh && path_mesh != geometry.get_mesh()) {
      geometry.replace_mesh(path_mesh);
    }
  }

  params.set_output("Constraints"_ustr, std::move(geometry));
}

static void register_jolt_set_constraint()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeJoltSetConstraint"_ustr, GEO_NODE_JOLT_SET_CONSTRAINT);
  ntype.ui_name = "Jolt Set Constraint";
  ntype.ui_description =
      "Add one Jolt constraint (fixed, distance, point, hinge, cone, slider, swing-twist, "
      "six-DOF, path, gear, rack-and-pinion, pulley, vehicle). Plug into Jolt Solver";
  ntype.enum_name_legacy = "JOLT_SET_CONSTRAINT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(register_jolt_set_constraint)

}  // namespace blender::nodes::node_geo_jolt_set_constraint_cc
