/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Geometry Nodes: Jolt Follow Animation.
 *
 * Inside a Simulation Zone the state geometry only knows the previous frame. Links from outside
 * the zone are evaluated at the current frame, so feeding the tagged rest geometry in as
 * "Animated" brings keyframed / constraint-driven transforms and the current body types in:
 *
 *   Collection Info → Jolt Set Rigid Body (Animated = keyframed bool) ─┬→ Simulation Input
 *                                                                      └→ Follow Animation · Animated
 *   Simulation Input → Jolt Follow Animation → Jolt Solver → Simulation Output
 *
 * Bodies are matched by body_name (index when names are missing). Static / kinematic bodies
 * take the animated transform; prev_position / prev_rotation get the state pose so the solver
 * sweeps from last frame. Dynamic bodies keep the simulated pose. A body switching kinematic →
 * dynamic keeps the animation's velocity in Jolt Solver (same body, motion type switched).
 * Works with Box Solver too (same attributes).
 */

#include "BKE_attribute.hh"
#include "BKE_instances.hh"
#include "BKE_pointcloud.hh"

#include "BLI_map.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion.hh"

#include "DNA_pointcloud_types.h"

#include "box_engine_shared.hh"
#include "node_geometry_util.hh"

#include <string>

namespace blender::nodes::node_geo_jolt_follow_animation_cc {

namespace attr {
constexpr StringRefNull body_type = "body_type";
constexpr StringRefNull rotation = "rotation";
constexpr StringRefNull prev_position = "prev_position";
constexpr StringRefNull prev_rotation = "prev_rotation";
constexpr StringRefNull has_prev = "has_prev";
}  // namespace attr

constexpr int body_dynamic = 2;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .description("Simulation state (from Simulation Input)");
  b.add_output<decl::Geometry>("Geometry"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("State with animated bodies moved to this frame's pose");
  b.add_input<decl::Geometry>("Animated"_ustr)
      .description(
          "The same tagged bodies, linked from outside the zone so they are evaluated at the "
          "current frame (keyframes, drivers, constraints, Animated field)");
  b.add_input<decl::Bool>("Selection"_ustr)
      .default_value(true)
      .hide_value()
      .evaluated_geometry_field()
      .description("State bodies that may follow the animation");
  b.add_input<decl::Bool>("Body Type"_ustr)
      .default_value(true)
      .description(
          "Also take body_type from Animated, so switching Static / Kinematic / Dynamic (or the "
          "Animated field) outside the zone reaches the simulation");
}

struct AnimatedSource {
  Map<std::string, int> index_by_name;
  int size = 0;
  VArray<int> body_types;
  bool has_body_type = false;
};

static AnimatedSource read_source(const bke::AttributeAccessor &attributes,
                                  const bke::AttrDomain domain,
                                  const int size)
{
  AnimatedSource src;
  src.size = size;
  src.has_body_type = attributes.contains(attr::body_type);
  src.body_types = *attributes.lookup_or_default<int>(attr::body_type, domain, body_dynamic);
  const VArray<MStringProperty> names = *attributes.lookup_or_default<MStringProperty>(
      box_engine::attr_body_name, domain, MStringProperty{});
  for (const int i : IndexRange(size)) {
    const std::string name = box_engine::mstring_to_std(names[i]);
    if (!name.empty()) {
      src.index_by_name.add(name, i);
    }
  }
  return src;
}

/** Index into Animated for state element `i`, or -1. */
static int match_index(const AnimatedSource &src, const std::string &name, const int i)
{
  if (!name.empty()) {
    return src.index_by_name.lookup_default(name, -1);
  }
  return (src.index_by_name.is_empty() && i < src.size) ? i : -1;
}

static int follow_instances(bke::Instances &state,
                            const bke::Instances &anim,
                            const fn::Field<bool> &selection_field,
                            const bool copy_type)
{
  const int n = state.instances_num();
  const AnimatedSource src = read_source(
      anim.attributes(), bke::AttrDomain::Instance, anim.instances_num());
  const Span<float4x4> anim_tf = anim.transforms();

  bke::InstancesFieldContext context(state);
  fn::FieldEvaluator evaluator(context, n);
  evaluator.set_selection(selection_field);
  evaluator.evaluate();
  const IndexMask selection = evaluator.get_evaluated_selection_as_mask();

  bke::MutableAttributeAccessor attributes = state.attributes_for_write();
  const VArray<MStringProperty> names = *attributes.lookup_or_default<MStringProperty>(
      box_engine::attr_body_name, bke::AttrDomain::Instance, MStringProperty{});
  bke::SpanAttributeWriter<int> types = attributes.lookup_or_add_for_write_span<int>(
      attr::body_type,
      bke::AttrDomain::Instance,
      bke::AttributeInitVArray(VArray<int>::from_single(body_dynamic, n)));
  bke::SpanAttributeWriter<float3> prev_pos = attributes.lookup_or_add_for_write_span<float3>(
      attr::prev_position, bke::AttrDomain::Instance);
  bke::SpanAttributeWriter<math::Quaternion> prev_rot =
      attributes.lookup_or_add_for_write_span<math::Quaternion>(attr::prev_rotation,
                                                               bke::AttrDomain::Instance);
  bke::SpanAttributeWriter<bool> has_prev = attributes.lookup_or_add_for_write_span<bool>(
      attr::has_prev, bke::AttrDomain::Instance);
  MutableSpan<float4x4> transforms = state.transforms_for_write();

  int matched = 0;
  selection.foreach_index([&](const int i) {
    const int j = match_index(src, box_engine::mstring_to_std(names[i]), i);
    if (j < 0) {
      return;
    }
    matched++;
    if (copy_type && src.has_body_type && types) {
      types.span[i] = src.body_types[j];
    }
    const int type = types ? types.span[i] : body_dynamic;
    if (type == body_dynamic) {
      return;
    }
    if (prev_pos && prev_rot && has_prev) {
      prev_pos.span[i] = transforms[i].location();
      prev_rot.span[i] = math::normalize(math::to_quaternion(math::normalize(transforms[i])));
      has_prev.span[i] = true;
    }
    transforms[i] = anim_tf[j];
  });
  types.finish();
  prev_pos.finish();
  prev_rot.finish();
  has_prev.finish();
  return matched;
}

static int follow_points(PointCloud &state,
                         const PointCloud &anim,
                         const fn::Field<bool> &selection_field,
                         const bool copy_type)
{
  const int n = state.totpoint;
  const AnimatedSource src = read_source(anim.attributes(), bke::AttrDomain::Point, anim.totpoint);
  const Span<float3> anim_pos = anim.positions();
  const VArray<math::Quaternion> anim_rot = *anim.attributes().lookup_or_default<math::Quaternion>(
      attr::rotation, bke::AttrDomain::Point, math::Quaternion::identity());

  bke::PointCloudFieldContext context(state);
  fn::FieldEvaluator evaluator(context, n);
  evaluator.set_selection(selection_field);
  evaluator.evaluate();
  const IndexMask selection = evaluator.get_evaluated_selection_as_mask();

  bke::MutableAttributeAccessor attributes = state.attributes_for_write();
  const VArray<MStringProperty> names = *attributes.lookup_or_default<MStringProperty>(
      box_engine::attr_body_name, bke::AttrDomain::Point, MStringProperty{});
  bke::SpanAttributeWriter<int> types = attributes.lookup_or_add_for_write_span<int>(
      attr::body_type,
      bke::AttrDomain::Point,
      bke::AttributeInitVArray(VArray<int>::from_single(body_dynamic, n)));
  bke::SpanAttributeWriter<math::Quaternion> rot =
      attributes.lookup_or_add_for_write_span<math::Quaternion>(
          attr::rotation,
          bke::AttrDomain::Point,
          bke::AttributeInitVArray(
              VArray<math::Quaternion>::from_single(math::Quaternion::identity(), n)));
  bke::SpanAttributeWriter<float3> prev_pos = attributes.lookup_or_add_for_write_span<float3>(
      attr::prev_position, bke::AttrDomain::Point);
  bke::SpanAttributeWriter<math::Quaternion> prev_rot =
      attributes.lookup_or_add_for_write_span<math::Quaternion>(attr::prev_rotation,
                                                               bke::AttrDomain::Point);
  bke::SpanAttributeWriter<bool> has_prev = attributes.lookup_or_add_for_write_span<bool>(
      attr::has_prev, bke::AttrDomain::Point);
  MutableSpan<float3> positions = state.positions_for_write();

  int matched = 0;
  selection.foreach_index([&](const int i) {
    const int j = match_index(src, box_engine::mstring_to_std(names[i]), i);
    if (j < 0) {
      return;
    }
    matched++;
    if (copy_type && src.has_body_type && types) {
      types.span[i] = src.body_types[j];
    }
    const int type = types ? types.span[i] : body_dynamic;
    if (type == body_dynamic) {
      return;
    }
    if (prev_pos && prev_rot && has_prev && rot) {
      prev_pos.span[i] = positions[i];
      prev_rot.span[i] = rot.span[i];
      has_prev.span[i] = true;
    }
    positions[i] = anim_pos[j];
    if (rot) {
      rot.span[i] = anim_rot[j];
    }
  });
  types.finish();
  rot.finish();
  prev_pos.finish();
  prev_rot.finish();
  has_prev.finish();
  state.tag_positions_changed();
  return matched;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry"_ustr);
  const GeometrySet animated = params.extract_input<GeometrySet>("Animated"_ustr);
  const fn::Field<bool> selection = params.extract_input<fn::Field<bool>>("Selection"_ustr);
  const bool copy_type = params.extract_input<bool>("Body Type"_ustr);

  int bodies = 0;
  int matched = 0;
  if (geometry.has_instances() && animated.has_instances()) {
    bke::Instances &state = *geometry.get_instances_for_write();
    bodies = state.instances_num();
    matched = follow_instances(state, *animated.get_instances(), selection, copy_type);
  }
  else if (geometry.has_pointcloud() && animated.has_pointcloud()) {
    PointCloud &state = *geometry.get_pointcloud_for_write();
    bodies = state.totpoint;
    matched = follow_points(state, *animated.get_pointcloud(), selection, copy_type);
  }
  if (bodies > 0 && matched == 0) {
    params.error_message_add(
        NodeWarningType::Warning,
        TIP_("No body of Geometry matches Animated (body_name, or index when names are missing)"));
  }
  params.set_output("Geometry"_ustr, std::move(geometry));
}

static void register_jolt_follow_animation()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeJoltFollowAnimation"_ustr);
  ntype.ui_name = "Jolt Follow Animation";
  ntype.ui_description =
      "Inside a Simulation Zone: move static / kinematic bodies to the transforms of the same "
      "bodies evaluated outside the zone (keyframes) and take their body type, so animated "
      "bodies can push the simulation and be handed over to it. Also works with Box Solver";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(register_jolt_follow_animation)

}  // namespace blender::nodes::node_geo_jolt_follow_animation_cc
