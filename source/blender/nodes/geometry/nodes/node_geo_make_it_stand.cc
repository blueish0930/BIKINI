/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "FN_field.hh"
#include "GEO_make_it_stand.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_make_it_stand_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Closed solid. Outer topology is kept unless Boolean is on.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Outer surface (same topology). Chain this into the next Make It Stand / Simulation.");
  b.add_input<decl::Geometry>("Inner Void"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Cavity from a previous Make It Stand. Chain Inner Void → Inner Void.");
  b.add_output<decl::Geometry>("Inner Void"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Accumulated inner cavity. Chain into the next node's Inner Void.");
  b.add_output<decl::Geometry>("Support"_ustr).description("Support polygon on the contact plane.");
  b.add_output<decl::Vector>("Center"_ustr).description("Volume center of mass after balancing.");
  b.add_output<decl::Vector>("Projected"_ustr).description("CoM projected onto the contact plane.");
  b.add_output<decl::Vector>("Target"_ustr).description("Support centroid (or custom target).");
  b.add_output<decl::Bool>("Stable"_ustr).description("Projected CoM is inside the support.");
  b.add_output<decl::Float>("Margin"_ustr).description(
      "Signed in-hull distance of the projected CoM (positive = inside).");
  b.add_output<decl::Int>("Carved"_ustr).description("Interior voxels emptied.");
  b.add_input<decl::Vector>("Gravity"_ustr)
      .default_value(float3(0.0f, 0.0f, -1.0f))
      .description("Gravity direction (points down).");
  b.add_input<decl::Bool>("Use Target"_ustr)
      .default_value(false)
      .description("Use Target instead of the support-polygon centroid.");
  b.add_input<decl::Vector>("Target"_ustr).description("Desired CoM projection on the ground plane.");
  b.add_input<decl::Float>("Shell"_ustr)
      .default_value(0.05f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Uncarvable wall thickness.");
  b.add_input<decl::Int>("Resolution"_ustr)
      .default_value(32)
      .min(8)
      .max(64)
      .description("Interior voxel samples along the longest bbox axis.");
  b.add_input<decl::Int>("Iterations"_ustr)
      .default_value(4)
      .min(1)
      .max(16)
      .description("Carve then optional deform, re-voxelizing the solid each step.");
  b.add_input<decl::Float>("Deform"_ustr)
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .description(
          "How far each iteration moves the CoM toward the landing point "
          "(Laplace handles: pin the feet, translate the heavy side). 0 = carve only.");
  b.add_input<decl::Float>("Lower Mass"_ustr)
      .default_value(0.25f)
      .min(0.0f)
      .max(4.0f)
      .description("Prefer carving higher voxels, which also drops the CoM.");
  b.add_input<decl::Float>("Max Empty"_ustr)
      .default_value(0.8f)
      .min(0.05f)
      .max(0.95f)
      .description("Maximum fraction of interior voxels that may be removed.");
  b.add_input<decl::Float>("Epsilon"_ustr)
      .default_value(0.01f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Auto-contact plane thickness.");
  b.add_input<decl::Float>("Density"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .description("Uniform density for mass.");
  b.add_input<decl::Bool>("Contact"_ustr)
      .default_value(false)
      .hide_value()
      .evaluated_geometry_field()
      .description("Explicit contact vertices. If none are true, the lowest verts are used.");
  b.add_input<decl::Geometry>("Support"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::PointCloud})
      .description("Optional extra contact points.");
  b.add_input<decl::Bool>("Boolean"_ustr)
      .default_value(false)
      .description(
          "Subtract Inner Void from Mesh (remeshes the outer surface). "
          "Leave off to chain nodes or use in a Simulation zone.");
}

static Span<float3> extra_positions(const GeometrySet &geometry)
{
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    return pc->positions();
  }
  if (const Mesh *mesh = geometry.get_mesh()) {
    return mesh->vert_positions();
  }
  return {};
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  GeometrySet void_geo = params.extract_input<GeometrySet>("Inner Void"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh) {
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Inner Void"_ustr, std::move(void_geo));
    params.set_output("Support"_ustr, GeometrySet());
    params.set_output("Center"_ustr, float3(0.0f));
    params.set_output("Projected"_ustr, float3(0.0f));
    params.set_output("Target"_ustr, float3(0.0f));
    params.set_output("Stable"_ustr, false);
    params.set_output("Margin"_ustr, 0.0f);
    params.set_output("Carved"_ustr, 0);
    return;
  }

  Array<bool> contact_bool(mesh->verts_num, false);
  {
    const Field<bool> contact_field = params.extract_input<Field<bool>>("Contact"_ustr);
    const bke::MeshFieldContext context(*mesh, bke::AttrDomain::Point);
    fn::FieldEvaluator evaluator(context, mesh->verts_num);
    evaluator.add_with_destination(contact_field, contact_bool.as_mutable_span());
    evaluator.evaluate();
  }
  GeometrySet support_geo = params.extract_input<GeometrySet>("Support"_ustr);

  geometry::MakeItStandParams stand_params;
  stand_params.gravity = params.extract_input<float3>("Gravity"_ustr);
  stand_params.use_target = params.extract_input<bool>("Use Target"_ustr);
  stand_params.target = params.extract_input<float3>("Target"_ustr);
  stand_params.shell_thickness = params.extract_input<float>("Shell"_ustr);
  stand_params.resolution = params.extract_input<int>("Resolution"_ustr);
  stand_params.iterations = params.extract_input<int>("Iterations"_ustr);
  stand_params.deform_strength = params.extract_input<float>("Deform"_ustr);
  stand_params.lower_mass = params.extract_input<float>("Lower Mass"_ustr);
  stand_params.max_empty_fraction = params.extract_input<float>("Max Empty"_ustr);
  stand_params.contact_epsilon = params.extract_input<float>("Epsilon"_ustr);
  stand_params.density = params.extract_input<float>("Density"_ustr);
  stand_params.boolean_cavity = params.extract_input<bool>("Boolean"_ustr);
  stand_params.contact_selection = contact_bool;
  stand_params.extra_contact_points = extra_positions(support_geo);
  stand_params.existing_void = void_geo.get_mesh();

  std::string error;
  geometry::MakeItStandResult result = geometry::mesh_make_it_stand(*mesh, stand_params, error);
  if (!error.empty()) {
    params.error_message_add(NodeWarningType::Warning, error);
  }
  else if (!result.warning.empty()) {
    params.error_message_add(NodeWarningType::Info, result.warning);
  }
  if (result.is_stable) {
    params.error_message_add(NodeWarningType::Info, TIP_("Stable"));
  }
  else {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Still unbalanced — raise Resolution, Deform, or Iterations"));
  }

  params.set_output("Mesh"_ustr, result.mesh ? GeometrySet::from_mesh(result.mesh) : GeometrySet());
  params.set_output("Inner Void"_ustr,
                    result.inner_void ? GeometrySet::from_mesh(result.inner_void) : GeometrySet());
  params.set_output("Support"_ustr,
                    result.support ? GeometrySet::from_mesh(result.support) : GeometrySet());
  params.set_output("Center"_ustr, result.center);
  params.set_output("Projected"_ustr, result.projected);
  params.set_output("Target"_ustr, result.target);
  params.set_output("Stable"_ustr, result.is_stable);
  params.set_output("Margin"_ustr, result.margin);
  params.set_output("Carved"_ustr, result.carved_voxels);
  if (result.contact_points) {
    BKE_id_free(nullptr, result.contact_points);
  }
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeMakeItStand"_ustr, GEO_NODE_MAKE_IT_STAND);
  ntype.ui_name = "Make It Stand";
  ntype.ui_description =
      "Make It Stand (SIGGRAPH 2013): carve the heavy-side interior, then Laplacian-deform "
      "so the center of mass projects onto the landing. Not smoothing, not base-scale.";
  ntype.enum_name_legacy = "MAKE_IT_STAND";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_make_it_stand_cc
