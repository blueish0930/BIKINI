/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Cotangent Laplacian mesh editing with ROI / control handles / targets.
 * Not an alias of Fair — dedicated handle-driven solve (Eigen).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "FN_field.hh"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_laplace_deform_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Surface mesh to deform (triangulated internally).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Deformed mesh (same topology, attributes preserved).");
  b.add_input<decl::Bool>("ROI"_ustr)
      .default_value(true)
      .hide_value()
      .evaluated_geometry_field()
      .description("Region of interest: vertices free to move (include handles).");
  b.add_input<decl::Bool>("Control"_ustr)
      .default_value(false)
      .hide_value()
      .evaluated_geometry_field()
      .description("Control / handle vertices whose Target positions drive the deform.");
  b.add_input<decl::Vector>("Target"_ustr)
      .evaluated_geometry_field()
      .default_input_type(NODE_DEFAULT_INPUT_POSITION_FIELD)
      .description("Target position for each control vertex (ignored for non-controls).");
  b.add_input<decl::Float>("Weight"_ustr)
      .default_value(1.0e6f)
      .min(1.0e-3f)
      .description("Soft pin weight for fixed / control vertices (higher = harder pins).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Field<bool> roi_field = params.extract_input<Field<bool>>("ROI"_ustr);
  const Field<bool> control_field = params.extract_input<Field<bool>>("Control"_ustr);
  const Field<float3> target_field = params.extract_input<Field<float3>>("Target"_ustr);
  const float weight = params.extract_input<float>("Weight"_ustr);

  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in || mesh_in->verts_num == 0) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  Array<bool> roi_bool(mesh_in->verts_num);
  Array<bool> control_bool(mesh_in->verts_num);
  Array<float3> targets(mesh_in->verts_num);

  const bke::MeshFieldContext context(*mesh_in, bke::AttrDomain::Point);
  fn::FieldEvaluator evaluator(context, mesh_in->verts_num);
  evaluator.add_with_destination(roi_field, roi_bool.as_mutable_span());
  evaluator.add_with_destination(control_field, control_bool.as_mutable_span());
  evaluator.add_with_destination(target_field, targets.as_mutable_span());
  evaluator.evaluate();

  Array<uint8_t> roi_mask(mesh_in->verts_num);
  Array<uint8_t> control_mask(mesh_in->verts_num);
  int control_count = 0;
  for (const int i : IndexRange(mesh_in->verts_num)) {
    roi_mask[i] = roi_bool[i] ? 1 : 0;
    control_mask[i] = control_bool[i] ? 1 : 0;
    if (control_mask[i]) {
      control_count++;
      roi_mask[i] = 1;
    }
  }
  if (control_count == 0) {
    params.error_message_add(NodeWarningType::Warning, TIP_("Select at least one Control vertex"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  std::string error;
  Mesh *mesh = geometry::cgal_mesh_laplace_deform(*mesh_in,
                                                  roi_mask.as_span(),
                                                  control_mask.as_span(),
                                                  targets.as_span(),
                                                  weight,
                                                  error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Laplace Deform failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalLaplaceDeform"_ustr, GEO_NODE_CGAL_LAPLACE_DEFORM);
  ntype.ui_name = "Laplace Deform";
  ntype.ui_description =
      "Cotangent Laplacian mesh editing with ROI and control handles (Eigen sparse solve). "
      "One Laplacian position LS, then one rotation-compensation LS. "
      "Not the same as Fair / Smooth Shape.";
  ntype.enum_name_legacy = "CGAL_LAPLACE_DEFORM";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_laplace_deform_cc
