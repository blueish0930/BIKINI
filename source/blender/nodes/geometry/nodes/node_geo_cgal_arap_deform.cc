/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "FN_field.hh"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_arap_deform_cc {

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
  b.add_input<decl::Int>("Method"_ustr)
      .default_value(1)
      .min(0)
      .max(1)
      .description("0 = Original ARAP, 1 = Spokes-and-Rims ARAP (default).");
  b.add_input<decl::Int>("Iterations"_ustr).default_value(5).min(1).max(200).description(
      "ARAP local/global iterations.");
  b.add_input<decl::Float>("Tolerance"_ustr)
      .default_value(1e-4f)
      .min(0.0f)
      .description("Energy convergence tolerance (0 = ignore).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Field<bool> roi_field = params.extract_input<Field<bool>>("ROI"_ustr);
  const Field<bool> control_field = params.extract_input<Field<bool>>("Control"_ustr);
  const Field<float3> target_field = params.extract_input<Field<float3>>("Target"_ustr);
  const int method = params.extract_input<int>("Method"_ustr);
  const int iterations = params.extract_input<int>("Iterations"_ustr);
  const float tolerance = params.extract_input<float>("Tolerance"_ustr);

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
      /* Controls must be in ROI for a well-defined solve. */
      roi_mask[i] = 1;
    }
  }
  if (control_count == 0) {
    params.error_message_add(NodeWarningType::Warning, TIP_("Select at least one Control vertex"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  std::string error;
  Mesh *mesh = geometry::cgal_mesh_arap_deform(*mesh_in,
                                               roi_mask.as_span(),
                                               control_mask.as_span(),
                                               targets.as_span(),
                                               method,
                                               iterations,
                                               tolerance,
                                               error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("ARAP Deform failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalArapDeform"_ustr, GEO_NODE_CGAL_ARAP_DEFORM);
  ntype.ui_name = "ARAP Deform";
  ntype.ui_description =
      "As-Rigid-As-Possible surface mesh deformation (CGAL Surface_mesh_deformation). "
      "Select ROI and Control verts; set Target positions on controls (e.g. Set Position field). "
      "Method 0=Original ARAP, 1=Spokes-and-Rims.";
  ntype.enum_name_legacy = "CGAL_ARAP_DEFORM";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_arap_deform_cc
