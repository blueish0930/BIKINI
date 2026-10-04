/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * QEM (Garland–Heckbert). Keep Boundary / Preserve lock edges via Constrained_placement.
 * Replaces the old unconstrained GeometryNodeCgalQemSimplify.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "FN_field.hh"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_constrained_simplify_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Surface mesh. Internally triangulated for collapse.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("QEM-simplified mesh; constrained edges are kept.");
  b.add_input<decl::Float>("Keep Ratio"_ustr)
      .default_value(0.5f)
      .min(0.01f)
      .max(1.0f)
      .description("Fraction of unconstrained edges to keep.");
  b.add_input<decl::Bool>("Keep Boundary"_ustr)
      .default_value(true)
      .description("Do not collapse border edges (open rims stay).");
  b.add_input<decl::Bool>("Preserve"_ustr)
      .default_value(false)
      .hide_value()
      .evaluated_geometry_field()
      .description("Vertices that must stay: every incident edge is locked.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float keep = params.extract_input<float>("Keep Ratio"_ustr);
  const bool keep_boundary = params.extract_input<bool>("Keep Boundary"_ustr);
  const Field<bool> preserve_field = params.extract_input<Field<bool>>("Preserve"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  Array<bool> preserve_bool(mesh_in->verts_num, false);
  const bke::MeshFieldContext context(*mesh_in, bke::AttrDomain::Point);
  fn::FieldEvaluator evaluator(context, mesh_in->verts_num);
  evaluator.add_with_destination(preserve_field, preserve_bool.as_mutable_span());
  evaluator.evaluate();

  Array<uint8_t> preserve(mesh_in->verts_num);
  for (const int i : IndexRange(mesh_in->verts_num)) {
    preserve[i] = preserve_bool[i] ? 1 : 0;
  }

  std::string error;
  Mesh *out = geometry::cgal_mesh_constrained_simplify(
      *mesh_in, keep, keep_boundary, preserve.as_span(), error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("QEM Simplify failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalConstrainedSimplify"_ustr, GEO_NODE_CGAL_CONSTRAINED_SIMPLIFY);
  ntype.ui_name = "QEM Simplify";
  ntype.ui_description =
      "Garland–Heckbert QEM edge-collapse (CGAL Constrained_placement). "
      "Keep Ratio is the fraction of unconstrained edges to keep. "
      "Keep Boundary locks open rims; Preserve vertices lock every incident edge.";
  ntype.enum_name_legacy = "CGAL_CONSTRAINED_SIMPLIFY";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_constrained_simplify_cc
