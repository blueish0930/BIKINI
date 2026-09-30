/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL PMP refine_mesh_at_isolevel: split edges where a vertex scalar crosses isovalue.
 */

#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_refine_isolevel_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle surface to refine.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Mesh with edges split at the isoline and new isoline edges.");
  b.add_output<decl::Bool>("Isoline"_ustr)
      .anonymous_attribute_output()
      .description("Edge selection: edges lying on the isovalue contour.");
  b.add_input<decl::Float>("Value"_ustr)
      .default_value(0.0f)
      .structure_type(StructureType::Field)
      .description("Scalar at each vertex (Point domain). Edges are split where this crosses Isovalue.");
  b.add_input<decl::Float>("Isovalue"_ustr)
      .default_value(0.0f)
      .description("Contour level. Linear interpolation places new vertices on crossed edges.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float isovalue = params.extract_input<float>("Isovalue"_ustr);
  Field<float> value_field = params.extract_input<Field<float>>("Value"_ustr);

  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in || mesh_in->verts_num == 0) {
    params.set_default_remaining_outputs();
    return;
  }

  Array<float> values(mesh_in->verts_num);
  const bke::MeshFieldContext context(*mesh_in, bke::AttrDomain::Point);
  fn::FieldEvaluator evaluator(context, mesh_in->verts_num);
  evaluator.add_with_destination(value_field, values.as_mutable_span());
  evaluator.evaluate();

  const std::optional<std::string> isoline_id =
      params.get_output_anonymous_attribute_id_if_needed("Isoline"_ustr);

  std::string error;
  Array<bool> isoline_edges;
  Mesh *out = geometry::cgal_mesh_refine_at_isolevel(
      *mesh_in, values, isovalue, isoline_id ? &isoline_edges : nullptr, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Refine at Isolevel failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_default_remaining_outputs();
    return;
  }

  if (isoline_id && isoline_edges.size() == out->edges_num) {
    bke::MutableAttributeAccessor attrs = out->attributes_for_write();
    bke::SpanAttributeWriter<bool> w = attrs.lookup_or_add_for_write_only_span<bool>(
        *isoline_id, bke::AttrDomain::Edge);
    w.span.copy_from(isoline_edges);
    w.finish();
  }

  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalRefineIsolevel"_ustr, GEO_NODE_CGAL_REFINE_ISOLEVEL);
  ntype.ui_name = "Refine at Isolevel";
  ntype.ui_description =
      "Split edges where a vertex scalar field crosses Isovalue and insert contour edges "
      "(CGAL refine_mesh_at_isolevel). Use Isoline edges for selection / store.";
  ntype.enum_name_legacy = "CGAL_REFINE_ISOLEVEL";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_refine_isolevel_cc
