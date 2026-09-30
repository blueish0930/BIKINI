/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Shortest path along mesh edges (Dijkstra).
 * Distinct from Surface Shortest Path (geodesic MMP on the surface).
 */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_mesh_edge_path_cc {

static Curves *polyline_to_curves(const Span<float3> pts)
{
  if (pts.size() < 2) {
    return bke::curves_new_nomain(0, 0);
  }
  Curves *curves_id = bke::curves_new_nomain(int(pts.size()), 1);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  curves.offsets_for_write()[0] = 0;
  curves.offsets_for_write()[1] = int(pts.size());
  curves.positions_for_write().copy_from(pts);
  curves.fill_curve_types(CURVE_TYPE_POLY);
  return curves_id;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle mesh whose edges form the graph.");
  b.add_output<decl::Geometry>("Curves"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("One polyline along existing mesh edges.");
  b.add_input<decl::Bool>("Source"_ustr)
      .default_value(false)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Source vertices (Point domain).");
  b.add_input<decl::Bool>("Target"_ustr)
      .default_value(false)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Target vertices (Point domain). Path stops at the nearest.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  Field<bool> source_field = params.extract_input<Field<bool>>("Source"_ustr);
  Field<bool> target_field = params.extract_input<Field<bool>>("Target"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info, TIP_("Mesh Edge Path needs a mesh with faces"));
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  Array<bool> sources(mesh->verts_num, false);
  Array<bool> targets(mesh->verts_num, false);
  const bke::MeshFieldContext context{*mesh, AttrDomain::Point};
  fn::FieldEvaluator evaluator{context, mesh->verts_num};
  evaluator.add_with_destination(source_field, sources.as_mutable_span());
  evaluator.add_with_destination(target_field, targets.as_mutable_span());
  evaluator.evaluate();

  std::string error;
  Vector<float3> polyline;
  if (!geometry::cgal_mesh_edge_path(*mesh, sources, targets, polyline, error)) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Mesh Edge Path failed") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(polyline_to_curves(polyline)));
}

static void node_register()
{
  /* Deleted: Mesh Edge Path. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMeshEdgePath"_ustr, GEO_NODE_CGAL_MESH_EDGE_PATH);
  ntype.ui_name = "Mesh Edge Path";
  ntype.ui_description =
      "Shortest path that only walks existing mesh edges (Dijkstra). "
      "Surface Shortest Path is the geodesic on the surface (MMP), not constrained "
      "to edges. Mark Source and Target vertices.";
  ntype.enum_name_legacy = "CGAL_MESH_EDGE_PATH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_mesh_edge_path_cc
