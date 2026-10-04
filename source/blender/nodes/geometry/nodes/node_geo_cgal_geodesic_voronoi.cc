/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_geodesic_voronoi_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle surface.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Faces split along the geodesic-equidistant Voronoi bisector. Face attribute cell_id.");
  b.add_input<decl::Bool>("Source"_ustr)
      .default_value(false)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Source vertices (Point domain). Each source seeds one geodesic cell.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  Field<bool> source_field = params.extract_input<Field<bool>>("Source"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info, TIP_("Geodesic Voronoi needs a mesh with faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  Array<bool> sources(mesh->verts_num, false);
  const bke::MeshFieldContext context{*mesh, AttrDomain::Point};
  fn::FieldEvaluator evaluator{context, mesh->verts_num};
  evaluator.add_with_destination(source_field, sources.as_mutable_span());
  evaluator.evaluate();

  bool any = false;
  for (const bool v : sources) {
    if (v) {
      any = true;
      break;
    }
  }
  if (!any) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Geodesic Voronoi needs at least one source vertex"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  std::string error;
  Mesh *out = geometry::cgal_mesh_geodesic_voronoi(*mesh, sources, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Geodesic Voronoi failed") : error);
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
      &ntype, "GeometryNodeCgalGeodesicVoronoi"_ustr, GEO_NODE_CGAL_GEODESIC_VORONOI);
  ntype.ui_name = "Geodesic Voronoi";
  ntype.ui_description =
      "Partition a triangle mesh by nearest source along geodesic distance "
      "(CGAL Surface_mesh_shortest_path MMP). Faces are cut on the geodesic-equidistant "
      "bisector, not along the input triangulation. Face attribute cell_id is the source "
      "vertex index.";
  ntype.enum_name_legacy = "CGAL_GEODESIC_VORONOI";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_geodesic_voronoi_cc
