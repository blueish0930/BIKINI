/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Isolines of multi-source MMP geodesic distance.
 * Distinct from Exact Geodesic (field), Refine at Isolevel (mesh split),
 * Geodesic Voronoi (cells), and Mesh Slicer (plane).
 */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_geodesic_isolines_cc {

static Curves *polylines_to_curves(const Span<Vector<float3>> polylines)
{
  int points_num = 0;
  int curves_num = 0;
  for (const Vector<float3> &pl : polylines) {
    if (pl.size() >= 2) {
      points_num += pl.size();
      curves_num++;
    }
  }
  if (curves_num == 0 || points_num < 2) {
    return bke::curves_new_nomain(0, 0);
  }
  Curves *curves_id = bke::curves_new_nomain(points_num, curves_num);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  MutableSpan<int> offsets = curves.offsets_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();
  int p = 0;
  int c = 0;
  offsets[0] = 0;
  for (const Vector<float3> &pl : polylines) {
    if (pl.size() < 2) {
      continue;
    }
    for (const float3 &pt : pl) {
      positions[p++] = pt;
    }
    c++;
    offsets[c] = p;
  }
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
      .description("Triangle surface.");
  b.add_output<decl::Geometry>("Curves"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Geodesic isolines as poly curves.");
  b.add_input<decl::Bool>("Source"_ustr)
      .default_value(false)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Source vertices (Point domain). Distance is 0 at sources.");
  b.add_input<decl::Int>("Count"_ustr)
      .default_value(8)
      .min(1)
      .max(256)
      .description("Number of isolines between the sources and the farthest reachable vertex.");
  b.add_input<decl::Float>("Max Distance"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description("If > 0, clamp the outermost isoline to this geodesic length. 0 = auto.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  Field<bool> source_field = params.extract_input<Field<bool>>("Source"_ustr);
  const int count = std::max(1, params.extract_input<int>("Count"_ustr));
  const float max_distance = params.extract_input<float>("Max Distance"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Geodesic Isolines needs a mesh with faces"));
    params.set_output("Curves"_ustr, GeometrySet());
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
                             TIP_("Geodesic Isolines needs at least one source vertex"));
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }

  std::string error;
  Vector<Vector<float3>> polylines;
  if (!geometry::cgal_mesh_geodesic_isolines(
          *mesh, sources, count, max_distance, polylines, error))
  {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Geodesic Isolines failed") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  Curves *curves = polylines_to_curves(polylines);
  if (curves->geometry.wrap().is_empty()) {
    params.error_message_add(NodeWarningType::Info, TIP_("No geodesic isolines"));
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(curves));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalGeodesicIsolines"_ustr, GEO_NODE_CGAL_GEODESIC_ISOLINES);
  ntype.ui_name = "Geodesic Isolines";
  ntype.ui_description =
      "Draw isolines of multi-source geodesic distance on a triangle mesh "
      "(CGAL Surface_mesh_shortest_path MMP). Exact Geodesic is a vertex field; "
      "Refine at Isolevel splits the mesh; Geodesic Voronoi partitions cells.";
  ntype.enum_name_legacy = "CGAL_GEODESIC_ISOLINES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_geodesic_isolines_cc
