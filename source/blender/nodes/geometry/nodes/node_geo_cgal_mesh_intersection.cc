/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL PMP surface_intersection → Curves (polylines).
 * Differs from Mesh Boolean Intersect (solid volume).
 */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_mesh_intersection_cc {

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
  b.add_input<decl::Geometry>("Mesh 1"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("First surface mesh.");
  b.add_input<decl::Geometry>("Mesh 2"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Second surface mesh.");
  b.add_output<decl::Geometry>("Curves"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Intersection polylines as poly curves.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Mesh 1"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Mesh 2"_ustr);
  const Mesh *a = ga.get_mesh();
  const Mesh *b = gb.get_mesh();
  if (!a || !b) {
    params.error_message_add(NodeWarningType::Info, TIP_("Both inputs need a mesh"));
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Vector<Vector<float3>> polylines;
  if (!geometry::cgal_mesh_surface_intersection(*a, *b, polylines, error)) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Mesh Intersection failed") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  Curves *curves = polylines_to_curves(polylines);
  if (curves->geometry.wrap().is_empty()) {
    params.error_message_add(NodeWarningType::Info, TIP_("No intersection curves"));
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(curves));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalMeshIntersection"_ustr, GEO_NODE_CGAL_MESH_INTERSECTION);
  ntype.ui_name = "Mesh Intersection";
  ntype.ui_description =
      "Compute surface intersection polylines of two meshes (CGAL surface_intersection). "
      "Output is Curves, not a solid boolean volume (use Mesh Boolean for that).";
  ntype.enum_name_legacy = "CGAL_MESH_INTERSECTION";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_mesh_intersection_cc
