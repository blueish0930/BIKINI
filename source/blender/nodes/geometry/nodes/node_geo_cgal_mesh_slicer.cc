/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_mesh_slicer_cc {

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
      .description("Triangle mesh to slice.");
  b.add_output<decl::Geometry>("Curves"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Intersection polylines as Curves (one spline per connected slice).");
  b.add_input<decl::Vector>("Origin"_ustr)
      .default_value(float3(0.0f))
      .subtype(PROP_TRANSLATION)
      .description("A point on the slicing plane.");
  b.add_input<decl::Vector>("Normal"_ustr)
      .default_value(float3(0.0f, 0.0f, 1.0f))
      .description("Plane facing (default +Z is a horizontal slice).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float3 origin = params.extract_input<float3>("Origin"_ustr);
  const float3 normal = params.extract_input<float3>("Normal"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh) {
    params.error_message_add(NodeWarningType::Info, TIP_("Mesh Slicer needs a mesh"));
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Vector<Vector<float3>> polylines;
  if (!geometry::cgal_mesh_plane_slice(*mesh, origin, normal, polylines, error)) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Mesh Slicer failed") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  Curves *curves = polylines_to_curves(polylines);
  if (curves->geometry.wrap().is_empty()) {
    params.error_message_add(NodeWarningType::Info, TIP_("No slice curves"));
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(curves));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMeshSlicer"_ustr, GEO_NODE_CGAL_MESH_SLICER);
  ntype.ui_name = "Mesh Slicer";
  ntype.ui_description =
      "Slice a triangle mesh with an infinite plane and output the cut as Curves "
      "(CGAL Polygon_mesh_slicer). How to use: plug a Mesh → set Origin on the "
      "plane and Normal as the facing → get polylines. "
      "This is a contour, not a solid boolean (use Mesh Boolean / Clip to cut volume).";
  ntype.enum_name_legacy = "CGAL_MESH_SLICER";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_mesh_slicer_cc
