/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Slice a mesh with planes that all contain a shared axis.
 */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_radial_slices_cc {

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
      .description("Intersection polylines of every radial plane.");
  b.add_input<decl::Vector>("Origin"_ustr)
      .default_value(float3(0.0f))
      .subtype(PROP_TRANSLATION)
      .description("A point on the shared axis.");
  b.add_input<decl::Vector>("Axis"_ustr)
      .default_value(float3(0.0f, 0.0f, 1.0f))
      .description("Shared axis of all slicing planes (default +Z).");
  b.add_input<decl::Int>("Count"_ustr)
      .default_value(12)
      .min(1)
      .max(180)
      .description("Number of distinct planes around 180° (a plane equals its opposite).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float3 origin = params.extract_input<float3>("Origin"_ustr);
  const float3 axis = params.extract_input<float3>("Axis"_ustr);
  const int count = std::max(1, params.extract_input<int>("Count"_ustr));
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info, TIP_("Radial Slices needs a mesh"));
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Vector<Vector<float3>> polylines;
  if (!geometry::cgal_mesh_radial_slices(*mesh, origin, axis, count, polylines, error)) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Radial Slices failed") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  Curves *curves = polylines_to_curves(polylines);
  if (curves->geometry.wrap().is_empty()) {
    params.error_message_add(NodeWarningType::Info, TIP_("No radial slice curves"));
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(curves));
}

static void node_register()
{
  /* Deleted: Radial Slices. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalRadialSlices"_ustr, GEO_NODE_CGAL_RADIAL_SLICES);
  ntype.ui_name = "Radial Slices";
  ntype.ui_description =
      "Slice a mesh with Count planes that all contain Axis, equally spaced "
      "in angle (CGAL Polygon_mesh_slicer). Mesh Slicer is one plane. "
      "Contour Stack uses parallel planes.";
  ntype.enum_name_legacy = "CGAL_RADIAL_SLICES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_radial_slices_cc
