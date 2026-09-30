/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Mesh ∩ finite cone side → Curves. Distinct from Sphere / Cylinder Slice. */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_cone_slice_cc {

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
      .description("Intersection polylines as Curves.");
  b.add_input<decl::Vector>("Apex"_ustr)
      .default_value(float3(0.0f))
      .subtype(PROP_TRANSLATION)
      .description("Cone tip.");
  b.add_input<decl::Vector>("Axis"_ustr)
      .default_value(float3(0.0f, 0.0f, 1.0f))
      .description("Direction from Apex toward the base.");
  b.add_input<decl::Float>("Radius"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Radius at the base.");
  b.add_input<decl::Float>("Height"_ustr)
      .default_value(2.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Apex-to-base length along Axis (no cap).");
  b.add_input<decl::Int>("Segments"_ustr)
      .default_value(32)
      .min(8)
      .max(128)
      .description("Tessellation of the intersecting cone.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float3 apex = params.extract_input<float3>("Apex"_ustr);
  const float3 axis = params.extract_input<float3>("Axis"_ustr);
  const float radius = params.extract_input<float>("Radius"_ustr);
  const float height = params.extract_input<float>("Height"_ustr);
  const int segments = params.extract_input<int>("Segments"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh) {
    params.error_message_add(NodeWarningType::Info, TIP_("Cone Slice needs a mesh"));
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Vector<Vector<float3>> polylines;
  if (!geometry::cgal_mesh_cone_slice(
          *mesh, apex, axis, radius, height, segments, polylines, error))
  {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Cone Slice failed") : error);
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
  /* Deleted: Cone Slice. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalConeSlice"_ustr, GEO_NODE_CGAL_CONE_SLICE);
  ntype.ui_name = "Cone Slice";
  ntype.ui_description =
      "Intersect a mesh with a finite cone side and output the cut as Curves "
      "(CGAL PMP surface_intersection). Distinct from Sphere Slice and Cylinder Slice.";
  ntype.enum_name_legacy = "CGAL_CONE_SLICE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_cone_slice_cc
