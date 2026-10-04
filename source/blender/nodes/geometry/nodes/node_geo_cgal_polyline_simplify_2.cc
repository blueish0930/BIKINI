/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Simplify an XY polygon with squared-distance cost (CGAL Polyline_simplification_2).
 * Distinct from Simplify Polyline 3D (Curves in 3-space).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_polyline_simplify_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("XY polygon (or holes). Vertices are removed while the outline stays close.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Simplified n-gon(s).");
  b.add_input<decl::Float>("Cost"_ustr)
      .default_value(0.01f)
      .min(1.0e-8f)
      .description(
          "Stop when the next vertex removal would exceed this squared-distance cost. "
          "Never 0: values are clamped to a tiny epsilon.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float cost = std::max(params.extract_input<float>("Cost"_ustr), 1.0e-8f);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 3) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Shape Simplify 2D needs a polygon mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_polyline_simplify_2(*mesh, cost, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Shape Simplify 2D failed") : error);
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
      &ntype, "GeometryNodeCgalPolylineSimplify2"_ustr, GEO_NODE_CGAL_POLYLINE_SIMPLIFY_2);
  ntype.ui_name = "Shape Simplify 2D";
  ntype.ui_description =
      "Simplify an XY polygon/shape with squared-distance cost "
      "(CGAL Polyline_simplification_2). Distinct from Simplify Polyline 3D on Curves.";
  ntype.enum_name_legacy = "CGAL_POLYLINE_SIMPLIFY_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_polyline_simplify_2_cc
