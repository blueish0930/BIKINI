/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * 2D polygon intersection test of open-border islands.
 */

#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_do_intersect_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh A"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("First XY shape (face borders or closed edge rings).");
  b.add_input<decl::Geometry>("Mesh B"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Second XY shape to test against A.");
  b.add_output<decl::Bool>("Intersect"_ustr)
      .description("True if any XY border polygons intersect.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Mesh A"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Mesh B"_ustr);
  const Mesh *a = ga.get_mesh();
  const Mesh *b = gb.get_mesh();
  if (!a || !b) {
    params.error_message_add(NodeWarningType::Info, TIP_("Do Intersect 2D needs two meshes"));
    params.set_output("Intersect"_ustr, false);
    return;
  }
  std::string error;
  const bool hit = geometry::cgal_mesh_do_intersect_2(*a, *b, error);
  if (!error.empty() && !hit) {
    params.error_message_add(NodeWarningType::Warning, error);
  }
  params.set_output("Intersect"_ustr, hit);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalDoIntersect2"_ustr, GEO_NODE_CGAL_DO_INTERSECT_2);
  ntype.ui_name = "Do Intersect 2D";
  ntype.ui_description =
      "True if two XY polygons overlap — either their edges cross, or one "
      "contains the other. How to use: plug two 2D Meshes → read Intersect. "
      "This is a yes/no test, not a boolean volume (use Boolean Ops 2D for that).";
  ntype.enum_name_legacy = "CGAL_DO_INTERSECT_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_do_intersect_2_cc
