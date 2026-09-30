/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_does_self_intersect_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to test.");
  b.add_output<decl::Bool>("Self Intersects"_ustr)
      .description("True if at least one self-intersection is found.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
  params.set_output("Self Intersects"_ustr, false);
    return;
  }
  std::string error;
  const bool hit = geometry::cgal_mesh_does_self_intersect(*mesh_in, error);
  if (!error.empty()) {
    params.error_message_add(NodeWarningType::Warning, error);
  }
  params.set_output("Self Intersects"_ustr, hit);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalDoesSelfIntersect"_ustr, GEO_NODE_CGAL_DOES_SELF_INTERSECT);
  ntype.ui_name = "Does Self Intersect";
  ntype.ui_description =
      "Test whether any faces of the mesh intersect each other (excluding shared edges/verts).";
  ntype.enum_name_legacy = "CGAL_DOES_SELF_INTERSECT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_does_self_intersect_cc
