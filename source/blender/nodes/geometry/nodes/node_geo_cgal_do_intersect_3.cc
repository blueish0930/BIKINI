/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Test whether two triangle meshes intersect (CGAL PMP::do_intersect).
 */

#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_do_intersect_3_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh A"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("First triangle surface.");
  b.add_input<decl::Geometry>("Mesh B"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Second triangle surface.");
  b.add_output<decl::Bool>("Intersect"_ustr)
      .description("True if any triangles of A and B intersect.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Mesh A"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Mesh B"_ustr);
  const Mesh *a = ga.get_mesh();
  const Mesh *b = gb.get_mesh();
  if (!a || !b || a->faces_num == 0 || b->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info, TIP_("Do Intersect 3D needs two meshes"));
    params.set_output("Intersect"_ustr, false);
    return;
  }
  std::string error;
  const bool hit = geometry::cgal_mesh_do_intersect_3(*a, *b, error);
  if (!error.empty() && !hit) {
    params.error_message_add(NodeWarningType::Warning, error);
  }
  params.set_output("Intersect"_ustr, hit);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalDoIntersect3"_ustr, GEO_NODE_CGAL_DO_INTERSECT_3);
  ntype.ui_name = "Do Intersect 3D";
  ntype.ui_description =
      "Test whether two triangle meshes intersect (CGAL PMP::do_intersect). "
      "Different from Does Self Intersect (one mesh) and Mesh Intersection (curves).";
  ntype.enum_name_legacy = "CGAL_DO_INTERSECT_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_do_intersect_3_cc
