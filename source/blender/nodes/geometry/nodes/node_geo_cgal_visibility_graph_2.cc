/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Visibility graph of XY obstacle segments (walls plus non-crossing chords).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_visibility_graph_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Walls"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("XY obstacle segments: mesh edges and/or face borders.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Wire: original walls plus mutually visible vertex chords.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Walls"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 2) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Visibility Graph 2D needs wall edges or faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_visibility_graph_2(*mesh, error);
  if (!out || out->edges_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Visibility Graph 2D failed") : error);
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
  /* Deleted: Visibility Graph 2D. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalVisibilityGraph2"_ustr, GEO_NODE_CGAL_VISIBILITY_GRAPH_2);
  ntype.ui_name = "Visibility Graph 2D";
  ntype.ui_description =
      "Visibility graph of XY obstacle segments (CGAL Overmars–Welzl). "
      "Keeps the walls and adds every vertex-vertex chord that does not cross "
      "a wall. Visibility 2D is the visible polygon from one query point.";
  ntype.enum_name_legacy = "CGAL_VISIBILITY_GRAPH_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_visibility_graph_2_cc
