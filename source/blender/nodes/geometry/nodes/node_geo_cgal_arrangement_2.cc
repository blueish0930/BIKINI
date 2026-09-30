/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Find all closed 2D cells from a planar edge graph (CGAL Arrangement_2).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_arrangement_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("2D edges (wire or mesh). Faces are optional; crossings are split.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Every bounded loop as a welded n-gon. Face attribute cell_id.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in || mesh_in->verts_num < 2) {
    params.error_message_add(NodeWarningType::Info, TIP_("Find All Cells 2D needs edges"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_arrangement_2(*mesh_in, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Find All Cells 2D failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalArrangement2"_ustr, GEO_NODE_CGAL_ARRANGEMENT_2);
  ntype.ui_name = "Find All Cells 2D";
  ntype.ui_description =
      "From a 2D edge graph, find every closed loop (CGAL Arrangement_2). "
      "Crossing edges get a vertex; each bounded cell is an n-gon that shares "
      "verts with neighbors. Works with a pure wire / edge mesh. Face attribute cell_id.";
  ntype.enum_name_legacy = "CGAL_ARRANGEMENT_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_arrangement_2_cc
