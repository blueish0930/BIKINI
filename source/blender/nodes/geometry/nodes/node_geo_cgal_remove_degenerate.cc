/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_remove_degenerate_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh that may contain degenerates.");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous()
      .description("Mesh without degenerate faces/edges.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m) { params.set_output("Mesh"_ustr, GeometrySet()); return; }
  std::string error;
  Mesh *out = geometry::cgal_mesh_remove_degenerate(*m, error);
  if (!out || (out->faces_num == 0 && out->verts_num == 0)) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("CGAL operation failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalRemoveDegenerate"_ustr, GEO_NODE_CGAL_REMOVE_DEGENERATE);
  ntype.ui_name = "Remove Degenerate";
  ntype.ui_description =
      "Delete zero-area faces and zero-length edges that break CGAL algorithms.";
  ntype.enum_name_legacy = "CGAL_REMOVE_DEGENERATE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_remove_degenerate_cc
