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

namespace blender::nodes::node_geo_cgal_autorefine_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh)
      .description("Possibly self-intersecting mesh.");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous()
      .description("Mesh refined along self-intersection curves.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m) { params.set_output("Mesh"_ustr, GeometrySet()); return; }
  std::string error;
  Mesh *out = geometry::cgal_mesh_autorefine(*m, error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalAutorefine"_ustr, GEO_NODE_CGAL_AUTOREFINE);
  ntype.ui_name = "Autorefine";
  ntype.ui_description =
      "Refine a mesh at self-intersections so intersection curves become explicit edges (PMP autorefine). Helps before booleans/cleanup.";
  ntype.enum_name_legacy = "CGAL_AUTOREFINE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_autorefine_cc
