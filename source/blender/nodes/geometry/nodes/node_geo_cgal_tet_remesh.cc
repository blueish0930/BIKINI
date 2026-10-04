/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_tet_remesh_cc {


static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Closed triangle solid whose interior should be remeshed as tets.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Interior tetrahedra sized by Target Edge.");
  b.add_input<decl::Float>("Target Edge"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .description("Target Steiner spacing. Smaller = more tets.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m || m->faces_num < 1) {
    params.error_message_add(NodeWarningType::Info, TIP_("Tet Remesh needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  const float te = params.extract_input<float>("Target Edge"_ustr);
  std::string error;
  Mesh *out = geometry::cgal_mesh_tet_remesh(*m, te, error);
  if (!out || (out->faces_num == 0 && out->edges_num == 0)) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Tet Remesh failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}


static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalTetRemesh"_ustr, GEO_NODE_CGAL_TET_REMESH);
  ntype.ui_name = "Tet Remesh";
  ntype.ui_description =
      "Tetrahedral remesh of a closed solid: Steiner points on an interior grid, keeping "
      "Delaunay tets whose circumcenter is inside. Target Edge controls spacing.";
  ntype.enum_name_legacy = "CGAL_TET_REMESH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_tet_remesh_cc
