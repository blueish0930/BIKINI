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

namespace blender::nodes::node_geo_cgal_constrained_dt3_cc {


static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Closed triangle PLC. Only existing vertices are used (no Steiner grid).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "One tetrahedron per interior Delaunay cell (4 triangles, unscaled). Face attribute "
          "Tet = cell id. Point attributes copy from the source.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m || m->faces_num < 1) {
    params.error_message_add(NodeWarningType::Info, TIP_("Constraint Delaunay 3D needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  
  std::string error;
  Mesh *out = geometry::cgal_mesh_constrained_delaunay_3(*m, error);
  if (!out || (out->faces_num == 0 && out->edges_num == 0)) {
    params.error_message_add(
        NodeWarningType::Warning, error.empty() ? TIP_("Constraint Delaunay 3D failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}


static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalConstrainedDt3"_ustr, GEO_NODE_CGAL_CONSTRAINED_DT3);
  ntype.ui_name = "Constraint Delaunay 3D";
  ntype.ui_description =
      "Interior Delaunay tetrahedra of a closed triangle mesh using only the input vertices "
      "(no Steiner points). Each tet is a separate 4-triangle island at full scale. Face "
      "attribute Tet. Point attributes copy from the source. Distinct from Tet Remesh which "
      "inserts an interior grid.";
  ntype.enum_name_legacy = "CGAL_CONSTRAINED_DT3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_constrained_dt3_cc
