/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_repair_self_intersect_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Self-intersecting mesh to repair.");

  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous()
      .description("Mesh after attempting to remove self-intersections.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  std::string error;
  Mesh *mesh = geometry::cgal_mesh_repair_self_intersections(*mesh_in, error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("CGAL Repair Self Intersections failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalRepairSelfIntersect"_ustr, GEO_NODE_CGAL_REPAIR_SELF_INTERSECT);
  ntype.ui_name = "Repair Self Intersections";
  ntype.ui_description =
      "Attempt to remove self-intersections (CGAL PMP). Not always perfect on severe tangles.";
  ntype.enum_name_legacy = "CGAL_REPAIR_SELF_INTERSECT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_repair_self_intersect_cc
