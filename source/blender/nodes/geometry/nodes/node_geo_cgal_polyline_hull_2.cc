/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Melkman convex hull of an ordered XY polyline / polygon ring.
 * Distinct from deleted Convex Hull 2D (unordered point set).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_polyline_hull_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Ordered XY polygon rings, or a polyline wire.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Convex hull n-gon per ring. Face attribute island_id.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 3) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Polyline Hull 2D needs a polygon or polyline mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_polyline_hull_2(*mesh, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Polyline Hull 2D failed") : error);
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
  /* Deleted: Polyline Hull 2D (legacy 2594; hull family). */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalPolylineHull2"_ustr, GEO_NODE_CGAL_POLYLINE_HULL_2);
  ntype.ui_name = "Polyline Hull 2D";
  ntype.ui_description =
      "Convex hull of each ordered XY ring (CGAL ch_melkman). "
      "Needs a polygon or polyline, not an unordered point cloud.";
  ntype.enum_name_legacy = "CGAL_POLYLINE_HULL_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_polyline_hull_2_cc
