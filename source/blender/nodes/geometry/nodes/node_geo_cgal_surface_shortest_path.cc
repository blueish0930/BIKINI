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

namespace blender::nodes::node_geo_cgal_surface_shortest_path_cc {



static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh)
      .description("Surface to travel on.");
  b.add_output<decl::Geometry>("Path"_ustr).propagate_all_geometry().align_with_previous()
      .description("Wire mesh of the geodesic shortest path.");
  b.add_input<decl::Vector>("Source"_ustr).default_value({0, 0, 0}).subtype(PROP_TRANSLATION)
      .description("Start position (snapped to nearest surface location).");
  b.add_input<decl::Vector>("Target"_ustr).default_value({1, 0, 0}).subtype(PROP_TRANSLATION)
      .description("End position (snapped to nearest surface location).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float3 src = params.extract_input<float3>("Source"_ustr);
  const float3 dst = params.extract_input<float3>("Target"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m) {
    params.set_output("Path"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_surface_shortest_path(*m, src, dst, error);
  if (!out || out->edges_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Surface Shortest Path failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Path"_ustr, GeometrySet());
    return;
  }
  params.set_output("Path"_ustr, GeometrySet::from_mesh(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalSurfaceShortestPath"_ustr, GEO_NODE_CGAL_SURFACE_SHORTEST_PATH);
  ntype.ui_name = "Surface Shortest Path";
  ntype.ui_description =
      "Geodesic (surface) shortest path between Source and Target, snapped to the mesh. Output is a wire/path mesh along the surface.";
  ntype.enum_name_legacy = "CGAL_SURFACE_SHORTEST_PATH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_surface_shortest_path_cc
