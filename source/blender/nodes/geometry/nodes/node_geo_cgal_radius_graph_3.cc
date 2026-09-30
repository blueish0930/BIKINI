/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Connect 3D points whose Euclidean distance is at most Radius.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_radius_graph_3_cc {

static Span<float3> positions_from_geometry(const GeometrySet &geometry)
{
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    return pc->positions();
  }
  if (const Mesh *mesh = geometry.get_mesh()) {
    return mesh->vert_positions();
  }
  return {};
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("3D sites.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Wire mesh: an edge whenever two points are within Radius.");
  b.add_input<decl::Float>("Radius"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Connect pairs closer than this distance.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float radius = params.extract_input<float>("Radius"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 2) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Radius Graph 3D needs at least 2 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_radius_graph_3(pts, radius, error);
  if (!out || out->edges_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Radius Graph 3D failed") : error);
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
  /* Merged into Neighbor Graph 3D (GeometryNodeCgalKnnGraph3). */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalRadiusGraph3"_ustr, GEO_NODE_CGAL_RADIUS_GRAPH_3);
  ntype.ui_name = "Radius Graph 3D";
  ntype.ui_description =
      "Connect every pair of 3D points closer than Radius "
      "(CGAL Kd_tree + Fuzzy_sphere). Output is a wire mesh. "
      "Unlike KNN Graph this uses a distance threshold, not a neighbor count.";
  ntype.enum_name_legacy = "CGAL_RADIUS_GRAPH_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_radius_graph_3_cc
