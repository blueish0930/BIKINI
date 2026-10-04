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

namespace blender::nodes::node_geo_cgal_cluster_point_set_cc {


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
      .description("Point cloud (or mesh verts) to cluster.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Same points with int attribute \"Cluster\" (0,1,2...).");
  b.add_output<decl::Int>("Cluster Count"_ustr)
      .description("How many connected clusters were found.");
  b.add_input<decl::Float>("Neighbor Radius"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Connect two points if distance <= this (scene units). 0 = auto (1% of bbox diagonal). Too small -> many single-point clusters; too large -> one cluster.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float radius = params.extract_input<float>("Neighbor Radius"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.is_empty()) {
    params.set_output("Points"_ustr, GeometrySet());
    params.set_output("Cluster Count"_ustr, 0);
    return;
  }
  std::string error;
  int count = 0;
  PointCloud *pc = geometry::cgal_points_cluster(
      pts, radius, geometry.get_pointcloud(), count, error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Cluster Point Set failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    params.set_output("Cluster Count"_ustr, 0);
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
  params.set_output("Cluster Count"_ustr, count);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalClusterPointSet"_ustr, GEO_NODE_CGAL_CLUSTER_POINT_SET);
  ntype.ui_name = "Cluster Point Set";
  ntype.ui_description =
      "Spatial clustering: link points closer than Neighbor Radius and label each connected component with int attribute \"Cluster\" (0,1,2...). Does not move or delete points. Use Named Attribute + Separate Geometry to split groups.";
  ntype.enum_name_legacy = "CGAL_CLUSTER_POINT_SET";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_cluster_point_set_cc
