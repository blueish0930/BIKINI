/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_remove_outliers_cc {

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
      .description("Point set that may contain outliers.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Inlier points only (count reduced).");
  b.add_input<decl::Int>("Neighbors"_ustr).default_value(24).min(6).max(128).description(
      "k-nearest neighborhood for distance ranking.");
  b.add_input<decl::Float>("Percent"_ustr)
      .default_value(5.0f)
      .min(0.0f)
      .max(100.0f)
      .description("Percentage of points to remove as outliers (0-100).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int k = params.extract_input<int>("Neighbors"_ustr);
  const float percent = params.extract_input<float>("Percent"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_remove_outliers(pts, k, percent, error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Remove Outliers failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalRemoveOutliers"_ustr, GEO_NODE_CGAL_REMOVE_OUTLIERS);
  ntype.ui_name = "Remove Outliers";
  ntype.ui_description =
      "Remove the farthest percent of points by average k-neighbor distance "
      "(CGAL remove_outliers). Useful before reconstruction.";
  ntype.enum_name_legacy = "CGAL_REMOVE_OUTLIERS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_remove_outliers_cc
