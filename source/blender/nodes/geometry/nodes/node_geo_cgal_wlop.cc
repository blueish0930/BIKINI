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

namespace blender::nodes::node_geo_cgal_wlop_cc {

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
      .description("Noisy/dense point cloud.");
  b.add_output<decl::Geometry>("Points"_ustr).propagate_all_geometry().align_with_previous()
      .description("Simplified/regularized point subset.");
  b.add_input<decl::Float>("Percent"_ustr)
      .default_value(5.0f)
      .min(0.1f)
      .max(100.0f)
      .description("Keep percentage of points (0-100). 5-20 is common.");
  b.add_input<decl::Float>("Neighbor Radius"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Local neighborhood radius (scene units). Use a positive size relative to model scale.");
  b.add_input<decl::Int>("Iterations"_ustr).default_value(35).min(1).max(200)
      .description("Projection iterations. More = more regular, slower.");
  b.add_input<decl::Bool>("Uniform Sampling"_ustr)
      .default_value(false)
      .description("Prefer more uniform spacing in the result.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float percent = params.extract_input<float>("Percent"_ustr);
  const float radius = params.extract_input<float>("Neighbor Radius"_ustr);
  const int iterations = params.extract_input<int>("Iterations"_ustr);
  const bool uniform = params.extract_input<bool>("Uniform Sampling"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_wlop(
      pts, percent, radius, iterations, uniform, geometry.get_pointcloud(), error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("WLOP failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalWLOP"_ustr, GEO_NODE_CGAL_WLOP);
  ntype.ui_name = "WLOP";
  ntype.ui_description =
      "WLOP: simplify and regularize a point set (Weighted Locally Optimal Projection). Output count is about Percent of input.";
  ntype.enum_name_legacy = "CGAL_WLOP";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_wlop_cc
