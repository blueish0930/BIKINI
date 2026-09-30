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

namespace blender::nodes::node_geo_cgal_average_spacing_cc {

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
      .description("Point cloud or mesh vertices whose local spacing is measured.");
  b.add_output<decl::Float>("Spacing"_ustr)
      .description(
          "Average distance from each point to its k nearest neighbors (scene units). "
          "Use this as Alpha / Radius / Edge Length on other CGAL nodes.");
  b.add_input<decl::Int>("Neighbors"_ustr)
      .default_value(6)
      .min(1)
      .max(64)
      .description("k nearest neighbors per point (typical 6). Larger = smoother global estimate.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int k = params.extract_input<int>("Neighbors"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.set_output("Spacing"_ustr, 0.0f);
    return;
  }
  std::string error;
  const float spacing = geometry::cgal_points_average_spacing(pts, k, error);
  if (!error.empty()) {
    params.error_message_add(NodeWarningType::Warning, error);
  }
  params.set_output("Spacing"_ustr, spacing);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalAverageSpacing"_ustr, GEO_NODE_CGAL_AVERAGE_SPACING);
  ntype.ui_name = "Average Spacing";
  ntype.ui_description =
      "Measure the typical distance between neighboring points "
      "(CGAL compute_average_spacing). How to use: plug a Point Cloud → "
      "feed Spacing into Alpha Wrap, WLOP, Poisson, or Isotropic Remesh "
      "instead of guessing a radius. Neighbors is the k used for the local average.";
  ntype.enum_name_legacy = "CGAL_AVERAGE_SPACING";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_average_spacing_cc
