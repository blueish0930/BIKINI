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

namespace blender::nodes::node_geo_cgal_hierarchy_simplify_cc {

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
      .description("Input point set.");
  b.add_output<decl::Geometry>("Points"_ustr).propagate_all_geometry().align_with_previous()
      .description("Simplified point subset.");
  b.add_input<decl::Int>("Cluster Size"_ustr)
      .default_value(10)
      .min(1)
      .max(10000)
      .description("Target points per leaf cluster (larger = stronger simplify).");
  b.add_input<decl::Float>("Max Variation"_ustr)
      .default_value(0.333f)
      .min(0.001f)
      .max(0.333f)
      .description("Max spatial variation inside a cluster (0 = size-only; larger allows bigger clusters on flats).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int size = params.extract_input<int>("Cluster Size"_ustr);
  const float var = params.extract_input<float>("Max Variation"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_hierarchy_simplify(
      pts, size, var, geometry.get_pointcloud(), error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Hierarchy Simplify failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalHierarchySimplify"_ustr, GEO_NODE_CGAL_HIERARCHY_SIMPLIFY);
  ntype.ui_name = "Hierarchy Simplify";
  ntype.ui_description =
      "Hierarchical cluster simplification: recursively merge nearby points until clusters reach Cluster Size / variation limit.";
  ntype.enum_name_legacy = "CGAL_HIERARCHY_SIMPLIFY";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_hierarchy_simplify_cc
