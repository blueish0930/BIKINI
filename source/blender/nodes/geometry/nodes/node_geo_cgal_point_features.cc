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

namespace blender::nodes::node_geo_cgal_point_features_cc {


static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Dense point cloud (or mesh verts). A handful of cube corners will look uniform.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Same points with omnivariance, sphericity, anisotropy. Color by Named Attribute.");
  b.add_input<decl::Int>("Neighbors"_ustr)
      .default_value(24)
      .min(4)
      .max(128)
      .description("k-nearest neighbors for the local covariance.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Points"_ustr);
  const int knn = params.extract_input<int>("Neighbors"_ustr);
  Span<float3> pts;
  if (const PointCloud *pc = g.get_pointcloud()) pts = pc->positions();
  else if (const Mesh *m = g.get_mesh()) pts = m->vert_positions();
  if (pts.size() < 4) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_classification_features(pts, knn, g.get_pointcloud(), g.get_mesh(), error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Point Features failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}


static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalPointFeatures"_ustr, GEO_NODE_CGAL_POINT_FEATURES);
  ntype.ui_name = "Point Features";
  ntype.ui_description =
      "Local PCA eigen-features (Weinmann). Writes only omnivariance, sphericity, and "
      "anisotropy (anisotropy = 1 − sphericity). Color by Named Attribute.";
  ntype.enum_name_legacy = "CGAL_POINT_FEATURES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_point_features_cc
