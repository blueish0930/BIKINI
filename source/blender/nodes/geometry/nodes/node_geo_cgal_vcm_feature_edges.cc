/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL VCM feature edges — mark points on sharp features.
 * Writes float "Feature" (0/1), "Feature Ratio", and ".selection".
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

#include <string>

namespace blender::nodes::node_geo_cgal_vcm_feature_edges_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Point cloud or mesh verts.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Same points (or only features) with Feature / Feature Ratio / .selection attributes.");
  b.add_input<decl::Float>("Offset Radius"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description("VCM offset radius. 0 = auto from average point spacing * 2.");
  b.add_input<decl::Float>("Convolution Radius"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description("VCM convolution radius. 0 = auto from average spacing * 6.");
  b.add_input<decl::Float>("Threshold"_ustr)
      .default_value(0.1f)
      .min(0.001f)
      .max(0.999f)
      .description(
          "Eigenvalue ratio threshold (Feature when ratio >= Threshold). "
          "Lower marks more points as features. Inspect \"Feature Ratio\" attribute.");
  b.add_input<decl::Bool>("Only Features"_ustr)
      .default_value(false)
      .description("If true, output only feature points (empty if none).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float offset = params.extract_input<float>("Offset Radius"_ustr);
  const float conv = params.extract_input<float>("Convolution Radius"_ustr);
  const float thr = params.extract_input<float>("Threshold"_ustr);
  const bool only_features = params.extract_input<bool>("Only Features"_ustr);
  const PointCloud *src_pc = geometry.get_pointcloud();
  const Mesh *src_mesh = geometry.get_mesh();
  Span<float3> pts;
  if (src_pc) {
    pts = src_pc->positions();
  }
  else if (src_mesh) {
    pts = src_mesh->vert_positions();
  }
  if (pts.size() < 3) {
    params.error_message_add(NodeWarningType::Info, TIP_("VCM Feature Edges needs points"));
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  int feature_count = 0;
  PointCloud *out = geometry::cgal_points_vcm_feature_edges(
      pts, offset, conv, thr, only_features, src_pc, src_mesh, feature_count, error);
  if (!out) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("VCM Feature Edges failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  if (feature_count == 0) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("No feature points: lower Threshold, or set Offset/Convolution Radius "
             "to ~point spacing (0 = auto). Check spreadsheet attribute \"Feature Ratio\"."));
  }
  else {
    const std::string msg = "Feature points: " + std::to_string(feature_count) + " / " +
                            std::to_string(int(pts.size()));
    params.error_message_add(NodeWarningType::Info, msg);
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalVcmFeatureEdges"_ustr, GEO_NODE_CGAL_VCM_FEATURE_EDGES);
  ntype.ui_name = "VCM Feature Edges";
  ntype.ui_description =
      "Mark sharp-feature points via Voronoi Covariance Measure (CGAL VCM). "
      "Writes float Feature (0/1), Feature Ratio, and .selection. "
      "Does not change positions — use Separate Geometry on .selection, "
      "or enable Only Features.";
  ntype.enum_name_legacy = "CGAL_VCM_FEATURE_EDGES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_vcm_feature_edges_cc
