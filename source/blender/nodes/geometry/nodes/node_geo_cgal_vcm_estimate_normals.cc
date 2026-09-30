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

namespace blender::nodes::node_geo_cgal_vcm_estimate_normals_cc {

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
      .description("Point cloud or mesh verts.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Same points with float3 attribute \"normal\" (VCM).");
  b.add_input<decl::Float>("Offset Radius"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("VCM offset radius (local ball for covariance). Start at a few times mean spacing.");
  b.add_input<decl::Float>("Convolution Radius"_ustr)
      .default_value(0.2f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Smoothing radius for VCM (>= Offset). Larger = smoother normals, less feature fidelity.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float offset = params.extract_input<float>("Offset Radius"_ustr);
  const float conv = params.extract_input<float>("Convolution Radius"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_vcm_estimate_normals(
      pts, offset, conv, geometry.get_pointcloud(), geometry.get_mesh(), error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("VCM Estimate Normals failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalVcmEstimateNormals"_ustr, GEO_NODE_CGAL_VCM_ESTIMATE_NORMALS);
  ntype.ui_name = "VCM Estimate Normals";
  ntype.ui_description =
      "Estimate normals with Voronoi Covariance Measure (feature-aware), then MST orient. Often better than Jet near sharp edges. Radii are absolute scene units.";
  ntype.enum_name_legacy = "CGAL_VCM_ESTIMATE_NORMALS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_vcm_estimate_normals_cc
