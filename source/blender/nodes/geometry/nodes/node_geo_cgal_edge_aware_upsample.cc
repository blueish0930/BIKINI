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

namespace blender::nodes::node_geo_cgal_edge_aware_upsample_cc {

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
      .description("Input points; uses attribute \"normal\" if present, else estimates.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Original points plus new samples (densified cloud with normals).");
  b.add_input<decl::Int>("Number of Points"_ustr)
      .default_value(2000)
      .min(1)
      .max(10000000)
      .description("Target total point count after densify (must be greater than input). Example: 6k in -> set 12k for about 2x density.");
  b.add_input<decl::Float>("Sharpness Angle"_ustr)
      .default_value(30.0f)
      .min(0.0f)
      .max(90.0f)
      .description("Feature angle in degrees. Larger keeps softer transitions; smaller preserves sharper creases.");
  b.add_input<decl::Float>("Edge Sensitivity"_ustr)
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .description("0 = fairly uniform densify; 1 = strongly prefer sharp edges (new points pile on creases).");
  b.add_input<decl::Float>("Neighbor Radius"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Neighborhood for densify (0 = auto ~3x average spacing). Too small -> few/no new points.");
  b.add_input<decl::Int>("Normal Neighbors"_ustr)
      .default_value(18)
      .min(6)
      .max(128)
      .description("k for jet normal estimate when normals are missing.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int count = params.extract_input<int>("Number of Points"_ustr);
  const float sharpness = params.extract_input<float>("Sharpness Angle"_ustr);
  const float edge_sens = params.extract_input<float>("Edge Sensitivity"_ustr);
  const float radius = params.extract_input<float>("Neighbor Radius"_ustr);
  const int nk = params.extract_input<int>("Normal Neighbors"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  if (count <= int(pts.size())) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("Number of Points should be greater than the input count (target total after densify)"));
  }
  Span<float3> normals;
  Array<float3> normals_buf;
  const PointCloud *src_pc = geometry.get_pointcloud();
  if (src_pc) {
    if (const VArray n_varray = *src_pc->attributes().lookup<float3>("normal")) {
      if (n_varray.size() == pts.size()) {
        normals_buf.reinitialize(pts.size());
        n_varray.materialize(normals_buf.as_mutable_span());
        normals = normals_buf.as_span();
      }
    }
  }
  std::string error;
  PointCloud *out = geometry::cgal_points_edge_aware_upsample(pts,
                                                              normals,
                                                              count,
                                                              sharpness,
                                                              edge_sens,
                                                              radius,
                                                              nk,
                                                              src_pc,
                                                              geometry.get_mesh(),
                                                              error);
  if (!out) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Edge Aware Upsample failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  if (!error.empty()) {
    params.error_message_add(NodeWarningType::Info, error);
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalEdgeAwareUpsample"_ustr, GEO_NODE_CGAL_EDGE_AWARE_UPSAMPLE);
  ntype.ui_name = "Edge Aware Upsample";
  ntype.ui_description =
      "Densify a point set: output = originals + new samples. Number of Points is the target TOTAL count. Edge Sensitivity concentrates samples on creases. Needs good normals.";
  ntype.enum_name_legacy = "CGAL_EDGE_AWARE_UPSAMPLE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_edge_aware_upsample_cc
