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

namespace blender::nodes::node_geo_cgal_jet_smooth_cc {

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
      .description("Noisy point set (mesh verts also accepted).");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Smoothed points (same count).");
  b.add_input<decl::Int>("Neighbors"_ustr).default_value(24).min(6).max(128).description(
      "k-nearest neighborhood size.");
  b.add_input<decl::Int>("Iterations"_ustr).default_value(1).min(1).max(50).description(
      "Smoothing passes.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int k = params.extract_input<int>("Neighbors"_ustr);
  const int iterations = params.extract_input<int>("Iterations"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_jet_smooth(pts, k, iterations, error);
  if (!pc || pc->totpoint == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Jet Smooth failed") : error);
    if (pc) {
      BKE_id_free(nullptr, pc);
    }
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalJetSmooth"_ustr, GEO_NODE_CGAL_JET_SMOOTH);
  ntype.ui_name = "Jet Smooth";
  ntype.ui_description =
      "Smooth a point set with jet fitting (CGAL jet_smooth_point_set). Good denoising before "
      "normals/reconstruction; may blur sharp features (prefer Bilateral Smooth for creases).";
  ntype.enum_name_legacy = "CGAL_JET_SMOOTH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_jet_smooth_cc
