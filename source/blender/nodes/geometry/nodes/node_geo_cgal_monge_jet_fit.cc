/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Per-point Monge form via jet fitting (fast double k-NN + Simple_cartesian jet).
 * Preserves input point attributes; writes monge_* attributes.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_monge_jet_fit_cc {

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
      .description("Point cloud or mesh verts. Existing attributes are preserved.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Same points (attrs kept) + monge_k1/k2, monge_d1/d2, monge_normal.");
  b.add_input<decl::Int>("Neighbors"_ustr)
      .default_value(12)
      .min(6)
      .max(100000)
      .description(
          "k nearest neighbors for the local jet (hash-grid kNN, multi-threaded). "
          "Larger = smoother but slower SVD per point. Must be ≥ (d+1)(d+2)/2 for Fitting Degree d "
          "(degree 2 → ≥6). Clamped to point count.");
  b.add_input<decl::Int>("Fitting Degree"_ustr)
      .default_value(2)
      .min(1)
      .max(4)
      .description("Polynomial jet degree (2 is usual; 3–4 much slower).");
  b.add_input<decl::Int>("Monge Degree"_ustr)
      .default_value(2)
      .min(1)
      .max(4)
      .description("Monge form order (≤ Fitting Degree). 2 gives principal curvatures.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int knn = params.extract_input<int>("Neighbors"_ustr);
  const int deg_fit = params.extract_input<int>("Fitting Degree"_ustr);
  const int deg_monge = params.extract_input<int>("Monge Degree"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 6) {
    params.error_message_add(NodeWarningType::Info, TIP_("Monge Jet Fit needs at least 6 points"));
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }

  std::string error;
  PointCloud *pc = geometry::cgal_points_monge_jet_fit_pc(
      pts, knn, deg_fit, deg_monge, geometry.get_pointcloud(), geometry.get_mesh(), error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Monge Jet Fit failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMongeJetFit"_ustr, GEO_NODE_CGAL_MONGE_JET_FIT);
  ntype.ui_name = "Monge Jet Fit";
  ntype.ui_description =
      "Local Monge form (k1/k2, principal directions, normal) via jet fitting on k nearest "
      "neighbors (double-precision kd-tree). Preserves input attributes; writes "
      "monge_k1/k2, monge_d1/d2, monge_normal.";
  ntype.enum_name_legacy = "CGAL_MONGE_JET_FIT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_monge_jet_fit_cc
