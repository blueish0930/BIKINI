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

namespace blender::nodes::node_geo_cgal_super4pcs_cc {


static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Source"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Moving cloud.");
  b.add_input<decl::Geometry>("Target"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Fixed cloud.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Source after global 4-point alignment + ICP refine. Source point attributes are kept.");
  b.add_input<decl::Int>("Samples"_ustr)
      .default_value(80)
      .min(8)
      .max(400)
      .description("Random congruent bases to try. More = slower, more chance of a good rotation.");
  b.add_input<decl::Int>("ICP Iterations"_ustr)
      .default_value(12)
      .min(1)
      .max(80)
      .description("ICP refine steps after the global 4-point guess.");
}
static Span<float3> pts_of(const GeometrySet &g)
{
  if (const PointCloud *pc = g.get_pointcloud()) return pc->positions();
  if (const Mesh *m = g.get_mesh()) return m->vert_positions();
  return {};
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet gs = params.extract_input<GeometrySet>("Source"_ustr);
  GeometrySet gt = params.extract_input<GeometrySet>("Target"_ustr);
  Span<float3> src = pts_of(gs), tgt = pts_of(gt);
  if (src.size() < 4 || tgt.size() < 4) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_register_4pcs(src,
                                                       tgt,
                                                       params.extract_input<int>("Samples"_ustr),
                                                       params.extract_input<int>("ICP Iterations"_ustr),
                                                       gs.get_pointcloud(),
                                                       gs.get_mesh(),
                                                       error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Super4PCS failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}


static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalSuper4pcs"_ustr, GEO_NODE_CGAL_SUPER4PCS);
  ntype.ui_name = "4-Point Congruent Sets";
  ntype.ui_description =
      "Global rigid registration (Super4PCS-style): sample congruent 4-point bases on Source and "
      "Target, pick the transform with most inliers, then refine with ICP. Handles large rotations "
      "that Iterative Closest Point cannot. Search names: 4PCS, Super4PCS, global align.";
  ntype.enum_name_legacy = "CGAL_SUPER4PCS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_super4pcs_cc
