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

namespace blender::nodes::node_geo_cgal_register_icp_cc {


static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Source"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Point set (or mesh verts) to move.");
  b.add_input<decl::Geometry>("Target"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Fixed cloud that Source is aligned onto.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Source points after the rigid ICP transform. Source point attributes are kept.");
  b.add_input<decl::Int>("Iterations"_ustr)
      .default_value(20)
      .min(1)
      .max(80)
      .description("Closest-point + Kabsch cycles. More is slower, usually 10–40 is enough.");
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
  if (src.size() < 3 || tgt.size() < 3) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_register_icp(src,
                                                     tgt,
                                                     params.extract_input<int>("Iterations"_ustr),
                                                     gs.get_pointcloud(),
                                                     gs.get_mesh(),
                                                     error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("ICP failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}


static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalRegisterIcp"_ustr, GEO_NODE_CGAL_REGISTER_ICP);
  ntype.ui_name = "Iterative Closest Point";
  ntype.ui_description =
      "Iterative Closest Point (ICP): rigid-align Source onto Target by repeatedly pairing each "
      "source point to its nearest target neighbor and solving a Kabsch rotation+translation. "
      "Needs a rough initial overlap; for large rotations use 4-Point Congruent Sets first.";
  ntype.enum_name_legacy = "CGAL_REGISTER_ICP";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_register_icp_cc
