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

namespace blender::nodes::node_geo_cgal_alpha_wrap_2_cc {


static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("XY points (or mesh verts). Z is ignored.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("2D alpha-shape interior, optionally offset. Stand-in for CGAL 6.2 Alpha_wrap_2.");
  b.add_input<decl::Float>("Alpha"_ustr)
      .default_value(0.1f)
      .min(1e-6f)
      .description("Alpha radius. Larger = coarser wrap.");
  b.add_input<decl::Float>("Offset"_ustr)
      .default_value(0.01f)
      .min(0.0f)
      .description("Outward offset of the alpha-shape boundary.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Geometry"_ustr);
  const float alpha = params.extract_input<float>("Alpha"_ustr);
  const float offset = params.extract_input<float>("Offset"_ustr);
  Span<float3> pts;
  const Mesh *mesh = g.get_mesh();
  if (const PointCloud *pc = g.get_pointcloud()) pts = pc->positions();
  else if (mesh) pts = mesh->vert_positions();
  std::string error;
  Mesh *out = geometry::cgal_points_alpha_wrap_2(pts, mesh, alpha, offset, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("Alpha Wrap 2D failed") : error);
    if (out) BKE_id_free(nullptr, out);
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}


static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalAlphaWrap2"_ustr, GEO_NODE_CGAL_ALPHA_WRAP_2);
  ntype.ui_name = "Alpha Wrap 2D";
  ntype.ui_description = "Watertight 2D wrap of XY points or segments (CGAL 6.2 Alpha_wrap_2).";
  ntype.enum_name_legacy = "CGAL_ALPHA_WRAP_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_alpha_wrap_2_cc
