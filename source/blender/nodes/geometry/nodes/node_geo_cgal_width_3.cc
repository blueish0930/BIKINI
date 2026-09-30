/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL Width_3 — minimal width of a 3D point set.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_width_3_cc {

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
      .description("Points or mesh vertices. The convex-hull width is computed.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Two supporting plane quads.");
  b.add_output<decl::Float>("Width"_ustr).description("Distance between the two supporting planes.");
  b.add_output<decl::Vector>("Center"_ustr).description("Midpoint between the two supporting planes.");
  b.add_input<decl::Float>("Plane Scale"_ustr)
      .default_value(1.0f)
      .min(0.01f)
      .description("Half-size of supporting plane quads.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float plane_scale = params.extract_input<float>("Plane Scale"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info, TIP_("Width 3D needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Width"_ustr, 0.0f);
    params.set_output("Center"_ustr, float3(0.0f));
    return;
  }
  std::string error;
  float width = 0.0f;
  float3 center(0.0f);
  Mesh *out = geometry::cgal_points_width_3(pts, plane_scale, width, center, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Width 3D failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Width"_ustr, 0.0f);
    params.set_output("Center"_ustr, float3(0.0f));
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Width"_ustr, width);
  params.set_output("Center"_ustr, center);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalWidth3"_ustr, GEO_NODE_CGAL_WIDTH_3);
  ntype.ui_name = "Width 3D";
  ntype.ui_description =
      "Minimal width of a 3D point set with parallel supporting planes (CGAL Width_3).";
  ntype.enum_name_legacy = "CGAL_WIDTH_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_width_3_cc
