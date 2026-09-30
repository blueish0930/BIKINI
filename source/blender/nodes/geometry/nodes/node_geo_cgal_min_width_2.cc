/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Minimal caliper width of points projected to a plane.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_min_width_2_cc {

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
      .description("Points whose XY convex hull is measured (point cloud or mesh verts).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Preview quad spanning the two parallel supporting lines.");
  b.add_output<decl::Float>("Width"_ustr)
      .description("Minimal caliper width (distance between the two supporting lines).");
  b.add_output<decl::Vector>("Center"_ustr)
      .description("Midpoint of the min-width strip.");
  b.add_input<decl::Float>("Plane Scale"_ustr)
      .default_value(1.0f)
      .min(0.01f)
      .description("Half-length of the supporting-strip quad.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float plane_scale = params.extract_input<float>("Plane Scale"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 2) {
    params.error_message_add(NodeWarningType::Info, TIP_("Min Width 2D needs at least 2 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Width"_ustr, 0.0f);
    params.set_output("Center"_ustr, float3(0.0f));
    return;
  }
  std::string error;
  float width = 0.0f;
  float3 center(0.0f);
  Mesh *out = geometry::cgal_points_min_width_2(pts, 0, plane_scale, width, center, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Min Width 2D failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalMinWidth2"_ustr, GEO_NODE_CGAL_MIN_WIDTH_2);
  ntype.ui_name = "Min Width 2D";
  ntype.ui_description =
      "Minimal caliper width of the XY convex hull (rotating calipers): "
      "the smallest gap between two parallel supporting lines. "
      "How to use: plug Points → read Width, or look at the preview strip. "
      "Complementary to Width 3D (two supporting planes in space).";
  ntype.enum_name_legacy = "CGAL_MIN_WIDTH_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_min_width_2_cc
