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

namespace blender::nodes::node_geo_cgal_alpha_shape_2_cc {

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
      .description("XY point set (point cloud or mesh verts). Z is ignored.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Filled 2D alpha-shape triangles of the point set.");
  b.add_output<decl::Float>("Alpha Used"_ustr)
      .description("Actual alpha used (manual Alpha, or the auto-picked value when Optimal is on).");
  b.add_input<decl::Float>("Alpha"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .description(
          "Manual alpha radius (scene units). Smaller = tighter / more holes; "
          "larger = smoother bulk. Ignored when Optimal is on.");
  b.add_input<decl::Bool>("Optimal"_ustr)
      .default_value(false)
      .description("On: ignore Alpha and auto-pick a value that yields one solid 2D component.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float alpha = params.extract_input<float>("Alpha"_ustr);
  const bool optimal = params.extract_input<bool>("Optimal"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.error_message_add(NodeWarningType::Info, TIP_("Alpha Shape 2D needs at least 3 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Alpha Used"_ustr, 0.0f);
    return;
  }
  std::string error;
  float alpha_used = 0.0f;
  Mesh *out = geometry::cgal_points_alpha_shape_2(pts, 0, alpha, optimal, alpha_used, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Alpha Shape 2D failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Alpha Used"_ustr, 0.0f);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Alpha Used"_ustr, alpha_used);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalAlphaShape2"_ustr, GEO_NODE_CGAL_ALPHA_SHAPE_2);
  ntype.ui_name = "Alpha Shape 2D";
  ntype.ui_description =
      "2D alpha shape of XY points (CGAL Alpha_shape_2): the region you can cover "
      "with disks of radius Alpha that contain no other sample. "
      "How to use: plug Points → start with a large Alpha, then lower it until the "
      "outline hugs the cloud; or turn Optimal on to auto-pick. "
      "Output is filled triangles (not just a boundary). "
      "Different from Alpha Shape (3D surface) and Alpha Wrap (watertight offset).";
  ntype.enum_name_legacy = "CGAL_ALPHA_SHAPE_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_alpha_shape_2_cc
