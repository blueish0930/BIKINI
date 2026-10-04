/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL maximum_area_inscribed_k_gon_2 on projected convex hull.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_max_area_k_gon_2_cc {

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
      .description("Sites in XY (point cloud or mesh vertices).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Maximum-area inscribed k-gon.");
  b.add_input<decl::Int>("K"_ustr).default_value(4).min(3).max(64)
      .description("Number of vertices (>= 3).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int plane = 0; /* XY only. */
  const int k = params.extract_input<int>("K"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.error_message_add(NodeWarningType::Info, TIP_("Max Area K-gon 2D needs at least 3 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_max_area_k_gon_2(pts, plane, k, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Max Area K-gon 2D failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMaxAreaKGon2"_ustr, GEO_NODE_CGAL_MAX_AREA_K_GON_2);
  ntype.ui_name = "Max Area K-gon 2D";
  ntype.ui_description =
      "Maximum-area k-gon inscribed in the projected convex hull "
      "(CGAL maximum_area_inscribed_k_gon_2).";
  ntype.enum_name_legacy = "CGAL_MAX_AREA_K_GON_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_max_area_k_gon_2_cc
