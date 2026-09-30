/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Cover XY points with p equal axis-aligned squares of minimal size (p=2..4).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_rectangular_p_center_2_cc {

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
      .description("Sites to cover (point cloud or mesh verts).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("P isolated axis-aligned squares. Face attribute square_id.");
  b.add_output<decl::Float>("Radius"_ustr).description("Half-side of each covering square.");
  b.add_input<decl::Int>("P"_ustr)
      .default_value(2)
      .min(2)
      .max(4)
      .description("Number of equal squares (2, 3, or 4).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int p = params.extract_input<int>("P"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 2) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Rectangular P-Center 2D needs at least 2 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Radius"_ustr, 0.0f);
    return;
  }
  std::string error;
  float radius = 0.0f;
  Mesh *out = geometry::cgal_points_rectangular_p_center_2(pts, p, radius, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Rectangular P-Center 2D failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Radius"_ustr, 0.0f);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Radius"_ustr, radius);
}

static void node_register()
{
  /* Deleted: Rectangular P-Center 2D (legacy 2464 / 2593). */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalRectangularPCenter2"_ustr, GEO_NODE_CGAL_RECTANGULAR_P_CENTER_2);
  ntype.ui_name = "Rectangular P-Center 2D";
  ntype.ui_description =
      "Cover XY points with P equal axis-aligned squares of minimal size "
      "(CGAL rectangular_p_center_2, P=2..4). Distinct from Largest Empty Iso Rectangle.";
  ntype.enum_name_legacy = "CGAL_RECTANGULAR_P_CENTER_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_rectangular_p_center_2_cc
