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

namespace blender::nodes::node_geo_cgal_periodic_voronoi_2_cc {

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
      .description("XY sites. Sites are wrapped into the periodic domain.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Isolated n-gon Voronoi cells. Face attribute cell_id.");
  b.add_input<decl::Float>("Domain X"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Periodic width. 0 = 1.05 x the XY span.");
  b.add_input<decl::Float>("Domain Y"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Periodic height. 0 = 1.05 x the XY span.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float dx = params.extract_input<float>("Domain X"_ustr);
  const float dy = params.extract_input<float>("Domain Y"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Periodic Voronoi 2D needs at least 3 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_periodic_voronoi_2(pts, dx, dy, error);
  if (out == nullptr || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Periodic Voronoi 2D failed") : error);
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
  geo_node_type_base(
      &ntype, "GeometryNodeCgalPeriodicVoronoi2"_ustr, GEO_NODE_CGAL_PERIODIC_VORONOI_2);
  ntype.ui_name = "Periodic Voronoi 2D";
  ntype.ui_description =
      "Voronoi cells on a flat torus (dual of Periodic Delaunay 2D). "
      "Not a weighted / regular triangulation.";
  ntype.enum_name_legacy = "CGAL_PERIODIC_VORONOI_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_periodic_voronoi_2_cc
