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

namespace blender::nodes::node_geo_cgal_periodic_voronoi_3_cc {

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
      .description("3D sites. Sites are wrapped into the periodic box.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Closed Voronoi cell volumes. Face attribute cell_id.");
  b.add_input<decl::Float>("Domain X"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Periodic domain size along X. 0 = use the point bounding box.");
  b.add_input<decl::Float>("Domain Y"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Periodic domain size along Y. 0 = use the point bounding box.");
  b.add_input<decl::Float>("Domain Z"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Periodic domain size along Z. 0 = use the point bounding box.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float dx = params.extract_input<float>("Domain X"_ustr);
  const float dy = params.extract_input<float>("Domain Y"_ustr);
  const float dz = params.extract_input<float>("Domain Z"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Periodic Voronoi 3D needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_periodic_voronoi_3(pts, dx, dy, dz, error);
  if (out == nullptr || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Periodic Voronoi 3D failed") : error);
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
      &ntype, "GeometryNodeCgalPeriodicVoronoi3"_ustr, GEO_NODE_CGAL_PERIODIC_VORONOI_3);
  ntype.ui_name = "Periodic Voronoi 3D";
  ntype.ui_description =
      "Periodic Voronoi cells of 3D points as closed mesh volumes "
      "(dual of Periodic Delaunay 3D, clipped by neighbor bisectors). "
      "Not a weighted / regular triangulation.";
  ntype.enum_name_legacy = "CGAL_PERIODIC_VORONOI_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_periodic_voronoi_3_cc
