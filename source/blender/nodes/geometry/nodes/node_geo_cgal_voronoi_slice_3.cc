/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Slice of the 3D Voronoi diagram by a plane (restricted Voronoi).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_voronoi_slice_3_cc {

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
      .description("3D sites. Distance to the plane changes the cell sizes.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Restricted Voronoi cells on the plane. Face attribute cell_id = site index.");
  b.add_input<decl::Vector>("Origin"_ustr)
      .default_value(float3(0.0f))
      .subtype(PROP_TRANSLATION)
      .description("A point on the slicing plane.");
  b.add_input<decl::Vector>("Normal"_ustr)
      .default_value(float3(0.0f, 0.0f, 1.0f))
      .subtype(PROP_DIRECTION)
      .description("Plane normal.");
  b.add_input<decl::Float>("Clip Margin"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("How far unbounded cells extend past the site bbox in the plane.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float3 origin = params.extract_input<float3>("Origin"_ustr);
  const float3 normal = params.extract_input<float3>("Normal"_ustr);
  const float clip = params.extract_input<float>("Clip Margin"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.error_message_add(NodeWarningType::Info, TIP_("Voronoi Slice 3D needs at least 3 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_voronoi_slice_3(pts, origin, normal, clip, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Voronoi Slice 3D failed") : error);
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
  /* Deleted: Voronoi Slice 3D node. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalVoronoiSlice3"_ustr, GEO_NODE_CGAL_VORONOI_SLICE_3);
  ntype.ui_name = "Voronoi Slice 3D";
  ntype.ui_description =
      "Intersect the 3D Voronoi diagram of the sites with a plane "
      "(CGAL Voronoi_intersection_2_traits_3). Unlike Voronoi 2D this does not flatten the "
      "sites first — a site closer to the plane owns a larger cell. Face attribute cell_id.";
  ntype.enum_name_legacy = "CGAL_VORONOI_SLICE_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_voronoi_slice_3_cc
