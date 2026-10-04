/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Delaunay triangulation on the bounding sphere of 3D points.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_delaunay_on_sphere_cc {

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
      .description("3D sites. They are projected onto their bounding sphere.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Spherical Delaunay triangles on the bounding sphere.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Delaunay on Sphere needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_delaunay_on_sphere(pts, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Delaunay on Sphere failed") : error);
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
      &ntype, "GeometryNodeCgalDelaunayOnSphere"_ustr, GEO_NODE_CGAL_DELAUNAY_ON_SPHERE);
  ntype.ui_name = "Delaunay on Sphere";
  ntype.ui_description =
      "Delaunay triangulation of points projected onto their bounding sphere "
      "(CGAL Delaunay_triangulation_on_sphere_2). Use for globe / spherical meshes.";
  ntype.enum_name_legacy = "CGAL_DELAUNAY_ON_SPHERE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_delaunay_on_sphere_cc
