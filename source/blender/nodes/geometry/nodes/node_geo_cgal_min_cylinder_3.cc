/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * PCA axis + min enclosing circle of the radial plane → finite cylinder.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_min_cylinder_3_cc {

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
      .description("Points to enclose (point cloud or mesh verts).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Finite cylinder (capped) along the PCA axis.");
  b.add_output<decl::Vector>("Center"_ustr).description("Midpoint of the cylinder axis.");
  b.add_output<decl::Float>("Radius"_ustr).description("Radius of the enclosing circle.");
  b.add_input<decl::Int>("Segments"_ustr)
      .default_value(24)
      .min(8)
      .max(64)
      .description("How many sides approximate the cylinder.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int segments = params.extract_input<int>("Segments"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 2) {
    params.error_message_add(NodeWarningType::Info, TIP_("Min Cylinder 3D needs at least 2 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Center"_ustr, float3(0.0f));
    params.set_output("Radius"_ustr, 0.0f);
    return;
  }
  std::string error;
  float3 center(0.0f);
  float radius = 0.0f;
  Mesh *out = geometry::cgal_points_min_cylinder_3(pts, segments, center, radius, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Min Cylinder 3D failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Center"_ustr, float3(0.0f));
    params.set_output("Radius"_ustr, 0.0f);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Center"_ustr, center);
  params.set_output("Radius"_ustr, radius);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMinCylinder3"_ustr, GEO_NODE_CGAL_MIN_CYLINDER_3);
  ntype.ui_name = "Min Cylinder 3D";
  ntype.ui_description =
      "Finite cylinder along the PCA axis that encloses the points "
      "(CGAL linear_least_squares_fitting_3 + Min_circle_2 of the radial plane). "
      "Use for pipes / packing. Different from Min Sphere (ball) and Optimal Bounding Box.";
  ntype.enum_name_legacy = "CGAL_MIN_CYLINDER_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_min_cylinder_3_cc
