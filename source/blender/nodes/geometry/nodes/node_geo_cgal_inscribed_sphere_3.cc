/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Largest inscribed sphere of the convex hull of a point set (Chebyshev center).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_inscribed_sphere_3_cc {

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
      .description("Sites whose convex hull is inscribed. Point cloud or mesh vertices.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("UV sphere of the largest ball inside the convex hull.");
  b.add_output<decl::Vector>("Center"_ustr).description("Chebyshev center of the convex hull.");
  b.add_output<decl::Float>("Radius"_ustr).description("Inradius of the convex hull.");
  b.add_input<decl::Int>("Segments"_ustr)
      .default_value(24)
      .min(8)
      .max(64)
      .description("UV sphere tessellation.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int segments = params.extract_input<int>("Segments"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Inscribed Sphere 3D needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Center"_ustr, float3(0.0f));
    params.set_output("Radius"_ustr, 0.0f);
    return;
  }
  std::string error;
  Mesh *hull = geometry::cgal_convex_hull_3(pts, error);
  if (!hull || hull->faces_num < 4) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Inscribed Sphere 3D: convex hull failed") :
                                             error);
    if (hull) {
      BKE_id_free(nullptr, hull);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Center"_ustr, float3(0.0f));
    params.set_output("Radius"_ustr, 0.0f);
    return;
  }
  float3 center(0.0f);
  float radius = 0.0f;
  Mesh *out = geometry::cgal_mesh_inscribed_sphere_3(*hull, segments, center, radius, error);
  BKE_id_free(nullptr, hull);
  if (!out || out->faces_num == 0 || radius <= 0.0f) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Inscribed Sphere 3D failed") : error);
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
  geo_node_type_base(
      &ntype, "GeometryNodeCgalInscribedSphere3"_ustr, GEO_NODE_CGAL_INSCRIBED_SPHERE_3);
  ntype.ui_name = "Inscribed Sphere 3D";
  ntype.ui_description =
      "Largest sphere inside the convex hull of the input points (Chebyshev center). "
      "Takes a point cloud or mesh vertices. The 3D analog of Largest Inscribed Circle 2D. "
      "Coplanar or nearly flat point sets have no interior and fail.";
  ntype.enum_name_legacy = "CGAL_INSCRIBED_SPHERE_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_inscribed_sphere_3_cc
