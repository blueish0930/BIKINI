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

namespace blender::nodes::node_geo_cgal_min_ellipsoid_cc {

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
      .description("Points or mesh verts.");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous()
      .description("Tessellated min-volume enclosing ellipsoid.");
  b.add_input<decl::Int>("Segments"_ustr).default_value(24).min(8).max(64)
      .description("Tessellation density of the ellipsoid mesh.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int segments = params.extract_input<int>("Segments"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info, TIP_("Min Ellipsoid needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *mesh = geometry::cgal_min_ellipsoid(pts, segments, error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Min Ellipsoid failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMinEllipsoid"_ustr, GEO_NODE_CGAL_MIN_ELLIPSOID);
  ntype.ui_name = "Min Ellipsoid";
  ntype.ui_description =
      "Approximate minimum-volume enclosing ellipsoid of points, as a tessellated ellipsoid mesh.";
  ntype.enum_name_legacy = "CGAL_MIN_ELLIPSOID";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_min_ellipsoid_cc
