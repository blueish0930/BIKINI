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

namespace blender::nodes::node_geo_cgal_periodic_delaunay_3_cc {

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
      .description("Delaunay tetrahedra inside one period of the periodic box.");
  b.add_output<decl::Int>("Tets"_ustr).description("Number of tetrahedra produced.");
  b.add_input<decl::Bool>("Separate Tetrahedra"_ustr)
      .default_value(false)
      .description("Keep each tetrahedron as an isolated element instead of a welded mesh.");
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
  const bool separate = params.extract_input<bool>("Separate Tetrahedra"_ustr);
  const float dx = params.extract_input<float>("Domain X"_ustr);
  const float dy = params.extract_input<float>("Domain Y"_ustr);
  const float dz = params.extract_input<float>("Domain Z"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Periodic Delaunay 3D needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Tets"_ustr, 0);
    return;
  }
  std::string error;
  int tet_count = 0;
  Mesh *mesh = geometry::cgal_points_periodic_delaunay_3(
      pts, dx, dy, dz, separate, tet_count, error);
  if (mesh == nullptr || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Periodic Delaunay 3D failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Tets"_ustr, 0);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
  params.set_output("Tets"_ustr, tet_count);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalPeriodicDelaunay3"_ustr, GEO_NODE_CGAL_PERIODIC_DELAUNAY_3);
  ntype.ui_name = "Periodic Delaunay 3D";
  ntype.ui_description =
      "Delaunay tetrahedra on a periodic box (CGAL Periodic_3_Delaunay_triangulation_3). "
      "Wrapping cells use offset vertices so they sit on the domain boundary. "
      "Domain 0 uses a padded bbox.";
  ntype.enum_name_legacy = "CGAL_PERIODIC_DELAUNAY_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_periodic_delaunay_3_cc
