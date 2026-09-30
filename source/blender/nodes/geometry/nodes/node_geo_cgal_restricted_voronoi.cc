/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Restricted Voronoi diagram: 3D Voronoi cells of sites clipped to a surface.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_restricted_voronoi_cc {

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
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Surface the Voronoi cells are clipped onto.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Surface cells. Face attribute cell_id is the site index.");
  b.add_input<decl::Geometry>("Sites"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("3D Voronoi sites (point cloud or mesh verts).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  GeometrySet sites_geo = params.extract_input<GeometrySet>("Sites"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  const Span<float3> sites = positions_from_geometry(sites_geo);
  if (!mesh || mesh->faces_num < 1) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Restricted Voronoi needs a mesh with faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  if (sites.size() < 1) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Restricted Voronoi needs at least 1 site"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_restricted_voronoi(*mesh, sites, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Restricted Voronoi failed") : error);
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
  /* Deleted: Restricted Voronoi. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalRestrictedVoronoi"_ustr, GEO_NODE_CGAL_RESTRICTED_VORONOI);
  ntype.ui_name = "Restricted Voronoi";
  ntype.ui_description =
      "Clip 3D Voronoi cells of the sites onto the mesh (restricted Voronoi "
      "diagram). Geodesic Voronoi uses MMP geodesic distance; this uses "
      "Euclidean 3D bisectors. Voronoi Slice 3D clips to a plane, not a "
      "surface. Face attribute cell_id is the site index.";
  ntype.enum_name_legacy = "CGAL_RESTRICTED_VORONOI";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_restricted_voronoi_cc
