/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Tangent-plane Delaunay 1-ring graph of a 3D point set.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_surface_delaunay_graph_cc {

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
      .description("Points sampled on a surface (point cloud or mesh verts).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Wire: tangent-plane Delaunay 1-ring of each point.");
  b.add_input<decl::Int>("K"_ustr)
      .default_value(16)
      .min(6)
      .max(64)
      .description("How many Euclidean neighbors to project onto the local tangent plane.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int knn = params.extract_input<int>("K"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Surface Delaunay Graph needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_surface_delaunay_graph(pts, knn, error);
  if (!out || out->edges_num == 0) {
    params.error_message_add(
        NodeWarningType::Warning,
        error.empty() ? TIP_("Surface Delaunay Graph failed") : error);
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
  /* Deleted: Surface Delaunay Graph. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype,
                     "GeometryNodeCgalSurfaceDelaunayGraph"_ustr,
                     GEO_NODE_CGAL_SURFACE_DELAUNAY_GRAPH);
  ntype.ui_name = "Surface Delaunay Graph";
  ntype.ui_description =
      "Connect each point to its 1-ring in a Delaunay triangulation of nearby "
      "points projected onto the local tangent plane (CGAL Delaunay_triangulation_2). "
      "KNN Graph 3D uses Euclidean k-nearest; Delaunay 3D fills tetrahedra.";
  ntype.enum_name_legacy = "CGAL_SURFACE_DELAUNAY_GRAPH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_surface_delaunay_graph_cc
