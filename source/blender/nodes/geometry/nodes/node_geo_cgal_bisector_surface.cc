/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Voronoi interface between two point sets (dual of mixed DT edges).
 * Distinct from Voronoi 3D (all cells) and Mesh Intersection (surface-surface).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_bisector_surface_cc {

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
  b.add_input<decl::Geometry>("A"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("First site set (point cloud or mesh vertices).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Finite Voronoi faces between A and B.");
  b.add_input<decl::Geometry>("B"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Second site set.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("A"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("B"_ustr);
  const Span<float3> a = positions_from_geometry(ga);
  const Span<float3> b = positions_from_geometry(gb);
  if (a.size() < 1 || b.size() < 1) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Bisector Surface needs points on both A and B"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_bisector_surface(a, b, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Bisector Surface failed") : error);
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
  /* Deleted: Bisector Surface. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalBisectorSurface"_ustr, GEO_NODE_CGAL_BISECTOR_SURFACE);
  ntype.ui_name = "Bisector Surface";
  ntype.ui_description =
      "Approximate the set of points equally far from site set A and site set B: "
      "Voronoi faces dual to Delaunay edges that connect A to B. "
      "Voronoi 3D outputs every cell; Mesh Intersection is the actual surface cut.";
  ntype.enum_name_legacy = "CGAL_BISECTOR_SURFACE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_bisector_surface_cc
