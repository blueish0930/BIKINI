/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * 1-skeleton of a 3D alpha shape (REGULAR + SINGULAR edges).
 * Distinct from Alpha Shape (REGULARIZED triangles) and Alpha Complex 3D (tets).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_alpha_edges_3_cc {

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
      .description("Point cloud (or mesh verts) to extract alpha-shape edges from.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Wire of REGULAR and SINGULAR alpha-shape edges.");
  b.add_input<decl::Bool>("Optimal Alpha"_ustr)
      .default_value(true)
      .description("Pick alpha for the requested number of solid components.");
  b.add_input<decl::Int>("Solid Components"_ustr)
      .default_value(1)
      .min(1)
      .max(64)
      .description("Target solid pieces when Optimal Alpha is on.");
  b.add_input<decl::Float>("Alpha"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description("Manual alpha (scene units squared scale). 0 = bbox heuristic.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const bool optimal = params.extract_input<bool>("Optimal Alpha"_ustr);
  const int solid = params.extract_input<int>("Solid Components"_ustr);
  const float alpha = params.extract_input<float>("Alpha"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info, TIP_("Alpha Edges 3D needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  float alpha_used = 0.0f;
  Mesh *out = geometry::cgal_points_alpha_edges_3(pts, alpha, optimal, solid, alpha_used, error);
  if (!out || out->edges_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Alpha Edges 3D failed") : error);
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
  /* Deleted: Alpha Edges 3D. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalAlphaEdges3"_ustr, GEO_NODE_CGAL_ALPHA_EDGES_3);
  ntype.ui_name = "Alpha Edges 3D";
  ntype.ui_description =
      "Wire 1-skeleton of a 3D alpha shape (CGAL Alpha_shape_3 REGULAR + SINGULAR edges). "
      "Alpha Shape outputs a triangle surface in REGULARIZED mode; Alpha Complex 3D "
      "outputs tetrahedral faces.";
  ntype.enum_name_legacy = "CGAL_ALPHA_EDGES_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_alpha_edges_3_cc
