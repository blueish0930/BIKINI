/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Delaunay tetrahedra stabbed by a query segment.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_walk_tets_cc {

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
      .description("Sites of the Delaunay tetrahedralization.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Triangle soup of tetrahedra the segment passes through.");
  b.add_input<decl::Vector>("Start"_ustr)
      .default_value(float3(0.0f, 0.0f, 0.0f))
      .subtype(PROP_TRANSLATION)
      .description("Segment start.");
  b.add_input<decl::Vector>("End"_ustr)
      .default_value(float3(0.0f, 0.0f, 1.0f))
      .subtype(PROP_TRANSLATION)
      .description("Segment end.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float3 start = params.extract_input<float3>("Start"_ustr);
  const float3 end = params.extract_input<float3>("End"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info, TIP_("Walk Tets needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_walk_tets(pts, start, end, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Walk Tets failed") : error);
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
  /* Deleted: Walk Tets. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalWalkTets"_ustr, GEO_NODE_CGAL_WALK_TETS);
  ntype.ui_name = "Walk Tets";
  ntype.ui_description =
      "Keep only the Delaunay tetrahedra stabbed by the Start–End segment "
      "(CGAL Triangulation_segment_traverser_3). Delaunay 3D emits every tet "
      "of the convex hull.";
  ntype.enum_name_legacy = "CGAL_WALK_TETS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_walk_tets_cc
