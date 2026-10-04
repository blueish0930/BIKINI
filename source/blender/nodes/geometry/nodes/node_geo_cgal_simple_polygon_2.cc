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

namespace blender::nodes::node_geo_cgal_simple_polygon_2_cc {

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
      .description(
          "Point Cloud or Mesh vertices in XY. Every distinct point becomes a vertex "
          "of one polygon (not just the convex hull). Collinear-only sets fail.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "One simple (non-self-intersecting) n-gon through all distinct XY points. "
          "The vertex order is rewritten so edges no longer cross.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Simple Polygon 2D needs at least 3 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_simple_polygon_2(pts, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Simple Polygon 2D failed") : error);
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
      &ntype, "GeometryNodeCgalSimplePolygon2"_ustr, GEO_NODE_CGAL_SIMPLE_POLYGON_2);
  ntype.ui_name = "Simple Polygon 2D";
  ntype.ui_description =
      "Turn a cloud of XY points into one simple polygon that visits every point. "
      "How to use: plug Points → get a single n-gon whose edges do not cross. "
      "Unlike Convex Hull this keeps interior points as vertices. "
      "If all points sit on one line the node fails. Good for turning a 2D scatter "
      "into a fillable outline before Polygon Fill 2D or Boolean Ops 2D.";
  ntype.enum_name_legacy = "CGAL_SIMPLE_POLYGON_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_simple_polygon_2_cc
