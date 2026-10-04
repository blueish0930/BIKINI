/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Exterior straight skeleton of an XY polygon (wavefront growing outward).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_exterior_skeleton_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("XY polygon. Face borders or closed edge rings.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Exterior straight skeleton as a wire mesh (frame discarded).");
  b.add_input<decl::Float>("Max Offset"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description(
          "Maximum distance from the polygon. Bisectors are clipped at this radius "
          "(not the helper frame size).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float max_offset = params.extract_input<float>("Max Offset"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 3) {
    params.error_message_add(NodeWarningType::Info, TIP_("Exterior Skeleton 2D needs a polygon"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_exterior_skeleton_2(*mesh, max_offset, error);
  if (!out || out->edges_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Exterior Skeleton 2D failed") : error);
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
      &ntype, "GeometryNodeCgalExteriorSkeleton2"_ustr, GEO_NODE_CGAL_EXTERIOR_SKELETON_2);
  ntype.ui_name = "Exterior Skeleton 2D";
  ntype.ui_description =
      "Exterior straight skeleton of an XY polygon "
      "(CGAL create_exterior_straight_skeleton_2). Outward wavefront, opposite of "
      "Straight Skeleton 2D. Max Offset is the distance from the polygon; "
      "the helper frame is discarded. Holes use Straight Skeleton 2D (interior).";
  ntype.enum_name_legacy = "CGAL_EXTERIOR_SKELETON_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_exterior_skeleton_2_cc
