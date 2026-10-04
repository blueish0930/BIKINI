/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * MCF skeleton plus spokes to the surface vertices that contracted into each node.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_skeleton_spokes_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Closed triangle mesh.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Wire: skeleton centerline plus a spoke to each contracted surface vertex.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num < 1) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Skeleton Spokes needs a closed mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_skeleton_spokes(*mesh, error);
  if (!out || out->edges_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Skeleton Spokes failed") : error);
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
      &ntype, "GeometryNodeCgalSkeletonSpokes"_ustr, GEO_NODE_CGAL_SKELETON_SPOKES);
  ntype.ui_name = "Skeleton Spokes";
  ntype.ui_description =
      "Mean-curvature-flow skeleton plus a spoke from each node to the "
      "surface vertices that contracted into it (CGAL "
      "extract_mean_curvature_flow_skeleton correspondence). Skeleton is "
      "only the centerline.";
  ntype.enum_name_legacy = "CGAL_SKELETON_SPOKES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_skeleton_spokes_cc
