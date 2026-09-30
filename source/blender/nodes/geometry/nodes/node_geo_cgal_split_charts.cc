/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Cut a mesh along sharp dihedral edges into disconnected chart islands.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_split_charts_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to cut along sharp edges.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Disconnected chart islands. Separate Geometry to pull them apart.");
  b.add_input<decl::Float>("Angle"_ustr)
      .default_value(60.0f)
      .min(0.0f)
      .max(180.0f)
      .description(
          "Cut where adjacent face normals differ by more than this (degrees). "
          "Lower = more cuts. 60 splits a cube into 6 faces.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float angle_deg = params.extract_input<float>("Angle"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num < 1) {
    params.error_message_add(NodeWarningType::Info, TIP_("Split Charts needs a mesh with faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_split_charts(*mesh, angle_deg, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Split Charts failed") : error);
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
  /* Deleted: Split Charts. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalSplitCharts"_ustr, GEO_NODE_CGAL_SPLIT_CHARTS);
  ntype.ui_name = "Split Charts";
  ntype.ui_description =
      "Cut the mesh along edges whose adjacent face normals differ by more "
      "than Angle, then duplicate those edges so each chart is a separate "
      "island (CGAL detect_sharp_edges + split_connected_components). "
      "Detect Features only marked edges. Region Growing labels planar "
      "patches without cutting topology.";
  ntype.enum_name_legacy = "CGAL_SPLIT_CHARTS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_split_charts_cc
