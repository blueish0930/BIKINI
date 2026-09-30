/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * 2D medial axis (MAT) of XY polygons via discrete boundary Voronoi.
 * Distinct from Straight Skeleton 2D (angle-bisector / parallel roof).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_medial_axis_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description(
          "Filled XY region: face borders or closed edge rings. "
          "Open polylines are skipped (MAT needs a filled domain).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Interior medial axis as wire edges (Voronoi of boundary samples; "
          "curved/piecewise-linear branches — not straight skeleton).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh) {
    params.error_message_add(NodeWarningType::Info, TIP_("Medial Axis 2D needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_medial_axis_2(*mesh, error);
  if (!out || out->edges_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Medial Axis 2D failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalMedialAxis2"_ustr, GEO_NODE_CGAL_MEDIAL_AXIS_2);
  ntype.ui_name = "Medial Axis 2D";
  ntype.ui_description =
      "2D medial axis (MAT) of an XY polygon: discrete Voronoi of the boundary "
      "(parabolic branches approximated as polylines). "
      "Different from Straight Skeleton 2D (angle bisectors). "
      "Not a 3D MAT — for solids use Skeleton (MCF).";
  ntype.enum_name_legacy = "CGAL_MEDIAL_AXIS_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_medial_axis_2_cc
