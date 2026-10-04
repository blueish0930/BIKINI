/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Minkowski Sum 2D
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_minkowski_sum_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh A"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("First XY polygon (face borders). This is the shape being grown.");
  b.add_input<decl::Geometry>("Mesh B"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Second XY polygon used as the structuring element (e.g. a disk).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Triangulated Minkowski sum (may have holes).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Mesh A"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Mesh B"_ustr);
  const Mesh *a = ga.get_mesh();
  const Mesh *b = gb.get_mesh();
  if (!a || !b) {
    params.error_message_add(NodeWarningType::Info, TIP_("Minkowski Sum 2D needs two meshes"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_minkowski_sum_2(*a, *b, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Minkowski Sum 2D failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalMinkowskiSum2"_ustr, GEO_NODE_CGAL_MINKOWSKI_SUM_2);
  ntype.ui_name = "Minkowski Sum 2D";
  ntype.ui_description =
      "2D Minkowski sum A ⊕ B of two XY polygons (CGAL reduced convolution): "
      "the region swept by placing a copy of B at every point of A. "
      "How to use: Mesh A = the part, Mesh B = a disk / square → get the offset / "
      "grown outline (holes possible). Different from Polygon Offset 2D (skeleton "
      "offset of one shape) and Convex Minkowski Sum 3D (3D convex hulls).";
  ntype.enum_name_legacy = "CGAL_MINKOWSKI_SUM_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_minkowski_sum_2_cc
