/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Convex Minkowski sum of two meshes: conv(A) ⊕ conv(B).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_minkowski_sum_3_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh A"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("First operand (convex hull is taken automatically).");
  b.add_input<decl::Geometry>("Mesh B"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Second operand (convex hull is taken automatically).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Convex hull of pairwise vertex sums.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Mesh A"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Mesh B"_ustr);
  const Mesh *a = ga.get_mesh();
  const Mesh *b = gb.get_mesh();
  if (!a || !b || a->verts_num < 1 || b->verts_num < 1) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Convex Minkowski Sum 3D needs two meshes"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_minkowski_sum_3_convex(*a, *b, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Convex Minkowski Sum 3D failed") : error);
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
      &ntype, "GeometryNodeCgalMinkowskiSum3"_ustr, GEO_NODE_CGAL_MINKOWSKI_SUM_3);
  ntype.ui_name = "Convex Minkowski Sum 3D";
  ntype.ui_description =
      "Convex Minkowski sum of two meshes: conv(A) ⊕ conv(B) = conv({a+b}). "
      "Non-convex inputs are replaced by their convex hulls first (not exact Nef Minkowski).";
  ntype.enum_name_legacy = "CGAL_MINKOWSKI_SUM_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_minkowski_sum_3_cc
