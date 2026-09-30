/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Fill boundary holes then fair the patch (C0/C1/C2).
 * Different from Hole Fill (triangulate_and_refine_hole only).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_fair_hole_fill_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh with open boundary holes.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Original faces kept; only filled holes are triangles. Attributes inherited.");
  b.add_input<decl::Int>("Continuity"_ustr)
      .default_value(1)
      .min(0)
      .max(2)
      .description("Fairing order of the patch: 0 = C0, 1 = C1 (default), 2 = C2.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const int continuity = params.extract_input<int>("Continuity"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *mesh = geometry::cgal_mesh_fair_hole_fill(*mesh_in, continuity, error);
  if (mesh == nullptr || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Fair Hole Fill failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalFairHoleFill"_ustr, GEO_NODE_CGAL_FAIR_HOLE_FILL);
  ntype.ui_name = "Fair Hole Fill";
  ntype.ui_description =
      "Fill every boundary loop then fair the patch "
      "(CGAL triangulate_refine_and_fair_hole). Original n-gons are kept; "
      "only the new hole patches are triangles. Attributes copy from the source "
      "mesh; new hole verts solve a cotangent Laplace system (harmonic). Continuity 0/1/2.";
  ntype.enum_name_legacy = "CGAL_FAIR_HOLE_FILL";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_fair_hole_fill_cc
