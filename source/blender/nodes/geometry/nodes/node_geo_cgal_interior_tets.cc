/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Delaunay tetrahedra whose circumcenter lies inside a closed mesh.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_interior_tets_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Closed surface. Interior Delaunay tets of its vertices are kept.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Triangle soup of interior tetrahedron faces.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 4 || mesh->faces_num < 4) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Interior Tets needs a closed mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_interior_tets(*mesh, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Interior Tets failed") : error);
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
  /* Deleted: Interior Tets. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalInteriorTets"_ustr, GEO_NODE_CGAL_INTERIOR_TETS);
  ntype.ui_name = "Interior Tets";
  ntype.ui_description =
      "Keep Delaunay tetrahedra of the mesh vertices whose circumcenter lies "
      "inside the closed surface (CGAL Delaunay_triangulation_3 + "
      "Side_of_triangle_mesh). Delaunay 3D keeps every tet of the convex hull, "
      "including outside the object. Not Mesh_3 quality volume meshing.";
  ntype.enum_name_legacy = "CGAL_INTERIOR_TETS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_interior_tets_cc
