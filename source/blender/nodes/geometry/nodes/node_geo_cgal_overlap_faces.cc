/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Extract faces of Mesh that intersect Other (AABB + triangle test).
 * Distinct from Do Intersect 3D (boolean), Mesh Intersection (curves),
 * and Mark Self Intersect (self).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_overlap_faces_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Faces of this mesh are tested and kept when they hit Other.");
  b.add_input<decl::Geometry>("Other"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Second triangle mesh.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Intersecting faces of Mesh as a new mesh.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Mesh"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Other"_ustr);
  const Mesh *a = ga.get_mesh();
  const Mesh *b = gb.get_mesh();
  if (!a || !b || a->faces_num == 0 || b->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Overlap Faces needs two meshes with faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_overlap_faces(*a, *b, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Overlap Faces failed") : error);
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
  /* Deleted: Overlap Faces. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalOverlapFaces"_ustr, GEO_NODE_CGAL_OVERLAP_FACES);
  ntype.ui_name = "Overlap Faces";
  ntype.ui_description =
      "Keep the faces of Mesh that intersect Other "
      "(CGAL AABB tree + triangle-triangle test). "
      "Do Intersect 3D is a boolean; Mesh Intersection outputs curves; "
      "Mark Self Intersect flags self-hits on one mesh.";
  ntype.enum_name_legacy = "CGAL_OVERLAP_FACES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_overlap_faces_cc
