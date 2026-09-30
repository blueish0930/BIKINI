/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Intersect inward half-spaces of every mesh face (CGAL halfspace_intersection_3).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_halfspace_intersection_3_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Every face is one half-space. The interior side of the face normal is kept.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Convex kernel = intersection of the inward half-spaces.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 4) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Halfspace Intersection 3D needs at least 4 vertices"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_halfspace_intersection_3(*mesh, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Halfspace Intersection 3D failed") : error);
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
  /* Deleted: Halfspace Intersection 3D node. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype,
                     "GeometryNodeCgalHalfspaceIntersection3"_ustr,
                     GEO_NODE_CGAL_HALFSPACE_INTERSECTION_3);
  ntype.ui_name = "Halfspace Intersection 3D";
  ntype.ui_description =
      "Intersect the inward half-space of every input face "
      "(CGAL halfspace_intersection_with_constructions_3). Face winding is treated as "
      "outward; the kept side is the interior. The seed point is computed automatically. "
      "A non-convex mesh shrinks to its convex kernel.";
  ntype.enum_name_legacy = "CGAL_HALFSPACE_INTERSECTION_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_halfspace_intersection_3_cc
