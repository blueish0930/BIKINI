/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Boolean union of triangles projected along a direction (silhouette card).
 * Distinct from deleted Project Mesh 2D (flatten, keep overlaps) and Convex Hull 2D.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_projected_outline_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh whose silhouette is taken along Direction.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Filled outline in the projection plane, including holes.");
  b.add_input<decl::Vector>("Direction"_ustr)
      .default_value(float3(0.0f, 0.0f, 1.0f))
      .subtype(PROP_DIRECTION)
      .description("Projection / view direction. Outline lies in a plane perpendicular to this.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float3 direction = params.extract_input<float3>("Direction"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Projected Outline needs a mesh with faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_projected_outline(*mesh, direction, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Projected Outline failed") : error);
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
  /* Deleted: Projected Outline. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalProjectedOutline"_ustr, GEO_NODE_CGAL_PROJECTED_OUTLINE);
  ntype.ui_name = "Projected Outline";
  ntype.ui_description =
      "Boolean union of every face projected along Direction — the true "
      "silhouette, concavities and holes included. Project Mesh 2D only "
      "flattened overlapping n-gons; Convex Hull 2D is the convex envelope.";
  ntype.enum_name_legacy = "CGAL_PROJECTED_OUTLINE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_projected_outline_cc
