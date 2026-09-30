/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Offset every face plane, then intersect the half-spaces (convex / convex kernel).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_convex_offset_3_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Convex mesh. Non-convex input is reduced to its convex kernel first.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Expanded or inset convex polyhedron (faces stay planar).");
  b.add_input<decl::Float>("Offset"_ustr)
      .default_value(0.1f)
      .subtype(PROP_DISTANCE)
      .description("Positive grows the solid, negative shrinks it. Too large an inset is empty.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float offset = params.extract_input<float>("Offset"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Offset Convex 3D needs a mesh with faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_convex_offset_3(*mesh, offset, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Offset Convex 3D failed") : error);
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
  /* Deleted: Offset Convex 3D / Convex Offset 3D. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalConvexOffset3"_ustr, GEO_NODE_CGAL_CONVEX_OFFSET_3);
  ntype.ui_name = "Offset Convex 3D";
  ntype.ui_description =
      "Expand or shrink a convex solid by moving every face along its normal "
      "(parallel-face offset — edges stay sharp, not a rounded Minkowski ball). "
      "Positive Offset grows the mesh; negative insets it. "
      "A non-convex mesh is reduced to its convex kernel first. "
      "Empty if the inset is larger than the inradius.";
  ntype.enum_name_legacy = "CGAL_CONVEX_OFFSET_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_convex_offset_3_cc
