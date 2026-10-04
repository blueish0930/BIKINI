/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Visibility among XY walls (face borders and/or loose edges).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_visibility_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Walls"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description(
          "XY walls. Filled faces: only the border is a solid obstacle (room or thick wall). "
          "Loose edges: thin walls. Join Geometry to mix rooms and extra wall edges.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Visibility polygon from Query as one connected n-gon.");
  b.add_input<decl::Vector>("Query"_ustr)
      .default_value(float3(0.0f))
      .subtype(PROP_TRANSLATION)
      .description(
          "Viewpoint (XY). Must lie inside the walls bounding box, in free space "
          "(not inside a solid wall).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Walls"_ustr);
  const float3 query = params.extract_input<float3>("Query"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 2) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Visibility 2D needs wall faces or edges"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_visibility_2(*mesh, query, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Visibility 2D failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalVisibility2"_ustr, GEO_NODE_CGAL_VISIBILITY_2);
  ntype.ui_name = "Visibility 2D";
  ntype.ui_description =
      "Visibility polygon from a query among XY walls (one connected n-gon). "
      "Filled faces: only the border is solid, so holes stay free space. "
      "Loose edges are thin walls. Query must be inside the walls bounding box.";
  ntype.enum_name_legacy = "CGAL_VISIBILITY_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_visibility_2_cc
