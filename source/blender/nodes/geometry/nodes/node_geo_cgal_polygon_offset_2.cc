/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL polygon offset (XY).
 *
 *  - Mesh WITH faces: straight-skeleton offset of filled border regions
 *    (+ expand / − shrink).
 *  - Mesh edges ONLY (no faces): strip buffer of each edge chain.
 *    Cyclic (closed) wire → expand to BOTH sides (hollow frame).
 *    Open wire → strip with end caps. |Offset| = half-width.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_polygon_offset_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description(
          "Faces: offset filled border regions (skeleton). "
          "Edges only (no faces): cyclic wires expand both sides as a frame; "
          "open chains become a strip.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Triangulated offset region or wire strip.");
  b.add_input<decl::Float>("Offset"_ustr)
      .default_value(0.1f)
      .description(
          "Faces: signed (+ expand / − shrink). "
          "Edge-only wires: |Offset| = half-width to each side of the line.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float offset = params.extract_input<float>("Offset"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh) {
    params.error_message_add(NodeWarningType::Info, TIP_("Polygon Offset 2D needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  if (offset == 0.0f) {
    params.error_message_add(NodeWarningType::Info, TIP_("Offset must be non-zero"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_polygon_offset_2(*mesh, offset, 0, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Polygon Offset 2D failed") : error);
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
      &ntype, "GeometryNodeCgalPolygonOffset2"_ustr, GEO_NODE_CGAL_POLYGON_OFFSET_2);
  ntype.ui_name = "Polygon Offset 2D";
  ntype.ui_description =
      "Offset filled face borders (skeleton), or expand pure edge wires "
      "to both sides when the mesh has no faces (cyclic → hollow frame).";
  ntype.enum_name_legacy = "CGAL_POLYGON_OFFSET_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_polygon_offset_2_cc
