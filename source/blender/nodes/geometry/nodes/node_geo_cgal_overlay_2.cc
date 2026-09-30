/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Arrangement overlay of two XY edge sets. Keeps every labeled cell
 * (unlike Boolean Ops 2D, which outputs one region).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_overlay_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh A"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("First 2D shape (faces and/or wires).");
  b.add_input<decl::Geometry>("Mesh B"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Second 2D shape (faces and/or wires).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Every bounded overlay cell as a welded n-gon. "
          "Face attribute overlay_id: 1=A only, 2=B only, 3=both.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Mesh A"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Mesh B"_ustr);
  const Mesh *a = ga.get_mesh();
  const Mesh *b = gb.get_mesh();
  if (!a && !b) {
    params.error_message_add(NodeWarningType::Info, TIP_("Overlay 2D needs Mesh A or Mesh B"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  Mesh *owned_a = nullptr;
  Mesh *owned_b = nullptr;
  if (!a) {
    owned_a = BKE_mesh_new_nomain(0, 0, 0, 0);
    a = owned_a;
  }
  if (!b) {
    owned_b = BKE_mesh_new_nomain(0, 0, 0, 0);
    b = owned_b;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_overlay_2(*a, *b, error);
  if (owned_a) {
    BKE_id_free(nullptr, owned_a);
  }
  if (owned_b) {
    BKE_id_free(nullptr, owned_b);
  }
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Overlay 2D failed") : error);
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
  /* Deleted: Overlay 2D. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalOverlay2"_ustr, GEO_NODE_CGAL_OVERLAY_2);
  ntype.ui_name = "Overlay 2D";
  ntype.ui_description =
      "Arrangement overlay of two XY edge sets (CGAL Arrangement_2). "
      "Crossings are split; every bounded cell that lies in A, B, or both is kept. "
      "Face attribute overlay_id: 1=A only, 2=B only, 3=both. "
      "Unlike Boolean Ops 2D this does not discard cells.";
  ntype.enum_name_legacy = "CGAL_OVERLAY_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_overlay_2_cc
