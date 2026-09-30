/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_vertical_decomp_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description(
          "XY linework (Mesh faces and/or wires). Crossings are split first, then every "
          "vertex sends a vertical (world +Y) ray to the next edge above and below.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Trapezoids / triangles that tile the drawing. Face integer \"island_id\" "
          "numbers each cell. Use when you need a y-monotone partition (offset, fill, "
          "or scanline-style processing).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Vertical Decomposition 2D needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_vertical_decomposition_2(*mesh, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Vertical Decomposition 2D failed") : error);
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
      &ntype, "GeometryNodeCgalVerticalDecomp2"_ustr, GEO_NODE_CGAL_VERTICAL_DECOMP_2);
  ntype.ui_name = "Vertical Decomposition 2D";
  ntype.ui_description =
      "Split an XY drawing into trapezoids by dropping a vertical line from every vertex. "
      "How to use: feed a Mesh of 2D edges or faces → get a mesh of non-overlapping "
      "trapezoids (Face attribute island_id). Compared with Find All Cells 2D / Arrangement 2D, "
      "this further cuts each cell with vertical walls so every face is y-monotone. "
      "Typical use: prepare a floor plan or 2D boolean result for offset / scanline fill.";
  ntype.enum_name_legacy = "CGAL_VERTICAL_DECOMP_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_vertical_decomp_2_cc
