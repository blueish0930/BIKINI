/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Collapse edges shorter than Max Length (shortest-first, midpoint placement).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_collapse_short_edges_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle or n-gon mesh. Internally triangulated for collapse.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Mesh after collapsing every edge shorter than Max Length.");
  b.add_input<decl::Float>("Max Length"_ustr)
      .default_value(0.01f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Collapse edges whose length is at most this (shortest first).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float max_length = params.extract_input<float>("Max Length"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_collapse_short_edges(*mesh, max_length, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Collapse Short Edges failed") : error);
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
      &ntype, "GeometryNodeCgalCollapseShortEdges"_ustr, GEO_NODE_CGAL_COLLAPSE_SHORT_EDGES);
  ntype.ui_name = "Collapse Short Edges";
  ntype.ui_description =
      "Collapse edges shorter than Max Length, shortest first, placing the new "
      "vertex at the midpoint (CGAL Surface_mesh_simplification Edge_length_cost). "
      "Different from Simplify (Lindstrom–Turk volume) and QEM Simplify (quadric error).";
  ntype.enum_name_legacy = "CGAL_COLLAPSE_SHORT_EDGES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_collapse_short_edges_cc
