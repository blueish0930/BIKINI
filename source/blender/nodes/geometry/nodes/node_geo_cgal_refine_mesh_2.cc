/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Mesh_2 Delaunay refinement of XY polygons (quality triangulation).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_refine_mesh_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("XY border islands (holes stay empty).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Quality triangle mesh of the polygon domain.");
  b.add_input<decl::Float>("Max Edge"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Maximum Steiner edge length. 0 = shape-only (no size bound).");
  b.add_input<decl::Float>("Shape Bound"_ustr)
      .default_value(0.125f)
      .min(0.01f)
      .max(0.5f)
      .description("CGAL B parameter (0.125 ≈ 20.7° minimum angle). Smaller = better triangles.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float max_edge = params.extract_input<float>("Max Edge"_ustr);
  const float shape = params.extract_input<float>("Shape Bound"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->verts_num < 3) {
    params.error_message_add(NodeWarningType::Info, TIP_("Refine Mesh 2D needs a polygon mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_refine_2(*mesh, max_edge, shape, 0, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Refine Mesh 2D failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalRefineMesh2"_ustr, GEO_NODE_CGAL_REFINE_MESH_2);
  ntype.ui_name = "Refine Mesh 2D";
  ntype.ui_description =
      "Quality 2D Delaunay refinement of XY polygons (CGAL Mesh_2). "
      "Unlike Polygon Fill 2D this inserts Steiner vertices to meet size/shape bounds.";
  ntype.enum_name_legacy = "CGAL_REFINE_MESH_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_refine_mesh_2_cc
