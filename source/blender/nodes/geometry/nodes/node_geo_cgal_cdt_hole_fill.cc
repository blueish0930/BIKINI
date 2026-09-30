/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Fill boundary holes with a 2D CDT in the fitted plane (3D DT fallback).
 * No refine, no fair. Distinct from Fair Hole Fill.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_cdt_hole_fill_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh with open boundary holes.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Original n-gons kept; new hole patches are triangles.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *mesh = geometry::cgal_mesh_cdt_hole_fill(*mesh_in, error);
  if (mesh == nullptr || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("CDT Hole Fill failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  /* Deleted: CDT Hole Fill. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalCdtHoleFill"_ustr, GEO_NODE_CGAL_CDT_HOLE_FILL);
  ntype.ui_name = "CDT Hole Fill";
  ntype.ui_description =
      "Fill every boundary loop with a constrained Delaunay patch in the "
      "least-squares plane of that loop (CGAL triangulate_hole, 2D CDT). "
      "Non-planar holes fall back to 3D Delaunay. Does not fair or refine — "
      "Fair Hole Fill bulges the patch to match surrounding curvature.";
  ntype.enum_name_legacy = "CGAL_CDT_HOLE_FILL";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_cdt_hole_fill_cc
