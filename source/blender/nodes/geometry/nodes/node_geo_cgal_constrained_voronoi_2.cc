/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Interior constrained Voronoi diagram of XY border polygons.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_constrained_voronoi_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("XY polygon (filled or border). Holes are obstacles.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Interior Voronoi wire: each cell is closer to one vertex than to others.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in || mesh_in->verts_num < 3) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Constrained Voronoi 2D needs a polygon mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_constrained_voronoi_2(*mesh_in, error);
  if (!out || out->edges_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Constrained Voronoi 2D failed") : error);
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
  /* Deleted: Constrained Voronoi 2D. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalConstrainedVoronoi2"_ustr, GEO_NODE_CGAL_CONSTRAINED_VORONOI_2);
  ntype.ui_name = "Constrained Voronoi 2D";
  ntype.ui_description =
      "Interior Voronoi diagram of an XY polygon (clipped to the outline). "
      "Use a 2D filled shape or a border loop. Output is a wire mesh of cells: "
      "each cell is the region closer to one polygon vertex than to the others. "
      "Typical uses: floor-plan rooms, offset guides, path-like skeletons.";
  ntype.enum_name_legacy = "CGAL_CONSTRAINED_VORONOI_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_constrained_voronoi_2_cc
