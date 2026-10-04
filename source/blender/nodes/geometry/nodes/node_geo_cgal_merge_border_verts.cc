/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_merge_border_verts_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description(
          "Mesh with open borders whose coincident seam vertices should be welded.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Same mesh with duplicated vertices on each border cycle merged.");
  b.add_input<decl::Float>("Distance"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description(
          "Reserved. Currently unused — only vertices that already occupy "
          "the same location on a border cycle are merged. "
          "Use Snap Borders if the lips are still a gap apart.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  const float distance = params.extract_input<float>("Distance"_ustr);
  std::string error;
  Mesh *mesh = geometry::cgal_mesh_merge_border_vertices(*mesh_in, distance, error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("CGAL Merge Border Vertices failed") : error);
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
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMergeBorderVerts"_ustr, GEO_NODE_CGAL_MERGE_BORDER_VERTS);
  ntype.ui_name = "Merge Border Vertices";
  ntype.ui_description =
      "Weld duplicated vertices that already sit on the same open-border cycle "
      "(CGAL merge_duplicated_vertices_in_boundary_cycles). How to use: plug a mesh "
      "whose seam verts occupy the same location (split UV island, boolean leftover) "
      "→ get a stitched border. Distance is reserved and currently unused. "
      "If the lips are still a gap apart, use Snap Borders first.";
  ntype.enum_name_legacy = "CGAL_MERGE_BORDER_VERTS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_merge_border_verts_cc
