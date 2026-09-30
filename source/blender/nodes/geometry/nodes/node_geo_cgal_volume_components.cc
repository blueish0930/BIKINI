/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Volume-connected components of a closed triangle mesh (nested shells).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_volume_components_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Closed triangle mesh. Nested shells become different volumes.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Triangulated mesh. Face attribute volume_id is the enclosed-volume index.");
  b.add_output<decl::Int>("Volume Count"_ustr).description("Number of enclosed volumes.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Volume Components needs a closed mesh with faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Volume Count"_ustr, 0);
    return;
  }
  std::string error;
  int volume_count = 0;
  Mesh *out = geometry::cgal_mesh_volume_components(*mesh, volume_count, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Volume Components failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Volume Count"_ustr, 0);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Volume Count"_ustr, volume_count);
}

static void node_register()
{
  /* Deleted: Volume Components node. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalVolumeComponents"_ustr, GEO_NODE_CGAL_VOLUME_COMPONENTS);
  ntype.ui_name = "Volume Components";
  ntype.ui_description =
      "Classify faces of a closed mesh by the 3D volume they bound "
      "(CGAL PMP volume_connected_components). Nested cubes are two volumes, not one "
      "face-island. Face attribute volume_id. The mesh must be closed.";
  ntype.enum_name_legacy = "CGAL_VOLUME_COMPONENTS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_volume_components_cc
