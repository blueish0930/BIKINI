/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_mesh_volume_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Preferably a closed, consistently oriented solid.");
  b.add_output<decl::Float>("Value"_ustr)
      .description("Volume in scene units cubed.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
  params.set_output("Value"_ustr, 0.0f);
    return;
  }
  std::string error;
  const float v = float(geometry::cgal_mesh_volume(*mesh_in, error));
  if (!error.empty()) {
    params.error_message_add(NodeWarningType::Warning, error);
  }
  params.set_output("Value"_ustr, v);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMeshVolume"_ustr, GEO_NODE_CGAL_MESH_VOLUME);
  ntype.ui_name = "Mesh Volume";
  ntype.ui_description =
      "Compute signed volume of a closed mesh. Open meshes may return 0 or unreliable values.";
  ntype.enum_name_legacy = "CGAL_MESH_VOLUME";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_mesh_volume_cc
