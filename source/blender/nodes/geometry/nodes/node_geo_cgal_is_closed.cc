/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_is_closed_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to test.");
  b.add_output<decl::Bool>("Closed"_ustr)
      .description("True when there is no border.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m) { params.set_output("Closed"_ustr, false); return; }
  std::string error;
  const bool v = geometry::cgal_mesh_is_closed(*m, error);
  if (!error.empty()) params.error_message_add(NodeWarningType::Warning, error);
  params.set_output("Closed"_ustr, v);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalIsClosed"_ustr, GEO_NODE_CGAL_IS_CLOSED);
  ntype.ui_name = "Is Closed";
  ntype.ui_description =
      "True if the mesh has no boundary edges (topologically closed). Does not prove it is a solid without self-intersections.";
  ntype.enum_name_legacy = "CGAL_IS_CLOSED";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_is_closed_cc
