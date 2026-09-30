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

namespace blender::nodes::node_geo_cgal_centroid_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Mesh"_ustr).only_realized_data().supported_type(GeometryComponent::Type::Mesh)
      .description("Input mesh.");
  b.add_output<decl::Vector>("Centroid"_ustr)
      .description("World-space center point as a vector.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m) { params.set_output("Centroid"_ustr, float3(0)); return; }
  std::string error;
  float3 c(0);
  if (!geometry::cgal_mesh_centroid(*m, c, error)) {
    if (!error.empty()) params.error_message_add(NodeWarningType::Warning, error);
    params.set_output("Centroid"_ustr, float3(0));
    return;
  }
  params.set_output("Centroid"_ustr, c);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalCentroid"_ustr, GEO_NODE_CGAL_CENTROID);
  ntype.ui_name = "Centroid";
  ntype.ui_description =
      "Mesh centroid: volume centroid if closed, otherwise area-weighted surface centroid.";
  ntype.enum_name_legacy = "CGAL_CENTROID";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_centroid_cc
