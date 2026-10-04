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

namespace blender::nodes::node_geo_cgal_region_growing_cc {



static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Surface mesh to segment into planar patches.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Same mesh with Face int \"Region\" (planar patch id).");
  b.add_output<decl::Int>("Region Count"_ustr)
      .description("Number of planar regions found.");
  b.add_input<decl::Float>("Max Distance"_ustr)
      .default_value(0.01f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Max point-to-plane distance when growing (scene units). Larger tolerates more noise/bend.");
  b.add_input<decl::Float>("Max Angle"_ustr)
      .default_value(25.0f)
      .min(0.0f)
      .max(90.0f)
      .description("Max degrees between face normal and region plane. Smaller = stricter planes (e.g. 10-20 for CAD); larger merges more.");
  b.add_input<decl::Int>("Min Region Size"_ustr)
      .default_value(1)
      .min(1)
      .max(1000000)
      .description("Drop regions with fewer faces than this (filters scraps).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float max_d = params.extract_input<float>("Max Distance"_ustr);
  const float max_a = params.extract_input<float>("Max Angle"_ustr);
  const int min_s = params.extract_input<int>("Min Region Size"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m) {
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Region Count"_ustr, 0);
    return;
  }
  std::string error;
  int count = 0;
  Mesh *out = geometry::cgal_mesh_region_growing(*m, max_d, max_a, min_s, count, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Region Growing failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Region Count"_ustr, 0);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Region Count"_ustr, count);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalRegionGrowing"_ustr, GEO_NODE_CGAL_REGION_GROWING);
  ntype.ui_name = "Region Growing";
  ntype.ui_description =
      "Grow planar face regions. Writes Face int \"Region\" (0,1,2...; -1 unassigned). Topology unchanged — label only. Color by Named Attribute on Face domain.";
  ntype.enum_name_legacy = "CGAL_REGION_GROWING";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_region_growing_cc
