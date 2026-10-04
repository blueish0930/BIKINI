/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_segmentation_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Closed mesh to segment.");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous()
      .description("Same mesh with Face attributes \"Segment\" and \"SDF\".");
  b.add_input<decl::Int>("Clusters"_ustr).default_value(5).min(2).max(64)
      .description("Target number of parts (higher = more pieces).");
  b.add_input<decl::Float>("Smoothing"_ustr)
      .default_value(0.26f)
      .min(0.0f)
      .max(1.0f)
      .description("SDF field smoothing (higher blends segment boundaries).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const int clusters = params.extract_input<int>("Clusters"_ustr);
  const float smoothing = params.extract_input<float>("Smoothing"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *mesh = geometry::cgal_mesh_segmentation(*mesh_in, clusters, smoothing, error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Segmentation failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalSegmentation"_ustr, GEO_NODE_CGAL_SEGMENTATION);
  ntype.ui_name = "Segment Mesh";
  ntype.ui_description =
      "Segment a closed mesh into parts using Shape Diameter Function (SDF) + clustering. Writes Face attributes \"Segment\" and \"SDF\".";
  ntype.enum_name_legacy = "CGAL_SEGMENTATION";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_segmentation_cc
