/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Neighbor graph of 3D points: k-nearest or radius (merged KNN / Radius Graph).
 * Output vertices are 1:1 with input points so point attributes are preserved.
 */

#include "BKE_attribute.hh"
#include "BKE_attribute_filters.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "NOD_socket_usage_inference.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_knn_graph_3_cc {

enum class Mode {
  Knn = 0,
  Radius = 1,
};

static Span<float3> positions_from_geometry(const GeometrySet &geometry)
{
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    return pc->positions();
  }
  if (const Mesh *mesh = geometry.get_mesh()) {
    return mesh->vert_positions();
  }
  return {};
}

static void copy_input_point_attributes_to_mesh(const GeometrySet &geometry, Mesh &mesh)
{
  const Span<StringRef> skip{"position"};
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    if (pc->totpoint != mesh.verts_num) {
      return;
    }
    bke::copy_attributes(pc->attributes(),
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         bke::attribute_filter_from_skip_ref(skip),
                         mesh.attributes_for_write());
    return;
  }
  if (const Mesh *src_mesh = geometry.get_mesh()) {
    if (src_mesh->verts_num != mesh.verts_num) {
      return;
    }
    bke::copy_attributes(src_mesh->attributes(),
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         bke::attribute_filter_from_skip_ref(skip),
                         mesh.attributes_for_write());
  }
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  static const EnumPropertyItem mode_items[] = {
      {int(Mode::Knn),
       "KNN",
       0,
       N_("K Nearest"),
       N_("Connect each point to its k nearest neighbors")},
      {int(Mode::Radius),
       "RADIUS",
       0,
       N_("Radius"),
       N_("Connect every pair closer than Radius")},
      {0, nullptr, 0, nullptr, nullptr},
  };
  b.add_input<decl::Menu>("Mode"_ustr)
      .static_items(mode_items)
      .default_value(MenuValue(Mode::Knn))
      .optional_label()
      .description("K nearest neighbors or radius graph.");
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("3D sites. Output vertices match this point order.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Wire mesh. Vertices are 1:1 with input points (attributes copied).");
  auto &k_in = b.add_input<decl::Int>("K"_ustr)
                   .default_value(8)
                   .min(1)
                   .max(64)
                   .description("Neighbors per point (self is excluded).");
  k_in.usage_inference([](const socket_usage_inference::SocketUsageParams &params) {
    return params.menu_input_may_be("Mode"_ustr, int(Mode::Knn));
  });
  auto &radius_in = b.add_input<decl::Float>("Radius"_ustr)
                        .default_value(1.0f)
                        .min(0.0f)
                        .subtype(PROP_DISTANCE)
                        .description("Connect pairs closer than this distance.");
  radius_in.usage_inference([](const socket_usage_inference::SocketUsageParams &params) {
    return params.menu_input_may_be("Mode"_ustr, int(Mode::Radius));
  });
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const Mode mode = params.extract_input<Mode>("Mode"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 2) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Neighbor Graph 3D needs at least 2 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = nullptr;
  if (mode == Mode::Radius) {
    const float radius = params.extract_input<float>("Radius"_ustr);
    out = geometry::cgal_points_radius_graph_3(pts, radius, error);
  }
  else {
    const int k = params.extract_input<int>("K"_ustr);
    out = geometry::cgal_points_knn_graph_3(pts, k, error);
  }
  if (!out || out->edges_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Neighbor Graph 3D failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  copy_input_point_attributes_to_mesh(geometry, *out);
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalKnnGraph3"_ustr, GEO_NODE_CGAL_KNN_GRAPH_3);
  ntype.ui_name = "Neighbor Graph 3D";
  ntype.ui_description =
      "Connect 3D points into a wire mesh. Mode K Nearest uses k neighbors "
      "(CGAL Orthogonal_k_neighbor_search); Mode Radius uses a distance threshold "
      "(Kd_tree + Fuzzy_sphere). Vertices keep the input point order and attributes.";
  ntype.enum_name_legacy = "CGAL_KNN_GRAPH_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_knn_graph_3_cc
