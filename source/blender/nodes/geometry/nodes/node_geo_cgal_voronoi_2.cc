/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_node_runtime.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_voronoi_2_cc {

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

/**
 * Domain sockets stay hidden unless Periodic is checked.
 * Padding is the non-periodic clip and is hidden while Periodic is checked.
 * A link on Periodic keeps both, because the value is not known until evaluation.
 */
static void periodic_socket_visibility(const bNode *node, bool &r_show_domain, bool &r_show_padding)
{
  r_show_domain = false;
  r_show_padding = true;
  if (node == nullptr) {
    return;
  }
  const bNodeSocket *sock = bke::node_find_socket(*node, SOCK_IN, "Periodic"_ustr);
  if (sock == nullptr || sock->default_value == nullptr) {
    return;
  }
  if (sock->link != nullptr) {
    r_show_domain = true;
    r_show_padding = true;
    return;
  }
  const bool periodic = sock->default_value_typed<bNodeSocketValueBoolean>()->value;
  r_show_domain = periodic;
  r_show_padding = !periodic;
}

static float input_float_if_available(GeoNodeExecParams &params, const UString identifier)
{
  const bNodeSocket *sock = bke::node_find_socket(params.node(), SOCK_IN, identifier);
  if (sock == nullptr || !sock->is_available()) {
    return 0.0f;
  }
  return params.extract_input<float>(identifier);
}

static void node_declare(NodeDeclarationBuilder &b)
{
  bool show_domain = false;
  bool show_padding = true;
  periodic_socket_visibility(b.node_or_null(), show_domain, show_padding);

  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Sites (point cloud or mesh vertices). Voronoi in XY.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("One isolated n-gon per Voronoi cell. Face attribute cell_id.");
  b.add_input<decl::Float>("Padding"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Outward padding of the input-point bounding box. 0 = clip exactly to the bbox.")
      .available(show_padding);
  b.add_input<decl::Bool>("Periodic"_ustr)
      .default_value(false)
      .description("Use a periodic domain. Domain size is hidden while this is off.");
  b.add_input<decl::Float>("Domain X"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Periodic width. 0 = 1.05 times the XY span.")
      .available(show_domain);
  b.add_input<decl::Float>("Domain Y"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Periodic height. 0 = 1.05 times the XY span.")
      .available(show_domain);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const bool periodic = params.extract_input<bool>("Periodic"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  const int min_points = periodic ? 3 : 2;
  if (pts.size() < min_points) {
    params.error_message_add(NodeWarningType::Info,
                             periodic ? TIP_("Periodic Voronoi 2D needs at least 3 points") :
                                        TIP_("Voronoi 2D needs at least 2 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = nullptr;
  if (periodic) {
    const float dx = input_float_if_available(params, "Domain X"_ustr);
    const float dy = input_float_if_available(params, "Domain Y"_ustr);
    out = geometry::cgal_points_periodic_voronoi_2(pts, dx, dy, error);
  }
  else {
    const float margin = input_float_if_available(params, "Padding"_ustr);
    out = geometry::cgal_points_voronoi_2(pts, 0, margin, error);
  }
  if (!out || out->faces_num == 0) {
    params.error_message_add(
        NodeWarningType::Warning,
        error.empty() ? (periodic ? TIP_("Periodic Voronoi 2D failed") : TIP_("Voronoi 2D failed")) :
                        error);
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
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalVoronoi2"_ustr, GEO_NODE_CGAL_VORONOI_2);
  ntype.ui_name = "Voronoi 2D";
  ntype.ui_description =
      "Voronoi cells of XY points (dual of Delaunay). "
      "Each cell is an isolated n-gon (no shared verts). "
      "Unbounded cells are clipped to the input-point bounding box + Padding. "
      "Face attribute cell_id.";
  ntype.enum_name_legacy = "CGAL_VORONOI_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_voronoi_2_cc
