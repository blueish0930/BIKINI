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

namespace blender::nodes::node_geo_cgal_delaunay_2_cc {

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

/** Domain sockets stay hidden unless Periodic is checked. A link keeps them visible. */
static bool show_periodic_domain(const bNode *node)
{
  if (node == nullptr) {
    return false;
  }
  const bNodeSocket *sock = bke::node_find_socket(*node, SOCK_IN, "Periodic"_ustr);
  if (sock == nullptr || sock->default_value == nullptr) {
    return false;
  }
  if (sock->link != nullptr) {
    return true;
  }
  return sock->default_value_typed<bNodeSocketValueBoolean>()->value;
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
  const bool show_domain = show_periodic_domain(b.node_or_null());

  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Sites on the XY plane (point cloud or mesh verts). Z is ignored.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Delaunay triangulation as a triangle mesh on XY.");
  b.add_input<decl::Bool>("Periodic"_ustr)
      .default_value(false)
      .description("Use a periodic domain. Domain size is hidden while this is off.");
  b.add_input<decl::Float>("Domain X"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Period width. 0 = padded point bbox.")
      .available(show_domain);
  b.add_input<decl::Float>("Domain Y"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Period height. 0 = padded point bbox.")
      .available(show_domain);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const bool periodic = params.extract_input<bool>("Periodic"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.error_message_add(NodeWarningType::Info,
                             periodic ? TIP_("Periodic Delaunay 2D needs at least 3 points") :
                                        TIP_("Delaunay 2D needs at least 3 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = nullptr;
  if (periodic) {
    const float domain_x = input_float_if_available(params, "Domain X"_ustr);
    const float domain_y = input_float_if_available(params, "Domain Y"_ustr);
    out = geometry::cgal_points_periodic_delaunay_2(pts, 0, domain_x, domain_y, error);
  }
  else {
    out = geometry::cgal_points_delaunay_2(pts, 0, error);
  }
  if (!out || out->faces_num == 0) {
    params.error_message_add(
        NodeWarningType::Warning,
        error.empty() ? (periodic ? TIP_("Periodic Delaunay 2D failed") : TIP_("Delaunay 2D failed")) :
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
  geo_node_type_base(&ntype, "GeometryNodeCgalDelaunay2"_ustr, GEO_NODE_CGAL_DELAUNAY_2);
  ntype.ui_name = "Delaunay 2D";
  ntype.ui_description =
      "Delaunay triangulation of XY points (CGAL Delaunay_triangulation_2). "
      "How to use: plug a Point Cloud → get a triangle mesh that connects every "
      "site with empty circumcircles. Z is ignored. "
      "For constrained edges / holes use Constrained Delaunay 2D; "
      "for a 3D volume use Delaunay 3D.";
  ntype.enum_name_legacy = "CGAL_DELAUNAY_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_delaunay_2_cc
