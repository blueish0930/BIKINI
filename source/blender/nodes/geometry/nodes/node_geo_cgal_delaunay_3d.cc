/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_node_runtime.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_delaunay_3d_cc {

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

static void set_tets_if_available(GeoNodeExecParams &params, const int tets)
{
  const bNodeSocket *sock = bke::node_find_socket(params.node(), SOCK_OUT, "Tets"_ustr);
  if (sock != nullptr && sock->is_available()) {
    params.set_output("Tets"_ustr, tets);
  }
}

static void node_declare(NodeDeclarationBuilder &b)
{
  const bool show_domain = show_periodic_domain(b.node_or_null());

  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Input points (point cloud or mesh verts).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Tetrahedra as triangle shells. With Separate Tetrahedra: each tet has its own "
          "4 vertices (no sharing). Off: welded shared vertices.");
  b.add_output<decl::Int>("Tets"_ustr)
      .description("Number of tetrahedra.")
      .available(show_domain);
  b.add_input<decl::Bool>("Separate Tetrahedra"_ustr)
      .default_value(true)
      .description(
          "On: each tetrahedron is an independent island (duplicated verts, can explode/"
          "separate). Off: share vertices between adjacent tets (merged mesh).");
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
      .description("Period depth. 0 = padded point bbox.")
      .available(show_domain);
  b.add_input<decl::Float>("Domain Z"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Period height. 0 = padded point bbox.")
      .available(show_domain);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const bool separate = params.extract_input<bool>("Separate Tetrahedra"_ustr);
  const bool periodic = params.extract_input<bool>("Periodic"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info,
                             periodic ? TIP_("Periodic Delaunay 3D needs at least 4 points") :
                                        TIP_("Delaunay 3D needs >=4 non-coplanar points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    set_tets_if_available(params, 0);
    return;
  }
  std::string error;
  Mesh *mesh = nullptr;
  int tets = 0;
  if (periodic) {
    const float dx = input_float_if_available(params, "Domain X"_ustr);
    const float dy = input_float_if_available(params, "Domain Y"_ustr);
    const float dz = input_float_if_available(params, "Domain Z"_ustr);
    mesh = geometry::cgal_points_periodic_delaunay_3(pts, dx, dy, dz, separate, tets, error);
  }
  else {
    mesh = geometry::cgal_delaunay_3d(
        pts, geometry.get_pointcloud(), geometry.get_mesh(), separate, error);
  }
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ?
                                 (periodic ? TIP_("Periodic Delaunay 3D failed") :
                                             TIP_("CGAL Delaunay 3D failed")) :
                                 error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    set_tets_if_available(params, 0);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
  set_tets_if_available(params, tets);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalDelaunay3D"_ustr, GEO_NODE_CGAL_DELAUNAY_3D);
  ntype.ui_name = "Delaunay 3D";
  ntype.ui_description =
      "3D Delaunay tetrahedralization. Enable Separate Tetrahedra for independent tet "
      "shells (no shared vertices); disable to weld verts into one mesh.";
  ntype.enum_name_legacy = "CGAL_DELAUNAY_3D";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_delaunay_3d_cc
