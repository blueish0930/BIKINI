/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_alpha_wrap_cc {

static Span<float3> positions_from_geometry(const GeometrySet &geometry)
{
  if (const PointCloud *pc = geometry.get_pointcloud()) { return pc->positions(); }
  if (const Mesh *mesh = geometry.get_mesh()) { return mesh->vert_positions(); }
  return {};
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Geometry"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::PointCloud})
      .description("Mesh or points to wrap.");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous()
      .description("Watertight wrap mesh around the input.");
  b.add_input<decl::Float>("Alpha"_ustr).default_value(0.1f).min(0.0f).subtype(PROP_DISTANCE)
      .description("Feature size / wrap resolution (scene units). Smaller tracks detail more closely but is slower.");
  b.add_input<decl::Float>("Offset"_ustr).default_value(0.0f).min(0.0f).subtype(PROP_DISTANCE)
      .description("Extra outward offset after wrap (scene units). 0 = tight wrap.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry"_ustr);
  const float alpha = params.extract_input<float>("Alpha"_ustr);
  const float offset = params.extract_input<float>("Offset"_ustr);
  std::string error;
  Mesh *mesh = nullptr;
  if (const Mesh *m = geometry.get_mesh()) {
    mesh = geometry::cgal_mesh_alpha_wrap(*m, alpha, offset, error);
  }
  else {
    const Span<float3> pts = positions_from_geometry(geometry);
    if (pts.size() < 4) {
      params.error_message_add(NodeWarningType::Info, TIP_("Alpha Wrap needs a mesh or >=4 points"));
      params.set_output("Mesh"_ustr, GeometrySet());
      return;
    }
    mesh = geometry::cgal_alpha_wrap_points(pts, alpha, offset, error);
  }
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("CGAL Alpha Wrap failed") : error);
    if (mesh) { BKE_id_free(nullptr, mesh); }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalAlphaWrap"_ustr, GEO_NODE_CGAL_ALPHA_WRAP);
  ntype.ui_name = "Alpha Wrap";
  ntype.ui_description =
      "Offset/wrap geometry with a watertight alpha-wrap envelope (CGAL alpha_wrap_3). Useful to close holes and get a robust outer shell.";
  ntype.enum_name_legacy = "CGAL_ALPHA_WRAP";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_alpha_wrap_cc
