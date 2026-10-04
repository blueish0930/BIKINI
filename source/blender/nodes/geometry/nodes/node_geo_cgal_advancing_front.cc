/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_advancing_front_cc {

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
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Input point cloud (or mesh verts). No normals required.");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous()
      .description("Reconstructed triangle surface mesh.");
  b.add_input<decl::Float>("Radius Ratio"_ustr)
      .default_value(5.0f)
      .min(1.0f)
      .description("Max circumradius / shortest edge of candidate triangles. Larger allows flatter/skinnier triangles (default ~5).");
  b.add_input<decl::Float>("Beta"_ustr)
      .default_value(0.52f)
      .min(0.01f)
      .max(1.57f)
      .description("Front advance parameter; higher is more aggressive growth (typical 0.5-1.5).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float radius_ratio = params.extract_input<float>("Radius Ratio"_ustr);
  const float beta = params.extract_input<float>("Beta"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info, TIP_("Advancing Front needs >=4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *mesh = geometry::cgal_advancing_front(pts, radius_ratio, beta, error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("CGAL Advancing Front failed") : error);
    if (mesh) { BKE_id_free(nullptr, mesh); }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalAdvancingFront"_ustr, GEO_NODE_CGAL_ADVANCING_FRONT);
  ntype.ui_name = "Advancing Front";
  ntype.ui_description =
      "From a point cloud, grow a triangle surface by advancing a front (CGAL advancing_front_surface_reconstruction). Good for open surfaces; Radius Ratio / Beta control triangle shape and front advance.";
  ntype.enum_name_legacy = "CGAL_ADVANCING_FRONT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_advancing_front_cc
