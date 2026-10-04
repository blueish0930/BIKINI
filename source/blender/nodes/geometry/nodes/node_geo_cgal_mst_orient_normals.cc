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

namespace blender::nodes::node_geo_cgal_mst_orient_normals_cc {


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

static bool read_normals(const GeometrySet &geometry,
                         const Span<float3> pts,
                         Array<float3> &buf,
                         Span<float3> &out)
{
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    if (const VArray n = *pc->attributes().lookup<float3>("normal")) {
      if (n.size() == pts.size()) {
        buf.reinitialize(pts.size());
        n.materialize(buf.as_mutable_span());
        out = buf.as_span();
        return true;
      }
    }
  }
  return false;
}


static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Point cloud that already has float3 attribute \"normal\".");
  b.add_output<decl::Geometry>("Points"_ustr).propagate_all_geometry().align_with_previous()
      .description("Same points with consistently oriented \"normal\".");
  b.add_input<decl::Int>("Neighbors"_ustr).default_value(18).min(6).max(128)
      .description("k for building the MST adjacency (typical 12-24).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int k = params.extract_input<int>("Neighbors"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  Array<float3> nbuf;
  Span<float3> normals;
  if (pts.size() < 3 || !read_normals(geometry, pts, nbuf, normals)) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Needs points with normal attribute"));
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_points_mst_orient_normals(
      pts, normals, k, geometry.get_pointcloud(), error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("MST Orient Normals failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMstOrientNormals"_ustr, GEO_NODE_CGAL_MST_ORIENT_NORMALS);
  ntype.ui_name = "MST Orient Normals";
  ntype.ui_description =
      "Flip existing normals so they are consistent via a minimum spanning tree. Does not compute normals from scratch — run Estimate/PCA/VCM first.";
  ntype.enum_name_legacy = "CGAL_MST_ORIENT_NORMALS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_mst_orient_normals_cc
