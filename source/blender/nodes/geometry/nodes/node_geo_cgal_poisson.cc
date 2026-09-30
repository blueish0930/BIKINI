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

namespace blender::nodes::node_geo_cgal_poisson_cc {

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



static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Oriented point cloud (\"normal\" attribute recommended).");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous()
      .description("Poisson-reconstructed triangle surface.");
  b.add_input<decl::Float>("Spacing"_ustr).default_value(0.0f).min(0.0f)
      .description("Target sample spacing for the octree (0 = auto). Smaller = more detail / heavier.");
  b.add_input<decl::Int>("Neighbors"_ustr).default_value(18).min(6).max(128)
      .description("If normals missing: k used to estimate them.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float spacing = params.extract_input<float>("Spacing"_ustr);
  const int k = params.extract_input<int>("Neighbors"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 10) {
    params.error_message_add(NodeWarningType::Info, TIP_("Poisson needs at least 10 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  Array<float3> normals_buf;
  Span<float3> normals;
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    if (const VArray n_varray = *pc->attributes().lookup<float3>("normal")) {
      if (n_varray.size() == pts.size()) {
        normals_buf.reinitialize(pts.size());
        n_varray.materialize(normals_buf.as_mutable_span());
        normals = normals_buf.as_span();
      }
    }
  }
  std::string error;
  Mesh *mesh = geometry::cgal_points_poisson(pts, normals, spacing, k, error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Poisson reconstruction failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalPoisson"_ustr, GEO_NODE_CGAL_POISSON);
  ntype.ui_name = "Poisson Reconstruct";
  ntype.ui_description =
      "Poisson surface reconstruction from oriented points. Uses attribute \"normal\" when present, else estimates.";
  ntype.enum_name_legacy = "CGAL_POISSON";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_poisson_cc
