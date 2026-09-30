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

namespace blender::nodes::node_geo_cgal_sample_points_cc {

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
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle surface to sample. Area-weighted: larger faces get more points.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Point cloud of random samples on the faces (no normals written).");
  b.add_input<decl::Int>("Count"_ustr)
      .default_value(1000)
      .min(1)
      .max(10000000)
      .description("Number of samples. More = denser cloud (heavier).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet mesh_geo = params.extract_input<GeometrySet>("Mesh"_ustr);
  const int count = params.extract_input<int>("Count"_ustr);
  const Mesh *mesh = mesh_geo.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  std::string error;
  PointCloud *pc = geometry::cgal_mesh_sample_points(*mesh, count, error);
  if (!pc || pc->totpoint == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Sample Points failed") : error);
    if (pc) {
      BKE_id_free(nullptr, pc);
    }
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalSamplePoints"_ustr, GEO_NODE_CGAL_SAMPLE_POINTS);
  ntype.ui_name = "Sample Points";
  ntype.ui_description =
      "Scatter random points on a triangle surface (CGAL sample_triangle_mesh). "
      "How to use: plug a Mesh → set Count → get a Point Cloud. "
      "Good first step before Estimate Normals + Poisson / Advancing Front. "
      "Does not write normals — run Jet / PCA / VCM Estimate Normals next.";
  ntype.enum_name_legacy = "CGAL_SAMPLE_POINTS";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_sample_points_cc
