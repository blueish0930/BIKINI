/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * 3D alpha complex as tetrahedron shells (volume), not the Alpha Shape surface.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_alpha_complex_3_cc {

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
      .description("Input points (point cloud or mesh verts).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Tetrahedra of the alpha complex as triangle shells. Face attr alpha_tet.");
  b.add_output<decl::Float>("Alpha Used"_ustr).description("Actual alpha value used.");
  b.add_output<decl::Int>("Tets"_ustr).description("Number of tetrahedra in the complex.");
  b.add_input<decl::Bool>("Optimal Alpha"_ustr)
      .default_value(true)
      .description("Pick an alpha that yields one solid component.");
  b.add_input<decl::Float>("Alpha"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description("Manual alpha (ignored when Optimal Alpha is on). 0 ≈ bbox diagonal / 20.");
  b.add_input<decl::Bool>("Separate Tetrahedra"_ustr)
      .default_value(true)
      .description("On: each tet is an independent island. Off: weld shared vertices.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const bool optimal = params.extract_input<bool>("Optimal Alpha"_ustr);
  const float alpha = params.extract_input<float>("Alpha"_ustr);
  const bool separate = params.extract_input<bool>("Separate Tetrahedra"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Alpha Complex 3D needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Alpha Used"_ustr, 0.0f);
    params.set_output("Tets"_ustr, 0);
    return;
  }
  std::string error;
  float alpha_used = 0.0f;
  int tets = 0;
  Mesh *out = geometry::cgal_points_alpha_complex_3(
      pts, alpha, optimal, separate, alpha_used, tets, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Alpha Complex 3D failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Alpha Used"_ustr, alpha_used);
    params.set_output("Tets"_ustr, 0);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Alpha Used"_ustr, alpha_used);
  params.set_output("Tets"_ustr, tets);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalAlphaComplex3"_ustr, GEO_NODE_CGAL_ALPHA_COMPLEX_3);
  ntype.ui_name = "Alpha Complex 3D";
  ntype.ui_description =
      "3D alpha complex as tetrahedron shells (CGAL Alpha_shape_3 cells). "
      "Unlike Alpha Shape (surface only) this exports the volume tets. Face attribute alpha_tet.";
  ntype.enum_name_legacy = "CGAL_ALPHA_COMPLEX_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_alpha_complex_3_cc
