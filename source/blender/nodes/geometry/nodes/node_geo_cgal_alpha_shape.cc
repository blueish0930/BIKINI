/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_alpha_shape_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Input points to wrap with an alpha shape.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Triangle mesh of the alpha-shape surface.");
  b.add_output<decl::Float>("Alpha Used"_ustr)
      .description("Actual alpha value used (manual or auto-chosen).");
  b.add_input<decl::Bool>("Optimal Alpha"_ustr)
      .default_value(true)
      .description("If on, ignore manual Alpha and pick a value that yields Solid Components solid pieces.");
  b.add_input<decl::Int>("Solid Components"_ustr)
      .default_value(1)
      .min(1)
      .max(64)
      .description("When Optimal Alpha is on: target number of solid components (usually 1).");
  b.add_input<decl::Float>("Alpha"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description("Manual alpha radius (scene units). Smaller = tighter / more detail; larger = smoother bulk.");
}

static Span<float3> positions_from_geometry(const GeometrySet &geometry)
{
  if (const PointCloud *pointcloud = geometry.get_pointcloud()) {
    return pointcloud->positions();
  }
  if (const Mesh *mesh = geometry.get_mesh()) {
    return mesh->vert_positions();
  }
  return {};
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const bool optimal = params.extract_input<bool>("Optimal Alpha"_ustr);
  const int solid_components = params.extract_input<int>("Solid Components"_ustr);
  const float alpha = params.extract_input<float>("Alpha"_ustr);
  const Span<float3> positions = positions_from_geometry(geometry);
  if (positions.size() < 4) {
    params.error_message_add(NodeWarningType::Info, TIP_("Alpha Shape needs at least 4 points"));
    params.set_default_remaining_outputs();
    return;
  }

  if (geometry.get_mesh() && !geometry.get_pointcloud()) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("Alpha Shape uses only positions (mesh verts). It will not reconstruct "
             "the original mesh faces. Prefer a Point Cloud; for dense surface "
             "reconstruction use Poisson / Advancing Front later"));
  }

  std::string error;
  float alpha_used = 0.0f;
  Mesh *mesh = geometry::cgal_alpha_shape_3(
      positions, alpha, optimal, solid_components, alpha_used, error);
  if (mesh == nullptr || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("CGAL Alpha Shape failed") : error);
    if (mesh) {
      BKE_id_free(nullptr, mesh);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Alpha Used"_ustr, alpha_used);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
  params.set_output("Alpha Used"_ustr, alpha_used);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalAlphaShape"_ustr, GEO_NODE_CGAL_ALPHA_SHAPE);
  ntype.ui_name = "Alpha Shape";
  ntype.ui_description =
      "Build a 3D alpha-shape surface from points (Delaunay filtration). Smaller Alpha keeps denser parts; Optimal Alpha auto-picks a solid component threshold.";
  ntype.enum_name_legacy = "CGAL_ALPHA_SHAPE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_alpha_shape_cc
