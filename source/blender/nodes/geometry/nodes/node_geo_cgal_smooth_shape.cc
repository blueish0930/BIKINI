/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL Polygon Mesh Processing: smooth_shape (mean curvature flow).
 *
 * Mesh is unit-normalized before MCF so Time behaves like CGAL docs/examples
 * regardless of Blender object scale.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_smooth_shape_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Input mesh (topology kept; vertices move).");
  b.add_output<decl::Geometry>("Mesh"_ustr).propagate_all_geometry().align_with_previous()
      .description("Smoothed mesh (same topology).");

  /*
   * After unit-normalization, CGAL Time is dimensionless.
   * Default 0.01 is clearly visible on typical models; 1e-4 was nearly invisible.
   */
  b.add_input<decl::Float>("Time"_ustr)
      .default_value(0.05f)
      .min(0.0f)
      .max(1.0f)
      .description("Flow step size. Larger = stronger smoothing (try 0.01-0.1).");
  b.add_input<decl::Int>("Iterations"_ustr)
      .default_value(20)
      .min(1)
      .max(200)
      .description("Number of flow steps. More = smoother, more volume loss.");
  b.add_input<decl::Bool>("Preserve Scale"_ustr)
      .default_value(true)
      .description("If on, restore mesh size after unit-box normalization.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float time = params.extract_input<float>("Time"_ustr);
  const int iterations = params.extract_input<int>("Iterations"_ustr);
  const bool do_scale = params.extract_input<bool>("Preserve Scale"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  if (!(time > 0.0f)) {
    params.error_message_add(
        NodeWarningType::Error,
        TIP_("Time must be > 0. After unit-normalization try 0.01 (default)"));
    params.set_output("Mesh"_ustr, std::move(geometry));
    return;
  }
  if (time > 0.2f) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("Large Time can be unstable; try smaller Time with more Iterations"));
  }

  std::string error;
  Mesh *mesh = geometry::cgal_mesh_smooth_shape(*mesh_in, time, iterations, do_scale, error);
  if (mesh == nullptr || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("CGAL smooth_shape failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalSmoothShape"_ustr, GEO_NODE_CGAL_SMOOTH_SHAPE);
  ntype.ui_name = "Smooth Shape";
  ntype.ui_description =
      "Mean-curvature flow smoothing (PMP smooth_shape). Mesh is temporarily unit-normalized so Time is scale-stable; enable Preserve Scale to restore size.";
  ntype.enum_name_legacy = "CGAL_SMOOTH_SHAPE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_smooth_shape_cc
