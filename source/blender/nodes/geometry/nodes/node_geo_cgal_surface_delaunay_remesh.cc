/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL PMP surface_Delaunay_remeshing — heavy Mesh_3-based surface remesh.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_surface_delaunay_remesh_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Clean triangle surface (no self-intersections). Heavy remesh.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Remeshed triangle surface from Delaunay refinement.");
  b.add_input<decl::Float>("Facet Size"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .description("Upper bound for surface Delaunay ball radii (0 = ignore).");
  b.add_input<decl::Float>("Facet Angle"_ustr)
      .default_value(25.0f)
      .min(0.0f)
      .max(30.0f)
      .description("Lower bound for facet angles in degrees (0 = ignore).");
  b.add_input<decl::Float>("Facet Distance"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .description("Approximation error bound to the input surface (0 = ignore).");
  b.add_input<decl::Float>("Feature Angle"_ustr)
      .default_value(60.0f)
      .min(0.0f)
      .max(180.0f)
      .description("Dihedral angle (degrees) for sharp feature protection when Protect is on.");
  b.add_input<decl::Bool>("Protect Features"_ustr)
      .default_value(true)
      .description("Protect sharp / border edges during remeshing.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float facet_size = params.extract_input<float>("Facet Size"_ustr);
  const float facet_angle = params.extract_input<float>("Facet Angle"_ustr);
  const float facet_distance = params.extract_input<float>("Facet Distance"_ustr);
  const float feature_angle = params.extract_input<float>("Feature Angle"_ustr);
  const bool protect = params.extract_input<bool>("Protect Features"_ustr);

  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in || mesh_in->faces_num < 1) {
    params.error_message_add(NodeWarningType::Info, TIP_("Surface Delaunay Remesh needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  std::string error;
  Mesh *out = geometry::cgal_mesh_surface_delaunay_remesh(*mesh_in,
                                                          facet_size,
                                                          facet_angle,
                                                          facet_distance,
                                                          feature_angle,
                                                          protect,
                                                          error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Surface Delaunay Remesh failed") : error);
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
  geo_node_type_base(&ntype,
                     "GeometryNodeCgalSurfaceDelaunayRemesh"_ustr,
                     GEO_NODE_CGAL_SURFACE_DELAUNAY_REMESH);
  ntype.ui_name = "Surface Delaunay Remesh";
  ntype.ui_description =
      "Remesh a triangle surface with CGAL surface_Delaunay_remeshing (Mesh_3 criteria). "
      "Heavy; input should be free of self-intersections.";
  ntype.enum_name_legacy = "CGAL_SURFACE_DELAUNAY_REMESH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_surface_delaunay_remesh_cc
