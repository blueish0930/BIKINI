/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_isotropic_remesh_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Surface to remesh. Quads/n-gons are triangulated first.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("New triangle mesh with roughly uniform edge lengths (attributes reprojected).");
  b.add_input<decl::Float>("Edge Length"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description(
          "Target edge length in scene units. Smaller = denser. "
          "Tip: Average Spacing on the same mesh is a good starting value.");
  b.add_input<decl::Int>("Iterations"_ustr)
      .default_value(3)
      .min(1)
      .max(50)
      .description("Remesh passes. 3 is typical; more evens the triangles further.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float edge_length = params.extract_input<float>("Edge Length"_ustr);
  const int iterations = params.extract_input<int>("Iterations"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) {
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  std::string error;
  Mesh *mesh = geometry::cgal_mesh_isotropic_remesh(*mesh_in, edge_length, iterations, error);
  if (mesh == nullptr || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("CGAL Isotropic Remesh failed") : error);
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
  geo_node_type_base(
      &ntype, "GeometryNodeCgalIsotropicRemesh"_ustr, GEO_NODE_CGAL_ISOTROPIC_REMESH);
  ntype.ui_name = "Isotropic Remesh";
  ntype.ui_description =
      "Rebuild a surface as even-sized triangles (CGAL isotropic_remeshing). "
      "How to use: plug a Mesh → set Edge Length to the desired triangle size → "
      "raise Iterations if the result is still uneven. "
      "Different from Surface Delaunay Remesh (quality Delaunay refinement) "
      "and Split Long Edges (only splits, never collapses).";
  ntype.enum_name_legacy = "CGAL_ISOTROPIC_REMESH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_isotropic_remesh_cc
