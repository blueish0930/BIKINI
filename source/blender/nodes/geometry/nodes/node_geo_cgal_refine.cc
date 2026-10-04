/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_refine_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle mesh to densify. Topology is refined (new verts/faces).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Denser triangle mesh. Shape stays close to the input.");
  b.add_input<decl::Float>("Density Factor"_ustr)
      .default_value(2.0f)
      .min(1.0f)
      .max(16.0f)
      .description(
          "How much to densify. 1 = almost no change; 2 ≈ double the local density "
          "(CGAL default is ~1.41). Higher = more faces.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float density = params.extract_input<float>("Density Factor"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) { params.set_output("Mesh"_ustr, GeometrySet()); return; }
  std::string error;
  Mesh *mesh = geometry::cgal_mesh_refine(*mesh_in, density, error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("CGAL Refine failed") : error);
    if (mesh) { BKE_id_free(nullptr, mesh); }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalRefine"_ustr, GEO_NODE_CGAL_REFINE);
  ntype.ui_name = "Refine";
  ntype.ui_description =
      "Densify a triangle mesh by inserting vertices (CGAL PMP refine). "
      "How to use: plug a Mesh → raise Density Factor until you have enough faces "
      "(2 is a typical first try). Shape is kept; only density changes. "
      "Different from Isotropic Remesh (rebuilds to a target edge length) "
      "and Split Long Edges (only splits edges over a length).";
  ntype.enum_name_legacy = "CGAL_REFINE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_refine_cc
