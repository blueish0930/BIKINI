/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_clip_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle mesh to cut with an infinite plane.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Part of the mesh that lies on the Plane Normal side, capped on the plane.");
  b.add_input<decl::Vector>("Plane Point"_ustr)
      .default_value(float3(0.0f))
      .subtype(PROP_TRANSLATION)
      .description("A point that lies on the cutting plane.");
  b.add_input<decl::Vector>("Plane Normal"_ustr)
      .default_value(float3(0.0f, 0.0f, 1.0f))
      .description(
          "Plane facing. The half-space in this direction is kept "
          "(default +Z keeps everything above the plane).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float3 point = params.extract_input<float3>("Plane Point"_ustr);
  const float3 normal = params.extract_input<float3>("Plane Normal"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in) { params.set_output("Mesh"_ustr, GeometrySet()); return; }
  std::string error;
  Mesh *mesh = geometry::cgal_mesh_clip_plane(*mesh_in, point, normal, error);
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning, error.empty() ? TIP_("CGAL Clip failed") : error);
    if (mesh) { BKE_id_free(nullptr, mesh); }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(mesh));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalClip"_ustr, GEO_NODE_CGAL_CLIP);
  ntype.ui_name = "Clip";
  ntype.ui_description =
      "Cut a mesh with an infinite plane and keep one side "
      "(CGAL PMP clip). How to use: plug a Mesh → set Plane Point on the cut "
      "and Plane Normal toward the part you want to keep. "
      "Unlike Mesh Boolean this uses a plane, not a second solid. "
      "Closed meshes are capped on the cut; open shells are just trimmed.";
  ntype.enum_name_legacy = "CGAL_CLIP";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_clip_cc
