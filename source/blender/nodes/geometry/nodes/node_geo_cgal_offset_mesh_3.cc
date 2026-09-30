/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Offset / inset a triangle mesh by extracting an isosurface of the
 * AABB signed-distance field. Distinct from Alpha Wrap and Offset Convex 3D.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_offset_mesh_3_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle mesh to offset. Signed mode needs a closed solid.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Isosurface at the requested offset.");
  b.add_input<decl::Float>("Offset"_ustr)
      .default_value(0.1f)
      .subtype(PROP_DISTANCE)
      .description("Positive grows a closed solid; negative shrinks it. Unsigned uses |Offset|.");
  b.add_input<decl::Int>("Resolution"_ustr)
      .default_value(24)
      .min(8)
      .max(64)
      .description("Grid cells along the longest bbox axis.");
  b.add_input<decl::Bool>("Signed"_ustr)
      .default_value(true)
      .description("Use inside/outside sign. Off = offset both sides of an open shell.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float offset = params.extract_input<float>("Offset"_ustr);
  const int resolution = params.extract_input<int>("Resolution"_ustr);
  const bool signed_distance = params.extract_input<bool>("Signed"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info, TIP_("Offset Mesh 3D needs a mesh with faces"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_offset_sdf(*mesh, offset, resolution, signed_distance, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Offset Mesh 3D failed") : error);
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
  /* Deleted: Offset Mesh 3D. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalOffsetMesh3"_ustr, GEO_NODE_CGAL_OFFSET_MESH_3);
  ntype.ui_name = "Offset Mesh 3D";
  ntype.ui_description =
      "Grow or shrink a triangle mesh by extracting an isosurface of the "
      "AABB signed-distance field. Alpha Wrap is an outward envelope that "
      "closes holes; this follows the surface inward and outward. "
      "Offset Convex 3D (deleted) moved planes of a convex solid.";
  ntype.enum_name_legacy = "CGAL_OFFSET_MESH_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_offset_mesh_3_cc
