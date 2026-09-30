/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Keep the part of a mesh inside an axis-aligned box (CGAL PMP clip Iso_cuboid).
 * Distinct from Clip (infinite plane) and Mesh Boolean.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_clip_box_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle mesh to clip.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Part of the mesh inside the box.");
  b.add_input<decl::Vector>("Center"_ustr)
      .default_value(float3(0.0f))
      .subtype(PROP_TRANSLATION)
      .description("Box center.");
  b.add_input<decl::Vector>("Size"_ustr)
      .default_value(float3(2.0f))
      .min(0.0f)
      .subtype(PROP_XYZ)
      .description("Full box size (like Cube).");
  b.add_input<decl::Bool>("Clip Volume"_ustr)
      .default_value(true)
      .description("If the mesh is closed, cap the cut so it stays a volume.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const float3 center = params.extract_input<float3>("Center"_ustr);
  const float3 size = params.extract_input<float3>("Size"_ustr);
  const bool clip_volume = params.extract_input<bool>("Clip Volume"_ustr);
  const Mesh *mesh = geometry.get_mesh();
  if (!mesh) {
    params.error_message_add(NodeWarningType::Info, TIP_("Clip Box needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_clip_box(*mesh, center, size, clip_volume, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Clip Box failed") : error);
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
  /* Deleted: Clip Box. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalClipBox"_ustr, GEO_NODE_CGAL_CLIP_BOX);
  ntype.ui_name = "Clip Box";
  ntype.ui_description =
      "Keep the part of a mesh inside an axis-aligned box "
      "(CGAL PMP clip Iso_cuboid). Distinct from Clip (plane) and Mesh Boolean.";
  ntype.enum_name_legacy = "CGAL_CLIP_BOX";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_clip_box_cc
