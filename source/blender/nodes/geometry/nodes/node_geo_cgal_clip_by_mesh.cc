/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Keep the part of Mesh inside a closed Clipper (CGAL PMP clip).
 * Distinct from Mesh Boolean (both operands tessellated into the result)
 * and from deleted Split by Mesh (keeps both sides).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_clip_by_mesh_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Mesh to clip.");
  b.add_input<decl::Geometry>("Clipper"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Closed volume. The part of Mesh inside this is kept.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Clipped Mesh.");
  b.add_input<decl::Bool>("Clip Volume"_ustr)
      .default_value(true)
      .description("If Mesh is closed, cap the cut with the clipper so it stays a volume.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet ga = params.extract_input<GeometrySet>("Mesh"_ustr);
  GeometrySet gb = params.extract_input<GeometrySet>("Clipper"_ustr);
  const bool clip_volume = params.extract_input<bool>("Clip Volume"_ustr);
  const Mesh *mesh = ga.get_mesh();
  const Mesh *clipper = gb.get_mesh();
  if (!mesh || !clipper) {
    params.error_message_add(NodeWarningType::Info, TIP_("Clip by Mesh needs Mesh and Clipper"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_clip_by_mesh(*mesh, *clipper, clip_volume, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Clip by Mesh failed") : error);
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
  /* Deleted: Clip by Mesh. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalClipByMesh"_ustr, GEO_NODE_CGAL_CLIP_BY_MESH);
  ntype.ui_name = "Clip by Mesh";
  ntype.ui_description =
      "Keep the part of Mesh inside a closed Clipper (CGAL PMP clip). "
      "Mesh Boolean rebuilds both solids; this only trims Mesh. "
      "Not the deleted Split by Mesh (which kept both sides).";
  ntype.enum_name_legacy = "CGAL_CLIP_BY_MESH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_clip_by_mesh_cc
