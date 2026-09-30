/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_approx_convex_decomp_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Closed triangle mesh. Open meshes should be Alpha-wrapped first.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Convex volume islands covering the input. Face attribute Part.");
  b.add_input<decl::Int>("Count"_ustr)
      .default_value(8)
      .min(1)
      .max(256)
      .description("Maximum number of convex parts.");
  b.add_input<decl::Int>("Resolution"_ustr)
      .default_value(32)
      .min(8)
      .max(100)
      .description(
          "Cells along the longest bounding-box axis. The other two axes scale with bbox size, "
          "so a long thin mesh is not cubed. 32 is interactive; 100 is tighter and slower.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Mesh"_ustr);
  const int count = params.extract_input<int>("Count"_ustr);
  const int resolution = params.extract_input<int>("Resolution"_ustr);
  const Mesh *m = g.get_mesh();
  if (!m || m->faces_num < 1) {
    params.error_message_add(
        NodeWarningType::Info, TIP_("Approximate Convex Decomposition needs a mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_mesh_approximate_convex_decomposition(*m, count, resolution, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(
        NodeWarningType::Warning,
        error.empty() ? TIP_("Approximate Convex Decomposition failed") : error);
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
  geo_node_type_base(
      &ntype, "GeometryNodeCgalApproxConvexDecomp"_ustr, GEO_NODE_CGAL_APPROX_CONVEX_DECOMP);
  ntype.ui_name = "Approximate Convex Decomposition";
  ntype.ui_description =
      "Cover a closed mesh with a bounded number of convex volumes (CGAL 6.2 voxel ACD). Face "
      "attribute Part. Resolution is cells on the longest bbox axis; the other axes follow the "
      "bbox. Distinct from Convex Decomposition 3, which is exact Nef.";
  ntype.enum_name_legacy = "CGAL_APPROX_CONVEX_DECOMP";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_approx_convex_decomp_cc
