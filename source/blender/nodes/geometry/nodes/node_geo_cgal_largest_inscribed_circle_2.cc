/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Largest inscribed circle of XY border polygons.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "DNA_mesh_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_largest_inscribed_circle_2_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("XY polygon (open border or filled). Closed 3D meshes use the XY silhouette.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Disk tangent to the XY boundary (largest inscribed circle).");
  b.add_output<decl::Vector>("Center"_ustr)
      .description("Center of the largest circle that fits inside the polygon.");
  b.add_output<decl::Float>("Radius"_ustr)
      .description("Radius of that inscribed circle.");
  b.add_input<decl::Int>("Segments"_ustr)
      .default_value(48)
      .min(8)
      .max(128)
      .description("How many sides approximate the preview disk.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Mesh"_ustr);
  const int segments = params.extract_input<int>("Segments"_ustr);
  const Mesh *mesh_in = geometry.get_mesh();
  if (!mesh_in || mesh_in->verts_num < 3) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Largest Inscribed Circle 2D needs a polygon mesh"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Center"_ustr, float3(0.0f));
    params.set_output("Radius"_ustr, 0.0f);
    return;
  }
  std::string error;
  float3 center(0.0f);
  float radius = 0.0f;
  Mesh *out = geometry::cgal_mesh_largest_inscribed_circle_2(
      *mesh_in, segments, center, radius, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(
        NodeWarningType::Warning,
        error.empty() ? TIP_("Largest Inscribed Circle 2D failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Center"_ustr, float3(0.0f));
    params.set_output("Radius"_ustr, 0.0f);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Center"_ustr, center);
  params.set_output("Radius"_ustr, radius);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype,
                     "GeometryNodeCgalLargestInscribedCircle2"_ustr,
                     GEO_NODE_CGAL_LARGEST_INSCRIBED_CIRCLE_2);
  ntype.ui_name = "Largest Inscribed Circle 2D";
  ntype.ui_description =
      "Largest circle that fits inside the XY polygon and is tangent to the boundary "
      "(segment Voronoi / medial-axis vertex). Holes are obstacles. "
      "Use a 2D filled or border polygon; closed 3D meshes use the XY silhouette.";
  ntype.enum_name_legacy = "CGAL_LARGEST_INSCRIBED_CIRCLE_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_largest_inscribed_circle_2_cc
