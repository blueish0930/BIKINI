/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL Min_annulus_d — min enclosing annulus of projected points.
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_min_annulus_2_cc {

static Span<float3> positions_from_geometry(const GeometrySet &geometry)
{
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    return pc->positions();
  }
  if (const Mesh *mesh = geometry.get_mesh()) {
    return mesh->vert_positions();
  }
  return {};
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Points to enclose.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Annulus as a ring of quads (outer-inner).");
  b.add_output<decl::Vector>("Center"_ustr)
      .description("Common center of the inner and outer circles.");
  b.add_output<decl::Float>("Outer Radius"_ustr)
      .description("Radius of the outer enclosing circle.");
  b.add_output<decl::Float>("Inner Radius"_ustr)
      .description("Radius of the inner empty circle (0 if a point sits at the center).");
  b.add_input<decl::Int>("Segments"_ustr)
      .default_value(48)
      .min(8)
      .max(128)
      .description("How many quads approximate the ring. Higher = rounder preview.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const int plane = 0; /* XY only. */
  const int segments = params.extract_input<int>("Segments"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.error_message_add(NodeWarningType::Info, TIP_("Min Annulus 2D needs at least 3 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Center"_ustr, float3(0.0f));
    params.set_output("Outer Radius"_ustr, 0.0f);
    params.set_output("Inner Radius"_ustr, 0.0f);
    return;
  }
  std::string error;
  float3 center(0.0f);
  float outer_r = 0.0f;
  float inner_r = 0.0f;
  Mesh *out = geometry::cgal_points_min_annulus_2(
      pts, plane, segments, center, outer_r, inner_r, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Min Annulus 2D failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Center"_ustr, float3(0.0f));
    params.set_output("Outer Radius"_ustr, 0.0f);
    params.set_output("Inner Radius"_ustr, 0.0f);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Center"_ustr, center);
  params.set_output("Outer Radius"_ustr, outer_r);
  params.set_output("Inner Radius"_ustr, inner_r);
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalMinAnnulus2"_ustr, GEO_NODE_CGAL_MIN_ANNULUS_2);
  ntype.ui_name = "Min Annulus 2D";
  ntype.ui_description =
      "Thinnest ring (annulus) that contains every XY point "
      "(CGAL Min_annulus_d). How to use: plug Points → get a ring Mesh plus "
      "Center / Outer Radius / Inner Radius. Useful to measure how far a cloud "
      "is from a circle. Different from Min Circle 2D (solid disk, no inner hole).";
  ntype.enum_name_legacy = "CGAL_MIN_ANNULUS_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_min_annulus_2_cc
