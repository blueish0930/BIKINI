/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Intersect 3D curves with a triangle mesh (CGAL AABB Segment_3).
 * Distinct from Mesh Intersection (two surfaces) and Mesh Slicer (plane).
 */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_curve_mesh_intersect_cc {

static Curves *polylines_to_curves(const Span<Vector<float3>> polylines)
{
  int points_num = 0;
  int curves_num = 0;
  for (const Vector<float3> &pl : polylines) {
    if (pl.size() >= 2) {
      points_num += pl.size();
      curves_num++;
    }
  }
  if (curves_num == 0 || points_num < 2) {
    return bke::curves_new_nomain(0, 0);
  }
  Curves *curves_id = bke::curves_new_nomain(points_num, curves_num);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  MutableSpan<int> offsets = curves.offsets_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();
  int p = 0;
  int c = 0;
  offsets[0] = 0;
  for (const Vector<float3> &pl : polylines) {
    if (pl.size() < 2) {
      continue;
    }
    for (const float3 &pt : pl) {
      positions[p++] = pt;
    }
    c++;
    offsets[c] = p;
  }
  curves.fill_curve_types(CURVE_TYPE_POLY);
  return curves_id;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Triangle surface to intersect against.");
  b.add_input<decl::Geometry>("Curves"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Curve)
      .description("Polylines. Each segment is tested against the mesh.");
  b.add_output<decl::Geometry>("Curves"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Coplanar overlapping pieces as poly curves.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .description("Isolated pierce points where a segment crosses a face.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet mesh_set = params.extract_input<GeometrySet>("Mesh"_ustr);
  GeometrySet curve_set = params.extract_input<GeometrySet>("Curves"_ustr);
  const Mesh *mesh = mesh_set.get_mesh();
  const Curves *curves_id = curve_set.get_curves();
  if (!mesh || mesh->faces_num == 0) {
    params.error_message_add(NodeWarningType::Info, TIP_("Curve Mesh Intersect needs a mesh"));
    params.set_output("Curves"_ustr, GeometrySet());
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  if (!curves_id) {
    params.error_message_add(NodeWarningType::Info, TIP_("Curve Mesh Intersect needs Curves"));
    params.set_output("Curves"_ustr, GeometrySet());
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }
  const bke::CurvesGeometry &cg = curves_id->geometry.wrap();
  if (cg.points_num() < 2 || cg.curves_num() < 1) {
    params.error_message_add(NodeWarningType::Info, TIP_("Need at least one polyline"));
    params.set_output("Curves"_ustr, GeometrySet());
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }

  const OffsetIndices points_by_curve = cg.points_by_curve();
  const Span<float3> pos = cg.positions();
  const VArray cyclic = cg.cyclic();
  Vector<float3> xyz;
  Vector<int> offsets;
  offsets.append(0);
  xyz.reserve(cg.points_num() + cg.curves_num());
  for (const int c : cg.curves_range()) {
    const IndexRange r = points_by_curve[c];
    if (r.size() < 2) {
      offsets.append(xyz.size());
      continue;
    }
    for (const int i : r) {
      xyz.append(pos[i]);
    }
    if (cyclic[c]) {
      xyz.append(pos[r.first()]);
    }
    offsets.append(xyz.size());
  }

  std::string error;
  Vector<Vector<float3>> segments;
  Vector<float3> points;
  if (!geometry::cgal_mesh_curve_intersect(*mesh, xyz, offsets, segments, points, error)) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Curve Mesh Intersect failed") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    params.set_output("Points"_ustr, GeometrySet());
    return;
  }

  Curves *out_curves = polylines_to_curves(segments);
  if (out_curves->geometry.wrap().is_empty() && points.is_empty()) {
    params.error_message_add(NodeWarningType::Info, TIP_("No curve-mesh intersections"));
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(out_curves));
  if (points.is_empty()) {
    params.set_output("Points"_ustr, GeometrySet());
  }
  else {
    PointCloud *pc = BKE_pointcloud_new_nomain(PointCloudType::Points, points.size());
    pc->positions_for_write().copy_from(points);
    params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
  }
}

static void node_register()
{
  /* Deleted: Curve Mesh Intersect. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalCurveMeshIntersect"_ustr, GEO_NODE_CGAL_CURVE_MESH_INTERSECT);
  ntype.ui_name = "Curve Mesh Intersect";
  ntype.ui_description =
      "Intersect 3D polylines with a triangle mesh (CGAL AABB Segment_3 vs faces). "
      "Coplanar overlaps become Curves; isolated pierce points become a Point Cloud. "
      "Mesh Intersection is two surfaces; Mesh Slicer is an infinite plane.";
  ntype.enum_name_legacy = "CGAL_CURVE_MESH_INTERSECT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_curve_mesh_intersect_cc
