/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_sphere_intersect_cc {


static Curves *polylines_to_curves(const Span<Vector<float3>> polylines)
{
  int points_num = 0, curves_num = 0;
  for (const Vector<float3> &pl : polylines) {
    if (pl.size() >= 2) {
      points_num += pl.size();
      curves_num++;
    }
  }
  if (curves_num == 0) {
    return bke::curves_new_nomain(0, 0);
  }
  Curves *curves_id = bke::curves_new_nomain(points_num, curves_num);
  bke::CurvesGeometry &curves = curves_id->geometry.wrap();
  MutableSpan<int> offsets = curves.offsets_for_write();
  MutableSpan<float3> positions = curves.positions_for_write();
  int p = 0, c = 0;
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
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Sphere centers (point cloud or mesh vertices).");
  b.add_output<decl::Geometry>("Curves"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Intersection circles as curves.");
  b.add_input<decl::Float>("Radius"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .description("Used when the Radius field is unused.");
  b.add_input<decl::Float>("Point Radius"_ustr)
      .default_value(1.0f)
      .hide_value()
      .structure_type(StructureType::Field)
      .description("Per-point sphere radius field.");
  b.add_input<decl::Int>("Segments"_ustr)
      .default_value(32)
      .min(8)
      .max(256)
      .description("Segments per intersection circle.");
}
static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet g = params.extract_input<GeometrySet>("Points"_ustr);
  const float uniform = params.extract_input<float>("Radius"_ustr);
  Field<float> rad_f = params.extract_input<Field<float>>("Point Radius"_ustr);
  const int seg = params.extract_input<int>("Segments"_ustr);
  Span<float3> pts;
  int n = 0;
  const PointCloud *pc = g.get_pointcloud();
  const Mesh *mesh = g.get_mesh();
  if (pc) { pts = pc->positions(); n = pc->totpoint; }
  else if (mesh) { pts = mesh->vert_positions(); n = mesh->verts_num; }
  if (n < 2) {
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  Array<float> radii(n, uniform);
  if (pc) {
    const bke::PointCloudFieldContext ctx{*pc};
    fn::FieldEvaluator ev{ctx, n};
    ev.add_with_destination(rad_f, radii.as_mutable_span());
    ev.evaluate();
  }
  std::string error;
  Vector<Vector<float3>> pl;
  if (!geometry::cgal_points_sphere_intersect(pts, radii, uniform, seg, pl, error)) {
    params.error_message_add(NodeWarningType::Info, error.empty() ? TIP_("No intersecting spheres") : error);
    params.set_output("Curves"_ustr, GeometrySet());
    return;
  }
  params.set_output("Curves"_ustr, GeometrySet::from_curves(polylines_to_curves(pl)));
}


static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalSphereIntersect"_ustr, GEO_NODE_CGAL_SPHERE_INTERSECT);
  ntype.ui_name = "Sphere Intersect";
  ntype.ui_description = "Intersection circles of spheres centered at points (Circular kernel constructions).";
  ntype.enum_name_legacy = "CGAL_SPHERE_INTERSECT";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_sphere_intersect_cc
