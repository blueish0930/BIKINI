/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL Min_sphere_of_spheres_d — smallest sphere enclosing input balls.
 * Distinct from Min Sphere (points only, radius 0).
 */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_min_sphere_of_spheres_cc {

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
      .description("Ball centers.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("UV sphere visualizing the minimum enclosing sphere of the balls.");
  b.add_output<decl::Vector>("Center"_ustr).description("Center of the enclosing sphere.");
  b.add_output<decl::Float>("Radius"_ustr).description("Radius of the enclosing sphere.");
  b.add_input<decl::Float>("Ball Radius"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .structure_type(StructureType::Field)
      .description("Input ball radius per point (0 = point).");
  b.add_input<decl::Int>("Segments"_ustr)
      .default_value(24)
      .min(8)
      .max(64)
      .description("Visualization sphere tessellation.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  Field<float> radius_field = params.extract_input<Field<float>>("Ball Radius"_ustr);
  const int segments = params.extract_input<int>("Segments"_ustr);

  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.is_empty()) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Min Sphere of Spheres needs at least 1 ball"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Center"_ustr, float3(0.0f));
    params.set_output("Radius"_ustr, 0.0f);
    return;
  }

  Array<float> radii(pts.size());
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    const bke::PointCloudFieldContext ctx(*pc);
    fn::FieldEvaluator ev(ctx, pc->totpoint);
    ev.add_with_destination(radius_field, radii.as_mutable_span());
    ev.evaluate();
  }
  else if (const Mesh *mesh = geometry.get_mesh()) {
    const bke::MeshFieldContext ctx(*mesh, bke::AttrDomain::Point);
    fn::FieldEvaluator ev(ctx, mesh->verts_num);
    ev.add_with_destination(radius_field, radii.as_mutable_span());
    ev.evaluate();
  }
  else {
    radii.fill(0.1f);
  }

  std::string error;
  float3 center(0.0f);
  float radius = 0.0f;
  Mesh *out = geometry::cgal_points_min_sphere_of_spheres(
      pts, radii, 0.1f, segments, center, radius, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Min Sphere of Spheres failed") : error);
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
  geo_node_type_base(
      &ntype, "GeometryNodeCgalMinSphereOfSpheres"_ustr, GEO_NODE_CGAL_MIN_SPHERE_OF_SPHERES);
  ntype.ui_name = "Min Sphere of Spheres";
  ntype.ui_description =
      "Smallest sphere enclosing input balls with radii (CGAL Min_sphere_of_spheres_d). "
      "Unlike Min Sphere (points only), each input has a radius. Outputs visualization mesh, Center, Radius.";
  ntype.enum_name_legacy = "CGAL_MIN_SPHERE_OF_SPHERES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_min_sphere_of_spheres_cc
