/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_sphere_region_growing_cc {

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

static bool read_normals(const GeometrySet &geometry,
                         const Span<float3> pts,
                         Array<float3> &buf,
                         Span<float3> &out)
{
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    if (const VArray n = *pc->attributes().lookup<float3>("normal")) {
      if (n.size() == pts.size()) {
        buf.reinitialize(pts.size());
        n.materialize(buf.as_mutable_span());
        out = buf.as_span();
        return true;
      }
    }
  }
  return false;
}

static PointCloud *tagged_points(const GeometrySet &geometry,
                                 const Span<float3> pts,
                                 const Span<int> region)
{
  PointCloud *pc = nullptr;
  if (const PointCloud *src = geometry.get_pointcloud()) {
    pc = BKE_pointcloud_copy_for_eval(src);
  }
  else {
    pc = BKE_pointcloud_new_nomain(PointCloudType::Points, int(pts.size()));
    pc->positions_for_write().copy_from(pts);
  }
  bke::SpanAttributeWriter<int> w = pc->attributes_for_write().lookup_or_add_for_write_only_span<int>(
      "Region", bke::AttrDomain::Point);
  w.span.copy_from(region);
  w.finish();
  return pc;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Point cloud (optional float3 \"normal\"; else jet-estimated).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Fitted UV spheres. Face int \"region_id\".");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .description("Same points with Point int \"Region\" (-1 unassigned).");
  b.add_output<decl::Int>("Region Count"_ustr).description("Number of spherical regions.");
  b.add_input<decl::Float>("Neighbor Radius"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE);
  b.add_input<decl::Float>("Max Distance"_ustr)
      .default_value(0.02f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Max point-to-sphere distance.");
  b.add_input<decl::Float>("Max Angle"_ustr).default_value(25.0f).min(0.0f).max(90.0f);
  b.add_input<decl::Int>("Min Region Size"_ustr).default_value(10).min(3);
  b.add_input<decl::Float>("Min Radius"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE);
  b.add_input<decl::Float>("Max Radius"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("0 = unlimited.");
  b.add_input<decl::Int>("Segments"_ustr).default_value(24).min(8).max(64);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float radius = params.extract_input<float>("Neighbor Radius"_ustr);
  const float max_d = params.extract_input<float>("Max Distance"_ustr);
  const float max_a = params.extract_input<float>("Max Angle"_ustr);
  const int min_s = params.extract_input<int>("Min Region Size"_ustr);
  const float min_r = params.extract_input<float>("Min Radius"_ustr);
  const float max_r = params.extract_input<float>("Max Radius"_ustr);
  const int segments = params.extract_input<int>("Segments"_ustr);

  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 3) {
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Points"_ustr, GeometrySet());
    params.set_output("Region Count"_ustr, 0);
    return;
  }
  Array<float3> nbuf;
  Span<float3> normals;
  read_normals(geometry, pts, nbuf, normals);
  Array<int> region(pts.size(), -1);
  int count = 0;
  std::string error;
  Mesh *out = geometry::cgal_points_sphere_region_growing(pts,
                                                          normals,
                                                          radius,
                                                          max_d,
                                                          max_a,
                                                          min_s,
                                                          min_r,
                                                          max_r,
                                                          segments,
                                                          region.as_mutable_span(),
                                                          count,
                                                          error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Sphere Region Growing failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Points"_ustr, GeometrySet::from_pointcloud(tagged_points(geometry, pts, region)));
    params.set_output("Region Count"_ustr, count);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(tagged_points(geometry, pts, region)));
  params.set_output("Region Count"_ustr, count);
}

static void node_register()
{
  /* Merged into Point Shape Fitting (GeometryNodeCgalPointShapeFitting). */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalSphereRegionGrowing"_ustr, GEO_NODE_CGAL_SPHERE_REGION_GROWING);
  ntype.ui_name = "Sphere Region Growing";
  ntype.ui_description =
      "Grow spherical regions on a point cloud (CGAL Least_squares_sphere_fit_region). "
      "Outputs fitted UV spheres and Point int \"Region\".";
  ntype.enum_name_legacy = "CGAL_SPHERE_REGION_GROWING";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_sphere_region_growing_cc
