/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Unified point-set shape fitting (region growing).
 * Main output: original points with Region ids.
 * Secondary panel output: fitted preview mesh.
 */

#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "NOD_socket_usage_inference.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_point_shape_fitting_cc {

enum class Shape {
  Plane = 0,
  Sphere = 1,
  Cylinder = 2,
  Circle = 3,
  Line = 4,
};

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
  bke::SpanAttributeWriter<int> w =
      pc->attributes_for_write().lookup_or_add_for_write_only_span<int>("Region",
                                                                       bke::AttrDomain::Point);
  w.span.copy_from(region);
  w.finish();
  return pc;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  static const EnumPropertyItem shape_items[] = {
      {int(Shape::Plane), "PLANE", 0, N_("Plane"), N_("Least-squares planes")},
      {int(Shape::Sphere), "SPHERE", 0, N_("Sphere"), N_("Least-squares spheres")},
      {int(Shape::Cylinder), "CYLINDER", 0, N_("Cylinder"), N_("Least-squares cylinders")},
      {int(Shape::Circle), "CIRCLE", 0, N_("Circle"), N_("Circles in the PCA plane")},
      {int(Shape::Line), "LINE", 0, N_("Line"), N_("3D PCA lines")},
      {0, nullptr, 0, nullptr, nullptr},
  };

  b.add_input<decl::Menu>("Shape"_ustr)
      .static_items(shape_items)
      .default_value(MenuValue(Shape::Plane))
      .optional_label()
      .description("Primitive type fitted to each region.");
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Point cloud (optional float3 \"normal\"; else jet-estimated).");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Input points with Point int \"Region\" (-1 unassigned).");
  b.add_output<decl::Int>("Region Count"_ustr).description("Number of fitted regions.");

  b.add_input<decl::Float>("Neighbor Radius"_ustr)
      .default_value(0.1f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Sphere neighborhood radius for growing.");
  b.add_input<decl::Float>("Max Distance"_ustr)
      .default_value(0.02f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Max point-to-primitive distance.");
  b.add_input<decl::Float>("Max Angle"_ustr)
      .default_value(25.0f)
      .min(0.0f)
      .max(90.0f)
      .usage_by_menu("Shape"_ustr,
                     {int(Shape::Plane),
                      int(Shape::Sphere),
                      int(Shape::Cylinder),
                      int(Shape::Circle)})
      .description("Max degrees between point normal and the primitive.");
  b.add_input<decl::Int>("Min Region Size"_ustr)
      .default_value(10)
      .min(1)
      .description("Minimum number of points in a fitted region.");
  b.add_input<decl::Float>("Min Radius"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .usage_by_menu("Shape"_ustr,
                     {int(Shape::Sphere), int(Shape::Cylinder), int(Shape::Circle)})
      .description("Reject fitted primitives smaller than this radius. 0 = no minimum.");
  b.add_input<decl::Float>("Max Radius"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .usage_by_menu("Shape"_ustr,
                     {int(Shape::Sphere), int(Shape::Cylinder), int(Shape::Circle)})
      .description("0 = unlimited.");

  auto &panel = b.add_panel("Fitted Shape"_ustr)
                    .description("Preview mesh of the fitted primitives.");
  panel.add_output<decl::Geometry>("Shape"_ustr)
      .propagate_all_geometry()
      .description("Fitted planes / spheres / cylinders / disks / line segments.");
  panel.add_input<decl::Int>("Resolution"_ustr)
      .default_value(24)
      .min(8)
      .max(64)
      .usage_by_menu("Shape"_ustr,
                     {int(Shape::Sphere), int(Shape::Cylinder), int(Shape::Circle)})
      .description("UV / cylinder tessellation of the preview mesh.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const Shape shape = params.extract_input<Shape>("Shape"_ustr);
  const float radius = params.extract_input<float>("Neighbor Radius"_ustr);
  const float max_d = params.extract_input<float>("Max Distance"_ustr);
  const bool needs_angle = shape != Shape::Line;
  const bool needs_radius_lim = ELEM(
      shape, Shape::Sphere, Shape::Cylinder, Shape::Circle);
  const bool needs_res = needs_radius_lim;
  const float max_a = needs_angle ? params.extract_input<float>("Max Angle"_ustr) : 25.0f;
  const int min_s = params.extract_input<int>("Min Region Size"_ustr);
  const float min_r = needs_radius_lim ? params.extract_input<float>("Min Radius"_ustr) : 0.0f;
  const float max_r = needs_radius_lim ? params.extract_input<float>("Max Radius"_ustr) : 0.0f;
  const int segments = needs_res ? params.extract_input<int>("Resolution"_ustr) : 24;

  const Span<float3> pts = positions_from_geometry(geometry);
  auto empty_out = [&]() {
    params.set_output("Points"_ustr, GeometrySet());
    params.set_output("Region Count"_ustr, 0);
    if (params.output_is_required("Shape"_ustr)) {
      params.set_output("Shape"_ustr, GeometrySet());
    }
  };
  if (pts.size() < 2) {
    empty_out();
    return;
  }
  Array<float3> nbuf;
  Span<float3> normals;
  read_normals(geometry, pts, nbuf, normals);
  Array<int> region(pts.size(), -1);
  int count = 0;
  std::string error;
  Mesh *shape_mesh = geometry::cgal_points_shape_fitting(pts,
                                                         normals,
                                                         int(shape),
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
  params.set_output("Points"_ustr,
                    GeometrySet::from_pointcloud(tagged_points(geometry, pts, region)));
  params.set_output("Region Count"_ustr, count);
  if (params.output_is_required("Shape"_ustr)) {
    const bool has_shape = shape_mesh &&
                           (shape_mesh->faces_num > 0 || shape_mesh->edges_num > 0);
    if (!has_shape) {
      if (!error.empty()) {
        params.error_message_add(NodeWarningType::Warning, error);
      }
      if (shape_mesh) {
        BKE_id_free(nullptr, shape_mesh);
      }
      params.set_output("Shape"_ustr, GeometrySet());
    }
    else {
      params.set_output("Shape"_ustr, GeometrySet::from_mesh(shape_mesh));
    }
  }
  else if (shape_mesh) {
    BKE_id_free(nullptr, shape_mesh);
  }
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalPointShapeFitting"_ustr, GEO_NODE_CGAL_POINT_SHAPE_FITTING);
  ntype.ui_name = "Point Shape Fitting";
  ntype.ui_description =
      "Grow regions on a point cloud and fit a primitive per region "
      "(CGAL Region_growing: plane / sphere / cylinder / circle / line). "
      "Main output is the original points with Point int \"Region\". "
      "The Fitted Shape panel holds the preview mesh and its resolution.";
  ntype.enum_name_legacy = "CGAL_POINT_SHAPE_FITTING";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_point_shape_fitting_cc
