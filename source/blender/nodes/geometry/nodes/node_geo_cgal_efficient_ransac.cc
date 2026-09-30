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

namespace blender::nodes::node_geo_cgal_efficient_ransac_cc {

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

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("Oriented points (optional Point float3 \"normal\"; else jet-estimated).");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Same points with int \"Shape\" and \"Shape Type\".");
  b.add_output<decl::Int>("Shape Count"_ustr).description("Number of detected shapes.");
  b.add_input<decl::Float>("Epsilon"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description(
          "Distance tolerance: how far a point may be from a shape and still count as an inlier. "
          "Use scene units (e.g. 0.01 on a 1m object). 0 = auto (~1% of point-cloud diagonal). "
          "Too small = few matches; too large = wrong shapes merge.");
  b.add_input<decl::Float>("Cluster Epsilon"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description(
          "Connectivity radius among inliers: points farther than this are not connected into "
          "the same shape region. 0 = auto (same order as Epsilon). Raise if one plane splits "
          "into many pieces; lower if separate planes stick together.");
  b.add_input<decl::Float>("Normal Threshold"_ustr)
      .default_value(0.9f)
      .min(0.0f)
      .max(1.0f)
      .description(
          "Normal alignment as cos(angle): point normal vs shape normal. "
          "0.9 ≈ within ~25°, 0.98 ≈ ~11° (stricter). Lower accepts noisier/curved areas.");
  b.add_input<decl::Int>("Min Points"_ustr)
      .default_value(0)
      .min(0)
      .description(
          "Minimum inliers required to accept a shape. 0 = auto (~1% of all points). "
          "Raise to drop small junk shapes; lower to keep small features.");
  b.add_input<decl::Float>("Probability"_ustr)
      .default_value(0.01f)
      .min(0.001f)
      .max(0.5f)
      .description(
          "Search endurance (not a success rate). Smaller = more random trials, slower but "
          "more reliable (e.g. 0.01). Larger (e.g. 0.1) = faster, may miss shapes.");
  b.add_input<decl::Int>("Seed"_ustr)
      .default_value(0)
      .min(0)
      .description(
          "Random seed. Same Seed + same points/params => same Shape IDs (stable viewport). "
          "Change Seed to try another RANSAC solution.");
  b.add_input<decl::Bool>("Planes"_ustr)
      .default_value(true)
      .description("Detect planar regions (walls, floors, flat panels).");
  b.add_input<decl::Bool>("Spheres"_ustr)
      .default_value(false)
      .description("Detect spherical patches (balls, rounded joints). Needs good normals.");
  b.add_input<decl::Bool>("Cylinders"_ustr)
      .default_value(false)
      .description("Detect cylinders (pipes, columns). Needs good normals.");
  b.add_input<decl::Bool>("Cones"_ustr)
      .default_value(false)
      .description("Detect cones. More fragile; use larger Min Points if noisy.");
  b.add_input<decl::Bool>("Tori"_ustr)
      .default_value(false)
      .description("Detect tori (donut shapes). Expensive; enable only if needed.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float epsilon = params.extract_input<float>("Epsilon"_ustr);
  const float cluster_eps = params.extract_input<float>("Cluster Epsilon"_ustr);
  const float normal_th = params.extract_input<float>("Normal Threshold"_ustr);
  const int min_points = params.extract_input<int>("Min Points"_ustr);
  const float probability = params.extract_input<float>("Probability"_ustr);
  const int seed = params.extract_input<int>("Seed"_ustr);
  int flags = 0;
  if (params.extract_input<bool>("Planes"_ustr)) {
    flags |= 1;
  }
  if (params.extract_input<bool>("Spheres"_ustr)) {
    flags |= 2;
  }
  if (params.extract_input<bool>("Cylinders"_ustr)) {
    flags |= 4;
  }
  if (params.extract_input<bool>("Cones"_ustr)) {
    flags |= 8;
  }
  if (params.extract_input<bool>("Tori"_ustr)) {
    flags |= 16;
  }

  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 10) {
    params.set_output("Points"_ustr, GeometrySet());
    params.set_output("Shape Count"_ustr, 0);
    return;
  }
  Array<float3> nbuf;
  Span<float3> normals;
  read_normals(geometry, pts, nbuf, normals);
  std::string error;
  int shape_count = 0;
  PointCloud *pc = geometry::cgal_points_efficient_ransac(pts,
                                                          normals,
                                                          epsilon,
                                                          cluster_eps,
                                                          normal_th,
                                                          min_points,
                                                          probability,
                                                          flags,
                                                          seed,
                                                          geometry.get_pointcloud(),
                                                          geometry.get_mesh(),
                                                          shape_count,
                                                          error);
  if (!pc) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Efficient RANSAC failed") : error);
    params.set_output("Points"_ustr, GeometrySet());
    params.set_output("Shape Count"_ustr, 0);
    return;
  }
  params.set_output("Points"_ustr, GeometrySet::from_pointcloud(pc));
  params.set_output("Shape Count"_ustr, shape_count);
}

static void node_register()
{
  /* Deleted: Efficient RANSAC node. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalEfficientRansac"_ustr, GEO_NODE_CGAL_EFFICIENT_RANSAC);
  ntype.ui_name = "Efficient RANSAC";
  ntype.ui_description =
      "Fit primitive shapes to a point cloud (CGAL Efficient RANSAC). "
      "Output: Point int \"Shape\" (region id, -1 = leftover) and \"Shape Type\" "
      "(0=Plane,1=Sphere,2=Cylinder,3=Cone,4=Torus). Color with Named Attribute. "
      "Tip: provide float3 \"normal\" first; start with Planes only; set Epsilon in scene units; "
      "use Seed for stable viewport (RANSAC is random).";
  ntype.enum_name_legacy = "CGAL_EFFICIENT_RANSAC";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_efficient_ransac_cc
