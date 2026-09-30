/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Efficient RANSAC shapes extracted as preview meshes (not point labels).
 */

#include "BKE_attribute.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_ransac_primitives_cc {

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
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Detected primitives as meshes. Face attribute shape_id is the primitive index.");
  b.add_output<decl::Int>("Shape Count"_ustr).description("Number of extracted primitives.");
  b.add_input<decl::Float>("Epsilon"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Inlier distance. 0 = auto.");
  b.add_input<decl::Float>("Cluster Epsilon"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .subtype(PROP_DISTANCE)
      .description("Inlier connectivity radius. 0 = auto.");
  b.add_input<decl::Float>("Normal Threshold"_ustr)
      .default_value(0.9f)
      .min(0.0f)
      .max(1.0f)
      .description("cos(angle) between point normal and the shape.");
  b.add_input<decl::Int>("Min Points"_ustr)
      .default_value(0)
      .min(0)
      .description("Minimum inliers per shape. 0 = auto.");
  b.add_input<decl::Float>("Probability"_ustr)
      .default_value(0.01f)
      .min(0.001f)
      .max(0.5f)
      .description("Smaller = more trials, slower, more reliable.");
  b.add_input<decl::Int>("Seed"_ustr).default_value(0).min(0).description("RANSAC seed.");
  b.add_input<decl::Int>("Segments"_ustr)
      .default_value(24)
      .min(8)
      .max(64)
      .description("Tessellation for spheres / cylinders / cones / tori.");
  b.add_input<decl::Bool>("Planes"_ustr).default_value(true).description("Extract planar n-gons.");
  b.add_input<decl::Bool>("Spheres"_ustr).default_value(false).description("Extract spheres.");
  b.add_input<decl::Bool>("Cylinders"_ustr).default_value(false).description("Extract cylinders.");
  b.add_input<decl::Bool>("Cones"_ustr).default_value(false).description("Extract cones.");
  b.add_input<decl::Bool>("Tori"_ustr).default_value(false).description("Extract tori.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const float epsilon = params.extract_input<float>("Epsilon"_ustr);
  const float cluster_epsilon = params.extract_input<float>("Cluster Epsilon"_ustr);
  const float normal_threshold = params.extract_input<float>("Normal Threshold"_ustr);
  const int min_points = params.extract_input<int>("Min Points"_ustr);
  const float probability = params.extract_input<float>("Probability"_ustr);
  const int seed = params.extract_input<int>("Seed"_ustr);
  const int segments = params.extract_input<int>("Segments"_ustr);
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
    params.error_message_add(NodeWarningType::Info,
                             TIP_("RANSAC Primitives needs at least 10 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Shape Count"_ustr, 0);
    return;
  }
  Array<float3> nbuf;
  Span<float3> normals;
  read_normals(geometry, pts, nbuf, normals);

  std::string error;
  int shape_count = 0;
  Mesh *out = geometry::cgal_points_ransac_primitives(pts,
                                                      normals,
                                                      epsilon,
                                                      cluster_epsilon,
                                                      normal_threshold,
                                                      min_points,
                                                      probability,
                                                      flags,
                                                      seed,
                                                      segments,
                                                      shape_count,
                                                      error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("RANSAC Primitives failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    params.set_output("Shape Count"_ustr, 0);
    return;
  }
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
  params.set_output("Shape Count"_ustr, shape_count);
}

static void node_register()
{
  /* Deleted: RANSAC Primitives node. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalRansacPrimitives"_ustr, GEO_NODE_CGAL_RANSAC_PRIMITIVES);
  ntype.ui_name = "RANSAC Primitives";
  ntype.ui_description =
      "Run Efficient RANSAC and emit each detected primitive as a mesh "
      "(plane = inlier convex n-gon, plus UV sphere / cylinder / cone / torus). "
      "Face attribute shape_id is the primitive index. "
      "Different from Efficient RANSAC, which only tags the input points.";
  ntype.enum_name_legacy = "CGAL_RANSAC_PRIMITIVES";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_ransac_primitives_cc
