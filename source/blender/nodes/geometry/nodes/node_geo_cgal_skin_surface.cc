/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * CGAL Skin Surface: smooth union of balls from point set (metaball-like).
 */

#include <fmt/format.h>

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_skin_surface_cc {

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
      .description("Ball centers (point cloud or mesh verts).");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Skin surface mesh (smooth union of balls).");
  b.add_input<decl::Float>("Radius"_ustr)
      .default_value(0.1f)
      .min(1e-6f)
      .subtype(PROP_DISTANCE)
      .structure_type(StructureType::Field)
      .description("Ball radius per point (Point domain). Constant if unconnected.");
  b.add_input<decl::Float>("Shrink"_ustr)
      .default_value(0.5f)
      .min(0.01f)
      .max(0.99f)
      .description(
          "Skin shrink factor (0–1). Smaller = more blob-like union; closer to 1 = tighter to balls.");
  b.add_input<decl::Int>("Subdivisions"_ustr)
      .default_value(0)
      .min(0)
      .max(2)
      .description("Skin surface subdivision passes (smoother, denser mesh).");
  b.add_input<decl::Int>("Max Points"_ustr)
      .default_value(128)
      .min(2)
      .max(2000)
      .description(
          "Maximum input balls. Excess points are subsampled (skin is O(n²+) and freezes on dense meshes).");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  Field<float> radius_field = params.extract_input<Field<float>>("Radius"_ustr);
  const float shrink = params.extract_input<float>("Shrink"_ustr);
  const int subdiv = math::clamp(params.extract_input<int>("Subdivisions"_ustr), 0, 2);
  const int max_points = math::max(2, params.extract_input<int>("Max Points"_ustr));

  const Span<float3> pts_all = positions_from_geometry(geometry);
  if (pts_all.size() < 2) {
    params.error_message_add(NodeWarningType::Info, TIP_("Skin Surface needs at least 2 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  Array<float> radii_all(pts_all.size());
  /* Evaluate radius on point/mesh vertices. */
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    const bke::PointCloudFieldContext ctx(*pc);
    fn::FieldEvaluator ev(ctx, pc->totpoint);
    ev.add_with_destination(radius_field, radii_all.as_mutable_span());
    ev.evaluate();
  }
  else if (const Mesh *mesh = geometry.get_mesh()) {
    const bke::MeshFieldContext ctx(*mesh, bke::AttrDomain::Point);
    fn::FieldEvaluator ev(ctx, mesh->verts_num);
    ev.add_with_destination(radius_field, radii_all.as_mutable_span());
    ev.evaluate();
  }
  else {
    radii_all.fill(0.1f);
  }

  /* Uniform subsample — dense mesh verts as balls is the common freeze cause. */
  Span<float3> pts = pts_all;
  Span<float> radii = radii_all.as_span();
  Array<float3> pts_sub;
  Array<float> radii_sub;
  if (pts_all.size() > max_points) {
    params.error_message_add(
        NodeWarningType::Info,
        fmt::format(fmt::runtime(TIP_(
                        "Skin Surface: too many points ({}), using {} "
                        "(raise Max Points carefully)")),
                    pts_all.size(),
                    max_points));
    pts_sub.reinitialize(max_points);
    radii_sub.reinitialize(max_points);
    /* Stride subsample for spatial spread (better than first N only). */
    const double step = double(pts_all.size()) / double(max_points);
    for (int i = 0; i < max_points; i++) {
      const int src = math::min(int(double(i) * step), int(pts_all.size()) - 1);
      pts_sub[i] = pts_all[src];
      radii_sub[i] = radii_all[src];
    }
    pts = pts_sub.as_span();
    radii = radii_sub.as_span();
  }

  std::string error;
  Mesh *out = geometry::cgal_points_skin_surface(pts, radii, 0.1f, shrink, subdiv, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Skin Surface failed") : error);
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
  geo_node_type_base(&ntype, "GeometryNodeCgalSkinSurface"_ustr, GEO_NODE_CGAL_SKIN_SURFACE);
  ntype.ui_name = "Skin Surface";
  ntype.ui_description =
      "Reconstruct a smooth surface as the skin (shrunk union) of balls centered on the input "
      "points (CGAL Skin_surface_3). Radius field controls ball size.";
  ntype.enum_name_legacy = "CGAL_SKIN_SURFACE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_skin_surface_cc
