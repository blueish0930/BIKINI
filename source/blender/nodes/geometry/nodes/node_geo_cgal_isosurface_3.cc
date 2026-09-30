/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_isosurface_3_cc {

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
      .description("3D samples. A Delaunay tetrahedralization is built from these points.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Marching-tetrahedra isosurface.");
  b.add_input<decl::Float>("Value"_ustr)
      .default_value(0.0f)
      .structure_type(StructureType::Field)
      .description("Scalar at each sample (Point domain).");
  b.add_input<decl::Float>("Isolevel"_ustr)
      .default_value(0.0f)
      .description("Extract the surface Value = Isolevel.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  Field<float> value_field = params.extract_input<Field<float>>("Value"_ustr);
  const float isolevel = params.extract_input<float>("Isolevel"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 4) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Isosurface 3D needs at least 4 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }

  Array<float> values(pts.size());
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    const bke::PointCloudFieldContext ctx(*pc);
    fn::FieldEvaluator ev(ctx, pc->totpoint);
    ev.add_with_destination(value_field, values.as_mutable_span());
    ev.evaluate();
  }
  else if (const Mesh *mesh = geometry.get_mesh()) {
    const bke::MeshFieldContext ctx(*mesh, bke::AttrDomain::Point);
    fn::FieldEvaluator ev(ctx, mesh->verts_num);
    ev.add_with_destination(value_field, values.as_mutable_span());
    ev.evaluate();
  }
  else {
    values.fill(0.0f);
  }

  std::string error;
  Mesh *out = geometry::cgal_points_isosurface_3(pts, values, isolevel, error);
  if (!out || out->faces_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Isosurface 3D failed") : error);
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
  /* Deleted: Isosurface 3D node. */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalIsosurface3"_ustr, GEO_NODE_CGAL_ISOSURFACE_3);
  ntype.ui_name = "Isosurface 3D";
  ntype.ui_description =
      "Marching tetrahedra on the 3D Delaunay triangulation of the samples "
      "(CGAL Delaunay_triangulation_3). Connect Value at each point and set Isolevel. "
      "A regular grid of points with a scalar field works well.";
  ntype.enum_name_legacy = "CGAL_ISOSURFACE_3";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_isosurface_3_cc
