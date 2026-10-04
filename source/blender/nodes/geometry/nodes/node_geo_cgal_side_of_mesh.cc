/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_side_of_mesh_cc {

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

static void write_inside_on_query(GeometrySet &geometry, const Span<bool> inside)
{
  if (PointCloud *pc = geometry.get_pointcloud_for_write()) {
    if (pc->totpoint != inside.size()) {
      return;
    }
    bke::SpanAttributeWriter<bool> w = pc->attributes_for_write().lookup_or_add_for_write_only_span<
        bool>("Inside", bke::AttrDomain::Point);
    if (w) {
      w.span.copy_from(inside);
      w.finish();
    }
    return;
  }
  if (Mesh *mesh = geometry.get_mesh_for_write()) {
    if (mesh->verts_num != inside.size()) {
      return;
    }
    bke::SpanAttributeWriter<bool> w =
        mesh->attributes_for_write().lookup_or_add_for_write_only_span<bool>(
            "Inside", bke::AttrDomain::Point);
    if (w) {
      w.span.copy_from(inside);
      w.finish();
    }
  }
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Mesh)
      .description("Closed triangle mesh used as solid.");
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description(
          "Query geometry (point cloud or mesh). Kept as-is; only boolean \"Inside\" is added "
          "on points / vertices.");
  b.add_output<decl::Geometry>("Points"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description(
          "Same query geometry with boolean attribute \"Inside\" (true = bounded side). "
          "Mesh queries stay meshes.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet mesh_geo = params.extract_input<GeometrySet>("Mesh"_ustr);
  GeometrySet query_geo = params.extract_input<GeometrySet>("Points"_ustr);
  const Mesh *mesh = mesh_geo.get_mesh();
  const Span<float3> pts = positions_from_geometry(query_geo);
  if (!mesh || pts.is_empty()) {
    params.set_output("Points"_ustr, std::move(query_geo));
    return;
  }
  Array<bool> inside(pts.size(), false);
  std::string error;
  if (!geometry::cgal_mesh_side_of_query(*mesh, pts, inside, error)) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Side Of Mesh failed") : error);
    params.set_output("Points"_ustr, std::move(query_geo));
    return;
  }
  write_inside_on_query(query_geo, inside);
  params.set_output("Points"_ustr, std::move(query_geo));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeCgalSideOfMesh"_ustr, GEO_NODE_CGAL_SIDE_OF_MESH);
  ntype.ui_name = "Side Of Mesh";
  ntype.ui_description =
      "For each query position, test inside/outside a closed triangle mesh. Writes boolean "
      "attribute \"Inside\" on the query geometry (mesh verts or points). Does not convert a "
      "mesh query to a point cloud.";
  ntype.enum_name_legacy = "CGAL_SIDE_OF_MESH";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)
}  // namespace blender::nodes::node_geo_cgal_side_of_mesh_cc
