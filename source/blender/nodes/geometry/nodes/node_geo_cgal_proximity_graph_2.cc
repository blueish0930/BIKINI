/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * XY proximity graphs: Gabriel, relative-neighborhood, Euclidean MST.
 * Distinct from KNN/Radius Graph 3D and deleted Gabriel Graph 3D.
 */

#include "BKE_attribute.hh"
#include "BKE_attribute_filters.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.hh"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"
#include "GEO_cgal.hh"
#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_proximity_graph_2_cc {

enum class Mode {
  Gabriel = 0,
  Rng = 1,
  Mst = 2,
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

static void copy_input_point_attributes_to_mesh(const GeometrySet &geometry, Mesh &mesh)
{
  const Span<StringRef> skip{"position"};
  if (const PointCloud *pc = geometry.get_pointcloud()) {
    if (pc->totpoint != mesh.verts_num) {
      return;
    }
    bke::copy_attributes(pc->attributes(),
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         bke::attribute_filter_from_skip_ref(skip),
                         mesh.attributes_for_write());
    return;
  }
  if (const Mesh *src_mesh = geometry.get_mesh()) {
    if (src_mesh->verts_num != mesh.verts_num) {
      return;
    }
    bke::copy_attributes(src_mesh->attributes(),
                         bke::AttrDomain::Point,
                         bke::AttrDomain::Point,
                         bke::attribute_filter_from_skip_ref(skip),
                         mesh.attributes_for_write());
  }
}

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  static const EnumPropertyItem mode_items[] = {
      {int(Mode::Gabriel),
       "GABRIEL",
       0,
       N_("Gabriel"),
       N_("Keep Delaunay edges whose diametral disk is empty")},
      {int(Mode::Rng),
       "RNG",
       0,
       N_("Relative Neighbor"),
       N_("Keep edges with no third point closer to both endpoints")},
      {int(Mode::Mst),
       "MST",
       0,
       N_("Euclidean MST"),
       N_("Minimum spanning tree of the XY Delaunay graph")},
      {0, nullptr, 0, nullptr, nullptr},
  };
  b.add_input<decl::Menu>("Mode"_ustr)
      .static_items(mode_items)
      .default_value(MenuValue(Mode::Gabriel))
      .optional_label();
  b.add_input<decl::Geometry>("Points"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::PointCloud, GeometryComponent::Type::Mesh})
      .description("XY sites. Output vertices match this point order.");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Wire mesh. Vertices are 1:1 with input points.");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Points"_ustr);
  const Mode mode = params.extract_input<Mode>("Mode"_ustr);
  const Span<float3> pts = positions_from_geometry(geometry);
  if (pts.size() < 2) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Proximity Graph 2D needs at least 2 points"));
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  std::string error;
  Mesh *out = geometry::cgal_points_proximity_graph_2(pts, int(mode), error);
  if (!out || out->edges_num == 0) {
    params.error_message_add(NodeWarningType::Warning,
                             error.empty() ? TIP_("Proximity Graph 2D failed") : error);
    if (out) {
      BKE_id_free(nullptr, out);
    }
    params.set_output("Mesh"_ustr, GeometrySet());
    return;
  }
  copy_input_point_attributes_to_mesh(geometry, *out);
  params.set_output("Mesh"_ustr, GeometrySet::from_mesh(out));
}

static void node_register()
{
  /* Deleted: Proximity Graph 2D (legacy 2500 / 2591). */
  return;
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalProximityGraph2"_ustr, GEO_NODE_CGAL_PROXIMITY_GRAPH_2);
  ntype.ui_name = "Proximity Graph 2D";
  ntype.ui_description =
      "Gabriel, relative-neighborhood, or Euclidean MST of XY points "
      "(subgraph of the Delaunay triangulation). Distinct from KNN Graph 3D.";
  ntype.enum_name_legacy = "CGAL_PROXIMITY_GRAPH_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_proximity_graph_2_cc
