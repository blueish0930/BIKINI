/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_kdtree.hh"
#include "BLI_task.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "NOD_geometry_nodes_list.hh"
#include "NOD_rna_define.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "MEM_guardedalloc.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_k_nearest_cc {

enum class Mode {
  KNearest = 0,
  Radius = 1,
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Source Geometry"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::PointCloud})
      .description("Geometry whose points are used as query positions");
  b.add_input<decl::Geometry>("Target Geometry"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::PointCloud})
      .description("Geometry whose points are searched for neighbors");
  auto &k = b.add_input<decl::Int>("K"_ustr)
                .default_value(1)
                .min(1)
                .make_available([](bNode &node) { node.custom1 = int8_t(Mode::KNearest); })
                .description("Number of nearest target points returned for every context point");
  auto &radius = b.add_input<decl::Float>("Radius"_ustr)
                     .default_value(1.0f)
                     .min(0.0f)
                     .subtype(PROP_DISTANCE)
                     .make_available([](bNode &node) { node.custom1 = int8_t(Mode::Radius); })
                     .description("Maximum distance from each context point to target points");
  b.add_output<decl::Int>("Indices"_ustr)
      .structure_type(StructureType::List)
      .description("Target point indices grouped by source point and sorted by distance");
  auto &source_indices =
      b.add_output<decl::Int>("Source Indices"_ustr)
          .structure_type(StructureType::List)
          .make_available([](bNode &node) { node.custom1 = int8_t(Mode::Radius); })
          .description("Source point index corresponding to every item in the Indices list");

  /* Mode-dependent availability must live in declare (context-dependent), not updatefunc.
   * Link-drag search calls updatefunc before ensure_topology_cache(); input_by_identifier
   * then returns null and crashes. Match Trim Curve / other official mode nodes. */
  if (const bNode *node = b.node_or_null()) {
    const Mode mode = Mode(node->custom1);
    k.available(mode == Mode::KNearest);
    radius.available(mode == Mode::Radius);
    source_indices.available(mode == Mode::Radius);
  }
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

static Span<float3> positions_from_geometry(const GeometrySet &geometry)
{
  if (const Mesh *mesh = geometry.get_mesh()) {
    return mesh->vert_positions();
  }
  if (const PointCloud *pointcloud = geometry.get_pointcloud()) {
    return pointcloud->positions();
  }
  return {};
}

static KDTree<float3> *build_tree(const Span<float3> positions)
{
  KDTree<float3> *tree = kdtree_new<float3>(positions.size());
  for (const int i : positions.index_range()) {
    kdtree_insert<float3>(tree, i, positions[i]);
  }
  kdtree_balance<float3>(tree);
  return tree;
}

static void execute_k_nearest(const KDTree<float3> &tree,
                              const int target_size,
                              const Span<float3> source_positions,
                              const int k,
                              MutableSpan<int> r_indices)
{
  const int search_count = std::min(k, target_size);
  threading::parallel_for(source_positions.index_range(), 128, [&](const IndexRange range) {
    Vector<KDTreeNearest<float3>, 16> nearest(search_count);
    for (const int source_i : range) {
      const int found = kdtree_find_nearest_n<float3>(
          &tree, source_positions[source_i], nearest.data(), search_count);
      const int64_t offset = int64_t(source_i) * k;
      for (const int neighbor_i : IndexRange(found)) {
        r_indices[offset + neighbor_i] = nearest[neighbor_i].index;
      }
      for (const int neighbor_i : IndexRange(found, k - found)) {
        r_indices[offset + neighbor_i] = -1;
      }
    }
  });
}

static void execute_radius(const KDTree<float3> &tree,
                           const Span<float3> source_positions,
                           const float radius,
                           Vector<int> &r_indices,
                           Vector<int> &r_source_indices)
{
  for (const int source_i : source_positions.index_range()) {
    KDTreeNearest<float3> *nearest = nullptr;
    const int found = kdtree_range_search<float3>(
        &tree, source_positions[source_i], &nearest, radius);
    for (const int neighbor_i : IndexRange(found)) {
      r_indices.append(nearest[neighbor_i].index);
      r_source_indices.append(source_i);
    }
    MEM_SAFE_DELETE(nearest);
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometrySet source_geometry = params.extract_input<GeometrySet>("Source Geometry"_ustr);
  const GeometrySet target_geometry = params.extract_input<GeometrySet>("Target Geometry"_ustr);
  const Span<float3> source_positions = positions_from_geometry(source_geometry);
  const Span<float3> target_positions = positions_from_geometry(target_geometry);
  if (target_positions.is_empty() || source_positions.is_empty()) {
    params.set_output("Indices"_ustr, GList::from_container(Array<int>()));
    if (Mode(params.node().custom1) == Mode::Radius) {
      params.set_output("Source Indices"_ustr, GList::from_container(Array<int>()));
    }
    return;
  }

  KDTree<float3> *tree = build_tree(target_positions);
  const Mode mode = Mode(params.node().custom1);
  if (mode == Mode::KNearest) {
    const int k = params.extract_input<int>("K"_ustr);
    if (k < 1 || int64_t(k) > std::numeric_limits<int64_t>::max() / source_positions.size()) {
      kdtree_free<float3>(tree);
      params.error_message_add(NodeWarningType::Error, "Invalid K or requested list is too large");
      params.set_default_remaining_outputs();
      return;
    }
    Array<int> indices(int64_t(source_positions.size()) * k);
    execute_k_nearest(*tree, target_positions.size(), source_positions, k, indices);
    params.set_output("Indices"_ustr, GList::from_container(std::move(indices)));
  }
  else {
    const float radius = params.extract_input<float>("Radius"_ustr);
    Vector<int> indices;
    Vector<int> source_indices;
    execute_radius(*tree, source_positions, radius, indices, source_indices);
    params.set_output("Indices"_ustr, GList::from_container(std::move(indices)));
    params.set_output("Source Indices"_ustr,
                      GList::from_container(std::move(source_indices)));
  }
  kdtree_free<float3>(tree);
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem mode_items[] = {
      {int(Mode::KNearest),
       "K_NEAREST",
       0,
       "K Nearest",
       "Find a fixed number of nearest target points for every context point"},
      {int(Mode::Radius),
       "RADIUS",
       0,
       "Radius",
       "Find all target points within a radius of every context point"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "Neighbor query mode",
                    mode_items,
                    NOD_inline_enum_accessors(custom1),
                    int(Mode::KNearest));
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeKNearest"_ustr, GEO_NODE_K_NEAREST);
  ntype.ui_name = "Nearest Neighbors";
  ntype.ui_description =
      "Find target points nearest to every source point using either a count or a radius";
  ntype.enum_name_legacy = "K_NEAREST";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_k_nearest_cc
