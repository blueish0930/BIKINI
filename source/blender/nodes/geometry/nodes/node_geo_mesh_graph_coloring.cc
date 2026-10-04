/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Mesh Graph Coloring — Wikipedia graph-coloring heuristics on the mesh 1-ring.
 *
 * Algorithm is a Menu socket on the node body. Seed only appears for Greedy Random.
 */

#include "BLI_math_base.hh"
#include "BLI_string_ref.hh"

#include "GEO_foreach_geometry.hh"
#include "GEO_mesh_graph_coloring.hh"

#include "BKE_mesh.hh"

#include "DNA_mesh_types.h"

#include "node_geometry_util.hh"

#include <atomic>

namespace blender::nodes::node_geo_mesh_graph_coloring_cc {

enum class AlgorithmMode {
  Greedy = 0,
  GreedyRandom = 1,
  WelshPowell = 2,
  DSatur = 3,
  RLF = 4,
};

static EnumPropertyItem algorithm_items[] = {
    {int(AlgorithmMode::Greedy),
     "GREEDY",
     0,
     N_("Greedy"),
     N_("Sequential greedy coloring in vertex index order")},
    {int(AlgorithmMode::GreedyRandom),
     "GREEDY_RANDOM",
     0,
     N_("Greedy Random"),
     N_("Greedy after a random vertex permutation (uses Seed)")},
    {int(AlgorithmMode::WelshPowell),
     "WELSH_POWELL",
     0,
     N_("Welsh–Powell"),
     N_("Greedy after sorting vertices by nonincreasing degree")},
    {int(AlgorithmMode::DSatur),
     "DSATUR",
     0,
     N_("DSatur"),
     N_("Brélaz DSatur: dynamic order by saturation degree (recommended)")},
    {int(AlgorithmMode::RLF),
     "RLF",
     0,
     N_("RLF"),
     N_("Recursive Largest First: build color classes as maximal independent sets")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Geometry>("Mesh"_ustr)
      .supported_type(GeometryComponent::Type::Mesh)
      .is_default_link_socket()
      .description("Mesh whose edge-adjacency (1-ring) graph will be colored");
  b.add_output<decl::Geometry>("Mesh"_ustr)
      .propagate_all_geometry()
      .align_with_previous()
      .description("Mesh with a point integer attribute of color IDs");

  b.add_input<decl::String>("Name"_ustr)
      .default_value("color")
      .is_attribute_name()
      .description("Point-domain integer attribute; color IDs start at 0");

  b.add_input<decl::Menu>("Algorithm"_ustr)
      .static_items(algorithm_items)
      .default_value(MenuValue(int(AlgorithmMode::DSatur)))
      .optional_label()
      .description("Graph coloring heuristic");

  b.add_input<decl::Int>("Seed"_ustr)
      .default_value(0)
      .min(0)
      .description("Random seed for Greedy Random vertex ordering")
      .usage_by_single_menu(int(AlgorithmMode::GreedyRandom));

  b.add_output<decl::Int>("Color Count"_ustr)
      .description("Number of distinct colors produced by the heuristic");
}

static geometry::GraphColoringAlgorithm to_algorithm(const AlgorithmMode mode)
{
  switch (mode) {
    case AlgorithmMode::Greedy:
      return geometry::GraphColoringAlgorithm::Greedy;
    case AlgorithmMode::GreedyRandom:
      return geometry::GraphColoringAlgorithm::GreedyRandom;
    case AlgorithmMode::WelshPowell:
      return geometry::GraphColoringAlgorithm::WelshPowell;
    case AlgorithmMode::DSatur:
      return geometry::GraphColoringAlgorithm::DSatur;
    case AlgorithmMode::RLF:
      return geometry::GraphColoringAlgorithm::RLF;
  }
  return geometry::GraphColoringAlgorithm::DSatur;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh"_ustr);
  const std::string name = params.extract_input<std::string>("Name"_ustr);
  const AlgorithmMode mode = params.extract_input<AlgorithmMode>("Algorithm"_ustr);

  int seed = 0;
  if (mode == AlgorithmMode::GreedyRandom) {
    seed = params.extract_input<int>("Seed"_ustr);
  }

  if (name.empty()) {
    params.error_message_add(NodeWarningType::Error, TIP_("Attribute name must not be empty"));
    params.set_default_remaining_outputs();
    return;
  }

  geometry::MeshGraphColoringOptions options;
  options.algorithm = to_algorithm(mode);
  options.seed = seed;

  std::atomic<int> color_count = 0;
  std::atomic<bool> found_mesh = false;

  geometry::foreach_real_geometry(geometry_set, [&](GeometrySet &geometry) {
    Mesh *mesh = geometry.get_mesh_for_write();
    if (!mesh) {
      return;
    }
    found_mesh.store(true, std::memory_order_relaxed);
    const int n = geometry::mesh_graph_coloring_store_attribute(*mesh, name, options);
    color_count.store(math::max(color_count.load(std::memory_order_relaxed), n),
                      std::memory_order_relaxed);
  });

  if (!found_mesh.load(std::memory_order_relaxed)) {
    params.error_message_add(NodeWarningType::Warning,
                             TIP_("Input geometry does not contain a mesh"));
  }

  params.set_output("Mesh"_ustr, std::move(geometry_set));
  params.set_output("Color Count"_ustr, color_count.load(std::memory_order_relaxed));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeMeshGraphColoring"_ustr, GEO_NODE_MESH_GRAPH_COLORING);
  ntype.ui_name = "Mesh Graph Coloring";
  ntype.ui_description =
      "Color mesh vertices so adjacent vertices get different integer IDs. Implements "
      "Greedy, Welsh–Powell, DSatur, and RLF heuristics from graph coloring literature";
  ntype.enum_name_legacy = "MESH_GRAPH_COLORING";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_mesh_graph_coloring_cc
