/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup geo
 *
 * Mesh 1-ring vertex graph coloring.
 * Algorithms follow Wikipedia "Graph coloring" / "Greedy coloring" / "DSatur" /
 * "Recursive largest first algorithm" (Welsh–Powell, Brélaz DSatur, RLF, …).
 */

#include "BLI_span.hh"
#include "BLI_string_ref.hh"

namespace blender {

struct Mesh;

namespace geometry {

/** Standard vertex-coloring heuristics (see Wikipedia: Graph coloring). */
enum class GraphColoringAlgorithm {
  /**
   * Sequential greedy in fixed vertex index order (v0, v1, …).
   * Always proper; color count depends on ordering.
   */
  Greedy = 0,
  /**
   * Greedy after a random permutation of vertices (seeded).
   * Same algorithm as Greedy with a different static order.
   */
  GreedyRandom = 1,
  /**
   * Welsh–Powell: greedy after sorting vertices by nonincreasing degree.
   * Uses at most max_i min{d(x_i)+1, i} colors.
   */
  WelshPowell = 2,
  /**
   * DSatur (Brélaz 1979): dynamic order — next vertex has highest saturation
   * (distinct neighbor colors), ties broken by degree.
   */
  DSatur = 3,
  /**
   * Recursive Largest First (Leighton / RLF): build one maximal independent set
   * (color class) at a time with the standard RLF selection heuristic.
   */
  RLF = 4,
};

struct MeshGraphColoringOptions {
  GraphColoringAlgorithm algorithm = GraphColoringAlgorithm::DSatur;
  /** Seed for GreedyRandom (and RLF internal tie-breaking if used). */
  int seed = 0;
};

/**
 * Color the mesh 1-ring graph.
 * \param r_colors  Per-vertex color IDs in [0, color_count).
 * \return Number of colors used, or 0 if invalid.
 */
int mesh_graph_coloring(const Mesh &mesh,
                        MutableSpan<int> r_colors,
                        const MeshGraphColoringOptions &options);

/**
 * Store colors as a point-domain integer attribute.
 * \return Number of colors used.
 */
int mesh_graph_coloring_store_attribute(Mesh &mesh,
                                        StringRef attribute_name,
                                        const MeshGraphColoringOptions &options);

}  // namespace geometry
}  // namespace blender
