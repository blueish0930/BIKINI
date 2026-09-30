/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Vertex graph coloring on the mesh 1-ring.
 *
 * References (Wikipedia Graph coloring / Greedy coloring / DSatur / RLF):
 *  - Greedy sequential coloring
 *  - Welsh–Powell (degree-ordered greedy)
 *  - DSatur (Brélaz 1979)
 *  - Recursive Largest First (RLF)
 */

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_math_base.hh"
#include "BLI_rand.hh"
#include "BLI_span.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "DNA_mesh_types.h"

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"

#include "GEO_mesh_graph_coloring.hh"

#include <algorithm>
#include <limits>
#include <utility>

namespace blender::geometry {
namespace {

static void build_adjacency(const Mesh &mesh, Array<Vector<int>> &adj, Array<int> &degree)
{
  const int n = mesh.verts_num;
  adj.reinitialize(n);
  degree.reinitialize(n);
  degree.fill(0);

  for (const int2 e : mesh.edges()) {
    const int a = e[0];
    const int b = e[1];
    if (a == b || a < 0 || b < 0 || a >= n || b >= n) {
      continue;
    }
    adj[a].append(b);
    adj[b].append(a);
  }

  for (const int v : IndexRange(n)) {
    Vector<int> &nbrs = adj[v];
    if (nbrs.size() > 1) {
      std::sort(nbrs.begin(), nbrs.end());
      nbrs.resize(std::unique(nbrs.begin(), nbrs.end()) - nbrs.begin());
    }
    degree[v] = int(nbrs.size());
  }
}

/** Smallest non-negative color not used by already-colored neighbors. */
static int smallest_free_color(const Span<int> colors,
                               const Span<int> neighbors,
                               Vector<uint8_t> &used_scratch)
{
  int max_needed = 0;
  for (const int nb : neighbors) {
    if (colors[nb] >= 0) {
      max_needed = math::max(max_needed, colors[nb] + 1);
    }
  }
  used_scratch.resize(max_needed + 1);
  used_scratch.fill(0);
  for (const int nb : neighbors) {
    if (colors[nb] >= 0) {
      used_scratch[colors[nb]] = 1;
    }
  }
  for (int c = 0; c < max_needed; c++) {
    if (!used_scratch[c]) {
      return c;
    }
  }
  return max_needed;
}

static int count_colors(const Span<int> colors)
{
  int max_c = -1;
  for (const int c : colors) {
    max_c = math::max(max_c, c);
  }
  return max_c + 1;
}

/** Greedy coloring for a given vertex order (permutation of 0..n-1). */
static int greedy_in_order(const Array<Vector<int>> &adj,
                           const Span<int> order,
                           MutableSpan<int> colors)
{
  const int n = int(colors.size());
  colors.fill(-1);
  Vector<uint8_t> used;
  for (const int i : IndexRange(n)) {
    const int v = order[i];
    colors[v] = smallest_free_color(colors, adj[v], used);
  }
  return count_colors(colors);
}

static int algorithm_greedy_index(const Array<Vector<int>> &adj, MutableSpan<int> colors)
{
  const int n = int(colors.size());
  Array<int> order(n);
  for (const int i : IndexRange(n)) {
    order[i] = i;
  }
  return greedy_in_order(adj, order, colors);
}

static int algorithm_greedy_random(const Array<Vector<int>> &adj,
                                   MutableSpan<int> colors,
                                   const int seed)
{
  const int n = int(colors.size());
  Array<int> order(n);
  for (const int i : IndexRange(n)) {
    order[i] = i;
  }
  /* Deterministic LCG shuffle (avoid RandomNumberGenerator API mismatches). */
  uint32_t state = uint32_t(seed) | 1u;
  for (int i = n - 1; i > 0; --i) {
    state = state * 1664525u + 1013904223u;
    const int j = int(state % uint32_t(i + 1));
    std::swap(order[i], order[j]);
  }
  return greedy_in_order(adj, order, colors);
}

/** Welsh–Powell: sort by nonincreasing degree, then greedy. */
static int algorithm_welsh_powell(const Array<Vector<int>> &adj,
                                  const Span<int> degree,
                                  MutableSpan<int> colors)
{
  const int n = int(colors.size());
  Array<int> order(n);
  for (const int i : IndexRange(n)) {
    order[i] = i;
  }
  std::stable_sort(order.begin(), order.end(), [&](const int a, const int b) {
    if (degree[a] != degree[b]) {
      return degree[a] > degree[b];
    }
    return a < b;
  });
  return greedy_in_order(adj, order, colors);
}

/**
 * DSatur (Brélaz): repeatedly color the uncolored vertex with highest saturation
 * (#distinct colors among neighbors); break ties by higher degree, then lower index.
 */
static int algorithm_dsatur(const Array<Vector<int>> &adj,
                            const Span<int> degree,
                            MutableSpan<int> colors)
{
  const int n = int(colors.size());
  colors.fill(-1);
  Array<int> saturation(n, 0);
  Array<bool> colored(n, false);
  Vector<uint8_t> used;
  /* neighbor_color_mask[v] = set of colors used by neighbors of v (bitsets via small arrays). */
  Array<Vector<int>> neigh_colors(n);

  auto recompute_saturation = [&](const int v) {
    if (colored[v]) {
      return;
    }
    Vector<int> &nc = neigh_colors[v];
    nc.clear();
    for (const int nb : adj[v]) {
      if (colors[nb] >= 0) {
        nc.append(colors[nb]);
      }
    }
    if (!nc.is_empty()) {
      std::sort(nc.begin(), nc.end());
      nc.resize(std::unique(nc.begin(), nc.end()) - nc.begin());
    }
    saturation[v] = int(nc.size());
  };

  int remaining = n;
  while (remaining > 0) {
    int best = -1;
    int best_sat = -1;
    int best_deg = -1;
    for (const int v : IndexRange(n)) {
      if (colored[v]) {
        continue;
      }
      const int sat = saturation[v];
      const int deg = degree[v];
      if (best < 0 || sat > best_sat || (sat == best_sat && deg > best_deg) ||
          (sat == best_sat && deg == best_deg && v < best))
      {
        best = v;
        best_sat = sat;
        best_deg = deg;
      }
    }
    BLI_assert(best >= 0);
    colors[best] = smallest_free_color(colors, adj[best], used);
    colored[best] = true;
    remaining--;

    /* Update saturation of uncolored neighbors. */
    for (const int nb : adj[best]) {
      if (!colored[nb]) {
        recompute_saturation(nb);
      }
    }
  }
  return count_colors(colors);
}

/**
 * Recursive Largest First (RLF):
 * While uncolored vertices remain, build a maximal independent set S and assign one color.
 * Selection: first pick uncolored v maximizing |U ∩ N(v)| (neighbors still uncolored);
 * then grow S by repeatedly adding the uncolored, non-adjacent-to-S vertex that maximizes
 * neighbors in the "excluded" set (adjacent to S), with ties broken by degree then index.
 */
static int algorithm_rlf(const Array<Vector<int>> &adj,
                         const Span<int> degree,
                         MutableSpan<int> colors)
{
  const int n = int(colors.size());
  colors.fill(-1);
  Array<bool> colored(n, false);
  int remaining = n;
  int next_color = 0;

  Array<bool> in_S(n);
  Array<bool> blocked(n); /* adjacent to S or in S */
  Array<bool> uncolored(n);

  while (remaining > 0) {
    for (const int v : IndexRange(n)) {
      uncolored[v] = !colored[v];
    }

    /* Pick seed: uncolored vertex maximizing uncolored-neighbor count. */
    int seed = -1;
    int best_score = -1;
    for (const int v : IndexRange(n)) {
      if (!uncolored[v]) {
        continue;
      }
      int score = 0;
      for (const int nb : adj[v]) {
        if (uncolored[nb]) {
          score++;
        }
      }
      if (seed < 0 || score > best_score ||
          (score == best_score && degree[v] > degree[seed]) ||
          (score == best_score && degree[v] == degree[seed] && v < seed))
      {
        seed = v;
        best_score = score;
      }
    }
    BLI_assert(seed >= 0);

    in_S.fill(false);
    blocked.fill(false);
    in_S[seed] = true;
    blocked[seed] = true;
    for (const int nb : adj[seed]) {
      blocked[nb] = true;
    }

    /* Grow independent set. */
    for (;;) {
      int cand = -1;
      int best_excl = -1;
      for (const int v : IndexRange(n)) {
        if (!uncolored[v] || blocked[v] || in_S[v]) {
          continue;
        }
        /* Count neighbors that are blocked but not in S (the "excluded" set Y). */
        int excl = 0;
        for (const int nb : adj[v]) {
          if (blocked[nb] && !in_S[nb]) {
            excl++;
          }
        }
        if (cand < 0 || excl > best_excl ||
            (excl == best_excl && degree[v] > degree[cand]) ||
            (excl == best_excl && degree[v] == degree[cand] && v < cand))
        {
          cand = v;
          best_excl = excl;
        }
      }
      if (cand < 0) {
        break;
      }
      in_S[cand] = true;
      blocked[cand] = true;
      for (const int nb : adj[cand]) {
        blocked[nb] = true;
      }
    }

    /* Assign color to S. */
    for (const int v : IndexRange(n)) {
      if (in_S[v]) {
        colors[v] = next_color;
        colored[v] = true;
        remaining--;
      }
    }
    next_color++;
  }

  return next_color;
}

}  // namespace

int mesh_graph_coloring(const Mesh &mesh,
                        MutableSpan<int> r_colors,
                        const MeshGraphColoringOptions &options)
{
  const int n = mesh.verts_num;
  if (n == 0 || r_colors.size() != n) {
    return 0;
  }

  Array<Vector<int>> adj;
  Array<int> degree;
  build_adjacency(mesh, adj, degree);

  switch (options.algorithm) {
    case GraphColoringAlgorithm::Greedy:
      return algorithm_greedy_index(adj, r_colors);
    case GraphColoringAlgorithm::GreedyRandom:
      return algorithm_greedy_random(adj, r_colors, options.seed);
    case GraphColoringAlgorithm::WelshPowell:
      return algorithm_welsh_powell(adj, degree, r_colors);
    case GraphColoringAlgorithm::DSatur:
      return algorithm_dsatur(adj, degree, r_colors);
    case GraphColoringAlgorithm::RLF:
      return algorithm_rlf(adj, degree, r_colors);
  }
  return algorithm_dsatur(adj, degree, r_colors);
}

int mesh_graph_coloring_store_attribute(Mesh &mesh,
                                        StringRef attribute_name,
                                        const MeshGraphColoringOptions &options)
{
  if (attribute_name.is_empty() || mesh.verts_num == 0) {
    return 0;
  }

  Array<int> colors(mesh.verts_num);
  const int ncolors = mesh_graph_coloring(mesh, colors, options);
  if (ncolors <= 0) {
    return 0;
  }

  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  attributes.remove(attribute_name);
  bke::SpanAttributeWriter<int> writer = attributes.lookup_or_add_for_write_only_span<int>(
      attribute_name, bke::AttrDomain::Point);
  if (!writer) {
    return 0;
  }
  writer.span.copy_from(colors);
  writer.finish();
  return ncolors;
}

}  // namespace blender::geometry
