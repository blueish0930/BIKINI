/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_map.hh"
#include "BLI_math_geom.hh"
#include "BLI_math_vector.hh"
#include "BLI_ordered_edge.hh"
#include "BLI_set.hh"

#include <algorithm>

#include "DNA_mesh_types.h"

#include "BKE_mesh.hh"

#include "GEO_mesh_laplacian.hh"

namespace blender::geometry {
namespace {

/** Pack (row, col) into a single key for Map lookup. */
static int64_t entry_key(const int row, const int col, const int64_t n)
{
  return int64_t(row) * n + int64_t(col);
}

static void add_entry(Map<int64_t, float> &entries,
                      const int row,
                      const int col,
                      const int64_t n,
                      const float value)
{
  if (value == 0.0f) {
    return;
  }
  entries.add_or_modify(
      entry_key(row, col, n),
      [&](float *dst) { *dst = value; },
      [&](float *dst) { *dst += value; });
}

/**
 * Build the raw Laplacian L as a sparse map of nonzero entries (diagonal filled after
 * off-diagonals so L_ii = −sum_j L_ij).
 */
static Map<int64_t, float> build_laplacian_L(const Span<float3> positions,
                                             const Span<int> corner_verts,
                                             const int faces_num,
                                             const MeshLaplacianOptions &options)
{
  const MeshLaplacianWeightMode mode = options.mode;
  const int n = positions.size();
  const int64_t n64 = n;
  Map<int64_t, float> entries;

  if (mode == MeshLaplacianWeightMode::Custom) {
    /* Caller-provided entries are the matrix: no sign change and no derived diagonal. */
    BLI_assert(options.custom_rows.size() == options.custom_weights.size());
    BLI_assert(options.custom_cols.size() == options.custom_weights.size());
    for (const int i : options.custom_weights.index_range()) {
      const int row = options.custom_rows[i];
      const int col = options.custom_cols[i];
      if (row < 0 || row >= n || col < 0 || col >= n) {
        continue;
      }
      add_entry(entries, row, col, n64, options.custom_weights[i]);
    }
    return entries;
  }
  if (mode == MeshLaplacianWeightMode::Uniform) {
    /* Graph Laplacian over unique undirected edges from triangle connectivity. */
    Set<OrderedEdge> edges;
    for (const int face_i : IndexRange(faces_num)) {
      const int i0 = corner_verts[face_i * 3 + 0];
      const int i1 = corner_verts[face_i * 3 + 1];
      const int i2 = corner_verts[face_i * 3 + 2];
      edges.add(OrderedEdge(i0, i1));
      edges.add(OrderedEdge(i1, i2));
      edges.add(OrderedEdge(i2, i0));
    }
    for (const OrderedEdge &e : edges) {
      add_entry(entries, e.v_low, e.v_high, n64, -1.0f);
      add_entry(entries, e.v_high, e.v_low, n64, -1.0f);
    }
  }
  else {
    /* Cotangent: for each triangle, edge opposite vertex gets −½ cot(angle at vertex). */
    for (const int face_i : IndexRange(faces_num)) {
      const int i0 = corner_verts[face_i * 3 + 0];
      const int i1 = corner_verts[face_i * 3 + 1];
      const int i2 = corner_verts[face_i * 3 + 2];
      const float3 &p0 = positions[i0];
      const float3 &p1 = positions[i1];
      const float3 &p2 = positions[i2];

      const float cot0 = 0.5f * math::cotangent_tri_weight(p0, p1, p2);
      const float cot1 = 0.5f * math::cotangent_tri_weight(p1, p2, p0);
      const float cot2 = 0.5f * math::cotangent_tri_weight(p2, p0, p1);

      /* Edge (1,2) opposite vertex 0. */
      add_entry(entries, i1, i2, n64, -cot0);
      add_entry(entries, i2, i1, n64, -cot0);
      /* Edge (2,0) opposite vertex 1. */
      add_entry(entries, i2, i0, n64, -cot1);
      add_entry(entries, i0, i2, n64, -cot1);
      /* Edge (0,1) opposite vertex 2. */
      add_entry(entries, i0, i1, n64, -cot2);
      add_entry(entries, i1, i0, n64, -cot2);
    }
  }

  /* Diagonal = negative sum of off-diagonals per row. */
  Array<float> row_off_sum(n, 0.0f);
  for (const auto item : entries.items()) {
    const int row = int(item.key / n64);
    const int col = int(item.key % n64);
    if (row != col) {
      row_off_sum[row] += item.value;
    }
  }
  for (const int i : IndexRange(n)) {
    const float diag = -row_off_sum[i];
    if (diag != 0.0f) {
      entries.add_new(entry_key(i, i, n64), diag);
    }
  }

  return entries;
}

/** Lumped barycentric mass: M_ii = Σ (face_area / 3) over incident triangles. */
static Array<float> build_lumped_mass(const Span<float3> positions,
                                      const Span<int> corner_verts,
                                      const int faces_num)
{
  Array<float> mass(positions.size(), 0.0f);
  for (const int face_i : IndexRange(faces_num)) {
    const int i0 = corner_verts[face_i * 3 + 0];
    const int i1 = corner_verts[face_i * 3 + 1];
    const int i2 = corner_verts[face_i * 3 + 2];
    const float area = math::area_tri(positions[i0], positions[i1], positions[i2]);
    const float third = area / 3.0f;
    mass[i0] += third;
    mass[i1] += third;
    mass[i2] += third;
  }
  return mass;
}

struct COOTriple {
  int row;
  int col;
  float weight;
};

/**
 * Emit COO triples in strict row-major order:
 *   all nonzeros of vertex/row 0 (cols ascending), then row 1, …, row N-1.
 * Parallel arrays stay matched: weight[i] is the value at (row[i], col[i]).
 * Never emit Map iteration order (hash-unordered).
 */
static MeshLaplacianCOO emit_coo(const Map<int64_t, float> &entries, const int64_t n)
{
  Vector<COOTriple> triples;
  triples.reserve(entries.size());
  for (const auto item : entries.items()) {
    if (item.value == 0.0f) {
      continue;
    }
    triples.append(COOTriple{int(item.key / n), int(item.key % n), item.value});
  }

  std::sort(triples.begin(), triples.end(), [](const COOTriple &a, const COOTriple &b) {
    if (a.row != b.row) {
      return a.row < b.row;
    }
    return a.col < b.col;
  });

  MeshLaplacianCOO coo;
  coo.weights.reserve(triples.size());
  coo.rows.reserve(triples.size());
  coo.cols.reserve(triples.size());
  for (const COOTriple &t : triples) {
    coo.rows.append(t.row);
    coo.cols.append(t.col);
    coo.weights.append(t.weight);
  }
  return coo;
}

}  // namespace

MeshLaplacianCOO mesh_laplacian_build(const Span<float3> positions,
                                      const Span<int> corner_verts,
                                      const int faces_num,
                                      const MeshLaplacianOptions &options)
{
  const int n = positions.size();
  MeshLaplacianCOO empty;
  if (n == 0 || corner_verts.size() < int64_t(faces_num) * 3) {
    return empty;
  }
  if (faces_num <= 0 && options.mode != MeshLaplacianWeightMode::Custom) {
    return empty;
  }

  const int64_t n64 = n;
  Map<int64_t, float> L = build_laplacian_L(positions, corner_verts, faces_num, options);

  const bool use_mass = options.use_mass;
  const bool diffuse = options.build_diffusion;
  const float t = options.diffusion_t;

  Array<float> mass;
  if (use_mass) {
    mass = build_lumped_mass(positions, corner_verts, faces_num);
  }

  /*
   * Assemble final operator:
   *   plain L
   *   M L          (mass, no diffusion)
   *   I + t L      (diffusion, no mass)
   *   M + t M L    (diffusion + mass)
   */
  Map<int64_t, float> result;

  if (!use_mass && !diffuse) {
    result = std::move(L);
  }
  else if (use_mass && !diffuse) {
    /* Row-scale L by M_ii. */
    for (const auto item : L.items()) {
      const int row = int(item.key / n64);
      const float value = mass[row] * item.value;
      if (value != 0.0f) {
        result.add_new(item.key, value);
      }
    }
  }
  else if (!use_mass && diffuse) {
    /* I + t L. */
    for (const int i : IndexRange(n)) {
      result.add_new(entry_key(i, i, n64), 1.0f);
    }
    for (const auto item : L.items()) {
      result.add_or_modify(
          item.key,
          [&](float *dst) { *dst = t * item.value; },
          [&](float *dst) { *dst += t * item.value; });
    }
  }
  else {
    /* M + t M L. */
    for (const int i : IndexRange(n)) {
      if (mass[i] != 0.0f) {
        result.add_new(entry_key(i, i, n64), mass[i]);
      }
    }
    for (const auto item : L.items()) {
      const int row = int(item.key / n64);
      const float value = t * mass[row] * item.value;
      if (value == 0.0f) {
        continue;
      }
      result.add_or_modify(
          item.key,
          [&](float *dst) { *dst = value; },
          [&](float *dst) { *dst += value; });
    }
  }

  return emit_coo(result, n64);
}

MeshLaplacianCOO mesh_laplacian_from_mesh(const Mesh &mesh, const MeshLaplacianOptions &options)
{
  const Span<float3> positions = mesh.vert_positions();
  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();

  /* Collect only triangular faces into a compact corner list. */
  Vector<int> tri_corners;
  tri_corners.reserve(faces.size() * 3);
  int faces_num = 0;
  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    if (face.size() != 3) {
      continue;
    }
    tri_corners.append(corner_verts[face[0]]);
    tri_corners.append(corner_verts[face[1]]);
    tri_corners.append(corner_verts[face[2]]);
    faces_num++;
  }

  return mesh_laplacian_build(positions, tri_corners.as_span(), faces_num, options);
}

}  // namespace blender::geometry
