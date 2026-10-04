/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "mesh_heat_geodesic.hh"

#include "BLI_array.hh"
#include "BLI_math_geom.hh"
#include "BLI_math_vector.hh"
#include "BLI_ordered_edge.hh"
#include "BLI_set.hh"

#include "GEO_mesh_laplacian.hh"

#include "linear_solver_math.hh"

#include <algorithm>
#include <cmath>
#include <vector>

namespace blender::nodes::mesh_heat_geodesic {
namespace {

namespace ls = blender::geometry::linear_solver;

static ls::CooMatrix coo_from_laplacian(const geometry::MeshLaplacianCOO &src)
{
  ls::CooMatrix coo;
  const int nnz = int(src.weights.size());
  coo.weight.resize(size_t(nnz));
  coo.row.resize(size_t(nnz));
  coo.col.resize(size_t(nnz));
  for (int i = 0; i < nnz; i++) {
    coo.weight[size_t(i)] = double(src.weights[i]);
    coo.row[size_t(i)] = src.rows[i];
    coo.col[size_t(i)] = src.cols[i];
  }
  return coo;
}

/** Mean length of unique undirected edges from triangle connectivity. */
static float mean_edge_length(const Span<float3> positions,
                              const Span<int> corner_verts,
                              const int faces_num)
{
  Set<OrderedEdge> edges;
  for (const int f : IndexRange(faces_num)) {
    const int i0 = corner_verts[f * 3 + 0];
    const int i1 = corner_verts[f * 3 + 1];
    const int i2 = corner_verts[f * 3 + 2];
    edges.add(OrderedEdge(i0, i1));
    edges.add(OrderedEdge(i1, i2));
    edges.add(OrderedEdge(i2, i0));
  }
  if (edges.is_empty()) {
    return 1.0f;
  }
  double sum = 0.0;
  for (const OrderedEdge &e : edges) {
    sum += double(math::distance(positions[e.v_low], positions[e.v_high]));
  }
  return float(sum / double(edges.size()));
}

/** Lumped barycentric mass: M_ii = Σ (face_area / 3). */
static Array<float> build_lumped_mass(const Span<float3> positions,
                                      const Span<int> corner_verts,
                                      const int faces_num)
{
  Array<float> mass(positions.size(), 0.0f);
  for (const int f : IndexRange(faces_num)) {
    const int i0 = corner_verts[f * 3 + 0];
    const int i1 = corner_verts[f * 3 + 1];
    const int i2 = corner_verts[f * 3 + 2];
    const float area = math::area_tri(positions[i0], positions[i1], positions[i2]);
    const float third = area / 3.0f;
    mass[i0] += third;
    mass[i1] += third;
    mass[i2] += third;
  }
  return mass;
}

/**
 * Assemble the heat operator A = M + t L (paper / libigl form).
 *
 * IMPORTANT: Mesh Laplacian's "Diffusion + Mass" path builds M + t M L
 * (row-scaled), which is NOT the heat method operator. We must form M + t L
 * explicitly from plain L and the lumped mass diagonal.
 */
static ls::CooMatrix assemble_M_plus_tL(const geometry::MeshLaplacianCOO &L,
                                        const Span<float> mass,
                                        const float t,
                                        const int n)
{
  /* Collect diagonal contribution of t*L first, then add M. */
  Array<double> diag(n, 0.0);

  ls::CooMatrix coo;
  const int nnz_L = int(L.weights.size());
  coo.weight.reserve(size_t(nnz_L) + size_t(n));
  coo.row.reserve(size_t(nnz_L) + size_t(n));
  coo.col.reserve(size_t(nnz_L) + size_t(n));

  /* Off-diagonals: t * L_ij. Diagonals accumulated then emitted with M. */
  for (int k = 0; k < nnz_L; k++) {
    const int r = L.rows[k];
    const int c = L.cols[k];
    if (r < 0 || r >= n || c < 0 || c >= n) {
      continue;
    }
    const double v = double(t) * double(L.weights[k]);
    if (r == c) {
      diag[r] += v;
    }
    else if (v != 0.0) {
      coo.row.push_back(r);
      coo.col.push_back(c);
      coo.weight.push_back(v);
    }
  }

  for (const int i : IndexRange(n)) {
    double d = diag[i];
    if (i < mass.size()) {
      d += double(mass[i]);
    }
    /* Guard isolated zero-mass vertices so LLT does not fail. */
    if (d == 0.0) {
      d = 1e-12;
    }
    coo.row.push_back(i);
    coo.col.push_back(i);
    coo.weight.push_back(d);
  }
  return coo;
}

/**
 * Face gradient of a vertex scalar field (constant per triangle).
 * ∇u = (1/(2A)) Σ u_i (n̂ × e_opposite_i), A = triangle area, n̂ unit normal.
 */
static float3 face_gradient(const float3 &p0,
                            const float3 &p1,
                            const float3 &p2,
                            const float u0,
                            const float u1,
                            const float u2)
{
  const float3 n = math::cross(p1 - p0, p2 - p0);
  const float area2 = math::length(n); /* 2A */
  if (area2 < 1e-30f) {
    return float3(0.0f);
  }
  const float3 unit_n = n / area2;
  const float3 g = u0 * math::cross(unit_n, p2 - p1) + u1 * math::cross(unit_n, p0 - p2) +
                   u2 * math::cross(unit_n, p1 - p0);
  return g / area2;
}

/**
 * Integrated divergence of a per-face vector field X into vertex DOFs
 * (Crane / libigl cot form).
 */
static void integrated_divergence(const Span<float3> positions,
                                  const Span<int> corner_verts,
                                  const int faces_num,
                                  const Span<float3> face_X,
                                  MutableSpan<double> div)
{
  const int n = positions.size();
  div.fill(0.0);
  for (const int f : IndexRange(faces_num)) {
    const int i0 = corner_verts[f * 3 + 0];
    const int i1 = corner_verts[f * 3 + 1];
    const int i2 = corner_verts[f * 3 + 2];
    if (i0 < 0 || i0 >= n || i1 < 0 || i1 >= n || i2 < 0 || i2 >= n) {
      continue;
    }
    const float3 &p0 = positions[i0];
    const float3 &p1 = positions[i1];
    const float3 &p2 = positions[i2];
    const float3 X = face_X[f];

    const float cot0 = math::cotangent_tri_weight(p0, p1, p2);
    const float cot1 = math::cotangent_tri_weight(p1, p2, p0);
    const float cot2 = math::cotangent_tri_weight(p2, p0, p1);

    div[i0] += 0.5 * double(cot2 * math::dot(X, p1 - p0) + cot1 * math::dot(X, p2 - p0));
    div[i1] += 0.5 * double(cot0 * math::dot(X, p2 - p1) + cot2 * math::dot(X, p0 - p1));
    div[i2] += 0.5 * double(cot1 * math::dot(X, p0 - p2) + cot0 * math::dot(X, p1 - p2));
  }
}

static ls::SolveOptions direct_llt_options()
{
  ls::SolveOptions opt;
  opt.solver_class = ls::SolverClass::Direct;
  opt.direct = ls::DirectMethod::LLT;
  return opt;
}

static ls::SolveOptions direct_ldlt_options()
{
  ls::SolveOptions opt;
  opt.solver_class = ls::SolverClass::Direct;
  opt.direct = ls::DirectMethod::LDLT;
  return opt;
}

static bool solve_spd(const ls::CooMatrix &A,
                      const std::vector<double> &b,
                      const std::vector<char> &pin,
                      std::vector<double> &x,
                      std::string &message)
{
  ls::SolveResult r = ls::solve_system(A, b, pin, direct_llt_options());
  if (!r.success) {
    r = ls::solve_system(A, b, pin, direct_ldlt_options());
  }
  if (!r.success) {
    message = r.message.empty() ? "Sparse solve failed" : r.message;
    return false;
  }
  x = std::move(r.x);
  return true;
}

}  // namespace

HeatGeodesicResult compute(const Span<float3> positions,
                           const Span<int> corner_verts,
                           const int faces_num,
                           const Span<bool> is_source)
{
  HeatGeodesicResult result;
  const int n = positions.size();
  if (n == 0) {
    result.success = true;
    return result;
  }
  if (faces_num <= 0 || corner_verts.size() < int64_t(faces_num) * 3) {
    result.message = "Mesh has no triangles";
    result.distance = Vector<float>(n, 0.0f);
    return result;
  }
  if (is_source.size() < n) {
    result.message = "Source mask shorter than vertex count";
    result.distance = Vector<float>(n, 0.0f);
    return result;
  }

  int source_count = 0;
  for (const int i : IndexRange(n)) {
    if (is_source[i]) {
      source_count++;
    }
  }
  if (source_count == 0) {
    result.success = true;
    result.distance = Vector<float>(n, 0.0f);
    result.message = "No source vertices";
    return result;
  }

  /* Paper default: t = h² (mean edge length squared). */
  const float h = mean_edge_length(positions, corner_verts, faces_num);
  result.mean_edge = h;
  const float t = h * h;
  if (!(t > 0.0f) || !std::isfinite(t)) {
    result.message = "Invalid mean edge length (cannot form t = h²)";
    result.distance = Vector<float>(n, 0.0f);
    return result;
  }
  result.time_used = t;

  /* Cotangent Laplacian only (positive semi-definite Mesh Laplacian). */
  geometry::MeshLaplacianOptions L_opts;
  L_opts.mode = geometry::MeshLaplacianWeightMode::Cotangent;
  L_opts.build_diffusion = false;
  L_opts.use_mass = false;
  const geometry::MeshLaplacianCOO L_coo = geometry::mesh_laplacian_build(
      positions, corner_verts, faces_num, L_opts);
  if (L_coo.weights.is_empty()) {
    result.message = "Failed to build Laplacian";
    result.distance = Vector<float>(n, 0.0f);
    return result;
  }

  /* --- Step 1: heat diffusion (M + t L) u = δ --- */
  const Array<float> mass = build_lumped_mass(positions, corner_verts, faces_num);
  const ls::CooMatrix A_heat = assemble_M_plus_tL(L_coo, mass, t, n);

  std::vector<double> delta(size_t(n), 0.0);
  for (const int i : IndexRange(n)) {
    if (is_source[i]) {
      delta[size_t(i)] = 1.0;
    }
  }
  std::vector<char> no_pin(size_t(n), 0);
  std::vector<double> u_vec;
  std::string err;
  if (!solve_spd(A_heat, delta, no_pin, u_vec, err) || int(u_vec.size()) < n) {
    result.message = err.empty() ? "Heat solve failed" : err;
    result.distance = Vector<float>(n, 0.0f);
    return result;
  }

  Array<float> u(n);
  for (const int i : IndexRange(n)) {
    u[i] = float(u_vec[size_t(i)]);
  }

  /* --- Step 2: unit vector field X = −∇u / |∇u| --- */
  Array<float3> face_X(faces_num, float3(0.0f));
  for (const int f : IndexRange(faces_num)) {
    const int i0 = corner_verts[f * 3 + 0];
    const int i1 = corner_verts[f * 3 + 1];
    const int i2 = corner_verts[f * 3 + 2];
    const float3 g = face_gradient(
        positions[i0], positions[i1], positions[i2], u[i0], u[i1], u[i2]);
    const float len = math::length(g);
    if (len > 1e-20f) {
      face_X[f] = -g / len;
    }
  }

  /* --- Step 3: integrated divergence --- */
  std::vector<double> div(size_t(n), 0.0);
  integrated_divergence(positions, corner_verts, faces_num, face_X, div);

  /* --- Step 4: Poisson L φ = div, pin sources to 0 --- */
  std::vector<char> pin(size_t(n), 0);
  for (const int i : IndexRange(n)) {
    if (is_source[i]) {
      pin[size_t(i)] = 1;
      /* Dirichlet values come from b on pinned DOFs. */
      div[size_t(i)] = 0.0;
    }
  }

  const ls::CooMatrix A_L = coo_from_laplacian(L_coo);
  std::vector<double> phi;
  if (!solve_spd(A_L, div, pin, phi, err) || int(phi.size()) < n) {
    result.message = err.empty() ? "Poisson solve failed" : err;
    result.distance = Vector<float>(n, 0.0f);
    return result;
  }

  result.distance = Vector<float>(n);
  for (const int i : IndexRange(n)) {
    result.distance[i] = float(phi[size_t(i)]);
  }

  /* Distance unique up to sign; flip so the field is non-negative away from sources. */
  double sum = 0.0;
  int free_count = 0;
  for (const int i : IndexRange(n)) {
    if (!is_source[i]) {
      sum += double(result.distance[i]);
      free_count++;
    }
  }
  if (free_count > 0 && sum < 0.0) {
    for (const int i : IndexRange(n)) {
      result.distance[i] = -result.distance[i];
    }
  }

  for (const int i : IndexRange(n)) {
    if (is_source[i]) {
      result.distance[i] = 0.0f;
    }
    else if (result.distance[i] < 0.0f) {
      /* Numerical noise near sources; do not abs() far-field negatives (sign bug). */
      if (result.distance[i] > -1e-4f) {
        result.distance[i] = 0.0f;
      }
    }
  }

  result.success = true;
  return result;
}

}  // namespace blender::nodes::mesh_heat_geodesic
