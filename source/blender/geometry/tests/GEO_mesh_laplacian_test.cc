/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_math_geom.hh"
#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include "GEO_mesh_laplacian.hh"

#include "testing/testing.h"

namespace blender::geometry::tests {
namespace {

/* Pure assembly tests only need BLI + mesh_laplacian_build; no full BKE gtest setup. */
class MeshLaplacianTest : public testing::Test {};

/** Reconstruct dense N×N from COO (missing entries are zero). */
static Array<float> dense_from_coo(const MeshLaplacianCOO &coo, const int n)
{
  Array<float> dense(int64_t(n) * n, 0.0f);
  EXPECT_EQ(coo.weights.size(), coo.rows.size());
  EXPECT_EQ(coo.weights.size(), coo.cols.size());
  for (const int i : coo.weights.index_range()) {
    EXPECT_NE(coo.weights[i], 0.0f);
    EXPECT_GE(coo.rows[i], 0);
    EXPECT_LT(coo.rows[i], n);
    EXPECT_GE(coo.cols[i], 0);
    EXPECT_LT(coo.cols[i], n);
    dense[int64_t(coo.rows[i]) * n + coo.cols[i]] = coo.weights[i];
  }
  return dense;
}

/** For plain Laplacian L: each row sums to ~0 (diag = −sum off-diag). */
static void expect_row_sum_zero(const Array<float> &dense, const int n, const float tol = 1e-5f)
{
  for (const int r : IndexRange(n)) {
    float sum = 0.0f;
    for (const int c : IndexRange(n)) {
      sum += dense[int64_t(r) * n + c];
    }
    EXPECT_NEAR(sum, 0.0f, tol) << "row " << r;
  }
}

/**
 * Single equilateral triangle in the XY plane (side length 2).
 * Corners: 0:(0,0,0), 1:(2,0,0), 2:(1,√3,0).
 */
static void equilateral_triangle(Vector<float3> &positions, Vector<int> &corners)
{
  positions = {float3(0.0f, 0.0f, 0.0f),
               float3(2.0f, 0.0f, 0.0f),
               float3(1.0f, math::sqrt(3.0f), 0.0f)};
  corners = {0, 1, 2};
}

/**
 * Two triangles sharing edge (0,1).
 */
static void two_triangle_mesh(Vector<float3> &positions, Vector<int> &corners)
{
  positions = {
      float3(0.0f, 0.0f, 0.0f),
      float3(1.0f, 0.0f, 0.0f),
      float3(0.5f, 1.0f, 0.0f),
      float3(0.5f, -1.0f, 0.0f),
  };
  corners = {0, 1, 2, 0, 3, 1};
}

TEST_F(MeshLaplacianTest, UniformSingleTriangle_RowSumAndAdjacency)
{
  Vector<float3> positions;
  Vector<int> corners;
  equilateral_triangle(positions, corners);

  MeshLaplacianOptions opt;
  opt.mode = MeshLaplacianWeightMode::Uniform;
  opt.build_diffusion = false;
  opt.use_mass = false;

  const MeshLaplacianCOO coo = mesh_laplacian_build(positions, corners, 1, opt);
  const int n = positions.size();
  const Array<float> dense = dense_from_coo(coo, n);

  /* Every pair is connected: off-diag −1, diagonal 2. */
  for (const int i : IndexRange(n)) {
    EXPECT_NEAR(dense[int64_t(i) * n + i], 2.0f, 1e-6f);
    for (const int j : IndexRange(n)) {
      if (i != j) {
        EXPECT_NEAR(dense[int64_t(i) * n + j], -1.0f, 1e-6f);
      }
    }
  }
  expect_row_sum_zero(dense, n);
  /* Only nonzeros stored: 9 entries (full 3×3 all nonzero). */
  EXPECT_EQ(coo.weights.size(), 9);
}

TEST_F(MeshLaplacianTest, CotSingleEquilateralTriangle_OppositeAngleWeights)
{
  Vector<float3> positions;
  Vector<int> corners;
  equilateral_triangle(positions, corners);

  MeshLaplacianOptions opt;
  opt.mode = MeshLaplacianWeightMode::Cotangent;
  opt.build_diffusion = false;
  opt.use_mass = false;

  const MeshLaplacianCOO coo = mesh_laplacian_build(positions, corners, 1, opt);
  const int n = positions.size();
  const Array<float> dense = dense_from_coo(coo, n);

  /* Equilateral: off-diagonal = −½ cot(60°) via shipped helper. */
  const float expected_off = -0.5f *
                             math::cotangent_tri_weight(positions[0], positions[1], positions[2]);
  EXPECT_NEAR(expected_off, -0.5f / math::sqrt(3.0f), 1e-5f);

  for (const int i : IndexRange(n)) {
    for (const int j : IndexRange(n)) {
      if (i != j) {
        EXPECT_NEAR(dense[int64_t(i) * n + j], expected_off, 1e-5f)
            << "L[" << i << "," << j << "]";
      }
    }
    /* Diagonal = −2 * off (two neighbors). */
    EXPECT_NEAR(dense[int64_t(i) * n + i], -2.0f * expected_off, 1e-5f);
  }
  expect_row_sum_zero(dense, n);
}

TEST_F(MeshLaplacianTest, CotTwoTriangles_SharedEdgeAccumulatesBothCots)
{
  Vector<float3> positions;
  Vector<int> corners;
  two_triangle_mesh(positions, corners);

  MeshLaplacianOptions opt;
  opt.mode = MeshLaplacianWeightMode::Cotangent;

  const MeshLaplacianCOO coo = mesh_laplacian_build(positions, corners, 2, opt);
  const int n = positions.size();
  const Array<float> dense = dense_from_coo(coo, n);

  /* Shared edge (0,1): contributions from both opposite angles at 2 and at 3. */
  const float cot_at_2 = 0.5f *
                         math::cotangent_tri_weight(positions[2], positions[0], positions[1]);
  const float cot_at_3 = 0.5f *
                         math::cotangent_tri_weight(positions[3], positions[0], positions[1]);
  const float expected_01 = -(cot_at_2 + cot_at_3);
  EXPECT_NEAR(dense[0 * n + 1], expected_01, 1e-5f);
  EXPECT_NEAR(dense[1 * n + 0], expected_01, 1e-5f);

  expect_row_sum_zero(dense, n);

  EXPECT_EQ(coo.weights.size(), coo.rows.size());
  EXPECT_EQ(coo.weights.size(), coo.cols.size());
  for (const float w : coo.weights) {
    EXPECT_NE(w, 0.0f);
  }
}

TEST_F(MeshLaplacianTest, Diffusion_I_plus_tL)
{
  Vector<float3> positions;
  Vector<int> corners;
  equilateral_triangle(positions, corners);

  MeshLaplacianOptions base;
  base.mode = MeshLaplacianWeightMode::Uniform;
  base.build_diffusion = false;
  base.use_mass = false;
  const MeshLaplacianCOO L_coo = mesh_laplacian_build(positions, corners, 1, base);

  const float t = 0.25f;
  MeshLaplacianOptions diff = base;
  diff.build_diffusion = true;
  diff.diffusion_t = t;
  const MeshLaplacianCOO D_coo = mesh_laplacian_build(positions, corners, 1, diff);

  const int n = positions.size();
  const Array<float> L = dense_from_coo(L_coo, n);
  const Array<float> D = dense_from_coo(D_coo, n);

  for (const int i : IndexRange(n)) {
    for (const int j : IndexRange(n)) {
      const float expected = (i == j ? 1.0f : 0.0f) + t * L[int64_t(i) * n + j];
      EXPECT_NEAR(D[int64_t(i) * n + j], expected, 1e-5f) << i << "," << j;
    }
  }
}

TEST_F(MeshLaplacianTest, MassDiffusion_M_plus_tML)
{
  Vector<float3> positions;
  Vector<int> corners;
  equilateral_triangle(positions, corners);

  MeshLaplacianOptions base;
  base.mode = MeshLaplacianWeightMode::Uniform;
  base.build_diffusion = false;
  base.use_mass = false;
  const MeshLaplacianCOO L_coo = mesh_laplacian_build(positions, corners, 1, base);

  /* Lumped mass: each vertex gets area/3 of the single triangle. */
  const float area = math::area_tri(positions[0], positions[1], positions[2]);
  const float m_ii = area / 3.0f;
  EXPECT_GT(m_ii, 0.0f);

  const float t = 0.5f;
  MeshLaplacianOptions opt;
  opt.mode = MeshLaplacianWeightMode::Uniform;
  opt.build_diffusion = true;
  opt.diffusion_t = t;
  opt.use_mass = true;
  const MeshLaplacianCOO coo = mesh_laplacian_build(positions, corners, 1, opt);

  const int n = positions.size();
  const Array<float> L = dense_from_coo(L_coo, n);
  const Array<float> R = dense_from_coo(coo, n);

  for (const int i : IndexRange(n)) {
    for (const int j : IndexRange(n)) {
      const float expected = (i == j ? m_ii : 0.0f) + t * m_ii * L[int64_t(i) * n + j];
      EXPECT_NEAR(R[int64_t(i) * n + j], expected, 1e-4f) << i << "," << j;
    }
  }
}

TEST_F(MeshLaplacianTest, MassOnly_ScalesRowsByM)
{
  Vector<float3> positions;
  Vector<int> corners;
  equilateral_triangle(positions, corners);

  MeshLaplacianOptions base;
  base.mode = MeshLaplacianWeightMode::Cotangent;
  const MeshLaplacianCOO L_coo = mesh_laplacian_build(positions, corners, 1, base);

  MeshLaplacianOptions mass_opt = base;
  mass_opt.use_mass = true;
  const MeshLaplacianCOO ML_coo = mesh_laplacian_build(positions, corners, 1, mass_opt);

  const float area = math::area_tri(positions[0], positions[1], positions[2]);
  const float m_ii = area / 3.0f;
  const int n = positions.size();
  const Array<float> L = dense_from_coo(L_coo, n);
  const Array<float> ML = dense_from_coo(ML_coo, n);

  for (const int i : IndexRange(n)) {
    for (const int j : IndexRange(n)) {
      EXPECT_NEAR(ML[int64_t(i) * n + j], m_ii * L[int64_t(i) * n + j], 1e-4f);
    }
  }
}

TEST_F(MeshLaplacianTest, EmptyMesh_ReturnsEmpty)
{
  MeshLaplacianOptions opt;
  const MeshLaplacianCOO coo = mesh_laplacian_build({}, {}, 0, opt);
  EXPECT_TRUE(coo.weights.is_empty());
  EXPECT_TRUE(coo.rows.is_empty());
  EXPECT_TRUE(coo.cols.is_empty());
}

TEST_F(MeshLaplacianTest, COO_IsRowMajorOrdered)
{
  Vector<float3> positions;
  Vector<int> corners;
  two_triangle_mesh(positions, corners);

  MeshLaplacianOptions opt;
  opt.mode = MeshLaplacianWeightMode::Uniform;
  const MeshLaplacianCOO coo = mesh_laplacian_build(positions, corners, 2, opt);

  EXPECT_GT(coo.weights.size(), 0);
  for (const int i : IndexRange(coo.rows.size() - 1)) {
    const int r0 = coo.rows[i];
    const int c0 = coo.cols[i];
    const int r1 = coo.rows[i + 1];
    const int c1 = coo.cols[i + 1];
    EXPECT_TRUE(r0 < r1 || (r0 == r1 && c0 <= c1))
        << "out of order at " << i << ": (" << r0 << "," << c0 << ") then (" << r1 << "," << c1
        << ")";
  }
  /* First stored row should be the lowest connected vertex index (0). */
  EXPECT_EQ(coo.rows[0], 0);
}

TEST_F(MeshLaplacianTest, COO_Equal_lengths)
{
  Vector<float3> positions;
  Vector<int> corners;
  two_triangle_mesh(positions, corners);

  MeshLaplacianOptions opt;
  opt.mode = MeshLaplacianWeightMode::Uniform;
  opt.build_diffusion = true;
  opt.diffusion_t = 1.0f;
  opt.use_mass = true;

  const MeshLaplacianCOO coo = mesh_laplacian_build(positions, corners, 2, opt);
  EXPECT_EQ(coo.weights.size(), coo.rows.size());
  EXPECT_EQ(coo.weights.size(), coo.cols.size());
  EXPECT_GT(coo.weights.size(), 0);
}

}  // namespace
}  // namespace blender::geometry::tests
