/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "testing/testing.h"

#include "NOD_geometry_nodes_sparse_matrix.hh"

namespace blender::nodes::sparse_matrix::tests {

/**
 * Drive the shipped sparse COO implementation (not a re-implementation).
 * Covers add/sub/mul, zero cancellation, and empty inputs.
 */

TEST(SparseMatrixMath, AddSharedAndUnique)
{
  /* A: (0,0)=1, (0,1)=2 ; B: (0,1)=3, (1,0)=4 → (0,0)=1, (0,1)=5, (1,0)=4 */
  const std::vector<float> a_w = {1.0f, 2.0f};
  const std::vector<int> a_r = {0, 0};
  const std::vector<int> a_c = {0, 1};
  const std::vector<float> b_w = {3.0f, 4.0f};
  const std::vector<int> b_r = {0, 1};
  const std::vector<int> b_c = {1, 0};

  const COOMatrix out = sparse_matrix_add(a_w, a_r, a_c, b_w, b_r, b_c);
  ASSERT_EQ(out.size(), 3);
  EXPECT_EQ(out.row[0], 0);
  EXPECT_EQ(out.col[0], 0);
  EXPECT_FLOAT_EQ(out.weight[0], 1.0f);
  EXPECT_EQ(out.row[1], 0);
  EXPECT_EQ(out.col[1], 1);
  EXPECT_FLOAT_EQ(out.weight[1], 5.0f);
  EXPECT_EQ(out.row[2], 1);
  EXPECT_EQ(out.col[2], 0);
  EXPECT_FLOAT_EQ(out.weight[2], 4.0f);
}

TEST(SparseMatrixMath, SubtractZeroCancellation)
{
  /* A: (0,0)=5, (1,1)=2 ; B: (0,0)=5 → only (1,1)=2 remains */
  const std::vector<float> a_w = {5.0f, 2.0f};
  const std::vector<int> a_r = {0, 1};
  const std::vector<int> a_c = {0, 1};
  const std::vector<float> b_w = {5.0f};
  const std::vector<int> b_r = {0};
  const std::vector<int> b_c = {0};

  const COOMatrix out = sparse_matrix_subtract(a_w, a_r, a_c, b_w, b_r, b_c);
  ASSERT_EQ(out.size(), 1);
  EXPECT_EQ(out.row[0], 1);
  EXPECT_EQ(out.col[0], 1);
  EXPECT_FLOAT_EQ(out.weight[0], 2.0f);
}

TEST(SparseMatrixMath, MultiplyKnownPair)
{
  /* A = [[1, 2], [0, 3]] COO; B = [[4, 0], [5, 6]] COO
   * C = [[1*4+2*5, 1*0+2*6], [0*4+3*5, 0*0+3*6]] = [[14, 12], [15, 18]] */
  const std::vector<float> a_w = {1.0f, 2.0f, 3.0f};
  const std::vector<int> a_r = {0, 0, 1};
  const std::vector<int> a_c = {0, 1, 1};
  const std::vector<float> b_w = {4.0f, 5.0f, 6.0f};
  const std::vector<int> b_r = {0, 1, 1};
  const std::vector<int> b_c = {0, 0, 1};

  const COOMatrix out = sparse_matrix_multiply(a_w, a_r, a_c, b_w, b_r, b_c);
  ASSERT_EQ(out.size(), 4);
  EXPECT_EQ(out.row[0], 0);
  EXPECT_EQ(out.col[0], 0);
  EXPECT_FLOAT_EQ(out.weight[0], 14.0f);
  EXPECT_EQ(out.row[1], 0);
  EXPECT_EQ(out.col[1], 1);
  EXPECT_FLOAT_EQ(out.weight[1], 12.0f);
  EXPECT_EQ(out.row[2], 1);
  EXPECT_EQ(out.col[2], 0);
  EXPECT_FLOAT_EQ(out.weight[2], 15.0f);
  EXPECT_EQ(out.row[3], 1);
  EXPECT_EQ(out.col[3], 1);
  EXPECT_FLOAT_EQ(out.weight[3], 18.0f);
}

TEST(SparseMatrixMath, EmptyInputs)
{
  const std::vector<float> empty_w;
  const std::vector<int> empty_i;
  const std::vector<float> a_w = {1.0f};
  const std::vector<int> a_r = {0};
  const std::vector<int> a_c = {0};

  const COOMatrix add_empty = sparse_matrix_add(
      empty_w, empty_i, empty_i, empty_w, empty_i, empty_i);
  EXPECT_TRUE(add_empty.empty());

  const COOMatrix add_right_empty = sparse_matrix_add(a_w, a_r, a_c, empty_w, empty_i, empty_i);
  ASSERT_EQ(add_right_empty.size(), 1);
  EXPECT_FLOAT_EQ(add_right_empty.weight[0], 1.0f);

  const COOMatrix mul_empty = sparse_matrix_multiply(a_w, a_r, a_c, empty_w, empty_i, empty_i);
  EXPECT_TRUE(mul_empty.empty());

  const COOMatrix mul_both_empty = sparse_matrix_multiply(
      empty_w, empty_i, empty_i, empty_w, empty_i, empty_i);
  EXPECT_TRUE(mul_both_empty.empty());
}

TEST(SparseMatrixMath, ApplyDispatch)
{
  const std::vector<float> a_w = {2.0f};
  const std::vector<int> a_r = {0};
  const std::vector<int> a_c = {0};
  const std::vector<float> b_w = {3.0f};
  const std::vector<int> b_r = {0};
  const std::vector<int> b_c = {0};

  const COOMatrix add = sparse_matrix_apply(Operation::Add,
                                            a_w.data(),
                                            a_r.data(),
                                            a_c.data(),
                                            1,
                                            b_w.data(),
                                            b_r.data(),
                                            b_c.data(),
                                            1);
  ASSERT_EQ(add.size(), 1);
  EXPECT_FLOAT_EQ(add.weight[0], 5.0f);

  const COOMatrix sub = sparse_matrix_apply(Operation::Subtract,
                                            a_w.data(),
                                            a_r.data(),
                                            a_c.data(),
                                            1,
                                            b_w.data(),
                                            b_r.data(),
                                            b_c.data(),
                                            1);
  ASSERT_EQ(sub.size(), 1);
  EXPECT_FLOAT_EQ(sub.weight[0], -1.0f);
}

TEST(SparseMatrixMath, ScaleElements)
{
  const std::vector<float> a_w = {1.0f, 2.0f, 0.0f};
  const std::vector<int> a_r = {0, 1, 2};
  const std::vector<int> a_c = {0, 1, 2};

  const COOMatrix out = sparse_matrix_scale(a_w, a_r, a_c, 3.0f);
  ASSERT_EQ(out.size(), 2);
  EXPECT_EQ(out.row[0], 0);
  EXPECT_EQ(out.col[0], 0);
  EXPECT_FLOAT_EQ(out.weight[0], 3.0f);
  EXPECT_EQ(out.row[1], 1);
  EXPECT_EQ(out.col[1], 1);
  EXPECT_FLOAT_EQ(out.weight[1], 6.0f);

  const COOMatrix zero = sparse_matrix_scale(a_w, a_r, a_c, 0.0f);
  EXPECT_TRUE(zero.empty());
}

TEST(SparseMatrixMath, TransposeSwapsRowsAndCols)
{
  /* 2x3 matrix [[1, 0, 2], [0, 3, 0]] with an explicit zero that must be dropped. */
  const std::vector<float> a_w = {2.0f, 1.0f, 3.0f, 0.0f};
  const std::vector<int> a_r = {0, 0, 1, 1};
  const std::vector<int> a_c = {2, 0, 1, 0};

  const COOMatrix out = sparse_matrix_transpose(a_w, a_r, a_c);
  ASSERT_EQ(out.size(), 3);
  /* Result is the 3x2 matrix [[1, 0], [0, 3], [2, 0]] sorted by (row, col). */
  EXPECT_EQ(out.row[0], 0);
  EXPECT_EQ(out.col[0], 0);
  EXPECT_FLOAT_EQ(out.weight[0], 1.0f);
  EXPECT_EQ(out.row[1], 1);
  EXPECT_EQ(out.col[1], 1);
  EXPECT_FLOAT_EQ(out.weight[1], 3.0f);
  EXPECT_EQ(out.row[2], 2);
  EXPECT_EQ(out.col[2], 0);
  EXPECT_FLOAT_EQ(out.weight[2], 2.0f);

  const COOMatrix back = sparse_matrix_transpose(out.weight, out.row, out.col);
  ASSERT_EQ(back.size(), 3);
  EXPECT_EQ(back.row[1], 0);
  EXPECT_EQ(back.col[1], 2);
  EXPECT_FLOAT_EQ(back.weight[1], 2.0f);

  EXPECT_TRUE(sparse_matrix_transpose({}, {}, {}).empty());
}

TEST(SparseMatrixMath, DuplicateKeysWithinMatrixAreSummed)
{
  /* Two entries at (0,0) in A: 1 and 2 → treated as 3 when adding empty B. */
  const std::vector<float> a_w = {1.0f, 2.0f};
  const std::vector<int> a_r = {0, 0};
  const std::vector<int> a_c = {0, 0};
  const std::vector<float> empty_w;
  const std::vector<int> empty_i;

  const COOMatrix out = sparse_matrix_add(a_w, a_r, a_c, empty_w, empty_i, empty_i);
  ASSERT_EQ(out.size(), 1);
  EXPECT_FLOAT_EQ(out.weight[0], 3.0f);
}

}  // namespace blender::nodes::sparse_matrix::tests
