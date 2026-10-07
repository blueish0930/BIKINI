/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Standalone driver for the shipped sparse matrix math header.
 *
 * Build (from this directory, header one level up):
 *   cl /EHsc /std:c++17 /I.. /DSPARSE_MATRIX_SELFTEST_MAIN ^
 *      /Fe:sparse_matrix_selftest.exe geometry_nodes_sparse_matrix_selftest.cc
 *
 * Or:
 *   g++ -std=c++17 -O2 -I.. -DSPARSE_MATRIX_SELFTEST_MAIN \
 *       -o sparse_matrix_selftest geometry_nodes_sparse_matrix_selftest.cc
 *
 * Calls the same functions used by GeometryNodeSparseMatrixMath (Eigen-backed).
 */

#include "NOD_geometry_nodes_sparse_matrix.hh"

#include <cstdio>

namespace {

using blender::nodes::sparse_matrix::COOMatrix;
using blender::nodes::sparse_matrix::Operation;
using blender::nodes::sparse_matrix::sparse_matrix_add;
using blender::nodes::sparse_matrix::sparse_matrix_apply;
using blender::nodes::sparse_matrix::sparse_matrix_multiply;
using blender::nodes::sparse_matrix::sparse_matrix_scale;
using blender::nodes::sparse_matrix::sparse_matrix_transpose;
using blender::nodes::sparse_matrix::sparse_matrix_subtract;

int g_failures = 0;

void check(const bool cond, const char *msg)
{
  if (!cond) {
    std::fprintf(stderr, "FAIL: %s\n", msg);
    g_failures++;
  }
  else {
    std::printf("PASS: %s\n", msg);
  }
}

void check_float_eq(const float a, const float b, const char *msg)
{
  check(a == b, msg);
}

void test_add()
{
  const std::vector<float> a_w = {1.0f, 2.0f};
  const std::vector<int> a_r = {0, 0};
  const std::vector<int> a_c = {0, 1};
  const std::vector<float> b_w = {3.0f, 4.0f};
  const std::vector<int> b_r = {0, 1};
  const std::vector<int> b_c = {1, 0};
  const COOMatrix out = sparse_matrix_add(a_w, a_r, a_c, b_w, b_r, b_c);
  check(out.size() == 3, "add size");
  check_float_eq(out.weight[0], 1.0f, "add (0,0)");
  check_float_eq(out.weight[1], 5.0f, "add (0,1)");
  check_float_eq(out.weight[2], 4.0f, "add (1,0)");
}

void test_sub_zero_cancel()
{
  const std::vector<float> a_w = {5.0f, 2.0f};
  const std::vector<int> a_r = {0, 1};
  const std::vector<int> a_c = {0, 1};
  const std::vector<float> b_w = {5.0f};
  const std::vector<int> b_r = {0};
  const std::vector<int> b_c = {0};
  const COOMatrix out = sparse_matrix_subtract(a_w, a_r, a_c, b_w, b_r, b_c);
  check(out.size() == 1, "sub cancel size");
  check(out.row[0] == 1 && out.col[0] == 1, "sub remaining key");
  check_float_eq(out.weight[0], 2.0f, "sub remaining weight");
}

void test_mul()
{
  const std::vector<float> a_w = {1.0f, 2.0f, 3.0f};
  const std::vector<int> a_r = {0, 0, 1};
  const std::vector<int> a_c = {0, 1, 1};
  const std::vector<float> b_w = {4.0f, 5.0f, 6.0f};
  const std::vector<int> b_r = {0, 1, 1};
  const std::vector<int> b_c = {0, 0, 1};
  const COOMatrix out = sparse_matrix_multiply(a_w, a_r, a_c, b_w, b_r, b_c);
  check(out.size() == 4, "mul size");
  check_float_eq(out.weight[0], 14.0f, "mul (0,0)=14");
  check_float_eq(out.weight[1], 12.0f, "mul (0,1)=12");
  check_float_eq(out.weight[2], 15.0f, "mul (1,0)=15");
  check_float_eq(out.weight[3], 18.0f, "mul (1,1)=18");
}

void test_empty()
{
  const std::vector<float> empty_w;
  const std::vector<int> empty_i;
  const std::vector<float> a_w = {1.0f};
  const std::vector<int> a_r = {0};
  const std::vector<int> a_c = {0};
  check(sparse_matrix_add(empty_w, empty_i, empty_i, empty_w, empty_i, empty_i).empty(),
        "add both empty");
  check(sparse_matrix_multiply(a_w, a_r, a_c, empty_w, empty_i, empty_i).empty(),
        "mul right empty");
  check(sparse_matrix_multiply(empty_w, empty_i, empty_i, empty_w, empty_i, empty_i).empty(),
        "mul both empty");
}

void test_scale()
{
  const std::vector<float> a_w = {1.0f, 2.0f};
  const std::vector<int> a_r = {0, 1};
  const std::vector<int> a_c = {0, 1};
  const COOMatrix out = sparse_matrix_scale(a_w, a_r, a_c, 3.0f);
  check(out.size() == 2, "scale size");
  check_float_eq(out.weight[0], 3.0f, "scale (0,0)");
  check_float_eq(out.weight[1], 6.0f, "scale (1,1)");
  check(sparse_matrix_scale(a_w, a_r, a_c, 0.0f).empty(), "scale by zero");
}

void test_transpose()
{
  const std::vector<float> a_w = {2.0f, 1.0f, 3.0f};
  const std::vector<int> a_r = {0, 0, 1};
  const std::vector<int> a_c = {2, 0, 1};
  const COOMatrix out = sparse_matrix_transpose(a_w, a_r, a_c);
  check(out.size() == 3, "transpose size");
  check(out.row[2] == 2 && out.col[2] == 0, "transpose (0,2) -> (2,0)");
  check_float_eq(out.weight[2], 2.0f, "transpose value");
}

void test_apply()
{
  const std::vector<float> a_w = {2.0f};
  const std::vector<int> a_r = {0};
  const std::vector<int> a_c = {0};
  const std::vector<float> b_w = {3.0f};
  const std::vector<int> b_r = {0};
  const std::vector<int> b_c = {0};
  const COOMatrix add = sparse_matrix_apply(
      Operation::Add, a_w.data(), a_r.data(), a_c.data(), 1, b_w.data(), b_r.data(), b_c.data(), 1);
  check_float_eq(add.weight[0], 5.0f, "apply add");
}

}  // namespace

int sparse_matrix_selftest_run()
{
  g_failures = 0;
  std::printf("=== sparse matrix shipped math selftest ===\n");
  test_add();
  test_sub_zero_cancel();
  test_mul();
  test_empty();
  test_scale();
  test_transpose();
  test_apply();
  if (g_failures == 0) {
    std::printf("ALL PASSED\n");
  }
  else {
    std::printf("%d FAILURE(S)\n", g_failures);
  }
  return g_failures == 0 ? 0 : 1;
}

#ifdef SPARSE_MATRIX_SELFTEST_MAIN
int main()
{
  return sparse_matrix_selftest_run();
}
#endif
