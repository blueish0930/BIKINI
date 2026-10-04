/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_geometry_nodes_sparse_matrix.hh"

#include <Eigen/Sparse>

#include <algorithm>
#include <vector>

namespace blender::nodes::sparse_matrix {

using SpMat = Eigen::SparseMatrix<float>;
using SpMatRow = Eigen::SparseMatrix<float, Eigen::RowMajor>;
using Triplet = Eigen::Triplet<float>;

static int max_index(const int *idx, const int64_t size)
{
  int m = -1;
  for (int64_t i = 0; i < size; i++) {
    if (idx[i] > m) {
      m = idx[i];
    }
  }
  return m;
}

static SpMat coo_to_eigen(const float *weight,
                         const int *row,
                         const int *col,
                         const int64_t nnz,
                         const int rows,
                         const int cols)
{
  SpMat M(std::max(rows, 0), std::max(cols, 0));
  if (rows <= 0 || cols <= 0 || nnz <= 0) {
    return M;
  }
  std::vector<Triplet> trips;
  trips.reserve(size_t(nnz));
  for (int64_t i = 0; i < nnz; i++) {
    const float w = weight[i];
    if (is_stored_zero(w)) {
      continue;
    }
    const int r = row[i];
    const int c = col[i];
    if (r >= 0 && r < rows && c >= 0 && c < cols) {
      trips.emplace_back(r, c, w);
    }
  }
  M.setFromTriplets(trips.begin(), trips.end());
  M.makeCompressed();
  return M;
}

static COOMatrix eigen_to_coo(const SpMat &matrix)
{
  /* Row-major walk so results stay sorted by (row, col) like the old COO path. */
  const SpMatRow row_major(matrix);
  COOMatrix out;
  out.weight.reserve(size_t(row_major.nonZeros()));
  out.row.reserve(size_t(row_major.nonZeros()));
  out.col.reserve(size_t(row_major.nonZeros()));
  for (int r = 0; r < row_major.outerSize(); r++) {
    for (SpMatRow::InnerIterator it(row_major, r); it; ++it) {
      const float w = it.value();
      if (is_stored_zero(w)) {
        continue;
      }
      out.weight.push_back(w);
      out.row.push_back(it.row());
      out.col.push_back(it.col());
    }
  }
  return out;
}

COOMatrix sparse_matrix_add(const float *a_w,
                            const int *a_r,
                            const int *a_c,
                            const int64_t a_size,
                            const float *b_w,
                            const int *b_r,
                            const int *b_c,
                            const int64_t b_size)
{
  const int rows = std::max(max_index(a_r, a_size), max_index(b_r, b_size)) + 1;
  const int cols = std::max(max_index(a_c, a_size), max_index(b_c, b_size)) + 1;
  if (rows <= 0 || cols <= 0) {
    return {};
  }
  const SpMat A = coo_to_eigen(a_w, a_r, a_c, a_size, rows, cols);
  const SpMat B = coo_to_eigen(b_w, b_r, b_c, b_size, rows, cols);
  return eigen_to_coo(A + B);
}

COOMatrix sparse_matrix_subtract(const float *a_w,
                                 const int *a_r,
                                 const int *a_c,
                                 const int64_t a_size,
                                 const float *b_w,
                                 const int *b_r,
                                 const int *b_c,
                                 const int64_t b_size)
{
  const int rows = std::max(max_index(a_r, a_size), max_index(b_r, b_size)) + 1;
  const int cols = std::max(max_index(a_c, a_size), max_index(b_c, b_size)) + 1;
  if (rows <= 0 || cols <= 0) {
    return {};
  }
  const SpMat A = coo_to_eigen(a_w, a_r, a_c, a_size, rows, cols);
  const SpMat B = coo_to_eigen(b_w, b_r, b_c, b_size, rows, cols);
  return eigen_to_coo(A - B);
}

COOMatrix sparse_matrix_multiply(const float *a_w,
                                 const int *a_r,
                                 const int *a_c,
                                 const int64_t a_size,
                                 const float *b_w,
                                 const int *b_r,
                                 const int *b_c,
                                 const int64_t b_size)
{
  if (a_size == 0 || b_size == 0) {
    return {};
  }
  const int a_rows = max_index(a_r, a_size) + 1;
  const int a_cols = max_index(a_c, a_size) + 1;
  const int b_rows = max_index(b_r, b_size) + 1;
  const int b_cols = max_index(b_c, b_size) + 1;
  const int inner = std::max(a_cols, b_rows);
  if (a_rows <= 0 || inner <= 0 || b_cols <= 0) {
    return {};
  }
  const SpMat A = coo_to_eigen(a_w, a_r, a_c, a_size, a_rows, inner);
  const SpMat B = coo_to_eigen(b_w, b_r, b_c, b_size, inner, b_cols);
  return eigen_to_coo(A * B);
}

COOMatrix sparse_matrix_scale(const float *a_w,
                              const int *a_r,
                              const int *a_c,
                              const int64_t a_size,
                              const float scale)
{
  if (a_size == 0 || is_stored_zero(scale)) {
    return {};
  }
  const int rows = max_index(a_r, a_size) + 1;
  const int cols = max_index(a_c, a_size) + 1;
  if (rows <= 0 || cols <= 0) {
    return {};
  }
  const SpMat A = coo_to_eigen(a_w, a_r, a_c, a_size, rows, cols);
  return eigen_to_coo(A * scale);
}

COOMatrix sparse_matrix_apply(const Operation op,
                              const float *a_w,
                              const int *a_r,
                              const int *a_c,
                              const int64_t a_size,
                              const float *b_w,
                              const int *b_r,
                              const int *b_c,
                              const int64_t b_size)
{
  switch (op) {
    case Operation::Add:
      return sparse_matrix_add(a_w, a_r, a_c, a_size, b_w, b_r, b_c, b_size);
    case Operation::Subtract:
      return sparse_matrix_subtract(a_w, a_r, a_c, a_size, b_w, b_r, b_c, b_size);
    case Operation::Multiply:
      return sparse_matrix_multiply(a_w, a_r, a_c, a_size, b_w, b_r, b_c, b_size);
    case Operation::Scale:
      break;
  }
  return {};
}

}  // namespace blender::nodes::sparse_matrix
