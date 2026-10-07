/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/**
 * Sparse matrix math over COO (coordinate) storage.
 *
 * Each matrix is three parallel arrays of equal length for nonzero entries only:
 *   weight[i], row[i], col[i]
 * Entries with value 0 are never stored in results.
 *
 * Add/subtract/multiply use Eigen compressed sparse matrices (same backend as Linear Solver).
 */

#include <cstdint>
#include <vector>

namespace blender::nodes::sparse_matrix {

struct COOMatrix {
  std::vector<float> weight;
  std::vector<int> row;
  std::vector<int> col;

  int64_t size() const
  {
    return int64_t(weight.size());
  }

  bool empty() const
  {
    return weight.empty();
  }

  void clear()
  {
    weight.clear();
    row.clear();
    col.clear();
  }
};

enum class Operation {
  Add = 0,
  Subtract = 1,
  Multiply = 2,
  Scale = 3,
  Transpose = 4,
};

inline bool is_stored_zero(const float value)
{
  return value == 0.0f;
}

inline bool lengths_equal(const int64_t w, const int64_t r, const int64_t c)
{
  return w == r && r == c;
}

COOMatrix sparse_matrix_add(const float *a_w,
                            const int *a_r,
                            const int *a_c,
                            int64_t a_size,
                            const float *b_w,
                            const int *b_r,
                            const int *b_c,
                            int64_t b_size);

COOMatrix sparse_matrix_subtract(const float *a_w,
                                 const int *a_r,
                                 const int *a_c,
                                 int64_t a_size,
                                 const float *b_w,
                                 const int *b_r,
                                 const int *b_c,
                                 int64_t b_size);

COOMatrix sparse_matrix_multiply(const float *a_w,
                                 const int *a_r,
                                 const int *a_c,
                                 int64_t a_size,
                                 const float *b_w,
                                 const int *b_r,
                                 const int *b_c,
                                 int64_t b_size);

COOMatrix sparse_matrix_scale(const float *a_w,
                              const int *a_r,
                              const int *a_c,
                              int64_t a_size,
                              float scale);

/** Swap rows and columns. Like the other operations the result is sorted by (row, col). */
COOMatrix sparse_matrix_transpose(const float *a_w, const int *a_r, const int *a_c, int64_t a_size);

COOMatrix sparse_matrix_apply(Operation op,
                              const float *a_w,
                              const int *a_r,
                              const int *a_c,
                              int64_t a_size,
                              const float *b_w,
                              const int *b_r,
                              const int *b_c,
                              int64_t b_size);

inline COOMatrix sparse_matrix_add(const std::vector<float> &a_w,
                                   const std::vector<int> &a_r,
                                   const std::vector<int> &a_c,
                                   const std::vector<float> &b_w,
                                   const std::vector<int> &b_r,
                                   const std::vector<int> &b_c)
{
  return sparse_matrix_add(a_w.data(),
                           a_r.data(),
                           a_c.data(),
                           int64_t(a_w.size()),
                           b_w.data(),
                           b_r.data(),
                           b_c.data(),
                           int64_t(b_w.size()));
}

inline COOMatrix sparse_matrix_subtract(const std::vector<float> &a_w,
                                        const std::vector<int> &a_r,
                                        const std::vector<int> &a_c,
                                        const std::vector<float> &b_w,
                                        const std::vector<int> &b_r,
                                        const std::vector<int> &b_c)
{
  return sparse_matrix_subtract(a_w.data(),
                                a_r.data(),
                                a_c.data(),
                                int64_t(a_w.size()),
                                b_w.data(),
                                b_r.data(),
                                b_c.data(),
                                int64_t(b_w.size()));
}

inline COOMatrix sparse_matrix_multiply(const std::vector<float> &a_w,
                                        const std::vector<int> &a_r,
                                        const std::vector<int> &a_c,
                                        const std::vector<float> &b_w,
                                        const std::vector<int> &b_r,
                                        const std::vector<int> &b_c)
{
  return sparse_matrix_multiply(a_w.data(),
                                a_r.data(),
                                a_c.data(),
                                int64_t(a_w.size()),
                                b_w.data(),
                                b_r.data(),
                                b_c.data(),
                                int64_t(b_w.size()));
}

inline COOMatrix sparse_matrix_scale(const std::vector<float> &a_w,
                                     const std::vector<int> &a_r,
                                     const std::vector<int> &a_c,
                                     const float scale)
{
  return sparse_matrix_scale(a_w.data(), a_r.data(), a_c.data(), int64_t(a_w.size()), scale);
}

inline COOMatrix sparse_matrix_transpose(const std::vector<float> &a_w,
                                         const std::vector<int> &a_r,
                                         const std::vector<int> &a_c)
{
  return sparse_matrix_transpose(a_w.data(), a_r.data(), a_c.data(), int64_t(a_w.size()));
}

}  // namespace blender::nodes::sparse_matrix
