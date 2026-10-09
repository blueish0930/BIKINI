/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "linear_solver_math.hh"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>
#include <unsupported/Eigen/IterativeSolvers>

#include <Spectra/GenEigsRealShiftSolver.h>
#include <Spectra/GenEigsSolver.h>
#include <Spectra/MatOp/SparseGenMatProd.h>
#include <Spectra/MatOp/SparseGenRealShiftSolve.h>
#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/MatOp/SparseSymShiftSolve.h>
#include <Spectra/SymEigsShiftSolver.h>
#include <Spectra/SymEigsSolver.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace blender::geometry::linear_solver {

using SpMat = Eigen::SparseMatrix<double>;
using DenseMat = Eigen::MatrixXd;
using DenseVec = Eigen::VectorXd;

int infer_dimension(const CooMatrix &coo, const int vector_size)
{
  int n = std::max(0, vector_size);
  const int nnz = int(std::min({coo.weight.size(), coo.row.size(), coo.col.size()}));
  for (int i = 0; i < nnz; i++) {
    if (coo.row[i] >= 0) {
      n = std::max(n, coo.row[i] + 1);
    }
    if (coo.col[i] >= 0) {
      n = std::max(n, coo.col[i] + 1);
    }
  }
  return n;
}

void classify_bool_pins(const std::vector<char> &pin_mask,
                        const std::vector<double> &pin_values_src,
                        const int n,
                        std::vector<char> &is_pinned,
                        std::vector<double> &pin_values)
{
  is_pinned.assign(n, 0);
  pin_values.assign(n, 0.0);
  if (n <= 0) {
    return;
  }
  for (int i = 0; i < n; i++) {
    const bool pinned = (i < int(pin_mask.size())) && (pin_mask[i] != 0);
    if (pinned) {
      is_pinned[i] = 1;
      pin_values[i] = (i < int(pin_values_src.size())) ? pin_values_src[i] : 0.0;
    }
  }
}

std::vector<double> coo_to_dense(const CooMatrix &coo, const int n)
{
  std::vector<double> A(size_t(n) * size_t(n), 0.0);
  if (n <= 0) {
    return A;
  }
  const int nnz = int(std::min({coo.weight.size(), coo.row.size(), coo.col.size()}));
  for (int k = 0; k < nnz; k++) {
    const int r = coo.row[k];
    const int c = coo.col[k];
    if (r >= 0 && r < n && c >= 0 && c < n) {
      A[size_t(r) * size_t(n) + size_t(c)] += coo.weight[k];
    }
  }
  return A;
}

static SpMat coo_to_sparse(const CooMatrix &coo, const int n)
{
  SpMat A(n, n);
  const int nnz = int(std::min({coo.weight.size(), coo.row.size(), coo.col.size()}));
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(size_t(nnz));
  for (int k = 0; k < nnz; k++) {
    const int r = coo.row[k];
    const int c = coo.col[k];
    if (r >= 0 && r < n && c >= 0 && c < n) {
      trips.emplace_back(r, c, coo.weight[k]);
    }
  }
  A.setFromTriplets(trips.begin(), trips.end());
  A.makeCompressed();
  return A;
}

/**
 * Build free-block sparse system:
 *   A_free = A[free, free]
 *   b_free[i] = b[free_i] - sum_j A[free_i, pinned_j] * pin_values[j]
 * Optionally record all free-row couplings for later RHS updates.
 */
static void eliminate_pins_sparse(const SpMat &A,
                                  const DenseVec &b,
                                  const std::vector<char> &is_pinned,
                                  const std::vector<double> &pin_values,
                                  SpMat &A_free,
                                  DenseVec &b_free,
                                  std::vector<int> &free_map,
                                  std::vector<double> *A_row_values,
                                  std::vector<int> *A_row_i,
                                  std::vector<int> *A_row_j)
{
  const int n = int(A.rows());
  free_map.clear();
  free_map.reserve(n);
  std::vector<int> old_to_free(n, -1);
  for (int i = 0; i < n; i++) {
    if (!is_pinned[i]) {
      old_to_free[i] = int(free_map.size());
      free_map.push_back(i);
    }
  }
  const int nf = int(free_map.size());
  b_free = DenseVec::Zero(nf);
  if (A_row_values) {
    A_row_values->clear();
    A_row_i->clear();
    A_row_j->clear();
  }
  if (nf == 0) {
    A_free.resize(0, 0);
    return;
  }

  for (int ii = 0; ii < nf; ii++) {
    b_free[ii] = b[free_map[ii]];
  }

  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(size_t(A.nonZeros()));

  for (int j = 0; j < n; j++) {
    for (SpMat::InnerIterator it(A, j); it; ++it) {
      const int i = int(it.row());
      const double v = it.value();
      const int ii = old_to_free[i];
      if (ii < 0) {
        continue; /* pinned row dropped */
      }
      if (A_row_values) {
        A_row_values->push_back(v);
        A_row_i->push_back(ii);
        A_row_j->push_back(j);
      }
      if (is_pinned[j]) {
        b_free[ii] -= v * pin_values[j];
      }
      else {
        const int jj = old_to_free[j];
        trips.emplace_back(ii, jj, v);
      }
    }
  }

  A_free.resize(nf, nf);
  A_free.setFromTriplets(trips.begin(), trips.end());
  A_free.makeCompressed();
}

static DenseVec build_b_free_from_couplings(const Decomposition &decomp,
                                            const DenseVec &b,
                                            const std::vector<double> &pin_values)
{
  DenseVec b_free = DenseVec::Zero(decomp.n_free);
  for (int ii = 0; ii < decomp.n_free; ii++) {
    const int gi = decomp.free_map[ii];
    b_free[ii] = (gi < b.size()) ? b[gi] : 0.0;
  }
  const size_t nnz = decomp.A_row_values.size();
  for (size_t k = 0; k < nnz; k++) {
    const int ii = decomp.A_row_i[k];
    const int j = decomp.A_row_j[k];
    if (j >= 0 && j < decomp.n && decomp.is_pinned[j]) {
      b_free[ii] -= decomp.A_row_values[k] * pin_values[j];
    }
  }
  return b_free;
}

static DenseVec scatter_solution(const DenseVec &x_free,
                                 const std::vector<int> &free_map,
                                 const std::vector<char> &is_pinned,
                                 const std::vector<double> &pin_values,
                                 const int n)
{
  DenseVec x = DenseVec::Zero(n);
  for (int i = 0; i < n; i++) {
    if (is_pinned[i]) {
      x[i] = pin_values[i];
    }
  }
  for (int k = 0; k < int(free_map.size()); k++) {
    x[free_map[k]] = x_free[k];
  }
  return x;
}

template<typename Solver>
static bool solve_iterative(Solver &solver,
                            const SpMat &A,
                            const DenseVec &b,
                            DenseVec &x,
                            const SolveOptions &options,
                            std::string &message)
{
  solver.setMaxIterations(options.max_iterations);
  solver.setTolerance(options.tolerance);
  solver.compute(A);
  if (solver.info() != Eigen::Success) {
    message = "Iterative solver setup failed";
    return false;
  }
  x = solver.solve(b);
  if (solver.info() != Eigen::Success) {
    message = "Iterative solver did not converge";
    return false;
  }
  return true;
}

template<typename Precond>
static bool run_iterative_with_precond(const SpMat &A,
                                       const DenseVec &b,
                                       DenseVec &x,
                                       const SolveOptions &options,
                                       std::string &message)
{
  using IM = IterativeMethod;
  switch (options.iterative) {
    case IM::CG: {
      Eigen::ConjugateGradient<SpMat, Eigen::Lower | Eigen::Upper, Precond> solver;
      return solve_iterative(solver, A, b, x, options, message);
    }
    case IM::BiCGSTAB: {
      Eigen::BiCGSTAB<SpMat, Precond> solver;
      return solve_iterative(solver, A, b, x, options, message);
    }
    case IM::LSCG: {
      Eigen::LeastSquaresConjugateGradient<SpMat, Precond> solver;
      return solve_iterative(solver, A, b, x, options, message);
    }
    case IM::GMRES: {
      Eigen::GMRES<SpMat, Precond> solver;
      solver.set_restart(std::max(1, options.gmres_restart));
      return solve_iterative(solver, A, b, x, options, message);
    }
    case IM::DGMRES: {
      Eigen::DGMRES<SpMat, Precond> solver;
      return solve_iterative(solver, A, b, x, options, message);
    }
    case IM::MINRES: {
      Eigen::MINRES<SpMat, Eigen::Lower | Eigen::Upper, Precond> solver;
      return solve_iterative(solver, A, b, x, options, message);
    }
    case IM::IDRS: {
      Eigen::IDRS<SpMat, Precond> solver;
      return solve_iterative(solver, A, b, x, options, message);
    }
  }
  message = "Unknown iterative method";
  return false;
}

static bool solve_iterative_sparse(const SpMat &A_free,
                                   const DenseVec &b_free,
                                   DenseVec &x_free,
                                   const SolveOptions &options,
                                   std::string &message)
{
  if (A_free.rows() == 0) {
    x_free.resize(0);
    return true;
  }
  switch (options.preconditioner) {
    case Preconditioner::Identity:
      return run_iterative_with_precond<Eigen::IdentityPreconditioner>(
          A_free, b_free, x_free, options, message);
    case Preconditioner::Diagonal:
      return run_iterative_with_precond<Eigen::DiagonalPreconditioner<double>>(
          A_free, b_free, x_free, options, message);
    case Preconditioner::IncompleteLUT:
      return run_iterative_with_precond<Eigen::IncompleteLUT<double>>(
          A_free, b_free, x_free, options, message);
    case Preconditioner::IncompleteCholesky:
      return run_iterative_with_precond<Eigen::IncompleteCholesky<double>>(
          A_free, b_free, x_free, options, message);
  }
  message = "Unknown preconditioner";
  return false;
}

static bool solve_direct_sparse(const SpMat &A_free,
                                const DenseVec &b_free,
                                DenseVec &x_free,
                                const DirectMethod method,
                                std::string &message)
{
  if (A_free.rows() == 0) {
    x_free.resize(0);
    return true;
  }
  switch (method) {
    case DirectMethod::LLT: {
      Eigen::SimplicialLLT<SpMat> llt;
      llt.compute(A_free);
      if (llt.info() != Eigen::Success) {
        message = "Sparse LLT failed (matrix may not be SPD)";
        return false;
      }
      x_free = llt.solve(b_free);
      return llt.info() == Eigen::Success;
    }
    case DirectMethod::LDLT: {
      Eigen::SimplicialLDLT<SpMat> ldlt;
      ldlt.compute(A_free);
      if (ldlt.info() != Eigen::Success) {
        message = "Sparse LDLT failed";
        return false;
      }
      x_free = ldlt.solve(b_free);
      return ldlt.info() == Eigen::Success;
    }
    case DirectMethod::LU: {
      Eigen::SparseLU<SpMat> lu;
      lu.compute(A_free);
      if (lu.info() != Eigen::Success) {
        message = "Sparse LU failed";
        return false;
      }
      x_free = lu.solve(b_free);
      return lu.info() == Eigen::Success;
    }
    case DirectMethod::QR: {
      Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int>> qr;
      qr.compute(A_free);
      if (qr.info() != Eigen::Success) {
        message = "Sparse QR failed";
        return false;
      }
      x_free = qr.solve(b_free);
      return true;
    }
  }
  message = "Unknown direct method";
  return false;
}

struct DecompositionFactor {
  virtual ~DecompositionFactor() = default;
  /** Apply the factorization. Const and without shared state, so safe from several threads. */
  virtual bool solve(const DenseVec &b, DenseVec &x) const = 0;
};

template<typename Solver> struct DecompositionFactorImpl : public DecompositionFactor {
  Solver solver;

  bool solve(const DenseVec &b, DenseVec &x) const override
  {
    x = solver.solve(b);
    return solver.info() == Eigen::Success;
  }
};

template<typename Solver>
static std::shared_ptr<const DecompositionFactor> factorize_with(const SpMat &A_free,
                                                                 const char *failure,
                                                                 std::string &message)
{
  auto factor = std::make_shared<DecompositionFactorImpl<Solver>>();
  factor->solver.compute(A_free);
  if (factor->solver.info() != Eigen::Success) {
    message = failure;
    return nullptr;
  }
  return factor;
}

static std::shared_ptr<const DecompositionFactor> factorize_sparse(const SpMat &A_free,
                                                                   const DirectMethod method,
                                                                   std::string &message)
{
  switch (method) {
    case DirectMethod::LLT:
      return factorize_with<Eigen::SimplicialLLT<SpMat>>(
          A_free, "Sparse LLT failed (matrix may not be SPD)", message);
    case DirectMethod::LDLT:
      return factorize_with<Eigen::SimplicialLDLT<SpMat>>(A_free, "Sparse LDLT failed", message);
    case DirectMethod::LU:
      return factorize_with<Eigen::SparseLU<SpMat>>(A_free, "Sparse LU failed", message);
    case DirectMethod::QR:
      return factorize_with<Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int>>>(
          A_free, "Sparse QR failed", message);
  }
  message = "Unknown direct method";
  return nullptr;
}

static SolveResult solve_prepared_sparse(const SpMat &A,
                                         const DenseVec &b,
                                         const std::vector<char> &is_pinned,
                                         const std::vector<double> &pin_values,
                                         const SolveOptions &options)
{
  SolveResult result;
  const int n = int(A.rows());
  SpMat A_free;
  DenseVec b_free;
  std::vector<int> free_map;
  eliminate_pins_sparse(
      A, b, is_pinned, pin_values, A_free, b_free, free_map, nullptr, nullptr, nullptr);

  DenseVec x_free;
  bool ok = false;
  if (options.solver_class == SolverClass::Direct) {
    ok = solve_direct_sparse(A_free, b_free, x_free, options.direct, result.message);
  }
  else {
    ok = solve_iterative_sparse(A_free, b_free, x_free, options, result.message);
  }
  if (!ok) {
    return result;
  }
  const DenseVec x = scatter_solution(x_free, free_map, is_pinned, pin_values, n);
  result.success = true;
  result.x.assign(x.data(), x.data() + n);
  result.components = 1;
  return result;
}

/**
 * Factor free block once, then solve multiple RHS (vector components).
 * Pin mask is shared; pin values may differ per component.
 */
static bool factor_and_solve_multi_rhs(const SpMat &A,
                                       const std::vector<char> &is_pinned,
                                       const std::vector<DenseVec> &bs,
                                       const std::vector<std::vector<double>> &pin_vals_per_c,
                                       const SolveOptions &options,
                                       std::vector<DenseVec> &xs,
                                       std::string &message)
{
  const int n = int(A.rows());
  const int nc = int(bs.size());
  xs.resize(nc);
  if (nc == 0) {
    return true;
  }

  /* Free map from first component pin mask (shared). */
  std::vector<int> free_map;
  free_map.reserve(n);
  std::vector<int> old_to_free(n, -1);
  for (int i = 0; i < n; i++) {
    if (!is_pinned[i]) {
      old_to_free[i] = int(free_map.size());
      free_map.push_back(i);
    }
  }
  const int nf = int(free_map.size());

  /* Build A_free once (pin values only affect RHS). */
  SpMat A_free;
  {
    DenseVec dummy = DenseVec::Zero(n);
    DenseVec b_free_dummy;
    std::vector<double> zero_pins(n, 0.0);
    eliminate_pins_sparse(A,
                          dummy,
                          is_pinned,
                          zero_pins,
                          A_free,
                          b_free_dummy,
                          free_map,
                          nullptr,
                          nullptr,
                          nullptr);
  }

  if (nf == 0) {
    for (int c = 0; c < nc; c++) {
      xs[c] = scatter_solution(DenseVec(), free_map, is_pinned, pin_vals_per_c[c], n);
    }
    return true;
  }

  auto build_rhs = [&](const DenseVec &b, const std::vector<double> &pin_values) -> DenseVec {
    DenseVec b_free = DenseVec::Zero(nf);
    for (int ii = 0; ii < nf; ii++) {
      b_free[ii] = b[free_map[ii]];
    }
    for (int j = 0; j < n; j++) {
      if (!is_pinned[j]) {
        continue;
      }
      for (SpMat::InnerIterator it(A, j); it; ++it) {
        const int ii = old_to_free[int(it.row())];
        if (ii >= 0) {
          b_free[ii] -= it.value() * pin_values[j];
        }
      }
    }
    return b_free;
  };

  if (options.solver_class == SolverClass::Direct) {
    switch (options.direct) {
      case DirectMethod::LLT: {
        Eigen::SimplicialLLT<SpMat> solver;
        solver.compute(A_free);
        if (solver.info() != Eigen::Success) {
          message = "Sparse LLT failed (matrix may not be SPD)";
          return false;
        }
        for (int c = 0; c < nc; c++) {
          const DenseVec bf = build_rhs(bs[c], pin_vals_per_c[c]);
          const DenseVec xf = solver.solve(bf);
          xs[c] = scatter_solution(xf, free_map, is_pinned, pin_vals_per_c[c], n);
        }
        return true;
      }
      case DirectMethod::LDLT: {
        Eigen::SimplicialLDLT<SpMat> solver;
        solver.compute(A_free);
        if (solver.info() != Eigen::Success) {
          message = "Sparse LDLT failed";
          return false;
        }
        for (int c = 0; c < nc; c++) {
          const DenseVec bf = build_rhs(bs[c], pin_vals_per_c[c]);
          const DenseVec xf = solver.solve(bf);
          xs[c] = scatter_solution(xf, free_map, is_pinned, pin_vals_per_c[c], n);
        }
        return true;
      }
      case DirectMethod::LU: {
        Eigen::SparseLU<SpMat> solver;
        solver.compute(A_free);
        if (solver.info() != Eigen::Success) {
          message = "Sparse LU failed";
          return false;
        }
        for (int c = 0; c < nc; c++) {
          const DenseVec bf = build_rhs(bs[c], pin_vals_per_c[c]);
          const DenseVec xf = solver.solve(bf);
          xs[c] = scatter_solution(xf, free_map, is_pinned, pin_vals_per_c[c], n);
        }
        return true;
      }
      case DirectMethod::QR: {
        Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int>> solver;
        solver.compute(A_free);
        if (solver.info() != Eigen::Success) {
          message = "Sparse QR failed";
          return false;
        }
        for (int c = 0; c < nc; c++) {
          const DenseVec bf = build_rhs(bs[c], pin_vals_per_c[c]);
          const DenseVec xf = solver.solve(bf);
          xs[c] = scatter_solution(xf, free_map, is_pinned, pin_vals_per_c[c], n);
        }
        return true;
      }
    }
  }

  /* Iterative: recompute per RHS (preconditioner cheap relative to dense). */
  for (int c = 0; c < nc; c++) {
    const DenseVec bf = build_rhs(bs[c], pin_vals_per_c[c]);
    DenseVec xf;
    if (!solve_iterative_sparse(A_free, bf, xf, options, message)) {
      return false;
    }
    xs[c] = scatter_solution(xf, free_map, is_pinned, pin_vals_per_c[c], n);
  }
  return true;
}

SolveResult solve_system(const CooMatrix &coo,
                         const std::vector<double> &b_in,
                         const std::vector<char> &pin_mask,
                         const SolveOptions &options)
{
  SolveResult result;
  const int n = infer_dimension(coo, int(b_in.size()));
  if (n <= 0) {
    result.message = "Empty system";
    return result;
  }
  DenseVec b = DenseVec::Zero(n);
  for (int i = 0; i < n && i < int(b_in.size()); i++) {
    b[i] = b_in[i];
  }
  std::vector<char> is_pinned;
  std::vector<double> pin_values;
  classify_bool_pins(pin_mask, b_in, n, is_pinned, pin_values);

  const SpMat A = coo_to_sparse(coo, n);
  return solve_prepared_sparse(A, b, is_pinned, pin_values, options);
}

SolveResult solve_system_components(const CooMatrix &coo,
                                    const std::vector<double> &b_planar,
                                    const int components,
                                    const std::vector<char> &pin_mask,
                                    const SolveOptions &options)
{
  SolveResult result;
  if (components < 1) {
    result.message = "Invalid component count";
    return result;
  }
  const int n = infer_dimension(coo, int(b_planar.size()) / components);
  if (n <= 0) {
    result.message = "Empty system";
    return result;
  }
  result.x.assign(size_t(n) * size_t(components), 0.0);
  result.components = components;

  const SpMat A = coo_to_sparse(coo, n);

  /* Shared pin mask; pin values from each component of b. */
  std::vector<char> is_pinned(n, 0);
  for (int i = 0; i < n && i < int(pin_mask.size()); i++) {
    is_pinned[i] = pin_mask[i] ? 1 : 0;
  }

  std::vector<DenseVec> bs(components);
  std::vector<std::vector<double>> pin_vals(components);
  for (int c = 0; c < components; c++) {
    bs[c] = DenseVec::Zero(n);
    pin_vals[c].assign(n, 0.0);
    for (int i = 0; i < n; i++) {
      const int idx = c * n + i;
      if (idx < int(b_planar.size())) {
        bs[c][i] = b_planar[size_t(idx)];
        if (is_pinned[i]) {
          pin_vals[c][i] = b_planar[size_t(idx)];
        }
      }
    }
  }

  std::vector<DenseVec> xs;
  if (!factor_and_solve_multi_rhs(A, is_pinned, bs, pin_vals, options, xs, result.message)) {
    result.success = false;
    return result;
  }
  for (int c = 0; c < components; c++) {
    for (int i = 0; i < n; i++) {
      result.x[size_t(c) * size_t(n) + size_t(i)] = xs[c][i];
    }
  }
  result.success = true;
  return result;
}

Decomposition decompose_system(const CooMatrix &coo,
                               const std::vector<char> &pin_mask,
                               const std::vector<double> &pin_values_src,
                               const DirectMethod method)
{
  Decomposition decomp;
  decomp.method = method;
  const int dim = infer_dimension(coo, int(pin_mask.size()));
  if (dim <= 0) {
    decomp.message = "Empty system";
    return decomp;
  }
  classify_bool_pins(pin_mask, pin_values_src, dim, decomp.is_pinned, decomp.pin_values);

  const SpMat A = coo_to_sparse(coo, dim);
  SpMat A_free;
  DenseVec b_dummy = DenseVec::Zero(dim);
  DenseVec b_free;
  eliminate_pins_sparse(A,
                        b_dummy,
                        decomp.is_pinned,
                        decomp.pin_values,
                        A_free,
                        b_free,
                        decomp.free_map,
                        &decomp.A_row_values,
                        &decomp.A_row_i,
                        &decomp.A_row_j);

  decomp.n = dim;
  decomp.n_free = int(decomp.free_map.size());
  if (decomp.n_free == 0) {
    decomp.valid = true;
    decomp.message = "All DOFs pinned";
    return decomp;
  }

  /* Factor the free block once; Solve with Decomposition only applies it. */
  std::string factor_msg;
  decomp.factor = factorize_sparse(A_free, method, factor_msg);
  if (!decomp.factor) {
    decomp.message = factor_msg.empty() ? "Sparse factorization failed" : factor_msg;
    return decomp;
  }
  decomp.valid = true;
  return decomp;
}

SolveResult solve_with_decomposition(const Decomposition &decomp, const std::vector<double> &b_in)
{
  SolveResult result;
  if (!decomp.valid) {
    result.message = decomp.message.empty() ? "Invalid decomposition" : decomp.message;
    return result;
  }
  const int n = decomp.n;
  DenseVec b = DenseVec::Zero(n);
  for (int i = 0; i < n && i < int(b_in.size()); i++) {
    b[i] = b_in[i];
  }

  std::vector<double> pin_values = decomp.pin_values;
  for (int i = 0; i < n; i++) {
    if (decomp.is_pinned[i] && i < int(b_in.size())) {
      pin_values[i] = b_in[i];
    }
  }

  DenseVec b_free = build_b_free_from_couplings(decomp, b, pin_values);
  DenseVec x_free = DenseVec::Zero(decomp.n_free);

  if (decomp.n_free > 0) {
    if (!decomp.factor || !decomp.factor->solve(b_free, x_free)) {
      result.message = "Applying the stored factorization failed";
      return result;
    }
  }

  const DenseVec x = scatter_solution(x_free, decomp.free_map, decomp.is_pinned, pin_values, n);
  result.success = true;
  result.x.assign(x.data(), x.data() + n);
  return result;
}

/** Dirichlet pins as identity rows/cols on a sparse matrix (for eigen). */
static SpMat apply_pin_sparse(const SpMat &A_in, const std::vector<char> &is_pinned)
{
  const int n = int(A_in.rows());
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(size_t(A_in.nonZeros()) + size_t(n));
  for (int j = 0; j < n; j++) {
    if (is_pinned[j]) {
      continue;
    }
    for (SpMat::InnerIterator it(A_in, j); it; ++it) {
      const int i = int(it.row());
      if (!is_pinned[i]) {
        trips.emplace_back(i, j, it.value());
      }
    }
  }
  for (int i = 0; i < n; i++) {
    if (is_pinned[i]) {
      trips.emplace_back(i, i, 1.0);
    }
  }
  SpMat A(n, n);
  A.setFromTriplets(trips.begin(), trips.end());
  A.makeCompressed();
  return A;
}

static EigenResult extract_k_pairs(const DenseVec &evals,
                                   const DenseMat &evecs,
                                   const int k_want,
                                   const bool sort_by_magnitude)
{
  EigenResult result;
  const int n = int(evals.size());
  const int k = std::clamp(k_want > 0 ? k_want : n, 1, n);
  std::vector<int> order(n);
  for (int i = 0; i < n; i++) {
    order[i] = i;
  }
  if (sort_by_magnitude) {
    std::sort(order.begin(), order.end(), [&](const int a, const int b) {
      return std::abs(evals[a]) > std::abs(evals[b]);
    });
  }
  else {
    std::sort(order.begin(), order.end(), [&](const int a, const int b) {
      return evals[a] < evals[b];
    });
  }
  result.eigenvalues.resize(k);
  result.eigenvectors_flat.resize(size_t(n) * size_t(k));
  for (int j = 0; j < k; j++) {
    const int src = order[j];
    result.eigenvalues[j] = evals[src];
    for (int i = 0; i < n; i++) {
      result.eigenvectors_flat[size_t(j) * size_t(n) + size_t(i)] = evecs(i, src);
    }
  }
  result.success = true;
  return result;
}

/**
 * Pack Spectra output.
 * \param sort_near_shift When true (shift-invert solvers), reorder pairs so that
 *        |λ − σ| is ascending. Spectra's final sort is LargestAlge on λ, which
 *        scrambles "closest to shift first" — that made Num Eigenpairs appear to
 *        reshuffle results even though selection itself was shift-invert.
 */
static EigenResult pack_spectra_result(const DenseVec &evals,
                                       const DenseMat &evecs,
                                       const int k,
                                       const bool sort_near_shift,
                                       const double shift)
{
  EigenResult result;
  const int n = int(evecs.rows());
  const int kk = std::min(k, int(evals.size()));
  if (kk <= 0 || n <= 0) {
    result.message = "Empty eigen result";
    return result;
  }

  std::vector<int> order(kk);
  for (int j = 0; j < kk; j++) {
    order[j] = j;
  }
  if (sort_near_shift) {
    std::stable_sort(order.begin(), order.end(), [&](const int a, const int b) {
      const double da = std::abs(evals[a] - shift);
      const double db = std::abs(evals[b] - shift);
      if (da < db - 1e-15) {
        return true;
      }
      if (db < da - 1e-15) {
        return false;
      }
      /* Tie-break: smaller λ, then original index. */
      if (evals[a] != evals[b]) {
        return evals[a] < evals[b];
      }
      return a < b;
    });
  }

  result.eigenvalues.resize(kk);
  result.eigenvectors_flat.resize(size_t(n) * size_t(kk));
  for (int j = 0; j < kk; j++) {
    const int src = order[j];
    result.eigenvalues[j] = evals[src];
    /* Canonicalize sign: flip so the largest-magnitude entry is positive.
     * Eigenvectors are defined up to ±1; this keeps k=3 vs k=5 comparable. */
    int pivot = 0;
    double pivot_abs = 0.0;
    for (int i = 0; i < n; i++) {
      const double a = std::abs(evecs(i, src));
      if (a > pivot_abs) {
        pivot_abs = a;
        pivot = i;
      }
    }
    const double sign = (evecs(pivot, src) < 0.0) ? -1.0 : 1.0;
    for (int i = 0; i < n; i++) {
      result.eigenvectors_flat[size_t(j) * size_t(n) + size_t(i)] = sign * evecs(i, src);
    }
  }
  result.success = true;
  return result;
}

static EigenResult compute_eigenpairs_dense_fallback(const DenseMat &A,
                                                     const EigenOptions &options)
{
  const bool use_shift = options.kind == EigenSolverKind::GenEigsRealShiftSolver ||
                         options.kind == EigenSolverKind::SymEigsShiftSolver;
  const bool symmetric = options.kind == EigenSolverKind::SymEigsSolver ||
                         options.kind == EigenSolverKind::SymEigsShiftSolver;
  const int dim = int(A.rows());
  const int k_want = options.num_eigenpairs > 0 ? options.num_eigenpairs : std::min(dim, 6);

  if (symmetric) {
    Eigen::SelfAdjointEigenSolver<DenseMat> es(A);
    if (es.info() != Eigen::Success) {
      EigenResult r;
      r.message = "Dense SymEigs fallback failed";
      return r;
    }
    if (use_shift) {
      /* Pick k eigenvalues closest to σ (true shift-invert selection). */
      const DenseVec evals = es.eigenvalues();
      const DenseMat evecs = es.eigenvectors();
      std::vector<int> order(dim);
      for (int i = 0; i < dim; i++) {
        order[i] = i;
      }
      std::stable_sort(order.begin(), order.end(), [&](const int a, const int b) {
        return std::abs(evals[a] - options.shift) < std::abs(evals[b] - options.shift);
      });
      const int k = std::clamp(k_want, 1, dim);
      DenseVec evals_k(k);
      DenseMat evecs_k(dim, k);
      for (int j = 0; j < k; j++) {
        evals_k[j] = evals[order[j]];
        evecs_k.col(j) = evecs.col(order[j]);
      }
      return pack_spectra_result(evals_k, evecs_k, k, true, options.shift);
    }
    return extract_k_pairs(es.eigenvalues(), es.eigenvectors(), k_want, false);
  }
  Eigen::EigenSolver<DenseMat> es(A);
  if (es.info() != Eigen::Success) {
    EigenResult r;
    r.message = "Dense GenEigs fallback failed";
    return r;
  }
  DenseVec evals(dim);
  DenseMat evecs(dim, dim);
  for (int k = 0; k < dim; k++) {
    evals[k] = es.eigenvalues()[k].real();
    for (int i = 0; i < dim; i++) {
      evecs(i, k) = es.eigenvectors()(i, k).real();
    }
  }
  if (use_shift) {
    std::vector<int> order(dim);
    for (int i = 0; i < dim; i++) {
      order[i] = i;
    }
    std::stable_sort(order.begin(), order.end(), [&](const int a, const int b) {
      return std::abs(evals[a] - options.shift) < std::abs(evals[b] - options.shift);
    });
    const int k = std::clamp(k_want, 1, dim);
    DenseVec evals_k(k);
    DenseMat evecs_k(dim, k);
    for (int j = 0; j < k; j++) {
      evals_k[j] = evals[order[j]];
      evecs_k.col(j) = evecs.col(order[j]);
    }
    return pack_spectra_result(evals_k, evecs_k, k, true, options.shift);
  }
  return extract_k_pairs(evals, evecs, k_want, true);
}

/** Force symmetric storage so SparseSymShiftSolve (Lower) is correct. */
static SpMat symmetrize_sparse(const SpMat &A)
{
  SpMat At = A.transpose();
  SpMat S = 0.5 * (A + At);
  S.makeCompressed();
  return S;
}

EigenResult compute_eigenpairs(const CooMatrix &coo,
                               const std::vector<char> &pin_mask,
                               const std::vector<double> &pin_values_src,
                               const EigenOptions &options)
{
  EigenResult result;
  const int dim = infer_dimension(coo, int(pin_mask.size()));
  if (dim <= 0) {
    result.message = "Empty system";
    return result;
  }
  std::vector<char> is_pinned;
  std::vector<double> pin_values;
  classify_bool_pins(pin_mask, pin_values_src, dim, is_pinned, pin_values);

  SpMat As = apply_pin_sparse(coo_to_sparse(coo, dim), is_pinned);
  const bool symmetric = options.kind == EigenSolverKind::SymEigsSolver ||
                         options.kind == EigenSolverKind::SymEigsShiftSolver;
  if (symmetric) {
    As = symmetrize_sparse(As);
  }

  const int want = options.num_eigenpairs > 0 ? options.num_eigenpairs : std::min(dim, 6);
  /* Full spectrum: only allow dense for small systems. */
  constexpr int kDenseEigenMax = 512;
  auto to_dense = [&]() -> DenseMat {
    DenseMat A = DenseMat::Zero(dim, dim);
    for (int j = 0; j < dim; j++) {
      for (SpMat::InnerIterator it(As, j); it; ++it) {
        A(it.row(), j) = it.value();
      }
    }
    return A;
  };

  if (want >= dim || dim < 3) {
    if (dim > kDenseEigenMax) {
      result.message =
          "Full eigen spectrum requires dense O(n^3); reduce Num Eigenpairs or system size";
      return result;
    }
    return compute_eigenpairs_dense_fallback(to_dense(), options);
  }

  const int nev = std::clamp(want, 1, dim - 1);
  /* Generous Krylov space (Spectra: nev < ncv <= n). Slightly larger ncv improves
   * stability when comparing results across different nev. */
  const int ncv = std::min(dim, std::max({2 * nev + 1, nev + 10, std::min(dim, 2 * nev + 20)}));
  if (ncv <= nev) {
    result.message = "System too small for requested Num Eigenpairs";
    return result;
  }

  /* selection = LargestMagn: for shift-invert this means closest to σ (on ν = 1/(λ−σ)).
   * Final output order for shift solvers is re-sorted by |λ−σ| in pack_spectra_result. */
  constexpr int kMaxit = 2000;
  constexpr double kTol = 1e-12;

  try {
    switch (options.kind) {
      case EigenSolverKind::SymEigsSolver: {
        Spectra::SparseSymMatProd<double> op(As);
        Spectra::SymEigsSolver<Spectra::SparseSymMatProd<double>> eigs(op, nev, ncv);
        eigs.init();
        const int nconv = eigs.compute(
            Spectra::SortRule::LargestMagn, kMaxit, kTol, Spectra::SortRule::LargestAlge);
        if (eigs.info() != Spectra::CompInfo::Successful || nconv < 1) {
          result.message = "Spectra SymEigsSolver failed";
          return result;
        }
        return pack_spectra_result(
            eigs.eigenvalues(), eigs.eigenvectors(), nconv, false, 0.0);
      }
      case EigenSolverKind::GenEigsSolver: {
        Spectra::SparseGenMatProd<double> op(As);
        Spectra::GenEigsSolver<Spectra::SparseGenMatProd<double>> eigs(op, nev, ncv);
        eigs.init();
        const int nconv = eigs.compute(
            Spectra::SortRule::LargestMagn, kMaxit, kTol, Spectra::SortRule::LargestMagn);
        if (eigs.info() != Spectra::CompInfo::Successful || nconv < 1) {
          result.message = "Spectra GenEigsSolver failed";
          return result;
        }
        const auto evals_c = eigs.eigenvalues();
        const auto evecs_c = eigs.eigenvectors();
        DenseVec evals(nconv);
        DenseMat evecs(dim, nconv);
        for (int j = 0; j < nconv; j++) {
          evals[j] = evals_c[j].real();
          for (int i = 0; i < dim; i++) {
            evecs(i, j) = evecs_c(i, j).real();
          }
        }
        return pack_spectra_result(evals, evecs, nconv, false, 0.0);
      }
      case EigenSolverKind::SymEigsShiftSolver: {
        /* Shift-invert: y = (A−σI)^{-1} x via SparseLU; LargestMagn on ν ⇒ λ near σ. */
        Spectra::SparseSymShiftSolve<double> op(As);
        Spectra::SymEigsShiftSolver<Spectra::SparseSymShiftSolve<double>> eigs(
            op, nev, ncv, options.shift);
        eigs.init();
        const int nconv = eigs.compute(
            Spectra::SortRule::LargestMagn, kMaxit, kTol, Spectra::SortRule::LargestAlge);
        if (eigs.info() != Spectra::CompInfo::Successful || nconv < 1) {
          result.message = "Spectra SymEigsShiftSolver failed (check shift / SPD of A−σI)";
          return result;
        }
        return pack_spectra_result(
            eigs.eigenvalues(), eigs.eigenvectors(), nconv, true, options.shift);
      }
      case EigenSolverKind::GenEigsRealShiftSolver: {
        Spectra::SparseGenRealShiftSolve<double> op(As);
        Spectra::GenEigsRealShiftSolver<Spectra::SparseGenRealShiftSolve<double>> eigs(
            op, nev, ncv, options.shift);
        eigs.init();
        const int nconv = eigs.compute(
            Spectra::SortRule::LargestMagn, kMaxit, kTol, Spectra::SortRule::LargestAlge);
        if (eigs.info() != Spectra::CompInfo::Successful || nconv < 1) {
          result.message = "Spectra GenEigsRealShiftSolver failed";
          return result;
        }
        const auto evals_c = eigs.eigenvalues();
        const auto evecs_c = eigs.eigenvectors();
        DenseVec evals(nconv);
        DenseMat evecs(dim, nconv);
        for (int j = 0; j < nconv; j++) {
          evals[j] = evals_c[j].real();
          for (int i = 0; i < dim; i++) {
            evecs(i, j) = evecs_c(i, j).real();
          }
        }
        return pack_spectra_result(evals, evecs, nconv, true, options.shift);
      }
    }
  }
  catch (const std::exception &e) {
    result.message = std::string("Spectra exception: ") + e.what();
    if (dim <= kDenseEigenMax) {
      return compute_eigenpairs_dense_fallback(to_dense(), options);
    }
    return result;
  }

  result.message = "Unknown eigen solver kind";
  return result;
}

MultiplyResult multiply(const CooMatrix &coo, const std::vector<double> &x_in)
{
  MultiplyResult result;
  const int n = infer_dimension(coo, int(x_in.size()));
  if (n <= 0) {
    result.message = "Empty matrix or vector";
    return result;
  }
  DenseVec x = DenseVec::Zero(n);
  for (int i = 0; i < n && i < int(x_in.size()); i++) {
    x[i] = x_in[i];
  }
  const DenseVec y = coo_to_sparse(coo, n) * x;
  result.success = true;
  result.y.assign(y.data(), y.data() + n);
  result.components = 1;
  return result;
}

MultiplyResult multiply_components(const CooMatrix &coo,
                                   const std::vector<double> &x_planar,
                                   const int components)
{
  MultiplyResult result;
  if (components < 1) {
    result.message = "Invalid component count";
    return result;
  }
  const int n = infer_dimension(coo, int(x_planar.size()) / components);
  if (n <= 0) {
    result.message = "Empty matrix or vector";
    return result;
  }
  const SpMat A = coo_to_sparse(coo, n);
  result.y.assign(size_t(n) * size_t(components), 0.0);
  result.components = components;
  for (int c = 0; c < components; c++) {
    DenseVec x = DenseVec::Zero(n);
    for (int i = 0; i < n; i++) {
      const int idx = c * n + i;
      if (idx < int(x_planar.size())) {
        x[i] = x_planar[size_t(idx)];
      }
    }
    const DenseVec y = A * x;
    for (int i = 0; i < n; i++) {
      result.y[size_t(c) * size_t(n) + size_t(i)] = y[i];
    }
  }
  result.success = true;
  return result;
}

}  // namespace blender::geometry::linear_solver
