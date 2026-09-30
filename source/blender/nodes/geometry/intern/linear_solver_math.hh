/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * Eigen sparse linear-algebra helpers used by GeometryNodeLinearSolver.
 *
 * All solve / factor paths stay sparse (COO → Eigen::SparseMatrix). Dense
 * eigen fallbacks are only used for full-spectrum requests on tiny systems.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace blender::geometry::linear_solver {

/** Houdini-style: direct vs iterative family. */
enum class SolverClass : int8_t {
  Direct = 0,
  Iterative = 1,
};

enum class DirectMethod : int8_t {
  LLT = 0,
  LDLT = 1,
  LU = 2,
  QR = 3,
};

enum class IterativeMethod : int8_t {
  GMRES = 0,
  DGMRES = 1,
  MINRES = 2,
  CG = 3,
  LSCG = 4,
  BiCGSTAB = 5,
  IDRS = 6,
};

enum class Preconditioner : int8_t {
  Identity = 0,
  Diagonal = 1,
  IncompleteLUT = 2,
  IncompleteCholesky = 3,
};

/** Spectra eigen backends (sparse mat-vec / shift-invert). */
enum class EigenSolverKind : int8_t {
  GenEigsSolver = 0,
  SymEigsSolver = 1,
  GenEigsRealShiftSolver = 2,
  SymEigsShiftSolver = 3,
};

struct CooMatrix {
  std::vector<double> weight;
  std::vector<int> row;
  std::vector<int> col;
};

struct SolveOptions {
  SolverClass solver_class = SolverClass::Direct;
  DirectMethod direct = DirectMethod::LLT;
  IterativeMethod iterative = IterativeMethod::CG;
  Preconditioner preconditioner = Preconditioner::Diagonal;
  int max_iterations = 1000;
  double tolerance = 1e-10;
  int gmres_restart = 30;
};

struct EigenOptions {
  EigenSolverKind kind = EigenSolverKind::SymEigsSolver;
  /** Number of eigenpairs to return (clamped to 1..n). */
  int num_eigenpairs = 0;
  /** Shift σ for shift / shift-invert solvers. */
  double shift = 0.0;
  /**
   * When true (default for shift solvers), use shift-invert on (A−σI)⁻¹ then map
   * eigenvalues back to A. When false, compute eigen of (A−σI) directly.
   */
  bool shift_invert = true;
};

struct SolveResult {
  bool success = false;
  std::string message;
  /** Length n (float system) or planar n*components when components > 1. */
  std::vector<double> x;
  int components = 1;
};

struct EigenResult {
  bool success = false;
  std::string message;
  /** Flattened: first n of vector 0, next n of vector 1, ... (length n * k). */
  std::vector<double> eigenvectors_flat;
  std::vector<double> eigenvalues;
};

struct MultiplyResult {
  bool success = false;
  std::string message;
  std::vector<double> y;
  int components = 1;
};

/**
 * Sparse free-block system for Solve-with-Decomposition.
 *
 * Stores the free submatrix A_free (CSC) plus pin bookkeeping. Apply uses a
 * sparse direct factorization (SimplicialLLT/LDLT, SparseLU, SparseQR) — still
 * O(nnz) class work, not dense O(n³).
 */
struct Decomposition {
  bool valid = false;
  DirectMethod method = DirectMethod::LLT;
  int n = 0;
  int n_free = 0;
  std::string message;

  /** Free-block A_free in CSC (column-compressed). */
  std::vector<double> A_values;
  std::vector<int> A_inner;
  std::vector<int> A_outer;

  std::vector<int> free_map;
  std::vector<char> is_pinned;
  std::vector<double> pin_values;

  /**
   * Free-row couplings into original A (COO triples) to rebuild RHS when pin
   * Dirichlet values change: b_free[i] -= A_ij * pin[j] for pinned j.
   */
  std::vector<double> A_row_values;
  std::vector<int> A_row_i; /* free-local row 0..n_free-1 */
  std::vector<int> A_row_j; /* global column 0..n-1 */
};

int infer_dimension(const CooMatrix &coo, int vector_size);

/**
 * Pin is a boolean mask of length n.
 * Pinned DOF i is fixed to pin_values[i] (typically taken from b[i]).
 */
void classify_bool_pins(const std::vector<char> &pin_mask,
                        const std::vector<double> &pin_values_src,
                        int n,
                        std::vector<char> &is_pinned,
                        std::vector<double> &pin_values);

/** Debug / tests: dense n×n (row-major flat). Avoid in hot paths. */
std::vector<double> coo_to_dense(const CooMatrix &coo, int n);

/** Solve Ax=b with optional bool pin (Dirichlet values from b on pinned DOFs). */
SolveResult solve_system(const CooMatrix &coo,
                         const std::vector<double> &b,
                         const std::vector<char> &pin_mask,
                         const SolveOptions &options);

/**
 * Multi-component: planar layout (all x then all y...). A is n×n applied
 * independently per component; the free-block factorization is built once.
 */
SolveResult solve_system_components(const CooMatrix &coo,
                                    const std::vector<double> &b_planar,
                                    int components,
                                    const std::vector<char> &pin_mask,
                                    const SolveOptions &options);

Decomposition decompose_system(const CooMatrix &coo,
                               const std::vector<char> &pin_mask,
                               const std::vector<double> &pin_values_src,
                               DirectMethod method);

SolveResult solve_with_decomposition(const Decomposition &decomp,
                                     const CooMatrix &coo,
                                     const std::vector<double> &b);

EigenResult compute_eigenpairs(const CooMatrix &coo,
                               const std::vector<char> &pin_mask,
                               const std::vector<double> &pin_values_src,
                               const EigenOptions &options);

MultiplyResult multiply(const CooMatrix &coo, const std::vector<double> &x);
MultiplyResult multiply_components(const CooMatrix &coo,
                                   const std::vector<double> &x_planar,
                                   int components);

}  // namespace blender::geometry::linear_solver
