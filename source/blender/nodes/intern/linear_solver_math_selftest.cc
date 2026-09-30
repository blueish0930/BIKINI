/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "../geometry/intern/linear_solver_math.hh"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace ls = blender::geometry::linear_solver;

static int g_failures = 0;

static void expect_true(const bool cond, const char *msg)
{
  if (!cond) {
    std::fprintf(stderr, "FAIL: %s\n", msg);
    g_failures++;
  }
  else {
    std::printf("OK: %s\n", msg);
  }
}

static void expect_near(const double a, const double b, const double tol, const char *msg)
{
  if (std::abs(a - b) > tol) {
    std::fprintf(stderr, "FAIL: %s (got %g expected %g)\n", msg, a, b);
    g_failures++;
  }
  else {
    std::printf("OK: %s\n", msg);
  }
}

static ls::CooMatrix make_spd2()
{
  ls::CooMatrix coo;
  coo.weight = {4, 1, 1, 3};
  coo.row = {0, 0, 1, 1};
  coo.col = {0, 1, 0, 1};
  return coo;
}

static double residual(const ls::CooMatrix &coo,
                       const std::vector<double> &x,
                       const std::vector<double> &b)
{
  const int n = int(b.size());
  std::vector<double> Ax(n, 0.0);
  const int nnz = int(std::min({coo.weight.size(), coo.row.size(), coo.col.size()}));
  for (int k = 0; k < nnz; k++) {
    const int r = coo.row[k];
    const int c = coo.col[k];
    if (r >= 0 && r < n && c >= 0 && c < n) {
      Ax[r] += coo.weight[k] * x[c];
    }
  }
  double r2 = 0.0;
  for (int i = 0; i < n; i++) {
    const double d = Ax[i] - b[i];
    r2 += d * d;
  }
  return std::sqrt(r2);
}

static void test_direct()
{
  const ls::CooMatrix A = make_spd2();
  const std::vector<double> b = {1.0, 2.0};
  const std::vector<char> pin;
  for (const ls::DirectMethod dm : {ls::DirectMethod::LLT,
                                    ls::DirectMethod::LDLT,
                                    ls::DirectMethod::LU,
                                    ls::DirectMethod::QR})
  {
    ls::SolveOptions opt;
    opt.solver_class = ls::SolverClass::Direct;
    opt.direct = dm;
    const ls::SolveResult r = ls::solve_system(A, b, pin, opt);
    const char *name = dm == ls::DirectMethod::LLT    ? "LLT" :
                       dm == ls::DirectMethod::LDLT   ? "LDLT" :
                       dm == ls::DirectMethod::LU     ? "LU" :
                                                        "QR";
    expect_true(r.success, (std::string("direct ") + name + " success").c_str());
    if (r.success) {
      expect_true(residual(A, r.x, b) < 1e-6, (std::string("direct ") + name + " residual").c_str());
    }
  }
}

static void test_iterative()
{
  const ls::CooMatrix A = make_spd2();
  const std::vector<double> b = {1.0, 2.0};
  const std::vector<char> pin;
  for (const ls::IterativeMethod im : {ls::IterativeMethod::CG,
                                       ls::IterativeMethod::BiCGSTAB,
                                       ls::IterativeMethod::GMRES,
                                       ls::IterativeMethod::MINRES,
                                       ls::IterativeMethod::LSCG,
                                       ls::IterativeMethod::DGMRES,
                                       ls::IterativeMethod::IDRS})
  {
    ls::SolveOptions opt;
    opt.solver_class = ls::SolverClass::Iterative;
    opt.iterative = im;
    opt.preconditioner = ls::Preconditioner::Diagonal;
    opt.max_iterations = 500;
    opt.tolerance = 1e-10;
    const ls::SolveResult r = ls::solve_system(A, b, pin, opt);
    const char *name = im == ls::IterativeMethod::CG         ? "CG" :
                       im == ls::IterativeMethod::BiCGSTAB   ? "BiCGSTAB" :
                       im == ls::IterativeMethod::GMRES      ? "GMRES" :
                       im == ls::IterativeMethod::MINRES     ? "MINRES" :
                       im == ls::IterativeMethod::LSCG       ? "LSCG" :
                       im == ls::IterativeMethod::DGMRES     ? "DGMRES" :
                                                               "IDRS";
    expect_true(r.success, (std::string("iter ") + name + " success").c_str());
    if (r.success) {
      expect_true(residual(A, r.x, b) < 1e-5, (std::string("iter ") + name + " residual").c_str());
    }
  }
}

static void test_pin_bool()
{
  ls::CooMatrix A;
  A.weight = {1, 1, 1};
  A.row = {0, 1, 2};
  A.col = {0, 1, 2};
  const std::vector<double> b = {5.0, 2.0, 3.0};
  const std::vector<char> pin = {1, 0, 0}; /* pin DOF0 to b[0]=5 */
  ls::SolveOptions opt;
  opt.solver_class = ls::SolverClass::Direct;
  opt.direct = ls::DirectMethod::LLT;
  const ls::SolveResult r = ls::solve_system(A, b, pin, opt);
  expect_true(r.success, "pin success");
  if (r.success && r.x.size() == 3) {
    expect_near(r.x[0], 5.0, 1e-9, "pinned x0 from b");
    expect_near(r.x[1], 2.0, 1e-9, "free x1");
    expect_near(r.x[2], 3.0, 1e-9, "free x2");
  }
}

/** Diagonal 4x4 — large enough for Spectra partial path (nev < n). */
static ls::CooMatrix make_diag4()
{
  ls::CooMatrix coo;
  coo.weight = {1, 2, 3, 4};
  coo.row = {0, 1, 2, 3};
  coo.col = {0, 1, 2, 3};
  return coo;
}

static void test_eigen()
{
  /* Small full spectrum (dense fallback). */
  ls::CooMatrix A2;
  A2.weight = {2, 3};
  A2.row = {0, 1};
  A2.col = {0, 1};
  ls::EigenOptions eopt;
  eopt.kind = ls::EigenSolverKind::SymEigsSolver;
  eopt.num_eigenpairs = 2;
  const ls::EigenResult er = ls::compute_eigenpairs(A2, {}, {}, eopt);
  expect_true(er.success, "sym eigen small");
  expect_true(er.eigenvalues.size() == 2, "k=2");
  expect_true(er.eigenvectors_flat.size() == 4, "flat n*k");
  if (er.success) {
    expect_near(er.eigenvalues[0], 2.0, 1e-8, "lambda0");
    expect_near(er.eigenvalues[1], 3.0, 1e-8, "lambda1");
  }

  /* Spectra partial path on 4x4. */
  const ls::CooMatrix A4 = make_diag4();
  eopt.kind = ls::EigenSolverKind::SymEigsSolver;
  eopt.num_eigenpairs = 2;
  const ls::EigenResult er_s = ls::compute_eigenpairs(A4, {}, {}, eopt);
  expect_true(er_s.success, "spectra sym partial");
  expect_true(er_s.eigenvalues.size() == 2, "spectra k=2");
  expect_true(er_s.eigenvectors_flat.size() == 8, "spectra flat 4*2");

  eopt.kind = ls::EigenSolverKind::GenEigsSolver;
  eopt.num_eigenpairs = 2;
  const ls::EigenResult er_g = ls::compute_eigenpairs(A4, {}, {}, eopt);
  expect_true(er_g.success, "spectra gen partial");

  eopt.kind = ls::EigenSolverKind::SymEigsShiftSolver;
  eopt.shift = 2.5;
  eopt.num_eigenpairs = 1;
  const ls::EigenResult er2 = ls::compute_eigenpairs(A4, {}, {}, eopt);
  expect_true(er2.success, "spectra sym shift");
  expect_true(er2.eigenvalues.size() == 1, "k=1 shift");

  eopt.kind = ls::EigenSolverKind::GenEigsRealShiftSolver;
  eopt.shift = 1.5;
  eopt.num_eigenpairs = 1;
  const ls::EigenResult er3 = ls::compute_eigenpairs(A4, {}, {}, eopt);
  expect_true(er3.success, "spectra gen real shift");
}

static void test_decomp_lu()
{
  const ls::CooMatrix A = make_spd2();
  const std::vector<double> b = {1.0, 2.0};
  const ls::Decomposition d = ls::decompose_system(A, {}, {}, ls::DirectMethod::LU);
  expect_true(d.valid, "LU decomp");
  const ls::SolveResult via = ls::solve_with_decomposition(d, A, b);
  expect_true(via.success, "LU solve via decomp");
  if (via.success) {
    expect_true(residual(A, via.x, b) < 1e-6, "LU via residual");
  }
}

static void test_multiply()
{
  ls::CooMatrix A;
  A.weight = {1, 2, 3, 4};
  A.row = {0, 0, 1, 1};
  A.col = {0, 1, 0, 1};
  const ls::MultiplyResult mr = ls::multiply(A, {1.0, 1.0});
  expect_true(mr.success, "mul");
  if (mr.success) {
    expect_near(mr.y[0], 3.0, 1e-9, "y0");
    expect_near(mr.y[1], 7.0, 1e-9, "y1");
  }
}

int main()
{
  test_direct();
  test_iterative();
  test_pin_bool();
  test_eigen();
  test_decomp_lu();
  test_multiply();
  if (g_failures != 0) {
    std::fprintf(stderr, "\n%d failure(s)\n", g_failures);
    return 1;
  }
  std::printf("\nAll linear_solver_math tests passed.\n");
  return 0;
}
