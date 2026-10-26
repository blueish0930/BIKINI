/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Standalone numerical acceptance driver, intentionally not part of normal
 * builds. Compile manually with the geometry/nodes and Eigen include
 * directories. Do not use NDEBUG: assertions are acceptance checks. */
#include "bijective_uv_solver.hh"
#include <cassert>
#include <iostream>

namespace uv = blender::geometry::bijective_uv;

int main()
{
  std::vector<uv::V3> positions;
  std::vector<std::array<int, 3>> triangles;
  constexpr int n = 8;
  for (int j = 0; j < n; j++) {
    for (int i = 0; i < n; i++) {
      const double x = double(i) / (n - 1), y = double(j) / (n - 1);
      positions.emplace_back(x, y, .3 * std::sin(3 * x) * std::sin(3 * y));
      if (i < n - 1 && j < n - 1) {
        const int a = j * n + i;
        triangles.push_back({a, a + 1, a + n + 1});
        triangles.push_back({a, a + n + 1, a + n});
      }
    }
  }
  uv::Options options;
  options.epsilon = .24; /* Exercise active boundary contacts. */
  options.iterations = 100;
  uv::Solver solver(positions, triangles, options);
  uv::Vec initial(2 * n * n);
  for (int i = 0; i < n * n; i++) {
    initial.segment<2>(2 * i) = positions[i].head<2>();
  }
  uv::Vec gradient;
  double ed, eb;
  const double before = solver.evaluate(initial, gradient, ed, eb);
  assert(eb > 0 && std::isfinite(before));
  uv::Vec numerical(initial.size());
  for (int i = 0; i < initial.size(); i++) {
    uv::Vec plus = initial, minus = initial, scratch;
    constexpr double h = 1e-6;
    plus[i] += h;
    minus[i] -= h;
    double a, b;
    numerical[i] =
        (solver.evaluate(plus, scratch, a, b) - solver.evaluate(minus, scratch, a, b)) / (2 * h);
  }
  const double error = (gradient - numerical).norm() / std::max(1., numerical.norm());
  assert(error < 1e-5);
  for (int method = 0; method < 3; method++) {
    options.method = method;
    uv::Solver candidate(positions, triangles, options);
    const uv::Result result = candidate.solve(initial);
    assert(result.iterations > 0 && result.energy < before);
    assert(candidate.boundary_valid(result.uv));
    assert(std::isfinite(candidate.evaluate(result.uv, gradient, ed, eb)));
    std::cout << "method=" << method << " iterations=" << result.iterations
              << " energy=" << result.energy << " status=" << result.status << '\n';
  }
  const uv::Vec tutte = solver.initialize();
  assert(solver.boundary_valid(tutte));
  assert(std::isfinite(solver.evaluate(tutte, gradient, ed, eb)));
  uv::Vec bad = initial;
  bad.segment<2>(0) = bad.segment<2>(2);
  bool rejected = false;
  try {
    solver.solve(bad);
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  assert(rejected);
  /* A linear trajectory must remain valid up to the continuous collision cap.
   */
  uv::Vec direction = uv::Vec::Zero(initial.size());
  direction.segment<2>(2 * (n + 1)) = uv::V2(-3, -2);
  const double cap = solver.step_cap(initial, direction);
  assert(cap > 0 && cap < 1);
  assert(std::isfinite(solver.evaluate(initial + cap * direction, gradient, ed, eb)));
  std::cout << "gradient_relative_error=" << error << " numerical acceptance passed\n";
}
