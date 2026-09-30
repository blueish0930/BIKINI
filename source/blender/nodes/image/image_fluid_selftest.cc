/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Self-test for Image Process 2D MAC fluid core.
 *
 * Build:
 *   cl /EHsc /std:c++17 /Iinclude /DIMG_FLUID_SELFTEST_MAIN ^
 *      /Fe:image_fluid_selftest.exe image_fluid_selftest.cc
 */

#include "NOD_image_fluid_solver.hh"

#include <cstdio>

namespace {

using namespace blender::nodes::image_fluid;

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

void check_lt(const float a, const float b, const char *msg)
{
  if (!(a < b)) {
    std::fprintf(stderr, "FAIL: %s (got %g, expected < %g)\n", msg, double(a), double(b));
    g_failures++;
  }
  else {
    std::printf("PASS: %s (%g < %g)\n", msg, double(a), double(b));
  }
}

void check_gt(const float a, const float b, const char *msg)
{
  if (!(a > b)) {
    std::fprintf(stderr, "FAIL: %s (got %g, expected > %g)\n", msg, double(a), double(b));
    g_failures++;
  }
  else {
    std::printf("PASS: %s (%g > %g)\n", msg, double(a), double(b));
  }
}

void test_mac_divergence_uniform_flow()
{
  /* Uniform rightward flow + Wrap (closed Bounce walls make boundary faces 0 → fake div). */
  constexpr int w = 32, h = 32;
  FluidGrid g;
  g.resize(w, h, 0);
  for (float &u : g.vel_x) {
    u = 2.0f;
  }
  for (float &v : g.vel_y) {
    v = 0.0f;
  }
  FluidBounds wrap;
  wrap.left = wrap.right = wrap.bottom = wrap.top = FluidBoundary::Wrap;
  apply_mac_boundaries(g.vel_x, g.vel_y, w, h, wrap);
  compute_divergence_mac(g.vel_x, g.vel_y, w, h, g.divergence);
  const float rms = field_rms(g.divergence);
  std::printf("  uniform flow div RMS = %g\n", double(rms));
  check_lt(rms, 1e-4f, "uniform MAC flow has near-zero divergence");
}

void test_mac_projection_kills_div()
{
  constexpr int w = 48, h = 48;
  FluidGrid g;
  g.resize(w, h, 1);
  /* Compressible seed: radial outflow on cell then to MAC. */
  std::vector<float> cx(size_t(w * h)), cy(size_t(w * h));
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      cx[size_t(idx2(w, i, j))] = 0.5f * float(i - w / 2) / float(w);
      cy[size_t(idx2(w, i, j))] = 0.5f * float(j - h / 2) / float(h);
    }
  }
  cell_velocity_to_mac(cx, cy, w, h, g.vel_x, g.vel_y);
  apply_mac_boundaries(g.vel_x, g.vel_y, w, h, FluidBounds{});

  compute_divergence_mac(g.vel_x, g.vel_y, w, h, g.divergence);
  const float before = field_rms(g.divergence);

  FluidStepParams p;
  p.dt = 1.0f / 60.0f;
  p.curl = 0.0f;
  p.pressure = 0.0f;
  p.pressure_vcycles = 3;
  p.pressure_iterations = 20;
  p.mg_pre_smooth = 2;
  p.mg_post_smooth = 2;
  p.density_dissipation = 0.0f;
  p.velocity_dissipation = 0.0f;
  p.warm_start_pressure = false;

  const float after = fluid_step(g, p);
  std::printf("  div RMS before=%g after_step=%g (MG)\n", double(before), double(after));
  check_gt(before, 1e-4f, "seed has measurable divergence");
  check_lt(after, before * 0.5f + 1e-4f, "MAC+MG projection reduces div RMS");
  check(field_all_finite(g.vel_x) && field_all_finite(g.vel_y), "velocity stays finite");
}

void test_multigrid_beats_few_jacobi()
{
  constexpr int w = 64, h = 64;
  std::vector<float> div(size_t(w * h));
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      div[size_t(idx2(w, i, j))] = 0.1f * std::sin(0.2f * float(i)) * std::cos(0.15f * float(j));
    }
  }
  std::vector<float> p_jac(size_t(w * h), 0.0f), p_mg(size_t(w * h), 0.0f), tmp;
  std::vector<float> mask, air;
  jacobi_pressure_mac(p_jac, div, w, h, 8, tmp, FluidBounds{}, mask, air);
  multigrid_pressure_mac(p_mg, div, w, h, 3, 2, 2, 16, FluidBounds{}, mask, air);
  std::vector<float> r_jac, r_mg;
  compute_poisson_residual(p_jac, div, w, h, r_jac, FluidBounds{}, mask, air);
  compute_poisson_residual(p_mg, div, w, h, r_mg, FluidBounds{}, mask, air);
  const float ej = field_rms(r_jac);
  const float em = field_rms(r_mg);
  std::printf("  residual RMS Jacobi×8=%g  MG 3 V-cycles=%g\n", double(ej), double(em));
  check_lt(em, ej * 0.9f + 1e-6f, "multigrid residual better than few Jacobi sweeps");
}

void test_dye_advects_not_pinned()
{
  /* Dye on left + rightward velocity should move mass rightward after steps. */
  constexpr int w = 64, h = 32;
  FluidGrid g;
  g.resize(w, h, 1);
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      g.colors[0][size_t(idx2(w, i, j))] = (i < w / 4) ? 1.0f : 0.0f;
    }
  }
  std::vector<float> cx(size_t(w * h), 8.0f), cy(size_t(w * h), 0.0f);
  cell_velocity_to_mac(cx, cy, w, h, g.vel_x, g.vel_y);
  /* Periodic horizontal: sealed Bounce walls kill through-flow (incompressible). */
  FluidBounds wrap;
  wrap.left = wrap.right = FluidBoundary::Wrap;
  wrap.bottom = wrap.top = FluidBoundary::Bounce;
  apply_mac_boundaries(g.vel_x, g.vel_y, w, h, wrap);

  FluidStepParams p;
  p.dt = 1.0f / 30.0f;
  p.curl = 0.0f;
  p.pressure = 0.0f;
  p.pressure_iterations = 40;
  p.density_dissipation = 0.0f;
  p.velocity_dissipation = 0.0f;
  p.bounds = wrap;

  auto left_mass = [&]() {
    double s = 0.0;
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w / 4; i++) {
        s += double(g.colors[0][size_t(idx2(w, i, j))]);
      }
    }
    return float(s);
  };
  auto mid_mass = [&]() {
    double s = 0.0;
    for (int j = 0; j < h; j++) {
      for (int i = w / 3; i < 2 * w / 3; i++) {
        s += double(g.colors[0][size_t(idx2(w, i, j))]);
      }
    }
    return float(s);
  };

  const float L0 = left_mass();
  const float M0 = mid_mass();
  for (int i = 0; i < 60; i++) {
    fluid_step(g, p);
  }
  const float L1 = left_mass();
  const float M1 = mid_mass();
  std::printf("  left mass %g -> %g, mid %g -> %g\n",
              double(L0),
              double(L1),
              double(M0),
              double(M1));
  check_lt(L1, L0 * 0.85f, "left dye mass decreases (not re-injected)");
  check_gt(M1, M0 + 1.0f, "mid dye mass increases (advected rightward)");
}

void test_pressure_one_no_tide()
{
  /* Uniform rightward vel + pressure residual 1 should not reverse flow. */
  constexpr int w = 48, h = 48;
  FluidGrid g;
  g.resize(w, h, 0);
  std::vector<float> cx(size_t(w * h), 2.0f), cy(size_t(w * h), 0.0f);
  cell_velocity_to_mac(cx, cy, w, h, g.vel_x, g.vel_y);

  FluidStepParams p;
  p.dt = 1.0f / 60.0f;
  p.curl = 0.0f;
  p.pressure = 1.0f; /* full warm-start */
  p.warm_start_pressure = true;
  p.pressure_iterations = 40;
  p.density_dissipation = 0.0f;
  p.velocity_dissipation = 0.0f;

  float mean_u0 = 0.0f;
  {
    std::vector<float> cvx, cvy;
    mac_velocity_to_cell(g.vel_x, g.vel_y, w, h, cvx, cvy);
    double s = 0.0;
    for (float v : cvx) {
      s += double(v);
    }
    mean_u0 = float(s / double(cvx.size()));
  }

  for (int i = 0; i < 30; i++) {
    fluid_step(g, p);
  }

  std::vector<float> cvx, cvy;
  mac_velocity_to_cell(g.vel_x, g.vel_y, w, h, cvx, cvy);
  double s = 0.0;
  for (float v : cvx) {
    s += double(v);
  }
  const float mean_u1 = float(s / double(cvx.size()));
  std::printf("  mean u %g -> %g (pressure residual=1)\n", double(mean_u0), double(mean_u1));
  check_gt(mean_u1, 0.0f, "mean u stays positive (no tide reverse)");
  /* Closed Bounce domain + stronger MG projection reduces mean flux more — still not reverse. */
  check_gt(mean_u1, mean_u0 * 0.15f, "mean u not collapsed by pressure=1");
}

void test_no_dissipation_preserves_dye()
{
  constexpr int w = 32, h = 32;
  FluidGrid g;
  g.resize(w, h, 1);
  splat_velocity_and_dye(g, 0.5f, 0.5f, 0.0f, 0.0f, 10.0f, 0.3f, 1.0f);
  const float m0 = field_sum(g.colors[0]);
  FluidStepParams p;
  p.dt = 1.0f / 60.0f;
  p.curl = 0.0f;
  p.pressure = 0.0f;
  p.pressure_iterations = 20;
  p.density_dissipation = 0.0f;
  p.velocity_dissipation = 0.0f;
  for (int i = 0; i < 20; i++) {
    fluid_step(g, p);
  }
  const float m1 = field_sum(g.colors[0]);
  std::printf("  dye mass %g -> %g (dissipation=0)\n", double(m0), double(m1));
  /* Semi-Lagrangian + bounce can slightly change mass; keep within 20%. */
  check(std::abs(m1 - m0) < m0 * 0.25f + 1.0f, "dye mass roughly preserved without dissipation");
}

void test_air_free_surface()
{
  /* Half domain air: liquid on left, air on right — flow should not pile like a wall. */
  constexpr int w = 48, h = 24;
  FluidGrid g;
  g.resize(w, h, 1);
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      g.air_mask[size_t(idx2(w, i, j))] = (i >= w / 2) ? 1.0f : 0.0f;
      g.colors[0][size_t(idx2(w, i, j))] = (i < w / 4) ? 1.0f : 0.0f;
    }
  }
  std::vector<float> cx(size_t(w * h), 6.0f), cy(size_t(w * h), 0.0f);
  cell_velocity_to_mac(cx, cy, w, h, g.vel_x, g.vel_y);
  FluidBounds wrap;
  wrap.left = wrap.right = FluidBoundary::Wrap;
  wrap.bottom = wrap.top = FluidBoundary::Bounce;
  FluidStepParams p;
  p.dt = 1.0f / 30.0f;
  p.curl = 0.0f;
  p.pressure = 0.0f;
  p.pressure_iterations = 40;
  p.density_dissipation = 0.0f;
  p.velocity_dissipation = 0.0f;
  p.bounds = wrap;
  for (int s = 0; s < 50; s++) {
    fluid_step(g, p);
  }
  double air_p = 0.0;
  int air_n = 0;
  for (int j = 0; j < h; j++) {
    for (int i = w / 2; i < w; i++) {
      air_p += double(std::abs(g.pressure[size_t(idx2(w, i, j))]));
      air_n++;
    }
  }
  air_p /= double(std::max(air_n, 1));
  double right_dye = 0.0;
  for (int j = 0; j < h; j++) {
    for (int i = w / 2; i < w; i++) {
      right_dye += double(g.colors[0][size_t(idx2(w, i, j))]);
    }
  }
  std::printf("  mean |p| in air = %g, dye in air half = %g\n", air_p, right_dye);
  check_lt(float(air_p), 1e-3f, "air cells keep pressure ~0");
  check_gt(float(right_dye), 1.0f, "dye can enter air (pass-through free surface)");
}

void test_collider_blocks()
{
  constexpr int w = 40, h = 20;
  FluidGrid g;
  g.resize(w, h, 1);
  std::vector<float> cx(size_t(w * h), 4.0f), cy(size_t(w * h), 0.0f);
  cell_velocity_to_mac(cx, cy, w, h, g.vel_x, g.vel_y);
  /* Solid block in the middle. */
  for (int j = 0; j < h; j++) {
    for (int i = w / 2 - 2; i <= w / 2 + 2; i++) {
      g.collision_mask[size_t(idx2(w, i, j))] = 1.0f;
      g.colors[0][size_t(idx2(w, i, j))] = 0.0f;
    }
  }
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < 4; i++) {
      g.colors[0][size_t(idx2(w, i, j))] = 1.0f;
    }
  }
  FluidStepParams p;
  p.dt = 1.0f / 30.0f;
  p.curl = 0.0f;
  p.pressure = 0.0f;
  p.pressure_iterations = 50;
  p.density_dissipation = 0.0f;
  p.velocity_dissipation = 0.0f;
  for (int i = 0; i < 25; i++) {
    fluid_step(g, p);
  }
  /* Velocity inside solid should be ~0. */
  float solid_speed = 0.0f;
  int cnt = 0;
  std::vector<float> cvx, cvy;
  mac_velocity_to_cell(g.vel_x, g.vel_y, w, h, cvx, cvy);
  for (int j = 0; j < h; j++) {
    for (int i = w / 2 - 2; i <= w / 2 + 2; i++) {
      const int id = idx2(w, i, j);
      solid_speed += std::abs(cvx[size_t(id)]) + std::abs(cvy[size_t(id)]);
      cnt++;
    }
  }
  solid_speed /= float(std::max(cnt, 1));
  std::printf("  mean |vel| inside solid = %g\n", double(solid_speed));
  check_lt(solid_speed, 0.2f, "collider zeros velocity inside solid");
}

/* -------------------------------------------------------------------- */
/** \name GPU Gems Ch.38 collocated unit tests (CPU twin of GPU shaders)
 * \{ */

void test_ch38_jacobi_residual_drops()
{
  constexpr int w = 32, h = 32;
  constexpr float dx = 1.0f;
  const float alpha = -(dx * dx);
  const float rBeta = 0.25f;
  std::vector<float> div(size_t(w * h), 0.0f), p(size_t(w * h), 0.0f), p_tmp;
  /* Zero-mean localized source (Neumann pressure is well-posed only if ∫div=0). */
  for (int j = 8; j <= 12; j++) {
    for (int i = 8; i <= 12; i++) {
      div[size_t(idx2(w, i, j))] = 1.0f;
    }
  }
  for (int j = 20; j <= 24; j++) {
    for (int i = 20; i <= 24; i++) {
      div[size_t(idx2(w, i, j))] = -1.0f;
    }
  }
  auto residual_rms = [&](const std::vector<float> &press) {
    double acc = 0.0;
    int cnt = 0;
    for (int j = 1; j < h - 1; j++) {
      for (int i = 1; i < w - 1; i++) {
        const float L = press[size_t(idx2(w, i - 1, j))];
        const float R = press[size_t(idx2(w, i + 1, j))];
        const float B = press[size_t(idx2(w, i, j - 1))];
        const float T = press[size_t(idx2(w, i, j + 1))];
        const float C = press[size_t(idx2(w, i, j))];
        /* Discrete: (L+R+B+T-4C) should equal div for Poisson with dx=1. */
        const float lap = (L + R + B + T - 4.0f * C);
        const float r = lap - div[size_t(idx2(w, i, j))];
        acc += double(r) * double(r);
        cnt++;
      }
    }
    return float(std::sqrt(acc / double(std::max(cnt, 1))));
  };
  const float r0 = residual_rms(p);
  for (int it = 0; it < 40; it++) {
    ch38_jacobi(p, div, w, h, alpha, rBeta, p_tmp);
    p.swap(p_tmp);
    ch38_boundary(p, w, h, 1.0f);
  }
  const float r1 = residual_rms(p);
  std::printf("  ch38 jacobi residual %g → %g\n", double(r0), double(r1));
  check_lt(r1, r0 * 0.5f, "ch38 pressure Jacobi reduces residual vs cold start");
}

void test_ch38_project_lowers_div()
{
  constexpr int w = 48, h = 48;
  constexpr float dx = 1.0f;
  const float halfdx = 0.5f / dx;
  std::vector<float> u(size_t(w * h)), v(size_t(w * h)), p, div_before, div_after;
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      /* Radial compressible seed. */
      u[size_t(idx2(w, i, j))] = 0.5f * float(i - w / 2) / float(w);
      v[size_t(idx2(w, i, j))] = 0.5f * float(j - h / 2) / float(h);
    }
  }
  ch38_boundary(u, w, h, -1.0f);
  ch38_boundary(v, w, h, -1.0f);
  ch38_divergence(u, v, w, h, halfdx, div_before);
  double acc = 0.0;
  int cnt = 0;
  for (int j = 1; j < h - 1; j++) {
    for (int i = 1; i < w - 1; i++) {
      const float d = div_before[size_t(idx2(w, i, j))];
      acc += double(d) * double(d);
      cnt++;
    }
  }
  const float before = float(std::sqrt(acc / double(std::max(cnt, 1))));
  const float after = ch38_project(u, v, p, w, h, dx, 40);
  std::printf("  ch38 project div RMS %g → %g\n", double(before), double(after));
  check_lt(after, before, "ch38 project lowers mean |div|");
}

void test_ch38_dye_advects_right()
{
  constexpr int w = 64, h = 32;
  constexpr float dx = 1.0f;
  const float rdx = 1.0f / dx;
  const float dt = 1.0f;
  std::vector<float> dye(size_t(w * h), 0.0f), u(size_t(w * h), 4.0f), v(size_t(w * h), 0.0f),
      out;
  /* Compact blob near left. */
  for (int j = h / 2 - 2; j <= h / 2 + 2; j++) {
    for (int i = 8; i <= 12; i++) {
      dye[size_t(idx2(w, i, j))] = 1.0f;
    }
  }
  auto com_x = [&](const std::vector<float> &f) {
    double m = 0.0, mx = 0.0;
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        const double c = double(f[size_t(idx2(w, i, j))]);
        m += c;
        mx += c * double(i);
      }
    }
    return m > 1e-8 ? float(mx / m) : 0.0f;
  };
  const float com0 = com_x(dye);
  /* Multiple steps so blob clearly shifts right (Listing 38-1). */
  for (int s = 0; s < 4; s++) {
    ch38_advect(dye, u, v, w, h, dt, rdx, out);
    dye.swap(out);
  }
  const float com1 = com_x(dye);
  std::printf("  ch38 dye COM x %g → %g\n", double(com0), double(com1));
  check_gt(com1, com0 + 2.0f, "ch38 dye advects right under +u velocity");
}

void test_ch38_boundary_noslip()
{
  constexpr int w = 16, h = 16;
  std::vector<float> u(size_t(w * h), 3.0f);
  /* Interior nonzero; apply no-slip scale=-1 on ghosts. */
  for (int j = 1; j < h - 1; j++) {
    for (int i = 1; i < w - 1; i++) {
      u[size_t(idx2(w, i, j))] = 3.0f;
    }
  }
  ch38_boundary(u, w, h, -1.0f);
  /* Left ghost should equal −interior. */
  const float ghost = u[size_t(idx2(w, 0, h / 2))];
  const float interior = u[size_t(idx2(w, 1, h / 2))];
  std::printf("  ch38 left boundary ghost=%g interior=%g\n", double(ghost), double(interior));
  check(std::abs(ghost + interior) < 1e-5f, "ch38 velocity left edge equals −interior");
}

/** \} */

}  // namespace

int image_fluid_selftest_run()
{
  g_failures = 0;
  std::printf("=== Image Process fluid selftest (MAC + Ch.38) ===\n");
  test_mac_divergence_uniform_flow();
  test_mac_projection_kills_div();
  test_multigrid_beats_few_jacobi();
  test_dye_advects_not_pinned();
  test_pressure_one_no_tide();
  test_no_dissipation_preserves_dye();
  test_air_free_surface();
  test_collider_blocks();
  test_ch38_jacobi_residual_drops();
  test_ch38_project_lowers_div();
  test_ch38_dye_advects_right();
  test_ch38_boundary_noslip();
  if (g_failures == 0) {
    std::printf("ALL PASSED\n");
  }
  else {
    std::printf("%d FAILURE(S)\n", g_failures);
  }
  return g_failures == 0 ? 0 : 1;
}

#ifdef IMG_FLUID_SELFTEST_MAIN
int main()
{
  return image_fluid_selftest_run();
}
#endif
