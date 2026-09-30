/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Standalone self-test for NOD_sdf_math.hh
 */

#include "NOD_sdf_math.hh"

#include <cmath>
#include <cstdio>

namespace {

using namespace blender::nodes::sdf_math;

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

void check_near(const float got, const float expected, const float tol, const char *msg)
{
  if (!(std::fabs(got - expected) <= tol)) {
    std::fprintf(stderr,
                 "FAIL: %s (got %g, expected %g ± %g)\n",
                 msg,
                 double(got),
                 double(expected),
                 double(tol));
    g_failures++;
  }
  else {
    std::printf("PASS: %s (%g ≈ %g)\n", msg, double(got), double(expected));
  }
}

void test_fractals()
{
  check(fractal_primitive_count() == 2, "only mandelbrot + julia");

  check(de_mandelbrot_2d(float2(0.0f, 0.0f), 64, 4.0f, 2.0f) <= 0.5f, "mandelbrot c=0 small DE");
  check(de_mandelbrot_2d(float2(3.0f, 0.0f), 64, 4.0f, 2.0f) > 0.0f, "mandelbrot outside");

  /* Power changes DE for same point outside. */
  const float d2 = de_mandelbrot_2d(float2(0.5f, 0.5f), 64, 4.0f, 2.0f);
  const float d4 = de_mandelbrot_2d(float2(0.5f, 0.5f), 64, 4.0f, 4.0f);
  check(std::isfinite(d2) && std::isfinite(d4), "power DE finite");
  check(std::fabs(d2 - d4) > 1e-6f || d2 == 0.0f || d4 == 0.0f, "power changes DE");

  /* Bailout changes smooth iter. */
  const float it_lo = smooth_iter_mandelbrot(float2(0.3f, 0.5f), 64, 2.0f, 2.0f);
  const float it_hi = smooth_iter_mandelbrot(float2(0.3f, 0.5f), 64, 256.0f, 2.0f);
  check(std::isfinite(it_lo) && std::isfinite(it_hi), "bailout smooth iter finite");

  const float2 jc0(0.0f, 0.0f);
  const float d_unit = de_julia_2d(float2(2.0f, 0.0f), jc0, 128, 4.0f, 2.0f);
  check(d_unit > 0.5f && d_unit < 1.5f, "julia c=0 |z|=2 ~ distance 1");

  for (int i = 0; i < fractal_primitive_count(); i++) {
    const float d = fractal_primitive_distance(
        FractalPrimitiveType(i), float3(4.0f, 0.0f, 0.0f), 2.0f, 32, 4.0f);
    check(d > 0.0f && std::isfinite(d), "dispatch fractal outside");
  }
}

void test_sdf_dispatch()
{
  check(sd_sphere(float3(0.0f), 1.0f) < 0.0f, "sphere interior");
  check(sd_circle(float2(0.0f, 0.0f), 1.0f) < 0.0f, "circle interior");
  for (int i = 0; i < sdf_shape_count(); i++) {
    if (!sdf_shape_index_valid(i)) {
      continue;
    }
    SDFShapeParams p;
    const float d = sdf_shape_distance(SDFShapeType(i), float3(3.0f, 0.0f, 0.0f), p);
    check(std::isfinite(d), "shape finite");
  }
}

}  // namespace

int main()
{
  std::printf("=== NOD_sdf_math selftest ===\n");
  test_sdf_dispatch();
  test_fractals();
  if (g_failures != 0) {
    std::fprintf(stderr, "SELFTEST_FAIL failures=%d\n", g_failures);
    return 1;
  }
  std::printf("SELFTEST_OK\n");
  return 0;
}
