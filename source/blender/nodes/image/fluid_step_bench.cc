#include "NOD_image_fluid_solver.hh"
#include <chrono>
#include <cstdio>
using namespace blender::nodes::image_fluid;
int main() {
  FluidGrid grid;
  grid.resize(128, 128, 1);
  splat_velocity_and_dye(grid, 0.5f, 0.5f, 800.0f, -400.0f, 6.0f, 0.25f, 1.0f);
  FluidStepParams p;
  p.dt = 1.0f/60.0f;
  p.curl = 30.0f;
  p.pressure = 0.8f;
  p.pressure_iterations = 20;
  p.density_dissipation = 1.0f;
  p.velocity_dissipation = 0.2f;
  // warmup
  for (int i=0;i<3;i++) fluid_step(grid, p);
  const int N = 60;
  auto t0 = std::chrono::steady_clock::now();
  for (int i=0;i<N;i++) fluid_step(grid, p);
  auto t1 = std::chrono::steady_clock::now();
  double ms = std::chrono::duration<double,std::milli>(t1-t0).count() / double(N);
  std::printf("sim=128x128 pressure_iters=20 steps=%d ms/step=%.3f finite=%d\n",
    N, ms, field_all_finite(grid.vel_x) && field_all_finite(grid.colors[0]) ? 1 : 0);
  return 0;
}
