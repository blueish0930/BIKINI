/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/**
 * Image Process 2D Navier–Stokes fluid core (CPU) — MAC / staggered grid.
 *
 * Layout (Harlow–Welch MAC, Bridson-style):
 *   - Pressure, dye, div, mask: cell centers, size w×h
 *   - vel_x (u): vertical faces between cells, size (w+1)×h
 *       u(i,j) lives on the left face of cell (i,j), i∈[0,w], j∈[0,h)
 *   - vel_y (v): horizontal faces, size w×(h+1)
 *       v(i,j) lives on the bottom face of cell (i,j), i∈[0,w), j∈[0,h]
 *
 * Grid spacing Δx = Δy = 1 (velocity in cells/second).
 *
 *   step(dt):
 *     1. apply solid / domain BC
 *     2. optional vorticity confinement (via cell-centered curl)
 *     3. div = ∇·u on cells
 *     4. pressure residual scale (warm-start damp)
 *     5. Jacobi Poisson ∇²p = div
 *     6. u -= ∇p on faces  (exact staggered projection)
 *     7. BC again
 *     8. advect u,v (semi-Lagrangian on faces)
 *     9. advect dye at cell centers
 */

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#ifndef IMG_FLUID_SELFTEST_MAIN
#  include "BLI_math_base.hh"
#  include "BLI_task.hh"
#endif

namespace blender::nodes::image_fluid {

#ifndef IMG_FLUID_SELFTEST_MAIN
template<typename Fn> inline void fluid_for_2d(const int w, const int h, const Fn &fn)
{
  blender::threading::parallel_for(
      blender::IndexRange(h), 8, [&](const blender::IndexRange y_range) {
        for (const int y : y_range) {
          for (int x = 0; x < w; x++) {
            fn(x, y);
          }
        }
      });
}
#else
template<typename Fn> inline void fluid_for_2d(const int w, const int h, const Fn &fn)
{
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      fn(x, y);
    }
  }
}
#endif

inline int idx2(const int w, const int x, const int y)
{
  return y * w + x;
}

/** Domain edge treatment (per-side). */
enum class FluidBoundary : char {
  Outflow = 0,
  Wrap = 1,
  Bounce = 2,
};

struct FluidBounds {
  FluidBoundary left = FluidBoundary::Bounce;
  FluidBoundary right = FluidBoundary::Bounce;
  FluidBoundary bottom = FluidBoundary::Bounce;
  FluidBoundary top = FluidBoundary::Bounce;
};

struct FluidStepParams {
  float dt = 1.0f / 60.0f;
  float curl = 0.0f;
  /** Warm-start residual: p *= value before Jacobi (0 = cold p each frame). */
  float pressure = 0.0f;
  int pressure_iterations = 40;
  float density_dissipation = 0.0f;
  float velocity_dissipation = 0.0f;
  float splat_radius = 0.25f;
  float splat_force = 6000.0f;
  FluidBounds bounds;

  float viscosity = 0.0f;
  int viscosity_iterations = 0;
  float dissipation = 0.0f;
  float buoyancy = 0.0f;
  float vorticity = 0.0f;
  /** Geometric multigrid V-cycle count (2–4 typical). */
  int pressure_vcycles = 3;
  int mg_pre_smooth = 2;
  int mg_post_smooth = 2;
  bool warm_start_pressure = true;
  bool maccormack = false;
  FluidBoundary boundary_left = FluidBoundary::Bounce;
  FluidBoundary boundary_right = FluidBoundary::Bounce;
  FluidBoundary boundary_bottom = FluidBoundary::Bounce;
  FluidBoundary boundary_top = FluidBoundary::Bounce;
};

/**
 * MAC fluid grid.
 * vel_x: (width+1)*height, vel_y: width*(height+1), cell fields: width*height.
 */
struct FluidGrid {
  int width = 0;
  int height = 0;

  std::vector<float> vel_x; /* u faces */
  std::vector<float> vel_y; /* v faces */
  std::vector<float> temperature;
  std::vector<float> divergence;
  std::vector<float> pressure;
  std::vector<float> curl;
  std::vector<float> collision_mask; /* cell-centered solid >0.5 */
  /**
   * Air / free-space mask (cell-centered). >0.5 = air:
   * not simulated as incompressible liquid (p=0 free surface).
   * Solid overrides air. Empty mask = entire domain is liquid.
   */
  std::vector<float> air_mask;
  std::vector<float> collider_vel_x; /* cell-centered collider vel */
  std::vector<float> collider_vel_y;
  std::vector<std::vector<float>> colors;

  std::vector<float> tmp_a;
  std::vector<float> tmp_b;
  std::vector<float> tmp_c;
  std::vector<float> tmp_d;

  int u_w() const
  {
    return width + 1;
  }
  int u_h() const
  {
    return height;
  }
  int v_w() const
  {
    return width;
  }
  int v_h() const
  {
    return height + 1;
  }

  int u_index(const int i, const int j) const
  {
    return j * (width + 1) + i;
  }
  int v_index(const int i, const int j) const
  {
    return j * width + i;
  }
  int index(const int x, const int y) const
  {
    return y * width + x;
  }

  bool is_mac() const
  {
    return int(vel_x.size()) == (width + 1) * height && int(vel_y.size()) == width * (height + 1);
  }

  void resize(const int w, const int h, const int color_count)
  {
    width = std::max(w, 1);
    height = std::max(h, 1);
    const int n = width * height;
    const int nu = (width + 1) * height;
    const int nv = width * (height + 1);
    vel_x.assign(size_t(nu), 0.0f);
    vel_y.assign(size_t(nv), 0.0f);
    temperature.assign(size_t(n), 0.0f);
    divergence.assign(size_t(n), 0.0f);
    pressure.assign(size_t(n), 0.0f);
    curl.assign(size_t(n), 0.0f);
    collision_mask.assign(size_t(n), 0.0f);
    air_mask.assign(size_t(n), 0.0f);
    collider_vel_x.assign(size_t(n), 0.0f);
    collider_vel_y.assign(size_t(n), 0.0f);
    tmp_a.assign(size_t(std::max(nu, nv)), 0.0f);
    tmp_b.assign(size_t(std::max(nu, nv)), 0.0f);
    tmp_c.assign(size_t(n), 0.0f);
    tmp_d.assign(size_t(n), 0.0f);
    colors.resize(std::max(color_count, 0));
    for (std::vector<float> &c : colors) {
      c.assign(size_t(n), 0.0f);
    }
  }
};

/* -------------------------------------------------------------------- */
/** \name Sampling helpers
 * \{ */

inline int wrap_i(int i, const int n)
{
  if (n <= 0) {
    return 0;
  }
  i %= n;
  if (i < 0) {
    i += n;
  }
  return i;
}

inline float sample_cell(const std::vector<float> &f,
                         const int w,
                         const int h,
                         float x,
                         float y,
                         const FluidBounds &b = FluidBounds{})
{
  /* x,y in continuous cell indices (0..w, 0..h), sample as bilinear on cells. */
  auto remap = [&](float &p, const int n, const FluidBoundary lo, const FluidBoundary hi) -> bool {
    if (p < 0.0f) {
      if (lo == FluidBoundary::Wrap) {
        p = float(wrap_i(int(std::floor(p)), n)) + (p - std::floor(p));
      }
      else if (lo == FluidBoundary::Bounce) {
        p = -p;
      }
      else {
        return false;
      }
    }
    else if (p > float(n - 1)) {
      if (hi == FluidBoundary::Wrap) {
        p = float(wrap_i(int(std::floor(p)), n)) + (p - std::floor(p));
      }
      else if (hi == FluidBoundary::Bounce) {
        p = 2.0f * float(n - 1) - p;
      }
      else {
        return false;
      }
    }
    p = std::clamp(p, 0.0f, float(std::max(n - 1, 0)));
    return true;
  };
  if (!remap(x, w, b.left, b.right) || !remap(y, h, b.bottom, b.top)) {
    return 0.0f;
  }
  const int x0 = std::clamp(int(std::floor(x)), 0, w - 1);
  const int y0 = std::clamp(int(std::floor(y)), 0, h - 1);
  const int x1 = std::min(x0 + 1, w - 1);
  const int y1 = std::min(y0 + 1, h - 1);
  const float tx = x - float(x0);
  const float ty = y - float(y0);
  const float a = f[size_t(idx2(w, x0, y0))];
  const float bb = f[size_t(idx2(w, x1, y0))];
  const float c = f[size_t(idx2(w, x0, y1))];
  const float d = f[size_t(idx2(w, x1, y1))];
  return (1.0f - tx) * (1.0f - ty) * a + tx * (1.0f - ty) * bb + (1.0f - tx) * ty * c + tx * ty * d;
}

/** Bilinear sample u on (w+1)×h face grid; sx,sy continuous face indices. */
inline float sample_u_face(const std::vector<float> &u,
                           const int w,
                           const int h,
                           float sx,
                           float sy,
                           const FluidBounds &b = FluidBounds{})
{
  const int uw = w + 1;
  auto clamp_or_wrap_x = [&](float &p) {
    if (p < 0.0f) {
      p = (b.left == FluidBoundary::Wrap) ? float(wrap_i(int(std::floor(p)), uw)) +
                                                (p - std::floor(p)) :
                                            0.0f;
    }
    if (p > float(uw - 1)) {
      p = (b.right == FluidBoundary::Wrap) ? float(wrap_i(int(std::floor(p)), uw)) +
                                                 (p - std::floor(p)) :
                                             float(uw - 1);
    }
    p = std::clamp(p, 0.0f, float(uw - 1));
  };
  auto clamp_or_wrap_y = [&](float &p) {
    if (p < 0.0f) {
      p = (b.bottom == FluidBoundary::Wrap) ? float(wrap_i(int(std::floor(p)), h)) +
                                                  (p - std::floor(p)) :
                                              0.0f;
    }
    if (p > float(h - 1)) {
      p = (b.top == FluidBoundary::Wrap) ?
              float(wrap_i(int(std::floor(p)), h)) + (p - std::floor(p)) :
              float(h - 1);
    }
    p = std::clamp(p, 0.0f, float(h - 1));
  };
  clamp_or_wrap_x(sx);
  clamp_or_wrap_y(sy);
  const int x0 = int(std::floor(sx));
  const int y0 = int(std::floor(sy));
  const int x1 = std::min(x0 + 1, uw - 1);
  const int y1 = std::min(y0 + 1, h - 1);
  const float tx = sx - float(x0);
  const float ty = sy - float(y0);
  const float a = u[size_t(y0 * uw + x0)];
  const float bb = u[size_t(y0 * uw + x1)];
  const float c = u[size_t(y1 * uw + x0)];
  const float d = u[size_t(y1 * uw + x1)];
  return (1.0f - tx) * (1.0f - ty) * a + tx * (1.0f - ty) * bb + (1.0f - tx) * ty * c + tx * ty * d;
}

inline float sample_v_face(const std::vector<float> &v,
                           const int w,
                           const int h,
                           float sx,
                           float sy,
                           const FluidBounds &b = FluidBounds{})
{
  const int vh = h + 1;
  auto clamp_or_wrap_x = [&](float &p) {
    if (p < 0.0f) {
      p = (b.left == FluidBoundary::Wrap) ? float(wrap_i(int(std::floor(p)), w)) +
                                                (p - std::floor(p)) :
                                            0.0f;
    }
    if (p > float(w - 1)) {
      p = (b.right == FluidBoundary::Wrap) ?
              float(wrap_i(int(std::floor(p)), w)) + (p - std::floor(p)) :
              float(w - 1);
    }
    p = std::clamp(p, 0.0f, float(w - 1));
  };
  auto clamp_or_wrap_y = [&](float &p) {
    if (p < 0.0f) {
      p = (b.bottom == FluidBoundary::Wrap) ? float(wrap_i(int(std::floor(p)), vh)) +
                                                  (p - std::floor(p)) :
                                              0.0f;
    }
    if (p > float(vh - 1)) {
      p = (b.top == FluidBoundary::Wrap) ? float(wrap_i(int(std::floor(p)), vh)) +
                                               (p - std::floor(p)) :
                                           float(vh - 1);
    }
    p = std::clamp(p, 0.0f, float(vh - 1));
  };
  clamp_or_wrap_x(sx);
  clamp_or_wrap_y(sy);
  const int x0 = int(std::floor(sx));
  const int y0 = int(std::floor(sy));
  const int x1 = std::min(x0 + 1, w - 1);
  const int y1 = std::min(y0 + 1, vh - 1);
  const float tx = sx - float(x0);
  const float ty = sy - float(y0);
  const float a = v[size_t(y0 * w + x0)];
  const float bb = v[size_t(y0 * w + x1)];
  const float c = v[size_t(y1 * w + x0)];
  const float d = v[size_t(y1 * w + x1)];
  return (1.0f - tx) * (1.0f - ty) * a + tx * (1.0f - ty) * bb + (1.0f - tx) * ty * c + tx * ty * d;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Cell ↔ MAC conversion (zone sockets are cell-centered)
 * \{ */

/** Cell-centered (w×h) → MAC faces. Domain-normalized cell vel → cells/sec via scale. */
inline void cell_velocity_to_mac(const std::vector<float> &cell_vx,
                                 const std::vector<float> &cell_vy,
                                 const int w,
                                 const int h,
                                 std::vector<float> &u,
                                 std::vector<float> &v,
                                 const float scale_x = 1.0f,
                                 const float scale_y = 1.0f)
{
  u.assign(size_t((w + 1) * h), 0.0f);
  v.assign(size_t(w * (h + 1)), 0.0f);
  if (int(cell_vx.size()) < w * h || int(cell_vy.size()) < w * h) {
    return;
  }
  for (int j = 0; j < h; j++) {
    for (int i = 1; i < w; i++) {
      const float a = cell_vx[size_t(idx2(w, i - 1, j))] * scale_x;
      const float b = cell_vx[size_t(idx2(w, i, j))] * scale_x;
      u[size_t(j * (w + 1) + i)] = 0.5f * (a + b);
    }
    /* Boundary faces: copy nearest cell (BC applied later). */
    u[size_t(j * (w + 1) + 0)] = cell_vx[size_t(idx2(w, 0, j))] * scale_x;
    u[size_t(j * (w + 1) + w)] = cell_vx[size_t(idx2(w, w - 1, j))] * scale_x;
  }
  for (int j = 1; j < h; j++) {
    for (int i = 0; i < w; i++) {
      const float a = cell_vy[size_t(idx2(w, i, j - 1))] * scale_y;
      const float b = cell_vy[size_t(idx2(w, i, j))] * scale_y;
      v[size_t(j * w + i)] = 0.5f * (a + b);
    }
  }
  for (int i = 0; i < w; i++) {
    v[size_t(0 * w + i)] = cell_vy[size_t(idx2(w, i, 0))] * scale_y;
    v[size_t(h * w + i)] = cell_vy[size_t(idx2(w, i, h - 1))] * scale_y;
  }
}

inline void mac_velocity_to_cell(const std::vector<float> &u,
                                 const std::vector<float> &v,
                                 const int w,
                                 const int h,
                                 std::vector<float> &cell_vx,
                                 std::vector<float> &cell_vy)
{
  cell_vx.assign(size_t(w * h), 0.0f);
  cell_vy.assign(size_t(w * h), 0.0f);
  if (int(u.size()) < (w + 1) * h || int(v.size()) < w * (h + 1)) {
    return;
  }
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      cell_vx[size_t(idx2(w, i, j))] = 0.5f *
                                       (u[size_t(j * (w + 1) + i)] + u[size_t(j * (w + 1) + i + 1)]);
      cell_vy[size_t(idx2(w, i, j))] = 0.5f * (v[size_t(j * w + i)] + v[size_t((j + 1) * w + i)]);
    }
  }
}

/** If vel arrays are still cell-sized (legacy cache), promote to MAC in place. */
inline void ensure_mac(FluidGrid &grid)
{
  const int w = grid.width;
  const int h = grid.height;
  if (grid.is_mac()) {
    return;
  }
  std::vector<float> u, v;
  if (int(grid.vel_x.size()) >= w * h && int(grid.vel_y.size()) >= w * h) {
    cell_velocity_to_mac(grid.vel_x, grid.vel_y, w, h, u, v, 1.0f, 1.0f);
  }
  else {
    u.assign(size_t((w + 1) * h), 0.0f);
    v.assign(size_t(w * (h + 1)), 0.0f);
  }
  grid.vel_x.swap(u);
  grid.vel_y.swap(v);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name MAC operators
 * \{ */

inline bool is_solid_cell(const std::vector<float> &mask, const int w, const int h, const int x, const int y)
{
  if (mask.empty() || x < 0 || y < 0 || x >= w || y >= h) {
    return false;
  }
  return mask[size_t(idx2(w, x, y))] > 0.5f;
}

/** Air free-space (not solid). Empty air_mask ⇒ no air. Solid wins over air. */
inline bool is_air_cell(const std::vector<float> &air,
                        const std::vector<float> &solid,
                        const int w,
                        const int h,
                        const int x,
                        const int y)
{
  if (air.empty() || x < 0 || y < 0 || x >= w || y >= h) {
    return false;
  }
  if (is_solid_cell(solid, w, h, x, y)) {
    return false;
  }
  return air[size_t(idx2(w, x, y))] > 0.5f;
}

/** Liquid = not solid and not air (default whole domain liquid). */
inline bool is_liquid_cell(const std::vector<float> &air,
                           const std::vector<float> &solid,
                           const int w,
                           const int h,
                           const int x,
                           const int y)
{
  if (x < 0 || y < 0 || x >= w || y >= h) {
    return false;
  }
  if (is_solid_cell(solid, w, h, x, y)) {
    return false;
  }
  return !is_air_cell(air, solid, w, h, x, y);
}

inline void apply_mac_boundaries(std::vector<float> &u,
                                 std::vector<float> &v,
                                 const int w,
                                 const int h,
                                 const FluidBounds &b,
                                 const std::vector<float> &mask = {},
                                 const std::vector<float> &cvel_x = {},
                                 const std::vector<float> &cvel_y = {})
{
  const int uw = w + 1;
  /* Domain walls. */
  for (int j = 0; j < h; j++) {
    if (b.left == FluidBoundary::Bounce) {
      u[size_t(j * uw + 0)] = 0.0f;
    }
    else if (b.left == FluidBoundary::Wrap) {
      u[size_t(j * uw + 0)] = u[size_t(j * uw + w)];
    }
    if (b.right == FluidBoundary::Bounce) {
      u[size_t(j * uw + w)] = 0.0f;
    }
    else if (b.right == FluidBoundary::Wrap) {
      u[size_t(j * uw + w)] = u[size_t(j * uw + 0)];
    }
  }
  for (int i = 0; i < w; i++) {
    if (b.bottom == FluidBoundary::Bounce) {
      v[size_t(0 * w + i)] = 0.0f;
    }
    else if (b.bottom == FluidBoundary::Wrap) {
      v[size_t(0 * w + i)] = v[size_t(h * w + i)];
    }
    if (b.top == FluidBoundary::Bounce) {
      v[size_t(h * w + i)] = 0.0f;
    }
    else if (b.top == FluidBoundary::Wrap) {
      v[size_t(h * w + i)] = v[size_t(0 * w + i)];
    }
  }
  /* Solid cells: zero normal face flux; optional collider velocity on faces. */
  if (!mask.empty() && int(mask.size()) >= w * h) {
    const bool has_c = int(cvel_x.size()) >= w * h && int(cvel_y.size()) >= w * h;
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        if (mask[size_t(idx2(w, i, j))] <= 0.5f) {
          continue;
        }
        const float cx = has_c ? cvel_x[size_t(idx2(w, i, j))] : 0.0f;
        const float cy = has_c ? cvel_y[size_t(idx2(w, i, j))] : 0.0f;
        u[size_t(j * uw + i)] = cx;
        u[size_t(j * uw + i + 1)] = cx;
        v[size_t(j * w + i)] = cy;
        v[size_t((j + 1) * w + i)] = cy;
      }
    }
  }
}

/**
 * Exact MAC divergence at cell centers.
 * Only liquid cells form the Poisson RHS; solid/air get div=0.
 */
inline void compute_divergence_mac(const std::vector<float> &u,
                                   const std::vector<float> &v,
                                   const int w,
                                   const int h,
                                   std::vector<float> &div,
                                   const std::vector<float> &mask = {},
                                   const std::vector<float> &air = {})
{
  div.assign(size_t(w * h), 0.0f);
  const int uw = w + 1;
  fluid_for_2d(w, h, [&](const int i, const int j) {
    if (!is_liquid_cell(air, mask, w, h, i, j)) {
      div[size_t(idx2(w, i, j))] = 0.0f;
      return;
    }
    const float du = u[size_t(j * uw + i + 1)] - u[size_t(j * uw + i)];
    const float dv = v[size_t((j + 1) * w + i)] - v[size_t(j * w + i)];
    div[size_t(idx2(w, i, j))] = du + dv;
  });
}

/**
 * Jacobi: ∇²p = div on liquid cells only.
 *  - Solid: Neumann (copy self)
 *  - Air: Dirichlet p = 0 (free surface / open air)
 *  - Liquid neighbor that is air contributes p=0
 */
inline void jacobi_pressure_mac(std::vector<float> &p,
                                const std::vector<float> &div,
                                const int w,
                                const int h,
                                const int iterations,
                                std::vector<float> &tmp,
                                const FluidBounds &b = FluidBounds{},
                                const std::vector<float> &mask = {},
                                const std::vector<float> &air = {})
{
  tmp.resize(size_t(w * h));
  if (int(p.size()) != w * h) {
    p.assign(size_t(w * h), 0.0f);
  }
  /* Air cells always stay at zero pressure. */
  if (!air.empty() && int(air.size()) >= w * h) {
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        if (is_air_cell(air, mask, w, h, i, j)) {
          p[size_t(idx2(w, i, j))] = 0.0f;
        }
      }
    }
  }

  auto neigh_p = [&](const int i, const int j, const int di, const int dj) -> float {
    int ni = i + di;
    int nj = j + dj;
    if (ni < 0) {
      if (b.left == FluidBoundary::Wrap) {
        ni = w - 1;
      }
      else {
        return p[size_t(idx2(w, i, j))]; /* Neumann wall */
      }
    }
    else if (ni >= w) {
      if (b.right == FluidBoundary::Wrap) {
        ni = 0;
      }
      else {
        return p[size_t(idx2(w, i, j))];
      }
    }
    if (nj < 0) {
      if (b.bottom == FluidBoundary::Wrap) {
        nj = h - 1;
      }
      else {
        return p[size_t(idx2(w, i, j))];
      }
    }
    else if (nj >= h) {
      if (b.top == FluidBoundary::Wrap) {
        nj = 0;
      }
      else {
        return p[size_t(idx2(w, i, j))];
      }
    }
    /* Solid neighbor → Neumann. Air neighbor → free-surface p=0. */
    if (is_solid_cell(mask, w, h, ni, nj)) {
      return p[size_t(idx2(w, i, j))];
    }
    if (is_air_cell(air, mask, w, h, ni, nj)) {
      return 0.0f;
    }
    return p[size_t(idx2(w, ni, nj))];
  };

  for (int it = 0; it < iterations; it++) {
    fluid_for_2d(w, h, [&](const int i, const int j) {
      const int id = idx2(w, i, j);
      if (is_solid_cell(mask, w, h, i, j)) {
        tmp[size_t(id)] = p[size_t(id)];
        return;
      }
      if (is_air_cell(air, mask, w, h, i, j)) {
        tmp[size_t(id)] = 0.0f; /* free surface */
        return;
      }
      const float L = neigh_p(i, j, -1, 0);
      const float R = neigh_p(i, j, +1, 0);
      const float B = neigh_p(i, j, 0, -1);
      const float T = neigh_p(i, j, 0, +1);
      tmp[size_t(id)] = (L + R + B + T - div[size_t(id)]) * 0.25f;
    });
    p.swap(tmp);
  }
}

/* -------------------------------------------------------------------- */
/** \name Geometric Multigrid Poisson (∇²p = div) — mask / air aware
 * \{ */

/** Neighbor pressure with domain / solid / air BC (shared by Jacobi residual). */
inline float mac_neigh_p(const std::vector<float> &p,
                         const int w,
                         const int h,
                         const int i,
                         const int j,
                         const int di,
                         const int dj,
                         const FluidBounds &b,
                         const std::vector<float> &mask,
                         const std::vector<float> &air)
{
  int ni = i + di;
  int nj = j + dj;
  if (ni < 0) {
    if (b.left == FluidBoundary::Wrap) {
      ni = w - 1;
    }
    else {
      return p[size_t(idx2(w, i, j))];
    }
  }
  else if (ni >= w) {
    if (b.right == FluidBoundary::Wrap) {
      ni = 0;
    }
    else {
      return p[size_t(idx2(w, i, j))];
    }
  }
  if (nj < 0) {
    if (b.bottom == FluidBoundary::Wrap) {
      nj = h - 1;
    }
    else {
      return p[size_t(idx2(w, i, j))];
    }
  }
  else if (nj >= h) {
    if (b.top == FluidBoundary::Wrap) {
      nj = 0;
    }
    else {
      return p[size_t(idx2(w, i, j))];
    }
  }
  if (is_solid_cell(mask, w, h, ni, nj)) {
    return p[size_t(idx2(w, i, j))];
  }
  if (is_air_cell(air, mask, w, h, ni, nj)) {
    return 0.0f;
  }
  return p[size_t(idx2(w, ni, nj))];
}

/** r = div - ∇²p  (only liquid; solid/air r=0). */
inline void compute_poisson_residual(const std::vector<float> &p,
                                     const std::vector<float> &div,
                                     const int w,
                                     const int h,
                                     std::vector<float> &r,
                                     const FluidBounds &b,
                                     const std::vector<float> &mask,
                                     const std::vector<float> &air)
{
  r.assign(size_t(w * h), 0.0f);
  fluid_for_2d(w, h, [&](const int i, const int j) {
    const int id = idx2(w, i, j);
    if (!is_liquid_cell(air, mask, w, h, i, j)) {
      r[size_t(id)] = 0.0f;
      return;
    }
    const float L = mac_neigh_p(p, w, h, i, j, -1, 0, b, mask, air);
    const float R = mac_neigh_p(p, w, h, i, j, +1, 0, b, mask, air);
    const float B = mac_neigh_p(p, w, h, i, j, 0, -1, b, mask, air);
    const float T = mac_neigh_p(p, w, h, i, j, 0, +1, b, mask, air);
    /* Discrete: ∇²p ≈ L+R+B+T-4p ; equation ∇²p = div ⇒ r = div - ∇²p */
    const float lap = L + R + B + T - 4.0f * p[size_t(id)];
    r[size_t(id)] = div[size_t(id)] - lap;
  });
}

/** Full-weighting restriction fine → coarse (half resolution, floor). */
inline void restrict_scalar_fw(const std::vector<float> &fine,
                               const int fw,
                               const int fh,
                               std::vector<float> &coarse,
                               const int cw,
                               const int ch)
{
  coarse.assign(size_t(cw * ch), 0.0f);
  for (int j = 0; j < ch; j++) {
    for (int i = 0; i < cw; i++) {
      const int i0 = std::min(i * 2, fw - 1);
      const int j0 = std::min(j * 2, fh - 1);
      const int i1 = std::min(i0 + 1, fw - 1);
      const int j1 = std::min(j0 + 1, fh - 1);
      const float a = fine[size_t(idx2(fw, i0, j0))];
      const float b = fine[size_t(idx2(fw, i1, j0))];
      const float c = fine[size_t(idx2(fw, i0, j1))];
      const float d = fine[size_t(idx2(fw, i1, j1))];
      coarse[size_t(idx2(cw, i, j))] = 0.25f * (a + b + c + d);
    }
  }
}

/** Restrict solid: coarse solid if any fine child is solid. */
inline void restrict_solid_mask(const std::vector<float> &fine,
                                const int fw,
                                const int fh,
                                std::vector<float> &coarse,
                                const int cw,
                                const int ch)
{
  coarse.assign(size_t(cw * ch), 0.0f);
  if (fine.empty()) {
    return;
  }
  for (int j = 0; j < ch; j++) {
    for (int i = 0; i < cw; i++) {
      const int i0 = std::min(i * 2, fw - 1);
      const int j0 = std::min(j * 2, fh - 1);
      const int i1 = std::min(i0 + 1, fw - 1);
      const int j1 = std::min(j0 + 1, fh - 1);
      float m = 0.0f;
      if (fine[size_t(idx2(fw, i0, j0))] > 0.5f || fine[size_t(idx2(fw, i1, j0))] > 0.5f ||
          fine[size_t(idx2(fw, i0, j1))] > 0.5f || fine[size_t(idx2(fw, i1, j1))] > 0.5f)
      {
        m = 1.0f;
      }
      coarse[size_t(idx2(cw, i, j))] = m;
    }
  }
}

/** Restrict air: majority of non-solid children; solid coarse clears air. */
inline void restrict_air_mask(const std::vector<float> &fine_air,
                              const std::vector<float> &fine_solid,
                              const int fw,
                              const int fh,
                              std::vector<float> &coarse_air,
                              const std::vector<float> &coarse_solid,
                              const int cw,
                              const int ch)
{
  coarse_air.assign(size_t(cw * ch), 0.0f);
  if (fine_air.empty()) {
    return;
  }
  for (int j = 0; j < ch; j++) {
    for (int i = 0; i < cw; i++) {
      if (!coarse_solid.empty() && coarse_solid[size_t(idx2(cw, i, j))] > 0.5f) {
        coarse_air[size_t(idx2(cw, i, j))] = 0.0f;
        continue;
      }
      const int i0 = std::min(i * 2, fw - 1);
      const int j0 = std::min(j * 2, fh - 1);
      const int i1 = std::min(i0 + 1, fw - 1);
      const int j1 = std::min(j0 + 1, fh - 1);
      int air_votes = 0, n = 0;
      auto consider = [&](const int x, const int y) {
        if (!fine_solid.empty() && fine_solid[size_t(idx2(fw, x, y))] > 0.5f) {
          return;
        }
        n++;
        if (fine_air[size_t(idx2(fw, x, y))] > 0.5f) {
          air_votes++;
        }
      };
      consider(i0, j0);
      consider(i1, j0);
      consider(i0, j1);
      consider(i1, j1);
      coarse_air[size_t(idx2(cw, i, j))] = (n > 0 && air_votes * 2 >= n) ? 1.0f : 0.0f;
    }
  }
}

/** Bilinear prolongation coarse → fine (add correction).
 * Half-index mapping matches full-weighting restrict (coarse i ← fine 2i..2i+1).
 * Domain-UV * coarse_size is wrong when 2*cw != fw (orphan right/top columns). */
inline void prolong_add(const std::vector<float> &coarse,
                        const int cw,
                        const int ch,
                        std::vector<float> &fine,
                        const int fw,
                        const int fh,
                        const std::vector<float> &mask,
                        const std::vector<float> &air)
{
  for (int j = 0; j < fh; j++) {
    const float fy = (float(j) + 0.5f) * 0.5f - 0.5f;
    const int j0 = std::clamp(int(std::floor(fy)), 0, ch - 1);
    const int j1 = std::min(j0 + 1, ch - 1);
    const float ty = std::clamp(fy - float(j0), 0.0f, 1.0f);
    for (int i = 0; i < fw; i++) {
      if (!is_liquid_cell(air, mask, fw, fh, i, j)) {
        if (is_air_cell(air, mask, fw, fh, i, j)) {
          fine[size_t(idx2(fw, i, j))] = 0.0f;
        }
        continue;
      }
      const float fx = (float(i) + 0.5f) * 0.5f - 0.5f;
      const int i0 = std::clamp(int(std::floor(fx)), 0, cw - 1);
      const int i1 = std::min(i0 + 1, cw - 1);
      const float tx = std::clamp(fx - float(i0), 0.0f, 1.0f);
      const float a = coarse[size_t(idx2(cw, i0, j0))];
      const float b = coarse[size_t(idx2(cw, i1, j0))];
      const float c = coarse[size_t(idx2(cw, i0, j1))];
      const float d = coarse[size_t(idx2(cw, i1, j1))];
      const float e = (1.0f - tx) * (1.0f - ty) * a + tx * (1.0f - ty) * b +
                      (1.0f - tx) * ty * c + tx * ty * d;
      fine[size_t(idx2(fw, i, j))] += e;
    }
  }
}

/**
 * One V-cycle for ∇²p = rhs on liquid cells.
 * Recursive geometric coarsening until min(w,h) < 4.
 */
inline void multigrid_vcycle(std::vector<float> &p,
                             const std::vector<float> &rhs,
                             const int w,
                             const int h,
                             const int pre_smooth,
                             const int post_smooth,
                             const int coarse_iters,
                             const FluidBounds &b,
                             const std::vector<float> &mask,
                             const std::vector<float> &air,
                             std::vector<float> &tmp)
{
  /* Need even dims so restrict(texel*2) covers the whole fine grid.
   * Odd width/height leaves the right/top column out of restriction and
   * corrupts the high-index half after prolongation. */
  if (w < 4 || h < 4 || (w & 1) || (h & 1) || w < 8 || h < 8) {
    jacobi_pressure_mac(p, rhs, w, h, coarse_iters, tmp, b, mask, air);
    return;
  }

  /* Pre-smooth. */
  jacobi_pressure_mac(p, rhs, w, h, pre_smooth, tmp, b, mask, air);

  /* Residual. */
  std::vector<float> residual;
  compute_poisson_residual(p, rhs, w, h, residual, b, mask, air);

  /* Restrict (exact half — dims guaranteed even). */
  const int cw = w / 2;
  const int ch = h / 2;
  std::vector<float> rhs_c, p_c, mask_c, air_c, tmp_c;
  restrict_scalar_fw(residual, w, h, rhs_c, cw, ch);
  restrict_solid_mask(mask, w, h, mask_c, cw, ch);
  restrict_air_mask(air, mask, w, h, air_c, mask_c, cw, ch);
  /* Zero residual on coarse non-liquid. */
  for (int j = 0; j < ch; j++) {
    for (int i = 0; i < cw; i++) {
      if (!is_liquid_cell(air_c, mask_c, cw, ch, i, j)) {
        rhs_c[size_t(idx2(cw, i, j))] = 0.0f;
      }
    }
  }

  p_c.assign(size_t(cw * ch), 0.0f);
  multigrid_vcycle(
      p_c, rhs_c, cw, ch, pre_smooth, post_smooth, coarse_iters, b, mask_c, air_c, tmp_c);

  /* Prolong correction. */
  prolong_add(p_c, cw, ch, p, w, h, mask, air);

  /* Post-smooth. */
  jacobi_pressure_mac(p, rhs, w, h, post_smooth, tmp, b, mask, air);
}

/**
 * Geometric Multigrid pressure solve: n_vcycles × V-cycle.
 * Returns liquid residual RMS after last cycle.
 */
inline float multigrid_pressure_mac(std::vector<float> &p,
                                    const std::vector<float> &div,
                                    const int w,
                                    const int h,
                                    const int n_vcycles,
                                    const int pre_smooth,
                                    const int post_smooth,
                                    const int coarse_iters,
                                    const FluidBounds &b = FluidBounds{},
                                    const std::vector<float> &mask = {},
                                    const std::vector<float> &air = {})
{
  if (int(p.size()) != w * h) {
    p.assign(size_t(w * h), 0.0f);
  }
  std::vector<float> tmp;
  const int cycles = std::max(n_vcycles, 1);
  const int pre = std::max(pre_smooth, 1);
  const int post = std::max(post_smooth, 1);
  const int coarse = std::max(coarse_iters, 4);

  /* Tiny grids: pure Jacobi is fine / cheaper. */
  if (w * h <= 64 || w < 8 || h < 8) {
    jacobi_pressure_mac(p, div, w, h, coarse * cycles, tmp, b, mask, air);
  }
  else {
    for (int c = 0; c < cycles; c++) {
      multigrid_vcycle(p, div, w, h, pre, post, coarse, b, mask, air, tmp);
    }
  }

  std::vector<float> r;
  compute_poisson_residual(p, div, w, h, r, b, mask, air);
  double acc = 0.0;
  int cnt = 0;
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      if (!is_liquid_cell(air, mask, w, h, i, j)) {
        continue;
      }
      const float v = r[size_t(idx2(w, i, j))];
      acc += double(v) * double(v);
      cnt++;
    }
  }
  return (cnt > 0) ? float(std::sqrt(acc / double(cnt))) : 0.0f;
}

/** \} */

/**
 * u -= ∂p/∂x, v -= ∂p/∂y on faces.
 * Skip solid faces. Air has p=0 so liquid↔air faces get free-surface pull.
 * Pure air↔air faces: p=0 on both sides → no force (pass-through).
 */
inline void gradient_subtract_mac(std::vector<float> &u,
                                  std::vector<float> &v,
                                  const std::vector<float> &p,
                                  const int w,
                                  const int h,
                                  const std::vector<float> &mask = {},
                                  const std::vector<float> & /*air*/ = {})
{
  const int uw = w + 1;
  for (int j = 0; j < h; j++) {
    for (int i = 1; i < w; i++) {
      if (is_solid_cell(mask, w, h, i - 1, j) || is_solid_cell(mask, w, h, i, j)) {
        continue;
      }
      u[size_t(j * uw + i)] -= (p[size_t(idx2(w, i, j))] - p[size_t(idx2(w, i - 1, j))]);
    }
  }
  for (int j = 1; j < h; j++) {
    for (int i = 0; i < w; i++) {
      if (is_solid_cell(mask, w, h, i, j - 1) || is_solid_cell(mask, w, h, i, j)) {
        continue;
      }
      v[size_t(j * w + i)] -= (p[size_t(idx2(w, i, j))] - p[size_t(idx2(w, i, j - 1))]);
    }
  }
}

inline void scale_field(std::vector<float> &f, const float value)
{
  for (float &v : f) {
    v *= value;
  }
}

/** Cell-centered curl from MAC (for vorticity confinement). */
inline void compute_curl_mac(const std::vector<float> &u,
                             const std::vector<float> &v,
                             const int w,
                             const int h,
                             std::vector<float> &curl)
{
  curl.assign(size_t(w * h), 0.0f);
  const int uw = w + 1;
  fluid_for_2d(w, h, [&](const int i, const int j) {
    /* ω = ∂v/∂x - ∂u/∂y at cell center using face averages. */
    const float v_r = (i + 1 < w) ? 0.5f * (v[size_t(j * w + i + 1)] + v[size_t((j + 1) * w + i + 1)]) :
                                    0.5f * (v[size_t(j * w + i)] + v[size_t((j + 1) * w + i)]);
    const float v_l = (i > 0) ? 0.5f * (v[size_t(j * w + i - 1)] + v[size_t((j + 1) * w + i - 1)]) :
                                0.5f * (v[size_t(j * w + i)] + v[size_t((j + 1) * w + i)]);
    const float u_t = (j + 1 < h) ?
                          0.5f * (u[size_t((j + 1) * uw + i)] + u[size_t((j + 1) * uw + i + 1)]) :
                          0.5f * (u[size_t(j * uw + i)] + u[size_t(j * uw + i + 1)]);
    const float u_b = (j > 0) ? 0.5f * (u[size_t((j - 1) * uw + i)] + u[size_t((j - 1) * uw + i + 1)]) :
                                0.5f * (u[size_t(j * uw + i)] + u[size_t(j * uw + i + 1)]);
    curl[size_t(idx2(w, i, j))] = (v_r - v_l) - (u_t - u_b);
  });
}

inline void apply_vorticity_mac(std::vector<float> &u,
                                std::vector<float> &v,
                                const std::vector<float> &curl,
                                const int w,
                                const int h,
                                const float strength,
                                const float dt)
{
  if (strength == 0.0f) {
    return;
  }
  /* Force at cell centers, then scatter half to faces. */
  std::vector<float> fx(size_t(w * h), 0.0f), fy(size_t(w * h), 0.0f);
  fluid_for_2d(w, h, [&](const int i, const int j) {
    const int id = idx2(w, i, j);
    const int il = std::max(i - 1, 0);
    const int ir = std::min(i + 1, w - 1);
    const int jb = std::max(j - 1, 0);
    const int jt = std::min(j + 1, h - 1);
    float eta_x = 0.5f * (std::abs(curl[size_t(idx2(w, i, jt))]) -
                          std::abs(curl[size_t(idx2(w, i, jb))]));
    float eta_y = 0.5f * (std::abs(curl[size_t(idx2(w, ir, j))]) -
                          std::abs(curl[size_t(idx2(w, il, j))]));
    const float len = std::sqrt(eta_x * eta_x + eta_y * eta_y) + 1e-4f;
    eta_x = (eta_x / len) * strength * curl[size_t(id)];
    eta_y = (eta_y / len) * strength * curl[size_t(id)];
    /* N = (ηx, ηy), force = N × ω ẑ → (ηy * ω? standard: force = ε (N×ω))
     * WebGL: force = 0.5*normalize(|T|-|B|, |R|-|L|) * curl * C; fy *= -1
     * So fx = ηx * C, fy = -ηy * C with η already including C... */
    fx[size_t(id)] = eta_x * dt;
    fy[size_t(id)] = -eta_y * dt;
  });
  const int uw = w + 1;
  for (int j = 0; j < h; j++) {
    for (int i = 1; i < w; i++) {
      u[size_t(j * uw + i)] += 0.5f * (fx[size_t(idx2(w, i - 1, j))] + fx[size_t(idx2(w, i, j))]);
    }
  }
  for (int j = 1; j < h; j++) {
    for (int i = 0; i < w; i++) {
      v[size_t(j * w + i)] += 0.5f * (fy[size_t(idx2(w, i, j - 1))] + fy[size_t(idx2(w, i, j))]);
    }
  }
}

/**
 * Semi-Lagrangian advection of face velocity.
 * u face at (i, j+0.5): position (i, j+0.5) in cell coords.
 */
inline void advect_mac_velocity(const std::vector<float> &u_src,
                                const std::vector<float> &v_src,
                                const int w,
                                const int h,
                                const float dt,
                                const float dissipation,
                                std::vector<float> &u_dst,
                                std::vector<float> &v_dst,
                                const FluidBounds &b = FluidBounds{})
{
  const int uw = w + 1;
  const float decay = 1.0f + dissipation * dt;
  u_dst.assign(size_t(uw * h), 0.0f);
  v_dst.assign(size_t(w * (h + 1)), 0.0f);

  /* Advect u faces. */
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < uw; i++) {
      const float px = float(i);
      const float py = float(j) + 0.5f;
      /* Velocity at face: u known, v interpolated. */
      const float uu = u_src[size_t(j * uw + i)];
      const float vv = sample_v_face(v_src, w, h, float(i) - 0.5f, float(j) + 0.5f, b);
      const float bx = px - dt * uu;
      const float by = py - dt * vv;
      u_dst[size_t(j * uw + i)] = sample_u_face(u_src, w, h, bx, by - 0.5f, b) / decay;
    }
  }
  /* Advect v faces. */
  for (int j = 0; j < h + 1; j++) {
    for (int i = 0; i < w; i++) {
      const float px = float(i) + 0.5f;
      const float py = float(j);
      const float vv = v_src[size_t(j * w + i)];
      const float uu = sample_u_face(u_src, w, h, float(i) + 0.5f, float(j) - 0.5f, b);
      const float bx = px - dt * uu;
      const float by = py - dt * vv;
      v_dst[size_t(j * w + i)] = sample_v_face(v_src, w, h, bx - 0.5f, by, b) / decay;
    }
  }
}

/** Advect cell-centered scalar using MAC velocity. */
inline void advect_scalar_mac(const std::vector<float> &src,
                              const std::vector<float> &u,
                              const std::vector<float> &v,
                              const int w,
                              const int h,
                              const float dt,
                              const float dissipation,
                              std::vector<float> &dst,
                              const FluidBounds &b = FluidBounds{})
{
  dst.assign(size_t(w * h), 0.0f);
  const float decay = 1.0f + dissipation * dt;
  const int uw = w + 1;
  fluid_for_2d(w, h, [&](const int i, const int j) {
    const float uu = 0.5f * (u[size_t(j * uw + i)] + u[size_t(j * uw + i + 1)]);
    const float vv = 0.5f * (v[size_t(j * w + i)] + v[size_t((j + 1) * w + i)]);
    const float px = float(i) + 0.5f - dt * uu;
    const float py = float(j) + 0.5f - dt * vv;
    dst[size_t(idx2(w, i, j))] = sample_cell(src, w, h, px - 0.5f, py - 0.5f, b) / decay;
  });
}

inline float field_rms(const std::vector<float> &f)
{
  if (f.empty()) {
    return 0.0f;
  }
  double acc = 0.0;
  for (const float v : f) {
    acc += double(v) * double(v);
  }
  return float(std::sqrt(acc / double(f.size())));
}

inline float field_max_abs(const std::vector<float> &f)
{
  float m = 0.0f;
  for (const float v : f) {
    m = std::max(m, std::abs(v));
  }
  return m;
}

inline float field_sum(const std::vector<float> &f)
{
  double s = 0.0;
  for (const float v : f) {
    s += double(v);
  }
  return float(s);
}

inline float field_energy(const std::vector<float> &vx, const std::vector<float> &vy)
{
  double e = 0.0;
  const size_t n = std::min(vx.size(), vy.size());
  for (size_t i = 0; i < n; i++) {
    e += double(vx[i]) * double(vx[i]) + double(vy[i]) * double(vy[i]);
  }
  return float(e);
}

inline bool field_all_finite(const std::vector<float> &f)
{
  for (const float v : f) {
    if (!std::isfinite(v)) {
      return false;
    }
  }
  return true;
}

inline void resize_scalar_field(const std::vector<float> &src,
                                const int sw,
                                const int sh,
                                std::vector<float> &dst,
                                const int dw,
                                const int dh)
{
  dst.resize(size_t(dw) * size_t(dh));
  if (sw <= 0 || sh <= 0 || dw <= 0 || dh <= 0 || src.empty()) {
    std::fill(dst.begin(), dst.end(), 0.0f);
    return;
  }
  if (sw == dw && sh == dh && int(src.size()) >= dw * dh) {
    dst.assign(src.begin(), src.begin() + dw * dh);
    return;
  }
  for (int y = 0; y < dh; y++) {
    const float v = (float(y) + 0.5f) / float(dh) * float(sh) - 0.5f;
    const int y0 = std::clamp(int(std::floor(v)), 0, sh - 1);
    const int y1 = std::min(y0 + 1, sh - 1);
    const float ty = std::clamp(v - float(y0), 0.0f, 1.0f);
    for (int x = 0; x < dw; x++) {
      const float u = (float(x) + 0.5f) / float(dw) * float(sw) - 0.5f;
      const int x0 = std::clamp(int(std::floor(u)), 0, sw - 1);
      const int x1 = std::min(x0 + 1, sw - 1);
      const float tx = std::clamp(u - float(x0), 0.0f, 1.0f);
      const float a = src[size_t(y0 * sw + x0)];
      const float bb = src[size_t(y0 * sw + x1)];
      const float c = src[size_t(y1 * sw + x0)];
      const float d = src[size_t(y1 * sw + x1)];
      dst[size_t(y * dw + x)] = (1.0f - tx) * (1.0f - ty) * a + tx * (1.0f - ty) * bb +
                                (1.0f - tx) * ty * c + tx * ty * d;
    }
  }
}

inline void splat_field(std::vector<float> &field,
                        const int w,
                        const int h,
                        const float point_u,
                        const float point_v,
                        const float amount,
                        const float radius,
                        const float aspect)
{
  if (radius <= 0.0f || amount == 0.0f || int(field.size()) < w * h) {
    return;
  }
  fluid_for_2d(w, h, [&](const int x, const int y) {
    const float u = (float(x) + 0.5f) / float(w);
    const float v = (float(y) + 0.5f) / float(h);
    float px = (u - point_u) * aspect;
    float py = v - point_v;
    const float d2 = px * px + py * py;
    field[size_t(idx2(w, x, y))] += std::exp(-d2 / radius) * amount;
  });
}

/** Splat into cell-centered velocity then convert — or splat on MAC directly. */
inline void splat_velocity_and_dye(FluidGrid &grid,
                                   const float point_u,
                                   const float point_v,
                                   const float force_x,
                                   const float force_y,
                                   const float dye_amount,
                                   const float radius_param,
                                   const float aspect = 1.0f)
{
  ensure_mac(grid);
  const int w = grid.width;
  const int h = grid.height;
  float radius = radius_param / 100.0f;
  if (aspect > 1.0f) {
    radius *= aspect;
  }
  /* Build cell force, scatter to faces. */
  std::vector<float> cx(size_t(w * h), 0.0f), cy(size_t(w * h), 0.0f);
  splat_field(cx, w, h, point_u, point_v, force_x, radius, aspect);
  splat_field(cy, w, h, point_u, point_v, force_y, radius, aspect);
  std::vector<float> du, dv;
  cell_velocity_to_mac(cx, cy, w, h, du, dv, 1.0f, 1.0f);
  for (size_t i = 0; i < grid.vel_x.size(); i++) {
    grid.vel_x[i] += du[i];
  }
  for (size_t i = 0; i < grid.vel_y.size(); i++) {
    grid.vel_y[i] += dv[i];
  }
  if (!grid.colors.empty() && dye_amount != 0.0f) {
    splat_field(grid.colors[0], w, h, point_u, point_v, dye_amount, radius, aspect);
  }
}

inline FluidStepParams resolve_params(const FluidStepParams &in)
{
  FluidStepParams p = in;
  p.dt = std::max(p.dt, 1e-6f);
  p.pressure_iterations = std::max(p.pressure_iterations, 1);
  p.pressure = std::clamp(p.pressure, 0.0f, 1.0f);
  const bool legacy_non_default = (in.boundary_left != FluidBoundary::Bounce ||
                                   in.boundary_right != FluidBoundary::Bounce ||
                                   in.boundary_bottom != FluidBoundary::Bounce ||
                                   in.boundary_top != FluidBoundary::Bounce);
  if (legacy_non_default) {
    p.bounds.left = in.boundary_left;
    p.bounds.right = in.boundary_right;
    p.bounds.bottom = in.boundary_bottom;
    p.bounds.top = in.boundary_top;
  }
  else {
    p.bounds = in.bounds;
  }
  p.boundary_left = p.bounds.left;
  p.boundary_right = p.bounds.right;
  p.boundary_bottom = p.bounds.bottom;
  p.boundary_top = p.bounds.top;
  return p;
}

/* Forward decls: Ch.38 collocated twins defined below (used by fluid_step viscosity). */
inline void ch38_jacobi(const std::vector<float> &x,
                        const std::vector<float> &b,
                        const int w,
                        const int h,
                        const float alpha,
                        const float rBeta,
                        std::vector<float> &x_out);
inline void ch38_boundary(std::vector<float> &field, const int w, const int h, const float scale);

/**
 * One MAC fluid step. Returns RMS of post-projection cell divergence.
 */
inline float fluid_step(FluidGrid &grid, const FluidStepParams &params_in)
{
  const FluidStepParams params = resolve_params(params_in);
  const FluidBounds &b = params.bounds;
  const int w = grid.width;
  const int h = grid.height;
  if (w <= 0 || h <= 0) {
    return 0.0f;
  }
  ensure_mac(grid);
  if (int(grid.pressure.size()) != w * h) {
    grid.pressure.assign(size_t(w * h), 0.0f);
  }
  if (int(grid.divergence.size()) != w * h) {
    grid.divergence.assign(size_t(w * h), 0.0f);
  }
  if (int(grid.curl.size()) != w * h) {
    grid.curl.assign(size_t(w * h), 0.0f);
  }

  const float dt = params.dt;
  const std::vector<float> &mask = grid.collision_mask;
  const std::vector<float> &air = grid.air_mask;
  if (int(grid.air_mask.size()) != w * h) {
    grid.air_mask.assign(size_t(w * h), 0.0f);
  }

  apply_mac_boundaries(
      grid.vel_x, grid.vel_y, w, h, b, mask, grid.collider_vel_x, grid.collider_vel_y);

  if (params.curl != 0.0f) {
    compute_curl_mac(grid.vel_x, grid.vel_y, w, h, grid.curl);
    apply_vorticity_mac(grid.vel_x, grid.vel_y, grid.curl, w, h, params.curl, dt);
    apply_mac_boundaries(
        grid.vel_x, grid.vel_y, w, h, b, mask, grid.collider_vel_x, grid.collider_vel_y);
  }

  /*
   * Ch.38 viscous diffusion (collocated twin on cell velocity) when ν > 0.
   * α = dx²/(ν·dt), rβ = 1/(4+α); BC scale=-1 after each Jacobi iter.
   */
  if (params.viscosity > 1e-8f && params.viscosity_iterations > 0) {
    constexpr float dx_c = 1.0f;
    const float alpha = (dx_c * dx_c) / std::max(params.viscosity * dt, 1e-12f);
    const float rBeta = 1.0f / (4.0f + alpha);
    std::vector<float> ux, uy, bux, buy, tmp;
    mac_velocity_to_cell(grid.vel_x, grid.vel_y, w, h, ux, uy);
    bux = ux;
    buy = uy;
    const int n_it = std::max(params.viscosity_iterations, 1);
    for (int it = 0; it < n_it; it++) {
      ch38_jacobi(ux, bux, w, h, alpha, rBeta, tmp);
      ux.swap(tmp);
      ch38_boundary(ux, w, h, -1.0f);
      ch38_jacobi(uy, buy, w, h, alpha, rBeta, tmp);
      uy.swap(tmp);
      ch38_boundary(uy, w, h, -1.0f);
    }
    cell_velocity_to_mac(ux, uy, w, h, grid.vel_x, grid.vel_y);
    apply_mac_boundaries(
        grid.vel_x, grid.vel_y, w, h, b, mask, grid.collider_vel_x, grid.collider_vel_y);
  }

  compute_divergence_mac(grid.vel_x, grid.vel_y, w, h, grid.divergence, mask, air);

  /* Warm-start damp. pressure=1 keeps previous p (stable with MAC projection). */
  if (!params.warm_start_pressure || params.pressure <= 0.0f) {
    std::fill(grid.pressure.begin(), grid.pressure.end(), 0.0f);
  }
  else if (params.pressure < 1.0f) {
    scale_field(grid.pressure, params.pressure);
  }
  /* Air always free-surface p=0. */
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      if (is_air_cell(air, mask, w, h, i, j)) {
        grid.pressure[size_t(idx2(w, i, j))] = 0.0f;
      }
    }
  }

  /* Geometric Multigrid V-cycles (cheap defaults: few cycles, light smooth). */
  {
    int n_vc = params.pressure_vcycles > 0 ? params.pressure_vcycles : 3;
    if (n_vc > 8) {
      n_vc = std::clamp(n_vc / 10, 2, 4); /* old Jacobi-era DNA */
    }
    n_vc = std::clamp(n_vc, 1, 6);
    const int pre = std::max(params.mg_pre_smooth, 1);
    const int post = std::max(params.mg_post_smooth, 1);
    const int coarse = std::clamp(params.pressure_iterations, 8, 24);
    multigrid_pressure_mac(
        grid.pressure, grid.divergence, w, h, n_vc, pre, post, coarse, b, mask, air);
  }

  gradient_subtract_mac(grid.vel_x, grid.vel_y, grid.pressure, w, h, mask, air);
  apply_mac_boundaries(
      grid.vel_x, grid.vel_y, w, h, b, mask, grid.collider_vel_x, grid.collider_vel_y);

  compute_divergence_mac(grid.vel_x, grid.vel_y, w, h, grid.divergence, mask, air);
  /* RMS only over liquid cells (air/solid div is forced 0). */
  float div_after = 0.0f;
  {
    double acc = 0.0;
    int cnt = 0;
    for (int j = 0; j < h; j++) {
      for (int i = 0; i < w; i++) {
        if (!is_liquid_cell(air, mask, w, h, i, j)) {
          continue;
        }
        const float d = grid.divergence[size_t(idx2(w, i, j))];
        acc += double(d) * double(d);
        cnt++;
      }
    }
    div_after = (cnt > 0) ? float(std::sqrt(acc / double(cnt))) : 0.0f;
  }

  advect_mac_velocity(grid.vel_x,
                      grid.vel_y,
                      w,
                      h,
                      dt,
                      params.velocity_dissipation,
                      grid.tmp_a,
                      grid.tmp_b,
                      b);
  grid.vel_x.swap(grid.tmp_a);
  grid.vel_y.swap(grid.tmp_b);
  apply_mac_boundaries(
      grid.vel_x, grid.vel_y, w, h, b, mask, grid.collider_vel_x, grid.collider_vel_y);

  for (std::vector<float> &color : grid.colors) {
    if (int(color.size()) != w * h) {
      color.assign(size_t(w * h), 0.0f);
    }
    advect_scalar_mac(color,
                      grid.vel_x,
                      grid.vel_y,
                      w,
                      h,
                      dt,
                      params.density_dissipation,
                      grid.tmp_c,
                      b);
    color.swap(grid.tmp_c);
  }
  if (int(grid.temperature.size()) == w * h) {
    advect_scalar_mac(grid.temperature,
                      grid.vel_x,
                      grid.vel_y,
                      w,
                      h,
                      dt,
                      params.density_dissipation,
                      grid.tmp_c,
                      b);
    grid.temperature.swap(grid.tmp_c);
  }

  return div_after;
}

/**
 * Advect dye at dye_w×dye_h using MAC velocity at vel_w×vel_h
 * (velocity is first converted to cell-centered and resized).
 */
inline void advect_dye_fields_mac(const std::vector<float> &u,
                                  const std::vector<float> &v,
                                  const int vel_w,
                                  const int vel_h,
                                  std::vector<std::vector<float>> &dye_channels,
                                  const int dye_w,
                                  const int dye_h,
                                  const float dt,
                                  const float density_dissipation,
                                  const FluidBounds &bounds = FluidBounds{})
{
  if (dye_w <= 0 || dye_h <= 0 || dye_channels.empty()) {
    return;
  }
  std::vector<float> cell_vx, cell_vy, vx_dye, vy_dye;
  mac_velocity_to_cell(u, v, vel_w, vel_h, cell_vx, cell_vy);
  if (vel_w == dye_w && vel_h == dye_h) {
    vx_dye.swap(cell_vx);
    vy_dye.swap(cell_vy);
  }
  else {
    resize_scalar_field(cell_vx, vel_w, vel_h, vx_dye, dye_w, dye_h);
    resize_scalar_field(cell_vy, vel_w, vel_h, vy_dye, dye_w, dye_h);
  }
  /* Build temporary MAC at dye res for consistent advection, or cell SL: */
  std::vector<float> tmp;
  for (std::vector<float> &ch : dye_channels) {
    if (int(ch.size()) != dye_w * dye_h) {
      ch.assign(size_t(dye_w * dye_h), 0.0f);
    }
    tmp.assign(size_t(dye_w * dye_h), 0.0f);
    const float decay = 1.0f + density_dissipation * dt;
    fluid_for_2d(dye_w, dye_h, [&](const int i, const int j) {
      const int id = idx2(dye_w, i, j);
      const float uu = vx_dye[size_t(id)];
      const float vv = vy_dye[size_t(id)];
      /* Velocity in cells/sec at dye res: scale if resized from different grid. */
      const float sx = float(dye_w) / float(std::max(vel_w, 1));
      const float sy = float(dye_h) / float(std::max(vel_h, 1));
      const float px = float(i) + 0.5f - dt * uu * sx;
      const float py = float(j) + 0.5f - dt * vv * sy;
      tmp[size_t(id)] = sample_cell(ch, dye_w, dye_h, px - 0.5f, py - 0.5f, bounds) / decay;
    });
    ch.swap(tmp);
  }
}

/* Back-compat name used by zone. */
inline void advect_dye_fields(const std::vector<float> &vel_x,
                              const std::vector<float> &vel_y,
                              const int vel_w,
                              const int vel_h,
                              std::vector<std::vector<float>> &dye_channels,
                              const int dye_w,
                              const int dye_h,
                              const float dt,
                              const float density_dissipation,
                              const FluidBounds &bounds = FluidBounds{})
{
  /* Accept either MAC or cell-centered vel_x/vel_y. */
  if (int(vel_x.size()) == (vel_w + 1) * vel_h && int(vel_y.size()) == vel_w * (vel_h + 1)) {
    advect_dye_fields_mac(vel_x,
                          vel_y,
                          vel_w,
                          vel_h,
                          dye_channels,
                          dye_w,
                          dye_h,
                          dt,
                          density_dissipation,
                          bounds);
    return;
  }
  /* Cell-centered fallback. */
  std::vector<float> u, v;
  cell_velocity_to_mac(vel_x, vel_y, vel_w, vel_h, u, v);
  advect_dye_fields_mac(
      u, v, vel_w, vel_h, dye_channels, dye_w, dye_h, dt, density_dissipation, bounds);
}

inline float dual_res_feedback_step(FluidGrid &sim_grid,
                                    const int domain_w,
                                    const int domain_h,
                                    const FluidStepParams &params)
{
  ensure_mac(sim_grid);
  const int sw = sim_grid.width;
  const int sh = sim_grid.height;
  if (sw <= 0 || sh <= 0 || domain_w <= 0 || domain_h <= 0) {
    return 0.0f;
  }
  std::vector<float> cell_vx, cell_vy, dom_vx, dom_vy, dom_dye;
  mac_velocity_to_cell(sim_grid.vel_x, sim_grid.vel_y, sw, sh, cell_vx, cell_vy);
  resize_scalar_field(cell_vx, sw, sh, dom_vx, domain_w, domain_h);
  resize_scalar_field(cell_vy, sw, sh, dom_vy, domain_w, domain_h);
  if (!sim_grid.colors.empty()) {
    resize_scalar_field(sim_grid.colors[0], sw, sh, dom_dye, domain_w, domain_h);
  }
  else {
    dom_dye.assign(size_t(domain_w * domain_h), 0.0f);
  }
  resize_scalar_field(dom_vx, domain_w, domain_h, cell_vx, sw, sh);
  resize_scalar_field(dom_vy, domain_w, domain_h, cell_vy, sw, sh);
  cell_velocity_to_mac(cell_vx, cell_vy, sw, sh, sim_grid.vel_x, sim_grid.vel_y);
  if (sim_grid.colors.empty()) {
    sim_grid.colors.resize(1);
  }
  resize_scalar_field(dom_dye, domain_w, domain_h, sim_grid.colors[0], sw, sh);
  fluid_step(sim_grid, params);
  return field_sum(sim_grid.colors[0]);
}

/* -------------------------------------------------------------------- */
/** \name Compatibility shims
 * \{ */

inline FluidBounds fluid_bounds_from_params(const FluidStepParams &p)
{
  return resolve_params(p).bounds;
}

/** Collocated-style div for tests: convert to MAC first if needed. */
inline void compute_divergence(const std::vector<float> &vel_x,
                               const std::vector<float> &vel_y,
                               const int w,
                               const int h,
                               std::vector<float> &div,
                               const FluidBounds & /*b*/ = FluidBounds{},
                               const std::vector<float> &mask = {})
{
  if (int(vel_x.size()) == (w + 1) * h && int(vel_y.size()) == w * (h + 1)) {
    compute_divergence_mac(vel_x, vel_y, w, h, div, mask);
    return;
  }
  std::vector<float> u, v;
  cell_velocity_to_mac(vel_x, vel_y, w, h, u, v);
  compute_divergence_mac(u, v, w, h, div, mask);
}

inline void compute_divergence(const std::vector<float> &vel_x,
                               const std::vector<float> &vel_y,
                               const std::vector<float> &mask,
                               const int w,
                               const int h,
                               std::vector<float> &div)
{
  compute_divergence(vel_x, vel_y, w, h, div, FluidBounds{}, mask);
}

inline void compute_divergence(const std::vector<float> &vel_x,
                               const std::vector<float> &vel_y,
                               const std::vector<float> &mask,
                               const int w,
                               const int h,
                               const FluidBounds &b,
                               std::vector<float> &div)
{
  compute_divergence(vel_x, vel_y, w, h, div, b, mask);
}

inline void compute_curl(const std::vector<float> &vel_x,
                         const std::vector<float> &vel_y,
                         const int w,
                         const int h,
                         std::vector<float> &curl_out)
{
  if (int(vel_x.size()) == (w + 1) * h && int(vel_y.size()) == w * (h + 1)) {
    compute_curl_mac(vel_x, vel_y, w, h, curl_out);
    return;
  }
  std::vector<float> u, v;
  cell_velocity_to_mac(vel_x, vel_y, w, h, u, v);
  compute_curl_mac(u, v, w, h, curl_out);
}

inline void apply_vorticity(std::vector<float> &vel_x,
                            std::vector<float> &vel_y,
                            const std::vector<float> &curl_field,
                            const int w,
                            const int h,
                            const float curl_strength,
                            const float dt)
{
  if (int(vel_x.size()) == (w + 1) * h && int(vel_y.size()) == w * (h + 1)) {
    apply_vorticity_mac(vel_x, vel_y, curl_field, w, h, curl_strength, dt);
    return;
  }
  std::vector<float> u, v;
  cell_velocity_to_mac(vel_x, vel_y, w, h, u, v);
  apply_vorticity_mac(u, v, curl_field, w, h, curl_strength, dt);
  mac_velocity_to_cell(u, v, w, h, vel_x, vel_y);
}

inline void jacobi_pressure(std::vector<float> &pressure,
                            const std::vector<float> &div,
                            const int w,
                            const int h,
                            const int iterations,
                            std::vector<float> &tmp,
                            const FluidBounds &b = FluidBounds{},
                            const std::vector<float> &mask = {})
{
  jacobi_pressure_mac(pressure, div, w, h, iterations, tmp, b, mask);
}

inline void gradient_subtract(std::vector<float> &vel_x,
                              std::vector<float> &vel_y,
                              const std::vector<float> &pressure,
                              const int w,
                              const int h,
                              const FluidBounds & /*b*/ = FluidBounds{},
                              const std::vector<float> &mask = {})
{
  if (int(vel_x.size()) == (w + 1) * h && int(vel_y.size()) == w * (h + 1)) {
    gradient_subtract_mac(vel_x, vel_y, pressure, w, h, mask);
    return;
  }
  std::vector<float> u, v;
  cell_velocity_to_mac(vel_x, vel_y, w, h, u, v);
  gradient_subtract_mac(u, v, pressure, w, h, mask);
  mac_velocity_to_cell(u, v, w, h, vel_x, vel_y);
}

inline void apply_velocity_boundaries(std::vector<float> &vel_x,
                                      std::vector<float> &vel_y,
                                      const int w,
                                      const int h,
                                      const FluidBounds &b)
{
  if (int(vel_x.size()) == (w + 1) * h && int(vel_y.size()) == w * (h + 1)) {
    apply_mac_boundaries(vel_x, vel_y, w, h, b);
    return;
  }
  /* Collocated no-op-ish: zero edge normal via bounce on converted MAC. */
  std::vector<float> u, v;
  cell_velocity_to_mac(vel_x, vel_y, w, h, u, v);
  apply_mac_boundaries(u, v, w, h, b);
  mac_velocity_to_cell(u, v, w, h, vel_x, vel_y);
}

inline void enforce_collider_velocity(std::vector<float> &vel_x,
                                      std::vector<float> &vel_y,
                                      const std::vector<float> &mask,
                                      const std::vector<float> &cvel_x,
                                      const std::vector<float> &cvel_y,
                                      const int w,
                                      const int h)
{
  if (int(vel_x.size()) == (w + 1) * h && int(vel_y.size()) == w * (h + 1)) {
    apply_mac_boundaries(vel_x, vel_y, w, h, FluidBounds{}, mask, cvel_x, cvel_y);
    return;
  }
  if (mask.empty() || int(mask.size()) < w * h) {
    return;
  }
  const bool has_c = int(cvel_x.size()) >= w * h && int(cvel_y.size()) >= w * h;
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      const int id = idx2(w, i, j);
      if (mask[size_t(id)] > 0.5f) {
        vel_x[size_t(id)] = has_c ? cvel_x[size_t(id)] : 0.0f;
        vel_y[size_t(id)] = has_c ? cvel_y[size_t(id)] : 0.0f;
      }
    }
  }
}

inline float sample_clamp(const std::vector<float> &f, const int w, const int h, float x, float y)
{
  return sample_cell(f, w, h, x, y);
}

inline float sample_scalar(const std::vector<float> &f,
                           const int w,
                           const int h,
                           float x,
                           float y,
                           const FluidBounds &b)
{
  return sample_cell(f, w, h, x, y, b);
}

inline float sample_uv(const std::vector<float> &f,
                       const int w,
                       const int h,
                       float u,
                       float v,
                       const FluidBounds &b = FluidBounds{})
{
  return sample_cell(f, w, h, u * float(w) - 0.5f, v * float(h) - 0.5f, b);
}

inline void project_velocity(std::vector<float> &vel_x,
                             std::vector<float> &vel_y,
                             const std::vector<float> &pressure,
                             const std::vector<float> &mask,
                             const int w,
                             const int h,
                             const FluidBounds &b = FluidBounds{})
{
  gradient_subtract(vel_x, vel_y, pressure, w, h, b, mask);
  apply_velocity_boundaries(vel_x, vel_y, w, h, b);
}

inline float jacobi_poisson_solve(const std::vector<float> &rhs_in,
                                  const std::vector<float> &mask,
                                  std::vector<float> &p,
                                  const int w,
                                  const int h,
                                  const int iterations,
                                  const FluidBounds &b = FluidBounds{})
{
  p.resize(size_t(w * h), 0.0f);
  std::vector<float> tmp;
  std::vector<float> div = rhs_in;
  jacobi_pressure_mac(p, div, w, h, iterations, tmp, b, mask);
  double acc = 0.0;
  for (int y = 1; y < h - 1; y++) {
    for (int x = 1; x < w - 1; x++) {
      const int i = idx2(w, x, y);
      const float L = p[size_t(idx2(w, x - 1, y))];
      const float R = p[size_t(idx2(w, x + 1, y))];
      const float B = p[size_t(idx2(w, x, y - 1))];
      const float T = p[size_t(idx2(w, x, y + 1))];
      const float r = (L + R + B + T - 4.0f * p[size_t(i)]) - div[size_t(i)];
      acc += double(r) * double(r);
    }
  }
  const int count = std::max((w - 2) * (h - 2), 1);
  return float(std::sqrt(acc / double(count)));
}

inline float multigrid_poisson_solve(const std::vector<float> &rhs_in,
                                     const std::vector<float> &mask,
                                     std::vector<float> &p,
                                     const int w,
                                     const int h,
                                     const int vcycles,
                                     const int pre = 2,
                                     const int post = 2,
                                     const FluidBounds &b = FluidBounds{})
{
  std::vector<float> air; /* no air */
  return multigrid_pressure_mac(
      p, rhs_in, w, h, std::max(vcycles, 1), pre, post, 20, b, mask, air);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name GPU Gems Ch.38 collocated operators (CPU twin of GPU shaders)
 *
 * Same discrete formulas as Listings 38-1…38-5 for unit tests.
 * \{ */

/** Listing 38-2 Jacobi: xNew = (L+R+B+T + alpha*b) * rBeta on interior. */
inline void ch38_jacobi(const std::vector<float> &x,
                        const std::vector<float> &b,
                        const int w,
                        const int h,
                        const float alpha,
                        const float rBeta,
                        std::vector<float> &x_out)
{
  x_out.assign(size_t(w * h), 0.0f);
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      const size_t c = size_t(idx2(w, i, j));
      if (i == 0 || j == 0 || i == w - 1 || j == h - 1) {
        x_out[c] = x[c];
        continue;
      }
      const float xL = x[size_t(idx2(w, i - 1, j))];
      const float xR = x[size_t(idx2(w, i + 1, j))];
      const float xB = x[size_t(idx2(w, i, j - 1))];
      const float xT = x[size_t(idx2(w, i, j + 1))];
      x_out[c] = (xL + xR + xB + xT + alpha * b[c]) * rBeta;
    }
  }
}

/** Listing 38-3: div = halfdx * ((uR-uL)+(vT-vB)). */
inline void ch38_divergence(const std::vector<float> &u,
                            const std::vector<float> &v,
                            const int w,
                            const int h,
                            const float halfdx,
                            std::vector<float> &div)
{
  div.assign(size_t(w * h), 0.0f);
  for (int j = 1; j < h - 1; j++) {
    for (int i = 1; i < w - 1; i++) {
      const float uL = u[size_t(idx2(w, i - 1, j))];
      const float uR = u[size_t(idx2(w, i + 1, j))];
      const float vB = v[size_t(idx2(w, i, j - 1))];
      const float vT = v[size_t(idx2(w, i, j + 1))];
      div[size_t(idx2(w, i, j))] = halfdx * ((uR - uL) + (vT - vB));
    }
  }
}

/** Listing 38-4: u -= halfdx * (pR-pL), v -= halfdx * (pT-pB). */
inline void ch38_gradient_subtract(std::vector<float> &u,
                                   std::vector<float> &v,
                                   const std::vector<float> &p,
                                   const int w,
                                   const int h,
                                   const float halfdx)
{
  for (int j = 1; j < h - 1; j++) {
    for (int i = 1; i < w - 1; i++) {
      const size_t c = size_t(idx2(w, i, j));
      const float pL = p[size_t(idx2(w, i - 1, j))];
      const float pR = p[size_t(idx2(w, i + 1, j))];
      const float pB = p[size_t(idx2(w, i, j - 1))];
      const float pT = p[size_t(idx2(w, i, j + 1))];
      u[c] -= halfdx * (pR - pL);
      v[c] -= halfdx * (pT - pB);
    }
  }
}

/**
 * Listing 38-5 boundary: left/right/bottom/top ghost cells.
 * scale=-1 no-slip velocity; scale=+1 Neumann pressure.
 */
inline void ch38_boundary(std::vector<float> &field, const int w, const int h, const float scale)
{
  if (w < 2 || h < 2) {
    return;
  }
  for (int j = 0; j < h; j++) {
    field[size_t(idx2(w, 0, j))] = scale * field[size_t(idx2(w, 1, j))];
    field[size_t(idx2(w, w - 1, j))] = scale * field[size_t(idx2(w, w - 2, j))];
  }
  for (int i = 0; i < w; i++) {
    field[size_t(idx2(w, i, 0))] = scale * field[size_t(idx2(w, i, 1))];
    field[size_t(idx2(w, i, h - 1))] = scale * field[size_t(idx2(w, i, h - 2))];
  }
}

/** Listing 38-1 semi-Lagrangian advection (bilinear). */
inline void ch38_advect(const std::vector<float> &src,
                        const std::vector<float> &u,
                        const std::vector<float> &v,
                        const int w,
                        const int h,
                        const float dt,
                        const float rdx,
                        std::vector<float> &dst)
{
  auto sample = [&](const float x, const float y) -> float {
    const float px = std::clamp(x, 0.0f, float(w - 1));
    const float py = std::clamp(y, 0.0f, float(h - 1));
    const int i0 = int(std::floor(px));
    const int j0 = int(std::floor(py));
    const int i1 = std::min(i0 + 1, w - 1);
    const int j1 = std::min(j0 + 1, h - 1);
    const float fx = px - float(i0);
    const float fy = py - float(j0);
    const float a = src[size_t(idx2(w, i0, j0))];
    const float b = src[size_t(idx2(w, i1, j0))];
    const float c = src[size_t(idx2(w, i0, j1))];
    const float d = src[size_t(idx2(w, i1, j1))];
    return (1.0f - fx) * (1.0f - fy) * a + fx * (1.0f - fy) * b + (1.0f - fx) * fy * c +
           fx * fy * d;
  };

  dst.assign(size_t(w * h), 0.0f);
  for (int j = 0; j < h; j++) {
    for (int i = 0; i < w; i++) {
      const size_t c = size_t(idx2(w, i, j));
      if (i == 0 || j == 0 || i == w - 1 || j == h - 1) {
        dst[c] = src[c];
        continue;
      }
      const float pos_x = float(i) - dt * rdx * u[c];
      const float pos_y = float(j) - dt * rdx * v[c];
      dst[c] = sample(pos_x, pos_y);
    }
  }
}

/** Full Ch.38 project: div → Jacobi pressure → gradient subtract. */
inline float ch38_project(std::vector<float> &u,
                          std::vector<float> &v,
                          std::vector<float> &p,
                          const int w,
                          const int h,
                          const float dx,
                          const int pressure_iters)
{
  const float halfdx = 0.5f / dx;
  const float alpha = -(dx * dx);
  const float rBeta = 0.25f;
  std::vector<float> div, p_tmp;
  ch38_divergence(u, v, w, h, halfdx, div);
  p.assign(size_t(w * h), 0.0f);
  for (int it = 0; it < pressure_iters; it++) {
    ch38_jacobi(p, div, w, h, alpha, rBeta, p_tmp);
    p.swap(p_tmp);
    ch38_boundary(p, w, h, 1.0f);
  }
  ch38_gradient_subtract(u, v, p, w, h, halfdx);
  ch38_boundary(u, w, h, -1.0f);
  ch38_boundary(v, w, h, -1.0f);
  ch38_divergence(u, v, w, h, halfdx, div);
  double acc = 0.0;
  int cnt = 0;
  for (int j = 1; j < h - 1; j++) {
    for (int i = 1; i < w - 1; i++) {
      const float d = div[size_t(idx2(w, i, j))];
      acc += double(d) * double(d);
      cnt++;
    }
  }
  return float(std::sqrt(acc / double(std::max(cnt, 1))));
}

/** \} */

}  // namespace blender::nodes::image_fluid
