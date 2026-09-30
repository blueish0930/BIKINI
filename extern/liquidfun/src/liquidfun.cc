/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: MIT */

#include "liquidfun/liquidfun.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace {

struct Vec2 {
  float x = 0.0f, y = 0.0f;
};
inline Vec2 operator+(Vec2 a, Vec2 b)
{
  return {a.x + b.x, a.y + b.y};
}
inline Vec2 operator-(Vec2 a, Vec2 b)
{
  return {a.x - b.x, a.y - b.y};
}
inline Vec2 operator*(Vec2 a, float s)
{
  return {a.x * s, a.y * s};
}
inline Vec2 &operator+=(Vec2 &a, Vec2 b)
{
  a = a + b;
  return a;
}
inline float dot(Vec2 a, Vec2 b)
{
  return a.x * b.x + a.y * b.y;
}
inline float length(Vec2 a)
{
  return std::sqrt(dot(a, a));
}

constexpr float PI = 3.14159265358979323846f;

inline float poly6_2(float r2, float h)
{
  const float h2 = h * h;
  if (r2 >= h2) {
    return 0.0f;
  }
  const float x = h2 - r2;
  const float h8 = h2 * h2 * h2 * h2;
  return 4.0f / (PI * h8) * x * x * x;
}

inline Vec2 spiky_grad_2(Vec2 d, float r, float h)
{
  if (r <= 1e-8f || r >= h) {
    return {};
  }
  const float h5 = h * h * h * h * h;
  const float c = -30.0f / (PI * h5) * (h - r) * (h - r);
  return d * (c / r);
}

struct Particle {
  Vec2 x, v, pred, rest;
  float mass = 1.0f;
  float lambda = 0.0f;
  float unused_z = 0.0f;
  int has_rest = 0;
};

struct Spring {
  int a = 0, b = 0;
  float rest = 1.0f;
};

struct Seg {
  Vec2 a, b;
};

}  // namespace

struct LiquidFunWorld {
  std::vector<Particle> particles;
  std::vector<Spring> springs;
  std::vector<Seg> colliders;
  LiquidFunPlane plane = LIQUIDFUN_PLANE_XY;
  Vec2 gravity{0.0f, -9.81f};
  float radius = 0.08f;
  float rest_density = 0.0f;
  float viscosity = 0.03f;
  int iterations = 4;
  int elastic = 0;
  float elastic_stiffness = 0.2f;
  int bounds_enabled = 0;
  Vec2 bmin{-2, -2}, bmax{2, 2};
};

static Vec2 world_to_2d(const LiquidFunWorld *w, float x, float y, float z)
{
  if (w->plane == LIQUIDFUN_PLANE_XZ) {
    return {x, z};
  }
  return {x, y};
}

static void world_from_2d(const LiquidFunWorld *w, Vec2 p, float unused, float *x, float *y, float *z)
{
  if (w->plane == LIQUIDFUN_PLANE_XZ) {
    *x = p.x;
    *y = unused;
    *z = p.y;
  }
  else {
    *x = p.x;
    *y = p.y;
    *z = unused;
  }
}

static void hash_neighbors(const LiquidFunWorld *w,
                           float h,
                           std::vector<std::vector<int>> &neigh)
{
  const int n = int(w->particles.size());
  neigh.assign(n, {});
  const float inv = 1.0f / h;
  std::unordered_map<long long, std::vector<int>> grid;
  auto key = [](int ix, int iy) -> long long {
    return (static_cast<long long>(ix) << 32) ^ static_cast<unsigned>(iy);
  };
  for (int i = 0; i < n; i++) {
    const int ix = int(std::floor(w->particles[i].x.x * inv));
    const int iy = int(std::floor(w->particles[i].x.y * inv));
    grid[key(ix, iy)].push_back(i);
  }
  const float h2 = h * h;
  for (int i = 0; i < n; i++) {
    const int ix = int(std::floor(w->particles[i].x.x * inv));
    const int iy = int(std::floor(w->particles[i].x.y * inv));
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        auto it = grid.find(key(ix + dx, iy + dy));
        if (it == grid.end()) {
          continue;
        }
        for (int j : it->second) {
          if (j <= i) {
            continue;
          }
          const Vec2 d = w->particles[j].x - w->particles[i].x;
          if (dot(d, d) < h2) {
            neigh[i].push_back(j);
            neigh[j].push_back(i);
          }
        }
      }
    }
  }
}

static void clamp_bounds(LiquidFunWorld *w, Particle &p)
{
  if (!w->bounds_enabled) {
    return;
  }
  auto clamp_axis = [](float &x, float &v, float lo, float hi) {
    if (x < lo) {
      x = lo;
      v = v < 0.0f ? -0.3f * v : v;
    }
    else if (x > hi) {
      x = hi;
      v = v > 0.0f ? -0.3f * v : v;
    }
  };
  clamp_axis(p.x.x, p.v.x, w->bmin.x, w->bmax.x);
  clamp_axis(p.x.y, p.v.y, w->bmin.y, w->bmax.y);
}

static Vec2 closest_on_seg(Vec2 p, Vec2 a, Vec2 b)
{
  const Vec2 ab = b - a;
  const float denom = dot(ab, ab);
  if (denom < 1e-12f) {
    return a;
  }
  float t = dot(p - a, ab) / denom;
  t = std::max(0.0f, std::min(1.0f, t));
  return a + ab * t;
}

static void collide_segments(LiquidFunWorld *w)
{
  if (w->colliders.empty()) {
    return;
  }
  const float thick = std::max(w->radius * 0.55f, 1e-4f);
  for (Particle &p : w->particles) {
    for (const Seg &s : w->colliders) {
      const Vec2 q = closest_on_seg(p.x, s.a, s.b);
      const Vec2 d = p.x - q;
      const float dist = length(d);
      if (dist < thick) {
        const Vec2 n = dist > 1e-6f ? d * (1.0f / dist) : Vec2{0.0f, 1.0f};
        p.x = q + n * thick;
        const float vn = dot(p.v, n);
        if (vn < 0.0f) {
          p.v = p.v - n * vn;
        }
      }
    }
  }
}

LiquidFunWorld *LF_world_create(void)
{
  return new LiquidFunWorld();
}
void LF_world_destroy(LiquidFunWorld *world)
{
  delete world;
}
void LF_world_clear(LiquidFunWorld *world)
{
  if (!world) {
    return;
  }
  world->particles.clear();
  world->springs.clear();
}
void LF_world_set_plane(LiquidFunWorld *world, LiquidFunPlane plane)
{
  if (world) {
    world->plane = plane;
  }
}
void LF_world_set_gravity(LiquidFunWorld *world, float x, float y, float z)
{
  if (!world) {
    return;
  }
  world->gravity = world_to_2d(world, x, y, z);
}
void LF_world_set_radius(LiquidFunWorld *world, float radius)
{
  if (world) {
    world->radius = std::max(radius, 1e-5f);
  }
}
void LF_world_set_rest_density(LiquidFunWorld *world, float density)
{
  if (world) {
    world->rest_density = density;
  }
}
float LF_world_get_rest_density(const LiquidFunWorld *world)
{
  return world ? world->rest_density : 0.0f;
}
float LF_calibrate_rest_density(LiquidFunWorld *world)
{
  if (!world || world->particles.empty()) {
    return 0.0f;
  }
  const float h = world->radius * 2.2f;
  std::vector<std::vector<int>> neigh;
  hash_neighbors(world, h, neigh);
  std::vector<float> rhos;
  rhos.reserve(world->particles.size());
  for (size_t i = 0; i < world->particles.size(); i++) {
    float rho = world->particles[i].mass * poly6_2(0.0f, h);
    for (int j : neigh[i]) {
      const Vec2 d = world->particles[i].x - world->particles[j].x;
      rho += world->particles[j].mass * poly6_2(dot(d, d), h);
    }
    rhos.push_back(rho);
  }
  /* Mean of all particles is biased low by the free surface, so PBF treats
   * the interior as over-dense and the blob inflates until it fills the tank.
   * Use the 80th percentile (interior particles). */
  std::sort(rhos.begin(), rhos.end());
  const int idx = std::clamp(int(rhos.size() * 0.80f), 0, int(rhos.size()) - 1);
  const float rho0 = std::max(rhos[idx], 1e-3f);
  world->rest_density = rho0;
  return rho0;
}
void LF_world_set_viscosity(LiquidFunWorld *world, float viscosity)
{
  if (world) {
    world->viscosity = std::max(viscosity, 0.0f);
  }
}
void LF_world_set_iterations(LiquidFunWorld *world, int iterations)
{
  if (world) {
    world->iterations = iterations < 1 ? 1 : iterations;
  }
}
void LF_world_set_elastic(LiquidFunWorld *world, int enabled, float stiffness)
{
  if (!world) {
    return;
  }
  world->elastic = enabled;
  world->elastic_stiffness = std::max(stiffness, 0.0f);
}
void LF_world_set_bounds(LiquidFunWorld *world,
                         float min0,
                         float min1,
                         float max0,
                         float max1,
                         int enabled)
{
  if (!world) {
    return;
  }
  world->bounds_enabled = enabled;
  world->bmin = {min0, min1};
  world->bmax = {max0, max1};
}

void LF_add_collider_segment(LiquidFunWorld *world, float ax, float ay, float bx, float by)
{
  if (!world) {
    return;
  }
  world->colliders.push_back(Seg{{ax, ay}, {bx, by}});
}

int LF_add_particle(LiquidFunWorld *world, float x, float y, float z, float mass)
{
  if (!world) {
    return -1;
  }
  Particle p;
  p.x = world_to_2d(world, x, y, z);
  p.pred = p.x;
  p.rest = p.x;
  p.has_rest = 1;
  p.mass = mass > 1e-8f ? mass : 1.0f;
  p.unused_z = (world->plane == LIQUIDFUN_PLANE_XZ) ? y : z;
  world->particles.push_back(p);
  return int(world->particles.size()) - 1;
}

void LF_set_velocity(LiquidFunWorld *world, int i, float x, float y, float z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  world->particles[i].v = world_to_2d(world, x, y, z);
}

void LF_set_rest_position(LiquidFunWorld *world, int i, float x, float y, float z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  world->particles[i].rest = world_to_2d(world, x, y, z);
  world->particles[i].has_rest = 1;
}

void LF_capture_elastic_rest(LiquidFunWorld *world)
{
  if (!world) {
    return;
  }
  world->springs.clear();
  if (!world->elastic) {
    return;
  }
  const float h = world->radius * 2.2f;
  /* Neighbor query in rest pose so springs survive later frames. */
  std::vector<Vec2> saved;
  saved.reserve(world->particles.size());
  for (Particle &p : world->particles) {
    saved.push_back(p.x);
    if (p.has_rest) {
      p.x = p.rest;
    }
  }
  std::vector<std::vector<int>> neigh;
  hash_neighbors(world, h, neigh);
  for (size_t i = 0; i < world->particles.size(); i++) {
    for (int j : neigh[i]) {
      if (j <= int(i)) {
        continue;
      }
      const float rest = length(world->particles[j].x - world->particles[i].x);
      if (rest > 1e-6f && rest < h) {
        world->springs.push_back({int(i), j, rest});
      }
    }
  }
  for (size_t i = 0; i < world->particles.size(); i++) {
    world->particles[i].x = saved[i];
  }
}

void LF_step(LiquidFunWorld *world, float dt)
{
  if (!world || dt <= 1e-8f || world->particles.empty()) {
    return;
  }
  if (world->rest_density <= 1e-6f) {
    LF_calibrate_rest_density(world);
  }
  const float h = world->radius * 2.2f;
  const float rho0 = std::max(world->rest_density, 1e-3f);
  const float vmax = 0.4f * h / dt;
  const float max_corr = 0.2f * h;

  for (Particle &p : world->particles) {
    p.v += world->gravity * dt;
    const float sp = length(p.v);
    if (sp > vmax) {
      p.v = p.v * (vmax / sp);
    }
    p.pred = p.x;
    p.x += p.v * dt;
    clamp_bounds(world, p);
  }
  collide_segments(world);

  std::vector<std::vector<int>> neigh;
  for (int it = 0; it < world->iterations; it++) {
    hash_neighbors(world, h, neigh);
    std::vector<float> density(world->particles.size(), 0.0f);
    for (size_t i = 0; i < world->particles.size(); i++) {
      float rho = world->particles[i].mass * poly6_2(0.0f, h);
      for (int j : neigh[i]) {
        const Vec2 d = world->particles[i].x - world->particles[j].x;
        rho += world->particles[j].mass * poly6_2(dot(d, d), h);
      }
      density[i] = std::max(rho, 1e-6f);
    }
    for (size_t i = 0; i < world->particles.size(); i++) {
      float C = density[i] / rho0 - 1.0f;
      C = std::clamp(C, -0.5f, 2.0f);
      float sum = 0.0f;
      Vec2 gi{};
      for (int j : neigh[i]) {
        const Vec2 d = world->particles[i].x - world->particles[j].x;
        const Vec2 g = spiky_grad_2(d, length(d), h) * (world->particles[j].mass / rho0);
        sum += dot(g, g);
        gi += g;
      }
      sum += dot(gi, gi);
      float lam = -C / (sum + 1.0e-4f);
      world->particles[i].lambda = std::clamp(lam, -400.0f, 400.0f);
    }
    for (size_t i = 0; i < world->particles.size(); i++) {
      Vec2 dp{};
      for (int j : neigh[i]) {
        const Vec2 d = world->particles[i].x - world->particles[j].x;
        const float r = length(d);
        /* Macklin tensile term keeps the surface from clumping / exploding. */
        const float w = poly6_2(r * r, h);
        const float wq = poly6_2((0.2f * h) * (0.2f * h), h);
        float scorr = 0.0f;
        if (wq > 1e-8f) {
          const float ratio = w / wq;
          scorr = -0.01f * ratio * ratio * ratio * ratio;
        }
        dp += spiky_grad_2(d, r, h) *
              ((world->particles[i].lambda + world->particles[j].lambda + scorr) *
               (world->particles[j].mass / rho0));
      }
      const float dpl = length(dp);
      if (dpl > max_corr) {
        dp = dp * (max_corr / dpl);
      }
      world->particles[i].x += dp;
      clamp_bounds(world, world->particles[i]);
    }
    if (world->elastic && !world->springs.empty()) {
      const float k = std::min(world->elastic_stiffness, 0.35f);
      for (const Spring &s : world->springs) {
        Particle &a = world->particles[s.a];
        Particle &b = world->particles[s.b];
        const Vec2 d = b.x - a.x;
        const float len = length(d);
        if (len < 1e-8f) {
          continue;
        }
        const Vec2 n = d * (1.0f / len);
        float corr = (len - s.rest) * 0.5f * k;
        corr = std::clamp(corr, -max_corr, max_corr);
        a.x += n * -corr;
        b.x += n * corr;
      }
    }
  }

  const float inv_dt = 1.0f / dt;
  hash_neighbors(world, h, neigh);
  for (size_t i = 0; i < world->particles.size(); i++) {
    Particle &p = world->particles[i];
    p.v = (p.x - p.pred) * inv_dt;
    Vec2 visc{};
    float wsum = 0.0f;
    for (int j : neigh[i]) {
      const Vec2 d = world->particles[j].x - p.x;
      const float w = poly6_2(dot(d, d), h);
      visc += (world->particles[j].v - p.v) * w;
      wsum += w;
    }
    if (wsum > 1e-8f) {
      p.v += visc * (world->viscosity / wsum);
    }
    const float sp = length(p.v);
    if (sp > vmax) {
      p.v = p.v * (vmax / sp);
    }
  }
}

int LF_particle_count(const LiquidFunWorld *world)
{
  return world ? int(world->particles.size()) : 0;
}
void LF_get_position(const LiquidFunWorld *world, int i, float *x, float *y, float *z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  float xx, yy, zz;
  world_from_2d(world, world->particles[i].x, world->particles[i].unused_z, &xx, &yy, &zz);
  if (x) {
    *x = xx;
  }
  if (y) {
    *y = yy;
  }
  if (z) {
    *z = zz;
  }
}
void LF_get_velocity(const LiquidFunWorld *world, int i, float *x, float *y, float *z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  float xx, yy, zz;
  world_from_2d(world, world->particles[i].v, 0.0f, &xx, &yy, &zz);
  if (x) {
    *x = xx;
  }
  if (y) {
    *y = yy;
  }
  if (z) {
    *z = zz;
  }
}
