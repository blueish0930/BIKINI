/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: MIT */

#include "ipc/ipc.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

namespace {

struct Vec3 {
  float x = 0.0f, y = 0.0f, z = 0.0f;
};
inline Vec3 operator+(Vec3 a, Vec3 b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vec3 operator-(Vec3 a, Vec3 b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vec3 operator*(Vec3 a, float s)
{
  return {a.x * s, a.y * s, a.z * s};
}
inline Vec3 operator*(float s, Vec3 a)
{
  return a * s;
}
inline Vec3 &operator+=(Vec3 &a, Vec3 b)
{
  a = a + b;
  return a;
}
inline Vec3 &operator-=(Vec3 &a, Vec3 b)
{
  a = a - b;
  return a;
}
inline float dot(Vec3 a, Vec3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline float length(Vec3 a)
{
  return std::sqrt(dot(a, a));
}

/* IPC barrier b(d) = -(d-dhat)^2 * log(d/dhat),  0 < d < dhat.
 * b'(d) = -2(d-dhat)log(d/dhat) - (d-dhat)^2 / d */
inline float barrier_grad(float d, float dhat)
{
  if (d <= 1e-8f || d >= dhat) {
    return 0.0f;
  }
  const float t = d - dhat;
  return -2.0f * t * std::log(d / dhat) - t * t / d;
}

struct Particle {
  Vec3 x, v, xhat;
  float mass = 1.0f;
  int pin = 0;
};

struct Edge {
  int a = 0, b = 0;
  float rest = 1.0f;
};

}  // namespace

struct IPCWorld {
  std::vector<Particle> particles;
  std::vector<Edge> edges;
  Vec3 gravity{0.0f, -9.81f, 0.0f};
  float thickness = 0.02f;
  float barrier_stiffness = 1000.0f;
  float stretch_stiffness = 10000.0f;
  int iterations = 12;
  int ground_enabled = 0;
  int ground_axis = 1;
  float ground_height = 0.0f;
};

static void accumulate_energy_grad(IPCWorld *w, std::vector<Vec3> &g, float dt)
{
  const float inv_dt2 = 1.0f / (dt * dt);
  const int n = int(w->particles.size());
  g.assign(n, Vec3{});

  /* Inertia: 0.5 m / dt^2 ||x - xhat||^2 */
  for (int i = 0; i < n; i++) {
    const Particle &p = w->particles[i];
    if (p.pin) {
      continue;
    }
    g[i] += (p.x - p.xhat) * (p.mass * inv_dt2);
  }

  /* Stretch springs. */
  for (const Edge &e : w->edges) {
    Particle &pa = w->particles[e.a];
    Particle &pb = w->particles[e.b];
    const Vec3 d = pb.x - pa.x;
    const float len = length(d);
    if (len < 1e-8f) {
      continue;
    }
    const float C = len - e.rest;
    const Vec3 n = d * (1.0f / len);
    const Vec3 f = n * (w->stretch_stiffness * C);
    if (!pa.pin) {
      g[e.a] -= f;
    }
    if (!pb.pin) {
      g[e.b] += f;
    }
  }

  /* Particle-particle IPC barrier. */
  const float dhat = std::max(w->thickness, 1e-5f);
  const float inv = 1.0f / dhat;
  std::unordered_map<long long, std::vector<int>> grid;
  auto key = [](int ix, int iy, int iz) -> long long {
    return (static_cast<long long>(ix) * 73856093LL) ^
           (static_cast<long long>(iy) * 19349663LL) ^
           (static_cast<long long>(iz) * 83492791LL);
  };
  for (int i = 0; i < n; i++) {
    const int ix = int(std::floor(w->particles[i].x.x * inv));
    const int iy = int(std::floor(w->particles[i].x.y * inv));
    const int iz = int(std::floor(w->particles[i].x.z * inv));
    grid[key(ix, iy, iz)].push_back(i);
  }
  for (int i = 0; i < n; i++) {
    const int ix = int(std::floor(w->particles[i].x.x * inv));
    const int iy = int(std::floor(w->particles[i].x.y * inv));
    const int iz = int(std::floor(w->particles[i].x.z * inv));
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        for (int dz = -1; dz <= 1; dz++) {
          auto it = grid.find(key(ix + dx, iy + dy, iz + dz));
          if (it == grid.end()) {
            continue;
          }
          for (int j : it->second) {
            if (j <= i) {
              continue;
            }
            const Vec3 d = w->particles[j].x - w->particles[i].x;
            const float dist = length(d);
            if (dist <= 1e-8f || dist >= dhat) {
              continue;
            }
            const float bp = barrier_grad(dist, dhat) * w->barrier_stiffness;
            const Vec3 n = d * (1.0f / dist);
            if (!w->particles[i].pin) {
              g[i] += n * (-bp);
            }
            if (!w->particles[j].pin) {
              g[j] += n * bp;
            }
          }
        }
      }
    }
  }

  /* Ground barrier. */
  if (w->ground_enabled) {
    for (int i = 0; i < n; i++) {
      if (w->particles[i].pin) {
        continue;
      }
      float coord = w->ground_axis == 0 ? w->particles[i].x.x :
                    (w->ground_axis == 2 ? w->particles[i].x.z : w->particles[i].x.y);
      const float dist = coord - w->ground_height;
      if (dist <= 1e-8f || dist >= dhat) {
        continue;
      }
      const float bp = barrier_grad(dist, dhat) * w->barrier_stiffness;
      if (w->ground_axis == 0) {
        g[i].x += bp;
      }
      else if (w->ground_axis == 2) {
        g[i].z += bp;
      }
      else {
        g[i].y += bp;
      }
    }
  }
}

IPCWorld *IPC_world_create(void)
{
  return new IPCWorld();
}
void IPC_world_destroy(IPCWorld *world)
{
  delete world;
}
void IPC_world_clear(IPCWorld *world)
{
  if (!world) {
    return;
  }
  world->particles.clear();
  world->edges.clear();
}
void IPC_world_set_gravity(IPCWorld *world, float x, float y, float z)
{
  if (world) {
    world->gravity = {x, y, z};
  }
}
void IPC_world_set_ground(IPCWorld *world, int enabled, int axis, float height)
{
  if (!world) {
    return;
  }
  world->ground_enabled = enabled;
  world->ground_axis = axis;
  world->ground_height = height;
}
void IPC_world_set_params(IPCWorld *world,
                          float thickness,
                          float barrier_stiffness,
                          float stretch_stiffness,
                          int iterations)
{
  if (!world) {
    return;
  }
  world->thickness = std::max(thickness, 1e-5f);
  world->barrier_stiffness = std::max(barrier_stiffness, 0.0f);
  world->stretch_stiffness = std::max(stretch_stiffness, 0.0f);
  world->iterations = iterations < 1 ? 1 : iterations;
}

int IPC_add_particle(IPCWorld *world, float x, float y, float z, float mass, int pinned)
{
  if (!world) {
    return -1;
  }
  Particle p;
  p.x = {x, y, z};
  p.xhat = p.x;
  p.mass = mass > 1e-8f ? mass : 1.0f;
  p.pin = pinned ? 1 : 0;
  world->particles.push_back(p);
  return int(world->particles.size()) - 1;
}

void IPC_set_velocity(IPCWorld *world, int i, float x, float y, float z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  world->particles[i].v = {x, y, z};
}

int IPC_add_edge(IPCWorld *world, int a, int b, float rest)
{
  if (!world) {
    return -1;
  }
  world->edges.push_back({a, b, rest});
  return int(world->edges.size()) - 1;
}

void IPC_step(IPCWorld *world, float dt)
{
  if (!world || dt <= 1e-8f || world->particles.empty()) {
    return;
  }
  for (Particle &p : world->particles) {
    if (p.pin) {
      p.xhat = p.x;
      p.v = {};
      continue;
    }
    p.v += world->gravity * dt;
    p.xhat = p.x + p.v * dt;
    p.x = p.xhat;
  }

  /* Fixed-step GD with k~1e4 overshoots and NaNs the first time a pin
   * creates a large stretch residual. Clamp the displacement per iteration. */
  const float max_dx = std::max(world->thickness * 2.0f, 0.015f);
  std::vector<Vec3> grad;
  for (int it = 0; it < world->iterations; it++) {
    accumulate_energy_grad(world, grad, dt);
    const float step = 0.06f / (1.0f + 0.25f * float(it));
    for (size_t i = 0; i < world->particles.size(); i++) {
      Particle &p = world->particles[i];
      if (p.pin) {
        continue;
      }
      Vec3 dx = grad[i] * (step / std::max(p.mass, 1e-6f));
      const float L = length(dx);
      if (!std::isfinite(L)) {
        p.x = p.xhat;
        continue;
      }
      if (L > max_dx) {
        dx = dx * (max_dx / L);
      }
      p.x.x -= dx.x;
      p.x.y -= dx.y;
      p.x.z -= dx.z;
      if (!std::isfinite(p.x.x) || !std::isfinite(p.x.y) || !std::isfinite(p.x.z)) {
        p.x = p.xhat;
      }
    }
  }

  const float inv_dt = 1.0f / dt;
  for (Particle &p : world->particles) {
    if (p.pin) {
      p.v = {};
      continue;
    }
    p.v = (p.x - (p.xhat - p.v * dt)) * inv_dt;
    if (!std::isfinite(p.v.x) || !std::isfinite(p.v.y) || !std::isfinite(p.v.z)) {
      p.v = {};
    }
  }

  /* Hard ground: the barrier only acts in (0, dhat) and a single implicit
   * predict step can tunnel through that slab. */
  if (world->ground_enabled) {
    const float floor = world->ground_height + 1e-4f;
    for (Particle &p : world->particles) {
      if (p.pin) {
        continue;
      }
      float *coord = world->ground_axis == 0 ? &p.x.x :
                     (world->ground_axis == 2 ? &p.x.z : &p.x.y);
      float *vel = world->ground_axis == 0 ? &p.v.x :
                   (world->ground_axis == 2 ? &p.v.z : &p.v.y);
      if (*coord < floor) {
        *coord = floor;
        if (*vel < 0.0f) {
          *vel = 0.0f;
        }
      }
    }
  }
}

int IPC_particle_count(const IPCWorld *world)
{
  return world ? int(world->particles.size()) : 0;
}
void IPC_get_position(const IPCWorld *world, int i, float *x, float *y, float *z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  if (x) {
    *x = world->particles[i].x.x;
  }
  if (y) {
    *y = world->particles[i].x.y;
  }
  if (z) {
    *z = world->particles[i].x.z;
  }
}
void IPC_get_velocity(const IPCWorld *world, int i, float *x, float *y, float *z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  if (x) {
    *x = world->particles[i].v.x;
  }
  if (y) {
    *y = world->particles[i].v.y;
  }
  if (z) {
    *z = world->particles[i].v.z;
  }
}
