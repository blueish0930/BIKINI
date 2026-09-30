/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: MIT */

#include "pbd/pbd.h"

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
inline float dot(Vec3 a, Vec3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3 cross(Vec3 a, Vec3 b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float length(Vec3 a)
{
  return std::sqrt(dot(a, a));
}
inline Vec3 normalize_or(Vec3 a, Vec3 fallback)
{
  const float l = length(a);
  return l > 1e-8f ? a * (1.0f / l) : fallback;
}

struct Tri {
  Vec3 a, b, c, n;
};

static Vec3 closest_on_triangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c)
{
  const Vec3 ab = b - a;
  const Vec3 ac = c - a;
  const Vec3 ap = p - a;
  const float d1 = dot(ab, ap);
  const float d2 = dot(ac, ap);
  if (d1 <= 0.0f && d2 <= 0.0f) {
    return a;
  }
  const Vec3 bp = p - b;
  const float d3 = dot(ab, bp);
  const float d4 = dot(ac, bp);
  if (d3 >= 0.0f && d4 <= d3) {
    return b;
  }
  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
    const float v = d1 / (d1 - d3);
    return a + ab * v;
  }
  const Vec3 cp = p - c;
  const float d5 = dot(ab, cp);
  const float d6 = dot(ac, cp);
  if (d6 >= 0.0f && d5 <= d6) {
    return c;
  }
  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
    const float w = d2 / (d2 - d6);
    return a + ac * w;
  }
  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
    const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + (c - b) * w;
  }
  const float denom = 1.0f / (va + vb + vc);
  return a + ab * (vb * denom) + ac * (vc * denom);
}

struct Particle {
  Vec3 x, v, pred;
  float inv_mass = 1.0f;
  int pin = 0;
};

struct DistC {
  int a = 0, b = 0;
  float rest = 1.0f;
  float compliance = 0.0f;
  float lambda = 0.0f;
};

}  // namespace

struct PBDWorld {
  std::vector<Particle> particles;
  std::vector<DistC> distances;
  std::vector<DistC> bends;
  std::vector<int> volume_tris;
  float rest_volume = -1.0f;
  float volume_compliance = 0.0f;
  float volume_lambda = 0.0f;
  Vec3 gravity{0.0f, -9.81f, 0.0f};
  int iterations = 10;
  int substeps = 1;
  int ground_enabled = 0;
  int ground_axis = 1;
  float ground_height = 0.0f;
  float ground_friction = 0.4f;
  int self_collision = 1;
  float self_thickness = 0.02f;
  float collision_thickness = 0.02f;
  float collision_friction = 0.4f;
  std::vector<Tri> colliders;
  std::vector<std::vector<int>> adj;
};

static void solve_distance(PBDWorld *w, DistC &c, float dt)
{
  if (c.a < 0 || c.b < 0 || c.a >= int(w->particles.size()) || c.b >= int(w->particles.size())) {
    return;
  }
  Particle &pa = w->particles[c.a];
  Particle &pb = w->particles[c.b];
  const float wa = pa.pin ? 0.0f : pa.inv_mass;
  const float wb = pb.pin ? 0.0f : pb.inv_mass;
  const float wsum = wa + wb;
  if (wsum <= 0.0f) {
    return;
  }
  const Vec3 nraw = pb.x - pa.x;
  const float len = length(nraw);
  if (len < 1e-8f) {
    return;
  }
  const Vec3 n = nraw * (1.0f / len);
  const float C = len - c.rest;
  const float alpha = c.compliance / (dt * dt);
  const float dlambda = (-C - alpha * c.lambda) / (wsum + alpha);
  c.lambda += dlambda;
  pa.x += n * (-dlambda * wa);
  pb.x += n * (dlambda * wb);
}

static float mesh_volume(const PBDWorld *w)
{
  float vol = 0.0f;
  const std::vector<int> &t = w->volume_tris;
  for (size_t i = 0; i + 2 < t.size(); i += 3) {
    const Vec3 a = w->particles[t[i]].x;
    const Vec3 b = w->particles[t[i + 1]].x;
    const Vec3 c = w->particles[t[i + 2]].x;
    vol += dot(a, cross(b, c));
  }
  return vol / 6.0f;
}

static void solve_volume(PBDWorld *w, float dt)
{
  if (w->rest_volume < 0.0f || w->volume_tris.size() < 3) {
    return;
  }
  const float C = mesh_volume(w) - w->rest_volume;
  std::vector<Vec3> grad(w->particles.size(), Vec3{});
  const std::vector<int> &t = w->volume_tris;
  for (size_t i = 0; i + 2 < t.size(); i += 3) {
    const int ia = t[i], ib = t[i + 1], ic = t[i + 2];
    const Vec3 a = w->particles[ia].x;
    const Vec3 b = w->particles[ib].x;
    const Vec3 c = w->particles[ic].x;
    grad[ia] += cross(b, c) * (1.0f / 6.0f);
    grad[ib] += cross(c, a) * (1.0f / 6.0f);
    grad[ic] += cross(a, b) * (1.0f / 6.0f);
  }
  float denom = 0.0f;
  for (size_t i = 0; i < w->particles.size(); i++) {
    const float wi = w->particles[i].pin ? 0.0f : w->particles[i].inv_mass;
    denom += wi * dot(grad[i], grad[i]);
  }
  const float alpha = w->volume_compliance / (dt * dt);
  if (denom + alpha <= 1e-12f) {
    return;
  }
  const float dlambda = (-C - alpha * w->volume_lambda) / (denom + alpha);
  w->volume_lambda += dlambda;
  for (size_t i = 0; i < w->particles.size(); i++) {
    if (w->particles[i].pin) {
      continue;
    }
    w->particles[i].x += grad[i] * (dlambda * w->particles[i].inv_mass);
  }
}

static void collide_ground(PBDWorld *w)
{
  if (!w->ground_enabled) {
    return;
  }
  const int axis = w->ground_axis;
  const float thick = std::max(w->collision_thickness, 0.0f);
  const float plane = w->ground_height + thick;
  for (Particle &p : w->particles) {
    if (p.pin) {
      continue;
    }
    float *comp = axis == 0 ? &p.x.x : (axis == 2 ? &p.x.z : &p.x.y);
    if (*comp < plane) {
      *comp = plane;
      if (axis == 0) {
        p.v.x = std::max(p.v.x, 0.0f);
        p.v.y *= (1.0f - w->ground_friction);
        p.v.z *= (1.0f - w->ground_friction);
      }
      else if (axis == 2) {
        p.v.z = std::max(p.v.z, 0.0f);
        p.v.x *= (1.0f - w->ground_friction);
        p.v.y *= (1.0f - w->ground_friction);
      }
      else {
        p.v.y = std::max(p.v.y, 0.0f);
        p.v.x *= (1.0f - w->ground_friction);
        p.v.z *= (1.0f - w->ground_friction);
      }
    }
  }
}

static long long hash_key(int ix, int iy, int iz)
{
  return (static_cast<long long>(ix) * 73856093LL) ^ (static_cast<long long>(iy) * 19349663LL) ^
         (static_cast<long long>(iz) * 83492791LL);
}

static void collide_mesh(PBDWorld *w)
{
  if (w->colliders.empty()) {
    return;
  }
  const float thick = std::max(w->collision_thickness, 1e-4f);
  const float cell = std::max(thick * 2.0f, 0.05f);
  const float inv = 1.0f / cell;
  std::unordered_map<long long, std::vector<int>> grid;
  auto mark_aabb = [&](Vec3 mn, Vec3 mx, int tri_i) {
    const int x0 = int(std::floor((mn.x - thick) * inv));
    const int y0 = int(std::floor((mn.y - thick) * inv));
    const int z0 = int(std::floor((mn.z - thick) * inv));
    const int x1 = int(std::floor((mx.x + thick) * inv));
    const int y1 = int(std::floor((mx.y + thick) * inv));
    const int z1 = int(std::floor((mx.z + thick) * inv));
    for (int ix = x0; ix <= x1; ix++) {
      for (int iy = y0; iy <= y1; iy++) {
        for (int iz = z0; iz <= z1; iz++) {
          grid[hash_key(ix, iy, iz)].push_back(tri_i);
        }
      }
    }
  };
  for (int t = 0; t < int(w->colliders.size()); t++) {
    const Tri &tri = w->colliders[t];
    Vec3 mn{std::min(tri.a.x, std::min(tri.b.x, tri.c.x)),
            std::min(tri.a.y, std::min(tri.b.y, tri.c.y)),
            std::min(tri.a.z, std::min(tri.b.z, tri.c.z))};
    Vec3 mx{std::max(tri.a.x, std::max(tri.b.x, tri.c.x)),
            std::max(tri.a.y, std::max(tri.b.y, tri.c.y)),
            std::max(tri.a.z, std::max(tri.b.z, tri.c.z))};
    mark_aabb(mn, mx, t);
  }

  for (Particle &p : w->particles) {
    if (p.pin) {
      continue;
    }
    const int ix = int(std::floor(p.x.x * inv));
    const int iy = int(std::floor(p.x.y * inv));
    const int iz = int(std::floor(p.x.z * inv));
    float best = thick;
    Vec3 best_pt = p.x;
    Vec3 best_n{0, 1, 0};
    bool hit = false;
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        for (int dz = -1; dz <= 1; dz++) {
          auto it = grid.find(hash_key(ix + dx, iy + dy, iz + dz));
          if (it == grid.end()) {
            continue;
          }
          for (int ti : it->second) {
            const Tri &tri = w->colliders[ti];
            const Vec3 q = closest_on_triangle(p.x, tri.a, tri.b, tri.c);
            const Vec3 d = p.x - q;
            const float dist = length(d);
            if (dist < best) {
              best = dist;
              best_pt = q;
              best_n = dist > 1e-6f ? d * (1.0f / dist) : tri.n;
              hit = true;
            }
            /* Tunneling: segment pred -> x vs triangle plane. */
            const float d0 = dot(p.pred - tri.a, tri.n);
            const float d1 = dot(p.x - tri.a, tri.n);
            if (d0 * d1 < 0.0f && std::fabs(d0 - d1) > 1e-8f) {
              const float u = d0 / (d0 - d1);
              const Vec3 hitp = p.pred + (p.x - p.pred) * u;
              const Vec3 q2 = closest_on_triangle(hitp, tri.a, tri.b, tri.c);
              if (length(hitp - q2) < thick * 2.0f) {
                best = 0.0f;
                best_pt = q2;
                best_n = d0 > 0.0f ? tri.n : tri.n * -1.0f;
                hit = true;
              }
            }
          }
        }
      }
    }
    if (hit && best < thick) {
      p.x = best_pt + best_n * thick;
      const float vn = dot(p.v, best_n);
      if (vn < 0.0f) {
        p.v = p.v - best_n * vn;
      }
      p.v = p.v * (1.0f - w->collision_friction);
    }
  }
}

static bool is_adjacent(const PBDWorld *w, int a, int b)
{
  if (a < 0 || b < 0 || a >= int(w->adj.size())) {
    return false;
  }
  for (int n : w->adj[a]) {
    if (n == b) {
      return true;
    }
  }
  /* 2-ring: skip grid neighbors that share a vertex but not an edge. */
  for (int n : w->adj[a]) {
    if (n < 0 || n >= int(w->adj.size())) {
      continue;
    }
    for (int m : w->adj[n]) {
      if (m == b) {
        return true;
      }
    }
  }
  return false;
}

static void collide_self(PBDWorld *w)
{
  if (!w->self_collision || w->particles.size() < 2) {
    return;
  }
  const float thick = std::max(w->self_thickness, 1e-4f);
  const float min_d = thick * 2.0f;
  const float min_d2 = min_d * min_d;
  const float cell = min_d;
  const float inv = 1.0f / cell;
  std::unordered_map<long long, std::vector<int>> grid;
  for (int i = 0; i < int(w->particles.size()); i++) {
    const Vec3 p = w->particles[i].x;
    grid[hash_key(int(std::floor(p.x * inv)), int(std::floor(p.y * inv)), int(std::floor(p.z * inv)))]
        .push_back(i);
  }
  for (int i = 0; i < int(w->particles.size()); i++) {
    const Vec3 pi = w->particles[i].x;
    const int ix = int(std::floor(pi.x * inv));
    const int iy = int(std::floor(pi.y * inv));
    const int iz = int(std::floor(pi.z * inv));
    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        for (int dz = -1; dz <= 1; dz++) {
          auto it = grid.find(hash_key(ix + dx, iy + dy, iz + dz));
          if (it == grid.end()) {
            continue;
          }
          for (int j : it->second) {
            if (j <= i || is_adjacent(w, i, j)) {
              continue;
            }
            Vec3 d = w->particles[j].x - w->particles[i].x;
            const float d2 = dot(d, d);
            if (d2 >= min_d2 || d2 < 1e-12f) {
              continue;
            }
            const float dist = std::sqrt(d2);
            const Vec3 n = d * (1.0f / dist);
            /* Keep the push well below a rest-edge length or a dense grid explodes. */
            const float corr = 0.25f * (min_d - dist);
            const float wi = w->particles[i].pin ? 0.0f : 1.0f;
            const float wj = w->particles[j].pin ? 0.0f : 1.0f;
            const float ws = wi + wj;
            if (ws <= 0.0f) {
              continue;
            }
            w->particles[i].x += n * (-corr * (wi / ws));
            w->particles[j].x += n * (corr * (wj / ws));
          }
        }
      }
    }
  }
}

PBDWorld *PBD_world_create(void)
{
  return new PBDWorld();
}

void PBD_world_destroy(PBDWorld *world)
{
  delete world;
}

void PBD_world_clear(PBDWorld *world)
{
  if (!world) {
    return;
  }
  world->particles.clear();
  world->distances.clear();
  world->bends.clear();
  world->volume_tris.clear();
  world->colliders.clear();
  world->adj.clear();
  world->rest_volume = -1.0f;
  world->volume_lambda = 0.0f;
}

void PBD_world_set_gravity(PBDWorld *world, float x, float y, float z)
{
  if (world) {
    world->gravity = {x, y, z};
  }
}

void PBD_world_set_ground(PBDWorld *world, int enabled, int axis, float height, float friction)
{
  if (!world) {
    return;
  }
  world->ground_enabled = enabled;
  world->ground_axis = axis;
  world->ground_height = height;
  world->ground_friction = friction;
}

void PBD_world_set_iterations(PBDWorld *world, int iterations)
{
  if (world) {
    world->iterations = iterations < 1 ? 1 : iterations;
  }
}

void PBD_world_set_substeps(PBDWorld *world, int substeps)
{
  if (world) {
    world->substeps = substeps < 1 ? 1 : substeps;
  }
}

void PBD_world_set_self_collision(PBDWorld *world, int enabled, float thickness)
{
  if (!world) {
    return;
  }
  world->self_collision = enabled;
  world->self_thickness = std::max(thickness, 1e-4f);
}

void PBD_world_set_collision_thickness(PBDWorld *world, float thickness)
{
  if (world) {
    world->collision_thickness = std::max(thickness, 1e-4f);
  }
}

void PBD_world_set_collision_friction(PBDWorld *world, float friction)
{
  if (world) {
    world->collision_friction = std::min(std::max(friction, 0.0f), 1.0f);
  }
}

void PBD_world_clear_colliders(PBDWorld *world)
{
  if (world) {
    world->colliders.clear();
  }
}

void PBD_add_collider_triangle(PBDWorld *world,
                               float ax,
                               float ay,
                               float az,
                               float bx,
                               float by,
                               float bz,
                               float cx,
                               float cy,
                               float cz)
{
  if (!world) {
    return;
  }
  Tri t;
  t.a = {ax, ay, az};
  t.b = {bx, by, bz};
  t.c = {cx, cy, cz};
  t.n = normalize_or(cross(t.b - t.a, t.c - t.a), Vec3{0, 1, 0});
  world->colliders.push_back(t);
}

int PBD_add_particle(PBDWorld *world, float x, float y, float z, float mass, int pinned)
{
  if (!world) {
    return -1;
  }
  Particle p;
  p.x = {x, y, z};
  p.pred = p.x;
  p.v = {};
  p.pin = pinned ? 1 : 0;
  p.inv_mass = (p.pin || mass <= 1e-8f) ? 0.0f : 1.0f / mass;
  world->particles.push_back(p);
  world->adj.emplace_back();
  return int(world->particles.size()) - 1;
}

void PBD_set_velocity(PBDWorld *world, int i, float x, float y, float z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  world->particles[i].v = {x, y, z};
}

int PBD_add_distance(PBDWorld *world, int a, int b, float rest, float compliance)
{
  if (!world) {
    return -1;
  }
  world->distances.push_back({a, b, rest, compliance, 0.0f});
  if (a >= 0 && b >= 0 && a < int(world->adj.size()) && b < int(world->adj.size())) {
    world->adj[a].push_back(b);
    world->adj[b].push_back(a);
  }
  return int(world->distances.size()) - 1;
}

int PBD_add_bending(PBDWorld *world, int a, int b, float rest, float compliance)
{
  if (!world) {
    return -1;
  }
  world->bends.push_back({a, b, rest, compliance, 0.0f});
  return int(world->bends.size()) - 1;
}

void PBD_set_volume_constraint(PBDWorld *world,
                               const int *tri_indices,
                               int tri_count,
                               float rest_volume,
                               float compliance)
{
  if (!world) {
    return;
  }
  world->volume_tris.clear();
  if (tri_indices && tri_count > 0) {
    world->volume_tris.assign(tri_indices, tri_indices + tri_count * 3);
  }
  world->rest_volume = rest_volume;
  world->volume_compliance = compliance;
  world->volume_lambda = 0.0f;
}

void PBD_step(PBDWorld *world, float dt)
{
  if (!world || dt <= 1e-8f || world->particles.empty()) {
    return;
  }
  float min_rest = 1.0e9f;
  for (const DistC &c : world->distances) {
    if (c.rest > 1e-6f) {
      min_rest = std::min(min_rest, c.rest);
    }
  }
  if (min_rest < 1.0e8f) {
    /* Thickness bigger than ~1/3 of an edge is what blows up high-res cloth. */
    world->self_thickness = std::min(world->self_thickness, 0.32f * min_rest);
    world->collision_thickness = std::min(world->collision_thickness, 0.45f * min_rest);
  }
  const int sub = world->substeps < 1 ? 1 : std::min(world->substeps, 16);
  const int iters = world->iterations < 1 ? 1 : std::min(world->iterations, 16);
  const float h = dt / float(sub);
  for (int s = 0; s < sub; s++) {
    for (DistC &c : world->distances) {
      c.lambda = 0.0f;
    }
    for (DistC &c : world->bends) {
      c.lambda = 0.0f;
    }
    world->volume_lambda = 0.0f;

    for (Particle &p : world->particles) {
      if (p.pin || p.inv_mass <= 0.0f) {
        p.pred = p.x;
        continue;
      }
      p.v += world->gravity * h;
      p.pred = p.x;
      p.x += p.v * h;
    }

    for (int it = 0; it < iters; it++) {
      for (DistC &c : world->distances) {
        solve_distance(world, c, h);
      }
      for (DistC &c : world->bends) {
        solve_distance(world, c, h);
      }
      solve_volume(world, h);
    }
    /* Collision once per substep — doing it every XPBD iteration is too slow
     * and over-corrects dense grids. */
    collide_mesh(world);
    collide_self(world);
    collide_ground(world);

    const float inv_h = 1.0f / h;
    for (Particle &p : world->particles) {
      if (p.pin || p.inv_mass <= 0.0f) {
        p.v = {};
        continue;
      }
      p.v = (p.x - p.pred) * inv_h;
    }
  }
}

int PBD_particle_count(const PBDWorld *world)
{
  return world ? int(world->particles.size()) : 0;
}

void PBD_get_position(const PBDWorld *world, int i, float *x, float *y, float *z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  const Vec3 p = world->particles[i].x;
  if (x) {
    *x = p.x;
  }
  if (y) {
    *y = p.y;
  }
  if (z) {
    *z = p.z;
  }
}

void PBD_get_velocity(const PBDWorld *world, int i, float *x, float *y, float *z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  const Vec3 p = world->particles[i].v;
  if (x) {
    *x = p.x;
  }
  if (y) {
    *y = p.y;
  }
  if (z) {
    *z = p.z;
  }
}

float PBD_compute_volume(const PBDWorld *world)
{
  return world ? mesh_volume(world) : 0.0f;
}
