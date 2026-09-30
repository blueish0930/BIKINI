/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: MIT */

#include "mpm/mpm.h"

#include <algorithm>
#include <cmath>
#include <cstring>
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
inline Vec3 &operator+=(Vec3 &a, Vec3 b)
{
  a = a + b;
  return a;
}

struct Mat3 {
  float m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

inline Mat3 mat_zero()
{
  Mat3 a;
  for (int i = 0; i < 9; i++) {
    a.m[i] = 0.0f;
  }
  return a;
}
inline Mat3 mat_identity()
{
  return Mat3{};
}
inline Mat3 mat_add(Mat3 a, Mat3 b)
{
  Mat3 c;
  for (int i = 0; i < 9; i++) {
    c.m[i] = a.m[i] + b.m[i];
  }
  return c;
}
inline Mat3 mat_scale(Mat3 a, float s)
{
  Mat3 c;
  for (int i = 0; i < 9; i++) {
    c.m[i] = a.m[i] * s;
  }
  return c;
}
inline Mat3 mat_mul(Mat3 a, Mat3 b)
{
  Mat3 c = mat_zero();
  for (int r = 0; r < 3; r++) {
    for (int col = 0; col < 3; col++) {
      c.m[r * 3 + col] = a.m[r * 3 + 0] * b.m[0 * 3 + col] + a.m[r * 3 + 1] * b.m[1 * 3 + col] +
                         a.m[r * 3 + 2] * b.m[2 * 3 + col];
    }
  }
  return c;
}
inline float mat_det(Mat3 a)
{
  return a.m[0] * (a.m[4] * a.m[8] - a.m[5] * a.m[7]) -
         a.m[1] * (a.m[3] * a.m[8] - a.m[5] * a.m[6]) +
         a.m[2] * (a.m[3] * a.m[7] - a.m[4] * a.m[6]);
}
inline Mat3 mat_transpose(Mat3 a)
{
  Mat3 c;
  c.m[0] = a.m[0];
  c.m[1] = a.m[3];
  c.m[2] = a.m[6];
  c.m[3] = a.m[1];
  c.m[4] = a.m[4];
  c.m[5] = a.m[7];
  c.m[6] = a.m[2];
  c.m[7] = a.m[5];
  c.m[8] = a.m[8];
  return c;
}
inline Mat3 outer(Vec3 a, Vec3 b)
{
  Mat3 c;
  c.m[0] = a.x * b.x;
  c.m[1] = a.x * b.y;
  c.m[2] = a.x * b.z;
  c.m[3] = a.y * b.x;
  c.m[4] = a.y * b.y;
  c.m[5] = a.y * b.z;
  c.m[6] = a.z * b.x;
  c.m[7] = a.z * b.y;
  c.m[8] = a.z * b.z;
  return c;
}
inline Vec3 mat_mul_vec(Mat3 a, Vec3 v)
{
  return {a.m[0] * v.x + a.m[1] * v.y + a.m[2] * v.z,
          a.m[3] * v.x + a.m[4] * v.y + a.m[5] * v.z,
          a.m[6] * v.x + a.m[7] * v.y + a.m[8] * v.z};
}

/* Polar-ish extraction via Newton on A^T A is overkill; use a cheap fixed-corotated
 * via SVD-free approximation: polar iteration. */
inline Mat3 polar_r(Mat3 F)
{
  Mat3 R = F;
  for (int it = 0; it < 4; it++) {
    const float det = mat_det(R);
    if (std::fabs(det) < 1e-12f) {
      return mat_identity();
    }
    /* R <- 0.5 (R + adj(R)^T)  (simplified Cayley) */
    Mat3 adj;
    adj.m[0] = R.m[4] * R.m[8] - R.m[5] * R.m[7];
    adj.m[1] = R.m[2] * R.m[7] - R.m[1] * R.m[8];
    adj.m[2] = R.m[1] * R.m[5] - R.m[2] * R.m[4];
    adj.m[3] = R.m[5] * R.m[6] - R.m[3] * R.m[8];
    adj.m[4] = R.m[0] * R.m[8] - R.m[2] * R.m[6];
    adj.m[5] = R.m[2] * R.m[3] - R.m[0] * R.m[5];
    adj.m[6] = R.m[3] * R.m[7] - R.m[4] * R.m[6];
    adj.m[7] = R.m[1] * R.m[6] - R.m[0] * R.m[7];
    adj.m[8] = R.m[0] * R.m[4] - R.m[1] * R.m[3];
    const float invdet = 1.0f / det;
    for (int i = 0; i < 9; i++) {
      R.m[i] = 0.5f * (R.m[i] + adj.m[i] * invdet);
    }
  }
  return R;
}

struct Particle {
  Vec3 x, v;
  Mat3 F = mat_identity();
  Mat3 C = mat_zero();
  float mass = 1.0f;
  float volume = 1.0f;
  float J = 1.0f;
};

struct GridNode {
  float mass = 0.0f;
  Vec3 v{};
};

}  // namespace

struct ColTri {
  Vec3 a, b, c, n;
};

struct MPMWorld {
  std::vector<Particle> particles;
  std::vector<ColTri> colliders;
  Vec3 gravity{0.0f, -9.81f, 0.0f};
  float dx = 0.1f;
  MPMMaterial material = MPM_MATERIAL_ELASTIC;
  float young = 1.0e5f;
  float poisson = 0.3f;
  int bounds_enabled = 0;
  Vec3 bmin{-1, -1, -1}, bmax{1, 1, 1};
};

static void lame(const MPMWorld *w, float &mu, float &lambda)
{
  const float e = w->young;
  const float nu = std::min(std::max(w->poisson, 0.0f), 0.49f);
  mu = e / (2.0f * (1.0f + nu));
  lambda = e * nu / ((1.0f + nu) * (1.0f - 2.0f * nu));
}

MPMWorld *MPM_world_create(void)
{
  return new MPMWorld();
}
void MPM_world_destroy(MPMWorld *world)
{
  delete world;
}
void MPM_world_clear(MPMWorld *world)
{
  if (world) {
    world->particles.clear();
  }
}
void MPM_world_set_gravity(MPMWorld *world, float x, float y, float z)
{
  if (world) {
    world->gravity = {x, y, z};
  }
}
void MPM_world_set_dx(MPMWorld *world, float dx)
{
  if (world) {
    world->dx = std::max(dx, 1e-4f);
  }
}
void MPM_world_set_material(MPMWorld *world, MPMMaterial material)
{
  if (world) {
    world->material = material;
  }
}
void MPM_world_set_lame(MPMWorld *world, float young, float poisson)
{
  if (!world) {
    return;
  }
  world->young = std::max(young, 1.0f);
  world->poisson = poisson;
}
void MPM_world_set_bounds(MPMWorld *world,
                          float minx,
                          float miny,
                          float minz,
                          float maxx,
                          float maxy,
                          float maxz,
                          int enabled)
{
  if (!world) {
    return;
  }
  world->bounds_enabled = enabled;
  world->bmin = {minx, miny, minz};
  world->bmax = {maxx, maxy, maxz};
}

void MPM_add_collider_triangle(MPMWorld *world,
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
  ColTri t;
  t.a = {ax, ay, az};
  t.b = {bx, by, bz};
  t.c = {cx, cy, cz};
  const Vec3 ab = t.b - t.a;
  const Vec3 ac = t.c - t.a;
  Vec3 n{ab.y * ac.z - ab.z * ac.y, ab.z * ac.x - ab.x * ac.z, ab.x * ac.y - ab.y * ac.x};
  const float ln = std::sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
  t.n = ln > 1e-8f ? n * (1.0f / ln) : Vec3{0, 0, 1};
  world->colliders.push_back(t);
}

int MPM_add_particle(MPMWorld *world, float x, float y, float z, float mass, float volume)
{
  if (!world) {
    return -1;
  }
  Particle p;
  p.x = {x, y, z};
  p.mass = mass > 1e-8f ? mass : 1.0f;
  p.volume = volume > 1e-8f ? volume : 1.0f;
  world->particles.push_back(p);
  return int(world->particles.size()) - 1;
}

void MPM_set_velocity(MPMWorld *world, int i, float x, float y, float z)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  world->particles[i].v = {x, y, z};
}
void MPM_set_F(MPMWorld *world, int i, const float F[9])
{
  if (!world || i < 0 || i >= int(world->particles.size()) || !F) {
    return;
  }
  std::memcpy(world->particles[i].F.m, F, 9 * sizeof(float));
}
void MPM_set_C(MPMWorld *world, int i, const float C[9])
{
  if (!world || i < 0 || i >= int(world->particles.size()) || !C) {
    return;
  }
  std::memcpy(world->particles[i].C.m, C, 9 * sizeof(float));
}
void MPM_set_J(MPMWorld *world, int i, float J)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return;
  }
  world->particles[i].J = J;
}

void MPM_step(MPMWorld *world, float dt)
{
  if (!world || dt <= 1e-8f || world->particles.empty()) {
    return;
  }
  const float dx = world->dx;
  const float inv_dx = 1.0f / dx;
  float mu, lambda;
  lame(world, mu, lambda);

  Vec3 gmin = world->particles[0].x;
  Vec3 gmax = gmin;
  for (const Particle &p : world->particles) {
    gmin.x = std::min(gmin.x, p.x.x);
    gmin.y = std::min(gmin.y, p.x.y);
    gmin.z = std::min(gmin.z, p.x.z);
    gmax.x = std::max(gmax.x, p.x.x);
    gmax.y = std::max(gmax.y, p.x.y);
    gmax.z = std::max(gmax.z, p.x.z);
  }
  const float pad = dx * 4.0f;
  gmin = gmin - Vec3{pad, pad, pad};
  gmax = gmax + Vec3{pad, pad, pad};
  int nx = std::max(4, int(std::ceil((gmax.x - gmin.x) * inv_dx)) + 1);
  int ny = std::max(4, int(std::ceil((gmax.y - gmin.y) * inv_dx)) + 1);
  int nz = std::max(4, int(std::ceil((gmax.z - gmin.z) * inv_dx)) + 1);
  constexpr int kMaxDim = 40;
  if (nx > kMaxDim || ny > kMaxDim || nz > kMaxDim) {
    const float sx = (gmax.x - gmin.x) / float(kMaxDim);
    const float sy = (gmax.y - gmin.y) / float(kMaxDim);
    const float sz = (gmax.z - gmin.z) / float(kMaxDim);
    const float dx_cap = std::max(dx, std::max(sx, std::max(sy, sz)));
    const float inv_cap = 1.0f / dx_cap;
    nx = std::max(4, std::min(kMaxDim, int(std::ceil((gmax.x - gmin.x) * inv_cap)) + 1));
    ny = std::max(4, std::min(kMaxDim, int(std::ceil((gmax.y - gmin.y) * inv_cap)) + 1));
    nz = std::max(4, std::min(kMaxDim, int(std::ceil((gmax.z - gmin.z) * inv_cap)) + 1));
  }
  const int ngrid = nx * ny * nz;
  std::vector<GridNode> grid(ngrid);

  auto gindex = [&](int i, int j, int k) { return (k * ny + j) * nx + i; };

  /* P2G */
  for (Particle &p : world->particles) {
    const float fx = (p.x.x - gmin.x) * inv_dx;
    const float fy = (p.x.y - gmin.y) * inv_dx;
    const float fz = (p.x.z - gmin.z) * inv_dx;
    const int bx = int(std::floor(fx));
    const int by = int(std::floor(fy));
    const int bz = int(std::floor(fz));

    Mat3 stress = mat_zero();
    if (world->material == MPM_MATERIAL_FLUID) {
      const float J = std::max(p.J, 0.05f);
      const float pres = lambda * (1.0f - J);
      stress.m[0] = stress.m[4] = stress.m[8] = -pres;
    }
    else {
      const Mat3 R = polar_r(p.F);
      /* P = 2 mu (F - R) + lambda (J-1) J F^{-T}  ~  2 mu (F-R) + lambda (J-1) R */
      const float J = std::max(mat_det(p.F), 0.05f);
      p.J = J;
      Mat3 P = mat_scale(mat_add(p.F, mat_scale(R, -1.0f)), 2.0f * mu);
      P = mat_add(P, mat_scale(R, lambda * (J - 1.0f)));
      stress = mat_scale(mat_mul(P, mat_transpose(p.F)), 1.0f / std::max(J, 1e-6f));
    }
    const float D_inv = 4.0f * inv_dx * inv_dx;
    const Mat3 affine = mat_add(mat_scale(stress, -dt * p.volume * D_inv),
                                mat_scale(p.C, p.mass));

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
          const int gi = bx + i - 1;
          const int gj = by + j - 1;
          const int gk = bz + k - 1;
          if (gi < 0 || gj < 0 || gk < 0 || gi >= nx || gj >= ny || gk >= nz) {
            continue;
          }
          const float wx = 1.0f - std::fabs(fx - float(gi));
          const float wy = 1.0f - std::fabs(fy - float(gj));
          const float wz = 1.0f - std::fabs(fz - float(gk));
          if (wx <= 0.0f || wy <= 0.0f || wz <= 0.0f) {
            continue;
          }
          const float wgt = wx * wy * wz;
          const Vec3 dpos{(float(gi) - fx) * dx, (float(gj) - fy) * dx, (float(gk) - fz) * dx};
          const Vec3 mom = (p.v * p.mass + mat_mul_vec(affine, dpos)) * wgt;
          GridNode &node = grid[gindex(gi, gj, gk)];
          node.mass += p.mass * wgt;
          node.v += mom;
        }
      }
    }
  }

  /* Grid update */
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      for (int k = 0; k < nz; k++) {
        GridNode &node = grid[gindex(i, j, k)];
        if (node.mass <= 1e-12f) {
          node.v = {};
          continue;
        }
        node.v = node.v * (1.0f / node.mass);
        node.v += world->gravity * dt;
        const float x = gmin.x + float(i) * dx;
        const float y = gmin.y + float(j) * dx;
        const float z = gmin.z + float(k) * dx;
        if (world->bounds_enabled) {
          if (x < world->bmin.x && node.v.x < 0) {
            node.v.x = 0;
          }
          if (x > world->bmax.x && node.v.x > 0) {
            node.v.x = 0;
          }
          if (y < world->bmin.y && node.v.y < 0) {
            node.v.y = 0;
          }
          if (y > world->bmax.y && node.v.y > 0) {
            node.v.y = 0;
          }
          if (z < world->bmin.z && node.v.z < 0) {
            node.v.z = 0;
          }
          if (z > world->bmax.z && node.v.z > 0) {
            node.v.z = 0;
          }
        }
      }
    }
  }

  /* G2P */
  for (Particle &p : world->particles) {
    const float fx = (p.x.x - gmin.x) * inv_dx;
    const float fy = (p.x.y - gmin.y) * inv_dx;
    const float fz = (p.x.z - gmin.z) * inv_dx;
    const int bx = int(std::floor(fx));
    const int by = int(std::floor(fy));
    const int bz = int(std::floor(fz));
    Vec3 new_v{};
    Mat3 new_C = mat_zero();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
          const int gi = bx + i - 1;
          const int gj = by + j - 1;
          const int gk = bz + k - 1;
          if (gi < 0 || gj < 0 || gk < 0 || gi >= nx || gj >= ny || gk >= nz) {
            continue;
          }
          const float wx = 1.0f - std::fabs(fx - float(gi));
          const float wy = 1.0f - std::fabs(fy - float(gj));
          const float wz = 1.0f - std::fabs(fz - float(gk));
          if (wx <= 0.0f || wy <= 0.0f || wz <= 0.0f) {
            continue;
          }
          const float wgt = wx * wy * wz;
          const Vec3 dpos{(float(gi) - fx) * dx, (float(gj) - fy) * dx, (float(gk) - fz) * dx};
          const Vec3 gv = grid[gindex(gi, gj, gk)].v;
          new_v += gv * wgt;
          new_C = mat_add(new_C, mat_scale(outer(gv, dpos), 4.0f * wgt * inv_dx * inv_dx));
        }
      }
    }
    p.v = new_v;
    p.C = new_C;
    p.x += p.v * dt;
    if (!world->colliders.empty()) {
      const float thick = world->dx * 0.35f;
      for (const ColTri &tri : world->colliders) {
        const Vec3 ab = tri.b - tri.a;
        const Vec3 ac = tri.c - tri.a;
        const Vec3 ap = p.x - tri.a;
        /* Planar push only — good enough for a floor / wall. */
        const float dn = ap.x * tri.n.x + ap.y * tri.n.y + ap.z * tri.n.z;
        if (dn < thick && dn > -thick) {
          const float u = (ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) /
                          std::max(ab.x * ab.x + ab.y * ab.y + ab.z * ab.z, 1e-8f);
          const float v = (ap.x * ac.x + ap.y * ac.y + ap.z * ac.z) /
                          std::max(ac.x * ac.x + ac.y * ac.y + ac.z * ac.z, 1e-8f);
          if (u >= -0.15f && v >= -0.15f && u + v <= 1.15f) {
            const float push = thick - dn;
            p.x = p.x + tri.n * push;
            const float vn = p.v.x * tri.n.x + p.v.y * tri.n.y + p.v.z * tri.n.z;
            if (vn < 0.0f) {
              p.v = p.v - tri.n * vn;
            }
          }
        }
      }
    }
    Mat3 I = mat_identity();
    const Mat3 Fp = mat_add(I, mat_scale(p.C, dt));
    if (world->material == MPM_MATERIAL_FLUID) {
      p.J *= mat_det(Fp);
      p.J = std::min(std::max(p.J, 0.05f), 20.0f);
      p.F = mat_identity();
      p.F.m[0] = p.F.m[4] = p.F.m[8] = std::cbrt(p.J);
    }
    else {
      p.F = mat_mul(Fp, p.F);
      p.J = mat_det(p.F);
    }
    if (world->bounds_enabled) {
      if (p.x.x < world->bmin.x) {
        p.x.x = world->bmin.x;
        p.v.x = std::max(p.v.x, 0.0f);
      }
      if (p.x.x > world->bmax.x) {
        p.x.x = world->bmax.x;
        p.v.x = std::min(p.v.x, 0.0f);
      }
      if (p.x.y < world->bmin.y) {
        p.x.y = world->bmin.y;
        p.v.y = std::max(p.v.y, 0.0f);
      }
      if (p.x.y > world->bmax.y) {
        p.x.y = world->bmax.y;
        p.v.y = std::min(p.v.y, 0.0f);
      }
      if (p.x.z < world->bmin.z) {
        p.x.z = world->bmin.z;
        p.v.z = std::max(p.v.z, 0.0f);
      }
      if (p.x.z > world->bmax.z) {
        p.x.z = world->bmax.z;
        p.v.z = std::min(p.v.z, 0.0f);
      }
    }
  }
}

int MPM_particle_count(const MPMWorld *world)
{
  return world ? int(world->particles.size()) : 0;
}
void MPM_get_position(const MPMWorld *world, int i, float *x, float *y, float *z)
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
void MPM_get_velocity(const MPMWorld *world, int i, float *x, float *y, float *z)
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
void MPM_get_F(const MPMWorld *world, int i, float F[9])
{
  if (!world || i < 0 || i >= int(world->particles.size()) || !F) {
    return;
  }
  std::memcpy(F, world->particles[i].F.m, 9 * sizeof(float));
}
void MPM_get_C(const MPMWorld *world, int i, float C[9])
{
  if (!world || i < 0 || i >= int(world->particles.size()) || !C) {
    return;
  }
  std::memcpy(C, world->particles[i].C.m, 9 * sizeof(float));
}
float MPM_get_J(const MPMWorld *world, int i)
{
  if (!world || i < 0 || i >= int(world->particles.size())) {
    return 1.0f;
  }
  return world->particles[i].J;
}
