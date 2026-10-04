/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Fast convex polyhedron clipper: start from an AABB, cut by halfspaces.
 * Used by Voronoi 3D / Power 3D so we never call convex_hull_3 per plane.
 */

#pragma once

#include "cgal_bridge.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace blender::cgal_bridge::clip3 {

struct Poly3 {
  std::vector<std::array<double, 3>> verts;
  std::vector<std::vector<int>> faces;
};

inline double dot3(const std::array<double, 3> &a, double nx, double ny, double nz)
{
  return a[0] * nx + a[1] * ny + a[2] * nz;
}

inline Poly3 make_aabb(const double mn[3], const double mx[3])
{
  Poly3 p;
  p.verts = {
      {mn[0], mn[1], mn[2]},
      {mx[0], mn[1], mn[2]},
      {mx[0], mx[1], mn[2]},
      {mn[0], mx[1], mn[2]},
      {mn[0], mn[1], mx[2]},
      {mx[0], mn[1], mx[2]},
      {mx[0], mx[1], mx[2]},
      {mn[0], mx[1], mx[2]},
  };
  /* Outward CCW. */
  p.faces = {
      {0, 3, 2, 1}, /* -Z */
      {4, 5, 6, 7}, /* +Z */
      {0, 1, 5, 4}, /* -Y */
      {3, 7, 6, 2}, /* +Y */
      {0, 4, 7, 3}, /* -X */
      {1, 2, 6, 5}, /* +X */
  };
  return p;
}

inline void uniquify_ring(std::vector<int> &ring)
{
  if (ring.empty()) {
    return;
  }
  std::vector<int> out;
  out.reserve(ring.size());
  for (int v : ring) {
    if (out.empty() || out.back() != v) {
      out.push_back(v);
    }
  }
  if (out.size() >= 2 && out.front() == out.back()) {
    out.pop_back();
  }
  ring.swap(out);
}

/** Keep the halfspace n·x <= c. Returns false if the cell vanishes. */
inline bool clip_halfspace(Poly3 &cell, double nx, double ny, double nz, double c)
{
  const int nv = int(cell.verts.size());
  if (nv == 0 || cell.faces.empty()) {
    cell.verts.clear();
    cell.faces.clear();
    return false;
  }
  const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
  if (nlen < 1e-18) {
    return true;
  }
  const double scale = nlen + std::abs(c) + 1.0;
  const double eps = 1e-10 * scale;

  std::vector<double> hs;
  std::vector<char> inside;
  hs.resize(size_t(nv));
  inside.resize(size_t(nv), 0);
  int nkeep = 0;
  for (int i = 0; i < nv; i++) {
    hs[size_t(i)] = dot3(cell.verts[size_t(i)], nx, ny, nz) - c;
    inside[size_t(i)] = hs[size_t(i)] <= eps;
    nkeep += int(inside[size_t(i)]);
  }
  if (nkeep == nv) {
    return true;
  }
  if (nkeep == 0) {
    cell.verts.clear();
    cell.faces.clear();
    return false;
  }

  Poly3 out;
  out.verts.reserve(size_t(nkeep + 8));
  std::vector<int> remap(size_t(nv), -1);
  for (int i = 0; i < nv; i++) {
    if (inside[size_t(i)]) {
      remap[size_t(i)] = int(out.verts.size());
      out.verts.push_back(cell.verts[size_t(i)]);
    }
  }

  auto lerp = [&](int a, int b) -> std::array<double, 3> {
    const double sa = hs[size_t(a)];
    const double sb = hs[size_t(b)];
    double t = sa / (sa - sb);
    t = std::min(1.0, std::max(0.0, t));
    const auto &pa = cell.verts[size_t(a)];
    const auto &pb = cell.verts[size_t(b)];
    return {pa[0] + t * (pb[0] - pa[0]),
            pa[1] + t * (pb[1] - pa[1]),
            pa[2] + t * (pb[2] - pa[2])};
  };

  std::map<std::pair<int, int>, int> isect;
  auto isect_idx = [&](int a, int b) {
    const int lo = std::min(a, b);
    const int hi = std::max(a, b);
    const auto key = std::make_pair(lo, hi);
    const auto it = isect.find(key);
    if (it != isect.end()) {
      return it->second;
    }
    const int idx = int(out.verts.size());
    out.verts.push_back(lerp(a, b));
    isect[key] = idx;
    return idx;
  };

  for (const auto &face : cell.faces) {
    const int fn = int(face.size());
    if (fn < 3) {
      continue;
    }
    std::vector<int> nf;
    nf.reserve(size_t(fn + 2));
    for (int i = 0; i < fn; i++) {
      const int a = face[size_t(i)];
      const int b = face[size_t((i + 1) % fn)];
      const bool ia = inside[size_t(a)] != 0;
      const bool ib = inside[size_t(b)] != 0;
      if (ia) {
        nf.push_back(remap[size_t(a)]);
        if (!ib) {
          nf.push_back(isect_idx(a, b));
        }
      }
      else if (ib) {
        nf.push_back(isect_idx(a, b));
      }
    }
    uniquify_ring(nf);
    if (nf.size() >= 3) {
      out.faces.push_back(std::move(nf));
    }
  }

  if (isect.size() >= 3) {
    std::vector<int> cap;
    cap.reserve(isect.size());
    for (const auto &kv : isect) {
      cap.push_back(kv.second);
    }
    std::array<double, 3> cen = {0, 0, 0};
    for (int i : cap) {
      cen[0] += out.verts[size_t(i)][0];
      cen[1] += out.verts[size_t(i)][1];
      cen[2] += out.verts[size_t(i)][2];
    }
    const double inv = 1.0 / double(cap.size());
    cen[0] *= inv;
    cen[1] *= inv;
    cen[2] *= inv;
    const double inx = nx / nlen;
    const double iny = ny / nlen;
    const double inz = nz / nlen;
    std::array<double, 3> u;
    if (std::abs(inx) < std::abs(iny) && std::abs(inx) < std::abs(inz)) {
      u = {0.0, -inz, iny};
    }
    else if (std::abs(iny) < std::abs(inz)) {
      u = {-inz, 0.0, inx};
    }
    else {
      u = {-iny, inx, 0.0};
    }
    const double ul = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    if (ul > 1e-18) {
      u[0] /= ul;
      u[1] /= ul;
      u[2] /= ul;
    }
    const std::array<double, 3> v = {iny * u[2] - inz * u[1],
                                     inz * u[0] - inx * u[2],
                                     inx * u[1] - iny * u[0]};
    std::sort(cap.begin(), cap.end(), [&](int ia, int ib) {
      const auto &a = out.verts[size_t(ia)];
      const auto &b = out.verts[size_t(ib)];
      const double ax = a[0] - cen[0], ay = a[1] - cen[1], az = a[2] - cen[2];
      const double bx = b[0] - cen[0], by = b[1] - cen[1], bz = b[2] - cen[2];
      const double aa = std::atan2(ax * v[0] + ay * v[1] + az * v[2],
                                   ax * u[0] + ay * u[1] + az * u[2]);
      const double ba = std::atan2(bx * v[0] + by * v[1] + bz * v[2],
                                   bx * u[0] + by * u[1] + bz * u[2]);
      return aa < ba;
    });
    uniquify_ring(cap);
    if (cap.size() >= 3) {
      /* Newell: flip if the cap does not point along +n (outward, discarded side). */
      double cx = 0, cy = 0, cz = 0;
      const int cn = int(cap.size());
      for (int i = 0; i < cn; i++) {
        const auto &a = out.verts[size_t(cap[size_t(i)])];
        const auto &b = out.verts[size_t(cap[size_t((i + 1) % cn)])];
        cx += (a[1] - b[1]) * (a[2] + b[2]);
        cy += (a[2] - b[2]) * (a[0] + b[0]);
        cz += (a[0] - b[0]) * (a[1] + b[1]);
      }
      if (cx * nx + cy * ny + cz * nz < 0.0) {
        std::reverse(cap.begin(), cap.end());
      }
      out.faces.push_back(std::move(cap));
    }
  }

  cell = std::move(out);
  return !cell.faces.empty();
}

inline void emit_isolated(MeshResult &result, const Poly3 &cell, int tag)
{
  if (cell.faces.empty() || cell.verts.size() < 3) {
    return;
  }
  if (result.face_offsets.empty()) {
    result.face_offsets.push_back(0);
  }
  const int base = result.verts_num();
  for (const auto &v : cell.verts) {
    result.positions.push_back(float(v[0]));
    result.positions.push_back(float(v[1]));
    result.positions.push_back(float(v[2]));
  }
  for (const auto &f : cell.faces) {
    if (f.size() < 3) {
      continue;
    }
    for (int i : f) {
      result.corner_verts.push_back(base + i);
    }
    result.face_offsets.push_back(int(result.corner_verts.size()));
    result.face_tag.push_back(tag);
  }
}

inline void pad_aabb_extent(double mn[3], double mx[3], double margin)
{
  margin = std::max(margin, 0.0);
  double ext = 0.0;
  for (int k = 0; k < 3; k++) {
    mn[k] -= margin;
    mx[k] += margin;
    ext = std::max(ext, mx[k] - mn[k]);
  }
  ext = std::max(ext, 1e-8);
  for (int k = 0; k < 3; k++) {
    if (mx[k] - mn[k] < 1e-9 * ext) {
      const double mid = 0.5 * (mn[k] + mx[k]);
      mn[k] = mid - 1e-6 * ext;
      mx[k] = mid + 1e-6 * ext;
    }
  }
}

}  // namespace blender::cgal_bridge::clip3
