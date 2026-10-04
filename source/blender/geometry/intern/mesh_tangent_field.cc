/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Tangent field via Directional (avaxman/Directional) power_field + power_to_raw.
 *
 * Multi-component meshes: solve each face-connected island separately.
 *
 * Alignment (Houdini-style channels, soft constraints into power_field):
 *   - Curvature: principal directions from discrete edge dihedral shape operator
 *   - Boundary: open-boundary edge tangents
 *   - Guide: optional per-vertex vectors
 *
 * Without any alignment (alignment_weight==0 or all channel weights 0), falls back
 * to the smoothest eigenmode (Directional's gauge-fixed unconstrained solve).
 */

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"

#include <numbers>
#include "BLI_offset_indices.hh"
#include "BLI_span.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "DNA_mesh_types.h"

#include "BKE_attribute.hh"
#include "BKE_mesh.hh"

#include "GEO_mesh_tangent_field.hh"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#ifdef WITH_DIRECTIONAL
#  include <Eigen/Core>
#  include <directional/CartesianField.h>
#  include <directional/IntrinsicVertexTangentBundle.h>
#  include <directional/TriMesh.h>
#  include <directional/power_field.h>
#  include <directional/power_to_raw.h>
#endif

namespace blender::geometry {

#ifdef WITH_DIRECTIONAL

static void build_face_adjacency(const Mesh &mesh, Array<Vector<int>> &face_adj)
{
  const int faces_num = mesh.faces_num;
  face_adj.reinitialize(faces_num);

  const Span<int> corner_verts = mesh.corner_verts();
  const OffsetIndices faces = mesh.faces();
  const Span<int2> edges = mesh.edges();

  Map<std::pair<int, int>, int> vert_pair_to_edge;
  for (const int e : edges.index_range()) {
    const int a = math::min(edges[e][0], edges[e][1]);
    const int b = math::max(edges[e][0], edges[e][1]);
    vert_pair_to_edge.lookup_or_add({a, b}, e);
  }

  Array<Vector<int>> edge_faces(edges.size());
  for (const int f : faces.index_range()) {
    const IndexRange face = faces[f];
    const int n = face.size();
    for (int i = 0; i < n; i++) {
      const int v0 = corner_verts[face[i]];
      const int v1 = corner_verts[face[(i + 1) % n]];
      const int a = math::min(v0, v1);
      const int b = math::max(v0, v1);
      if (const int *ei = vert_pair_to_edge.lookup_ptr({a, b})) {
        edge_faces[*ei].append(f);
      }
    }
  }

  for (const int e : edges.index_range()) {
    const Span<int> fs = edge_faces[e];
    for (const int i : fs.index_range()) {
      for (const int j : fs.index_range()) {
        if (i != j) {
          face_adj[fs[i]].append(fs[j]);
        }
      }
    }
  }

  for (const int f : faces.index_range()) {
    Vector<int> &nbrs = face_adj[f];
    if (nbrs.size() > 1) {
      std::sort(nbrs.begin(), nbrs.end());
      nbrs.resize(std::unique(nbrs.begin(), nbrs.end()) - nbrs.begin());
    }
  }
}

/** Rotate vector `d` around unit normal `n` by `angle` radians. */
static float3 rotate_around_normal(const float3 &d, const float3 &n, const float angle)
{
  if (math::abs(angle) < 1.0e-12f) {
    return d;
  }
  const float c = math::cos(angle);
  const float s = math::sin(angle);
  return d * c + math::cross(n, d) * s + n * math::dot(n, d) * (1.0f - c);
}

/** Project `v` into the tangent plane of unit normal `n` and renormalize. */
static float3 project_to_tangent(const float3 &v, const float3 &n)
{
  float3 t = v - n * math::dot(v, n);
  const float len = math::length(t);
  if (len <= 1.0e-12f) {
    return float3(0.0f);
  }
  return t / len;
}

/**
 * Build local orthonormal tangent frame (t1, t2) at a vertex from its normal
 * and a preferred in-plane reference direction (may be zero).
 */
static void tangent_frame(const float3 &n, const float3 &hint, float3 &r_t1, float3 &r_t2)
{
  float3 t1 = project_to_tangent(hint, n);
  if (math::length_squared(t1) < 1.0e-12f) {
    /* Stable pick orthogonal to n. */
    const float3 a = math::abs(n.x) < 0.9f ? float3(1, 0, 0) : float3(0, 1, 0);
    t1 = math::normalize(math::cross(n, a));
  }
  r_t1 = t1;
  r_t2 = math::normalize(math::cross(n, t1));
}

struct SoftConstraint {
  int local_vert = 0;
  float3 direction = float3(0.0f);
  float weight = 0.0f;
};

/**
 * Soft weight for Directional polyvector alignment.
 *
 * Energy scale (see polyvector_field):
 *   smooth ~ wSmooth(=1) / totalSmoothWeight
 *   align  ~ wAlignment * mass / sum(mass of constrained spaces)
 * So wAlignment of order ~0.2–2.0 is soft guidance; values >> 5 overpower
 * smoothness and amplify noisy principal directions into a messy field.
 *
 * Alignment factor a∈[0,1] uses a gentle curve (not a/(1-a)).
 */
static float soft_weight_from_alignment(const float alignment_weight, const float channel_weight)
{
  if (alignment_weight <= 1.0e-6f || channel_weight <= 1.0e-6f) {
    return 0.0f;
  }
  const float a = math::clamp(alignment_weight, 0.0f, 1.0f);
  /* 0→0, 0.5→~0.35, 1.0→1.0 — stays in the soft regime of Directional. */
  const float base = a * a * (0.75f + 0.25f * a);
  return math::max(channel_weight, 0.0f) * base;
}

/**
 * Estimate principal curvature directions + anisotropy strength.
 *
 * r_principal: unit tangent direction (or zero if unreliable)
 * r_anisotropy: |κ1−κ2| (relative), used as a mask so flat regions do not
 *               inject noisy constraints.
 */
static void estimate_principal_directions(const Span<float3> positions,
                                          const Span<int3> tris,
                                          const Span<float3> vert_normals,
                                          MutableSpan<float3> r_principal,
                                          MutableSpan<float> r_anisotropy)
{
  const int nV = positions.size();
  r_principal.fill(float3(0.0f));
  r_anisotropy.fill(0.0f);

  Map<uint64_t, Vector<float3, 2>> edge_face_normals;
  for (const int fi : tris.index_range()) {
    const int3 t = tris[fi];
    const float3 fn = math::normal_tri(positions[t[0]], positions[t[1]], positions[t[2]]);
    for (int i = 0; i < 3; i++) {
      const int a = t[i];
      const int b = t[(i + 1) % 3];
      const uint32_t lo = uint32_t(math::min(a, b));
      const uint32_t hi = uint32_t(math::max(a, b));
      const uint64_t key = (uint64_t(hi) << 32) | uint64_t(lo);
      edge_face_normals.lookup_or_add(key, {}).append(fn);
    }
  }

  Array<float3> t1s(nV, float3(0.0f));
  Array<float3> t2s(nV, float3(0.0f));
  Array<float> s00(nV, 0.0f);
  Array<float> s01(nV, 0.0f);
  Array<float> s11(nV, 0.0f);
  Array<float> weight_sum(nV, 0.0f);

  for (const int v : IndexRange(nV)) {
    tangent_frame(vert_normals[v], float3(1, 0, 0), t1s[v], t2s[v]);
  }

  for (const auto &item : edge_face_normals.items()) {
    if (item.value.size() != 2) {
      continue;
    }
    const uint64_t key = item.key;
    const int v0 = int(key & 0xffffffffu);
    const int v1 = int(key >> 32);
    const float3 edge = positions[v1] - positions[v0];
    const float elen = math::length(edge);
    if (elen <= 1.0e-12f) {
      continue;
    }
    const float3 n0 = item.value[0];
    const float3 n1 = item.value[1];
    float cosang = math::clamp(math::dot(n0, n1), -1.0f, 1.0f);
    float beta = math::acos(cosang);
    const float3 edge_dir = edge / elen;
    if (math::dot(math::cross(n0, n1), edge_dir) < 0.0f) {
      beta = -beta;
    }
    /* Dimensionless-ish: dihedral only (avoid 1/length blowing up on dense meshes). */
    const float kappa = beta;
    const float w = elen;

    auto accumulate = [&](const int v) {
      const float3 n = vert_normals[v];
      const float3 t = project_to_tangent(edge_dir, n);
      if (math::length_squared(t) < 1.0e-12f) {
        return;
      }
      const float x = math::dot(t, t1s[v]);
      const float y = math::dot(t, t2s[v]);
      s00[v] += w * kappa * x * x;
      s01[v] += w * kappa * x * y;
      s11[v] += w * kappa * y * y;
      weight_sum[v] += w;
    };
    accumulate(v0);
    accumulate(v1);
  }

  for (const int v : IndexRange(nV)) {
    if (weight_sum[v] <= 1.0e-12f) {
      continue;
    }
    const float a = s00[v] / weight_sum[v];
    const float b = s01[v] / weight_sum[v];
    const float c = s11[v] / weight_sum[v];
    const float trace = a + c;
    const float det = a * c - b * b;
    const float disc = math::max(0.0f, 0.25f * trace * trace - det);
    const float sqrt_d = math::sqrt(disc);
    const float l1 = 0.5f * trace + sqrt_d;
    const float l2 = 0.5f * trace - sqrt_d;
    /* Anisotropy: how directional the curvature is (flat / umbilical → 0). */
    r_anisotropy[v] = math::abs(l1 - l2);

    const bool use_l1 = math::abs(l1) >= math::abs(l2);
    float e0, e1;
    if (use_l1) {
      if (math::abs(b) > 1.0e-8f || math::abs(a - l1) > 1.0e-8f) {
        e0 = l1 - c;
        e1 = b;
        if (math::abs(e0) + math::abs(e1) < 1.0e-12f) {
          e0 = b;
          e1 = l1 - a;
        }
      }
      else {
        e0 = 1.0f;
        e1 = 0.0f;
      }
    }
    else {
      if (math::abs(b) > 1.0e-8f || math::abs(a - l2) > 1.0e-8f) {
        e0 = l2 - c;
        e1 = b;
        if (math::abs(e0) + math::abs(e1) < 1.0e-12f) {
          e0 = b;
          e1 = l2 - a;
        }
      }
      else {
        e0 = 0.0f;
        e1 = 1.0f;
      }
    }
    const float el = math::sqrt(e0 * e0 + e1 * e1);
    if (el <= 1.0e-12f) {
      continue;
    }
    e0 /= el;
    e1 /= el;
    r_principal[v] = math::normalize(t1s[v] * e0 + t2s[v] * e1);
  }
}

/** 1-ring smooth of unoriented principal dirs (N-RoSy aware), reduces noise. */
static void smooth_principal_directions(const Span<int3> tris,
                                        const Span<float3> vert_normals,
                                        const int N,
                                        MutableSpan<float3> principal,
                                        MutableSpan<float> anisotropy)
{
  const int nV = principal.size();
  Array<Vector<int>> adj(nV);
  for (const int3 t : tris) {
    for (int i = 0; i < 3; i++) {
      const int a = t[i];
      const int b = t[(i + 1) % 3];
      adj[a].append(b);
      adj[b].append(a);
    }
  }
  Array<float3> smoothed(nV, float3(0.0f));
  Array<float> smoothed_a(nV, 0.0f);
  for (const int v : IndexRange(nV)) {
    if (math::length_squared(principal[v]) < 1.0e-12f) {
      continue;
    }
    float3 acc = principal[v] * anisotropy[v];
    float wsum = anisotropy[v];
    const float3 n = vert_normals[v];
    for (const int nb : adj[v]) {
      float3 d = principal[nb];
      if (math::length_squared(d) < 1.0e-12f) {
        continue;
      }
      /* Align neighbor to same N-RoSy sector as self. */
      if (N >= 2) {
        float best = -2.0f;
        float3 best_d = d;
        for (int k = 0; k < N; k++) {
          const float ang = float(k) * (2.0f * float(std::numbers::pi) / float(N));
          const float3 cand = rotate_around_normal(d, n, ang);
          const float dotv = math::dot(cand, principal[v]);
          if (dotv > best) {
            best = dotv;
            best_d = cand;
          }
        }
        d = best_d;
      }
      else if (math::dot(d, principal[v]) < 0.0f) {
        d = -d;
      }
      const float w = anisotropy[nb];
      acc += d * w;
      wsum += w;
    }
    if (wsum > 1.0e-12f) {
      smoothed[v] = project_to_tangent(acc, n);
      smoothed_a[v] = anisotropy[v];
    }
  }
  principal.copy_from(smoothed);
  anisotropy.copy_from(smoothed_a);
}

static void compute_vertex_normals(const Span<float3> positions,
                                   const Span<int3> tris,
                                   MutableSpan<float3> r_normals)
{
  r_normals.fill(float3(0.0f));
  for (const int3 t : tris) {
    const float3 fn = math::normal_tri(positions[t[0]], positions[t[1]], positions[t[2]]);
    r_normals[t[0]] += fn;
    r_normals[t[1]] += fn;
    r_normals[t[2]] += fn;
  }
  for (float3 &n : r_normals) {
    const float len = math::length(n);
    n = len > 1.0e-12f ? n / len : float3(0, 0, 1);
  }
}

/**
 * Collect soft constraints for one island (local vertex indexing).
 */
static void build_island_constraints(const Span<float3> positions,
                                     const Span<int3> island_tris,
                                     const Span<int> local_to_global,
                                     const Map<int, int> &global_to_local,
                                     const Span<float3> vert_normals_global,
                                     const TangentFieldOptions &options,
                                     Vector<SoftConstraint> &r_constraints)
{
  r_constraints.clear();
  const int nV = local_to_global.size();
  const int N = options.n;
  const float align = math::clamp(options.alignment_weight, 0.0f, 1.0f);
  if (align <= 1.0e-6f) {
    return;
  }

  Array<float3> local_positions(nV);
  for (const int i : IndexRange(nV)) {
    local_positions[i] = positions[local_to_global[i]];
  }
  Array<float3> local_normals(nV);
  for (const int i : IndexRange(nV)) {
    local_normals[i] = vert_normals_global[local_to_global[i]];
  }

  /* Edge adjacency for boundary detection. */
  Map<uint64_t, int> edge_face_count;
  for (const int3 t : island_tris) {
    for (int i = 0; i < 3; i++) {
      const int a = global_to_local.lookup(t[i]);
      const int b = global_to_local.lookup(t[(i + 1) % 3]);
      const uint32_t lo = uint32_t(math::min(a, b));
      const uint32_t hi = uint32_t(math::max(a, b));
      const uint64_t key = (uint64_t(hi) << 32) | uint64_t(lo);
      int &c = edge_face_count.lookup_or_add(key, 0);
      c += 1;
    }
  }

  Array<float3> blended_dir(nV, float3(0.0f));
  Array<float> blended_w(nV, 0.0f);

  auto add_channel = [&](const int local_v, float3 dir, const float channel_w) {
    const float sw = soft_weight_from_alignment(align, channel_w);
    if (sw <= 1.0e-8f) {
      return;
    }
    dir = project_to_tangent(dir, local_normals[local_v]);
    if (math::length_squared(dir) < 1.0e-12f) {
      return;
    }
    /* If already have a direction, pick N-RoSy equivalent closest to existing. */
    if (blended_w[local_v] > 1.0e-8f && N >= 2) {
      const float3 n = local_normals[local_v];
      const float3 base = blended_dir[local_v];
      float best_dot = -2.0f;
      float3 best = dir;
      for (int k = 0; k < N; k++) {
        const float ang = float(k) * (2.0f * float(std::numbers::pi) / float(N));
        const float3 cand = rotate_around_normal(dir, n, ang);
        const float d = math::dot(cand, base);
        if (d > best_dot) {
          best_dot = d;
          best = cand;
        }
      }
      dir = best;
    }
    blended_dir[local_v] += dir * sw;
    blended_w[local_v] += sw;
  };

  /* --- Boundary channel (one constraint per boundary vertex) --- */
  if (options.boundary_weight > 1.0e-8f) {
    Array<float3> bdir(nV, float3(0.0f));
    Array<float> bw(nV, 0.0f);
    for (const auto &item : edge_face_count.items()) {
      if (item.value != 1) {
        continue;
      }
      const uint64_t key = item.key;
      const int a = int(key & 0xffffffffu);
      const int b = int(key >> 32);
      const float3 edge = local_positions[b] - local_positions[a];
      const float elen = math::length(edge);
      if (elen <= 1.0e-12f) {
        continue;
      }
      for (const int lv : {a, b}) {
        float3 dir = project_to_tangent(edge, local_normals[lv]);
        if (math::length_squared(dir) < 1.0e-12f) {
          continue;
        }
        if (options.boundary_rotation != 0.0f) {
          dir = rotate_around_normal(dir, local_normals[lv], options.boundary_rotation);
        }
        /* Prefer longer edges at corners (avoid averaging two 90° edges to diagonal). */
        if (bw[lv] > 0.0f && N >= 2) {
          const float3 base = bdir[lv];
          float best = -2.0f;
          float3 best_d = dir;
          for (int k = 0; k < N; k++) {
            const float ang = float(k) * (2.0f * float(std::numbers::pi) / float(N));
            const float3 cand = rotate_around_normal(dir, local_normals[lv], ang);
            const float d = math::dot(cand, base);
            if (d > best) {
              best = d;
              best_d = cand;
            }
          }
          dir = best_d;
        }
        bdir[lv] += dir * elen;
        bw[lv] += elen;
      }
    }
    for (const int lv : IndexRange(nV)) {
      if (bw[lv] <= 1.0e-12f) {
        continue;
      }
      /* Boundary is sparse and reliable → slightly stronger relative channel. */
      add_channel(lv, bdir[lv], options.boundary_weight * 1.25f);
    }
  }

  /* --- Curvature channel (N ≥ 2): mask by anisotropy so flats stay free --- */
  if (options.curvature_weight > 1.0e-8f && N >= 2) {
    Array<float3> principal(nV);
    Array<float> anisotropy(nV);
    Array<int3> local_tris(island_tris.size());
    for (const int i : island_tris.index_range()) {
      const int3 g = island_tris[i];
      local_tris[i] = int3(global_to_local.lookup(g[0]),
                           global_to_local.lookup(g[1]),
                           global_to_local.lookup(g[2]));
    }
    estimate_principal_directions(
        local_positions, local_tris, local_normals, principal, anisotropy);
    smooth_principal_directions(local_tris, local_normals, N, principal, anisotropy);

    /* Normalize anisotropy to [0,1] by a robust high percentile proxy (max*0.5). */
    float max_a = 0.0f;
    for (const float a : anisotropy) {
      max_a = math::max(max_a, a);
    }
    const float a_scale = math::max(max_a * 0.5f, 1.0e-8f);

    for (const int lv : IndexRange(nV)) {
      float3 dir = principal[lv];
      if (math::length_squared(dir) < 1.0e-12f) {
        continue;
      }
      /* Soft mask: flat / umbilical regions contribute little or nothing. */
      const float mask = math::clamp(anisotropy[lv] / a_scale, 0.0f, 1.0f);
      const float mask_sq = mask * mask;
      if (mask_sq < 0.04f) {
        continue; /* Below ~20% of typical anisotropy → skip (noise). */
      }
      if (options.curvature_rotation != 0.0f) {
        dir = rotate_around_normal(dir, local_normals[lv], options.curvature_rotation);
      }
      add_channel(lv, dir, options.curvature_weight * mask_sq);
    }
  }

  /* --- Guide channel --- */
  if (options.guide_weight > 1.0e-8f && !options.guide_vectors.is_empty()) {
    for (const int lv : IndexRange(nV)) {
      const int g = local_to_global[lv];
      if (g < 0 || g >= options.guide_vectors.size()) {
        continue;
      }
      float3 dir = options.guide_vectors[g];
      if (math::length_squared(dir) < 1.0e-12f) {
        continue;
      }
      add_channel(lv, dir, options.guide_weight);
    }
  }

  constexpr float k_max_soft = 2.5f; /* Hard cap vs smoothness (wSmooth=1). */
  for (const int lv : IndexRange(nV)) {
    if (blended_w[lv] <= 1.0e-8f) {
      continue;
    }
    float3 dir = blended_dir[lv];
    const float len = math::length(dir);
    if (len <= 1.0e-12f) {
      continue;
    }
    dir /= len;
    SoftConstraint c;
    c.local_vert = lv;
    c.direction = dir;
    c.weight = math::min(blended_w[lv], k_max_soft);
    r_constraints.append(c);
  }
}

/**
 * Run Directional on one face-connected island; write into global dir buffers.
 */
static bool compute_island(const Span<float3> positions,
                           const Span<int3> island_tris,
                           const Span<float3> vert_normals_global,
                           const TangentFieldOptions &options,
                           MutableSpan<Array<float3>> dirs_per_k)
{
  Vector<int> local_to_global;
  Map<int, int> global_to_local;
  for (const int3 tri : island_tris) {
    for (int k = 0; k < 3; k++) {
      const int g = tri[k];
      if (!global_to_local.contains(g)) {
        global_to_local.add(g, local_to_global.size());
        local_to_global.append(g);
      }
    }
  }
  const int nV = local_to_global.size();
  const int nF = island_tris.size();
  if (nV < 3 || nF < 1) {
    return false;
  }

  const int N = options.n;

  Eigen::MatrixXd V(nV, 3);
  for (const int i : IndexRange(nV)) {
    const float3 &p = positions[local_to_global[i]];
    V(i, 0) = p.x;
    V(i, 1) = p.y;
    V(i, 2) = p.z;
  }
  Eigen::MatrixXi F(nF, 3);
  for (const int f : IndexRange(nF)) {
    F(f, 0) = global_to_local.lookup(island_tris[f][0]);
    F(f, 1) = global_to_local.lookup(island_tris[f][1]);
    F(f, 2) = global_to_local.lookup(island_tris[f][2]);
  }

  directional::TriMesh tri_mesh;
  tri_mesh.set_mesh(V, F);
  directional::IntrinsicVertexTangentBundle vtb;
  vtb.init(tri_mesh);

  Vector<SoftConstraint> constraints;
  build_island_constraints(positions,
                           island_tris,
                           local_to_global,
                           global_to_local,
                           vert_normals_global,
                           options,
                           constraints);

  Eigen::VectorXi const_spaces;
  Eigen::MatrixXd const_vectors;
  Eigen::VectorXd align_weights;

  if (!constraints.is_empty()) {
    const int nc = constraints.size();
    const_spaces.resize(nc);
    const_vectors.resize(nc, 3);
    align_weights.resize(nc);
    for (const int i : IndexRange(nc)) {
      const SoftConstraint &c = constraints[i];
      const_spaces(i) = c.local_vert;
      const_vectors(i, 0) = double(c.direction.x);
      const_vectors(i, 1) = double(c.direction.y);
      const_vectors(i, 2) = double(c.direction.z);
      /* Soft positive weight; Directional uses negative for hard fix. */
      align_weights(i) = double(math::max(c.weight, 1.0e-4f));
    }
  }

  directional::CartesianField power_field;
  directional::power_field(
      vtb, const_spaces, const_vectors, align_weights, N, power_field, false);
  directional::CartesianField raw_field;
  directional::power_to_raw(power_field, N, raw_field, options.normalize);

  const Eigen::MatrixXd &ext = raw_field.extField;
  if (ext.rows() != nV || ext.cols() < 3 * N) {
    return false;
  }

  for (const int i : IndexRange(nV)) {
    const int g = local_to_global[i];
    for (int k = 0; k < N; k++) {
      dirs_per_k[k][g] = float3(
          float(ext(i, 3 * k + 0)), float(ext(i, 3 * k + 1)), float(ext(i, 3 * k + 2)));
    }
  }
  return true;
}

#endif

int mesh_tangent_field_store_attributes(Mesh &mesh, const TangentFieldOptions &options)
{
#ifndef WITH_DIRECTIONAL
  UNUSED_VARS(mesh, options);
  return 0;
#else
  if (mesh.verts_num < 3 || mesh.faces_num < 1 || options.attribute_prefix.empty()) {
    return 0;
  }

  const int N = options.n;
  if (N < 1 || N > 24) {
    return 0;
  }

  const OffsetIndices faces = mesh.faces();
  const Span<int> corner_verts = mesh.corner_verts();
  for (const int face_i : faces.index_range()) {
    if (faces[face_i].size() != 3) {
      return 0;
    }
  }

  const Span<float3> positions = mesh.vert_positions();

  Array<int3> tris(mesh.faces_num);
  for (const int f : faces.index_range()) {
    const IndexRange face = faces[f];
    tris[f] = int3(corner_verts[face[0]], corner_verts[face[1]], corner_verts[face[2]]);
  }

  Array<float3> vert_normals(mesh.verts_num);
  compute_vertex_normals(positions, tris, vert_normals);

  Array<Vector<int>> face_adj;
  build_face_adjacency(mesh, face_adj);

  Array<int> face_island(mesh.faces_num, -1);
  int island_count = 0;
  Vector<int> stack;
  for (const int seed : faces.index_range()) {
    if (face_island[seed] >= 0) {
      continue;
    }
    stack.clear();
    stack.append(seed);
    face_island[seed] = island_count;
    while (!stack.is_empty()) {
      const int f = stack.pop_last();
      for (const int nb : face_adj[f]) {
        if (face_island[nb] < 0) {
          face_island[nb] = island_count;
          stack.append(nb);
        }
      }
    }
    island_count++;
  }

  Array<Array<float3>> dirs_per_k(N);
  for (int k = 0; k < N; k++) {
    dirs_per_k[k].reinitialize(mesh.verts_num);
    dirs_per_k[k].fill(float3(0.0f));
  }

  bool any_ok = false;
  try {
    for (int isl = 0; isl < island_count; isl++) {
      Vector<int3> island_tris;
      for (const int f : faces.index_range()) {
        if (face_island[f] == isl) {
          island_tris.append(tris[f]);
        }
      }
      if (compute_island(positions, island_tris, vert_normals, options, dirs_per_k)) {
        any_ok = true;
      }
    }
  }
  catch (...) {
    return 0;
  }

  if (!any_ok) {
    return 0;
  }

  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  const std::string &prefix = options.attribute_prefix;

  for (int k = 0; k < N; k++) {
    const std::string name = prefix + std::to_string(k);
    attributes.remove(name);
    bke::SpanAttributeWriter<float3> writer =
        attributes.lookup_or_add_for_write_only_span<float3>(name, bke::AttrDomain::Point);
    if (!writer) {
      continue;
    }
    writer.span.copy_from(dirs_per_k[k]);
    writer.finish();
  }

  return N;
#endif
}

}  // namespace blender::geometry
