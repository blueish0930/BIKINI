/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * In-memory Instant Meshes batch pipeline (mirrors upstream batch.cpp without mesh I/O).
 */

#include <cstdlib>
// Allocations use malloc/free so this CAPI can live in a separate DLL.

#include "instant_meshes_capi.hpp"

#include "adjacency.h"
#include "bvh.h"
#include "dedge.h"
#include "extract.h"
#include "field.h"
#include "hierarchy.h"
#include "meshstats.h"
#include "normal.h"
#include "subdivide.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <vector>

/* Defined in field.cpp (Blender patch). */
extern int instant_meshes_level_iterations;

static bool check_canceled(const float progress,
                           void (*update_cb)(void *, float progress, int *cancel),
                           void *update_cb_data)
{
  if (!update_cb) {
    return false;
  }
  int cancel = 0;
  update_cb(update_cb_data, progress, &cancel);
  return cancel != 0;
}

void IMESH_instant_meshes_remesh(InstantMeshesRemeshData *data,
                                 void (*update_cb)(void *, float progress, int *cancel),
                                 void *update_cb_data)
{
  data->out_verts = nullptr;
  data->out_face_offsets = nullptr;
  data->out_corner_verts = nullptr;
  data->out_totverts = 0;
  data->out_totfaces = 0;
  data->out_totcorners = 0;

  if (!data->verts || !data->faces || data->totverts < 3 || data->totfaces < 1) {
    return;
  }

  int rosy = data->rosy;
  int posy = data->posy;
  if (posy == 6) {
    posy = 3;
  }
  if ((posy != 3 && posy != 4) || (rosy != 2 && rosy != 4 && rosy != 6)) {
    return;
  }

  Float scale = data->scale;
  int face_count = data->face_count;
  int vertex_count = data->vertex_count;
  const Float creaseAngle = data->crease_angle_deg;
  const bool extrinsic = data->extrinsic;
  const bool align_to_boundaries = data->align_to_boundaries;
  const bool pure_quad = data->pure_quad;
  const bool deterministic = data->deterministic;
  const int smooth_iter = std::max(0, data->smooth_iter);

  instant_meshes_level_iterations = std::max(1, data->iterations > 0 ? data->iterations : 6);

  if (check_canceled(0.0f, update_cb, update_cb_data)) {
    return;
  }

  /* ---- Convert Blender mesh → Instant Meshes Eigen layout. ---- */
  MatrixXu F(3, data->totfaces);
  MatrixXf V(3, data->totverts);
  for (int i = 0; i < data->totverts; i++) {
    V(0, i) = data->verts[i * 3 + 0];
    V(1, i) = data->verts[i * 3 + 1];
    V(2, i) = data->verts[i * 3 + 2];
  }
  for (int i = 0; i < data->totfaces; i++) {
    F(0, i) = uint32_t(data->faces[i * 3 + 0]);
    F(1, i) = uint32_t(data->faces[i * 3 + 1]);
    F(2, i) = uint32_t(data->faces[i * 3 + 2]);
  }

  MatrixXf N;
  VectorXf A;
  std::set<uint32_t> crease_in, crease_out;
  BVH *bvh = nullptr;
  AdjacencyMatrix adj = nullptr;

  try {
    MeshStats stats = compute_mesh_stats(F, V, deterministic);

    /* Target resolution (same priority rules as upstream batch.cpp). */
    if (scale < 0 && vertex_count < 0 && face_count < 0) {
      vertex_count = int(V.cols() / 16);
    }

    if (scale > 0) {
      Float face_area = posy == 4 ? (scale * scale) :
                                    (std::sqrt(3.f) / 4.f * scale * scale);
      face_count = int(stats.mSurfaceArea / face_area);
      vertex_count = posy == 4 ? face_count : (face_count / 2);
    }
    else if (face_count > 0) {
      Float face_area = stats.mSurfaceArea / Float(face_count);
      vertex_count = posy == 4 ? face_count : (face_count / 2);
      scale = posy == 4 ? std::sqrt(face_area) :
                          (2 * std::sqrt(face_area * std::sqrt(1.f / 3.f)));
    }
    else if (vertex_count > 0) {
      face_count = posy == 4 ? vertex_count : (vertex_count * 2);
      Float face_area = stats.mSurfaceArea / Float(face_count);
      scale = posy == 4 ? std::sqrt(face_area) :
                          (2 * std::sqrt(face_area * std::sqrt(1.f / 3.f)));
    }

    if (!(scale > 0) || !std::isfinite(scale)) {
      return;
    }

    if (check_canceled(0.05f, update_cb, update_cb_data)) {
      return;
    }

    MultiResolutionHierarchy mRes;

    /* Subdivide if input is too coarse for the desired edge length. */
    VectorXu V2E, E2E;
    VectorXb boundary, nonManifold;
    if (stats.mMaximumEdgeLength * 2 > scale ||
        stats.mMaximumEdgeLength > stats.mAverageEdgeLength * 2)
    {
      build_dedge(F, V, V2E, E2E, boundary, nonManifold);
      subdivide(F,
                V,
                V2E,
                E2E,
                boundary,
                nonManifold,
                std::min(scale / 2, (Float)stats.mAverageEdgeLength * 2),
                deterministic);
    }

    build_dedge(F, V, V2E, E2E, boundary, nonManifold);
    adj = generate_adjacency_matrix_uniform(F, V2E, E2E, nonManifold);

    if (creaseAngle >= 0) {
      generate_crease_normals(
          F, V, V2E, E2E, boundary, nonManifold, creaseAngle, N, crease_in);
    }
    else {
      generate_smooth_normals(F, V, V2E, E2E, nonManifold, N);
    }

    if (data->hard_verts && data->hard_verts_num > 0) {
      for (int i = 0; i < data->hard_verts_num; i++) {
        const int vi = data->hard_verts[i];
        if (vi >= 0 && vi < int(V.cols())) {
          crease_in.insert(uint32_t(vi));
        }
      }
    }
    if (data->hard_edge_v0 && data->hard_edge_v1 && data->hard_edges_num > 0) {
      for (int i = 0; i < data->hard_edges_num; i++) {
        const int a = data->hard_edge_v0[i];
        const int b = data->hard_edge_v1[i];
        if (a < 0 || b < 0 || a >= int(V.cols()) || b >= int(V.cols()) || a == b) {
          continue;
        }
        crease_in.insert(uint32_t(a));
        crease_in.insert(uint32_t(b));
      }
    }

    compute_dual_vertex_areas(F, V, V2E, E2E, nonManifold, A);
    mRes.setE2E(std::move(E2E));

    mRes.setAdj(std::move(adj));
    adj = nullptr;
    mRes.setF(std::move(F));
    mRes.setV(std::move(V));
    mRes.setA(std::move(A));
    mRes.setN(std::move(N));
    mRes.setScale(scale);
    mRes.build(deterministic);
    mRes.resetSolution();

    if (check_canceled(0.15f, update_cb, update_cb_data)) {
      return;
    }

    if (align_to_boundaries) {
      mRes.clearConstraints();
      for (uint32_t i = 0; i < 3 * mRes.F().cols(); ++i) {
        if (mRes.E2E()[i] == INVALID) {
          uint32_t i0 = mRes.F()(i % 3, i / 3);
          uint32_t i1 = mRes.F()((i + 1) % 3, i / 3);
          Vector3f p0 = mRes.V().col(i0), p1 = mRes.V().col(i1);
          Vector3f edge = p1 - p0;
          if (edge.squaredNorm() > 0) {
            edge.normalize();
            mRes.CO().col(i0) = p0;
            mRes.CO().col(i1) = p1;
            mRes.CQ().col(i0) = mRes.CQ().col(i1) = edge;
            mRes.CQw()[i0] = mRes.CQw()[i1] = mRes.COw()[i0] = mRes.COw()[i1] = 1.0f;
          }
        }
      }
      mRes.propagateConstraints(rosy, posy);
    }

    /* User hard points / edges: pin positions and align orientation along hard edges. */
    {
      bool any = false;
      const int nv = int(mRes.V().cols());
      if (data->hard_verts && data->hard_verts_num > 0) {
        for (int i = 0; i < data->hard_verts_num; i++) {
          const int vi = data->hard_verts[i];
          if (vi < 0 || vi >= nv) {
            continue;
          }
          mRes.CO().col(vi) = mRes.V().col(vi);
          mRes.COw()[uint32_t(vi)] = 1.0f;
          any = true;
        }
      }
      if (data->hard_edge_v0 && data->hard_edge_v1 && data->hard_edges_num > 0) {
        for (int i = 0; i < data->hard_edges_num; i++) {
          const int a = data->hard_edge_v0[i];
          const int b = data->hard_edge_v1[i];
          if (a < 0 || b < 0 || a >= nv || b >= nv || a == b) {
            continue;
          }
          Vector3f p0 = mRes.V().col(a), p1 = mRes.V().col(b);
          Vector3f edge = p1 - p0;
          if (edge.squaredNorm() > 0) {
            edge.normalize();
            mRes.CO().col(a) = p0;
            mRes.CO().col(b) = p1;
            mRes.CQ().col(a) = mRes.CQ().col(b) = edge;
            mRes.CQw()[uint32_t(a)] = mRes.CQw()[uint32_t(b)] = mRes.COw()[uint32_t(a)] =
                mRes.COw()[uint32_t(b)] = 1.0f;
            any = true;
          }
        }
      }
      if (any) {
        mRes.propagateConstraints(rosy, posy);
      }
    }

    if (smooth_iter > 0) {
      bvh = new BVH(&mRes.F(), &mRes.V(), &mRes.N(), stats.mAABB);
      bvh->build();
    }

    if (check_canceled(0.2f, update_cb, update_cb_data)) {
      delete bvh;
      return;
    }

    Optimizer optimizer(mRes, false);
    optimizer.setRoSy(rosy);
    optimizer.setPoSy(posy);
    optimizer.setExtrinsic(extrinsic);

    optimizer.optimizeOrientations(-1);
    optimizer.notify();
    optimizer.wait();

    if (check_canceled(0.55f, update_cb, update_cb_data)) {
      optimizer.shutdown();
      delete bvh;
      return;
    }

    optimizer.optimizePositions(-1);
    optimizer.notify();
    optimizer.wait();
    optimizer.shutdown();

    if (check_canceled(0.8f, update_cb, update_cb_data)) {
      delete bvh;
      return;
    }

    MatrixXf O_extr, N_extr, Nf_extr;
    std::vector<std::vector<TaggedLink>> adj_extr;
    extract_graph(mRes,
                  extrinsic,
                  rosy,
                  posy,
                  adj_extr,
                  O_extr,
                  N_extr,
                  crease_in,
                  crease_out,
                  deterministic);

    MatrixXu F_extr;
    extract_faces(adj_extr,
                  O_extr,
                  N_extr,
                  Nf_extr,
                  F_extr,
                  posy,
                  mRes.scale(),
                  crease_out,
                  true,
                  pure_quad,
                  bvh,
                  smooth_iter);

    delete bvh;
    bvh = nullptr;

    if (F_extr.cols() == 0 || O_extr.cols() == 0) {
      return;
    }

    /* ---- Pack variable faces for Blender. ----
     * Instant Meshes stores triangles as degenerate quads when F.rows()==4
     * (F(2,f) == F(3,f)). Emit true 3- or 4-gons. */
    const int face_rows = int(F_extr.rows());
    const int nF = int(F_extr.cols());
    const int nV = int(O_extr.cols());

    std::vector<int> offsets;
    std::vector<int> corners;
    offsets.reserve(nF + 1);
    corners.reserve(nF * face_rows);
    offsets.push_back(0);

    for (int f = 0; f < nF; f++) {
      int sides = face_rows;
      if (face_rows == 4 && F_extr(2, f) == F_extr(3, f)) {
        sides = 3;
      }
      for (int k = 0; k < sides; k++) {
        const uint32_t vi = F_extr(k, f);
        if (vi >= uint32_t(nV)) {
          /* Skip broken face. */
          while (int(corners.size()) > offsets.back()) {
            corners.pop_back();
          }
          sides = 0;
          break;
        }
        corners.push_back(int(vi));
      }
      if (sides >= 3) {
        offsets.push_back(int(corners.size()));
      }
    }

    const int out_faces = int(offsets.size()) - 1;
    if (out_faces <= 0) {
      return;
    }

    data->out_totverts = nV;
    data->out_totfaces = out_faces;
    data->out_totcorners = int(corners.size());

    data->out_verts = static_cast<float *>(std::malloc(sizeof(float) * size_t(size_t(nV) * 3)));
    data->out_face_offsets = static_cast<int *>(std::malloc(sizeof(int) * size_t(size_t(out_faces) + 1)));
    data->out_corner_verts = static_cast<int *>(std::malloc(sizeof(int) * size_t(corners.size())));

    for (int i = 0; i < nV; i++) {
      data->out_verts[i * 3 + 0] = O_extr(0, i);
      data->out_verts[i * 3 + 1] = O_extr(1, i);
      data->out_verts[i * 3 + 2] = O_extr(2, i);
    }
    std::memcpy(data->out_face_offsets, offsets.data(), sizeof(int) * (out_faces + 1));
    std::memcpy(data->out_corner_verts, corners.data(), sizeof(int) * corners.size());

    check_canceled(1.0f, update_cb, update_cb_data);
  }
  catch (...) {
    delete bvh;
    if (data->out_verts) {
      std::free(data->out_verts);
      data->out_verts = nullptr;
    }
    if (data->out_face_offsets) {
      std::free(data->out_face_offsets);
      data->out_face_offsets = nullptr;
    }
    if (data->out_corner_verts) {
      std::free(data->out_corner_verts);
      data->out_corner_verts = nullptr;
    }
    data->out_totverts = 0;
    data->out_totfaces = 0;
    data->out_totcorners = 0;
  }
}

void IMESH_orientation_field(InstantMeshesOrientationData *data)
{
  data->out_Q = nullptr;
  data->out_N = nullptr;
  data->out_dirs = nullptr;
  data->success = false;

  if (!data->verts || !data->faces || data->totverts < 3 || data->totfaces < 1) {
    return;
  }

  int rosy = data->rosy;
  if (rosy != 2 && rosy != 4 && rosy != 6) {
    return;
  }

  const Float creaseAngle = data->crease_angle_deg;
  const bool extrinsic = data->extrinsic;
  const bool deterministic = data->deterministic;
  instant_meshes_level_iterations = std::max(1, data->iterations > 0 ? data->iterations : 6);

  MatrixXu F(3, data->totfaces);
  MatrixXf V(3, data->totverts);
  for (int i = 0; i < data->totverts; i++) {
    V(0, i) = data->verts[i * 3 + 0];
    V(1, i) = data->verts[i * 3 + 1];
    V(2, i) = data->verts[i * 3 + 2];
  }
  for (int i = 0; i < data->totfaces; i++) {
    F(0, i) = uint32_t(data->faces[i * 3 + 0]);
    F(1, i) = uint32_t(data->faces[i * 3 + 1]);
    F(2, i) = uint32_t(data->faces[i * 3 + 2]);
  }

  try {
    MeshStats stats = compute_mesh_stats(F, V, deterministic);
    /* Orientation field does not need a target edge length for extraction, but hierarchy
     * still stores scale; use mean edge length as a neutral value. */
    Float scale = Float(stats.mAverageEdgeLength);
    if (!(scale > 0) || !std::isfinite(scale)) {
      scale = 1.0f;
    }

    MatrixXf N;
    VectorXf A;
    std::set<uint32_t> crease_in;
    VectorXu V2E, E2E;
    VectorXb boundary, nonManifold;

    build_dedge(F, V, V2E, E2E, boundary, nonManifold);
    AdjacencyMatrix adj = generate_adjacency_matrix_uniform(F, V2E, E2E, nonManifold);

    if (creaseAngle >= 0) {
      generate_crease_normals(
          F, V, V2E, E2E, boundary, nonManifold, creaseAngle, N, crease_in);
    }
    else {
      generate_smooth_normals(F, V, V2E, E2E, nonManifold, N);
    }
    compute_dual_vertex_areas(F, V, V2E, E2E, nonManifold, A);

    MultiResolutionHierarchy mRes;
    mRes.setE2E(std::move(E2E));
    mRes.setAdj(std::move(adj));
    mRes.setF(std::move(F));
    mRes.setV(std::move(V));
    mRes.setA(std::move(A));
    mRes.setN(std::move(N));
    mRes.setScale(scale);
    mRes.build(deterministic);
    mRes.resetSolution();

    Optimizer optimizer(mRes, false);
    optimizer.setRoSy(rosy);
    optimizer.setPoSy(4); /* unused for orientation-only */
    optimizer.setExtrinsic(extrinsic);
    optimizer.optimizeOrientations(-1);
    optimizer.notify();
    optimizer.wait();
    optimizer.shutdown();

    const MatrixXf &Q = mRes.Q(0);
    const MatrixXf &N0 = mRes.N(0);
    const int nV = int(Q.cols());
    if (nV != data->totverts) {
      /* Hierarchy may drop isolated verts only in theory; Instant Meshes keeps V.cols(). */
    }
    if (nV <= 0) {
      return;
    }

    data->out_Q = static_cast<float *>(std::malloc(sizeof(float) * size_t(size_t(nV) * 3)));
    data->out_N = static_cast<float *>(std::malloc(sizeof(float) * size_t(size_t(nV) * 3)));
    data->out_dirs = static_cast<float *>(std::malloc(sizeof(float) * size_t(size_t(nV) * size_t(rosy) * 3)));

    for (int i = 0; i < nV; i++) {
      Vector3f q = Q.col(i);
      Vector3f n = N0.col(i);
      /* Project and normalize representative. */
      q = q - n * n.dot(q);
      Float qn = q.norm();
      if (qn > RCPOVERFLOW) {
        q /= qn;
      }
      else {
        /* Degenerate: pick any tangent. */
        Vector3f t0, t1;
        coordinate_system(n, t0, t1);
        q = t0;
      }

      data->out_Q[i * 3 + 0] = q.x();
      data->out_Q[i * 3 + 1] = q.y();
      data->out_Q[i * 3 + 2] = q.z();
      data->out_N[i * 3 + 0] = n.x();
      data->out_N[i * 3 + 1] = n.y();
      data->out_N[i * 3 + 2] = n.z();

      for (int k = 0; k < rosy; k++) {
        Vector3f d;
        if (rosy == 2) {
          d = rotate180_by(q, n, k);
        }
        else if (rosy == 4) {
          d = rotate90_by(q, n, k);
        }
        else {
          d = rotate60_by(q, n, k);
        }
        const int idx = (i * rosy + k) * 3;
        data->out_dirs[idx + 0] = d.x();
        data->out_dirs[idx + 1] = d.y();
        data->out_dirs[idx + 2] = d.z();
      }
    }

    data->success = true;
  }
  catch (...) {
    if (data->out_Q) {
      std::free(data->out_Q);
      data->out_Q = nullptr;
    }
    if (data->out_N) {
      std::free(data->out_N);
      data->out_N = nullptr;
    }
    if (data->out_dirs) {
      std::free(data->out_dirs);
      data->out_dirs = nullptr;
    }
    data->success = false;
  }
}
