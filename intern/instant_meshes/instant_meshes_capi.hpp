/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup intern_instant_meshes
 *
 * C API around the official Instant Meshes remeshing core (wjakob/instant-meshes, BSD).
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Input / output buffers for Instant Meshes batch remesh.
 *
 * Input verts: 3 * totverts floats (xyz interleaved).
 * Input faces: 3 * totfaces ints (triangle corner indices).
 *
 * Output faces use variable sides (3 or 4 typically). Faces are stored as
 * flat corner index list; face_offsets has (out_totfaces + 1) entries like Blender.
 * Caller must free out_verts, out_face_offsets, out_corner_verts with free().
 */
struct InstantMeshesRemeshData {
  const float *verts;
  const int *faces;
  int totverts;
  int totfaces;

  float *out_verts;
  int *out_face_offsets;
  int *out_corner_verts;
  int out_totverts;
  int out_totfaces;
  int out_totcorners;

  /** Target edge length ρ. ≤ 0 means derive from face/vertex count or auto. */
  float scale;
  /** Desired face count; ignored if ≤ 0. Mutually exclusive with scale/vertex_count. */
  int face_count;
  /** Desired vertex count; ignored if ≤ 0. */
  int vertex_count;

  /**
   * Crease dihedral angle in degrees.
   * < 0 disables creases (smooth normals). Official CLI default is disabled.
   */
  float crease_angle_deg;

  /** Orientation symmetry: 2, 4, or 6. */
  int rosy;
  /** Position symmetry: 3 (triangle lattice) or 4 (quad lattice). posy=6 is accepted as 3. */
  int posy;

  bool extrinsic;
  bool align_to_boundaries;
  /** Official default: pure tri/quad (not dominant). */
  bool pure_quad;
  bool deterministic;

  /** Gauss–Seidel iterations per hierarchy level (paper / official = 6). */
  int iterations;
  /** Extract-time smooth + ray reproject iterations (official default 2). */
  int smooth_iter;

  /** Extra crease / constraint vertices (0-based). Nullable. */
  const int *hard_verts;
  int hard_verts_num;
  /** Extra crease edges as input vertex pairs. Nullable. */
  const int *hard_edge_v0;
  const int *hard_edge_v1;
  int hard_edges_num;
};

/**
 * Run the full Instant Meshes pipeline in memory (batch path without file I/O).
 * On cancel or failure, out_* pointers remain nullptr.
 */
void IMESH_instant_meshes_remesh(struct InstantMeshesRemeshData *data,
                                 void (*update_cb)(void *, float progress, int *cancel),
                                 void *update_cb_data);

/**
 * N-RoSy orientation field only (no position field / extract).
 *
 * Input: triangle mesh.
 * Output (caller frees with free()):
 *   out_Q: 3 * totverts — representative tangent direction per vertex
 *   out_N: 3 * totverts — vertex normals used by the solver
 *   out_dirs: 3 * totverts * rosy — all N symmetric directions
 *             layout: vertex v, direction k → out_dirs[(v * rosy + k) * 3 + {0,1,2}]
 *
 * On failure out_* stay nullptr and success is false.
 */
struct InstantMeshesOrientationData {
  const float *verts;
  const int *faces;
  int totverts;
  int totfaces;

  /** 2, 4, or 6. */
  int rosy;
  float crease_angle_deg;
  bool extrinsic;
  bool deterministic;
  int iterations;

  float *out_Q;
  float *out_N;
  float *out_dirs;
  bool success;
};

void IMESH_orientation_field(struct InstantMeshesOrientationData *data);

#ifdef __cplusplus
}
#endif
