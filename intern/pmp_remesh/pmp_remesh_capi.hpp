/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup intern_pmp_remesh
 *
 * C API around pmp-library isotropic / adaptive triangle remeshing
 * (https://github.com/pmp-library/pmp-library, MIT).
 *
 * Algorithm: Botsch & Kobbelt, "A Remeshing Approach to Multiresolution Modeling"
 * (SGP 2004) + Dunyach et al. adaptive sizing — the same family as Houdini Remesh SOP.
 */

#ifdef __cplusplus
extern "C" {
#endif

/** Remesh mode. */
enum {
  PMP_REMESH_UNIFORM = 0,
  PMP_REMESH_ADAPTIVE = 1,
};

/**
 * Input / output for triangle remesh.
 *
 * Input verts: 3 * totverts floats (xyz interleaved).
 * Input faces: 3 * totfaces ints (triangle corner indices, 0-based).
 *
 * Output is always triangles. Caller frees out_verts / out_faces with free().
 */
struct PMPRemeshData {
  const float *verts;
  const int *faces;
  int totverts;
  int totfaces;

  float *out_verts;
  int *out_faces; /* 3 * out_totfaces */
  int out_totverts;
  int out_totfaces;

  /** PMP_REMESH_UNIFORM or PMP_REMESH_ADAPTIVE. */
  int mode;

  /** Uniform: target edge length. Must be > 0. */
  float edge_length;

  /** Adaptive: min / max edge length and approximation error. Must be > 0. */
  float min_edge_length;
  float max_edge_length;
  float approx_error;

  /** Remesh iterations (split/collapse/flip/smooth cycles). Default 10. */
  int iterations;

  /** Project vertices back onto the input surface after relaxation. */
  bool use_projection;

  /**
   * Dihedral angle in degrees for feature (crease) detection.
   * Edges with larger dihedral angles are locked. ≤ 0 disables.
   */
  float crease_angle_deg;

  /** Lock open boundary edges/vertices as features. */
  bool protect_boundaries;

  /**
   * Freeze the open-boundary vertex ring: do not split or collapse
   * open-boundary edges (no vertices inserted or deleted on the boundary).
   */
  bool freeze_boundary_topology;

  /**
   * Extra feature (hard) vertices / edges, 0-based indices into the input mesh.
   * Nullable. Hard edges also lock their endpoints.
   */
  const int *hard_verts;
  int hard_verts_num;
  const int *hard_edge_v0;
  const int *hard_edge_v1;
  int hard_edges_num;
};

/**
 * Run pmp-library remeshing in memory.
 * On failure, out_* stay nullptr / 0.
 * \return true on success with non-empty triangle mesh.
 */
bool PMP_triangle_remesh(struct PMPRemeshData *data);

#ifdef __cplusplus
}
#endif
