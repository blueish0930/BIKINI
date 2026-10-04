/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Triangle remesh Geometry Node backend.
 *
 * Wraps pmp-library (MIT) uniform / adaptive isotropic remeshing.
 */

#include <cstdlib>

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_math_base.hh"
#include "BLI_math_constants.hh"
#include "BLI_offset_indices.hh"
#include "BLI_span.hh"

#include "DNA_mesh_types.h"

#include "BKE_mesh.hh"

#include "MEM_guardedalloc.h"

#include "GEO_mesh_triangle_remesh.hh"

#ifdef WITH_PMP_REMESH
#  include "pmp_remesh_capi.hpp"
#endif

namespace blender::geometry {

Mesh *mesh_triangle_remesh(const Mesh &src_mesh,
                           const TriangleRemeshOptions &options,
                           Span<int> hard_verts,
                           Span<int2> hard_edges)
{
#ifndef WITH_PMP_REMESH
  UNUSED_VARS(src_mesh, options, hard_verts, hard_edges);
  return BKE_mesh_new_nomain(0, 0, 0, 0);
#else
  if (src_mesh.verts_num < 3 || src_mesh.faces_num < 1) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }

  const Span<float3> positions = src_mesh.vert_positions();
  const Span<int> corner_verts = src_mesh.corner_verts();
  const OffsetIndices faces = src_mesh.faces();

  Array<int> tri_indices(src_mesh.faces_num * 3);
  for (const int face_i : faces.index_range()) {
    const IndexRange face = faces[face_i];
    if (face.size() != 3) {
      return BKE_mesh_new_nomain(0, 0, 0, 0);
    }
    tri_indices[face_i * 3 + 0] = corner_verts[face[0]];
    tri_indices[face_i * 3 + 1] = corner_verts[face[1]];
    tri_indices[face_i * 3 + 2] = corner_verts[face[2]];
  }

  PMPRemeshData data{};
  data.verts = positions.cast<float>().data();
  data.faces = tri_indices.data();
  data.totverts = positions.size();
  data.totfaces = src_mesh.faces_num;

  data.mode = (options.mode == TriangleRemeshMode::Adaptive) ? PMP_REMESH_ADAPTIVE :
                                                               PMP_REMESH_UNIFORM;
  data.edge_length = options.edge_length;
  data.min_edge_length = options.min_edge_length;
  data.max_edge_length = options.max_edge_length;
  data.approx_error = options.approx_error;
  data.iterations = math::max(options.iterations, 1);
  data.use_projection = options.use_projection;
  data.protect_boundaries = options.protect_boundaries;
  data.freeze_boundary_topology = options.preserve_boundary_topology;

  /* pmp detect_features expects degrees. */
  if (options.crease_angle > 0.0f) {
    data.crease_angle_deg = options.crease_angle * (180.0f / float(M_PI));
  }
  else {
    data.crease_angle_deg = 0.0f;
  }

  data.hard_verts = hard_verts.is_empty() ? nullptr : hard_verts.data();
  data.hard_verts_num = int(hard_verts.size());
  Array<int> hard_a;
  Array<int> hard_b;
  if (!hard_edges.is_empty()) {
    hard_a.reinitialize(hard_edges.size());
    hard_b.reinitialize(hard_edges.size());
    for (const int i : hard_edges.index_range()) {
      hard_a[i] = hard_edges[i][0];
      hard_b[i] = hard_edges[i][1];
    }
    data.hard_edge_v0 = hard_a.data();
    data.hard_edge_v1 = hard_b.data();
    data.hard_edges_num = int(hard_edges.size());
  }

  if (!PMP_triangle_remesh(&data) || data.out_verts == nullptr || data.out_totfaces <= 0) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }

  const int nV = data.out_totverts;
  const int nF = data.out_totfaces;
  const int nC = nF * 3;

  Mesh *mesh = BKE_mesh_new_nomain(nV, 0, nF, nC);
  BKE_mesh_copy_parameters(mesh, &src_mesh);

  mesh->vert_positions_for_write().copy_from(
      Span(reinterpret_cast<float3 *>(data.out_verts), nV));

  MutableSpan<int> face_offsets = mesh->face_offsets_for_write();
  MutableSpan<int> out_corners = mesh->corner_verts_for_write();
  for (int f = 0; f < nF; f++) {
    face_offsets[f] = f * 3;
    out_corners[f * 3 + 0] = data.out_faces[f * 3 + 0];
    out_corners[f * 3 + 1] = data.out_faces[f * 3 + 1];
    out_corners[f * 3 + 2] = data.out_faces[f * 3 + 2];
  }
  face_offsets[nF] = nC;

  bke::mesh_calc_edges(*mesh, false, false);

  free(data.out_verts);
  free(data.out_faces);

  return mesh;
#endif
}

}  // namespace geometry
