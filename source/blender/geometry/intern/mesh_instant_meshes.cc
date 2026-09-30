/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Instant Field-Aligned Meshes Geometry Node backend.
 *
 * Wraps the official Instant Meshes core (wjakob/instant-meshes, BSD):
 * multiresolution hierarchy 鈫?orientation field 鈫?position field 鈫?extract + cleanup.
 */

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_math_base.hh"
#include "BLI_math_constants.hh"
#include "BLI_offset_indices.hh"
#include "BLI_span.hh"

#include "DNA_mesh_types.h"

#include "BKE_mesh.hh"

#include "MEM_guardedalloc.h"
#include <cstdlib>

#include "GEO_mesh_instant_meshes.hh"

#ifdef WITH_INSTANT_MESHES
#  include "instant_meshes_capi.hpp"
#endif

namespace blender::geometry {

Mesh *mesh_instant_meshes(const Mesh &src_mesh,
                          const InstantMeshesOptions &options,
                          Span<int> hard_verts,
                          Span<int2> hard_edges)
{
#ifndef WITH_INSTANT_MESHES
  UNUSED_VARS(src_mesh, options, hard_verts, hard_edges);
  return BKE_mesh_new_nomain(0, 0, 0, 0);
#else
  if (src_mesh.verts_num < 3 || src_mesh.faces_num < 1) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }

  /* Geometry nodes always triangulate before calling us. */
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

  InstantMeshesRemeshData data{};
  data.verts = positions.cast<float>().data();
  data.faces = tri_indices.data();
  data.totverts = positions.size();
  data.totfaces = src_mesh.faces_num;

  data.scale = options.edge_length > 0.0f ? options.edge_length : -1.0f;
  data.face_count = options.face_count > 0 ? options.face_count : -1;
  data.vertex_count = options.vertex_count > 0 ? options.vertex_count : -1;

  /* Official core uses degrees; Geometry Nodes socket is radians. */
  if (options.crease_angle >= 0.0f) {
    data.crease_angle_deg = options.crease_angle * (180.0f / float(M_PI));
  }
  else {
    data.crease_angle_deg = -1.0f;
  }

  switch (options.mode) {
    case InstantMeshesMode::Quad:
      data.rosy = 4;
      data.posy = 4;
      break;
    case InstantMeshesMode::Triangle:
      /* Official triangle mode: 6-RoSy orientation, 3-PoSy position lattice. */
      data.rosy = 6;
      data.posy = 3;
      break;
  }

  data.extrinsic = options.extrinsic;
  data.align_to_boundaries = options.align_to_boundaries;
  data.pure_quad = options.pure_quad && options.mode == InstantMeshesMode::Quad;
  data.deterministic = options.deterministic;
  data.iterations = math::max(options.iterations_per_level, 1);
  data.smooth_iter = math::max(options.smooth_iterations, 0);

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

  IMESH_instant_meshes_remesh(&data, nullptr, nullptr);

  if (data.out_verts == nullptr || data.out_totfaces <= 0) {
    return BKE_mesh_new_nomain(0, 0, 0, 0);
  }

  Mesh *mesh = BKE_mesh_new_nomain(
      data.out_totverts, 0, data.out_totfaces, data.out_totcorners);
  BKE_mesh_copy_parameters(mesh, &src_mesh);

  mesh->vert_positions_for_write().copy_from(
      Span(reinterpret_cast<float3 *>(data.out_verts), data.out_totverts));

  MutableSpan<int> face_offsets = mesh->face_offsets_for_write();
  face_offsets.copy_from(Span(data.out_face_offsets, data.out_totfaces + 1));

  mesh->corner_verts_for_write().copy_from(
      Span(data.out_corner_verts, data.out_totcorners));

  bke::mesh_calc_edges(*mesh, false, false);

  free(data.out_verts);
  free(data.out_face_offsets);
  free(data.out_corner_verts);

  return mesh;
#endif
}

}  // namespace blender::geometry
