/* SPDX-FileCopyrightText: 2021 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "BLI_array.hh"

#include "BKE_mesh.hh"

#include "ED_uvedit.hh"

#include "extract_mesh.hh"

#include "draw_cache_impl.hh"

#include "draw_subdivision.hh"

namespace blender::draw {

/* ---------------------------------------------------------------------- */
/** \name Extract Edit UV Data / Flags
 * \{ */

static const GPUVertFormat &edituv_data_format()
{
  static const GPUVertFormat format = []() {
    GPUVertFormat format{};
    /* WARNING: Adjust #EditLoopData struct accordingly. */
    GPU_vertformat_attr_add(&format, "data", gpu::VertAttrType::UINT_8_8_8_8);
    GPU_vertformat_alias_add(&format, "flag");
    return format;
  }();
  return format;
}

static void extract_edituv_data_bm(const MeshRenderData &mr, MutableSpan<EditLoopData> vbo_data)
{
  const BMesh &bm = *mr.bm;
  const BMUVOffsets offsets = BM_uv_map_offsets_get(&bm);
  threading::parallel_for(IndexRange(bm.totface), 2048, [&](const IndexRange range) {
    for (const int face_index : range) {
      const BMFace &face = *BM_face_at_index(&const_cast<BMesh &>(bm), face_index);
      EditLoopData face_value = {};
      mesh_render_data_face_flag(mr, &face, offsets, face_value);
      const BMLoop *loop = BM_FACE_FIRST_LOOP(&face);
      for ([[maybe_unused]] const int i : IndexRange(face.len)) {
        const int index = BM_elem_index_get(loop);
        EditLoopData &value = vbo_data[index];
        value = face_value;
        mesh_render_data_loop_flag(mr, loop, offsets, value);
        mesh_render_data_loop_edge_flag(mr, loop, offsets, value);
        loop = loop->next;
      }
    }
  });
}

static void extract_edituv_data_mesh(const MeshRenderData &mr, MutableSpan<EditLoopData> vbo_data)
{
  const BMesh &bm = *mr.bm;
  const BMUVOffsets offsets = BM_uv_map_offsets_get(&bm);
  const OffsetIndices faces = mr.faces;
  const Span<int> corner_verts = mr.corner_verts;
  const Span<int> corner_edges = mr.corner_edges;
  threading::parallel_for(faces.index_range(), 2048, [&](const IndexRange range) {
    for (const int face_index : range) {
      const IndexRange face = faces[face_index];
      BMFace *face_orig = bm_original_face_get(mr, face_index);
      if (!face_orig) {
        vbo_data.slice(face).fill({});
        continue;
      }
      for (const int corner : face) {
        EditLoopData &value = vbo_data[corner];
        value = {};
        BMVert *vert = bm_original_vert_get(mr, corner_verts[corner]);
        BMEdge *edge = bm_original_edge_get(mr, corner_edges[corner]);
        if (edge && vert) {
          /* Loop on an edge endpoint. */
          BMLoop *l = BM_face_edge_share_loop(face_orig, edge);
          mesh_render_data_loop_flag(mr, l, offsets, value);
          mesh_render_data_loop_edge_flag(mr, l, offsets, value);
        }
        else {
          if (edge == nullptr) {
            /* Find if the loop's vert is not part of an edit edge.
             * For this, we check if the previous loop was on an edge. */
            const int corner_prev = bke::mesh::face_corner_prev(face, corner);
            edge = bm_original_edge_get(mr, corner_edges[corner_prev]);
          }
          if (edge) {
            /* Mapped points on an edge between two edit verts. */
            BMLoop *l = BM_face_edge_share_loop(face_orig, edge);
            mesh_render_data_loop_edge_flag(mr, l, offsets, value);
          }
        }
      }
    }
  });
}

/** e_flag bits. Match #UV_VERT_MIRROR / #UV_EDGE_MIRROR / #UV_FACE_MIRROR (second byte). */
static constexpr uchar UV_MIRROR_VERT_EF = 1 << 0;
static constexpr uchar UV_MIRROR_EDGE_EF = 1 << 1;
static constexpr uchar UV_MIRROR_FACE_EF = 1 << 2;

static void extract_edituv_mirror_flags(const MeshRenderData &mr, MutableSpan<EditLoopData> vbo_data)
{
  if (mr.bm == nullptr || mr.edit_bmesh == nullptr || mr.toolsettings == nullptr) {
    return;
  }
  /* The cage/evaluated mesh does not reliably carry #Mesh::uv_symmetry. */
  const Mesh *uv_mesh = (mr.mesh_orig != nullptr) ? mr.mesh_orig : mr.mesh;
  if (uv_mesh == nullptr) {
    return;
  }
  UVMirrorSettings settings;
  if (!ED_uvedit_mirror_settings(mr.toolsettings, uv_mesh, &settings)) {
    return;
  }
  UVMirrorIndex *index = ED_uvedit_mirror_index_create(mr.toolsettings, mr.edit_bmesh, uv_mesh);
  if (index == nullptr) {
    return;
  }
  const BMesh &bm = *mr.bm;
  const BMUVOffsets offsets = BM_uv_map_offsets_get(&bm);

  float active_uv[2] = {settings.center[0], settings.center[1]};
  bool have_active = false;
  if (mr.efa_act_uv != nullptr && !BM_elem_flag_test(mr.efa_act_uv, BM_ELEM_HIDDEN) &&
      mr.efa_act_uv->len > 0)
  {
    float acc[2] = {0.0f, 0.0f};
    const BMLoop *l = mr.efa_act_uv->l_first;
    do {
      const float *uv = BM_ELEM_CD_GET_FLOAT_P(l, offsets.uv);
      acc[0] += uv[0];
      acc[1] += uv[1];
    } while ((l = l->next) != mr.efa_act_uv->l_first);
    active_uv[0] = acc[0] / float(mr.efa_act_uv->len);
    active_uv[1] = acc[1] / float(mr.efa_act_uv->len);
    have_active = true;
  }

  auto on_active_side = [&](const float uv[2]) {
    if (!have_active) {
      return true;
    }
    const float axis_u = settings.center[0];
    if ((settings.axis_mask & ME_UV_SYMMETRY_U) &&
        fabsf(active_uv[0] - axis_u) > settings.threshold &&
        fabsf(uv[0] - axis_u) > settings.threshold &&
        ((active_uv[0] > axis_u) != (uv[0] > axis_u)))
    {
      return false;
    }
    const float axis_v = settings.center[1];
    if ((settings.axis_mask & ME_UV_SYMMETRY_V) &&
        fabsf(active_uv[1] - axis_v) > settings.threshold &&
        fabsf(uv[1] - axis_v) > settings.threshold &&
        ((active_uv[1] > axis_v) != (uv[1] > axis_v)))
    {
      return false;
    }
    return true;
  };

  BMIter fiter, liter;
  BMFace *f;
  BMLoop *l;
  BM_ITER_MESH (f, &fiter, &const_cast<BMesh &>(bm), BM_FACES_OF_MESH) {
    if (!uvedit_face_visible_test_ex(mr.toolsettings, f)) {
      continue;
    }
    BM_ITER_ELEM (l, &liter, f, BM_LOOPS_OF_FACE) {
      if (!uvedit_uv_select_test_ex(mr.toolsettings, &bm, l, offsets)) {
        continue;
      }
      ED_uvedit_mirror_partners(index, l, [&](BMLoop *partner, int /*axis_mask*/) {
        const int i = BM_elem_index_get(partner);
        if (i < 0 || i >= vbo_data.size()) {
          return;
        }
        const float *uv = BM_ELEM_CD_GET_FLOAT_P(partner, offsets.uv);
        const bool selected = uvedit_uv_select_test_ex(mr.toolsettings, &bm, partner, offsets);
        if (selected && on_active_side(uv)) {
          return;
        }
        vbo_data[i].e_flag |= UV_MIRROR_VERT_EF;
      });
    }
  }

  BM_ITER_MESH (f, &fiter, &const_cast<BMesh &>(bm), BM_FACES_OF_MESH) {
    if (!uvedit_face_visible_test_ex(mr.toolsettings, f)) {
      continue;
    }
    /* An edge tints only that edge. The face interior fills only when every
     * corner is a mirror counterpart — two edges are not enough. */
    int corners = 0;
    int mirror_verts = 0;
    BM_ITER_ELEM (l, &liter, f, BM_LOOPS_OF_FACE) {
      const int i = BM_elem_index_get(l);
      const int i_next = BM_elem_index_get(l->next);
      const bool vert_mirror = i >= 0 && i < vbo_data.size() &&
                               (vbo_data[i].e_flag & UV_MIRROR_VERT_EF) != 0;
      const bool next_mirror = i_next >= 0 && i_next < vbo_data.size() &&
                               (vbo_data[i_next].e_flag & UV_MIRROR_VERT_EF) != 0;
      corners++;
      if (vert_mirror) {
        mirror_verts++;
      }
      if (vert_mirror && next_mirror && i >= 0 && i < vbo_data.size()) {
        vbo_data[i].e_flag |= UV_MIRROR_EDGE_EF;
      }
    }
    const bool face_mirror = corners > 0 && mirror_verts == corners;
    if (!face_mirror) {
      continue;
    }
    BM_ITER_ELEM (l, &liter, f, BM_LOOPS_OF_FACE) {
      const int i = BM_elem_index_get(l);
      if (i >= 0 && i < vbo_data.size()) {
        vbo_data[i].e_flag |= UV_MIRROR_FACE_EF;
      }
    }
  }

  ED_uvedit_mirror_index_free(index);
}

void extract_edituv_mirror_face_dots(const MeshRenderData &mr, MutableSpan<EditLoopData> face_data)
{
  if (face_data.is_empty() || mr.bm == nullptr || mr.edit_bmesh == nullptr ||
      mr.toolsettings == nullptr)
  {
    return;
  }
  const Mesh *uv_mesh = (mr.mesh_orig != nullptr) ? mr.mesh_orig : mr.mesh;
  if (uv_mesh == nullptr) {
    return;
  }
  UVMirrorSettings settings;
  if (!ED_uvedit_mirror_settings(mr.toolsettings, uv_mesh, &settings)) {
    return;
  }
  UVMirrorIndex *index = ED_uvedit_mirror_index_create(mr.toolsettings, mr.edit_bmesh, uv_mesh);
  if (index == nullptr) {
    return;
  }
  const BMesh &bm = *mr.bm;
  const BMUVOffsets offsets = BM_uv_map_offsets_get(&bm);
  Array<bool> loop_mirror(bm.totloop, false);

  float active_uv[2] = {settings.center[0], settings.center[1]};
  bool have_active = false;
  if (mr.efa_act_uv != nullptr && !BM_elem_flag_test(mr.efa_act_uv, BM_ELEM_HIDDEN) &&
      mr.efa_act_uv->len > 0)
  {
    float acc[2] = {0.0f, 0.0f};
    const BMLoop *loop = mr.efa_act_uv->l_first;
    do {
      const float *uv = BM_ELEM_CD_GET_FLOAT_P(loop, offsets.uv);
      acc[0] += uv[0];
      acc[1] += uv[1];
    } while ((loop = loop->next) != mr.efa_act_uv->l_first);
    active_uv[0] = acc[0] / float(mr.efa_act_uv->len);
    active_uv[1] = acc[1] / float(mr.efa_act_uv->len);
    have_active = true;
  }
  auto on_active_side = [&](const float uv[2]) {
    if (!have_active) {
      return true;
    }
    const float axis_u = settings.center[0];
    if ((settings.axis_mask & ME_UV_SYMMETRY_U) &&
        fabsf(active_uv[0] - axis_u) > settings.threshold &&
        fabsf(uv[0] - axis_u) > settings.threshold &&
        ((active_uv[0] > axis_u) != (uv[0] > axis_u)))
    {
      return false;
    }
    const float axis_v = settings.center[1];
    if ((settings.axis_mask & ME_UV_SYMMETRY_V) &&
        fabsf(active_uv[1] - axis_v) > settings.threshold &&
        fabsf(uv[1] - axis_v) > settings.threshold &&
        ((active_uv[1] > axis_v) != (uv[1] > axis_v)))
    {
      return false;
    }
    return true;
  };

  BMIter fiter, liter;
  BMFace *f;
  BMLoop *l;
  BM_ITER_MESH (f, &fiter, &const_cast<BMesh &>(bm), BM_FACES_OF_MESH) {
    if (!uvedit_face_visible_test_ex(mr.toolsettings, f)) {
      continue;
    }
    BM_ITER_ELEM (l, &liter, f, BM_LOOPS_OF_FACE) {
      if (!uvedit_uv_select_test_ex(mr.toolsettings, &const_cast<BMesh &>(bm), l, offsets)) {
        continue;
      }
      ED_uvedit_mirror_partners(index, l, [&](BMLoop *partner, int /*axis_mask*/) {
        const int i = BM_elem_index_get(partner);
        if (i < 0 || i >= loop_mirror.size()) {
          return;
        }
        const float *uv = BM_ELEM_CD_GET_FLOAT_P(partner, offsets.uv);
        const bool selected = uvedit_uv_select_test_ex(
            mr.toolsettings, &const_cast<BMesh &>(bm), partner, offsets);
        if (selected && on_active_side(uv)) {
          return;
        }
        loop_mirror[i] = true;
      });
    }
  }
  ED_uvedit_mirror_index_free(index);

  auto face_is_mirror = [&](const BMFace *face) {
    if (face == nullptr || face->len == 0 || BM_elem_flag_test(face, BM_ELEM_HIDDEN)) {
      return false;
    }
    int corners = 0;
    int mirror_verts = 0;
    const BMLoop *loop = face->l_first;
    do {
      const int i = BM_elem_index_get(loop);
      corners++;
      if (i >= 0 && i < loop_mirror.size() && loop_mirror[i]) {
        mirror_verts++;
      }
    } while ((loop = loop->next) != face->l_first);
    return corners > 0 && mirror_verts == corners;
  };

  if (mr.extract_type == MeshExtractType::BMesh) {
    threading::parallel_for(IndexRange(bm.totface), 2048, [&](const IndexRange range) {
      for (const int face_index : range) {
        if (face_index < 0 || face_index >= face_data.size()) {
          continue;
        }
        const BMFace *face = BM_face_at_index(&const_cast<BMesh &>(bm), face_index);
        if (face_is_mirror(face)) {
          face_data[face_index].e_flag |= UV_MIRROR_FACE_EF;
        }
      }
    });
    return;
  }
  if (mr.orig_index_face == nullptr) {
    return;
  }
  threading::parallel_for(IndexRange(face_data.size()), 2048, [&](const IndexRange range) {
    for (const int face_index : range) {
      const BMFace *face = bm_original_face_get(mr, face_index);
      if (face_is_mirror(face)) {
        face_data[face_index].e_flag |= UV_MIRROR_FACE_EF;
      }
    }
  });
}

gpu::VertBufPtr extract_edituv_data(const MeshRenderData &mr)
{
  gpu::VertBufPtr vbo = gpu::VertBufPtr(GPU_vertbuf_create_with_format(edituv_data_format()));
  GPU_vertbuf_data_alloc(*vbo, mr.corners_num);
  MutableSpan vbo_data = vbo->data<EditLoopData>();

  if (mr.extract_type == MeshExtractType::BMesh) {
    extract_edituv_data_bm(mr, vbo_data);
    extract_edituv_mirror_flags(mr, vbo_data);
  }
  else {
    extract_edituv_data_mesh(mr, vbo_data);
  }
  return vbo;
}

static void extract_edituv_data_iter_subdiv_bm(const MeshRenderData &mr,
                                               const BMUVOffsets &offsets,
                                               const Span<int> subdiv_loop_vert_index,
                                               const Span<int> subdiv_loop_edge_index,
                                               const int subdiv_quad_index,
                                               const BMFace *coarse_quad,
                                               MutableSpan<EditLoopData> vbo_data)
{

  uint start_loop_idx = subdiv_quad_index * 4;
  uint end_loop_idx = (subdiv_quad_index + 1) * 4;
  EditLoopData edit_loop_data_face = {};
  mesh_render_data_face_flag(mr, coarse_quad, offsets, edit_loop_data_face);
  for (uint i = start_loop_idx; i < end_loop_idx; i++) {
    const int vert_origindex = subdiv_loop_vert_index[i];
    int edge_origindex = subdiv_loop_edge_index[i];

    EditLoopData *edit_loop_data = &vbo_data[i];
    *edit_loop_data = edit_loop_data_face;

    if (vert_origindex != -1 && edge_origindex != -1) {
      BMEdge *eed = BM_edge_at_index(mr.bm, edge_origindex);
      /* Loop on an edge endpoint. */
      BMLoop *l = BM_face_edge_share_loop(const_cast<BMFace *>(coarse_quad), eed);
      mesh_render_data_loop_flag(mr, l, offsets, *edit_loop_data);
      mesh_render_data_loop_edge_flag(mr, l, offsets, *edit_loop_data);
    }
    else {
      if (edge_origindex == -1) {
        /* Find if the loop's vert is not part of an edit edge.
         * For this, we check if the previous loop was on an edge. */
        const uint loop_index_last = (i == start_loop_idx) ? end_loop_idx - 1 : i - 1;
        edge_origindex = subdiv_loop_edge_index[loop_index_last];
      }
      if (edge_origindex != -1) {
        /* Mapped points on an edge between two edit verts. */
        BMEdge *eed = BM_edge_at_index(mr.bm, edge_origindex);
        BMLoop *l = BM_face_edge_share_loop(const_cast<BMFace *>(coarse_quad), eed);
        mesh_render_data_loop_edge_flag(mr, l, offsets, *edit_loop_data);
      }
    }
  }
}

static void extract_edituv_subdiv_data_bm(const MeshRenderData &mr,
                                          const DRWSubdivCache &subdiv_cache,
                                          MutableSpan<EditLoopData> vbo_data)
{
  const int corners_num = subdiv_cache.num_subdiv_loops;
  const Span<int> subdiv_loop_face_index(subdiv_cache.subdiv_loop_face_index, corners_num);
  const Span<int> subdiv_loop_vert_index = subdiv_cache.verts_orig_index->data<int>();
  /* NOTE: #subdiv_loop_edge_index already has the origindex layer baked in. */
  const Span<int> subdiv_loop_edge_index = subdiv_cache.edges_orig_index->data<int>();

  const BMUVOffsets offsets = BM_uv_map_offsets_get(mr.bm);
  threading::parallel_for(IndexRange(subdiv_cache.num_subdiv_quads), 2048, [&](IndexRange range) {
    for (const int subdiv_quad : range) {
      const int coarse_face = subdiv_loop_face_index[subdiv_quad * 4];
      extract_edituv_data_iter_subdiv_bm(mr,
                                         offsets,
                                         subdiv_loop_vert_index,
                                         subdiv_loop_edge_index,
                                         subdiv_quad,
                                         BM_face_at_index(mr.bm, coarse_face),
                                         vbo_data);
    }
  });
}

static void extract_edituv_subdiv_data_mesh(const MeshRenderData &mr,
                                            const DRWSubdivCache &subdiv_cache,
                                            MutableSpan<EditLoopData> vbo_data)
{
  const int corners_num = subdiv_cache.num_subdiv_loops;
  const Span<int> subdiv_loop_face_index(subdiv_cache.subdiv_loop_face_index, corners_num);
  const Span<int> subdiv_loop_vert_index = subdiv_cache.verts_orig_index->data<int>();
  /* NOTE: #subdiv_loop_edge_index already has the origindex layer baked in. */
  const Span<int> subdiv_loop_edge_index = subdiv_cache.edges_orig_index->data<int>();

  const BMUVOffsets offsets = BM_uv_map_offsets_get(mr.bm);
  threading::parallel_for(IndexRange(subdiv_cache.num_subdiv_quads), 2048, [&](IndexRange range) {
    for (const int subdiv_quad : range) {
      const int coarse_face = subdiv_loop_face_index[subdiv_quad * 4];
      extract_edituv_data_iter_subdiv_bm(mr,
                                         offsets,
                                         subdiv_loop_vert_index,
                                         subdiv_loop_edge_index,
                                         subdiv_quad,
                                         bm_original_face_get(mr, coarse_face),
                                         vbo_data);
    }
  });
}

gpu::VertBufPtr extract_edituv_data_subdiv(const MeshRenderData &mr,
                                           const DRWSubdivCache &subdiv_cache)
{
  gpu::VertBufPtr vbo = gpu::VertBufPtr(GPU_vertbuf_create_with_format(edituv_data_format()));
  const int size = subdiv_cache.num_subdiv_loops;
  GPU_vertbuf_data_alloc(*vbo, size);
  MutableSpan vbo_data = vbo->data<EditLoopData>();

  if (mr.extract_type == MeshExtractType::BMesh) {
    extract_edituv_subdiv_data_bm(mr, subdiv_cache, vbo_data);
  }
  else {
    extract_edituv_subdiv_data_mesh(mr, subdiv_cache, vbo_data);
  }
  return vbo;
}

/** \} */

}  // namespace blender::draw
