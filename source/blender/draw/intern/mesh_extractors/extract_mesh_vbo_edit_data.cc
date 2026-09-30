/* SPDX-FileCopyrightText: 2021 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#include "DNA_meshdata_types.h"
#include "DNA_mesh_types.h"

#include "bmesh.hh"

#include "BLI_array.hh"
#include "BLI_kdtree.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_math_vector_types.hh"

#include "ED_mesh.hh"

#include "extract_mesh.hh"

#include "draw_cache_impl.hh"

#include "draw_subdivision.hh"

namespace blender::draw {

static void mesh_render_data_edge_flag(const MeshRenderData &mr,
                                       const BMEdge *eed,
                                       EditLoopData &eattr)
{
  const ToolSettings *ts = mr.toolsettings;
  const bool is_vertex_select_mode = (ts != nullptr) && (ts->selectmode & SCE_SELECT_VERTEX) != 0;
  const bool is_face_only_select_mode = (ts != nullptr) && (ts->selectmode == SCE_SELECT_FACE);

  if (eed == mr.eed_act) {
    eattr.e_flag |= VFLAG_EDGE_ACTIVE;
  }
  if (!is_vertex_select_mode && BM_elem_flag_test(eed, BM_ELEM_SELECT)) {
    eattr.e_flag |= VFLAG_EDGE_SELECTED;
  }
  if (is_vertex_select_mode && BM_elem_flag_test(eed->v1, BM_ELEM_SELECT) &&
      BM_elem_flag_test(eed->v2, BM_ELEM_SELECT))
  {
    eattr.e_flag |= VFLAG_EDGE_SELECTED;
    eattr.e_flag |= VFLAG_VERT_SELECTED;
  }
  if (BM_elem_flag_test(eed, BM_ELEM_SEAM)) {
    eattr.e_flag |= VFLAG_EDGE_SEAM;
  }
  if (!BM_elem_flag_test(eed, BM_ELEM_SMOOTH)) {
    eattr.e_flag |= VFLAG_EDGE_SHARP;
  }

  /* Use active edge color for active face edges because
   * specular highlights make it hard to see #55456#510873.
   *
   * This isn't ideal since it can't be used when mixing edge/face modes
   * but it's still better than not being able to see the active face. */
  if (is_face_only_select_mode) {
    if (mr.efa_act != nullptr) {
      if (BM_edge_in_face(eed, mr.efa_act)) {
        eattr.e_flag |= VFLAG_EDGE_ACTIVE;
      }
    }
  }

  /* Use half a byte for value range */
  if (mr.edge_crease_ofs != -1) {
    float crease = BM_ELEM_CD_GET_FLOAT(eed, mr.edge_crease_ofs);
    if (crease > 0) {
      eattr.crease = uchar(ceil(crease * 15.0f));
    }
  }
  /* Use a byte for value range */
  if (mr.bweight_ofs != -1) {
    float bweight = BM_ELEM_CD_GET_FLOAT(eed, mr.bweight_ofs);
    if (bweight > 0) {
      eattr.bweight = uchar(bweight * 255.0f);
    }
  }
#ifdef WITH_FREESTYLE
  if (mr.freestyle_edge_ofs != -1) {
    if (BM_ELEM_CD_GET_BOOL(eed, mr.freestyle_edge_ofs)) {
      eattr.e_flag |= VFLAG_EDGE_FREESTYLE;
    }
  }
#endif
}

static void mesh_render_data_vert_flag(const MeshRenderData &mr,
                                       const BMVert *eve,
                                       EditLoopData &eattr)
{
  if (eve == mr.eve_act) {
    eattr.e_flag |= VFLAG_VERT_ACTIVE;
  }
  if (BM_elem_flag_test(eve, BM_ELEM_SELECT)) {
    eattr.e_flag |= VFLAG_VERT_SELECTED;
  }
  /* Use half a byte for value range */
  if (mr.vert_crease_ofs != -1) {
    float crease = BM_ELEM_CD_GET_FLOAT(eve, mr.vert_crease_ofs);
    if (crease > 0) {
      eattr.crease |= uchar(ceil(crease * 15.0f)) << 4;
    }
  }
}

static const GPUVertFormat &get_edit_data_format()
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

static void extract_edit_data_mesh(const MeshRenderData &mr, MutableSpan<EditLoopData> vbo_data)
{
  MutableSpan corners_data = vbo_data.take_front(mr.corners_num);
  MutableSpan loose_edge_data = vbo_data.slice(mr.corners_num, mr.loose_edges.size() * 2);
  MutableSpan loose_vert_data = vbo_data.take_back(mr.loose_verts.size());

  const BMUVOffsets uv_offsets_none = BMUVOFFSETS_NONE;
  const OffsetIndices faces = mr.faces;
  const Span<int> corner_verts = mr.corner_verts;
  const Span<int> corner_edges = mr.corner_edges;
  threading::parallel_for(faces.index_range(), 2048, [&](const IndexRange range) {
    for (const int face : range) {
      EditLoopData face_value = {};
      if (const BMFace *bm_face = bm_original_face_get(mr, face)) {
        mesh_render_data_face_flag(mr, bm_face, uv_offsets_none, face_value);
      }
      for (const int corner : faces[face]) {
        EditLoopData &value = corners_data[corner];
        value = face_value;
        if (const BMVert *bm_vert = bm_original_vert_get(mr, corner_verts[corner])) {
          mesh_render_data_vert_flag(mr, bm_vert, value);
        }
        if (const BMEdge *bm_edge = bm_original_edge_get(mr, corner_edges[corner])) {
          mesh_render_data_edge_flag(mr, bm_edge, value);
        }
      }
    }
  });

  const Span<int2> edges = mr.edges;
  mr.loose_edges.foreach_index(
      [&](const int edge_i, const int pos) {
        EditLoopData &value_1 = loose_edge_data[pos * 2 + 0];
        EditLoopData &value_2 = loose_edge_data[pos * 2 + 1];
        if (const BMEdge *bm_edge = bm_original_edge_get(mr, edge_i)) {
          value_1 = {};
          mesh_render_data_edge_flag(mr, bm_edge, value_1);
          value_2 = value_1;
        }
        else {
          value_2 = value_1 = {};
        }
        const int2 edge = edges[edge_i];
        if (const BMVert *bm_vert = bm_original_vert_get(mr, edge[0])) {
          mesh_render_data_vert_flag(mr, bm_vert, value_1);
        }
        if (const BMVert *bm_vert = bm_original_vert_get(mr, edge[1])) {
          mesh_render_data_vert_flag(mr, bm_vert, value_2);
        }
      },
      exec_mode::grain_size(2048));

  mr.loose_verts.foreach_index(
      [&](const int vert, const int pos) {
        loose_vert_data[pos] = {};
        if (const BMVert *eve = bm_original_vert_get(mr, vert)) {
          mesh_render_data_vert_flag(mr, eve, loose_vert_data[pos]);
        }
      },
      exec_mode::grain_size(2048));
}

static void extract_edit_data_bm(const MeshRenderData &mr, MutableSpan<EditLoopData> vbo_data)
{
  MutableSpan corners_data = vbo_data.take_front(mr.corners_num);
  MutableSpan loose_edge_data = vbo_data.slice(mr.corners_num, mr.loose_edges.size() * 2);
  MutableSpan loose_vert_data = vbo_data.take_back(mr.loose_verts.size());

  const BMesh &bm = *mr.bm;
  const BMUVOffsets uv_offsets_none = BMUVOFFSETS_NONE;

  threading::parallel_for(IndexRange(bm.totface), 2048, [&](const IndexRange range) {
    for (const int face_index : range) {
      const BMFace &face = *BM_face_at_index(&const_cast<BMesh &>(bm), face_index);
      EditLoopData face_value = {};
      mesh_render_data_face_flag(mr, &face, uv_offsets_none, face_value);
      const BMLoop *loop = BM_FACE_FIRST_LOOP(&face);
      for ([[maybe_unused]] const int i : IndexRange(face.len)) {
        const int index = BM_elem_index_get(loop);
        EditLoopData &value = corners_data[index];
        value = face_value;
        mesh_render_data_edge_flag(mr, loop->e, value);
        mesh_render_data_vert_flag(mr, loop->v, value);
        loop = loop->next;
      }
    }
  });

  mr.loose_edges.foreach_index(
      [&](const int edge_i, const int pos) {
        EditLoopData &value_1 = loose_edge_data[pos * 2 + 0];
        EditLoopData &value_2 = loose_edge_data[pos * 2 + 1];
        const BMEdge &edge = *BM_edge_at_index(&const_cast<BMesh &>(bm), edge_i);
        value_1 = {};
        mesh_render_data_edge_flag(mr, &edge, value_1);
        value_2 = value_1;
        mesh_render_data_vert_flag(mr, edge.v1, value_1);
        mesh_render_data_vert_flag(mr, edge.v2, value_2);
      },
      exec_mode::grain_size(2048));

  mr.loose_verts.foreach_index(
      [&](const int vert_i, const int pos) {
        loose_vert_data[pos] = {};
        const BMVert &vert = *BM_vert_at_index(&const_cast<BMesh &>(bm), vert_i);
        mesh_render_data_vert_flag(mr, &vert, loose_vert_data[pos]);
      },
      exec_mode::grain_size(2048));
}

/** \name BIKINI: Mirror Selection Highlight
 *
 * Flag unselected elements whose mirror counterpart is selected, so the overlay shaders can
 * draw the mirrored side of the user's selection in a distinct color.
 * \{ */

/** Fast overlay maps: one KDTree, mark counterparts of the *selected* elements
 * across every combination of enabled axes. Avoids #EDBM_verts_mirror_cache_begin_ex
 * (multi-pass 16-nearest + edge walks × 3 axes) which rebuilt every redraw. */
struct MirrorHighlightMaps {
  float maxdist = 0.00002f; /* Same as #BM_SEARCH_MAXDIST_MIRR unless the user overrides it. */

  Array<bool> vert_hl;
  Array<bool> edge_hl;
  Array<bool> face_hl;
  /** Active-element center. Selected counterparts on the opposite side are highlighted
   * instead of drawn with the orange selection color after a mirrored topology op. */
  float3 active_co = float3(0.0f);
  bool have_active = false;
  short symmetry = 0;

  bool on_active_side(const float3 &co) const
  {
    if (!have_active) {
      return true;
    }
    for (int a = 0; a < 3; a++) {
      if ((symmetry & (1 << a)) == 0) {
        continue;
      }
      if (fabsf(active_co[a]) <= 1e-4f) {
        continue;
      }
      if (active_co[a] > 0.0f && co[a] < -1e-4f) {
        return false;
      }
      if (active_co[a] < 0.0f && co[a] > 1e-4f) {
        return false;
      }
    }
    return true;
  }

  bool init(const MeshRenderData &mr)
  {
    if (mr.edit_mirror_symmetry == 0 || mr.edit_bmesh == nullptr || mr.bm == nullptr ||
        mr.bm->totvertsel == 0)
    {
      return false;
    }
    BMesh &bm = *mr.bm;
    const int enabled = int(mr.edit_mirror_symmetry) & 7;
    if (enabled == 0) {
      return false;
    }
    symmetry = short(enabled);
    if (mr.toolsettings != nullptr && mr.toolsettings->mesh_mirror_threshold > 0.0f) {
      maxdist = mr.toolsettings->mesh_mirror_threshold;
    }

    BM_mesh_elem_index_ensure(&bm, BM_VERT | BM_EDGE | BM_FACE);
    vert_hl = Array<bool>(bm.totvert, false);
    edge_hl = Array<bool>(bm.totedge, false);
    face_hl = Array<bool>(bm.totface, false);

    KDTree<float3> *tree = kdtree_new<float3>(bm.totvert);
    {
      BMIter iter;
      BMVert *v;
      int i;
      BM_ITER_MESH_INDEX (v, &iter, &bm, BM_VERTS_OF_MESH, i) {
        if (!BM_elem_flag_test(v, BM_ELEM_HIDDEN)) {
          kdtree_insert<float3>(tree, i, float3(v->co));
        }
      }
      kdtree_balance(tree);
    }

    auto flip_co = [&](float3 co, const int mask) {
      for (int a = 0; a < 3; a++) {
        if (mask & (1 << a)) {
          co[a] = -co[a];
        }
      }
      return co;
    };
    /* Unique hit at `query`. Rejects nearest-neighbor guesses (Suzanne is only X-symmetric:
     * flipping Y/Z must not light a random nearby vert). */
    auto find_unique = [&](const float3 &src, const float3 &query) -> BMVert * {
      if (len_squared_v3v3(src, query) <= maxdist * maxdist) {
        return nullptr; /* On-plane / self, not a counterpart. */
      }
      KDTreeNearest<float3> nearest[2];
      const int found = kdtree_find_nearest_n<float3>(tree, query, nearest, 2);
      if (found < 1 || nearest[0].dist > maxdist) {
        return nullptr;
      }
      BMVert *w = BM_vert_at_index(&bm, nearest[0].index);
      if (w == nullptr || len_squared_v3v3(w->co, src) <= maxdist * maxdist) {
        return nullptr;
      }
      if (found > 1 && nearest[1].dist <= maxdist) {
        return nullptr; /* Ambiguous cluster. */
      }
      return w;
    };
    /* Faces/edges that touch the symmetry-plane seam share on-plane verts.
     * Those verts map to themselves so the counterpart face/edge can still
     * be found (the pair of faces on either side of the center line). */
    auto counterpart = [&](BMVert *v, const int mask) -> BMVert * {
      const float3 src = v->co;
      const float3 query = flip_co(src, mask);
      if (len_squared_v3v3(src, query) <= maxdist * maxdist) {
        return v;
      }
      return find_unique(src, query);
    };

    BMIter iter;
    BMVert *v;
    BM_ITER_MESH (v, &iter, &bm, BM_VERTS_OF_MESH) {
      if (!BM_elem_flag_test(v, BM_ELEM_SELECT) || BM_elem_flag_test(v, BM_ELEM_HIDDEN)) {
        continue;
      }
      const float3 src = v->co;
      for (int mask = 1; mask < 8; mask++) {
        if ((mask & enabled) != mask) {
          continue;
        }
        BMVert *w = find_unique(src, flip_co(src, mask));
        if (w == nullptr || w == v || BM_elem_flag_test(w, BM_ELEM_HIDDEN)) {
          continue;
        }
        /* Reciprocal: the counterpart's flip must land back on the source. */
        BMVert *back = find_unique(w->co, flip_co(w->co, mask));
        if (back != v) {
          continue;
        }
        vert_hl[BM_elem_index_get(w)] = true;
      }
    }
    BMEdge *e;
    BM_ITER_MESH (e, &iter, &bm, BM_EDGES_OF_MESH) {
      if (!BM_elem_flag_test(e, BM_ELEM_SELECT) || BM_elem_flag_test(e, BM_ELEM_HIDDEN)) {
        continue;
      }
      for (int mask = 1; mask < 8; mask++) {
        if ((mask & enabled) != mask) {
          continue;
        }
        BMVert *w1 = counterpart(e->v1, mask);
        BMVert *w2 = counterpart(e->v2, mask);
        if (w1 == nullptr || w2 == nullptr || w1 == w2) {
          continue;
        }
        BMEdge *e_mirr = BM_edge_exists(w1, w2);
        if (e_mirr && e_mirr != e && !BM_elem_flag_test(e_mirr, BM_ELEM_HIDDEN)) {
          edge_hl[BM_elem_index_get(e_mirr)] = true;
        }
      }
    }
    BMFace *f;
    BM_ITER_MESH (f, &iter, &bm, BM_FACES_OF_MESH) {
      if (!BM_elem_flag_test(f, BM_ELEM_SELECT) || BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
        continue;
      }
      Array<BMVert *, BM_DEFAULT_NGON_STACK_SIZE> v_mirr(f->len);
      for (int mask = 1; mask < 8; mask++) {
        if ((mask & enabled) != mask) {
          continue;
        }
        bool ok = true;
        bool any_moved = false;
        int vi = 0;
        const BMLoop *l_iter = f->l_first;
        do {
          BMVert *w = counterpart(l_iter->v, mask);
          if (w == nullptr) {
            ok = false;
            break;
          }
          any_moved |= (w != l_iter->v);
          v_mirr[vi++] = w;
        } while ((l_iter = l_iter->next) != f->l_first);
        if (!ok || !any_moved) {
          continue;
        }
        BMFace *f_mirr = BM_face_exists(v_mirr.data(), v_mirr.size());
        if (f_mirr && f_mirr != f && !BM_elem_flag_test(f_mirr, BM_ELEM_HIDDEN)) {
          face_hl[BM_elem_index_get(f_mirr)] = true;
        }
      }
    }

    kdtree_free<float3>(tree);

    if (mr.eve_act) {
      active_co = mr.eve_act->co;
      have_active = true;
    }
    else if (mr.eed_act) {
      active_co = (float3(mr.eed_act->v1->co) + float3(mr.eed_act->v2->co)) * 0.5f;
      have_active = true;
    }
    else if (mr.efa_act) {
      BM_face_calc_center_median(mr.efa_act, active_co);
      have_active = true;
    }

    return true;
  }

  bool vert_mirror_selected(const BMesh & /*bm*/, const BMVert *v) const
  {
    const int i = BM_elem_index_get(v);
    if (i < 0 || i >= vert_hl.size() || !vert_hl[i]) {
      return false;
    }
    /* Unselected counterparts always take the highlight color.
     * Selected ones do too, but only on the side opposite the active element —
     * otherwise a mirrored extrude/subdivide paints that side with the orange
     * selection color. The user's own side stays the normal selection color. */
    if (!BM_elem_flag_test(v, BM_ELEM_SELECT)) {
      return true;
    }
    return !on_active_side(v->co);
  }

  bool edge_mirror_selected(const BMesh & /*bm*/, const BMEdge *e) const
  {
    const int i = BM_elem_index_get(e);
    if (i < 0 || i >= edge_hl.size() || !edge_hl[i]) {
      return false;
    }
    if (!BM_elem_flag_test(e, BM_ELEM_SELECT)) {
      return true;
    }
    const float3 center = (float3(e->v1->co) + float3(e->v2->co)) * 0.5f;
    return !on_active_side(center);
  }

  bool face_mirror_selected(const BMesh & /*bm*/, const BMFace *f) const
  {
    const int i = BM_elem_index_get(f);
    if (i < 0 || i >= face_hl.size() || !face_hl[i]) {
      return false;
    }
    if (!BM_elem_flag_test(f, BM_ELEM_SELECT)) {
      return true;
    }
    float3 center(0.0f);
    BM_face_calc_center_median(f, center);
    return !on_active_side(center);
  }
};

static void edit_data_mirror_highlight_bm(const MeshRenderData &mr,
                                          const MirrorHighlightMaps &mirror,
                                          MutableSpan<EditLoopData> vbo_data)
{
  const BMesh &bm = *mr.bm;
  MutableSpan corners_data = vbo_data.take_front(mr.corners_num);
  MutableSpan loose_edge_data = vbo_data.slice(mr.corners_num, mr.loose_edges.size() * 2);
  MutableSpan loose_vert_data = vbo_data.take_back(mr.loose_verts.size());

  threading::parallel_for(IndexRange(bm.totface), 2048, [&](const IndexRange range) {
    for (const int face_index : range) {
      const BMFace &face = *BM_face_at_index(&const_cast<BMesh &>(bm), face_index);
      const bool face_mirr = mirror.face_mirror_selected(bm, &face);
      const BMLoop *loop = BM_FACE_FIRST_LOOP(&face);
      for ([[maybe_unused]] const int i : IndexRange(face.len)) {
        EditLoopData &value = corners_data[BM_elem_index_get(loop)];
        if (face_mirr) {
          value.v_flag |= VFLAG_FACE_MIRROR_SELECTED;
        }
        if (mirror.edge_mirror_selected(bm, loop->e)) {
          value.v_flag |= VFLAG_EDGE_MIRROR_SELECTED;
        }
        if (mirror.vert_mirror_selected(bm, loop->v)) {
          value.e_flag |= VFLAG_VERT_MIRROR_SELECTED;
        }
        loop = loop->next;
      }
    }
  });

  mr.loose_edges.foreach_index(
      [&](const int edge_i, const int pos) {
        const BMEdge &edge = *BM_edge_at_index(&const_cast<BMesh &>(bm), edge_i);
        if (mirror.edge_mirror_selected(bm, &edge)) {
          loose_edge_data[pos * 2 + 0].v_flag |= VFLAG_EDGE_MIRROR_SELECTED;
          loose_edge_data[pos * 2 + 1].v_flag |= VFLAG_EDGE_MIRROR_SELECTED;
        }
        if (mirror.vert_mirror_selected(bm, edge.v1)) {
          loose_edge_data[pos * 2 + 0].e_flag |= VFLAG_VERT_MIRROR_SELECTED;
        }
        if (mirror.vert_mirror_selected(bm, edge.v2)) {
          loose_edge_data[pos * 2 + 1].e_flag |= VFLAG_VERT_MIRROR_SELECTED;
        }
      },
      exec_mode::grain_size(2048));

  mr.loose_verts.foreach_index(
      [&](const int vert_i, const int pos) {
        if (mirror.vert_mirror_selected(bm, BM_vert_at_index(&const_cast<BMesh &>(bm), vert_i)))
        {
          loose_vert_data[pos].e_flag |= VFLAG_VERT_MIRROR_SELECTED;
        }
      },
      exec_mode::grain_size(2048));
}

static void edit_data_mirror_highlight_mesh(const MeshRenderData &mr,
                                            const MirrorHighlightMaps &mirror,
                                            MutableSpan<EditLoopData> vbo_data)
{
  const BMesh &bm = *mr.bm;
  MutableSpan corners_data = vbo_data.take_front(mr.corners_num);
  MutableSpan loose_edge_data = vbo_data.slice(mr.corners_num, mr.loose_edges.size() * 2);
  MutableSpan loose_vert_data = vbo_data.take_back(mr.loose_verts.size());

  const OffsetIndices faces = mr.faces;
  const Span<int> corner_verts = mr.corner_verts;
  const Span<int> corner_edges = mr.corner_edges;
  threading::parallel_for(faces.index_range(), 2048, [&](const IndexRange range) {
    for (const int face : range) {
      const BMFace *bm_face = bm_original_face_get(mr, face);
      const bool face_mirr = bm_face && mirror.face_mirror_selected(bm, bm_face);
      for (const int corner : faces[face]) {
        EditLoopData &value = corners_data[corner];
        if (face_mirr) {
          value.v_flag |= VFLAG_FACE_MIRROR_SELECTED;
        }
        if (const BMEdge *bm_edge = bm_original_edge_get(mr, corner_edges[corner])) {
          if (mirror.edge_mirror_selected(bm, bm_edge)) {
            value.v_flag |= VFLAG_EDGE_MIRROR_SELECTED;
          }
        }
        if (const BMVert *bm_vert = bm_original_vert_get(mr, corner_verts[corner])) {
          if (mirror.vert_mirror_selected(bm, bm_vert)) {
            value.e_flag |= VFLAG_VERT_MIRROR_SELECTED;
          }
        }
      }
    }
  });

  const Span<int2> edges = mr.edges;
  mr.loose_edges.foreach_index(
      [&](const int edge_i, const int pos) {
        if (const BMEdge *bm_edge = bm_original_edge_get(mr, edge_i)) {
          if (mirror.edge_mirror_selected(bm, bm_edge)) {
            loose_edge_data[pos * 2 + 0].v_flag |= VFLAG_EDGE_MIRROR_SELECTED;
            loose_edge_data[pos * 2 + 1].v_flag |= VFLAG_EDGE_MIRROR_SELECTED;
          }
        }
        const int2 edge = edges[edge_i];
        if (const BMVert *bm_vert = bm_original_vert_get(mr, edge[0])) {
          if (mirror.vert_mirror_selected(bm, bm_vert)) {
            loose_edge_data[pos * 2 + 0].e_flag |= VFLAG_VERT_MIRROR_SELECTED;
          }
        }
        if (const BMVert *bm_vert = bm_original_vert_get(mr, edge[1])) {
          if (mirror.vert_mirror_selected(bm, bm_vert)) {
            loose_edge_data[pos * 2 + 1].e_flag |= VFLAG_VERT_MIRROR_SELECTED;
          }
        }
      },
      exec_mode::grain_size(2048));

  mr.loose_verts.foreach_index(
      [&](const int vert, const int pos) {
        if (const BMVert *bm_vert = bm_original_vert_get(mr, vert)) {
          if (mirror.vert_mirror_selected(bm, bm_vert)) {
            loose_vert_data[pos].e_flag |= VFLAG_VERT_MIRROR_SELECTED;
          }
        }
      },
      exec_mode::grain_size(2048));
}

/** \} */

gpu::VertBufPtr extract_edit_data(const MeshRenderData &mr)
{
  gpu::VertBufPtr vbo = gpu::VertBufPtr(GPU_vertbuf_create_with_format(get_edit_data_format()));
  const int size = mr.corners_num + mr.loose_indices_num;
  GPU_vertbuf_data_alloc(*vbo, size);
  MutableSpan vbo_data = vbo->data<EditLoopData>();
  MutableSpan<EditLoopData> vbo_data_all = vbo_data;
  if (mr.extract_type == MeshExtractType::Mesh) {
    extract_edit_data_mesh(mr, vbo_data);
  }
  else {
    extract_edit_data_bm(mr, vbo_data);
  }
  /* BIKINI: flag the mirrored side of the selection for the overlay highlight. */
  MirrorHighlightMaps mirror;
  if (mirror.init(mr)) {
    if (mr.extract_type == MeshExtractType::Mesh) {
      edit_data_mirror_highlight_mesh(mr, mirror, vbo_data_all);
    }
    else {
      edit_data_mirror_highlight_bm(mr, mirror, vbo_data_all);
    }
  }
  return vbo;
}

gpu::VertBufPtr extract_face_dots_edit_data(const MeshRenderData &mr)
{
  /* One record per face. The face-dot batch indexes faces, not corners, so the
   * corner EditData buffer cannot be reused. v_flag carries FACE_MIRROR_SELECTED. */
  gpu::VertBufPtr vbo = gpu::VertBufPtr(GPU_vertbuf_create_with_format(get_edit_data_format()));
  const int faces_num = (mr.extract_type == MeshExtractType::BMesh && mr.bm != nullptr) ?
                            mr.bm->totface :
                            mr.faces_num;
  GPU_vertbuf_data_alloc(*vbo, faces_num);
  MutableSpan face_data = vbo->data<EditLoopData>();
  face_data.fill({});
  if (mr.bm == nullptr) {
    return vbo;
  }
  MirrorHighlightMaps mirror;
  if (!mirror.init(mr)) {
    return vbo;
  }
  const BMesh &bm = *mr.bm;
  if (mr.extract_type == MeshExtractType::BMesh) {
    threading::parallel_for(IndexRange(bm.totface), 2048, [&](const IndexRange range) {
      for (const int face_index : range) {
        const BMFace *face = BM_face_at_index(&const_cast<BMesh &>(bm), face_index);
        if (mirror.face_mirror_selected(bm, face)) {
          face_data[face_index].v_flag |= VFLAG_FACE_MIRROR_SELECTED;
        }
      }
    });
  }
  else {
    threading::parallel_for(IndexRange(faces_num), 2048, [&](const IndexRange range) {
      for (const int face_index : range) {
        const BMFace *face = bm_original_face_get(mr, face_index);
        if (face && mirror.face_mirror_selected(bm, face)) {
          face_data[face_index].v_flag |= VFLAG_FACE_MIRROR_SELECTED;
        }
      }
    });
  }
  return vbo;
}

static void extract_edit_subdiv_data_mesh(const MeshRenderData &mr,
                                          const DRWSubdivCache &subdiv_cache,
                                          MutableSpan<EditLoopData> vbo_data)
{
  const BMUVOffsets uv_offsets_none = BMUVOFFSETS_NONE;
  const int corners_num = subdiv_cache.num_subdiv_loops;
  const int loose_edges_num = mr.loose_edges.size();
  const int verts_per_edge = subdiv_verts_per_coarse_edge(subdiv_cache);
  const Span<int> subdiv_loop_face_index(subdiv_cache.subdiv_loop_face_index, corners_num);
  const Span<int> subdiv_loop_vert_index = subdiv_cache.verts_orig_index->data<int>();
  /* NOTE: #subdiv_loop_edge_index already has the origindex layer baked in. */
  const Span<int> subdiv_loop_edge_index = subdiv_cache.edges_orig_index->data<int>();

  MutableSpan corners_data = vbo_data.take_front(corners_num);
  MutableSpan loose_edge_data = vbo_data.slice(corners_num, loose_edges_num * verts_per_edge);
  MutableSpan loose_vert_data = vbo_data.take_back(mr.loose_verts.size());

  threading::parallel_for(IndexRange(subdiv_cache.num_subdiv_quads), 2048, [&](IndexRange range) {
    for (const int subdiv_quad : range) {
      const int coarse_face = subdiv_loop_face_index[subdiv_quad * 4];

      EditLoopData face_value = {};
      if (const BMFace *bm_face = bm_original_face_get(mr, coarse_face)) {
        mesh_render_data_face_flag(mr, bm_face, uv_offsets_none, face_value);
      }
      for (const int subdiv_corner : IndexRange(subdiv_quad * 4, 4)) {
        EditLoopData &value = corners_data[subdiv_corner];
        value = face_value;

        const int vert_origindex = subdiv_loop_vert_index[subdiv_corner];
        if (vert_origindex != -1) {
          if (const BMVert *bm_vert = bm_original_vert_get(mr, vert_origindex)) {
            mesh_render_data_vert_flag(mr, bm_vert, value);
          }
        }

        const int edge_origindex = subdiv_loop_edge_index[subdiv_corner];
        if (edge_origindex != -1) {
          if (const BMEdge *bm_edge = BM_edge_at_index(mr.bm, edge_origindex)) {
            mesh_render_data_edge_flag(mr, bm_edge, value);
          }
        }
      }
    }
  });

  const Span<int2> edges = mr.edges;
  mr.loose_edges.foreach_index(
      [&](const int edge_i, const int pos) {
        MutableSpan<EditLoopData> data = loose_edge_data.slice(pos * verts_per_edge,
                                                               verts_per_edge);
        if (const BMEdge *edge = bm_original_edge_get(mr, edge_i)) {
          EditLoopData value{};
          mesh_render_data_edge_flag(mr, edge, value);
          data.fill(value);
        }
        else {
          data.fill({});
        }
        const int2 edge = edges[edge_i];
        if (const BMVert *bm_vert = bm_original_vert_get(mr, edge[0])) {
          mesh_render_data_vert_flag(mr, bm_vert, data.first());
        }
        if (const BMVert *bm_vert = bm_original_vert_get(mr, edge[1])) {
          mesh_render_data_vert_flag(mr, bm_vert, data.last());
        }
      },
      exec_mode::grain_size(2048));

  mr.loose_verts.foreach_index(
      [&](const int vert, const int pos) {
        loose_vert_data[pos] = {};
        if (const BMVert *eve = bm_original_vert_get(mr, vert)) {
          mesh_render_data_vert_flag(mr, eve, loose_vert_data[pos]);
        }
      },
      exec_mode::grain_size(2048));
}

static void extract_edit_subdiv_data_bm(const MeshRenderData &mr,
                                        const DRWSubdivCache &subdiv_cache,
                                        MutableSpan<EditLoopData> vbo_data)
{
  const BMUVOffsets uv_offsets_none = BMUVOFFSETS_NONE;
  const int corners_num = subdiv_cache.num_subdiv_loops;
  const int loose_edges_num = mr.loose_edges.size();
  const int verts_per_edge = subdiv_verts_per_coarse_edge(subdiv_cache);
  const Span<int> subdiv_loop_face_index(subdiv_cache.subdiv_loop_face_index, corners_num);
  const Span<int> subdiv_loop_vert_index = subdiv_cache.verts_orig_index->data<int>();
  const Span<int> subdiv_loop_edge_index = subdiv_cache.edges_orig_index->data<int>();

  MutableSpan corners_data = vbo_data.take_front(corners_num);
  MutableSpan loose_edge_data = vbo_data.slice(corners_num, loose_edges_num * verts_per_edge);
  MutableSpan loose_vert_data = vbo_data.take_back(mr.loose_verts.size());

  BMesh &bm = *mr.bm;
  threading::parallel_for(IndexRange(subdiv_cache.num_subdiv_quads), 2048, [&](IndexRange range) {
    for (const int subdiv_quad : range) {
      const int coarse_face = subdiv_loop_face_index[subdiv_quad * 4];
      const BMFace *bm_face = BM_face_at_index(&bm, coarse_face);

      EditLoopData face_value = {};
      mesh_render_data_face_flag(mr, bm_face, uv_offsets_none, face_value);

      for (const int subdiv_corner : IndexRange(subdiv_quad * 4, 4)) {
        EditLoopData &value = corners_data[subdiv_corner];
        value = face_value;

        const int vert_origindex = subdiv_loop_vert_index[subdiv_corner];
        if (vert_origindex != -1) {
          const BMVert *bm_vert = BM_vert_at_index(mr.bm, vert_origindex);
          mesh_render_data_vert_flag(mr, bm_vert, value);
        }

        const int edge_origindex = subdiv_loop_edge_index[subdiv_corner];
        if (edge_origindex != -1) {
          const BMEdge *bm_edge = BM_edge_at_index(mr.bm, edge_origindex);
          mesh_render_data_edge_flag(mr, bm_edge, value);
        }
      }
    }
  });

  mr.loose_edges.foreach_index(
      [&](const int edge_i, const int pos) {
        MutableSpan<EditLoopData> data = loose_edge_data.slice(pos * verts_per_edge,
                                                               verts_per_edge);
        const BMEdge *edge = BM_edge_at_index(&bm, edge_i);
        EditLoopData value{};
        mesh_render_data_edge_flag(mr, edge, value);
        data.fill(value);
        mesh_render_data_vert_flag(mr, edge->v1, data.first());
        mesh_render_data_vert_flag(mr, edge->v2, data.last());
      },
      exec_mode::grain_size(2048));

  mr.loose_verts.foreach_index(
      [&](const int vert_i, const int pos) {
        loose_vert_data[pos] = {};
        const BMVert *vert = BM_vert_at_index(&bm, vert_i);
        mesh_render_data_vert_flag(mr, vert, loose_vert_data[pos]);
      },
      exec_mode::grain_size(2048));
}

static void edit_data_mirror_highlight_subdiv(const MeshRenderData &mr,
                                              const MirrorHighlightMaps &mirror,
                                              const DRWSubdivCache &subdiv_cache,
                                              MutableSpan<EditLoopData> vbo_data)
{
  const BMesh &bm = *mr.bm;
  const int corners_num = subdiv_cache.num_subdiv_loops;
  const int loose_edges_num = mr.loose_edges.size();
  const int verts_per_edge = subdiv_verts_per_coarse_edge(subdiv_cache);
  const Span<int> subdiv_loop_face_index(subdiv_cache.subdiv_loop_face_index, corners_num);
  const Span<int> subdiv_loop_vert_index = subdiv_cache.verts_orig_index->data<int>();
  /* NOTE: #subdiv_loop_edge_index already has the origindex layer baked in. */
  const Span<int> subdiv_loop_edge_index = subdiv_cache.edges_orig_index->data<int>();

  MutableSpan corners_data = vbo_data.take_front(corners_num);
  MutableSpan loose_edge_data = vbo_data.slice(corners_num, loose_edges_num * verts_per_edge);
  MutableSpan loose_vert_data = vbo_data.take_back(mr.loose_verts.size());

  threading::parallel_for(IndexRange(subdiv_cache.num_subdiv_quads), 2048, [&](IndexRange range) {
    for (const int subdiv_quad : range) {
      const int coarse_face = subdiv_loop_face_index[subdiv_quad * 4];
      const BMFace *bm_face = bm_original_face_get(mr, coarse_face);
      const bool face_mirr = bm_face && mirror.face_mirror_selected(bm, bm_face);
      for (const int subdiv_corner : IndexRange(subdiv_quad * 4, 4)) {
        EditLoopData &value = corners_data[subdiv_corner];
        if (face_mirr) {
          value.v_flag |= VFLAG_FACE_MIRROR_SELECTED;
        }
        const int vert_origindex = subdiv_loop_vert_index[subdiv_corner];
        if (vert_origindex != -1) {
          if (const BMVert *bm_vert = bm_original_vert_get(mr, vert_origindex)) {
            if (mirror.vert_mirror_selected(bm, bm_vert)) {
              value.e_flag |= VFLAG_VERT_MIRROR_SELECTED;
            }
          }
        }
        const int edge_origindex = subdiv_loop_edge_index[subdiv_corner];
        if (edge_origindex != -1) {
          if (const BMEdge *bm_edge = BM_edge_at_index(mr.bm, edge_origindex)) {
            if (mirror.edge_mirror_selected(bm, bm_edge)) {
              value.v_flag |= VFLAG_EDGE_MIRROR_SELECTED;
            }
          }
        }
      }
    }
  });

  const Span<int2> edges = mr.edges;
  mr.loose_edges.foreach_index(
      [&](const int edge_i, const int pos) {
        MutableSpan<EditLoopData> data = loose_edge_data.slice(pos * verts_per_edge,
                                                               verts_per_edge);
        if (const BMEdge *edge = bm_original_edge_get(mr, edge_i)) {
          if (mirror.edge_mirror_selected(bm, edge)) {
            for (EditLoopData &value : data) {
              value.v_flag |= VFLAG_EDGE_MIRROR_SELECTED;
            }
          }
        }
        const int2 edge = edges[edge_i];
        if (const BMVert *bm_vert = bm_original_vert_get(mr, edge[0])) {
          if (mirror.vert_mirror_selected(bm, bm_vert)) {
            data.first().e_flag |= VFLAG_VERT_MIRROR_SELECTED;
          }
        }
        if (const BMVert *bm_vert = bm_original_vert_get(mr, edge[1])) {
          if (mirror.vert_mirror_selected(bm, bm_vert)) {
            data.last().e_flag |= VFLAG_VERT_MIRROR_SELECTED;
          }
        }
      },
      exec_mode::grain_size(2048));

  mr.loose_verts.foreach_index(
      [&](const int vert, const int pos) {
        if (const BMVert *bm_vert = bm_original_vert_get(mr, vert)) {
          if (mirror.vert_mirror_selected(bm, bm_vert)) {
            loose_vert_data[pos].e_flag |= VFLAG_VERT_MIRROR_SELECTED;
          }
        }
      },
      exec_mode::grain_size(2048));
}

gpu::VertBufPtr extract_edit_data_subdiv(const MeshRenderData &mr,
                                         const DRWSubdivCache &subdiv_cache)
{
  gpu::VertBufPtr vbo = gpu::VertBufPtr(GPU_vertbuf_create_with_format(get_edit_data_format()));
  const int size = subdiv_full_vbo_size(mr, subdiv_cache);
  GPU_vertbuf_data_alloc(*vbo, size);
  MutableSpan vbo_data = vbo->data<EditLoopData>();
  MutableSpan<EditLoopData> vbo_data_all = vbo_data;
  if (mr.extract_type == MeshExtractType::Mesh) {
    extract_edit_subdiv_data_mesh(mr, subdiv_cache, vbo_data);
  }
  else {
    extract_edit_subdiv_data_bm(mr, subdiv_cache, vbo_data);
  }
  /* BIKINI: flag the mirrored side of the selection for the overlay highlight. */
  MirrorHighlightMaps mirror;
  if (mirror.init(mr)) {
    edit_data_mirror_highlight_subdiv(mr, mirror, subdiv_cache, vbo_data_all);
  }
  return vbo;
}

}  // namespace blender::draw
