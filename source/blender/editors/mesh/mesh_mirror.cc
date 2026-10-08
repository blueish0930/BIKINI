/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edmesh
 *
 * Mirror calculation for edit-mode and object mode.
 */

#include "MEM_guardedalloc.h"

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BKE_editmesh.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_types.hh"

#include "BLI_array.hh"
#include "BLI_kdtree.hh"
#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_bits.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_set.hh"
#include "BLI_vector.hh"

#include "DEG_depsgraph.hh"

#include "ED_mesh.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "mesh_intern.hh"

#include <array>

namespace blender {

/* -------------------------------------------------------------------- */
/** \name Mesh Spatial Mirror API
 * \{ */

#define KD_THRESH 0.00002f

static struct {
  KDTree<float3> *tree;
} MirrKdStore = {nullptr};

void ED_mesh_mirror_spatial_table_begin(Object *ob, Mesh *mesh_eval)
{
  Mesh *mesh = id_cast<Mesh *>(ob->data);
  BMEditMesh *em = BKE_editmesh_from_object(ob);
  const bool use_em = (!mesh_eval && em);
  BMesh *bm = use_em ? BKE_editmesh_bmesh_get_for_write(ob) : nullptr;
  const int totvert = use_em ? bm->totvert : mesh_eval ? mesh_eval->verts_num : mesh->verts_num;

  if (MirrKdStore.tree) { /* happens when entering this call without ending it */
    ED_mesh_mirror_spatial_table_end(ob);
  }

  MirrKdStore.tree = kdtree_new<float3>(totvert);

  if (use_em) {
    BMVert *eve;
    BMIter iter;
    int i;

    /* this needs to be valid for index lookups later (callers need) */
    BM_mesh_elem_table_ensure(bm, BM_VERT);

    BM_ITER_MESH_INDEX (eve, &iter, bm, BM_VERTS_OF_MESH, i) {
      kdtree_insert<float3>(MirrKdStore.tree, i, eve->co);
    }
  }
  else {
    const Span<float3> positions = mesh_eval ? mesh_eval->vert_positions() :
                                               mesh->vert_positions();
    for (int i = 0; i < totvert; i++) {
      kdtree_insert<float3>(MirrKdStore.tree, i, positions[i]);
    }
  }

  kdtree_balance<float3>(MirrKdStore.tree);
}

int ED_mesh_mirror_spatial_table_lookup(Object *ob, Mesh *mesh_eval, const float co[3])
{
  if (MirrKdStore.tree == nullptr) {
    ED_mesh_mirror_spatial_table_begin(ob, mesh_eval);
  }

  if (MirrKdStore.tree) {
    KDTreeNearest<float3> nearest;
    const int i = kdtree_find_nearest<float3>(MirrKdStore.tree, co, &nearest);

    if (i != -1) {
      if (nearest.dist < KD_THRESH) {
        return i;
      }
    }
  }
  return -1;
}

void ED_mesh_mirror_spatial_table_end(Object * /*ob*/)
{
  /* TODO: store this in object/object-data (keep unused argument for now). */
  if (MirrKdStore.tree) {
    kdtree_free<float3>(MirrKdStore.tree);
    MirrKdStore.tree = nullptr;
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Mesh Topology Mirror API
 * \{ */

using MirrTopoHash_t = uint;

struct MirrTopoVert_t {
  MirrTopoHash_t hash;
  int v_index;
};

static int mirrtopo_hash_sort(const void *l1, const void *l2)
{
  if (MirrTopoHash_t(intptr_t(l1)) > MirrTopoHash_t(intptr_t(l2))) {
    return 1;
  }
  if (MirrTopoHash_t(intptr_t(l1)) < MirrTopoHash_t(intptr_t(l2))) {
    return -1;
  }
  return 0;
}

static int mirrtopo_vert_sort(const void *v1, const void *v2)
{
  if ((static_cast<MirrTopoVert_t *>(const_cast<void *>(v1)))->hash >
      (static_cast<MirrTopoVert_t *>(const_cast<void *>(v2)))->hash)
  {
    return 1;
  }
  if ((static_cast<MirrTopoVert_t *>(const_cast<void *>(v1)))->hash <
      (static_cast<MirrTopoVert_t *>(const_cast<void *>(v2)))->hash)
  {
    return -1;
  }
  return 0;
}

bool ED_mesh_mirrtopo_recalc_check(BMesh *bm, Mesh *mesh, MirrTopoStore_t *mesh_topo_store)
{
  const bool is_editmode = bm != nullptr;
  int totvert;
  int totedge;

  if (bm) {
    totvert = bm->totvert;
    totedge = bm->totedge;
  }
  else {
    totvert = mesh->verts_num;
    totedge = mesh->edges_num;
  }

  if ((mesh_topo_store->index_lookup == nullptr) ||
      (mesh_topo_store->prev_is_editmode != is_editmode) ||
      (totvert != mesh_topo_store->prev_vert_tot) || (totedge != mesh_topo_store->prev_edge_tot))
  {
    return true;
  }
  return false;
}

void ED_mesh_mirrtopo_init(BMesh *bm,
                           Mesh *mesh,
                           MirrTopoStore_t *mesh_topo_store,
                           const bool skip_bm_vert_array_init)
{
  if (bm) {
    BLI_assert(mesh == nullptr);
  }
  const bool is_editmode = (bm != nullptr);

  /* Edit-mode variables. */
  BMEdge *eed;
  BMIter iter;

  int a, last;
  int totvert, totedge;
  int tot_unique = -1, tot_unique_prev = -1;
  int tot_unique_edges = 0, tot_unique_edges_prev;

  MirrTopoHash_t topo_pass = 1;

  /* reallocate if needed */
  ED_mesh_mirrtopo_free(mesh_topo_store);

  mesh_topo_store->prev_is_editmode = is_editmode;

  if (bm) {
    BM_mesh_elem_index_ensure(bm, BM_VERT);

    totvert = bm->totvert;
  }
  else {
    totvert = mesh->verts_num;
  }

  MirrTopoHash_t *topo_hash = MEM_new_array_zeroed<MirrTopoHash_t>(totvert, __func__);

  /* Initialize the vert-edge-user counts used to detect unique topology */
  if (bm) {
    totedge = bm->totedge;

    BM_ITER_MESH (eed, &iter, bm, BM_EDGES_OF_MESH) {
      const int i1 = BM_elem_index_get(eed->v1), i2 = BM_elem_index_get(eed->v2);
      topo_hash[i1]++;
      topo_hash[i2]++;
    }
  }
  else {
    totedge = mesh->edges_num;
    for (const int2 &edge : mesh->edges()) {
      topo_hash[edge[0]]++;
      topo_hash[edge[1]]++;
    }
  }

  MirrTopoHash_t *topo_hash_prev = MEM_dupalloc(topo_hash);

  tot_unique_prev = -1;
  tot_unique_edges_prev = -1;
  while (true) {
    /* use the number of edges per vert to give verts unique topology IDs */

    tot_unique_edges = 0;

    /* This can make really big numbers, wrapping around here is fine */
    if (bm) {
      BM_ITER_MESH (eed, &iter, bm, BM_EDGES_OF_MESH) {
        const int i1 = BM_elem_index_get(eed->v1), i2 = BM_elem_index_get(eed->v2);
        topo_hash[i1] += topo_hash_prev[i2] * topo_pass;
        topo_hash[i2] += topo_hash_prev[i1] * topo_pass;
        tot_unique_edges += (topo_hash[i1] != topo_hash[i2]);
      }
    }
    else {
      for (const int2 &edge : mesh->edges()) {
        const int i1 = edge[0], i2 = edge[1];
        topo_hash[i1] += topo_hash_prev[i2] * topo_pass;
        topo_hash[i2] += topo_hash_prev[i1] * topo_pass;
        tot_unique_edges += (topo_hash[i1] != topo_hash[i2]);
      }
    }
    memcpy(topo_hash_prev, topo_hash, sizeof(MirrTopoHash_t) * totvert);

    /* sort so we can count unique values */
    qsort(topo_hash_prev, totvert, sizeof(MirrTopoHash_t), mirrtopo_hash_sort);

    tot_unique = 1; /* account for skipping the first value */
    for (a = 1; a < totvert; a++) {
      if (topo_hash_prev[a - 1] != topo_hash_prev[a]) {
        tot_unique++;
      }
    }

    if ((tot_unique <= tot_unique_prev) && (tot_unique_edges <= tot_unique_edges_prev)) {
      /* Finish searching for unique values when 1 loop doesn't give a
       * higher number of unique values compared to the previous loop. */
      break;
    }
    tot_unique_prev = tot_unique;
    tot_unique_edges_prev = tot_unique_edges;
    /* Copy the hash calculated this iteration, so we can use them next time */
    memcpy(topo_hash_prev, topo_hash, sizeof(MirrTopoHash_t) * totvert);

    topo_pass++;
  }

  /* Hash/Index pairs are needed for sorting to find index pairs */
  MirrTopoVert_t *topo_pairs = MEM_new_array_zeroed<MirrTopoVert_t>(totvert, "MirrTopoPairs");

  /* since we are looping through verts, initialize these values here too */
  intptr_t *index_lookup = MEM_new_array_uninitialized<intptr_t>(totvert, "mesh_topo_lookup");

  if (bm) {
    if (skip_bm_vert_array_init == false) {
      BM_mesh_elem_table_ensure(bm, BM_VERT);
    }
  }

  for (a = 0; a < totvert; a++) {
    topo_pairs[a].hash = topo_hash[a];
    topo_pairs[a].v_index = a;

    /* initialize lookup */
    index_lookup[a] = -1;
  }

  qsort(topo_pairs, totvert, sizeof(MirrTopoVert_t), mirrtopo_vert_sort);

  last = 0;

  /* Get the pairs out of the sorted hashes.
   * NOTE: `totvert + 1` means we can use the previous 2,
   * but you can't ever access the last 'a' index of #MirrTopoPairs. */
  if (bm) {
    BMVert **vtable = bm->vtable;
    for (a = 1; a <= totvert; a++) {
      // printf("I %d %ld %d\n",
      //        (a - last), MirrTopoPairs[a].hash, MirrTopoPairs[a].v_index);
      if ((a == totvert) || (topo_pairs[a - 1].hash != topo_pairs[a].hash)) {
        const int match_count = a - last;
        if (match_count == 2) {
          const int j = topo_pairs[a - 1].v_index, k = topo_pairs[a - 2].v_index;
          index_lookup[j] = intptr_t(vtable[k]);
          index_lookup[k] = intptr_t(vtable[j]);
        }
        else if (match_count == 1) {
          /* Center vertex. */
          const int j = topo_pairs[a - 1].v_index;
          index_lookup[j] = intptr_t(vtable[j]);
        }
        last = a;
      }
    }
  }
  else {
    /* same as above, for mesh */
    for (a = 1; a <= totvert; a++) {
      if ((a == totvert) || (topo_pairs[a - 1].hash != topo_pairs[a].hash)) {
        const int match_count = a - last;
        if (match_count == 2) {
          const int j = topo_pairs[a - 1].v_index, k = topo_pairs[a - 2].v_index;
          index_lookup[j] = k;
          index_lookup[k] = j;
        }
        else if (match_count == 1) {
          /* Center vertex. */
          const int j = topo_pairs[a - 1].v_index;
          index_lookup[j] = j;
        }
        last = a;
      }
    }
  }

  MEM_delete(topo_pairs);
  topo_pairs = nullptr;

  MEM_delete(topo_hash);
  MEM_delete(topo_hash_prev);

  mesh_topo_store->index_lookup = index_lookup;
  mesh_topo_store->prev_vert_tot = totvert;
  mesh_topo_store->prev_edge_tot = totedge;
}

void ED_mesh_mirrtopo_free(MirrTopoStore_t *mesh_topo_store)
{
  MEM_SAFE_DELETE(mesh_topo_store->index_lookup);
  mesh_topo_store->prev_vert_tot = -1;
  mesh_topo_store->prev_edge_tot = -1;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name EditMeshMirrorLookup API
 *
 * Pair only the verts we actually query. The KD-tree (or topology hash) still
 * contains every visible vertex so unselected counterparts resolve; the expensive
 * 16-nearest multi-pass in #EDBM_verts_mirror_cache_begin is skipped.
 * \{ */

struct EditMeshMirrorLookup::Impl {
  BMesh *bm = nullptr;
  bool use_topology = false;
  bool respecthide = true;
  float maxdist = 0.00002f;
  KDTree<float3> *tree = nullptr;
  MirrTopoStore_t topo{nullptr, -1, -1, false};
  /* Per axis-mask 1..7: -2 unknown, -1 none, >=0 partner index (self if on plane).
   * Allocated on first use, most lookups only ever touch one mask. */
  std::array<Array<int>, 7> cache;

  /* The tree only holds verts around the flipped copies of the selection. A vert may be
   * looked up from as long as it is at most #hop_max counterparts away from a selected
   * vert: each hop can drift by `maxdist`, and the tree was gathered with that margin. */
  static constexpr int8_t hop_max = 8;
  bool sparse = false;
  Array<int8_t> hops;

  void tree_build_full();
  void tree_build(const MirrorSelectionFilter &filter, Span<BMVert *> sources);
  BMVert *compute(BMVert *v, int axis_mask);
};

/** Bit N for each axis mask N (0..7) that only flips axes of \a symmetry. */
static uint mirror_symmetry_axis_masks(const int symmetry)
{
  uint axis_masks = 0;
  if (symmetry & 7) {
    for (int mask = 0; mask < 8; mask++) {
      if ((mask & symmetry) == mask) {
        axis_masks |= 1u << mask;
      }
    }
  }
  return axis_masks;
}

/* -------------------------------------------------------------------- */

MirrorSelectionFilter::MirrorSelectionFilter(BMesh *bm,
                                             const uint axis_masks,
                                             const float radius,
                                             const bool respecthide)
{
  const int masks_num = count_bits_i(axis_masks & 0xff);
  if (masks_num == 0) {
    return;
  }
  /* Gathering and a tree over the candidates have to beat a tree over everything. */
  if (int64_t(bm->totvertsel) * masks_num * 4 > int64_t(bm->totvert)) {
    return;
  }

  Vector<float3> selected;
  selected.reserve(bm->totvertsel);
  BMIter iter;
  BMVert *v;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    if (!BM_elem_flag_test(v, BM_ELEM_SELECT)) {
      continue;
    }
    if (respecthide && BM_elem_flag_test(v, BM_ELEM_HIDDEN)) {
      continue;
    }
    selected.append(float3(v->co));
  }
  this->init(selected, bm->totvert, axis_masks, radius);
}

MirrorSelectionFilter::MirrorSelectionFilter(const Span<float3> locations,
                                             const int verts_num,
                                             const uint axis_masks,
                                             const float radius)
{
  this->init(locations, verts_num, axis_masks, radius);
}

void MirrorSelectionFilter::init(const Span<float3> selected,
                                 const int verts_num,
                                 const uint axis_masks,
                                 const float radius)
{
  const int masks_num = count_bits_i(axis_masks & 0xff);
  if (masks_num == 0) {
    return;
  }
  if (int64_t(selected.size()) * masks_num * 4 > int64_t(verts_num)) {
    return;
  }
  active_ = true;
  if (selected.is_empty()) {
    /* Nothing to look up, nothing passes. */
    return;
  }

  float3 sel_min(FLT_MAX);
  float3 sel_max(-FLT_MAX);
  for (const float3 &co : selected) {
    sel_min = math::min(sel_min, co);
    sel_max = math::max(sel_max, co);
  }

  /* One box per flipped copy of the selection, a cheap first rejection. */
  float3 all_min(FLT_MAX);
  float3 all_max(-FLT_MAX);
  Vector<int, 8> masks;
  for (int mask = 0; mask < 8; mask++) {
    if ((axis_masks & (1u << mask)) == 0) {
      continue;
    }
    masks.append(mask);
    Box box;
    for (int a = 0; a < 3; a++) {
      if (mask & (1 << a)) {
        box.min[a] = -sel_max[a] - radius;
        box.max[a] = -sel_min[a] + radius;
      }
      else {
        box.min[a] = sel_min[a] - radius;
        box.max[a] = sel_max[a] + radius;
      }
    }
    boxes_.append(box);
    all_min = math::min(all_min, box.min);
    all_max = math::max(all_max, box.max);
  }

  /* Cells two radii wide, so a location touches at most two per axis. Wider when the
   * keys would not fit their 21 bits per axis. */
  origin_ = all_min;
  const float extent = math::reduce_max(all_max - all_min);
  const float cell_size = max_ff(2.0f * radius, extent / float((1 << 21) - 2));
  cell_inv_ = 1.0f / cell_size;

  cells_.reserve(int64_t(selected.size()) * masks.size());
  for (const float3 &co : selected) {
    for (const int mask : masks) {
      int lo[3], hi[3];
      for (int a = 0; a < 3; a++) {
        const float c = (mask & (1 << a)) ? -co[a] : co[a];
        lo[a] = max_ii(0, int(((c - radius) - origin_[a]) * cell_inv_));
        hi[a] = max_ii(0, int(((c + radius) - origin_[a]) * cell_inv_));
      }
      int cell[3];
      for (cell[0] = lo[0]; cell[0] <= hi[0]; cell[0]++) {
        for (cell[1] = lo[1]; cell[1] <= hi[1]; cell[1]++) {
          for (cell[2] = lo[2]; cell[2] <= hi[2]; cell[2]++) {
            cells_.add(this->cell_key(cell));
          }
        }
      }
    }
  }
}

bool MirrorSelectionFilter::test(const float co[3]) const
{
  for (const Box &box : boxes_) {
    if (co[0] < box.min[0] || co[0] > box.max[0] || co[1] < box.min[1] || co[1] > box.max[1] ||
        co[2] < box.min[2] || co[2] > box.max[2])
    {
      continue;
    }
    const int cell[3] = {
        int((co[0] - origin_[0]) * cell_inv_),
        int((co[1] - origin_[1]) * cell_inv_),
        int((co[2] - origin_[2]) * cell_inv_),
    };
    return cells_.contains(this->cell_key(cell));
  }
  return false;
}

/**
 * \param sources: The verts lookups start from, the selected verts when empty.
 */
void EditMeshMirrorLookup::Impl::tree_build(const MirrorSelectionFilter &filter,
                                            const Span<BMVert *> sources)
{
  if (!filter.is_active()) {
    this->tree_build_full();
    return;
  }
  this->sparse = true;
  this->hops = Array<int8_t>(this->bm->totvert, int8_t(-1));
  for (const BMVert *v_source : sources) {
    this->hops[BM_elem_index_get(v_source)] = 0;
  }
  Vector<int> candidates;
  BMIter iter;
  BMVert *v;
  int i;
  BM_ITER_MESH_INDEX (v, &iter, this->bm, BM_VERTS_OF_MESH, i) {
    if (this->respecthide && BM_elem_flag_test(v, BM_ELEM_HIDDEN)) {
      continue;
    }
    if (sources.is_empty() && BM_elem_flag_test(v, BM_ELEM_SELECT)) {
      this->hops[i] = 0;
    }
    if (filter.test(v->co)) {
      candidates.append(i);
    }
  }
  this->tree = kdtree_new<float3>(max_ii(1, int(candidates.size())));
  for (const int index : candidates) {
    kdtree_insert<float3>(this->tree, index, BM_vert_at_index(this->bm, index)->co);
  }
  kdtree_balance<float3>(this->tree);
}

void EditMeshMirrorLookup::Impl::tree_build_full()
{
  if (this->tree) {
    kdtree_free<float3>(this->tree);
  }
  this->sparse = false;
  this->hops = {};
  this->tree = kdtree_new<float3>(this->bm->totvert);
  BMIter iter;
  BMVert *v;
  int i;
  BM_ITER_MESH_INDEX (v, &iter, this->bm, BM_VERTS_OF_MESH, i) {
    if (this->respecthide && BM_elem_flag_test(v, BM_ELEM_HIDDEN)) {
      continue;
    }
    kdtree_insert<float3>(this->tree, i, v->co);
  }
  kdtree_balance<float3>(this->tree);
}

EditMeshMirrorLookup::EditMeshMirrorLookup(BMEditMesh *em,
                                           const bool use_topology,
                                           const bool respecthide,
                                           const float maxdist,
                                           const int selection_symmetry)
    : impl_(MEM_new<Impl>(__func__))
{
  impl_->bm = em->bm;
  impl_->use_topology = use_topology;
  impl_->respecthide = respecthide;
  impl_->maxdist = (maxdist > 0.0f) ? maxdist : 0.00002f;

  BMesh *bm = em->bm;
  BM_mesh_elem_table_ensure(bm, BM_VERT);
  BM_mesh_elem_index_ensure(bm, BM_VERT);

  /* Topology hash is X-only. Spatial tree is required for Y/Z and for XY/XZ/YZ
   * combos (the diagonal octants). Always build it. */
  const MirrorSelectionFilter filter(bm,
                                     mirror_symmetry_axis_masks(selection_symmetry),
                                     float(Impl::hop_max + 2) * impl_->maxdist + 1e-4f,
                                     respecthide);
  impl_->tree_build(filter, {});
  if (use_topology) {
    ED_mesh_mirrtopo_init(bm, nullptr, &impl_->topo, true);
  }
}

EditMeshMirrorLookup::EditMeshMirrorLookup(BMEditMesh *em,
                                           const bool use_topology,
                                           const bool respecthide,
                                           const float maxdist,
                                           const int symmetry,
                                           const Span<BMVert *> sources)
    : impl_(MEM_new<Impl>(__func__))
{
  impl_->bm = em->bm;
  impl_->use_topology = use_topology;
  impl_->respecthide = respecthide;
  impl_->maxdist = (maxdist > 0.0f) ? maxdist : 0.00002f;

  BMesh *bm = em->bm;
  BM_mesh_elem_table_ensure(bm, BM_VERT);
  BM_mesh_elem_index_ensure(bm, BM_VERT);

  Vector<float3, 8> locations;
  for (const BMVert *v : sources) {
    locations.append(float3(v->co));
  }
  /* No sources means no lookups to prepare for, keep the full tree then. */
  const MirrorSelectionFilter filter(locations,
                                     bm->totvert,
                                     sources.is_empty() ? 0 : mirror_symmetry_axis_masks(symmetry),
                                     float(Impl::hop_max + 2) * impl_->maxdist + 1e-4f);
  impl_->tree_build(filter, sources);
  if (use_topology) {
    ED_mesh_mirrtopo_init(bm, nullptr, &impl_->topo, true);
  }
}

EditMeshMirrorLookup::~EditMeshMirrorLookup()
{
  if (impl_ == nullptr) {
    return;
  }
  if (impl_->tree) {
    kdtree_free<float3>(impl_->tree);
  }
  ED_mesh_mirrtopo_free(&impl_->topo);
  MEM_delete(impl_);
}

BMVert *EditMeshMirrorLookup::Impl::compute(BMVert *v, const int axis_mask)
{
  if (this->respecthide && BM_elem_flag_test(v, BM_ELEM_HIDDEN)) {
    return nullptr;
  }

  /* Official topology hash pairs across X only. */
  if (this->use_topology && axis_mask == 1) {
    const int i = BM_elem_index_get(v);
    const intptr_t eve_i = this->topo.index_lookup[i];
    BMVert *v_mirr = (eve_i == -1) ? nullptr : reinterpret_cast<BMVert *>(eve_i);
    if (v_mirr && this->respecthide && BM_elem_flag_test(v_mirr, BM_ELEM_HIDDEN)) {
      return nullptr;
    }
    return v_mirr;
  }

  if (this->tree == nullptr) {
    return nullptr;
  }

  int8_t hop = 0;
  if (this->sparse) {
    hop = this->hops[BM_elem_index_get(v)];
    if (hop < 0 || hop >= hop_max) {
      /* Not something the sparse tree was gathered for. */
      this->tree_build_full();
    }
  }

  const float maxdist_sq = square_f(this->maxdist);
  float3 query = v->co;
  bool flipped = false;
  for (int a = 0; a < 3; a++) {
    if ((axis_mask & (1 << a)) == 0) {
      continue;
    }
    if (square_f(2.0f * v->co[a]) <= maxdist_sq) {
      continue;
    }
    query[a] = -query[a];
    flipped = true;
  }
  if (!flipped) {
    return v;
  }

  KDTreeNearest<float3> nearest[16];
  const int found = kdtree_find_nearest_n<float3>(this->tree, query, nearest, 16);
  /* Skip the source vert (it is often nearest when close to the plane). */
  BMVert *best = nullptr;
  float cluster_dist = -1.0f;
  int ties = 0;
  for (int n = 0; n < found; n++) {
    if (nearest[n].dist > this->maxdist) {
      break;
    }
    BMVert *w = BM_vert_at_index(this->bm, nearest[n].index);
    if (w == nullptr || w == v) {
      continue;
    }
    if (this->respecthide && BM_elem_flag_test(w, BM_ELEM_HIDDEN)) {
      continue;
    }
    if (cluster_dist < 0.0f) {
      cluster_dist = nearest[n].dist;
      best = w;
      ties = 1;
    }
    else if (nearest[n].dist > cluster_dist + 1e-5f) {
      break;
    }
    else {
      ties++;
      best = nullptr;
    }
  }
  if (best == nullptr || ties != 1) {
    return nullptr;
  }

  /* Reciprocal: flipping best by the same axes must land back on v. */
  float3 back_query = best->co;
  for (int a = 0; a < 3; a++) {
    if ((axis_mask & (1 << a)) == 0) {
      continue;
    }
    if (square_f(2.0f * best->co[a]) <= maxdist_sq) {
      continue;
    }
    back_query[a] = -back_query[a];
  }
  KDTreeNearest<float3> back_n[16];
  const int back_found = kdtree_find_nearest_n<float3>(this->tree, back_query, back_n, 16);
  BMVert *back = nullptr;
  float back_dist = -1.0f;
  int back_ties = 0;
  for (int n = 0; n < back_found; n++) {
    if (back_n[n].dist > this->maxdist) {
      break;
    }
    BMVert *w = BM_vert_at_index(this->bm, back_n[n].index);
    if (w == nullptr || w == best) {
      continue;
    }
    if (this->respecthide && BM_elem_flag_test(w, BM_ELEM_HIDDEN)) {
      continue;
    }
    if (back_dist < 0.0f) {
      back_dist = back_n[n].dist;
      back = w;
      back_ties = 1;
    }
    else if (back_n[n].dist > back_dist + 1e-5f) {
      break;
    }
    else {
      back_ties++;
      back = nullptr;
    }
  }
  if (back != v || back_ties != 1) {
    return nullptr;
  }
  if (this->sparse) {
    int8_t &hop_best = this->hops[BM_elem_index_get(best)];
    if (hop_best < 0 || hop_best > hop + 1) {
      hop_best = hop + 1;
    }
  }
  return best;
}

BMVert *EditMeshMirrorLookup::vert_axes(BMVert *v, int axis_mask)
{
  axis_mask &= 7;
  if (v == nullptr || axis_mask == 0) {
    return v;
  }
  const int i = BM_elem_index_get(v);
  BLI_assert(i >= 0 && i < impl_->bm->totvert);
  Array<int> &cache = impl_->cache[axis_mask - 1];
  if (cache.is_empty()) {
    cache = Array<int>(impl_->bm->totvert, -2);
  }
  int &slot = cache[i];
  if (slot >= -1) {
    return (slot < 0) ? nullptr : BM_vert_at_index(impl_->bm, slot);
  }
  BMVert *found = impl_->compute(v, axis_mask);
  slot = found ? BM_elem_index_get(found) : -1;
  if (found && found != v) {
    cache[BM_elem_index_get(found)] = i;
  }
  return found;
}

BMVert *EditMeshMirrorLookup::vert(BMVert *v, const int axis)
{
  BLI_assert(axis >= 0 && axis < 3);
  return this->vert_axes(v, 1 << axis);
}

BMEdge *EditMeshMirrorLookup::edge_axes(BMEdge *e, const int axis_mask)
{
  if (e == nullptr) {
    return nullptr;
  }
  if (axis_mask == 0) {
    return e;
  }
  BMVert *v1_mirr = this->vert_axes(e->v1, axis_mask);
  BMVert *v2_mirr = this->vert_axes(e->v2, axis_mask);
  if (v1_mirr && v2_mirr && LIKELY(v1_mirr != v2_mirr)) {
    return BM_edge_exists(v1_mirr, v2_mirr);
  }
  return nullptr;
}

BMEdge *EditMeshMirrorLookup::edge(BMEdge *e, const int axis)
{
  return this->edge_axes(e, 1 << axis);
}

BMFace *EditMeshMirrorLookup::face_axes(BMFace *f, const int axis_mask)
{
  if (f == nullptr) {
    return nullptr;
  }
  if (axis_mask == 0) {
    return f;
  }
  Array<BMVert *, BM_DEFAULT_NGON_STACK_SIZE> v_mirr_arr(f->len);
  BMLoop *l_iter = f->l_first;
  uint i = 0;
  do {
    BMVert *v_mirr = this->vert_axes(l_iter->v, axis_mask);
    if (v_mirr == nullptr) {
      return nullptr;
    }
    v_mirr_arr[i++] = v_mirr;
  } while ((l_iter = l_iter->next) != f->l_first);
  return BM_face_exists(v_mirr_arr.data(), v_mirr_arr.size());
}

BMFace *EditMeshMirrorLookup::face(BMFace *f, const int axis)
{
  return this->face_axes(f, 1 << axis);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name EditMeshSymmetryHelper API
 * \{ */

std::optional<EditMeshSymmetryHelper> EditMeshSymmetryHelper::create_if_needed(Object *ob,
                                                                               uchar htype)
{
  BLI_assert(htype != 0);
  BLI_assert((htype & ~(BM_VERT | BM_EDGE | BM_FACE)) == 0);

  if (!ob || !ob->data) {
    return std::nullopt;
  }
  Mesh *mesh = id_cast<Mesh *>(ob->data);
  BMEditMesh *em = BKE_editmesh_from_object(ob);
  BMesh *bm = BKE_editmesh_bmesh_get_for_write(mesh);

  if (!em || !bm || mesh->symmetry == 0) {
    return std::nullopt;
  }
  return EditMeshSymmetryHelper(ob, htype);
}

/** Close a per-element partner map under composition so X+Y+Z selects all octants,
 * not just the first axis. A vert at (1,1,1) must map to all seven other corners. */
template<typename T> static void mirror_map_add_partner(Map<T *, Vector<T *>> &map, T *a, T *b)
{
  if (a == nullptr || b == nullptr || a == b) {
    return;
  }
  Vector<T *> &va = map.lookup_or_add(a, {});
  if (!va.contains(b)) {
    va.append(b);
  }
  Vector<T *> &vb = map.lookup_or_add(b, {});
  if (!vb.contains(a)) {
    vb.append(a);
  }
}

template<typename T> static void mirror_map_close_transitive(Map<T *, Vector<T *>> &map)
{
  bool progress = true;
  while (progress) {
    progress = false;
    Vector<T *> keys;
    keys.reserve(map.size());
    for (const auto &item : map.items()) {
      keys.append(item.key);
    }
    for (T *v : keys) {
      const Vector<T *> *partners = map.lookup_ptr(v);
      if (partners == nullptr) {
        continue;
      }
      const Vector<T *> current = *partners;
      for (T *p : current) {
        const Vector<T *> *pp = map.lookup_ptr(p);
        if (pp == nullptr) {
          continue;
        }
        for (T *p2 : *pp) {
          if (p2 == v) {
            continue;
          }
          Vector<T *> &dst = map.lookup_or_add(v, {});
          if (!dst.contains(p2)) {
            dst.append(p2);
            progress = true;
          }
          Vector<T *> &dst2 = map.lookup_or_add(p2, {});
          if (!dst2.contains(v)) {
            dst2.append(v);
          }
        }
      }
    }
  }
}

EditMeshSymmetryHelper::EditMeshSymmetryHelper(Object *ob, uchar htype)
    : em_(BKE_editmesh_from_object(ob)),
      mesh_(id_cast<Mesh *>(ob->data)),
      bm_(BKE_editmesh_bmesh_get_for_write(mesh_)),
      htype_(htype)
{
  use_topology_mirror_ = (mesh_->editflag & ME_EDIT_MIRROR_TOPO) != 0;

  BMIter iter;

  /* Pair each enabled axis independently. use_select=true: only *query* selected
   * verts (the KD-tree still holds the whole visible mesh, so unselected
   * counterparts resolve). Iterating only selected edges/faces is what keeps
   * this cheaper than a full-mesh map. */
  for (int axis = 0; axis < 3; axis++) {
    if ((mesh_->symmetry & (ME_SYMMETRY_X << axis)) == 0) {
      continue;
    }
    EDBM_verts_mirror_cache_begin(em_, bm_, axis, true, true, true, use_topology_mirror_);

    if (htype_ & BM_VERT) {
      BMVert *v_curr;
      BM_ITER_MESH (v_curr, &iter, bm_, BM_VERTS_OF_MESH) {
        if (!BM_elem_flag_test(v_curr, BM_ELEM_SELECT) || BM_elem_flag_test(v_curr, BM_ELEM_HIDDEN))
        {
          continue;
        }
        BMVert *v_mirr = EDBM_verts_mirror_get(em_, bm_, v_curr);
        if (v_mirr && v_mirr != v_curr) {
          BMVert *v_mirr_check = EDBM_verts_mirror_get(em_, bm_, v_mirr);
          if (v_mirr_check == v_curr) {
            mirror_map_add_partner(vert_to_mirror_map_, v_curr, v_mirr);
          }
        }
      }
    }

    if (htype_ & BM_EDGE) {
      BMEdge *e_curr;
      BM_ITER_MESH (e_curr, &iter, bm_, BM_EDGES_OF_MESH) {
        if (!BM_elem_flag_test(e_curr, BM_ELEM_SELECT) || BM_elem_flag_test(e_curr, BM_ELEM_HIDDEN))
        {
          continue;
        }
        BMEdge *e_mirr = EDBM_verts_mirror_get_edge(em_, bm_, e_curr);
        if (e_mirr && e_mirr != e_curr) {
          BMEdge *e_mirr_check = EDBM_verts_mirror_get_edge(em_, bm_, e_mirr);
          if (e_mirr_check == e_curr) {
            mirror_map_add_partner(edge_to_mirror_map_, e_curr, e_mirr);
          }
        }
      }
    }

    if (htype_ & BM_FACE) {
      BMFace *f_curr;
      BM_ITER_MESH (f_curr, &iter, bm_, BM_FACES_OF_MESH) {
        if (!BM_elem_flag_test(f_curr, BM_ELEM_SELECT) || BM_elem_flag_test(f_curr, BM_ELEM_HIDDEN))
        {
          continue;
        }
        BMFace *f_mirr = EDBM_verts_mirror_get_face(em_, bm_, f_curr);
        if (f_mirr && f_mirr != f_curr) {
          BMFace *f_mirr_check = EDBM_verts_mirror_get_face(em_, bm_, f_mirr);
          if (f_mirr_check == f_curr) {
            mirror_map_add_partner(face_to_mirror_map_, f_curr, f_mirr);
          }
        }
      }
    }

    EDBM_verts_mirror_cache_end(em_);
  }

  /* Verts/edges/faces need XY/XZ/YZ images so a corner still expands under
   * multi-axis symmetry. Query each combo in one flip (not chained unique-hits).
   * Do not close_transitive on edges/faces: that made Mark Sharp tag a ring. */
  const int enabled = int(mesh_->symmetry) &
                      (ME_SYMMETRY_X | ME_SYMMETRY_Y | ME_SYMMETRY_Z);
  int axis_count = 0;
  for (int axis = 0; axis < 3; axis++) {
    axis_count += (enabled & (1 << axis)) != 0;
  }
  if (axis_count > 1) {
    EditMeshMirrorLookup lookup(em_, use_topology_mirror_, true, 0.00002f, enabled);
    Vector<int> masks;
    for (int bits = 1; bits < 8; bits++) {
      if ((bits & enabled) == bits) {
        masks.append(bits);
      }
    }
    if (htype_ & BM_VERT) {
      Vector<BMVert *> work;
      Set<BMVert *> seen;
      /* Selected verts first: the lookup reaches their counterparts from them. */
      BMIter iter;
      BMVert *v_curr;
      BM_ITER_MESH (v_curr, &iter, bm_, BM_VERTS_OF_MESH) {
        if (!BM_elem_flag_test(v_curr, BM_ELEM_SELECT) ||
            BM_elem_flag_test(v_curr, BM_ELEM_HIDDEN))
        {
          continue;
        }
        if (seen.add(v_curr)) {
          work.append(v_curr);
        }
      }
      /* Also start from the per-axis partners not reached that way. */
      for (const auto &item : vert_to_mirror_map_.items()) {
        if (seen.add(item.key)) {
          work.append(item.key);
        }
      }
      for (int i = 0; i < work.size(); i++) {
        BMVert *v = work[i];
        for (const int bits : masks) {
          BMVert *v_mirr = lookup.vert_axes(v, bits);
          if (v_mirr && v_mirr != v) {
            mirror_map_add_partner(vert_to_mirror_map_, v, v_mirr);
            if (seen.add(v_mirr)) {
              work.append(v_mirr);
            }
          }
        }
      }
    }
    if (htype_ & BM_EDGE) {
      BMIter iter;
      BMEdge *e_curr;
      BM_ITER_MESH (e_curr, &iter, bm_, BM_EDGES_OF_MESH) {
        if (!BM_elem_flag_test(e_curr, BM_ELEM_SELECT) ||
            BM_elem_flag_test(e_curr, BM_ELEM_HIDDEN))
        {
          continue;
        }
        for (const int bits : masks) {
          BMEdge *e_mirr = lookup.edge_axes(e_curr, bits);
          if (e_mirr && e_mirr != e_curr) {
            mirror_map_add_partner(edge_to_mirror_map_, e_curr, e_mirr);
          }
        }
      }
    }
    if (htype_ & BM_FACE) {
      BMIter iter;
      BMFace *f_curr;
      BM_ITER_MESH (f_curr, &iter, bm_, BM_FACES_OF_MESH) {
        if (!BM_elem_flag_test(f_curr, BM_ELEM_SELECT) ||
            BM_elem_flag_test(f_curr, BM_ELEM_HIDDEN))
        {
          continue;
        }
        for (const int bits : masks) {
          BMFace *f_mirr = lookup.face_axes(f_curr, bits);
          if (f_mirr && f_mirr != f_curr) {
            mirror_map_add_partner(face_to_mirror_map_, f_curr, f_mirr);
          }
        }
      }
    }
  }
  if (htype_ & BM_VERT) {
    mirror_map_close_transitive(vert_to_mirror_map_);
  }
}
void EditMeshSymmetryHelper::apply_on_mirror_verts(BMVert *v, FunctionRef<void(BMVert *)> op) const
{
  BLI_assert((this->htype_ & BM_VERT) != 0);
  const Vector<BMVert *> *mirrors = this->vert_to_mirror_map_.lookup_ptr(v);
  if (mirrors) {
    for (BMVert *v_mirr : *mirrors) {
      op(v_mirr);
    }
  }
}

void EditMeshSymmetryHelper::apply_on_mirror_edges(BMEdge *e, FunctionRef<void(BMEdge *)> op) const
{
  BLI_assert((this->htype_ & BM_EDGE) != 0);
  const Vector<BMEdge *> *mirrors = this->edge_to_mirror_map_.lookup_ptr(e);
  if (mirrors) {
    for (BMEdge *e_mirr : *mirrors) {
      op(e_mirr);
    }
  }
}

void EditMeshSymmetryHelper::apply_on_mirror_faces(BMFace *f, FunctionRef<void(BMFace *)> op) const
{
  BLI_assert((this->htype_ & BM_FACE) != 0);
  const Vector<BMFace *> *mirrors = this->face_to_mirror_map_.lookup_ptr(f);
  if (mirrors) {
    for (BMFace *f_mirr : *mirrors) {
      op(f_mirr);
    }
  }
}

bool EditMeshSymmetryHelper::any_mirror_vert_selected(BMVert *v, const char hflag) const
{
  BLI_assert((this->htype_ & BM_VERT) != 0);
  const Vector<BMVert *> *mirrors = this->vert_to_mirror_map_.lookup_ptr(v);
  if (mirrors) {
    for (BMVert *v_mirr : *mirrors) {
      if (BM_elem_flag_test(v_mirr, hflag) && !BM_elem_flag_test(v_mirr, BM_ELEM_HIDDEN)) {
        return true;
      }
    }
  }
  return false;
}

bool EditMeshSymmetryHelper::any_mirror_edge_selected(BMEdge *e, const char hflag) const
{
  BLI_assert((this->htype_ & BM_EDGE) != 0);
  const Vector<BMEdge *> *mirrors = this->edge_to_mirror_map_.lookup_ptr(e);
  if (mirrors) {
    for (BMEdge *e_mirr : *mirrors) {
      if (BM_elem_flag_test(e_mirr, hflag) && !BM_elem_flag_test(e_mirr, BM_ELEM_HIDDEN)) {
        return true;
      }
    }
  }
  return false;
}

bool EditMeshSymmetryHelper::any_mirror_face_selected(BMFace *f, const char hflag) const
{
  BLI_assert((this->htype_ & BM_FACE) != 0);
  const Vector<BMFace *> *mirrors = this->face_to_mirror_map_.lookup_ptr(f);
  if (mirrors) {
    for (BMFace *f_mirr : *mirrors) {
      if (BM_elem_flag_test(f_mirr, hflag) && !BM_elem_flag_test(f_mirr, BM_ELEM_HIDDEN)) {
        return true;
      }
    }
  }
  return false;
}

void EditMeshSymmetryHelper::set_hflag_on_mirror_verts(BMVert *v,
                                                       const char hflag,
                                                       const bool value) const
{
  apply_on_mirror_verts(v, [this, hflag, value](BMVert *v_mirr) {
    if (hflag & BM_ELEM_SELECT) {
      BM_vert_select_set(this->bm_, v_mirr, value);
    }
    const char hflag_test = char(hflag & ~BM_ELEM_SELECT);
    if (hflag_test) {
      BM_elem_flag_set(v_mirr, hflag_test, value);
    }
  });
}

void EditMeshSymmetryHelper::set_hflag_on_mirror_edges(BMEdge *e,
                                                       char hflag,
                                                       const bool value) const
{
  apply_on_mirror_edges(e, [this, hflag, value](BMEdge *e_mirr) {
    if (hflag & BM_ELEM_SELECT) {
      BM_edge_select_set(this->bm_, e_mirr, value);
    }
    char hflag_test = char(hflag & ~BM_ELEM_SELECT);
    if (hflag_test) {
      BM_elem_flag_set(e_mirr, hflag_test, value);
    }
  });
}

void EditMeshSymmetryHelper::set_hflag_on_mirror_faces(BMFace *f,
                                                       const char hflag,
                                                       const bool value) const
{
  apply_on_mirror_faces(f, [this, hflag, value](BMFace *f_mirr) {
    if (hflag & BM_ELEM_SELECT) {
      BM_face_select_set(this->bm_, f_mirr, value);
    }
    char hflag_test = char(hflag & ~BM_ELEM_SELECT);
    if (hflag_test) {
      BM_elem_flag_set(f_mirr, hflag_test, value);
    }
  });
}

bool EditMeshSymmetryHelper::has_mirror_edge(BMEdge *e) const
{
  return edge_to_mirror_map_.contains(e);
}

bool EditMeshSymmetryHelper::has_mirror_face(BMFace *f) const
{
  return face_to_mirror_map_.contains(f);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Mirrored Selection Expansion
 * \{ */

/* An element "spans" the mirror plane when it has verts strictly on both sides of an
 * enabled symmetry plane. Such elements are their own mirror counterpart; if the selection
 * flush promotes them (both sides' selections combine to select them), topology operators
 * would create bridge geometry across the plane. Elements lying on the plane itself do not
 * span it and must stay selected for region connectivity. */
static bool edge_spans_mirror_plane(const BMEdge *e, const short symmetry)
{
  for (int a = 0; a < 3; a++) {
    if (symmetry & (ME_SYMMETRY_X << a)) {
      const float c1 = e->v1->co[a];
      const float c2 = e->v2->co[a];
      if ((c1 > 1e-4f && c2 < -1e-4f) || (c2 > 1e-4f && c1 < -1e-4f)) {
        return true;
      }
    }
  }
  return false;
}

static bool face_spans_mirror_plane(const BMFace *f, const short symmetry)
{
  for (int a = 0; a < 3; a++) {
    if (symmetry & (ME_SYMMETRY_X << a)) {
      bool pos = false, neg = false;
      const BMLoop *l_iter = f->l_first;
      do {
        const float c = l_iter->v->co[a];
        pos |= c > 1e-4f;
        neg |= c < -1e-4f;
      } while ((l_iter = l_iter->next) != f->l_first);
      if (pos && neg) {
        return true;
      }
    }
  }
  return false;
}

/* Per-axis helper maps only store partners of the *original* selection. With
 * X+Y that is Q1 and Q3 of a Q4 face, not Q2. Query every non-empty subset of
 * enabled axes in one spatial flip so the diagonal octant is selected too. */
static void expand_flood_mirror_lookup(BMEditMesh *em,
                                          const Mesh *mesh,
                                          const uchar htype,
                                          const float pair_dist)
{
  BMesh *bm = em->bm;
  const bool use_topology = (mesh->editflag & ME_EDIT_MIRROR_TOPO) != 0;
  const int enabled = int(mesh->symmetry) &
                      (ME_SYMMETRY_X | ME_SYMMETRY_Y | ME_SYMMETRY_Z);
  EditMeshMirrorLookup lookup(em, use_topology, true, pair_dist, enabled);
  BMIter iter;

  auto enabled_masks = [enabled]() {
    Vector<int> masks;
    for (int bits = 1; bits < 8; bits++) {
      if ((bits & enabled) == bits) {
        masks.append(bits);
      }
    }
    return masks;
  };
  const Vector<int> masks = enabled_masks();
  if (masks.is_empty()) {
    return;
  }

  if (htype & BM_VERT) {
    Vector<BMVert *> work;
    Set<BMVert *> seen;
    BMVert *v;
    BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
      if (BM_elem_flag_test(v, BM_ELEM_SELECT) && !BM_elem_flag_test(v, BM_ELEM_HIDDEN)) {
        if (seen.add(v)) {
          work.append(v);
        }
      }
    }
    for (int i = 0; i < work.size(); i++) {
      BMVert *curr = work[i];
      for (const int bits : masks) {
        BMVert *mirr = lookup.vert_axes(curr, bits);
        if (mirr == nullptr || mirr == curr || BM_elem_flag_test(mirr, BM_ELEM_HIDDEN)) {
          continue;
        }
        if (!BM_elem_flag_test(mirr, BM_ELEM_SELECT)) {
          BM_vert_select_set(bm, mirr, true);
        }
        if (seen.add(mirr)) {
          work.append(mirr);
        }
      }
    }
  }

  if (htype & BM_EDGE) {
    Vector<BMEdge *> work;
    Set<BMEdge *> seen;
    BMEdge *e;
    BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
      if (BM_elem_flag_test(e, BM_ELEM_SELECT) && !BM_elem_flag_test(e, BM_ELEM_HIDDEN)) {
        if (seen.add(e)) {
          work.append(e);
        }
      }
    }
    for (int i = 0; i < work.size(); i++) {
      BMEdge *curr = work[i];
      for (const int bits : masks) {
        BMEdge *mirr = lookup.edge_axes(curr, bits);
        if (mirr == nullptr || mirr == curr || BM_elem_flag_test(mirr, BM_ELEM_HIDDEN)) {
          continue;
        }
        if (!BM_elem_flag_test(mirr, BM_ELEM_SELECT)) {
          BM_edge_select_set(bm, mirr, true);
        }
        BM_elem_flag_enable(mirr, BM_ELEM_TAG);
        if (seen.add(mirr)) {
          work.append(mirr);
        }
      }
    }
  }

  if (htype & BM_FACE) {
    Vector<BMFace *> work;
    Set<BMFace *> seen;
    BMFace *f;
    BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
      if (BM_elem_flag_test(f, BM_ELEM_SELECT) && !BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
        if (seen.add(f)) {
          work.append(f);
        }
      }
    }
    for (int i = 0; i < work.size(); i++) {
      BMFace *curr = work[i];
      for (const int bits : masks) {
        BMFace *mirr = lookup.face_axes(curr, bits);
        if (mirr == nullptr || mirr == curr || BM_elem_flag_test(mirr, BM_ELEM_HIDDEN)) {
          continue;
        }
        if (!BM_elem_flag_test(mirr, BM_ELEM_SELECT)) {
          BM_face_select_set(bm, mirr, true);
        }
        BM_elem_flag_enable(mirr, BM_ELEM_TAG);
        if (seen.add(mirr)) {
          work.append(mirr);
        }
      }
    }
  }
}

/* -------------------------------------------------------------------- */
/* Expansion from the selected elements.
 *
 * The expansion below walks every edge and face of the mesh several times (tagging, the
 * selection flush, the cleanup after it). On a dense mesh those passes cost far more than
 * finding the counterparts of a handful of selected elements, so a small selection is
 * expanded from the selected elements themselves and only touches what is next to them. */

struct MirrorSelection {
  Vector<BMVert *> verts;
  Vector<BMEdge *> edges;
  Vector<BMFace *> faces;
};

static bool mirror_elem_is_selected(const BMHeader *head)
{
  return (head->hflag & (BM_ELEM_SELECT | BM_ELEM_HIDDEN)) == BM_ELEM_SELECT;
}

/**
 * Gather the selection with one pass over the verts: a selected edge or face hangs off
 * selected verts. Returns false when the selection is too large for this to pay off, or
 * when the counts show that something selected was not found this way.
 */
static bool mirror_selection_gather(BMesh *bm, MirrorSelection &r_selection)
{
  if (int64_t(bm->totvertsel) * 8 > int64_t(bm->totvert)) {
    return false;
  }
  r_selection.verts.reserve(bm->totvertsel);
  r_selection.edges.reserve(bm->totedgesel);
  r_selection.faces.reserve(bm->totfacesel);

  BMIter iter;
  BMVert *v;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    if (mirror_elem_is_selected(&v->head)) {
      r_selection.verts.append(v);
    }
  }
  for (BMVert *v_sel : r_selection.verts) {
    BMIter eiter;
    BMEdge *e;
    BM_ITER_ELEM (e, &eiter, v_sel, BM_EDGES_OF_VERT) {
      if (!mirror_elem_is_selected(&e->head)) {
        continue;
      }
      /* Each edge once, from its first selected vert. */
      const BMVert *v_owner = mirror_elem_is_selected(&e->v1->head) ? e->v1 : e->v2;
      if (v_owner == v_sel) {
        r_selection.edges.append(e);
      }
    }
    BMIter fiter;
    BMFace *f;
    BM_ITER_ELEM (f, &fiter, v_sel, BM_FACES_OF_VERT) {
      if (!mirror_elem_is_selected(&f->head)) {
        continue;
      }
      /* Each face once, from its first selected vert. */
      const BMLoop *l_iter = f->l_first;
      do {
        if (mirror_elem_is_selected(&l_iter->v->head)) {
          break;
        }
      } while ((l_iter = l_iter->next) != f->l_first);
      if (l_iter->v == v_sel) {
        r_selection.faces.append(f);
      }
    }
  }

  return (r_selection.verts.size() == bm->totvertsel) &&
         (r_selection.edges.size() == bm->totedgesel) &&
         (r_selection.faces.size() == bm->totfacesel);
}

static void expand_mirrored_from_selection(BMEditMesh *em,
                                           const Mesh *mesh,
                                           const MirrorSelection &selection,
                                           const uchar htype,
                                           const float pair_dist)
{
  BMesh *bm = em->bm;
  const short symmetry = mesh->symmetry;
  const bool use_topology = (mesh->editflag & ME_EDIT_MIRROR_TOPO) != 0;
  const int enabled = int(symmetry) & (ME_SYMMETRY_X | ME_SYMMETRY_Y | ME_SYMMETRY_Z);
  Vector<int, 7> masks;
  for (int bits = 1; bits < 8; bits++) {
    if ((bits & enabled) == bits) {
      masks.append(bits);
    }
  }
  if (masks.is_empty()) {
    return;
  }
  EditMeshMirrorLookup lookup(em, use_topology, true, pair_dist, enabled);

  /* Everything that becomes selected, to finish the selection around it afterwards. */
  Vector<BMVert *> new_verts;
  Vector<BMEdge *> new_edges;
  Vector<BMFace *> new_faces;
  auto vert_select = [&](BMVert *v) {
    if ((v->head.hflag & (BM_ELEM_SELECT | BM_ELEM_HIDDEN)) == 0) {
      BM_vert_select_set(bm, v, true);
      new_verts.append(v);
    }
  };
  auto edge_select = [&](BMEdge *e) {
    if (BM_elem_flag_test(e, BM_ELEM_HIDDEN)) {
      return;
    }
    if (!BM_elem_flag_test(e, BM_ELEM_SELECT)) {
      BM_edge_select_set_noflush(bm, e, true);
      new_edges.append(e);
    }
    vert_select(e->v1);
    vert_select(e->v2);
  };
  auto face_select = [&](BMFace *f) {
    if (BM_elem_flag_test(f, BM_ELEM_HIDDEN)) {
      return;
    }
    if (!BM_elem_flag_test(f, BM_ELEM_SELECT)) {
      BM_face_select_set_noflush(bm, f, true);
      new_faces.append(f);
    }
    BMLoop *l_iter = f->l_first;
    do {
      vert_select(l_iter->v);
      edge_select(l_iter->e);
    } while ((l_iter = l_iter->next) != f->l_first);
  };

  /* Flood through the lookup so X+Y (and XYZ) reach the diagonal octants, same as
   * #expand_flood_mirror_lookup. */
  if (htype & BM_VERT) {
    Vector<BMVert *> work = selection.verts;
    Set<BMVert *> seen;
    seen.add_multiple(work);
    for (int i = 0; i < work.size(); i++) {
      BMVert *curr = work[i];
      for (const int bits : masks) {
        BMVert *mirr = lookup.vert_axes(curr, bits);
        if (mirr == nullptr || mirr == curr || BM_elem_flag_test(mirr, BM_ELEM_HIDDEN)) {
          continue;
        }
        vert_select(mirr);
        if (seen.add(mirr)) {
          work.append(mirr);
        }
      }
    }
  }
  if (htype & BM_EDGE) {
    Vector<BMEdge *> work = selection.edges;
    Set<BMEdge *> seen;
    seen.add_multiple(work);
    for (int i = 0; i < work.size(); i++) {
      BMEdge *curr = work[i];
      for (const int bits : masks) {
        BMEdge *mirr = lookup.edge_axes(curr, bits);
        if (mirr == nullptr || mirr == curr || BM_elem_flag_test(mirr, BM_ELEM_HIDDEN)) {
          continue;
        }
        edge_select(mirr);
        if (seen.add(mirr)) {
          work.append(mirr);
        }
      }
    }
  }
  if (htype & BM_FACE) {
    Vector<BMFace *> work = selection.faces;
    Set<BMFace *> seen;
    seen.add_multiple(work);
    for (int i = 0; i < work.size(); i++) {
      BMFace *curr = work[i];
      for (const int bits : masks) {
        BMFace *mirr = lookup.face_axes(curr, bits);
        if (mirr == nullptr || mirr == curr || BM_elem_flag_test(mirr, BM_ELEM_HIDDEN)) {
          continue;
        }
        face_select(mirr);
        if (seen.add(mirr)) {
          work.append(mirr);
        }
      }
    }
  }

  /* What the selection flush adds around the new elements: in vertex mode an edge or face
   * whose verts are all selected, in edge mode a face whose edges are all selected.
   * Bridges across a mirror plane are left out. They are their own counterpart, and if
   * both sides' selections combine to select them, topology operators build geometry
   * across the plane. Elements lying on the plane do not span it and are promoted. */
  Vector<BMEdge *> promoted_edges;
  Vector<BMFace *> promoted_faces;
  if (em->selectmode & SCE_SELECT_VERTEX) {
    for (BMVert *v : new_verts) {
      BMIter eiter;
      BMEdge *e;
      BM_ITER_ELEM (e, &eiter, v, BM_EDGES_OF_VERT) {
        if (e->head.hflag & (BM_ELEM_SELECT | BM_ELEM_HIDDEN)) {
          continue;
        }
        if (!BM_elem_flag_test(BM_edge_other_vert(e, v), BM_ELEM_SELECT)) {
          continue;
        }
        if (edge_spans_mirror_plane(e, symmetry)) {
          continue;
        }
        BM_edge_select_set_noflush(bm, e, true);
        promoted_edges.append(e);
      }
    }
    for (BMVert *v : new_verts) {
      BMIter fiter;
      BMFace *f;
      BM_ITER_ELEM (f, &fiter, v, BM_FACES_OF_VERT) {
        if (f->head.hflag & (BM_ELEM_SELECT | BM_ELEM_HIDDEN)) {
          continue;
        }
        bool all_selected = true;
        const BMLoop *l_iter = f->l_first;
        do {
          if (!BM_elem_flag_test(l_iter->v, BM_ELEM_SELECT)) {
            all_selected = false;
            break;
          }
        } while ((l_iter = l_iter->next) != f->l_first);
        if (!all_selected || face_spans_mirror_plane(f, symmetry)) {
          continue;
        }
        BM_face_select_set_noflush(bm, f, true);
        promoted_faces.append(f);
      }
    }
  }
  else if (em->selectmode & SCE_SELECT_EDGE) {
    for (BMEdge *e : new_edges) {
      BMIter fiter;
      BMFace *f;
      BM_ITER_ELEM (f, &fiter, e, BM_FACES_OF_EDGE) {
        if (f->head.hflag & (BM_ELEM_SELECT | BM_ELEM_HIDDEN)) {
          continue;
        }
        bool all_selected = true;
        const BMLoop *l_iter = f->l_first;
        do {
          if (!BM_elem_flag_test(l_iter->e, BM_ELEM_SELECT)) {
            all_selected = false;
            break;
          }
        } while ((l_iter = l_iter->next) != f->l_first);
        if (!all_selected || face_spans_mirror_plane(f, symmetry)) {
          continue;
        }
        BM_face_select_set_noflush(bm, f, true);
        promoted_faces.append(f);
      }
    }
  }

  /* Guarantee down-consistency in every select mode (selected faces flag their edges and
   * verts, selected edges flag their verts): region detection in the BMOs relies on the
   * shared edges of multi-face regions being flagged. */
  auto face_flush_down = [&](BMFace *f) {
    BMLoop *l_iter = f->l_first;
    do {
      BM_edge_select_set_noflush(bm, l_iter->e, true);
      BM_vert_select_set(bm, l_iter->v, true);
    } while ((l_iter = l_iter->next) != f->l_first);
  };
  auto edge_flush_down = [&](BMEdge *e) {
    BM_vert_select_set(bm, e->v1, true);
    BM_vert_select_set(bm, e->v2, true);
  };
  for (BMFace *f : selection.faces) {
    face_flush_down(f);
  }
  for (BMFace *f : promoted_faces) {
    face_flush_down(f);
  }
  for (BMEdge *e : selection.edges) {
    edge_flush_down(e);
  }
  for (BMEdge *e : promoted_edges) {
    edge_flush_down(e);
  }
}

float EDBM_mirror_pair_threshold(const Scene *scene)
{
  constexpr float fallback = 0.00002f;
  if (scene == nullptr || scene->toolsettings == nullptr) {
    return fallback;
  }
  const float threshold = scene->toolsettings->mesh_mirror_threshold;
  return (threshold > 0.0f) ? threshold : fallback;
}

void EDBM_select_expand_mirrored(Object *ob, BMEditMesh *em, const float pair_dist)
{
  BMesh *bm = em->bm;
  const float maxdist = (pair_dist > 0.0f) ? pair_dist : 0.00002f;
  uchar htype = 0;
  if (bm->totvertsel) {
    htype |= BM_VERT;
  }
  if (bm->totedgesel) {
    htype |= BM_EDGE;
  }
  if (bm->totfacesel) {
    htype |= BM_FACE;
  }
  if (htype == 0) {
    return;
  }

  Mesh *mesh = id_cast<Mesh *>(ob->data);
  if (mesh->symmetry != 0) {
    /* A small selection only touches what is next to it. */
    MirrorSelection selection;
    if (mirror_selection_gather(bm, selection)) {
      expand_mirrored_from_selection(em, mesh, selection, htype, maxdist);
      return;
    }
  }

  std::optional<EditMeshSymmetryHelper> symmetry = EditMeshSymmetryHelper::create_if_needed(
      ob, htype);
  if (!symmetry) {
    return;
  }

  BMIter iter;
  BMEdge *e;
  BMFace *f;

  /* Tag the user's current edge/face selection, so elements merely promoted by the
   * selection flush below can be told apart from intended ones. */
  BM_mesh_elem_hflag_disable_all(bm, BM_EDGE | BM_FACE, BM_ELEM_TAG, false);
  if (htype & BM_EDGE) {
    BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
      BM_elem_flag_set(e, BM_ELEM_TAG, BM_elem_flag_test(e, BM_ELEM_SELECT));
    }
  }
  if (htype & BM_FACE) {
    BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
      BM_elem_flag_set(f, BM_ELEM_TAG, BM_elem_flag_test(f, BM_ELEM_SELECT));
    }
  }

  /* Flood through lookup so X+Y (and XYZ) reach diagonal octants. The helper
   * maps stay per-axis for Mark Sharp; they only see the original selection. */
  expand_flood_mirror_lookup(em, mesh, htype, maxdist);

  EDBM_selectmode_flush(em->bm, em->selectmode);

  /* Drop flush collateral: cross-plane bridges that neither the user selected (tagged)
   * nor mirroring added. A legitimate X-mirror face can still straddle Y=0 or Z=0
   * (e.g. a Suzanne cheek quad with one vertex just below Z=0); those must not be
   * dropped just because additional symmetry axes are enabled. Self-mirrored
   * elements are never stored in the mirror maps, so "in the map" means a real
   * counterpart and must be kept. */
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    if (BM_elem_flag_test(e, BM_ELEM_SELECT) && !BM_elem_flag_test(e, BM_ELEM_TAG) &&
        !symmetry->has_mirror_edge(e) && edge_spans_mirror_plane(e, mesh->symmetry))
    {
      BM_elem_flag_disable(e, BM_ELEM_SELECT);
    }
  }
  BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
    if (BM_elem_flag_test(f, BM_ELEM_SELECT) && !BM_elem_flag_test(f, BM_ELEM_TAG) &&
        !symmetry->has_mirror_face(f) && face_spans_mirror_plane(f, mesh->symmetry))
    {
      BM_elem_flag_disable(f, BM_ELEM_SELECT);
    }
  }

  /* Guarantee down-consistency in every select mode (selected faces flag their edges and
   * verts, selected edges flag their verts). The mode-based flush only goes down in
   * edge/face modes, and region detection in the BMOs relies on the shared edges of
   * multi-face regions being flagged. */
  BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
    if (BM_elem_flag_test(f, BM_ELEM_SELECT)) {
      BMLoop *l_iter = f->l_first;
      do {
        BM_elem_flag_enable(l_iter->e, BM_ELEM_SELECT);
        BM_elem_flag_enable(l_iter->v, BM_ELEM_SELECT);
      } while ((l_iter = l_iter->next) != f->l_first);
    }
  }
  BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
    if (BM_elem_flag_test(e, BM_ELEM_SELECT)) {
      BM_elem_flag_enable(e->v1, BM_ELEM_SELECT);
      BM_elem_flag_enable(e->v2, BM_ELEM_SELECT);
    }
  }

  BM_mesh_elem_hflag_disable_all(bm, BM_EDGE | BM_FACE, BM_ELEM_TAG, false);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Mirrored Active-Side Restore
 *
 * Topology operators that store select history for their newly created elements
 * (e.g. extrude with `use_select_history`) leave the active element on an arbitrary side
 * of the mirror plane when the selection was mirror-expanded. The transform mirror pairing
 * follows the active element's side, so an arbitrary active flips which side follows the
 * mouse. Capture the user's active element before the op and restore the newest same-side
 * history entry afterwards.
 * \{ */

static bool mirror_ele_center(const BMElem *ele, const char htype, float r_co[3])
{
  if (htype == BM_VERT) {
    copy_v3_v3(r_co, reinterpret_cast<const BMVert *>(ele)->co);
    return true;
  }
  if (htype == BM_EDGE) {
    const BMEdge *e = reinterpret_cast<const BMEdge *>(ele);
    mid_v3_v3v3(r_co, e->v1->co, e->v2->co);
    return true;
  }
  if (htype == BM_FACE) {
    const BMFace *f = reinterpret_cast<const BMFace *>(ele);
    if (f->l_first) {
      zero_v3(r_co);
      const BMLoop *l_iter = f->l_first;
      int tot = 0;
      do {
        add_v3_v3(r_co, l_iter->v->co);
        tot++;
      } while ((l_iter = l_iter->next) != f->l_first);
      if (tot) {
        mul_v3_fl(r_co, 1.0f / float(tot));
      }
      return true;
    }
  }
  return false;
}

float3 EDBM_mirror_active_center_capture(Object *ob, BMEditMesh *em, bool &r_has_active)
{
  float3 co(0.0f);
  r_has_active = false;
  const Mesh *mesh = id_cast<Mesh *>(ob->data);
  if (mesh->symmetry == 0) {
    return co;
  }
  BMEditSelection ese;
  if (BM_select_history_active_get(em->bm, &ese)) {
    if (mirror_ele_center(ese.ele, ese.htype, co)) {
      r_has_active = true;
      return co;
    }
  }
  /* No active element: use the selection centroid so we still know the user's side. */
  BMesh *bm = em->bm;
  if (bm->totvertsel > 0) {
    BMIter iter;
    BMVert *v;
    int tot = 0;
    BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
      if (BM_elem_flag_test(v, BM_ELEM_SELECT)) {
        add_v3_v3(co, v->co);
        tot++;
      }
    }
    if (tot) {
      mul_v3_fl(co, 1.0f / float(tot));
      r_has_active = true;
    }
  }
  return co;
}

static bool co_on_user_side(const float co[3], const int want[3])
{
  for (int a = 0; a < 3; a++) {
    if (want[a] != 0 && co[a] * float(want[a]) < -1e-4f) {
      return false;
    }
  }
  return true;
}

void EDBM_mirror_active_side_restore(Object *ob,
                                     BMEditMesh *em,
                                     const float3 &ref_co,
                                     const bool deselect_other)
{
  const Mesh *mesh = id_cast<Mesh *>(ob->data);
  if (mesh->symmetry == 0) {
    return;
  }
  BMesh *bm = em->bm;

  /* Desired side per enabled axis (0 = don't care, e.g. the reference was on the plane). */
  int want[3] = {0, 0, 0};
  bool any = false;
  for (int a = 0; a < 3; a++) {
    if (mesh->symmetry & (ME_SYMMETRY_X << a)) {
      if (ref_co[a] > 1e-4f) {
        want[a] = 1;
        any = true;
      }
      else if (ref_co[a] < -1e-4f) {
        want[a] = -1;
        any = true;
      }
    }
  }
  if (!any) {
    return;
  }

  /* Loop-cut + edge-slide: both rings stay selected so slide applies the same factor
   * on both sides (same world direction). Deselect the opposite ring; mesh-symmetry
   * then copies the flipped location. Extrude keeps both caps selected (pairing). */
  if (deselect_other) {
    BMIter iter;
    BMFace *f;
    BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
      if (!BM_elem_flag_test(f, BM_ELEM_SELECT)) {
        continue;
      }
      float co[3];
      if (mirror_ele_center(reinterpret_cast<BMElem *>(f), BM_FACE, co) &&
          !co_on_user_side(co, want))
      {
        BM_face_select_set(bm, f, false);
      }
    }
    BMEdge *e;
    BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
      if (!BM_elem_flag_test(e, BM_ELEM_SELECT)) {
        continue;
      }
      float co[3];
      if (mirror_ele_center(reinterpret_cast<BMElem *>(e), BM_EDGE, co) &&
          !co_on_user_side(co, want))
      {
        BM_edge_select_set(bm, e, false);
      }
    }
    BMVert *v;
    BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
      if (!BM_elem_flag_test(v, BM_ELEM_SELECT)) {
        continue;
      }
      if (!co_on_user_side(v->co, want)) {
        BM_vert_select_set(bm, v, false);
      }
    }
    EDBM_selectmode_flush(em->bm, em->selectmode);
  }

  /* Find the newest history entry on the user's side and make it the active element. */
  BMElem *active_ele = nullptr;
  for (BMEditSelection *ese = static_cast<BMEditSelection *>(bm->selected.last()); ese;
       ese = ese->prev)
  {
    float3 co;
    if (!mirror_ele_center(ese->ele, ese->htype, co)) {
      continue;
    }
    if (co_on_user_side(co, want)) {
      active_ele = ese->ele;
      break;
    }
  }
  if (active_ele == nullptr) {
    /* Scripted selections and some BMOs leave history empty. Fall back to any
     * selected element on the user's side so NORMAL orientation and the
     * transform quadrant still follow that side. */
    BMIter iter;
    if (bm->totfacesel > 0) {
      BMFace *f;
      BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
        if (!BM_elem_flag_test(f, BM_ELEM_SELECT)) {
          continue;
        }
        float3 co;
        if (mirror_ele_center(reinterpret_cast<BMElem *>(f), BM_FACE, co) &&
            co_on_user_side(co, want))
        {
          active_ele = reinterpret_cast<BMElem *>(f);
          bm->act_face = f;
          break;
        }
      }
    }
    if (active_ele == nullptr && bm->totedgesel > 0) {
      BMEdge *e;
      BM_ITER_MESH (e, &iter, bm, BM_EDGES_OF_MESH) {
        if (!BM_elem_flag_test(e, BM_ELEM_SELECT)) {
          continue;
        }
        float3 co;
        if (mirror_ele_center(reinterpret_cast<BMElem *>(e), BM_EDGE, co) &&
            co_on_user_side(co, want))
        {
          active_ele = reinterpret_cast<BMElem *>(e);
          break;
        }
      }
    }
    if (active_ele == nullptr) {
      BMVert *v;
      BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
        if (!BM_elem_flag_test(v, BM_ELEM_SELECT)) {
          continue;
        }
        if (co_on_user_side(v->co, want)) {
          active_ele = reinterpret_cast<BMElem *>(v);
          break;
        }
      }
    }
  }
  if (active_ele) {
    BM_select_history_remove(bm, active_ele);
    BM_select_history_store(bm, active_ele);
  }
}

float3 EDBM_mirror_user_side_capture(Object *ob, BMEditMesh *em, bool &r_has_side)
{
  float3 co(0.0f);
  r_has_side = false;
  const Mesh *mesh = id_cast<Mesh *>(ob->data);
  BMesh *bm = em->bm;
  if (mesh->symmetry == 0 || bm->totvertsel == 0) {
    return co;
  }

  bool pos[3] = {false, false, false};
  bool neg[3] = {false, false, false};
  BMIter iter;
  BMVert *v;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    if (!BM_elem_flag_test(v, BM_ELEM_SELECT) || BM_elem_flag_test(v, BM_ELEM_HIDDEN)) {
      continue;
    }
    for (int a = 0; a < 3; a++) {
      if (v->co[a] > 1e-4f) {
        pos[a] = true;
      }
      else if (v->co[a] < -1e-4f) {
        neg[a] = true;
      }
    }
  }
  for (int a = 0; a < 3; a++) {
    if ((mesh->symmetry & (ME_SYMMETRY_X << a)) && (pos[a] != neg[a])) {
      co[a] = pos[a] ? 1.0f : -1.0f;
      r_has_side = true;
    }
  }
  return co;
}

void EDBM_mirror_user_side_restore(Object *ob, BMEditMesh *em, const float3 &side_co)
{
  Mesh *mesh = id_cast<Mesh *>(ob->data);
  if (mesh->symmetry == 0) {
    return;
  }
  BMesh *bm = em->bm;

  int want[3] = {0, 0, 0};
  bool any = false;
  for (int a = 0; a < 3; a++) {
    if (mesh->symmetry & (ME_SYMMETRY_X << a)) {
      if (side_co[a] > 1e-4f) {
        want[a] = 1;
        any = true;
      }
      else if (side_co[a] < -1e-4f) {
        want[a] = -1;
        any = true;
      }
    }
  }
  if (!any) {
    return;
  }

  /* The select mode decides which element type carries the selection. In vertex mode a
   * selected edge across the plane must not hold on to its vert on the other side. In
   * edge/face mode an element whose center is on the plane straddles it and is kept whole. */
  const bool use_edges = (em->selectmode & SCE_SELECT_VERTEX) == 0;
  const bool use_faces = use_edges && (em->selectmode & SCE_SELECT_EDGE) == 0;

  /* Whatever has to go has a selected vert on the other side, so only those verts and
   * what they connect to need a look. Nothing else is touched, which keeps this cheap on
   * a dense mesh. */
  Vector<BMVert *> other_verts;
  BMIter iter;
  BMVert *v;
  BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
    if (BM_elem_flag_test(v, BM_ELEM_SELECT) && !co_on_user_side(v->co, want)) {
      other_verts.append(v);
    }
  }

  if (!other_verts.is_empty()) {
    Set<BMVert *> keep_verts;
    Set<BMEdge *> keep_edges;
    Set<BMFace *> keep_faces;
    BMIter eiter;
    BMIter fiter;
    BMEdge *e;
    BMFace *f;
    float co[3];
    if (use_edges) {
      for (BMVert *v_other : other_verts) {
        BM_ITER_ELEM (e, &eiter, v_other, BM_EDGES_OF_VERT) {
          if (BM_elem_flag_test(e, BM_ELEM_SELECT) &&
              (!mirror_ele_center(reinterpret_cast<BMElem *>(e), BM_EDGE, co) ||
               co_on_user_side(co, want)))
          {
            keep_edges.add(e);
            keep_verts.add(e->v1);
            keep_verts.add(e->v2);
          }
        }
        if (!use_faces) {
          continue;
        }
        BM_ITER_ELEM (f, &fiter, v_other, BM_FACES_OF_VERT) {
          if (BM_elem_flag_test(f, BM_ELEM_SELECT) &&
              (!mirror_ele_center(reinterpret_cast<BMElem *>(f), BM_FACE, co) ||
               co_on_user_side(co, want)))
          {
            if (keep_faces.add(f)) {
              BMLoop *l_iter = f->l_first;
              do {
                keep_edges.add(l_iter->e);
                keep_verts.add(l_iter->v);
              } while ((l_iter = l_iter->next) != f->l_first);
            }
          }
        }
      }
    }

    for (BMVert *v_other : other_verts) {
      if (use_faces) {
        BM_ITER_ELEM (f, &fiter, v_other, BM_FACES_OF_VERT) {
          if (!keep_faces.contains(f)) {
            BM_face_select_set_noflush(bm, f, false);
          }
        }
      }
      BM_ITER_ELEM (e, &eiter, v_other, BM_EDGES_OF_VERT) {
        if (!keep_edges.contains(e)) {
          BM_edge_select_set_noflush(bm, e, false);
        }
      }
      if (!keep_verts.contains(v_other)) {
        BM_vert_select_set(bm, v_other, false);
      }
    }
    if (!use_faces) {
      /* Faces follow their edges here. */
      for (BMVert *v_other : other_verts) {
        BM_ITER_ELEM (f, &fiter, v_other, BM_FACES_OF_VERT) {
          if (!BM_elem_flag_test(f, BM_ELEM_SELECT)) {
            continue;
          }
          BMLoop *l_iter = f->l_first;
          do {
            if (!BM_elem_flag_test(l_iter->e, BM_ELEM_SELECT)) {
              BM_face_select_set_noflush(bm, f, false);
              break;
            }
          } while ((l_iter = l_iter->next) != f->l_first);
        }
      }
    }
    BM_select_history_validate(bm);
  }

  EDBM_mirror_active_side_restore(ob, em, side_co, false);

  if (!other_verts.is_empty()) {
    DEG_id_tag_update(&mesh->id, ID_RECALC_SELECT);
    WM_main_add_notifier(NC_GEOM | ND_SELECT, mesh);
  }
}

EditMeshMirrorUserSideScope::EditMeshMirrorUserSideScope(Object *ob, BMEditMesh *em)
    : ob_(ob), em_(em)
{
  side_co_ = EDBM_mirror_user_side_capture(ob, em, has_side_);
}

EditMeshMirrorUserSideScope::~EditMeshMirrorUserSideScope()
{
  if (has_side_) {
    EDBM_mirror_user_side_restore(ob_, em_, side_co_);
  }
}

/** \} */

}  // namespace blender
