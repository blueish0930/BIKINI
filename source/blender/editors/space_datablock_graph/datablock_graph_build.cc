/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include "BLI_index_range.hh"
#include "BLI_listbase.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_rect.hh"
#include "BLI_set.hh"
#include "BLI_string.hh"
#include "BLI_string_ref.hh"
#include "BLI_string_utf8.hh"

#include "BKE_lib_id.hh"
#include "BKE_lib_query.hh"
#include "BKE_main.hh"

#include "DNA_ID.h"
#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_enums.h"
#include "DNA_space_types.h"
#include "DNA_view2d_types.h"
#include "DNA_windowmanager_types.h"

#include "MEM_guardedalloc.h"

#include "datablock_graph_intern.hh"

namespace blender::ed::datablock_graph {

void datablock_graph_runtime_free(SpaceDataBlockGraph &sdbg)
{
  MEM_delete(sdbg.runtime);
  sdbg.runtime = nullptr;
}

void datablock_graph_ensure_runtime(SpaceDataBlockGraph &sdbg)
{
  if (sdbg.runtime == nullptr) {
    sdbg.runtime = MEM_new<SpaceDataBlockGraph_Runtime>(__func__);
  }
}

void datablock_graph_free_persistent_state(SpaceDataBlockGraph &sdbg)
{
  while (SpaceDataBlockGraphNodePos *pos = static_cast<SpaceDataBlockGraphNodePos *>(
             BLI_pophead(&sdbg.node_positions)))
  {
    MEM_delete(pos);
  }
  while (SpaceDataBlockGraphTreePath *path = static_cast<SpaceDataBlockGraphTreePath *>(
             BLI_pophead(&sdbg.browser_collapsed)))
  {
    MEM_delete(path);
  }
}

static ID *find_seed_id_for_uid(const SpaceDataBlockGraph &sdbg, const uint seed_uid)
{
  for (const SpaceDataBlockGraphSeed &seed : sdbg.seeds) {
    if (seed.id && seed.id->session_uid == seed_uid) {
      return seed.id;
    }
  }
  if (sdbg.runtime) {
    for (const GraphNode &node : sdbg.runtime->nodes) {
      if (node.is_seed && node.seed_uid == seed_uid) {
        return node.id;
      }
    }
  }
  return nullptr;
}

void datablock_graph_store_positions(SpaceDataBlockGraph &sdbg)
{
  if (sdbg.runtime == nullptr) {
    return;
  }
  while (SpaceDataBlockGraphNodePos *pos = static_cast<SpaceDataBlockGraphNodePos *>(
             BLI_pophead(&sdbg.node_positions)))
  {
    MEM_delete(pos);
  }
  for (const GraphNode &node : sdbg.runtime->nodes) {
    /* Virtual modifier nodes are not Main IDs — skip DNA persistence. */
    if (node.kind != GraphNodeKind::ID || node.id == nullptr) {
      continue;
    }
    ID *seed_id = find_seed_id_for_uid(sdbg, node.seed_uid);
    if (seed_id == nullptr) {
      continue;
    }
    SpaceDataBlockGraphNodePos *pos = MEM_new<SpaceDataBlockGraphNodePos>(__func__);
    pos->id = node.id;
    pos->seed_id = seed_id;
    pos->loc[0] = node.location.x;
    pos->loc[1] = node.location.y;
    if (node.position_locked) {
      pos->flag |= DATABLOCK_GRAPH_NODEPOS_LOCKED;
    }
    BLI_addtail(&sdbg.node_positions, pos);
  }
}

static GraphUndoSnapshot capture_undo_snapshot(SpaceDataBlockGraph &sdbg)
{
  /* Ensure DNA positions match runtime before snapshot. */
  datablock_graph_store_positions(sdbg);

  GraphUndoSnapshot snap;
  for (const SpaceDataBlockGraphSeed &seed : sdbg.seeds) {
    if (seed.id) {
      snap.seed_session_uids.append(seed.id->session_uid);
    }
  }
  for (const SpaceDataBlockGraphNodePos &pos : sdbg.node_positions) {
    if (pos.id == nullptr || pos.seed_id == nullptr) {
      continue;
    }
    GraphUndoSnapshot::Pos p;
    p.seed_session_uid = pos.seed_id->session_uid;
    p.id_session_uid = pos.id->session_uid;
    p.loc[0] = pos.loc[0];
    p.loc[1] = pos.loc[1];
    p.flag = pos.flag;
    snap.positions.append(p);
  }
  if (sdbg.runtime) {
    snap.active_key = sdbg.runtime->active_key;
    for (const GraphNode &node : sdbg.runtime->nodes) {
      if (node.selected) {
        snap.selected_keys.append(
            datablock_graph_node_key(node.seed_uid, node.id_session_uid));
      }
    }
  }
  return snap;
}

static void apply_undo_snapshot(SpaceDataBlockGraph &sdbg, Main &bmain, const GraphUndoSnapshot &snap)
{
  /* Clear seeds / positions DNA. */
  while (SpaceDataBlockGraphSeed *seed = static_cast<SpaceDataBlockGraphSeed *>(
             BLI_pophead(&sdbg.seeds)))
  {
    MEM_delete(seed);
  }
  while (SpaceDataBlockGraphNodePos *pos = static_cast<SpaceDataBlockGraphNodePos *>(
             BLI_pophead(&sdbg.node_positions)))
  {
    MEM_delete(pos);
  }

  for (const uint seed_uid : snap.seed_session_uids) {
    ID *id = BKE_libblock_find_session_uid(&bmain, seed_uid);
    if (id == nullptr) {
      continue;
    }
    SpaceDataBlockGraphSeed *seed = MEM_new<SpaceDataBlockGraphSeed>(__func__);
    seed->id = id;
    BLI_addtail(&sdbg.seeds, seed);
  }

  for (const GraphUndoSnapshot::Pos &p : snap.positions) {
    ID *id = BKE_libblock_find_session_uid(&bmain, p.id_session_uid);
    ID *seed_id = BKE_libblock_find_session_uid(&bmain, p.seed_session_uid);
    if (id == nullptr || seed_id == nullptr) {
      continue;
    }
    SpaceDataBlockGraphNodePos *pos = MEM_new<SpaceDataBlockGraphNodePos>(__func__);
    pos->id = id;
    pos->seed_id = seed_id;
    pos->loc[0] = p.loc[0];
    pos->loc[1] = p.loc[1];
    pos->flag = p.flag;
    BLI_addtail(&sdbg.node_positions, pos);
  }

  datablock_graph_ensure_runtime(sdbg);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;
  /*
   * Critical: clear live nodes before rebuild. rebuild_graph() re-applies saved_locations from
   * current runtime *after* DNA positions — that used to overwrite restored move/select state
   * (delete seemed to work because seed sets differed).
   * Keep undo/redo stacks intact.
   */
  runtime.nodes.clear();
  runtime.edges.clear();
  runtime.key_to_index.clear();
  runtime.graph_dirty = true;
  runtime.layout_dirty = true;
  runtime.active_key = 0;
  runtime.last_built_max_depth = -1;
  runtime.last_built_flag = 0;

  datablock_graph_rebuild_if_needed(sdbg, bmain);

  /* Restore selection from snapshot (runtime-only). */
  Set<uint64_t> sel;
  for (const uint64_t k : snap.selected_keys) {
    sel.add(k);
  }
  for (GraphNode &node : runtime.nodes) {
    const uint64_t k = datablock_graph_node_key(node.seed_uid, node.id_session_uid);
    node.selected = sel.contains(k);
  }
  runtime.active_key = snap.active_key;
  if (runtime.active_key != 0 && !runtime.key_to_index.contains(runtime.active_key)) {
    runtime.active_key = 0;
  }
}

void datablock_graph_undo_push(SpaceDataBlockGraph &sdbg)
{
  datablock_graph_ensure_runtime(sdbg);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;
  /* New edit branch invalidates redo. */
  runtime.redo_stack.clear();
  constexpr int max_steps = 64;
  runtime.undo_stack.append(capture_undo_snapshot(sdbg));
  while (int(runtime.undo_stack.size()) > max_steps) {
    runtime.undo_stack.remove(0);
  }
}

bool datablock_graph_undo_can_undo(const SpaceDataBlockGraph &sdbg)
{
  return sdbg.runtime && !sdbg.runtime->undo_stack.is_empty();
}

bool datablock_graph_undo_can_redo(const SpaceDataBlockGraph &sdbg)
{
  return sdbg.runtime && !sdbg.runtime->redo_stack.is_empty();
}

bool datablock_graph_undo_step(SpaceDataBlockGraph &sdbg, Main &bmain)
{
  if (!datablock_graph_undo_can_undo(sdbg)) {
    return false;
  }
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;
  /* Current state becomes redo. */
  runtime.redo_stack.append(capture_undo_snapshot(sdbg));
  GraphUndoSnapshot snap = runtime.undo_stack.pop_last();
  apply_undo_snapshot(sdbg, bmain, snap);
  return true;
}

bool datablock_graph_redo_step(SpaceDataBlockGraph &sdbg, Main &bmain)
{
  if (!datablock_graph_undo_can_redo(sdbg)) {
    return false;
  }
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;
  runtime.undo_stack.append(capture_undo_snapshot(sdbg));
  GraphUndoSnapshot snap = runtime.redo_stack.pop_last();
  apply_undo_snapshot(sdbg, bmain, snap);
  return true;
}

void datablock_graph_apply_positions(SpaceDataBlockGraph &sdbg)
{
  if (sdbg.runtime == nullptr || BLI_listbase_is_empty(&sdbg.node_positions)) {
    return;
  }
  for (GraphNode &node : sdbg.runtime->nodes) {
    if (node.id == nullptr) {
      continue;
    }
    ID *seed_id = find_seed_id_for_uid(sdbg, node.seed_uid);
    if (seed_id == nullptr) {
      continue;
    }
    for (const SpaceDataBlockGraphNodePos &pos : sdbg.node_positions) {
      if (pos.id == node.id && pos.seed_id == seed_id) {
        node.location = float2(pos.loc[0], pos.loc[1]);
        /* Always lock restored placements so auto-layout does not flatten Y again. */
        node.position_locked = true;
        break;
      }
    }
  }
}

bool datablock_graph_browser_path_collapsed(const SpaceDataBlockGraph &sdbg, const StringRef path)
{
  for (const SpaceDataBlockGraphTreePath &item : sdbg.browser_collapsed) {
    if (path == StringRef(item.path)) {
      return true;
    }
  }
  return false;
}

void datablock_graph_browser_path_set_collapsed(SpaceDataBlockGraph &sdbg,
                                               const StringRef path,
                                               const bool collapsed)
{
  SpaceDataBlockGraphTreePath *found = nullptr;
  for (SpaceDataBlockGraphTreePath &item : sdbg.browser_collapsed) {
    if (path == StringRef(item.path)) {
      found = &item;
      break;
    }
  }
  if (collapsed) {
    if (found == nullptr) {
      SpaceDataBlockGraphTreePath *item = MEM_new<SpaceDataBlockGraphTreePath>(__func__);
      BLI_strncpy_utf8(item->path, std::string(path).c_str(), sizeof(item->path));
      BLI_addtail(&sdbg.browser_collapsed, item);
    }
  }
  else if (found != nullptr) {
    BLI_remlink(&sdbg.browser_collapsed, found);
    MEM_delete(found);
  }
}

bool datablock_graph_has_seed(const SpaceDataBlockGraph &sdbg, const ID *id)
{
  if (id == nullptr) {
    return false;
  }
  for (const SpaceDataBlockGraphSeed &seed : sdbg.seeds) {
    if (seed.id == id) {
      return true;
    }
  }
  return false;
}

void datablock_graph_clear_seeds(SpaceDataBlockGraph &sdbg)
{
  while (SpaceDataBlockGraphSeed *seed = static_cast<SpaceDataBlockGraphSeed *>(
             BLI_pophead(&sdbg.seeds)))
  {
    MEM_delete(seed);
  }
  /* Drop saved layouts with the seeds; browser collapse stays. */
  while (SpaceDataBlockGraphNodePos *pos = static_cast<SpaceDataBlockGraphNodePos *>(
             BLI_pophead(&sdbg.node_positions)))
  {
    MEM_delete(pos);
  }
  datablock_graph_ensure_runtime(sdbg);
  sdbg.runtime->frame_view = true;
  datablock_graph_tag_rebuild(sdbg);
}

int datablock_graph_delete_selected(SpaceDataBlockGraph &sdbg)
{
  datablock_graph_ensure_runtime(sdbg);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;

  Set<uint> seed_uids_to_remove;
  for (const GraphNode &node : runtime.nodes) {
    if (node.selected) {
      seed_uids_to_remove.add(node.seed_uid);
    }
  }
  if (seed_uids_to_remove.is_empty()) {
    return 0;
  }

  Set<ID *> seed_ids_to_remove;
  for (const uint seed_uid : seed_uids_to_remove) {
    if (ID *seed_id = find_seed_id_for_uid(sdbg, seed_uid)) {
      seed_ids_to_remove.add(seed_id);
    }
  }

  int removed = 0;
  Vector<SpaceDataBlockGraphSeed *> seeds_to_unlink;
  for (SpaceDataBlockGraphSeed &seed : sdbg.seeds) {
    if (seed.id == nullptr || seed_ids_to_remove.contains(seed.id) ||
        seed_uids_to_remove.contains(seed.id->session_uid))
    {
      seeds_to_unlink.append(&seed);
    }
  }
  for (SpaceDataBlockGraphSeed *seed : seeds_to_unlink) {
    BLI_remlink(&sdbg.seeds, seed);
    MEM_delete(seed);
    removed++;
  }

  /* Drop saved positions for removed trees. */
  Vector<SpaceDataBlockGraphNodePos *> pos_to_unlink;
  for (SpaceDataBlockGraphNodePos &pos : sdbg.node_positions) {
    if (pos.seed_id == nullptr || seed_ids_to_remove.contains(pos.seed_id)) {
      pos_to_unlink.append(&pos);
    }
  }
  for (SpaceDataBlockGraphNodePos *pos : pos_to_unlink) {
    BLI_remlink(&sdbg.node_positions, pos);
    MEM_delete(pos);
  }

  runtime.active_key = 0;
  datablock_graph_tag_rebuild(sdbg);
  return removed;
}

void datablock_graph_deselect_all(SpaceDataBlockGraph &sdbg)
{
  datablock_graph_ensure_runtime(sdbg);
  for (GraphNode &node : sdbg.runtime->nodes) {
    node.selected = false;
  }
  sdbg.runtime->active_key = 0;
}

void datablock_graph_select_all(SpaceDataBlockGraph &sdbg)
{
  datablock_graph_ensure_runtime(sdbg);
  for (GraphNode &node : sdbg.runtime->nodes) {
    node.selected = true;
  }
}

void datablock_graph_select_seed_graph(SpaceDataBlockGraph &sdbg, const uint seed_uid)
{
  datablock_graph_ensure_runtime(sdbg);
  sdbg.runtime->active_key = 0;
  for (GraphNode &node : sdbg.runtime->nodes) {
    node.selected = (node.seed_uid == seed_uid);
    if (node.selected && node.is_seed) {
      sdbg.runtime->active_key = datablock_graph_node_key(node.seed_uid, node.id_session_uid);
    }
  }
}

void datablock_graph_select_connected_tree(SpaceDataBlockGraph &sdbg, GraphNode &start)
{
  datablock_graph_ensure_runtime(sdbg);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;

  /* Build undirected adjacency within the same seed only. */
  const int n = int(runtime.nodes.size());
  Vector<Vector<int>> adj(n);
  for (const GraphEdge &edge : runtime.edges) {
    if (edge.from_index < 0 || edge.to_index < 0 || edge.from_index >= n || edge.to_index >= n) {
      continue;
    }
    if (runtime.nodes[edge.from_index].seed_uid != start.seed_uid ||
        runtime.nodes[edge.to_index].seed_uid != start.seed_uid)
    {
      continue;
    }
    adj[edge.from_index].append(edge.to_index);
    adj[edge.to_index].append(edge.from_index);
  }

  int start_index = -1;
  for (const int i : runtime.nodes.index_range()) {
    if (&runtime.nodes[i] == &start) {
      start_index = i;
      break;
    }
  }
  if (start_index < 0) {
    return;
  }

  datablock_graph_deselect_all(sdbg);

  Vector<int> stack;
  Set<int> visited;
  stack.append(start_index);
  visited.add(start_index);
  while (!stack.is_empty()) {
    const int cur = stack.pop_last();
    runtime.nodes[cur].selected = true;
    for (const int next : adj[cur]) {
      if (visited.add(next)) {
        stack.append(next);
      }
    }
  }

  /* Fallback: isolated node or missing edges — at least the seed tree. */
  if (datablock_graph_count_selected(sdbg) <= 1) {
    datablock_graph_select_seed_graph(sdbg, start.seed_uid);
  }

  runtime.active_key = datablock_graph_node_key(start.seed_uid, start.id_session_uid);
}

int datablock_graph_count_selected(const SpaceDataBlockGraph &sdbg)
{
  if (sdbg.runtime == nullptr) {
    return 0;
  }
  int n = 0;
  for (const GraphNode &node : sdbg.runtime->nodes) {
    if (node.selected) {
      n++;
    }
  }
  return n;
}

void datablock_graph_foreach_selected(SpaceDataBlockGraph &sdbg, FunctionRef<void(GraphNode &)> fn)
{
  if (sdbg.runtime == nullptr) {
    return;
  }
  for (GraphNode &node : sdbg.runtime->nodes) {
    if (node.selected) {
      fn(node);
    }
  }
}

void datablock_graph_tag_rebuild(SpaceDataBlockGraph &sdbg)
{
  datablock_graph_ensure_runtime(sdbg);
  sdbg.runtime->graph_dirty = true;
  sdbg.runtime->layout_dirty = true;
}

void datablock_graph_force_refresh(SpaceDataBlockGraph &sdbg, Main &bmain)
{
  /* Drop cached Main relations so the next walk sees current ID pointers. */
  if (bmain.relations != nullptr) {
    BKE_main_relations_free(&bmain);
  }
  datablock_graph_ensure_runtime(sdbg);
  sdbg.runtime->graph_dirty = true;
  /* Keep layout_dirty false until rebuild decides; rebuild restores positions then may layout. */
  sdbg.runtime->last_built_max_depth = -1;
  sdbg.runtime->last_built_flag = 0;
  datablock_graph_rebuild_if_needed(sdbg, bmain);
}

rctf datablock_graph_node_rect(const GraphNode &node)
{
  rctf rect;
  rect.xmin = node.location.x;
  rect.xmax = node.location.x + node.size.x;
  rect.ymin = node.location.y - node.size.y * 0.5f;
  rect.ymax = node.location.y + node.size.y * 0.5f;
  return rect;
}

bool datablock_graph_nodes_bounds(const SpaceDataBlockGraph &sdbg, rctf &r_bounds)
{
  if (sdbg.runtime == nullptr || sdbg.runtime->nodes.is_empty()) {
    return false;
  }
  BLI_rctf_init_minmax(&r_bounds);
  for (const GraphNode &node : sdbg.runtime->nodes) {
    const rctf nr = datablock_graph_node_rect(node);
    const float minc[2] = {nr.xmin, nr.ymin};
    const float maxc[2] = {nr.xmax, nr.ymax};
    BLI_rctf_do_minmax_v(&r_bounds, minc);
    BLI_rctf_do_minmax_v(&r_bounds, maxc);
  }
  return true;
}

GraphNode *datablock_graph_find_node_mut(SpaceDataBlockGraph &sdbg,
                                         const uint seed_uid,
                                         const uint id_session_uid)
{
  if (sdbg.runtime == nullptr) {
    return nullptr;
  }
  const int *index = sdbg.runtime->key_to_index.lookup_ptr(
      datablock_graph_node_key(seed_uid, id_session_uid));
  if (index == nullptr) {
    return nullptr;
  }
  return &sdbg.runtime->nodes[*index];
}

const GraphNode *datablock_graph_find_node(const SpaceDataBlockGraph &sdbg,
                                           const uint seed_uid,
                                           const uint id_session_uid)
{
  return datablock_graph_find_node_mut(
      const_cast<SpaceDataBlockGraph &>(sdbg), seed_uid, id_session_uid);
}

GraphNode *datablock_graph_find_node_at(SpaceDataBlockGraph &sdbg, const float2 view_co)
{
  if (sdbg.runtime == nullptr) {
    return nullptr;
  }
  for (int i = int(sdbg.runtime->nodes.size()) - 1; i >= 0; i--) {
    GraphNode &node = sdbg.runtime->nodes[i];
    const rctf rect = datablock_graph_node_rect(node);
    if (BLI_rctf_isect_pt(&rect, view_co.x, view_co.y)) {
      return &node;
    }
  }
  return nullptr;
}

static bool is_noise_user_id(const ID &id)
{
  switch (GS(id.name)) {
    case ID_WM:
    case ID_SCR:
    case ID_WS:
    case ID_LI:
      return true;
    default:
      return false;
  }
}

/**
 * Whether this pointer-walk entry should contribute to edge "×N" usage counts.
 * Only fully refcounted (or USER_ONE) links match "used N times" in the UI sense.
 */
static bool flag_counts_as_use(const LibraryForeachIDCallbackFlag flag)
{
  return (flag & (IDWALK_CB_USER | IDWALK_CB_USER_ONE)) != 0;
}

static bool usage_flag_allowed(const SpaceDataBlockGraph &sdbg,
                               const LibraryForeachIDCallbackFlag flag)
{
  if (flag & IDWALK_CB_INTERNAL) {
    return false;
  }
  /*
   * bNodeTree runtime #eval_dependencies re-lists every dependency with
   * #IDWALK_CB_HASH_IGNORE. Those are not extra uses — they duplicate real
   * node/socket pointers and would turn every Object Info / Set Material into ×2.
   */
  if ((flag & IDWALK_CB_HASH_IGNORE) != 0 && !flag_counts_as_use(flag)) {
    return false;
  }
  /*
   * ViewLayer's first LayerCollection.collection is a non-owning pointer to the
   * scene master collection (#IDWALK_CB_EMBEDDED_NOT_OWNING). Scene already owns
   * it via #Scene.master_collection (#IDWALK_CB_EMBEDDED). Counting both → ×2.
   */
  if ((flag & IDWALK_CB_EMBEDDED_NOT_OWNING) != 0 && !flag_counts_as_use(flag) &&
      (flag & IDWALK_CB_EMBEDDED) == 0)
  {
    return false;
  }
  if ((sdbg.flag & DATABLOCK_GRAPH_FLAG_HIDE_LOOPBACK) && (flag & IDWALK_CB_LOOPBACK)) {
    return false;
  }
  if ((sdbg.flag & DATABLOCK_GRAPH_FLAG_HIDE_EMBEDDED) &&
      (flag & (IDWALK_CB_EMBEDDED | IDWALK_CB_EMBEDDED_NOT_OWNING)))
  {
    return false;
  }
  if ((sdbg.filter_flag & DATABLOCK_GRAPH_FILTER_USER_ONLY) &&
      (flag & (IDWALK_CB_USER | IDWALK_CB_USER_ONE)) == 0)
  {
    return false;
  }
  return true;
}

static int ensure_node_for_seed(SpaceDataBlockGraph_Runtime &runtime,
                                ID &id,
                                const uint seed_uid,
                                const short depth,
                                const bool is_seed)
{
  const uint id_uid = id.session_uid;
  const uint64_t key = datablock_graph_node_key(seed_uid, id_uid);
  if (const int *existing = runtime.key_to_index.lookup_ptr(key)) {
    GraphNode &node = runtime.nodes[*existing];
    node.id = &id;
    if (is_seed) {
      node.is_seed = true;
      node.depth_from_seed = 0;
    }
    else if (!node.is_seed && depth < node.depth_from_seed) {
      node.depth_from_seed = depth;
    }
    return *existing;
  }

  GraphNode node;
  node.id = &id;
  node.id_session_uid = id_uid;
  node.seed_uid = seed_uid;
  node.is_seed = is_seed;
  node.depth_from_seed = depth;
  const int index = runtime.nodes.append_and_get_index(node);
  runtime.key_to_index.add(key, index);
  return index;
}

static void add_edge_indices(SpaceDataBlockGraph_Runtime &runtime,
                             const int from_index,
                             const int to_index,
                             const uint seed_uid,
                             const LibraryForeachIDCallbackFlag flags,
                             const bool is_cycle)
{
  const bool counts = flag_counts_as_use(flags);
  for (GraphEdge &edge : runtime.edges) {
    if (edge.seed_uid == seed_uid && edge.from_index == from_index && edge.to_index == to_index) {
      edge.usage_flags = LibraryForeachIDCallbackFlag(edge.usage_flags | flags);
      edge.is_cycle = edge.is_cycle || is_cycle;
      /* Only USER / USER_ONE links increment ×N (real multi-use, e.g. 3 Group nodes). */
      if (counts) {
        edge.usage_count = std::max(1, edge.usage_count + 1);
      }
      return;
    }
  }
  GraphEdge edge;
  edge.from_index = from_index;
  edge.to_index = to_index;
  edge.seed_uid = seed_uid;
  edge.usage_flags = flags;
  edge.is_cycle = is_cycle;
  /* Structural-only links (embedded master collection, etc.) still show as ×1. */
  edge.usage_count = 1;
  runtime.edges.append(edge);
}

/**
 * Synthetic session_uid for virtual (non-ID) nodes. High bit set so early sequential
 * Main session_uids do not collide under normal file sizes.
 */
static uint virtual_modifier_session_uid(const uint object_uid,
                                         const char *mod_name,
                                         const int mod_stack_index)
{
  uint h = object_uid ^ 0xA5C39E37u;
  if (mod_name) {
    for (const char *p = mod_name; *p; p++) {
      h = (h * 16777619u) ^ uint(unsigned char(*p));
    }
  }
  h ^= uint(mod_stack_index) * 0x9E3779B9u;
  if (h == 0) {
    h = 1;
  }
  return h | 0x80000000u;
}

/**
 * Relations walk Object → node_group as a direct ID user link, but the real path is
 * Object → Nodes Modifier → NodeTree. Split those edges so the graph matches usage.
 *
 * Important: never keep #GraphNode references across #runtime.nodes appends (Vector
 * reallocation would dangle and can corrupt heap → free(nullptr) crash).
 */
static void insert_modifier_bridges_for_seed(SpaceDataBlockGraph_Runtime &runtime,
                                             const uint seed_uid)
{
  struct SplitEdge {
    int from_index = -1;
    int to_index = -1;
    LibraryForeachIDCallbackFlag flags = IDWALK_CB_NOP;
    bool is_cycle = false;
    int usage_count = 1;
  };
  Vector<SplitEdge> to_split;
  Vector<GraphEdge> kept;
  kept.reserve(runtime.edges.size());

  const int node_count = int(runtime.nodes.size());
  for (const GraphEdge &edge : runtime.edges) {
    if (edge.seed_uid != seed_uid) {
      kept.append(edge);
      continue;
    }
    if (edge.from_index < 0 || edge.to_index < 0 || edge.from_index >= node_count ||
        edge.to_index >= node_count)
    {
      kept.append(edge);
      continue;
    }
    const GraphNode &from = runtime.nodes[edge.from_index];
    const GraphNode &to = runtime.nodes[edge.to_index];
    if (from.kind != GraphNodeKind::ID || to.kind != GraphNodeKind::ID || from.id == nullptr ||
        to.id == nullptr || GS(from.id->name) != ID_OB || GS(to.id->name) != ID_NT)
    {
      kept.append(edge);
      continue;
    }

    Object *ob = reinterpret_cast<Object *>(from.id);
    bNodeTree *ntree = reinterpret_cast<bNodeTree *>(to.id);
    if (ob == nullptr || ntree == nullptr) {
      kept.append(edge);
      continue;
    }

    bool has_nodes_mod = false;
    for (ModifierData &md : ob->modifiers) {
      if (md.type != eModifierType_Nodes) {
        continue;
      }
      const NodesModifierData &nmd = reinterpret_cast<const NodesModifierData &>(md);
      if (nmd.node_group == ntree) {
        has_nodes_mod = true;
        break;
      }
    }

    if (!has_nodes_mod) {
      kept.append(edge);
      continue;
    }

    SplitEdge se;
    se.from_index = edge.from_index;
    se.to_index = edge.to_index;
    se.flags = edge.usage_flags;
    se.is_cycle = edge.is_cycle;
    se.usage_count = edge.usage_count;
    to_split.append(se);
  }

  if (to_split.is_empty()) {
    return;
  }

  runtime.edges = std::move(kept);

  for (const SplitEdge &se : to_split) {
    /* Re-read by index each time — nodes vector may grow below. */
    if (se.from_index < 0 || se.to_index < 0 || se.from_index >= int(runtime.nodes.size()) ||
        se.to_index >= int(runtime.nodes.size()))
    {
      continue;
    }
    ID *from_id = runtime.nodes[se.from_index].id;
    ID *to_id = runtime.nodes[se.to_index].id;
    if (from_id == nullptr || to_id == nullptr || GS(from_id->name) != ID_OB ||
        GS(to_id->name) != ID_NT)
    {
      /* Fall back: keep a direct edge if topology changed unexpectedly. */
      add_edge_indices(runtime, se.from_index, se.to_index, seed_uid, se.flags, se.is_cycle);
      continue;
    }

    Object *ob = reinterpret_cast<Object *>(from_id);
    bNodeTree *ntree = reinterpret_cast<bNodeTree *>(to_id);
    const short ntree_depth = runtime.nodes[se.to_index].depth_from_seed;
    const short mod_depth = short(std::max(0, int(ntree_depth) + 1));

    int stack_i = 0;
    int matched = 0;
    for (ModifierData &md : ob->modifiers) {
      if (md.type == eModifierType_Nodes) {
        const NodesModifierData &nmd = reinterpret_cast<const NodesModifierData &>(md);
        if (nmd.node_group == ntree) {
          /* Snapshot fields before nodes vector may reallocate. */
          const short md_type = short(md.type);
          char md_name[64];
          BLI_strncpy(md_name, md.name[0] ? md.name : "GeometryNodes", sizeof(md_name));
          const int mod_stack_index = stack_i;

          const uint virt_uid = virtual_modifier_session_uid(
              ob->id.session_uid, md_name, mod_stack_index);
          const uint64_t vkey = datablock_graph_node_key(seed_uid, virt_uid);
          int mod_index = -1;
          if (const int *existing = runtime.key_to_index.lookup_ptr(vkey)) {
            mod_index = *existing;
          }
          else {
            GraphNode vnode;
            vnode.id = nullptr;
            vnode.host_id = &ob->id;
            vnode.id_session_uid = virt_uid;
            vnode.seed_uid = seed_uid;
            vnode.is_seed = false;
            vnode.depth_from_seed = mod_depth;
            vnode.kind = GraphNodeKind::VirtualModifier;
            vnode.modifier_type = md_type;
            BLI_strncpy(vnode.display_name, md_name, sizeof(vnode.display_name));
            mod_index = runtime.nodes.append_and_get_index(vnode);
            runtime.key_to_index.add_overwrite(vkey, mod_index);
          }

          add_edge_indices(runtime, se.from_index, mod_index, seed_uid, se.flags, se.is_cycle);
          add_edge_indices(runtime, mod_index, se.to_index, seed_uid, se.flags, se.is_cycle);
          matched++;
          if (se.usage_count > 1) {
            for (GraphEdge &e : runtime.edges) {
              if (e.seed_uid == seed_uid && e.from_index == mod_index &&
                  e.to_index == se.to_index)
              {
                e.usage_count = se.usage_count;
                break;
              }
            }
          }
        }
      }
      stack_i++;
    }

    if (matched == 0) {
      add_edge_indices(runtime, se.from_index, se.to_index, seed_uid, se.flags, se.is_cycle);
    }
  }
}

/**
 * Synthetic session_uid for virtual UI-space users (Image Editor, Node Editor, …).
 * High bits 0b11… distinguish from modifier virtuals (0b10…).
 */
static uint virtual_ui_session_uid(const short space_type, const uint used_id_uid)
{
  uint h = used_id_uid ^ (uint(space_type) * 0x9E3779B9u) ^ 0xE01A70A5u;
  if (h == 0) {
    h = 1;
  }
  return h | 0xC0000000u;
}

static const char *ui_space_display_name(const short space_type)
{
  switch (space_type) {
    case SPACE_IMAGE:
      return "Image Editor";
    case SPACE_NODE:
      return "Node Editor";
    case SPACE_TEXT:
      return "Text Editor";
    case SPACE_CLIP:
      return "Movie Clip Editor";
    default:
      return "Editor";
  }
}

/**
 * Screens / editor spaces hold USER_ONE refs (e.g. Image Editor → Image) that bump
 * id->us but are invisible in Main relations unless IDWALK_INCLUDE_UI — and Screens
 * are filtered as noise. Inject virtual editor nodes so the graph matches the count.
 */
static void inject_ui_space_users_for_seed(SpaceDataBlockGraph_Runtime &runtime,
                                           Main &bmain,
                                           const uint seed_uid)
{
  /* Map data-block ID* → node index within this seed (real ID nodes only). */
  Map<const ID *, int> id_to_index;
  for (const int i : runtime.nodes.index_range()) {
    const GraphNode &node = runtime.nodes[i];
    if (node.seed_uid != seed_uid || node.kind != GraphNodeKind::ID || node.id == nullptr) {
      continue;
    }
    id_to_index.add_overwrite(node.id, i);
  }
  if (id_to_index.is_empty()) {
    return;
  }

  auto note_ui_user = [&](const short space_type, ID *used_id, ID *screen_id) {
    if (used_id == nullptr) {
      return;
    }
    const int *used_index = id_to_index.lookup_ptr(used_id);
    if (used_index == nullptr) {
      return;
    }

    const uint virt_uid = virtual_ui_session_uid(space_type, used_id->session_uid);
    const uint64_t vkey = datablock_graph_node_key(seed_uid, virt_uid);
    int ui_index = -1;
    if (const int *existing = runtime.key_to_index.lookup_ptr(vkey)) {
      ui_index = *existing;
    }
    else {
      const short used_depth = runtime.nodes[*used_index].depth_from_seed;
      GraphNode vnode;
      vnode.id = nullptr;
      vnode.host_id = screen_id;
      vnode.id_session_uid = virt_uid;
      vnode.seed_uid = seed_uid;
      vnode.is_seed = false;
      vnode.depth_from_seed = short(std::max(0, int(used_depth) + 1));
      vnode.kind = GraphNodeKind::VirtualUISpace;
      vnode.space_type = space_type;
      BLI_strncpy(vnode.display_name, ui_space_display_name(space_type), sizeof(vnode.display_name));
      ui_index = runtime.nodes.append_and_get_index(vnode);
      runtime.key_to_index.add_overwrite(vkey, ui_index);
    }

    /* Editor (user) → data-block (used). USER_ONE matches Image Editor id_us. */
    add_edge_indices(runtime,
                     ui_index,
                     *used_index,
                     seed_uid,
                     LibraryForeachIDCallbackFlag(IDWALK_CB_USER_ONE),
                     false);
  };

  for (bScreen &screen : bmain.screens) {
    for (ScrArea &area : screen.areabase) {
      for (SpaceLink &sl : area.spacedata) {
        switch (sl.spacetype) {
          case SPACE_IMAGE: {
            const SpaceImage &sima = reinterpret_cast<const SpaceImage &>(sl);
            note_ui_user(SPACE_IMAGE, reinterpret_cast<ID *>(sima.image), &screen.id);
            break;
          }
          case SPACE_NODE: {
            const SpaceNode &snode = reinterpret_cast<const SpaceNode &>(sl);
            note_ui_user(SPACE_NODE, reinterpret_cast<ID *>(snode.nodetree), &screen.id);
            if (snode.edittree != snode.nodetree) {
              note_ui_user(SPACE_NODE, reinterpret_cast<ID *>(snode.edittree), &screen.id);
            }
            if (snode.selected_node_group != snode.nodetree &&
                snode.selected_node_group != snode.edittree)
            {
              note_ui_user(
                  SPACE_NODE, reinterpret_cast<ID *>(snode.selected_node_group), &screen.id);
            }
            break;
          }
          case SPACE_TEXT: {
            const SpaceText &st = reinterpret_cast<const SpaceText &>(sl);
            note_ui_user(SPACE_TEXT, reinterpret_cast<ID *>(st.text), &screen.id);
            break;
          }
          case SPACE_CLIP: {
            const SpaceClip &sclip = reinterpret_cast<const SpaceClip &>(sl);
            note_ui_user(SPACE_CLIP, reinterpret_cast<ID *>(sclip.clip), &screen.id);
            break;
          }
          default:
            break;
        }
      }
    }
  }
}

static bool path_exists_without_edge(const Map<int, Vector<int>> &adj,
                                     const int from_index,
                                     const int to_index)
{
  Set<int> visited;
  Vector<int> queue;
  if (const Vector<int> *outs = adj.lookup_ptr(from_index)) {
    for (const int next : *outs) {
      if (next == to_index) {
        continue;
      }
      if (visited.add(next)) {
        queue.append(next);
      }
    }
  }
  while (!queue.is_empty()) {
    const int cur = queue.pop_last();
    if (cur == to_index) {
      return true;
    }
    if (const Vector<int> *outs = adj.lookup_ptr(cur)) {
      for (const int next : *outs) {
        if (visited.add(next)) {
          queue.append(next);
        }
      }
    }
  }
  return false;
}

/** Transitive reduction for edges of a single seed only. */
static void remove_transitive_shortcut_edges_for_seed(SpaceDataBlockGraph_Runtime &runtime,
                                                      const uint seed_uid)
{
  for (int pass = 0; pass < 8; pass++) {
    Map<int, Vector<int>> adj;
    for (const GraphEdge &edge : runtime.edges) {
      if (edge.seed_uid != seed_uid || edge.is_cycle) {
        continue;
      }
      Vector<int> &outs = adj.lookup_or_add_default(edge.from_index);
      if (!outs.contains(edge.to_index)) {
        outs.append(edge.to_index);
      }
    }

    Vector<GraphEdge> kept;
    kept.reserve(runtime.edges.size());
    bool removed_any = false;
    for (const GraphEdge &edge : runtime.edges) {
      if (edge.seed_uid != seed_uid) {
        kept.append(edge);
        continue;
      }
      if (edge.is_cycle) {
        kept.append(edge);
        continue;
      }
      if (path_exists_without_edge(adj, edge.from_index, edge.to_index)) {
        removed_any = true;
        continue;
      }
      kept.append(edge);
    }
    runtime.edges = std::move(kept);
    if (!removed_any) {
      break;
    }
  }
}

/**
 * Expand users for one seed into its own independent node instances.
 * \return indices of nodes that belong to this seed after expansion.
 */
static void expand_users_dfs_for_seed(SpaceDataBlockGraph &sdbg,
                                      MainIDRelations &relations,
                                      SpaceDataBlockGraph_Runtime &runtime,
                                      ID &id,
                                      const uint seed_uid,
                                      const short depth,
                                      Set<uint> &path,
                                      Set<uint> &expanded)
{
  const bool recursive = (sdbg.flag & DATABLOCK_GRAPH_FLAG_RECURSIVE) != 0;
  const short max_depth = recursive ? short(256) : short(std::max<int>(0, sdbg.max_depth));
  if (depth >= max_depth) {
    return;
  }

  MainIDRelationsEntry *entry = relations.relations_from_pointers->lookup_default(&id, nullptr);
  if (entry == nullptr) {
    return;
  }

  const int id_index = ensure_node_for_seed(runtime, id, seed_uid, depth, false);

  for (MainIDRelationsEntryItem *from_item = entry->from_ids; from_item;
       from_item = from_item->next)
  {
    ID *user = from_item->id_pointer.from;
    if (user == nullptr) {
      continue;
    }
    if (is_noise_user_id(*user)) {
      continue;
    }
    if (!usage_flag_allowed(sdbg, from_item->usage_flag)) {
      continue;
    }

    const uint user_uid = user->session_uid;
    if (path.contains(user_uid)) {
      const int user_index = ensure_node_for_seed(
          runtime, *user, seed_uid, short(depth + 1), false);
      add_edge_indices(
          runtime, user_index, id_index, seed_uid, from_item->usage_flag, true);
      continue;
    }

    const int user_index = ensure_node_for_seed(runtime, *user, seed_uid, short(depth + 1), false);
    add_edge_indices(
        runtime, user_index, id_index, seed_uid, from_item->usage_flag, false);

    if (expanded.contains(user_uid)) {
      continue;
    }

    path.add(user_uid);
    expand_users_dfs_for_seed(
        sdbg, relations, runtime, *user, seed_uid, short(depth + 1), path, expanded);
    path.remove(user_uid);
    expanded.add(user_uid);
  }
}

/**
 * After transitive reduction, rediscover layer = longest path distance from the seed
 * along remaining edges (user → used). Discovery-depth alone stacks GN and TestGeo
 * in one column when both are direct users of Rubbertoy; longest-path puts
 * GN → TestGeo → Rubbertoy on a horizontal chain.
 */
static void recompute_depths_longest_path(SpaceDataBlockGraph_Runtime &runtime,
                                          const uint seed_uid)
{
  int seed_index = -1;
  for (const int i : runtime.nodes.index_range()) {
    GraphNode &node = runtime.nodes[i];
    if (node.seed_uid != seed_uid) {
      continue;
    }
    if (node.is_seed) {
      seed_index = i;
      node.depth_from_seed = 0;
    }
    else {
      /* Unset; recompute from edges. */
      node.depth_from_seed = -1;
    }
  }
  if (seed_index < 0) {
    return;
  }

  /* Relax: for edge from_user → to_used, depth[from] = max(depth[from], depth[to] + 1). */
  const int max_iters = int(runtime.nodes.size()) + 2;
  for (int iter = 0; iter < max_iters; iter++) {
    bool changed = false;
    for (const GraphEdge &edge : runtime.edges) {
      if (edge.seed_uid != seed_uid || edge.is_cycle) {
        continue;
      }
      if (edge.from_index < 0 || edge.to_index < 0) {
        continue;
      }
      GraphNode &from = runtime.nodes[edge.from_index];
      GraphNode &to = runtime.nodes[edge.to_index];
      if (from.seed_uid != seed_uid || to.seed_uid != seed_uid) {
        continue;
      }
      if (to.depth_from_seed < 0) {
        continue;
      }
      const short nd = short(to.depth_from_seed + 1);
      if (from.depth_from_seed < 0 || from.depth_from_seed < nd) {
        from.depth_from_seed = nd;
        changed = true;
      }
    }
    if (!changed) {
      break;
    }
  }

  /* Orphans (no path after reduction): park at layer 1. */
  for (GraphNode &node : runtime.nodes) {
    if (node.seed_uid == seed_uid && node.depth_from_seed < 0) {
      node.depth_from_seed = 1;
    }
  }
}

constexpr float LAYOUT_NODE_W = 180.0f;
/* Tall enough for name + type/users line without glyph overlap. */
constexpr float LAYOUT_NODE_H = 66.0f;
constexpr float LAYOUT_GAP_X = 100.0f;
constexpr float LAYOUT_GAP_Y = 36.0f;

/** Layout one seed's nodes at \a origin (seed is rightmost, users flow left). */
static void layout_seed_graph(SpaceDataBlockGraph_Runtime &runtime,
                              const uint seed_uid,
                              const float2 origin,
                              const bool force_all)
{
  recompute_depths_longest_path(runtime, seed_uid);

  short max_depth = 0;
  Vector<int> seed_node_indices;
  for (const int i : runtime.nodes.index_range()) {
    if (runtime.nodes[i].seed_uid != seed_uid) {
      continue;
    }
    seed_node_indices.append(i);
    max_depth = std::max(max_depth, short(std::max(0, int(runtime.nodes[i].depth_from_seed))));
  }
  if (seed_node_indices.is_empty()) {
    return;
  }

  Vector<Vector<int>> layers(max_depth + 1);
  for (const int i : seed_node_indices) {
    short d = runtime.nodes[i].depth_from_seed;
    if (d < 0) {
      d = 0;
    }
    layers[d].append(i);
  }

  /*
   * Order within a column by relation structure, NOT by data-block name.
   * Name sort made rename (.001 → .007) reshuffle Y and cross wires.
   *
   * 1) Stable discovery order = runtime node index (from_ids / node-tree walk order).
   * 2) Barycenter sweeps along edges to reduce crossings (Sugiyama-style).
   *    Edge direction: from_user → to_used; users sit at higher depth (left).
   */
  for (Vector<int> &layer : layers) {
    std::sort(layer.begin(), layer.end(), [](const int a, const int b) { return a < b; });
  }

  auto order_index_in_layer = [](const Vector<int> &layer, const int node_index) -> int {
    for (const int i : layer.index_range()) {
      if (layer[i] == node_index) {
        return i;
      }
    }
    return -1;
  };

  auto barycenter = [&](const int node_index, const Vector<int> &other_layer) -> float {
    float sum = 0.0f;
    int count = 0;
    for (const GraphEdge &edge : runtime.edges) {
      if (edge.seed_uid != seed_uid || edge.is_cycle) {
        continue;
      }
      int other = -1;
      if (edge.from_index == node_index) {
        other = edge.to_index;
      }
      else if (edge.to_index == node_index) {
        other = edge.from_index;
      }
      if (other < 0) {
        continue;
      }
      const int oi = order_index_in_layer(other_layer, other);
      if (oi >= 0) {
        sum += float(oi);
        count++;
      }
    }
    if (count == 0) {
      /* No edge to the fixed layer: keep discovery order (node index). */
      return float(node_index) * 1000.0f;
    }
    return sum / float(count);
  };

  auto sort_layer_by_bary = [&](Vector<int> &layer, const Vector<int> &fixed_layer) {
    if (layer.size() <= 1) {
      return;
    }
    Vector<std::pair<float, int>> keyed;
    keyed.reserve(layer.size());
    for (const int idx : layer) {
      keyed.append({barycenter(idx, fixed_layer), idx});
    }
    std::sort(keyed.begin(), keyed.end(), [](const auto &a, const auto &b) {
      if (a.first != b.first) {
        return a.first < b.first;
      }
      /* Tie-break: discovery order, never name. */
      return a.second < b.second;
    });
    for (const int i : layer.index_range()) {
      layer[i] = keyed[i].second;
    }
  };

  /* Alternate sweeps: toward users (depth↑) then back toward seed (depth↓). */
  for (int pass = 0; pass < 4; pass++) {
    for (int d = 1; d <= max_depth; d++) {
      sort_layer_by_bary(layers[d], layers[d - 1]);
    }
    for (int d = max_depth - 1; d >= 0; d--) {
      if (d + 1 <= max_depth) {
        sort_layer_by_bary(layers[d], layers[d + 1]);
      }
    }
  }

  for (const short depth : IndexRange(max_depth + 1)) {
    Vector<int> &layer = layers[depth];

    const float x = origin.x - float(depth) * (LAYOUT_NODE_W + LAYOUT_GAP_X);
    const int count = int(layer.size());
    /* Center the column on origin.y; step Y so no two siblings share a row. */
    const float total_h = float(count) * LAYOUT_NODE_H +
                          float(std::max(0, count - 1)) * LAYOUT_GAP_Y;
    float y = origin.y + total_h * 0.5f - LAYOUT_NODE_H * 0.5f;

    for (const int index : layer) {
      GraphNode &node = runtime.nodes[index];
      node.size = float2(LAYOUT_NODE_W, LAYOUT_NODE_H);
      if (force_all || !node.position_locked) {
        node.location = float2(x, y);
        node.position_locked = false;
      }
      y -= LAYOUT_NODE_H + LAYOUT_GAP_Y;
    }
  }

  /* force_all: resolve any remaining overlaps within this seed (defensive). */
  if (force_all) {
    for (int pass = 0; pass < 8; pass++) {
      bool moved = false;
      for (const int i : seed_node_indices) {
        for (const int j : seed_node_indices) {
          if (j <= i) {
            continue;
          }
          GraphNode &a = runtime.nodes[i];
          GraphNode &b = runtime.nodes[j];
          const rctf ra = datablock_graph_node_rect(a);
          const rctf rb = datablock_graph_node_rect(b);
          if (!BLI_rctf_isect(&ra, &rb, nullptr)) {
            continue;
          }
          /* Push the lower one further down. */
          const float overlap = std::min(ra.ymax, rb.ymax) - std::max(ra.ymin, rb.ymin);
          const float push = overlap + LAYOUT_GAP_Y;
          if (a.location.y >= b.location.y) {
            b.location.y -= push;
          }
          else {
            a.location.y -= push;
          }
          moved = true;
        }
      }
      if (!moved) {
        break;
      }
    }
  }
}

static bool accumulate_nodes_bounds(const SpaceDataBlockGraph_Runtime &runtime,
                                    const int index_begin,
                                    const int index_end,
                                    rctf &r_bounds)
{
  bool any = false;
  BLI_rctf_init_minmax(&r_bounds);
  for (int i = index_begin; i < index_end; i++) {
    const rctf nr = datablock_graph_node_rect(runtime.nodes[i]);
    const float minc[2] = {nr.xmin, nr.ymin};
    const float maxc[2] = {nr.xmax, nr.ymax};
    BLI_rctf_do_minmax_v(&r_bounds, minc);
    BLI_rctf_do_minmax_v(&r_bounds, maxc);
    any = true;
  }
  return any;
}

/**
 * Place newly created seed nodes so that:
 * - horizontal center of new AABB matches horizontal center of previous AABB
 * - top of new AABB sits on the bottom of previous AABB (new tree below old, with gap)
 * If there was no previous graph, leave layout at \a layout_origin (drop or 0).
 */
static void place_new_seed_against_existing(SpaceDataBlockGraph_Runtime &runtime,
                                            const uint seed_uid,
                                            const int nodes_before,
                                            const float2 layout_origin)
{
  constexpr float gap = 48.0f;

  rctf new_bb;
  if (!accumulate_nodes_bounds(runtime, nodes_before, int(runtime.nodes.size()), new_bb)) {
    return;
  }

  rctf old_bb;
  const bool has_old = (nodes_before > 0) &&
                       accumulate_nodes_bounds(runtime, 0, nodes_before, old_bb);

  float2 delta(0.0f);
  if (has_old) {
    const float old_cx = BLI_rctf_cent_x(&old_bb);
    const float new_cx = BLI_rctf_cent_x(&new_bb);
    /* Center-align horizontally. */
    delta.x = old_cx - new_cx;
    /* Top of new = bottom of old (minus gap): new_ymax + dy = old_ymin - gap. */
    delta.y = (old_bb.ymin - gap) - new_bb.ymax;
  }
  else {
    /* Empty canvas: shift so seed-side layout origin sits at layout_origin. */
    delta.x = layout_origin.x - new_bb.xmax; /* seed is rightmost after layout */
    delta.y = layout_origin.y - BLI_rctf_cent_y(&new_bb);
  }

  if (math::abs(delta.x) < 1e-6f && math::abs(delta.y) < 1e-6f) {
    return;
  }
  for (int i = nodes_before; i < int(runtime.nodes.size()); i++) {
    GraphNode &node = runtime.nodes[i];
    if (node.seed_uid != seed_uid) {
      continue;
    }
    node.location += delta;
    node.position_locked = true;
  }
}

static void expand_one_seed(SpaceDataBlockGraph &sdbg,
                            Main &bmain,
                            MainIDRelations &relations,
                            SpaceDataBlockGraph_Runtime &runtime,
                            ID &seed_id)
{
  const uint seed_uid = seed_id.session_uid;
  ensure_node_for_seed(runtime, seed_id, seed_uid, 0, true);
  Set<uint> path;
  Set<uint> expanded;
  path.add(seed_uid);
  expand_users_dfs_for_seed(sdbg, relations, runtime, seed_id, seed_uid, 0, path, expanded);
  expanded.add(seed_uid);
  /* Object → node_group is walked as a direct ID link; insert Modifier in between. */
  insert_modifier_bridges_for_seed(runtime, seed_uid);
  /* Image Editor / Node Editor USER_ONE refs are not in default Main relations. */
  inject_ui_space_users_for_seed(runtime, bmain, seed_uid);
  remove_transitive_shortcut_edges_for_seed(runtime, seed_uid);
  recompute_depths_longest_path(runtime, seed_uid);
}

static void rebuild_graph(SpaceDataBlockGraph &sdbg, Main &bmain)
{
  datablock_graph_ensure_runtime(sdbg);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;

  Map<uint64_t, float2> saved_locations;
  Map<uint64_t, bool> saved_locked;
  Set<uint64_t> saved_selected;
  for (const GraphNode &node : runtime.nodes) {
    const uint64_t key = datablock_graph_node_key(node.seed_uid, node.id_session_uid);
    saved_locations.add(key, node.location);
    saved_locked.add(key, node.position_locked);
    if (node.selected) {
      saved_selected.add(key);
    }
  }
  const uint64_t prev_active = runtime.active_key;

  runtime.nodes.clear();
  runtime.edges.clear();
  runtime.key_to_index.clear();
  runtime.active_key = prev_active;

  Vector<SpaceDataBlockGraphSeed *> seeds_to_remove;
  for (SpaceDataBlockGraphSeed &seed : sdbg.seeds) {
    if (seed.id == nullptr) {
      seeds_to_remove.append(&seed);
    }
  }
  for (SpaceDataBlockGraphSeed *seed : seeds_to_remove) {
    BLI_remlink(&sdbg.seeds, seed);
    MEM_delete(seed);
  }

  if (BLI_listbase_is_empty(&sdbg.seeds)) {
    runtime.graph_dirty = false;
    runtime.layout_dirty = true;
    return;
  }

  BKE_main_relations_create(&bmain, 0);
  MainIDRelations *relations = bmain.relations;
  BLI_assert(relations != nullptr);

  for (SpaceDataBlockGraphSeed &seed : sdbg.seeds) {
    expand_one_seed(sdbg, bmain, *relations, runtime, *seed.id);
  }

  BKE_main_relations_free(&bmain);

  /* DNA positions first (survive file load); session map overrides when fresher. */
  datablock_graph_apply_positions(sdbg);
  for (GraphNode &node : runtime.nodes) {
    const uint64_t key = datablock_graph_node_key(node.seed_uid, node.id_session_uid);
    if (const float2 *loc = saved_locations.lookup_ptr(key)) {
      node.location = *loc;
      node.position_locked = saved_locked.lookup_default(key, false);
    }
    node.selected = saved_selected.contains(key);
  }
  if (runtime.active_key != 0 && !runtime.key_to_index.contains(runtime.active_key)) {
    runtime.active_key = 0;
  }

  runtime.graph_dirty = false;
  /* Only re-layout nodes that still have no locked placement. */
  runtime.layout_dirty = true;
}

void datablock_graph_layout(SpaceDataBlockGraph &sdbg, const bool force_all)
{
  datablock_graph_ensure_runtime(sdbg);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;
  if (runtime.nodes.is_empty()) {
    runtime.layout_dirty = false;
    return;
  }

  Vector<uint> seed_uids;
  for (const GraphNode &node : runtime.nodes) {
    if (node.is_seed && !seed_uids.contains(node.seed_uid)) {
      seed_uids.append(node.seed_uid);
    }
  }

  if (force_all) {
    /* Unlock everything and stack each seed-tree top-to-bottom with clear Y separation. */
    for (GraphNode &node : runtime.nodes) {
      node.position_locked = false;
    }

    float cursor_y = 0.0f;
    bool first = true;
    for (const uint seed_uid : seed_uids) {
      /* Layout around a temporary origin, then measure and shift. */
      layout_seed_graph(runtime, seed_uid, float2(0.0f, 0.0f), true);

      rctf bb;
      BLI_rctf_init_minmax(&bb);
      bool any = false;
      for (GraphNode &node : runtime.nodes) {
        if (node.seed_uid != seed_uid) {
          continue;
        }
        const rctf nr = datablock_graph_node_rect(node);
        const float minc[2] = {nr.xmin, nr.ymin};
        const float maxc[2] = {nr.xmax, nr.ymax};
        BLI_rctf_do_minmax_v(&bb, minc);
        BLI_rctf_do_minmax_v(&bb, maxc);
        any = true;
      }
      if (!any) {
        continue;
      }

      float dy;
      if (first) {
        /* First tree: center at y=0. */
        dy = -BLI_rctf_cent_y(&bb);
        first = false;
      }
      else {
        /* Subsequent trees: top of new = bottom of previous stack - gap. */
        dy = (cursor_y - LAYOUT_GAP_Y) - bb.ymax;
      }
      const float dx = -BLI_rctf_cent_x(&bb); /* center on x=0 */
      for (GraphNode &node : runtime.nodes) {
        if (node.seed_uid != seed_uid) {
          continue;
        }
        node.location.x += dx;
        node.location.y += dy;
        node.position_locked = false;
      }
      cursor_y = bb.ymin + dy; /* bottom of this tree after shift */
    }
  }
  else {
    for (const uint seed_uid : seed_uids) {
      float2 origin(0.0f);
      for (const GraphNode &node : runtime.nodes) {
        if (node.is_seed && node.seed_uid == seed_uid) {
          origin = node.location;
          break;
        }
      }
      layout_seed_graph(runtime, seed_uid, origin, false);
    }
  }

  runtime.layout_dirty = false;
  /* Persist so reopen of .blend restores Y layout. */
  datablock_graph_store_positions(sdbg);
}

void datablock_graph_activate_or_create(SpaceDataBlockGraph &sdbg,
                                        Main &bmain,
                                        ID *id,
                                        const float2 *drop_view_co)
{
  if (id == nullptr) {
    return;
  }
  datablock_graph_ensure_runtime(sdbg);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;

  /* Already have this seed graph: clear others, highlight only this tree. */
  if (datablock_graph_has_seed(sdbg, id)) {
    datablock_graph_select_seed_graph(sdbg, id->session_uid);
    runtime.frame_view = true;
    return;
  }

  /* Create a brand-new independent ref-graph. */
  const float2 layout_origin = drop_view_co ? *drop_view_co : float2(0.0f);

  SpaceDataBlockGraphSeed *seed = MEM_new<SpaceDataBlockGraphSeed>(__func__);
  seed->id = id;
  BLI_addtail(&sdbg.seeds, seed);

  BKE_main_relations_create(&bmain, 0);
  MainIDRelations *relations = bmain.relations;
  BLI_assert(relations != nullptr);

  const int nodes_before = int(runtime.nodes.size());
  expand_one_seed(sdbg, bmain, *relations, runtime, *id);
  BKE_main_relations_free(&bmain);

  const uint seed_uid = id->session_uid;
  /* Temporary layout at origin, then snap AABB vs previous trees. */
  layout_seed_graph(runtime, seed_uid, float2(0.0f), true);
  place_new_seed_against_existing(runtime, seed_uid, nodes_before, layout_origin);

  /* Clear previous selection; select only the newly created ref-graph. */
  datablock_graph_deselect_all(sdbg);
  for (int i = nodes_before; i < int(runtime.nodes.size()); i++) {
    GraphNode &node = runtime.nodes[i];
    if (node.seed_uid == seed_uid) {
      node.selected = true;
      node.position_locked = true;
    }
  }
  runtime.active_key = datablock_graph_node_key(seed_uid, seed_uid);

  runtime.graph_dirty = false;
  runtime.layout_dirty = false;
  datablock_graph_store_positions(sdbg);
  /* Frame the whole canvas (all ref-graphs) after creating a new tree. */
  runtime.frame_view = true;
}

void datablock_graph_rebuild_if_needed(SpaceDataBlockGraph &sdbg, Main &bmain)
{
  datablock_graph_ensure_runtime(sdbg);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;
  const uint32_t walk_flags = sdbg.flag & (DATABLOCK_GRAPH_FLAG_RECURSIVE |
                                           DATABLOCK_GRAPH_FLAG_HIDE_EMBEDDED |
                                           DATABLOCK_GRAPH_FLAG_HIDE_LOOPBACK);
  if (runtime.last_built_max_depth != sdbg.max_depth || runtime.last_built_flag != walk_flags) {
    if (runtime.last_built_max_depth != sdbg.max_depth ||
        (runtime.last_built_flag & DATABLOCK_GRAPH_FLAG_RECURSIVE) !=
            (sdbg.flag & DATABLOCK_GRAPH_FLAG_RECURSIVE) ||
        (runtime.last_built_flag & DATABLOCK_GRAPH_FLAG_HIDE_EMBEDDED) !=
            (sdbg.flag & DATABLOCK_GRAPH_FLAG_HIDE_EMBEDDED) ||
        (runtime.last_built_flag & DATABLOCK_GRAPH_FLAG_HIDE_LOOPBACK) !=
            (sdbg.flag & DATABLOCK_GRAPH_FLAG_HIDE_LOOPBACK))
    {
      runtime.graph_dirty = true;
    }
  }
  if (runtime.graph_dirty) {
    rebuild_graph(sdbg, bmain);
    runtime.last_built_max_depth = sdbg.max_depth;
    runtime.last_built_flag = walk_flags;
  }
  if (runtime.layout_dirty) {
    datablock_graph_layout(sdbg, false);
  }
}

void datablock_graph_update_view2d(ARegion &region, SpaceDataBlockGraph &sdbg, const bool frame_all)
{
  datablock_graph_ensure_runtime(sdbg);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;
  View2D &v2d = region.v2d;

  const int winx = std::max(int(region.winx), 1);
  const int winy = std::max(int(region.winy), 1);
  const float win_aspect = float(winx) / float(winy);

  auto set_tot_and_maybe_frame = [&](rctf bounds) {
    const float pad = 120.0f;
    bounds.xmin -= pad;
    bounds.xmax += pad;
    bounds.ymin -= pad;
    bounds.ymax += pad;

    float bw = std::max(BLI_rctf_size_x(&bounds), 1.0f);
    float bh = std::max(BLI_rctf_size_y(&bounds), 1.0f);
    const float content_aspect = bw / bh;
    if (content_aspect > win_aspect) {
      const float new_h = bw / win_aspect;
      const float mid = BLI_rctf_cent_y(&bounds);
      bounds.ymin = mid - new_h * 0.5f;
      bounds.ymax = mid + new_h * 0.5f;
    }
    else {
      const float new_w = bh * win_aspect;
      const float mid = BLI_rctf_cent_x(&bounds);
      bounds.xmin = mid - new_w * 0.5f;
      bounds.xmax = mid + new_w * 0.5f;
    }

    v2d.tot = bounds;
    if (frame_all || runtime.frame_view) {
      v2d.cur = bounds;
      runtime.frame_view = false;
    }
  };

  rctf bounds;
  if (!datablock_graph_nodes_bounds(sdbg, bounds)) {
    BLI_rctf_init(&bounds, -200.0f, 200.0f, -200.0f, 200.0f);
  }
  set_tot_and_maybe_frame(bounds);
}

bool datablock_graph_align_selection(SpaceDataBlockGraph &sdbg, const bool horizontal)
{
  datablock_graph_ensure_runtime(sdbg);
  SpaceDataBlockGraph_Runtime &runtime = *sdbg.runtime;

  Vector<int> selected;
  for (const int i : runtime.nodes.index_range()) {
    if (runtime.nodes[i].selected) {
      selected.append(i);
    }
  }
  if (selected.size() < 2) {
    return false;
  }

  Map<int, int> index_to_sel;
  for (const int i : selected.index_range()) {
    index_to_sel.add(selected[i], i);
  }

  Vector<Vector<int>> children(selected.size());
  Vector<Vector<int>> parents(selected.size());
  int link_count = 0;
  for (const GraphEdge &edge : runtime.edges) {
    const int *from_i = index_to_sel.lookup_ptr(edge.from_index);
    const int *to_i = index_to_sel.lookup_ptr(edge.to_index);
    if (!from_i || !to_i || *from_i == *to_i) {
      continue;
    }
    children[*from_i].append(*to_i);
    parents[*to_i].append(*from_i);
    link_count++;
  }

  constexpr float gap = 40.0f;
  float2 free_center = float2(0.0f);
  for (const int idx : selected) {
    const GraphNode &n = runtime.nodes[idx];
    free_center += n.location + float2(n.size.x * 0.5f, 0.0f);
  }
  free_center /= float(selected.size());

  if (horizontal || link_count == 0) {
    std::sort(selected.begin(), selected.end(), [&](const int a, const int b) {
      return runtime.nodes[a].location.x < runtime.nodes[b].location.x;
    });
    float total = 0.0f;
    for (const int idx : selected) {
      total += runtime.nodes[idx].size.x;
    }
    total += gap * float(selected.size() - 1);
    float x = free_center.x - total * 0.5f;
    float y_sum = 0.0f;
    for (const int idx : selected) {
      y_sum += runtime.nodes[idx].location.y;
    }
    const float ref_y = y_sum / float(selected.size());
    for (const int idx : selected) {
      GraphNode &n = runtime.nodes[idx];
      if (horizontal || link_count == 0) {
        if (horizontal) {
          n.location = float2(x, ref_y);
          n.position_locked = true;
          x += n.size.x + gap;
        }
        else {
          /* vertical even pack handled below */
        }
      }
    }
    if (horizontal || link_count == 0) {
      if (!horizontal) {
        std::sort(selected.begin(), selected.end(), [&](const int a, const int b) {
          return runtime.nodes[a].location.y > runtime.nodes[b].location.y;
        });
        float total_h = 0.0f;
        for (const int idx : selected) {
          total_h += runtime.nodes[idx].size.y;
        }
        total_h += gap * float(selected.size() - 1);
        float y = free_center.y + total_h * 0.5f;
        float x_sum = 0.0f;
        for (const int idx : selected) {
          x_sum += runtime.nodes[idx].location.x;
        }
        const float ref_x = x_sum / float(selected.size());
        for (const int idx : selected) {
          GraphNode &n = runtime.nodes[idx];
          n.location = float2(ref_x, y);
          n.position_locked = true;
          y -= n.size.y + gap;
        }
      }
      datablock_graph_store_positions(sdbg);
      return true;
    }
  }

  /* Connected: simple even pack by axis. */
  if (!horizontal) {
    std::sort(selected.begin(), selected.end(), [&](const int a, const int b) {
      return runtime.nodes[a].location.y > runtime.nodes[b].location.y;
    });
    float total = 0.0f;
    for (const int idx : selected) {
      total += runtime.nodes[idx].size.y;
    }
    total += gap * float(selected.size() - 1);
    float y = free_center.y + total * 0.5f;
    float x_sum = 0.0f;
    for (const int idx : selected) {
      x_sum += runtime.nodes[idx].location.x;
    }
    const float ref_x = x_sum / float(selected.size());
    for (const int idx : selected) {
      GraphNode &n = runtime.nodes[idx];
      n.location = float2(ref_x, y);
      n.position_locked = true;
      y -= n.size.y + gap;
    }
  }
  datablock_graph_store_positions(sdbg);
  return true;
}

}  // namespace blender::ed::datablock_graph
