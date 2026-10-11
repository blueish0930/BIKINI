/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

#include "BLI_listbase.hh"
#include "BLI_memory_counter.hh"
#include "BLI_sub_frame.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_userdef_types.h"

#include "BKE_bake_geometry_nodes_modifier.hh"
#include "BKE_main.hh"
#include "BKE_modifier.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"
#include "BKE_scene.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"
#include "DEG_depsgraph_writeback_sync.hh"

#include "MOD_nodes.hh"
#include "MOD_nodes_time_shift.hh"

namespace blender::time_shift {

namespace bake = bke::bake;

/* -------------------------------------------------------------------- */
/** \name Stored Geometries
 * \{ */

/** Memory used by the stored geometries of all modifiers. */
static std::atomic<int64_t> total_bytes = 0;

static int64_t memory_limit()
{
  /* Half of the cache limit in the preferences, the other caches need room too. */
  return std::max<int64_t>(U.memcachelimit, 64) * 1024 * 1024 / 2;
}

static int64_t geometry_bytes(const bke::GeometrySet &geometry)
{
  MemoryCount count;
  MemoryCounter counter{count};
  geometry.count_memory(counter);
  return count.total_bytes;
}

static void store_clear_entries(Store &store)
{
  for (const Entry &entry : store.entries.values()) {
    total_bytes -= entry.bytes;
  }
  store.entries.clear();
}

static void store_add(Store &store,
                      const EntryKey &key,
                      const bke::GeometrySet &geometry,
                      const int64_t bytes)
{
  Entry entry;
  entry.geometry = geometry;
  entry.bytes = bytes;
  total_bytes += entry.bytes;
  if (Entry *old_entry = store.entries.lookup_ptr(key)) {
    total_bytes -= old_entry->bytes;
    *old_entry = std::move(entry);
  }
  else {
    store.entries.add_new(key, std::move(entry));
  }
}

/** Remove what a node has stored outside of the given frames. */
static void store_keep_frames(Store &store,
                              const int32_t node_id,
                              const int64_t first_frame,
                              const int64_t last_frame)
{
  Vector<EntryKey> keys_to_remove;
  for (const EntryKey &key : store.entries.keys()) {
    if (key.node_id == node_id && (key.frame < first_frame || key.frame > last_frame)) {
      keys_to_remove.append(key);
    }
  }
  for (const EntryKey &key : keys_to_remove) {
    total_bytes -= store.entries.lookup(key).bytes;
    store.entries.remove(key);
  }
}

/** Remove the frames furthest away from the current one when too much memory is used. */
static void store_free_memory(Store &store, const int64_t current_frame)
{
  const int64_t limit = memory_limit();
  if (total_bytes <= limit) {
    return;
  }
  Vector<EntryKey> candidates;
  for (const EntryKey &key : store.entries.keys()) {
    if (key.frame != current_frame && !store.protected_frames.contains(key.frame)) {
      candidates.append(key);
    }
  }
  std::sort(candidates.begin(), candidates.end(), [&](const EntryKey &a, const EntryKey &b) {
    return std::abs(a.frame - current_frame) > std::abs(b.frame - current_frame);
  });
  for (const EntryKey &key : candidates) {
    if (total_bytes <= limit) {
      break;
    }
    total_bytes -= store.entries.lookup(key).bytes;
    store.entries.remove(key);
  }
}

static void wipe_locked(NodesModifierTimeShiftCache &cache)
{
  bool had_entries = false;
  for (Store &store : cache.stores.values()) {
    had_entries |= !store.entries.is_empty();
    store_clear_entries(store);
    store.node_states.clear();
  }
  if (had_entries) {
    cache.revision++;
  }
  cache.fill_time_ns.clear();
}

void wipe(NodesModifierTimeShiftCache &cache)
{
  std::lock_guard lock{cache.mutex};
  wipe_locked(cache);
}

int64_t frame_to_key(const float frame)
{
  if (!std::isfinite(frame)) {
    return 0;
  }
  return int64_t(std::llround(double(frame) * 1024.0));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Requests
 *
 * What the nodes ask for is collected per depsgraph and computed after the evaluation.
 * \{ */

struct Request {
  std::shared_ptr<NodesModifierTimeShiftCache> cache;
  /** Original object the modifier is on. */
  Object *object = nullptr;
  int32_t node_id = 0;
  float frame = 0.0f;
  int64_t frame_key = 0;
  /**
   * False when a node did not find its frame while another frame is computed and nothing that
   * was asked for in the same modifier uses the output of that node. It still matters when
   * another object that was asked for a frame reads the geometry of this one.
   */
  bool needed = true;
};

struct CookTarget {
  const NodesModifierTimeShiftCache *cache = nullptr;
  int32_t node_id = 0;
};

struct DepsgraphState {
  /** The depsgraph is evaluated at other frames right now. */
  bool filling = false;
  /** A callback that computes #requests has been added to the depsgraph. */
  bool scheduled = false;
  /** Asked for by a normal evaluation. */
  Vector<Request> requests;
  /** Asked for by the evaluation of a frame that is computed for #cook_targets. */
  Vector<Request> fill_requests;
  /** The frame the nodes in #cook_targets need to see. */
  std::optional<int64_t> cook_frame;
  Vector<CookTarget> cook_targets;
  /** Caches that store geometries for this depsgraph. */
  Vector<std::weak_ptr<NodesModifierTimeShiftCache>> caches;
};

static Mutex &registry_mutex()
{
  static Mutex *mutex = new Mutex();
  return *mutex;
}

static Map<const Depsgraph *, std::unique_ptr<DepsgraphState>> &registry()
{
  static auto *map = new Map<const Depsgraph *, std::unique_ptr<DepsgraphState>>();
  return *map;
}

static void depsgraph_freed(const Depsgraph *depsgraph)
{
  std::unique_ptr<DepsgraphState> state;
  {
    std::lock_guard lock{registry_mutex()};
    if (std::unique_ptr<DepsgraphState> *state_ptr = registry().lookup_ptr(depsgraph)) {
      state = std::move(*state_ptr);
      registry().remove(depsgraph);
    }
  }
  if (!state) {
    return;
  }
  for (const std::weak_ptr<NodesModifierTimeShiftCache> &cache_weak : state->caches) {
    if (const std::shared_ptr<NodesModifierTimeShiftCache> cache = cache_weak.lock()) {
      std::lock_guard lock{cache->mutex};
      if (Store *store = cache->stores.lookup_ptr(depsgraph)) {
        store_clear_entries(*store);
        cache->stores.remove(depsgraph);
      }
    }
  }
}

/** The registry has to be locked. */
static DepsgraphState &ensure_state(Depsgraph &depsgraph)
{
  std::unique_ptr<DepsgraphState> &state = registry().lookup_or_add_default(&depsgraph);
  if (!state) {
    state = std::make_unique<DepsgraphState>();
    deg::post_eval::add_free_callback(depsgraph,
                                      [depsgraph = &depsgraph]() { depsgraph_freed(depsgraph); });
  }
  return *state;
}

bool is_filling(const Depsgraph *depsgraph)
{
  std::lock_guard lock{registry_mutex()};
  const std::unique_ptr<DepsgraphState> *state = registry().lookup_ptr(depsgraph);
  return state && *state && (*state)->filling;
}

static void register_cache(Depsgraph &depsgraph,
                           const std::shared_ptr<NodesModifierTimeShiftCache> &cache)
{
  std::lock_guard lock{registry_mutex()};
  DepsgraphState &state = ensure_state(depsgraph);
  state.caches.remove_if(
      [](const std::weak_ptr<NodesModifierTimeShiftCache> &other) { return other.expired(); });
  state.caches.append(cache);
}

/** A node that is computed right now for the frame being evaluated uses one of the given nodes. */
static bool cook_uses_nodes(const Depsgraph &depsgraph,
                            const NodesModifierTimeShiftCache &cache,
                            const int64_t frame_key,
                            const Span<int32_t> node_ids)
{
  std::lock_guard lock{registry_mutex()};
  const std::unique_ptr<DepsgraphState> *state_ptr = registry().lookup_ptr(&depsgraph);
  if (!state_ptr || !*state_ptr) {
    return false;
  }
  const DepsgraphState &state = **state_ptr;
  if (state.cook_frame != frame_key) {
    return false;
  }
  for (const CookTarget &target : state.cook_targets) {
    if (target.cache == &cache && node_ids.contains(target.node_id)) {
      return true;
    }
  }
  return false;
}

static void run_fill(Depsgraph &depsgraph);

static void add_request(Depsgraph &depsgraph, Request request)
{
  bool schedule = false;
  {
    std::lock_guard lock{registry_mutex()};
    DepsgraphState &state = ensure_state(depsgraph);
    if (state.filling) {
      state.fill_requests.append(std::move(request));
    }
    else {
      state.requests.append(std::move(request));
      if (!state.scheduled) {
        state.scheduled = true;
        schedule = true;
      }
    }
  }
  if (schedule) {
    deg::post_eval::add(depsgraph, [depsgraph = &depsgraph]() { run_fill(*depsgraph); });
  }
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Node Tree Analysis
 * \{ */

struct TreeInfo {
  /** Time Shift nodes in front of each Time Shift node, by nested node id of this tree. */
  Map<int32_t, Set<int32_t>> upstream;
  /** Nodes that use a group input. */
  Set<int32_t> from_input;
  /** Nodes the group output uses. */
  Set<int32_t> to_output;
};

static void set_merge(Set<int32_t> &dst, const Set<int32_t> &src)
{
  for (const int32_t value : src) {
    dst.add(value);
  }
}

static const TreeInfo &analyze_tree(const bNodeTree &tree,
                                    Map<const bNodeTree *, std::unique_ptr<TreeInfo>> &infos)
{
  if (const std::unique_ptr<TreeInfo> *info = infos.lookup_ptr(&tree)) {
    /* Also reached for a group that contains itself, with what is known so far. */
    return **info;
  }
  TreeInfo &info = *infos.lookup_or_add_cb(&tree, []() { return std::make_unique<TreeInfo>(); });

  tree.ensure_topology_cache();
  Map<bNestedNodePath, int32_t> id_by_path;
  for (const bNestedNodeRef &ref : tree.nested_node_refs_span()) {
    id_by_path.add(ref.path, ref.id);
  }

  const Span<const bNode *> nodes = tree.toposort_left_to_right();
  Array<Set<int32_t>> upstream_by_node(tree.all_nodes().size());
  Array<bool> from_input_by_node(tree.all_nodes().size(), false);

  for (const bNode *node : nodes) {
    Set<int32_t> upstream;
    bool from_input = node->is_group_input();
    for (const bNodeSocket *socket : node->input_sockets()) {
      for (const bNodeSocket *origin : socket->directly_linked_sockets()) {
        const bNode &origin_node = origin->owner_node();
        set_merge(upstream, upstream_by_node[origin_node.index()]);
        from_input |= from_input_by_node[origin_node.index()];
      }
    }
    if (node->is_type("GeometryNodeTimeShift"_ustr)) {
      if (const int32_t *id = id_by_path.lookup_ptr({node->identifier, -1})) {
        info.upstream.add(*id, upstream);
        if (from_input) {
          info.from_input.add(*id);
        }
        upstream.add(*id);
      }
    }
    else if (node->is_group() && node->id != nullptr) {
      const bNodeTree &group = *reinterpret_cast<const bNodeTree *>(node->id);
      const TreeInfo &group_info = analyze_tree(group, infos);
      Set<int32_t> output = upstream;
      for (const bNestedNodeRef &group_ref : group.nested_node_refs_span()) {
        const Set<int32_t> *group_upstream = group_info.upstream.lookup_ptr(group_ref.id);
        if (group_upstream == nullptr) {
          continue;
        }
        const int32_t *id = id_by_path.lookup_ptr({node->identifier, group_ref.id});
        if (id == nullptr) {
          continue;
        }
        Set<int32_t> nested_upstream;
        for (const int32_t group_id : *group_upstream) {
          if (const int32_t *other_id = id_by_path.lookup_ptr({node->identifier, group_id})) {
            nested_upstream.add(*other_id);
          }
        }
        if (group_info.from_input.contains(group_ref.id)) {
          set_merge(nested_upstream, upstream);
          if (from_input) {
            info.from_input.add(*id);
          }
        }
        info.upstream.add(*id, std::move(nested_upstream));
        if (group_info.to_output.contains(group_ref.id)) {
          output.add(*id);
        }
      }
      upstream = std::move(output);
    }
    if (node->is_group_output()) {
      set_merge(info.to_output, upstream);
    }
    upstream_by_node[node->index()] = std::move(upstream);
    from_input_by_node[node->index()] = from_input;
  }
  return info;
}

static int count_time_shift_nodes(const bNodeTree &tree)
{
  int count = 0;
  for (const bNestedNodeRef &ref : tree.nested_node_refs_span()) {
    const bNode *node = tree.find_nested_node(ref.id);
    if (node != nullptr && node->is_type("GeometryNodeTimeShift"_ustr)) {
      count++;
    }
  }
  return count;
}

static TreeAnalysis analyze_modifier_tree(const bNodeTree &tree)
{
  TreeAnalysis analysis;
  analysis.nodes_num = count_time_shift_nodes(tree);
  if (analysis.nodes_num < 2) {
    /* A single node can't depend on another one. */
    return analysis;
  }
  Map<const bNodeTree *, std::unique_ptr<TreeInfo>> infos;
  const TreeInfo &info = analyze_tree(tree, infos);
  for (const auto item : info.upstream.items()) {
    if (item.value.is_empty()) {
      continue;
    }
    Vector<int32_t> &upstream = analysis.upstream.lookup_or_add_default(item.key);
    for (const int32_t other_id : item.value) {
      if (other_id == item.key) {
        continue;
      }
      upstream.append(other_id);
      analysis.downstream.lookup_or_add_default(other_id).append(item.key);
    }
  }
  return analysis;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Modifier Evaluation
 * \{ */

ModifierParams::ModifierParams(NodesModifierData &nmd, const ModifierEvalContext &ctx)
    : cache_(nmd.runtime->time_shift_cache), depsgraph_(ctx.depsgraph)
{
  if (!cache_ || nmd.node_group == nullptr) {
    cache_.reset();
    return;
  }
  const bNodeTree &tree = *nmd.node_group;
  analysis_ = analyze_modifier_tree(tree);
  if (analysis_.nodes_num == 0) {
    /* Free what the nodes stored before they were removed. */
    wipe(*cache_);
    cache_.reset();
    return;
  }
  object_orig_ = DEG_get_original(ctx.object);
  ctime_ = DEG_get_ctime(depsgraph_);
  ctime_key_ = frame_to_key(ctime_);
  filling_ = is_filling(depsgraph_);

  /* Frame inputs that are not linked, to tell apart a change of them from other edits. */
  Map<int32_t, float> frame_values;
  tree.ensure_topology_cache();
  for (const bNode *node : tree.nodes_by_type("GeometryNodeTimeShift"_ustr)) {
    const bNodeSocket *socket = bke::node_find_socket(*node, SOCK_IN, "Frame"_ustr);
    if (socket == nullptr || socket->type != SOCK_FLOAT || socket->is_logically_linked()) {
      continue;
    }
    frame_values.add(node->identifier,
                     socket->default_value_typed<bNodeSocketValueFloat>()->value);
  }

  std::lock_guard lock{cache_->mutex};
  const uint32_t topology_hash = tree.runtime->output_topology_hash;
  if (cache_->output_topology_hash != topology_hash) {
    if (cache_->output_topology_hash != 0) {
      wipe_locked(*cache_);
    }
    cache_->output_topology_hash = topology_hash;
  }
  else if (!filling_ && (nmd.modifier.flag & eModifierFlag_UserModified)) {
    /* Something the geometry depends on was changed, what was stored for other frames does not
     * fit anymore. The exception is a change of the Frame input of a node, which is what one
     * drags to look at another frame. Then the geometry in front of that node stays the same,
     * unless a node in front of another one was changed. */
    bool only_frames_changed = false;
    for (const auto item : frame_values.items()) {
      const float *old_value = cache_->last_frame_socket_values.lookup_ptr(item.key);
      if (old_value == nullptr || *old_value == item.value) {
        continue;
      }
      only_frames_changed = true;
      const bNestedNodeRef *ref = tree.nested_node_ref_from_node_id_path({item.key});
      if (ref == nullptr || analysis_.downstream.contains(ref->id)) {
        only_frames_changed = false;
        break;
      }
    }
    if (!only_frames_changed) {
      wipe_locked(*cache_);
    }
  }
  cache_->last_frame_socket_values = std::move(frame_values);
}

uint64_t ModifierParams::revision() const
{
  if (!cache_) {
    return 0;
  }
  std::lock_guard lock{cache_->mutex};
  return cache_->revision;
}

bool ModifierParams::evaluate(const int id,
                              const float frame,
                              bke::GeometrySet current_geometry,
                              bke::GeometrySet &r_shifted_geometry) const
{
  if (!cache_ || !std::isfinite(frame)) {
    r_shifted_geometry = std::move(current_geometry);
    return true;
  }
  const int64_t frame_key = frame_to_key(frame);
  const EntryKey current_key{id, ctime_key_};

  /* The input is not what it should be when a node in front of this one passed its input
   * through. Such a geometry is not remembered. */
  bool input_is_final = true;
  if (const Vector<int32_t> *upstream = analysis_.upstream.lookup_ptr(id)) {
    std::lock_guard lock{missed_mutex_};
    for (const int32_t other_id : *upstream) {
      if (missed_nodes_.contains(other_id)) {
        input_is_final = false;
        break;
      }
    }
  }
  /* This frame is evaluated because the node asked for it. */
  const bool is_cook_target = filling_ &&
                              cook_uses_nodes(*depsgraph_, *cache_, ctime_key_, {id});

  /* Decide whether to remember the input for the frame that is evaluated. */
  bool remember = false;
  bool store_is_new = false;
  {
    std::lock_guard lock{cache_->mutex};
    store_is_new = !cache_->stores.contains(depsgraph_);
    Store &store = cache_->stores.lookup_or_add_default(depsgraph_);
    if (filling_) {
      remember = is_cook_target;
      if (is_cook_target && !input_is_final) {
        if (const Entry *entry = store.entries.lookup_ptr(current_key)) {
          total_bytes -= entry->bytes;
          store.entries.remove(current_key);
        }
      }
    }
    else {
      /* Only what the node is going to output is kept, everything else is computed again when
       * it is asked for. That is the requested frame, and the frames between it and the
       * evaluated one when the requested frame follows the evaluated one, like `Scene Time - 5`
       * while playing forward: those have been evaluated already and are asked for next. A node
       * that always outputs the same frame, or one that looks ahead, never comes back to the
       * frames it has seen. */
      NodeState &node_state = store.node_states.lookup_or_add_default(id);
      if (node_state.is_set) {
        const bool moved = node_state.evaluated_frame != ctime_key_;
        const bool request_changed = node_state.requested_frame != frame_key;
        bool follows = false;
        if (moved && request_changed && frame_key != ctime_key_) {
          const bool moves_forward = ctime_key_ > node_state.evaluated_frame;
          const bool looks_back = frame_key < ctime_key_;
          follows = moves_forward == looks_back;
        }
        if (follows) {
          remember = !store.entries.contains(current_key);
          store_keep_frames(
              store, id, std::min(frame_key, ctime_key_), std::max(frame_key, ctime_key_));
        }
        else if (moved || request_changed) {
          store_keep_frames(store, id, frame_key, frame_key);
        }
      }
      node_state.is_set = true;
      node_state.evaluated_frame = ctime_key_;
      node_state.requested_frame = frame_key;
    }
    remember &= input_is_final;
  }
  if (store_is_new) {
    register_cache(*depsgraph_, cache_);
  }
  if (remember) {
    /* The geometry outlives this evaluation, it must not reference data of the object. This
     * and counting the memory is done before the cache is locked, both may compute things. */
    current_geometry.ensure_owns_all_data();
    const int64_t bytes = geometry_bytes(current_geometry);
    std::lock_guard lock{cache_->mutex};
    Store &store = cache_->stores.lookup_or_add_default(depsgraph_);
    store_add(store, current_key, current_geometry, bytes);
    store_free_memory(store, ctime_key_);
  }

  if (frame_key == ctime_key_) {
    r_shifted_geometry = std::move(current_geometry);
    return true;
  }
  {
    std::lock_guard lock{cache_->mutex};
    Store &store = cache_->stores.lookup_or_add_default(depsgraph_);
    if (const Entry *entry = store.entries.lookup_ptr({id, frame_key})) {
      r_shifted_geometry = entry->geometry;
      return true;
    }
    store.protected_frames.add(frame_key);
  }

  {
    std::lock_guard lock{missed_mutex_};
    missed_nodes_.add(id);
  }
  had_miss_.store(true, std::memory_order_relaxed);

  Request request;
  request.cache = cache_;
  request.object = object_orig_;
  request.node_id = id;
  request.frame = frame;
  request.frame_key = frame_key;
  if (filling_) {
    const Vector<int32_t> *downstream = analysis_.downstream.lookup_ptr(id);
    request.needed = downstream != nullptr &&
                     cook_uses_nodes(*depsgraph_, *cache_, ctime_key_, *downstream);
  }
  add_request(*depsgraph_, std::move(request));

  r_shifted_geometry = std::move(current_geometry);
  return false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Computing Missing Frames
 * \{ */

struct FillContext {
  Depsgraph &depsgraph;
  Scene &scene;
  Main &bmain;
  /** Time of the depsgraph before other frames are evaluated. */
  float original_ctime = 0.0f;
  /** Objects with a modifier that asked for a frame. */
  Set<const Object *> objects;
  int cooks_num = 0;
  /** Object pairs for which it is known whether the first one uses the second one. */
  Map<std::pair<const Object *, const Object *>, bool> dependency_cache;
};

/** Enough for a simulation that has to be stepped from its start. */
static constexpr int max_cooks_per_fill = 200000;
/** How many Time Shift nodes in a row are followed. */
static constexpr int max_resolve_depth = 8;

static void evaluate_at(FillContext &fill, const float ctime)
{
  const float framelen = fill.scene.r.framelen;
  const float scene_frame = (framelen > 1e-6f) ? ctime / framelen : ctime;
  /* The time of the depsgraph is derived from the frame of the scene. */
  BKE_scene_frame_set(&fill.scene, scene_frame);
  DEG_evaluate_on_framechange(&fill.depsgraph, scene_frame, DEG_EVALUATE_SYNC_WRITEBACK_NO);
  fill.cooks_num++;
}

static bool cache_has_frame(const bake::NodeBakeCache &bake_cache, const int64_t frame_key)
{
  for (const std::unique_ptr<bake::FrameCache> &frame_cache : bake_cache.frames) {
    if (frame_cache && frame_to_key(float(frame_cache->frame)) == frame_key) {
      return true;
    }
  }
  return false;
}

/**
 * Simulations only give the right state for a frame when they went through the frames before it.
 * Returns the frames that have to be evaluated before \a frame so that all simulations that
 * are running get there one step at a time. Simulations that already know the frame are left
 * alone, as are the ones where it lies in the middle of what they have cached.
 */
static std::optional<std::pair<int, int>> simulation_step_frames(FillContext &fill,
                                                                const float frame)
{
  const int64_t original_key = frame_to_key(fill.original_ctime);
  std::optional<int> first_frame;
  int last_frame = std::numeric_limits<int>::min();
  /* The simulation has to go from \a first to \a last one frame at a time. */
  auto add_steps = [&](const int first, const int last) {
    first_frame = first_frame ? std::min(*first_frame, first) : first;
    last_frame = std::max(last_frame, last);
  };

  for (Object &object : fill.bmain.objects) {
    for (ModifierData &md : object.modifiers) {
      if (md.type != eModifierType_Nodes) {
        continue;
      }
      NodesModifierData &nmd = reinterpret_cast<NodesModifierData &>(md);
      if (nmd.node_group == nullptr || nmd.runtime == nullptr || !nmd.runtime->cache) {
        continue;
      }
      bool reset_any = false;
      {
        bake::ModifierCache &modifier_cache = *nmd.runtime->cache;
        std::lock_guard lock{modifier_cache.mutex};
        for (const auto item : modifier_cache.simulation_cache_by_id.items()) {
          bake::SimulationNodeCache &node_cache = *item.value;
          if (node_cache.cache_status == bake::CacheStatus::Baked) {
            continue;
          }
          /* Only simulations that are evaluated in this depsgraph: the ones that were at the
           * original frame, and the ones of the objects that asked for a frame. */
          const bool is_live = cache_has_frame(node_cache.bake, original_key) ||
                               (node_cache.prev_cache &&
                                frame_to_key(float(node_cache.prev_cache->frame)) == original_key);
          if (!is_live && !fill.objects.contains(&object)) {
            continue;
          }
          const std::optional<IndexRange> range = bake::get_node_bake_frame_range(
              fill.scene, object, nmd, item.key);
          if (!range || range->is_empty()) {
            continue;
          }
          const int start_frame = int(range->first());
          const int end_frame = int(range->last());
          if (frame <= float(start_frame)) {
            continue;
          }
          /* A simulation stays in its last state after its end. */
          const float sim_frame = std::min(frame, float(end_frame));
          const int64_t sim_frame_key = frame_to_key(sim_frame);
          const int last_step = int(std::ceil(sim_frame)) - (sim_frame < frame ? 0 : 1);
          if (node_cache.bake.frames.is_empty() && node_cache.prev_cache) {
            /* The object does not cache its simulations. Continue from the only state there is.
             * While other frames are computed the states are kept per frame. */
            auto frame_cache = std::make_unique<bake::FrameCache>();
            frame_cache->frame = node_cache.prev_cache->frame;
            frame_cache->values = node_cache.prev_cache->values;
            node_cache.bake.frames.append(std::move(frame_cache));
          }
          if (cache_has_frame(node_cache.bake, sim_frame_key)) {
            continue;
          }
          const bool never_ran = node_cache.bake.frames.is_empty() ||
                                 (node_cache.cache_status == bake::CacheStatus::Invalid &&
                                  node_cache.bake.frames.size() == 1);
          if (never_ran) {
            /* The simulation was entered in the middle, there is nothing to continue from. */
            node_cache.reset();
            reset_any = true;
            add_steps(start_frame, last_step);
            continue;
          }
          const float last_cached_frame = float(node_cache.bake.frames.last()->frame);
          if (sim_frame > last_cached_frame) {
            add_steps(int(std::floor(last_cached_frame)) + 1, last_step);
          }
        }
      }
      if (reset_any && nmd.runtime->time_shift_cache) {
        /* What the nodes behind the simulation have seen is not what it computes now. */
        wipe(*nmd.runtime->time_shift_cache);
      }
    }
  }
  if (first_frame && *first_frame <= last_frame) {
    return std::pair<int, int>{*first_frame, last_frame};
  }
  return std::nullopt;
}

static bool object_uses_object(FillContext &fill, const Object *user, const Object *used)
{
  if (user == used) {
    /* Another modifier on the same object. */
    return true;
  }
  if (user == nullptr || used == nullptr) {
    return false;
  }
  return fill.dependency_cache.lookup_or_add_cb({user, used}, [&]() {
    bool found = false;
    DEG_foreach_dependent_ID(&fill.depsgraph, &used->id, [&](ID *id) {
      found |= (id == &user->id);
    });
    return found;
  });
}

static bool request_is_satisfied(const FillContext &fill, const Request &request)
{
  std::lock_guard lock{request.cache->mutex};
  const Store *store = request.cache->stores.lookup_ptr(&fill.depsgraph);
  return store != nullptr && store->entries.contains({request.node_id, request.frame_key});
}

/** Evaluate the depsgraph at the frame, the nodes of the requests remember what they get. */
static void cook_frame(FillContext &fill, const float frame, const Span<Request> targets)
{
  Vector<float> frames;
  if (DEG_is_active(&fill.depsgraph)) {
    /* Only the active depsgraph steps simulations, the others read what is cached. */
    if (const std::optional<std::pair<int, int>> steps = simulation_step_frames(fill, frame)) {
      for (int i = steps->first; i <= steps->second; i++) {
        if (float(i) >= frame || frames.size() > max_cooks_per_fill) {
          break;
        }
        frames.append(float(i));
      }
    }
  }
  frames.append(frame);

  for (const int i : frames.index_range()) {
    if (fill.cooks_num >= max_cooks_per_fill) {
      break;
    }
    {
      std::lock_guard lock{registry_mutex()};
      DepsgraphState &state = ensure_state(fill.depsgraph);
      state.fill_requests.clear();
      state.cook_targets.clear();
      state.cook_frame.reset();
      if (i == frames.index_range().last()) {
        state.cook_frame = frame_to_key(frame);
        for (const Request &request : targets) {
          state.cook_targets.append({request.cache.get(), request.node_id});
        }
      }
    }
    evaluate_at(fill, frames[i]);
  }
}

/**
 * What the evaluation of a frame asked for and what is required to get that frame right.
 * \param r_recook: Set when geometry that was remembered in this evaluation may be wrong.
 */
static Vector<Request> take_dependencies(FillContext &fill,
                                         const Span<Request> targets,
                                         bool &r_recook)
{
  Vector<Request> fill_requests;
  {
    std::lock_guard lock{registry_mutex()};
    DepsgraphState &state = ensure_state(fill.depsgraph);
    fill_requests = std::move(state.fill_requests);
    state.fill_requests.clear();
    state.cook_targets.clear();
    state.cook_frame.reset();
  }
  Vector<Request> dependencies;
  Set<std::pair<const NodesModifierTimeShiftCache *, std::pair<int32_t, int64_t>>> added;
  for (Request &request : fill_requests) {
    bool needed = request.needed;
    if (!needed) {
      /* The geometry of the object is not complete at this frame. That matters when a node of
       * another modifier that reads this geometry is computed. */
      for (const Request &target : targets) {
        if (target.cache != request.cache &&
            object_uses_object(fill, target.object, request.object))
        {
          needed = true;
          r_recook = true;
          break;
        }
      }
    }
    if (!needed) {
      continue;
    }
    if (added.add({request.cache.get(), {request.node_id, request.frame_key}})) {
      dependencies.append(std::move(request));
    }
  }
  return dependencies;
}

static void resolve_requests(FillContext &fill, Vector<Request> requests, const int depth)
{
  std::sort(requests.begin(), requests.end(), [](const Request &a, const Request &b) {
    return a.frame_key < b.frame_key;
  });
  int group_start = 0;
  while (group_start < requests.size()) {
    int group_end = group_start + 1;
    while (group_end < requests.size() &&
           requests[group_end].frame_key == requests[group_start].frame_key)
    {
      group_end++;
    }
    const Span<Request> group = requests.as_span().slice(group_start, group_end - group_start);
    group_start = group_end;

    bool recook = false;
    for (int iteration = 0; iteration < 3; iteration++) {
      Vector<Request> targets;
      for (const Request &request : group) {
        if (recook || !request_is_satisfied(fill, request)) {
          targets.append(request);
        }
      }
      if (targets.is_empty() || fill.cooks_num >= max_cooks_per_fill) {
        break;
      }
      recook = false;
      cook_frame(fill, group.first().frame, targets);
      Vector<Request> dependencies = take_dependencies(fill, targets, recook);
      if (dependencies.is_empty() || depth >= max_resolve_depth) {
        break;
      }
      /* The frame is evaluated again after what it needs is known. */
      resolve_requests(fill, std::move(dependencies), depth + 1);
    }
  }
}

static void run_fill(Depsgraph &depsgraph)
{
  Vector<Request> requests;
  {
    std::lock_guard lock{registry_mutex()};
    DepsgraphState &state = ensure_state(depsgraph);
    state.scheduled = false;
    requests = std::move(state.requests);
    state.requests.clear();
    if (requests.is_empty()) {
      return;
    }
    state.filling = true;
    state.fill_requests.clear();
  }

  Scene &scene = *DEG_get_input_scene(&depsgraph);
  const int old_cfra = scene.r.cfra;
  const float old_subframe = scene.r.subframe;
  const float old_frame = BKE_scene_frame_get(&scene);

  FillContext fill{depsgraph, scene, *DEG_get_bmain(&depsgraph)};
  fill.original_ctime = DEG_get_ctime(&depsgraph);
  Vector<std::shared_ptr<NodesModifierTimeShiftCache>> caches;
  for (const Request &request : requests) {
    fill.objects.add(request.object);
    caches.append_non_duplicates(request.cache);
  }

  const auto start_time = std::chrono::steady_clock::now();
  resolve_requests(fill, requests, 0);
  const int64_t fill_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now() - start_time)
                                   .count();

  for (const std::shared_ptr<NodesModifierTimeShiftCache> &cache : caches) {
    std::lock_guard lock{cache->mutex};
    /* The evaluation below gives another result than the one that asked for the frames. */
    cache->revision++;
    for (const Request &request : requests) {
      if (request.cache == cache) {
        cache->fill_time_ns.add_overwrite(request.node_id, fill_time_ns);
      }
    }
  }

  /* Back to the frame the depsgraph was at. The nodes find their frames now. What is still
   * missing can't be computed, asking for it again here would never end. */
  scene.r.cfra = old_cfra;
  scene.r.subframe = old_subframe;
  DEG_evaluate_on_framechange(&depsgraph, old_frame, DEG_EVALUATE_SYNC_WRITEBACK_NO);

  {
    std::lock_guard lock{registry_mutex()};
    DepsgraphState &state = ensure_state(depsgraph);
    state.filling = false;
    state.fill_requests.clear();
    state.cook_targets.clear();
    state.cook_frame.reset();
  }
  for (const std::shared_ptr<NodesModifierTimeShiftCache> &cache : caches) {
    std::lock_guard lock{cache->mutex};
    if (Store *store = cache->stores.lookup_ptr(&depsgraph)) {
      store->protected_frames.clear();
    }
  }
}

/** \} */

}  // namespace blender::time_shift

namespace blender {

NodesModifierTimeShiftCache::~NodesModifierTimeShiftCache()
{
  for (time_shift::Store &store : stores.values()) {
    time_shift::store_clear_entries(store);
  }
}

}  // namespace blender
