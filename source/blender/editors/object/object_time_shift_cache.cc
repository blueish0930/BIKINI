/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Geometry Nodes Time Shift — own absolute-frame cache (not Bake-node pipeline).
 *
 * - Pure animation: one light cook at the requested absolute frame.
 * - Unbaked simulation: play forward to the target (resume when possible).
 * - Never leaves the wait cursor stuck; never re-tags the object for a full node-tree re-eval
 *   after fill (that caused "refresh every frame / spinning cursor" freezes).
 * - Frame numbers clamped to >= 1.
 */

#include <chrono>
#include <cmath>

#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_vector.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "ED_object.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_scene_types.h"
#include "DNA_windowmanager_types.h"

#include "BKE_bake_geometry_nodes_modifier.hh"
#include "BKE_global.hh"
#include "BKE_main.hh"
#include "BKE_modifier.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_runtime.hh"
#include "BKE_scene.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "MOD_nodes.hh"

#include "object_intern.hh"

namespace blender::ed::object::time_shift {

namespace bake = bke::bake;

static constexpr int min_absolute_frame = 1;

/** Attribute wall-clock of one light-cook onto Simulation Output / Time Shift nodes. */
static void attribute_light_cook_time(NodesModifierData &nmd,
                                      const std::shared_ptr<NodesModifierTimeShiftCache> &ts_cache,
                                      const int frame,
                                      const int64_t wall_ns)
{
  if (!ts_cache || wall_ns <= 0) {
    return;
  }
  std::lock_guard lock{ts_cache->mutex};
  ts_cache->cook_time_ns_by_frame.add_overwrite(frame, wall_ns);

  if (!nmd.node_group) {
    return;
  }
  nmd.node_group->ensure_topology_cache();
  for (const bNode *node : nmd.node_group->all_nodes()) {
    if (node->type_legacy != GEO_NODE_SIMULATION_OUTPUT &&
        node->type_legacy != GEO_NODE_TIME_SHIFT)
    {
      continue;
    }
    int64_t &accum = ts_cache->attributed_node_time_ns.lookup_or_add(node->identifier, 0);
    accum += wall_ns;
  }
}

static int clamp_absolute_frame(const int frame)
{
  return std::max(frame, min_absolute_frame);
}

/** Clears fill/pending flags even when ensure/fill returns early. */
static void clear_pending_flags(const std::shared_ptr<NodesModifierTimeShiftCache> &ts_cache)
{
  if (!ts_cache) {
    return;
  }
  std::lock_guard lock{ts_cache->mutex};
  ts_cache->background_cache_active = false;
  ts_cache->simulation_warmup_active = false;
  ts_cache->ignore_user_modified_during_warmup = false;
  ts_cache->warmup_target_frame = std::nullopt;
  ts_cache->ensure_in_flight_frame = std::nullopt;
  ts_cache->fill_viewer_frame = std::nullopt;
}

struct FillFlagsGuard {
  std::shared_ptr<NodesModifierTimeShiftCache> ts_cache;
  bool armed = false;

  FillFlagsGuard(std::shared_ptr<NodesModifierTimeShiftCache> cache,
                 const int target_frame,
                 const int viewer_frame)
      : ts_cache(std::move(cache))
  {
    if (!ts_cache) {
      return;
    }
    std::lock_guard lock{ts_cache->mutex};
    ts_cache->simulation_warmup_active = true;
    ts_cache->ignore_user_modified_during_warmup = true;
    ts_cache->background_cache_active = true;
    ts_cache->warmup_target_frame = target_frame;
    ts_cache->ensure_in_flight_frame = target_frame;
    ts_cache->fill_viewer_frame = viewer_frame;
    armed = true;
  }

  ~FillFlagsGuard()
  {
    if (!armed || !ts_cache) {
      return;
    }
    std::lock_guard lock{ts_cache->mutex};
    ts_cache->simulation_warmup_active = false;
    ts_cache->ignore_user_modified_during_warmup = false;
    ts_cache->background_cache_active = false;
    ts_cache->warmup_target_frame = std::nullopt;
    ts_cache->ensure_in_flight_frame = std::nullopt;
    ts_cache->fill_viewer_frame = std::nullopt;
  }
};

static void purge_invalid_frame_keys(const std::shared_ptr<NodesModifierTimeShiftCache> &ts_cache)
{
  if (!ts_cache) {
    return;
  }
  std::lock_guard lock{ts_cache->mutex};
  Vector<int> bad_keys;
  for (const int key : ts_cache->geometries_by_frame.keys()) {
    if (key < min_absolute_frame) {
      bad_keys.append(key);
    }
  }
  for (const int key : bad_keys) {
    ts_cache->geometries_by_frame.remove(key);
    ts_cache->content_token_by_frame.remove(key);
  }
  if (ts_cache->requested_frame.has_value() && *ts_cache->requested_frame < min_absolute_frame) {
    ts_cache->requested_frame = min_absolute_frame;
  }
}

static NodesModifierData *find_nmd_sharing_cache(
    Object &object, const std::shared_ptr<NodesModifierTimeShiftCache> &ts_cache)
{
  if (!ts_cache) {
    return nullptr;
  }
  for (ModifierData &md : object.modifiers) {
    if (md.type != eModifierType_Nodes) {
      continue;
    }
    auto &nmd = reinterpret_cast<NodesModifierData &>(md);
    if (nmd.runtime && nmd.runtime->time_shift_cache == ts_cache) {
      return &nmd;
    }
  }
  return nullptr;
}

static bool target_geometry_ready(const std::shared_ptr<NodesModifierTimeShiftCache> &ts_cache,
                                  const int frame)
{
  if (!ts_cache || frame < min_absolute_frame) {
    return false;
  }
  std::lock_guard lock{ts_cache->mutex};
  return ts_cache->geometries_by_frame.contains(frame);
}

static bool has_unbaked_simulation(NodesModifierData &nmd)
{
  if (!nmd.runtime || !nmd.runtime->cache) {
    return false;
  }
  std::lock_guard lock{nmd.runtime->cache->mutex};
  if (!nmd.runtime->cache->simulation_cache_by_id.is_empty()) {
    for (const std::unique_ptr<bake::SimulationNodeCache> &node_cache :
         nmd.runtime->cache->simulation_cache_by_id.values())
    {
      if (node_cache && node_cache->cache_status != bake::CacheStatus::Baked) {
        return true;
      }
    }
    return false;
  }
  if (!nmd.node_group) {
    return false;
  }
  for (bNode *node = static_cast<bNode *>(nmd.node_group->nodes.first()); node; node = node->next) {
    if (node->type_legacy == GEO_NODE_SIMULATION_OUTPUT) {
      return true;
    }
  }
  return false;
}

static bool simulation_cache_has_frame(const bake::SimulationNodeCache &node_cache, const int frame)
{
  for (const std::unique_ptr<bake::FrameCache> &frame_cache : node_cache.bake.frames) {
    if (frame_cache && frame_cache->frame.frame() == frame) {
      return true;
    }
  }
  return false;
}

/** Last frame of the contiguous run at the start of the cache. Does not modify the cache. */
static std::optional<int> simulation_contiguous_last_frame(const bake::SimulationNodeCache &node_cache)
{
  if (node_cache.bake.frames.is_empty() || !node_cache.bake.frames.first()) {
    return std::nullopt;
  }
  int last_index = 0;
  for (const int i : node_cache.bake.frames.index_range().drop_front(1)) {
    if (!node_cache.bake.frames[i - 1] || !node_cache.bake.frames[i]) {
      break;
    }
    const int prev = node_cache.bake.frames[i - 1]->frame.frame();
    const int cur = node_cache.bake.frames[i]->frame.frame();
    if (cur != prev + 1) {
      break;
    }
    last_index = i;
  }
  if (!node_cache.bake.frames[last_index]) {
    return std::nullopt;
  }
  const int last = node_cache.bake.frames[last_index]->frame.frame();
  if (last < min_absolute_frame) {
    return std::nullopt;
  }
  return last;
}

/**
 * Where a forward sim fill should resume.
 * - nullopt: target is already in every unbaked zone cache (or there is nothing to simulate).
 *   The caller cooks that one frame and the zone reads its cache. Later frames stay put.
 * - start_frame: cache is missing or invalid, replay from the beginning.
 * - last+1: cache is valid up to last and target is still ahead.
 *
 * Must not delete cached frames. Sampling an earlier absolute frame (Time Shift after the
 * simulation) used to pop every frame after the target, so the playhead came back to a hole
 * and both the simulation cache and the Time Shift store were wiped.
 */
static std::optional<int> simulation_resume_frame(NodesModifierData &nmd,
                                                  const int start_frame,
                                                  const int target)
{
  if (target < start_frame || target < min_absolute_frame) {
    return std::nullopt;
  }
  if (!nmd.runtime || !nmd.runtime->cache) {
    return start_frame;
  }
  std::lock_guard lock{nmd.runtime->cache->mutex};
  if (nmd.runtime->cache->simulation_cache_by_id.is_empty()) {
    return start_frame;
  }

  std::optional<int> common_last;
  bool saw_unbaked = false;
  bool target_cached = true;
  bool can_resume = true;

  for (const std::unique_ptr<bake::SimulationNodeCache> &node_cache :
       nmd.runtime->cache->simulation_cache_by_id.values())
  {
    if (!node_cache) {
      continue;
    }
    if (node_cache->cache_status == bake::CacheStatus::Baked) {
      continue;
    }
    saw_unbaked = true;
    if (node_cache->cache_status != bake::CacheStatus::Valid || node_cache->bake.frames.is_empty())
    {
      target_cached = false;
      can_resume = false;
      break;
    }
    if (!simulation_cache_has_frame(*node_cache, target)) {
      target_cached = false;
    }
    const std::optional<int> last = simulation_contiguous_last_frame(*node_cache);
    if (!last) {
      can_resume = false;
      break;
    }
    common_last = common_last ? std::min(*common_last, *last) : *last;
  }

  if (!saw_unbaked) {
    return std::nullopt;
  }
  /* Exact frame is already stored. One cook reads it; do not replay or truncate. */
  if (target_cached) {
    return std::nullopt;
  }
  if (!can_resume || !common_last) {
    return start_frame;
  }
  const int next = *common_last + 1;
  if (next > target) {
    return std::nullopt;
  }
  return std::max(start_frame, next);
}

static int64_t light_cook_object_at_frame(Depsgraph &depsgraph,
                                          Scene &scene,
                                          Object &object_original,
                                          const int frame)
{
  const int safe_frame = clamp_absolute_frame(frame);
  const auto t0 = std::chrono::steady_clock::now();
  BKE_scene_frame_set(&scene, safe_frame);
  DEG_id_tag_update_for_side_effect_request(
      &depsgraph, &object_original.id, ID_RECALC_GEOMETRY);
  DEG_evaluate_on_framechange(&depsgraph, float(safe_frame), DEG_EVALUATE_SYNC_WRITEBACK_NO);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() -
                                                              t0)
      .count();
}

static void light_restore_frame(Depsgraph &depsgraph,
                                Scene &scene,
                                Object &object_original,
                                const float old_frame)
{
  const float safe_old = std::max(old_frame, float(min_absolute_frame));
  BKE_scene_frame_set(&scene, safe_old);
  DEG_id_tag_update_for_side_effect_request(
      &depsgraph, &object_original.id, ID_RECALC_GEOMETRY);
  DEG_evaluate_on_framechange(&depsgraph, safe_old, DEG_EVALUATE_SYNC_WRITEBACK_NO);
}

static void fill_blocking(Depsgraph &depsgraph,
                          Scene &scene,
                          Object &object_original,
                          NodesModifierData &nmd,
                          const std::shared_ptr<NodesModifierTimeShiftCache> &ts_cache,
                          const int start_frame,
                          const int target_in,
                          wmWindow * /*window*/)
{
  const int target = clamp_absolute_frame(target_in);
  const int safe_start = std::max(start_frame, min_absolute_frame);

  if (target < safe_start) {
    clear_pending_flags(ts_cache);
    return;
  }
  if (target_geometry_ready(ts_cache, target)) {
    clear_pending_flags(ts_cache);
    return;
  }

  uint64_t revision = 0;
  {
    std::lock_guard lock{ts_cache->mutex};
    revision = ts_cache->revision;
  }

  const float old_frame = BKE_scene_frame_get(&scene);
  const int viewer_frame = int(std::floor(old_frame));
  const bool needs_sim = has_unbaked_simulation(nmd);

  /* No WM_cursor_wait: multi-frame sim fill already blocks the main thread; a wait cursor that
   * outlives a re-eval storm looked like permanent spinning. */

  FillFlagsGuard flags_guard(ts_cache, target, viewer_frame);

  auto revision_ok = [&]() -> bool {
    std::lock_guard lock{ts_cache->mutex};
    return ts_cache->revision == revision;
  };

  /* Fresh attribution for this fill — display eval only shows cache-hit (~0.1ms) otherwise. */
  {
    std::lock_guard lock{ts_cache->mutex};
    ts_cache->attributed_node_time_ns.clear();
    ts_cache->last_fill_time_ns = 0;
  }

  const auto fill_t0 = std::chrono::steady_clock::now();
  int64_t fill_total_ns = 0;

  auto cook_and_attribute = [&](const int frame) {
    const int64_t wall_ns = light_cook_object_at_frame(depsgraph, scene, object_original, frame);
    fill_total_ns += wall_ns;
    attribute_light_cook_time(nmd, ts_cache, frame, wall_ns);
  };

  if (!needs_sim) {
    if (revision_ok() && !G.is_break) {
      cook_and_attribute(target);
    }
  }
  else {
    std::optional<int> sim_next = simulation_resume_frame(nmd, safe_start, target);
    if (sim_next) {
      const int begin = clamp_absolute_frame(*sim_next);
      for (int frame = begin; frame <= target; frame++) {
        if (G.is_break || !revision_ok()) {
          break;
        }
        cook_and_attribute(frame);
      }
    }
    if (!target_geometry_ready(ts_cache, target) && revision_ok() && !G.is_break) {
      cook_and_attribute(target);
    }
  }

  /* Whole-fill wall clock (includes bookkeeping) for the requested absolute frame. */
  const int64_t fill_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::steady_clock::now() - fill_t0)
                                   .count();
  {
    std::lock_guard lock{ts_cache->mutex};
    ts_cache->last_fill_time_ns = fill_wall_ns;
    /* Target frame reports the full multi-frame cost (what the user waited for). */
    if (fill_total_ns > 0) {
      ts_cache->cook_time_ns_by_frame.add_overwrite(target, fill_total_ns);
    }
    else if (fill_wall_ns > 0) {
      ts_cache->cook_time_ns_by_frame.add_overwrite(target, fill_wall_ns);
    }
  }

  /* Restore view time. Do not ID_RECALC / ND_MODIFIER after this — that re-queues a full Geometry
   * Nodes evaluation every time and freezes the UI ("refreshing every frame"). Restore already
   * evaluated the object with Time Shift cache hits for the current view. */
  light_restore_frame(depsgraph, scene, object_original, old_frame);
  /* flags_guard clears in-flight flags on scope exit. */
}

void cache_ensure(wmWindowManager & /*wm*/,
                  wmWindow *window,
                  Main & /*bmain*/,
                  Depsgraph &depsgraph,
                  Scene &scene,
                  Object &object,
                  NodesModifierData &nmd,
                  const int frame_in)
{
  if (!nmd.runtime || !nmd.runtime->time_shift_cache) {
    return;
  }
  const int frame = clamp_absolute_frame(frame_in);
  std::shared_ptr<NodesModifierTimeShiftCache> ts_cache = nmd.runtime->time_shift_cache;

  purge_invalid_frame_keys(ts_cache);

  if (target_geometry_ready(ts_cache, frame)) {
    clear_pending_flags(ts_cache);
    return;
  }

  Object &object_original = *DEG_get_original(&object);

  NodesModifierData *nmd_for_work = &nmd;
  if (Object *object_eval = DEG_get_evaluated(&depsgraph, &object_original)) {
    if (NodesModifierData *nmd_orig = find_nmd_sharing_cache(object_original, ts_cache)) {
      nmd_for_work = nmd_orig;
    }
    else if (NodesModifierData *nmd_eval = find_nmd_sharing_cache(*object_eval, ts_cache)) {
      nmd_for_work = nmd_eval;
    }
  }
  if (!nmd_for_work || !nmd_for_work->runtime) {
    clear_pending_flags(ts_cache);
    return;
  }

  int start_frame = scene.r.sfra;
  if (scene.flag & SCE_CUSTOM_SIMULATION_RANGE) {
    start_frame = std::min(start_frame, scene.simulation_frame_start);
  }
  start_frame = std::max(start_frame, min_absolute_frame);
  if (frame < start_frame) {
    clear_pending_flags(ts_cache);
    return;
  }

  {
    std::lock_guard lock{ts_cache->mutex};
    if (ts_cache->simulation_warmup_active) {
      /* Nested ensure from light_cook — outer fill owns flags. */
      return;
    }
  }

  fill_blocking(
      depsgraph, scene, object_original, *nmd_for_work, ts_cache, start_frame, frame, window);
}

}  // namespace blender::ed::object::time_shift
