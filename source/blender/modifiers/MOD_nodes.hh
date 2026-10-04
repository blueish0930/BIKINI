/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#pragma once

#include <memory>
#include <optional>

#include "BLI_array.hh"
#include "BLI_map.hh"
#include "BLI_mutex.hh"
#include "BLI_sub_frame.hh"
#include "BLI_vector.hh"

#include "BKE_geometry_set.hh"

#include "NOD_socket_usage_inference_fwd.hh"

namespace blender {

class ComputeContext;
struct Depsgraph;
struct Main;
struct bNodeTree;
struct NodesModifierData;
struct NodesModifierDataBlock;
struct Object;
struct NodesModifierPackedBake;
struct NodesModifierBake;

namespace bke::bake {
struct ModifierCache;
}
namespace nodes::eval_log {
class NodesEvalLog;
}
namespace nodes {
class GeometryNodesOutputCache;
}

/**
 * Rebuild the list of properties based on the sockets exposed as the modifier's node group
 * inputs. If any properties correspond to the old properties by name and type, carry over
 * the values.
 */
void MOD_nodes_update_interface(Main &bmain, Object *object, NodesModifierData *nmd);

/**
 * Evaluate one call of a recursive node group on its own, with the inputs it had in the last
 * evaluation of the modifier. Calls nested in it output their recorded values instead of being
 * evaluated again. The values and viewers logged for the call are shown in place of the ones from
 * the full evaluation until the modifier is evaluated again.
 *
 * Returns false when the call was not recorded, then the modifier has to be evaluated.
 */
bool MOD_nodes_evaluate_recursive_call(Depsgraph &depsgraph,
                                       Object &object,
                                       NodesModifierData &nmd,
                                       const bNodeTree &called_tree,
                                       const ComputeContext &call_context);

class NodesModifierUsageInferenceCache {
 private:
  uint64_t input_values_hash_ = 0;

 public:
  Array<nodes::socket_usage_inference::SocketUsage> inputs;
  Array<nodes::socket_usage_inference::SocketUsage> outputs;

  void ensure(const Object &object, const NodesModifierData &nmd);
  void reset();
};

/**
 * Shared Time Shift cache (original + evaluated modifiers share one instance).
 *
 * Absolute scene frame → geometry (one map for the modifier; not keyed by nested node id).
 * Simulation cache is only used to resume sequential plays when needed.
 */
struct NodesModifierTimeShiftCache {
  mutable Mutex mutex;
  /** Time Shift's own store: absolute frame → geometry snapshot. */
  Map<int, bke::GeometrySet> geometries_by_frame;
  /**
   * Content fingerprint of each stored frame (cheap mesh/points hash). When the live upstream
   * geometry at the current scene frame disagrees with the stored fingerprint for that frame,
   * the whole store is wiped — geometry nodes re-evaluation always feeds the new upstream.
   */
  Map<int, uint64_t> content_token_by_frame;
  /** Last absolute frame requested (any Time Shift on this modifier). */
  std::optional<int> requested_frame;
  /**
   * Frame currently claimed for ensure/fill. Prevents re-scheduling the same miss every redraw
   * (which re-ran the whole node tree and left the wait cursor spinning).
   */
  std::optional<int> ensure_in_flight_frame;

  bool simulation_warmup_active = false;
  bool ignore_user_modified_during_warmup = false;
  /** Fill in progress (blocks nested ensure). */
  bool background_cache_active = false;
  /**
   * True only for this evaluation when Time Shift Frame sockets changed.
   * Must NOT stay sticky and must NOT include fill/warmup — that blocked simulation
   * UserModified invalidation and left stale sim caches forever after upstream edits.
   */
  bool frame_socket_scrub_active = false;
  /**
   * Last scene playhead frame from a non-fill evaluation. Used to edge-detect rewind to
   * simulation start. Intentionally not updated during Time Shift light-cook fills (those
   * temporarily jump ctime and must not poison rewind detection).
   */
  std::optional<int> last_scene_frame;
  Vector<int> last_frame_socket_values;
  std::optional<int> warmup_target_frame;
  /**
   * Scene frame that was current when this fill started (the viewport playhead).
   * Cooks of any other frame pass Time Shift through: the Frame socket is sampled only at this
   * playhead. Re-evaluating it during the cook (Scene Time ± offset) would request a second
   * shift and store/return the wrong absolute frame.
   */
  std::optional<int> fill_viewer_frame;
  uint64_t revision = 0;
  uint32_t output_topology_hash = 0;
  /**
   * Wall-clock nanoseconds spent producing each absolute frame (Time Shift fill / light-cook).
   * Used so the node editor can show real cost when the display eval only hits the cache
   * (~0.1 ms) after a multi-frame simulation cook happened off the timed path.
   */
  Map<int, int64_t> cook_time_ns_by_frame;
  /**
   * Last fill's attributed node times (node identifier → ns). Simulation Output / Time Shift
   * (and other nodes timed during light-cooks) accumulate here so the UI still shows them after
   * the fill eval_log is overwritten by a cheap restore/cache-hit eval.
   */
  Map<int32_t, int64_t> attributed_node_time_ns;
  /** Total wall time of the most recent cache_ensure fill (all frames). */
  int64_t last_fill_time_ns = 0;
};

struct NodesModifierRuntime {
  /**
   * Contains logged information from the last evaluation.
   * This can be used to help the user to debug a node tree.
   * This is a shared pointer because we might want to keep it around in some cases after the
   * evaluation (e.g. for gizmo backpropagation).
   */
  std::shared_ptr<nodes::eval_log::NodesEvalLog> eval_log;
  /**
   * Simulation cache that is shared between original and evaluated modifiers. This allows the
   * original modifier to be removed, without also removing the simulation state which may still be
   * used by the evaluated modifier.
   */
  std::shared_ptr<bke::bake::ModifierCache> cache;
  /** Geometry snapshots used by Time Shift nodes (shared orig/eval). */
  std::shared_ptr<NodesModifierTimeShiftCache> time_shift_cache;
  /**
   * Bake-style in-memory cache of geometry / single-value / list node outputs.
   * Shared between original and evaluated modifier.
   */
  std::shared_ptr<nodes::GeometryNodesOutputCache> output_cache;
  /**
   * Cache the usage of the node group inputs and outputs to accelerate drawing the UI when no
   * properties change.
   */
  NodesModifierUsageInferenceCache usage_cache;
};

void nodes_modifier_data_block_destruct(NodesModifierDataBlock *data_block, bool do_id_user);
void nodes_modifier_packed_bake_copy(NodesModifierBake &bake_dst,
                                     const NodesModifierBake &bake_src);
void nodes_modifier_packed_bake_free(NodesModifierPackedBake *packed_bake);
void nodes_modifier_bake_destruct(NodesModifierBake *bake, bool do_id_user);

}  // namespace blender
