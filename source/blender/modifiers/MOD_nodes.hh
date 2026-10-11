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

/** What the Time Shift nodes of the modifier remember, see `MOD_nodes_time_shift.hh`. */
struct NodesModifierTimeShiftCache;

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
