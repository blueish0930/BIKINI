/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup modifiers
 *
 * Cache and evaluation of the Time Shift geometry node.
 *
 * A Time Shift node outputs the geometry its input has at another scene frame. Geometry nodes
 * can only be evaluated at the frame of the depsgraph. When the frame a node has to output is
 * not known, the node asks for it and passes its input through. After that evaluation the
 * depsgraph is evaluated at the missing frames, where the nodes remember their input, and then
 * once more at the original frame, which finds everything it needs.
 *
 * - Only the frames a node is going to output are kept: the one it asked for, and the frames
 *   between that and the evaluated frame when it follows the playhead (`Scene Time - 5`). The
 *   latter are remembered while they are evaluated anyway, so that case costs nothing extra.
 *
 * - The geometries are stored per node and per depsgraph: two nodes in one modifier never see
 *   each other's geometry, and a render computes what it needs without the viewport.
 * - A frame is the time of the depsgraph (what the Scene Time node outputs) and can be fractional.
 * - Simulations in front of the node are stepped up to the missing frame.
 */

#include <atomic>
#include <memory>

#include "BLI_map.hh"
#include "BLI_mutex.hh"
#include "BLI_set.hh"
#include "BLI_vector.hh"

#include "BKE_geometry_set.hh"

#include "NOD_geometry_nodes_lazy_function.hh"

namespace blender {

struct Depsgraph;
struct ModifierEvalContext;
struct NodesModifierData;
struct Object;

namespace time_shift {

struct EntryKey {
  /** Nested node id of the Time Shift node, see #bNestedNodeRef. */
  int32_t node_id;
  /** See #frame_to_key. */
  int64_t frame;

  uint64_t hash() const
  {
    return get_default_hash(node_id, frame);
  }
  friend bool operator==(const EntryKey &a, const EntryKey &b) = default;
};

struct Entry {
  bke::GeometrySet geometry;
  int64_t bytes = 0;
};

/** What a node asked for in the last evaluation that was not done to compute another frame. */
struct NodeState {
  bool is_set = false;
  int64_t evaluated_frame = 0;
  int64_t requested_frame = 0;
};

/** What the Time Shift nodes of one modifier have seen in one depsgraph. */
struct Store {
  Map<EntryKey, Entry> entries;
  Map<int32_t, NodeState> node_states;
  /** Frames that were asked for and are not used yet. They are not removed to free memory. */
  Set<int64_t> protected_frames;
};

}  // namespace time_shift

/** Shared between the original modifier and its evaluated copies. */
struct NodesModifierTimeShiftCache {
  mutable Mutex mutex;
  Map<const Depsgraph *, time_shift::Store> stores;
  /**
   * Changes whenever an evaluation at a frame that was evaluated before can give another result,
   * i.e. after missing frames were computed and when the stored geometries are discarded.
   */
  uint64_t revision = 1;
  uint32_t output_topology_hash = 0;
  /** Unlinked Frame inputs in the root tree by node identifier, to tell a change of them apart
   * from other changes to the node tree. */
  Map<int32_t, float> last_frame_socket_values;
  /** Time it took to compute the frames a node asked for the last time, by nested node id. */
  Map<int32_t, int64_t> fill_time_ns;

  ~NodesModifierTimeShiftCache();
};

namespace time_shift {

/** Frames are compared with this precision (1/1024 of a frame). */
int64_t frame_to_key(float frame);

/**
 * True while the depsgraph is evaluated at other frames to compute what Time Shift nodes asked
 * for, including the final evaluation at the original frame.
 */
bool is_filling(const Depsgraph *depsgraph);

/** Discard all stored geometries of the modifier. */
void wipe(NodesModifierTimeShiftCache &cache);

/**
 * Which Time Shift nodes of a modifier feed into which other ones, by nested node id. Only filled
 * when the modifier has more than one of them.
 */
struct TreeAnalysis {
  Map<int32_t, Vector<int32_t>> upstream;
  Map<int32_t, Vector<int32_t>> downstream;
  /** Number of Time Shift nodes, including the ones in nested groups. */
  int nodes_num = 0;
};

class ModifierParams : public nodes::GeoNodesTimeShiftParams {
 private:
  std::shared_ptr<NodesModifierTimeShiftCache> cache_;
  Depsgraph *depsgraph_ = nullptr;
  Object *object_orig_ = nullptr;
  float ctime_ = 0.0f;
  int64_t ctime_key_ = 0;
  bool filling_ = false;
  TreeAnalysis analysis_;

  mutable Mutex missed_mutex_;
  /** Nodes that did not find their frame in this evaluation. */
  mutable Set<int32_t> missed_nodes_;
  mutable std::atomic<bool> had_miss_ = false;

 public:
  /** Also discards the stored geometries when the modifier was changed by the user. */
  ModifierParams(NodesModifierData &nmd, const ModifierEvalContext &ctx);

  bool evaluate(int id,
                float frame,
                bke::GeometrySet current_geometry,
                bke::GeometrySet &r_shifted_geometry) const override;

  /** A node passed its input through in this evaluation, the result is not final. */
  bool had_miss() const
  {
    return had_miss_.load(std::memory_order_relaxed);
  }

  /** Zero when the modifier has no Time Shift nodes, see #NodesModifierTimeShiftCache::revision. */
  uint64_t revision() const;
};

}  // namespace time_shift
}  // namespace blender
