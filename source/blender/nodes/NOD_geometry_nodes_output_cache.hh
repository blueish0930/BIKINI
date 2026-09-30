/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup nodes
 *
 * Bake-style in-memory cache of geometry-node outputs.
 *
 * Cacheable: geometry, single values, and lists. Fields / grids / closures / bundles are not
 * stored. Cache hits never request lazy-function inputs, so unchanged upstream is skipped.
 *
 * Invalidation is per-node + downstream (property edits, links, Object Info). The modifier's
 * geometry input changing forces a full re-evaluation of nodes that used the previous input
 * (separate cache worlds per input token so interleaved depsgraph evals do not wipe each other).
 */

#include "BLI_compute_context.hh"
#include "BLI_map.hh"
#include "BLI_mutex.hh"
#include "BLI_set.hh"
#include "BLI_span.hh"
#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "BKE_node_socket_value.hh"
#include "FN_lazy_function.hh"

struct Depsgraph;
struct Object;
struct bNode;
struct bNodeTree;

namespace blender::bke {
struct GeometrySet;
}

namespace blender::nodes {

class GeometryNodesOutputCache;

bool socket_value_is_output_cacheable(const bke::SocketValueVariant &value);

/** Session uid of the original node tree (evaluated copies map back to original). */
uint32_t geometry_nodes_output_cache_tree_uid(const bNodeTree &tree);

/** True when the only cache dirty-seeds for this tree are Viewer nodes (preview rewiring). */
bool geometry_nodes_output_cache_only_viewer_seeds(const bNodeTree &ntree);

/** Copy current dirty-seed node identifiers (does not consume them). */
void geometry_nodes_output_cache_copy_dirty_seeds(const bNodeTree &ntree, Vector<int32_t> &r_seeds);

/** Hash of a node's input sockets: linked-or-not and unlinked default values. */
uint64_t geometry_nodes_output_cache_hash_node_inputs(const bNode &node);
uint64_t geometry_nodes_output_cache_hash_value(const bke::SocketValueVariant &value);

/**
 * True when the user enabled in-memory output cache on this node (opt-in).
 * #cache (the modifier's table, may be null) is consulted so an eval/COW copy that
 * has not yet received the DNA flag still sees the original tree's toggle.
 */
bool geometry_nodes_output_cache_node_enabled(const bNode &node,
                                              const GeometryNodesOutputCache *cache = nullptr);

/** Record a node whose output may have changed (property, socket, link target, …). */
void geometry_nodes_output_cache_tag_node(bNodeTree *ntree, const bNode *node);

/** Drop every cached entry for this node (delete, disable cache, or force recache). */
void geometry_nodes_output_cache_discard_node(bNodeTree *ntree, const bNode *node);

/** Number of stored entries for this node across all modifier caches (tests / debugging). */
int geometry_nodes_output_cache_debug_count(const bNodeTree *ntree, const bNode *node);

/**
 * After a geometry node tree update, bump generations for tagged nodes and everything
 * downstream of them.
 */
void geometry_nodes_output_cache_propagate(bNodeTree &ntree);

void geometry_nodes_output_cache_log_hit(const fn::lazy_function::Context &context,
                                         int32_t node_id,
                                         int64_t exec_time_ns);

/** Mark a just-cooked node as frozen without adding a second timing sample. */
void geometry_nodes_output_cache_mark_frozen(const fn::lazy_function::Context &context,
                                             int32_t node_id);

/**
 * Overlay time for a cached node: always the first complete cook, even if this eval re-ran the
 * wrapper (Repeat graph already built, ~60ms). Replaces the live ScopedNodeTimer sample.
 */
void geometry_nodes_output_cache_log_frozen_overlay(const fn::lazy_function::Context &context,
                                                    int32_t node_id,
                                                    int64_t exec_time_ns);

/** After a cache hit, tell the executor not to pull inputs that were never requested. */
inline void geometry_nodes_output_cache_skip_unprovided_inputs(fn::lazy_function::Params &params,
                                                               const int inputs_num)
{
  for (int i = 0; i < inputs_num; i++) {
    if (params.try_get_input_data_ptr(i) == nullptr) {
      params.set_input_unused(i);
    }
  }
}

/** True when every Used SocketValueVariant output has been written (lazy request-only passes fail). */
inline bool geometry_nodes_output_cache_used_values_ready(const fn::lazy_function::Params &params,
                                                          const int outputs_num)
{
  namespace lf = fn::lazy_function;
  const Span<lf::Output> outputs = params.fn_.outputs();
  for (int i = 0; i < outputs_num; i++) {
    if (outputs[i].type != &CPPType::get<bke::SocketValueVariant>()) {
      continue;
    }
    if (params.get_output_usage(i) != lf::ValueUsage::Used) {
      continue;
    }
    if (!params.output_was_set(i)) {
      return false;
    }
  }
  return true;
}

class GeometryNodesOutputCache : NonCopyable, NonMovable {
 public:
  GeometryNodesOutputCache();
  ~GeometryNodesOutputCache();

  struct Key {
    uint32_t tree_session_uid = 0;
    int32_t node_id = 0;
    ComputeContextHash context_hash;
    /** Modifier geometry-input identity. Different inputs keep separate entries. */
    uint64_t input_token = 0;
    /** This node's input link topology + unlinked defaults (disconnect must miss). */
    uint64_t sockets_token = 0;
    /** Parent group/zone input identity so inner nodes miss when the caller’s inputs change. */
    uint64_t caller_token = 0;

    uint64_t hash() const
    {
      return get_default_hash(get_default_hash(tree_session_uid, node_id, context_hash.v1),
                              get_default_hash(context_hash.v2, input_token),
                              sockets_token,
                              caller_token);
    }
    friend bool operator==(const Key &a, const Key &b) = default;
  };

  struct CachedSocket {
    /** Lazy-function output index. */
    int output_index = 0;
    bke::SocketValueVariant value;
  };

  struct Entry {
    uint64_t node_gen = 0;
    uint64_t topology_gen = 0;
    uint64_t objects_token = 0;
    uint64_t camera_token = 0;
    float ctime = 0.0f;
    Vector<CachedSocket> sockets;
    int64_t exec_time_ns = 0;
  };

  struct Hit {
    bool hit = false;
    int64_t exec_time_ns = 0;
  };

  struct NodeRef {
    uint32_t tree_session_uid = 0;
    int32_t node_id = 0;
    uint64_t hash() const
    {
      return get_default_hash(tree_session_uid, node_id);
    }
    friend bool operator==(const NodeRef &a, const NodeRef &b) = default;
  };

 private:
  mutable Mutex mutex_;
  Map<Key, Entry> entries_;

  uint64_t current_input_token_ = 0;
  uint64_t current_objects_token_ = 0;
  uint64_t current_camera_token_ = 0;
  float current_ctime_ = 0.0f;
  uint64_t prev_input_token_ = 0;
  bool has_prev_input_token_ = false;

  Map<uint32_t, uint64_t> topology_gen_by_tree_;
  Map<uint32_t, Map<int32_t, uint64_t>> node_gen_by_tree_;
  Set<NodeRef> object_dependent_;
  Set<NodeRef> time_dependent_;
  Set<NodeRef> camera_dependent_;
  /** Nodes whose original (or eval) flag has NODE_OUTPUT_CACHE, snapshotted at prepare. */
  Set<NodeRef> enabled_nodes_;
  /** Frozen overlay time per node, kept across cache-key changes until the node gen bumps. */
  Map<NodeRef, int64_t> frozen_time_ns_;
  Map<NodeRef, uint64_t> frozen_time_gen_;

 public:
  void prepare_for_eval(const bNodeTree &root_tree,
                        const bke::GeometrySet &input_geometry,
                        const Object *self_object,
                        Depsgraph *depsgraph,
                        float ctime);

  /**
   * Restore cached SocketValueVariant outputs. Unset bool outputs are filled with false
   * (zone input-usage flags). Returns {hit, stored exec time}.
   *
   * Only #ValueUsage::Used sockets must be present; unconnected #Maybe outputs are ignored so a
   * node that never wrote them still hits.
   */
  Hit try_restore(const Key &key, fn::lazy_function::Params &params) const;

  /**
   * Returns false when nothing was stored (empty output set).
   * On success, #r_frozen_ns (if given) is the kept first-complete-eval time for this node gen.
   */
  bool store(const Key &key,
             Vector<CachedSocket> sockets,
             int64_t exec_time_ns,
             int64_t *r_frozen_ns = nullptr);

  /** Remove every entry for this node so a recache cannot coexist with the old data. */
  void discard_node(uint32_t tree_session_uid, int32_t node_id);

  int debug_count_node(uint32_t tree_session_uid, int32_t node_id) const;

  bool is_node_enabled(uint32_t tree_session_uid, int32_t node_id) const;
};

Vector<GeometryNodesOutputCache::CachedSocket> geometry_nodes_output_cache_collect_outputs(
    fn::lazy_function::Params &params, int outputs_num);

/**
 * Copy a cacheable SocketValueVariant from the buffer the callee constructed into.
 * Must run before #Params::output_set: the graph executor then forwards (moves) that buffer.
 */
inline void geometry_nodes_output_cache_capture_constructed(
    Vector<GeometryNodesOutputCache::CachedSocket> &captured,
    const Span<fn::lazy_function::Output> outputs,
    const int index,
    void *ptr)
{
  if (ptr == nullptr || index < 0 || index >= outputs.size()) {
    return;
  }
  if (outputs[index].type != &CPPType::get<bke::SocketValueVariant>()) {
    return;
  }
  const auto &value = *static_cast<const bke::SocketValueVariant *>(ptr);
  if (!socket_value_is_output_cacheable(value)) {
    return;
  }
  for (GeometryNodesOutputCache::CachedSocket &cached : captured) {
    if (cached.output_index == index) {
      cached.value = value;
      cached.value.ensure_owns_direct_data();
      return;
    }
  }
  GeometryNodesOutputCache::CachedSocket cached;
  cached.output_index = index;
  cached.value = value;
  cached.value.ensure_owns_direct_data();
  captured.append(std::move(cached));
}

/**
 * Forwards to another #Params, copying cacheable SocketValueVariant outputs at #output_set.
 *
 * GraphExecutor writes with get_output_data_ptr → construct → output_set. A second
 * get_output_data_ptr in output_set can allocate a fresh uninitialized buffer (the first
 * pointer was already handed to construct). Remember the construct pointer and copy from
 * that. After output_set the executor moves the value away, so post-execute reads are empty.
 */
class GeometryNodesOutputCaptureParams final : public fn::lazy_function::Params {
 private:
  fn::lazy_function::Params &base_;
  Vector<void *> constructed_ptrs_;
  Vector<GeometryNodesOutputCache::CachedSocket> captured_;
  Mutex mutex_;

 public:
  GeometryNodesOutputCaptureParams(const fn::lazy_function::LazyFunction &fn,
                                   fn::lazy_function::Params &base)
      : Params(fn, true),
        base_(base),
        constructed_ptrs_(fn.outputs().size(), nullptr)
  {
  }

  Vector<GeometryNodesOutputCache::CachedSocket> &captured()
  {
    return captured_;
  }

  void *try_get_input_data_ptr_impl(const int index) const override
  {
    return base_.try_get_input_data_ptr(index);
  }
  void *try_get_input_data_ptr_or_request_impl(const int index) override
  {
    return base_.try_get_input_data_ptr_or_request(index);
  }
  void *get_output_data_ptr_impl(const int index) override
  {
    void *ptr = base_.get_output_data_ptr(index);
    if (index >= 0 && index < constructed_ptrs_.size()) {
      constructed_ptrs_[index] = ptr;
    }
    return ptr;
  }
  void output_set_impl(const int index) override
  {
    void *ptr = nullptr;
    if (index >= 0 && index < constructed_ptrs_.size()) {
      ptr = constructed_ptrs_[index];
    }
    if (ptr == nullptr) {
      ptr = base_.get_output_data_ptr(index);
    }
    {
      std::lock_guard lock{mutex_};
      geometry_nodes_output_cache_capture_constructed(
          captured_, fn_.outputs(), index, ptr);
    }
    if (!base_.output_was_set(index)) {
      base_.output_set(index);
    }
  }
  bool output_was_set_impl(const int index) const override
  {
    return base_.output_was_set(index);
  }
  fn::lazy_function::ValueUsage get_output_usage_impl(const int index) const override
  {
    return base_.get_output_usage(index);
  }
  void set_input_unused_impl(const int index) override
  {
    base_.set_input_unused(index);
  }
  bool try_enable_multi_threading_impl() override
  {
    return base_.try_enable_multi_threading();
  }
};

}  // namespace blender::nodes
