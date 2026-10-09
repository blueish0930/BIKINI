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
 * stored. Cache hits never request lazy-function data inputs, so unchanged upstream is skipped.
 *
 * An entry is valid while the node's *state token* is unchanged. The token is built before any
 * input is computed, from everything the node's result can depend on:
 * - Edits of the node or anything upstream of it in its tree (per-node generations).
 * - The modifier inputs it is downstream of (content identity of the input geometry, values of
 *   the modifier panel), or the state of the calling group node when nested.
 * - Data-blocks, scene time, cameras and animated node values, for nodes downstream of a node
 *   that reads them.
 *
 * Besides the token, an entry remembers which anonymous attributes its consumers asked for. A
 * consumer that needs more than what was cooked misses.
 */

#include <memory>
#include <string>

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
struct IDProperty;
struct Object;
struct bNode;
struct bNodeTree;

namespace blender::bke {
struct GeometrySet;
}

namespace blender::nodes {

struct GeoNodesUserData;
class GeometryNodesOutputCache;
struct GeometryNodesOutputCacheEvalState;

bool socket_value_is_output_cacheable(const bke::SocketValueVariant &value);

/** Session uid of the original node tree (evaluated copies map back to original). */
uint32_t geometry_nodes_output_cache_tree_uid(const bNodeTree &tree);

/** True when the only cache dirty-seeds for this tree are Viewer nodes (preview rewiring). */
bool geometry_nodes_output_cache_only_viewer_seeds(const bNodeTree &ntree);

/** Copy current dirty-seed node identifiers (does not consume them). */
void geometry_nodes_output_cache_copy_dirty_seeds(const bNodeTree &ntree, Vector<int32_t> &r_seeds);

/** Hash of a node's input sockets: linked-or-not and unlinked default values. */
uint64_t geometry_nodes_output_cache_hash_node_inputs(const bNode &node);

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

  /** One entry per node per compute context. Validity is decided by #StateToken, not the key. */
  struct Key {
    uint32_t tree_session_uid = 0;
    int32_t node_id = 0;
    ComputeContextHash context_hash;

    uint64_t hash() const
    {
      return get_default_hash(tree_session_uid, node_id, context_hash.v1, context_hash.v2);
    }
    friend bool operator==(const Key &a, const Key &b) = default;
  };

  /**
   * Everything a node's result depends on, split by source so a miss can say what changed.
   * Parts the node does not depend on stay zero.
   */
  struct StateToken {
    /** Generation of the node and its tree, and the node's own unlinked socket values. */
    uint64_t tree = 0;
    /** Modifier inputs (root tree) or the calling node's state (nested). */
    uint64_t context = 0;
    /** Referenced data-blocks: objects, collections, images, … */
    uint64_t ids = 0;
    uint64_t time = 0;
    uint64_t camera = 0;
    /** Values of animated or driven node properties. */
    uint64_t animation = 0;

    friend bool operator==(const StateToken &a, const StateToken &b) = default;
  };

  /**
   * What the consumers of a node asked it to provide beyond its socket values: which
   * attribute outputs are used and which anonymous attributes must survive on each geometry
   * output. Indexed by lazy-function input.
   */
  struct Requirements {
    Vector<std::pair<int, bool>> usages;
    Vector<std::pair<int, std::shared_ptr<Set<std::string>>>> reference_sets;
  };

  struct CachedSocket {
    /** Lazy-function output index. */
    int output_index = 0;
    bke::SocketValueVariant value;
  };

  struct Entry {
    StateToken state;
    Requirements requirements;
    Vector<CachedSocket> sockets;
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

  struct PrepareParams {
    const bNodeTree *root_tree = nullptr;
    const bke::GeometrySet *input_geometry = nullptr;
    const Object *self_object = nullptr;
    Depsgraph *depsgraph = nullptr;
    float ctime = 0.0f;
    /** Modifier input values (`ModifierData::system_properties`). */
    const IDProperty *properties = nullptr;
  };

  /**
   * Snapshot everything cached nodes can depend on for one evaluation. Returns null when no
   * node in the tree hierarchy has its cache enabled; the evaluation then skips the cache.
   *
   * The result belongs to the evaluation, not to the cache: the viewport and a render can
   * evaluate the same modifier at the same time.
   */
  std::shared_ptr<const GeometryNodesOutputCacheEvalState> prepare_for_eval(
      const PrepareParams &params);

  enum class Lookup {
    Miss,
    Hit,
  };

  /**
   * Restore cached SocketValueVariant outputs. Only #ValueUsage::Used sockets must be present;
   * unconnected #Maybe outputs are ignored so a node that never wrote them still hits.
   *
   * \param restore_usage_outputs: Also write `false` to bool outputs (zone and group input
   * usages). Off when the body still runs for its side effects and reports usages itself.
   * \param r_miss_reason: Static string describing a miss (debug output).
   */
  Lookup try_restore(const Key &key,
                     const StateToken &state,
                     const Requirements &requirements,
                     fn::lazy_function::Params &params,
                     bool restore_usage_outputs,
                     int64_t &r_exec_time_ns,
                     const char *&r_miss_reason) const;

  /**
   * Returns false when nothing was stored (empty output set).
   * On success, #r_frozen_ns is the kept first-complete-eval time for this node state.
   */
  bool store(const Key &key,
             const StateToken &state,
             Requirements requirements,
             Vector<CachedSocket> sockets,
             int64_t exec_time_ns,
             int64_t &r_frozen_ns);

  /** Remove every entry for this node so a recache cannot coexist with the old data. */
  void discard_node(uint32_t tree_session_uid, int32_t node_id);

  int debug_count_node(uint32_t tree_session_uid, int32_t node_id) const;

  /** Memoized content hashes of shared arrays, see the implementation. */
  struct ContentHasher;

 private:
  mutable Mutex mutex_;
  Map<Key, Entry> entries_;
  /** Frozen overlay time per node, kept until the node's tree state changes. */
  Map<NodeRef, int64_t> frozen_time_ns_;
  Map<NodeRef, uint64_t> frozen_time_state_;

  /** Serializes #prepare_for_eval, which owns #content_hasher_. */
  Mutex prepare_mutex_;
  std::unique_ptr<ContentHasher> content_hasher_;
};

/** Identifies the node whose outputs are cached at one call site. */
struct OutputCacheSite {
  /** Tree that owns #node (the evaluated copy). */
  const bNodeTree *tree = nullptr;
  /** Node that carries the cache toggle and keys the entry (for zones: the output node). */
  const bNode *node = nullptr;
  /** Node whose unlinked socket values feed the result, when not #node (zone input node). */
  const bNode *sockets_node = nullptr;
};

enum class OutputCacheLookup {
  /** The cache is not used for this node. Evaluate as usual, do not store. */
  Disabled,
  /** Inputs that describe what consumers need are not available yet. Return and wait. */
  Pending,
  /** Evaluate the node and store the result. */
  Miss,
  /** The outputs were restored. */
  Hit,
};

/**
 * The cache decision for one node in one evaluation. A node function can run several times per
 * evaluation while its inputs arrive, so nodes with lazy-function storage keep this there;
 * nodes that cook in a single pass can keep it on the stack.
 */
struct OutputCacheSession {
  enum class Status {
    Undecided,
    Disabled,
    Hit,
    Miss,
  };
  Status status = Status::Undecided;
  GeometryNodesOutputCache::Key key;
  GeometryNodesOutputCache::StateToken state;
  /** Read before the node runs: evaluating it may consume the inputs they come from. */
  GeometryNodesOutputCache::Requirements requirements;
  /** Outputs captured so far. They can be written in different passes. */
  Vector<GeometryNodesOutputCache::CachedSocket> captured;
};

/** True when the cache toggle of the site's node is on for this evaluation. */
bool geometry_nodes_output_cache_site_enabled(const GeoNodesUserData &user_data,
                                              const OutputCacheSite &site);

/**
 * Try to restore the outputs of a cached node. On #OutputCacheLookup::Hit with
 * #body_runs_for_side_effects off, unrequested inputs are marked unused and the hit is logged;
 * the caller only has to return.
 *
 * \param body_runs_for_side_effects: The caller evaluates the body afterwards for an inner
 * viewer. Usage outputs are then left to the body and inputs stay requestable.
 */
OutputCacheLookup geometry_nodes_output_cache_restore(const fn::lazy_function::Context &context,
                                                      fn::lazy_function::Params &params,
                                                      const OutputCacheSite &site,
                                                      OutputCacheSession &session,
                                                      bool body_runs_for_side_effects = false);

/**
 * Add the outputs captured in this pass and store the entry once every used output is there.
 * Logs the frozen overlay time and writes it to #exec_time_ns on success. Does nothing unless
 * the session's lookup was a miss.
 */
bool geometry_nodes_output_cache_store(const fn::lazy_function::Context &context,
                                       fn::lazy_function::Params &params,
                                       const OutputCacheSite &site,
                                       OutputCacheSession &session,
                                       Vector<GeometryNodesOutputCache::CachedSocket> &&captured,
                                       int64_t &exec_time_ns);

/**
 * Token for the compute context entered through the site's node (group body, closure body).
 * Nodes in there that depend on the context's inputs mix it into their state, so they miss
 * exactly when the calling node would. Never zero while the cache is in use.
 */
uint64_t geometry_nodes_output_cache_child_token(const GeoNodesUserData &user_data,
                                                 const OutputCacheSite &site);

/** Add outputs captured in a later execution pass to the ones captured before. */
void geometry_nodes_output_cache_merge_captured(
    Vector<GeometryNodesOutputCache::CachedSocket> &accumulated,
    Vector<GeometryNodesOutputCache::CachedSocket> &&captured);

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

/**
 * Runs a group body whose socket values already came from the cache: every SocketValueVariant
 * output reads as computed and unused, so the body only evaluates what its side-effect nodes
 * (an inner viewer) pull. Usage outputs and all inputs go to the wrapped params.
 */
class GeometryNodesOutputCacheSideEffectParams final : public fn::lazy_function::Params {
 private:
  fn::lazy_function::Params &base_;
  /** Receives a value the body writes although the output reads as unused. */
  Vector<std::unique_ptr<bke::SocketValueVariant>> discarded_;

  bool is_cached_output(const int index) const
  {
    return fn_.outputs()[index].type == &CPPType::get<bke::SocketValueVariant>();
  }

 public:
  GeometryNodesOutputCacheSideEffectParams(const fn::lazy_function::LazyFunction &fn,
                                           fn::lazy_function::Params &base)
      : Params(fn, true), base_(base)
  {
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
    if (this->is_cached_output(index)) {
      /* Not expected: the executor checks the usage first. Hand out valid storage anyway. */
      discarded_.append(std::make_unique<bke::SocketValueVariant>());
      bke::SocketValueVariant *value = discarded_.last().get();
      std::destroy_at(value);
      return value;
    }
    return base_.get_output_data_ptr(index);
  }
  void output_set_impl(const int index) override
  {
    if (this->is_cached_output(index)) {
      return;
    }
    if (!base_.output_was_set(index)) {
      base_.output_set(index);
    }
  }
  bool output_was_set_impl(const int index) const override
  {
    if (this->is_cached_output(index)) {
      return true;
    }
    return base_.output_was_set(index);
  }
  fn::lazy_function::ValueUsage get_output_usage_impl(const int index) const override
  {
    if (this->is_cached_output(index)) {
      return fn::lazy_function::ValueUsage::Unused;
    }
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
