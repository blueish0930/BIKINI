/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#include "NOD_geometry_nodes_lazy_function.hh"
#include "NOD_geometry_nodes_output_cache.hh"

#include <chrono>
#include <optional>

#include "BKE_compute_contexts.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_socket_value.hh"

#include "FN_lazy_function_execute.hh"

#include "BLT_translation.hh"

#include "BLI_array_utils.hh"

#include "DEG_depsgraph_query.hh"

#include "FN_lazy_function_graph_executor.hh"

namespace blender::nodes {

using bke::SocketValueVariant;

/**
 * Wraps the execution of a repeat loop body. The purpose is to setup the correct #ComputeContext
 * inside of the loop body. This is necessary to support correct logging inside of a repeat zone.
 * An alternative would be to use a separate `LazyFunction` for every iteration, but that would
 * have higher overhead.
 */
class RepeatBodyNodeExecuteWrapper : public lf::GraphExecutorNodeExecuteWrapper {
 public:
  const bNode *repeat_output_bnode_ = nullptr;
  VectorSet<lf::FunctionNode *> *lf_body_nodes_ = nullptr;

  void execute_node(const lf::FunctionNode &node,
                    lf::Params &params,
                    const lf::Context &context) const override
  {
    GeoNodesUserData &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
    const int iteration = lf_body_nodes_->index_of_try(const_cast<lf::FunctionNode *>(&node));
    const LazyFunction &fn = node.function();
    if (iteration == -1) {
      /* The node is not a loop body node, just execute it normally. */
      fn.execute(params, context);
      return;
    }

    /* Setup context for the loop body evaluation. */
    bke::RepeatZoneComputeContext body_compute_context{
        user_data.compute_context, *repeat_output_bnode_, iteration};
    GeoNodesUserData body_user_data = user_data;
    body_user_data.compute_context = &body_compute_context;
    body_user_data.verbose_log = should_log_verbose_in_context(user_data,
                                                               body_compute_context.hash());

    GeoNodesLocalUserData body_local_user_data{body_user_data};
    lf::Context body_context{context.storage, &body_user_data, &body_local_user_data};
    fn.execute(params, body_context);
  }
};

/**
 * Knows which iterations of the loop evaluation have side effects.
 */
class RepeatZoneSideEffectProvider : public lf::GraphExecutorSideEffectProvider {
 public:
  const bNode *repeat_output_bnode_ = nullptr;
  Span<lf::FunctionNode *> lf_body_nodes_;

  Vector<const lf::FunctionNode *> get_nodes_with_side_effects(
      const lf::Context &context) const override
  {
    GeoNodesUserData &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
    const GeoNodesCallData &call_data = *user_data.call_data;
    if (!call_data.side_effect_nodes) {
      return {};
    }
    const ComputeContextHash &context_hash = user_data.compute_context->hash();
    const Span<int> iterations_with_side_effects =
        call_data.side_effect_nodes->iterations_by_iteration_zone.lookup(
            {context_hash, repeat_output_bnode_->identifier});

    Vector<const lf::FunctionNode *> lf_nodes;
    for (const int i : iterations_with_side_effects) {
      if (i >= 0 && i < lf_body_nodes_.size()) {
        lf_nodes.append(lf_body_nodes_[i]);
      }
    }
    return lf_nodes;
  }
};

/**
 * Select between two #SocketValueVariant inputs based on a single-value boolean condition.
 * Only the selected input is requested so later repeat bodies stay unevaluated after Break.
 */
class LazyFunctionForValueSwitch : public LazyFunction {
 public:
  LazyFunctionForValueSwitch()
  {
    debug_name_ = "Repeat Break Value Switch";
    inputs_.append_as("Condition", CPPType::get<SocketValueVariant>());
    inputs_.append_as("False", CPPType::get<SocketValueVariant>(), lf::ValueUsage::Maybe);
    inputs_.append_as("True", CPPType::get<SocketValueVariant>(), lf::ValueUsage::Maybe);
    outputs_.append_as("Value", CPPType::get<SocketValueVariant>());
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    const SocketValueVariant *condition_variant =
        params.try_get_input_data_ptr_or_request<SocketValueVariant>(0);
    if (condition_variant == nullptr) {
      return;
    }
    const bool condition = condition_variant->copy_as<bool>();
    const int input_to_forward = condition ? 2 : 1;
    const int input_to_ignore = condition ? 1 : 2;

    params.set_input_unused(input_to_ignore);
    void *value_to_forward = params.try_get_input_data_ptr_or_request(input_to_forward);
    if (value_to_forward == nullptr) {
      return;
    }

    const CPPType &type = *outputs_[0].type;
    void *output_ptr = params.get_output_data_ptr(0);
    type.move_construct(value_to_forward, output_ptr);
    params.output_set(0);
  }
};

/**
 * Combine border-link usage from the current iteration with remaining iterations.
 * When Break is true, remaining iterations are not requested.
 * result = current_usage || (!break && rest)
 */
class LazyFunctionForBreakAwareUsage : public LazyFunction {
 public:
  LazyFunctionForBreakAwareUsage()
  {
    debug_name_ = "Repeat Break Aware Usage";
    inputs_.append_as("Current", CPPType::get<bool>(), lf::ValueUsage::Maybe);
    inputs_.append_as("Break", CPPType::get<SocketValueVariant>());
    inputs_.append_as("Rest", CPPType::get<bool>(), lf::ValueUsage::Maybe);
    outputs_.append_as("Usage", CPPType::get<bool>());
  }

  void execute_impl(lf::Params &params, const lf::Context & /*context*/) const override
  {
    const SocketValueVariant *break_variant =
        params.try_get_input_data_ptr_or_request<SocketValueVariant>(1);
    if (break_variant == nullptr) {
      return;
    }
    const bool should_break = break_variant->copy_as<bool>();
    if (should_break) {
      params.set_input_unused(2);
      const bool *current = params.try_get_input_data_ptr_or_request<bool>(0);
      if (current == nullptr) {
        return;
      }
      params.set_output(0, *current);
      return;
    }

    const bool *current = params.try_get_input_data_ptr_or_request<bool>(0);
    const bool *rest = params.try_get_input_data_ptr_or_request<bool>(2);
    if (current == nullptr || rest == nullptr) {
      return;
    }
    params.set_output(0, *current || *rest);
  }
};

struct RepeatEvalStorage {
  LinearAllocator<> allocator;
  VectorSet<lf::FunctionNode *> lf_body_nodes;
  lf::Graph graph;
  std::optional<LazyFunctionForValueSwitch> value_switch_function;
  std::optional<LazyFunctionForBreakAwareUsage> break_aware_usage_function;
  std::optional<RepeatZoneSideEffectProvider> side_effect_provider;
  std::optional<RepeatBodyNodeExecuteWrapper> body_execute_wrapper;
  std::optional<lf::GraphExecutor> graph_executor;
  Array<SocketValueVariant> index_values;
  void *graph_executor_storage = nullptr;
  bool multi_threading_enabled = false;
  Vector<int> input_index_map;
  Vector<int> output_index_map;
  /** Wall time of every execute_impl in this eval (graph build + lazy passes). */
  int64_t accumulated_cook_ns = 0;
  /** Output-cache decision and captured outputs of this evaluation. */
  OutputCacheSession cache_session;
};

class LazyFunctionForRepeatZone : public LazyFunction {
 private:
  const bNodeTree &btree_;
  const bke::bNodeTreeZone &zone_;
  const bNode &repeat_output_bnode_;
  const ZoneBuildInfo &zone_info_;
  const ZoneBodyFunction &body_fn_;

 public:
  LazyFunctionForRepeatZone(const bNodeTree &btree,
                            const bke::bNodeTreeZone &zone,
                            ZoneBuildInfo &zone_info,
                            const ZoneBodyFunction &body_fn)
      : btree_(btree),
        zone_(zone),
        repeat_output_bnode_(*zone.output_node()),
        zone_info_(zone_info),
        body_fn_(body_fn)
  {
    debug_name_ = "Repeat Zone";

    initialize_zone_wrapper(zone, zone_info, body_fn, true, inputs_, outputs_);
    /* Iterations input is always used. */
    inputs_[zone_info.indices.inputs.main[0]].usage = lf::ValueUsage::Used;
  }

  void *init_storage(LinearAllocator<> &allocator) const override
  {
    return allocator.construct<RepeatEvalStorage>().release();
  }

  void destruct_storage(void *storage) const override
  {
    RepeatEvalStorage *s = static_cast<RepeatEvalStorage *>(storage);
    if (s->graph_executor_storage) {
      s->graph_executor->destruct_storage(s->graph_executor_storage);
    }
    std::destroy_at(s);
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const override
  {
    auto &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
    auto &local_user_data = *static_cast<GeoNodesLocalUserData *>(context.local_user_data);

    const int iterations_usage_index = zone_info_.indices.outputs.input_usages[0];
    if (!params.output_was_set(iterations_usage_index)) {
      /* The iterations input is always used. */
      params.set_output(iterations_usage_index, true);
    }

    const bool zone_has_inner_side_effects = [&]() {
      if (user_data.call_data == nullptr || user_data.call_data->side_effect_nodes == nullptr ||
          user_data.compute_context == nullptr)
      {
        return false;
      }
      const Span<int> iterations_with_side_effects =
          user_data.call_data->side_effect_nodes->iterations_by_iteration_zone.lookup(
              {user_data.compute_context->hash(), repeat_output_bnode_.identifier});
      return !iterations_with_side_effects.is_empty();
    }();

    RepeatEvalStorage &eval_storage = *static_cast<RepeatEvalStorage *>(context.storage);
    const OutputCacheSite cache_site{&btree_, &repeat_output_bnode_, zone_.input_node()};
    OutputCacheSession &cache_session = eval_storage.cache_session;
    if (zone_has_inner_side_effects) {
      /* Inner viewers need the zone body. Restoring then executing would set outputs twice. */
      cache_session.status = OutputCacheSession::Status::Disabled;
    }
    switch (geometry_nodes_output_cache_restore(context, params, cache_site, cache_session)) {
      case OutputCacheLookup::Hit:
      case OutputCacheLookup::Pending:
        return;
      case OutputCacheLookup::Miss:
      case OutputCacheLookup::Disabled:
        break;
    }

    const ScopedNodeTimer node_timer{context, repeat_output_bnode_};

    const NodeGeometryRepeatOutput &node_storage = *static_cast<const NodeGeometryRepeatOutput *>(
        repeat_output_bnode_.storage);

    if (!eval_storage.graph_executor) {
      /* Create the execution graph in the first evaluation. The GraphExecutor storage must
       * survive later execute_impl calls in this same eval (lazy input requests). */
      this->initialize_execution_graph(
          params, eval_storage, node_storage, user_data, local_user_data);
    }

    GeometryNodesOutputCaptureParams capture{*this, params};
    lf::RemappedParams eval_graph_params{*eval_storage.graph_executor,
                                         capture,
                                         eval_storage.input_index_map,
                                         eval_storage.output_index_map,
                                         eval_storage.multi_threading_enabled};
    lf::Context eval_graph_context{
        eval_storage.graph_executor_storage, context.user_data, context.local_user_data};
    eval_storage.graph_executor->execute(eval_graph_params, eval_graph_context);

    eval_storage.accumulated_cook_ns += node_timer.elapsed_ns();
    const int64_t cook_ns = eval_storage.accumulated_cook_ns;

    int64_t frozen_ns = cook_ns;
    geometry_nodes_output_cache_store(
        context, params, cache_site, cache_session, std::move(capture.captured()), frozen_ns);
  }

  /**
   * Generate a lazy-function graph that contains the loop body (`body_fn_`) as many times
   * as there are iterations. Since this graph depends on the number of iterations, it can't be
   * reused in general. We could consider caching a version of this graph per number of iterations,
   * but right now that doesn't seem worth it. In practice, it takes much less time to create the
   * graph than to execute it (for intended use cases of this generic implementation, more special
   * case repeat loop evaluations could be implemented separately).
   *
   * Body main outputs are `[Break] + items`. Break is loop control (not carried state). Item
   * values chain between iterations. Nested value-switches select the first body whose Break is
   * true (or the last body if none break), so later bodies stay lazy-unrequested after an early
   * exit.
   */
  void initialize_execution_graph(lf::Params &params,
                                  RepeatEvalStorage &eval_storage,
                                  const NodeGeometryRepeatOutput &node_storage,
                                  GeoNodesUserData &user_data,
                                  GeoNodesLocalUserData &local_user_data) const
  {
    const int num_repeat_items = node_storage.items_num;
    const int num_border_links = body_fn_.indices.inputs.border_links.size();

    /* Body main outputs: index 0 is Break, remaining are carried items. */
    BLI_assert(body_fn_.indices.outputs.main.size() == num_repeat_items + 1);
    const int body_break_output = 0;
    const int body_items_output_offset = 1;

    /* Number of iterations to evaluate. */
    const int iterations = std::max<int>(
        0,
        params.get_input<SocketValueVariant>(zone_info_.indices.inputs.main[0])
            .ensure_type<int>());

    if (iterations >= 10) {
      /* Constructing and running the repeat zone has some overhead so that it's probably worth
       * trying to do something else in the meantime already. */
      lazy_threading::send_hint();
    }

    /* Show a warning when the inspection index is out of range. */
    if (node_storage.inspection_index > 0) {
      if (node_storage.inspection_index >= iterations) {
        if (eval_log::NodeTreeLogger *tree_logger = local_user_data.try_get_tree_logger(user_data))
        {
          tree_logger->node_warnings.append(
              *tree_logger->allocator,
              {repeat_output_bnode_.identifier,
               {NodeWarningType::Info, N_("Inspection index is out of range")}});
        }
      }
    }

    /* Take iterations input into account. */
    const int main_inputs_offset = 1;
    const int body_inputs_offset = 1;

    lf::Graph &lf_graph = eval_storage.graph;

    Vector<lf::GraphInputSocket *> lf_inputs;
    Vector<lf::GraphOutputSocket *> lf_outputs;

    for (const int i : inputs_.index_range()) {
      const lf::Input &input = inputs_[i];
      lf_inputs.append(&lf_graph.add_input(*input.type, this->input_name(i)));
    }
    for (const int i : outputs_.index_range()) {
      const lf::Output &output = outputs_[i];
      lf_outputs.append(&lf_graph.add_output(*output.type, this->output_name(i)));
    }

    /* Create body nodes. */
    VectorSet<lf::FunctionNode *> &lf_body_nodes = eval_storage.lf_body_nodes;
    for ([[maybe_unused]] const int i : IndexRange(iterations)) {
      lf::FunctionNode &lf_node = lf_graph.add_function(*body_fn_.function);
      lf_body_nodes.add_new(&lf_node);
    }

    eval_storage.value_switch_function.emplace();
    eval_storage.break_aware_usage_function.emplace();

    const bool use_index_values = zone_.input_node()->output_socket(0).is_directly_linked();

    if (use_index_values) {
      eval_storage.index_values.reinitialize(iterations);
      threading::parallel_for(IndexRange(iterations), 1024, [&](const IndexRange range) {
        for (const int i : range) {
          eval_storage.index_values[i].emplace<int>(i);
        }
      });
    }

    /* Handle body nodes one by one. */
    static const SocketValueVariant static_unused_index{-1};
    for (const int iter_i : lf_body_nodes.index_range()) {
      lf::FunctionNode &lf_node = *lf_body_nodes[iter_i];
      const SocketValueVariant *index_value = use_index_values ?
                                                  &eval_storage.index_values[iter_i] :
                                                  &static_unused_index;
      lf_node.input(body_fn_.indices.inputs.main[0]).set_default_value(index_value);
      for (const int i : IndexRange(num_border_links)) {
        lf_graph.add_link(*lf_inputs[zone_info_.indices.inputs.border_links[i]],
                          lf_node.input(body_fn_.indices.inputs.border_links[i]));
      }

      /* Handle reference sets. */
      for (const auto &item : body_fn_.indices.inputs.reference_sets.items()) {
        lf_graph.add_link(*lf_inputs[zone_info_.indices.inputs.reference_sets.lookup(item.key)],
                          lf_node.input(item.value));
      }
    }

    static bool static_true = true;
    static bool static_false = false;

    /* Handle body nodes pair-wise: chain only carried items (not Break). */
    for (const int iter_i : lf_body_nodes.index_range().drop_back(1)) {
      lf::FunctionNode &lf_node = *lf_body_nodes[iter_i];
      lf::FunctionNode &lf_next_node = *lf_body_nodes[iter_i + 1];
      /* Always evaluate Break on intermediate bodies so the switch cascade can early-exit. */
      lf_node.input(body_fn_.indices.inputs.output_usages[body_break_output])
          .set_default_value(&static_true);
      for (const int i : IndexRange(num_repeat_items)) {
        lf_graph.add_link(
            lf_node.output(body_fn_.indices.outputs.main[i + body_items_output_offset]),
            lf_next_node.input(body_fn_.indices.inputs.main[i + body_inputs_offset]));
        /* TODO: Add back-link after being able to check for cyclic dependencies. */
        // lf_graph.add_link(lf_next_node.output(body_fn_.indices.outputs.input_usages[i]),
        //                   lf_node.input(body_fn_.indices.inputs.output_usages[i]));
        lf_node.input(body_fn_.indices.inputs.output_usages[i + body_items_output_offset])
            .set_default_value(&static_true);
      }
    }

    /* Border link usages: only iterations that actually run contribute (Break-aware cascade). */
    if (iterations > 0) {
      for (const int border_i : IndexRange(num_border_links)) {
        lf::OutputSocket *usage_socket =
            &lf_body_nodes.as_span().last()->output(
                body_fn_.indices.outputs.border_link_usages[border_i]);
        for (int iter_i = iterations - 2; iter_i >= 0; iter_i--) {
          lf::FunctionNode &lf_node = *lf_body_nodes[iter_i];
          lf::FunctionNode &lf_combine = lf_graph.add_function(
              *eval_storage.break_aware_usage_function);
          lf_graph.add_link(lf_node.output(body_fn_.indices.outputs.border_link_usages[border_i]),
                            lf_combine.input(0));
          lf_graph.add_link(lf_node.output(body_fn_.indices.outputs.main[body_break_output]),
                            lf_combine.input(1));
          lf_graph.add_link(*usage_socket, lf_combine.input(2));
          usage_socket = &lf_combine.output(0);
        }
        lf_graph.add_link(*usage_socket,
                          *lf_outputs[zone_info_.indices.outputs.border_link_usages[border_i]]);
      }
    }
    else {
      for (const int i : IndexRange(num_border_links)) {
        lf_outputs[zone_info_.indices.outputs.border_link_usages[i]]->set_default_value(
            &static_false);
      }
    }

    if (iterations > 0) {
      {
        /* Link first body node to input/output nodes. */
        lf::FunctionNode &lf_first_body_node = *lf_body_nodes[0];
        for (const int i : IndexRange(num_repeat_items)) {
          lf_graph.add_link(
              *lf_inputs[zone_info_.indices.inputs.main[i + main_inputs_offset]],
              lf_first_body_node.input(body_fn_.indices.inputs.main[i + body_inputs_offset]));
          lf_graph.add_link(
              lf_first_body_node.output(
                  body_fn_.indices.outputs.input_usages[i + body_inputs_offset]),
              *lf_outputs[zone_info_.indices.outputs.input_usages[i + main_inputs_offset]]);
        }
      }
      {
        /* Link item outputs through a Break-aware switch cascade so early exit skips later
         * bodies. From the last iteration backwards: if Break is true, take this iteration's
         * items; otherwise take the result of later iterations. */
        Array<lf::OutputSocket *> item_result_sockets(num_repeat_items);
        lf::FunctionNode &lf_last_body_node = *lf_body_nodes.as_span().last();
        /* Last body does not need Break for the cascade. */
        lf_last_body_node.input(body_fn_.indices.inputs.output_usages[body_break_output])
            .set_default_value(&static_false);
        for (const int i : IndexRange(num_repeat_items)) {
          item_result_sockets[i] = &lf_last_body_node.output(
              body_fn_.indices.outputs.main[i + body_items_output_offset]);
          lf_graph.add_link(
              *lf_inputs[zone_info_.indices.inputs.output_usages[i]],
              lf_last_body_node.input(
                  body_fn_.indices.inputs.output_usages[i + body_items_output_offset]));
        }
        for (int iter_i = iterations - 2; iter_i >= 0; iter_i--) {
          lf::FunctionNode &lf_node = *lf_body_nodes[iter_i];
          for (const int i : IndexRange(num_repeat_items)) {
            lf::FunctionNode &lf_switch = lf_graph.add_function(
                *eval_storage.value_switch_function);
            lf_graph.add_link(lf_node.output(body_fn_.indices.outputs.main[body_break_output]),
                              lf_switch.input(0));
            lf_graph.add_link(*item_result_sockets[i], lf_switch.input(1));
            lf_graph.add_link(
                lf_node.output(body_fn_.indices.outputs.main[i + body_items_output_offset]),
                lf_switch.input(2));
            item_result_sockets[i] = &lf_switch.output(0);
          }
        }
        for (const int i : IndexRange(num_repeat_items)) {
          lf_graph.add_link(*item_result_sockets[i],
                            *lf_outputs[zone_info_.indices.outputs.main[i]]);
        }
      }
    }
    else {
      /* There are no iterations, just link the input directly to the output. */
      for (const int i : IndexRange(num_repeat_items)) {
        lf_graph.add_link(*lf_inputs[zone_info_.indices.inputs.main[i + main_inputs_offset]],
                          *lf_outputs[zone_info_.indices.outputs.main[i]]);
        lf_graph.add_link(
            *lf_inputs[zone_info_.indices.inputs.output_usages[i]],
            *lf_outputs[zone_info_.indices.outputs.input_usages[i + main_inputs_offset]]);
      }
    }

    lf_outputs[zone_info_.indices.outputs.input_usages[0]]->set_default_value(&static_true);

    /* The graph is ready, update the node indices which are required by the executor. */
    lf_graph.update_node_indices();

    // std::cout << "\n\n" << lf_graph.to_dot() << "\n\n";

    /* Create a mapping from parameter indices inside of this graph to parameters of the repeat
     * zone. The main complexity below stems from the fact that the iterations input is handled
     * outside of this graph. */
    eval_storage.output_index_map.reinitialize(outputs_.size() - 1);
    eval_storage.input_index_map.resize(inputs_.size() - 1);
    array_utils::fill_index_range<int>(eval_storage.input_index_map, 1);

    Vector<const lf::GraphInputSocket *> lf_graph_inputs = lf_inputs.as_span().drop_front(1);

    const int iteration_usage_index = zone_info_.indices.outputs.input_usages[0];
    array_utils::fill_index_range<int>(
        eval_storage.output_index_map.as_mutable_span().take_front(iteration_usage_index));
    array_utils::fill_index_range<int>(
        eval_storage.output_index_map.as_mutable_span().drop_front(iteration_usage_index),
        iteration_usage_index + 1);

    Vector<const lf::GraphOutputSocket *> lf_graph_outputs = lf_outputs.as_span().take_front(
        iteration_usage_index);
    lf_graph_outputs.extend(lf_outputs.as_span().drop_front(iteration_usage_index + 1));

    eval_storage.body_execute_wrapper.emplace();
    eval_storage.body_execute_wrapper->repeat_output_bnode_ = &repeat_output_bnode_;
    eval_storage.body_execute_wrapper->lf_body_nodes_ = &lf_body_nodes;
    eval_storage.side_effect_provider.emplace();
    eval_storage.side_effect_provider->repeat_output_bnode_ = &repeat_output_bnode_;
    eval_storage.side_effect_provider->lf_body_nodes_ = lf_body_nodes;

    eval_storage.graph_executor.emplace(lf_graph,
                                        std::move(lf_graph_inputs),
                                        std::move(lf_graph_outputs),
                                        nullptr,
                                        &*eval_storage.side_effect_provider,
                                        &*eval_storage.body_execute_wrapper);
    eval_storage.graph_executor_storage = eval_storage.graph_executor->init_storage(
        eval_storage.allocator);

    /* Log graph for debugging purposes. */
    const bNodeTree &btree_orig = *DEG_get_original(&btree_);
    if (btree_orig.runtime->logged_zone_graphs) {
      std::lock_guard lock{btree_orig.runtime->logged_zone_graphs->mutex};
      btree_orig.runtime->logged_zone_graphs->graph_by_zone_id.lookup_or_add_cb(
          repeat_output_bnode_.identifier, [&]() { return lf_graph.to_dot(); });
    }
  }

  std::string input_name(const int i) const override
  {
    return zone_wrapper_input_name(zone_info_, zone_, inputs_, i);
  }

  std::string output_name(const int i) const override
  {
    return zone_wrapper_output_name(zone_info_, zone_, outputs_, i);
  }
};

LazyFunction &build_repeat_zone_lazy_function(ResourceScope &scope,
                                              const bNodeTree &btree,
                                              const bke::bNodeTreeZone &zone,
                                              ZoneBuildInfo &zone_info,
                                              const ZoneBodyFunction &body_fn)
{
  return scope.construct<LazyFunctionForRepeatZone>(btree, zone, zone_info, body_fn);
}

}  // namespace blender::nodes
