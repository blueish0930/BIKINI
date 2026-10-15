/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup nodes
 */

#include "NOD_geo_foreach_geometry_element.hh"
#include "NOD_geometry_nodes_lazy_function.hh"
#include "NOD_geometry_nodes_output_cache.hh"

#include <chrono>

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_mask.hh"
#include "BLI_lazy_threading.hh"
#include "BLI_task.hh"

#include "BKE_anonymous_attribute_make.hh"
#include "BKE_compute_contexts.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_node_runtime.hh"
#include "BKE_node_socket_value.hh"
#include "BKE_pointcloud.hh"

#include "GEO_extract_elements.hh"
#include "GEO_foreach_geometry.hh"
#include "GEO_join_geometries.hh"
#include "GEO_mesh_copy_selection.hh"

#include "FN_lazy_function_graph_executor.hh"

#include "BLT_translation.hh"

#include "DEG_depsgraph_query.hh"

namespace blender::nodes {

using bke::AttrDomain;
using bke::AttributeAccessor;
using bke::GeometryComponent;
using bke::GeometrySet;
using bke::MutableAttributeAccessor;
using bke::SocketValueVariant;
using fn::Field;
using fn::GField;

class LazyFunctionForForeachGeometryElementZone;
struct ForeachGeometryElementEvalStorage;

/**
 * Name of the anonymous attribute that holds the values of a main or generation item. It only
 * depends on where the zone is evaluated, not on what the iterations compute.
 */
static std::string foreach_item_attribute_name(const GeoNodesUserData &user_data,
                                               const bNode &output_bnode,
                                               const int item_identifier)
{
  const char *self_name = "";
  if (const Object *self_object = user_data.call_data->self_object()) {
    self_name = self_object->id.name;
  }
  else if (user_data.call_data->root_ntree) {
    self_name = user_data.call_data->root_ntree->id.name;
  }
  return bke::hash_to_anonymous_attribute_name(
      self_name, user_data.compute_context->hash(), output_bnode.identifier, item_identifier);
}

/**
 * Passed to the graph that runs the iterations. Outputs the zone has provided itself already
 * (see #LazyFunctionForForeachGeometryElementZone::output_attribute_fields) read as computed and
 * unused there, so the graph neither computes nor writes them a second time.
 */
class ForeachAttributeFieldOutputsParams final : public lf::Params {
 private:
  lf::Params &base_;
  Span<bool> is_provided_output_;
  /** Receives a value the graph writes although the output reads as unused. */
  Vector<std::unique_ptr<SocketValueVariant>> discarded_;
  Mutex discarded_mutex_;

 public:
  ForeachAttributeFieldOutputsParams(const LazyFunction &fn,
                                     lf::Params &base,
                                     const Span<bool> is_provided_output)
      : Params(fn, true), base_(base), is_provided_output_(is_provided_output)
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
    if (is_provided_output_[index]) {
      /* Not expected: the executor checks the usage first. Hand out valid storage anyway. */
      std::lock_guard lock{discarded_mutex_};
      discarded_.append(std::make_unique<SocketValueVariant>());
      SocketValueVariant *value = discarded_.last().get();
      std::destroy_at(value);
      return value;
    }
    return base_.get_output_data_ptr(index);
  }
  void output_set_impl(const int index) override
  {
    if (is_provided_output_[index]) {
      return;
    }
    base_.output_set(index);
  }
  bool output_was_set_impl(const int index) const override
  {
    if (is_provided_output_[index]) {
      return true;
    }
    return base_.output_was_set(index);
  }
  lf::ValueUsage get_output_usage_impl(const int index) const override
  {
    if (is_provided_output_[index]) {
      return lf::ValueUsage::Unused;
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

struct ForeachElementComponentID {
  GeometryComponent::Type component_type;
  AttrDomain domain;
  std::optional<int> layer_index;
};

/**
 * The For Each Geometry Element can iterate over multiple components at the same time. That can
 * happen when the input geometry is e.g. a mesh and a pointcloud and we're iterating over points.
 *
 * This struct contains evaluation data for each component.
 */
struct ForeachElementComponent {
  ForeachElementComponentID id;
  /** Used for field evaluation on the output node. */
  std::optional<bke::GeometryFieldContext> field_context;
  std::optional<fn::FieldEvaluator> field_evaluator;
  /** Index values passed into each body node. */
  Array<SocketValueVariant> index_values;
  /** Evaluated input values passed into each body node. */
  Array<Array<SocketValueVariant>> item_input_values;
  /** Geometry for each iteration. */
  std::optional<Array<SocketValueVariant>> element_geometries;
  /**
   * Only used when iterating over groups: the elements of every iteration and the ID they
   * share.
   */
  IndexMaskMemory group_memory;
  Vector<IndexMask, 4> group_masks;
  Array<SocketValueVariant> group_id_values;
  /**
   * The set of body evaluation nodes that correspond to this component.
   * This indexes into `lf_body_nodes`.
   */
  IndexRange body_nodes_range;

  void emplace_field_context(const GeometrySet &geometry)
  {
    if (this->id.component_type == GeometryComponent::Type::GreasePencil &&
        ELEM(this->id.domain, AttrDomain::Point, AttrDomain::Curve))
    {
      this->field_context.emplace(
          *geometry.get_grease_pencil(), this->id.domain, *this->id.layer_index);
    }
    else {
      this->field_context.emplace(*geometry.get_component(this->id.component_type),
                                  this->id.domain);
    }
  }

  AttributeAccessor input_attributes() const
  {
    return *this->field_context->attributes();
  }

  MutableAttributeAccessor attributes_for_write(GeometrySet &geometry) const
  {
    if (this->id.component_type == GeometryComponent::Type::GreasePencil &&
        ELEM(this->id.domain, AttrDomain::Point, AttrDomain::Curve))
    {
      BLI_assert(this->id.layer_index.has_value());
      GreasePencil *grease_pencil = geometry.get_grease_pencil_for_write();
      const bke::greasepencil::Layer &layer = grease_pencil->layer(*this->id.layer_index);
      bke::greasepencil::Drawing *drawing = grease_pencil->get_eval_drawing(layer);
      BLI_assert(drawing);
      return drawing->strokes_for_write().attributes_for_write();
    }
    GeometryComponent &component = geometry.get_component_for_write(this->id.component_type);
    return *component.attributes_for_write();
  }
};

/**
 * A lazy-function that takes the result from all loop body evaluations and reduces them to the
 * final output of the entire zone.
 */
struct LazyFunctionForReduceForeachGeometryElement : public LazyFunction {
  const LazyFunctionForForeachGeometryElementZone &parent_;
  ForeachGeometryElementEvalStorage &eval_storage_;

 public:
  LazyFunctionForReduceForeachGeometryElement(
      const LazyFunctionForForeachGeometryElementZone &parent,
      ForeachGeometryElementEvalStorage &eval_storage);

  void execute_impl(lf::Params &params, const lf::Context &context) const override;

  void handle_main_items_and_geometry(lf::Params &params, const lf::Context &context) const;
  void handle_generation_items(lf::Params &params, const lf::Context &context) const;
  int handle_invalid_generation_items(lf::Params &params) const;
  void handle_generation_item_groups(lf::Params &params,
                                     const lf::Context &context,
                                     int first_valid_item_i) const;
  void handle_generation_items_group(lf::Params &params,
                                     const lf::Context &context,
                                     int geometry_item_i,
                                     IndexRange generation_items_range) const;
  bool handle_generation_items_group_lazyness(lf::Params &params,
                                              const lf::Context &context,
                                              int geometry_item_i,
                                              IndexRange generation_items_range) const;
};

/**
 * This is called whenever an evaluation node is entered. It sets up the compute context if the
 * node is a loop body node.
 */
class ForeachGeometryElementNodeExecuteWrapper : public lf::GraphExecutorNodeExecuteWrapper {
 public:
  const bNode *output_bnode_ = nullptr;
  VectorSet<lf::FunctionNode *> *lf_body_nodes_ = nullptr;
  bool force_threading_ = false;

  void execute_node(const lf::FunctionNode &node,
                    lf::Params &params,
                    const lf::Context &context) const override
  {
    GeoNodesUserData &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
    const int index = lf_body_nodes_->index_of_try(const_cast<lf::FunctionNode *>(&node));
    const LazyFunction &fn = node.function();
    if (index == -1) {
      /* The node is not a loop body node, just execute it normally. */
      fn.execute(params, context);
      return;
    }

    if (force_threading_) {
      /* An iteration over a group handles a whole piece of geometry, which the evaluator can't
       * know. Without this, the iterations that wait on this thread only move to other threads
       * once more than a hundred of them are queued, or once a node in the body uses threads
       * itself. */
      lazy_threading::send_hint();
    }

    /* Setup context for the loop body evaluation. */
    bke::ForeachGeometryElementZoneComputeContext body_compute_context{
        user_data.compute_context, *output_bnode_, index};
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
 * Tells the lazy-function graph executor which loop bodies should be evaluated even if they are
 * not requested by the output.
 */
class ForeachGeometryElementZoneSideEffectProvider : public lf::GraphExecutorSideEffectProvider {
 public:
  const bNode *output_bnode_ = nullptr;
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
            {context_hash, output_bnode_->identifier});

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
 * This is only evaluated when the zone is actually evaluated. It contains all the temporary data
 * that is needed for that specific evaluation.
 */
struct ForeachGeometryElementEvalStorage {
  LinearAllocator<> allocator;

  /** The lazy-function graph and its executor. */
  lf::Graph graph;
  std::optional<ForeachGeometryElementZoneSideEffectProvider> side_effect_provider;
  std::optional<ForeachGeometryElementNodeExecuteWrapper> body_execute_wrapper;
  std::optional<lf::GraphExecutor> graph_executor;
  void *graph_executor_storage = nullptr;

  /** Some lazy-functions that are constructed once the total number of iterations is known. */
  std::optional<LazyFunctionForLogicalOr> or_function;
  std::optional<LazyFunctionForReduceForeachGeometryElement> reduce_function;

  /**
   * All the body nodes in the lazy-function graph in order. This only contains nodes for the
   * selected indices.
   */
  VectorSet<lf::FunctionNode *> lf_body_nodes;

  /** The main input geometry that is iterated over. */
  GeometrySet main_geometry;
  /** Data for each geometry component that is iterated over. */
  Array<ForeachElementComponent> components;
  /** Amount of iterations across all components. */
  int total_iterations_num = 0;
  int64_t accumulated_cook_ns = 0;
  /** Output-cache decision and captured outputs of this evaluation. */
  OutputCacheSession cache_session;
};

class LazyFunctionForForeachGeometryElementZone : public LazyFunction {
 private:
  const bNodeTree &btree_;
  const bke::bNodeTreeZone &zone_;
  const bNode &output_bnode_;
  const ZoneBuildInfo &zone_info_;
  const ZoneBodyFunction &body_fn_;
  /** Iterate over groups of elements that share a Group ID instead of single elements. */
  bool group_mode_ = false;

  struct ItemIndices {
    /* `outer` refers to sockets on the outside of the zone, and `inner` to the sockets on the
     * inside. The `lf` and `bsocket` indices are similar, but the `lf` indices skip unavailable
     * and extend sockets. */
    IndexRange lf_outer;
    IndexRange lf_inner;
    IndexRange bsocket_outer;
    IndexRange bsocket_inner;
  };

  /** Reduces the hard-coding of index offsets in lots of places below which is quite brittle. */
  struct {
    ItemIndices inputs;
    ItemIndices main;
    ItemIndices generation;
    /** Only valid when iterating over groups. */
    int group_id_lf_outer = -1;
    int group_id_lf_inner = -1;
  } indices_;

  /** An output that is a field reading an anonymous attribute the zone creates. */
  struct AttributeFieldOutput {
    int lf_index;
    int bsocket_index;
    int item_identifier;
    const CPPType *type;
  };
  Vector<AttributeFieldOutput> attribute_field_outputs_;
  /** True for the lazy-function outputs listed in #attribute_field_outputs_. */
  Array<bool> is_attribute_field_output_;

  friend LazyFunctionForReduceForeachGeometryElement;

 public:
  LazyFunctionForForeachGeometryElementZone(const bNodeTree &btree,
                                            const bke::bNodeTreeZone &zone,
                                            ZoneBuildInfo &zone_info,
                                            const ZoneBodyFunction &body_fn)
      : btree_(btree),
        zone_(zone),
        output_bnode_(*zone.output_node()),
        zone_info_(zone_info),
        body_fn_(body_fn),
        group_mode_(ForeachGeometryElementIterationMode(output_bnode_.custom1) ==
                    ForeachGeometryElementIterationMode::Group)
  {
    debug_name_ = "Foreach Geometry Element";

    initialize_zone_wrapper(zone, zone_info, body_fn, true, inputs_, outputs_);
    /* All main inputs are always used. Cache hits still skip the zone body; pulling the input
     * geometry is cheap compared with iterating every element. These used to stay Maybe so a
     * hit could skip upstream, but initialize_execution_graph extract_input() then crashed
     * on a not-yet-provided buffer — even with freeze off. */
    for (const int i : zone_info.indices.inputs.main) {
      inputs_[i].usage = lf::ValueUsage::Used;
    }

    const auto &node_storage = *static_cast<const NodeGeometryForeachGeometryElementOutput *>(
        output_bnode_.storage);
    const AttrDomain iteration_domain = AttrDomain(node_storage.domain);
    BLI_assert(zone_.input_node()->output_socket(1).is_available() ==
               (iteration_domain != AttrDomain::Corner));

    const int input_items_num = node_storage.input_items.items_num;
    const int main_items_num = node_storage.main_items.items_num;
    const int generation_items_num = node_storage.generation_items.items_num;

    /* The Group ID sockets come before the items. They are unavailable, and therefore have no
     * lazy-function socket, unless the zone iterates over groups. */
    const int group_sockets_num = group_mode_ ? 1 : 0;
    const int inner_static_sockets_num = iteration_domain == AttrDomain::Corner ? 1 : 2;
    indices_.group_id_lf_outer = 2;
    indices_.group_id_lf_inner = inner_static_sockets_num;
    indices_.inputs.lf_outer = IndexRange::from_begin_size(2 + group_sockets_num,
                                                           input_items_num);
    indices_.inputs.lf_inner = IndexRange::from_begin_size(
        inner_static_sockets_num + group_sockets_num, input_items_num);
    indices_.inputs.bsocket_outer = IndexRange::from_begin_size(3, input_items_num);
    indices_.inputs.bsocket_inner = IndexRange::from_begin_size(3, input_items_num);

    indices_.main.lf_outer = IndexRange::from_begin_size(1, main_items_num);
    indices_.main.lf_inner = IndexRange::from_begin_size(0, main_items_num);
    indices_.main.bsocket_outer = indices_.main.lf_outer;
    indices_.main.bsocket_inner = indices_.main.lf_inner;

    indices_.generation.lf_outer = IndexRange::from_begin_size(1 + main_items_num,
                                                               generation_items_num);
    indices_.generation.lf_inner = IndexRange::from_begin_size(main_items_num,
                                                               generation_items_num);
    indices_.generation.bsocket_outer = IndexRange::from_begin_size(2 + main_items_num,
                                                                    generation_items_num);
    indices_.generation.bsocket_inner = IndexRange::from_begin_size(1 + main_items_num,
                                                                    generation_items_num);

    is_attribute_field_output_.reinitialize(outputs_.size());
    is_attribute_field_output_.fill(false);
    const auto add_attribute_field_output = [&](const int lf_outer,
                                                const int bsocket_outer,
                                                const int item_identifier,
                                                const eNodeSocketDatatype socket_type) {
      const CPPType *type = bke::socket_type_to_geo_nodes_base_cpp_type(socket_type);
      if (type == nullptr) {
        return;
      }
      const int lf_index = zone_info.indices.outputs.main[lf_outer];
      attribute_field_outputs_.append({lf_index, bsocket_outer, item_identifier, type});
      is_attribute_field_output_[lf_index] = true;
    };
    for (const int item_i : IndexRange(main_items_num)) {
      const NodeForeachGeometryElementMainItem &item = node_storage.main_items.items[item_i];
      add_attribute_field_output(indices_.main.lf_outer[item_i],
                                 indices_.main.bsocket_outer[item_i],
                                 item.identifier,
                                 eNodeSocketDatatype(item.socket_type));
    }
    /* Generation items in front of the first geometry belong to no geometry and output a
     * default value instead. */
    bool geometry_item_found = false;
    for (const int item_i : IndexRange(generation_items_num)) {
      const NodeForeachGeometryElementGenerationItem &item =
          node_storage.generation_items.items[item_i];
      if (item.socket_type == SOCK_GEOMETRY) {
        geometry_item_found = true;
        continue;
      }
      if (geometry_item_found) {
        add_attribute_field_output(indices_.generation.lf_outer[item_i],
                                   indices_.generation.bsocket_outer[item_i],
                                   item.identifier,
                                   eNodeSocketDatatype(item.socket_type));
      }
    }
  }

  /**
   * The field outputs only name an anonymous attribute that the zone puts on its output
   * geometries, and that name is known without running a single iteration. Providing them right
   * away means a consumer that needs just the field, or wants to know which attribute it reads,
   * does not make the zone iterate. That matters when the geometry comes from an output cache
   * behind the zone.
   */
  void output_attribute_fields(lf::Params &params, const GeoNodesUserData &user_data) const
  {
    for (const AttributeFieldOutput &output : attribute_field_outputs_) {
      if (params.output_was_set(output.lf_index) ||
          params.get_output_usage(output.lf_index) == lf::ValueUsage::Unused)
      {
        continue;
      }
      auto attribute_field = bke::AttributeFieldInput::from(
          foreach_item_attribute_name(user_data, output_bnode_, output.item_identifier),
          *output.type,
          make_anonymous_attribute_socket_inspection_string(
              output_bnode_.output_socket(output.bsocket_index)));
      params.set_output(output.lf_index, SocketValueVariant::from(std::move(attribute_field)));
    }
  }

  void *init_storage(LinearAllocator<> &allocator) const override
  {
    return allocator.construct<ForeachGeometryElementEvalStorage>().release();
  }

  void destruct_storage(void *storage) const override
  {
    auto *s = static_cast<ForeachGeometryElementEvalStorage *>(storage);
    if (s->graph_executor_storage) {
      s->graph_executor->destruct_storage(s->graph_executor_storage);
    }
    std::destroy_at(s);
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const override
  {
    auto &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
    auto &local_user_data = *static_cast<GeoNodesLocalUserData *>(context.local_user_data);

    const bool zone_has_inner_side_effects = [&]() {
      if (user_data.call_data == nullptr || user_data.call_data->side_effect_nodes == nullptr ||
          user_data.compute_context == nullptr)
      {
        return false;
      }
      const Span<int> iterations_with_side_effects =
          user_data.call_data->side_effect_nodes->iterations_by_iteration_zone.lookup(
              {user_data.compute_context->hash(), output_bnode_.identifier});
      return !iterations_with_side_effects.is_empty();
    }();

    this->output_attribute_fields(params, user_data);

    auto &eval_storage = *static_cast<ForeachGeometryElementEvalStorage *>(context.storage);
    const OutputCacheSite cache_site{&btree_, &output_bnode_, zone_.input_node()};
    OutputCacheSession &cache_session = eval_storage.cache_session;
    if (zone_has_inner_side_effects) {
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

    if (!eval_storage.graph_executor && !zone_has_inner_side_effects) {
      /* Nothing besides the attribute fields may be needed so far. Don't set up the iterations
       * before something asks for their result. */
      bool any_output_missing = false;
      for (const int i : outputs_.index_range()) {
        if (params.get_output_usage(i) == lf::ValueUsage::Used && !params.output_was_set(i)) {
          any_output_missing = true;
          break;
        }
      }
      if (!any_output_missing) {
        return;
      }
    }

    /* Main inputs are Maybe so a cache hit can skip the body. Vanilla For Each marked them Used;
     * initialize_execution_graph extract_input() on a not-yet-provided Maybe crashed even with
     * freeze off. Request them lazily like other geometry nodes. */
    if (!eval_storage.graph_executor) {
      bool missing_input = false;
      for (const int i : zone_info_.indices.inputs.main) {
        if (params.try_get_input_data_ptr_or_request(i) == nullptr) {
          missing_input = true;
        }
      }
      if (missing_input) {
        return;
      }
    }

    const ScopedNodeTimer node_timer{context, output_bnode_};

    const auto &node_storage = *static_cast<const NodeGeometryForeachGeometryElementOutput *>(
        output_bnode_.storage);
    eval_log::NodeTreeLogger *tree_logger = local_user_data.try_get_tree_logger(user_data);

    if (!eval_storage.graph_executor) {
      /* Create the execution graph in the first evaluation. */
      this->initialize_execution_graph(params, eval_storage, node_storage);

      if (tree_logger) {
        if (eval_storage.total_iterations_num == 0) {
          if (!eval_storage.main_geometry.is_empty()) {
            tree_logger->node_warnings.append(
                *tree_logger->allocator,
                {zone_.input_node()->identifier,
                 {NodeWarningType::Info,
                  N_("Input geometry has no elements in the iteration domain.")}});
          }
        }
      }
    }

    lf::Context eval_graph_context{
        eval_storage.graph_executor_storage, context.user_data, context.local_user_data};

    /* The attribute fields were provided above. */
    ForeachAttributeFieldOutputsParams graph_params{*this, params, is_attribute_field_output_};
    if (cache_session.status == OutputCacheSession::Status::Miss) {
      GeometryNodesOutputCaptureParams capture{*this, graph_params};
      eval_storage.graph_executor->execute(capture, eval_graph_context);
      eval_storage.accumulated_cook_ns += node_timer.elapsed_ns();
      int64_t frozen_ns = eval_storage.accumulated_cook_ns;
      geometry_nodes_output_cache_store(
          context, params, cache_site, cache_session, std::move(capture.captured()), frozen_ns);
    }
    else {
      eval_storage.graph_executor->execute(graph_params, eval_graph_context);
      eval_storage.accumulated_cook_ns += node_timer.elapsed_ns();
    }
  }

  void initialize_execution_graph(
      lf::Params &params,
      ForeachGeometryElementEvalStorage &eval_storage,
      const NodeGeometryForeachGeometryElementOutput &node_storage) const
  {
    eval_storage.main_geometry = params
                                     .extract_input<SocketValueVariant>(
                                         zone_info_.indices.inputs.main[0])
                                     .extract<GeometrySet>();

    /* Find all the things we need to iterate over in the input geometry. */
    this->prepare_components(params, eval_storage, node_storage);

    /* Add interface sockets for the zone graph. Those are the same as for the entire zone, even
     * though some of the inputs are not strictly needed anymore. It's easier to avoid another
     * level of index remapping though. */
    lf::Graph &lf_graph = eval_storage.graph;
    Vector<lf::GraphInputSocket *> graph_inputs;
    Vector<lf::GraphOutputSocket *> graph_outputs;
    for (const int i : inputs_.index_range()) {
      const lf::Input &input = inputs_[i];
      graph_inputs.append(&lf_graph.add_input(*input.type, this->input_name(i)));
    }
    for (const int i : outputs_.index_range()) {
      const lf::Output &output = outputs_[i];
      graph_outputs.append(&lf_graph.add_output(*output.type, this->output_name(i)));
    }

    /* Add all the nodes and links to the graph. */
    this->build_graph_contents(eval_storage, node_storage, graph_inputs, graph_outputs);

    eval_storage.side_effect_provider.emplace();
    eval_storage.side_effect_provider->output_bnode_ = &output_bnode_;
    eval_storage.side_effect_provider->lf_body_nodes_ = eval_storage.lf_body_nodes;

    eval_storage.body_execute_wrapper.emplace();
    eval_storage.body_execute_wrapper->output_bnode_ = &output_bnode_;
    eval_storage.body_execute_wrapper->lf_body_nodes_ = &eval_storage.lf_body_nodes;
    /* Iterations over single elements stay with the evaluator's own threading decisions. */
    eval_storage.body_execute_wrapper->force_threading_ = group_mode_;

    lf_graph.update_node_indices();
    eval_storage.graph_executor.emplace(lf_graph,
                                        graph_inputs.as_span(),
                                        graph_outputs.as_span(),
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
          output_bnode_.identifier, [&]() { return lf_graph.to_dot(); });
    }
  }

  void prepare_components(lf::Params &params,
                          ForeachGeometryElementEvalStorage &eval_storage,
                          const NodeGeometryForeachGeometryElementOutput &node_storage) const
  {
    const AttrDomain iteration_domain = AttrDomain(node_storage.domain);

    /* TODO: Get propagation info from input, but that's not necessary for correctness for now. */
    bke::AttributeFilter attribute_filter;

    const bNodeSocket &element_geometry_bsocket = zone_.input_node()->output_socket(1);
    const bool create_element_geometries = element_geometry_bsocket.is_available() &&
                                           element_geometry_bsocket.is_directly_linked();

    /* Gather components to process. */
    Vector<ForeachElementComponentID> component_ids;
    for (const GeometryComponent *src_component : eval_storage.main_geometry.get_components()) {
      const GeometryComponent::Type component_type = src_component->type();
      if (src_component->type() == GeometryComponent::Type::GreasePencil &&
          ELEM(iteration_domain, AttrDomain::Point, AttrDomain::Curve))
      {
        const GreasePencil &grease_pencil = *eval_storage.main_geometry.get_grease_pencil();
        for (const int layer_i : grease_pencil.layers().index_range()) {
          const bke::greasepencil::Drawing *drawing = grease_pencil.get_eval_drawing(
              grease_pencil.layer(layer_i));
          if (drawing == nullptr) {
            continue;
          }
          const bke::CurvesGeometry &curves = drawing->strokes();
          if (curves.is_empty()) {
            continue;
          }
          component_ids.append({component_type, iteration_domain, layer_i});
        }
      }
      else {
        const int domain_size = src_component->attribute_domain_size(iteration_domain);
        if (domain_size > 0) {
          component_ids.append({component_type, iteration_domain});
        }
      }
    }

    const Field<bool> selection_field = params
                                            .extract_input<SocketValueVariant>(
                                                zone_info_.indices.inputs.main[1])
                                            .extract<Field<bool>>();
    std::optional<Field<int>> group_id_field;
    if (group_mode_) {
      group_id_field = params
                           .extract_input<SocketValueVariant>(
                               zone_info_.indices.inputs.main[indices_.group_id_lf_outer])
                           .extract<Field<int>>();
    }

    /* Evaluate the selection and field inputs for all components. */
    int body_nodes_offset = 0;
    eval_storage.components.reinitialize(component_ids.size());
    for (const int component_i : component_ids.index_range()) {
      const ForeachElementComponentID id = component_ids[component_i];
      ForeachElementComponent &component_info = eval_storage.components[component_i];
      component_info.id = id;
      component_info.emplace_field_context(eval_storage.main_geometry);

      const int domain_size = component_info.input_attributes().domain_size(id.domain);
      BLI_assert(domain_size > 0);

      /* Prepare field evaluation for the zone inputs. */
      component_info.field_evaluator.emplace(*component_info.field_context, domain_size);
      component_info.field_evaluator->set_selection(selection_field);
      for (const int item_i : IndexRange(node_storage.input_items.items_num)) {
        const GField item_field =
            params
                .get_input<SocketValueVariant>(
                    zone_info_.indices.inputs.main[indices_.inputs.lf_outer[item_i]])
                .ensure_type<GField>();
        component_info.field_evaluator->add(item_field);
      }

      if (group_id_field) {
        component_info.field_evaluator->add(*group_id_field);
      }

      /* Evaluate all fields passed to the zone input. */
      component_info.field_evaluator->evaluate();

      /* The mask contains all the indices that should be iterated over in the component. */
      const IndexMask mask = component_info.field_evaluator->get_evaluated_selection_as_mask();
      if (group_mode_ && !mask.is_empty()) {
        const VArray<int> group_ids = component_info.field_evaluator->get_evaluated<int>(
            node_storage.input_items.items_num);
        component_info.group_masks = IndexMask::from_group_ids(
            mask, group_ids, component_info.group_memory);
        component_info.group_id_values.reinitialize(component_info.group_masks.size());
        for (const int group_i : component_info.group_masks.index_range()) {
          component_info.group_id_values[group_i].emplace<int>(
              group_ids[component_info.group_masks[group_i].first()]);
        }
      }
      const int iterations_num = group_mode_ ? component_info.group_masks.size() : mask.size();
      component_info.body_nodes_range = IndexRange::from_begin_size(body_nodes_offset,
                                                                    iterations_num);
      body_nodes_offset += iterations_num;

      /* Prepare indices that are passed into each iteration. */
      component_info.index_values.reinitialize(iterations_num);
      if (group_mode_) {
        for (const int group_i : IndexRange(iterations_num)) {
          component_info.index_values[group_i].emplace<int>(group_i);
        }
      }
      else {
        mask.foreach_index_optimized<int>([&](const int i, const int pos) {
          component_info.index_values[pos].emplace<int>(i);
        });
      }

      if (create_element_geometries) {
        if (std::optional<Array<GeometrySet>> element_geometries =
                group_mode_ ? this->try_extract_group_geometries(eval_storage.main_geometry,
                                                                 id,
                                                                 component_info.group_masks,
                                                                 attribute_filter) :
                              this->try_extract_element_geometries(
                                  eval_storage.main_geometry, id, mask, attribute_filter))
        {
          component_info.element_geometries.emplace(element_geometries->size());
          threading::parallel_for(
              element_geometries->index_range(), 256, [&](const IndexRange range) {
                for (const int i : range) {
                  (*component_info.element_geometries)[i] = SocketValueVariant::from(
                      (*element_geometries)[i]);
                }
              });
        }
      }

      /* Prepare remaining inputs that come from the field evaluation. */
      component_info.item_input_values.reinitialize(node_storage.input_items.items_num);
      for (const int item_i : IndexRange(node_storage.input_items.items_num)) {
        const NodeForeachGeometryElementInputItem &item = node_storage.input_items.items[item_i];
        const eNodeSocketDatatype socket_type = item.socket_type;
        const CPPType &base_cpp_type = *bke::socket_type_to_geo_nodes_base_cpp_type(socket_type);
        component_info.item_input_values[item_i].reinitialize(iterations_num);
        const GVArray &values = component_info.field_evaluator->get_evaluated(item_i);
        if (group_mode_) {
          /* An iteration gets the value of the first element in its group. */
          threading::parallel_for(IndexRange(iterations_num), 1024, [&](const IndexRange range) {
            for (const int group_i : range) {
              SocketValueVariant &value_variant =
                  component_info.item_input_values[item_i][group_i];
              void *buffer = value_variant.allocate_single(base_cpp_type);
              values.get_to_uninitialized(component_info.group_masks[group_i].first(), buffer);
            }
          });
          continue;
        }
        mask.foreach_index(
            [&](const int i, const int pos) {
              SocketValueVariant &value_variant = component_info.item_input_values[item_i][pos];
              void *buffer = value_variant.allocate_single(base_cpp_type);
              values.get_to_uninitialized(i, buffer);
            },
            exec_mode::grain_size(1024));
      }
    }

    eval_storage.total_iterations_num = body_nodes_offset;
  }

  std::optional<Array<GeometrySet>> try_extract_element_geometries(
      const GeometrySet &main_geometry,
      const ForeachElementComponentID &id,
      const IndexMask &mask,
      const bke::AttributeFilter &attribute_filter) const
  {
    switch (id.component_type) {
      case GeometryComponent::Type::Mesh: {
        const Mesh &main_mesh = *main_geometry.get_mesh();
        Array<Mesh *> meshes;
        switch (id.domain) {
          case AttrDomain::Point: {
            meshes = geometry::extract_mesh_vertices(main_mesh, mask, attribute_filter);
            break;
          }
          case AttrDomain::Edge: {
            meshes = geometry::extract_mesh_edges(main_mesh, mask, attribute_filter);
            break;
          }
          case AttrDomain::Face: {
            meshes = geometry::extract_mesh_faces(main_mesh, mask, attribute_filter);
            break;
          }
          default: {
            return std::nullopt;
          }
        }
        Array<GeometrySet> element_geometries(meshes.size());
        for (const int i : meshes.index_range()) {
          element_geometries[i].replace_mesh(meshes[i]);
        }
        return element_geometries;
      }
      case GeometryComponent::Type::PointCloud: {
        if (id.domain != AttrDomain::Point) {
          return std::nullopt;
        }
        const PointCloud &main_pointcloud = *main_geometry.get_pointcloud();
        Array<PointCloud *> pointclouds = geometry::extract_pointcloud_points(
            main_pointcloud, mask, attribute_filter);
        Array<GeometrySet> element_geometries(pointclouds.size());
        for (const int i : pointclouds.index_range()) {
          element_geometries[i].replace_pointcloud(pointclouds[i]);
        }
        return element_geometries;
      }
      case GeometryComponent::Type::Curve: {
        const Curves &main_curves = *main_geometry.get_curves();
        Array<Curves *> element_curves;
        switch (id.domain) {
          case AttrDomain::Point: {
            element_curves = geometry::extract_curves_points(main_curves, mask, attribute_filter);
            break;
          }
          case AttrDomain::Curve: {
            element_curves = geometry::extract_curves(main_curves, mask, attribute_filter);
            break;
          }
          default:
            return std::nullopt;
        }
        Array<GeometrySet> element_geometries(element_curves.size());
        for (const int i : element_curves.index_range()) {
          element_geometries[i].replace_curves(element_curves[i]);
        }
        return element_geometries;
      }
      case GeometryComponent::Type::Instance: {
        if (id.domain != AttrDomain::Instance) {
          return std::nullopt;
        }
        const bke::Instances &main_instances = *main_geometry.get_instances();
        Array<bke::Instances *> element_instances = geometry::extract_instances(
            main_instances, mask, attribute_filter);
        Array<GeometrySet> element_geometries(element_instances.size());
        for (const int i : element_instances.index_range()) {
          element_geometries[i].replace_instances(element_instances[i]);
        }
        return element_geometries;
      }
      case GeometryComponent::Type::GreasePencil: {
        const GreasePencil &main_grease_pencil = *main_geometry.get_grease_pencil();
        Array<GreasePencil *> element_grease_pencils;
        switch (id.domain) {
          case AttrDomain::Layer: {
            element_grease_pencils = geometry::extract_greasepencil_layers(
                main_grease_pencil, mask, attribute_filter);
            break;
          }
          case AttrDomain::Point: {
            element_grease_pencils = geometry::extract_greasepencil_layer_points(
                main_grease_pencil, *id.layer_index, mask, attribute_filter);
            break;
          }
          case AttrDomain::Curve: {
            element_grease_pencils = geometry::extract_greasepencil_layer_curves(
                main_grease_pencil, *id.layer_index, mask, attribute_filter);
            break;
          }
          default:
            return std::nullopt;
        }
        Array<GeometrySet> element_geometries(element_grease_pencils.size());
        for (const int i : element_geometries.index_range()) {
          element_geometries[i].replace_grease_pencil(element_grease_pencils[i]);
        }
        return element_geometries;
      }
      default:
        break;
    }
    return std::nullopt;
  }

  /**
   * Split a component into one geometry per group. Returns nothing if the component can't be
   * split on this domain.
   */
  std::optional<Array<GeometrySet>> try_extract_group_geometries(
      const GeometrySet &main_geometry,
      const ForeachElementComponentID &id,
      const Span<IndexMask> group_masks,
      const bke::AttributeFilter &attribute_filter) const
  {
    Array<GeometrySet> group_geometries(group_masks.size());
    switch (id.component_type) {
      case GeometryComponent::Type::Mesh: {
        const Mesh &main_mesh = *main_geometry.get_mesh();
        if (id.domain == AttrDomain::Face) {
          const Array<Mesh *> meshes = geometry::mesh_copy_face_groups(
              main_mesh, group_masks, attribute_filter);
          for (const int i : meshes.index_range()) {
            group_geometries[i].replace_mesh(meshes[i]);
          }
          return group_geometries;
        }
        if (!ELEM(id.domain, AttrDomain::Point, AttrDomain::Edge)) {
          return std::nullopt;
        }
        /* TODO: This looks at the whole mesh for every group, which is slow with many groups. */
        const int domain_size = main_mesh.attributes().domain_size(id.domain);
        threading::EnumerableThreadSpecific<Array<bool>> selection_per_thread{
            [&]() { return Array<bool>(domain_size, false); }};
        threading::parallel_for(group_masks.index_range(), 16, [&](const IndexRange range) {
          /* Need task isolation because of the thread local variable. */
          threading::isolate_task([&]() {
            MutableSpan<bool> selection = selection_per_thread.local();
            const VArray<bool> selection_varray = VArray<bool>::from_span(selection);
            for (const int group_i : range) {
              const IndexMask &mask = group_masks[group_i];
              index_mask::masked_fill(selection, true, mask);
              const std::optional<Mesh *> group_mesh = geometry::mesh_copy_selection(
                  main_mesh, selection_varray, id.domain, attribute_filter);
              if (group_mesh.has_value()) {
                group_geometries[group_i].replace_mesh(*group_mesh);
              }
              else {
                /* The group is the whole mesh. */
                group_geometries[group_i].add(
                    *main_geometry.get_component(GeometryComponent::Type::Mesh));
              }
              index_mask::masked_fill(selection, false, mask);
            }
          });
        });
        return group_geometries;
      }
      case GeometryComponent::Type::PointCloud: {
        if (id.domain != AttrDomain::Point) {
          return std::nullopt;
        }
        const PointCloud &main_pointcloud = *main_geometry.get_pointcloud();
        threading::parallel_for(group_masks.index_range(), 16, [&](const IndexRange range) {
          for (const int group_i : range) {
            const IndexMask &mask = group_masks[group_i];
            PointCloud *group_pointcloud = BKE_pointcloud_new_nomain(main_pointcloud.type,
                                                                      mask.size());
            bke::gather_attributes(main_pointcloud.attributes(),
                                   AttrDomain::Point,
                                   AttrDomain::Point,
                                   attribute_filter,
                                   mask,
                                   group_pointcloud->attributes_for_write());
            group_geometries[group_i].replace_pointcloud(group_pointcloud);
          }
        });
        return group_geometries;
      }
      case GeometryComponent::Type::Curve: {
        if (!ELEM(id.domain, AttrDomain::Point, AttrDomain::Curve)) {
          return std::nullopt;
        }
        const bke::CurvesGeometry &main_curves = main_geometry.get_curves()->geometry.wrap();
        threading::parallel_for(group_masks.index_range(), 16, [&](const IndexRange range) {
          for (const int group_i : range) {
            const IndexMask &mask = group_masks[group_i];
            bke::CurvesGeometry group_curves =
                id.domain == AttrDomain::Point ?
                    bke::curves_copy_point_selection(main_curves, mask, attribute_filter) :
                    bke::curves_copy_curve_selection(main_curves, mask, attribute_filter);
            group_geometries[group_i].replace_curves(
                bke::curves_new_nomain(std::move(group_curves)));
          }
        });
        return group_geometries;
      }
      case GeometryComponent::Type::Instance: {
        if (id.domain != AttrDomain::Instance) {
          return std::nullopt;
        }
        const bke::Instances &main_instances = *main_geometry.get_instances();
        threading::parallel_for(group_masks.index_range(), 16, [&](const IndexRange range) {
          for (const int group_i : range) {
            const IndexMask &mask = group_masks[group_i];
            auto group_instances = std::make_unique<bke::Instances>(mask.size());
            for (const bke::InstanceReference &reference : main_instances.references()) {
              group_instances->add_reference(reference);
            }
            bke::gather_attributes(main_instances.attributes(),
                                   AttrDomain::Instance,
                                   AttrDomain::Instance,
                                   attribute_filter,
                                   mask,
                                   group_instances->attributes_for_write());
            group_instances->remove_unused_references();
            group_geometries[group_i].replace_instances(group_instances.release());
          }
        });
        return group_geometries;
      }
      default:
        break;
    }
    return std::nullopt;
  }

  void build_graph_contents(ForeachGeometryElementEvalStorage &eval_storage,
                            const NodeGeometryForeachGeometryElementOutput &node_storage,
                            Span<lf::GraphInputSocket *> graph_inputs,
                            Span<lf::GraphOutputSocket *> graph_outputs) const
  {
    lf::Graph &lf_graph = eval_storage.graph;

    /* Create body nodes. */
    VectorSet<lf::FunctionNode *> &lf_body_nodes = eval_storage.lf_body_nodes;
    for ([[maybe_unused]] const int i : IndexRange(eval_storage.total_iterations_num)) {
      lf::FunctionNode &lf_node = lf_graph.add_function(*body_fn_.function);
      lf_body_nodes.add_new(&lf_node);
    }

    /* Link up output usages to body nodes. */
    for (const int zone_output_i : body_fn_.indices.inputs.output_usages.index_range()) {
      /* +1 because of geometry output. */
      lf::GraphInputSocket &lf_graph_input =
          *graph_inputs[zone_info_.indices.inputs.output_usages[1 + zone_output_i]];
      for (const int i : lf_body_nodes.index_range()) {
        lf::FunctionNode &lf_node = *lf_body_nodes[i];
        lf_graph.add_link(lf_graph_input,
                          lf_node.input(body_fn_.indices.inputs.output_usages[zone_output_i]));
      }
    }

    const bNodeSocket &element_geometry_bsocket = zone_.input_node()->output_socket(1);

    static const SocketValueVariant empty_geometry_value = SocketValueVariant::from(GeometrySet());
    for (const ForeachElementComponent &component_info : eval_storage.components) {
      for (const int i : component_info.body_nodes_range.index_range()) {
        const int body_i = component_info.body_nodes_range[i];
        lf::FunctionNode &lf_body_node = *lf_body_nodes[body_i];
        /* Set index input for loop body. */
        lf_body_node.input(body_fn_.indices.inputs.main[0])
            .set_default_value(&component_info.index_values[i]);
        /* Set geometry element input for loop body. */
        if (element_geometry_bsocket.is_available()) {
          const SocketValueVariant *element_geometry =
              component_info.element_geometries.has_value() ?
                  &(*component_info.element_geometries)[i] :
                  &empty_geometry_value;
          lf_body_node.input(body_fn_.indices.inputs.main[1]).set_default_value(element_geometry);
        }
        if (group_mode_) {
          lf_body_node.input(body_fn_.indices.inputs.main[indices_.group_id_lf_inner])
              .set_default_value(&component_info.group_id_values[i]);
        }
        /* Set main input values for loop body. */
        for (const int item_i : IndexRange(node_storage.input_items.items_num)) {
          lf_body_node.input(body_fn_.indices.inputs.main[indices_.inputs.lf_inner[item_i]])
              .set_default_value(&component_info.item_input_values[item_i][i]);
        }
        /* Link up border-link inputs to the loop body. */
        for (const int border_link_i : zone_info_.indices.inputs.border_links.index_range()) {
          lf_graph.add_link(
              *graph_inputs[zone_info_.indices.inputs.border_links[border_link_i]],
              lf_body_node.input(body_fn_.indices.inputs.border_links[border_link_i]));
        }
        /* Link up reference sets. */
        for (const auto &item : body_fn_.indices.inputs.reference_sets.items()) {
          lf_graph.add_link(
              *graph_inputs[zone_info_.indices.inputs.reference_sets.lookup(item.key)],
              lf_body_node.input(item.value));
        }
      }
    }

    /* Add the reduce function that has all outputs from the zone bodies as input. */
    eval_storage.reduce_function.emplace(*this, eval_storage);
    lf::FunctionNode &lf_reduce = lf_graph.add_function(*eval_storage.reduce_function);

    /* Link up body outputs to reduce function. */
    const int body_main_outputs_num = node_storage.main_items.items_num +
                                      node_storage.generation_items.items_num;
    BLI_assert(body_main_outputs_num == body_fn_.indices.outputs.main.size());
    for (const int i : IndexRange(eval_storage.total_iterations_num)) {
      lf::FunctionNode &lf_body_node = *lf_body_nodes[i];
      for (const int item_i : IndexRange(node_storage.main_items.items_num)) {
        lf_graph.add_link(lf_body_node.output(body_fn_.indices.outputs.main[item_i]),
                          lf_reduce.input(i * body_main_outputs_num + item_i));
      }
      for (const int item_i : IndexRange(node_storage.generation_items.items_num)) {
        const int body_output_i = item_i + node_storage.main_items.items_num;
        lf_graph.add_link(lf_body_node.output(body_fn_.indices.outputs.main[body_output_i]),
                          lf_reduce.input(i * body_main_outputs_num + body_output_i));
      }
    }

    /* Link up reduce function outputs to final zone outputs. */
    lf_graph.add_link(lf_reduce.output(0), *graph_outputs[zone_info_.indices.outputs.main[0]]);
    for (const int item_i : IndexRange(node_storage.main_items.items_num)) {
      const int output_i = indices_.main.lf_outer[item_i];
      lf_graph.add_link(lf_reduce.output(output_i),
                        *graph_outputs[zone_info_.indices.outputs.main[output_i]]);
    }
    for (const int item_i : IndexRange(node_storage.generation_items.items_num)) {
      const int output_i = indices_.generation.lf_outer[item_i];
      lf_graph.add_link(lf_reduce.output(output_i),
                        *graph_outputs[zone_info_.indices.outputs.main[output_i]]);
    }

    /* All zone inputs are used for now. */
    static bool static_true{true};
    for (const int i : zone_info_.indices.outputs.input_usages) {
      graph_outputs[i]->set_default_value(&static_true);
    }

    /* Handle usage outputs for border-links. A border-link is used if it's used by any of the
     * iterations. */
    eval_storage.or_function.emplace(eval_storage.total_iterations_num);
    for (const int border_link_i : zone_.border_links.index_range()) {
      lf::FunctionNode &lf_or = lf_graph.add_function(*eval_storage.or_function);
      for (const int i : lf_body_nodes.index_range()) {
        lf::FunctionNode &lf_body_node = *lf_body_nodes[i];
        lf_graph.add_link(
            lf_body_node.output(body_fn_.indices.outputs.border_link_usages[border_link_i]),
            lf_or.input(i));
      }
      lf_graph.add_link(
          lf_or.output(0),
          *graph_outputs[zone_info_.indices.outputs.border_link_usages[border_link_i]]);
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

LazyFunctionForReduceForeachGeometryElement::LazyFunctionForReduceForeachGeometryElement(
    const LazyFunctionForForeachGeometryElementZone &parent,
    ForeachGeometryElementEvalStorage &eval_storage)
    : parent_(parent), eval_storage_(eval_storage)
{
  debug_name_ = "Reduce";

  const auto &node_storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(
      parent.output_bnode_.storage);

  inputs_.reserve(eval_storage.total_iterations_num *
                  (node_storage.main_items.items_num + node_storage.generation_items.items_num));

  for ([[maybe_unused]] const int i : eval_storage.lf_body_nodes.index_range()) {
    /* Add parameters for main items. */
    for (const int item_i : IndexRange(node_storage.main_items.items_num)) {
      const NodeForeachGeometryElementMainItem &item = node_storage.main_items.items[item_i];
      inputs_.append_as(item.name, CPPType::get<SocketValueVariant>(), lf::ValueUsage::Used);
    }
    /* Add parameters for generation items. */
    for (const int item_i : IndexRange(node_storage.generation_items.items_num)) {
      const NodeForeachGeometryElementGenerationItem &item =
          node_storage.generation_items.items[item_i];
      inputs_.append_as(item.name, CPPType::get<SocketValueVariant>(), lf::ValueUsage::Maybe);
    }
  }

  /* Add output for main geometry. */
  outputs_.append_as("Geometry", CPPType::get<SocketValueVariant>());
  /* Add outputs for main items. */
  for (const int item_i : IndexRange(node_storage.main_items.items_num)) {
    const NodeForeachGeometryElementMainItem &item = node_storage.main_items.items[item_i];
    outputs_.append_as(item.name, CPPType::get<SocketValueVariant>());
  }
  /* Add outputs for generation items. */
  for (const int item_i : IndexRange(node_storage.generation_items.items_num)) {
    const NodeForeachGeometryElementGenerationItem &item =
        node_storage.generation_items.items[item_i];
    outputs_.append_as(item.name, CPPType::get<SocketValueVariant>());
  }
}

/** Gives the domain with the smallest number of elements that always exists. */
static std::optional<AttrDomain> get_foreach_attribute_propagation_target_domain(
    const GeometryComponent::Type component_type)
{
  switch (component_type) {
    case GeometryComponent::Type::Mesh:
    case GeometryComponent::Type::PointCloud:
      return AttrDomain::Point;
    case GeometryComponent::Type::Curve:
      return AttrDomain::Curve;
    case GeometryComponent::Type::Instance:
      return AttrDomain::Instance;
    case GeometryComponent::Type::GreasePencil:
      return AttrDomain::Layer;
    default:
      break;
  }
  return std::nullopt;
}

void LazyFunctionForReduceForeachGeometryElement::execute_impl(lf::Params &params,
                                                               const lf::Context &context) const
{
  const auto &node_storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(
      parent_.output_bnode_.storage);

  this->handle_main_items_and_geometry(params, context);
  if (node_storage.generation_items.items_num == 0) {
    return;
  }
  this->handle_generation_items(params, context);
}

void LazyFunctionForReduceForeachGeometryElement::handle_main_items_and_geometry(
    lf::Params &params, const lf::Context &context) const
{
  auto &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
  const auto &node_storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(
      parent_.output_bnode_.storage);
  const int body_main_outputs_num = node_storage.main_items.items_num +
                                    node_storage.generation_items.items_num;

  const int main_geometry_output = 0;
  if (params.output_was_set(main_geometry_output)) {
    /* Done already. */
    return;
  }

  GeometrySet output_geometry = eval_storage_.main_geometry;

  for (const int item_i : IndexRange(node_storage.main_items.items_num)) {
    const NodeForeachGeometryElementMainItem &item = node_storage.main_items.items[item_i];
    const eNodeSocketDatatype socket_type = item.socket_type;
    const CPPType *base_cpp_type = bke::socket_type_to_geo_nodes_base_cpp_type(socket_type);
    if (!base_cpp_type) {
      continue;
    }
    const bke::AttrType cd_type = bke::cpp_type_to_attribute_type(*base_cpp_type);

    /* Compute output attribute name for this item. */
    const std::string attribute_name = foreach_item_attribute_name(
        user_data, parent_.output_bnode_, item.identifier);

    /* Create a new output attribute for the current item on each iteration component. */
    for (const ForeachElementComponent &component_info : eval_storage_.components) {
      MutableAttributeAccessor attributes = component_info.attributes_for_write(output_geometry);
      const int domain_size = attributes.domain_size(component_info.id.domain);
      const IndexMask mask = component_info.field_evaluator->get_evaluated_selection_as_mask();

      /* Actually create the attribute. */
      bke::GSpanAttributeWriter attribute = attributes.lookup_or_add_for_write_only_span(
          attribute_name, component_info.id.domain, cd_type);

      /* Fill the elements of the attribute that we didn't iterate over because they were not
       * selected. */
      IndexMaskMemory memory;
      const IndexMask inverted_mask = mask.complement(IndexRange(domain_size), memory);
      base_cpp_type->value_initialize_indices(attribute.span.data(), inverted_mask);

      /* Copy the values from each iteration into the attribute. */
      const auto get_iteration_value = [&](const int local_body_i) -> const void * {
        const int body_i = component_info.body_nodes_range[local_body_i];
        const int lf_param_index = body_i * body_main_outputs_num + item_i;
        SocketValueVariant &value_variant = params.get_input<SocketValueVariant>(lf_param_index);
        return value_variant.ensure_type(*base_cpp_type);
      };
      if (parent_.group_mode_) {
        /* All elements of a group get the value of their iteration. */
        for (const int group_i : component_info.group_masks.index_range()) {
          base_cpp_type->fill_construct_indices(get_iteration_value(group_i),
                                                attribute.span.data(),
                                                component_info.group_masks[group_i]);
        }
      }
      else {
        mask.foreach_index([&](const int i, const int pos) {
          base_cpp_type->copy_construct(get_iteration_value(pos), attribute.span[i]);
        });
      }

      attribute.finish();
    }

    /* Output the field for the anonymous attribute. */
    auto attribute_field = bke::AttributeFieldInput::from(
        attribute_name,
        *base_cpp_type,
        make_anonymous_attribute_socket_inspection_string(
            parent_.output_bnode_.output_socket(parent_.indices_.main.bsocket_outer[item_i])));
    params.set_output(1 + item_i, SocketValueVariant::from(std::move(attribute_field)));
  }

  /* Output the original geometry with potentially additional attributes. */
  params.set_output(main_geometry_output, SocketValueVariant::from(std::move(output_geometry)));
}

void LazyFunctionForReduceForeachGeometryElement::handle_generation_items(
    lf::Params &params, const lf::Context &context) const
{
  const auto &node_storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(
      parent_.output_bnode_.storage);

  const int first_valid_item_i = this->handle_invalid_generation_items(params);
  if (first_valid_item_i == node_storage.generation_items.items_num) {
    return;
  }
  this->handle_generation_item_groups(params, context, first_valid_item_i);
}

int LazyFunctionForReduceForeachGeometryElement::handle_invalid_generation_items(
    lf::Params &params) const
{
  const auto &node_storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(
      parent_.output_bnode_.storage);

  int item_i = 0;
  /* Handle invalid generation items that come before a geometry. */
  for (; item_i < node_storage.generation_items.items_num; item_i++) {
    const NodeForeachGeometryElementGenerationItem &item =
        node_storage.generation_items.items[item_i];
    const eNodeSocketDatatype socket_type = item.socket_type;
    if (socket_type == SOCK_GEOMETRY) {
      break;
    }
    const int lf_socket_i = parent_.indices_.generation.lf_outer[item_i];
    if (!params.output_was_set(lf_socket_i)) {
      const int bsocket_i = parent_.indices_.generation.bsocket_outer[item_i];
      set_default_value_for_output_socket(
          params, lf_socket_i, parent_.zone_.output_node()->output_socket(bsocket_i));
    }
  }
  return item_i;
}

void LazyFunctionForReduceForeachGeometryElement::handle_generation_item_groups(
    lf::Params &params, const lf::Context &context, const int first_valid_item_i) const
{
  const auto &node_storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(
      parent_.output_bnode_.storage);
  int previous_geometry_item_i = first_valid_item_i;
  /* Iterate over all groups. A group starts with a geometry socket followed by an arbitrary number
   * of non-geometry sockets. */
  for (const int item_i :
       IndexRange::from_begin_end(first_valid_item_i + 1, node_storage.generation_items.items_num))
  {
    const NodeForeachGeometryElementGenerationItem &item =
        node_storage.generation_items.items[item_i];
    const eNodeSocketDatatype socket_type = item.socket_type;
    if (socket_type == SOCK_GEOMETRY) {
      this->handle_generation_items_group(
          params,
          context,
          previous_geometry_item_i,
          IndexRange::from_begin_end(previous_geometry_item_i + 1, item_i));
      previous_geometry_item_i = item_i;
    }
  }
  this->handle_generation_items_group(
      params,
      context,
      previous_geometry_item_i,
      IndexRange::from_begin_end(previous_geometry_item_i + 1,
                                 node_storage.generation_items.items_num));
}

void LazyFunctionForReduceForeachGeometryElement::handle_generation_items_group(
    lf::Params &params,
    const lf::Context &context,
    const int geometry_item_i,
    const IndexRange generation_items_range) const
{
  auto &user_data = *static_cast<GeoNodesUserData *>(context.user_data);
  const auto &node_storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(
      parent_.output_bnode_.storage);
  const int body_main_outputs_num = node_storage.main_items.items_num +
                                    node_storage.generation_items.items_num;

  /* Handle the case when the output is not needed or the inputs have not been computed yet. */
  if (!this->handle_generation_items_group_lazyness(
          params, context, geometry_item_i, generation_items_range))
  {
    return;
  }

  /* TODO: Get propagation info from input, but that's not necessary for correctness for now. */
  bke::AttributeFilter attribute_filter;

  const int bodies_num = eval_storage_.lf_body_nodes.size();
  Array<GeometrySet> geometries(bodies_num + 1);

  /* Create attribute names for the outputs. */
  Array<std::string> attribute_names(generation_items_range.size());
  for (const int i : generation_items_range.index_range()) {
    const int item_i = generation_items_range[i];
    const NodeForeachGeometryElementGenerationItem &item =
        node_storage.generation_items.items[item_i];
    attribute_names[i] = foreach_item_attribute_name(
        user_data, parent_.output_bnode_, item.identifier);
  }

  for (const ForeachElementComponent &component_info : eval_storage_.components) {
    const AttributeAccessor src_attributes = component_info.input_attributes();

    /* These are the attributes we need to propagate from the original input geometry. */
    struct NameWithType {
      StringRef name;
      bke::AttrType type;
    };
    Vector<NameWithType> attributes_to_propagate;
    /* The geometry of a group still has the attributes of its elements. There is no single
     * value per iteration that could be propagated. */
    const bool propagate_attributes = !parent_.group_mode_;
    src_attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
      if (!propagate_attributes) {
        return;
      }
      if (iter.data_type == bke::AttrType::String) {
        return;
      }
      if (attribute_filter.allow_skip(iter.name)) {
        return;
      }
      attributes_to_propagate.append({iter.name, iter.data_type});
    });
    Map<StringRef, GVArray> cached_adapted_src_attributes;

    const IndexMask mask = component_info.field_evaluator->get_evaluated_selection_as_mask();

    /* Add attributes for each field on the geometry created by each iteration. */
    const auto handle_iteration = [&](const int element_i, const int local_body_i) {
      const int body_i = component_info.body_nodes_range[local_body_i];
      const int geometry_param_i = body_i * body_main_outputs_num +
                                   parent_.indices_.generation.lf_inner[geometry_item_i];
      GeometrySet &geometry = geometries[body_i];
      geometry = params.extract_input<SocketValueVariant>(geometry_param_i).extract<GeometrySet>();

      for (const GeometryComponent::Type dst_component_type :
           {GeometryComponent::Type::Mesh,
            GeometryComponent::Type::PointCloud,
            GeometryComponent::Type::Curve,
            GeometryComponent::Type::GreasePencil,
            GeometryComponent::Type::Instance})
      {
        if (attributes_to_propagate.is_empty() || !geometry.has(dst_component_type)) {
          continue;
        }
        GeometryComponent &dst_component = geometry.get_component_for_write(dst_component_type);
        MutableAttributeAccessor dst_attributes = *dst_component.attributes_for_write();

        /* Determine the domain that we propagate the input attribute to. Technically, this is only
         * a single value for the entire geometry, but we can't optimize for that yet. */
        const std::optional<AttrDomain> propagation_domain =
            get_foreach_attribute_propagation_target_domain(dst_component_type);
        if (!propagation_domain) {
          continue;
        }

        /* Propagate attributes from the input geometry. */
        for (const NameWithType &name_with_type : attributes_to_propagate) {
          const StringRef name = name_with_type.name;
          const bke::AttrType data_type = name_with_type.type;
          if (src_attributes.is_builtin(name) && !dst_attributes.is_builtin(name)) {
            continue;
          }
          if (dst_attributes.contains(name)) {
            /* Attributes created in the zone shouldn't be overridden. */
            continue;
          }
          /* Get the source attribute adapted to the iteration domain. */
          const GVArray &src_attribute = cached_adapted_src_attributes.lookup_or_add_cb(
              name, [&]() {
                bke::GAttributeReader attribute = src_attributes.lookup(name);
                return src_attributes.adapt_domain(
                    *attribute, attribute.domain, component_info.id.domain);
              });
          if (!src_attribute) {
            continue;
          }
          const CPPType &type = src_attribute.type();
          BUFFER_FOR_CPP_TYPE_VALUE(type, element_value);
          src_attribute.get_to_uninitialized(element_i, element_value);

          /* Actually create the attribute. */
          bke::GSpanAttributeWriter dst_attribute =
              dst_attributes.lookup_or_add_for_write_only_span(
                  name, *propagation_domain, data_type);
          type.fill_assign_n(element_value, dst_attribute.span.data(), dst_attribute.span.size());
          dst_attribute.finish();

          type.destruct(element_value);
        }
      }

      /* Create an attribute for each field that corresponds to the current geometry. */
      for (const int local_item_i : generation_items_range.index_range()) {
        const int item_i = generation_items_range[local_item_i];
        const NodeForeachGeometryElementGenerationItem &item =
            node_storage.generation_items.items[item_i];
        const AttrDomain capture_domain = AttrDomain(item.domain);
        const int field_param_i = body_i * body_main_outputs_num +
                                  parent_.indices_.generation.lf_inner[item_i];
        GField field = params.get_input<SocketValueVariant>(field_param_i).copy_as<GField>();

        if (capture_domain == AttrDomain::Instance) {
          if (geometry.has_instances()) {
            bke::try_capture_field_on_geometry(
                geometry.get_component_for_write(GeometryComponent::Type::Instance),
                attribute_names[local_item_i],
                capture_domain,
                field);
          }
        }
        else {
          geometry::foreach_real_geometry(geometry, [&](GeometrySet &sub_geometry) {
            for (const GeometryComponent::Type component_type :
                 {GeometryComponent::Type::Mesh,
                  GeometryComponent::Type::PointCloud,
                  GeometryComponent::Type::Curve,
                  GeometryComponent::Type::GreasePencil})
            {
              if (sub_geometry.has(component_type)) {
                bke::try_capture_field_on_geometry(
                    sub_geometry.get_component_for_write(component_type),
                    attribute_names[local_item_i],
                    capture_domain,
                    field);
              }
            }
          });
        }
      }
    };
    if (parent_.group_mode_) {
      for (const int group_i : component_info.group_masks.index_range()) {
        handle_iteration(-1, group_i);
      }
    }
    else {
      mask.foreach_index(handle_iteration);
    }
  }

  /* The last geometry contains the edit data from the main geometry. */
  GeometrySet &edit_data_geometry = geometries.last();
  edit_data_geometry = eval_storage_.main_geometry;
  edit_data_geometry.keep_only({GeometryComponent::Type::Edit});

  /* Join the geometries from all iterations into a single one. */
  GeometrySet joined_geometry = geometry::join_geometries(geometries, attribute_filter);

  /* Output the joined geometry. */
  params.set_output(parent_.indices_.generation.lf_outer[geometry_item_i],
                    SocketValueVariant::from(std::move(joined_geometry)));

  /* Output the anonymous attribute fields. */
  for (const int local_item_i : generation_items_range.index_range()) {
    const int item_i = generation_items_range[local_item_i];
    const NodeForeachGeometryElementGenerationItem &item =
        node_storage.generation_items.items[item_i];
    const eNodeSocketDatatype socket_type = item.socket_type;
    const CPPType &base_cpp_type = *bke::socket_type_to_geo_nodes_base_cpp_type(socket_type);
    const StringRef attribute_name = attribute_names[local_item_i];
    auto attribute_field = bke::AttributeFieldInput::from(
        attribute_name,
        base_cpp_type,
        make_anonymous_attribute_socket_inspection_string(
            parent_.output_bnode_.output_socket(2 + node_storage.main_items.items_num + item_i)));
    params.set_output(parent_.indices_.generation.lf_outer[item_i],
                      bke::SocketValueVariant::from(std::move(attribute_field)));
  }
}

bool LazyFunctionForReduceForeachGeometryElement::handle_generation_items_group_lazyness(
    lf::Params &params,
    const lf::Context & /*context*/,
    const int geometry_item_i,
    const IndexRange generation_items_range) const
{
  const auto &node_storage = *static_cast<NodeGeometryForeachGeometryElementOutput *>(
      parent_.output_bnode_.storage);
  const int body_main_outputs_num = node_storage.main_items.items_num +
                                    node_storage.generation_items.items_num;

  const int geometry_output_param = parent_.indices_.generation.lf_outer[geometry_item_i];

  if (params.output_was_set(geometry_output_param)) {
    /* Done already. */
    return false;
  }
  const lf::ValueUsage geometry_output_usage = params.get_output_usage(geometry_output_param);
  if (geometry_output_usage == lf::ValueUsage::Unused) {
    /* Output dummy values. */
    const int start_bsocket_i = parent_.indices_.generation.bsocket_outer[geometry_item_i];
    for (const int i : IndexRange(1 + generation_items_range.size())) {
      const bNodeSocket &bsocket = parent_.output_bnode_.output_socket(start_bsocket_i + i);
      set_default_value_for_output_socket(params, geometry_output_param + i, bsocket);
    }
    return false;
  }
  bool any_output_used = false;
  for (const int i : IndexRange(1 + generation_items_range.size())) {
    const lf::ValueUsage usage = params.get_output_usage(geometry_output_param + i);
    if (usage == lf::ValueUsage::Used) {
      any_output_used = true;
      break;
    }
  }
  if (!any_output_used) {
    /* Only execute below if we are sure that the output is actually needed. */
    return false;
  }
  const int bodies_num = eval_storage_.lf_body_nodes.size();

  /* Check if all inputs are available, and request them if not. */
  bool has_missing_input = false;
  for (const int body_i : IndexRange(bodies_num)) {
    const int offset = body_i * body_main_outputs_num +
                       parent_.indices_.generation.lf_inner[geometry_item_i];
    for (const int i : IndexRange(1 + generation_items_range.size())) {
      const bool is_available = params.try_get_input_data_ptr_or_request(offset + i) != nullptr;
      if (!is_available) {
        has_missing_input = true;
      }
    }
  }
  if (has_missing_input) {
    /* Come back when all inputs are available. */
    return false;
  }
  return true;
}

LazyFunction &build_foreach_geometry_element_zone_lazy_function(ResourceScope &scope,
                                                                const bNodeTree &btree,
                                                                const bke::bNodeTreeZone &zone,
                                                                ZoneBuildInfo &zone_info,
                                                                const ZoneBodyFunction &body_fn)
{
  return scope.construct<LazyFunctionForForeachGeometryElementZone>(
      btree, zone, zone_info, body_fn);
}

}  // namespace blender::nodes
