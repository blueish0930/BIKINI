/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_compute_context.hh"
#include "BLI_index_range.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"
#include "BLI_string_ref.hh"
#include "BLI_vector.hh"

#include "DNA_node_types.h"

#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "GPU_texture.hh"

#include "NOD_eval_log.hh"

#include "COM_context.hh"
#include "COM_input_descriptor.hh"
#include "COM_node_group_operation.hh"
#include "COM_node_tree_evaluator.hh"
#include "COM_operation.hh"
#include "COM_scheduler.hh"
#include "COM_undefined_node_operation.hh"
#include "COM_utilities.hh"

namespace blender::compositor {

/* Image Process: force an image-like Result to the compositing domain (nearest resample / expand).
 * IMPORTANT: Result::share_data / allocate_texture require !is_allocated — always release first. */
static void force_result_to_compositing_domain(Context &context, Result &result)
{
  if (Result::is_single_value_only_type(result.type())) {
    return;
  }
  if (!result.is_allocated() && !result.is_single_value()) {
    return;
  }

  const Domain target = context.get_compositing_domain();
  if (target.data_size.x < 1 || target.data_size.y < 1) {
    return;
  }

  /* Already matches. */
  if (!result.is_single_value() && result.is_allocated() &&
      result.domain().data_size == target.data_size)
  {
    return;
  }

  auto replace_with = [&](Result &replacement) {
    /* Drop old data then take ownership of replacement's buffer. */
    result.release();
    result.share_data(replacement);
    replacement.release();
  };

  auto expand_single = [&](auto fill_pixel) {
    Result dst = context.create_result(result.type());
    dst.allocate_texture(target, false, ResultStorageType::CPUImage);
    parallel_for(target.data_size, [&](const int2 texel) { fill_pixel(dst, texel); });
    if (context.use_gpu()) {
      Result gpu = dst.upload_to_gpu(true);
      dst.release();
      replace_with(gpu);
    }
    else {
      replace_with(dst);
    }
  };

  if (result.is_single_value()) {
    switch (result.type()) {
      case ResultType::Color: {
        const Color c = result.get_single_value_default<Color>();
        if (context.use_gpu()) {
          result.release();
          result.allocate_texture(target, false, ResultStorageType::GPUImage);
          if (gpu::Texture *tex = result.gpu_texture()) {
            GPU_texture_clear(tex, GPU_DATA_FLOAT, c);
          }
          else {
            result.release();
            expand_single([&](Result &dst, const int2 texel) { dst.store_pixel(texel, c); });
          }
        }
        else {
          expand_single([&](Result &dst, const int2 texel) { dst.store_pixel(texel, c); });
        }
        break;
      }
      case ResultType::Float: {
        const float v = result.get_single_value_default<float>();
        expand_single([&](Result &dst, const int2 texel) { dst.store_pixel(texel, v); });
        break;
      }
      case ResultType::Float2: {
        const float2 v = result.get_single_value_default<float2>();
        expand_single([&](Result &dst, const int2 texel) { dst.store_pixel(texel, v); });
        break;
      }
      case ResultType::Float3: {
        const float3 v = result.get_single_value_default<float3>();
        expand_single([&](Result &dst, const int2 texel) { dst.store_pixel(texel, v); });
        break;
      }
      case ResultType::Float4: {
        const float4 v = result.get_single_value_default<float4>();
        expand_single([&](Result &dst, const int2 texel) { dst.store_pixel(texel, v); });
        break;
      }
      case ResultType::Int: {
        const int v = result.get_single_value_default<int>();
        expand_single([&](Result &dst, const int2 texel) { dst.store_pixel(texel, v); });
        break;
      }
      case ResultType::Bool: {
        const bool v = result.get_single_value_default<bool>();
        expand_single([&](Result &dst, const int2 texel) { dst.store_pixel(texel, v); });
        break;
      }
      default:
        break;
    }
    return;
  }

  /* Texture → texture nearest resample. */
  Result src_cpu = context.use_gpu() ? result.download_to_cpu() :
                                       context.create_result(result.type());
  if (!context.use_gpu()) {
    /* Borrow without claiming ownership of `result` — download path owns its copy. */
    src_cpu.share_data(result);
  }
  const int2 src_size = src_cpu.domain().data_size;
  if (src_size.x < 1 || src_size.y < 1) {
    src_cpu.release();
    return;
  }

  Result dst = context.create_result(result.type());
  dst.allocate_texture(target, false, ResultStorageType::CPUImage);

  auto sample_src = [&](const int2 texel) -> int2 {
    return int2(math::clamp(int(float(texel.x) * src_size.x / float(target.data_size.x)),
                            0,
                            src_size.x - 1),
                math::clamp(int(float(texel.y) * src_size.y / float(target.data_size.y)),
                            0,
                            src_size.y - 1));
  };

  switch (result.type()) {
    case ResultType::Color:
      parallel_for(target.data_size, [&](const int2 texel) {
        dst.store_pixel(texel, src_cpu.load_pixel<Color>(sample_src(texel)));
      });
      break;
    case ResultType::Float:
      parallel_for(target.data_size, [&](const int2 texel) {
        dst.store_pixel(texel, src_cpu.load_pixel<float>(sample_src(texel)));
      });
      break;
    case ResultType::Float2:
      parallel_for(target.data_size, [&](const int2 texel) {
        dst.store_pixel(texel, src_cpu.load_pixel<float2>(sample_src(texel)));
      });
      break;
    case ResultType::Float3:
      parallel_for(target.data_size, [&](const int2 texel) {
        dst.store_pixel(texel, src_cpu.load_pixel<float3>(sample_src(texel)));
      });
      break;
    case ResultType::Float4:
      parallel_for(target.data_size, [&](const int2 texel) {
        dst.store_pixel(texel, src_cpu.load_pixel<float4>(sample_src(texel)));
      });
      break;
    case ResultType::Int:
      parallel_for(target.data_size, [&](const int2 texel) {
        dst.store_pixel(texel, src_cpu.load_pixel<int>(sample_src(texel)));
      });
      break;
    case ResultType::Bool:
      parallel_for(target.data_size, [&](const int2 texel) {
        dst.store_pixel(texel, src_cpu.load_pixel<bool>(sample_src(texel)));
      });
      break;
    default:
      dst.release();
      src_cpu.release();
      return;
  }

  src_cpu.release();
  if (context.use_gpu()) {
    Result gpu = dst.upload_to_gpu(true);
    dst.release();
    replace_with(gpu);
  }
  else {
    replace_with(dst);
  }
}

static void force_image_process_group_inputs_to_domain(NodeGroupOperation &operation)
{
  if (operation.node_group().type != NTREE_IMAGE) {
    return;
  }
  /* Group Input sockets feed the fixed texture resolution. Only expand Color singles; Float/Vector
   * parameter sockets must remain true single values. */
  for (const bNodeTreeInterfaceSocket *input : operation.node_group().interface_inputs()) {
    Result &result = operation.get_input(input->identifier);
    if (result.is_single_value() && result.type() != ResultType::Color) {
      continue;
    }
    force_result_to_compositing_domain(operation.context(), result);
  }
}

NodeGroupOperation::NodeGroupOperation(Context &context,
                                       const bNodeTree &node_group,
                                       const ComputeContext &compute_context)
    : NodeGroupOperation(context,
                         node_group,
                         to_node_group_output_types(context.needed_side_effect_output_types()),
                         compute_context)
{
}

NodeGroupOperation::NodeGroupOperation(Context &context,
                                       const bNodeTree &node_group,
                                       const NodeGroupOutputTypes needed_outputs,
                                       const ComputeContext &compute_context)
    : Operation(context),
      node_group_(node_group),
      needed_output_types_(needed_outputs),
      compute_context_(compute_context)
{
  node_group.ensure_interface_cache();
  for (const bNodeTreeInterfaceSocket *input : node_group.interface_inputs()) {
    const InputDescriptor input_descriptor = input_descriptor_from_interface_input(node_group,
                                                                                   *input);
    this->declare_input_descriptor(input->identifier, input_descriptor);
  }

  for (const bNodeTreeInterfaceSocket *output : node_group.interface_outputs()) {
    this->populate_result(output->identifier, get_node_interface_socket_result_type(*output));
  }
}

class ScopedNodeGroupTimer {
 private:
  const ComputeContext &compute_context_;
  nodes::eval_log::NodesEvalLog *log_;

  nodes::eval_log::TimePoint start_;

 public:
  ScopedNodeGroupTimer(const ComputeContext &compute_context, nodes::eval_log::NodesEvalLog *log)
      : compute_context_(compute_context), log_(log)
  {
    start_ = nodes::eval_log::Clock::now();
  }

  ~ScopedNodeGroupTimer()
  {
    if (!log_) {
      return;
    }
    const nodes::eval_log::TimePoint end = nodes::eval_log::Clock::now();
    nodes::eval_log::NodeTreeLogger &tree_logger = log_->get_local_tree_logger(compute_context_);
    tree_logger.execution_time = end - start_;
  }
};

static int read_repeat_iterations_socket(const bNode &node)
{
  int n = 1;
  for (const bNodeSocket *input : node.input_sockets()) {
    if (!is_socket_available(input) || input->type != SOCK_INT) {
      continue;
    }
    if (StringRef(input->name) != "Iterations") {
      continue;
    }
    if (const auto *val = input->default_value_typed<bNodeSocketValueInt>()) {
      n = val->value;
    }
    if (input->is_logically_linked()) {
      if (const bNodeSocket *from = input->logically_linked_sockets().is_empty() ?
                                        nullptr :
                                        input->logically_linked_sockets()[0])
      {
        if (const auto *from_val = from->default_value_typed<bNodeSocketValueInt>()) {
          n = from_val->value;
        }
      }
    }
    break;
  }
  return math::max(n, 0);
}

static bool is_terminal_image_output(const bNode &node)
{
  return node.is_type("ImageNodeViewer"_ustr) || node.is_type("CompositorNodeViewer"_ustr) ||
         node.is_type("CompositorNodeOutputFile"_ustr) ||
         node.is_type("ImageNodeFileOutput"_ustr) || node.is_type("ImageNodeBakeImage"_ustr);
}

static int schedule_repeat_iterations(const Schedule &schedule)
{
  int repeat_iterations = 1;
  for (const bNode *node : schedule.nodes) {
    if (!node->is_type("GeometryNodeRepeatInput"_ustr) || node->is_muted()) {
      continue;
    }
    repeat_iterations = math::max(repeat_iterations, read_repeat_iterations_socket(*node));
  }
  return math::max(repeat_iterations, 1);
}

/* Get the results associated with sockets in the node group prior to its evaluation. */
static Result *get_node_group_result_for_socket(NodeGroupOperation &operation,
                                                const bNodeSocket &socket)
{
  if (socket.owner_node().is_group_input()) {
    return &operation.get_input(socket.identifier);
  }
  if (socket.owner_node().is_group_output()) {
    return &operation.get_result(socket.identifier);
  }
  return nullptr;
}

/**
 * Evaluate one demand schedule with optional Repeat multipass.
 * Each pass uses the official NodeTreeEvaluator. Intermediate passes free with the evaluator
 * destructor (zone body + its own upstream only).
 */
void NodeGroupOperation::evaluate_schedule_multipass(const Schedule &schedule)
{
  const int repeat_iterations = (node_group_.type == NTREE_IMAGE) ?
                                    schedule_repeat_iterations(schedule) :
                                    1;

  for (int iter = 0; iter < repeat_iterations; iter++) {
    image_process_set_repeat_iteration(iter);
    NodeTreeEvaluator node_tree_evaluator(this->context(), schedule, *this, compute_context_);
    node_tree_evaluator.evaluate();
  }

  image_process_set_repeat_iteration(0);
}

void NodeGroupOperation::execute()
{
  const ScopedNodeGroupTimer node_group_timer{compute_context_,
                                              this->context().nodes_evaluation_log()};
  auto socket_result_fn = [&](const bNodeSocket &socket) {
    return get_node_group_result_for_socket(*this, socket);
  };

  force_image_process_group_inputs_to_domain(*this);

  /* Houdini-style demand cook: only nodes feeding Viewer / File Output / Bake are scheduled.
   * Unused Repeat/Simulation zones never enter the schedule and cannot affect performance. */
  const Schedule full_schedule = compute_schedule(
      this->context(), this->node_group(), compute_context_, socket_result_fn);

  /* Image Process: isolate each demand terminal into its own schedule. Otherwise a single
   * multipass over a combined schedule re-evaluates independent branches every Repeat
   * iteration (Showcase1: Rasterize→Viewer thrashing while Repeat→Image Output multipasses). */
  if (node_group_.type == NTREE_IMAGE) {
    Vector<const bNode *> terminals;
    for (const bNode *node : full_schedule.nodes) {
      if (is_terminal_image_output(*node)) {
        terminals.append(node);
      }
    }

    if (terminals.size() > 1) {
      const Vector<int> saved_filter(image_process_terminal_filter_get());
      const bool had_filter = !saved_filter.is_empty();

      for (const int term_i : terminals.index_range()) {
        if (this->context().is_canceled()) {
          break;
        }
        const bNode *term = terminals[term_i];
        const int term_id = term->identifier;
        image_process_set_terminal_filter(Span<int>(&term_id, 1));

        const Schedule schedule = compute_schedule(
            this->context(), this->node_group(), compute_context_, socket_result_fn);
        this->evaluate_schedule_multipass(schedule);
      }

      if (had_filter) {
        image_process_set_terminal_filter(saved_filter.as_span());
      }
      else {
        image_process_clear_terminal_filter();
      }

      this->allocate_default_remaining_outputs();
      return;
    }
  }

  this->evaluate_schedule_multipass(full_schedule);

  /* Some of the needed outputs might not be allocated even after execution. This could happen for
   * instance when no Group Output node exist or when the evaluation gets canceled before the
   * output is written. */
  this->allocate_default_remaining_outputs();
}

const bNodeTree &NodeGroupOperation::node_group() const
{
  return node_group_;
}

const ComputeContext &NodeGroupOperation::compute_context() const
{
  return compute_context_;
}

NodeGroupOutputTypes NodeGroupOperation::needed_output_types() const
{
  return needed_output_types_;
}

}  // namespace blender::compositor
