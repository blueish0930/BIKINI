/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <cstdint>

#include "BLI_enum_flags.hh"

#include "DNA_node_types.h"

#include "BKE_node.hh"

#include "COM_context.hh"
#include "COM_node_operation.hh"
#include "COM_operation.hh"
#include "COM_pixel_operation.hh"
#include "COM_result.hh"

namespace blender {
class ComputeContext;
}  // namespace blender

namespace blender::compositor {

struct Schedule;

/* Enumerates the possible node group outputs can be computed. Those can be combined into a bit
 * flag. Kept alongside Context::needed_side_effect_output_types() so image-process and nested
 * group evaluation can still request outputs per operation. */
enum class NodeGroupOutputTypes : uint8_t {
  None = 0,
  ViewerNode = 1 << 0,
  FileOutputNode = 1 << 1,
  NodePreviews = 1 << 2,
};
ENUM_OPERATORS(NodeGroupOutputTypes)

inline NodeGroupOutputTypes to_node_group_output_types(const SideEffectOutputTypes types)
{
  return NodeGroupOutputTypes(uint8_t(types));
}

/* ------------------------------------------------------------------------------------------------
 * Node Group Operation
 *
 * The node group operation represents and evaluates a node group. Evaluation is performed by the
 * generalized NodeTreeEvaluator (with official node tree zone scheduling). Image Process trees
 * additionally isolate demand terminals and Repeat-zone multipass around that evaluator. */
class NodeGroupOperation : public Operation {
 private:
  /* The node group that this operation represents. */
  const bNodeTree &node_group_;
  /* The node group outputs that should be computed. See NodeGroupOutputTypes for more details. */
  const NodeGroupOutputTypes needed_output_types_;
  /* A compute context that identifies the particular group node that uses this node group or the
   * scene for the top-level compositor node tree. */
  const ComputeContext &compute_context_;

 public:
  /* Populate the output results based on the node group interface outputs and populate the input
   * descriptors based on the node group interface inputs. Needed outputs are taken from the
   * context's side-effect output types. */
  NodeGroupOperation(Context &context,
                     const bNodeTree &node_group,
                     const ComputeContext &compute_context);

  /* Same as the context-derived constructor, but with an explicit set of node group outputs. */
  NodeGroupOperation(Context &context,
                     const bNodeTree &node_group,
                     const NodeGroupOutputTypes needed_outputs,
                     const ComputeContext &compute_context);

  /* Compile and evaluate the node group. */
  void execute() override;

  /* An accessors for node_group_. */
  const bNodeTree &node_group() const;

  /* An accessors for compute_context_. */
  const ComputeContext &compute_context() const;

  /* An accessors for needed_output_types_. */
  NodeGroupOutputTypes needed_output_types() const;

 private:
  /* Evaluate one demand schedule (with Repeat multipass when a zone is demand-reachable). */
  void evaluate_schedule_multipass(const Schedule &schedule);
};

}  // namespace blender::compositor
