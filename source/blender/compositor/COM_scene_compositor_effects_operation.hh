/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_simple_operation.hh"

namespace blender {
struct Scene;
struct PointerRNA;
struct bNodeTreeInterfaceSocket;
}  // namespace blender

namespace blender::compositor {

class Context;
enum class NodeGroupOutputTypes : uint8_t;

/* ------------------------------------------------------------------------------------------------
 * Scene Compositor Effects Operation
 *
 * An operation that creates a node group operation for each enabled scene compositor effect in
 * the scene and evaluates them serially. The fits node group takes the input of the operation as
 * an input. The has_output() and has_viewer_output() methods can be queried after evaluation to
 * identify of the operation has computed an output or a viewer output. In all cases, the output
 * will be allocated, albeit with a default value in case has_output() is false. */
class SceneCompositorEffectsOperation : public SimpleOperation {
 private:
  /* The outputs that the operation should compute. */
  NodeGroupOutputTypes needed_outputs_;
  /* True if the operation wrote an output. */
  bool has_output_ = false;

 public:
  /* Declares an input of type color and an output of type color. Needed outputs are taken from the
   * context's side-effect output types. */
  SceneCompositorEffectsOperation(Context &context);

  /* Same as the context-derived constructor, but with an explicit set of node group outputs. */
  SceneCompositorEffectsOperation(Context &context, NodeGroupOutputTypes needed_outputs);

  /* Compile and evaluate the node group. */
  void execute() override;

  /* An assessor for has_output_. This is only initialized after the operation was evaluated. Note
   * that the output of the operation still needs to be released if this is false as it is default
   * allocated in those cases. */
  bool has_output()
  {
    return has_output_;
  }
};

}  // namespace blender::compositor
