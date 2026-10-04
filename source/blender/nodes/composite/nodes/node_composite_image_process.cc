/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Compositor: evaluate a GPU Texture Editor (ImageNodeTree) node group.
 * Socket layout mirrors the selected group's Group Input / Group Output interface. */

#include <memory>

#include "BLI_vector.hh"

#include "DNA_node_types.h"

#include "BKE_compute_contexts.hh"
#include "BKE_node.hh"
#include "BKE_node_runtime.hh"

#include "NOD_common.hh"
#include "node_common.h"

#include "COM_node_group_operation.hh"
#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.hh"

#include "node_composite_util.hh"

namespace blender::nodes::node_composite_image_process_cc {

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "node_tree", UI_ITEM_NONE, std::nullopt, ICON_NODE_TEXTURE);
}

using namespace blender::compositor;

/**
 * Nested evaluation of an ImageNodeTree, mirroring GroupNodeOperation but for a foreign tree type.
 * Extra null/type checks — compositor must never crash if the group is missing or half-updated.
 */
class ImageProcessGroupOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  ImageProcessGroupOperation(Context &context, const bNode &node) : NodeOperation(context, node)
  {
    /* Same as GroupNodeOperation: do not force realization; infer structure.
     * Descriptors are declared in NodeOperation's ctor — has_input is still empty here. */
    for (const bNodeSocket *input : node.input_sockets()) {
      if (!is_socket_available(input)) {
        continue;
      }
      InputDescriptor &descriptor = this->get_input_descriptor(input->identifier);
      descriptor.expects_single_value = false;
      descriptor.realization_mode = InputRealizationMode::None;
    }
  }

  void execute() override
  {
    const bNodeTree *group = this->get_image_process_group();
    if (!group) {
      this->allocate_default_remaining_outputs();
      return;
    }

    /* Guard against deep recursive nesting (RAII so exceptions/returns always unwind). */
    static thread_local int nest_depth = 0;
    if (nest_depth > 6) {
      this->allocate_default_remaining_outputs();
      return;
    }
    struct NestGuard {
      int &depth;
      explicit NestGuard(int &d) : depth(d)
      {
        depth++;
      }
      ~NestGuard()
      {
        depth--;
      }
    } nest_guard(nest_depth);

    group->ensure_topology_cache();
    group->ensure_interface_cache();

    if (group->has_available_link_cycle()) {
      this->context().set_info_message("GPU Texture Editor group has cyclic links.");
      this->allocate_default_remaining_outputs();
      return;
    }

    /* No Group Output / empty interface → nothing to compute. */
    if (group->interface_outputs().is_empty() || group->group_output_node() == nullptr) {
      this->allocate_default_remaining_outputs();
      return;
    }

    const bke::GroupNodeComputeContext compute_context(
        &this->get_compute_context(), this->node().identifier, &this->node().owner_tree());

    NodeGroupOperation nested(
        this->context(), *group, NodeGroupOutputTypes::None, compute_context);

    this->set_reference_counts(nested);
    Vector<std::unique_ptr<Result>> temporary_inputs = this->map_inputs(nested);

    nested.evaluate();

    this->write_outputs(nested);
    (void)temporary_inputs;
  }

 private:
  const bNodeTree *get_image_process_group() const
  {
    ID *id = this->node().id;
    if (!id || GS(id->name) != ID_NT) {
      return nullptr;
    }
    const bNodeTree *tree = reinterpret_cast<const bNodeTree *>(id);
    if (tree->type != NTREE_IMAGE) {
      return nullptr;
    }
    if (ID_IS_LINKED(&tree->id) && ID_MISSING(&tree->id)) {
      return nullptr;
    }
    return tree;
  }

  void set_reference_counts(NodeGroupOperation &nested)
  {
    const bNodeTree *group = this->get_image_process_group();
    for (const bNodeTreeInterfaceSocket *output_socket : group->interface_outputs()) {
      if (!nested.has_result(output_socket->identifier)) {
        continue;
      }
      Result &nested_out = nested.get_result(output_socket->identifier);
      if (!this->has_result(output_socket->identifier)) {
        nested_out.set_reference_count(0);
        continue;
      }
      Result &node_out = this->get_result(output_socket->identifier);
      nested_out.set_reference_count(node_out.should_compute() ? 1 : 0);
    }
  }

  /**
   * Mirror GroupNodeOperation::map_inputs — same type/precision as the parent input so
   * share_data never hits type-mismatch asserts. When a parent socket is missing or unallocated,
   * allocate a zero single-value default (never leave an unmapped nested input).
   */
  Vector<std::unique_ptr<Result>> map_inputs(NodeGroupOperation &nested)
  {
    const bNodeTree *group = this->get_image_process_group();
    Vector<std::unique_ptr<Result>> temporary_inputs;
    for (const bNodeTreeInterfaceSocket *input_socket : group->interface_inputs()) {
      const ResultType iface_type = get_node_interface_socket_result_type(*input_socket);

      if (this->has_input(input_socket->identifier)) {
        const Result &input_result = this->get_input(input_socket->identifier);
        /* Prefer the actual parent result type so share_data is always valid. */
        if ((input_result.is_allocated() || input_result.is_single_value()) &&
            input_result.type() == iface_type)
        {
          std::unique_ptr<Result> temporary = std::make_unique<Result>(
              this->context().create_result(input_result.type(), input_result.precision()));
          temporary->share_data(input_result);
          nested.map_input_to_result(input_socket->identifier, temporary.get());
          temporary_inputs.append(std::move(temporary));
          continue;
        }
      }

      /* Missing / wrong-type / unallocated: zero single-value default of the interface type. */
      std::unique_ptr<Result> temporary = std::make_unique<Result>(
          this->context().create_result(iface_type));
      temporary->allocate_single_value();
      nested.map_input_to_result(input_socket->identifier, temporary.get());
      temporary_inputs.append(std::move(temporary));
    }
    return temporary_inputs;
  }

  void write_outputs(NodeGroupOperation &nested)
  {
    const bNodeTree *group = this->get_image_process_group();
    bool wrote_any = false;
    for (const bNodeTreeInterfaceSocket *output_socket : group->interface_outputs()) {
      if (!nested.has_result(output_socket->identifier)) {
        continue;
      }
      Result &nested_out = nested.get_result(output_socket->identifier);
      if (!this->has_result(output_socket->identifier)) {
        if (nested_out.is_allocated() || nested_out.is_single_value()) {
          nested_out.release();
        }
        continue;
      }
      Result &node_out = this->get_result(output_socket->identifier);
      if (!node_out.should_compute()) {
        if (nested_out.is_allocated() || nested_out.is_single_value()) {
          nested_out.release();
        }
        continue;
      }
      if ((nested_out.is_allocated() || nested_out.is_single_value()) &&
          nested_out.type() == node_out.type())
      {
        /* share_data requires destination unallocated — node outputs start empty.
         * Prefer GPU storage when the compositor runs on GPU so Viewer never gets CPU buffers. */
        if (this->context().use_gpu() && !nested_out.is_single_value() &&
            nested_out.is_allocated() && !nested_out.is_stored_on_gpu())
        {
          Result gpu = nested_out.upload_to_gpu(true);
          nested_out.release();
          node_out.share_data(gpu);
          gpu.release();
        }
        else {
          node_out.share_data(nested_out);
          nested_out.release();
        }
        wrote_any = true;
      }
      else if (nested_out.is_allocated() || nested_out.is_single_value()) {
        nested_out.release();
      }
    }
    if (!wrote_any) {
      this->allocate_default_remaining_outputs();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new ImageProcessGroupOperation(context, node);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->id = nullptr;
}

static void node_register()
{
  static bke::bNodeType ntype;

  cmp_node_type_base(&ntype, "CompositorNodeImageProcess"_ustr, CMP_NODE_IMAGE_PROCESS);
  ntype.ui_name = "GPU Texture Editor";
  ntype.ui_description =
      "Evaluate a GPU Texture Editor node group. Inputs and outputs match that group's "
      "Group Input / Group Output interface";
  ntype.enum_name_legacy = "IMAGE_PROCESS";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = nodes::node_group_declare;
  ntype.labelfunc = node_group_label;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.minwidth = bke::NodeWidth::GroupMin;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_composite_image_process_cc
