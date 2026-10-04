/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <optional>

#include "BKE_node.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "DNA_node_types.h"

#include "GPU_context.hh"

#include "COM_node_operation.hh"
#include "COM_utilities_dlssnr.hh"

#include "node_composite_util.hh"

namespace blender::nodes::node_composite_neural_cc {

enum CMPNodeNeuralStyle {
  CMP_NODE_NEURAL_STYLE_DEFAULT = 0,
  CMP_NODE_NEURAL_STYLE_NATURAL = 1,
  CMP_NODE_NEURAL_STYLE_CINEMATIC = 2,
};

static const EnumPropertyItem style_items[] = {
    {CMP_NODE_NEURAL_STYLE_DEFAULT,
     "DEFAULT",
     0,
     N_("Default"),
     N_("DLSSNR default appearance")},
    {CMP_NODE_NEURAL_STYLE_NATURAL,
     "NATURAL",
     0,
     N_("Natural"),
     N_("Softer, more natural look")},
    {CMP_NODE_NEURAL_STYLE_CINEMATIC,
     "CINEMATIC",
     0,
     N_("Cinematic"),
     N_("Higher local contrast and structure")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("Image"_ustr)
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic);
  b.add_output<decl::Color>("Image"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous();

  b.add_input<decl::Float>("Depth"_ustr)
      .default_value(1.0f)
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Linear depth. Improves temporal stability when available");
  b.add_input<decl::Vector>("Motion"_ustr)
      .dimensions(2)
      .default_value({0.0f, 0.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("2D motion vectors in pixels");
  b.add_input<decl::Float>("Mask"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(1.0f)
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Per-pixel amount. 0 keeps the original pixel");
  b.add_input<decl::Color>("Albedo"_ustr)
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic);
  b.add_input<decl::Vector>("Normal"_ustr)
      .default_value({0.0f, 0.0f, 1.0f})
      .min(-1.0f)
      .max(1.0f)
      .hide_value()
      .structure_type(StructureType::Dynamic);

  b.add_input<decl::Float>("Intensity"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(2.0f)
      .subtype(PROP_FACTOR)
      .description("DLSSNR.Intensity");
  b.add_input<decl::Menu>("Style"_ustr)
      .default_value(CMP_NODE_NEURAL_STYLE_CINEMATIC)
      .static_items(style_items)
      .optional_label()
      .description("DLSSNR.Style");
  b.add_input<decl::Float>("Structure"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(2.0f)
      .subtype(PROP_FACTOR)
      .description("DLSSNR.LocalStructureStrength");
  b.add_input<decl::Float>("Tone"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(2.0f)
      .subtype(PROP_FACTOR)
      .description("DLSSNR.LocalToneStrength");
  b.add_input<decl::Float>("Skin"_ustr)
      .default_value(-1.0f)
      .min(-1.0f)
      .max(2.0f)
      .description("DLSSNR.SkinStructureStrength. -1 disables");
  b.add_input<decl::Bool>("Auto Mask"_ustr)
      .default_value(false)
      .description("DLSSNR.UseAutoMask");
  b.add_input<decl::Bool>("UI Correction"_ustr)
      .default_value(false)
      .description("DLSSNR.UICorrection");
  b.add_input<decl::Bool>("Clamp"_ustr)
      .default_value(true)
      .description("Clamp RGB to 0-1 before NR. Prevents HDR block artifacts");
  b.add_input<decl::Bool>("Reset"_ustr)
      .default_value(false)
      .description(
          "Force-clear temporal history this eval. History also auto-clears when the "
          "camera or viewport view changes (same as viewport DLSS)");
}

static void node_draw_buttons(ui::Layout &layout, bContext * /*C*/, PointerRNA * /*ptr*/)
{
#ifdef WITH_DLSS
  layout.label(RPT_("Viewer / viewport compositor / F12"), ICON_INFO);
  if (!compositor::is_dlssnr_available()) {
    layout.label(RPT_(compositor::dlssnr_last_error()), ICON_STATUS_ERROR);
  }
#else
  layout.label(RPT_("Disabled. Built without DLSS"), ICON_STATUS_ERROR);
#endif
}

using namespace blender::compositor;

class NeuralOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void evaluate_input_processors() override
  {
    /* Mute/cancel must not run GPU conversion/realize on this node. D3D12 may have just
     * yielded the compositor context; processors would then crash in ConversionOperation. */
    if (this->node().is_muted() || this->context().is_canceled()) {
      return;
    }
    Operation::evaluate_input_processors();
  }

  void execute() override
  {
    Result *input_image = this->try_get_input("Image");
    if (input_image == nullptr || !this->has_result("Image")) {
      this->allocate_default_remaining_outputs();
      return;
    }

    Result &output_image = this->get_result("Image");

#ifdef WITH_DLSS
    /* Always write a valid output first so a failed NR pass still shows the input. */
    if (input_image->is_allocated()) {
      output_image.share_data(*input_image);
    }
    else {
      this->allocate_default_remaining_outputs();
      return;
    }

    if (this->node().is_muted() || input_image->is_single_value() || this->context().is_canceled()) {
      return;
    }

    if (!is_dlssnr_available()) {
      this->add_warning(nodes::NodeWarningType::Error, dlssnr_last_error());
      return;
    }

    /* Read the GPU image once inside apply_dlssnr. A download_to_cpu here
     * allocated a second full-frame buffer and copied it again. */
    const Result *color = input_image;
    const Result *depth = this->try_get_input("Depth");
    const Result *motion = this->try_get_input("Motion");
    const Result *mask = this->try_get_input("Mask");

    DLSSNRParams params;
    params.intensity = this->get_single("Intensity", 1.0f);
    params.style = this->get_style();
    params.structure = this->get_single("Structure", 1.0f);
    params.tone = this->get_single("Tone", 1.0f);
    params.skin = this->get_single("Skin", -1.0f);
    params.auto_mask = this->get_single("Auto Mask", false);
    params.ui_correction = this->get_single("UI Correction", false);
    params.clamp_input = this->get_single("Clamp", true);
    params.reset = this->get_single("Reset", false);
    params.view_key = this->context().temporal_view_key();

    Result output_cpu = this->context().create_result(ResultType::Color);
    GPUContext *gpu_ctx = GPU_context_active_get();
    const bool success = apply_dlssnr(this->context(),
                                      *color,
                                      depth,
                                      motion,
                                      mask,
                                      nullptr,
                                      nullptr,
                                      params,
                                      output_cpu);
    if (gpu_ctx != nullptr && GPU_context_active_get() != gpu_ctx) {
      GPU_context_active_set(gpu_ctx);
    }

    if (!success) {
      this->add_warning(nodes::NodeWarningType::Error, dlssnr_last_error());
    }
    else if (this->context().use_gpu()) {
      Result output_gpu = output_cpu.upload_to_gpu(true);
      output_image.share_data(output_gpu);
      output_gpu.release();
    }
    else {
      output_image.share_data(output_cpu);
    }

    if (output_cpu.is_allocated()) {
      output_cpu.release();
    }
#else
    output_image.share_data(*input_image);
#endif
  }

  template<typename T> T get_single(const char *identifier, const T fallback)
  {
    Result *input = this->try_get_input(identifier);
    if (input == nullptr || !input->is_allocated() || !input->is_single_value()) {
      return fallback;
    }
    return input->get_single_value<T>();
  }

  int get_style()
  {
    Result *input = this->try_get_input("Style");
    if (input == nullptr || !input->is_allocated() || !input->is_single_value()) {
      return CMP_NODE_NEURAL_STYLE_CINEMATIC;
    }
    if (input->type() == ResultType::Menu) {
      return int(input->get_single_value<MenuValue>().value);
    }
    if (input->type() == ResultType::Int) {
      return input->get_single_value<int>();
    }
    return CMP_NODE_NEURAL_STYLE_CINEMATIC;
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new NeuralOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  cmp_node_type_base(&ntype, "CompositorNodeNeural"_ustr, CMP_NODE_NEURAL);
  ntype.poll = cmp_node_poll_compositor_only;
  ntype.ui_name = "DLSS Neural";
  ntype.ui_description =
      "DLSS 5 Neural Rendering (Feature 18). Place nvngx_dlssnr.dll in blender/dlss5/";
  ntype.enum_name_legacy = "NEURAL";
  ntype.nclass = NODE_CLASS_OP_FILTER;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_draw_buttons;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_composite_neural_cc
