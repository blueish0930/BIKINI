/* SPDX-FileCopyrightText: 2019 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef __APPLE__
#  include "BLI_system.hh"
#endif

#include <optional>

#include "MEM_guardedalloc.h"

#include "BKE_node.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "DNA_node_types.h"

#include "COM_denoised_auxiliary_pass.hh"
#include "COM_derived_resources.hh"
#include "COM_node_operation.hh"
#include "COM_utilities.hh"
#include "COM_utilities_dlss.hh"
#include "COM_utilities_oidn.hh"

#include "GPU_context.hh"

#include "node_composite_util.hh"

#ifdef WITH_OPENIMAGEDENOISE
#  include <OpenImageDenoise/oidn.hpp>
#endif

namespace blender::nodes::node_composite_denoise_cc {

static const EnumPropertyItem prefilter_items[] = {
    {CMP_NODE_DENOISE_PREFILTER_NONE,
     "NONE",
     0,
     N_("None"),
     N_("No prefiltering, use when guiding passes are noise-free")},
    {CMP_NODE_DENOISE_PREFILTER_FAST,
     "FAST",
     0,
     N_("Fast"),
     N_("Denoise image and guiding passes together. Improves quality when guiding passes are "
        "noisy using least amount of extra processing time.")},
    {CMP_NODE_DENOISE_PREFILTER_ACCURATE,
     "ACCURATE",
     0,
     N_("Accurate"),
     N_("Prefilter noisy guiding passes before denoising image. Improves quality when guiding "
        "passes are noisy using extra processing time.")},
    {0, nullptr, 0, nullptr, nullptr}};

static const EnumPropertyItem quality_items[] = {
    {CMP_NODE_DENOISE_QUALITY_SCENE,
     "FOLLOW_SCENE",
     0,
     N_("Follow Scene"),
     N_("Use the scene's denoising quality setting")},
    {CMP_NODE_DENOISE_QUALITY_HIGH, "HIGH", 0, "High", "High quality"},
    {CMP_NODE_DENOISE_QUALITY_BALANCED,
     "BALANCED",
     0,
     N_("Balanced"),
     N_("Balanced between performance and quality")},
    {CMP_NODE_DENOISE_QUALITY_FAST, "FAST", 0, "Fast", "High performance"},
    {0, nullptr, 0, nullptr, nullptr}};

#ifdef WITH_DLSS
static const EnumPropertyItem denoiser_items[] = {
    {CMP_NODE_DENOISE_DENOISER_OPENIMAGEDENOISE,
     "OPENIMAGEDENOISE",
     0,
     N_("OpenImageDenoise"),
     N_("Use Intel OpenImageDenoise")},
    {CMP_NODE_DENOISE_DENOISER_DLSS,
     "DLSS",
     0,
     N_("DLSS"),
     N_("Use NVIDIA DLSS Ray Reconstruction")},
    {0, nullptr, 0, nullptr, nullptr}};
#endif

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

  b.add_input<decl::Color>("Albedo"_ustr)
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic);
  b.add_input<decl::Vector>("Normal"_ustr)
      .default_value({0.0f, 0.0f, 0.0f})
      .min(-1.0f)
      .max(1.0f)
      .hide_value()
      .structure_type(StructureType::Dynamic);
#ifdef WITH_DLSS
  b.add_input<decl::Menu>("Denoiser"_ustr)
      .default_value(CMP_NODE_DENOISE_DENOISER_OPENIMAGEDENOISE)
      .static_items(denoiser_items)
      .optional_label();
#endif
  b.add_input<decl::Bool>("HDR"_ustr)
      .default_value(true)
#ifdef WITH_DLSS
      .usage_by_menu("Denoiser"_ustr, CMP_NODE_DENOISE_DENOISER_OPENIMAGEDENOISE)
#endif
      ;
  b.add_input<decl::Menu>("Prefilter"_ustr)
      .default_value(CMP_NODE_DENOISE_PREFILTER_ACCURATE)
      .static_items(prefilter_items)
      .optional_label()
#ifdef WITH_DLSS
      .usage_by_menu("Denoiser"_ustr, CMP_NODE_DENOISE_DENOISER_OPENIMAGEDENOISE)
#endif
      ;
  b.add_input<decl::Menu>("Quality"_ustr)
      .default_value(CMP_NODE_DENOISE_QUALITY_SCENE)
      .static_items(quality_items)
      .optional_label()
#ifdef WITH_DLSS
      .usage_by_menu("Denoiser"_ustr, CMP_NODE_DENOISE_DENOISER_OPENIMAGEDENOISE)
#endif
      ;

#ifdef WITH_DLSS
  b.add_input<decl::Float>("Depth"_ustr)
      .default_value(1.0f)
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .usage_by_menu("Denoiser"_ustr, CMP_NODE_DENOISE_DENOISER_DLSS)
      .description("Linear depth from the renderer. Improves DLSS reconstruction");
  b.add_input<decl::Color>("Specular Albedo"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .usage_by_menu("Denoiser"_ustr, CMP_NODE_DENOISE_DENOISER_DLSS);
  b.add_input<decl::Float>("Roughness"_ustr)
      .default_value(0.5f)
      .min(0.0f)
      .max(1.0f)
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .usage_by_menu("Denoiser"_ustr, CMP_NODE_DENOISE_DENOISER_DLSS);
  b.add_input<decl::Vector>("Motion"_ustr)
      .dimensions(2)
      .default_value({0.0f, 0.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .usage_by_menu("Denoiser"_ustr, CMP_NODE_DENOISE_DENOISER_DLSS)
      .description("2D motion vectors in pixels");
  b.add_input<decl::Vector>("Specular Motion"_ustr)
      .dimensions(2)
      .default_value({0.0f, 0.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .usage_by_menu("Denoiser"_ustr, CMP_NODE_DENOISE_DENOISER_DLSS);
#endif
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  /* Unused, kept for forward compatibility. */
  NodeDenoise *ndg = MEM_new<NodeDenoise>(__func__);
  node->storage = ndg;
}

static bool is_oidn_supported()
{
#ifdef WITH_OPENIMAGEDENOISE
#  if defined(__APPLE__)
  /* Always supported through Accelerate framework BNNS. */
  return true;
#  elif defined(__aarch64__) || defined(_M_ARM64)
  /* OIDN 2.2 and up supports ARM64 on Windows and Linux. */
  return true;
#  else
  return BLI_cpu_support_sse42();
#  endif
#else
  return false;
#endif
}

static void node_draw_buttons(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
#ifndef WITH_OPENIMAGEDENOISE
  layout.label(RPT_("Disabled. Built without OpenImageDenoise"), ICON_STATUS_ERROR);
#else
  if (!is_oidn_supported()) {
    layout.label(RPT_("Disabled. Platform not supported"), ICON_STATUS_ERROR);
  }
#endif
#ifdef WITH_DLSS
  const bNode &node = *static_cast<const bNode *>(ptr->data);
  const bNodeSocket *denoiser_socket = bke::node_find_socket(node, SOCK_IN, "Denoiser"_ustr);
  if (denoiser_socket &&
      denoiser_socket->default_value_typed<bNodeSocketValueMenu>()->value ==
          CMP_NODE_DENOISE_DENOISER_DLSS)
  {
    if (!compositor::is_dlss_available()) {
      layout.label(RPT_(compositor::dlss_last_error()), ICON_STATUS_ERROR);
    }
  }
#else
  UNUSED_VARS(ptr);
#endif
}

using namespace blender::compositor;

/* A callback to cancel the filter operations by evaluating the context's is_canceled method. The
 * API specifies that true indicates the filter should continue, while false indicates it should
 * stop, so invert the condition. This callback can also be used to track progress using the given
 * n argument, but we currently don't make use of it. See OIDNProgressMonitorFunction in the API
 * for more information. */
[[maybe_unused]] static bool oidn_progress_monitor_function(void *user_ptr, double /*n*/)
{
  const Context *context = static_cast<const Context *>(user_ptr);
  return !context->is_canceled();
}

class DenoiseOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void evaluate_input_processors() override
  {
    if (this->node().is_muted() || this->context().is_canceled()) {
      return;
    }
    Operation::evaluate_input_processors();
  }

  void execute() override
  {
#ifdef WITH_DLSS
    if (this->get_denoiser_type() == CMP_NODE_DENOISE_DENOISER_DLSS) {
      this->execute_dlss();
      return;
    }
#endif

    const Result &input_image = this->get_input("Image");
    Result &output_image = this->get_result("Image");

    if (!is_oidn_supported() || input_image.is_single_value()) {
      output_image.share_data(input_image);
      return;
    }

#ifdef WITH_OPENIMAGEDENOISE
    oidn::DeviceRef device = create_oidn_device(this->context());
    device.set("setAffinity", false);
    device.commit();

    /* For GPU, we download the input into a temporary result and mark it to be freed later, and
     * since it is temporary, we also perform denoising in-place. */
    Result denoise_input = this->context().create_result(input_image.type());
    Result *denoise_output = nullptr;
    if (this->context().use_gpu()) {
      Result input_image_cpu = input_image.download_to_cpu();
      denoise_input.share_data(input_image_cpu);
      input_image_cpu.release();

      denoise_output = &denoise_input;
    }
    else {
      denoise_input.share_data(input_image);

      output_image.allocate_texture(input_image.domain());
      denoise_output = &output_image;
    }

    oidn::BufferRef input_buffer = create_oidn_buffer(device, denoise_input);
    oidn::BufferRef output_buffer = create_oidn_buffer(device, *denoise_output);

    oidn::FilterRef filter = device.newFilter("RT");
    const int2 size = input_image.domain().data_size;
    const int pixel_stride = sizeof(float) * denoise_input.channels_count();
    filter.setImage("color", input_buffer, oidn::Format::Float3, size.x, size.y, 0, pixel_stride);
    filter.setImage(
        "output", output_buffer, oidn::Format::Float3, size.x, size.y, 0, pixel_stride);
    filter.set("hdr", use_hdr());
    filter.set("cleanAux", auxiliary_passes_are_clean());
    this->set_filter_quality(filter);
    filter.setProgressMonitorFunction(oidn_progress_monitor_function, &context());

    /* If the albedo input is not a single value input, set it to the albedo input of the filter,
     * denoising it if needed. */
    Result &input_albedo = this->get_input("Albedo");
    Result denoise_albedo = this->context().create_result(input_albedo.type());
    if (!input_albedo.is_single_value()) {
      if (this->should_denoise_auxiliary_passes()) {
        denoise_albedo.share_data(input_albedo.derived_resources()
                                      .denoised_auxiliary_passes
                                      .get(this->context(),
                                           input_albedo,
                                           DenoisedAuxiliaryPassType::Albedo,
                                           this->get_quality())
                                      .result);
      }
      else {
        if (this->context().use_gpu()) {
          Result input_albedo_cpu = input_albedo.download_to_cpu();
          denoise_albedo.share_data(input_albedo_cpu);
          input_albedo_cpu.release();
        }
        else {
          denoise_albedo.share_data(input_albedo);
        }
      }

      oidn::BufferRef albedo_buffer = create_oidn_buffer(device, denoise_albedo);
      const int pixel_stride = sizeof(float) * denoise_albedo.channels_count();
      filter.setImage(
          "albedo", albedo_buffer, oidn::Format::Float3, size.x, size.y, 0, pixel_stride);
    }

    /* If the albedo and normal inputs are not single value inputs, set the normal input to the
     * albedo input of the filter, denoising it if needed. Notice that we also consider the albedo
     * input because OIDN doesn't support denoising with only the normal auxiliary pass. */
    Result &input_normal = this->get_input("Normal");
    Result denoise_normal = this->context().create_result(input_normal.type());
    if (!input_albedo.is_single_value() && !input_normal.is_single_value()) {
      if (should_denoise_auxiliary_passes()) {
        denoise_normal.share_data(input_normal.derived_resources()
                                      .denoised_auxiliary_passes
                                      .get(this->context(),
                                           input_normal,
                                           DenoisedAuxiliaryPassType::Normal,
                                           this->get_quality())
                                      .result);
      }
      else {
        if (this->context().use_gpu()) {
          Result input_normal_cpu = input_normal.download_to_cpu();
          denoise_normal.share_data(input_normal_cpu);
          input_normal_cpu.release();
        }
        else {
          denoise_normal.share_data(input_normal);
        }
      }

      oidn::BufferRef normal_buffer = create_oidn_buffer(device, denoise_normal);
      const int pixel_stride = sizeof(float) * denoise_normal.channels_count();
      filter.setImage(
          "normal", normal_buffer, oidn::Format::Float3, size.x, size.y, 0, pixel_stride);
    }

    filter.commit();
    filter.execute();

    if (output_buffer.getStorage() != oidn::Storage::Host) {
      output_buffer.read(
          0, denoise_output->size_in_bytes(), denoise_output->cpu_data_for_write().data());
    }

    if (this->context().use_gpu()) {
      Result output_gpu = denoise_output->upload_to_gpu(true);
      output_image.share_data(output_gpu);
      output_gpu.release();
    }
    else {
      /* OIDN already wrote to the output directly, however, OIDN skips the alpha channel, so we
       * need to restore it. */
      parallel_for(size, [&](const int2 texel) {
        const float alpha = input_image.load_pixel<Color>(texel).a;
        output_image.store_pixel(
            texel, Color(float4(float4(output_image.load_pixel<Color>(texel)).xyz(), alpha)));
      });
    }

    denoise_input.release();
    denoise_albedo.release();
    denoise_normal.release();
#endif
  }

  /* If the pre-filter mode is set to CMP_NODE_DENOISE_PREFILTER_NONE, that it means the supplied
   * auxiliary passes are already noise-free, if it is set to CMP_NODE_DENOISE_PREFILTER_ACCURATE,
   * the auxiliary passes will be denoised before denoising the main image, so in both cases, the
   * auxiliary passes are considered clean. If it is set to CMP_NODE_DENOISE_PREFILTER_FAST on the
   * other hand, the auxiliary passes are assumed to be noisy and are thus not clean, and will be
   * denoised while denoising the main image. */
  bool auxiliary_passes_are_clean()
  {
    return get_prefilter_mode() != CMP_NODE_DENOISE_PREFILTER_FAST;
  }

  /* Returns whether the auxiliary passes should be denoised, see the auxiliary_passes_are_clean
   * method for more information. */
  bool should_denoise_auxiliary_passes()
  {
    return get_prefilter_mode() == CMP_NODE_DENOISE_PREFILTER_ACCURATE;
  }

#ifdef WITH_OPENIMAGEDENOISE
#  if OIDN_VERSION_MAJOR >= 2
  oidn::Quality get_quality()
  {
    const CMPNodeDenoiseQuality node_quality = this->get_quality_mode();

    if (node_quality == CMP_NODE_DENOISE_QUALITY_SCENE) {
      const eCompositorDenoiseQaulity scene_quality = context().get_denoise_quality();
      switch (scene_quality) {
#    if OIDN_VERSION >= 20300
        case SCE_COMPOSITOR_DENOISE_FAST:
          return oidn::Quality::Fast;
#    endif
        case SCE_COMPOSITOR_DENOISE_BALANCED:
          return oidn::Quality::Balanced;
        case SCE_COMPOSITOR_DENOISE_HIGH:
          return oidn::Quality::High;
      }

      return oidn::Quality::High;
    }

    switch (node_quality) {
#    if OIDN_VERSION >= 20300
      case CMP_NODE_DENOISE_QUALITY_FAST:
        return oidn::Quality::Fast;
#    endif
      case CMP_NODE_DENOISE_QUALITY_BALANCED:
        return oidn::Quality::Balanced;
      case CMP_NODE_DENOISE_QUALITY_HIGH:
      case CMP_NODE_DENOISE_QUALITY_SCENE:
        return oidn::Quality::High;
    }

    return oidn::Quality::High;
  }
#  endif /* OIDN_VERSION_MAJOR >= 2 */

  void set_filter_quality([[maybe_unused]] oidn::FilterRef &filter)
  {
#  if OIDN_VERSION_MAJOR >= 2
    oidn::Quality quality = this->get_quality();
    filter.set("quality", quality);
#  endif
  }
#endif /* WITH_OPENIMAGEDENOISE */

  bool use_hdr()
  {
    return this->get_input("HDR").get_single_value_default<bool>();
  }

#ifdef WITH_DLSS
  CMPNodeDenoiseDenoiser get_denoiser_type()
  {
    if (!this->has_input("Denoiser")) {
      return CMP_NODE_DENOISE_DENOISER_OPENIMAGEDENOISE;
    }
    return CMPNodeDenoiseDenoiser(
        this->get_input("Denoiser").get_single_value_default<MenuValue>().value);
  }

  const Result *prepare_cpu_input(const char *identifier, std::optional<Result> &storage)
  {
    if (!this->has_input(identifier)) {
      return nullptr;
    }
    const Result &input = this->get_input(identifier);
    if (this->context().use_gpu() && !input.is_single_value()) {
      storage = input.download_to_cpu();
      return &*storage;
    }
    return &input;
  }

  void execute_dlss()
  {
    const Result &input_image = this->get_input("Image");
    Result &output_image = this->get_result("Image");

    if (this->node().is_muted() || this->context().is_canceled() ||
        !input_image.is_allocated())
    {
      if (input_image.is_allocated()) {
        output_image.share_data(input_image);
      }
      else {
        this->allocate_default_remaining_outputs();
      }
      return;
    }

    if (!is_dlss_available() || input_image.is_single_value()) {
      if (!is_dlss_available() && !input_image.is_single_value()) {
        this->add_warning(nodes::NodeWarningType::Error, dlss_last_error());
      }
      output_image.share_data(input_image);
      return;
    }

    /* Color stays on the GPU until denoise_with_dlss reads it once. */
    const Result *color = &input_image;

    std::optional<Result> albedo_storage;
    std::optional<Result> normal_storage;
    std::optional<Result> depth_storage;
    std::optional<Result> specular_albedo_storage;
    std::optional<Result> roughness_storage;
    std::optional<Result> motion_storage;
    std::optional<Result> specular_motion_storage;

    const Result *albedo = this->prepare_cpu_input("Albedo", albedo_storage);
    const Result *normal = this->prepare_cpu_input("Normal", normal_storage);
    const Result *depth = this->prepare_cpu_input("Depth", depth_storage);
    const Result *specular_albedo = this->prepare_cpu_input("Specular Albedo",
                                                            specular_albedo_storage);
    const Result *roughness = this->prepare_cpu_input("Roughness", roughness_storage);
    const Result *motion = this->prepare_cpu_input("Motion", motion_storage);
    const Result *specular_motion = this->prepare_cpu_input("Specular Motion",
                                                            specular_motion_storage);

    Result output_cpu = this->context().create_result(ResultType::Color);
    GPUContext *gpu_ctx = GPU_context_active_get();
    const bool success = denoise_with_dlss(this->context(),
                                           *color,
                                           albedo,
                                           normal,
                                           depth,
                                           specular_albedo,
                                           roughness,
                                           motion,
                                           specular_motion,
                                           output_cpu);
    if (gpu_ctx != nullptr && GPU_context_active_get() != gpu_ctx) {
      GPU_context_active_set(gpu_ctx);
    }

    auto release_storage = [](std::optional<Result> &storage) {
      if (storage.has_value()) {
        storage->release();
      }
    };

    if (!success) {
      this->add_warning(nodes::NodeWarningType::Error, dlss_last_error());
      output_image.share_data(input_image);
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
    release_storage(albedo_storage);
    release_storage(normal_storage);
    release_storage(depth_storage);
    release_storage(specular_albedo_storage);
    release_storage(roughness_storage);
    release_storage(motion_storage);
    release_storage(specular_motion_storage);
  }
#endif

  CMPNodeDenoisePrefilter get_prefilter_mode()
  {
    return CMPNodeDenoisePrefilter(
        this->get_input("Prefilter").get_single_value_default<MenuValue>().value);
  }

  CMPNodeDenoiseQuality get_quality_mode()
  {
    return CMPNodeDenoiseQuality(
        this->get_input("Quality").get_single_value_default<MenuValue>().value);
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new DenoiseOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  cmp_node_type_base(&ntype, "CompositorNodeDenoise"_ustr, CMP_NODE_DENOISE);
  ntype.poll = cmp_node_poll_compositor_only;
  ntype.ui_name = "Denoise";
  ntype.ui_description = "Denoise renders from Cycles and other ray tracing renderers";
  ntype.enum_name_legacy = "DENOISE";
  ntype.nclass = NODE_CLASS_OP_FILTER;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_draw_buttons;
  ntype.initfunc = node_init;
  bke::node_type_storage(
      ntype, "NodeDenoise", node_free_standard_storage, node_copy_standard_storage);
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_composite_denoise_cc
