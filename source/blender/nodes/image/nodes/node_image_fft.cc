/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Texture Editor FFT.
 *
 * One complex field packed in Color RG: R = real, G = imaginary, B = 0.
 * 2D DFT is separable: 1D FFT of every row, then 1D FFT of every column.
 *
 * Forward divides by width×height so Inverse (two 1D IFFTs, unnormalized)
 * reconstructs RG. Center Spectrum fftshifts the complex field.
 */

#include <complex>

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_memory_utils.hh"
#include "BLI_task.hh"

#if defined(WITH_FFTW3)
#  include <fftw3.h>
#endif

#include "BLT_translation.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_fft_cc {

enum class FFTDirection : int {
  Forward = 0,
  Inverse = 1,
};

static const EnumPropertyItem direction_items[] = {
    {int(FFTDirection::Forward),
     "FORWARD",
     0,
     N_("Forward"),
     N_("Spatial RG (real, imag) to frequency RG")},
    {int(FFTDirection::Inverse),
     "INVERSE",
     0,
     N_("Inverse"),
     N_("Frequency RG back to spatial RG")},
    {0, nullptr, 0, nullptr, nullptr},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Color>("Image"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Complex image: R = real, G = imaginary");
  b.add_output<decl::Color>("Image"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Complex image: R = real, G = imaginary");

  b.add_input<decl::Menu>("Direction"_ustr)
      .default_value(FFTDirection::Forward)
      .static_items(direction_items)
      .optional_label()
      .description("Forward (analysis) or Inverse (synthesis)");
  b.add_input<decl::Bool>("Center Spectrum"_ustr)
      .default_value(true)
      .description("Place DC at the center (fftshift). Use the same setting on Inverse");
}

using namespace blender::compositor;

static Color load_color(const Result &result, const int2 texel)
{
  if (!result.is_allocated()) {
    return Color(0.0f, 0.0f, 0.0f, 1.0f);
  }
  if (result.is_single_value()) {
    switch (result.type()) {
      case ResultType::Color:
        return result.get_single_value<Color>();
      case ResultType::Float2: {
        const float2 v = result.get_single_value<float2>();
        return Color(v.x, v.y, 0.0f, 1.0f);
      }
      case ResultType::Float3: {
        const float3 v = result.get_single_value<float3>();
        return Color(v.x, v.y, v.z, 1.0f);
      }
      case ResultType::Float: {
        const float v = result.get_single_value<float>();
        return Color(v, 0.0f, 0.0f, 1.0f);
      }
      default:
        return Color(0.0f, 0.0f, 0.0f, 1.0f);
    }
  }
  switch (result.type()) {
    case ResultType::Color:
      return result.load_pixel_zero<Color>(texel);
    case ResultType::Float2: {
      const float2 v = result.load_pixel_zero<float2>(texel);
      return Color(v.x, v.y, 0.0f, 1.0f);
    }
    case ResultType::Float3: {
      const float3 v = result.load_pixel_zero<float3>(texel);
      return Color(v.x, v.y, v.z, 1.0f);
    }
    case ResultType::Float: {
      const float v = result.load_pixel_zero<float>(texel);
      return Color(v, 0.0f, 0.0f, 1.0f);
    }
    default:
      return Color(0.0f, 0.0f, 0.0f, 1.0f);
  }
}

static const Result *ensure_cpu(Context &context, const Result &input, Result &storage)
{
  if (context.use_gpu() && input.is_allocated() && !input.is_single_value() &&
      input.is_stored_on_gpu())
  {
    storage = input.download_to_cpu();
    return &storage;
  }
  return &input;
}

static void share_cpu_to_output(Context &context, Result &output, Result &cpu)
{
  if (context.use_gpu()) {
    Result gpu = cpu.upload_to_gpu(true);
    output.share_data(gpu);
    gpu.release();
  }
  else {
    output.share_data(cpu);
  }
  cpu.release();
}

class FFTOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    const Result &input = this->get_input("Image");
    if (!input.is_single_value() && input.is_allocated()) {
      return input.domain();
    }
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    Result &input = this->get_input("Image");
    Result &output = this->get_result("Image");
    if (!output.should_compute()) {
      return;
    }

    const bool inverse = FFTDirection(this->get_input("Direction")
                                          .get_single_value_default<MenuValue>()
                                          .value) == FFTDirection::Inverse;
    const bool center = this->get_input("Center Spectrum").get_single_value_default<bool>();

    Result input_cpu_storage = this->context().create_result(input.type());
    const Result *input_cpu = ensure_cpu(this->context(), input, input_cpu_storage);

    const Domain domain = this->compute_domain();
    const int2 size = domain.data_size;
    if (size.x < 1 || size.y < 1) {
      output.allocate_invalid();
      if (input_cpu_storage.is_allocated()) {
        input_cpu_storage.release();
      }
      return;
    }

#if defined(WITH_FFTW3)
    this->execute_fftw(*input_cpu, domain, inverse, center);
#else
    UNUSED_VARS(inverse, center);
    this->add_warning(NodeWarningType::Error,
                      "This Blender build was compiled without FFTW; FFT is unavailable");
    output.allocate_invalid();
#endif

    if (input_cpu_storage.is_allocated()) {
      input_cpu_storage.release();
    }
  }

#if defined(WITH_FFTW3)
  void execute_fftw(const Result &input_cpu,
                    const Domain &domain,
                    const bool inverse,
                    const bool center)
  {
    Result &output = this->get_result("Image");
    const int2 size = domain.data_size;
    const int64_t pixel_count = int64_t(size.x) * int64_t(size.y);
    const int2 half = size / 2;
    const float inv_n = 1.0f / float(pixel_count);

    std::complex<float> *buffer = reinterpret_cast<std::complex<float> *>(
        fftwf_alloc_complex(pixel_count));
    BLI_SCOPED_DEFER([&]() { fftwf_free(buffer); });

    /* Inverse + Center: ifftshift so DC is at the origin before the 1D IFFTs. */
    parallel_for(size, [&](const int2 p) {
      int2 src = p;
      if (inverse && center) {
        src.x = (p.x + half.x) % size.x;
        src.y = (p.y + half.y) % size.y;
      }
      const Color c = load_color(input_cpu, src);
      buffer[int64_t(p.y) * int64_t(size.x) + int64_t(p.x)] = {c.r, c.g};
    });

    const int sign = inverse ? FFTW_BACKWARD : FFTW_FORWARD;
    int n_row = size.x;
    int n_col = size.y;

    /* 2D FFT = 1D FFT of every row, then 1D FFT of every column. */
    fftwf_plan plan_rows = fftwf_plan_many_dft(1,
                                               &n_row,
                                               size.y,
                                               reinterpret_cast<fftwf_complex *>(buffer),
                                               nullptr,
                                               1,
                                               size.x,
                                               reinterpret_cast<fftwf_complex *>(buffer),
                                               nullptr,
                                               1,
                                               size.x,
                                               sign,
                                               FFTW_ESTIMATE);
    fftwf_plan plan_cols = fftwf_plan_many_dft(1,
                                               &n_col,
                                               size.x,
                                               reinterpret_cast<fftwf_complex *>(buffer),
                                               nullptr,
                                               size.x,
                                               1,
                                               reinterpret_cast<fftwf_complex *>(buffer),
                                               nullptr,
                                               size.x,
                                               1,
                                               sign,
                                               FFTW_ESTIMATE);
    if (plan_rows == nullptr || plan_cols == nullptr) {
      this->add_warning(NodeWarningType::Error, "Failed to create FFTW plan");
      if (plan_rows) {
        fftwf_destroy_plan(plan_rows);
      }
      if (plan_cols) {
        fftwf_destroy_plan(plan_cols);
      }
      output.allocate_invalid();
      return;
    }
    BLI_SCOPED_DEFER([&]() {
      fftwf_destroy_plan(plan_rows);
      fftwf_destroy_plan(plan_cols);
    });

    fftwf_execute(plan_rows);
    fftwf_execute(plan_cols);

    Result cpu = this->context().create_result(ResultType::Color, ResultPrecision::Full);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    parallel_for(size, [&](const int2 p) {
      int2 src = p;
      if (!inverse && center) {
        src.x = mod_i(p.x - half.x, size.x);
        src.y = mod_i(p.y - half.y, size.y);
      }
      const std::complex<float> z = buffer[int64_t(src.y) * int64_t(size.x) + int64_t(src.x)];
      /* R = real, G = imag, B = 0. Do not copy the input's B (that looked like
       * "the FFT lives in the blue channel" because |F|/N is tiny except at DC). */
      if (inverse) {
        cpu.store_pixel(p, Color(z.real(), z.imag(), 0.0f, 1.0f));
      }
      else {
        cpu.store_pixel(p, Color(z.real() * inv_n, z.imag() * inv_n, 0.0f, 1.0f));
      }
    });
    share_cpu_to_output(this->context(), output, cpu);
  }
#endif
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new FFTOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeFFT"_ustr);
  ntype.ui_name = "FFT";
  ntype.ui_description =
      "2D FFT of a complex field packed as R = real, G = imaginary (row then column 1D FFTs)";
  ntype.nclass = NODE_CLASS_OP_FILTER;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_fft_cc
