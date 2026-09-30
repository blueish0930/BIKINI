/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Image Process: Sample at Pixel — gather Color at integer pixel coordinates.
 *
 * Pixel is Vector2 (PROP_PIXEL). Out-of-bounds samples return zero (load_pixel_zero).
 * Domain follows Pixel when it is not a single value; otherwise compositing domain.
 */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"

#include "BLT_translation.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_sample_at_pixel_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_input<decl::Color>("Image"_ustr)
      .default_value({0.0f, 0.0f, 0.0f, 1.0f})
      .hide_value()
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .structure_type(StructureType::Dynamic)
      .description("Texture to sample");
  b.add_output<decl::Color>("Value"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Sampled color at each Pixel coordinate");

  b.add_input<decl::Vector>("Pixel"_ustr)
      .default_value({0.0f, 0.0f})
      .dimensions(2)
      .subtype(PROP_PIXEL)
      .structure_type(StructureType::Dynamic)
      .description("Pixel coordinates (X, Y); floor is used for lookup");
}

using namespace blender::compositor;

/** Decode independent X/Y from Pixel; never expand a 1D scalar to (v,v). */
static int2 pixel_coord_from_result(const Result &pixel, const int2 src_texel)
{
  switch (pixel.type()) {
    case ResultType::Float2: {
      const float2 v = pixel.load_pixel<float2, true>(src_texel);
      return int2(int(math::floor(v.x)), int(math::floor(v.y)));
    }
    case ResultType::Float3: {
      const float3 v = pixel.load_pixel<float3, true>(src_texel);
      return int2(int(math::floor(v.x)), int(math::floor(v.y)));
    }
    case ResultType::Float4: {
      const float4 v = pixel.load_pixel<float4, true>(src_texel);
      return int2(int(math::floor(v.x)), int(math::floor(v.y)));
    }
    case ResultType::Color: {
      const Color c = pixel.load_pixel<Color, true>(src_texel);
      return int2(int(math::floor(c.r)), int(math::floor(c.g)));
    }
    case ResultType::Int2: {
      return pixel.load_pixel<int2, true>(src_texel);
    }
    case ResultType::Float: {
      /* True 1D input: identity fallback (never (v,v) diagonal). */
      return src_texel;
    }
    case ResultType::Int: {
      return src_texel;
    }
    default:
      return src_texel;
  }
}

class SampleAtPixelOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    const Result &pixel = this->get_input("Pixel");
    if (!pixel.is_single_value()) {
      return pixel.domain();
    }
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    Result &image_in = this->get_input("Image");
    Result &pixel_in = this->get_input("Pixel");
    Result &output = this->get_result("Value");

    /* GPU textures must be downloaded for CPU gather. */
    Result image_cpu_storage = this->context().create_result(ResultType::Color);
    Result pixel_cpu_storage = this->context().create_result(ResultType::Float2);
    const Result *image = &image_in;
    const Result *pixel = &pixel_in;

    if (this->context().use_gpu()) {
      if (!image_in.is_single_value()) {
        image_cpu_storage = image_in.download_to_cpu();
        image = &image_cpu_storage;
      }
      if (!pixel_in.is_single_value()) {
        pixel_cpu_storage = pixel_in.download_to_cpu();
        pixel = &pixel_cpu_storage;
      }
    }

    if (pixel->is_single_value() && image->is_single_value()) {
      const int2 p = pixel_coord_from_result(*pixel, int2(0));
      const Color value = image->load_pixel_zero<Color, true>(p);
      output.allocate_single_value();
      output.set_single_value(value);
      this->release_temp(image_cpu_storage, pixel_cpu_storage, image_in, pixel_in);
      return;
    }

    const Domain domain = this->compute_domain();
    Result output_cpu = this->context().create_result(ResultType::Color);
    output_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);

    parallel_for(domain.data_size, [&](const int2 texel) {
      const int2 p = pixel_coord_from_result(*pixel, texel);
      output_cpu.store_pixel(texel, image->load_pixel_zero<Color, true>(p));
    });

    if (this->context().use_gpu()) {
      Result output_gpu = output_cpu.upload_to_gpu(true);
      output.share_data(output_gpu);
      output_gpu.release();
    }
    else {
      output.share_data(output_cpu);
    }
    output_cpu.release();
    this->release_temp(image_cpu_storage, pixel_cpu_storage, image_in, pixel_in);
  }

 private:
  void release_temp(Result &image_cpu,
                    Result &pixel_cpu,
                    const Result &image_in,
                    const Result &pixel_in)
  {
    if (this->context().use_gpu()) {
      if (!image_in.is_single_value()) {
        image_cpu.release();
      }
      if (!pixel_in.is_single_value()) {
        pixel_cpu.release();
      }
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new SampleAtPixelOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeSampleAtPixel"_ustr, IMG_NODE_SAMPLE_AT_PIXEL);
  ntype.ui_name = "Sample at Pixel";
  ntype.ui_description = "Sample a color image at integer pixel coordinates";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_sample_at_pixel_cc
