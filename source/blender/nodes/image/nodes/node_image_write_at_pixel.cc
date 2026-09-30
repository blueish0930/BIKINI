/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Image Process: Write at Pixel — scatter Color onto compositing domain (no base Image).
 *
 * Atomic Operator (Menu on node body): Sum / Min / Max / Mean.
 * Unwritten pixels are zero. Out-of-bounds writes are ignored.
 * Pixel is Vector2 (PROP_PIXEL); never expand 1D scalars to (v,v).
 */

#include "BLI_array.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include "BLT_translation.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "NOD_menu_value.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_write_at_pixel_cc {

enum class AtomicOperator : int8_t {
  Sum = 0,
  Min = 1,
  Max = 2,
  Mean = 3,
};

static const EnumPropertyItem atomic_operator_items[] = {
    {int(AtomicOperator::Sum), "SUM", 0, N_("Sum"), N_("Add conflicting writes")},
    {int(AtomicOperator::Min), "MIN", 0, N_("Min"), N_("Keep the per-channel minimum")},
    {int(AtomicOperator::Max), "MAX", 0, N_("Max"), N_("Keep the per-channel maximum")},
    {int(AtomicOperator::Mean), "MEAN", 0, N_("Mean"), N_("Average conflicting writes")},
    {},
};

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_output<decl::Color>("Value"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Full-frame result; unwritten pixels are zero");

  b.add_input<decl::Menu>("Atomic Operator"_ustr)
      .default_value(AtomicOperator::Sum)
      .static_items(atomic_operator_items)
      .optional_label()
      .description("How multiple writes to the same pixel are combined");

  b.add_input<decl::Vector>("Pixel"_ustr)
      .default_value({0.0f, 0.0f})
      .dimensions(2)
      .subtype(PROP_PIXEL)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .structure_type(StructureType::Dynamic)
      .description("Target pixel coordinates (X, Y); floor is used");

  b.add_input<decl::Color>("Value"_ustr)
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .structure_type(StructureType::Dynamic)
      .description("Color written at each Pixel");
}

using namespace blender::compositor;

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
    case ResultType::Float:
    case ResultType::Int:
      /* True 1D: identity, never (v,v). */
      return src_texel;
    default:
      return src_texel;
  }
}

static Color color_min(const Color &a, const Color &b)
{
  return Color(math::min(a.r, b.r), math::min(a.g, b.g), math::min(a.b, b.b), math::min(a.a, b.a));
}

static Color color_max(const Color &a, const Color &b)
{
  return Color(math::max(a.r, b.r), math::max(a.g, b.g), math::max(a.b, b.b), math::max(a.a, b.a));
}

class WriteAtPixelOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    Result &pixel_in = this->get_input("Pixel");
    Result &value_in = this->get_input("Value");
    Result &output = this->get_result("Value");

    const AtomicOperator atomic = AtomicOperator(
        this->get_input("Atomic Operator").get_single_value_default<MenuValue>().value);

    Result pixel_cpu_storage = this->context().create_result(ResultType::Float2);
    Result value_cpu_storage = this->context().create_result(ResultType::Color);
    const Result *pixel = &pixel_in;
    const Result *value = &value_in;

    if (this->context().use_gpu()) {
      if (!pixel_in.is_single_value()) {
        pixel_cpu_storage = pixel_in.download_to_cpu();
        pixel = &pixel_cpu_storage;
      }
      if (!value_in.is_single_value()) {
        value_cpu_storage = value_in.download_to_cpu();
        value = &value_cpu_storage;
      }
    }

    const Domain out_domain = this->compute_domain();
    const int2 out_size = out_domain.data_size;
    const int64_t pixel_count = int64_t(out_size.x) * int64_t(out_size.y);

    Result output_cpu = this->context().create_result(ResultType::Color);
    output_cpu.allocate_texture(out_domain, false, ResultStorageType::CPUImage);

    parallel_for(out_size, [&](const int2 texel) {
      output_cpu.store_pixel(texel, Color(0.0f, 0.0f, 0.0f, 0.0f));
    });

    Array<int> write_count(pixel_count, 0);

    auto apply_write = [&](const int2 p, const Color &c) {
      if (p.x < 0 || p.y < 0 || p.x >= out_size.x || p.y >= out_size.y) {
        return;
      }
      const int64_t idx = int64_t(p.y) * int64_t(out_size.x) + int64_t(p.x);
      Color existing = output_cpu.load_pixel<Color>(p);
      switch (atomic) {
        case AtomicOperator::Sum:
          output_cpu.store_pixel(p,
                                 Color(existing.r + c.r,
                                       existing.g + c.g,
                                       existing.b + c.b,
                                       existing.a + c.a));
          write_count[idx] += 1;
          break;
        case AtomicOperator::Min:
          if (write_count[idx] == 0) {
            output_cpu.store_pixel(p, c);
          }
          else {
            output_cpu.store_pixel(p, color_min(existing, c));
          }
          write_count[idx] += 1;
          break;
        case AtomicOperator::Max:
          if (write_count[idx] == 0) {
            output_cpu.store_pixel(p, c);
          }
          else {
            output_cpu.store_pixel(p, color_max(existing, c));
          }
          write_count[idx] += 1;
          break;
        case AtomicOperator::Mean:
          output_cpu.store_pixel(p,
                                 Color(existing.r + c.r,
                                       existing.g + c.g,
                                       existing.b + c.b,
                                       existing.a + c.a));
          write_count[idx] += 1;
          break;
      }
    };

    if (pixel->is_single_value() && value->is_single_value()) {
      apply_write(pixel_coord_from_result(*pixel, int2(0)),
                  value->load_pixel<Color, true>(int2(0)));
    }
    else {
      Domain src_domain = out_domain;
      if (!pixel->is_single_value()) {
        src_domain = pixel->domain();
      }
      else if (!value->is_single_value()) {
        src_domain = value->domain();
      }
      /* Serial scatter for correct atomics (MVP). */
      const int2 src_size = src_domain.data_size;
      for (int y = 0; y < src_size.y; y++) {
        for (int x = 0; x < src_size.x; x++) {
          const int2 src(x, y);
          apply_write(pixel_coord_from_result(*pixel, src),
                      value->load_pixel<Color, true>(src));
        }
      }
    }

    if (atomic == AtomicOperator::Mean) {
      parallel_for(out_size, [&](const int2 texel) {
        const int64_t idx = int64_t(texel.y) * int64_t(out_size.x) + int64_t(texel.x);
        const int n = write_count[idx];
        if (n > 1) {
          Color c = output_cpu.load_pixel<Color>(texel);
          const float inv = 1.0f / float(n);
          output_cpu.store_pixel(texel, Color(c.r * inv, c.g * inv, c.b * inv, c.a * inv));
        }
      });
    }

    if (this->context().use_gpu()) {
      Result output_gpu = output_cpu.upload_to_gpu(true);
      output.share_data(output_gpu);
      output_gpu.release();
    }
    else {
      output.share_data(output_cpu);
    }
    output_cpu.release();

    if (this->context().use_gpu()) {
      if (!pixel_in.is_single_value()) {
        pixel_cpu_storage.release();
      }
      if (!value_in.is_single_value()) {
        value_cpu_storage.release();
      }
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new WriteAtPixelOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeWriteAtPixel"_ustr, IMG_NODE_WRITE_AT_PIXEL);
  ntype.ui_name = "Write at Pixel";
  ntype.ui_description =
      "Scatter-write colors at pixel coordinates into a zero-initialized image";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_write_at_pixel_cc
