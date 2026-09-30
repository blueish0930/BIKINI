/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Approximate reconstruction of a height map from a tangent-space normal map. */

#include <limits>

#include "BLI_array.hh"
#include "BLI_math_base.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_normal_to_height_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("Normal"_ustr)
      .default_value({0.5f, 0.5f, 1.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Tangent-space normal map (OpenGL-style RGB)");
  b.add_output<decl::Color>("Height"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Reconstructed grayscale height map");

  b.add_input<decl::Float>("Scale"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(100.0f)
      .description(
          "Height amplitude multiplier around mid-gray (1.0 = fit reconstructed range to 0–1; "
          "higher exaggerates relief, lower flattens)");
  b.add_input<decl::Bool>("Invert"_ustr)
      .default_value(false)
      .description("Invert the reconstructed height");
}

using namespace blender::compositor;

class NormalToHeightOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    Result &input = this->get_input("Normal");
    Result &output = this->get_result("Height");

    if (input.is_single_value()) {
      /* Flat normal → mid-gray height (no slopes to integrate). */
      output.allocate_single_value();
      output.set_single_value(Color(0.5f, 0.5f, 0.5f, 1.0f));
      return;
    }

    Result input_cpu_storage = this->context().create_result(ResultType::Color);
    const Result *input_cpu = &input;
    if (this->context().use_gpu()) {
      input_cpu_storage = input.download_to_cpu();
      input_cpu = &input_cpu_storage;
    }

    const Domain domain = this->compute_domain();
    const int2 size = domain.data_size;
    const int pixel_count = size.x * size.y;
    const float scale = this->get_input("Scale").get_single_value_default<float>();
    const bool invert = this->get_input("Invert").get_single_value_default<bool>();

    Array<float> slope_x(pixel_count);
    Array<float> slope_y(pixel_count);

    parallel_for(size, [&](const int2 texel) {
      const Color c = input_cpu->load_pixel<Color>(texel);
      float3 n(c.r * 2.0f - 1.0f, c.g * 2.0f - 1.0f, c.b * 2.0f - 1.0f);
      n = math::normalize(n);
      const float nz = math::max(math::abs(n.z), 1e-5f);
      /* Match Height to Normal: n ≈ normalize(-dx, -dy, 1) → dx = -nx/nz, dy = -ny/nz.
       * Scale is applied later as height amplitude, not on slopes (would cancel after fit). */
      const int i = texel.y * size.x + texel.x;
      slope_x[i] = -n.x / nz;
      slope_y[i] = -n.y / nz;
    });

    /* Dual-direction integration then average (stable, non-FFT approximation). */
    Array<float> height_h(pixel_count, 0.0f);
    Array<float> height_v(pixel_count, 0.0f);

    /* Horizontal integration per row. */
    for (int y = 0; y < size.y; y++) {
      height_h[y * size.x] = 0.0f;
      for (int x = 1; x < size.x; x++) {
        const int i = y * size.x + x;
        const int i_prev = i - 1;
        height_h[i] = height_h[i_prev] + 0.5f * (slope_x[i] + slope_x[i_prev]);
      }
      /* Remove per-row mean to reduce drift. */
      float mean = 0.0f;
      for (int x = 0; x < size.x; x++) {
        mean += height_h[y * size.x + x];
      }
      mean /= float(size.x);
      for (int x = 0; x < size.x; x++) {
        height_h[y * size.x + x] -= mean;
      }
    }

    /* Vertical integration per column. */
    for (int x = 0; x < size.x; x++) {
      height_v[x] = 0.0f;
      for (int y = 1; y < size.y; y++) {
        const int i = y * size.x + x;
        const int i_prev = (y - 1) * size.x + x;
        height_v[i] = height_v[i_prev] + 0.5f * (slope_y[i] + slope_y[i_prev]);
      }
      float mean = 0.0f;
      for (int y = 0; y < size.y; y++) {
        mean += height_v[y * size.x + x];
      }
      mean /= float(size.y);
      for (int y = 0; y < size.y; y++) {
        height_v[y * size.x + x] -= mean;
      }
    }

    Array<float> height(pixel_count);
    float h_min = std::numeric_limits<float>::max();
    float h_max = std::numeric_limits<float>::lowest();
    for (int i = 0; i < pixel_count; i++) {
      height[i] = 0.5f * (height_h[i] + height_v[i]);
      h_min = math::min(h_min, height[i]);
      h_max = math::max(h_max, height[i]);
    }

    const float range = math::max(h_max - h_min, 1e-6f);
    const float h_mid = 0.5f * (h_min + h_max);
    Result output_cpu = this->context().create_result(ResultType::Color);
    output_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);

    /* Map integrated height to 0–1, then apply Scale as amplitude around 0.5 so the control
     * is a true height multiplier (not canceled by min–max normalize). Scale=1 fills 0–1. */
    parallel_for(size, [&](const int2 texel) {
      const int i = texel.y * size.x + texel.x;
      float h = 0.5f + ((height[i] - h_mid) / range) * scale;
      if (invert) {
        h = 1.0f - h;
      }
      h = math::clamp(h, 0.0f, 1.0f);
      output_cpu.store_pixel(texel, Color(h, h, h, 1.0f));
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
    if (this->context().use_gpu()) {
      input_cpu_storage.release();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new NormalToHeightOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeNormalToHeight"_ustr);
  ntype.ui_name = "Normal to Height";
  ntype.ui_description =
      "Approximate a height map from a tangent-space normal map by integrating slopes";
  ntype.nclass = NODE_CLASS_OP_FILTER;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_normal_to_height_cc
