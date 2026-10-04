/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_vector_types.hh"

#include "COM_node_operation.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_divergence_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Vector>("Vector"_ustr)
      .default_value({0.0f, 0.0f, 0.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("2D or 3D vector field; only X and Y are used for divergence");
  b.add_output<decl::Float>("Divergence"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("∂Vx/∂x + ∂Vy/∂y via central differences (pixel spacing 1)");
}

using namespace blender::compositor;

/** Read 2D velocity (X,Y) from common compositor vector encodings. */
static float2 load_vector_xy(const Result &input, const int2 texel)
{
  switch (input.type()) {
    case ResultType::Float2:
      return input.load_pixel<float2>(texel);
    case ResultType::Float3: {
      const float3 v = input.load_pixel<float3>(texel);
      return float2(v.x, v.y);
    }
    case ResultType::Float4: {
      const float4 v = input.load_pixel<float4>(texel);
      return float2(v.x, v.y);
    }
    case ResultType::Color: {
      const Color c = input.load_pixel<Color>(texel);
      return float2(c.r, c.g);
    }
    case ResultType::Float: {
      /* Degenerate: treat as 1D field on X only. */
      const float v = input.load_pixel<float>(texel);
      return float2(v, 0.0f);
    }
    default:
      return float2(0.0f);
  }
}

class DivergenceOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    Result &input = this->get_input("Vector");
    Result &output = this->get_result("Divergence");

    if (input.is_single_value()) {
      /* Constant field → zero divergence. */
      output.allocate_single_value();
      output.set_single_value(0.0f);
      return;
    }

    /* CPU finite differences need a CPU buffer. GPU Image Process paths must download first
     * (same pattern as Height to Normal / Histogram); reading GPU results as CPU yields zeros. */
    Result input_cpu_storage = this->context().create_result(input.type());
    const Result *input_cpu = &input;
    if (this->context().use_gpu()) {
      input_cpu_storage = input.download_to_cpu();
      input_cpu = &input_cpu_storage;
    }

    const Domain domain = this->compute_domain();
    Result output_cpu = this->context().create_result(ResultType::Float);
    output_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);

    const int2 size = domain.data_size;
    parallel_for(size, [&](const int2 texel) {
      /* Clamp-to-edge neighbors; spacing = 1 pixel → central difference / 2. */
      const int2 left(math::max(texel.x - 1, 0), texel.y);
      const int2 right(math::min(texel.x + 1, size.x - 1), texel.y);
      const int2 down(texel.x, math::max(texel.y - 1, 0));
      const int2 up(texel.x, math::min(texel.y + 1, size.y - 1));

      const float2 v_right = load_vector_xy(*input_cpu, right);
      const float2 v_left = load_vector_xy(*input_cpu, left);
      const float2 v_up = load_vector_xy(*input_cpu, up);
      const float2 v_down = load_vector_xy(*input_cpu, down);

      /* When clamped to a single neighbor (border), fall back to one-sided difference. */
      const float dx = (right.x != left.x) ? (v_right.x - v_left.x) / float(right.x - left.x) :
                                             0.0f;
      const float dy = (up.y != down.y) ? (v_up.y - v_down.y) / float(up.y - down.y) : 0.0f;
      const float divergence = dx + dy;

      output_cpu.store_pixel(texel, divergence);
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
  return new DivergenceOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeDivergence"_ustr);
  ntype.ui_name = "Divergence";
  ntype.ui_description =
      "Compute 2D divergence of a vector field (∂Vx/∂x + ∂Vy/∂y) with central differences";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_divergence_cc
