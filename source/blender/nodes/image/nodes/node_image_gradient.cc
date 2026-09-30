/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_vector_types.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_gradient_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("Image"_ustr)
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Image whose luminance gradient is computed");
  b.add_output<decl::Vector>("Gradient"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("2D gradient (X, Y) from central differences of luminance");
}

using namespace blender::compositor;

class GradientOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    const Result &input = this->get_input("Image");
    if (input.is_single_value()) {
      Result &output = this->get_result("Gradient");
      output.allocate_single_value();
      output.set_single_value(float3(0.0f));
      return;
    }

    if (this->context().use_gpu()) {
      this->execute_cpu(); /* Fallback to CPU for now. */
    }
    else {
      this->execute_cpu();
    }
  }

  void execute_cpu()
  {
    Result &input = get_input("Image");
    const Domain domain = compute_domain();
    Result &output = get_result("Gradient");
    output.allocate_texture(domain);

    const int2 size = domain.data_size;
    parallel_for(domain.data_size, [&](const int2 texel) {
      /* Central differences for gradient. */
      int2 left = texel;
      left.x = math::max(0, texel.x - 1);
      int2 right = texel;
      right.x = math::min(size.x - 1, texel.x + 1);
      int2 down = texel;
      down.y = math::max(0, texel.y - 1);
      int2 up = texel;
      up.y = math::min(size.y - 1, texel.y + 1);

      /* Compute gradient using luminance. */
      auto luminance = [](const Color &c) {
        return c.r * 0.2126f + c.g * 0.7152f + c.b * 0.0722f;
      };

      const float x_grad = luminance(input.load_pixel<Color>(right)) -
                           luminance(input.load_pixel<Color>(left));
      const float y_grad = luminance(input.load_pixel<Color>(up)) -
                           luminance(input.load_pixel<Color>(down));

      output.store_pixel(texel, float3(x_grad, y_grad, 0.0f));
    });
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new GradientOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeGradient"_ustr);
  ntype.ui_name = "Gradient";
  ntype.ui_description = "Compute the 2D image gradient using central differences";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_gradient_cc
