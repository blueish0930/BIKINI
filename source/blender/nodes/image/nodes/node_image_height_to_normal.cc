/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Convert a height (displacement) map into a tangent-space normal map. */

#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_height_to_normal_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("Height"_ustr)
      .default_value({0.5f, 0.5f, 0.5f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Height map; luminance (or red channel) is used as height");
  b.add_output<decl::Color>("Normal"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Tangent-space normal map (OpenGL-style, RGB = XYZ * 0.5 + 0.5)");

  b.add_input<decl::Float>("Scale"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(1000.0f)
      .description(
          "Relative strength on the unit square [0,1]² (not pixel-meter units). "
          "1.0: full black→white height across the plane gives a clear tilt");
  b.add_input<decl::Bool>("Invert"_ustr)
      .default_value(false)
      .description("Invert height before computing normals");
}

using namespace blender::compositor;

static float height_from_color(const Color &c)
{
  /* Rec. 709 luminance. */
  return c.r * 0.2126f + c.g * 0.7152f + c.b * 0.0722f;
}

class HeightToNormalOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    /* Prefer image resolution domain so height gradients are stable. */
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    Result &input = this->get_input("Height");
    Result &output = this->get_result("Normal");

    if (input.is_single_value()) {
      /* Flat height → flat normal pointing up. */
      output.allocate_single_value();
      output.set_single_value(Color(0.5f, 0.5f, 1.0f, 1.0f));
      return;
    }

    Result input_cpu_storage = this->context().create_result(ResultType::Color);
    const Result *input_cpu = &input;
    if (this->context().use_gpu()) {
      input_cpu_storage = input.download_to_cpu();
      input_cpu = &input_cpu_storage;
    }

    const Domain domain = this->compute_domain();
    Result output_cpu = this->context().create_result(ResultType::Color);
    output_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);

    const float scale = this->get_input("Scale").get_single_value_default<float>();
    const bool invert = this->get_input("Invert").get_single_value_default<bool>();
    const float sign = invert ? -1.0f : 1.0f;
    const int2 size = domain.data_size;
    /* Unit-square [0,1]² gradients: neighbor spacing is 1/size, so
     * ∂h/∂u = (h_r-h_l)/(2/size.x) = (h_r-h_l)*0.5*size.x. Scale is relative strength
     * on that plane (not treating pixels as meters). */
    const float scale_u = 0.5f * float(size.x) * scale * sign;
    const float scale_v = 0.5f * float(size.y) * scale * sign;

    parallel_for(size, [&](const int2 texel) {
      const int2 left = int2(math::max(texel.x - 1, 0), texel.y);
      const int2 right = int2(math::min(texel.x + 1, size.x - 1), texel.y);
      const int2 down = int2(texel.x, math::max(texel.y - 1, 0));
      const int2 up = int2(texel.x, math::min(texel.y + 1, size.y - 1));

      const float h_l = height_from_color(input_cpu->load_pixel<Color>(left));
      const float h_r = height_from_color(input_cpu->load_pixel<Color>(right));
      const float h_d = height_from_color(input_cpu->load_pixel<Color>(down));
      const float h_u = height_from_color(input_cpu->load_pixel<Color>(up));

      const float dx = (h_r - h_l) * scale_u;
      const float dy = (h_u - h_d) * scale_v;

      /* OpenGL tangent normal: (-dH/du, -dH/dv, 1). */
      float3 n = math::normalize(float3(-dx, -dy, 1.0f));
      output_cpu.store_pixel(
          texel, Color(n.x * 0.5f + 0.5f, n.y * 0.5f + 0.5f, n.z * 0.5f + 0.5f, 1.0f));
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
  return new HeightToNormalOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeHeightToNormal"_ustr);
  ntype.ui_name = "Height to Normal";
  ntype.ui_description =
      "Convert a height/displacement map into a tangent-space normal map using central "
      "differences";
  ntype.nclass = NODE_CLASS_OP_FILTER;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_height_to_normal_cc
