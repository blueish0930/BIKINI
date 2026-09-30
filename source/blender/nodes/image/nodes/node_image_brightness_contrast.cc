/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* IMAGE_NODES_MVP: port of Compositor Brightness/Contrast (GPU shader + multi-function). */

#include <limits>

#include "BLI_math_vector_types.hh"

#include "FN_multi_function_builder.hh"

#include "NOD_multi_function.hh"

#include "GPU_material.hh"

#include "COM_result.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_brightness_contrast_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.is_function_node();
  b.add_input<decl::Color>("Image"_ustr)
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .hide_value()
      .description("Image whose brightness and contrast are adjusted");
  b.add_output<decl::Color>("Image"_ustr)
      .align_with_previous()
      .description("Image after brightness and contrast");

  b.add_input<decl::Float>("Brightness"_ustr, "Bright"_ustr)
      .min(-100.0f)
      .max(100.0f)
      .description("Brightness offset (−100 to 100)");
  b.add_input<decl::Float>("Contrast"_ustr)
      .min(-100.0f)
      .max(100.0f)
      .description("Contrast amount (−100 to 100)");
}

using namespace blender::compositor;

static int node_gpu_material(GPUMaterial *material,
                             bNode *node,
                             bNodeExecData * /*execdata*/,
                             GPUNodeStack *inputs,
                             GPUNodeStack *outputs)
{
  /* Reuse compositor GPU library shader function. */
  return GPU_stack_link(material, node, "node_composite_bright_contrast", inputs, outputs);
}

/* Werner D. Streidt algorithm (same as compositor Brightness/Contrast). */
static float4 brightness_and_contrast(const float4 &color,
                                      const float brightness,
                                      const float contrast)
{
  float scaled_brightness = brightness / 100.0f;
  float delta = contrast / 200.0f;

  float multiplier, offset;
  if (contrast > 0.0f) {
    multiplier = 1.0f - delta * 2.0f;
    multiplier = 1.0f / math::max(multiplier, std::numeric_limits<float>::epsilon());
    offset = multiplier * (scaled_brightness - delta);
  }
  else {
    delta *= -1.0f;
    multiplier = math::max(1.0f - delta * 2.0f, 0.0f);
    offset = multiplier * scaled_brightness + delta;
  }

  return float4(color.xyz() * multiplier + offset, color.w);
}

using compositor::Color;

static void node_build_multi_function(nodes::NodeMultiFunctionBuilder &builder)
{
  static auto function = mf::build::SI3_SO<Color, float, float, Color>(
      "Image Brightness And Contrast",
      [](const Color &color, const float brightness, const float contrast) -> Color {
        return Color(brightness_and_contrast(float4(color), brightness, contrast));
      },
      mf::build::exec_presets::SomeSpanOrSingle<0>());
  builder.set_matching_fn(function);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeBrightContrast"_ustr);
  ntype.ui_name = "Brightness/Contrast";
  ntype.ui_description = "Adjust brightness and contrast";
  ntype.nclass = NODE_CLASS_OP_COLOR;
  ntype.declare = node_declare;
  ntype.gpu_fn = node_gpu_material;
  ntype.build_multi_function = node_build_multi_function;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_brightness_contrast_cc
