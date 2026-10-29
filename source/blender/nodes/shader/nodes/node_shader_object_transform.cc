/* SPDX-FileCopyrightText: 2026 Blender Authors
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_shader_util.hh"

#include "BKE_shader_object_transform.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

namespace blender {
namespace nodes::node_shader_object_transform_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Matrix>("Transform"_ustr)
      .description("World-space transformation matrix of the specified object");
  b.add_output<decl::Vector>("Location"_ustr);
  b.add_output<decl::Rotation>("Rotation"_ustr);
  b.add_output<decl::Vector>("Scale"_ustr);
  b.add_output<decl::Int>("Seed"_ustr)
      .description("Session-unique object seed, matching Geometry Nodes Object Info");
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "object", ui::ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static int node_gpu(GPUMaterial *mat, bNode *node, bNodeExecData * /*execdata*/, GPUNodeStack *in,
                    GPUNodeStack *out)
{
  const Object *object = reinterpret_cast<Object *>(node->id);
  GPUNodeLink *values[7];
  const float4 zero(0.0f);
  for (const int field : IndexRange(7)) {
    const bool needed = field < 4 ? (out[0].hasoutput || (field == 3 && out[1].hasoutput))
                                  : out[field - 2].hasoutput;
    if (!needed) {
      values[field] = GPU_constant(zero);
      continue;
    }
    const std::string name = bke::shader_object_transform_attribute_name(object, field);
    float hash;
    GPUNodeLink *attribute =
        GPU_uniform_attribute(mat, name.c_str(), false, reinterpret_cast<uint32_t *>(&hash));
    GPU_link(mat, "node_attribute_uniform", attribute, GPU_constant(&hash), GPU_kernel_globals(),
             GPU_shading_data(), &values[field]);
  }
  return GPU_stack_link(mat, node, "node_object_transform", in, out, values[0], values[1],
                        values[2], values[3], values[4], values[5], values[6]);
}
} // namespace nodes::node_shader_object_transform_cc

void register_node_type_sh_object_transform()
{
  namespace file_ns = nodes::node_shader_object_transform_cc;
  static bke::bNodeType ntype;
  sh_node_type_base(&ntype, "ShaderNodeObjectTransform"_ustr);
  ntype.ui_name = "Object Transform";
  ntype.ui_description = "Retrieve the world-space transform and seed of a specified object";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.gpu_fn = file_ns::node_gpu;
  bke::node_register_type(ntype);
}
} // namespace blender
