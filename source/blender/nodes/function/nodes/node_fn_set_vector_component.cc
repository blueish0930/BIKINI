/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "FN_multi_function_registry.hh"

#include "GPU_material.hh"

#include "node_function_util.hh"
#include "node_shader_util.hh"

namespace blender::nodes::node_fn_set_vector_component_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Vector>("Vector"_ustr)
      .optional_label()
      .description("Vector to modify");
  b.add_output<decl::Vector>("Vector"_ustr)
      .align_with_previous()
      .description("Vector with the selected component replaced");
  b.add_input<decl::Int>("Index"_ustr)
      .min(0)
      .max(2)
      .description("Component to write: 0 = X, 1 = Y, 2 = Z");
  b.add_input<decl::Float>("Value"_ustr).description("Value written into the selected component");
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction &fn = fn::multi_function::registry::lookup("set(float3, int, float)"_ustr);
  builder.set_matching_fn(&fn);
}

static int node_gpu_material(GPUMaterial *mat,
                             bNode *node,
                             bNodeExecData * /*execdata*/,
                             GPUNodeStack *in,
                             GPUNodeStack *out)
{
  /* EEVEE represents integer sockets as floats, while the compositor uses native integers. */
  const char *function_name = in[1].type == GPU_FLOAT ? "node_set_vector_component_float" :
                                                        "node_set_vector_component";
  return GPU_stack_link(mat, node, function_name, in, out);
}

static void node_register()
{
  static bke::bNodeType ntype;

  common_node_type_base(&ntype, "FunctionNodeSetVectorComponent"_ustr);
  ntype.ui_name = "Set Vector Component";
  ntype.ui_description = "Set the X, Y or Z component of a vector by index";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  ntype.gpu_fn = node_gpu_material;
  ntype.default_width = bke::NodeWidth::_160;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_set_vector_component_cc
