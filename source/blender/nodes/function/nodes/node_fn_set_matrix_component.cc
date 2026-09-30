/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "FN_multi_function_registry.hh"

#include "GPU_material.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_set_matrix_component_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Matrix>("Matrix"_ustr)
      .optional_label()
      .description("Matrix to modify");
  b.add_output<decl::Matrix>("Matrix"_ustr)
      .align_with_previous()
      .description("Matrix with the selected component replaced");
  b.add_input<decl::Int>("Column"_ustr)
      .min(0)
      .max(3)
      .description("Column index in the range 0-3");
  b.add_input<decl::Int>("Row"_ustr).min(0).max(3).description("Row index in the range 0-3");
  b.add_input<decl::Float>("Value"_ustr).description("Value written into the selected component");
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction &fn = fn::multi_function::registry::lookup(
      "set(float4x4, int, int, float)"_ustr);
  builder.set_matching_fn(&fn);
}

static int node_gpu_material(GPUMaterial *mat,
                             bNode *node,
                             bNodeExecData * /*execdata*/,
                             GPUNodeStack *in,
                             GPUNodeStack *out)
{
  /* EEVEE represents integer sockets as floats, while the compositor uses native integers. */
  const bool index_as_float = in[1].type == GPU_FLOAT || in[2].type == GPU_FLOAT;
  const char *function_name = index_as_float ? "node_function_set_matrix_component_float" :
                                               "node_function_set_matrix_component";
  return GPU_stack_link(mat, node, function_name, in, out);
}

static void node_register()
{
  static bke::bNodeType ntype;

  fn_cmp_node_type_base(&ntype, "FunctionNodeSetMatrixComponent"_ustr);
  ntype.ui_name = "Set Matrix Component";
  ntype.ui_description = "Set a 4x4 matrix component by column and row index";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.build_multi_function = node_build_multi_function;
  ntype.gpu_fn = node_gpu_material;
  ntype.default_width = bke::NodeWidth::_160;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_set_matrix_component_cc
