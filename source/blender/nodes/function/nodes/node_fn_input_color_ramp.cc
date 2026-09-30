/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "MEM_guardedalloc.h"

#include "DNA_color_types.h"
#include "DNA_colorband_types.h"

#include "node_function_util.hh"
#include "node_shader_util.hh"

namespace blender::nodes::node_fn_input_color_ramp_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::ColorRamp>("Color Ramp"_ustr);
  b.add_output<decl::ColorRamp>("Color Ramp"_ustr);
}

static bNodeSocket *find_ramp_socket(ListBaseT<bNodeSocket> &sockets)
{
  for (bNodeSocket &socket : sockets) {
    if (socket.type == SOCK_COLOR_RAMP) {
      return &socket;
    }
  }
  return nullptr;
}

static bool socket_has_live_link(const bNodeSocket &socket)
{
  return socket.link != nullptr && (socket.link->flag & NODE_LINK_MUTED) == 0 &&
         socket.link->fromsock != nullptr;
}

static void copy_ramp_to_output(bNodeSocket &dst, const bNodeSocket &src)
{
  const auto *src_value = src.default_value_typed<bNodeSocketValueColorRamp>();
  auto *dst_value = dst.default_value_typed<bNodeSocketValueColorRamp>();
  if (src_value == nullptr || dst_value == nullptr || src_value->band == nullptr) {
    return;
  }
  if (dst_value->band == src_value->band) {
    return;
  }
  MEM_delete(dst_value->band);
  dst_value->band = MEM_new<ColorBand>(__func__, *src_value->band);
}

static void node_update(bNodeTree * /*ntree*/, bNode *node)
{
  bNodeSocket *input = find_ramp_socket(node->inputs);
  bNodeSocket *output = find_ramp_socket(node->outputs);
  if (input != nullptr && output != nullptr && !socket_has_live_link(*input)) {
    copy_ramp_to_output(*output, *input);
  }
}

static int gpu_forward(GPUMaterial * /*mat*/,
                       bNode * /*node*/,
                       bNodeExecData * /*execdata*/,
                       GPUNodeStack *in,
                       GPUNodeStack *out)
{
  out[0].link = in[0].link;
  out[0].type = in[0].type;
  out[0].sockettype = in[0].sockettype;
  out[0].hasoutput = true;
  return 1;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  builder.construct_and_set_matching_fn<mf::CustomMF_GenericCopy>(
      mf::DataType::ForSingle<ColorBand *>());
}

static void node_register()
{
  static bke::bNodeType ntype;

  fn_cmp_node_type_base(&ntype, "FunctionNodeInputColorRamp"_ustr);
  ntype.ui_name = "Color Ramp";
  ntype.ui_description = "Output a color ramp, edited on the input or taken from a link";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.updatefunc = node_update;
  ntype.gpu_fn = gpu_forward;
  ntype.build_multi_function = node_build_multi_function;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_input_color_ramp_cc
