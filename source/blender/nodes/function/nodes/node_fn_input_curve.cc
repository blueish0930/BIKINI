/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>

#include "BLI_set.hh"

#include "BKE_colortools.hh"
#include "BKE_node_runtime.hh"

#include "DNA_color_types.h"

#include "NOD_socket.hh"

#include "node_function_util.hh"
#include "node_shader_util.hh"

namespace blender::nodes::node_fn_input_curve_cc {

static eNodeSocketCurveSubtype curve_subtype_from_custom1(const int custom1)
{
  switch (custom1) {
    case NODE_SOCKET_CURVE_VECTOR:
      return NODE_SOCKET_CURVE_VECTOR;
    case NODE_SOCKET_CURVE_COLOR:
      return NODE_SOCKET_CURVE_COLOR;
    default:
      return NODE_SOCKET_CURVE_FLOAT;
  }
}

static int curve_subtype_channels(const eNodeSocketCurveSubtype subtype)
{
  switch (subtype) {
    case NODE_SOCKET_CURVE_VECTOR:
      return 3;
    case NODE_SOCKET_CURVE_COLOR:
      return 4;
    default:
      return 1;
  }
}

static eNodeSocketCurveSubtype curve_subtype_from_channels(const int channels)
{
  if (channels >= 4) {
    return NODE_SOCKET_CURVE_COLOR;
  }
  if (channels >= 3) {
    return NODE_SOCKET_CURVE_VECTOR;
  }
  return NODE_SOCKET_CURVE_FLOAT;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  /* `node_or_null` marks the declaration context-dependent so custom1 can change the subtype. */
  const bNode *node = b.node_or_null();
  const eNodeSocketCurveSubtype subtype = curve_subtype_from_custom1(node ? node->custom1 : 0);
  b.add_input<decl::Curve>("Curve"_ustr).subtype(subtype);
  b.add_output<decl::Curve>("Curve"_ustr).subtype(subtype);
}

static bNodeSocket *find_curve_socket(ListBaseT<bNodeSocket> &sockets)
{
  for (bNodeSocket &socket : sockets) {
    if (socket.type == SOCK_CURVE) {
      return &socket;
    }
  }
  return nullptr;
}

static eNodeSocketCurveSubtype target_curve_subtype(const bNodeSocket &socket)
{
  if (socket.runtime != nullptr && socket.runtime->declaration != nullptr) {
    if (const auto *curve_decl = dynamic_cast<const decl::Curve *>(socket.runtime->declaration)) {
      return curve_decl->subtype;
    }
  }
  if (const auto *value = socket.default_value_typed<bNodeSocketValueCurve>()) {
    return curve_subtype_from_custom1(value->curve_subtype);
  }
  return NODE_SOCKET_CURVE_FLOAT;
}

/** Highest channel count required by anything this output reaches, looking through reroutes. */
static void accumulate_required_curve_channels(const bNodeSocket &socket,
                                               int &required_channels,
                                               Set<const bNodeSocket *> &visited,
                                               const int depth)
{
  if (depth > 64 || required_channels >= 4) {
    return;
  }
  if (!visited.add(&socket)) {
    return;
  }
  for (const bNodeLink *link : socket.directly_linked_links()) {
    /* Link validation runs after node updates, so do not require NODE_LINK_VALID here. */
    if (link->is_muted() || link->tosock == nullptr) {
      continue;
    }
    const bNodeSocket &target = *link->tosock;
    const bNode &target_node = target.owner_node();
    if (target_node.is_reroute()) {
      if (const bNodeSocket *reroute_out = static_cast<bNodeSocket *>(target_node.outputs.first()))
      {
        accumulate_required_curve_channels(
            *reroute_out, required_channels, visited, depth + 1);
      }
      continue;
    }
    /* The value passes through another Curve node. Look at what that node feeds. */
    if (target_node.is_type("FunctionNodeInputCurve"_ustr) && target.type == SOCK_CURVE) {
      if (const bNodeSocket *forward_out = static_cast<bNodeSocket *>(target_node.outputs.first()))
      {
        if (forward_out->type == SOCK_CURVE) {
          accumulate_required_curve_channels(
              *forward_out, required_channels, visited, depth + 1);
        }
      }
      continue;
    }
    if (target.type == SOCK_CURVE) {
      required_channels = std::max(required_channels,
                                   curve_subtype_channels(target_curve_subtype(target)));
    }
  }
}

static bool socket_has_live_link(const bNodeSocket &socket)
{
  return socket.link != nullptr && (socket.link->flag & NODE_LINK_MUTED) == 0 &&
         socket.link->fromsock != nullptr;
}

static void copy_curve_to_output(bNodeSocket &dst, const bNodeSocket &src)
{
  const auto *src_value = src.default_value_typed<bNodeSocketValueCurve>();
  auto *dst_value = dst.default_value_typed<bNodeSocketValueCurve>();
  if (src_value == nullptr || dst_value == nullptr || src_value->curve == nullptr) {
    return;
  }
  if (dst_value->curve == src_value->curve) {
    dst_value->curve_subtype = src_value->curve_subtype;
    return;
  }
  BKE_curvemapping_free(dst_value->curve);
  dst_value->curve = BKE_curvemapping_copy(src_value->curve);
  dst_value->curve_subtype = src_value->curve_subtype;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  ntree->ensure_topology_cache();

  int required_channels = curve_subtype_channels(curve_subtype_from_custom1(node->custom1));
  if (const bNodeSocket *output = find_curve_socket(node->outputs)) {
    Set<const bNodeSocket *> visited;
    accumulate_required_curve_channels(*output, required_channels, visited, 0);
  }

  const eNodeSocketCurveSubtype required = curve_subtype_from_channels(required_channels);
  /* Only raise. A float curve plugged only into Float Curve must stay on channel 0. */
  if (node->custom1 != int(required)) {
    node->custom1 = int(required);
    /* A copied mapping can already have the channels while the socket is still tagged Float
     * (shader inlining assigns the curve without changing curve_subtype). Retag it so the
     * declaration refresh does not rebuild FLOAT→COLOR, which would keep channel 0 and drop C. */
    const int last = curve_subtype_channels(required) - 1;
    auto tag_if_channels_exist = [&](bNodeSocket *socket) {
      if (socket == nullptr) {
        return;
      }
      auto *value = socket->default_value_typed<bNodeSocketValueCurve>();
      if (value == nullptr || value->curve == nullptr || value->curve->cm[last].curve == nullptr) {
        return;
      }
      value->curve_subtype = required;
    };
    tag_if_channels_exist(find_curve_socket(node->inputs));
    tag_if_channels_exist(find_curve_socket(node->outputs));
    update_node_declaration_and_sockets(*ntree, *node);
  }

  bNodeSocket *input = find_curve_socket(node->inputs);
  bNodeSocket *output = find_curve_socket(node->outputs);
  if (input != nullptr && output != nullptr && !socket_has_live_link(*input)) {
    copy_curve_to_output(*output, *input);
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
      mf::DataType::ForSingle<CurveMapping *>());
}

static void node_register()
{
  static bke::bNodeType ntype;

  fn_cmp_node_type_base(&ntype, "FunctionNodeInputCurve"_ustr);
  ntype.ui_name = "Curve";
  ntype.ui_description = "Output a curve mapping, edited on the input or taken from a link";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.updatefunc = node_update;
  ntype.gpu_fn = gpu_forward;
  ntype.build_multi_function = node_build_multi_function;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_input_curve_cc
