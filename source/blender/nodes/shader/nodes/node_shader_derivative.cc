/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 *
 * Screen-space derivatives (EEVEE): DDX, DDY, Filter Width.
 * Cycles has no 2x2 raster quads; the node is ignored there.
 */

#include "node_shader_util.hh"

#include "BKE_context.hh"
#include "BKE_scene.hh"

#include "DNA_node_types.h"
#include "DNA_scene_types.h"

#include "BLI_string_utf8.hh"

#include "BLT_translation.hh"

#include "NOD_node_extra_info.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

namespace blender {

namespace nodes::node_shader_derivative_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Value"_ustr)
      .default_value(0.0f)
      .min(-10000.0f)
      .max(10000.0f)
      .description("Scalar to differentiate in screen space");
  b.add_input<decl::Vector>("Vector"_ustr)
      .min(-10000.0f)
      .max(10000.0f)
      .description("Vector to differentiate in screen space");
  b.add_input<decl::Color>("Color"_ustr)
      .default_value({0.8f, 0.8f, 0.8f, 1.0f})
      .description("Color to differentiate in screen space");
  b.add_output<decl::Float>("Value"_ustr).description("Screen-space derivative of Value");
  b.add_output<decl::Vector>("Vector"_ustr).description("Screen-space derivative of Vector");
  b.add_output<decl::Color>("Color"_ustr).description("Screen-space derivative of Color");
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "operation", ui::ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
  layout.prop(ptr, "data_type", ui::ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_SHADER_DERIVATIVE_DDX;
  node->custom2 = SOCK_FLOAT;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const int type = node->custom2;
  for (bNodeSocket &socket : node->inputs) {
    bke::node_set_socket_availability(*ntree, socket, socket.type == type);
  }
  for (bNodeSocket &socket : node->outputs) {
    bke::node_set_socket_availability(*ntree, socket, socket.type == type);
  }
}

static int node_ui_class(const bNode *node)
{
  switch (node->custom2) {
    case SOCK_VECTOR:
      return NODE_CLASS_OP_VECTOR;
    case SOCK_RGBA:
      return NODE_CLASS_OP_COLOR;
    default:
      return NODE_CLASS_CONVERTER;
  }
}

static void node_label(const bNodeTree * /*ntree*/,
                       const bNode *node,
                       char *label,
                       int label_maxncpy)
{
  switch (node->custom1) {
    case NODE_SHADER_DERIVATIVE_DDY:
      BLI_strncpy_utf8(label, IFACE_("DDY"), label_maxncpy);
      break;
    case NODE_SHADER_DERIVATIVE_FWIDTH:
      BLI_strncpy_utf8(label, IFACE_("Filter Width"), label_maxncpy);
      break;
    default:
      BLI_strncpy_utf8(label, IFACE_("DDX"), label_maxncpy);
      break;
  }
}

static void node_extra_info(NodeExtraInfoParams &parameters)
{
  const Scene *scene = CTX_data_scene(&parameters.C);
  if (scene && BKE_scene_uses_blender_eevee(scene)) {
    return;
  }
  NodeExtraInfoRow row;
  row.text = IFACE_("EEVEE Only");
  row.tooltip = TIP_(
      "DDX/DDY need raster 2x2 pixel quads. Cycles cannot evaluate screen-space derivatives");
  row.icon = ICON_STATUS_INFO;
  parameters.rows.append(std::move(row));
}

static int node_gpu(GPUMaterial *mat,
                    bNode *node,
                    bNodeExecData * /*execdata*/,
                    GPUNodeStack *in,
                    GPUNodeStack *out)
{
  const char *name = "node_ddx";
  switch (node->custom1) {
    case NODE_SHADER_DERIVATIVE_DDY:
      name = "node_ddy";
      break;
    case NODE_SHADER_DERIVATIVE_FWIDTH:
      name = "node_fwidth";
      break;
    default:
      break;
  }
  /* `kg` is not a socket. Screen-space scale lives on KernelGlobals. */
  return GPU_stack_link(mat, node, name, in, out, GPU_kernel_globals());
}

class SocketSearchOp {
 public:
  UString socket_name;
  int operation = NODE_SHADER_DERIVATIVE_DDX;
  int data_type = SOCK_FLOAT;

  void operator()(LinkSearchOpParams &params) const
  {
    bNode &node = params.add_node("ShaderNodeDerivative"_ustr);
    node.custom1 = operation;
    node.custom2 = data_type;
    params.update_and_connect_available_socket(node, socket_name);
  }
};

static std::optional<int> data_type_from_socket(const bNodeSocket &socket)
{
  switch (socket.type) {
    case SOCK_FLOAT:
    case SOCK_BOOLEAN:
    case SOCK_INT:
      return SOCK_FLOAT;
    case SOCK_VECTOR:
    case SOCK_INT_VECTOR:
      return SOCK_VECTOR;
    case SOCK_RGBA:
      return SOCK_RGBA;
    default:
      return {};
  }
}

static UString socket_name_for_type(const int data_type)
{
  switch (data_type) {
    case SOCK_VECTOR:
      return "Vector"_ustr;
    case SOCK_RGBA:
      return "Color"_ustr;
    default:
      return "Value"_ustr;
  }
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const std::optional<int> data_type = data_type_from_socket(params.other_socket());
  if (!data_type) {
    return;
  }
  const UString socket_name = socket_name_for_type(*data_type);
  params.add_item(IFACE_("DDX"),
                  SocketSearchOp{socket_name, NODE_SHADER_DERIVATIVE_DDX, *data_type},
                  0);
  params.add_item(IFACE_("DDY"),
                  SocketSearchOp{socket_name, NODE_SHADER_DERIVATIVE_DDY, *data_type},
                  -1);
  params.add_item(IFACE_("Filter Width"),
                  SocketSearchOp{socket_name, NODE_SHADER_DERIVATIVE_FWIDTH, *data_type},
                  -1);
}

}  // namespace nodes::node_shader_derivative_cc

void register_node_type_sh_derivative()
{
  namespace file_ns = nodes::node_shader_derivative_cc;

  static bke::bNodeType ntype;

  sh_node_type_base(&ntype, "ShaderNodeDerivative"_ustr, SH_NODE_DERIVATIVE);
  ntype.ui_name = "DDX / DDY";
  ntype.ui_description =
      "Screen-space partial derivative of a float, vector, or color (EEVEE). "
      "DDX/DDY match HLSL ddx/ddy; Filter Width is abs(DDX)+abs(DDY). Cycles ignores this node";
  ntype.enum_name_legacy = "DERIVATIVE";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.ui_class = file_ns::node_ui_class;
  ntype.labelfunc = file_ns::node_label;
  ntype.get_extra_info = file_ns::node_extra_info;
  ntype.gpu_fn = file_ns::node_gpu;
  ntype.gather_link_search_ops = file_ns::node_gather_link_searches;

  bke::node_register_type(ntype);
}

}  // namespace blender
