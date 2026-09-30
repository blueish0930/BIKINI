/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 *
 * Billboard Displacement — world-space offset that rotates the object to
 * face / align with the view. Plug Displacement into Material Output.
 */

#include "node_shader_util.hh"

#include "DNA_node_types.h"

#include "BLT_translation.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

namespace blender {

namespace nodes::node_shader_billboard_displacement_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Position"_ustr)
      .hide_value()
      .description("World-space shading point. Unconnected: Geometry Position");
  b.add_input<decl::Vector>("Pivot"_ustr)
      .hide_value()
      .description("World-space rotation center. Unconnected: Object Info Location");
  auto &up = b.add_input<decl::Vector>("Up"_ustr)
                 .default_value({0.0f, 0.0f, 1.0f})
                 .min(-1.0f)
                 .max(1.0f)
                 .description("World-space up hint for Spherical / Cylindrical modes")
                 .make_available([](bNode &node) { node.custom1 = NODE_BILLBOARD_SPHERICAL; });
  b.add_input<decl::Float>("Factor"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(1.0f)
      .subtype(PROP_FACTOR)
      .description("0 = original pose, 1 = fully view-aligned");
  b.add_input<decl::Vector>("Normal"_ustr)
      .min(-1.0f)
      .max(1.0f)
      .hide_value()
      .description("World-space normal to rotate with the billboard. Unconnected: Geometry Normal");

  b.add_output<decl::Vector>("Displacement"_ustr)
      .description(
          "World-space offset (aligned position minus original). "
          "Connect to Material Output Displacement");
  b.add_output<decl::Vector>("Position"_ustr)
      .description("World-space position after the billboard rotation");
  b.add_output<decl::Vector>("Rotation"_ustr)
      .subtype(PROP_EULER)
      .description(
          "XYZ Euler of the billboard rotation (radians). "
          "Connect to Vector Rotate set to Euler to rotate extra vectors with the mesh");
  b.add_output<decl::Vector>("Normal"_ustr)
      .description("World-space normal after the billboard rotation");

  const bNode *node = b.node_or_null();
  const int mode = node ? int(node->custom1) : int(NODE_BILLBOARD_VIEW);
  up.available(mode != NODE_BILLBOARD_VIEW);
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "mode", UI_ITEM_NONE, IFACE_("Mode"), ICON_NONE);
  layout.prop(ptr, "axis", UI_ITEM_NONE, IFACE_("Axis"), ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_BILLBOARD_VIEW;
  node->custom2 = NODE_BILLBOARD_AXIS_Y;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  bNodeSocket *sock_up = bke::node_find_socket(*node, SOCK_IN, "Up"_ustr);
  if (sock_up) {
    bke::node_set_socket_availability(*ntree, *sock_up, node->custom1 != NODE_BILLBOARD_VIEW);
  }
}

static int node_gpu(GPUMaterial *mat,
                    bNode *node,
                    bNodeExecData * /*execdata*/,
                    GPUNodeStack *in,
                    GPUNodeStack *out)
{
  GPUNodeStack &pos_in = GPU_node_get_input(*node, in, "Position");
  GPUNodeStack &pivot_in = GPU_node_get_input(*node, in, "Pivot");

  const bNodeSocket *pos_sock = bke::node_find_socket(*node, SOCK_IN, "Position"_ustr);
  const bool pos_unlinked = (pos_sock == nullptr) || (pos_sock->link == nullptr) ||
                            ((pos_sock->link->flag & NODE_LINK_MUTED) != 0);
  if (pos_unlinked) {
    pos_in.link = nullptr;
    GPU_link(mat, "world_position_get", GPU_shading_data(), &pos_in.link);
  }

  const bNodeSocket *pivot_sock = bke::node_find_socket(*node, SOCK_IN, "Pivot"_ustr);
  const bool pivot_unlinked = (pivot_sock == nullptr) || (pivot_sock->link == nullptr) ||
                              ((pivot_sock->link->flag & NODE_LINK_MUTED) != 0);
  if (pivot_unlinked) {
    pivot_in.link = nullptr;
    GPU_link(mat, "object_location_get", GPU_kernel_globals(), GPU_shading_data(), &pivot_in.link);
  }

  GPUNodeStack &normal_in = GPU_node_get_input(*node, in, "Normal");
  const bNodeSocket *normal_sock = bke::node_find_socket(*node, SOCK_IN, "Normal"_ustr);
  const bool normal_unlinked = (normal_sock == nullptr) || (normal_sock->link == nullptr) ||
                               ((normal_sock->link->flag & NODE_LINK_MUTED) != 0);
  if (normal_unlinked) {
    normal_in.link = nullptr;
    GPU_link(mat, "world_normals_get", GPU_shading_data(), &normal_in.link);
  }

  float params[4];
  params[0] = float(node->custom1);
  params[1] = float(node->custom2);
  params[2] = 0.0f;
  params[3] = 0.0f;
  return GPU_stack_link(mat,
                        node,
                        "node_billboard_displacement",
                        in,
                        out,
                        GPU_constant(params),
                        GPU_kernel_globals(),
                        GPU_shading_data());
}

NODE_SHADER_MATERIALX_BEGIN
#ifdef WITH_MATERIALX
{
  return get_output_default(socket_out_->identifier, NodeItem::Type::Any);
}
#endif
NODE_SHADER_MATERIALX_END

}  // namespace nodes::node_shader_billboard_displacement_cc

void register_node_type_sh_billboard_displacement()
{
  namespace file_ns = nodes::node_shader_billboard_displacement_cc;

  static bke::bNodeType ntype;

  sh_node_type_base(
      &ntype, "ShaderNodeBillboardDisplacement"_ustr, SH_NODE_BILLBOARD_DISPLACEMENT);
  ntype.ui_name = "Billboard Displacement";
  ntype.ui_description =
      "World-space displacement that rotates the object to align with the view. "
      "Connect Displacement to Material Output; set Displacement to Displacement or Both";
  ntype.enum_name_legacy = "BILLBOARD_DISPLACEMENT";
  ntype.nclass = NODE_CLASS_OP_VECTOR;
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.gpu_fn = file_ns::node_gpu;
  ntype.materialx_fn = file_ns::node_shader_materialx;

  bke::node_register_type(ntype);
}

}  // namespace blender
