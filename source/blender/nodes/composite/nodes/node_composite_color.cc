/* SPDX-FileCopyrightText: 2006 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_node_types.h"

#include "GPU_context.hh"
#include "GPU_texture.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "UI_interface.hh"
#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_composite_util.hh"

namespace blender::nodes::node_composite_color_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Color>("Color"_ustr)
      .default_value({0.5f, 0.5f, 0.5f, 1.0f})
      .custom_draw([](CustomSocketDrawParams &params) {
        params.layout.alignment_set(ui::LayoutAlign::Expand);
        ui::Layout &col = params.layout.column(false);
        template_color_picker(
            &col, &params.socket_ptr, "default_value", true, false, false, false);
        col.prop(&params.socket_ptr,
                 "default_value",
                 ui::ITEM_R_SLIDER | ui::ITEM_R_SPLIT_EMPTY_NAME,
                 "",
                 ICON_NONE);
      });
}

using namespace blender::compositor;

class RGBOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  void execute() override
  {
    Result &result = get_result("Color");
    Color color(0.5f, 0.5f, 0.5f, 1.0f);
    /* `default_value != nullptr` is not enough: a sentinel like 0x230 crashes in Color(). */
    const bNodeSocket *socket = this->node().outputs.first();
    if (socket != nullptr && intptr_t(socket) >= 4096 && socket->type == SOCK_RGBA &&
        intptr_t(socket->default_value) >= 4096)
    {
      color = Color(static_cast<const bNodeSocketValueRGBA *>(socket->default_value)->value);
    }

    /* Image Process: always emit a full texture at the editor texture resolution.
     * Do not use owner_tree().type (null tree → read at offset ~0x230) or the texture pool
     * (null GPUContext → same offset on texture_pool). */
    if (this->context().is_image_process()) {
      const Domain domain = this->context().get_compositing_domain();
      const bool can_gpu = this->context().use_gpu() && GPU_context_active_get() != nullptr;
      if (can_gpu) {
        result.allocate_texture(domain, false, ResultStorageType::GPUImage);
        if (gpu::Texture *tex = result.gpu_texture()) {
          GPU_texture_clear(tex, GPU_DATA_FLOAT, color);
          return;
        }
        result.release();
      }
      result.allocate_texture(domain, false, ResultStorageType::CPUImage);
      parallel_for(domain.data_size, [&](const int2 texel) { result.store_pixel(texel, color); });
      return;
    }

    result.allocate_single_value();
    result.set_single_value(color);
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new RGBOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  cmp_node_type_base(&ntype, "CompositorNodeRGB"_ustr, CMP_NODE_RGB);
  ntype.ui_name = "Color";
  ntype.ui_description = "A color picker";
  ntype.enum_name_legacy = "RGB";
  ntype.nclass = NODE_CLASS_INPUT;
  ntype.declare = node_declare;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_composite_color_cc
