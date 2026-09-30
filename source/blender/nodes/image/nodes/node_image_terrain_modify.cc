/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include <cmath>

#include "BKE_node.hh"
#include "DNA_node_types.h"
#include "GPU_shader.hh"
#include "BLT_translation.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_image_terrain_common.hh"
#include "node_image_util.hh"

namespace blender::nodes::node_image_terrain_modify_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("Height"_ustr)
      .default_value({0.5f, 0.5f, 0.5f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic);
  b.add_output<decl::Color>("Height"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous();
  b.add_input<decl::Color>("Guide"_ustr)
      .default_value({0.5f, 0.5f, 0.5f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Warp / slope guide");
  b.add_input<decl::Float>("Amount"_ustr).default_value(0.55f).min(0.0f).max(2.0f);
  b.add_input<decl::Float>("Steps"_ustr)
      .default_value(8.0f)
      .min(2.0f)
      .max(64.0f)
      .description("Terrace / strata / fold bands");
  b.add_input<decl::Float>("Sharpness"_ustr).default_value(0.65f).min(0.0f).max(1.0f);
  b.add_input<decl::Float>("Seed"_ustr).default_value(0.0f).min(0.0f).max(10000.0f);
  b.add_input<decl::Float>("Min"_ustr).default_value(0.0f).min(0.0f).max(1.0f);
  b.add_input<decl::Float>("Max"_ustr).default_value(1.0f).min(0.0f).max(1.0f);
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.column(true).prop(ptr, "modify_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_TERRAIN_MOD_TERRACE;
}

using namespace blender::compositor;
using namespace blender::nodes::terrain;

class TerrainModifyOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    const Result &height = this->get_input("Height");
    if (!height.is_single_value() && height.is_allocated()) {
      return height.domain();
    }
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    gpu::Shader *shader = try_shader(this->context(), "compositor_terrain_modify");
    if (shader) {
      this->execute_gpu(shader);
      return;
    }
    this->execute_cpu();
  }

  void execute_gpu(gpu::Shader *shader)
  {
    const Domain domain = this->compute_domain();
    GPU_shader_bind(shader);
    GPU_shader_uniform_2iv(shader, "domain_size", domain.data_size);
    GPU_shader_uniform_1i(shader, "modify_type", math::clamp(int(this->node().custom1), 0, 9));
    GPU_shader_uniform_1f(shader, "amount", read_float(this->get_input("Amount"), 0.55f));
    GPU_shader_uniform_1f(shader, "steps", read_float(this->get_input("Steps"), 8.0f));
    GPU_shader_uniform_1f(shader, "sharpness", read_float(this->get_input("Sharpness"), 0.65f));
    GPU_shader_uniform_1f(shader, "seed", read_float(this->get_input("Seed"), 0.0f));
    GPU_shader_uniform_1f(shader, "min_h", read_float(this->get_input("Min"), 0.0f));
    GPU_shader_uniform_1f(shader, "max_h", read_float(this->get_input("Max"), 1.0f));
    gpu::Texture *ht = this->get_input("Height").bind_as_texture_or_single_value(shader, "height_tx");
    gpu::Texture *gt = this->get_input("Guide").bind_as_texture_or_single_value(shader, "guide_tx");
    Result &out = this->get_result("Height");
    out.allocate_texture(domain);
    out.bind_as_image(shader, "height_img");
    compute_dispatch_threads_at_least(shader, domain.data_size);
    out.unbind_as_image();
    this->get_input("Height").unbind_as_texture_or_single_value(ht);
    this->get_input("Guide").unbind_as_texture_or_single_value(gt);
    GPU_shader_unbind();
  }

  void execute_cpu()
  {
    const Domain domain = this->compute_domain();
    Result store = this->context().create_result(this->get_input("Height").type());
    const Result *hin = ensure_cpu(this->context(), this->get_input("Height"), store);
    Result cpu = this->context().create_result(ResultType::Color);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const int type = math::clamp(int(this->node().custom1), 0, 9);
    const float amount = read_float(this->get_input("Amount"), 0.55f);
    const float steps = math::max(read_float(this->get_input("Steps"), 8.0f), 2.0f);
    parallel_for(domain.data_size, [&](const int2 texel) {
      Color hc(0.5f, 0.5f, 0.5f, 1.0f);
      if (hin->is_single_value()) {
        hc = hin->get_single_value_default<Color>();
      }
      else if (hin->is_allocated()) {
        hc = hin->load_pixel<Color>(texel);
      }
      float h = luma(hc);
      if (type == NODE_TERRAIN_MOD_TERRACE || type == NODE_TERRAIN_MOD_STEPS) {
        h = math::interpolate(h, std::floor(h * steps) / steps, amount);
      }
      else if (type == NODE_TERRAIN_MOD_CLAMP) {
        h = math::interpolate(
            h,
            math::clamp(h, read_float(this->get_input("Min"), 0.0f), read_float(this->get_input("Max"), 1.0f)),
            amount);
      }
      cpu.store_pixel(texel, gray(h));
    });
    share_cpu_to_output(this->context(), this->get_result("Height"), cpu);
    if (store.is_allocated()) {
      store.release();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new TerrainModifyOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeTerrainModify"_ustr, IMG_NODE_TERRAIN_MODIFY);
  ntype.ui_name = "Terrain Modify";
  ntype.ui_description =
      "Terrace, stratify, warp, fold, thermal shaper, sandstone, clamp, recurve";
  ntype.nclass = NODE_CLASS_DISTORT;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_terrain_modify_cc
