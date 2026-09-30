/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"
#include "BLI_utildefines.hh"

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

namespace blender::nodes::node_image_terrain_derive_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("Height"_ustr)
      .default_value({0.5f, 0.5f, 0.5f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic);
  b.add_output<decl::Color>("Mask"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Derived mask (slope, curvature, flow, …)");
  b.add_input<decl::Float>("Min"_ustr).default_value(0.0f).min(0.0f).max(10.0f);
  b.add_input<decl::Float>("Max"_ustr).default_value(1.0f).min(0.0f).max(10.0f);
  b.add_input<decl::Float>("Softness"_ustr).default_value(0.05f).min(0.0f).max(1.0f);
  b.add_input<decl::Float>("Azimuth"_ustr)
      .default_value(45.0f)
      .min(0.0f)
      .max(360.0f)
      .description("Aspect / lighting direction in degrees");
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.column(true).prop(ptr, "derive_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_TERRAIN_DRV_SLOPE;
}

using namespace blender::compositor;
using namespace blender::nodes::terrain;

class TerrainDeriveOperation : public NodeOperation {
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
    gpu::Shader *shader = try_shader(this->context(), "compositor_terrain_derive");
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
    GPU_shader_uniform_1i(shader, "derive_type", math::clamp(int(this->node().custom1), 0, 7));
    GPU_shader_uniform_1f(shader, "min_v", read_float(this->get_input("Min"), 0.0f));
    GPU_shader_uniform_1f(shader, "max_v", read_float(this->get_input("Max"), 1.0f));
    GPU_shader_uniform_1f(shader, "softness", read_float(this->get_input("Softness"), 0.05f));
    GPU_shader_uniform_1f(shader, "azimuth", read_float(this->get_input("Azimuth"), 45.0f));
    gpu::Texture *ht = this->get_input("Height").bind_as_texture_or_single_value(shader, "height_tx");
    Result &out = this->get_result("Mask");
    out.allocate_texture(domain);
    out.bind_as_image(shader, "mask_img");
    compute_dispatch_threads_at_least(shader, domain.data_size);
    out.unbind_as_image();
    this->get_input("Height").unbind_as_texture_or_single_value(ht);
    GPU_shader_unbind();
  }

  void execute_cpu()
  {
    const Domain domain = this->compute_domain();
    Result store = this->context().create_result(this->get_input("Height").type());
    const Result *hin = ensure_cpu(this->context(), this->get_input("Height"), store);
    Result cpu = this->context().create_result(ResultType::Color);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const int2 size = domain.data_size;
    const float lo = read_float(this->get_input("Min"), 0.0f);
    const float hi = read_float(this->get_input("Max"), 1.0f);
    parallel_for(size, [&](const int2 texel) {
      auto sample = [&](int2 t) {
        t.x = math::clamp(t.x, 0, size.x - 1);
        t.y = math::clamp(t.y, 0, size.y - 1);
        if (hin->is_single_value()) {
          return luma(hin->get_single_value_default<Color>());
        }
        return luma(hin->load_pixel<Color>(t));
      };
      const float h = sample(texel);
      const float hl = sample(texel + int2(-1, 0));
      const float hr = sample(texel + int2(1, 0));
      const float hd = sample(texel + int2(0, -1));
      const float hu = sample(texel + int2(0, 1));
      const float slope = math::length(float2(hr - hl, hu - hd)) * 0.5f * float(size.x);
      const float t = smoothstep(math::min(lo, hi), math::max(lo, hi) + 1.0e-5f, slope);
      UNUSED_VARS(h, hd, hu);
      cpu.store_pixel(texel, gray(t));
    });
    share_cpu_to_output(this->context(), this->get_result("Mask"), cpu);
    if (store.is_allocated()) {
      store.release();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new TerrainDeriveOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeTerrainDerive"_ustr, IMG_NODE_TERRAIN_DERIVE);
  ntype.ui_name = "Terrain Derive";
  ntype.ui_description = "Slope, curvature, flow, height, peaks, occlusion, aspect and soil masks";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_terrain_derive_cc
