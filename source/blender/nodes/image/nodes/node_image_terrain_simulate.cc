/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include <utility>

#include "BKE_node.hh"
#include "BLI_utildefines.hh"
#include "DNA_node_types.h"
#include "GPU_shader.hh"
#include "BLT_translation.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_image_terrain_common.hh"
#include "node_image_util.hh"

namespace blender::nodes::node_image_terrain_simulate_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("Height"_ustr)
      .default_value({0.5f, 0.5f, 0.5f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Input heightfield");
  b.add_output<decl::Color>("Height"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous()
      .description("Modified heightfield");
  b.add_output<decl::Color>("Mask"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Snow / water / debris mask");

  b.add_input<decl::Color>("Mask"_ustr)
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Restrict the effect");
  b.add_input<decl::Float>("Strength"_ustr)
      .default_value(0.7f)
      .min(0.0f)
      .max(4.0f);
  b.add_input<decl::Float>("Coverage"_ustr)
      .default_value(0.65f)
      .min(0.0f)
      .max(1.0f);
  b.add_input<decl::Float>("Melt"_ustr)
      .default_value(0.25f)
      .min(0.0f)
      .max(1.0f)
      .description("Reduce snow on lower / sunnier slopes");
  b.add_input<decl::Float>("Height Min"_ustr)
      .default_value(0.45f)
      .min(0.0f)
      .max(1.0f);
  b.add_input<decl::Float>("Height Max"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(1.0f);
  b.add_input<decl::Float>("Slope Max"_ustr)
      .default_value(1.2f)
      .min(0.01f)
      .max(8.0f);
  b.add_input<decl::Float>("Sea Level"_ustr)
      .default_value(0.22f)
      .min(0.0f)
      .max(1.0f);
  b.add_input<decl::Float>("Seed"_ustr).default_value(0.0f).min(0.0f).max(10000.0f);
  b.add_input<decl::Float>("Iterations"_ustr)
      .default_value(8.0f)
      .min(1.0f)
      .max(64.0f)
      .description("Passes for Rivers / Lake / Glacier");
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.column(true).prop(ptr, "simulate_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_TERRAIN_SIM_SNOW;
}

using namespace blender::compositor;
using namespace blender::nodes::terrain;

class TerrainSimulateOperation : public NodeOperation {
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
    gpu::Shader *shader = try_shader(this->context(), "compositor_terrain_simulate");
    if (shader) {
      this->execute_gpu(shader);
      return;
    }
    this->execute_cpu();
  }

  void execute_gpu(gpu::Shader *shader)
  {
    const Domain domain = this->compute_domain();
    const int2 size = domain.data_size;
    const int type = math::clamp(int(this->node().custom1), 0, 9);
    int passes = 1;
    if (ELEM(type,
             NODE_TERRAIN_SIM_RIVERS,
             NODE_TERRAIN_SIM_LAKE,
             NODE_TERRAIN_SIM_GLACIER,
             NODE_TERRAIN_SIM_CRUMBLE))
    {
      passes = math::clamp(int(read_float(this->get_input("Iterations"), 8.0f)), 1, 32);
    }

    Result ping = this->context().create_result(ResultType::Color);
    Result pong = this->context().create_result(ResultType::Color);
    ping.allocate_texture(domain, false);
    pong.allocate_texture(domain, false);

    Result &height_in = this->get_input("Height");
    Result &mask_in = this->get_input("Mask");
    Result *src = &ping;
    Result *dst = &pong;

    /* First pass reads the node input; later passes ping-pong. */
    Result &h_out = this->get_result("Height");
    Result &m_out = this->get_result("Mask");
    h_out.allocate_texture(domain);
    m_out.allocate_texture(domain);

    for (int i = 0; i < passes; i++) {
      GPU_shader_bind(shader);
      GPU_shader_uniform_2iv(shader, "domain_size", size);
      GPU_shader_uniform_1i(shader, "sim_type", type);
      GPU_shader_uniform_1f(shader, "strength", read_float(this->get_input("Strength"), 0.7f));
      GPU_shader_uniform_1f(shader, "coverage", read_float(this->get_input("Coverage"), 0.65f));
      GPU_shader_uniform_1f(shader, "melt", read_float(this->get_input("Melt"), 0.25f));
      GPU_shader_uniform_1f(shader, "height_min", read_float(this->get_input("Height Min"), 0.45f));
      GPU_shader_uniform_1f(shader, "height_max", read_float(this->get_input("Height Max"), 1.0f));
      GPU_shader_uniform_1f(shader, "slope_max", read_float(this->get_input("Slope Max"), 1.2f));
      GPU_shader_uniform_1f(shader, "sea_level", read_float(this->get_input("Sea Level"), 0.22f));
      GPU_shader_uniform_1f(shader, "seed", read_float(this->get_input("Seed"), 0.0f));
      GPU_shader_uniform_1f(shader, "iterations", float(i));
      gpu::Texture *ht = nullptr;
      gpu::Texture *mt = mask_in.bind_as_texture_or_single_value(shader, "mask_tx");
      if (i == 0) {
        ht = height_in.bind_as_texture_or_single_value(shader, "height_tx");
      }
      else {
        src->bind_as_texture(shader, "height_tx");
      }
      Result &out_h = (i == passes - 1) ? h_out : *dst;
      m_out.bind_as_image(shader, "mask_img");
      out_h.bind_as_image(shader, "height_img");
      compute_dispatch_threads_at_least(shader, size);
      out_h.unbind_as_image();
      m_out.unbind_as_image();
      if (i == 0) {
        height_in.unbind_as_texture_or_single_value(ht);
      }
      else {
        src->unbind_as_texture();
      }
      mask_in.unbind_as_texture_or_single_value(mt);
      GPU_shader_unbind();
      gpu_barrier();
      if (i != passes - 1) {
        std::swap(src, dst);
      }
    }
    ping.release();
    pong.release();
  }

  void execute_cpu()
  {
    const Domain domain = this->compute_domain();
    Result store = this->context().create_result(this->get_input("Height").type());
    const Result *hin = ensure_cpu(this->context(), this->get_input("Height"), store);
    Result cpu = this->context().create_result(ResultType::Color);
    Result mask_cpu = this->context().create_result(ResultType::Color);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    mask_cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const float strength = read_float(this->get_input("Strength"), 0.7f);
    const float sea = read_float(this->get_input("Sea Level"), 0.22f);
    const int type = math::clamp(int(this->node().custom1), 0, 9);
    parallel_for(domain.data_size, [&](const int2 texel) {
      Color hc(0.5f, 0.5f, 0.5f, 1.0f);
      if (hin->is_single_value()) {
        hc = hin->get_single_value_default<Color>();
      }
      else if (hin->is_allocated()) {
        hc = hin->load_pixel<Color>(texel);
      }
      float h = luma(hc);
      float mask = 0.0f;
      if (type <= 2) {
        mask = math::clamp(strength * smoothstep(0.4f, 0.9f, h), 0.0f, 1.0f);
        h += mask * 0.05f;
      }
      else if (type == NODE_TERRAIN_SIM_SEA) {
        mask = h < sea ? 1.0f : 0.0f;
        if (h < sea) {
          h = math::interpolate(h, sea, 0.3f * strength);
        }
      }
      else {
        mask = 0.0f;
      }
      cpu.store_pixel(texel, gray(h));
      mask_cpu.store_pixel(texel, gray(mask));
    });
    share_cpu_to_output(this->context(), this->get_result("Height"), cpu);
    share_cpu_to_output(this->context(), this->get_result("Mask"), mask_cpu);
    if (store.is_allocated()) {
      store.release();
    }
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new TerrainSimulateOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeTerrainSimulate"_ustr, IMG_NODE_TERRAIN_SIMULATE);
  ntype.ui_name = "Terrain Simulate";
  ntype.ui_description =
      "Snow, rivers, sea, lake, sediments, crumble, glacier and anastomosis";
  ntype.nclass = NODE_CLASS_OP_FILTER;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_terrain_simulate_cc
