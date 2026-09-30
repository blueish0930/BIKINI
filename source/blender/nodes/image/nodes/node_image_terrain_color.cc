/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

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

namespace blender::nodes::node_image_terrain_color_cc {

static void node_declare_color(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("Height"_ustr)
      .default_value({0.5f, 0.5f, 0.5f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic);
  b.add_output<decl::Color>("Color"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous();
  b.add_input<decl::Color>("Flow"_ustr)
      .default_value({0.5f, 0.5f, 0.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .description("Optional flow map (B = wetness)");
  b.add_input<decl::Float>("Sea Level"_ustr).default_value(0.22f).min(0.0f).max(1.0f);
  b.add_input<decl::Float>("Snow Level"_ustr).default_value(0.78f).min(0.0f).max(1.0f);
  b.add_input<decl::Float>("Weathering"_ustr).default_value(0.4f).min(0.0f).max(1.0f);
  b.add_input<decl::Color>("Water"_ustr).default_value({0.10f, 0.22f, 0.38f, 1.0f});
  b.add_input<decl::Color>("Sand"_ustr).default_value({0.76f, 0.68f, 0.45f, 1.0f});
  b.add_input<decl::Color>("Grass"_ustr).default_value({0.28f, 0.42f, 0.18f, 1.0f});
  b.add_input<decl::Color>("Rock"_ustr).default_value({0.38f, 0.36f, 0.34f, 1.0f});
  b.add_input<decl::Color>("Snow"_ustr).default_value({0.92f, 0.94f, 0.96f, 1.0f});
}

static void node_layout_color(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.column(true).prop(ptr, "color_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
}

static void node_init_color(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_TERRAIN_COL_SATMAP;
}

static void node_declare_combine(NodeDeclarationBuilder &b)
{
  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_input<decl::Color>("A"_ustr)
      .default_value({0.5f, 0.5f, 0.5f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic);
  b.add_output<decl::Color>("Height"_ustr)
      .structure_type(StructureType::Dynamic)
      .align_with_previous();
  b.add_input<decl::Color>("B"_ustr)
      .default_value({0.5f, 0.5f, 0.5f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic);
  b.add_input<decl::Color>("Mask"_ustr)
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .hide_value()
      .structure_type(StructureType::Dynamic);
  b.add_input<decl::Float>("Factor"_ustr).default_value(1.0f).min(0.0f).max(1.0f);
}

static void node_layout_combine(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.column(true).prop(ptr, "combine_type", UI_ITEM_NONE, IFACE_("Mode"), ICON_NONE);
}

static void node_init_combine(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_TERRAIN_CMB_MAX;
}

using namespace blender::compositor;
using namespace blender::nodes::terrain;

class TerrainColorOperation : public NodeOperation {
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
    gpu::Shader *shader = try_shader(this->context(), "compositor_terrain_color");
    if (!shader) {
      Result &out = this->get_result("Color");
      out.allocate_invalid();
      return;
    }
    const Domain domain = this->compute_domain();
    GPU_shader_bind(shader);
    GPU_shader_uniform_2iv(shader, "domain_size", domain.data_size);
    GPU_shader_uniform_1i(shader, "color_type", math::clamp(int(this->node().custom1), 0, 3));
    GPU_shader_uniform_1f(shader, "sea_level", read_float(this->get_input("Sea Level"), 0.22f));
    GPU_shader_uniform_1f(shader, "snow_level", read_float(this->get_input("Snow Level"), 0.78f));
    GPU_shader_uniform_1f(shader, "weathering", read_float(this->get_input("Weathering"), 0.4f));
    auto bind_col = [&](const char *sock, const char *uni, const Color fb) {
      const Color c = read_color(this->get_input(sock), fb);
      const float v[4] = {c.r, c.g, c.b, c.a};
      GPU_shader_uniform_4fv(shader, uni, v);
    };
    bind_col("Water", "water_color", Color(0.10f, 0.22f, 0.38f, 1.0f));
    bind_col("Sand", "sand_color", Color(0.76f, 0.68f, 0.45f, 1.0f));
    bind_col("Grass", "grass_color", Color(0.28f, 0.42f, 0.18f, 1.0f));
    bind_col("Rock", "rock_color", Color(0.38f, 0.36f, 0.34f, 1.0f));
    bind_col("Snow", "snow_color", Color(0.92f, 0.94f, 0.96f, 1.0f));
    gpu::Texture *ht = this->get_input("Height").bind_as_texture_or_single_value(shader, "height_tx");
    gpu::Texture *ft = this->get_input("Flow").bind_as_texture_or_single_value(shader, "flow_tx");
    Result &out = this->get_result("Color");
    out.allocate_texture(domain);
    out.bind_as_image(shader, "color_img");
    compute_dispatch_threads_at_least(shader, domain.data_size);
    out.unbind_as_image();
    this->get_input("Height").unbind_as_texture_or_single_value(ht);
    this->get_input("Flow").unbind_as_texture_or_single_value(ft);
    GPU_shader_unbind();
  }
};

class TerrainCombineOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    const Result &a = this->get_input("A");
    if (!a.is_single_value() && a.is_allocated()) {
      return a.domain();
    }
    const Result &b = this->get_input("B");
    if (!b.is_single_value() && b.is_allocated()) {
      return b.domain();
    }
    return this->context().get_compositing_domain();
  }

  void execute() override
  {
    gpu::Shader *shader = try_shader(this->context(), "compositor_terrain_combine");
    if (!shader) {
      this->get_result("Height").allocate_invalid();
      return;
    }
    const Domain domain = this->compute_domain();
    GPU_shader_bind(shader);
    GPU_shader_uniform_2iv(shader, "domain_size", domain.data_size);
    GPU_shader_uniform_1i(shader, "combine_type", math::clamp(int(this->node().custom1), 0, 8));
    GPU_shader_uniform_1f(shader, "factor", read_float(this->get_input("Factor"), 1.0f));
    gpu::Texture *at = this->get_input("A").bind_as_texture_or_single_value(shader, "a_tx");
    gpu::Texture *bt = this->get_input("B").bind_as_texture_or_single_value(shader, "b_tx");
    gpu::Texture *mt = this->get_input("Mask").bind_as_texture_or_single_value(shader, "mask_tx");
    Result &out = this->get_result("Height");
    out.allocate_texture(domain);
    out.bind_as_image(shader, "height_img");
    compute_dispatch_threads_at_least(shader, domain.data_size);
    out.unbind_as_image();
    this->get_input("A").unbind_as_texture_or_single_value(at);
    this->get_input("B").unbind_as_texture_or_single_value(bt);
    this->get_input("Mask").unbind_as_texture_or_single_value(mt);
    GPU_shader_unbind();
  }
};

static NodeOperation *get_color_op(Context &context, const bNode &node)
{
  return new TerrainColorOperation(context, node);
}

static NodeOperation *get_combine_op(Context &context, const bNode &node)
{
  return new TerrainCombineOperation(context, node);
}

static void node_register_color()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeTerrainColor"_ustr, IMG_NODE_TERRAIN_COLOR);
  ntype.ui_name = "Terrain Color";
  ntype.ui_description = "SatMap / weathering / super-color / water-color from height and flow";
  ntype.nclass = NODE_CLASS_OP_COLOR;
  ntype.declare = node_declare_color;
  ntype.draw_buttons = node_layout_color;
  ntype.initfunc = node_init_color;
  ntype.get_compositor_operation = get_color_op;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register_color)

static void node_register_combine()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeTerrainCombine"_ustr, IMG_NODE_TERRAIN_COMBINE);
  ntype.ui_name = "Terrain Combine";
  ntype.ui_description = "Blend two heightfields (mix, add, max, min, screen, overlay, hypot)";
  ntype.nclass = NODE_CLASS_OP_COLOR;
  ntype.declare = node_declare_combine;
  ntype.draw_buttons = node_layout_combine;
  ntype.initfunc = node_init_combine;
  ntype.get_compositor_operation = get_combine_op;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register_combine)

}  // namespace blender::nodes::node_image_terrain_color_cc
