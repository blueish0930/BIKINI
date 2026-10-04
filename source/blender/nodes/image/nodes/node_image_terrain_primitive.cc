/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Texture Editor terrain primitives (Gaea-style mountain / ridge / canyon / …).
 */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include <cmath>

#include "BKE_node.hh"

#include "DNA_node_types.h"

#include "GPU_shader.hh"
#include "GPU_texture.hh"

#include "BLT_translation.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_image_terrain_common.hh"
#include "node_image_util.hh"

namespace blender::nodes::node_image_terrain_primitive_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Vector"_ustr)
      .default_value({0.0f, 0.0f, 0.0f})
      .min(-10000.0f)
      .max(10000.0f)
      .dimensions(3)
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Sample position; single value uses domain UV in [-1,1]");
  b.add_input<decl::Float>("Scale"_ustr)
      .default_value(2.4f)
      .min(0.001f)
      .max(100.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Macro mountain frequency");
  b.add_input<decl::Float>("Amplitude"_ustr)
      .default_value(1.0f)
      .min(0.0f)
      .max(100.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Output height amplitude");
  b.add_input<decl::Float>("Detail"_ustr)
      .default_value(7.0f)
      .min(1.0f)
      .max(12.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Ridged-multifractal octaves");
  b.add_input<decl::Float>("Roughness"_ustr)
      .default_value(0.48f)
      .min(0.0f)
      .max(1.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Amplitude decay per octave");
  b.add_input<decl::Float>("Distortion"_ustr)
      .default_value(0.35f)
      .min(0.0f)
      .max(8.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Domain warp amount");
  b.add_input<decl::Float>("Lacunarity"_ustr)
      .default_value(2.05f)
      .min(1.01f)
      .max(4.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Frequency multiplier per octave");
  b.add_input<decl::Float>("Seed"_ustr)
      .default_value(0.0f)
      .min(0.0f)
      .max(10000.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Noise seed");
  b.add_input<decl::Vector>("Offset"_ustr)
      .default_value({0.0f, 0.0f, 0.0f})
      .dimensions(3)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("World-space XY offset (moves radial landforms and noise)");

  b.add_output<decl::Color>("Height"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Heightfield (grayscale). Connect to Erosion or Viewer");
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  ui::Layout &col = layout.column(true);
  col.prop(ptr, "primitive_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_TERRAIN_PRIM_MOUNTAIN;
}

using namespace blender::compositor;
using namespace blender::nodes::terrain;

class TerrainPrimitiveOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    return this->context().get_compositing_domain();
  }

  bool vector_uses_generated_uv() const
  {
    const Result &vector = this->get_input("Vector");
    if (vector.is_single_value() || !vector.is_allocated()) {
      return true;
    }
    return false;
  }

  void execute() override
  {
    gpu::Shader *shader = try_shader(this->context(), "compositor_terrain_primitive");
    if (shader) {
      this->execute_gpu(shader);
      return;
    }
    this->execute_cpu();
  }

  void execute_gpu(gpu::Shader *shader)
  {
    GPU_shader_bind(shader);
    const Domain domain = this->compute_domain();
    const int2 size_px = domain.data_size;
    const int kind = math::clamp(int(this->node().custom1), 0, 12);
    GPU_shader_uniform_2iv(shader, "domain_size", size_px);
    GPU_shader_uniform_1i(shader, "primitive_type", kind);
    GPU_shader_uniform_1i(shader, "vector_is_single", this->vector_uses_generated_uv() ? 1 : 0);
    GPU_shader_uniform_1f(shader, "scale", math::max(read_float(this->get_input("Scale"), 2.4f), 0.001f));
    GPU_shader_uniform_1f(
        shader, "height", math::max(read_float(this->get_input("Amplitude"), 1.0f), 0.0f));
    GPU_shader_uniform_1f(shader, "detail", read_float(this->get_input("Detail"), 7.0f));
    GPU_shader_uniform_1f(shader, "roughness", read_float(this->get_input("Roughness"), 0.48f));
    GPU_shader_uniform_1f(shader, "distortion", read_float(this->get_input("Distortion"), 0.35f));
    GPU_shader_uniform_1f(
        shader, "lacunarity", math::max(read_float(this->get_input("Lacunarity"), 2.05f), 1.01f));
    GPU_shader_uniform_1f(shader, "seed", read_float(this->get_input("Seed"), 0.0f));
    const float3 off = read_float3(this->get_input("Offset"), float3(0.0f));
    const float2 offset(off.x, off.y);
    GPU_shader_uniform_2fv(shader, "offset", offset);

    Result dummy = this->context().create_result(ResultType::Float3);
    Result &vector_in = this->get_input("Vector");
    if (this->vector_uses_generated_uv()) {
      dummy.allocate_texture(Domain(int2(1)));
      if (dummy.gpu_texture() == nullptr) {
        dummy.release();
        GPU_shader_unbind();
        this->execute_cpu();
        return;
      }
      const float3 zero(0.0f);
      GPU_texture_clear(dummy, GPU_DATA_FLOAT, &zero);
      dummy.bind_as_texture(shader, "vector_tx");
    }
    else {
      vector_in.bind_as_texture(shader, "vector_tx");
    }

    Result &height_out = this->get_result("Height");
    height_out.allocate_texture(domain);
    height_out.bind_as_image(shader, "height_img");
    compute_dispatch_threads_at_least(shader, size_px);
    height_out.unbind_as_image();
    if (this->vector_uses_generated_uv()) {
      dummy.unbind_as_texture();
      dummy.release();
    }
    else {
      vector_in.unbind_as_texture();
    }
    GPU_shader_unbind();
  }

  void execute_cpu()
  {
    const Domain domain = this->compute_domain();
    Result &height_out = this->get_result("Height");
    Result cpu = this->context().create_result(ResultType::Color);
    cpu.allocate_texture(domain, false, ResultStorageType::CPUImage);
    const int kind = math::clamp(int(this->node().custom1), 0, 12);
    const float scale = math::max(read_float(this->get_input("Scale"), 2.4f), 0.001f);
    const float amp = math::max(read_float(this->get_input("Amplitude"), 1.0f), 0.0f);
    const int oct = math::clamp(int(read_float(this->get_input("Detail"), 7.0f)), 1, 12);
    const float gain = read_float(this->get_input("Roughness"), 0.48f);
    const float distort = read_float(this->get_input("Distortion"), 0.35f);
    const float lac = math::max(read_float(this->get_input("Lacunarity"), 2.05f), 1.01f);
    const float seed = read_float(this->get_input("Seed"), 0.0f);
    const float3 off = read_float3(this->get_input("Offset"), float3(0.0f));
    const bool use_uv = this->vector_uses_generated_uv();
    Result &vector_in = this->get_input("Vector");
    const int2 size = domain.data_size;
    parallel_for(size, [&](const int2 texel) {
      float2 p;
      if (use_uv) {
        p.x = ((float(texel.x) + 0.5f) / float(size.x)) * 2.0f - 1.0f;
        p.y = ((float(texel.y) + 0.5f) / float(size.y)) * 2.0f - 1.0f;
      }
      else {
        const float3 v = vector_in.load_pixel<float3, true>(texel);
        p = float2(v.x, v.y);
      }
      const float h = primitive_eval(
          p, kind, scale, amp, oct, gain, distort, lac, seed, float2(off.x, off.y));
      cpu.store_pixel(texel, gray(h));
    });
    share_cpu_to_output(this->context(), height_out, cpu);
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new TerrainPrimitiveOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;
  img_node_type_base(&ntype, "ImageNodeTerrainPrimitive"_ustr, IMG_NODE_TERRAIN_PRIMITIVE);
  ntype.ui_name = "Terrain Primitive";
  ntype.ui_description =
      "Gaea-style terrain generators: mountain, mountainous, ridge, canyon, crater, dunes";
  ntype.nclass = NODE_CLASS_TEXTURE;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.get_compositor_operation = get_compositor_operation;
  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_terrain_primitive_cc
