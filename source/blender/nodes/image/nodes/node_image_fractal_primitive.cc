/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * GPU Texture Editor Fractal Primitive — 2D Mandelbrot / Julia.
 * GPU compute path is the primary path; CPU is fallback only.
 */

#include "BLI_math_base.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include "BKE_node.hh"

#include "DNA_node_types.h"

#include "GPU_shader.hh"
#include "GPU_texture.hh"

#include "BLT_translation.hh"

#include "COM_node_operation.hh"
#include "COM_utilities.hh"

#include "NOD_sdf_math.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_image_util.hh"

namespace blender::nodes::node_image_fractal_primitive_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  /* Vector: Realization None so unlinked/(0,0,0) stays a single value → domain UV path.
   * If realized onto the domain as a constant zero field, every pixel samples c=0 (solid blue). */
  b.add_input<decl::Vector>("Vector"_ustr)
      .default_value({0.0f, 0.0f, 0.0f})
      .min(-10000.0f)
      .max(10000.0f)
      .hide_value()
      .structure_type(StructureType::Dynamic)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Sample position; single value uses domain UV × Scale");
  b.add_input<decl::Float>("Power"_ustr)
      .default_value(2.0f)
      .min(1.0f)
      .max(16.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Polynomial degree p for z^p + c (2 = classic)");
  b.add_input<decl::Float>("Iterations"_ustr)
      .default_value(64.0f)
      .min(1.0f)
      .max(512.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Iteration count");
  b.add_input<decl::Float>("Bailout"_ustr)
      .default_value(4.0f)
      .min(1.01f)
      .max(10000.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Escape radius (DE + smooth color)");
  auto &julia_c = b.add_input<decl::Vector>("Julia C"_ustr)
                      .default_value({-0.8f, 0.156f, 0.0f})
                      .structure_type(StructureType::Single)
                      .compositor_realization_mode(CompositorInputRealizationMode::None)
                      .description("Julia constant (XY)")
                      .make_available([](bNode &node) { node.custom1 = NODE_FRACTAL_PRIM_JULIA; });
  b.add_input<decl::Float>("Scale"_ustr)
      .default_value(1.5f)
      .min(0.01f)
      .max(100.0f)
      .structure_type(StructureType::Single)
      .compositor_realization_mode(CompositorInputRealizationMode::None)
      .description("Domain UV half-extent when Vector is single");

  b.add_output<decl::Color>("Color"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("Smooth iteration color — connect to Viewer");
  b.add_output<decl::Float>("Distance"_ustr)
      .structure_type(StructureType::Dynamic)
      .description("IQ distance estimator");

  const int kind = (b.node_or_null() != nullptr) ? int(b.node_or_null()->custom1) :
                                                   int(NODE_FRACTAL_PRIM_MANDELBROT);
  julia_c.available(kind == NODE_FRACTAL_PRIM_JULIA);
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  ui::Layout &col = layout.column(true);
  col.prop(ptr, "fractal_type", UI_ITEM_NONE, IFACE_("Type"), ICON_NONE);
}

static void node_init(bNodeTree * /*ntree*/, bNode *node)
{
  node->custom1 = NODE_FRACTAL_PRIM_MANDELBROT;
}

using namespace blender::compositor;

class FractalPrimitiveOperation : public NodeOperation {
 public:
  using NodeOperation::NodeOperation;

  Domain compute_domain() override
  {
    /* Always paint on the editor / compositing domain. A realized constant Vector field must
     * not shrink or redefine the domain. */
    return this->context().get_compositing_domain();
  }

  /** True when Vector means "generate UV × Scale" rather than a position field. */
  bool vector_uses_generated_uv() const
  {
    const Result &vector = this->get_input("Vector");
    if (vector.is_single_value() || !vector.is_allocated()) {
      return true;
    }
    /* Constant domain-sized field (e.g. forced Group Input expand of (0,0,0)): treat as UV. */
    if (vector.domain().data_size == this->context().get_compositing_domain().data_size) {
      const float3 a = vector.load_pixel<float3, true>(int2(0));
      const int2 size = vector.domain().data_size;
      const float3 b = vector.load_pixel<float3, true>(
          int2(math::max(size.x - 1, 0), math::max(size.y - 1, 0)));
      const float3 c = vector.load_pixel<float3, true>(int2(size.x / 2, size.y / 2));
      if (math::distance(a, b) < 1e-6f && math::distance(a, c) < 1e-6f) {
        return true;
      }
    }
    return false;
  }

  blender::float3 read_julia_c() const
  {
    const bNodeSocket *sock = bke::node_find_socket(this->node(), SOCK_IN, "Julia C"_ustr);
    if (sock != nullptr && sock->is_available()) {
      return this->get_input("Julia C").get_single_value_default<blender::float3>();
    }
    if (sock != nullptr) {
      const float *v = sock->default_value_typed<bNodeSocketValueVector>()->value;
      return blender::float3(v[0], v[1], v[2]);
    }
    return blender::float3(-0.8f, 0.156f, 0.0f);
  }

  void execute() override
  {
    if (this->context().use_gpu()) {
      /* get_shader can throw/crash if the create-info source is not registered in the GPU
       * dependency dictionary (missing from compositor GLSL_SRC). Guard with create-info
       * presence first; still fall back to CPU if compile returns null. */
      if (GPU_shader_create_info_get("compositor_fractal_primitive") != nullptr) {
        gpu::Shader *shader = this->context().get_shader("compositor_fractal_primitive");
        if (shader) {
          this->execute_gpu(shader);
          return;
        }
      }
    }
    this->execute_cpu();
  }

  static float read_float_param(const Result &r, const float fallback)
  {
    if (r.is_single_value()) {
      return r.get_single_value_default<float>();
    }
    if (r.is_allocated()) {
      return r.load_pixel<float, true>(int2(0));
    }
    return fallback;
  }

  void execute_gpu(gpu::Shader *shader)
  {
    GPU_shader_bind(shader);

    const Domain domain = this->compute_domain();
    const int2 size_px = domain.data_size;

    const float use_power = math::max(read_float_param(this->get_input("Power"), 2.0f), 1.0f);
    const int use_iters = math::max(int(read_float_param(this->get_input("Iterations"), 64.0f)),
                                    1);
    const float use_bailout = math::max(read_float_param(this->get_input("Bailout"), 4.0f), 1.01f);
    const blender::float3 julia_c = this->read_julia_c();
    const float use_scale = math::max(read_float_param(this->get_input("Scale"), 1.5f), 0.01f);
    const int kind = blender::math::clamp(int(this->node().custom1), 0, 1);

    Result &vector_in = this->get_input("Vector");
    const bool use_uv = this->vector_uses_generated_uv();

    GPU_shader_uniform_2iv(shader, "domain_size", size_px);
    GPU_shader_uniform_1f(shader, "power", use_power);
    GPU_shader_uniform_1i(shader, "iterations", use_iters);
    GPU_shader_uniform_1f(shader, "bailout", use_bailout);
    const float2 jc(julia_c.x, julia_c.y);
    GPU_shader_uniform_2fv(shader, "julia_c", jc);
    GPU_shader_uniform_1f(shader, "scale", use_scale);
    GPU_shader_uniform_1i(shader, "fractal_kind", kind);
    GPU_shader_uniform_1i(shader, "vector_is_single", use_uv ? 1 : 0);

    /* Shader always samples vector_tx; bind a 1×1 dummy when using generated UV. */
    Result dummy_vector = this->context().create_result(ResultType::Float3);
    if (use_uv) {
      dummy_vector.allocate_texture(Domain(int2(1)));
      const float3 zero(0.0f);
      GPU_texture_clear(dummy_vector, GPU_DATA_FLOAT, &zero);
      dummy_vector.bind_as_texture(shader, "vector_tx");
    }
    else {
      vector_in.bind_as_texture(shader, "vector_tx");
    }

    Result &color_out = this->get_result("Color");
    Result &dist_out = this->get_result("Distance");
    color_out.allocate_texture(domain);
    dist_out.allocate_texture(domain);
    color_out.bind_as_image(shader, "color_img");
    dist_out.bind_as_image(shader, "distance_img");

    compute_dispatch_threads_at_least(shader, size_px);

    if (use_uv) {
      dummy_vector.unbind_as_texture();
      dummy_vector.release();
    }
    else {
      vector_in.unbind_as_texture();
    }
    color_out.unbind_as_image();
    dist_out.unbind_as_image();
    GPU_shader_unbind();
  }

  void execute_cpu()
  {
    using namespace blender::nodes::sdf_math;

    Result &vector_in = this->get_input("Vector");
    Result &color_out = this->get_result("Color");
    Result &dist_out = this->get_result("Distance");

    const float power = math::max(read_float_param(this->get_input("Power"), 2.0f), 1.0f);
    const int iterations = math::max(int(read_float_param(this->get_input("Iterations"), 64.0f)),
                                     1);
    const float bailout = math::max(read_float_param(this->get_input("Bailout"), 4.0f), 1.01f);
    const blender::float3 julia_c = this->read_julia_c();
    const float scale = math::max(read_float_param(this->get_input("Scale"), 1.5f), 0.01f);
    const int kind = blender::math::clamp(int(this->node().custom1), 0, 1);

    const Domain domain = this->compute_domain();
    dist_out.allocate_texture(domain, false, ResultStorageType::CPUImage);
    color_out.allocate_texture(domain, false, ResultStorageType::CPUImage);

    const int2 size_px = domain.data_size;
    const bool use_uv = this->vector_uses_generated_uv();

    parallel_for(size_px, [&](const int2 texel) {
      blender::float3 p;
      if (use_uv) {
        p.x = (((float(texel.x) + 0.5f) / float(size_px.x)) * 2.0f - 1.0f) * scale;
        p.y = (((float(texel.y) + 0.5f) / float(size_px.y)) * 2.0f - 1.0f) * scale;
        p.z = 0.0f;
      }
      else {
        p = vector_in.load_pixel<blender::float3, true>(texel);
      }
      const float2 jc(julia_c.x, julia_c.y);
      const FractalPrimitiveType ft = FractalPrimitiveType(kind);
      const float d = fractal_primitive_distance(
          ft, to_sdf_float3(p), power, iterations, bailout, jc);
      dist_out.store_pixel(texel, d);
      float it_smooth;
      if (ft == FractalPrimitiveType::Julia2D) {
        it_smooth = smooth_iter_julia(float2(p.x, p.y), jc, iterations, bailout, power);
      }
      else {
        it_smooth = smooth_iter_mandelbrot(float2(p.x, p.y), iterations, bailout, power);
      }
      const float3 c = smooth_iter_to_color(it_smooth, float(iterations));
      color_out.store_pixel(texel, Color(c.x, c.y, c.z, 1.0f));
    });

    /* Compositor Viewer / GPU consumers crash if given CPU textures — upload when needed. */
    auto ensure_gpu = [this](Result &result) {
      if (!this->context().use_gpu() || !result.should_compute() || result.is_single_value()) {
        return;
      }
      if (result.is_stored_on_gpu()) {
        return;
      }
      Result gpu = result.upload_to_gpu(true);
      result.release();
      result.share_data(gpu);
      gpu.release();
    };
    ensure_gpu(color_out);
    ensure_gpu(dist_out);
  }
};

static NodeOperation *get_compositor_operation(Context &context, const bNode &node)
{
  return new FractalPrimitiveOperation(context, node);
}

static void node_register()
{
  static bke::bNodeType ntype;

  img_node_type_base(&ntype, "ImageNodeFractalPrimitive"_ustr, IMG_NODE_FRACTAL_PRIMITIVE);
  ntype.ui_name = "Fractal Primitive";
  ntype.ui_description =
      "2D Mandelbrot / Julia on GPU (IQ DE + smooth iteration color)";
  ntype.nclass = NODE_CLASS_TEXTURE;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.get_compositor_operation = get_compositor_operation;

  bke::node_register_type(ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_image_fractal_primitive_cc
