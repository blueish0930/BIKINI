/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 *
 * Fractal Primitive (Shader) — 2D Mandelbrot / Julia only.
 * Default Vector = Texture Coordinate Object (same as SDF Shape).
 */

#include "node_shader_util.hh"

#include "BKE_context.hh"
#include "DNA_node_types.h"
#include "DNA_space_types.h"

#include "BLI_math_base.hh"

#include "NOD_multi_function.hh"
#include "NOD_sdf_math.hh"

#include <array>
#include <optional>

#include "BLT_translation.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

namespace blender {

namespace nodes::node_shader_fractal_primitive_cc {

using namespace blender::nodes::sdf_math;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();

  /* Unconnected Vector:
   * - Shader (EEVEE GPU): Texture Coordinate → Object via node_shader_gpu_object_tex_coord
   * - Geometry: Position field (object-local), analogous to Object coords on surfaces
   *
   * Must use POSITION_FIELD (not VALUE/(0,0,0)): function-node constant folding would
   * otherwise collapse the fractal to a single sample and skip node_gpu entirely.
   * Same pattern as Noise / Wave Vector defaults. */
  b.add_input<decl::Vector>("Vector"_ustr)
      .min(-10000.0f)
      .max(10000.0f)
      .hide_value()
      .default_input_type(NODE_DEFAULT_INPUT_POSITION_FIELD)
      .description(
          "Sample position. Unconnected: Object coordinates (origin-centered), "
          "same as Texture Coordinate → Object");
  b.add_input<decl::Float>("Scale"_ustr)
      .default_value(1.0f)
      .min(0.0001f)
      .max(10000.0f)
      .description("Multiplies Vector before the DE");
  b.add_input<decl::Float>("Power"_ustr)
      .default_value(2.0f)
      .min(1.0f)
      .max(16.0f)
      .description("Polynomial degree p for z^p + c (2 = classic Mandelbrot/Julia)");
  b.add_input<decl::Float>("Iterations"_ustr)
      .default_value(64.0f)
      .min(1.0f)
      .max(512.0f)
      .description("Iteration count");
  b.add_input<decl::Float>("Bailout"_ustr)
      .default_value(4.0f)
      .min(1.01f)
      .max(10000.0f)
      .description("Escape radius (affects DE and smooth color bands)");
  auto &julia_c = b.add_input<decl::Vector>("Julia C"_ustr)
                      .default_value({-0.8f, 0.156f, 0.0f})
                      .description("Julia set constant (XY)")
                      .make_available([](bNode &node) { node.custom1 = NODE_FRACTAL_PRIM_JULIA; });

  b.add_output<decl::Color>("Color"_ustr)
      .description("Smooth iteration color — connect to Principled Base Color");
  b.add_output<decl::Float>("Distance"_ustr).description("IQ distance estimator");

  const bNode *node = b.node_or_null();
  const int kind = node ? int(node->custom1) : int(NODE_FRACTAL_PRIM_MANDELBROT);
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

static int node_gpu(GPUMaterial *mat,
                    bNode *node,
                    bNodeExecData * /*execdata*/,
                    GPUNodeStack *in,
                    GPUNodeStack *out)
{
  /* Unlinked Vector → Texture Coordinate → Object (origin-centered).
   * Clear the (0,0,0) constant GPU link and inject Object coords. */
  const bNodeSocket *vector_sock = bke::node_find_socket(*node, SOCK_IN, "Vector"_ustr);
  const bool vector_unlinked = (vector_sock == nullptr) || (vector_sock->link == nullptr) ||
                               ((vector_sock->link->flag & NODE_LINK_MUTED) != 0);
  if (vector_unlinked) {
    in[0].link = nullptr;
    node_shader_gpu_object_tex_coord(mat, node, &in[0].link);
    if (in[0].link == nullptr) {
      return 0;
    }
  }
  float kind = float(node->custom1);
  return GPU_stack_link(mat, node, "node_fractal_primitive", in, out, GPU_constant(&kind));
}

class FractalPrimFn : public mf::MultiFunction {
 public:
  int kind = 0;

  explicit FractalPrimFn(const int kind_in) : kind(kind_in)
  {
    static std::array<mf::Signature, 2> signatures = []() {
      std::array<mf::Signature, 2> sigs;
      for (int k = 0; k < 2; k++) {
        mf::Signature signature;
        mf::SignatureBuilder b{"Fractal Primitive", signature};
        b.single_input<float3>("Vector");
        b.single_input<float>("Scale");
        b.single_input<float>("Power");
        b.single_input<float>("Iterations");
        b.single_input<float>("Bailout");
        if (k == NODE_FRACTAL_PRIM_JULIA) {
          b.single_input<float3>("Julia C");
        }
        b.single_output<ColorGeometry4f>("Color");
        b.single_output<float>("Distance");
        sigs[k] = std::move(signature);
      }
      return sigs;
    }();
    this->set_signature(&signatures[math::clamp(kind_in, 0, 1)]);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int pi = 0;
    const VArray<float3> &vector = params.readonly_single_input<float3>(pi++, "Vector");
    const VArray<float> &scale = params.readonly_single_input<float>(pi++, "Scale");
    const VArray<float> &power = params.readonly_single_input<float>(pi++, "Power");
    const VArray<float> &iterations = params.readonly_single_input<float>(pi++, "Iterations");
    const VArray<float> &bailout = params.readonly_single_input<float>(pi++, "Bailout");
    std::optional<VArray<float3>> julia;
    if (kind == NODE_FRACTAL_PRIM_JULIA) {
      julia = params.readonly_single_input<float3>(pi++, "Julia C");
    }
    MutableSpan<ColorGeometry4f> color = params.uninitialized_single_output<ColorGeometry4f>(
        pi++, "Color");
    MutableSpan<float> distance = params.uninitialized_single_output<float>(pi, "Distance");

    mask.foreach_index([&](const int64_t i) {
      const float s = math::max(scale[i], 1e-6f);
      const float3 p = vector[i] * s;
      const float pwr = power[i];
      const int it_count = int(iterations[i]);
      const float B = bailout[i];
      float2 jc(-0.8f, 0.156f);
      if (julia) {
        const float3 jc3 = (*julia)[i];
        jc = float2(jc3.x, jc3.y);
      }
      const FractalPrimitiveType ft = FractalPrimitiveType(kind);
      const float d = fractal_primitive_distance(
          ft, to_sdf_float3(p), pwr, it_count, B, jc);
      distance[i] = d;
      float it_smooth;
      if (ft == FractalPrimitiveType::Julia2D) {
        it_smooth = smooth_iter_julia(float2(p.x, p.y), jc, it_count, B, pwr);
      }
      else {
        it_smooth = smooth_iter_mandelbrot(float2(p.x, p.y), it_count, B, pwr);
      }
      const float3 c = smooth_iter_to_color(it_smooth, float(it_count));
      color[i] = ColorGeometry4f(c.x, c.y, c.z, 1.0f);
    });
  }
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static FractalPrimFn *functions[2] = {nullptr};
  static bool init = false;
  if (!init) {
    functions[0] = new FractalPrimFn(0);
    functions[1] = new FractalPrimFn(1);
    init = true;
  }
  builder.set_matching_fn(functions[math::clamp(int(builder.node().custom1), 0, 1)]);
}

}  // namespace nodes::node_shader_fractal_primitive_cc

static bool fractal_shader_poll(const bke::bNodeType * /*ntype*/,
                                const bNodeTree *ntree,
                                const char **r_disabled_hint)
{
  if (STR_ELEM(ntree->idname, "ShaderNodeTree", "GeometryNodeTree")) {
    return true;
  }
  *r_disabled_hint = RPT_("Not a shader or geometry node tree");
  return false;
}

static bool fractal_add_ui_poll(const bContext *C)
{
  const SpaceNode *snode = CTX_wm_space_node(C);
  if (snode && snode->edittree && snode->edittree->type == NTREE_GEOMETRY) {
    return true;
  }
  return object_shader_nodes_poll(C);
}

void register_node_type_sh_fractal_primitive()
{
  namespace file_ns = nodes::node_shader_fractal_primitive_cc;

  static bke::bNodeType ntype;

  sh_node_type_base(&ntype, "ShaderNodeFractalPrimitive"_ustr, SH_NODE_FRACTAL_PRIMITIVE);
  ntype.ui_name = "Fractal Primitive";
  ntype.ui_description =
      "2D Mandelbrot / Julia (IQ DE + smooth iteration color). Default Object coords";
  ntype.enum_name_legacy = "FRACTAL_PRIMITIVE";
  ntype.nclass = NODE_CLASS_TEXTURE;
  ntype.declare = file_ns::node_declare;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.initfunc = file_ns::node_init;
  ntype.gpu_fn = file_ns::node_gpu;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.poll = fractal_shader_poll;
  ntype.add_ui_poll = fractal_add_ui_poll;

  bke::node_register_type(ntype);
}

}  // namespace blender
