/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 */

#include "node_shader_util.hh"
#include "node_util.hh"

#include "BKE_texture.h"

#include "BLI_math_vector.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_noise.hh"

#include "NOD_multi_function.hh"

#include "RNA_access.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

namespace blender {

namespace nodes::node_shader_tex_noise_cc {

NODE_STORAGE_FUNCS(NodeTexNoise)

static float period_or_zero(const bool tiling, const float size)
{
  if (!tiling) {
    return 0.0f;
  }
  return float(math::max(int(math::round(size)), 2));
}

static void sh_node_tex_noise_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.use_custom_socket_order();
  b.allow_any_socket_order();

  b.add_output<decl::Float>("Factor"_ustr, "Fac"_ustr).no_muted_links();
  b.add_output<decl::Color>("Color"_ustr).no_muted_links();

  b.add_default_layout();

  const int dimensions = b.node_or_null() ? node_storage(*b.node_or_null()).dimensions : 3;
  const eNodeTree_Type tree_type = b.tree_or_null() ? eNodeTree_Type(b.tree_or_null()->type) :
                                                      NTREE_UNDEFINED;
  /* Image Process: UV [0,1]²; Compositor: uniform coords; Geometry/Shader: position field. */
  const NodeDefaultInputType default_input_type =
      tree_type == NTREE_IMAGE ? NODE_DEFAULT_INPUT_NORMALIZED_IMAGE_COORDINATES :
      tree_type == NTREE_COMPOSIT ? NODE_DEFAULT_INPUT_UNIFORM_IMAGE_COORDINATES :
                                    NODE_DEFAULT_INPUT_POSITION_FIELD;
  b.add_input<decl::Vector>("Vector"_ustr)
      .default_input_type(default_input_type)
      .available(dimensions != 1);

  b.add_input<decl::Float>("W"_ustr)
      .min(-1000.0f)
      .max(1000.0f)
      .available(ELEM(dimensions, 1, 4))
      .make_available([](bNode &node) {
        /* Default to 1 instead of 4, because it is much faster. */
        node_storage(node).dimensions = 1;
      });

  b.add_input<decl::Float>("Scale"_ustr)
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(5.0f)
      .description("Scale of the base noise octave");
  b.add_input<decl::Float>("Detail"_ustr)
      .min(0.0f)
      .max(15.0f)
      .default_value(2.0f)
      .description(
          "The number of noise octaves. Higher values give more detailed noise but increase "
          "render "
          "time");
  b.add_input<decl::Float>("Roughness"_ustr)
      .min(0.0f)
      .max(1.0f)
      .default_value(0.5f)
      .subtype(PROP_FACTOR)
      .description(
          "Blend factor between an octave and its previous one. A value of zero corresponds to "
          "zero detail");
  b.add_input<decl::Float>("Lacunarity"_ustr)
      .min(0.0f)
      .max(1000.0f)
      .default_value(2.0f)
      .description(
          "The difference between the scale of each two consecutive octaves. Larger values "
          "corresponds to larger scale for higher octaves");

  const eNodeNoiseTexture_Type noise_type = eNodeNoiseTexture_Type(
      b.node_or_null() ? eNodeNoiseTexture_Type(node_storage(*b.node_or_null()).type) :
                         SHD_NOISE_FBM);
  b.add_input<decl::Float>("Offset"_ustr)
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .available(!ELEM(noise_type, SHD_NOISE_MULTIFRACTAL, SHD_NOISE_FBM))
      .make_available([](bNode &node) { node_storage(node).type = SHD_NOISE_RIDGED_MULTIFRACTAL; })
      .description(
          "An added offset to each octave, determines the level where the highest octave will "
          "appear");
  b.add_input<decl::Float>("Gain"_ustr)
      .min(0.0f)
      .max(1000.0f)
      .default_value(1.0f)
      .available(ELEM(noise_type, SHD_NOISE_HYBRID_MULTIFRACTAL, SHD_NOISE_RIDGED_MULTIFRACTAL))
      .make_available([](bNode &node) { node_storage(node).type = SHD_NOISE_RIDGED_MULTIFRACTAL; })
      .description("An extra multiplier to tune the magnitude of octaves");
  b.add_input<decl::Float>("Distortion"_ustr)
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .description("Amount of distortion");

  auto &tiling_panel = b.add_panel("Tiling"_ustr);
  auto &period_1d = tiling_panel.add_input<decl::Float>("Size"_ustr, "Period"_ustr)
                        .min(2.0f)
                        .max(10000.0f)
                        .default_value(5.0f)
                        .description(
                            "Integer wrap period of the already-scaled W coordinate (after Scale). "
                            "Size 5 wraps a 5-cell lattice. Values are rounded; minimum 2")
                        .make_available([](bNode &node) { node_storage(node).tiling = 1; });
  auto &period_2d = tiling_panel.add_input<decl::Vector>("Size"_ustr, "Period_2D"_ustr)
                        .dimensions(2)
                        .min(2.0f)
                        .max(10000.0f)
                        .default_value(float2{5.0f, 5.0f})
                        .description(
                            "Integer wrap period of the already-scaled XY coordinate (after Scale). "
                            "Size 5 wraps a 5×5 cell lattice. Values are rounded; minimum 2")
                        .make_available([](bNode &node) { node_storage(node).tiling = 1; });
  auto &period_3d = tiling_panel.add_input<decl::Vector>("Size"_ustr, "Period_3D"_ustr)
                        .dimensions(3)
                        .min(2.0f)
                        .max(10000.0f)
                        .default_value(float3{5.0f, 5.0f, 5.0f})
                        .description(
                            "Integer wrap period of the already-scaled XYZ coordinate (after Scale). "
                            "Size 5 wraps a 5×5×5 cell lattice. Values are rounded; minimum 2")
                        .make_available([](bNode &node) { node_storage(node).tiling = 1; });
  auto &period_w = tiling_panel.add_input<decl::Float>("W"_ustr, "Period_W"_ustr)
                       .min(2.0f)
                       .max(10000.0f)
                       .default_value(5.0f)
                       .description(
                           "Integer wrap period of the already-scaled W coordinate (after Scale). "
                           "Values are rounded; minimum 2")
                       .make_available([](bNode &node) { node_storage(node).tiling = 1; });
  period_1d.available(dimensions == 1);
  period_2d.available(dimensions == 2);
  period_3d.available(dimensions == 3 || dimensions == 4);
  period_w.available(dimensions == 4);
}

static void node_shader_buts_tex_noise(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "noise_dimensions", ui::ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
  layout.prop(ptr, "noise_type", ui::ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
  if (ELEM(RNA_enum_get(ptr, "noise_type"), SHD_NOISE_FBM)) {
    layout.prop(ptr, "normalize", ui::ITEM_R_SPLIT_EMPTY_NAME, std::nullopt, ICON_NONE);
  }
  layout.prop(ptr, "tiling", ui::ITEM_R_SPLIT_EMPTY_NAME, std::nullopt, ICON_NONE);
}

static void node_shader_init_tex_noise(bNodeTree *node_tree, bNode *node)
{
  NodeTexNoise *tex = MEM_new<NodeTexNoise>(__func__);
  BKE_texture_mapping_default(&tex->base.tex_mapping, TEXMAP_TYPE_POINT);
  BKE_texture_colormapping_default(&tex->base.color_mapping);
  tex->dimensions = ELEM(node_tree->type, NTREE_COMPOSIT, NTREE_IMAGE) ? 2 : 3;
  tex->type = SHD_NOISE_FBM;
  tex->normalize = true;

  node->storage = tex;
}

static const char *gpu_shader_get_name(const int dimensions, const int type)
{
  BLI_assert(dimensions > 0 && dimensions < 5);
  BLI_assert(type >= 0 && type < 5);

  switch (type) {
    case SHD_NOISE_MULTIFRACTAL:
      return std::array{"node_noise_tex_multi_fractal_1d",
                        "node_noise_tex_multi_fractal_2d",
                        "node_noise_tex_multi_fractal_3d",
                        "node_noise_tex_multi_fractal_4d"}[dimensions - 1];
    case SHD_NOISE_FBM:
      return std::array{"node_noise_tex_fbm_1d",
                        "node_noise_tex_fbm_2d",
                        "node_noise_tex_fbm_3d",
                        "node_noise_tex_fbm_4d"}[dimensions - 1];
    case SHD_NOISE_HYBRID_MULTIFRACTAL:
      return std::array{"node_noise_tex_hybrid_multi_fractal_1d",
                        "node_noise_tex_hybrid_multi_fractal_2d",
                        "node_noise_tex_hybrid_multi_fractal_3d",
                        "node_noise_tex_hybrid_multi_fractal_4d"}[dimensions - 1];
    case SHD_NOISE_RIDGED_MULTIFRACTAL:
      return std::array{"node_noise_tex_ridged_multi_fractal_1d",
                        "node_noise_tex_ridged_multi_fractal_2d",
                        "node_noise_tex_ridged_multi_fractal_3d",
                        "node_noise_tex_ridged_multi_fractal_4d"}[dimensions - 1];
    case SHD_NOISE_HETERO_TERRAIN:
      return std::array{"node_noise_tex_hetero_terrain_1d",
                        "node_noise_tex_hetero_terrain_2d",
                        "node_noise_tex_hetero_terrain_3d",
                        "node_noise_tex_hetero_terrain_4d"}[dimensions - 1];
  }
  return nullptr;
}

static int node_shader_gpu_tex_noise(GPUMaterial *mat,
                                     bNode *node,
                                     bNodeExecData * /*execdata*/,
                                     GPUNodeStack *in,
                                     GPUNodeStack *out)
{
  node_shader_gpu_default_tex_coord(mat, node, &in[0].link);
  node_shader_gpu_tex_mapping(mat, node, in, out);

  const NodeTexNoise &storage = node_storage(*node);
  float normalize = storage.normalize;
  float compute_color = out[1].hasoutput;
  float tiling = storage.tiling ? 1.0f : 0.0f;

  const char *name = gpu_shader_get_name(storage.dimensions, storage.type);
  return GPU_stack_link(mat,
                        node,
                        name,
                        in,
                        out,
                        GPU_constant(&tiling),
                        GPU_constant(&normalize),
                        GPU_constant(&compute_color));
}

class NoiseFunction : public mf::MultiFunction {
 private:
  int dimensions_;
  int type_;
  bool normalize_;
  bool tiling_;

 public:
  NoiseFunction(int dimensions, int type, bool normalize, bool tiling)
      : dimensions_(dimensions), type_(type), normalize_(normalize), tiling_(tiling)
  {
    BLI_assert(dimensions >= 1 && dimensions <= 4);
    BLI_assert(type >= 0 && type <= 4);
    static std::array<mf::Signature, 20> signatures{
        create_signature(1, SHD_NOISE_MULTIFRACTAL),
        create_signature(2, SHD_NOISE_MULTIFRACTAL),
        create_signature(3, SHD_NOISE_MULTIFRACTAL),
        create_signature(4, SHD_NOISE_MULTIFRACTAL),

        create_signature(1, SHD_NOISE_FBM),
        create_signature(2, SHD_NOISE_FBM),
        create_signature(3, SHD_NOISE_FBM),
        create_signature(4, SHD_NOISE_FBM),

        create_signature(1, SHD_NOISE_HYBRID_MULTIFRACTAL),
        create_signature(2, SHD_NOISE_HYBRID_MULTIFRACTAL),
        create_signature(3, SHD_NOISE_HYBRID_MULTIFRACTAL),
        create_signature(4, SHD_NOISE_HYBRID_MULTIFRACTAL),

        create_signature(1, SHD_NOISE_RIDGED_MULTIFRACTAL),
        create_signature(2, SHD_NOISE_RIDGED_MULTIFRACTAL),
        create_signature(3, SHD_NOISE_RIDGED_MULTIFRACTAL),
        create_signature(4, SHD_NOISE_RIDGED_MULTIFRACTAL),

        create_signature(1, SHD_NOISE_HETERO_TERRAIN),
        create_signature(2, SHD_NOISE_HETERO_TERRAIN),
        create_signature(3, SHD_NOISE_HETERO_TERRAIN),
        create_signature(4, SHD_NOISE_HETERO_TERRAIN),
    };
    this->set_signature(&signatures[dimensions + type * 4 - 1]);
  }

  static mf::Signature create_signature(int dimensions, int type)
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"Noise", signature};

    if (ELEM(dimensions, 2, 3, 4)) {
      builder.single_input<float3>("Vector");
    }
    if (ELEM(dimensions, 1, 4)) {
      builder.single_input<float>("W");
    }

    builder.single_input<float>("Scale");
    builder.single_input<float>("Detail");
    builder.single_input<float>("Roughness");
    builder.single_input<float>("Lacunarity");
    if (ELEM(type,
             SHD_NOISE_RIDGED_MULTIFRACTAL,
             SHD_NOISE_HYBRID_MULTIFRACTAL,
             SHD_NOISE_HETERO_TERRAIN))
    {
      builder.single_input<float>("Offset");
    }
    if (ELEM(type, SHD_NOISE_RIDGED_MULTIFRACTAL, SHD_NOISE_HYBRID_MULTIFRACTAL)) {
      builder.single_input<float>("Gain");
    }
    builder.single_input<float>("Distortion");
    if (dimensions == 1) {
      builder.single_input<float>("Period");
    }
    else if (dimensions == 2) {
      builder.single_input<float3>("Period_2D");
    }
    else if (dimensions == 3) {
      builder.single_input<float3>("Period_3D");
    }
    else {
      builder.single_input<float3>("Period_3D");
      builder.single_input<float>("Period_W");
    }

    builder.single_output<float>("Fac", mf::ParamFlag::SupportsUnusedOutput);
    builder.single_output<ColorGeometry4f>("Color", mf::ParamFlag::SupportsUnusedOutput);

    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = ELEM(dimensions_, 2, 3, 4) + ELEM(dimensions_, 1, 4);
    const VArray<float> &scale = params.readonly_single_input<float>(param++, "Scale");
    const VArray<float> &detail = params.readonly_single_input<float>(param++, "Detail");
    const VArray<float> &roughness = params.readonly_single_input<float>(param++, "Roughness");
    const VArray<float> &lacunarity = params.readonly_single_input<float>(param++, "Lacunarity");
    /* Initialize to any other variable when unused to avoid unnecessary conditionals. */
    const VArray<float> &offset = ELEM(type_,
                                       SHD_NOISE_RIDGED_MULTIFRACTAL,
                                       SHD_NOISE_HYBRID_MULTIFRACTAL,
                                       SHD_NOISE_HETERO_TERRAIN) ?
                                      params.readonly_single_input<float>(param++, "Offset") :
                                      scale;
    /* Initialize to any other variable when unused to avoid unnecessary conditionals. */
    const VArray<float> &gain = ELEM(type_,
                                     SHD_NOISE_RIDGED_MULTIFRACTAL,
                                     SHD_NOISE_HYBRID_MULTIFRACTAL) ?
                                    params.readonly_single_input<float>(param++, "Gain") :
                                    scale;
    const VArray<float> &distortion = params.readonly_single_input<float>(param++, "Distortion");
    const VArray<float> period_1d = (dimensions_ == 1) ?
                                        params.readonly_single_input<float>(param++, "Period") :
                                        VArray<float>{};
    const VArray<float3> period_2d = (dimensions_ == 2) ?
                                         params.readonly_single_input<float3>(param++, "Period_2D") :
                                         VArray<float3>{};
    const VArray<float3> period_3d = ELEM(dimensions_, 3, 4) ?
                                         params.readonly_single_input<float3>(param++, "Period_3D") :
                                         VArray<float3>{};
    const VArray<float> period_w = (dimensions_ == 4) ?
                                       params.readonly_single_input<float>(param++, "Period_W") :
                                       VArray<float>{};

    MutableSpan<float> r_factor = params.uninitialized_single_output_if_required<float>(param++,
                                                                                        "Fac");
    MutableSpan<ColorGeometry4f> r_color =
        params.uninitialized_single_output_if_required<ColorGeometry4f>(param++, "Color");

    const bool compute_factor = !r_factor.is_empty();
    const bool compute_color = !r_color.is_empty();

    switch (dimensions_) {
      case 1: {
        const VArray<float> &w = params.readonly_single_input<float>(0, "W");
        if (compute_color) {
          mask.foreach_index([&](const int64_t i) {
            const float s = scale[i];
            const float position = w[i] * s;
            const float period = period_or_zero(tiling_, period_1d[i]);
            const float3 c = noise::perlin_float3_fractal_distorted(
                position,
                math::clamp(detail[i], 0.0f, 15.0f),
                math::max(roughness[i], 0.0f),
                lacunarity[i],
                offset[i],
                gain[i],
                distortion[i],
                type_,
                normalize_,
                period);
            r_color[i] = ColorGeometry4f(c[0], c[1], c[2], 1.0f);
            if (compute_factor) {
              r_factor[i] = c[0];
            }
          });
        }
        else if (compute_factor) {
          mask.foreach_index([&](const int64_t i) {
            const float s = scale[i];
            const float position = w[i] * s;
            const float period = period_or_zero(tiling_, period_1d[i]);
            r_factor[i] = noise::perlin_fractal_distorted(position,
                                                          math::clamp(detail[i], 0.0f, 15.0f),
                                                          math::max(roughness[i], 0.0f),
                                                          lacunarity[i],
                                                          offset[i],
                                                          gain[i],
                                                          distortion[i],
                                                          type_,
                                                          normalize_,
                                                          period);
          });
        }
        break;
      }
      case 2: {
        const VArray<float3> &vector = params.readonly_single_input<float3>(0, "Vector");
        if (compute_color) {
          mask.foreach_index([&](const int64_t i) {
            const float s = scale[i];
            const float2 position = float2(vector[i] * s);
            const float2 period{period_or_zero(tiling_, period_2d[i].x),
                                period_or_zero(tiling_, period_2d[i].y)};
            const float3 c = noise::perlin_float3_fractal_distorted(
                position,
                math::clamp(detail[i], 0.0f, 15.0f),
                math::max(roughness[i], 0.0f),
                lacunarity[i],
                offset[i],
                gain[i],
                distortion[i],
                type_,
                normalize_,
                period);
            r_color[i] = ColorGeometry4f(c[0], c[1], c[2], 1.0f);
            if (compute_factor) {
              r_factor[i] = c[0];
            }
          });
        }
        else if (compute_factor) {
          mask.foreach_index([&](const int64_t i) {
            const float s = scale[i];
            const float2 position = float2(vector[i] * s);
            const float2 period{period_or_zero(tiling_, period_2d[i].x),
                                period_or_zero(tiling_, period_2d[i].y)};
            r_factor[i] = noise::perlin_fractal_distorted(position,
                                                          math::clamp(detail[i], 0.0f, 15.0f),
                                                          math::max(roughness[i], 0.0f),
                                                          lacunarity[i],
                                                          offset[i],
                                                          gain[i],
                                                          distortion[i],
                                                          type_,
                                                          normalize_,
                                                          period);
          });
        }
        break;
      }
      case 3: {
        const VArray<float3> &vector = params.readonly_single_input<float3>(0, "Vector");
        if (compute_color) {
          mask.foreach_index([&](const int64_t i) {
            const float s = scale[i];
            const float3 position = vector[i] * s;
            const float3 period{period_or_zero(tiling_, period_3d[i].x),
                                period_or_zero(tiling_, period_3d[i].y),
                                period_or_zero(tiling_, period_3d[i].z)};
            const float3 c = noise::perlin_float3_fractal_distorted(
                position,
                math::clamp(detail[i], 0.0f, 15.0f),
                math::max(roughness[i], 0.0f),
                lacunarity[i],
                offset[i],
                gain[i],
                distortion[i],
                type_,
                normalize_,
                period);
            r_color[i] = ColorGeometry4f(c[0], c[1], c[2], 1.0f);
            if (compute_factor) {
              r_factor[i] = c[0];
            }
          });
        }
        else if (compute_factor) {
          mask.foreach_index([&](const int64_t i) {
            const float s = scale[i];
            const float3 position = vector[i] * s;
            const float3 period{period_or_zero(tiling_, period_3d[i].x),
                                period_or_zero(tiling_, period_3d[i].y),
                                period_or_zero(tiling_, period_3d[i].z)};
            r_factor[i] = noise::perlin_fractal_distorted(position,
                                                          math::clamp(detail[i], 0.0f, 15.0f),
                                                          math::max(roughness[i], 0.0f),
                                                          lacunarity[i],
                                                          offset[i],
                                                          gain[i],
                                                          distortion[i],
                                                          type_,
                                                          normalize_,
                                                          period);
          });
        }
        break;
      }
      case 4: {
        const VArray<float3> &vector = params.readonly_single_input<float3>(0, "Vector");
        const VArray<float> &w = params.readonly_single_input<float>(1, "W");
        if (compute_color) {
          mask.foreach_index([&](const int64_t i) {
            const float s = scale[i];
            const float3 position_vector = vector[i] * s;
            const float position_w = w[i] * s;
            const float4 position{
                position_vector[0], position_vector[1], position_vector[2], position_w};
            const float4 period{period_or_zero(tiling_, period_3d[i].x),
                                period_or_zero(tiling_, period_3d[i].y),
                                period_or_zero(tiling_, period_3d[i].z),
                                period_or_zero(tiling_, period_w[i])};
            const float3 c = noise::perlin_float3_fractal_distorted(
                position,
                math::clamp(detail[i], 0.0f, 15.0f),
                math::max(roughness[i], 0.0f),
                lacunarity[i],
                offset[i],
                gain[i],
                distortion[i],
                type_,
                normalize_,
                period);
            r_color[i] = ColorGeometry4f(c[0], c[1], c[2], 1.0f);
            if (compute_factor) {
              r_factor[i] = c[0];
            }
          });
        }
        else if (compute_factor) {
          mask.foreach_index([&](const int64_t i) {
            const float s = scale[i];
            const float3 position_vector = vector[i] * s;
            const float position_w = w[i] * s;
            const float4 position{
                position_vector[0], position_vector[1], position_vector[2], position_w};
            const float4 period{period_or_zero(tiling_, period_3d[i].x),
                                period_or_zero(tiling_, period_3d[i].y),
                                period_or_zero(tiling_, period_3d[i].z),
                                period_or_zero(tiling_, period_w[i])};
            r_factor[i] = noise::perlin_fractal_distorted(position,
                                                          math::clamp(detail[i], 0.0f, 15.0f),
                                                          math::max(roughness[i], 0.0f),
                                                          lacunarity[i],
                                                          offset[i],
                                                          gain[i],
                                                          distortion[i],
                                                          type_,
                                                          normalize_,
                                                          period);
          });
        }
        break;
      }
    }
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    hints.allocates_array = false;
    hints.min_grain_size = 100;
    return hints;
  }

  void hash_unique(UniqueHashBytes &hash) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
    hash.add(dimensions_);
    hash.add(type_);
    hash.add(normalize_);
    hash.add(tiling_);
  }
};

static void sh_node_noise_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeTexNoise &storage = node_storage(builder.node());
  builder.construct_and_set_matching_fn<NoiseFunction>(
      storage.dimensions, storage.type, storage.normalize, storage.tiling != 0);
}

NODE_SHADER_MATERIALX_BEGIN
#ifdef WITH_MATERIALX
{
  /* NOTE: Some inputs aren't supported by MaterialX. */
  NodeItem scale = get_input_value("Scale", NodeItem::Type::Float);
  NodeItem detail = get_input_default("Detail", NodeItem::Type::Float);
  NodeItem lacunarity = get_input_value("Lacunarity", NodeItem::Type::Float);
  /* Empirically, higher octaves lead to NaNs on e.g. Metal and NVIDIA. */
  const int octaves = int(math::clamp(detail.value->asA<float>(), 1.0f, 13.0f));

  NodeItem position = create_node("position", NodeItem::Type::Vector3);
  position = position * scale;

  return create_node(
      "fractal3d",
      STREQ(socket_out_->identifier, "Fac") ? NodeItem::Type::Float : NodeItem::Type::Color3,
      {{"position", position}, {"octaves", val(octaves)}, {"lacunarity", lacunarity}});
}
#endif
NODE_SHADER_MATERIALX_END

}  // namespace nodes::node_shader_tex_noise_cc

void register_node_type_sh_tex_noise()
{
  namespace file_ns = nodes::node_shader_tex_noise_cc;

  static bke::bNodeType ntype;

  common_node_type_base(&ntype, "ShaderNodeTexNoise"_ustr, SH_NODE_TEX_NOISE);
  ntype.ui_name = "Noise Texture";
  ntype.ui_description = "Generate fractal Perlin noise";
  ntype.enum_name_legacy = "TEX_NOISE";
  ntype.nclass = NODE_CLASS_TEXTURE;
  ntype.declare = file_ns::sh_node_tex_noise_declare;
  ntype.draw_buttons = file_ns::node_shader_buts_tex_noise;
  ntype.initfunc = file_ns::node_shader_init_tex_noise;
  bke::node_type_storage(
      ntype, "NodeTexNoise", node_free_standard_storage, node_copy_standard_storage);
  ntype.gpu_fn = file_ns::node_shader_gpu_tex_noise;
  ntype.build_multi_function = file_ns::sh_node_noise_build_multi_function;
  ntype.materialx_fn = file_ns::node_shader_materialx;
  ntype.default_width = bke::NodeWidth::_160;

  bke::node_register_type(ntype);
}

}  // namespace blender
