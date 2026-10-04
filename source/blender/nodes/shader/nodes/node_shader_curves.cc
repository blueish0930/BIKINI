/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup shdnodes
 */

#include "node_shader_util.hh"

#include "BKE_colortools.hh"

#include "BLI_math_vector_c.hh"

#include "NOD_multi_function.hh"

#include "node_util.hh"

namespace blender {

/** Linked curve socket, including one passed through a node group. Falls back when the linked
 * mapping does not have enough channels for this node. */
static CurveMapping *curve_from_node_input(const bNode &node,
                                           GPUNodeStack *in,
                                           CurveMapping *fallback,
                                           const int min_channels = 1)
{
  auto usable = [&](const CurveMapping *curve) -> bool {
    if (curve == nullptr) {
      return false;
    }
    const int last = min_ii(max_ii(min_channels, 1), CM_TOT) - 1;
    return curve->cm[last].curve != nullptr;
  };

  int index = 0;
  for (const bNodeSocket *sock : node.input_sockets()) {
    if (sock->type == SOCK_CURVE) {
      const bool linked = sock->link != nullptr && (sock->link->flag & NODE_LINK_MUTED) == 0 &&
                          sock->link->fromsock != nullptr;
      if (linked) {
        /* The GPU stack often holds the data node's unused output default. Resolve to the
         * input the user edited, then fall back to the stack pointer. */
        if (CurveMapping *resolved = bke::node_socket_curve_resolved(*sock)) {
          if (usable(resolved)) {
            return resolved;
          }
        }
        if (in != nullptr && in[index].link != nullptr) {
          auto *linked_curve = reinterpret_cast<CurveMapping *>(in[index].link);
          if (usable(linked_curve)) {
            return linked_curve;
          }
        }
      }
      return fallback;
    }
    index++;
  }
  return fallback;
}

/** Runtime curve passed through geometry nodes. Null or too few channels keeps node storage. */
static CurveMapping *curve_from_eval_input(const VArray<CurveMapping *> &curves,
                                           CurveMapping *fallback,
                                           const int min_channels)
{
  CurveMapping *linked = curves.is_single() ? curves.get_internal_single() : nullptr;
  if (linked != nullptr) {
    const int last = min_ii(max_ii(min_channels, 1), CM_TOT) - 1;
    if (linked->cm[last].curve != nullptr) {
      BKE_curvemapping_init(linked);
      return linked;
    }
  }
  BKE_curvemapping_init(fallback);
  return fallback;
}

namespace nodes::node_shader_curves_cc::vec {

static void sh_node_curve_vec_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  /* Socket only. The curve editor stays in the node body until this input is linked. */
  b.add_input<decl::Curve>("Curve"_ustr)
      .subtype(NODE_SOCKET_CURVE_VECTOR)
      .hide_value()
      .description(
          "Curve to sample. When unconnected, the curve on this node is used. Connect a Curve "
          "node or a node group Curve interface to drive it");
  b.add_input<decl::Float>("Factor"_ustr, "Fac"_ustr)
      .min(0.0f)
      .max(1.0f)
      .default_value(1.0f)
      .subtype(PROP_FACTOR)
      .no_muted_links()
      .description("Amount of influence the node exerts on the output vector")
      .compositor_domain_priority(1);
  b.add_input<decl::Vector>("Vector"_ustr)
      .min(-1.0f)
      .max(1.0f)
      .description("Vector which would be mapped to the curve")
      .compositor_domain_priority(0);
  b.add_output<decl::Vector>("Vector"_ustr);
}

static void node_shader_init_curve_vec(bNodeTree * /*ntree*/, bNode *node)
{
  node->storage = BKE_curvemapping_add(3, -1.0f, -1.0f, 1.0f, 1.0f);
}

static int gpu_shader_curve_vec(GPUMaterial *mat,
                                bNode *node,
                                bNodeExecData * /*execdata*/,
                                GPUNodeStack *in,
                                GPUNodeStack *out)
{
  CurveMapping *curve_mapping = curve_from_node_input(
      *node, in, static_cast<CurveMapping *>(node->storage), 3);

  BKE_curvemapping_init(curve_mapping);
  float *band_values;
  int band_size;
  BKE_curvemapping_table_RGBA(curve_mapping, &band_values, &band_size);
  float band_layer;
  GPUNodeLink *band_texture = GPU_color_band(mat, band_size, band_values, &band_layer);

  float start_slopes[CM_TOT];
  float end_slopes[CM_TOT];
  BKE_curvemapping_compute_slopes(curve_mapping, start_slopes, end_slopes);
  float range_minimums[CM_TOT];
  BKE_curvemapping_get_range_minimums(curve_mapping, range_minimums);
  float range_dividers[CM_TOT];
  BKE_curvemapping_compute_range_dividers(curve_mapping, range_dividers);

  return GPU_stack_link(mat,
                        node,
                        "curves_vector_mixed",
                        in,
                        out,
                        band_texture,
                        GPU_constant(&band_layer),
                        GPU_uniform(range_minimums),
                        GPU_uniform(range_dividers),
                        GPU_uniform(start_slopes),
                        GPU_uniform(end_slopes));
}

class CurveVecFunction : public mf::MultiFunction {
 private:
  /** Take ownership of the tree because it contains the curve mapping. */
  std::shared_ptr<const bNodeTree> tree_;
  const CurveMapping &cumap_;

 public:
  CurveVecFunction(const CurveMapping &cumap, std::shared_ptr<const bNodeTree> tree)
      : tree_(std::move(tree)), cumap_(cumap)
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Curve Vec", signature};
      builder.single_input<CurveMapping *>("Curve");
      builder.single_input<float>("Fac");
      builder.single_input<float3>("Vector");
      builder.single_output<float3>("Vector");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<CurveMapping *> &curves = params.readonly_single_input<CurveMapping *>(0, "Curve");
    const VArray<float> &fac = params.readonly_single_input<float>(1, "Fac");
    const VArray<float3> &vec_in = params.readonly_single_input<float3>(2, "Vector");
    MutableSpan<float3> vec_out = params.uninitialized_single_output<float3>(3, "Vector");
    const CurveMapping *cumap = curve_from_eval_input(curves, const_cast<CurveMapping *>(&cumap_), 3);

    mask.foreach_index([&](const int64_t i) {
      BKE_curvemapping_evaluate3F(cumap, vec_out[i], vec_in[i]);
      if (fac[i] != 1.0f) {
        interp_v3_v3v3(vec_out[i], vec_in[i], vec_out[i], fac[i]);
      }
    });
  }

  void hash_unique(UniqueHashBytes &hash) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
    hash.add(&cumap_);
  }
};

static void sh_node_curve_vec_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const bNode &bnode = builder.node();
  CurveMapping *cumap = curve_from_node_input(
      bnode, nullptr, static_cast<CurveMapping *>(bnode.storage), 3);
  BKE_curvemapping_init(cumap);
  builder.construct_and_set_matching_fn<CurveVecFunction>(*cumap, builder.shared_tree());
}

NODE_SHADER_MATERIALX_BEGIN
#ifdef WITH_MATERIALX
{
  /* TODO: implement */
  return get_input_value("Vector", NodeItem::Type::Vector3);
}
#endif
NODE_SHADER_MATERIALX_END

}  // namespace nodes::node_shader_curves_cc::vec

void register_node_type_sh_curve_vec()
{
  namespace file_ns = nodes::node_shader_curves_cc::vec;

  static bke::bNodeType ntype;

  common_node_type_base(&ntype, "ShaderNodeVectorCurve"_ustr, SH_NODE_CURVE_VEC);
  ntype.ui_name = "Vector Curves";
  ntype.ui_description = "Map input vector components with curves";
  ntype.enum_name_legacy = "CURVE_VEC";
  ntype.nclass = NODE_CLASS_OP_VECTOR;
  ntype.declare = file_ns::sh_node_curve_vec_declare;
  ntype.initfunc = file_ns::node_shader_init_curve_vec;
  ntype.default_width = bke::NodeWidth::_240;
  bke::node_type_storage(ntype, "CurveMapping", node_free_curves, node_copy_curves);
  ntype.gpu_fn = file_ns::gpu_shader_curve_vec;
  ntype.build_multi_function = file_ns::sh_node_curve_vec_build_multi_function;
  ntype.materialx_fn = file_ns::node_shader_materialx;

  bke::node_register_type(ntype);
}

/* **************** CURVE RGB  ******************** */

namespace nodes::node_shader_curves_cc::rgb {

static void sh_node_curve_rgb_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  /* Socket only. The curve editor stays in the node body until this input is linked. */
  b.add_input<decl::Curve>("Curve"_ustr)
      .subtype(NODE_SOCKET_CURVE_COLOR)
      .hide_value()
      .description(
          "Curve to sample. When unconnected, the curve on this node is used. Connect a Curve "
          "node or a node group Curve interface to drive it");
  b.add_input<decl::Float>("Factor"_ustr, "Fac"_ustr)
      .min(0.0f)
      .max(1.0f)
      .default_value(1.0f)
      .subtype(PROP_FACTOR)
      .no_muted_links()
      .description("Amount of influence the node exerts on the output color")
      .compositor_domain_priority(1);
  b.add_input<decl::Color>("Color"_ustr)
      .default_value({1.0f, 1.0f, 1.0f, 1.0f})
      .description("Color input on which correction will be applied")
      .compositor_domain_priority(0);
  b.add_output<decl::Color>("Color"_ustr);
}

static void node_shader_init_curve_rgb(bNodeTree * /*ntree*/, bNode *node)
{
  node->storage = BKE_curvemapping_add(4, 0.0f, 0.0f, 1.0f, 1.0f);
}

static int gpu_shader_curve_rgb(GPUMaterial *mat,
                                bNode *node,
                                bNodeExecData * /*execdata*/,
                                GPUNodeStack *in,
                                GPUNodeStack *out)
{
  CurveMapping *curve_mapping = curve_from_node_input(
      *node, in, static_cast<CurveMapping *>(node->storage), 4);

  BKE_curvemapping_init(curve_mapping);
  float *band_values;
  int band_size;
  BKE_curvemapping_table_RGBA(curve_mapping, &band_values, &band_size);
  float band_layer;
  GPUNodeLink *band_texture = GPU_color_band(mat, band_size, band_values, &band_layer);

  float start_slopes[CM_TOT];
  float end_slopes[CM_TOT];
  BKE_curvemapping_compute_slopes(curve_mapping, start_slopes, end_slopes);
  float range_minimums[CM_TOT];
  BKE_curvemapping_get_range_minimums(curve_mapping, range_minimums);
  float range_dividers[CM_TOT];
  BKE_curvemapping_compute_range_dividers(curve_mapping, range_dividers);

  /* Shader nodes don't do white balancing. */
  float black_level[4] = {0.0f, 0.0f, 0.0f, 1.0f};
  float white_level[4] = {1.0f, 1.0f, 1.0f, 1.0f};

  /* If the RGB curves do nothing, use a function that skips RGB computations. */
  if (BKE_curvemapping_is_map_identity(curve_mapping, 0) &&
      BKE_curvemapping_is_map_identity(curve_mapping, 1) &&
      BKE_curvemapping_is_map_identity(curve_mapping, 2))
  {
    return GPU_stack_link(mat,
                          node,
                          "curves_combined_only",
                          in,
                          out,
                          GPU_constant(black_level),
                          GPU_constant(white_level),
                          band_texture,
                          GPU_constant(&band_layer),
                          GPU_uniform(&range_minimums[3]),
                          GPU_uniform(&range_dividers[3]),
                          GPU_uniform(&start_slopes[3]),
                          GPU_uniform(&end_slopes[3]));
  }

  return GPU_stack_link(mat,
                        node,
                        "curves_combined_rgb",
                        in,
                        out,
                        GPU_constant(black_level),
                        GPU_constant(white_level),
                        band_texture,
                        GPU_constant(&band_layer),
                        GPU_uniform(range_minimums),
                        GPU_uniform(range_dividers),
                        GPU_uniform(start_slopes),
                        GPU_uniform(end_slopes));
}

class CurveRGBFunction : public mf::MultiFunction {
 private:
  /** Take ownership of the tree because it contains the curve mapping. */
  std::shared_ptr<const bNodeTree> tree_;
  const CurveMapping &cumap_;

 public:
  CurveRGBFunction(const CurveMapping &cumap, std::shared_ptr<const bNodeTree> tree)
      : tree_(std::move(tree)), cumap_(cumap)
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Curve RGB", signature};
      builder.single_input<CurveMapping *>("Curve");
      builder.single_input<float>("Fac");
      builder.single_input<ColorGeometry4f>("Color");
      builder.single_output<ColorGeometry4f>("Color");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<CurveMapping *> &curves = params.readonly_single_input<CurveMapping *>(0, "Curve");
    const VArray<float> &fac = params.readonly_single_input<float>(1, "Fac");
    const VArray<ColorGeometry4f> &col_in = params.readonly_single_input<ColorGeometry4f>(2,
                                                                                          "Color");
    MutableSpan<ColorGeometry4f> col_out = params.uninitialized_single_output<ColorGeometry4f>(
        3, "Color");
    const CurveMapping *cumap = curve_from_eval_input(curves, const_cast<CurveMapping *>(&cumap_), 4);

    mask.foreach_index([&](const int64_t i) {
      BKE_curvemapping_evaluateRGBF(cumap, col_out[i], col_in[i]);
      if (fac[i] != 1.0f) {
        interp_v3_v3v3(col_out[i], col_in[i], col_out[i], fac[i]);
      }
      col_out[i].a = 1.0f;
    });
  }

  void hash_unique(UniqueHashBytes &hash) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
    hash.add(&cumap_);
  }
};

static void sh_node_curve_rgb_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const bNode &bnode = builder.node();
  CurveMapping *cumap = curve_from_node_input(
      bnode, nullptr, static_cast<CurveMapping *>(bnode.storage), 4);
  BKE_curvemapping_init(cumap);
  builder.construct_and_set_matching_fn<CurveRGBFunction>(*cumap, builder.shared_tree());
}

NODE_SHADER_MATERIALX_BEGIN
#ifdef WITH_MATERIALX
{
  /* TODO: implement */
  return get_input_value("Color", NodeItem::Type::Color3);
}
#endif
NODE_SHADER_MATERIALX_END

}  // namespace nodes::node_shader_curves_cc::rgb

void register_node_type_sh_curve_rgb()
{
  namespace file_ns = nodes::node_shader_curves_cc::rgb;

  static bke::bNodeType ntype;

  common_node_type_base(&ntype, "ShaderNodeRGBCurve"_ustr, SH_NODE_CURVE_RGB);
  ntype.ui_name = "RGB Curves";
  ntype.ui_description = "Apply color corrections for each color channel";
  ntype.enum_name_legacy = "CURVE_RGB";
  ntype.nclass = NODE_CLASS_OP_COLOR;
  ntype.declare = file_ns::sh_node_curve_rgb_declare;
  ntype.initfunc = file_ns::node_shader_init_curve_rgb;
  ntype.default_width = bke::NodeWidth::_240;
  bke::node_type_storage(ntype, "CurveMapping", node_free_curves, node_copy_curves);
  ntype.gpu_fn = file_ns::gpu_shader_curve_rgb;
  ntype.build_multi_function = file_ns::sh_node_curve_rgb_build_multi_function;
  ntype.materialx_fn = file_ns::node_shader_materialx;

  bke::node_register_type(ntype);
}

/* **************** CURVE FLOAT  ******************** */

namespace nodes::node_shader_curves_cc::flt {

static void sh_node_curve_float_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  /* Socket only. The curve editor stays in the node body until this input is linked. */
  b.add_input<decl::Curve>("Curve"_ustr)
      .subtype(NODE_SOCKET_CURVE_FLOAT)
      .hide_value()
      .description(
          "Curve to sample. When unconnected, the curve on this node is used. Connect a Curve "
          "node or a node group Curve interface to drive it");
  b.add_input<decl::Float>("Factor"_ustr)
      .min(0.0f)
      .max(1.0f)
      .default_value(1.0f)
      .subtype(PROP_FACTOR)
      .no_muted_links()
      .compositor_domain_priority(1);
  b.add_input<decl::Float>("Value"_ustr)
      .default_value(1.0f)
      .is_default_link_socket()
      .compositor_domain_priority(0);
  b.add_output<decl::Float>("Value"_ustr);
}

static void node_shader_init_curve_float(bNodeTree * /*ntree*/, bNode *node)
{
  node->storage = BKE_curvemapping_add(1, 0.0f, 0.0f, 1.0f, 1.0f);
}

static int gpu_shader_curve_float(GPUMaterial *mat,
                                  bNode *node,
                                  bNodeExecData * /*execdata*/,
                                  GPUNodeStack *in,
                                  GPUNodeStack *out)
{
  CurveMapping *curve_mapping = curve_from_node_input(
      *node, in, static_cast<CurveMapping *>(node->storage));

  BKE_curvemapping_init(curve_mapping);
  float *band_values;
  int band_size;
  BKE_curvemapping_table_RGBA(curve_mapping, &band_values, &band_size);
  float band_layer;
  GPUNodeLink *band_texture = GPU_color_band(mat, band_size, band_values, &band_layer);

  float start_slopes[CM_TOT];
  float end_slopes[CM_TOT];
  BKE_curvemapping_compute_slopes(curve_mapping, start_slopes, end_slopes);
  float range_minimums[CM_TOT];
  BKE_curvemapping_get_range_minimums(curve_mapping, range_minimums);
  float range_dividers[CM_TOT];
  BKE_curvemapping_compute_range_dividers(curve_mapping, range_dividers);

  return GPU_stack_link(mat,
                        node,
                        "curves_float_mixed",
                        in,
                        out,
                        band_texture,
                        GPU_constant(&band_layer),
                        GPU_uniform(range_minimums),
                        GPU_uniform(range_dividers),
                        GPU_uniform(start_slopes),
                        GPU_uniform(end_slopes));
}

class CurveFloatFunction : public mf::MultiFunction {
 private:
  /** Take ownership of the tree because it contains the curve mapping. */
  std::shared_ptr<const bNodeTree> tree_;
  const CurveMapping &cumap_;

 public:
  CurveFloatFunction(const CurveMapping &cumap, std::shared_ptr<const bNodeTree> tree)
      : tree_(std::move(tree)), cumap_(cumap)
  {
    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Curve Float", signature};
      builder.single_input<CurveMapping *>("Curve");
      builder.single_input<float>("Factor");
      builder.single_input<float>("Value");
      builder.single_output<float>("Value");
      return signature;
    }();
    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<CurveMapping *> &curves = params.readonly_single_input<CurveMapping *>(0, "Curve");
    const VArray<float> &fac = params.readonly_single_input<float>(1, "Factor");
    const VArray<float> &val_in = params.readonly_single_input<float>(2, "Value");
    MutableSpan<float> val_out = params.uninitialized_single_output<float>(3, "Value");
    const CurveMapping *cumap = curve_from_eval_input(curves, const_cast<CurveMapping *>(&cumap_), 1);

    mask.foreach_index([&](const int64_t i) {
      val_out[i] = BKE_curvemapping_evaluateF(cumap, 0, val_in[i]);
      if (fac[i] != 1.0f) {
        val_out[i] = (1.0f - fac[i]) * val_in[i] + fac[i] * val_out[i];
      }
    });
  }

  void hash_unique(UniqueHashBytes &hash) const override
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
    hash.add(&cumap_);
  }
};

static void sh_node_curve_float_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const bNode &bnode = builder.node();
  CurveMapping *cumap = curve_from_node_input(
      bnode, nullptr, static_cast<CurveMapping *>(bnode.storage));
  BKE_curvemapping_init(cumap);
  builder.construct_and_set_matching_fn<CurveFloatFunction>(*cumap, builder.shared_tree());
}

NODE_SHADER_MATERIALX_BEGIN
#ifdef WITH_MATERIALX
{
  /* TODO: implement */
  return get_input_value("Value", NodeItem::Type::Float);
}
#endif
NODE_SHADER_MATERIALX_END

}  // namespace nodes::node_shader_curves_cc::flt

void register_node_type_sh_curve_float()
{
  namespace file_ns = nodes::node_shader_curves_cc::flt;

  static bke::bNodeType ntype;

  common_node_type_base(&ntype, "ShaderNodeFloatCurve"_ustr, SH_NODE_CURVE_FLOAT);
  ntype.ui_name = "Float Curve";
  ntype.ui_description = "Map an input float to a curve and outputs a float value";
  ntype.enum_name_legacy = "CURVE_FLOAT";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = file_ns::sh_node_curve_float_declare;
  ntype.initfunc = file_ns::node_shader_init_curve_float;
  ntype.default_width = bke::NodeWidth::_240;
  bke::node_type_storage(ntype, "CurveMapping", node_free_curves, node_copy_curves);
  ntype.gpu_fn = file_ns::gpu_shader_curve_float;
  ntype.build_multi_function = file_ns::sh_node_curve_float_build_multi_function;
  ntype.materialx_fn = file_ns::node_shader_materialx;

  bke::node_register_type(ntype);
}

}  // namespace blender
