/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * RBF attribute interpolation (exact at sample points).
 *
 * Given sample points with values, fit a radial-basis interpolant so that
 * f(x_i) = v_i at every sample (a "thin film" stretched through the samples),
 * then evaluate that function on the context Sample Position field.
 *
 * Layout:
 *   Source
 *   Value (in) | Value (out)   <- top-right output
 *   Sample Position
 *   [Kernel] Type, Size
 */

#include <Eigen/Dense>

#include "BLI_math_base.hh"
#include "BLI_task.hh"

#include "BKE_attribute_math.hh"
#include "BKE_geometry_fields.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_rbf_interpolate_cc {

enum class KernelType : int8_t {
  Gaussian = 0,
  InverseMultiquadric = 1,
  Linear = 2,
  Cubic = 3,
  Quintic = 4,
  WendlandC2 = 5,
};

static const EnumPropertyItem kernel_type_items[] = {
    {int(KernelType::Gaussian),
     "GAUSSIAN",
     0,
     N_("Gaussian"),
     N_("exp(-(r / size)^2)")},
    {int(KernelType::InverseMultiquadric),
     "INVERSE_MULTIQUADRIC",
     0,
     N_("Inverse Multiquadric"),
     N_("1 / sqrt(1 + (r / size)^2)")},
    {int(KernelType::Linear),
     "LINEAR",
     0,
     N_("Linear"),
     N_("max(0, 1 - r / size)")},
    {int(KernelType::Cubic),
     "CUBIC",
     0,
     N_("Cubic"),
     N_("max(0, (1 - r / size)^3)")},
    {int(KernelType::Quintic),
     "QUINTIC",
     0,
     N_("Quintic"),
     N_("max(0, (1 - r / size)^5)")},
    {int(KernelType::WendlandC2),
     "WENDLAND_C2",
     0,
     N_("Wendland C2"),
     N_("Compact C2 Wendland kernel with support radius = size")},
    {0, nullptr, 0, nullptr, nullptr},
};

/** Radial basis φ(r). At r = 0 these kernels evaluate to 1 (except after regularization). */
static float kernel_phi(const KernelType type, const float distance, const float size)
{
  const float s = math::max(size, 1e-8f);
  const float q = distance / s;
  switch (type) {
    case KernelType::Gaussian:
      return math::exp(-(q * q));
    case KernelType::InverseMultiquadric:
      return 1.0f / math::sqrt(1.0f + q * q);
    case KernelType::Linear:
      return math::max(0.0f, 1.0f - q);
    case KernelType::Cubic: {
      const float t = math::max(0.0f, 1.0f - q);
      return t * t * t;
    }
    case KernelType::Quintic: {
      const float t = math::max(0.0f, 1.0f - q);
      return t * t * t * t * t;
    }
    case KernelType::WendlandC2: {
      if (q >= 1.0f) {
        return 0.0f;
      }
      const float t = 1.0f - q;
      const float t2 = t * t;
      return t2 * t2 * (4.0f * q + 1.0f);
    }
  }
  return 0.0f;
}

static Span<float3> positions_from_geometry(const GeometrySet &geometry)
{
  if (const Mesh *mesh = geometry.get_mesh()) {
    return mesh->vert_positions();
  }
  if (const PointCloud *pointcloud = geometry.get_pointcloud()) {
    return pointcloud->positions();
  }
  return {};
}

static const GeometryComponent *find_point_component(const GeometrySet &geometry)
{
  if (geometry.has_mesh() && geometry.get_mesh()->verts_num > 0) {
    return geometry.get_component(GeometryComponent::Type::Mesh);
  }
  if (geometry.has_pointcloud() && geometry.get_pointcloud()->totpoint > 0) {
    return geometry.get_component(GeometryComponent::Type::PointCloud);
  }
  return nullptr;
}

/* -------------------------------------------------------------------- */
/** \name Channel packing helpers (solve RBF per float channel)
 * \{ */

template<typename T> struct RBFChannels;

template<> struct RBFChannels<float> {
  static constexpr int count = 1;
  static void load(const float &v, MutableSpan<float> ch)
  {
    ch[0] = v;
  }
  static float store(const Span<float> ch)
  {
    return ch[0];
  }
};

template<> struct RBFChannels<float2> {
  static constexpr int count = 2;
  static void load(const float2 &v, MutableSpan<float> ch)
  {
    ch[0] = v.x;
    ch[1] = v.y;
  }
  static float2 store(const Span<float> ch)
  {
    return float2(ch[0], ch[1]);
  }
};

template<> struct RBFChannels<float3> {
  static constexpr int count = 3;
  static void load(const float3 &v, MutableSpan<float> ch)
  {
    ch[0] = v.x;
    ch[1] = v.y;
    ch[2] = v.z;
  }
  static float3 store(const Span<float> ch)
  {
    return float3(ch[0], ch[1], ch[2]);
  }
};

template<> struct RBFChannels<float4> {
  static constexpr int count = 4;
  static void load(const float4 &v, MutableSpan<float> ch)
  {
    ch[0] = v.x;
    ch[1] = v.y;
    ch[2] = v.z;
    ch[3] = v.w;
  }
  static float4 store(const Span<float> ch)
  {
    return float4(ch[0], ch[1], ch[2], ch[3]);
  }
};

template<> struct RBFChannels<ColorGeometry4f> {
  static constexpr int count = 4;
  static void load(const ColorGeometry4f &v, MutableSpan<float> ch)
  {
    ch[0] = v.r;
    ch[1] = v.g;
    ch[2] = v.b;
    ch[3] = v.a;
  }
  static ColorGeometry4f store(const Span<float> ch)
  {
    return ColorGeometry4f(ch[0], ch[1], ch[2], ch[3]);
  }
};

template<> struct RBFChannels<ColorGeometry4b> {
  static constexpr int count = 4;
  static void load(const ColorGeometry4b &v, MutableSpan<float> ch)
  {
    ch[0] = float(v.r) * (1.0f / 255.0f);
    ch[1] = float(v.g) * (1.0f / 255.0f);
    ch[2] = float(v.b) * (1.0f / 255.0f);
    ch[3] = float(v.a) * (1.0f / 255.0f);
  }
  static ColorGeometry4b store(const Span<float> ch)
  {
    return ColorGeometry4b(uint8_t(math::clamp(ch[0] * 255.0f, 0.0f, 255.0f)),
                           uint8_t(math::clamp(ch[1] * 255.0f, 0.0f, 255.0f)),
                           uint8_t(math::clamp(ch[2] * 255.0f, 0.0f, 255.0f)),
                           uint8_t(math::clamp(ch[3] * 255.0f, 0.0f, 255.0f)));
  }
};

template<> struct RBFChannels<int> {
  static constexpr int count = 1;
  static void load(const int &v, MutableSpan<float> ch)
  {
    ch[0] = float(v);
  }
  static int store(const Span<float> ch)
  {
    return int(math::round(ch[0]));
  }
};

template<> struct RBFChannels<bool> {
  static constexpr int count = 1;
  static void load(const bool &v, MutableSpan<float> ch)
  {
    ch[0] = v ? 1.0f : 0.0f;
  }
  static bool store(const Span<float> ch)
  {
    return ch[0] >= 0.5f;
  }
};

template<> struct RBFChannels<int8_t> {
  static constexpr int count = 1;
  static void load(const int8_t &v, MutableSpan<float> ch)
  {
    ch[0] = float(v);
  }
  static int8_t store(const Span<float> ch)
  {
    return int8_t(math::clamp(math::round(ch[0]), -128.0f, 127.0f));
  }
};

template<> struct RBFChannels<int2> {
  static constexpr int count = 2;
  static void load(const int2 &v, MutableSpan<float> ch)
  {
    ch[0] = float(v.x);
    ch[1] = float(v.y);
  }
  static int2 store(const Span<float> ch)
  {
    return int2(int(math::round(ch[0])), int(math::round(ch[1])));
  }
};

template<> struct RBFChannels<math::Quaternion> {
  static constexpr int count = 3;
  static void load(const math::Quaternion &v, MutableSpan<float> ch)
  {
    const float3 e = v.expmap();
    ch[0] = e.x;
    ch[1] = e.y;
    ch[2] = e.z;
  }
  static math::Quaternion store(const Span<float> ch)
  {
    return math::Quaternion::expmap(float3(ch[0], ch[1], ch[2]));
  }
};

/** \} */

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();

  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_default_layout();

  b.add_input<decl::Geometry>("Source"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::PointCloud})
      .description("Sample points that carry the attribute to interpolate from");

  /* Value in + Value out on the same row (output at top-right). */
  if (node != nullptr) {
    const eCustomDataType data_type = eCustomDataType(node->custom1);
    b.add_input(data_type, "Value"_ustr)
        .hide_value()
        .evaluated_geometry_field()
        .description("Attribute / field defined on the sample points");
    b.add_output(data_type, "Value"_ustr)
        .structure_type(StructureType::Field)
        .propagate_references()
        .align_with_previous()
        .description(
            "Interpolated attribute; exact at sample positions, smooth elsewhere (RBF film)");
  }

  b.add_input<decl::Vector>("Sample Position"_ustr)
      .default_input_type(NODE_DEFAULT_INPUT_POSITION_FIELD)
      .structure_type(StructureType::Dynamic)
      .description(
          "Positions where the interpolant is evaluated (defaults to the context Position)");

  {
    auto &panel = b.add_panel("Kernel"_ustr);
    panel.add_input<decl::Menu>("Kernel Type"_ustr)
        .static_items(kernel_type_items)
        .default_value(KernelType::Gaussian)
        .optional_label()
        .description("Radial basis function φ(r) used for the interpolant");
    panel.add_input<decl::Float>("Kernel Size"_ustr)
        .default_value(1.0f)
        .min(0.0f)
        .subtype(PROP_DISTANCE)
        .description("Shape parameter / support radius of the kernel");
  }
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.prop(ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_FLOAT;
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const NodeDeclaration &declaration = *params.node_type().static_declaration;
  search_link_ops_for_declarations(params, declaration.inputs);

  const std::optional<eCustomDataType> type = bke::socket_type_to_custom_data_type(
      params.other_socket().type);
  if (type && *type != CD_PROP_STRING) {
    params.add_item(IFACE_("Value"), [type](LinkSearchOpParams &params) {
      bNode &node = params.add_node("GeometryNodeRBFInterpolate"_ustr);
      node.custom1 = *type;
      params.update_and_connect_available_socket(node, "Value"_ustr);
    });
  }
}

class RBFInterpolateFunction : public mf::MultiFunction {
 private:
  GeometrySet source_;
  GField value_field_;
  KernelType kernel_type_;
  float kernel_size_;
  mf::Signature signature_;

  mutable CacheMutex mutex_;
  mutable Array<float3> sample_positions_;
  /** Solved RBF weights: samples × channels. Empty if fit failed. */
  mutable Array<float> weights_; /* layout: weight[sample * channels + channel] */
  mutable int channels_ = 0;
  mutable bool prepared_ = false;
  mutable bool fit_ok_ = false;

 public:
  RBFInterpolateFunction(GeometrySet source,
                         GField value_field,
                         const KernelType kernel_type,
                         const float kernel_size)
      : source_(std::move(source)),
        value_field_(std::move(value_field)),
        kernel_type_(kernel_type),
        kernel_size_(kernel_size)
  {
    source_.ensure_owns_direct_data();

    mf::SignatureBuilder builder{"RBF Interpolate", signature_};
    builder.single_input<float3>("Sample Position");
    builder.single_output("Value", value_field_.cpp_type());
    this->set_signature(&signature_);
  }

  void prepare_for_execution() const override
  {
    mutex_.ensure([&]() {
      if (prepared_) {
        return;
      }
      prepared_ = true;

      const GeometryComponent *component = find_point_component(source_);
      if (!component) {
        return;
      }

      const Span<float3> positions = positions_from_geometry(source_);
      if (positions.is_empty()) {
        return;
      }

      sample_positions_ = positions;
      const int n = positions.size();

      bke::GeometryFieldContext field_context(*component, bke::AttrDomain::Point);
      fn::FieldEvaluator evaluator(field_context, n);
      evaluator.add(value_field_);
      evaluator.evaluate();
      const GVArray &evaluated = evaluator.get_evaluated(0);

      const CPPType &type = value_field_.cpp_type();
      GArray<> sample_values(type, n);
      evaluated.materialize(sample_values.data());

      bool handled = false;
      bke::attribute_math::to_static_type(type, [&]<typename T>() {
        if constexpr (std::is_same_v<T, float> || std::is_same_v<T, float2> ||
                      std::is_same_v<T, float3> || std::is_same_v<T, float4> ||
                      std::is_same_v<T, ColorGeometry4f> || std::is_same_v<T, ColorGeometry4b> ||
                      std::is_same_v<T, int> || std::is_same_v<T, bool> ||
                      std::is_same_v<T, int8_t> || std::is_same_v<T, int2> ||
                      std::is_same_v<T, math::Quaternion>)
        {
          handled = true;
          this->fit_weights_for_type<T>(sample_values.as_span().typed<T>());
        }
      });

      if (!handled) {
        fit_ok_ = false;
      }
    });
  }

  template<typename T> void fit_weights_for_type(const Span<T> values) const
  {
    using Channels = RBFChannels<T>;
    const int n = sample_positions_.size();
    const int c = Channels::count;
    channels_ = c;

    /* Build dense Φ with Φ_ij = φ(||x_i - x_j||). Small diagonal jitter for stability when
     * samples coincide. */
    Eigen::MatrixXd Phi(n, n);
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        const float dist = math::distance(sample_positions_[i], sample_positions_[j]);
        Phi(i, j) = double(kernel_phi(kernel_type_, dist, kernel_size_));
      }
      Phi(i, i) += 1e-8;
    }

    /* RHS: n × c */
    Eigen::MatrixXd V(n, c);
    float ch_buf[16];
    BLI_assert(c <= 16);
    for (int i = 0; i < n; i++) {
      MutableSpan<float> ch(ch_buf, c);
      Channels::load(values[i], ch);
      for (int k = 0; k < c; k++) {
        V(i, k) = double(ch[k]);
      }
    }

    /* Solve Φ W = V  →  W is n × c (weights per sample per channel). */
    Eigen::PartialPivLU<Eigen::MatrixXd> lu(Phi);
    if (lu.determinant() == 0.0) {
      /* Singular: fall back to identity weights (nearest-only is handled at eval by just
       * copying if we leave fit_ok_ false). */
      fit_ok_ = false;
      return;
    }
    const Eigen::MatrixXd W = lu.solve(V);

    weights_.reinitialize(n * c);
    for (int i = 0; i < n; i++) {
      for (int k = 0; k < c; k++) {
        weights_[i * c + k] = float(W(i, k));
      }
    }
    fit_ok_ = true;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float3> &query_positions = params.readonly_single_input<float3>(
        0, "Sample Position");
    GMutableSpan dst = params.uninitialized_single_output(1, "Value");
    const CPPType &type = dst.type();

    if (!fit_ok_ || sample_positions_.is_empty() || weights_.is_empty()) {
      type.value_initialize_indices(dst.data(), mask);
      return;
    }

    const int n = sample_positions_.size();
    const int c = channels_;
    BLI_assert(weights_.size() == n * c);

    bke::attribute_math::to_static_type(type, [&]<typename T>() {
      if constexpr (std::is_same_v<T, float> || std::is_same_v<T, float2> ||
                    std::is_same_v<T, float3> || std::is_same_v<T, float4> ||
                    std::is_same_v<T, ColorGeometry4f> || std::is_same_v<T, ColorGeometry4b> ||
                    std::is_same_v<T, int> || std::is_same_v<T, bool> ||
                    std::is_same_v<T, int8_t> || std::is_same_v<T, int2> ||
                    std::is_same_v<T, math::Quaternion>)
      {
        using Channels = RBFChannels<T>;
        BLI_assert(Channels::count == c);
        MutableSpan<T> dst_typed = dst.typed<T>();

        mask.foreach_index([&](const int i) {
          const float3 query = query_positions[i];
          float ch_acc[16] = {};
          BLI_assert(c <= 16);

          for (int j = 0; j < n; j++) {
            const float dist = math::distance(query, sample_positions_[j]);
            const float phi = kernel_phi(kernel_type_, dist, kernel_size_);
            const int base = j * c;
            for (int k = 0; k < c; k++) {
              ch_acc[k] += weights_[base + k] * phi;
            }
          }

          dst_typed[i] = Channels::store(Span<float>(ch_acc, c));
        });
      }
      else {
        type.value_initialize_indices(dst.data(), mask);
      }
    });
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    /* Each evaluation walks all samples. */
    hints.min_grain_size = 32;
    return hints;
  }

  void hash_unique(UniqueHashBytes &hash) const override
  {
    static constexpr int8_t id = 1; /* bump: true RBF solve */
    hash.add(&id);
    hash.add(kernel_type_);
    hash.add(kernel_size_);
    hash.add(find_point_component(source_));
    fn::FieldHashDeep field_hash;
    hash.add(field_hash.ensure(value_field_));
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet source = params.extract_input<GeometrySet>("Source"_ustr);
  if (!find_point_component(source)) {
    if (!source.is_empty()) {
      params.error_message_add(NodeWarningType::Error,
                               TIP_("Source geometry must contain a mesh or point cloud"));
    }
    params.set_default_remaining_outputs();
    return;
  }

  const Span<float3> positions = positions_from_geometry(source);
  if (positions.size() > 4096) {
    params.error_message_add(
        NodeWarningType::Error,
        TIP_("Too many sample points for dense RBF (max 4096); reduce Source point count"));
    params.set_default_remaining_outputs();
    return;
  }
  if (positions.size() > 512) {
    params.error_message_add(
        NodeWarningType::Info,
        TIP_("Dense RBF solve is O(n³); large sample counts may be slow"));
  }

  GField value_field = params.extract_input<GField>("Value"_ustr);
  const KernelType kernel_type = params.extract_input<KernelType>("Kernel Type"_ustr);
  const float kernel_size = params.extract_input<float>("Kernel Size"_ustr);
  auto sample_position = params.extract_input<bke::SocketValueVariant>("Sample Position"_ustr);

  std::string error_message;
  bke::SocketValueVariant output_value;
  if (!execute_multi_function_on_value_variant(
          std::make_shared<RBFInterpolateFunction>(
              std::move(source), std::move(value_field), kernel_type, kernel_size),
          {&sample_position},
          {&output_value},
          params.user_data(),
          error_message))
  {
    params.set_default_remaining_outputs();
    params.error_message_add(NodeWarningType::Error, std::move(error_message));
    return;
  }

  params.set_output("Value"_ustr, std::move(output_value));
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "",
                    rna_enum_attribute_type_items,
                    NOD_inline_enum_accessors(custom1),
                    CD_PROP_FLOAT,
                    enums::attribute_type_type_with_socket_fn);
}

static void node_register()
{
  static bke::bNodeType ntype;

  geo_node_type_base(&ntype, "GeometryNodeRBFInterpolate"_ustr, GEO_NODE_RBF_INTERPOLATE);
  ntype.ui_name = "RBF Interpolate";
  ntype.ui_description =
      "Fit a radial-basis interpolant through sample values (exact at samples) and evaluate it "
      "on the context geometry";
  ntype.enum_name_legacy = "RBF_INTERPOLATE";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.gather_link_search_ops = node_gather_link_searches;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_rbf_interpolate_cc
