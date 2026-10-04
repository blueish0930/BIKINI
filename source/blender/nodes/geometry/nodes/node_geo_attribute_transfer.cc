/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Attribute Transfer (Houdini-style kernel sample).
 *
 * Sample a point-domain field on a target geometry from context Sample Positions.
 * All neighbors inside Smooth Distance are gathered, then combined as a kernel
 * weighted sum Σ φ(r_i) v_i. The kernel is NOT normalized (except Mean), so
 * distance falloff stays in the output: a neighbor at 0.2 of a 0.3 radius
 * contributes a faded value, not the original.
 *
 * Layout:
 *   Geometry
 *   Value (in) | Value (out)
 *   Sample Position
 *   [Kernel] Type, Smooth Distance
 */

#include "BLI_cache_mutex.hh"
#include "BLI_generic_array.hh"
#include "BLI_kdtree.hh"
#include "BLI_math_base.hh"
#include "BLI_math_quaternion.hh"

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

namespace blender::nodes::node_geo_attribute_transfer_cc {

enum class KernelType : int8_t {
  Sum = 0,
  Mean = 1,
  Gaussian = 2,
  Linear = 3,
  Smooth = 4,
  InverseDistance = 5,
  InverseDistanceSquared = 6,
  Cubic = 7,
  Quintic = 8,
  WendlandC2 = 9,
};

static const EnumPropertyItem kernel_type_items[] = {
    {int(KernelType::Sum),
     "SUM",
     0,
     N_("Sum"),
     N_("Sum of neighboring values inside Smooth Distance (weight = 1, not normalized)")},
    {int(KernelType::Mean),
     "MEAN",
     0,
     N_("Mean"),
     N_("Arithmetic mean of neighbors inside Smooth Distance (uniform weights)")},
    {0, "", 0, nullptr, nullptr},
    {int(KernelType::Gaussian),
     "GAUSSIAN",
     0,
     N_("Gaussian"),
     N_("exp(-4.5 (r / distance)^2). Weighted sum, falloff is kept")},
    {int(KernelType::Linear),
     "LINEAR",
     0,
     N_("Linear"),
     N_("Tent max(0, 1 - r / distance). Weighted sum, falloff is kept")},
    {int(KernelType::Smooth),
     "SMOOTH",
     0,
     N_("Smooth"),
     N_("Cubic Hermite 1 - 3t^2 + 2t^3. Weighted sum, falloff is kept")},
    {int(KernelType::InverseDistance),
     "INVERSE_DISTANCE",
     0,
     N_("Inverse Distance"),
     N_("1 / r inside the radius. Weighted sum, falloff is kept")},
    {int(KernelType::InverseDistanceSquared),
     "INVERSE_DISTANCE_SQUARED",
     0,
     N_("Inverse Distance Squared"),
     N_("1 / r^2 inside the radius. Weighted sum, falloff is kept")},
    {int(KernelType::Cubic),
     "CUBIC",
     0,
     N_("Cubic"),
     N_("Wyvill (1 - t^2)^3. Weighted sum, falloff is kept")},
    {int(KernelType::Quintic),
     "QUINTIC",
     0,
     N_("Quintic"),
     N_("(1 - t)^5. Weighted sum, falloff is kept")},
    {int(KernelType::WendlandC2),
     "WENDLAND_C2",
     0,
     N_("Wendland C2"),
     N_("Compact C2 Wendland. Weighted sum, falloff is kept")},
    {0, nullptr, 0, nullptr, nullptr},
};

/** Only Mean divides by neighbor count. All other kernels keep φ(r) in the output. */
static bool kernel_normalizes(const KernelType type)
{
  return type == KernelType::Mean;
}

/** Kernel weight φ(r) with compact support r <= h. h is Smooth Distance. */
static float kernel_weight(const KernelType type, const float dist, const float h)
{
  if (h <= 1.0e-12f) {
    return 1.0f;
  }
  const float t = dist / h;
  if (t > 1.0f) {
    return 0.0f;
  }
  switch (type) {
    case KernelType::Sum:
    case KernelType::Mean:
      return 1.0f;
    case KernelType::Gaussian:
      return math::exp(-4.5f * t * t);
    case KernelType::Linear:
      return 1.0f - t;
    case KernelType::Smooth:
      /* Cubic Hermite smoothstep: 1 - 3t^2 + 2t^3. */
      return 1.0f - t * t * (3.0f - 2.0f * t);
    case KernelType::InverseDistance:
      return 1.0f / math::max(dist, 1.0e-8f);
    case KernelType::InverseDistanceSquared:
      return 1.0f / math::max(dist * dist, 1.0e-16f);
    case KernelType::Cubic: {
      /* Wyvill: (1 - t^2)^3. */
      const float u = 1.0f - t * t;
      return u * u * u;
    }
    case KernelType::Quintic: {
      const float u = 1.0f - t;
      const float u2 = u * u;
      return u2 * u2 * u;
    }
    case KernelType::WendlandC2: {
      const float u = 1.0f - t;
      const float u2 = u * u;
      return u2 * u2 * (4.0f * t + 1.0f);
    }
  }
  return 0.0f;
}

static Span<float3> positions_from_geometry(const GeometrySet &geometry)
{
  if (const Mesh *mesh = geometry.get_mesh()) {
    if (mesh->verts_num > 0) {
      return mesh->vert_positions();
    }
  }
  if (const PointCloud *pointcloud = geometry.get_pointcloud()) {
    if (pointcloud->totpoint > 0) {
      return pointcloud->positions();
    }
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
/** \name Channel packing (accumulate as float, convert back)
 * \{ */

template<typename T> struct TransferChannels;

template<> struct TransferChannels<float> {
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

template<> struct TransferChannels<float2> {
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

template<> struct TransferChannels<float3> {
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

template<> struct TransferChannels<float4> {
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

template<> struct TransferChannels<ColorGeometry4f> {
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

template<> struct TransferChannels<ColorGeometry4b> {
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

template<> struct TransferChannels<int> {
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

template<> struct TransferChannels<bool> {
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

template<> struct TransferChannels<int8_t> {
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

template<> struct TransferChannels<int2> {
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

template<> struct TransferChannels<math::Quaternion> {
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

template<typename T>
constexpr bool is_transfer_type()
{
  return std::is_same_v<T, float> || std::is_same_v<T, float2> || std::is_same_v<T, float3> ||
         std::is_same_v<T, float4> || std::is_same_v<T, ColorGeometry4f> ||
         std::is_same_v<T, ColorGeometry4b> || std::is_same_v<T, int> || std::is_same_v<T, bool> ||
         std::is_same_v<T, int8_t> || std::is_same_v<T, int2> ||
         std::is_same_v<T, math::Quaternion>;
}

/** \} */

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();

  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_default_layout();

  b.add_input<decl::Geometry>("Geometry"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::PointCloud})
      .description("Target geometry whose point-domain field is sampled from");

  if (node != nullptr) {
    const eCustomDataType data_type = eCustomDataType(node->custom1);
    b.add_input(data_type, "Value"_ustr)
        .hide_value()
        .evaluated_geometry_field()
        .description("Point-domain field on the target geometry");
    b.add_output(data_type, "Value"_ustr)
        .structure_type(StructureType::Field)
        .propagate_references()
        .align_with_previous()
        .description(
            "Kernel-weighted sum of neighbors at the sample position. Distance falloff is kept");
  }

  b.add_input<decl::Vector>("Sample Position"_ustr)
      .default_input_type(NODE_DEFAULT_INPUT_POSITION_FIELD)
      .structure_type(StructureType::Dynamic)
      .description(
          "Positions where the field is sampled (defaults to the context Position)");

  {
    auto &panel = b.add_panel("Kernel"_ustr);
    panel.add_input<decl::Menu>("Kernel Type"_ustr)
        .static_items(kernel_type_items)
        .default_value(KernelType::Gaussian)
        .optional_label()
        .description(
            "How neighbors inside Smooth Distance are weighted. Weighted kernels are not "
            "normalized, so farther samples contribute a faded value");
    panel.add_input<decl::Float>("Smooth Distance"_ustr)
        .default_value(1.0f)
        .min(0.0f)
        .subtype(PROP_DISTANCE)
        .description(
            "Kernel support radius. Zero falls back to nearest neighbor. Larger values blend "
            "more neighbors");
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
      bNode &node = params.add_node("GeometryNodeAttributeTransfer"_ustr);
      node.custom1 = *type;
      params.update_and_connect_available_socket(node, "Value"_ustr);
    });
  }
}

class AttributeTransferFunction : public mf::MultiFunction {
 private:
  GeometrySet source_;
  GField value_field_;
  KernelType kernel_type_;
  float smooth_distance_;
  mf::Signature signature_;

  mutable CacheMutex mutex_;
  mutable KDTree<float3> *tree_ = nullptr;
  mutable GArray<> sample_values_;
  mutable bool prepared_ = false;
  mutable bool ok_ = false;

 public:
  AttributeTransferFunction(GeometrySet source,
                            GField value_field,
                            const KernelType kernel_type,
                            const float smooth_distance)
      : source_(std::move(source)),
        value_field_(std::move(value_field)),
        kernel_type_(kernel_type),
        smooth_distance_(smooth_distance)
  {
    source_.ensure_owns_direct_data();

    mf::SignatureBuilder builder{"Attribute Transfer", signature_};
    builder.single_input<float3>("Sample Position");
    builder.single_output("Value", value_field_.cpp_type());
    this->set_signature(&signature_);
  }

  ~AttributeTransferFunction() override
  {
    kdtree_free(tree_);
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

      const int n = positions.size();
      bke::GeometryFieldContext field_context(*component, bke::AttrDomain::Point);
      fn::FieldEvaluator evaluator(field_context, n);
      evaluator.add(value_field_);
      evaluator.evaluate();
      const GVArray &evaluated = evaluator.get_evaluated(0);

      const CPPType &type = value_field_.cpp_type();
      sample_values_ = GArray<>(type, n);
      evaluated.materialize(sample_values_.data());

      tree_ = kdtree_new<float3>(uint(n));
      for (const int i : positions.index_range()) {
        kdtree_insert<float3>(tree_, i, positions[i]);
      }
      kdtree_balance<float3>(tree_);
      ok_ = true;
    });
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float3> &query_positions = params.readonly_single_input<float3>(
        0, "Sample Position");
    GMutableSpan dst = params.uninitialized_single_output(1, "Value");
    const CPPType &type = dst.type();

    if (!ok_ || tree_ == nullptr || sample_values_.is_empty()) {
      type.value_initialize_indices(dst.data(), mask);
      return;
    }

    const float radius = math::max(smooth_distance_, 0.0f);
    const bool normalize = kernel_normalizes(kernel_type_);

    bke::attribute_math::to_static_type(type, [&]<typename T>() {
      if constexpr (is_transfer_type<T>()) {
        using Channels = TransferChannels<T>;
        const Span<T> values = sample_values_.as_span().typed<T>();
        MutableSpan<T> dst_typed = dst.typed<T>();
        constexpr int c = Channels::count;

        mask.foreach_index([&](const int i) {
          const float3 query = query_positions[i];

          /* Radius 0: Houdini-style nearest neighbor. */
          if (radius <= 0.0f) {
            KDTreeNearest<float3> nearest;
            const int index = kdtree_find_nearest<float3>(tree_, query, &nearest);
            if (index >= 0) {
              dst_typed[i] = values[index];
            }
            else {
              dst_typed[i] = T();
            }
            return;
          }

          float acc[4] = {};
          float wsum = 0.0f;

          kdtree_range_search_cb<float3>(
              tree_,
              query,
              radius,
              [&](const int index, const float3 & /*co*/, const float dist_sq) {
                const float dist = math::sqrt(dist_sq);
                const float w = kernel_weight(kernel_type_, dist, radius);
                if (w <= 0.0f) {
                  return true;
                }
                float ch[4];
                Channels::load(values[index], MutableSpan<float>(ch, c));
                for (int k = 0; k < c; k++) {
                  acc[k] += w * ch[k];
                }
                wsum += w;
                return true;
              });

          if (wsum <= 0.0f) {
            dst_typed[i] = T();
            return;
          }
          /* Mean only: divide by count (w=1 for every neighbor). Everything else is Σ φ v. */
          if (normalize) {
            const float inv = math::safe_rcp(wsum);
            for (int k = 0; k < c; k++) {
              acc[k] *= inv;
            }
          }
          dst_typed[i] = Channels::store(Span<float>(acc, c));
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
    hints.min_grain_size = 16;
    return hints;
  }

  void hash_unique(UniqueHashBytes &hash) const override
  {
    static constexpr int8_t id = 2;
    hash.add(&id);
    hash.add(kernel_type_);
    hash.add(smooth_distance_);
    hash.add(find_point_component(source_));
    fn::FieldHashDeep field_hash;
    hash.add(field_hash.ensure(value_field_));
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet source = params.extract_input<GeometrySet>("Geometry"_ustr);
  if (!find_point_component(source)) {
    if (!source.is_empty()) {
      params.error_message_add(NodeWarningType::Error,
                               TIP_("Geometry must contain a mesh or a point cloud"));
    }
    params.set_default_remaining_outputs();
    return;
  }

  GField value_field = params.extract_input<GField>("Value"_ustr);
  const KernelType kernel_type = params.extract_input<KernelType>("Kernel Type"_ustr);
  const float smooth_distance = params.extract_input<float>("Smooth Distance"_ustr);
  auto sample_position = params.extract_input<bke::SocketValueVariant>("Sample Position"_ustr);

  std::string error_message;
  bke::SocketValueVariant output_value;
  if (!execute_multi_function_on_value_variant(
          std::make_shared<AttributeTransferFunction>(
              std::move(source), std::move(value_field), kernel_type, smooth_distance),
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

  geo_node_type_base(&ntype, "GeometryNodeAttributeTransfer"_ustr, GEO_NODE_ATTRIBUTE_TRANSFER);
  ntype.ui_name = "Attribute Transfer";
  ntype.ui_description =
      "Sample a point-domain field by kernel-weighted sum of all neighbors inside Smooth "
      "Distance. Weighted kernels keep distance falloff (not averaged)";
  ntype.enum_name_legacy = "ATTRIBUTE_TRANSFER";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.default_width = bke::NodeWidth::_160;
  bke::node_register_type(ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_attribute_transfer_cc
