/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Sibson natural-neighbor interpolation of a Mesh/PointCloud attribute on the
 * XY Delaunay of its vertices. Evaluated as a field at Sample Position
 * (defaults to the context Position).
 */

#include "BLI_generic_array.hh"
#include "BLI_math_matrix_types.hh"

#include "BKE_attribute_math.hh"
#include "BKE_geometry_fields.hh"

#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

#include "GEO_cgal.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_cgal_natural_neighbor_2_cc {

static eCustomDataType node_data_type(const bNode &node)
{
  const eCustomDataType data_type = eCustomDataType(node.custom1);
  if (ELEM(data_type, CD_PROP_FLOAT, CD_PROP_FLOAT3, CD_PROP_FLOAT4X4)) {
    return data_type;
  }
  return CD_PROP_FLOAT;
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

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();

  b.use_custom_socket_order();
  b.allow_any_socket_order();
  b.add_default_layout();

  b.add_input<decl::Geometry>("Mesh"_ustr)
      .only_realized_data()
      .supported_type({GeometryComponent::Type::Mesh, GeometryComponent::Type::PointCloud})
      .description("Target mesh or point cloud. Vertex values are interpolated on the XY Delaunay");

  blender::nodes::BaseSocketDeclarationBuilder *value_out = nullptr;
  if (node != nullptr) {
    const eCustomDataType data_type = node_data_type(*node);
    b.add_input(data_type, "Value"_ustr)
        .hide_value()
        .evaluated_geometry_field()
        .description("Attribute / field on the target Mesh points");
    value_out = &b.add_output(data_type, "Value"_ustr)
                     .align_with_previous()
                     .description("Sibson natural-neighbor interpolation at Sample Position");
  }

  auto &sample_position = b.add_input<decl::Vector>("Sample Position"_ustr)
                              .default_input_type(NODE_DEFAULT_INPUT_POSITION_FIELD)
                              .structure_type(StructureType::Dynamic)
                              .description(
                                  "Where to interpolate. Defaults to the context Position");
  std::array<int, 1> dynamic_inputs = {sample_position.index()};
  if (value_out != nullptr) {
    value_out->inferred_structure_type(dynamic_inputs).propagate_references(dynamic_inputs);
  }
  b.add_output<decl::Bool>("Is Valid"_ustr)
      .inferred_structure_type(dynamic_inputs)
      .propagate_references(dynamic_inputs)
      .description("True when the sample lies inside the XY convex hull of Mesh vertices");
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
  if (type && ELEM(*type, CD_PROP_FLOAT, CD_PROP_FLOAT3, CD_PROP_FLOAT4X4)) {
    params.add_item(IFACE_("Value"), [type](LinkSearchOpParams &params) {
      bNode &node = params.add_node("GeometryNodeCgalNaturalNeighbor2"_ustr);
      node.custom1 = *type;
      params.update_and_connect_available_socket(node, "Value"_ustr);
    });
  }
}

class NaturalNeighbor2Function : public mf::MultiFunction {
 private:
  GeometrySet source_;
  GField value_field_;
  mf::Signature signature_;

  mutable CacheMutex mutex_;
  mutable geometry::CgalNaturalNeighbor2 interpolator_;
  mutable GArray<> site_values_;
  mutable bool prepared_ = false;
  mutable bool ok_ = false;

 public:
  NaturalNeighbor2Function(GeometrySet source, GField value_field)
      : source_(std::move(source)), value_field_(std::move(value_field))
  {
    source_.ensure_owns_direct_data();

    mf::SignatureBuilder builder{"Natural Neighbor 2D", signature_};
    builder.single_input<float3>("Sample Position");
    builder.single_output("Value", value_field_.cpp_type());
    builder.single_output<bool>("Is Valid", mf::ParamFlag::SupportsUnusedOutput);
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
      if (positions.size() < 3) {
        return;
      }

      std::string error;
      if (!interpolator_.build(positions, error)) {
        return;
      }

      const int n = int(positions.size());
      bke::GeometryFieldContext field_context(*component, bke::AttrDomain::Point);
      fn::FieldEvaluator evaluator(field_context, n);
      evaluator.add(value_field_);
      evaluator.evaluate();
      const GVArray &evaluated = evaluator.get_evaluated(0);

      site_values_ = GArray<>(value_field_.cpp_type(), n);
      evaluated.materialize(site_values_.data());
      ok_ = true;
    });
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float3> &query_positions = params.readonly_single_input<float3>(
        0, "Sample Position");
    GMutableSpan dst = params.uninitialized_single_output(1, "Value");
    MutableSpan<bool> is_valid = params.uninitialized_single_output_if_required<bool>(2,
                                                                                      "Is Valid");
    const CPPType &type = dst.type();

    if (!ok_ || !interpolator_.is_valid()) {
      type.value_initialize_indices(dst.data(), mask);
      if (!is_valid.is_empty()) {
        index_mask::masked_fill(is_valid, false, mask);
      }
      return;
    }

    const int grain_n = int(mask.size());
    Array<float3> packed_query(grain_n);
    mask.foreach_index([&](const int i, const int pos) { packed_query[pos] = query_positions[i]; });

    std::vector<int> offsets;
    std::vector<int> indices;
    std::vector<float> weights;
    Array<bool> packed_valid(grain_n, false);
    interpolator_.query_many(packed_query, offsets, indices, weights, packed_valid.as_mutable_span());

    if (offsets.size() != size_t(grain_n) + 1) {
      type.value_initialize_indices(dst.data(), mask);
      if (!is_valid.is_empty()) {
        index_mask::masked_fill(is_valid, false, mask);
      }
      return;
    }

    if (!is_valid.is_empty()) {
      mask.foreach_index(
          [&](const int i, const int pos) { is_valid[i] = packed_valid[pos]; });
    }

    bool handled = false;
    bke::attribute_math::to_static_type(type, [&]<typename T>() {
      if constexpr (std::is_same_v<T, float> || std::is_same_v<T, float3> ||
                    std::is_same_v<T, float4x4>)
      {
        handled = true;
        const Span<T> src = site_values_.as_span().typed<T>();
        MutableSpan<T> dst_typed = dst.typed<T>();
        mask.foreach_index([&](const int i, const int pos) {
          const int start = offsets[size_t(pos)];
          const int count = offsets[size_t(pos) + 1] - start;
          if (count <= 0) {
            if constexpr (std::is_same_v<T, float4x4>) {
              dst_typed[i] = float4x4::zero();
            }
            else {
              dst_typed[i] = T();
            }
            return;
          }
          dst_typed[i] = bke::attribute_math::mix_indices(
              src, Span(indices.data() + start, count), Span(weights.data() + start, count));
        });
      }
    });
    if (!handled) {
      type.value_initialize_indices(dst.data(), mask);
    }
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    hints.min_grain_size = 32;
    return hints;
  }

  void hash_unique(UniqueHashBytes &hash) const override
  {
    static constexpr int8_t id = 3;
    hash.add(&id);
    hash.add(find_point_component(source_));
    fn::FieldHashDeep field_hash;
    hash.add(field_hash.ensure(value_field_));
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet source = params.extract_input<GeometrySet>("Mesh"_ustr);
  if (!find_point_component(source)) {
    if (!source.is_empty()) {
      params.error_message_add(NodeWarningType::Error,
                               TIP_("Mesh must contain a mesh or point cloud"));
    }
    params.set_default_remaining_outputs();
    return;
  }
  if (positions_from_geometry(source).size() < 3) {
    params.error_message_add(NodeWarningType::Error,
                             TIP_("Natural Neighbor 2D needs at least 3 vertices"));
    params.set_default_remaining_outputs();
    return;
  }

  GField value_field = params.extract_input<GField>("Value"_ustr);
  auto sample_position = params.extract_input<bke::SocketValueVariant>("Sample Position"_ustr);

  std::string error_message;
  bke::SocketValueVariant output_value;
  bke::SocketValueVariant is_valid;
  if (!execute_multi_function_on_value_variant(
          std::make_shared<NaturalNeighbor2Function>(std::move(source), std::move(value_field)),
          {&sample_position},
          {&output_value, &is_valid},
          params.user_data(),
          error_message))
  {
    params.set_default_remaining_outputs();
    params.error_message_add(NodeWarningType::Error, std::move(error_message));
    return;
  }

  params.set_output("Value"_ustr, std::move(output_value));
  params.set_output("Is Valid"_ustr, std::move(is_valid));
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
                    [](bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free)
                    {
                      *r_free = true;
                      return enum_items_filter(
                          rna_enum_attribute_type_items, [](const EnumPropertyItem &item) {
                            return ELEM(item.value,
                                        CD_PROP_FLOAT,
                                        CD_PROP_FLOAT3,
                                        CD_PROP_FLOAT4X4);
                          });
                    });
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, "GeometryNodeCgalNaturalNeighbor2"_ustr, GEO_NODE_CGAL_NATURAL_NEIGHBOR_2);
  ntype.ui_name = "Natural Neighbor 2D";
  ntype.ui_description =
      "Sibson natural-neighbor interpolation of a mesh attribute on the XY Delaunay of its "
      "vertices. Evaluated at the context Position (or Sample Position). Outside the hull "
      "Value is 0 and Is Valid is false";
  ntype.enum_name_legacy = "CGAL_NATURAL_NEIGHBOR_2";
  ntype.nclass = NODE_CLASS_GEOMETRY;
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.gather_link_search_ops = node_gather_link_searches;
  bke::node_register_type(ntype);
  if (ntype.rna_ext.srna) {
    node_rna(ntype.rna_ext.srna);
  }
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_cgal_natural_neighbor_2_cc
