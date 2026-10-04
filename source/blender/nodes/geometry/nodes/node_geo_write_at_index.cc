/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <algorithm>
#include <type_traits>

#include "BLI_array.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_virtual_array.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_enum_types.hh"

#include "UI_interface_layout.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_write_at_index_cc {

NODE_STORAGE_FUNCS(NodeGeometryWriteAtIndex)

enum class AtomicOperator : int8_t {
  Sum = 0,
  Min = 1,
  Max = 2,
  Mean = 3,
};

static void node_declare(NodeDeclarationBuilder &b)
{
  const bNode *node = b.node_or_null();
  if (node == nullptr) {
    return;
  }

  const eCustomDataType data_type = eCustomDataType(node_storage(*node).data_type);
  b.add_input<decl::Int>("Target Index"_ustr)
      .structure_type(StructureType::Field)
      .description(
          "Index on the target domain to write to. "
          "Evaluated on the source domain together with Value");
  b.add_input(data_type, "Value"_ustr)
      .structure_type(StructureType::Field)
      .description("Value written by every evaluated element on the source domain");
  b.add_output(data_type, "Value"_ustr)
      .structure_type(StructureType::Field)
      .propagate_references()
      .description("Values on the target domain after conflicting writes have been combined");
}

static void node_layout(ui::Layout &layout, bContext * /*C*/, PointerRNA *ptr)
{
  layout.use_property_split_set(true);
  layout.use_property_decorate_set(false);
  layout.prop(ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
  layout.prop(ptr, "source_domain", UI_ITEM_NONE, "", ICON_NONE);
  layout.prop(ptr, "target_domain", UI_ITEM_NONE, "", ICON_NONE);
  layout.prop(ptr, "atomic_operator", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryWriteAtIndex *data = MEM_new<NodeGeometryWriteAtIndex>(__func__);
  data->data_type = CD_PROP_FLOAT;
  data->source_domain = int8_t(AttrDomain::Point);
  data->target_domain = int8_t(AttrDomain::Point);
  data->atomic_operator = int8_t(AtomicOperator::Sum);
  node->storage = data;
}

static std::optional<eCustomDataType> supported_type_from_socket(const bNodeSocket &socket)
{
  switch (socket.type) {
    case SOCK_FLOAT:
      return CD_PROP_FLOAT;
    case SOCK_INT:
    case SOCK_BOOLEAN:
      return CD_PROP_INT32;
    case SOCK_VECTOR:
    case SOCK_RGBA:
    case SOCK_ROTATION:
      return CD_PROP_FLOAT3;
    default:
      return std::nullopt;
  }
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const std::optional<eCustomDataType> type = supported_type_from_socket(params.other_socket());
  if (!type) {
    return;
  }
  params.add_item(IFACE_("Value"), [type](LinkSearchOpParams &params) {
    bNode &node = params.add_node("GeometryNodeWriteAtIndex"_ustr);
    node_storage(node).data_type = *type;
    params.update_and_connect_available_socket(node, "Value"_ustr);
  });
}

template<typename T> static T component_min(const T &a, const T &b)
{
  if constexpr (std::is_same_v<T, float3>) {
    return float3(std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z));
  }
  else {
    return std::min(a, b);
  }
}

template<typename T> static T component_max(const T &a, const T &b)
{
  if constexpr (std::is_same_v<T, float3>) {
    return float3(std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z));
  }
  else {
    return std::max(a, b);
  }
}

template<typename T> static T divide_for_mean(const T &value, const int count)
{
  if constexpr (std::is_same_v<T, int>) {
    return value / count;
  }
  else {
    return value / float(count);
  }
}

class WriteAtIndexFieldInput final : public bke::GeometryFieldInput {
 private:
  GField value_;
  Field<int> target_index_;
  AttrDomain source_domain_;
  AttrDomain target_domain_;
  AtomicOperator operator_;

 public:
  WriteAtIndexFieldInput(GField value,
                         Field<int> target_index,
                         const AttrDomain source_domain,
                         const AttrDomain target_domain,
                         const AtomicOperator atomic_operator)
      : bke::GeometryFieldInput(value.cpp_type(), "Write at Index"),
        value_(std::move(value)),
        target_index_(std::move(target_index)),
        source_domain_(source_domain),
        target_domain_(target_domain),
        operator_(atomic_operator)
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask & /*mask*/) const final
  {
    const std::optional<AttributeAccessor> attributes = context.attributes();
    if (!attributes.has_value()) {
      return {};
    }

    const int64_t source_domain_size = attributes->domain_size(source_domain_);
    const int64_t target_domain_size = attributes->domain_size(target_domain_);

    if (source_domain_size == 0 || target_domain_size == 0) {
      return {};
    }

    /* Evaluate both fields on the source domain. */
    const bke::GeometryFieldContext source_context{context, source_domain_};
    fn::FieldEvaluator evaluator{source_context, source_domain_size};
    evaluator.add(target_index_);
    evaluator.add(value_);
    evaluator.evaluate();

    const VArray<int> target_indices = evaluator.get_evaluated<int>(0);
    const GVArray values = evaluator.get_evaluated(1);
    GVArray result;

    values.type().to_static_type<int, float, float3>([&]<typename T>() {
      const VArray<T> typed_values = values.typed<T>();
      /* Output buffer sized to the target domain. */
      Array<T> output(target_domain_size, T());
      Array<int> write_counts(target_domain_size, 0);

      /* Iterate over source-domain elements and scatter to target-domain indices. */
      for (const int64_t source_i : IndexRange(source_domain_size)) {
        const int target_i = target_indices[source_i];
        if (target_i < 0 || target_i >= target_domain_size) {
          continue;
        }

        const T value = typed_values[source_i];
        const int previous_count = write_counts[target_i]++;
        if (previous_count == 0) {
          output[target_i] = value;
          continue;
        }

        switch (operator_) {
          case AtomicOperator::Sum:
          case AtomicOperator::Mean:
            output[target_i] += value;
            break;
          case AtomicOperator::Min:
            output[target_i] = component_min(output[target_i], value);
            break;
          case AtomicOperator::Max:
            output[target_i] = component_max(output[target_i], value);
            break;
        }
      }

      if (operator_ == AtomicOperator::Mean) {
        for (const int64_t i : IndexRange(target_domain_size)) {
          if (write_counts[i] != 0) {
            output[i] = divide_for_mean(output[i], write_counts[i]);
          }
        }
      }

      result = VArray<T>::from_container(std::move(output));
    });

    /* Adapt from the target domain back to whatever the pipeline expects. */
    return attributes->adapt_domain(std::move(result), target_domain_, context.domain());
  }

  void foreach_recursive_field(FunctionRef<void(const GField &)> fn) const final
  {
    fn(value_);
    fn(target_index_);
  }

  void hash_unique(UniqueHashBytes &hash, fn::FieldHashDeep &deep_hash_cache) const final
  {
    static constexpr int8_t id = 0;
    hash.add(&id);
    hash.add(deep_hash_cache.ensure(value_));
    hash.add(deep_hash_cache.ensure(target_index_));
    hash.add(source_domain_);
    hash.add(target_domain_);
    hash.add(operator_);
  }

  std::optional<AttrDomain> preferred_domain(
      const GeometryComponent & /*component*/) const final
  {
    return target_domain_;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const NodeGeometryWriteAtIndex &storage = node_storage(params.node());
  GField value = params.extract_input<GField>("Value"_ustr);
  Field<int> target_index = params.extract_input<Field<int>>("Target Index"_ustr);
  params.set_output<GField>(
      "Value"_ustr,
      GField::from_input<WriteAtIndexFieldInput>(
          std::move(value),
          std::move(target_index),
          AttrDomain(storage.source_domain),
          AttrDomain(storage.target_domain),
          AtomicOperator(storage.atomic_operator)));
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem data_type_items[] = {
      {CD_PROP_FLOAT, "FLOAT", ICON_NODE_SOCKET_FLOAT, "Float", "Floating-point values"},
      {CD_PROP_INT32, "INT", ICON_NODE_SOCKET_INT, "Integer", "Integer values"},
      {CD_PROP_FLOAT3, "FLOAT_VECTOR", ICON_NODE_SOCKET_VECTOR, "Vector", "3D vector values"},
      {0, nullptr, 0, nullptr, nullptr},
  };
  static const EnumPropertyItem operator_items[] = {
      {int(AtomicOperator::Sum), "SUM", 0, "Sum", "Add all values targeting the same index"},
      {int(AtomicOperator::Min), "MIN", 0, "Minimum", "Use the component-wise minimum"},
      {int(AtomicOperator::Max), "MAX", 0, "Maximum", "Use the component-wise maximum"},
      {int(AtomicOperator::Mean),
       "MEAN",
       0,
       "Mean",
       "Average all values targeting the same index"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "Type of value to write",
                    data_type_items,
                    NOD_storage_enum_accessors(data_type),
                    CD_PROP_FLOAT);
  RNA_def_node_enum(srna,
                    "source_domain",
                    "Source Domain",
                    "Domain whose elements provide the Value and Target Index fields",
                    rna_enum_attribute_domain_items,
                    NOD_storage_enum_accessors(source_domain),
                    int(AttrDomain::Point));
  RNA_def_node_enum(srna,
                    "target_domain",
                    "Target Domain",
                    "Domain whose elements receive the indexed writes",
                    rna_enum_attribute_domain_items,
                    NOD_storage_enum_accessors(target_domain),
                    int(AttrDomain::Point));
  RNA_def_node_enum(srna,
                    "atomic_operator",
                    "Atomic Operator",
                    "Operation used when multiple elements write to the same index",
                    operator_items,
                    NOD_storage_enum_accessors(atomic_operator),
                    int(AtomicOperator::Sum));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(&ntype, "GeometryNodeWriteAtIndex"_ustr, GEO_NODE_WRITE_AT_INDEX);
  ntype.ui_name = "Write at Index";
  ntype.ui_description =
      "Scatter values from one domain to another by index, combining conflicting writes atomically";
  ntype.enum_name_legacy = "WRITE_AT_INDEX";
  ntype.nclass = NODE_CLASS_CONVERTER;
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.gather_link_search_ops = node_gather_link_searches;
  bke::node_type_storage(
      ntype, "NodeGeometryWriteAtIndex", node_free_standard_storage, node_copy_standard_storage);
  bke::node_register_type(ntype);
  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_write_at_index_cc
