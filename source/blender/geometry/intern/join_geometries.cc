/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup geo
 */

#include "BLI_array_utils.hh"
#include "BLI_cpp_type.hh"
#include "BLI_utildefines.hh"

#include "GEO_join_geometries.hh"
#include "GEO_realize_instances.hh"

#include "BKE_attribute.hh"
#include "BKE_instances.hh"

#include <algorithm>
#include <cstring>

namespace blender::geometry {

using bke::AttributeDomainAndType;
using bke::GeometryComponent;
using bke::GeometrySet;

static GeometrySet::GatheredAttributes get_final_attribute_info(
    const Span<const GeometryComponent *> components, const Span<StringRef> ignored_attributes)
{
  GeometrySet::GatheredAttributes info;

  for (const GeometryComponent *component : components) {
    component->attributes()->foreach_attribute([&](const bke::AttributeIter &iter) {
      if (ignored_attributes.contains(iter.name)) {
        return;
      }
      info.add(iter.name, AttributeDomainAndType{iter.domain, iter.data_type});
    });
  }

  return info;
}

static void fill_new_attribute(const Span<const GeometryComponent *> src_components,
                               const StringRef name,
                               const bke::AttrType data_type,
                               const bke::AttrDomain domain,
                               GMutableSpan dst_span)
{
  const CPPType &cpp_type = bke::attribute_type_to_cpp_type(data_type);

  int offset = 0;
  for (const GeometryComponent *component : src_components) {
    const int domain_num = component->attribute_domain_size(domain);
    if (domain_num == 0) {
      continue;
    }
    GVArray read_attribute = *component->attributes()->lookup_or_default(
        name, domain, data_type, nullptr);

    GVArraySpan src_span{read_attribute};
    const void *src_buffer = src_span.data();
    void *dst_buffer = dst_span[offset];
    const int64_t n = std::min<int64_t>(src_span.size(), domain_num);
    if (n > 0 && src_buffer != nullptr) {
      cpp_type.copy_construct_n(src_buffer, dst_buffer, n);
    }
    if (n < domain_num) {
      cpp_type.fill_construct_n(cpp_type.default_value(),
                                POINTER_OFFSET(dst_buffer, cpp_type.size * n),
                                domain_num - n);
    }

    offset += domain_num;
  }
}

static bool copy_single_varray_value(const GVArray &src, const CPPType &type, void *r_value)
{
  if (!src || src.type() != type) {
    return false;
  }
  const CommonVArrayInfo info = src.common_info();
  if (info.type != CommonVArrayInfo::Type::Single || info.data == nullptr) {
    return false;
  }
  type.copy_construct(info.data, r_value);
  return true;
}

static bool try_join_single_value_attribute(const Span<const GeometryComponent *> src_components,
                                            const StringRef name,
                                            const bke::AttrDomain domain,
                                            const bke::AttrType data_type,
                                            bke::MutableAttributeAccessor dst_attributes)
{
  if (ELEM(data_type, bke::AttrType::String, bke::AttrType::WrangleArray)) {
    return false;
  }
  if (src_components.first()->attribute_domain_size(domain) == 0) {
    return false;
  }
  const CPPType &type = bke::attribute_type_to_cpp_type(data_type);
  BUFFER_FOR_CPP_TYPE_VALUE(type, first_buffer);
  {
    const GVArray src = *src_components.first()->attributes()->lookup_or_default(
        name, domain, data_type);
    if (!copy_single_varray_value(src, type, first_buffer)) {
      return false;
    }
  }
  const bool all_equal = threading::parallel_reduce(
      src_components.index_range().drop_front(1),
      64,
      true,
      [&](const IndexRange range, bool value) {
        if (!value) {
          return false;
        }
        BUFFER_FOR_CPP_TYPE_VALUE(type, other_buffer);
        for (const int i : range) {
          const GVArray src = *src_components[i]->attributes()->lookup_or_default(
              name, domain, data_type);
          if (!copy_single_varray_value(src, type, other_buffer)) {
            return false;
          }
          if (std::memcmp(other_buffer, first_buffer, size_t(type.size)) != 0) {
            return false;
          }
        }
        return true;
      },
      std::logical_and<bool>());
  if (!all_equal) {
    return false;
  }
  return dst_attributes.add(
      name, domain, data_type, bke::AttributeInitValue(GPointer(type, first_buffer)));
}

static void join_attributes(const Span<const GeometryComponent *> src_components,
                            GeometryComponent &result,
                            const Span<StringRef> ignored_attributes)
{
  const GeometrySet::GatheredAttributes info = get_final_attribute_info(src_components,
                                                                        ignored_attributes);
  bke::MutableAttributeAccessor dst_attributes = *result.attributes_for_write();

  for (const int i : info.names.index_range()) {
    const StringRef name = info.names[i];
    const AttributeDomainAndType &meta_data = info.kinds[i];

    if (try_join_single_value_attribute(
            src_components, name, meta_data.domain, meta_data.data_type, dst_attributes))
    {
      continue;
    }

    bke::GSpanAttributeWriter write_attribute = dst_attributes.lookup_or_add_for_write_only_span(
        name, meta_data.domain, meta_data.data_type);
    if (!write_attribute) {
      continue;
    }
    fill_new_attribute(
        src_components, name, meta_data.data_type, meta_data.domain, write_attribute.span);
    write_attribute.finish();
  }
}

static void join_instances(const Span<const GeometryComponent *> src_components,
                           const bool allow_merging_instance_references,
                           GeometrySet &result)
{
  Array<int> offsets_data(src_components.size() + 1);
  for (const int i : src_components.index_range()) {
    const auto &src_component = static_cast<const bke::InstancesComponent &>(*src_components[i]);
    offsets_data[i] = src_component.get()->instances_num();
  }
  const OffsetIndices offsets = offset_indices::accumulate_counts_to_offsets(offsets_data);

  auto dst_instances = std::make_unique<bke::Instances>(offsets.total_size());

  MutableSpan<int> all_handles = dst_instances->reference_handles_for_write();

  Map<std::reference_wrapper<const bke::InstanceReference>, int> new_handle_by_src_reference_cache;

  for (const int i : src_components.index_range()) {
    const auto &src_component = static_cast<const bke::InstancesComponent &>(*src_components[i]);
    const bke::Instances &src_instances = *src_component.get();

    const Span<bke::InstanceReference> src_references = src_instances.references();
    Array<int> handle_map(src_references.size());
    for (const int src_handle : src_references.index_range()) {
      const bke::InstanceReference &src_reference = src_references[src_handle];
      if (allow_merging_instance_references) {
        handle_map[src_handle] = new_handle_by_src_reference_cache.lookup_or_add_cb(
            src_reference, [&]() { return dst_instances->add_new_reference(src_reference); });
      }
      else {
        handle_map[src_handle] = dst_instances->add_new_reference(src_reference);
      }
    }

    const IndexRange dst_range = offsets[i];

    const Span<int> src_handles = src_instances.reference_handles();
    array_utils::gather(handle_map.as_span(), src_handles, all_handles.slice(dst_range));
  }

  result.replace_instances(dst_instances.release());
  auto &dst_component = result.get_component_for_write<bke::InstancesComponent>();
  join_attributes(src_components, dst_component, {".reference_index"});
}

static void join_volumes(const Span<const GeometryComponent *> /*src_components*/,
                         GeometrySet & /*result*/)
{
  /* Not yet supported. Joining volume grids with the same name requires resampling of at least one
   * of the grids. The cell size of the resulting volume has to be determined somehow. */
}

static void join_component_type(const bke::GeometryComponent::Type component_type,
                                const Span<GeometrySet> src_geometry_sets,
                                const bke::AttributeFilter &attribute_filter,
                                const bool allow_merging_instance_references,
                                GeometrySet &result)
{
  Vector<const GeometryComponent *> components;
  for (const GeometrySet &geometry_set : src_geometry_sets) {
    const GeometryComponent *component = geometry_set.get_component(component_type);
    if (component != nullptr && !component->is_empty()) {
      components.append(component);
    }
  }

  if (components.is_empty()) {
    return;
  }
  if (components.size() == 1) {
    result.add(*components.first());
    return;
  }

  switch (component_type) {
    case bke::GeometryComponent::Type::Instance:
      join_instances(components, allow_merging_instance_references, result);
      return;
    case bke::GeometryComponent::Type::Volume:
      join_volumes(components, result);
      return;
    default:
      break;
  }

  auto instances = std::make_unique<bke::Instances>(components.size());
  instances->transforms_for_write().fill(float4x4::identity());
  MutableSpan<int> handles = instances->reference_handles_for_write();
  Map<const GeometryComponent *, int> handle_by_component;
  for (const int i : components.index_range()) {
    const GeometryComponent *component = components[i];
    handles[i] = handle_by_component.lookup_or_add_cb(component, [&]() {
      GeometrySet tmp_geo;
      tmp_geo.add(*components[i]);
      return instances->add_new_reference(bke::InstanceReference{tmp_geo});
    });
  }

  RealizeInstancesOptions options;
  options.keep_original_ids = true;
  options.realize_instance_attributes = false;
  options.attribute_filter = attribute_filter;
  GeometrySet joined_components =
      realize_instances(GeometrySet::from_instances(std::move(instances)), options).geometry;
  result.add(joined_components.get_component_for_write(component_type));
}

GeometrySet join_geometries(
    const Span<GeometrySet> geometries,
    const bke::AttributeFilter &attribute_filter,
    const std::optional<Span<GeometryComponent::Type>> &component_types_to_join,
    const bool allow_merging_instance_references)
{
  GeometrySet result;
  result.set_name(geometries.is_empty() ? "" : geometries[0].name());
  for (const GeometrySet &geometry_set : geometries) {
    result.merge_bundle_from(geometry_set);
  }
  static const Array<GeometryComponent::Type> supported_types(
      {GeometryComponent::Type::Mesh,
       GeometryComponent::Type::PointCloud,
       GeometryComponent::Type::Instance,
       GeometryComponent::Type::Volume,
       GeometryComponent::Type::Curve,
       GeometryComponent::Type::GreasePencil,
       GeometryComponent::Type::Edit});

  const Span<GeometryComponent::Type> types_to_join = component_types_to_join.has_value() ?
                                                          *component_types_to_join :
                                                          Span<GeometryComponent::Type>(
                                                              supported_types);

  for (const GeometryComponent::Type type : types_to_join) {
    join_component_type(
        type, geometries, attribute_filter, allow_merging_instance_references, result);
  }

  return result;
}

}  // namespace blender::geometry
