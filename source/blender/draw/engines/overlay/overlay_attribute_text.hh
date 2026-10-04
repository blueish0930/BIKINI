/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw_engine
 */

#pragma once

#include <algorithm>
#include <string>

#include "BLI_array.hh"
#include "BLI_color.hh"
#include "BLI_index_mask.hh"
#include "BLI_kdopbvh.hh"
#include "BLI_map.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_string_utf8.hh"
#include "DNA_curve_types.h"
#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_node_types.h"
#include "DNA_grease_pencil_types.h"
#include "DNA_pointcloud_types.h"

#include "BKE_attribute.hh"
#include "BKE_wrangle_array.hh"
#include "BKE_bvhutils.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_mesh.hh"
#include "BKE_object_types.hh"

#include "DRW_render.hh"

#include "ED_view3d.hh"

#include "UI_interface_c.hh"
#include "UI_resources.hh"

#include "draw_manager_text.hh"

#include "overlay_attribute_debug_util.hh"
#include "overlay_base.hh"

namespace blender::draw::overlay {

/**
 * Displays geometry node viewer output.
 * Values are displayed as text on top of the active object.
 */
class AttributeTexts : Overlay {
 public:
  void begin_sync(Resources &res, const State &state) final
  {
    /* Always enabled outside selection so Geometry Nodes Debug text works without the Viewer
     * Attribute Text overlay toggle. Active Viewer text still uses the same path. */
    enabled_ = !res.is_selection();
    font_size_ = state.overlay.viewer_attribute_text_size > 0.0f ?
                     state.overlay.viewer_attribute_text_size :
                     11.0f;
  }

  void object_sync(Manager & /*manager*/,
                   const ObjectRef &ob_ref,
                   Resources & /*res*/,
                   const State &state) final
  {
    if (!enabled_) {
      return;
    }

    const Object &object = *ob_ref.object;
    DRWTextStore *dt = state.dt;
    const float4x4 object_to_world = ob_ref.is_range() ? ob_ref.object_to_world(0) :
                                                         ob_ref.object_to_world();
    const bool is_preview = ob_ref.preview_base_geometry() != nullptr;
    /* Official Viewer text: only on preview geometry + Attribute Text overlay. */
    const bool include_viewer = is_preview && state.show_attribute_viewer_text();

    /* Official: Viewer instance-domain text. */
    if (include_viewer && ob_ref.preview_instance_index() >= 0) {
      if (const bke::Instances *instances = ob_ref.preview_base_geometry()->get_instances()) {
        if (instances->attributes().contains(".viewer")) {
          add_instance_attributes_to_text_cache(
              dt, instances->attributes(), object_to_world, ob_ref.preview_instance_index());
          return;
        }
      }
    }

    /* Debug Instance domain: read from geometry-set eval on the parent, or from each
     * dupli's instance stack (non-preview path). */
    if (object.runtime->geometry_set_eval != nullptr) {
      if (const bke::Instances *instances = object.runtime->geometry_set_eval->get_instances()) {
        if (debug_attr::instances_have_debug_layers(*instances)) {
          add_instances_debug_text(
              state, dt, *instances, object.object_to_world());
        }
      }
    }
    {
      const int count = ob_ref.instances_count();
      for (int i = 0; i < count; i++) {
        const DupliObject *dupli = ob_ref.dupli_object(i);
        if (dupli == nullptr) {
          continue;
        }
        for (const int s : IndexRange(ARRAY_SIZE(dupli->instance_data))) {
          if (dupli->instance_data[s] == nullptr) {
            continue;
          }
          const bke::Instances *instances = dupli->instance_data[s]->get_instances();
          if (instances == nullptr || !debug_attr::instances_have_debug_layers(*instances)) {
            continue;
          }
          /* Only the single instance index for this dupli — avoid N^2 when parent also draws. */
          if (object.runtime->geometry_set_eval == nullptr ||
              object.runtime->geometry_set_eval->get_instances() != instances)
          {
            add_instance_attributes_to_text_cache(dt,
                                                  instances->attributes(),
                                                  ob_ref.object_to_world(i),
                                                  dupli->instance_idx[s]);
          }
        }
      }
    }

    /* Geometry path: Mesh/PointCloud/Curves/GreasePencil.
     * `.viewer` only when include_viewer (preview + Attribute Text).
     * `.debug_*_value` always (Debug node text). */
    switch (object.type) {
      case OB_MESH: {
        const Mesh &mesh = DRW_object_get_data_for_drawing<Mesh>(object);
        add_mesh_attributes_to_text_cache(state, mesh, object_to_world, include_viewer);
        break;
      }
      case OB_POINTCLOUD: {
        const PointCloud &pointcloud = DRW_object_get_data_for_drawing<PointCloud>(object);
        add_attributes_to_text_cache(
            state, dt, pointcloud.attributes(), object_to_world, include_viewer, nullptr);
        break;
      }
      case OB_CURVES_LEGACY: {
        const Curve &curve = DRW_object_get_data_for_drawing<Curve>(object);
        if (curve.curve_eval) {
          const bke::CurvesGeometry &curves = curve.curve_eval->geometry.wrap();
          add_attributes_to_text_cache(
              state, dt, curves.attributes(), object_to_world, include_viewer, &curves);
        }
        break;
      }
      case OB_CURVES: {
        const Curves &curves_id = DRW_object_get_data_for_drawing<Curves>(object);
        const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
        add_attributes_to_text_cache(
            state, dt, curves.attributes(), object_to_world, include_viewer, &curves);
        break;
      }
      case OB_GREASE_PENCIL: {
        const GreasePencil &gp = DRW_object_get_data_for_drawing<GreasePencil>(object);
        add_grease_pencil_debug_text(state, dt, gp, object, object_to_world);
        break;
      }
      default:
        break;
    }
  }

 private:
  float font_size_ = 11.0f;

  static Vector<std::string> collect_text_attribute_names(
      const bke::AttributeAccessor &attributes, const bool include_viewer)
  {
    Vector<std::string> names;
    attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
      if (iter.name.startswith(".debug_") && iter.name.endswith("_value")) {
        names.append(iter.name);
      }
    });
    std::sort(names.begin(), names.end());
    if (include_viewer && attributes.contains(".viewer")) {
      names.append(".viewer");
    }
    return names;
  }

  static std::string layer_prefix_from_value_name(const StringRef name)
  {
    if (name.startswith(".debug_") && name.endswith("_value")) {
      return std::string(name.drop_suffix(6));
    }
    return "";
  }

  static int text_lines_for_type(const CPPType &type, const int display_mode)
  {
    if (type.is<int2>() || type.is<float2>()) {
      return 2;
    }
    if (type.is<float3>()) {
      /* Arrows mode: no XYZ text lines, only viewport arrows. */
      if (display_mode == NODE_GEO_DEBUG_VECTOR_ARROWS) {
        return 0;
      }
      return 3;
    }
    if (type.is<float4x4>()) {
      if (display_mode == NODE_GEO_DEBUG_MATRIX_AXES) {
        return 0;
      }
      if (display_mode == NODE_GEO_DEBUG_MATRIX_VALUES) {
        return 4;
      }
      return 3;
    }
    if (type.is<ColorGeometry4b>() || type.is<ColorGeometry4f>() ||
        type.is<math::Quaternion>())
    {
      return 4;
    }
    return 1;
  }

  static int layer_matrix_display(const bke::AttributeAccessor &attributes,
                                  const StringRef name,
                                  const bke::AttrDomain domain)
  {
    const std::string prefix = layer_prefix_from_value_name(name);
    if (!prefix.empty()) {
      if (const VArray<int> display = *attributes.lookup<int>(prefix + "_display", domain)) {
        if (!display.is_empty()) {
          return display[0];
        }
      }
    }
    return NODE_GEO_DEBUG_MATRIX_COMPONENTS;
  }

  static VArray<bool> layer_validity(const bke::AttributeAccessor &attributes,
                                     const StringRef name,
                                     const bke::AttrDomain domain)
  {
    const std::string prefix = layer_prefix_from_value_name(name);
    if (!prefix.empty()) {
      if (const VArray<bool> valid = *attributes.lookup<bool>(prefix + "_valid", domain)) {
        return valid;
      }
    }
    return VArray<bool>::from_single(true, attributes.domain_size(domain));
  }

  /** Debug node Text Overlay toggle writes `{prefix}_use_text`. Missing => true (old layers). */
  static bool layer_use_text(const bke::AttributeAccessor &attributes,
                             const StringRef name,
                             const bke::AttrDomain domain)
  {
    const std::string prefix = layer_prefix_from_value_name(name);
    if (prefix.empty()) {
      /* Official Viewer `.viewer` — always drawn when included. */
      return true;
    }
    if (const VArray<bool> flags = *attributes.lookup<bool>(prefix + "_use_text", domain)) {
      if (!flags.is_empty()) {
        return flags[0];
      }
    }
    return true;
  }

  static int64_t first_valid_index(const VArray<bool> &valid)
  {
    for (const int64_t i : IndexRange(valid.size())) {
      if (valid[i]) {
        return i;
      }
    }
    return 0;
  }

  static float local_font_size_for_name(const bke::AttributeAccessor &attributes,
                                        const StringRef name,
                                        const bke::AttrDomain domain,
                                        const VArray<bool> &valid,
                                        const float fallback)
  {
    const std::string prefix = layer_prefix_from_value_name(name);
    if (!prefix.empty()) {
      if (const VArray<float> sizes = *attributes.lookup<float>(prefix + "_text_size", domain)) {
        if (!sizes.is_empty()) {
          return math::clamp(sizes[first_valid_index(valid)], 1.0f, 64.0f);
        }
      }
    }
    if (const VArray<float> sizes = *attributes.lookup<float>(".viewer_text_size", domain)) {
      if (!sizes.is_empty()) {
        return math::clamp(sizes[0], 1.0f, 64.0f);
      }
    }
    return fallback;
  }

  static bool use_exclude_backfaces_for_name(const bke::AttributeAccessor &attributes,
                                             const StringRef name,
                                             const bke::AttrDomain domain,
                                             const VArray<bool> &valid,
                                             const State &state)
  {
    if (state.use_viewer_attribute_text_backface_culling()) {
      return true;
    }
    const std::string prefix = layer_prefix_from_value_name(name);
    if (!prefix.empty()) {
      if (const VArray<bool> flags = *attributes.lookup<bool>(
              prefix + "_exclude_backfaces", domain))
      {
        if (!flags.is_empty()) {
          return flags[first_valid_index(valid)];
        }
      }
    }
    if (const VArray<bool> flags = *attributes.lookup<bool>(".viewer_exclude_backfaces", domain)) {
      if (!flags.is_empty()) {
        return flags[0];
      }
    }
    return false;
  }

  static Array<bool> combine_visibility(const VArray<bool> &valid,
                                        const Span<bool> geometric_visibility)
  {
    Array<bool> visibility(valid.size());
    for (const int64_t i : visibility.index_range()) {
      visibility[i] = valid[i] &&
                      (geometric_visibility.is_empty() || geometric_visibility[i]);
    }
    return visibility;
  }

  void add_attributes_to_text_cache(const State &state,
                                    DRWTextStore *dt,
                                    bke::AttributeAccessor attributes,
                                    const float4x4 &object_to_world,
                                    const bool include_viewer,
                                    const bke::CurvesGeometry *curves_for_spline)
  {
    /* Official Viewer text: include `.viewer` on preview + Attribute Text for Mesh AND PointCloud/
     * Curves. (Older code passed false for non-mesh -> Points never showed Viewer numbers.) */
    const Vector<std::string> names = collect_text_attribute_names(attributes, include_viewer);
    Map<bke::AttrDomain, Array<float>> accumulated_offsets;
    for (const std::string &name : names) {
      const bke::GAttributeReader attribute = attributes.lookup(name);
      if (!attribute) {
        continue;
      }
      const bke::AttrDomain domain = attribute.domain;
      if (!layer_use_text(attributes, name, domain)) {
        continue;
      }
      const int display_mode = layer_matrix_display(attributes, name, domain);
      /* Arrow-only modes: skip text entirely (vector Arrows / matrix Axes). */
      if (text_lines_for_type(attribute.varray.type(), display_mode) == 0) {
        continue;
      }
      const VArray<bool> valid = layer_validity(attributes, name, domain);

      Array<float3> positions_storage;
      if (domain == bke::AttrDomain::Curve && curves_for_spline != nullptr) {
        positions_storage = debug_attr::curve_domain_positions(*curves_for_spline);
      }
      else {
        positions_storage = debug_attr::positions_for_domain(attributes, domain);
      }
      if (positions_storage.is_empty()) {
        continue;
      }
      const Span<float3> positions = positions_storage;

      const float size = local_font_size_for_name(attributes, name, domain, valid, font_size_);
      const float line_height = size * 1.1f * UI_SCALE_FAC;
      Array<float> &offsets = accumulated_offsets.lookup_or_add_cb(
          domain, [&]() { return Array<float>(positions.size(), 0.0f); });
      Array<float> y_offsets(positions.size());
      for (const int64_t i : positions.index_range()) {
        y_offsets[i] = -offsets[i];
        if (i < valid.size() && valid[i]) {
          offsets[i] += text_lines_for_type(attribute.varray.type(), display_mode) * line_height;
        }
      }
      const Array<bool> visibility = combine_visibility(valid, {});
      add_values_to_text_cache(dt,
                               attribute.varray,
                               positions,
                               object_to_world,
                               visibility,
                               size,
                               y_offsets,
                               display_mode);
    }
  }

  void add_instances_debug_text(const State & /*state*/,
                                DRWTextStore *dt,
                                const bke::Instances &instances,
                                const float4x4 &parent_to_world)
  {
    const bke::AttributeAccessor attributes = instances.attributes();
    const Array<float3> positions = debug_attr::instance_domain_positions(instances);
    if (positions.is_empty()) {
      return;
    }
    const Vector<std::string> names = collect_text_attribute_names(attributes, false);
    Map<bke::AttrDomain, Array<float>> accumulated_offsets;
    for (const std::string &name : names) {
      const bke::GAttributeReader attribute = attributes.lookup(name);
      if (!attribute) {
        continue;
      }
      const bke::AttrDomain domain = bke::AttrDomain::Instance;
      if (!layer_use_text(attributes, name, domain)) {
        continue;
      }
      const int display_mode = layer_matrix_display(attributes, name, domain);
      if (text_lines_for_type(attribute.varray.type(), display_mode) == 0) {
        continue;
      }
      const VArray<bool> valid = layer_validity(attributes, name, domain);
      const float size = local_font_size_for_name(attributes, name, domain, valid, font_size_);
      const float line_height = size * 1.1f * UI_SCALE_FAC;
      Array<float> &offsets = accumulated_offsets.lookup_or_add_cb(
          domain, [&]() { return Array<float>(positions.size(), 0.0f); });
      Array<float> y_offsets(positions.size());
      for (const int64_t i : positions.index_range()) {
        y_offsets[i] = -offsets[i];
        if (i < valid.size() && valid[i]) {
          offsets[i] += text_lines_for_type(attribute.varray.type(), display_mode) * line_height;
        }
      }
      const Array<bool> visibility = combine_visibility(valid, {});
      add_values_to_text_cache(dt,
                               attribute.varray,
                               positions,
                               parent_to_world,
                               visibility,
                               size,
                               y_offsets,
                               display_mode);
    }
  }

  void add_grease_pencil_debug_text(const State & /*state*/,
                                    DRWTextStore *dt,
                                    const GreasePencil &grease_pencil,
                                    const Object &object,
                                    const float4x4 &object_to_world)
  {
    const bke::AttributeAccessor attributes = grease_pencil.attributes();
    const Array<float3> positions = debug_attr::layer_domain_positions(grease_pencil, object);
    if (positions.is_empty()) {
      return;
    }
    const Vector<std::string> names = collect_text_attribute_names(attributes, false);
    Map<bke::AttrDomain, Array<float>> accumulated_offsets;
    for (const std::string &name : names) {
      const bke::GAttributeReader attribute = attributes.lookup(name);
      if (!attribute) {
        continue;
      }
      const bke::AttrDomain domain = attribute.domain;
      if (domain != bke::AttrDomain::Layer) {
        continue;
      }
      if (!layer_use_text(attributes, name, domain)) {
        continue;
      }
      const int display_mode = layer_matrix_display(attributes, name, domain);
      if (text_lines_for_type(attribute.varray.type(), display_mode) == 0) {
        continue;
      }
      const VArray<bool> valid = layer_validity(attributes, name, domain);
      const float size = local_font_size_for_name(attributes, name, domain, valid, font_size_);
      const float line_height = size * 1.1f * UI_SCALE_FAC;
      Array<float> &offsets = accumulated_offsets.lookup_or_add_cb(
          domain, [&]() { return Array<float>(positions.size(), 0.0f); });
      Array<float> y_offsets(positions.size());
      for (const int64_t i : positions.index_range()) {
        y_offsets[i] = -offsets[i];
        if (i < valid.size() && valid[i]) {
          offsets[i] += text_lines_for_type(attribute.varray.type(), display_mode) * line_height;
        }
      }
      const Array<bool> visibility = combine_visibility(valid, {});
      add_values_to_text_cache(dt,
                               attribute.varray,
                               positions,
                               object_to_world,
                               visibility,
                               size,
                               y_offsets,
                               display_mode);
    }
  }

  void add_mesh_attributes_to_text_cache(const State &state,
                                         const Mesh &mesh,
                                         const float4x4 &object_to_world,
                                         const bool include_viewer)
  {
    const bke::AttributeAccessor attributes = mesh.attributes();
    const Vector<std::string> names = collect_text_attribute_names(attributes, include_viewer);
    Map<bke::AttrDomain, Array<float>> accumulated_offsets;
    for (const std::string &name : names) {
      const bke::GAttributeReader attribute = attributes.lookup(name);
      if (!attribute) {
        continue;
      }
      const bke::AttrDomain domain = attribute.domain;
      if (!layer_use_text(attributes, name, domain)) {
        continue;
      }
      const int display_mode = layer_matrix_display(attributes, name, domain);
      if (text_lines_for_type(attribute.varray.type(), display_mode) == 0) {
        continue;
      }
      const VArray<bool> valid = layer_validity(attributes, name, domain);
      const VArraySpan<float3> positions = *attributes.lookup<float3>("position", domain);
      const float size = local_font_size_for_name(attributes, name, domain, valid, font_size_);
      const float line_height = size * 1.1f * UI_SCALE_FAC;

      Array<bool> geometric_visibility;
      if (use_exclude_backfaces_for_name(attributes, name, domain, valid, state)) {
        geometric_visibility = calculate_front_facing(
            state, mesh, positions, domain, object_to_world);
      }
      const Array<bool> visibility = combine_visibility(valid, geometric_visibility);

      Array<float> &offsets = accumulated_offsets.lookup_or_add_cb(
          domain, [&]() { return Array<float>(positions.size(), 0.0f); });
      Array<float> y_offsets(positions.size());
      for (const int64_t i : positions.index_range()) {
        y_offsets[i] = -offsets[i];
        if (visibility[i]) {
          offsets[i] += text_lines_for_type(attribute.varray.type(), display_mode) * line_height;
        }
      }

      if (domain == bke::AttrDomain::Corner) {
        const CPPType &type = attribute.varray.type();
        float offset_by_type = 1.0f;
        if (type.is<int2>() || type.is<float2>() || type.is<float3>() ||
            type.is<ColorGeometry4b>() || type.is<ColorGeometry4f>() || type.is<math::Quaternion>())
        {
          offset_by_type = 1.5f;
        }
        else if (type.is<float4x4>()) {
          offset_by_type = 3.0f;
        }

        Array<float3> corner_positions(positions.size());
        const Span<float3> vert_positions = mesh.vert_positions();
        const OffsetIndices<int> faces = mesh.faces();
        const Span<int> corner_verts = mesh.corner_verts();
        const Span<float3> face_normals = mesh.face_normals();
        threading::parallel_for(faces.index_range(), 512, [&](const IndexRange range) {
          for (const int face_index : range) {
            const float3 &face_normal = face_normals[face_index];
            const IndexRange face = faces[face_index];
            for (const int corner : face) {
              const int corner_prev = bke::mesh::face_corner_prev(face, corner);
              const int corner_next = bke::mesh::face_corner_next(face, corner);
              corner_positions[corner] = calc_corner_text_position(
                  vert_positions[corner_verts[corner]],
                  vert_positions[corner_verts[corner_prev]],
                  vert_positions[corner_verts[corner_next]],
                  face_normal,
                  state.rv3d,
                  object_to_world,
                  offset_by_type);
            }
          }
        });
        add_values_to_text_cache(state.dt,
                                 attribute.varray,
                                 corner_positions.as_span(),
                                 object_to_world,
                                 visibility,
                                 size,
                                 y_offsets,
                                 display_mode);
      }
      else {
        add_values_to_text_cache(state.dt,
                                 attribute.varray,
                                 positions,
                                 object_to_world,
                                 visibility,
                                 size,
                                 y_offsets,
                                 display_mode);
      }
    }
  }

  void add_instance_attributes_to_text_cache(DRWTextStore *dt,
                                             bke::AttributeAccessor attributes,
                                             const float4x4 &object_to_world,
                                             int instance_index)
  {
    const Vector<std::string> names = collect_text_attribute_names(attributes, true);
    int row = 0;
    for (const std::string &name : names) {
      const bke::GAttributeReader attribute = attributes.lookup(name);
      if (!attribute) {
        continue;
      }
      if (!layer_use_text(attributes, name, bke::AttrDomain::Instance)) {
        continue;
      }
      const int display_mode = layer_matrix_display(
          attributes, name, bke::AttrDomain::Instance);
      if (text_lines_for_type(attribute.varray.type(), display_mode) == 0) {
        continue;
      }
      const VArray<bool> valid = layer_validity(attributes, name, bke::AttrDomain::Instance);
      if (!valid[instance_index]) {
        continue;
      }
      const GVArray values = attribute.varray.slice(IndexRange(instance_index, 1));
      const float size = local_font_size_for_name(
          attributes, name, bke::AttrDomain::Instance, valid, font_size_);
      const float y_offset = -row * size * 1.1f * UI_SCALE_FAC;
      const std::array<float, 1> offsets = {y_offset};
      add_values_to_text_cache(
          dt, values, {float3(0, 0, 0)}, object_to_world, {}, size, offsets, display_mode);
      row += text_lines_for_type(attribute.varray.type(), display_mode);
    }
  }

  void add_text_to_cache(DRWTextStore *dt,
                         const float3 &position,
                         const StringRef text,
                         const uchar4 &color,
                         const float font_size,
                         const float y_offset = 0.0f) const
  {
    DRW_text_cache_add(dt,
                       position,
                       text.data(),
                       text.size(),
                       0,
                       short(y_offset),
                       DRW_TEXT_CACHE_GLOBALSPACE,
                       color,
                       true,
                       true,
                       font_size);
  }

  void add_lines_to_cache(DRWTextStore *dt,
                          const float3 &position,
                          const Span<StringRef> lines,
                          const uchar4 &color,
                          const float font_size,
                          const float base_y_offset = 0.0f) const
  {
    const float line_height = font_size * 1.1f * UI_SCALE_FAC;
    const float center_offset = (lines.size() - 1) / 2.0f;
    for (const int i : lines.index_range()) {
      const StringRef line = lines[i];
      DRW_text_cache_add(dt,
                         position,
                         line.data(),
                         line.size(),
                         0,
                         short(base_y_offset + (center_offset - i) * line_height),
                         DRW_TEXT_CACHE_GLOBALSPACE,
                         color,
                         true,
                         true,
                         font_size);
    }
  }

  void add_values_to_text_cache(DRWTextStore *dt,
                                const GVArray &values,
                                const Span<float3> positions,
                                const float4x4 &object_to_world,
                                const Span<bool> front_facing = {},
                                const float font_size = 0.0f,
                                const Span<float> y_offsets = {},
                                const int matrix_display = NODE_GEO_DEBUG_MATRIX_COMPONENTS)
  {
    const float used_font_size = font_size > 0.0f ? font_size : font_size_;

    uchar col[4];
    ui::theme::get_color_4ubv(TH_TEXT_HI, col);

    if (values.type().is<bke::WrangleArrayValue>()) {
      const VArray<bke::WrangleArrayValue> values_typed = values.typed<bke::WrangleArrayValue>();
      const int64_t values_num = std::min<int64_t>(values.size(), positions.size());
      for (const int64_t i : IndexRange(values_num)) {
        if (!front_facing.is_empty() && !front_facing[i]) {
          continue;
        }
        const float3 position = math::transform_point(object_to_world, positions[i]);
        const std::string text = values_typed[i].to_string();
        add_text_to_cache(dt,
                          position,
                          text,
                          col,
                          used_font_size,
                          (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
      }
      return;
    }

    if (values.type().is<MStringProperty>()) {
      const VArray<MStringProperty> values_typed = values.typed<MStringProperty>();
      const int64_t values_num = std::min<int64_t>(values.size(), positions.size());
      for (const int64_t i : IndexRange(values_num)) {
        if (!front_facing.is_empty() && !front_facing[i]) {
          continue;
        }
        const float3 position = math::transform_point(object_to_world, positions[i]);
        const MStringProperty &value = values_typed[i];
        const int length = std::min<int>(uint8_t(value.s_len), sizeof(value.s));
        add_text_to_cache(dt, position, StringRef(value.s, length), col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
      }
      return;
    }

    bke::attribute_math::to_static_type(values.type(), [&]<typename T>() {
      const VArray<T> &values_typed = values.typed<T>();
      const int64_t values_num = std::min<int64_t>(values.size(), positions.size());
      for (const int64_t i : IndexRange(values_num)) {
        if (!front_facing.is_empty() && !front_facing[i]) {
          continue;
        }
        const float3 position = math::transform_point(object_to_world, positions[i]);
        const T &value = values_typed[i];

        if constexpr (std::is_same_v<T, bool>) {
          char numstr[64];
          const size_t numstr_len = STRNCPY_UTF8_RLEN(numstr, value ? "True" : "False");
          ui::theme::get_color_blend_3ubv(TH_TEXT_HI, value ? TH_SUCCESS : TH_ERROR, 0.5f, col);
          add_text_to_cache(dt, position, StringRef(numstr, numstr_len), col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
        }
        else if constexpr (std::is_same_v<T, int8_t>) {
          char numstr[64];
          const size_t numstr_len = SNPRINTF_UTF8_RLEN(numstr, "%d", int(value));
          add_text_to_cache(dt, position, StringRef(numstr, numstr_len), col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
        }
        else if constexpr (std::is_same_v<T, int>) {
          char numstr[64];
          const size_t numstr_len = SNPRINTF_UTF8_RLEN(numstr, "%d", value);
          add_text_to_cache(dt, position, StringRef(numstr, numstr_len), col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
        }
        else if constexpr (std::is_same_v<T, int2>) {
          char x_str[64], y_str[64];
          const size_t x_str_len = SNPRINTF_UTF8_RLEN(x_str, "X: %d", value.x);
          const size_t y_str_len = SNPRINTF_UTF8_RLEN(y_str, "Y: %d", value.y);
          add_lines_to_cache(
              dt, position, {StringRef(x_str, x_str_len), StringRef(y_str, y_str_len)}, col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
        }
        else if constexpr (std::is_same_v<T, float>) {
          char numstr[64];
          const size_t numstr_len = SNPRINTF_UTF8_RLEN(numstr, "%g", value);
          add_text_to_cache(dt, position, StringRef(numstr, numstr_len), col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
        }
        else if constexpr (std::is_same_v<T, float2>) {
          char x_str[64], y_str[64];
          const size_t x_str_len = SNPRINTF_UTF8_RLEN(x_str, "X: %g", value.x);
          const size_t y_str_len = SNPRINTF_UTF8_RLEN(y_str, "Y: %g", value.y);
          add_lines_to_cache(
              dt, position, {StringRef(x_str, x_str_len), StringRef(y_str, y_str_len)}, col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
        }
        else if constexpr (std::is_same_v<T, float3>) {
          char x_str[64], y_str[64], z_str[64];
          const size_t x_str_len = SNPRINTF_UTF8_RLEN(x_str, "X: %g", value.x);
          const size_t y_str_len = SNPRINTF_UTF8_RLEN(y_str, "Y: %g", value.y);
          const size_t z_str_len = SNPRINTF_UTF8_RLEN(z_str, "Z: %g", value.z);
          add_lines_to_cache(dt,
                             position,
                             {StringRef(x_str, x_str_len),
                              StringRef(y_str, y_str_len),
                              StringRef(z_str, z_str_len)}, col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
        }
        else if constexpr (std::is_same_v<T, ColorGeometry4b>) {
          const ColorGeometry4f color = color::decode(value);
          char r_str[64], g_str[64], b_str[64], a_str[64];
          const size_t r_str_len = SNPRINTF_UTF8_RLEN(r_str, "R: %.3f", color.r);
          const size_t g_str_len = SNPRINTF_UTF8_RLEN(g_str, "G: %.3f", color.g);
          const size_t b_str_len = SNPRINTF_UTF8_RLEN(b_str, "B: %.3f", color.b);
          const size_t a_str_len = SNPRINTF_UTF8_RLEN(a_str, "A: %.3f", color.a);
          add_lines_to_cache(dt,
                             position,
                             {StringRef(r_str, r_str_len),
                              StringRef(g_str, g_str_len),
                              StringRef(b_str, b_str_len),
                              StringRef(a_str, a_str_len)}, col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
        }
        else if constexpr (std::is_same_v<T, ColorGeometry4f>) {
          char r_str[64], g_str[64], b_str[64], a_str[64];
          const size_t r_str_len = SNPRINTF_UTF8_RLEN(r_str, "R: %.3f", value.r);
          const size_t g_str_len = SNPRINTF_UTF8_RLEN(g_str, "G: %.3f", value.g);
          const size_t b_str_len = SNPRINTF_UTF8_RLEN(b_str, "B: %.3f", value.b);
          const size_t a_str_len = SNPRINTF_UTF8_RLEN(a_str, "A: %.3f", value.a);
          add_lines_to_cache(dt,
                             position,
                             {StringRef(r_str, r_str_len),
                              StringRef(g_str, g_str_len),
                              StringRef(b_str, b_str_len),
                              StringRef(a_str, a_str_len)}, col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
        }
        else if constexpr (std::is_same_v<T, math::Quaternion>) {
          char w_str[64], x_str[64], y_str[64], z_str[64];
          const size_t w_str_len = SNPRINTF_UTF8_RLEN(w_str, "W: %.3f", value.w);
          const size_t x_str_len = SNPRINTF_UTF8_RLEN(x_str, "X: %.3f", value.x);
          const size_t y_str_len = SNPRINTF_UTF8_RLEN(y_str, "Y: %.3f", value.y);
          const size_t z_str_len = SNPRINTF_UTF8_RLEN(z_str, "Z: %.3f", value.z);
          add_lines_to_cache(dt,
                             position,
                             {StringRef(w_str, w_str_len),
                              StringRef(x_str, x_str_len),
                              StringRef(y_str, y_str_len),
                              StringRef(z_str, z_str_len)}, col, used_font_size, (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
        }
        else if constexpr (std::is_same_v<T, float4x4>) {
          if (matrix_display == NODE_GEO_DEBUG_MATRIX_VALUES) {
            char r0[96], r1[96], r2[96], r3[96];
            const size_t r0_len = SNPRINTF_UTF8_RLEN(
                r0, "[%.3f %.3f %.3f %.3f]", value[0][0], value[1][0], value[2][0], value[3][0]);
            const size_t r1_len = SNPRINTF_UTF8_RLEN(
                r1, "[%.3f %.3f %.3f %.3f]", value[0][1], value[1][1], value[2][1], value[3][1]);
            const size_t r2_len = SNPRINTF_UTF8_RLEN(
                r2, "[%.3f %.3f %.3f %.3f]", value[0][2], value[1][2], value[2][2], value[3][2]);
            const size_t r3_len = SNPRINTF_UTF8_RLEN(
                r3, "[%.3f %.3f %.3f %.3f]", value[0][3], value[1][3], value[2][3], value[3][3]);
            add_lines_to_cache(dt,
                               position,
                               {StringRef(r0, r0_len),
                                StringRef(r1, r1_len),
                                StringRef(r2, r2_len),
                                StringRef(r3, r3_len)},
                               col,
                               used_font_size,
                               (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
          }
          else {
            float3 location;
            math::EulerXYZ rotation;
            float3 scale;
            math::to_loc_rot_scale_safe<true>(value, location, rotation, scale);

            char location_str[64];
            const size_t location_str_len = SNPRINTF_UTF8_RLEN(
                location_str, "Location: %.3f, %.3f, %.3f", location.x, location.y, location.z);
            char rotation_str[64];
            const size_t rotation_str_len = SNPRINTF_UTF8_RLEN(rotation_str,
                                                               "Rotation: %.3f°, %.3f°, %.3f°",
                                                               rotation.x().degree(),
                                                               rotation.y().degree(),
                                                               rotation.z().degree());
            char scale_str[64];
            const size_t scale_str_len = SNPRINTF_UTF8_RLEN(
                scale_str, "Scale: %.3f, %.3f, %.3f", scale.x, scale.y, scale.z);
            add_lines_to_cache(dt,
                               position,
                               {StringRef(location_str, location_str_len),
                                StringRef(rotation_str, rotation_str_len),
                                StringRef(scale_str, scale_str_len)},
                               col,
                               used_font_size,
                               (y_offsets.is_empty() ? 0.0f : y_offsets[i]));
          }
        }
        else {
          BLI_assert_unreachable();
        }
      }
    });
  }

  static Array<bool> calculate_front_facing(const State &state,
                                            const Mesh &mesh,
                                            const Span<float3> positions,
                                            const bke::AttrDomain domain,
                                            const float4x4 &object_to_world)
  {
    Array<bool> visible(positions.size(), true);
    if (mesh.faces_num == 0 || positions.is_empty()) {
      return visible;
    }

    bool invert_success = false;
    const float4x4 world_to_object = math::invert(object_to_world, invert_success);
    if (!invert_success) {
      return visible;
    }

    bke::BVHTreeFromMesh mesh_bvh = mesh.bvh_corner_tris();
    if (mesh_bvh.tree == nullptr) {
      return visible;
    }

    const std::optional<Bounds<float3>> bounds = mesh.bounds_min_max();
    if (!bounds) {
      return visible;
    }
    const float mesh_scale = math::distance(bounds->min, bounds->max);
    const float visibility_epsilon = std::max(mesh_scale * 1.0e-5f, 1.0e-6f);

    const float3 camera_position_object = math::transform_point(world_to_object,
                                                                 state.camera_position);
    const float3 view_direction_unscaled = math::transform_direction(world_to_object,
                                                                      state.camera_forward);
    const float view_direction_length = math::length(view_direction_unscaled);
    if (view_direction_length <= 1.0e-20f) {
      return visible;
    }
    const float3 view_direction_object = view_direction_unscaled / view_direction_length;
    const bool is_perspective = state.rv3d != nullptr && state.rv3d->is_persp;

    const GroupedSpan<int> vert_to_face_map = mesh.vert_to_face_map();
    const IndexMask &loose_edges = mesh.loose_edges();

    threading::parallel_for(positions.index_range(), 512, [&](const IndexRange range) {
      for (const int64_t i : range) {
        bool has_surface = true;
        switch (domain) {
          case bke::AttrDomain::Point:
            has_surface = i < mesh.verts_num && !vert_to_face_map[i].is_empty();
            break;
          case bke::AttrDomain::Edge:
            has_surface = i < mesh.edges_num && !loose_edges.contains(i);
            break;
          case bke::AttrDomain::Face:
          case bke::AttrDomain::Corner:
            break;
          default:
            has_surface = false;
            break;
        }
        if (!has_surface) {
          continue;
        }

        const float3 &position = positions[i];
        const float3 ray_start = is_perspective ?
                                     camera_position_object :
                                     position + view_direction_object * (mesh_scale + 1.0f);
        const float3 ray_delta = position - ray_start;
        const float point_distance = math::length(ray_delta);
        if (point_distance <= visibility_epsilon) {
          continue;
        }

        BVHTreeRayHit hit{};
        hit.index = -1;
        hit.dist = point_distance;
        BLI_bvhtree_ray_cast(mesh_bvh.tree,
                             ray_start,
                             ray_delta / point_distance,
                             0.0f,
                             &hit,
                             mesh_bvh.raycast_callback,
                             &mesh_bvh);
        visible[i] = hit.index == -1 || hit.dist >= point_distance - visibility_epsilon;
      }
    });
    return visible;
  }

  static float3 calc_corner_text_position(const float3 &corner_pos,
                                          const float3 &prev_corner_pos,
                                          const float3 &next_corner_pos,
                                          const float3 &face_normal,
                                          const RegionView3D *rv3d,
                                          const float4x4 &object_to_world,
                                          const float offset_scale = 1.0f)
  {
    const float3 prev_edge_vec = prev_corner_pos - corner_pos;
    const float3 next_edge_vec = next_corner_pos - corner_pos;
    const float3 prev_edge_dir = math::normalize(prev_edge_vec);
    const float3 next_edge_dir = math::normalize(next_edge_vec);

    const float pre_edge_len = math::length(prev_edge_vec);
    const float next_edge_len = math::length(next_edge_vec);
    const float max_offset = math::min(pre_edge_len, next_edge_len) / 2;

    const float3 corner_normal = math::cross(next_edge_dir, prev_edge_dir);
    const float concavity_check = math::dot(corner_normal, face_normal);
    const float direction_correct = concavity_check > 0.0f ? 1.0f : -1.0f;
    const float3 bisector_dir = (prev_edge_dir + next_edge_dir) / 2 * direction_correct;

    const float sharp_factor = std::clamp(math::dot(prev_edge_dir, next_edge_dir), 0.0f, 1.0f);
    const float sharp_multiplier = math::pow(sharp_factor, 4.0f) * 2 + 1;

    const float3 pos_o_world = math::transform_point(object_to_world, corner_pos);
    const float pixel_size = ED_view3d_pixel_size(rv3d, pos_o_world);
    const float pixel_offset = ui::style_get()->widget.points * 7.0f * UI_SCALE_FAC;
    const float screen_space_offset = pixel_size * pixel_offset;

    const float offset_distance = std::clamp(
        screen_space_offset * sharp_multiplier * offset_scale, 0.0f, max_offset);

    return corner_pos + bisector_dir * offset_distance;
  }
};

}  // namespace blender::draw::overlay
