/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 *
 * Draws Debug node vector arrows and matrix axes in the 3D viewport.
 */

#pragma once

#include <algorithm>
#include <string>

#include "BKE_attribute.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_object_types.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_pointcloud.hh"

#include "BLI_color_types.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"

#include "DNA_curves_types.h"
#include "DNA_node_types.h"
#include "DNA_grease_pencil_types.h"
#include "DNA_pointcloud_types.h"

#include "DRW_render.hh"

#include "draw_cache.hh"
#include "draw_common.hh"

#include "overlay_attribute_debug_util.hh"
#include "overlay_base.hh"
#include "overlay_shader_shared.hh"

namespace blender::draw::overlay {

class AttributeDebugVectors : Overlay {
  using InstanceBuf = ShapeInstanceBuf<ExtraInstanceData>;

 private:
  const SelectionType selection_type_;
  PassSimple ps_ = {"AttributeDebugVectors"};
  InstanceBuf arrow_buf_ = {selection_type_, "debug_arrow_buf"};
  InstanceBuf axes_x_buf_ = {selection_type_, "debug_axes_x_buf"};
  InstanceBuf axes_y_buf_ = {selection_type_, "debug_axes_y_buf"};
  InstanceBuf axes_z_buf_ = {selection_type_, "debug_axes_z_buf"};

 public:
  AttributeDebugVectors(const SelectionType selection_type) : selection_type_(selection_type) {}

  void begin_sync(Resources &res, const State &state) final
  {
    /* Always available in 3D view — Debug previews must not depend on Viewer overlay toggles. */
    enabled_ = state.is_space_v3d() && !res.is_selection();
    arrow_buf_.clear();
    axes_x_buf_.clear();
    axes_y_buf_.clear();
    axes_z_buf_.clear();
  }

  void object_sync(Manager & /*manager*/,
                   const ObjectRef &ob_ref,
                   Resources &res,
                   const State & /*state*/) final
  {
    if (!enabled_) {
      return;
    }
    Object &object = *ob_ref.object;
    const select::ID select_id = res.select_id(ob_ref);

    /* Instance-domain Debug arrows.
     *
     * GN instancers usually have OB_VISIBLE_SELF cleared, so the parent never reaches object_sync.
     * Instance attributes live on the InstancesComponent in DupliObject::instance_data, not on the
     * instanced mesh/curve. Draw one arrow set per drawn instance from that stack.
     *
     * When the parent *is* self-visible, also cover the geometry_set_eval path (no DupliObject). */
    if (ob_ref.is_dupli() || ob_ref.is_range()) {
      const int count = ob_ref.instances_count();
      for (int i = 0; i < count; i++) {
        populate_from_dupli_instance(
            ob_ref.dupli_object(i), ob_ref.object_to_world(i), select_id);
      }
    }
    else if (object.runtime->geometry_set_eval != nullptr) {
      if (const bke::Instances *instances = object.runtime->geometry_set_eval->get_instances()) {
        if (debug_attr::instances_have_debug_layers(*instances)) {
          populate_from_instances(*instances, object.object_to_world(), select_id);
        }
      }
    }

    const float4x4 object_to_world = ob_ref.is_range() ? ob_ref.object_to_world(0) :
                                                         object.object_to_world();

    switch (object.type) {
      case OB_MESH: {
        const Mesh &mesh = DRW_object_get_data_for_drawing<Mesh>(object);
        populate_from_attributes(mesh.attributes(), object_to_world, select_id, nullptr);
        break;
      }
      case OB_POINTCLOUD: {
        const PointCloud &pointcloud = DRW_object_get_data_for_drawing<PointCloud>(object);
        populate_from_attributes(pointcloud.attributes(), object_to_world, select_id, nullptr);
        break;
      }
      case OB_CURVES_LEGACY: {
        const Curve &curve = DRW_object_get_data_for_drawing<Curve>(object);
        if (curve.curve_eval) {
          const bke::CurvesGeometry &curves = curve.curve_eval->geometry.wrap();
          populate_from_attributes(curves.attributes(), object_to_world, select_id, &curves);
        }
        break;
      }
      case OB_CURVES: {
        const Curves &curves_id = DRW_object_get_data_for_drawing<Curves>(object);
        const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
        populate_from_attributes(curves.attributes(), object_to_world, select_id, &curves);
        break;
      }
      case OB_GREASE_PENCIL: {
        const GreasePencil &gp = DRW_object_get_data_for_drawing<GreasePencil>(object);
        populate_from_grease_pencil(gp, object, object_to_world, select_id);
        break;
      }
      default:
        break;
    }
  }

  void end_sync(Resources &res, const State &state) final
  {
    if (!enabled_) {
      return;
    }
    ps_.init();
    ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH | DRW_STATE_DEPTH_LESS_EQUAL,
                  state.clipping_plane_count);
    ps_.shader_set(res.shaders->extra_shape.get());
    ps_.bind_ubo(OVERLAY_GLOBALS_SLOT, &res.globals_buf);
    ps_.bind_ubo(DRW_CLIPPING_UBO_SLOT, &res.clip_planes_buf);
    res.select_bind(ps_);

    arrow_buf_.end_sync(ps_, res.shapes.single_arrow.get());
    axes_x_buf_.end_sync(ps_, res.shapes.single_arrow.get());
    axes_y_buf_.end_sync(ps_, res.shapes.single_arrow.get());
    axes_z_buf_.end_sync(ps_, res.shapes.single_arrow.get());
  }

  void draw_line(Framebuffer &framebuffer, Manager &manager, View &view) final
  {
    if (!enabled_) {
      return;
    }
    GPU_framebuffer_bind(framebuffer);
    manager.submit(ps_, view);
  }

 private:
  static Vector<std::string> collect_value_names(const bke::AttributeAccessor &attributes)
  {
    Vector<std::string> names;
    attributes.foreach_attribute([&](const bke::AttributeIter &iter) {
      if (iter.name.startswith(".debug_") && iter.name.endswith("_value")) {
        names.append(iter.name);
      }
    });
    std::sort(names.begin(), names.end());
    return names;
  }

  static std::string layer_prefix_from_value_name(const StringRef name)
  {
    if (name.startswith(".debug_") && name.endswith("_value")) {
      return std::string(name.drop_suffix(6));
    }
    return "";
  }

  static float4x4 arrow_matrix(const float3 &origin,
                               const float3 &direction,
                               const float4x4 &object_to_world)
  {
    const float3 world_origin = math::transform_point(object_to_world, origin);
    float3 world_dir = math::transform_direction(object_to_world, direction);
    const float length = math::length(world_dir);
    if (length < 1.0e-8f) {
      return float4x4::identity();
    }
    world_dir /= length;
    float4x4 mat = math::from_up_axis<float4x4>(world_dir);
    mat.location() = world_origin;
    /* Empty single-arrow geometry points along +Z and is unit-length; scale by vector length. */
    mat[0] *= length;
    mat[1] *= length;
    mat[2] *= length;
    return mat;
  }

  void append_arrow(const float3 &origin,
                    const float3 &direction,
                    const float4 &color,
                    const float4x4 &object_to_world,
                    const select::ID select_id,
                    InstanceBuf &buf)
  {
    if (math::length_squared(direction) < 1.0e-12f) {
      return;
    }
    const float4x4 mat = arrow_matrix(origin, direction, object_to_world);
    buf.append(ExtraInstanceData(mat, color, 1.0f), select_id);
  }

  static float3 prepare_vector_arrow(const float3 &vector,
                                     const bool normalize,
                                     const float scale)
  {
    float3 dir = vector;
    if (normalize) {
      const float len = math::length(dir);
      if (len < 1.0e-8f) {
        return float3(0.0f);
      }
      dir /= len;
    }
    return dir * scale;
  }

  void populate_from_attributes(const bke::AttributeAccessor &attributes,
                                const float4x4 &object_to_world,
                                const select::ID select_id,
                                const bke::CurvesGeometry *curves_for_spline)
  {
    const Vector<std::string> names = debug_attr::collect_debug_value_names(attributes);
    for (const std::string &name : names) {
      const bke::GAttributeReader attribute = attributes.lookup(name);
      if (!attribute) {
        continue;
      }
      const std::string prefix = debug_attr::layer_prefix_from_value_name(name);
      const bke::AttrDomain domain = attribute.domain;
      const VArray<bool> valid = *attributes.lookup_or_default<bool>(
          prefix + "_valid", domain, true);

      Array<float3> positions_storage;
      Span<float3> positions;
      if (domain == bke::AttrDomain::Curve && curves_for_spline != nullptr) {
        positions_storage = debug_attr::curve_domain_positions(*curves_for_spline);
        positions = positions_storage;
      }
      else {
        positions_storage = debug_attr::positions_for_domain(attributes, domain);
        positions = positions_storage;
      }
      if (positions.is_empty()) {
        continue;
      }

      append_arrows_for_attribute(
          attribute, attributes, prefix, domain, valid, positions, object_to_world, select_id);
    }
  }

  /**
   * Draw Instance-domain Debug arrows for a single drawn instance (DupliObject).
   * \param instance_to_world: Full world matrix of this instance (dupli.mat).
   * Origin is the instance location; vector directions use the same matrix.
   */
  void populate_from_dupli_instance(const DupliObject *dupli,
                                    const float4x4 &instance_to_world,
                                    const select::ID select_id)
  {
    if (dupli == nullptr) {
      return;
    }
    for (const int s : IndexRange(ARRAY_SIZE(dupli->instance_data))) {
      if (dupli->instance_data[s] == nullptr) {
        continue;
      }
      const bke::Instances *instances = dupli->instance_data[s]->get_instances();
      if (instances == nullptr || !debug_attr::instances_have_debug_layers(*instances)) {
        continue;
      }
      const int index = dupli->instance_idx[s];
      if (index < 0 || index >= instances->instances_num()) {
        continue;
      }
      const bke::AttributeAccessor attributes = instances->attributes();
      const Vector<std::string> names = debug_attr::collect_debug_value_names(attributes);
      for (const std::string &name : names) {
        const bke::GAttributeReader attribute = attributes.lookup(name);
        if (!attribute) {
          continue;
        }
        const std::string prefix = debug_attr::layer_prefix_from_value_name(name);
        const bke::AttrDomain domain = bke::AttrDomain::Instance;
        const VArray<bool> valid = *attributes.lookup_or_default<bool>(
            prefix + "_valid", domain, true);
        if (index >= valid.size() || !valid[index]) {
          continue;
        }
        /* Local origin (0) — instance_to_world places the arrow at the instance. */
        const float3 local_origin(0.0f);
        if (attribute.varray.type().is<float3>()) {
          if (index >= attribute.varray.size()) {
            continue;
          }
          const VArraySpan<float3> vectors = attribute.varray.typed<float3>();
          const VArray<int> display = *attributes.lookup_or_default<int>(
              prefix + "_display", domain, int(NODE_GEO_DEBUG_VECTOR_ARROWS));
          if (display[index] != NODE_GEO_DEBUG_VECTOR_ARROWS) {
            continue;
          }
          const VArray<bool> normalize = *attributes.lookup_or_default<bool>(
              prefix + "_arrow_normalize", domain, false);
          const VArray<float> scale = *attributes.lookup_or_default<float>(
              prefix + "_arrow_scale", domain, 1.0f);
          const VArray<bool> has_arrow_color = *attributes.lookup_or_default<bool>(
              prefix + "_has_arrow_color", domain, false);
          const VArray<ColorGeometry4f> arrow_colors =
              *attributes.lookup_or_default<ColorGeometry4f>(
                  prefix + "_arrow_color",
                  domain,
                  ColorGeometry4f(1.0f, 0.85f, 0.15f, 1.0f));
          float4 color(1.0f, 0.85f, 0.15f, 1.0f);
          if (has_arrow_color[index]) {
            const ColorGeometry4f c = arrow_colors[index];
            color = float4(c.r, c.g, c.b, c.a);
          }
          const float3 dir = prepare_vector_arrow(vectors[index], normalize[index], scale[index]);
          append_arrow(local_origin, dir, color, instance_to_world, select_id, arrow_buf_);
        }
        else if (attribute.varray.type().is<float4x4>()) {
          if (index >= attribute.varray.size()) {
            continue;
          }
          const VArraySpan<float4x4> matrices = attribute.varray.typed<float4x4>();
          const VArray<int> display = *attributes.lookup_or_default<int>(
              prefix + "_display", domain, int(NODE_GEO_DEBUG_MATRIX_COMPONENTS));
          if (display[index] != NODE_GEO_DEBUG_MATRIX_AXES) {
            continue;
          }
          const VArray<bool> normalize = *attributes.lookup_or_default<bool>(
              prefix + "_arrow_normalize", domain, false);
          const VArray<float> scale = *attributes.lookup_or_default<float>(
              prefix + "_arrow_scale", domain, 1.0f);
          const float4x4 &m = matrices[index];
          const float4 color_x(0.96f, 0.20f, 0.32f, 1.0f);
          const float4 color_y(0.55f, 0.86f, 0.00f, 1.0f);
          const float4 color_z(0.16f, 0.56f, 1.00f, 1.0f);
          append_arrow(local_origin,
                       prepare_vector_arrow(m.x_axis(), normalize[index], scale[index]),
                       color_x,
                       instance_to_world,
                       select_id,
                       axes_x_buf_);
          append_arrow(local_origin,
                       prepare_vector_arrow(m.y_axis(), normalize[index], scale[index]),
                       color_y,
                       instance_to_world,
                       select_id,
                       axes_y_buf_);
          append_arrow(local_origin,
                       prepare_vector_arrow(m.z_axis(), normalize[index], scale[index]),
                       color_z,
                       instance_to_world,
                       select_id,
                       axes_z_buf_);
        }
      }
    }
  }
  void populate_from_instances(const bke::Instances &instances,
                               const float4x4 &parent_to_world,
                               const select::ID select_id)
  {
    const bke::AttributeAccessor attributes = instances.attributes();
    const Array<float3> positions = debug_attr::instance_domain_positions(instances);
    if (positions.is_empty()) {
      return;
    }
    /* Instance positions are already in the component's space; parent_to_world maps them. */
    const Vector<std::string> names = debug_attr::collect_debug_value_names(attributes);
    for (const std::string &name : names) {
      const bke::GAttributeReader attribute = attributes.lookup(name);
      if (!attribute) {
        continue;
      }
      const std::string prefix = debug_attr::layer_prefix_from_value_name(name);
      const bke::AttrDomain domain = bke::AttrDomain::Instance;
      const VArray<bool> valid = *attributes.lookup_or_default<bool>(
          prefix + "_valid", domain, true);
      append_arrows_for_attribute(
          attribute, attributes, prefix, domain, valid, positions, parent_to_world, select_id);
    }
  }

  void populate_from_grease_pencil(const GreasePencil &grease_pencil,
                                   const Object &object,
                                   const float4x4 &object_to_world,
                                   const select::ID select_id)
  {
    const bke::AttributeAccessor attributes = grease_pencil.attributes();
    const Array<float3> positions = debug_attr::layer_domain_positions(grease_pencil, object);
    if (positions.is_empty()) {
      return;
    }
    const Vector<std::string> names = debug_attr::collect_debug_value_names(attributes);
    for (const std::string &name : names) {
      const bke::GAttributeReader attribute = attributes.lookup(name);
      if (!attribute) {
        continue;
      }
      const std::string prefix = debug_attr::layer_prefix_from_value_name(name);
      const bke::AttrDomain domain = attribute.domain;
      if (domain != bke::AttrDomain::Layer) {
        continue;
      }
      const VArray<bool> valid = *attributes.lookup_or_default<bool>(
          prefix + "_valid", domain, true);
      append_arrows_for_attribute(
          attribute, attributes, prefix, domain, valid, positions, object_to_world, select_id);
    }
  }

  void append_arrows_for_attribute(const bke::GAttributeReader &attribute,
                                   const bke::AttributeAccessor &attributes,
                                   const std::string &prefix,
                                   const bke::AttrDomain domain,
                                   const VArray<bool> &valid,
                                   const Span<float3> positions,
                                   const float4x4 &object_to_world,
                                   const select::ID select_id)
  {
    if (attribute.varray.type().is<float3>()) {
      const VArraySpan<float3> vectors = attribute.varray.typed<float3>();
      const VArray<int> display = *attributes.lookup_or_default<int>(
          prefix + "_display", domain, int(NODE_GEO_DEBUG_VECTOR_ARROWS));
      const VArray<bool> normalize = *attributes.lookup_or_default<bool>(
          prefix + "_arrow_normalize", domain, false);
      const VArray<float> scale = *attributes.lookup_or_default<float>(
          prefix + "_arrow_scale", domain, 1.0f);
      const VArray<bool> has_arrow_color = *attributes.lookup_or_default<bool>(
          prefix + "_has_arrow_color", domain, false);
      const VArray<ColorGeometry4f> arrow_colors = *attributes.lookup_or_default<ColorGeometry4f>(
          prefix + "_arrow_color", domain, ColorGeometry4f(1.0f, 0.85f, 0.15f, 1.0f));
      const float4 default_color(1.0f, 0.85f, 0.15f, 1.0f);
      const int64_t n = std::min<int64_t>(positions.size(), vectors.size());
      for (const int64_t i : IndexRange(n)) {
        if (!valid[i] || display[i] != NODE_GEO_DEBUG_VECTOR_ARROWS) {
          continue;
        }
        const float3 dir = prepare_vector_arrow(vectors[i], normalize[i], scale[i]);
        float4 color = default_color;
        if (has_arrow_color[i]) {
          const ColorGeometry4f c = arrow_colors[i];
          color = float4(c.r, c.g, c.b, c.a);
        }
        append_arrow(positions[i], dir, color, object_to_world, select_id, arrow_buf_);
      }
    }
    else if (attribute.varray.type().is<float4x4>()) {
      const VArraySpan<float4x4> matrices = attribute.varray.typed<float4x4>();
      const VArray<int> display = *attributes.lookup_or_default<int>(
          prefix + "_display", domain, int(NODE_GEO_DEBUG_MATRIX_COMPONENTS));
      const VArray<bool> normalize = *attributes.lookup_or_default<bool>(
          prefix + "_arrow_normalize", domain, false);
      const VArray<float> scale = *attributes.lookup_or_default<float>(
          prefix + "_arrow_scale", domain, 1.0f);
      const float4 color_x(0.96f, 0.20f, 0.32f, 1.0f);
      const float4 color_y(0.55f, 0.86f, 0.00f, 1.0f);
      const float4 color_z(0.16f, 0.56f, 1.00f, 1.0f);
      const int64_t n = std::min<int64_t>(positions.size(), matrices.size());
      for (const int64_t i : IndexRange(n)) {
        if (!valid[i] || display[i] != NODE_GEO_DEBUG_MATRIX_AXES) {
          continue;
        }
        const float4x4 &m = matrices[i];
        append_arrow(positions[i],
                     prepare_vector_arrow(m.x_axis(), normalize[i], scale[i]),
                     color_x,
                     object_to_world,
                     select_id,
                     axes_x_buf_);
        append_arrow(positions[i],
                     prepare_vector_arrow(m.y_axis(), normalize[i], scale[i]),
                     color_y,
                     object_to_world,
                     select_id,
                     axes_y_buf_);
        append_arrow(positions[i],
                     prepare_vector_arrow(m.z_axis(), normalize[i], scale[i]),
                     color_z,
                     object_to_world,
                     select_id,
                     axes_z_buf_);
      }
    }
  }

};

}  // namespace blender::draw::overlay
