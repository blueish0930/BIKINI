/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include <string>

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "DNA_curve_types.h"
#include "DNA_pointcloud_types.h"

#include "draw_cache.hh"
#include "draw_cache_impl.hh"
#include "overlay_attribute_debug_util.hh"
#include "overlay_base.hh"

namespace blender::draw::overlay {

/**
 * Displays geometry node viewer / debug attribute colors on top of objects.
 *
 * Two independent paths (do not mix attribute names):
 * - Official Viewer: preview geometry only, attribute `.viewer`, gated by Viewer Attribute overlay.
 * - Local Debug: evaluated geometry, attribute `.debug_color`, always available in 3D view.
 */
class AttributeViewer : Overlay {
 private:
  PassMain ps_ = {"attribute_viewer_ps_"};

  PassMain::Sub *mesh_sub_ = nullptr;
  PassMain::Sub *pointcloud_sub_ = nullptr;
  PassMain::Sub *gsplat_sub_ = nullptr;
  PassMain::Sub *curve_sub_ = nullptr;
  PassMain::Sub *curves_sub_ = nullptr;
  PassMain::Sub *instance_sub_ = nullptr;

 public:
  void begin_sync(Resources &res, const State &state) final
  {
    ps_.init();
    /* Always enabled in the 3D viewport so Geometry Nodes Debug color works without the Viewer
     * overlay toggle. Active Viewer previews use the same pass when the toggle is on. */
    enabled_ = state.is_space_v3d() && !res.is_selection();
    if (!enabled_) {
      return;
    }
    ps_.bind_ubo(OVERLAY_GLOBALS_SLOT, &res.globals_buf);
    ps_.bind_ubo(DRW_CLIPPING_UBO_SLOT, &res.clip_planes_buf);
    /* Solid mode + Mesh: Depth Equal so the color overlay sits on the already-drawn surface
     * (same fix as facing overlay, see #128113).
     * PointCloud / Curves: Depth Equal fails because procedural points and curve wires do not
     * share exact depth samples with the base pass - Debug/Viewer colors never appear. Use
     * Less-equal for those geometry types in all shading modes. */
    const bool is_solid_viewport = state.v3d && state.v3d->shading.type == OB_SOLID;
    const DRWState mesh_depth = is_solid_viewport ? DRW_STATE_DEPTH_EQUAL :
                                                    DRW_STATE_DEPTH_LESS_EQUAL;
    const DRWState non_mesh_depth = DRW_STATE_DEPTH_LESS_EQUAL;
    const DRWState color_blend = DRW_STATE_WRITE_COLOR | DRW_STATE_BLEND_ALPHA;
    ps_.state_set(color_blend | mesh_depth, state.clipping_plane_count);

    auto create_sub = [&](const char *name, gpu::Shader *shader, const DRWState draw_state) {
      auto &sub = ps_.sub(name);
      sub.state_set(draw_state, state.clipping_plane_count);
      sub.shader_set(shader);
      return &sub;
    };

    mesh_sub_ = create_sub(
        "mesh", res.shaders->attribute_viewer_mesh.get(), color_blend | mesh_depth);
    pointcloud_sub_ = create_sub("pointcloud",
                                 res.shaders->attribute_viewer_pointcloud.get(),
                                 color_blend | non_mesh_depth);
    gsplat_sub_ = create_sub("gsplat",
                             res.shaders->attribute_viewer_gsplat.get(),
                             color_blend | non_mesh_depth);
    curve_sub_ = create_sub(
        "curve", res.shaders->attribute_viewer_curve.get(), color_blend | non_mesh_depth);
    curves_sub_ = create_sub(
        "curves", res.shaders->attribute_viewer_curves.get(), color_blend | non_mesh_depth);
    instance_sub_ = create_sub(
        "instance", res.shaders->uniform_color.get(), color_blend | non_mesh_depth);
  }

  void object_sync(Manager &manager,
                   const ObjectRef &ob_ref,
                   Resources & /*res*/,
                   const State &state) final
  {
    if (!enabled_) {
      return;
    }

    /* Geometry Nodes Debug on Instance domain: color lives on the InstancesComponent of the
     * geometry-set stack (not on mesh/curve data). Handle before geometry-domain path. */
    if (populate_debug_instance_color(ob_ref, state, manager)) {
      return;
    }

    /* Geometry Nodes Debug `.debug_color` always wins when present - independent of the
     * Viewer Attribute overlay toggle. Uses the same mesh batch as Viewer (extract prefers
     * `.debug_color`). */
    if (object_has_debug_color(ob_ref)) {
      populate_debug_geometry(ob_ref, state, manager);
      return;
    }

    const bool is_preview = ob_ref.preview_base_geometry() != nullptr;

    /* Official Viewer path: Attribute overlay on + Viewer preview geometry. */
    if (state.show_attribute_viewer() && is_preview) {
      if (ob_ref.preview_instance_index() >= 0) {
        if (const bke::InstancesComponent *comp =
                ob_ref.preview_base_geometry()->get_component<bke::InstancesComponent>())
        {
          if (const std::optional<bke::AttributeAccessor> attrs = comp->attributes()) {
            if (const std::optional<bke::AttributeMetaData> meta =
                    attrs->lookup_meta_data(".viewer"))
            {
              if (attribute_type_supports_viewer_overlay(meta->data_type)) {
                populate_for_instance(ob_ref, state, manager);
                return;
              }
            }
          }
        }
      }
      /* Geometry-domain `.viewer` (Mesh / PointCloud / Curves) — same as official. */
      populate_viewer_geometry(ob_ref, state, manager);
    }
  }

  void pre_draw(Manager &manager, View &view) final
  {
    if (!enabled_) {
      return;
    }

    manager.generate_commands(ps_, view);
  }

  void draw_line(Framebuffer &framebuffer, Manager &manager, View &view) final
  {
    if (!enabled_) {
      return;
    }

    GPU_framebuffer_bind(framebuffer);
    manager.submit_only(ps_, view);
  }

 private:
  static bool attribute_type_supports_viewer_overlay(const bke::AttrType data_type)
  {
    /* Skip types the GPU color overlay cannot display as meaningful colors.
     * String is text-only (see overlay_attribute_text.hh); admitting it falls through to
     * extract_attr_viewer's ColorGeometry4f default {1,0,1,1} (solid purple). */
    return !ELEM(data_type,
                 bke::AttrType::Quaternion,
                 bke::AttrType::Float4x4,
                 bke::AttrType::String);
  }

  static bool object_has_debug_color(const ObjectRef &ob_ref)
  {
    Object &object = *ob_ref.object;
    switch (object.type) {
      case OB_MESH: {
        Mesh &mesh = DRW_object_get_data_for_drawing<Mesh>(object);
        return mesh.attributes().contains(".debug_color");
      }
      case OB_POINTCLOUD: {
        PointCloud &pointcloud = DRW_object_get_data_for_drawing<PointCloud>(object);
        return pointcloud.attributes().contains(".debug_color");
      }
      case OB_CURVES_LEGACY: {
        Curve &curve = DRW_object_get_data_for_drawing<Curve>(object);
        return curve.curve_eval &&
               curve.curve_eval->geometry.wrap().attributes().contains(".debug_color");
      }
      case OB_CURVES: {
        blender::Curves &curves_id = DRW_object_get_data_for_drawing<blender::Curves>(object);
        return curves_id.geometry.wrap().attributes().contains(".debug_color");
      }
      default:
        return false;
    }
  }


  /** Draw Instance-domain Debug `.debug_color` as a uniform tint on each instanced object. */
  bool populate_debug_instance_color(const ObjectRef &ob_ref, const State & /*state*/, Manager &manager)
  {
    Object &object = *ob_ref.object;
    auto draw_one = [&](const ColorGeometry4f &color, const ResourceHandleRange handle) {
      switch (object.type) {
        case OB_MESH: {
          if (gpu::Batch *batch = DRW_cache_mesh_surface_get(&object)) {
            auto &sub = *instance_sub_;
            sub.push_constant("ucolor", float4(color));
            sub.draw(batch, handle);
          }
          if (gpu::Batch *batch = DRW_cache_mesh_loose_edges_get(&object)) {
            auto &sub = *instance_sub_;
            sub.push_constant("ucolor", float4(color));
            sub.draw(batch, handle);
          }
          break;
        }
        case OB_POINTCLOUD: {
          auto &sub = *instance_sub_;
          if (gpu::Batch *batch = pointcloud_sub_pass_setup(sub, ob_ref, handle, nullptr)) {
            sub.push_constant("ucolor", float4(color));
            sub.draw(batch, handle);
          }
          break;
        }
        case OB_CURVES_LEGACY: {
          if (gpu::Batch *batch = DRW_cache_curve_edge_wire_get(&object)) {
            auto &sub = *instance_sub_;
            sub.push_constant("ucolor", float4(color));
            sub.draw(batch, handle);
          }
          break;
        }
        default:
          break;
      }
    };

    const int count = ob_ref.instances_count();
    bool any = false;
    for (int i = 0; i < count; i++) {
      const DupliObject *dupli = ob_ref.dupli_object(i);
      ColorGeometry4f color;
      if (!debug_attr::instance_debug_color_at(dupli, color)) {
        continue;
      }
      any = true;
      /* One resource handle per instance so each can have a different color. */
      const ResourceHandleRange handle(manager.resource_handle(ob_ref.object_to_world(i)));
      draw_one(color, handle);
    }
    return any;
  }
  void populate_for_instance(const ObjectRef &ob_ref, const State &state, Manager &manager)
  {
    Object &object = *ob_ref.object;
    const bke::GeometrySet &base_geometry = *ob_ref.preview_base_geometry();
    const bke::InstancesComponent &instances =
        *base_geometry.get_component<bke::InstancesComponent>();
    const bke::AttributeAccessor instance_attributes = *instances.attributes();
    const VArray attribute = *instance_attributes.lookup<ColorGeometry4f>(".viewer");
    if (!attribute) {
      return;
    }
    ColorGeometry4f color = attribute.get(ob_ref.preview_instance_index());
    color.a *= state.overlay.viewer_attribute_opacity;
    switch (object.type) {
      case OB_MESH: {
        ResourceHandleRange res_handle = manager.unique_handle(ob_ref);
        {
          gpu::Batch *batch = DRW_cache_mesh_surface_get(&object);
          auto &sub = *instance_sub_;
          sub.push_constant("ucolor", float4(color));
          sub.draw(batch, res_handle);
        }
        if (gpu::Batch *batch = DRW_cache_mesh_loose_edges_get(&object)) {
          auto &sub = *instance_sub_;
          sub.push_constant("ucolor", float4(color));
          sub.draw(batch, res_handle);
        }
        break;
      }
      case OB_POINTCLOUD: {
        auto &sub = *instance_sub_;
        ResourceHandleRange res_handle = manager.unique_handle(ob_ref);
        gpu::Batch *batch = pointcloud_sub_pass_setup(sub, ob_ref, res_handle, nullptr);
        sub.push_constant("ucolor", float4(color));
        sub.draw(batch, res_handle);
        break;
      }
      case OB_CURVES_LEGACY: {
        gpu::Batch *batch = DRW_cache_curve_edge_wire_get(&object);
        auto &sub = *instance_sub_;
        sub.push_constant("ucolor", float4(color));
        ResourceHandleRange res_handle = manager.unique_handle(ob_ref);
        sub.draw(batch, res_handle);
        break;
      }
      case OB_CURVES: {
        /* Not supported yet because instances of this type are currently drawn as legacy curves.
         */
        break;
      }
      default:
        break;
    }
  }

  /** Official Viewer geometry colors: always attribute name `.viewer`. */
  void populate_viewer_geometry(const ObjectRef &ob_ref, const State &state, Manager &manager)
  {
    const float opacity = state.overlay.viewer_attribute_opacity;
    Object &object = *ob_ref.object;
    switch (object.type) {
      case OB_MESH: {
        Mesh &mesh = DRW_object_get_data_for_drawing<Mesh>(object);
        if (const std::optional<bke::AttributeMetaData> meta =
                mesh.attributes().lookup_meta_data(".viewer"))
        {
          if (attribute_type_supports_viewer_overlay(meta->data_type)) {
            gpu::Batch *batch = DRW_cache_mesh_surface_viewer_attribute_get(&object);
            auto &sub = *mesh_sub_;
            sub.push_constant("opacity", opacity);
            sub.draw(batch, manager.unique_handle(ob_ref));
          }
        }
        break;
      }
      case OB_POINTCLOUD: {
        PointCloud &pointcloud = DRW_object_get_data_for_drawing<PointCloud>(object);
        if (const std::optional<bke::AttributeMetaData> meta =
                pointcloud.attributes().lookup_meta_data(".viewer"))
        {
          if (attribute_type_supports_viewer_overlay(meta->data_type)) {
            gpu::VertBuf **pointcloud_vertbuf = DRW_pointcloud_evaluated_attribute(&pointcloud,
                                                                                   ".viewer");
            gpu::VertBuf **gsplat_vertbuf = DRW_gsplat_evaluated_attribute(&pointcloud, ".viewer");

            /* Avoid trying to bind an empty `vertbuf` which causes assert / undefined behavior. */
            ResourceHandleRange res_handle = manager.unique_handle(ob_ref);
            if (pointcloud.totpoint > 0 && pointcloud_vertbuf != nullptr) {
              auto &sub = *pointcloud_sub_;
              gpu::Batch *batch = pointcloud_sub_pass_setup(sub, ob_ref, res_handle, nullptr);
              sub.push_constant("opacity", opacity);
              sub.bind_texture("attribute_tx", pointcloud_vertbuf);
              sub.draw(batch, res_handle);
            }
            if (pointcloud.totpoint > 0 && gsplat_vertbuf != nullptr) {
              auto &sub = *gsplat_sub_;
              gpu::Batch *batch = gsplat_sub_pass_setup(sub, ob_ref, res_handle, nullptr);
              sub.push_constant("opacity", opacity);
              sub.bind_texture("attribute_tx", gsplat_vertbuf);
              sub.draw(batch, res_handle);
            }
          }
        }
        break;
      }
      case OB_CURVES_LEGACY: {
        Curve &curve = DRW_object_get_data_for_drawing<Curve>(object);
        if (curve.curve_eval) {
          const bke::CurvesGeometry &curves = curve.curve_eval->geometry.wrap();
          if (const std::optional<bke::AttributeMetaData> meta =
                  curves.attributes().lookup_meta_data(".viewer"))
          {
            if (attribute_type_supports_viewer_overlay(meta->data_type)) {
              gpu::Batch *batch = DRW_cache_curve_edge_wire_viewer_attribute_get(&object);
              auto &sub = *curve_sub_;
              sub.push_constant("opacity", opacity);
              ResourceHandleRange res_handle = manager.unique_handle(ob_ref);
              sub.draw(batch, res_handle);
            }
          }
        }
        break;
      }
      case OB_CURVES: {
        blender::Curves &curves_id = DRW_object_get_data_for_drawing<blender::Curves>(object);
        const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
        if (const std::optional<bke::AttributeMetaData> meta =
                curves.attributes().lookup_meta_data(".viewer"))
        {
          if (attribute_type_supports_viewer_overlay(meta->data_type)) {
            bool is_point_domain = false;
            bool is_valid = false;
            gpu::VertBufPtr &texture = DRW_curves_texture_for_evaluated_attribute(
                &curves_id, ".viewer", is_point_domain, is_valid);
            if (is_valid) {
              auto &sub = *curves_sub_;
              const char *error = nullptr;
              gpu::Batch *batch = curves_sub_pass_setup(sub, state.scene, ob_ref.object, error);
              sub.push_constant("opacity", opacity);
              sub.push_constant("is_point_domain", is_point_domain);
              sub.bind_texture("color_tx", texture);
              sub.draw(batch, manager.unique_handle(ob_ref));
            }
          }
        }
        break;
      }
      default:
        break;
    }
  }

  /** Local Debug geometry colors: attribute `.debug_color` only. */
  void populate_debug_geometry(const ObjectRef &ob_ref, const State &state, Manager &manager)
  {
    /* Debug opacity is baked into .debug_color alpha; push 1.0 like prior local path. */
    constexpr float opacity = 1.0f;
    Object &object = *ob_ref.object;
    switch (object.type) {
      case OB_MESH: {
        Mesh &mesh = DRW_object_get_data_for_drawing<Mesh>(object);
        if (mesh.attributes().contains(".debug_color")) {
          gpu::Batch *batch = DRW_cache_mesh_surface_viewer_attribute_get(&object);
          auto &sub = *mesh_sub_;
          sub.push_constant("opacity", opacity);
          sub.draw(batch, manager.unique_handle(ob_ref));
        }
        break;
      }
      case OB_POINTCLOUD: {
        PointCloud &pointcloud = DRW_object_get_data_for_drawing<PointCloud>(object);
        if (pointcloud.attributes().contains(".debug_color")) {
          gpu::VertBuf **vertbuf = DRW_pointcloud_evaluated_attribute(&pointcloud, ".debug_color");
          if (pointcloud.totpoint > 0 && vertbuf != nullptr) {
            auto &sub = *pointcloud_sub_;
            gpu::Batch *batch = pointcloud_sub_pass_setup(
                sub, ob_ref, manager.unique_handle(ob_ref), nullptr);
            sub.push_constant("opacity", opacity);
            sub.bind_texture("attribute_tx", vertbuf);
            sub.draw(batch, manager.unique_handle(ob_ref));
          }
        }
        break;
      }
      case OB_CURVES_LEGACY: {
        Curve &curve = DRW_object_get_data_for_drawing<Curve>(object);
        if (curve.curve_eval) {
          const bke::CurvesGeometry &curves = curve.curve_eval->geometry.wrap();
          if (curves.attributes().contains(".debug_color")) {
            gpu::Batch *batch = DRW_cache_curve_edge_wire_viewer_attribute_get(&object);
            auto &sub = *curve_sub_;
            sub.push_constant("opacity", opacity);
            ResourceHandleRange res_handle = manager.unique_handle(ob_ref);
            sub.draw(batch, res_handle);
          }
        }
        break;
      }
      case OB_CURVES: {
        blender::Curves &curves_id = DRW_object_get_data_for_drawing<blender::Curves>(object);
        const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
        if (curves.attributes().contains(".debug_color")) {
          bool is_point_domain = false;
          bool is_valid = false;
          gpu::VertBufPtr &texture = DRW_curves_texture_for_evaluated_attribute(
              &curves_id, ".debug_color", is_point_domain, is_valid);
          if (is_valid) {
            auto &sub = *curves_sub_;
            const char *error = nullptr;
            gpu::Batch *batch = curves_sub_pass_setup(sub, state.scene, ob_ref.object, error);
            sub.push_constant("opacity", opacity);
            sub.push_constant("is_point_domain", is_point_domain);
            sub.bind_texture("color_tx", texture);
            sub.draw(batch, manager.unique_handle(ob_ref));
          }
        }
        break;
      }
      default:
        break;
    }
  }
};

}  // namespace blender::draw::overlay
