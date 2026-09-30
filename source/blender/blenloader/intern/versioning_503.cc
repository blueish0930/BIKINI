/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup blenloader
 */

#define DNA_DEPRECATED_ALLOW

#include "DNA_ID.h"
#include "DNA_brush_types.h"
#include "DNA_camera_types.h"
#include "DNA_collection_types.h"
#include "DNA_curves_types.h"
#include "DNA_grease_pencil_types.h"
#include "DNA_mesh_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_sequence_types.h"
#include "DNA_space_types.h"
#include "DNA_view3d_types.h"
#include "DNA_windowmanager_types.h"

#include "MEM_guardedalloc.h"

#include "BLI_listbase.hh"
#include "BLI_listbase_iterator.hh"
#include "BLI_math_base.hh"
#include "BLI_string_ref.hh"
#include "BLI_string_utf8.hh"
#include "BLI_sys_types.hh"

#include "BKE_attribute.h"
#include "BKE_attribute.hh"
#include "BKE_camera.h"
#include "BKE_image.hh"
#include "BKE_lib_id.hh"
#include "BKE_compositor.hh"
#include "BKE_main.hh"
#include "BKE_mesh_legacy_convert.hh"
#include "BKE_node.hh"
#include "BKE_node_legacy_types.hh"
#include "BKE_node_runtime.hh"
#include "BKE_paint.hh"
#include "BKE_paint_types.hh"
#include "BKE_screen.hh"
#include "BKE_texture.h"

#include "DNA_image_types.h"

#include "SEQ_iterator.hh"
#include "SEQ_sequencer.hh"

#include "readfile.hh"

#include "versioning_common.hh"

// #include "CLG_log.h"

namespace blender {

// static CLG_LogRef LOG = {"blend.doversion"};

static void do_version_set_grease_pencil_colors_options_to_inputs(bNodeTree &ntree, bNode &node)
{
  if (blender::bke::node_find_socket(node, SOCK_IN, "Mode"_ustr)) {
    return;
  }
  bNodeSocket &socket = version_node_add_socket(ntree, node, SOCK_IN, "NodeSocketMenu", "Mode");
  socket.default_value_typed<bNodeSocketValueMenu>()->value = node.custom1;
}

static void do_version_set_grease_pencil_depth_options_to_inputs(bNodeTree &ntree, bNode &node)
{
  if (blender::bke::node_find_socket(node, SOCK_IN, "Depth Order"_ustr)) {
    return;
  }
  bNodeSocket &socket = version_node_add_socket(
      ntree, node, SOCK_IN, "NodeSocketMenu", "Depth Order");
  socket.default_value_typed<bNodeSocketValueMenu>()->value = node.custom1;
}

static void do_version_merge_layers_options_to_inputs(bNodeTree &ntree, bNode &node)
{
  if (!version_node_ensure_storage_or_invalidate(node)) {
    return;
  }

  auto &storage = *reinterpret_cast<NodeGeometryMergeLayers *>(node.storage);

  if (blender::bke::node_find_socket(node, SOCK_IN, "Mode"_ustr)) {
    return;
  }
  bNodeSocket &socket = version_node_add_socket(ntree, node, SOCK_IN, "NodeSocketMenu", "Mode");
  socket.default_value_typed<bNodeSocketValueMenu>()->value = storage.mode;
}

static void compositing_node_group_to_effect(Main &main, Scene &scene)
{
  bNodeTree *node_group = version_get_scene_compositor_node_tree(&main, &scene);
  if (!node_group) {
    return;
  }

  SceneCompositorEffect &effect = bke::compositor::new_effect(scene, "Effect");
  effect.node_group = node_group;
  if (!node_group->compositor_node_asset_traits) {
    node_group->compositor_node_asset_traits = MEM_new<CompositorNodeAssetTraits>(__func__);
  }
  node_group->compositor_node_asset_traits->flag |= COMPOSIT_NODE_ASSET_SCENE_EFFECT;
  bke::node_update_asset_metadata(*node_group);
  scene.compositing_node_group = nullptr;
}

static void version_shader_tex_image_add_image_socket(bNodeTree &ntree, bNode &node)
{
  bNodeSocket *sock = bke::node_find_socket(node, SOCK_IN, "Image"_ustr);
  if (!sock) {
    sock = &version_node_add_socket(ntree, node, SOCK_IN, "NodeSocketImage", "Image");
  }
  if (sock != static_cast<bNodeSocket *>(node.inputs.first())) {
    BLI_remlink(&node.inputs, sock);
    BLI_addhead(&node.inputs, sock);
  }
  Image *ima = id_cast<Image *>(node.id);
  if (!ima) {
    return;
  }
  auto *val = sock->default_value_typed<bNodeSocketValueImage>();
  if (!val || val->value == ima) {
    return;
  }
  id_us_min(val->value ? &val->value->id : nullptr);
  val->value = ima;
  id_us_plus(&ima->id);
}

static void version_convert_shader_geo_image_texture(bNode &node)
{
  auto *old_tex = static_cast<NodeGeometryImageTexture *>(node.storage);
  NodeTexImage *tex = MEM_new<NodeTexImage>(__func__);
  BKE_texture_mapping_default(&tex->base.tex_mapping, TEXMAP_TYPE_POINT);
  BKE_texture_colormapping_default(&tex->base.color_mapping);
  BKE_imageuser_default(&tex->iuser);
  if (old_tex) {
    tex->interpolation = old_tex->interpolation;
    tex->extension = old_tex->extension;
    tex->projection = old_tex->projection;
    tex->projection_blend = old_tex->projection_blend;
    tex->iuser = old_tex->iuser;
    tex->iuser.scene = nullptr;
    MEM_delete(old_tex);
  }
  node.storage = tex;

  STRNCPY_UTF8(node.idname, "ShaderNodeTexImage");
  node.type_legacy = SH_NODE_TEX_IMAGE;
  if (bke::bNodeType *ntype = bke::node_type_find("ShaderNodeTexImage"_ustr)) {
    node.typeinfo = ntype;
  }

  if (const bNodeSocket *sock = bke::node_find_socket(node, SOCK_IN, "Image"_ustr)) {
    if (const auto *val = sock->default_value_typed<bNodeSocketValueImage>()) {
      if (val->value && node.id != &val->value->id) {
        id_us_min(node.id);
        node.id = &val->value->id;
        id_us_plus(node.id);
      }
    }
  }
}

static float version_503_camera_view_viewfac(const int sensor_fit,
                                             const int winx,
                                             const int winy,
                                             const float frame_aspect)
{
  const float2 frame_size = BKE_camera_frame_size(winx, winy, frame_aspect);

  return (BKE_camera_sensor_fit(sensor_fit, 1.0f, frame_aspect) == CAMERA_SENSOR_FIT_HOR) ?
             frame_size.x :
             frame_size.y;
}

static void version_503_camera_view_fit(RegionView3D *rv3d,
                                        const int sensor_fit,
                                        const int winx,
                                        const int winy,
                                        const float render_aspect)
{
  /* The camera frame is now automatically fitted to the viewport, taking into count
   * the render aspect ratio. Here we version existing camera views to match. */
  if ((rv3d->persp != RV3D_CAMOB) && (rv3d->camzoom == 0.0f) && (rv3d->camdx == 0.0f) &&
      (rv3d->camdy == 0.0f))
  {
    return;
  }

  const float viewfac_old = version_503_camera_view_viewfac(
      sensor_fit, winx, winy, float(winy) / float(winx));
  const float viewfac_new = version_503_camera_view_viewfac(sensor_fit, winx, winy, render_aspect);

  /* Adjust zoom factor. */
  const float zoomfac_old = BKE_screen_view3d_zoom_to_fac(rv3d->camzoom);
  rv3d->camzoom = math::clamp(
      BKE_screen_view3d_zoom_from_fac(zoomfac_old * viewfac_old / viewfac_new),
      float(RV3D_CAMZOOM_MIN),
      float(RV3D_CAMZOOM_MAX));

  /* Adjust panning. */
  const float zoomfac_new = BKE_screen_view3d_zoom_to_fac(rv3d->camzoom);
  rv3d->camdx = math::clamp(rv3d->camdx * zoomfac_old / zoomfac_new, -1.0f, 1.0f);
  rv3d->camdy = math::clamp(rv3d->camdy * zoomfac_old / zoomfac_new, -1.0f, 1.0f);
}

static void do_versioning_camera_view_zoom(Main *bmain)
{
  for (bScreen &screen : bmain->screens) {
    Scene *scene = bmain->scenes.first_as<Scene>();
    for (wmWindowManager &wm : bmain->wm) {
      for (wmWindow &win : wm.windows) {
        if (win.winid == screen.winid) {
          scene = win.scene;
        }
      }
    }
    if (scene == nullptr) {
      continue;
    }
    const float render_aspect = (float(scene->r.ysch) * scene->r.yasp) /
                                (float(scene->r.xsch) * scene->r.xasp);

    for (ScrArea &area : screen.areabase) {
      for (SpaceLink &space : area.spacedata) {
        if (space.spacetype != SPACE_VIEW3D) {
          continue;
        }
        View3D *v3d = reinterpret_cast<View3D *>(&space);

        const Object *camera = (v3d->camera != nullptr) ? v3d->camera : scene->camera;
        const int sensor_fit = (camera != nullptr && camera->type == OB_CAMERA &&
                                camera->data != nullptr) ?
                                   id_cast<const Camera *>(camera->data)->sensor_fit :
                                   CAMERA_SENSOR_FIT_AUTO;

        ListBaseT<ARegion> *regions = (area.spacedata.first() == &space) ? &area.regionbase :
                                                                           &space.regionbase;
        for (ARegion &region : *regions) {
          if (region.regiontype != RGN_TYPE_WINDOW) {
            continue;
          }
          RegionView3D *rv3d = static_cast<RegionView3D *>(region.regiondata);
          if (rv3d == nullptr) {
            continue;
          }
          if ((region.winx <= 1) || (region.winy <= 1)) {
            continue;
          }

          version_503_camera_view_fit(rv3d, sensor_fit, region.winx, region.winy, render_aspect);
          if (rv3d->localvd != nullptr) {
            version_503_camera_view_fit(
                rv3d->localvd, sensor_fit, region.winx, region.winy, render_aspect);
          }
        }
      }
    }
  }
}

static void clear_deprecated_brush_flags(Brush &brush)
{
  if (brush.gpencil_settings) {
    brush.gpencil_settings->flag &= ~GP_BRUSH_UNUSED_1;
    brush.gpencil_settings->flag2 &= ~(GP_BRUSH_UNUSED_2 | GP_BRUSH_UNUSED_3 | GP_BRUSH_UNUSED_4 |
                                       GP_BRUSH_UNUSED_5 | GP_BRUSH_UNUSED_6 | GP_BRUSH_UNUSED_7);
  }
  brush.paint_flags &= ~BRUSH_PAINT_UNUSED_2;
  brush.flag &= ~(BRUSH_UNUSED_7 | BRUSH_UNUSED_8);
}

void do_versions_after_linking_503(FileData * /*fd*/, Main *bmain)
{
  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 8)) {
    version_node_socket_index_animdata(
        bmain, NTREE_GEOMETRY, "GeometryNodeSetGreasePencilColor", 5, 1, 6);
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 15)) {
    for (Scene &scene : bmain->scenes) {
      compositing_node_group_to_effect(*bmain, scene);
    }
  }
  else if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 27)) {
    for (Scene &scene : bmain->scenes) {
      /* BIKINI files written at 503.16-26 still have a single compositing_node_group.
       * Official 503.15+ files already have compositor_effects; do not duplicate. */
      if (scene.compositor_effects.is_empty()) {
        compositing_node_group_to_effect(*bmain, scene);
      }
      else {
        scene.compositing_node_group = nullptr;
      }
    }
  }
  else {
    /* The now deprecated compositing_node_group is always written on file writes for forward
     * compatibility, so it has to be reset to nullptr if no versioning was needed.
     *
     * Todo(#140111): Forward compatibility support will be removed in 6.0, and this loop can then
     * be placed behind a `MAIN_VERSION_FILE_OLDER(bmain, 600, xxx)` check . */
    for (Scene &scene : bmain->scenes) {
      scene.compositing_node_group = nullptr;
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 16)) {
    /* Shift animation data to accommodate the new dispersion inputs. */
    version_node_socket_index_animdata(bmain, NTREE_SHADER, "ShaderNodeBsdfPrincipled", 20, 2, 33);
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 28)) {
    /* Vector was input 0 on vanilla ShaderNodeTexImage; Image is inserted at 0. */
    version_node_socket_index_animdata(bmain, NTREE_SHADER, SH_NODE_TEX_IMAGE, 0, 1, 1);

    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      if (node_tree->type != NTREE_SHADER) {
        continue;
      }
      for (bNode &node : node_tree->nodes) {
        if (STREQ(node.idname, "GeometryNodeImageTexture")) {
          version_convert_shader_geo_image_texture(node);
          continue;
        }
        if (node.type_legacy == SH_NODE_TEX_IMAGE) {
          version_shader_tex_image_add_image_socket(*node_tree, node);
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 29)) {
    /* 503.28 skipped copying node->id when declaration refresh already added an empty Image
     * socket, then updatefunc cleared the image. Re-sync for files that still have node->id. */
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      if (node_tree->type != NTREE_SHADER) {
        continue;
      }
      for (bNode &node : node_tree->nodes) {
        if (node.type_legacy == SH_NODE_TEX_IMAGE) {
          version_shader_tex_image_add_image_socket(*node_tree, node);
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 33)) {
    /* Official 503.19: camera view zoom. Remapped because BIKINI already used 19–32. */
    do_versioning_camera_view_zoom(bmain);
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 34)) {
    /* Official 503.20: anisotropic glass sockets. */
    version_node_socket_index_animdata(bmain, NTREE_SHADER, "ShaderNodeBsdfGlass", 5, 3, 7);
    version_node_socket_index_animdata(bmain, NTREE_SHADER, "ShaderNodeBsdfGlass", 2, 2, 4);
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 23)) {
    bool has_skip_alphabet_sort_method = false;
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &space : area.spacedata) {
          if (space.spacetype == SPACE_OUTLINER) {
            SpaceOutliner *space_outliner = reinterpret_cast<SpaceOutliner *>(&space);
            if (space_outliner->flag & SO_FLAG_UNUSED_4) {
              has_skip_alphabet_sort_method = true;
            }
          }
        }
      }
    }
    auto version_collection_fn = [&](Collection &collection) {
      Map<Object *, int> parent_child_indices;
      int index = 0;
      for (CollectionChild &child : collection.children) {
        child.sort_index = index++;
      }
      for (CollectionObject &cob : collection.gobject) {
        cob.sort_index = (has_skip_alphabet_sort_method) ? index++ : -1;
        if (has_skip_alphabet_sort_method && cob.ob != nullptr && cob.ob->parent != nullptr) {
          int &child_index = parent_child_indices.lookup_or_add(cob.ob->parent, 0);
          cob.parented_sort_index = child_index++;
        }
        else {
          cob.parented_sort_index = -1;
        }
      }
    };
    for (Collection &collection : bmain->collections) {
      version_collection_fn(collection);
    }
    for (Scene &scene : bmain->scenes) {
      if (scene.master_collection != nullptr) {
        version_collection_fn(*scene.master_collection);
      }
    }
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &space : area.spacedata) {
          if (space.spacetype == SPACE_OUTLINER) {
            SpaceOutliner *space_outliner = reinterpret_cast<SpaceOutliner *>(&space);
            if (has_skip_alphabet_sort_method && !(space_outliner->flag & SO_FLAG_UNUSED_4)) {
              space_outliner->sort_method = SO_SORT_ALPHA;
            }
            else {
              space_outliner->sort_method = SO_SORT_CUSTOM;
            }
            space_outliner->flag &= ~SO_FLAG_UNUSED_4;
          }
        }
      }
    }
  }

  /**
   * Always bump subversion in BKE_blender_version.h when adding versioning
   * code here, and wrap it inside a MAIN_VERSION_FILE_ATLEAST check.
   *
   * \note Keep this message at the bottom of the function.
   */
}

void blo_do_versions_503(FileData * /*fd*/, Library * /*lib*/, Main *bmain)
{
  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 1)) {
    for (Scene &scene : bmain->scenes) {
      VPaint *wpaint = scene.toolsettings->wpaint;
      if (wpaint && wpaint->paint.brush_asset_reference) {
        const StringRefNull old_asset_id =
            wpaint->paint.brush_asset_reference->relative_asset_identifier;
        if (wpaint->paint.brush == nullptr && old_asset_id.endswith("Paint")) {
          /* The "Paint" brush asset was renamed to "Add Weight", find it via the default instead
           * of hard-coding the new name. */
          if (std::optional<AssetWeakReference> paint_brush_asset_reference =
                  BKE_paint_brush_type_default_reference(bke::paint::AssetCategory::MeshWeight,
                                                         WPAINT_BRUSH_TYPE_DRAW))
          {
            BKE_paint_brush_set(bmain, &wpaint->paint, *paint_brush_asset_reference);
          }
        }
      }
    }
  }

  /* The compositor previously did not support default inputs for group nodes, but some built-in
   * nodes had the position field default type for some inputs, so node groups would gain it as a
   * default type through some operators. Later, the default inputs were supported for group nodes,
   * though position field were not supported in the compositor, so it would assert. To fix this,
   * we reset any position field default input to the default value. */
  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 3)) {
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id) {
      if (node_tree->type == NTREE_COMPOSIT) {
        node_tree->ensure_interface_cache();
        for (bNodeTreeInterfaceSocket *input : node_tree->interface_inputs()) {
          if (input->default_input == NODE_DEFAULT_INPUT_POSITION_FIELD) {
            input->default_input = NODE_DEFAULT_INPUT_VALUE;
          }
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 4)) {
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &sl : area.spacedata) {
          if (sl.spacetype == SPACE_ACTION) {
            SpaceAction *saction = reinterpret_cast<SpaceAction *>(&sl);
            saction->cache_display |= TIME_CACHE_COMPOSITOR;
          }
        }
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 6)) {
    for (Brush &brush : bmain->brushes) {
      if (ELEM(brush.ob_mode, OB_MODE_WEIGHT_PAINT, OB_MODE_VERTEX_PAINT)) {
        brush.mesh_automasking_settings = MEM_new<MeshAutomaskingSettings>(__func__);
        brush.mesh_automasking_settings->cavity_curve = BKE_sculpt_default_cavity_curve();
      }
    }

    auto apply_to_paint = [&](Paint *paint) {
      if (paint == nullptr) {
        return;
      }

      paint->mesh_automasking_settings = MEM_new<MeshAutomaskingSettings>("blo_do_versions_520");
      paint->mesh_automasking_settings->cavity_curve = BKE_sculpt_default_cavity_curve();
      paint->mesh_automasking_settings->cavity_curve_op = BKE_sculpt_default_cavity_curve();
    };

    for (Scene &scene : bmain->scenes) {
      apply_to_paint(reinterpret_cast<Paint *>(scene.toolsettings->vpaint));
      apply_to_paint(reinterpret_cast<Paint *>(scene.toolsettings->wpaint));
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 7)) {
    for (Scene &scene : bmain->scenes) {
      for (ViewLayer &view_layer : scene.view_layers) {
        view_layer.eevee.denoising_pass_flags =
            EEVEE_DENOISING_PASS_USE_ALBEDO_ROUGHNESS_WEIGHTING;
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 8)) {
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      for (bNode &node : node_tree->nodes) {
        if (STREQ(node.idname, "GeometryNodeSetGreasePencilColor")) {
          do_version_set_grease_pencil_colors_options_to_inputs(*node_tree, node);
        }
        if (STREQ(node.idname, "GeometryNodeSetGreasePencilDepth")) {
          do_version_set_grease_pencil_depth_options_to_inputs(*node_tree, node);
        }
        if (STREQ(node.idname, "GeometryNodeMergeLayers")) {
          do_version_merge_layers_options_to_inputs(*node_tree, node);
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 9)) {
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &space_link : area.spacedata) {
          if (space_link.spacetype == SPACE_VIEW3D) {
            View3D &view3d = reinterpret_cast<View3D &>(space_link);
            view3d.overlay.viewer_attribute_text_size = 11.0f;
          }
        }
      }
    }
    for (Brush &brush : bmain->brushes) {
      if (brush.curve_hardness == nullptr) {
        brush.curve_hardness = brush.paint_flags & BRUSH_PAINT_UNUSED_2 ?
                                   BKE_paint_default_curve_inverted() :
                                   BKE_paint_default_curve();
      }
      if (brush.curve_auto_smooth == nullptr) {
        brush.curve_auto_smooth = BKE_paint_default_curve_inverted();
      }
      if (brush.curve_spacing == nullptr) {
        brush.curve_spacing = BKE_paint_default_curve();
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 12)) {
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      for (bNode &node : node_tree->nodes) {
        if (node.type_legacy == GEO_NODE_SIMULATION_OUTPUT && node.storage) {
          auto &storage = *static_cast<NodeGeometrySimulationOutput *>(node.storage);
          storage.cache_limit = 250;
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 13)) {
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      for (bNode &node : node_tree->nodes) {
        if (node.type_legacy == GEO_NODE_SIMULATION_OUTPUT && node.storage) {
          auto &storage = *static_cast<NodeGeometrySimulationOutput *>(node.storage);
          storage.cache_limit_mode = GEO_NODE_SIMULATION_CACHE_LIMIT_FRAMES;
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 14)) {
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      for (bNode &node : node_tree->nodes) {
        if (node.type_legacy == GEO_NODE_SIMULATION_OUTPUT && node.storage) {
          auto &storage = *static_cast<NodeGeometrySimulationOutput *>(node.storage);
          storage.cache_memory_limit = 1024.0f;
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 15)) {
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      for (bNode &node : node_tree->nodes) {
        if (node.type_legacy == GEO_NODE_SIMULATION_OUTPUT && node.storage) {
          auto &storage = *static_cast<NodeGeometrySimulationOutput *>(node.storage);
          /* Image Process simulation dense cache + checkpoint defaults. */
          storage.cached_frames = 20;
          storage.checkpoint_rate = 24;
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  /* 503.17–20: do NOT rewrite Debug Text / Color overlay or View3D Attribute Text flags.
   * Saved blend values must stick (user choice). Only DNA defaults apply to newly created
   * nodes/viewports — never force on/off during file load. */

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 21)) {
    /* No data rewrite. Bumps subversion past the old force-on / force-off migrations. */
  }

  /* Official 503.11–13 landed after BIKINI had already consumed those subversions.
   * Catch them up at 22 so existing BIKINI files still receive the official migrations.
   * The blocks are idempotent (zero-fill / deprecated-flag / re-validate). */
  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 22)) {
    /* Official 503.11: sculpt defaults that were previously only set on mode enter. */
    for (Scene &scene : bmain->scenes) {
      Sculpt *sd = scene.toolsettings->sculpt;
      if (sd == nullptr) {
        continue;
      }

      const Sculpt defaults = {};
      if (sd->detail_percent == 0.0f) {
        sd->detail_percent = defaults.detail_percent;
      }
      if (sd->constant_detail == 0.0f) {
        sd->constant_detail = defaults.constant_detail;
      }
      if (sd->detail_size == 0.0f) {
        sd->detail_size = defaults.detail_size;
      }

      if (!sd->paint.tile_offset[0]) {
        sd->paint.tile_offset[0] = 1.0f;
      }
      if (!sd->paint.tile_offset[1]) {
        sd->paint.tile_offset[1] = 1.0f;
      }
      if (!sd->paint.tile_offset[2]) {
        sd->paint.tile_offset[2] = 1.0f;
      }
    }

    /* Official 503.12: migrate removed Front-Face Falloff on weight/vertex paint brushes. */
    for (Brush &brush : bmain->brushes) {
      if (brush.ob_mode & OB_MODE_WEIGHT_PAINT || brush.ob_mode & OB_MODE_VERTEX_PAINT) {
        if (brush.flag & BRUSH_UNUSED_7 && brush.falloff_angle_legacy != 0.0f) {
          if (brush.mesh_automasking_settings == nullptr) {
            continue;
          }
          switch (brush.falloff_shape) {
            case PAINT_FALLOFF_SHAPE_SPHERE:
              brush.mesh_automasking_settings->flags |= BRUSH_AUTOMASKING_BRUSH_NORMAL;
              brush.mesh_automasking_settings->start_normal_falloff = 0.5f;
              brush.mesh_automasking_settings->start_normal_limit = brush.falloff_angle_legacy;
              break;
            case PAINT_FALLOFF_SHAPE_TUBE:
              brush.mesh_automasking_settings->flags |= BRUSH_AUTOMASKING_VIEW_NORMAL;
              brush.mesh_automasking_settings->view_normal_falloff = 0.5f;
              brush.mesh_automasking_settings->view_normal_limit = brush.falloff_angle_legacy;
              break;
          }
        }
      }
    }

    /* Official 503.13: clamp invalid attributes_active_index to -1. */
    auto validate_active_index_fn = [](const AttributeOwner &owner, int &active_index) -> void {
      const bke::AttributeStorage *storage = owner.get_storage();
      if ((active_index < 0) || (active_index >= storage->count()) ||
          !bke::allow_procedural_attribute_access(storage->at_index(active_index).name()))
      {
        active_index = -1;
      }
    };

    for (Mesh &mesh : bmain->meshes) {
      validate_active_index_fn(AttributeOwner::from_id(&mesh.id), mesh.attributes_active_index);
    }
    for (Curves &curves : bmain->hair_curves) {
      validate_active_index_fn(AttributeOwner::from_id(&curves.id),
                               curves.geometry.attributes_active_index);
    }
    for (GreasePencil &grease_pencil : bmain->grease_pencils) {
      validate_active_index_fn(AttributeOwner::from_id(&grease_pencil.id),
                               grease_pencil.attributes_active_index);
      for (GreasePencilDrawingBase *drawing_base : grease_pencil.drawings()) {
        if (drawing_base->type == GP_DRAWING) {
          GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);
          validate_active_index_fn(
              AttributeOwner(AttributeOwnerType::GreasePencilDrawing, drawing),
              drawing->geometry.attributes_active_index);
        }
      }
    }
    for (PointCloud &pointcloud : bmain->pointclouds) {
      validate_active_index_fn(AttributeOwner::from_id(&pointcloud.id),
                               pointcloud.attributes_active_index);
    }

    /* Official files at 503.13 skipped BIKINI 12; fill unset simulation cache limits. */
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      for (bNode &node : node_tree->nodes) {
        if (node.type_legacy == GEO_NODE_SIMULATION_OUTPUT && node.storage) {
          auto &storage = *static_cast<NodeGeometrySimulationOutput *>(node.storage);
          if (storage.cache_limit == 0) {
            storage.cache_limit = 250;
          }
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 14)) {
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &space : area.spacedata) {
          if (space.spacetype == SPACE_OUTLINER) {
            SpaceOutliner *space_outliner = reinterpret_cast<SpaceOutliner *>(&space);
            space_outliner->flag |= SO_EXPAND_ON_FOCUS;
          }
        }
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 16)) {
    for (Brush &brush : bmain->brushes) {
      if (ELEM(brush.ob_mode,
               OB_MODE_SCULPT,
               OB_MODE_VERTEX_PAINT,
               OB_MODE_TEXTURE_PAINT,
               OB_MODE_SCULPT_GREASE_PENCIL,
               OB_MODE_VERTEX_GREASE_PENCIL))
      {
        brush.unified_paint_flags |= BRUSH_USE_UNIFIED_PAINT_SIZE | BRUSH_USE_UNIFIED_PAINT_COLOR;
      }
      if (ELEM(brush.ob_mode,
               OB_MODE_SCULPT_CURVES,
               OB_MODE_WEIGHT_PAINT,
               OB_MODE_WEIGHT_GREASE_PENCIL))
      {
        brush.unified_paint_flags |= BRUSH_USE_UNIFIED_PAINT_SIZE;
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 17)) {
    for (Scene &scene : bmain->scenes) {
      if (Editing *ed = seq::editing_get(&scene)) {
        seq::foreach_strip(&ed->seqbase, [&](Strip *strip) {
          switch (strip->type) {
            case STRIP_TYPE_IMAGE:
              /* Every image in the sequence has its own #StripElem. Content trimming
               * (#anim_startofs, #anim_endofs) hides elements at both ends without shrinking the
               * array, so the full array is always this many elements long. */
              strip->data->stripdata_num = strip->anim_startofs + strip->len + strip->anim_endofs;
              break;
            case STRIP_TYPE_MOVIE:
            case STRIP_TYPE_SOUND:
              /* Single #StripElem storing the file path. */
              strip->data->stripdata_num = strip->data->stripdata ? 1 : 0;
              break;
            default:
              strip->data->stripdata_num = 0;
              break;
          }
          return true;
        });
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 23)) {
    /* Shader Attribute: expose storage name as a String input so Repeat zones can drive it. */
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      if (node_tree->type != NTREE_SHADER) {
        continue;
      }
      for (bNode &node : node_tree->nodes) {
        if (node.type_legacy != SH_NODE_ATTRIBUTE || !node.storage) {
          continue;
        }
        auto &storage = *static_cast<NodeShaderAttribute *>(node.storage);
        bNodeSocket *name_sock = bke::node_find_socket(node, SOCK_IN, "Name"_ustr);
        if (!name_sock) {
          name_sock = &version_node_add_socket(*node_tree, node, SOCK_IN, "NodeSocketString", "Name");
        }
        if (name_sock->default_value && storage.name[0] != '\0') {
          STRNCPY_UTF8(name_sock->default_value_typed<bNodeSocketValueString>()->value, storage.name);
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 24)) {
    /* GPU Texture Editor trees used to have 0 users (editor holds only USER_ONE), so
     * File > Clean Up > Purge Unused Data deleted them. Give existing files a fake user. */
    for (bNodeTree &ntree : bmain->nodetrees) {
      if (ntree.type == NTREE_IMAGE) {
        id_fake_user_set(&ntree.id);
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 25)) {
    /* Fair: N-panel Max Continuity lives in custom1 (default 3). Old nodes left it at 0. */
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      for (bNode &node : node_tree->nodes) {
        if (node.type_legacy == GEO_NODE_CGAL_FAIR && node.custom1 == 0) {
          node.custom1 = 3;
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 26)) {
    /* Enable original-label overlay on existing node editors. */
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &sl : area.spacedata) {
          if (sl.spacetype == SPACE_NODE) {
            SpaceNode *snode = reinterpret_cast<SpaceNode *>(&sl);
            snode->overlay.flag |= SN_OVERLAY_SHOW_ORIGINAL_LABELS;
          }
        }
      }
    }
  }

  if (MAIN_VERSION_FILE_ATLEAST(bmain, 503, 19) && !MAIN_VERSION_FILE_ATLEAST(bmain, 503, 30)) {
    /* BIKINI used 720-725 for HLSL/SDF/etc. Official Material Lighting Nodes now occupy those IDs.
     * Skip nodes whose idname is already a lighting node. */
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      if (node_tree->type != NTREE_SHADER) {
        continue;
      }
      for (bNode &node : node_tree->nodes) {
        if (STRPREFIX(node.idname, "ShaderNodeLight") ||
            STREQ(node.idname, "ShaderNodeShadowRaycast"))
        {
          continue;
        }
        switch (node.type_legacy) {
          case 720:
            node.type_legacy = SH_NODE_HLSL;
            break;
          case 721:
            node.type_legacy = SH_NODE_SDF_SHAPE;
            break;
          case 722:
            node.type_legacy = SH_NODE_FRACTAL_PRIMITIVE;
            break;
          case 723:
            node.type_legacy = SH_NODE_PARALLAX_OCCLUSION;
            break;
          case 724:
            node.type_legacy = SH_NODE_SPOM;
            break;
          case 725:
            node.type_legacy = SH_NODE_BILLBOARD_DISPLACEMENT;
            break;
          default:
            break;
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 33)) {
    /* Official 503.19: VSE thumbnail overlay flag. Remapped past BIKINI 19–32. */
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &space : area.spacedata) {
          if (space.spacetype == SPACE_SEQ) {
            SpaceSeq *space_sequencer = reinterpret_cast<SpaceSeq *>(&space);
            SequencerTimelineOverlay &timeline_overlay = space_sequencer->timeline_overlay;
            const bool show_thumbnails =
                (timeline_overlay.flag & SEQ_TIMELINE_MIDDLE_THUMBNAILS) ||
                (timeline_overlay.flag & SEQ_TIMELINE_STRIP_END_THUMBNAILS) ||
                (timeline_overlay.flag & SEQ_TIMELINE_CONTINUOUS_THUMBNAILS);
            SET_FLAG_FROM_TEST(
                timeline_overlay.flag, show_thumbnails, SEQ_TIMELINE_SHOW_THUMBNAILS);
          }
        }
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 36)) {
    /* BIKINI stored original-type labels on overlay bit 9; official now uses bit 9 for text
     * info. Move the BIKINI flag to bit 10 and do not inherit it as text info. */
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &sl : area.spacedata) {
          if (sl.spacetype == SPACE_NODE) {
            SpaceNode *snode = reinterpret_cast<SpaceNode *>(&sl);
            if (snode->overlay.flag & (1 << 9)) {
              snode->overlay.flag |= SN_OVERLAY_SHOW_ORIGINAL_LABELS;
              snode->overlay.flag &= ~SN_OVERLAY_SHOW_TEXT_INFO;
            }
          }
        }
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 37)) {
    /* Restore BIKINI angled node links after the draw-path was dropped in an upstream merge.
     * Existing files still have the overlay byte as 0 (Bezier) from unused DNA padding. */
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &sl : area.spacedata) {
          if (sl.spacetype == SPACE_NODE) {
            SpaceNode *snode = reinterpret_cast<SpaceNode *>(&sl);
            snode->overlay.link_style = SN_OVERLAY_LINK_ANGLED;
          }
        }
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 38)) {
    /* BIKINI: node editor coordinate-system overlay. New DNA field is zero in old files. */
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &sl : area.spacedata) {
          if (sl.spacetype == SPACE_NODE) {
            SpaceNode *snode = reinterpret_cast<SpaceNode *>(&sl);
            snode->overlay.flag &= ~SN_OVERLAY_SHOW_COORDINATES;
            snode->overlay.coordinate_alpha = 0.5f;
          }
        }
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 39)) {
    /* === BIKINI LuxCore Begin === */
    for (Scene &scene : bmain->scenes) {
      if (scene.luxcore.halt_samples <= 0) {
        scene.luxcore.halt_samples = 64;
      }
      if (scene.luxcore.halt_preview_samples <= 0) {
        scene.luxcore.halt_preview_samples = 16;
      }
      if (scene.luxcore.path_depth <= 0) {
        scene.luxcore.path_depth = 8;
      }
    }
    /* === BIKINI LuxCore End === */
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 41)) {
    /* Expand intern LuxCore render settings. Old files only had a few halt
     * fields; PhotonGI caustics were hardcoded on for F12. */
    for (Scene &scene : bmain->scenes) {
      SceneLuxCore &lux = scene.luxcore;
      if (lux.path_depth_diffuse <= 0) {
        lux.path_depth_diffuse = 4;
      }
      if (lux.path_depth_glossy <= 0) {
        lux.path_depth_glossy = 4;
      }
      if (lux.path_depth_specular <= 0) {
        lux.path_depth_specular = (lux.path_depth > 12) ? lux.path_depth : 12;
      }
      if (lux.seed <= 0) {
        lux.seed = 1;
      }
      if (lux.photongi_photon_depth <= 0) {
        lux.photongi_photon_depth = 6;
      }
      if (lux.bidir_light_depth <= 0) {
        lux.bidir_light_depth = 10;
      }
      if (lux.bidir_eye_depth <= 0) {
        lux.bidir_eye_depth = 10;
      }
      if (lux.tile_size <= 0) {
        lux.tile_size = 64;
      }
      if (lux.tile_aa <= 0) {
        lux.tile_aa = 3;
      }
      if (lux.hybrid_partition <= 0.0f) {
        lux.hybrid_partition = 0.6f;
      }
      if (lux.hybrid_glossiness <= 0.0f) {
        lux.hybrid_glossiness = 0.049f;
      }
      if (lux.photongi_photon_millions <= 0.0f) {
        lux.photongi_photon_millions = 1.0f;
      }
      if (lux.photongi_caustic_millions <= 0.0f) {
        lux.photongi_caustic_millions = 0.2f;
      }
      if (lux.photongi_caustic_radius <= 0.0f) {
        lux.photongi_caustic_radius = 0.075f;
      }
      if (lux.photongi_glossiness <= 0.0f) {
        lux.photongi_glossiness = 0.049f;
      }
      if (lux.filter_width <= 0.0f) {
        lux.filter_width = 1.5f;
      }
      if (lux.metropolis_largestep <= 0.0f) {
        lux.metropolis_largestep = 0.4f;
      }
      if (lux.tonemap_scale <= 0.0f) {
        lux.tonemap_scale = 1.0f;
      }
      lux.flag |= SCE_LUXCORE_PHOTONGI | SCE_LUXCORE_PHOTONGI_CAUSTIC |
                  SCE_LUXCORE_VIEWPORT_LIGHTTRACE;
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 42)) {
    /* Image Process Viewer: on-node thumbnail like the GTE Python add-on. */
    FOREACH_NODETREE_BEGIN (bmain, node_tree, id_owner) {
      if (node_tree->type != NTREE_IMAGE) {
        continue;
      }
      for (bNode &node : node_tree->nodes) {
        if (STREQ(node.idname, "ImageNodeViewer")) {
          node.flag |= NODE_PREVIEW;
        }
      }
    }
    FOREACH_NODETREE_END;
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 43)) {
    /* Factory default: Bezier links, node-tree minimap off. */
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &sl : area.spacedata) {
          if (sl.spacetype == SPACE_NODE) {
            SpaceNode *snode = reinterpret_cast<SpaceNode *>(&sl);
            snode->overlay.link_style = SN_OVERLAY_LINK_BEZIER;
            snode->overlay.flag &= ~SN_OVERLAY_SHOW_THUMBNAIL;
          }
        }
      }
    }
  }

  /* Official 503.22–23 landed after BIKINI had already consumed those subversions.
   * Catch them up at 44–45 so existing BIKINI files still receive the official migrations. */
  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 44)) {
    for (Brush &brush : bmain->brushes) {
      if (brush.paint_flags & BRUSH_PAINT_UNUSED_1) {
        brush.paint_flags &= ~BRUSH_PAINT_UNUSED_1;
        brush.flag |= BRUSH_HARDNESS_PRESSURE;
      }
    }
  }

  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 45)) {
    for (Brush &brush : bmain->brushes) {
      clear_deprecated_brush_flags(brush);
    }
  }

  /* Official 503.24 (outliner expand-on-focus off) landed after BIKINI passed that
   * subversion. Apply it at 46 so existing BIKINI files still receive the migration. */
  if (!MAIN_VERSION_FILE_ATLEAST(bmain, 503, 46)) {
    for (bScreen &screen : bmain->screens) {
      for (ScrArea &area : screen.areabase) {
        for (SpaceLink &space : area.spacedata) {
          if (space.spacetype == SPACE_OUTLINER) {
            SpaceOutliner *space_outliner = reinterpret_cast<SpaceOutliner *>(&space);
            space_outliner->flag &= ~SO_EXPAND_ON_FOCUS;
          }
        }
      }
    }
  }

  /**
   * Always bump subversion in BKE_blender_version.h when adding versioning
   * code here, and wrap it inside a MAIN_VERSION_FILE_ATLEAST check.
   *
   * \note Keep this message at the bottom of the function.
   */

  /* Keep this versioning always enabled at the bottom of the function; it can only be moved
   * behind a subversion bump when the file format is changed (#mesh_skin_to_legacy is removed from
   * #mesh_blend_write). Since that function keeps writing the old-format #CD_MVERT_SKIN layer for
   * forward compatibility, files saved by *this* version also need
   * this conversion to run unconditionally on read, not just for files older than this subversion.
   */
  for (Mesh &mesh : bmain->meshes) {
    bke::mesh_skin_to_generic(mesh);
  }
}

}  // namespace blender
