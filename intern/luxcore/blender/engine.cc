/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Native C++ RenderEngineType for LuxCore. Not a Python addon. */

#include "LUX_api.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

#include "BLI_math_base_c.hh"
#include "BLI_listbase.hh"
#include "BLI_map.hh"
#include "BLI_math_constants.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_rotation.hh"
#include "BLI_math_vector.hh"
#include "BLI_rect.hh"
#include "BLI_string.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"

#include "DNA_camera_types.h"
#include "DNA_layer_types.h"
#include "DNA_node_types.h"
#include "DNA_object_enums.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_view3d_types.h"

#include "BKE_camera.h"
#include "BKE_context.hh"
#include "BKE_report.hh"
#include "BKE_scene.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "GPU_immediate.hh"
#include "GPU_shader.hh"
#include "GPU_shader_builtin.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"
#include "GPU_vertex_format.hh"

#include "IMB_imbuf_types.hh"

#include "RE_engine.h"
#include "RE_pipeline.h"

#include "BLT_translation.hh"

#include "nodes.hh"
#include "session.hh"
#include "sync.hh"

namespace blender {

namespace render::luxcore {

static bool camera_nearly_equal(const CameraDesc &a, const CameraDesc &b)
{
  auto close3 = [](const float3 &x, const float3 &y) {
    return math::length_squared(x - y) < 1e-8f;
  };
  return close3(a.orig, b.orig) && close3(a.target, b.target) && close3(a.up, b.up) &&
         fabsf(a.clip_start - b.clip_start) < 1e-5f && fabsf(a.clip_end - b.clip_end) < 1e-3f &&
         fabsf(a.fov_deg - b.fov_deg) < 1e-3f && a.is_ortho == b.is_ortho &&
         fabsf(a.screenwindow[0] - b.screenwindow[0]) < 1e-5f &&
         fabsf(a.screenwindow[1] - b.screenwindow[1]) < 1e-5f &&
         fabsf(a.screenwindow[2] - b.screenwindow[2]) < 1e-5f &&
         fabsf(a.screenwindow[3] - b.screenwindow[3]) < 1e-5f;
}

struct ViewportState {
  Session session;
  gpu::Texture *texture = nullptr;
  int w = 0;
  int h = 0;
  bool started = false;
  bool paused = false;
  bool camera_valid = false;
  bool use_scene_lights = true;
  bool use_scene_world = true;
  CameraDesc camera;
  Vector<float> pixels;

  ~ViewportState()
  {
    session.stop();
    if (texture) {
      GPU_texture_free(texture);
    }
  }
};

static std::mutex g_vp_mutex;
static Map<RenderEngine *, std::unique_ptr<ViewportState>> g_viewports;

static ViewportState &viewport_state(RenderEngine *engine)
{
  std::lock_guard lock(g_vp_mutex);
  return *g_viewports.lookup_or_add_cb(engine, []() { return std::make_unique<ViewportState>(); });
}

static void viewport_free(RenderEngine *engine)
{
  std::lock_guard lock(g_vp_mutex);
  g_viewports.remove(engine);
}

static bool read_view_camera(const bContext *C, Depsgraph *depsgraph, CameraDesc &cam)
{
  View3D *v3d = CTX_wm_view3d(C);
  ARegion *region = CTX_wm_region(C);
  RegionView3D *rv3d = static_cast<RegionView3D *>(CTX_wm_region_data(C));
  if (v3d == nullptr || region == nullptr || rv3d == nullptr) {
    return false;
  }

  const float width = float(max_ii(region->winx, 1));
  const float height = float(max_ii(region->winy, 1));
  Scene *scene = depsgraph ? DEG_get_evaluated_scene(depsgraph) : nullptr;
  cam.clip_start = math::max(v3d->clip_start, 1e-8f);
  cam.clip_end = v3d->clip_end;

  if (rv3d->persp == RV3D_CAMOB) {
    Object *cam_ob = v3d->camera;
    if (cam_ob == nullptr && scene) {
      cam_ob = scene->camera;
    }
    if (cam_ob && depsgraph) {
      cam_ob = DEG_get_evaluated(depsgraph, cam_ob);
    }
    if (cam_ob == nullptr) {
      return false;
    }
    lux_lookat_from_matrix(cam_ob->object_to_world(), cam);

    const Camera *bcam = (cam_ob->type == OB_CAMERA) ?
                             reinterpret_cast<const Camera *>(cam_ob->data) :
                             nullptr;
    const float zoom_div = 1.41421f + rv3d->camzoom / 50.0f;
    const float zoom_base = 4.0f / (zoom_div * zoom_div);
    float shift_x = 0.0f;
    float shift_y = 0.0f;
    int sensor_fit = CAMERA_SENSOR_FIT_AUTO;
    float scale = 1.0f;
    float zoom = zoom_base;
    cam.is_ortho = false;
    if (bcam) {
      shift_x = bcam->shiftx;
      shift_y = bcam->shifty;
      sensor_fit = bcam->sensor_fit;
      cam.clip_start = math::max(bcam->clip_start, 1e-8f);
      cam.clip_end = bcam->clip_end;
      if (bcam->type == CAM_ORTHO) {
        cam.is_ortho = true;
        const float ortho = math::max(bcam->ortho_scale, 1e-8f);
        zoom = zoom_base * 0.5f * ortho;
        scale = 0.5f * ortho;
      }
      else {
        const float sensor = BKE_camera_sensor_size(
            bcam->sensor_fit, bcam->sensor_x, bcam->sensor_y);
        cam.fov_deg = RAD2DEGF(
            2.0f * atanf(sensor / (2.0f * math::max(bcam->lens, 1e-8f))));
      }
    }
    const bool hor = BKE_camera_sensor_fit(sensor_fit, width, height) == CAMERA_SENSOR_FIT_HOR;
    float xa = 1.0f;
    float ya = 1.0f;
    lux_calc_aspect(width, height, xa, ya, hor);
    lux_calc_screenwindow(
        zoom, shift_x, shift_y, rv3d->camdx, rv3d->camdy, xa, ya, scale, cam.screenwindow);
    return true;
  }

  /* Free orbit: invert viewmat (same as Cycles) instead of trusting viewinv. */
  const float4x4 cam2world = math::invert(float4x4(rv3d->viewmat));
  lux_lookat_from_matrix(cam2world, cam);
  float xa = 1.0f;
  float ya = 1.0f;
  lux_calc_aspect(width, height, xa, ya, width > height);

  if (rv3d->persp == RV3D_ORTHO) {
    cam.is_ortho = true;
    const float3 delta = cam.orig - cam.target;
    cam.orig += delta * 50.0f;
    const float zoom = 1.0275f * rv3d->dist * 35.0f / math::max(v3d->lens, 1e-8f);
    lux_calc_screenwindow(zoom, 0.0f, 0.0f, 0.0f, 0.0f, xa, ya, 1.0f, cam.screenwindow);
  }
  else {
    cam.is_ortho = false;
    /* Viewport sensor is 32mm: fov = 2*atan(16/lens), zoom = 2.25. */
    cam.fov_deg = RAD2DEGF(2.0f * atanf(16.0f / math::max(v3d->lens, 1e-8f)));
    lux_calc_screenwindow(2.25f, 0.0f, 0.0f, 0.0f, 0.0f, xa, ya, 1.0f, cam.screenwindow);
  }
  return true;
}

static SyncOptions sync_options_from_view(const bContext *C, bool require_camera)
{
  SyncOptions opt;
  opt.require_camera = require_camera;
  View3D *v3d = CTX_wm_view3d(C);
  if (v3d == nullptr) {
    return opt;
  }
  const eView3DShading_Flag flag = v3d->shading.flag;
  if (v3d->shading.type == OB_RENDER) {
    opt.use_scene_lights = (flag & V3D_SHADING_SCENE_LIGHTS_RENDER) != 0;
    opt.use_scene_world = (flag & V3D_SHADING_SCENE_WORLD_RENDER) != 0;
  }
  else {
    opt.use_scene_lights = (flag & V3D_SHADING_SCENE_LIGHTS) != 0;
    opt.use_scene_world = (flag & V3D_SHADING_SCENE_WORLD) != 0;
  }
  opt.studiolight_intensity = v3d->shading.studiolight_intensity;
  opt.studiolight_rot_z = v3d->shading.studiolight_rot_z;
  opt.studio_light = v3d->shading.studio_light;
  return opt;
}

static void luxcore_update_passes(RenderEngine *engine, Scene *scene, ViewLayer *view_layer)
{
  if (view_layer->passflag & SCE_PASS_COMBINED) {
    RE_engine_register_pass(engine, scene, view_layer, RE_PASSNAME_COMBINED, 4, "RGBA", SOCK_RGBA);
  }
}

static void luxcore_render(RenderEngine *engine, Depsgraph *depsgraph)
{
  Scene *scene_eval = DEG_get_evaluated_scene(depsgraph);
  const ViewLayer *view_layer = DEG_get_evaluated_view_layer(depsgraph);
  if (scene_eval == nullptr || view_layer == nullptr) {
    RE_engine_report(engine, RPT_ERROR, "LuxCore: missing evaluated scene");
    return;
  }

  const int width = engine->resolution_x;
  const int height = engine->resolution_y;
  if (width <= 0 || height <= 0) {
    return;
  }

  SceneLuxCore settings{};
  memcpy(&settings, &scene_eval->luxcore, sizeof(settings));
  if (engine->flag & RE_ENGINE_PREVIEW) {
    if (settings.halt_preview_samples > 0) {
      settings.halt_samples = settings.halt_preview_samples;
    }
    else {
      settings.halt_samples = 16;
    }
  }

  Session session;
  std::string error;
  if (!session.begin_scene(width, height, settings, error, false)) {
    RE_engine_report(engine, RPT_ERROR, error.c_str());
    return;
  }

  if (!sync_scene(session, engine, depsgraph, error)) {
    RE_engine_report(engine, RPT_ERROR, error.c_str());
    return;
  }

  if (!session.render(engine, error)) {
    RE_engine_report(engine, RPT_ERROR, error.c_str());
    return;
  }

  if (RE_engine_test_break(engine)) {
    return;
  }

  RenderResult *rr = RE_engine_begin_result(
      engine, 0, 0, width, height, view_layer->name, nullptr);
  if (rr == nullptr) {
    return;
  }

  RenderLayer *rlayer = static_cast<RenderLayer *>(
      BLI_findstring(&rr->layers, view_layer->name, offsetof(RenderLayer, name)));
  if (rlayer == nullptr && !rr->layers.is_empty()) {
    rlayer = rr->layers.first();
  }
  if (rlayer) {
    for (RenderPass &rpass : rlayer->passes) {
      if (!STREQ(rpass.name, RE_PASSNAME_COMBINED) || rpass.ibuf == nullptr) {
        continue;
      }
      float *rect = rpass.ibuf->float_data_for_write();
      if (rect == nullptr) {
        continue;
      }
      MutableSpan<float> rgba(rect, int64_t(width) * int64_t(height) * 4);
      if (!session.copy_combined_rgba(rgba, error)) {
        RE_engine_report(engine, RPT_ERROR, error.c_str());
      }
    }
  }

  RE_engine_end_result(engine, rr, false, false, false);
}

static void luxcore_view_update(RenderEngine *engine, const bContext *C, Depsgraph *depsgraph)
{
  ARegion *region = CTX_wm_region(C);
  if (region == nullptr) {
    return;
  }
  const int width = max_ii(region->winx, 1);
  const int height = max_ii(region->winy, 1);

  Scene *scene_eval = DEG_get_evaluated_scene(depsgraph);
  SceneLuxCore settings{};
  if (scene_eval) {
    memcpy(&settings, &scene_eval->luxcore, sizeof(settings));
  }

  ViewportState &vp = viewport_state(engine);
  vp.session.stop();
  vp.started = false;
  vp.paused = false;
  vp.camera_valid = false;
  vp.w = width;
  vp.h = height;
  vp.pixels.resize(int64_t(width) * int64_t(height) * 4);
  for (int64_t i = 0; i < int64_t(width) * int64_t(height); i++) {
    vp.pixels[i * 4 + 0] = 0.07f;
    vp.pixels[i * 4 + 1] = 0.07f;
    vp.pixels[i * 4 + 2] = 0.08f;
    vp.pixels[i * 4 + 3] = 1.0f;
  }
  printf("LuxCore: viewport update %dx%d\n", width, height);

  std::string error;
  if (!vp.session.available()) {
    RE_engine_update_stats(engine, "LuxCore", "SDK not linked — compiling/linking luxcore.lib");
    engine->flag |= RE_ENGINE_DO_DRAW;
    return;
  }

  if (!vp.session.begin_scene(width, height, settings, error, true)) {
    RE_engine_update_stats(engine, "LuxCore", error.c_str());
    engine->flag |= RE_ENGINE_DO_DRAW;
    return;
  }
  const SyncOptions sync_opt = sync_options_from_view(C, false);
  vp.use_scene_lights = sync_opt.use_scene_lights;
  vp.use_scene_world = sync_opt.use_scene_world;
  if (!sync_scene(vp.session, engine, depsgraph, error, sync_opt)) {
    RE_engine_update_stats(engine, "LuxCore", error.c_str());
    engine->flag |= RE_ENGINE_DO_DRAW;
    return;
  }
  CameraDesc cam;
  if (!read_view_camera(C, depsgraph, cam)) {
    RE_engine_update_stats(engine, "LuxCore", "LuxCore: no viewport camera");
    engine->flag |= RE_ENGINE_DO_DRAW;
    return;
  }
  vp.session.set_camera(cam);
  vp.camera = cam;
  vp.camera_valid = true;
  printf("LuxCore: view cam orig=(%.4f %.4f %.4f) tgt=(%.4f %.4f %.4f) up=(%.4f %.4f %.4f) "
         "fov=%.3f ortho=%d sw=[%.4f %.4f %.4f %.4f]\n",
         cam.orig.x,
         cam.orig.y,
         cam.orig.z,
         cam.target.x,
         cam.target.y,
         cam.target.z,
         cam.up.x,
         cam.up.y,
         cam.up.z,
         cam.fov_deg,
         int(cam.is_ortho),
         cam.screenwindow[0],
         cam.screenwindow[1],
         cam.screenwindow[2],
         cam.screenwindow[3]);
  if (!vp.session.start(error)) {
    RE_engine_update_stats(engine, "LuxCore", error.c_str());
    engine->flag |= RE_ENGINE_DO_DRAW;
    return;
  }
  vp.started = true;
  RE_engine_update_stats(engine, "LuxCore", "LuxCore viewport");
  engine->flag |= RE_ENGINE_DO_DRAW;
}

static void luxcore_view_draw(RenderEngine *engine, const bContext *C, Depsgraph *depsgraph)
{
  ARegion *region = CTX_wm_region(C);
  if (region == nullptr) {
    return;
  }
  const int width = max_ii(region->winx, 1);
  const int height = max_ii(region->winy, 1);

  ViewportState &vp = viewport_state(engine);

  const SyncOptions shading = sync_options_from_view(C, false);
  if ((vp.w != width || vp.h != height || vp.use_scene_lights != shading.use_scene_lights ||
       vp.use_scene_world != shading.use_scene_world) &&
      depsgraph != nullptr)
  {
    luxcore_view_update(engine, C, depsgraph);
  }

  if (vp.pixels.is_empty()) {
    vp.pixels.resize(int64_t(max_ii(vp.w, 1)) * int64_t(max_ii(vp.h, 1)) * 4, 0.0f);
  }

  std::string error;
  if (vp.started && !vp.paused) {
    CameraDesc cam;
    if (read_view_camera(C, depsgraph, cam) &&
        (!vp.camera_valid || !camera_nearly_equal(cam, vp.camera)))
    {
      vp.session.update_camera_interactive(cam);
      vp.camera = cam;
      vp.camera_valid = true;
    }
    if (!vp.session.copy_combined_rgba(vp.pixels, error) && !error.empty()) {
      RE_engine_update_stats(engine, "LuxCore", error.c_str());
    }
    else {
      const unsigned pass = vp.session.pass_count();
      char info[256];
      SNPRINTF(info, "LuxCore viewport  Samples: %u", pass);
      RE_engine_update_stats(engine, "LuxCore", info);
    }
    engine->flag |= RE_ENGINE_DO_DRAW;
  }
  else if (!vp.started) {
    RE_engine_update_stats(engine, "LuxCore", "LuxCore kernel starting…");
    engine->flag |= RE_ENGINE_DO_DRAW;
  }

  const int tex_w = vp.w > 0 ? vp.w : width;
  const int tex_h = vp.h > 0 ? vp.h : height;
  if (vp.texture && (GPU_texture_width(vp.texture) != tex_w ||
                     GPU_texture_height(vp.texture) != tex_h))
  {
    GPU_texture_free(vp.texture);
    vp.texture = nullptr;
  }
  if (vp.texture == nullptr) {
    vp.texture = GPU_texture_create_2d("luxcore_viewport",
                                       tex_w,
                                       tex_h,
                                       1,
                                       gpu::TextureFormat::SFLOAT_32_32_32_32,
                                       GPU_TEXTURE_USAGE_GENERAL,
                                       nullptr);
  }
  if (vp.texture && !vp.pixels.is_empty()) {
    GPU_texture_update(vp.texture, GPU_DATA_FLOAT, vp.pixels.data());
  }

  GPU_depth_test(GPU_DEPTH_NONE);
  GPU_blend(GPU_BLEND_NONE);

  if (vp.texture == nullptr) {
    return;
  }

  GPUVertFormat *format = immVertexFormat();
  const uint pos = GPU_vertformat_attr_add(format, "pos", gpu::VertAttrType::SFLOAT_32_32);
  const uint tex = GPU_vertformat_attr_add(format, "texCoord", gpu::VertAttrType::SFLOAT_32_32);
  immBindBuiltinProgram(GPU_SHADER_3D_IMAGE);
  immBindTexture("image", vp.texture);

  /* Region pixel space: (0,0) is the lower-left of the 3D View, not the window. */
  immBegin(GPU_PRIM_TRI_FAN, 4);
  immAttr2f(tex, 0.0f, 0.0f);
  immVertex2f(pos, 0.0f, 0.0f);
  immAttr2f(tex, 1.0f, 0.0f);
  immVertex2f(pos, float(width), 0.0f);
  immAttr2f(tex, 1.0f, 1.0f);
  immVertex2f(pos, float(width), float(height));
  immAttr2f(tex, 0.0f, 1.0f);
  immVertex2f(pos, 0.0f, float(height));
  immEnd();

  immUnbindProgram();
}

static void luxcore_view_pause(RenderEngine *engine, const bContext * /*C*/)
{
  ViewportState &vp = viewport_state(engine);
  vp.session.pause();
  vp.paused = true;
}

static void luxcore_view_resume(RenderEngine *engine, const bContext * /*C*/)
{
  ViewportState &vp = viewport_state(engine);
  vp.session.resume();
  vp.paused = false;
  engine->flag |= RE_ENGINE_DO_DRAW;
}

}  // namespace render::luxcore

static RenderEngineType LUX_engine_type = {
    /*next*/ nullptr,
    /*prev*/ nullptr,
    /*idname*/ "LUXCORE",
    /*name*/ N_("LuxCore"),
    /* Keep RE_INTERNAL: the type is a static object, RE_engines_exit must not
     * MEM_delete it. Viewport routing for intern engines with view_draw is in
     * DRWContext::enable_engines. */
    /*flag*/ RE_INTERNAL | RE_USE_PREVIEW | RE_USE_EEVEE_VIEWPORT | RE_USE_POSTPROCESS,
    /*update*/ nullptr,
    /*render*/ &render::luxcore::luxcore_render,
    /*render_frame_finish*/ nullptr,
    /*draw*/ nullptr,
    /*bake*/ nullptr,
    /*view_update*/ &render::luxcore::luxcore_view_update,
    /*view_draw*/ &render::luxcore::luxcore_view_draw,
    /*view_pause*/ &render::luxcore::luxcore_view_pause,
    /*view_resume*/ &render::luxcore::luxcore_view_resume,
    /*update_script_node*/ nullptr,
    /*update_render_passes*/ &render::luxcore::luxcore_update_passes,
    /*update_custom_camera*/ nullptr,
    /*draw_engine*/ nullptr,
    /*rna_ext*/
    {
        /*data*/ nullptr,
        /*srna*/ nullptr,
        /*call*/ nullptr,
    },
};

void LUX_engines_register()
{
  LUX_nodes_register();
  RE_engines_register(&LUX_engine_type);
}

void LUX_engines_exit()
{
  std::lock_guard lock(render::luxcore::g_vp_mutex);
  render::luxcore::g_viewports.clear();
}

void LUX_view_engine_free(RenderEngine *engine)
{
  render::luxcore::viewport_free(engine);
}

}  // namespace blender
