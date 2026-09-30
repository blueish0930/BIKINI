/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "session.hh"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "DNA_scene_types.h"

#include "BLI_math_base_c.hh"
#include "BLI_math_vector.hh"
#include "BLI_string.hh"
#include "BLI_time.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"

#include "RE_engine.h"

#include <chrono>

#ifdef WITH_LUXCORE_SDK
#  include <luxcore/luxcore.h>
#endif

namespace blender::render::luxcore {

#ifdef WITH_LUXCORE_SDK

using Film = ::luxcore::Film;
using RenderConfig = ::luxcore::RenderConfig;
using RenderSession = ::luxcore::RenderSession;
using Scene = ::luxcore::Scene;
using Properties = ::luxrays::Properties;
using Property = ::luxrays::Property;

static void luxcore_log(const char *msg)
{
  if (msg && msg[0]) {
    printf("LuxCore: %s\n", msg);
  }
}

static const char *engine_type_name(eSceneLuxCore_Engine type)
{
  switch (type) {
    case SCE_LUXCORE_PATHOCL:
      return "PATHOCL";
    case SCE_LUXCORE_BIDIRCPU:
      return "BIDIRCPU";
    case SCE_LUXCORE_TILEPATHCPU:
      return "TILEPATHCPU";
    case SCE_LUXCORE_TILEPATHOCL:
      return "TILEPATHOCL";
    case SCE_LUXCORE_PATHCPU:
    default:
      return "PATHCPU";
  }
}

static const char *sampler_type_name(eSceneLuxCore_Sampler type)
{
  switch (type) {
    case SCE_LUXCORE_SAMPLER_METROPOLIS:
      return "METROPOLIS";
    case SCE_LUXCORE_SAMPLER_RANDOM:
      return "RANDOM";
    case SCE_LUXCORE_SAMPLER_SOBOL:
    default:
      return "SOBOL";
  }
}

static const char *light_strategy_name(eSceneLuxCore_LightStrategy type)
{
  switch (type) {
    case SCE_LUXCORE_LIGHT_POWER:
      return "POWER";
    case SCE_LUXCORE_LIGHT_UNIFORM:
      return "UNIFORM";
    case SCE_LUXCORE_LIGHT_LOG_POWER:
    default:
      return "LOG_POWER";
  }
}

static const char *filter_type_name(eSceneLuxCore_Filter type)
{
  switch (type) {
    case SCE_LUXCORE_FILTER_MITCHELL:
      return "MITCHELL_SS";
    case SCE_LUXCORE_FILTER_GAUSSIAN:
      return "GAUSSIAN";
    case SCE_LUXCORE_FILTER_BOX:
      return "BOX";
    case SCE_LUXCORE_FILTER_BLACKMANHARRIS:
      return "BLACKMANHARRIS";
    case SCE_LUXCORE_FILTER_NONE:
    default:
      return "NONE";
  }
}

static int lux_depth_or(int value, int fallback)
{
  return value > 0 ? value : fallback;
}

static unsigned int lux_thread_count(const bool interactive)
{
  unsigned int hw = std::thread::hardware_concurrency();
  if (hw == 0) {
    hw = 4;
  }
  /* Leave cores for Blender's UI / GPU driver. PATHCPU default is "use every
   * hardware thread", which freezes the viewport on the main thread. */
  if (interactive) {
    return hw <= 2 ? 1u : hw - 2;
  }
  return hw > 1 ? hw - 1 : 1u;
}

struct Session::Impl {
  ::luxcore::ScenePtr scene;
  ::luxcore::RenderConfigRPtr config;
  ::luxcore::RenderSessionRPtr session;
  std::unique_ptr<Properties> cfg_props;
  Vector<float> rgb_scratch;
  bool inited = false;
  bool interactive = false;

  bool ensure_init(std::string &r_error)
  {
    if (inited) {
      return true;
    }
    try {
      ::luxcore::Init(luxcore_log);
      inited = true;
      return true;
    }
    catch (const std::exception &ex) {
      r_error = ex.what();
      return false;
    }
  }

  bool parse(Properties &&props, std::string &r_error)
  {
    try {
      auto ptr = std::make_unique<Properties>(std::move(props));
      scene->Parse(ptr);
      return true;
    }
    catch (const std::exception &ex) {
      r_error = ex.what();
      return false;
    }
  }

  bool ensure_config(std::string &r_error)
  {
    if (config) {
      return true;
    }
    if (!scene || !cfg_props) {
      r_error = "LuxCore scene is not initialized";
      return false;
    }
    try {
      config = RenderConfig::Create(std::move(cfg_props), scene);
    }
    catch (const std::exception &ex) {
      r_error = ex.what();
      return false;
    }
    return true;
  }
};

Session::Session() = default;
Session::~Session() = default;

bool Session::available() const
{
  return true;
}

bool Session::begin_scene(int width,
                          int height,
                          const SceneLuxCore &settings,
                          std::string &r_error,
                          bool interactive)
{
  impl_ = std::make_unique<Impl>();
  if (!impl_->ensure_init(r_error)) {
    impl_.reset();
    return false;
  }

  width_ = width;
  height_ = height;
  impl_->interactive = interactive;
  if (interactive) {
    halt_samples_ = 0;
    halt_time_ = 0;
  }
  else {
    halt_samples_ = settings.halt_samples > 0 ? settings.halt_samples : 64;
    halt_time_ = settings.halt_time > 0 ? settings.halt_time : 0;
  }
  const int path_depth = lux_depth_or(settings.path_depth, 12);
  const int depth_diff = lux_depth_or(settings.path_depth_diffuse, std::min(path_depth, 4));
  const int depth_gloss = lux_depth_or(settings.path_depth_glossy, std::min(path_depth, 4));
  const int depth_spec = lux_depth_or(settings.path_depth_specular, path_depth);
  const unsigned int threads = settings.threads > 0 ?
                                   unsigned(settings.threads) :
                                   lux_thread_count(interactive);
  const bool is_bidir = settings.engine_type == SCE_LUXCORE_BIDIRCPU;
  const bool is_tile = ELEM(
      settings.engine_type, SCE_LUXCORE_TILEPATHCPU, SCE_LUXCORE_TILEPATHOCL);
  const bool photongi = !interactive && !is_bidir &&
                        (settings.flag & SCE_LUXCORE_PHOTONGI) != 0;
  const bool photongi_caustic = photongi &&
                                (settings.flag & SCE_LUXCORE_PHOTONGI_CAUSTIC) != 0;
  const bool photongi_indirect = photongi &&
                                 (settings.flag & SCE_LUXCORE_PHOTONGI_INDIRECT) != 0;
  /* PhotonGI caustic lookup at camera-hit (depth 0) is skipped when hybrid is
   * on. Prefer PhotonGI for final if both are requested. */
  bool hybrid = !is_bidir && !is_tile && (settings.flag & SCE_LUXCORE_HYBRID_BF) != 0;
  if (interactive) {
    hybrid = !is_bidir && (settings.flag & SCE_LUXCORE_VIEWPORT_LIGHTTRACE) != 0;
  }
  else if (hybrid && photongi_caustic) {
    hybrid = false;
  }
  const char *sampler = sampler_type_name(settings.sampler_type);
  if (photongi && settings.sampler_type == SCE_LUXCORE_SAMPLER_METROPOLIS) {
    /* Path-tracer Metropolis stalls on the first sample while PhotonGI builds. */
    sampler = "SOBOL";
  }
  if (interactive) {
    sampler = "SOBOL";
  }

  try {
    impl_->scene = Scene::Create();
    impl_->scene->SetDeleteMeshData(true);

    auto cfg = std::make_unique<Properties>();
    *cfg << Property("renderengine.type")(interactive ? "PATHCPU" :
                                                        engine_type_name(settings.engine_type));
    *cfg << Property("sampler.type")(sampler);
    *cfg << Property("native.threads.count")(int(threads));
    *cfg << Property("renderengine.seed")(std::max(settings.seed, 1));
    *cfg << Property("lightstrategy.type")(light_strategy_name(settings.light_strategy));

    if (is_bidir && !interactive) {
      *cfg << Property("path.maxdepth")(lux_depth_or(settings.bidir_eye_depth, 10));
      *cfg << Property("light.maxdepth")(lux_depth_or(settings.bidir_light_depth, 10));
    }
    else {
      const int vp_depth = interactive ? std::max(8, std::min(path_depth, 12)) : path_depth;
      *cfg << Property("path.pathdepth.total")(vp_depth);
      *cfg << Property("path.pathdepth.diffuse")(interactive ? std::min(vp_depth, depth_diff) :
                                                               depth_diff);
      *cfg << Property("path.pathdepth.glossy")(interactive ? std::min(vp_depth, depth_gloss) :
                                                              depth_gloss);
      *cfg << Property("path.pathdepth.specular")(interactive ? std::max(vp_depth, depth_spec) :
                                                                depth_spec);
    }

    *cfg << Property("film.width")(width_);
    *cfg << Property("film.height")(height_);
    *cfg << Property("film.outputs.0.type")("RGB");
    *cfg << Property("film.outputs.0.filename")("dummy_rgb.exr");
    *cfg << Property("film.imagepipeline.0.type")("TONEMAP_LINEAR");
    *cfg << Property("film.imagepipeline.0.scale")(
        settings.tonemap_scale > 0.0f ? settings.tonemap_scale : 1.0f);
    *cfg << Property("batch.haltspp")(interactive ? 0 : halt_samples_);
    *cfg << Property("batch.halttime")(interactive ? 0 : halt_time_);

    if (STREQ(sampler, "SOBOL") || STREQ(sampler, "RANDOM")) {
      *cfg << Property("sampler.sobol.adaptive.strength")(
          interactive ? 0.0f : clamp_f(settings.sobol_adaptive, 0.0f, 0.95f));
    }
    if (STREQ(sampler, "METROPOLIS")) {
      *cfg << Property("sampler.metropolis.largesteprate")(
          clamp_f(settings.metropolis_largestep, 0.0f, 1.0f));
    }

    *cfg << Property("path.hybridbackforward.enable")(hybrid ? 1 : 0);
    if (hybrid) {
      const float part = settings.hybrid_partition > 0.0f ? settings.hybrid_partition : 0.6f;
      *cfg << Property("path.hybridbackforward.partition")(clamp_f(part, 0.0f, 1.0f));
      *cfg << Property("path.hybridbackforward.glossinessthreshold")(
          settings.hybrid_glossiness > 0.0f ? settings.hybrid_glossiness : 0.049f);
    }

    if (photongi && (photongi_caustic || photongi_indirect)) {
      const int photon_count = std::max(
          1000, int(std::max(settings.photongi_photon_millions, 0.01f) * 1.0e6f));
      const int caustic_size = std::max(
          1000, int(std::max(settings.photongi_caustic_millions, 0.001f) * 1.0e6f));
      *cfg << Property("path.photongi.indirect.enabled")(photongi_indirect ? 1 : 0);
      *cfg << Property("path.photongi.caustic.enabled")(photongi_caustic ? 1 : 0);
      *cfg << Property("path.photongi.photon.maxcount")(photon_count);
      *cfg << Property("path.photongi.photon.maxdepth")(
          lux_depth_or(settings.photongi_photon_depth, 6));
      *cfg << Property("path.photongi.glossinessusagethreshold")(
          settings.photongi_glossiness > 0.0f ? settings.photongi_glossiness : 0.049f);
      *cfg << Property("path.photongi.caustic.maxsize")(caustic_size);
      *cfg << Property("path.photongi.caustic.lookup.radius")(
          settings.photongi_caustic_radius > 0.0f ? settings.photongi_caustic_radius : 0.075f);
      *cfg << Property("path.photongi.caustic.lookup.normalangle")(10.0f);
      *cfg << Property("path.photongi.caustic.updatespp")(
          std::max(settings.photongi_caustic_updatespp, 0));
      *cfg << Property("path.photongi.sampler.type")("METROPOLIS");
      *cfg << Property("path.photongi.debug.type")("none");
    }
    else {
      *cfg << Property("path.photongi.caustic.enabled")(0);
      *cfg << Property("path.photongi.indirect.enabled")(0);
    }

    if ((settings.flag & SCE_LUXCORE_FILTER) != 0 &&
        settings.filter_type != SCE_LUXCORE_FILTER_NONE)
    {
      *cfg << Property("film.filter.type")(filter_type_name(settings.filter_type));
      *cfg << Property("film.filter.width")(
          settings.filter_width > 0.5f ? settings.filter_width : 1.5f);
    }
    else {
      *cfg << Property("film.filter.type")("NONE");
    }

    if (is_tile && !interactive) {
      *cfg << Property("tile.size")(std::max(settings.tile_size, 16));
      *cfg << Property("tilepath.sampling.aa.size")(std::max(settings.tile_aa, 1));
    }

    if ((settings.flag & SCE_LUXCORE_CLAMP) != 0) {
      if (settings.clamp_direct > 0.0f) {
        *cfg << Property("path.clamping.direct")(settings.clamp_direct);
      }
      if (settings.clamp_indirect > 0.0f) {
        *cfg << Property("path.clamping.indirect")(settings.clamp_indirect);
      }
    }
    if (ELEM(settings.engine_type, SCE_LUXCORE_PATHOCL, SCE_LUXCORE_TILEPATHOCL) && !interactive) {
      const bool use_cpu = (settings.flag & SCE_LUXCORE_USE_CPU_OPENCL) != 0;
      *cfg << Property("opencl.cpu.use")(use_cpu);
      *cfg << Property("opencl.gpu.use")(true);
    }
    /* RenderConfig requires a camera. Created later in start()/render(). */
    impl_->cfg_props = std::move(cfg);
  }
  catch (const std::exception &ex) {
    r_error = ex.what();
    impl_.reset();
    return false;
  }
  return true;
}

void Session::set_camera(const CameraDesc &cam)
{
  if (!impl_ || !impl_->scene) {
    return;
  }
  Properties props;
  props << Property("scene.camera.type")(cam.is_ortho ? "orthographic" : "perspective");
  props << Property("scene.camera.lookat.orig")(cam.orig.x, cam.orig.y, cam.orig.z);
  props << Property("scene.camera.lookat.target")(cam.target.x, cam.target.y, cam.target.z);
  props << Property("scene.camera.up")(cam.up.x, cam.up.y, cam.up.z);
  /* BlendLuxCore: real FOV + screenwindow (not the 90°/viewplane trick). */
  if (!cam.is_ortho) {
    float fov = cam.fov_deg;
    if (!(fov > 0.05f) || fov >= 179.9f) {
      fov = 45.0f;
    }
    props << Property("scene.camera.fieldofview")(fov);
  }
  props << Property("scene.camera.screenwindow")(
      cam.screenwindow[0], cam.screenwindow[1], cam.screenwindow[2], cam.screenwindow[3]);
  props << Property("scene.camera.cliphither")(cam.clip_start);
  props << Property("scene.camera.clipyon")(cam.clip_end);
  std::string error;
  impl_->parse(std::move(props), error);
}

static void lux_set_common_mat(Properties &props, const Session::MaterialParams &p)
{
  const std::string pre = "scene.materials." + p.name + ".";
  if (p.opacity < 0.999f) {
    props << Property(pre + "transparency")(p.opacity);
  }
  if (p.emission.x > 0.0f || p.emission.y > 0.0f || p.emission.z > 0.0f) {
    props << Property(pre + "emission")(p.emission.x, p.emission.y, p.emission.z);
  }
  if (!p.interior_vol.empty()) {
    props << Property(pre + "volume.interior")(p.interior_vol);
  }
  if (!p.exterior_vol.empty()) {
    props << Property(pre + "volume.exterior")(p.exterior_vol);
  }
  if (p.filmthickness > 1.0e-4f) {
    props << Property(pre + "filmthickness")(p.filmthickness);
    props << Property(pre + "filmior")(p.filmior);
  }
}

void Session::add_material(const MaterialParams &p)
{
  if (!impl_ || !impl_->scene || p.name.empty()) {
    return;
  }
  Properties props;
  const std::string pre = "scene.materials." + p.name + ".";
  const float urough = p.uroughness >= 0.0f ? p.uroughness : p.roughness;
  const float vrough = p.vroughness >= 0.0f ? p.vroughness : urough;
  std::string type = p.type;

  if (type == "matte" && p.sigma > 1.0e-6f) {
    type = "roughmatte";
  }

  if (type == "glass" && p.interior_vol.empty()) {
    /* Nested dielectric: a matching clear volume keeps photon and eye
     * refraction on the same path (BlendLuxCore glass without a linked volume
     * still sets interiorior; intern also injects a clear volume). */
    const std::string vol = p.name + "_intvol";
    props << Property("scene.volumes." + vol + ".type")("clear");
    props << Property("scene.volumes." + vol + ".absorption")(0.0f, 0.0f, 0.0f);
    props << Property("scene.volumes." + vol + ".ior")(p.ior);
    props << Property(pre + "volume.interior")(vol);
  }

  props << Property(pre + "type")(type);

  if (type == "disney") {
    if (p.tex_name.empty()) {
      props << Property(pre + "basecolor")(p.kd.x, p.kd.y, p.kd.z);
    }
    else {
      props << Property(pre + "basecolor")(p.tex_name);
    }
    props << Property(pre + "subsurface")(p.subsurface);
    props << Property(pre + "metallic")(p.metallic);
    props << Property(pre + "specular")(p.specular);
    props << Property(pre + "specularvalue")(p.specular);
    props << Property(pre + "speculartint")(p.speculartint);
    props << Property(pre + "roughness")(p.roughness);
    props << Property(pre + "anisotropic")(p.anisotropic);
    props << Property(pre + "sheen")(p.sheen);
    props << Property(pre + "sheentint")(p.sheentint);
    props << Property(pre + "clearcoat")(p.clearcoat);
    props << Property(pre + "clearcoatgloss")(p.clearcoatgloss);
    if (p.filmthickness > 1.0e-4f) {
      props << Property(pre + "filmamount")(p.filmamount);
    }
  }
  else if (type == "matte" || type == "roughmatte") {
    props << Property(pre + "kd")(p.kd.x, p.kd.y, p.kd.z);
    if (type == "roughmatte") {
      props << Property(pre + "sigma")(p.sigma);
    }
  }
  else if (type == "glass" || type == "roughglass" || type == "archglass") {
    props << Property(pre + "kt")(p.kt.x, p.kt.y, p.kt.z);
    props << Property(pre + "kr")(p.kr.x, p.kr.y, p.kr.z);
    if (p.interior_vol.empty()) {
      props << Property(pre + "interiorior")(p.ior);
    }
    props << Property(pre + "exteriorior")(1.0f);
    /* Official BlendLuxCore greys Dispersion on rough/arch because stock
     * LuxCore roughglass ignored cauchyb. Intern kernel now samples Cauchy B
     * on roughglass too, so keep exporting it. */
    if ((type == "glass" || type == "roughglass") && p.cauchyb > 1.0e-8f) {
      props << Property(pre + "cauchyb")(p.cauchyb);
    }
    if (type == "roughglass") {
      props << Property(pre + "uroughness")(urough);
      props << Property(pre + "vroughness")(vrough);
    }
  }
  else if (type == "metal2") {
    if (p.use_n_k) {
      props << Property(pre + "n")(p.n.x, p.n.y, p.n.z);
      props << Property(pre + "k")(p.k.x, p.k.y, p.k.z);
    }
    else {
      const std::string tex = p.name + "_fresnel";
      props << Property("scene.textures." + tex + ".type")("fresnelcolor");
      props << Property("scene.textures." + tex + ".kr")(p.kd.x, p.kd.y, p.kd.z);
      props << Property(pre + "fresnel")(tex);
    }
    props << Property(pre + "uroughness")(urough);
    props << Property(pre + "vroughness")(vrough);
  }
  else if (type == "mirror") {
    props << Property(pre + "kr")(p.kr.x, p.kr.y, p.kr.z);
  }
  else if (type == "glossy2") {
    props << Property(pre + "kd")(p.kd.x, p.kd.y, p.kd.z);
    if (p.use_ior) {
      props << Property(pre + "index")(p.ior);
      props << Property(pre + "ks")(1.0f, 1.0f, 1.0f);
    }
    else {
      props << Property(pre + "ks")(p.ks.x, p.ks.y, p.ks.z);
    }
    props << Property(pre + "ka")(p.ka.x, p.ka.y, p.ka.z);
    props << Property(pre + "d")(p.absorption_depth);
    props << Property(pre + "multibounce")(p.multibounce);
    props << Property(pre + "uroughness")(urough);
    props << Property(pre + "vroughness")(vrough);
  }
  else if (type == "glossycoating") {
    props << Property(pre + "base")(p.base.empty() ? "mat_default" : p.base);
    if (p.use_ior) {
      props << Property(pre + "index")(p.ior);
      props << Property(pre + "ks")(1.0f, 1.0f, 1.0f);
    }
    else {
      props << Property(pre + "ks")(p.ks.x, p.ks.y, p.ks.z);
    }
    props << Property(pre + "ka")(p.ka.x, p.ka.y, p.ka.z);
    props << Property(pre + "d")(p.absorption_depth);
    props << Property(pre + "multibounce")(p.multibounce);
    props << Property(pre + "uroughness")(urough);
    props << Property(pre + "vroughness")(vrough);
  }
  else if (type == "glossytranslucent") {
    props << Property(pre + "kd")(p.kd.x, p.kd.y, p.kd.z);
    props << Property(pre + "kt")(p.kt.x, p.kt.y, p.kt.z);
    if (p.use_ior) {
      props << Property(pre + "index")(p.ior);
      props << Property(pre + "index_bf")(p.ior_bf);
      props << Property(pre + "ks")(1.0f, 1.0f, 1.0f);
      props << Property(pre + "ks_bf")(1.0f, 1.0f, 1.0f);
    }
    else {
      props << Property(pre + "ks")(p.ks.x, p.ks.y, p.ks.z);
      props << Property(pre + "ks_bf")(p.ks_bf.x, p.ks_bf.y, p.ks_bf.z);
    }
    props << Property(pre + "ka")(p.ka.x, p.ka.y, p.ka.z);
    props << Property(pre + "ka_bf")(p.ka_bf.x, p.ka_bf.y, p.ka_bf.z);
    props << Property(pre + "d")(p.absorption_depth);
    props << Property(pre + "d_bf")(p.absorption_depth_bf);
    props << Property(pre + "multibounce")(p.multibounce);
    props << Property(pre + "uroughness")(urough);
    props << Property(pre + "vroughness")(vrough);
  }
  else if (type == "mattetranslucent") {
    props << Property(pre + "kr")(p.kr.x, p.kr.y, p.kr.z);
    props << Property(pre + "kt")(p.kt.x, p.kt.y, p.kt.z);
  }
  else if (type == "null") {
    if (p.kt.x < 0.999f || p.kt.y < 0.999f || p.kt.z < 0.999f) {
      props << Property(pre + "transparency")(p.kt.x, p.kt.y, p.kt.z);
    }
  }
  else if (type == "velvet") {
    props << Property(pre + "kd")(p.kd.x, p.kd.y, p.kd.z);
    props << Property(pre + "thickness")(p.thickness);
    if (p.velvet_advanced) {
      props << Property(pre + "p1")(p.p1);
      props << Property(pre + "p2")(p.p2);
      props << Property(pre + "p3")(p.p3);
    }
  }
  else if (type == "carpaint") {
    if (!p.carpaint_preset.empty() && p.carpaint_preset != "manual") {
      props << Property(pre + "preset")(p.carpaint_preset);
    }
    else {
      props << Property(pre + "kd")(p.kd.x, p.kd.y, p.kd.z);
      props << Property(pre + "ks1")(p.ks1.x, p.ks1.y, p.ks1.z);
      props << Property(pre + "ks2")(p.ks2.x, p.ks2.y, p.ks2.z);
      props << Property(pre + "ks3")(p.ks3.x, p.ks3.y, p.ks3.z);
      props << Property(pre + "r1")(p.r1);
      props << Property(pre + "r2")(p.r2);
      props << Property(pre + "r3")(p.r3);
      props << Property(pre + "m1")(p.m1);
      props << Property(pre + "m2")(p.m2);
      props << Property(pre + "m3")(p.m3);
    }
    props << Property(pre + "ka")(p.ka.x, p.ka.y, p.ka.z);
    props << Property(pre + "d")(p.absorption_depth);
  }
  else if (type == "cloth") {
    props << Property(pre + "preset")(p.cloth_preset);
    props << Property(pre + "warp_kd")(p.warp_kd.x, p.warp_kd.y, p.warp_kd.z);
    props << Property(pre + "warp_ks")(p.warp_ks.x, p.warp_ks.y, p.warp_ks.z);
    props << Property(pre + "weft_kd")(p.weft_kd.x, p.weft_kd.y, p.weft_kd.z);
    props << Property(pre + "weft_ks")(p.weft_ks.x, p.weft_ks.y, p.weft_ks.z);
    props << Property(pre + "repeat_u")(p.repeat_u);
    props << Property(pre + "repeat_v")(p.repeat_v);
  }
  else if (type == "mix") {
    props << Property(pre + "material1")(p.mix1.empty() ? "mat_default" : p.mix1);
    props << Property(pre + "material2")(p.mix2.empty() ? "mat_default" : p.mix2);
    props << Property(pre + "amount")(p.mix_amount);
  }
  else if (type == "twosided") {
    props << Property(pre + "frontmaterial")(p.mix1.empty() ? "mat_default" : p.mix1);
    props << Property(pre + "backmaterial")(p.mix2.empty() ? "mat_default" : p.mix2);
  }
  else {
    /* Fallback: disney. */
    props << Property(pre + "type")("disney");
    props << Property(pre + "basecolor")(p.kd.x, p.kd.y, p.kd.z);
    props << Property(pre + "metallic")(p.metallic);
    props << Property(pre + "roughness")(p.roughness);
    props << Property(pre + "specular")(p.specular);
  }

  lux_set_common_mat(props, p);
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::add_volume(const VolumeParams &p)
{
  if (!impl_ || !impl_->scene || p.name.empty()) {
    return;
  }
  Properties props;
  const std::string pre = "scene.volumes." + p.name + ".";
  props << Property(pre + "type")(p.type);
  props << Property(pre + "absorption")(p.absorption.x, p.absorption.y, p.absorption.z);
  props << Property(pre + "ior")(p.ior);
  props << Property(pre + "priority")(p.priority);
  if (p.emission.x > 0.0f || p.emission.y > 0.0f || p.emission.z > 0.0f) {
    props << Property(pre + "emission")(p.emission.x, p.emission.y, p.emission.z);
  }
  if (p.type == "homogeneous" || p.type == "heterogeneous") {
    const float ss = p.scattering_scale;
    props << Property(pre + "scattering")(
        p.scattering.x * ss, p.scattering.y * ss, p.scattering.z * ss);
    props << Property(pre + "asymmetry")(p.asymmetry.x, p.asymmetry.y, p.asymmetry.z);
    props << Property(pre + "multiscattering")(p.multiscattering);
  }
  if (p.type == "heterogeneous") {
    props << Property(pre + "steps.size")(p.step_size);
    props << Property(pre + "steps.maxcount")(p.maxcount);
  }
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::set_material_volumes(const std::string &name,
                                   const std::string &interior,
                                   const std::string &exterior)
{
  if (!impl_ || !impl_->scene || name.empty()) {
    return;
  }
  Properties props;
  const std::string pre = "scene.materials." + name + ".";
  if (!interior.empty()) {
    props << Property(pre + "volume.interior")(interior);
  }
  if (!exterior.empty()) {
    props << Property(pre + "volume.exterior")(exterior);
  }
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::patch_transparency(const std::string &name, float front, float back)
{
  if (!impl_ || !impl_->scene || name.empty()) {
    return;
  }
  Properties props;
  const std::string pre = "scene.materials." + name + ".";
  props << Property(pre + "transparency.front")(front);
  props << Property(pre + "transparency.back")(back);
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::add_matte(const std::string &name, const float3 kd, const float3 emission)
{
  MaterialParams p;
  p.name = name;
  p.type = "matte";
  p.kd = kd;
  p.emission = emission;
  add_material(p);
}

void Session::add_disney(const std::string &name,
                         const float3 base_color,
                         float metallic,
                         float roughness,
                         float specular,
                         float /*ior*/,
                         const std::string &kd_tex)
{
  MaterialParams p;
  p.name = name;
  p.type = "disney";
  p.kd = base_color;
  p.metallic = metallic;
  p.roughness = roughness;
  p.specular = specular;
  p.tex_name = kd_tex;
  add_material(p);
}

void Session::add_glass(const std::string &name, const float3 kt, float ior, float cauchyb)
{
  MaterialParams p;
  p.name = name;
  p.type = "glass";
  p.kt = kt;
  p.ior = ior;
  p.cauchyb = cauchyb;
  add_material(p);
}

void Session::add_metal(const std::string &name, const float3 n, const float3 k, float roughness)
{
  MaterialParams p;
  p.name = name;
  p.type = "metal2";
  p.n = n;
  p.k = k;
  p.roughness = roughness;
  p.use_n_k = true;
  add_material(p);
}

void Session::add_mirror(const std::string &name, const float3 kr)
{
  MaterialParams p;
  p.name = name;
  p.type = "mirror";
  p.kr = kr;
  add_material(p);
}

void Session::add_glossy(const std::string &name, const float3 kd, const float3 ks, float roughness)
{
  MaterialParams p;
  p.name = name;
  p.type = "glossy2";
  p.kd = kd;
  p.ks = ks;
  p.roughness = roughness;
  add_material(p);
}

void Session::add_null(const std::string &name)
{
  MaterialParams p;
  p.name = name;
  p.type = "null";
  add_material(p);
}

void Session::add_velvet(const std::string &name, const float3 kd)
{
  MaterialParams p;
  p.name = name;
  p.type = "velvet";
  p.kd = kd;
  add_material(p);
}

void Session::add_matte_translucent(const std::string &name, const float3 kd, const float3 kt)
{
  MaterialParams p;
  p.name = name;
  p.type = "mattetranslucent";
  p.kr = kd;
  p.kt = kt;
  add_material(p);
}

void Session::add_roughglass(const std::string &name, const float3 kt, float ior, float roughness)
{
  MaterialParams p;
  p.name = name;
  p.type = "roughglass";
  p.kt = kt;
  p.ior = ior;
  p.roughness = roughness;
  add_material(p);
}

void Session::add_archglass(const std::string &name, const float3 kt, float ior)
{
  MaterialParams p;
  p.name = name;
  p.type = "archglass";
  p.kt = kt;
  p.ior = ior;
  add_material(p);
}

void Session::add_carpaint(const std::string &name, const float3 kd)
{
  MaterialParams p;
  p.name = name;
  p.type = "carpaint";
  p.kd = kd;
  add_material(p);
}

void Session::add_cloth(const std::string &name, const float3 warp_kd, const float3 weft_kd)
{
  MaterialParams p;
  p.name = name;
  p.type = "cloth";
  p.warp_kd = warp_kd;
  p.weft_kd = weft_kd;
  add_material(p);
}

void Session::add_twosided(const std::string &name, const std::string &front, const std::string &back)
{
  MaterialParams p;
  p.name = name;
  p.type = "twosided";
  p.mix1 = front;
  p.mix2 = back;
  add_material(p);
}

void Session::add_mix(const std::string &name,
                     const std::string &mat1,
                     const std::string &mat2,
                     float amount)
{
  MaterialParams p;
  p.name = name;
  p.type = "mix";
  p.mix1 = mat1;
  p.mix2 = mat2;
  p.mix_amount = amount;
  add_material(p);
}

void Session::add_imagemap(const std::string &name, const std::string &filepath, float gamma)
{
  if (!impl_ || !impl_->scene) {
    return;
  }
  Properties props;
  props << Property("scene.textures." + name + ".type")("imagemap");
  props << Property("scene.textures." + name + ".file")(filepath);
  props << Property("scene.textures." + name + ".gamma")(gamma);
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::add_mesh(const std::string &name,
                       Span<float3> positions,
                       Span<int> triangle_verts,
                       Span<float2> uvs,
                       Span<float3> normals)
{
  if (!impl_ || !impl_->scene || positions.is_empty() || triangle_verts.is_empty()) {
    return;
  }
  const unsigned int nverts = unsigned(positions.size());
  const unsigned int ntris = unsigned(triangle_verts.size() / 3);
  float *p = Scene::AllocVerticesBuffer(nverts);
  unsigned int *vi = Scene::AllocTrianglesBuffer(ntris);
  for (unsigned int i = 0; i < nverts; i++) {
    p[i * 3 + 0] = positions[i].x;
    p[i * 3 + 1] = positions[i].y;
    p[i * 3 + 2] = positions[i].z;
  }
  memcpy(vi, triangle_verts.data(), sizeof(int) * ntris * 3);

  float *n_ptr = nullptr;
  if (normals.size() == positions.size()) {
    n_ptr = Scene::AllocVerticesBuffer(nverts);
    for (unsigned int i = 0; i < nverts; i++) {
      n_ptr[i * 3 + 0] = normals[i].x;
      n_ptr[i * 3 + 1] = normals[i].y;
      n_ptr[i * 3 + 2] = normals[i].z;
    }
  }

  float *uv_ptr = nullptr;
  if (uvs.size() == positions.size()) {
    uv_ptr = new float[nverts * 2];
    for (unsigned int i = 0; i < nverts; i++) {
      uv_ptr[i * 2 + 0] = uvs[i].x;
      uv_ptr[i * 2 + 1] = uvs[i].y;
    }
  }

  try {
    impl_->scene->DefineMesh(name, long(nverts), long(ntris), p, vi, n_ptr, uv_ptr, nullptr, nullptr);
    printf("LuxCore: DefineMesh %s verts=%u tris=%u normals=%s\n",
           name.c_str(),
           nverts,
           ntris,
           n_ptr ? "yes" : "NO (flat)");
  }
  catch (const std::exception &ex) {
    printf("LuxCore DefineMesh failed: %s\n", ex.what());
  }
}

void Session::add_object(const std::string &name,
                         const std::string &mesh,
                         const std::string &material,
                         const float4x4 *transform)
{
  if (!impl_ || !impl_->scene) {
    return;
  }
  Properties props;
  props << Property("scene.objects." + name + ".shape")(mesh);
  props << Property("scene.objects." + name + ".material")(material);
  if (transform) {
    /* pyluxcore BlenderMatrix4x4ToList: for col, for row: m[row, col].
     * Python Matrix is m[row][col]; that emits column-major of the math matrix.
     * Blender float4x4 is also column-major in memory, BUT LuxCore Property::Get
     * reconstructs with (v0,v4,v8,v12) as the first row — i.e. it treats the
     * 16 floats as column-major. Identity and translation match. Rotation must
     * use the same order as BlenderMatrix4x4ToList: append m[row][col] with col
     * in the outer loop, which for float4x4 is base_ptr() as-is.
     *
     * A camera-space X mirror is NOT a transposed object matrix (that would
     * drop translation). It is the film/LookAt X axis vs Blender's right vector.
     * Keep the Blender column-major list; the view flip is handled in the film copy. */
    const float *p = transform->base_ptr();
    std::vector<float> mat(16);
    /* Explicit BlenderMatrix4x4ToList: result[col*4+row] = matrix[row][col].
     * float4x4[col][row] == Python matrix[row][col], so this is base_ptr order. */
    for (int col = 0; col < 4; col++) {
      for (int row = 0; row < 4; row++) {
        mat[col * 4 + row] = p[col * 4 + row];
      }
    }
    props << Property("scene.objects." + name + ".transformation")(mat);
  }
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::add_sun(const std::string &name, const float3 dir, const float3 gain, float turbidity)
{
  if (!impl_ || !impl_->scene) {
    return;
  }
  Properties props;
  props << Property("scene.lights." + name + ".type")("sun");
  props << Property("scene.lights." + name + ".dir")(dir.x, dir.y, dir.z);
  props << Property("scene.lights." + name + ".gain")(gain.x, gain.y, gain.z);
  props << Property("scene.lights." + name + ".turbidity")(turbidity);
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::add_point_light(const std::string &name,
                              const float3 pos,
                              const float3 gain,
                              float radius)
{
  if (!impl_ || !impl_->scene) {
    return;
  }
  Properties props;
  props << Property("scene.lights." + name + ".type")("point");
  props << Property("scene.lights." + name + ".position")(pos.x, pos.y, pos.z);
  props << Property("scene.lights." + name + ".gain")(gain.x, gain.y, gain.z);
  if (radius > 0.0f) {
    props << Property("scene.lights." + name + ".power")(0.0f);
    props << Property("scene.lights." + name + ".efficency")(0.0f);
  }
  (void)radius;
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::add_spot_light(const std::string &name,
                             const float3 pos,
                             const float3 target,
                             const float3 gain,
                             float cone_deg)
{
  if (!impl_ || !impl_->scene) {
    return;
  }
  Properties props;
  props << Property("scene.lights." + name + ".type")("spot");
  props << Property("scene.lights." + name + ".position")(pos.x, pos.y, pos.z);
  props << Property("scene.lights." + name + ".target")(target.x, target.y, target.z);
  props << Property("scene.lights." + name + ".gain")(gain.x, gain.y, gain.z);
  props << Property("scene.lights." + name + ".coneangle")(cone_deg);
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::add_area_light(const std::string &name,
                             const float3 pos,
                             const float3 target,
                             const float3 up,
                             const float3 gain,
                             float size_x,
                             float size_y)
{
  if (!impl_ || !impl_->scene) {
    return;
  }
  const float3 n = blender::math::normalize(target - pos);
  float3 xaxis = blender::math::cross(up, n);
  if (blender::math::length_squared(xaxis) < 1e-8f) {
    xaxis = blender::math::cross(float3(0.0f, 0.0f, 1.0f), n);
  }
  xaxis = blender::math::normalize(xaxis);
  const float3 yaxis = blender::math::normalize(blender::math::cross(n, xaxis));
  const float hx = size_x > 1e-6f ? size_x * 0.5f : 0.25f;
  const float hy = size_y > 1e-6f ? size_y * 0.5f : hx;
  float3 verts[4] = {
      pos - xaxis * hx - yaxis * hy,
      pos + xaxis * hx - yaxis * hy,
      pos + xaxis * hx + yaxis * hy,
      pos - xaxis * hx + yaxis * hy,
  };
  int tris[6] = {0, 1, 2, 0, 2, 3};
  const std::string mat = name + "_emit";
  add_matte(mat, float3(0.0f, 0.0f, 0.0f), gain);
  add_mesh(name + "_mesh",
           Span<float3>(verts, 4),
           Span<int>(tris, 6),
           Span<float2>());
  add_object(name, name + "_mesh", mat);
}

void Session::add_infinite(const std::string &name, const float3 color, float gain)
{
  if (!impl_ || !impl_->scene) {
    return;
  }
  Properties props;
  props << Property("scene.lights." + name + ".type")("constantinfinite");
  props << Property("scene.lights." + name + ".color")(color.x, color.y, color.z);
  props << Property("scene.lights." + name + ".gain")(gain, gain, gain);
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::add_infinite_map(const std::string &name,
                               const std::string &filepath,
                               float gain,
                               float rot_z)
{
  if (!impl_ || !impl_->scene || filepath.empty()) {
    return;
  }
  const float c = cosf(rot_z);
  const float s = sinf(rot_z);
  const std::vector<float> xform = {
      c, 0.0f, s, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -s, 0.0f, c, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  Properties props;
  props << Property("scene.lights." + name + ".type")("infinite");
  props << Property("scene.lights." + name + ".file")(filepath);
  props << Property("scene.lights." + name + ".gain")(gain, gain, gain);
  props << Property("scene.lights." + name + ".gamma")(1.0f);
  props << Property("scene.lights." + name + ".transformation")(xform);
  std::string error;
  impl_->parse(std::move(props), error);
}

void Session::add_sky2(const std::string &name, const float3 dir, const float3 gain, float turbidity)
{
  if (!impl_ || !impl_->scene) {
    return;
  }
  Properties props;
  props << Property("scene.lights." + name + ".type")("sky2");
  props << Property("scene.lights." + name + ".dir")(dir.x, dir.y, dir.z);
  props << Property("scene.lights." + name + ".gain")(gain.x, gain.y, gain.z);
  props << Property("scene.lights." + name + ".turbidity")(turbidity);
  std::string error;
  impl_->parse(std::move(props), error);
}

bool Session::start(std::string &r_error)
{
  if (!impl_ || !impl_->ensure_config(r_error)) {
    return false;
  }
  try {
    impl_->session = RenderSession::Create(impl_->config);
    impl_->session->Start();
    printf("LuxCore: session started (%dx%d, %s)\n",
           width_,
           height_,
           impl_->interactive ? "RTPATHCPU viewport" : "offline");
  }
  catch (const std::exception &ex) {
    r_error = ex.what();
    printf("LuxCore: Start failed: %s\n", ex.what());
    return false;
  }
  return true;
}

void Session::stop()
{
  if (impl_ && impl_->session) {
    try {
      if (impl_->session->IsInSceneEdit()) {
        impl_->session->EndSceneEdit();
      }
      if (impl_->session->IsStarted()) {
        impl_->session->Stop();
      }
    }
    catch (...) {
    }
    impl_->session.reset();
  }
}

void Session::pause()
{
  if (impl_ && impl_->session) {
    try {
      if (impl_->session->IsStarted() && !impl_->session->IsInPause()) {
        impl_->session->Pause();
      }
    }
    catch (...) {
    }
  }
}

void Session::resume()
{
  if (impl_ && impl_->session) {
    try {
      if (impl_->session->IsStarted() && impl_->session->IsInPause()) {
        impl_->session->Resume();
      }
    }
    catch (...) {
    }
  }
}

bool Session::is_started() const
{
  if (!impl_ || !impl_->session) {
    return false;
  }
  try {
    return impl_->session->IsStarted();
  }
  catch (...) {
    return false;
  }
}

bool Session::update_camera_interactive(const CameraDesc &cam)
{
  if (!impl_ || !impl_->scene) {
    return false;
  }
  try {
    const bool running = impl_->session && impl_->session->IsStarted() &&
                         !impl_->session->IsInSceneEdit();
    if (running) {
      impl_->session->BeginSceneEdit();
    }
    set_camera(cam);
    if (running && impl_->session && impl_->session->IsInSceneEdit()) {
      impl_->session->EndSceneEdit();
    }
  }
  catch (const std::exception &ex) {
    printf("LuxCore: camera edit failed: %s\n", ex.what());
    return false;
  }
  return true;
}

bool Session::has_done() const
{
  if (!impl_ || !impl_->session) {
    return true;
  }
  try {
    return impl_->session->HasDone();
  }
  catch (...) {
    return true;
  }
}

unsigned int Session::pass_count()
{
  if (!impl_ || !impl_->session) {
    return 0;
  }
  try {
    impl_->session->UpdateStats();
    const auto &stats = impl_->session->GetStats();
    if (stats) {
      return stats->Get("stats.renderengine.pass").Get<unsigned int>(0);
    }
  }
  catch (...) {
  }
  return 0;
}

bool Session::render(RenderEngine *engine, std::string &r_error)
{
  if (!impl_ || !impl_->ensure_config(r_error)) {
    return false;
  }
  try {
    impl_->session = RenderSession::Create(impl_->config);
    if (engine) {
      RE_engine_update_stats(engine, "LuxCore", "Building PhotonGI caustic cache…");
      RE_engine_update_progress(engine, 0.02f);
    }
    impl_->session->Start();

    const double t0 = BLI_time_now_seconds();
    while (!impl_->session->HasDone()) {
      if (engine && RE_engine_test_break(engine)) {
        impl_->session->Stop();
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      impl_->session->UpdateStats();

      const auto &stats = impl_->session->GetStats();
      unsigned int pass = 0;
      double elapsed = BLI_time_now_seconds() - t0;
      try {
        if (stats) {
          pass = stats->Get("stats.renderengine.pass").Get<unsigned int>(0);
          elapsed = stats->Get("stats.renderengine.time").Get<double>(0);
        }
      }
      catch (...) {
      }
      float progress = 0.02f;
      if (halt_samples_ > 0) {
        progress = 0.02f + 0.98f * (float(pass) / float(halt_samples_));
      }
      else if (halt_time_ > 0) {
        progress = float(elapsed) / float(halt_time_);
      }
      progress = clamp_f(progress, 0.0f, 1.0f);
      if (engine) {
        char info[256];
        SNPRINTF(info,
                 "LuxCore  Samples: %u / %d  Time: %.1fs",
                 pass,
                 halt_samples_,
                 elapsed);
        RE_engine_update_stats(engine, "LuxCore", info);
        RE_engine_update_progress(engine, progress);
      }
    }
    impl_->session->Stop();
  }
  catch (const std::exception &ex) {
    r_error = ex.what();
    return false;
  }
  return true;
}

bool Session::copy_combined_rgba(MutableSpan<float> rgba, std::string &r_error)
{
  if (!impl_ || !impl_->session) {
    r_error = "LuxCore session has no film";
    return false;
  }
  const int64_t npix = int64_t(width_) * int64_t(height_);
  if (rgba.size() < npix * 4) {
    r_error = "Combined buffer is too small";
    return false;
  }
  try {
    /* Merge worker films. WaitNewFrame is a no-op on PATHCPU (only RTPATHCPU
     * implements it); skip it so the viewport light-tracing engine is not
     * treated as a realtime zoom-phase sampler. */
    impl_->session->UpdateStats();
    Film &film = impl_->session->GetFilm();
    impl_->rgb_scratch.resize(npix * 3, 0.0f);
    if (film.HasOutput(Film::OUTPUT_RGB)) {
      film.GetOutput<float>(Film::OUTPUT_RGB, impl_->rgb_scratch.data(), 0, false);
    }
    else if (film.HasOutput(Film::OUTPUT_RGB_IMAGEPIPELINE)) {
      film.GetOutput<float>(Film::OUTPUT_RGB_IMAGEPIPELINE, impl_->rgb_scratch.data(), 0, false);
    }
    else {
      r_error = "LuxCore film has no RGB output";
      return false;
    }
    float peak = 0.0f;
    for (int64_t i = 0; i < npix * 3; i++) {
      peak = std::max(peak, impl_->rgb_scratch[i]);
    }
    if (peak < 1e-8f) {
      /* Keep the previous pixels (gray placeholder) instead of painting black. */
      return false;
    }
    const bool tone = impl_->interactive;
    const float lift = (tone && peak < 0.08f) ? (0.08f / peak) : 1.0f;
    /* Copy film as-is. Flipping X and Y together is a 180° rotation about the
     * screen center (position = center + (p-center)*-1), which is what the
     * viewport showed after the last change. */
    for (int y = 0; y < height_; y++) {
      const float *src = impl_->rgb_scratch.data() + int64_t(y) * width_ * 3;
      float *dst = rgba.data() + int64_t(y) * width_ * 4;
      for (int x = 0; x < width_; x++) {
        float r = src[0] * lift;
        float g = src[1] * lift;
        float b = src[2] * lift;
        if (tone) {
          r = r / (1.0f + r);
          g = g / (1.0f + g);
          b = b / (1.0f + b);
          r = powf(std::max(r, 0.0f), 1.0f / 2.2f);
          g = powf(std::max(g, 0.0f), 1.0f / 2.2f);
          b = powf(std::max(b, 0.0f), 1.0f / 2.2f);
        }
        dst[0] = r;
        dst[1] = g;
        dst[2] = b;
        dst[3] = 1.0f;
        src += 3;
        dst += 4;
      }
    }
  }
  catch (const std::exception &ex) {
    r_error = ex.what();
    return false;
  }
  return true;
}

#else /* !WITH_LUXCORE_SDK */

struct Session::Impl {};

Session::Session() = default;
Session::~Session() = default;

bool Session::available() const
{
  return false;
}

bool Session::begin_scene(int /*width*/,
                          int /*height*/,
                          const SceneLuxCore & /*settings*/,
                          std::string &r_error,
                          bool /*interactive*/)
{
  r_error =
      "LuxCore SDK is not linked. Build LuxCore (extern/luxcore) and set LUXCORE_ROOT to the "
      "install prefix, then rebuild BIKINI with WITH_LUXCORE.";
  return false;
}

void Session::set_camera(const CameraDesc &) {}

void Session::add_material(const MaterialParams &) {}
void Session::add_volume(const VolumeParams &) {}
void Session::set_material_volumes(const std::string &, const std::string &, const std::string &) {}
void Session::patch_transparency(const std::string &, float, float) {}

void Session::add_matte(const std::string &, const float3, const float3) {}
void Session::add_disney(const std::string &,
                         const float3,
                         float,
                         float,
                         float,
                         float,
                         const std::string &)
{
}
void Session::add_glass(const std::string &, const float3, float, float) {}
void Session::add_metal(const std::string &, const float3, const float3, float) {}
void Session::add_mirror(const std::string &, const float3) {}
void Session::add_glossy(const std::string &, const float3, const float3, float) {}
void Session::add_null(const std::string &) {}
void Session::add_velvet(const std::string &, const float3) {}
void Session::add_matte_translucent(const std::string &, const float3, const float3) {}
void Session::add_roughglass(const std::string &, const float3, float, float) {}
void Session::add_archglass(const std::string &, const float3, float) {}
void Session::add_carpaint(const std::string &, const float3) {}
void Session::add_cloth(const std::string &, const float3, const float3) {}
void Session::add_twosided(const std::string &, const std::string &, const std::string &) {}
void Session::add_mix(const std::string &, const std::string &, const std::string &, float) {}
void Session::add_imagemap(const std::string &, const std::string &, float) {}
void Session::add_mesh(const std::string &, Span<float3>, Span<int>, Span<float2>, Span<float3>) {}
void Session::add_object(const std::string &,
                         const std::string &,
                         const std::string &,
                         const float4x4 *)
{
}
void Session::add_sun(const std::string &, const float3, const float3, float) {}
void Session::add_point_light(const std::string &, const float3, const float3, float) {}
void Session::add_spot_light(const std::string &, const float3, const float3, const float3, float)
{
}
void Session::add_area_light(const std::string &,
                             const float3,
                             const float3,
                             const float3,
                             const float3,
                             float,
                             float)
{
}
void Session::add_infinite(const std::string &, const float3, float) {}
void Session::add_infinite_map(const std::string &, const std::string &, float, float) {}
void Session::add_sky2(const std::string &, const float3, const float3, float) {}
bool Session::start(std::string &r_error)
{
  r_error = "LuxCore SDK is not linked";
  return false;
}
void Session::stop() {}
void Session::pause() {}
void Session::resume() {}
bool Session::is_started() const
{
  return false;
}
bool Session::update_camera_interactive(const CameraDesc &)
{
  return false;
}
bool Session::has_done() const
{
  return true;
}
unsigned int Session::pass_count()
{
  return 0;
}
bool Session::render(RenderEngine *, std::string &r_error)
{
  r_error = "LuxCore SDK is not linked";
  return false;
}
bool Session::copy_combined_rgba(MutableSpan<float>, std::string &r_error)
{
  r_error = "LuxCore SDK is not linked";
  return false;
}

#endif

}  // namespace blender::render::luxcore
