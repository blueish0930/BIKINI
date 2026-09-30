/* SPDX-FileCopyrightText: 2026 BIKINI Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <memory>
#include <string>

#include "BLI_math_base.hh"
#include "BLI_math_constants.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

#include "DNA_scene_types.h"

namespace blender {

struct RenderEngine;

namespace render::luxcore {

struct CameraDesc {
  float3 orig = float3(0.0f);
  float3 target = float3(0.0f, 0.0f, -1.0f);
  float3 up = float3(0.0f, 1.0f, 0.0f);
  float clip_start = 0.1f;
  float clip_end = 1000.0f;
  float fov_deg = 45.0f;
  /* xmin xmax ymin ymax — BlendLuxCore calc_screenwindow layout. */
  float screenwindow[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
  bool is_ortho = false;
};

/* BlendLuxCore _calc_lookat: orig = translation, target = M*(0,0,-1), up = R*(0,1,0).
 * LuxCore LookAt is +Z forward; Blender cameras look down local -Z. No Y-up conversion. */
inline void lux_lookat_from_matrix(const float4x4 &cam_to_world, CameraDesc &cam)
{
  cam.orig = cam_to_world.location();
  cam.target = math::transform_point(cam_to_world, float3(0.0f, 0.0f, -1.0f));
  cam.up = math::transform_direction(cam_to_world, float3(0.0f, 1.0f, 0.0f));
}

inline void lux_calc_aspect(const float width,
                            const float height,
                            float &xaspect,
                            float &yaspect,
                            const bool horizontal_fit)
{
  const float w = (width > 1e-8f) ? width : 1.0f;
  const float h = (height > 1e-8f) ? height : 1.0f;
  if (horizontal_fit) {
    xaspect = 1.0f;
    yaspect = h / w;
  }
  else {
    xaspect = w / h;
    yaspect = 1.0f;
  }
}

/* BlendLuxCore utils.calc_screenwindow. shift in -2..2, offset in -1..1. */
inline void lux_calc_screenwindow(const float zoom,
                                  const float shift_x,
                                  const float shift_y,
                                  const float offset_x,
                                  const float offset_y,
                                  const float xaspect,
                                  const float yaspect,
                                  const float scale,
                                  float r_sw[4])
{
  const float dx = scale * 2.0f * (shift_x + 2.0f * xaspect * offset_x);
  const float dy = scale * 2.0f * (shift_y + 2.0f * yaspect * offset_y);
  r_sw[0] = -xaspect * zoom + dx;
  r_sw[1] = xaspect * zoom + dx;
  r_sw[2] = -yaspect * zoom + dy;
  r_sw[3] = yaspect * zoom + dy;
}

class Session {
 public:
  Session();
  ~Session();

  Session(const Session &) = delete;
  Session &operator=(const Session &) = delete;

  bool available() const;

  bool begin_scene(int width,
                   int height,
                   const SceneLuxCore &settings,
                   std::string &r_error,
                   bool interactive = false);

  void set_camera(const CameraDesc &cam);

  /* Official BlendLuxCore material SDL. type is the LuxCore material type string. */
  struct MaterialParams {
    std::string name;
    std::string type = "disney";
    float3 kd = float3(0.7f, 0.7f, 0.7f);
    float3 ks = float3(0.05f, 0.05f, 0.05f);
    float3 kt = float3(1.0f, 1.0f, 1.0f);
    float3 kr = float3(1.0f, 1.0f, 1.0f);
    float3 ka = float3(0.0f, 0.0f, 0.0f);
    float3 emission = float3(0.0f, 0.0f, 0.0f);
    float3 n = float3(0.2f, 0.2f, 0.2f);
    float3 k = float3(3.0f, 3.0f, 3.0f);
    float3 ks1 = float3(1.0f, 1.0f, 1.0f);
    float3 ks2 = float3(1.0f, 1.0f, 1.0f);
    float3 ks3 = float3(1.0f, 1.0f, 1.0f);
    float3 warp_kd = float3(0.7f, 0.05f, 0.05f);
    float3 warp_ks = float3(0.04f, 0.04f, 0.04f);
    float3 weft_kd = float3(0.64f, 0.64f, 0.64f);
    float3 weft_ks = float3(0.04f, 0.04f, 0.04f);
    float3 ks_bf = float3(0.05f, 0.05f, 0.05f);
    float3 ka_bf = float3(0.0f, 0.0f, 0.0f);
    float metallic = 0.0f;
    float roughness = 0.2f;
    float specular = 0.5f;
    float subsurface = 0.0f;
    float speculartint = 0.0f;
    float anisotropic = 0.0f;
    float sheen = 0.0f;
    float sheentint = 0.0f;
    float clearcoat = 0.0f;
    float clearcoatgloss = 1.0f;
    float ior = 1.5f;
    float ior_bf = 1.5f;
    float cauchyb = 0.0f;
    float sigma = 0.0f;
    float opacity = 1.0f;
    float filmamount = 1.0f;
    float filmthickness = 0.0f;
    float filmior = 1.5f;
    float absorption_depth = 0.0f;
    float absorption_depth_bf = 0.0f;
    float thickness = 0.1f;
    float p1 = 2.0f;
    float p2 = 10.0f;
    float p3 = 2.0f;
    float r1 = 0.95f;
    float r2 = 0.9f;
    float r3 = 0.7f;
    float m1 = 0.25f;
    float m2 = 0.1f;
    float m3 = 0.015f;
    float mix_amount = 0.5f;
    float repeat_u = 100.0f;
    float repeat_v = 100.0f;
    float uroughness = -1.0f;
    float vroughness = -1.0f;
    bool multibounce = false;
    bool use_ior = false;
    bool use_n_k = false;
    bool velvet_advanced = false;
    bool carpaint_manual = true;
    std::string mix1;
    std::string mix2;
    std::string base;
    std::string interior_vol;
    std::string exterior_vol;
    std::string tex_name;
    std::string cloth_preset = "denim";
    std::string carpaint_preset;
  };

  struct VolumeParams {
    std::string name;
    std::string type = "clear";
    float3 absorption = float3(0.0f, 0.0f, 0.0f);
    float3 scattering = float3(0.0f, 0.0f, 0.0f);
    float3 emission = float3(0.0f, 0.0f, 0.0f);
    float3 asymmetry = float3(0.0f, 0.0f, 0.0f);
    float ior = 1.5f;
    float scattering_scale = 1.0f;
    float step_size = 0.1f;
    int maxcount = 1024;
    int priority = 0;
    bool multiscattering = false;
  };

  void add_material(const MaterialParams &p);
  void add_volume(const VolumeParams &p);
  void set_material_volumes(const std::string &name,
                            const std::string &interior,
                            const std::string &exterior);
  void patch_transparency(const std::string &name, float front, float back);

  void add_matte(const std::string &name, const float3 kd, const float3 emission);
  void add_disney(const std::string &name,
                  const float3 base_color,
                  float metallic,
                  float roughness,
                  float specular,
                  float ior,
                  const std::string &kd_tex);
  void add_glass(const std::string &name, const float3 kt, float ior, float cauchyb = 0.0f);
  void add_metal(const std::string &name, const float3 n, const float3 k, float roughness);
  void add_mirror(const std::string &name, const float3 kr);
  void add_glossy(const std::string &name, const float3 kd, const float3 ks, float roughness);
  void add_null(const std::string &name);
  void add_velvet(const std::string &name, const float3 kd);
  void add_matte_translucent(const std::string &name, const float3 kd, const float3 kt);
  void add_roughglass(const std::string &name, const float3 kt, float ior, float roughness);
  void add_archglass(const std::string &name, const float3 kt, float ior);
  void add_carpaint(const std::string &name, const float3 kd);
  void add_cloth(const std::string &name, const float3 warp_kd, const float3 weft_kd);
  void add_twosided(const std::string &name, const std::string &front, const std::string &back);
  void add_mix(const std::string &name,
               const std::string &mat1,
               const std::string &mat2,
               float amount);
  void add_imagemap(const std::string &name, const std::string &filepath, float gamma);

  void add_mesh(const std::string &name,
                Span<float3> positions,
                Span<int> triangle_verts,
                Span<float2> uvs,
                Span<float3> normals = {});
  void add_object(const std::string &name,
                  const std::string &mesh,
                  const std::string &material,
                  const float4x4 *transform = nullptr);

  void add_sun(const std::string &name, const float3 dir, const float3 gain, float turbidity);
  void add_point_light(const std::string &name, const float3 pos, const float3 gain, float radius);
  void add_spot_light(const std::string &name,
                      const float3 pos,
                      const float3 target,
                      const float3 gain,
                      float cone_deg);
  void add_area_light(const std::string &name,
                      const float3 pos,
                      const float3 target,
                      const float3 up,
                      const float3 gain,
                      float size_x,
                      float size_y);
  void add_infinite(const std::string &name, const float3 color, float gain);
  void add_infinite_map(const std::string &name,
                        const std::string &filepath,
                        float gain,
                        float rot_z);
  void add_sky2(const std::string &name, const float3 dir, const float3 gain, float turbidity);

  bool start(std::string &r_error);
  void stop();
  void pause();
  void resume();
  bool is_started() const;
  bool has_done() const;
  unsigned int pass_count();

  /** Edit camera without tearing down the session (viewport orbit). */
  bool update_camera_interactive(const CameraDesc &cam);

  bool render(RenderEngine *engine, std::string &r_error);
  bool copy_combined_rgba(MutableSpan<float> rgba, std::string &r_error);

  int width() const
  {
    return width_;
  }
  int height() const
  {
    return height_;
  }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  int width_ = 0;
  int height_ = 0;
  int halt_samples_ = 64;
  int halt_time_ = 0;
};

}  // namespace render::luxcore
}  // namespace blender
