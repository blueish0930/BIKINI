/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edsculpt
 *
 * GPU Texture Paint — deferred projection.
 *
 * Each dab is a 2D stamp into a viewport-sized overlay (no mesh, no 4K/8K canvas).
 * The overlay is composited on the 3D view. When the user pans/zooms/rotates (or
 * leaves paint mode / saves / uses an unsupported brush), the overlay is projected
 * onto the UV canvas in one pass, with occlusion handled only at that point.
 *
 * Overlay storage matches official byte mix: straight (unpremultiplied) RGB + coverage.
 * RGB is scene-linear for Draw so the sRGB window framebuffer displays the picker color.
 */

#include "paint_image_proj_gpu.hh"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <optional>
#include <string>

#include "MEM_guardedalloc.h"

#include "BLI_math_base.hh"
#include "BLI_math_geom_c.hh"
#include "BLI_math_matrix_c.hh"
#include "BLI_math_vector.hh"
#include "BLI_math_vector_c.hh"
#include "BLI_rect.hh"
#include "BLI_utildefines.hh"
#include "BLI_vector.hh"

#include "BKE_attribute.hh"
#include "BKE_callbacks.hh"
#include "BKE_context.hh"
#include "BKE_image.hh"
#include "BKE_image_gpu.hh"
#include "BKE_image_paint_layers.hh"
#include "BKE_mesh.hh"
#include "BKE_object.hh"
#include "BKE_paint.hh"
#include "BKE_paint_types.hh"
#include "BKE_screen.hh"
#include "BKE_undo_system.hh"

#include "DNA_brush_enums.h"
#include "DNA_image_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_view3d_types.h"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "ED_paint.hh"
#include "ED_screen.hh"
#include "ED_space_api.hh"
#include "ED_undo.hh"
#include "ED_view3d.hh"

#include "GPU_batch.hh"
#include "GPU_capabilities.hh"
#include "GPU_common.hh"
#include "GPU_context.hh"
#include "GPU_framebuffer.hh"
#include "GPU_immediate.hh"
#include "GPU_init_exit.hh"
#include "GPU_shader.hh"
#include "GPU_shader_builtin.hh"
#include "GPU_state.hh"
#include "GPU_texture.hh"
#include "GPU_vertex_buffer.hh"
#include "GPU_vertex_format.hh"
#include "GPU_viewport.hh"

#include "IMB_imbuf.hh"
#include "IMB_imbuf_types.hh"
#include "IMB_partial_update.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "gpu/intern/gpu_shader_create_info.hh"

#include "../paint_intern.hh"

namespace blender {

using gpu::shader::ShaderCreateInfo;
using gpu::shader::StageInterfaceInfo;
using gpu::shader::Type;

/* -------------------------------------------------------------------- */
/** \name Persistent GPU resources
 * \{ */

struct PaintProjGPUResources {
  gpu::Shader *stamp_shader = nullptr;
  gpu::Shader *depth_shader = nullptr;
  gpu::Shader *bake_shader = nullptr;
  gpu::Shader *dilate_shader = nullptr;
  gpu::Shader *blit_shader = nullptr;
  gpu::Shader *canvas_shader = nullptr;
  bool shader_failed = false;

  gpu::Texture *curve_tx = nullptr;

  gpu::FrameBuffer *overlay_fb = nullptr;
  gpu::Texture *overlay_tx = nullptr;
  gpu::Texture *overlay_src_tx = nullptr;
  gpu::Texture *overlay_backup_tx = nullptr;
  gpu::Texture *screen_canvas_tx = nullptr;
  gpu::FrameBuffer *screen_canvas_fb = nullptr;
  int overlay_w = 0;
  int overlay_h = 0;
  bool overlay_has_paint = false;
  bool has_overlay_backup = false;
  bool stroke_active = false;
  bool screen_space_stroke = false;
  bool screen_canvas_ready = false;
  bool force_project = false;
  int view_winx = 0;
  int view_winy = 0;
  rcti overlay_dirty = {0, 0, 0, 0};

  float viewmat[4][4] = {};
  float winmat[4][4] = {};
  float project_mat[4][4] = {};
  float viewquat[4] = {};
  float ofs[3] = {};
  float dist = 0.0f;
  float camzoom = 0.0f;
  float camdx = 0.0f;
  float camdy = 0.0f;
  int persp = 0;
  int winx = 0;
  int winy = 0;
  ARegion *region = nullptr;
  Object *ob = nullptr;
  bool do_occlude = true;
  bool do_backfacecull = true;
  bool is_ortho = false;
  float clip_start = 0.1f;
  float seam_bleed_px = 2.0f;
  int blend = IMB_BLEND_MIX;
  short brush_type = IMAGE_PAINT_BRUSH_TYPE_DRAW;
  bool canvas_is_srgb = true;
  bool canvas_is_data = false;
  bool lock_alpha = false;
  bool src_is_linear = true;
  bool hard_edge = false;

  gpu::FrameBuffer *depth_fb = nullptr;
  gpu::Texture *depth_tx = nullptr;
  int depth_w = 0;
  int depth_h = 0;

  gpu::FrameBuffer *apply_fb = nullptr;
  gpu::Texture *orig_tx = nullptr;
  gpu::Texture *paint_tx = nullptr;
  int canvas_w = 0;
  int canvas_h = 0;
  bool is_float = false;
  gpu::TextureFormat orig_format = gpu::TextureFormat::UNORM_8_8_8_8;
  gpu::TextureFormat paint_format = gpu::TextureFormat::UNORM_8_8_8_8;

  Image *ima = nullptr;
  ImageUser iuser = {};
  ImBuf *ibuf = nullptr;

  gpu::Batch *fs_batch = nullptr;

  gpu::Batch *mesh_batch = nullptr;
  gpu::VertBuf *mesh_vbo = nullptr;
  uint mesh_vbo_alloc = 0;
  bool mesh_ready = false;
  /** CPU copy of the bake mesh so 4K/8K canvases can project without a full-size GPU FBO. */
  Vector<float2> mesh_uv_cpu;
  Vector<float4> mesh_screen_cpu;
  bool draw_cb_registered = false;
  bool undo_cb_registered = false;
  ARegionType *art = nullptr;
  void *draw_handle_pre = nullptr;
  void *draw_handle_post = nullptr;
};

static PaintProjGPUResources g_gpu;
static bCallbackFuncStore g_cb_undo_pre;
static bCallbackFuncStore g_cb_save_pre;
static bCallbackFuncStore g_cb_exit_pre;

/** One uncommitted screen-space stroke. A null texture means the overlay was empty. */
struct OverlayUndoSnap {
  gpu::Texture *tex = nullptr;
  bool had_paint = false;
};

static constexpr int overlay_undo_limit = 12;
static Vector<OverlayUndoSnap> g_overlay_undo;
static Vector<OverlayUndoSnap> g_overlay_redo;
static bool g_overlay_undo_pushed = false;
static bool g_overlay_redo_armed = false;

static void overlay_snap_free(OverlayUndoSnap &snap)
{
  if (snap.tex) {
    GPU_TEXTURE_FREE_SAFE(snap.tex);
  }
  snap.had_paint = false;
}

static void overlay_stack_clear(Vector<OverlayUndoSnap> &stack)
{
  for (OverlayUndoSnap &snap : stack) {
    overlay_snap_free(snap);
  }
  /* Shrink so the static destructor does not MEM_freeN after leak detection. */
  stack.clear_and_shrink();
}

static void overlay_undo_free_all()
{
  if (GPU_is_init() && GPU_context_active_get() != nullptr) {
    overlay_stack_clear(g_overlay_undo);
    overlay_stack_clear(g_overlay_redo);
  }
  else {
    /* Shutdown without a GL context: drop the handles. The process is exiting. */
    g_overlay_undo.clear_and_shrink();
    g_overlay_redo.clear_and_shrink();
  }
  g_overlay_undo_pushed = false;
  g_overlay_redo_armed = false;
}

struct OverlayPaintIBuf {
  Image *ima = nullptr;
  ImBuf *ibuf = nullptr;

  OverlayPaintIBuf(Image *image, ImageUser *iuser) : ima(image)
  {
    if (ima) {
      ibuf = BKE_image_paint_acquire_paint_ibuf(ima, iuser, nullptr);
    }
  }
  ~OverlayPaintIBuf()
  {
    if (ima && ibuf) {
      BKE_image_release_ibuf(ima, ibuf, nullptr);
    }
  }
  OverlayPaintIBuf(const OverlayPaintIBuf &) = delete;
  OverlayPaintIBuf &operator=(const OverlayPaintIBuf &) = delete;
};

static void write_ibuf_from_paint(ImBuf *ibuf, const rcti &rect);
static void bind_apply_fb();
static constexpr gpu::TextureFormat overlay_format = gpu::TextureFormat::SFLOAT_16_16_16_16;
static float overlay_linear_to_srgb_ch(float c)
{
  c = math::max(c, 0.0f);
  return (c <= 0.0031308f) ? (c * 12.92f) : (1.055f * powf(c, 1.0f / 2.4f) - 0.055f);
}

static float overlay_srgb_to_linear_ch(float c)
{
  return (c <= 0.04045f) ? (c * (1.0f / 12.92f)) : powf((c + 0.055f) * (1.0f / 1.055f), 2.4f);
}

static float3 overlay_linear_to_srgb(const float3 &c)
{
  return float3(
      overlay_linear_to_srgb_ch(c.x), overlay_linear_to_srgb_ch(c.y), overlay_linear_to_srgb_ch(c.z));
}

static float3 overlay_srgb_to_linear(const float3 &c)
{
  return float3(overlay_srgb_to_linear_ch(c.x),
                overlay_srgb_to_linear_ch(c.y),
                overlay_srgb_to_linear_ch(c.z));
}

/** Straight-alpha over. Opaque dest matches lerp; empty layers gain coverage. */
static void overlay_mix_over(const float3 &dst_rgb,
                             float dst_a,
                             const float3 &src_rgb,
                             float cov,
                             float3 &r_rgb,
                             float &r_a)
{
  r_a = cov + dst_a * (1.0f - cov);
  if (r_a > 1.0e-6f) {
    r_rgb = (src_rgb * cov + dst_rgb * dst_a * (1.0f - cov)) / r_a;
  }
  else {
    r_rgb = dst_rgb;
    r_a = dst_a;
  }
}

static float4 overlay_fetch_px(const float *px, int w, int h, int x, int y)
{
  if (x < 0 || y < 0 || x >= w || y >= h) {
    return float4(0.0f);
  }
  const int i = (y * w + x) * 4;
  return float4(px[i + 0], px[i + 1], px[i + 2], px[i + 3]);
}

static float4 overlay_sample_nearest(const float *px, int w, int h, const float2 &scr)
{
  const int x = int(floorf(scr.x));
  const int y = int(floorf(scr.y));
  return overlay_fetch_px(px, w, h, x, y);
}

static bool overlay_px_touches_empty(const float *px, int w, int h, int x, int y)
{
  if (overlay_fetch_px(px, w, h, x, y).w < 1.0e-5f) {
    return true;
  }
  const int dx[4] = {-1, 1, 0, 0};
  const int dy[4] = {0, 0, -1, 1};
  for (int i = 0; i < 4; i++) {
    if (overlay_fetch_px(px, w, h, x + dx[i], y + dy[i]).w < 1.0e-5f) {
      return true;
    }
  }
  return false;
}

static float4 overlay_sample_bilinear(const float *px, int w, int h, const float2 &scr, bool addend)
{
  const float2 p = scr - float2(0.5f);
  const int x0 = int(floorf(p.x));
  const int y0 = int(floorf(p.y));
  const float fx = p.x - float(x0);
  const float fy = p.y - float(y0);
  float4 s00 = overlay_fetch_px(px, w, h, x0, y0);
  float4 s10 = overlay_fetch_px(px, w, h, x0 + 1, y0);
  float4 s01 = overlay_fetch_px(px, w, h, x0, y0 + 1);
  float4 s11 = overlay_fetch_px(px, w, h, x0 + 1, y0 + 1);
  /* Mixing a painted overlay texel with empty neighbors is the 8K 毛边:
   * one screen-pixel of silhouette AA becomes many UV texels of fringe. */
  const bool any_empty = (s00.w < 1.0e-5f) || (s10.w < 1.0e-5f) || (s01.w < 1.0e-5f) ||
                         (s11.w < 1.0e-5f);
  if (any_empty) {
    return overlay_sample_nearest(px, w, h, scr);
  }
  if (!addend) {
    s00 = float4(s00.x * s00.w, s00.y * s00.w, s00.z * s00.w, s00.w);
    s10 = float4(s10.x * s10.w, s10.y * s10.w, s10.z * s10.w, s10.w);
    s01 = float4(s01.x * s01.w, s01.y * s01.w, s01.z * s01.w, s01.w);
    s11 = float4(s11.x * s11.w, s11.y * s11.w, s11.z * s11.w, s11.w);
  }
  const float4 a = s00 * (1.0f - fx) + s10 * fx;
  const float4 b = s01 * (1.0f - fx) + s11 * fx;
  float4 m = a * (1.0f - fy) + b * fy;
  if (!addend) {
    if (m.w > 1.0e-6f) {
      m.x /= m.w;
      m.y /= m.w;
      m.z /= m.w;
    }
    else {
      m.x = m.y = m.z = 0.0f;
    }
  }
  return m;
}

static bool overlay_tri_hits_dirty(const float4 &s0, const float4 &s1, const float4 &s2)
{
  if (BLI_rcti_is_empty(&g_gpu.overlay_dirty)) {
    return true;
  }
  const float xmin = math::min(s0.x, math::min(s1.x, s2.x));
  const float xmax = math::max(s0.x, math::max(s1.x, s2.x));
  const float ymin = math::min(s0.y, math::min(s1.y, s2.y));
  const float ymax = math::max(s0.y, math::max(s1.y, s2.y));
  rcti tb;
  BLI_rcti_init(&tb,
                int(floorf(xmin)) - 2,
                int(ceilf(xmax)) + 2,
                int(floorf(ymin)) - 2,
                int(ceilf(ymax)) + 2);
  rcti hit;
  return BLI_rcti_isect(&tb, &g_gpu.overlay_dirty, &hit);
}

static bool overlay_uv_dirty_from_mesh(int cw, int ch, rcti &r_dirty)
{
  BLI_rcti_init_minmax(&r_dirty);
  const int n = int(g_gpu.mesh_uv_cpu.size());
  if (n < 3 || n != int(g_gpu.mesh_screen_cpu.size()) || cw < 1 || ch < 1) {
    return false;
  }
  bool any = false;
  for (int t = 0; t + 2 < n; t += 3) {
    const float4 &s0 = g_gpu.mesh_screen_cpu[t];
    const float4 &s1 = g_gpu.mesh_screen_cpu[t + 1];
    const float4 &s2 = g_gpu.mesh_screen_cpu[t + 2];
    if (!overlay_tri_hits_dirty(s0, s1, s2)) {
      continue;
    }
    const float2 u0 = g_gpu.mesh_uv_cpu[t];
    const float2 u1 = g_gpu.mesh_uv_cpu[t + 1];
    const float2 u2 = g_gpu.mesh_uv_cpu[t + 2];
    const float pxmin = math::min(u0.x, math::min(u1.x, u2.x)) * float(cw);
    const float pxmax = math::max(u0.x, math::max(u1.x, u2.x)) * float(cw);
    const float pymin = math::min(u0.y, math::min(u1.y, u2.y)) * float(ch);
    const float pymax = math::max(u0.y, math::max(u1.y, u2.y)) * float(ch);
    const int xy0[2] = {int(floorf(pxmin)), int(floorf(pymin))};
    const int xy1[2] = {int(ceilf(pxmax)), int(ceilf(pymax))};
    BLI_rcti_do_minmax_v(&r_dirty, xy0);
    BLI_rcti_do_minmax_v(&r_dirty, xy1);
    any = true;
  }
  if (!any) {
    return false;
  }
  r_dirty.xmin = math::clamp(r_dirty.xmin, 0, cw);
  r_dirty.ymin = math::clamp(r_dirty.ymin, 0, ch);
  r_dirty.xmax = math::clamp(r_dirty.xmax + 1, 0, cw);
  r_dirty.ymax = math::clamp(r_dirty.ymax + 1, 0, ch);
  return !BLI_rcti_is_empty(&r_dirty);
}

static float *overlay_host_read()
{
  if (g_gpu.overlay_tx == nullptr || g_gpu.overlay_w < 1 || g_gpu.overlay_h < 1) {
    return nullptr;
  }
  GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_FRAMEBUFFER |
                     GPU_BARRIER_TEXTURE_UPDATE);
  gpu::FrameBuffer *prev = GPU_framebuffer_active_get();
  gpu::Texture *tmp = GPU_texture_create_2d("paint_proj_ovl_host",
                                            g_gpu.overlay_w,
                                            g_gpu.overlay_h,
                                            1,
                                            overlay_format,
                                            GPU_TEXTURE_USAGE_HOST_READ | GPU_TEXTURE_USAGE_SHADER_READ |
                                                GPU_TEXTURE_USAGE_ATTACHMENT,
                                            nullptr);
  if (tmp == nullptr) {
    return nullptr;
  }
  GPU_texture_copy(tmp, g_gpu.overlay_tx);
  GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);
  float *data = static_cast<float *>(GPU_texture_read(tmp, GPU_DATA_FLOAT, 0));
  GPU_TEXTURE_FREE_SAFE(tmp);
  if (prev) {
    GPU_framebuffer_bind(prev);
  }
  return data;
}

static void overlay_flush_ibuf_to_image(ImBuf *ibuf, const rcti &dirty)
{
  ImagePaintPartialRedraw pr;
  pr.dirty_region = dirty;
  set_imapaintpartial(&pr);
  if (g_gpu.ima && BKE_image_paint_layers_has_stack(*g_gpu.ima) &&
      !BKE_image_paint_layers_is_identity_background(*g_gpu.ima))
  {
    BKE_image_paint_layers_notify_written(g_gpu.ima, &g_gpu.iuser, dirty);
    ImBuf *cache = BKE_image_acquire_ibuf(g_gpu.ima, &g_gpu.iuser, nullptr);
    imapaint_image_update(cache);
    BKE_image_release_ibuf(g_gpu.ima, cache, nullptr);
  }
  else {
    imapaint_image_update(ibuf);
  }
}

static bool overlay_cpu_write_pixels(ImBuf *ibuf)
{
  if (ibuf == nullptr || g_gpu.mesh_uv_cpu.is_empty()) {
    return false;
  }
  const int n = int(g_gpu.mesh_uv_cpu.size());
  if (n < 3 || n != int(g_gpu.mesh_screen_cpu.size())) {
    return false;
  }

  float *overlay = overlay_host_read();
  if (overlay == nullptr) {
    return false;
  }
  const int ow = g_gpu.overlay_w;
  const int oh = g_gpu.overlay_h;

  IMB_ensure_host_buffer(ibuf);
  float *df = ibuf->float_data() ? ibuf->float_data_for_write() : nullptr;
  uchar *db = (df == nullptr && ibuf->byte_data()) ? ibuf->byte_data_for_write() : nullptr;
  if (df == nullptr && db == nullptr) {
    MEM_delete(overlay);
    return false;
  }

  const int cw = ibuf->x;
  const int ch = ibuf->y;
  const bool is_float = df != nullptr;
  const int mode = (g_gpu.brush_type == IMAGE_PAINT_BRUSH_TYPE_DRAW) ? g_gpu.blend : 0;
  const bool addend = (mode == IMB_BLEND_ADD || mode == IMB_BLEND_SUB);
  const rcti od = g_gpu.overlay_dirty;
  const bool use_od = !BLI_rcti_is_empty(&od);
  bool wrote = false;

  /* One write per texel. Adjacent triangles overlap by the edge margin; blending
   * the same dab twice darkens the shared edge. */
  rcti mask_rect{};
  const bool have_mask_rect = overlay_uv_dirty_from_mesh(cw, ch, mask_rect);
  constexpr int mask_pad = 48;
  int mask_w = 0;
  int mask_h = 0;
  Vector<uint8_t> claimed;
  bool use_claimed = false;
  if (have_mask_rect) {
    mask_rect.xmin = math::max(mask_rect.xmin - mask_pad, 0);
    mask_rect.ymin = math::max(mask_rect.ymin - mask_pad, 0);
    mask_rect.xmax = math::min(mask_rect.xmax + mask_pad, cw);
    mask_rect.ymax = math::min(mask_rect.ymax + mask_pad, ch);
    mask_w = mask_rect.xmax - mask_rect.xmin;
    mask_h = mask_rect.ymax - mask_rect.ymin;
    const uint64_t mask_n = uint64_t(mask_w) * uint64_t(mask_h);
    if (mask_w > 0 && mask_h > 0 && mask_n <= uint64_t(200) * 1024 * 1024) {
      claimed.resize(size_t(mask_n), 0);
      use_claimed = true;
    }
  }

  auto apply_px = [&](int x, int y, const float4 &over) -> bool {
    if (over.w < 1.0e-5f) {
      return false;
    }
    float3 src_rgb(over.x, over.y, over.z);
    const float cov = over.w;
    if (g_gpu.src_is_linear && !g_gpu.canvas_is_data && !is_float) {
      src_rgb = overlay_linear_to_srgb(src_rgb);
    }
    else if (!g_gpu.src_is_linear && is_float && !g_gpu.canvas_is_data) {
      src_rgb = overlay_srgb_to_linear(src_rgb);
    }
    if (mode == IMB_BLEND_MIX && math::dot(src_rgb, src_rgb) < 1.0e-6f) {
      return false;
    }
    const int i = (y * cw + x) * 4;
    if (df) {
      float4 orig(df[i + 0], df[i + 1], df[i + 2], df[i + 3]);
      float4 orig_s = (orig.w > 1.0e-6f) ?
                          float4(orig.x / orig.w, orig.y / orig.w, orig.z / orig.w, orig.w) :
                          float4(0.0f);
      float4 out_s = orig_s;
      if (mode == IMB_BLEND_ADD) {
        out_s.x = math::clamp(orig_s.x + src_rgb.x, 0.0f, 1.0f);
        out_s.y = math::clamp(orig_s.y + src_rgb.y, 0.0f, 1.0f);
        out_s.z = math::clamp(orig_s.z + src_rgb.z, 0.0f, 1.0f);
        out_s.w = math::min(orig_s.w + cov, 1.0f);
      }
      else if (mode == IMB_BLEND_SUB) {
        out_s.x = math::max(orig_s.x - src_rgb.x, 0.0f);
        out_s.y = math::max(orig_s.y - src_rgb.y, 0.0f);
        out_s.z = math::max(orig_s.z - src_rgb.z, 0.0f);
        out_s.w = math::max(orig_s.w, cov);
      }
      else if (mode == IMB_BLEND_ERASE_ALPHA) {
        out_s.w = math::max(orig_s.w - cov, 0.0f);
      }
      else {
        float3 mixed;
        float na;
        overlay_mix_over(float3(orig_s.x, orig_s.y, orig_s.z), orig_s.w, src_rgb, cov, mixed, na);
        out_s.x = mixed.x;
        out_s.y = mixed.y;
        out_s.z = mixed.z;
        out_s.w = na;
      }
      if (g_gpu.lock_alpha) {
        out_s.w = orig_s.w;
      }
      df[i + 0] = out_s.x * out_s.w;
      df[i + 1] = out_s.y * out_s.w;
      df[i + 2] = out_s.z * out_s.w;
      df[i + 3] = out_s.w;
    }
    else {
      const float da = float(db[i + 3]) / 255.0f;
      float3 orig(float(db[i + 0]) / 255.0f,
                  float(db[i + 1]) / 255.0f,
                  float(db[i + 2]) / 255.0f);
      float3 out_rgb = orig;
      float out_a = da;
      if (mode == IMB_BLEND_ADD) {
        out_rgb.x = math::clamp(orig.x + src_rgb.x, 0.0f, 1.0f);
        out_rgb.y = math::clamp(orig.y + src_rgb.y, 0.0f, 1.0f);
        out_rgb.z = math::clamp(orig.z + src_rgb.z, 0.0f, 1.0f);
        out_a = math::min(da + cov, 1.0f);
      }
      else if (mode == IMB_BLEND_SUB) {
        out_rgb.x = math::max(orig.x - src_rgb.x, 0.0f);
        out_rgb.y = math::max(orig.y - src_rgb.y, 0.0f);
        out_rgb.z = math::max(orig.z - src_rgb.z, 0.0f);
        out_a = math::max(da, cov);
      }
      else if (mode == IMB_BLEND_ERASE_ALPHA) {
        out_a = math::max(da - cov, 0.0f);
      }
      else {
        overlay_mix_over(orig, da, src_rgb, cov, out_rgb, out_a);
      }
      if (g_gpu.lock_alpha) {
        out_a = da;
      }
      db[i + 0] = uchar(math::clamp(int(out_rgb.x * 255.0f + 0.5f), 0, 255));
      db[i + 1] = uchar(math::clamp(int(out_rgb.y * 255.0f + 0.5f), 0, 255));
      db[i + 2] = uchar(math::clamp(int(out_rgb.z * 255.0f + 0.5f), 0, 255));
      db[i + 3] = uchar(math::clamp(int(out_a * 255.0f + 0.5f), 0, 255));
    }
    wrote = true;
    return true;
  };

  for (int t = 0; t + 2 < n; t += 3) {
    const float4 s0 = g_gpu.mesh_screen_cpu[t];
    const float4 s1 = g_gpu.mesh_screen_cpu[t + 1];
    const float4 s2 = g_gpu.mesh_screen_cpu[t + 2];
    if (!overlay_tri_hits_dirty(s0, s1, s2)) {
      continue;
    }
    const float2 u0 = g_gpu.mesh_uv_cpu[t];
    const float2 u1 = g_gpu.mesh_uv_cpu[t + 1];
    const float2 u2 = g_gpu.mesh_uv_cpu[t + 2];
    const float2 p0(u0.x * float(cw), u0.y * float(ch));
    const float2 p1(u1.x * float(cw), u1.y * float(ch));
    const float2 p2(u2.x * float(cw), u2.y * float(ch));
    const float area = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
    if (fabsf(area) < 1.0e-8f) {
      continue;
    }
    const float inv_area = 1.0f / area;
    const float abs_inv_area = fabsf(inv_area);
    const float edge_len0 = math::length(p2 - p1);
    const float edge_len1 = math::length(p0 - p2);
    const float edge_len2 = math::length(p1 - p0);
    const int xmin = math::clamp(int(floorf(math::min(p0.x, math::min(p1.x, p2.x)))), 0, cw - 1);
    const int xmax = math::clamp(int(ceilf(math::max(p0.x, math::max(p1.x, p2.x)))), 0, cw - 1);
    const int ymin = math::clamp(int(floorf(math::min(p0.y, math::min(p1.y, p2.y)))), 0, ch - 1);
    const int ymax = math::clamp(int(ceilf(math::max(p0.y, math::max(p1.y, p2.y)))), 0, ch - 1);
    for (int y = ymin; y <= ymax; y++) {
      for (int x = xmin; x <= xmax; x++) {
        uint8_t *claim = nullptr;
        if (use_claimed) {
          const int lx = x - mask_rect.xmin;
          const int ly = y - mask_rect.ymin;
          if (lx >= 0 && ly >= 0 && lx < mask_w && ly < mask_h) {
            claim = &claimed[size_t(ly) * size_t(mask_w) + size_t(lx)];
            if (*claim) {
              continue;
            }
          }
        }
        const float2 p(float(x) + 0.5f, float(y) + 0.5f);
        const float w0 = ((p1.x - p.x) * (p2.y - p.y) - (p1.y - p.y) * (p2.x - p.x)) * inv_area;
        const float w1 = ((p2.x - p.x) * (p0.y - p.y) - (p2.y - p.y) * (p0.x - p.x)) * inv_area;
        const float w2 = 1.0f - w0 - w1;
        /* Cover the shared edge (w == 0) but never paint past the UV island. */
        if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) {
          continue;
        }
        const float4 vs(w0 * s0.x * s0.w + w1 * s1.x * s1.w + w2 * s2.x * s2.w,
                        w0 * s0.y * s0.w + w1 * s1.y * s1.w + w2 * s2.y * s2.w,
                        w0 * s0.z * s0.w + w1 * s1.z * s1.w + w2 * s2.z * s2.w,
                        w0 * s0.w + w1 * s1.w + w2 * s2.w);
        const float invw = 1.0f / math::max(vs.w, 1.0e-8f);
        const float2 scr(vs.x * invw, vs.y * invw);
        if (use_od) {
          if (scr.x < float(od.xmin) - 1.0f || scr.x >= float(od.xmax) + 1.0f ||
              scr.y < float(od.ymin) - 1.0f || scr.y >= float(od.ymax) + 1.0f)
          {
            continue;
          }
        }
        const float dist0 = w0 / math::max(edge_len0 * abs_inv_area, 1.0e-8f);
        const float dist1 = w1 / math::max(edge_len1 * abs_inv_area, 1.0e-8f);
        const float dist2 = w2 / math::max(edge_len2 * abs_inv_area, 1.0e-8f);
        const bool on_uv_rim = math::min(dist0, math::min(dist1, dist2)) < 1.25f;
        float4 over;
        if (on_uv_rim) {
          over = overlay_sample_nearest(overlay, ow, oh, scr);
          const int ox = int(floorf(scr.x));
          const int oy = int(floorf(scr.y));
          /* Silhouette / MSAA overlay pixels splat into a hair of UV texels. */
          if (over.w < 0.6f && overlay_px_touches_empty(overlay, ow, oh, ox, oy)) {
            continue;
          }
        }
        else {
          over = overlay_sample_bilinear(overlay, ow, oh, scr, addend);
        }
        if (apply_px(x, y, over) && claim) {
          *claim = 1;
        }
      }
    }
  }

  MEM_delete(overlay);
  return wrote;
}

static bool paint_proj_gpu_overlay_project_ex(const bContext *C, bool push_undo);
static bool overlay_view_changed(const ARegion *region);
static void overlay_refresh_view(const ARegion *region);
static bool paint_proj_gpu_ensure_screen_canvas();

static void paint_proj_gpu_restore_draw_state(gpu::FrameBuffer *prev_fb,
                                              GPUBlend prev_blend,
                                              GPUDepthTest prev_depth,
                                              GPUFaceCullTest prev_cull,
                                              bool prev_depth_mask,
                                              GPUWriteMask prev_write_mask)
{
  GPU_blend(prev_blend);
  GPU_depth_test(prev_depth);
  GPU_face_culling(prev_cull);
  GPU_front_facing(false);
  GPU_depth_mask(prev_depth_mask);
  GPU_write_mask(prev_write_mask);
  GPU_scissor_test(false);
  GPU_shader_unbind();
  if (prev_fb) {
    GPU_framebuffer_bind(prev_fb);
  }
  else {
    GPU_framebuffer_restore();
  }
}

static void paint_proj_gpu_free_canvas()
{
  GPU_FRAMEBUFFER_FREE_SAFE(g_gpu.apply_fb);
  GPU_TEXTURE_FREE_SAFE(g_gpu.orig_tx);
  GPU_TEXTURE_FREE_SAFE(g_gpu.paint_tx);
  g_gpu.canvas_w = 0;
  g_gpu.canvas_h = 0;
}

static void paint_proj_gpu_free_depth()
{
  GPU_FRAMEBUFFER_FREE_SAFE(g_gpu.depth_fb);
  GPU_TEXTURE_FREE_SAFE(g_gpu.depth_tx);
  g_gpu.depth_w = 0;
  g_gpu.depth_h = 0;
}

static void paint_proj_gpu_free_screen_canvas()
{
  GPU_FRAMEBUFFER_FREE_SAFE(g_gpu.screen_canvas_fb);
  GPU_TEXTURE_FREE_SAFE(g_gpu.screen_canvas_tx);
  g_gpu.screen_canvas_ready = false;
}

static void paint_proj_gpu_free_overlay_textures()
{
  GPU_FRAMEBUFFER_FREE_SAFE(g_gpu.overlay_fb);
  GPU_TEXTURE_FREE_SAFE(g_gpu.overlay_tx);
  GPU_TEXTURE_FREE_SAFE(g_gpu.overlay_src_tx);
  GPU_TEXTURE_FREE_SAFE(g_gpu.overlay_backup_tx);
  paint_proj_gpu_free_screen_canvas();
  g_gpu.overlay_w = 0;
  g_gpu.overlay_h = 0;
  g_gpu.overlay_has_paint = false;
  g_gpu.has_overlay_backup = false;
  BLI_rcti_init_minmax(&g_gpu.overlay_dirty);
  g_gpu.region = nullptr;
}

static void paint_proj_gpu_free_mesh()
{
  GPU_BATCH_DISCARD_SAFE(g_gpu.mesh_batch);
  g_gpu.mesh_vbo = nullptr;
  g_gpu.mesh_vbo_alloc = 0;
  g_gpu.mesh_ready = false;
  g_gpu.mesh_uv_cpu.clear_and_shrink();
  g_gpu.mesh_screen_cpu.clear_and_shrink();
}

static void paint_proj_gpu_free_all()
{
  overlay_undo_free_all();
  if (g_gpu.art) {
    if (g_gpu.draw_handle_pre) {
      ED_region_draw_cb_exit(g_gpu.art, g_gpu.draw_handle_pre);
      g_gpu.draw_handle_pre = nullptr;
    }
    if (g_gpu.draw_handle_post) {
      ED_region_draw_cb_exit(g_gpu.art, g_gpu.draw_handle_post);
      g_gpu.draw_handle_post = nullptr;
    }
    g_gpu.art = nullptr;
  }
  g_gpu.draw_cb_registered = false;
  g_gpu.region = nullptr;
  g_gpu.stroke_active = false;

  if (GPU_is_init() && GPU_context_active_get() != nullptr) {
    paint_proj_gpu_free_overlay_textures();
    paint_proj_gpu_free_canvas();
    paint_proj_gpu_free_depth();
    paint_proj_gpu_free_mesh();
    GPU_SHADER_FREE_SAFE(g_gpu.stamp_shader);
    GPU_SHADER_FREE_SAFE(g_gpu.depth_shader);
    GPU_SHADER_FREE_SAFE(g_gpu.bake_shader);
    GPU_SHADER_FREE_SAFE(g_gpu.dilate_shader);
    GPU_SHADER_FREE_SAFE(g_gpu.blit_shader);
    GPU_SHADER_FREE_SAFE(g_gpu.canvas_shader);
    GPU_TEXTURE_FREE_SAFE(g_gpu.curve_tx);
    GPU_BATCH_DISCARD_SAFE(g_gpu.fs_batch);
  }
  else {
    g_gpu.mesh_batch = nullptr;
    g_gpu.mesh_vbo = nullptr;
    g_gpu.mesh_ready = false;
    g_gpu.overlay_has_paint = false;
    g_gpu.overlay_tx = nullptr;
    g_gpu.overlay_fb = nullptr;
    g_gpu.overlay_src_tx = nullptr;
    g_gpu.overlay_backup_tx = nullptr;
    g_gpu.screen_canvas_tx = nullptr;
    g_gpu.screen_canvas_fb = nullptr;
  }
  g_gpu.mesh_uv_cpu.clear_and_shrink();
  g_gpu.mesh_screen_cpu.clear_and_shrink();

  g_gpu.ima = nullptr;
  g_gpu.ibuf = nullptr;
  g_gpu.ob = nullptr;
  g_gpu.force_project = false;
  g_gpu.shader_failed = false;
}

static void paint_proj_gpu_unregister_callbacks()
{
  if (!g_gpu.undo_cb_registered) {
    return;
  }
  BKE_callback_remove(&g_cb_undo_pre, BKE_CB_EVT_UNDO_PRE);
  BKE_callback_remove(&g_cb_save_pre, BKE_CB_EVT_SAVE_PRE);
  BKE_callback_remove(&g_cb_exit_pre, BKE_CB_EVT_EXIT_PRE);
  g_gpu.undo_cb_registered = false;
}

void paint_proj_gpu_exit()
{
  paint_proj_gpu_unregister_callbacks();
  paint_proj_gpu_free_all();
}

static bool overlay_scene_is_drawing(const bContext *C)
{
  if (g_gpu.stroke_active) {
    return true;
  }
  if (C == nullptr) {
    return false;
  }
  const Scene *scene = CTX_data_scene(C);
  if (scene == nullptr || scene->toolsettings == nullptr) {
    return false;
  }
  return (scene->toolsettings->imapaint.flag & IMAGEPAINT_DRAWING) != 0;
}

static void overlay_apply_rv3d_view(RegionView3D *rv3d,
                                    const float viewquat[4],
                                    const float ofs[3],
                                    float dist,
                                    int persp,
                                    float camzoom,
                                    float camdx,
                                    float camdy)
{
  copy_v4_v4(rv3d->viewquat, viewquat);
  copy_v3_v3(rv3d->ofs, ofs);
  rv3d->dist = dist;
  rv3d->persp = eRegionView3D_Persp(persp);
  rv3d->camzoom = camzoom;
  rv3d->camdx = camdx;
  rv3d->camdy = camdy;
}

static void overlay_pre_view(const bContext *C, ARegion *region, void * /*arg*/)
{
  if (!g_gpu.overlay_has_paint) {
    return;
  }
  if (region == nullptr || region != g_gpu.region) {
    return;
  }
  if (overlay_scene_is_drawing(C)) {
    return;
  }

  Scene *scene = C ? CTX_data_scene(C) : nullptr;
  const bool screen_space = scene && scene->toolsettings &&
                            (scene->toolsettings->imapaint.flag & IMAGEPAINT_PROJECT_SCREEN_SPACE);
  if (!screen_space || g_gpu.force_project) {
    /* Screen Space off, or canvas/slot change: commit with the paint-time camera. */
    g_gpu.force_project = false;
    paint_proj_gpu_overlay_project_ex(C, true);
    return;
  }

  if (!overlay_view_changed(region)) {
    return;
  }

  RegionView3D *rv3d = static_cast<RegionView3D *>(region->regiondata);
  if (rv3d == nullptr) {
    paint_proj_gpu_overlay_project_ex(C, true);
    return;
  }

  /* Lock: keep the paint-time camera while we project, then apply the user's
   * new view. Overlay is cleared before the mesh is drawn at the new angle. */
  const float new_quat[4] = {
      rv3d->viewquat[0], rv3d->viewquat[1], rv3d->viewquat[2], rv3d->viewquat[3]};
  const float new_ofs[3] = {rv3d->ofs[0], rv3d->ofs[1], rv3d->ofs[2]};
  const float new_dist = rv3d->dist;
  const int new_persp = int(rv3d->persp);
  const float new_zoom = rv3d->camzoom;
  const float new_dx = rv3d->camdx;
  const float new_dy = rv3d->camdy;

  overlay_apply_rv3d_view(rv3d,
                          g_gpu.viewquat,
                          g_gpu.ofs,
                          g_gpu.dist,
                          g_gpu.persp,
                          g_gpu.camzoom,
                          g_gpu.camdx,
                          g_gpu.camdy);

  View3D *v3d = CTX_wm_view3d(C);
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  if (v3d && scene && depsgraph) {
    ED_view3d_update_viewmat(depsgraph, scene, v3d, region, nullptr, nullptr, nullptr, false);
  }

  paint_proj_gpu_overlay_project_ex(C, true);

  overlay_apply_rv3d_view(
      rv3d, new_quat, new_ofs, new_dist, new_persp, new_zoom, new_dx, new_dy);
  if (v3d && scene && depsgraph) {
    ED_view3d_update_viewmat(depsgraph, scene, v3d, region, nullptr, nullptr, nullptr, false);
  }
}

static void overlay_post_pixel(const bContext * /*C*/, ARegion *region, void * /*arg*/)
{
  if (!g_gpu.overlay_has_paint || g_gpu.overlay_tx == nullptr || region == nullptr) {
    return;
  }
  if (region != g_gpu.region || g_gpu.blit_shader == nullptr || g_gpu.fs_batch == nullptr) {
    return;
  }
  /* Screen overlay is only valid in the view it was painted. Hide it after orbit
   * so paint does not float in camera space; click Project to commit. */
  if (overlay_view_changed(region)) {
    return;
  }

  gpu::Texture *view_color = nullptr;
  if (GPUViewport *vp = WM_draw_region_get_viewport(region)) {
    view_color = GPU_viewport_color_texture(vp, 0);
  }
  const bool erase_preview = (g_gpu.brush_type == IMAGE_PAINT_BRUSH_TYPE_DRAW &&
                              g_gpu.blend == IMB_BLEND_ERASE_ALPHA);
  /* Mix/Smear/Blur must SRC_ALPHA-blend onto the *current* framebuffer (the
   * shaded mesh). Sampling color_render_tx while drawing POST_PIXEL often
   * returns black — mix(black, paint, falloff) is a halo thicker than the
   * brush (opaque core looks like the stroke, falloff is a fat black ring).
   * Add/Sub still read the underlay; Erase uses ALPHA over black. */
  const bool dest_lerp = (view_color != nullptr) &&
                         (g_gpu.brush_type == IMAGE_PAINT_BRUSH_TYPE_DRAW) &&
                         (g_gpu.blend != IMB_BLEND_MIX) && !erase_preview;
  GPU_blend(dest_lerp ? GPU_BLEND_NONE : GPU_BLEND_ALPHA);
  GPU_depth_test(GPU_DEPTH_NONE);
  GPU_depth_mask(false);
  GPU_batch_set_shader(g_gpu.fs_batch, g_gpu.blit_shader);
  GPU_shader_uniform_1i(g_gpu.blit_shader, "src_is_linear", g_gpu.src_is_linear ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.blit_shader, "canvas_is_srgb", g_gpu.canvas_is_srgb ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.blit_shader, "canvas_is_data", g_gpu.canvas_is_data ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.blit_shader, "is_float", g_gpu.is_float ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.blit_shader, "lock_alpha", g_gpu.lock_alpha ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.blit_shader, "blend", g_gpu.blend);
  GPU_shader_uniform_1i(g_gpu.blit_shader, "brush_type", int(g_gpu.brush_type));
  GPU_shader_uniform_1i(g_gpu.blit_shader, "dest_lerp", dest_lerp ? 1 : 0);
  const float under_scale[2] = {
      (view_color && g_gpu.winx > 0) ?
          float(GPU_texture_width(view_color)) / float(g_gpu.winx) :
          1.0f,
      (view_color && g_gpu.winy > 0) ?
          float(GPU_texture_height(view_color)) / float(g_gpu.winy) :
          1.0f,
  };
  GPU_shader_uniform_2fv(g_gpu.blit_shader, "underlay_scale", under_scale);
  GPU_texture_bind(g_gpu.overlay_tx, GPU_shader_get_sampler_binding(g_gpu.blit_shader, "image"));
  gpu::Texture *under = view_color ? view_color : g_gpu.overlay_tx;
  GPU_texture_bind(under, GPU_shader_get_sampler_binding(g_gpu.blit_shader, "underlay"));
  GPU_batch_draw(g_gpu.fs_batch);
  GPU_texture_unbind(g_gpu.overlay_tx);
  GPU_texture_unbind(under);
  GPU_blend(GPU_BLEND_NONE);
}

static void overlay_callback_undo_pre(Main * /*bmain*/,
                                      PointerRNA ** /*pointers*/,
                                      int /*pointers_num*/,
                                      void * /*arg*/)
{
  /* Screen-space strokes stay on the overlay until the view changes.
   * Ctrl+Z of those strokes is handled before the memfile step. Discarding
   * here would erase paint that has not been written to the image yet. */
}

static void overlay_callback_commit(Main * /*bmain*/,
                                    PointerRNA ** /*pointers*/,
                                    int /*pointers_num*/,
                                    void * /*arg*/)
{
  paint_proj_gpu_overlay_project_ex(nullptr, true);
}

static void overlay_callback_exit(Main * /*bmain*/,
                                  PointerRNA ** /*pointers*/,
                                  int /*pointers_num*/,
                                  void * /*arg*/)
{
  /* EXIT_PRE runs before the draw-manager GPU context is rebound for GPU_exit.
   * Do not bake or GPU_TEXTURE_FREE here — that abort()s on quit. Drop the
   * pending overlay and detach region callbacks while ARegionType is alive.
   * GL objects are freed later by #paint_proj_gpu_exit. */
  g_gpu.overlay_has_paint = false;
  g_gpu.ibuf = nullptr;
  g_gpu.ima = nullptr;
  g_gpu.ob = nullptr;
  g_gpu.force_project = false;
  if (g_gpu.art) {
    if (g_gpu.draw_handle_pre) {
      ED_region_draw_cb_exit(g_gpu.art, g_gpu.draw_handle_pre);
      g_gpu.draw_handle_pre = nullptr;
    }
    if (g_gpu.draw_handle_post) {
      ED_region_draw_cb_exit(g_gpu.art, g_gpu.draw_handle_post);
      g_gpu.draw_handle_post = nullptr;
    }
    g_gpu.art = nullptr;
  }
  g_gpu.draw_cb_registered = false;
  g_gpu.region = nullptr;
}

static void paint_proj_gpu_ensure_app_callbacks()
{
  if (g_gpu.undo_cb_registered) {
    return;
  }
  g_cb_undo_pre.func = overlay_callback_undo_pre;
  g_cb_undo_pre.alloc = false;
  BKE_callback_add(&g_cb_undo_pre, BKE_CB_EVT_UNDO_PRE);

  g_cb_save_pre.func = overlay_callback_commit;
  g_cb_save_pre.alloc = false;
  BKE_callback_add(&g_cb_save_pre, BKE_CB_EVT_SAVE_PRE);

  g_cb_exit_pre.func = overlay_callback_exit;
  g_cb_exit_pre.alloc = false;
  BKE_callback_add(&g_cb_exit_pre, BKE_CB_EVT_EXIT_PRE);

  g_gpu.undo_cb_registered = true;
}

static void paint_proj_gpu_ensure_draw_callbacks(ARegion *region)
{
  if (g_gpu.draw_cb_registered || region == nullptr || region->runtime == nullptr ||
      region->runtime->type == nullptr)
  {
    return;
  }
  ARegionType *art = region->runtime->type;
  g_gpu.art = art;
  g_gpu.draw_handle_pre = ED_region_draw_cb_activate(
      art, overlay_pre_view, nullptr, REGION_DRAW_PRE_VIEW);
  g_gpu.draw_handle_post = ED_region_draw_cb_activate(
      art, overlay_post_pixel, nullptr, REGION_DRAW_POST_PIXEL);
  g_gpu.draw_cb_registered = true;
}

void paint_proj_gpu_stroke_end()
{
  g_gpu.stroke_active = false;
  /* Mouse-up redraws with DRAWING already cleared. Re-snapshot the view so a
   * tiny quat/ofs drift during the stroke is not treated as "user rotated". */
  if (g_gpu.overlay_has_paint && g_gpu.region) {
    overlay_refresh_view(g_gpu.region);
  }
  GPU_framebuffer_restore();
  GPU_blend(GPU_BLEND_NONE);
  GPU_depth_test(GPU_DEPTH_LESS_EQUAL);
  GPU_face_culling(GPU_CULL_NONE);
  GPU_front_facing(false);
  GPU_depth_mask(true);
  GPU_write_mask(GPU_WRITE_COLOR | GPU_WRITE_DEPTH);
  GPU_scissor_test(false);
  GPU_shader_unbind();
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Shaders
 * \{ */

static constexpr const char *fs_vert_src = R"(
void main()
{
  gl_Position = float4(pos, 0.5f, 1.0f);
}
)";

/* Straight-alpha mix_byte analogue: RGB stays the paint/sample color; A is coverage.
 * Premultiplying into a cleared-black target is what made hard brushes show a dark halo. */
static constexpr const char *stamp_frag_src = R"(
float4 mix_coverage(float4 dst, float3 src_rgb, float src_a)
{
  float new_a = clamp(src_a + dst.a * (1.0f - src_a), 0.0f, 1.0f);
  float3 new_rgb = (new_a > 1.0e-6f) ?
                       (src_rgb * src_a + dst.rgb * dst.a * (1.0f - src_a)) / new_a :
                       dst.rgb;
  return float4(new_rgb, new_a);
}

float4 fetch_clamp(sampler2D s, int2 p, int2 size)
{
  return texelFetch(s, clamp(p, int2(0), size - 1), 0);
}

float4 sample_premul(sampler2D s, float2 px, int2 size)
{
  /* Premul bilinear: empty (0,0,0,0) taps only lower alpha, they do not tint RGB
   * toward black (smear-from-outside was dragging that fringe along the stroke). */
  float2 q = px - 0.5f;
  int2 p0 = int2(floor(q));
  float2 f = fract(q);
  float4 t00 = fetch_clamp(s, p0, size);
  float4 t10 = fetch_clamp(s, p0 + int2(1, 0), size);
  float4 t01 = fetch_clamp(s, p0 + int2(0, 1), size);
  float4 t11 = fetch_clamp(s, p0 + int2(1, 1), size);
  t00 = float4(t00.rgb * t00.a, t00.a);
  t10 = float4(t10.rgb * t10.a, t10.a);
  t01 = float4(t01.rgb * t01.a, t01.a);
  t11 = float4(t11.rgb * t11.a, t11.a);
  float4 m = mix(mix(t00, t10, f.x), mix(t01, t11, f.x), f.y);
  m.rgb = (m.a > 1.0e-6f) ? m.rgb / m.a : float3(0.0f);
  return m;
}

float3 sample_appearance(int2 p, int2 size)
{
  float4 over = fetch_clamp(overlay_src, p, size);
  float4 under = fetch_clamp(screen_canvas, p, size);
  return mix(under.rgb, over.rgb, over.a);
}

float3 sample_appearance_f(float2 px, int2 size)
{
  float4 over = sample_premul(overlay_src, px, size);
  float4 under = sample_premul(screen_canvas, px, size);
  return mix(under.rgb, over.rgb, over.a);
}

void main()
{
  float dist;
  if (disk_stamp != 0) {
    /* Airbrush / Dots / Smear: one dab at the current sample. A capsule from
     * mouse_prev fills the gaps and looks like Paint Soft/Hard. */
    dist = length(gl_FragCoord.xy - mouse);
  }
  else {
    float2 pa = gl_FragCoord.xy - mouse_prev;
    float2 ba = mouse - mouse_prev;
    float h = clamp(dot(pa, ba) / max(dot(ba, ba), 1.0e-8f), 0.0f, 1.0f);
    dist = length(pa - ba * h);
  }
  if (dist > brush_radius) {
    discard;
  }
  float t = clamp(dist / max(brush_radius, 1.0e-6f), 0.0f, 1.0f);
  float ct = t * 63.0f;
  int ci0 = int(clamp(floor(ct), 0.0f, 63.0f));
  int ci1 = int(clamp(float(ci0 + 1), 0.0f, 63.0f));
  float falloff = mix(texelFetch(curve_tx, int2(ci0, 0), 0).r,
                      texelFetch(curve_tx, int2(ci1, 0), 0).r,
                      fract(ct));
  float src_a = clamp(falloff * brush_alpha, 0.0f, 1.0f);
  if (hard_edge != 0) {
    /* Binary disk. Partial coverage at the 1px screen edge is what baked into a
     * fat UV mix-with-black halo when the surface was magnified. */
    src_a = (src_a >= 0.5f) ? clamp(brush_alpha, 0.0f, 1.0f) : 0.0f;
  }
  if (src_a < 1.0e-5f) {
    discard;
  }

  int2 p = int2(gl_FragCoord.xy);
  int2 size = textureSize(overlay_src, 0);
  float4 dst = texelFetch(overlay_src, p, 0);
  float3 src_rgb = paint_color;

  if (brush_type == 2) {
    /* Screen-space smear: dest must be on the mesh. If the pickup is off-mesh
     * (stroke coming in from empty screen), walk toward dest until we hit the
     * silhouette instead of dragging cleared-black along the path. */
    float mesh_a = fetch_clamp(screen_canvas, p, size).a;
    if (mesh_a < 1.0e-5f) {
      discard;
    }
    float2 src_px = gl_FragCoord.xy - (mouse - mouse_prev);
    float4 under_s = sample_premul(screen_canvas, src_px, size);
    float4 over_s = sample_premul(overlay_src, src_px, size);
    if (under_s.a < 0.2f) {
      bool hit = false;
      for (int i = 1; i <= 8; i++) {
        float t = float(i) / 8.0f;
        float2 q = mix(src_px, gl_FragCoord.xy, t);
        under_s = sample_premul(screen_canvas, q, size);
        over_s = sample_premul(overlay_src, q, size);
        if (under_s.a >= 0.2f) {
          hit = true;
          break;
        }
      }
      if (!hit) {
        discard;
      }
    }
    src_rgb = mix(under_s.rgb, over_s.rgb, over_s.a);
    src_a *= mesh_a;
  }
  else if (brush_type == 3) {
    /* Clone. */
    int2 sp = p - int2(round(clone_offset));
    src_rgb = sample_appearance(sp, size);
  }
  else if (brush_type == 1) {
    /* Screen-space blur / sharpen. Kernel scales with radius; only mesh pixels. */
    float mesh_a = fetch_clamp(screen_canvas, p, size).a;
    if (mesh_a < 0.5f) {
      discard;
    }
    const int kmax = 5;
    int k = int(clamp(brush_radius * 0.12f, 1.0f, float(kmax)));
    float3 acc = float3(0.0f);
    float wsum = 0.0f;
    for (int y = -kmax; y <= kmax; y++) {
      for (int x = -kmax; x <= kmax; x++) {
        if (abs(x) > k || abs(y) > k) {
          continue;
        }
        float2 o = float2(x, y);
        float w = 1.0f - length(o) / (float(k) + 0.001f);
        if (w <= 0.0f) {
          continue;
        }
        float2 q = gl_FragCoord.xy + o;
        float ma = sample_premul(screen_canvas, q, size).a;
        if (ma < 0.5f) {
          continue;
        }
        acc += sample_appearance_f(q, size) * w;
        wsum += w;
      }
    }
    float3 blurred = (wsum > 1.0e-6f) ? acc / wsum : sample_appearance(p, size);
    float3 center = sample_appearance(p, size);
    src_rgb = (sharpen != 0) ? clamp(center * 2.0f - blurred, 0.0f, 1.0f) : blurred;
    src_a *= mesh_a;
  }

  if (brush_type == 0 && (blend == 1 || blend == 2)) {
    /* Add/Sub: RGB is the accumulated paint*coverage addend (official
     * dest +/- paint * mask). Alpha tracks combined coverage. */
    fragColor = float4(clamp(dst.rgb + src_rgb * src_a, 0.0f, 1.0f),
                       clamp(dst.a + src_a, 0.0f, 1.0f));
  }
  else {
    fragColor = mix_coverage(dst, src_rgb, src_a);
  }
}
)";

static constexpr const char *blit_frag_src = R"(
void main()
{
  int2 p = int2(gl_FragCoord.xy);
  float4 over = texelFetch(image, p, 0);
  if (over.a < 1.0e-5f) {
    discard;
  }

  float3 src_srgb = over.rgb;
  if (src_is_linear != 0 && canvas_is_data == 0) {
    src_srgb = linear_to_srgb(clamp(src_srgb, 0.0f, 1.0f));
  }

  /* POST_PIXEL uses GPU_framebuffer_bind (sRGB ON). Output linear so the
   * hardware encode shows the picker color. Writing sRGB here became neon RGB. */
  if (brush_type == 0 && blend == 6) {
    /* Erase: Mix/Add blit against dest RGB left the stroke invisible on the
     * mesh (looked like the object occluded it). Darken on top with falloff. */
    fragColor = float4(0.0f, 0.0f, 0.0f, over.a);
    return;
  }

  /* dest_lerp: Mix/Smear/Blur lerp in display-referred space (official mix_byte).
   * Linear SRC_ALPHA over a lit mesh made the falloff a dark glaze. */
  if (dest_lerp == 0) {
    fragColor = float4(srgb_to_linear(src_srgb), over.a);
    return;
  }

  int2 up = int2(floor(float2(p) * underlay_scale));
  int2 usize = textureSize(underlay, 0);
  up = clamp(up, int2(0), usize - 1);
  float4 dest = texelFetch(underlay, up, 0);
  const bool blend_srgb = (is_float == 0 && canvas_is_data == 0);
  float3 dest_srgb = dest.rgb;
  if (blend_srgb) {
    dest_srgb = linear_to_srgb(clamp(dest.rgb, 0.0f, 1.0f));
  }

  float3 disp_srgb = dest_srgb;
  if (brush_type != 0 || blend == 0) {
    disp_srgb = mix(dest_srgb, src_srgb, over.a);
  }
  else if (blend == 1) {
    disp_srgb = clamp(dest_srgb + src_srgb, 0.0f, 1.0f);
  }
  else if (blend == 2) {
    disp_srgb = max(dest_srgb - src_srgb, 0.0f);
  }
  else {
    float4 out_s = blend_straight(float4(dest_srgb, 1.0f), float4(src_srgb, over.a), blend);
    disp_srgb = out_s.rgb;
  }
  fragColor = float4(srgb_to_linear(disp_srgb), 1.0f);
}
)";

static constexpr const char *depth_vert_src = R"(
void main()
{
  float2 ndc = screen.xy * viewport_inv * 2.0f - 1.0f;
  gl_Position = float4(ndc, 0.5f, 1.0f);
  v_z = screen.z;
}
)";

static constexpr const char *depth_frag_src = R"(
void main()
{
  fragZ = v_z;
}
)";

static constexpr const char *canvas_vert_src = R"(
void main()
{
  v_uv = uv;
  v_screen = screen.xyz;
  float2 ndc = screen.xy * viewport_inv * 2.0f - 1.0f;
  gl_Position = float4(ndc, 0.5f, 1.0f);
}
)";

static constexpr const char *canvas_frag_src = R"(
void main()
{
  /* Same space as the min-z pass (screen raster). Must stay strict so smear/blur
   * never pick the back of the mesh. */
  if (do_occlude != 0) {
    int2 overlay_size = textureSize(depth_tx, 0);
    int2 sp = int2(floor(v_screen.xy));
    if (any(lessThan(sp, int2(0))) || any(greaterThanEqual(sp, overlay_size))) {
      discard;
    }
    float closest = texelFetch(depth_tx, sp, 0).r;
    if (closest > 1.0e9f || v_screen.z > closest + 1.0e-4f) {
      discard;
    }
  }
  /* Nearest UV texel. Bilinear orig bleed into black areas showed up as a
   * faint tint while blurring, then disappeared after bake. */
  int2 ts = max(textureSize(orig_tx, 0), int2(1));
  int2 tp = clamp(int2(floor(v_uv * float2(ts))), int2(0), ts - 1);
  fragColor = texelFetch(orig_tx, tp, 0);
}
)";

static constexpr const char *bake_vert_src = R"(
void main()
{
  /* Rasterize in UV (clip w = 1). Interpolate window xy * clip_w so the fragment
   * can recover perspective-correct screen position. Affine lerp of already-divided
   * window xy misses the overlay by 1px+ up close, which is a fat falloff ring on
   * thin strokes. */
  float w = max(screen.w, 1.0e-8f);
  v_screen = float4(screen.xy * w, screen.z * w, w);
  gl_Position = float4(uv * 2.0f - 1.0f, 0.5f, 1.0f);
}
)";

/* Straight-alpha IMB blend modes (byte painter), plus linear<->sRGB for Draw. */
static constexpr const char *blend_lib_src = R"(
float linear_to_srgb_ch(float c)
{
  c = max(c, 0.0f);
  return (c <= 0.0031308f) ? (c * 12.92f) : (1.055f * pow(c, 1.0f / 2.4f) - 0.055f);
}
float3 linear_to_srgb(float3 c)
{
  return float3(linear_to_srgb_ch(c.r), linear_to_srgb_ch(c.g), linear_to_srgb_ch(c.b));
}
float srgb_to_linear_ch(float c)
{
  return (c <= 0.04045f) ? (c * (1.0f / 12.92f)) : pow((c + 0.055f) * (1.0f / 1.055f), 2.4f);
}
float3 srgb_to_linear(float3 c)
{
  return float3(srgb_to_linear_ch(c.r), srgb_to_linear_ch(c.g), srgb_to_linear_ch(c.b));
}
float3 rgb_to_hsv(float3 c)
{
  float4 k = float4(0.0f, -1.0f / 3.0f, 2.0f / 3.0f, -1.0f);
  float4 p = mix(float4(c.bg, k.wz), float4(c.gb, k.xy), step(c.b, c.g));
  float4 q = mix(float4(p.xyw, c.r), float4(c.r, p.yzx), step(p.x, c.r));
  float d = q.x - min(q.w, q.y);
  float e = 1.0e-10f;
  return float3(abs(q.z + (q.w - q.y) / (6.0f * d + e)), d / (q.x + e), q.x);
}
float3 hsv_to_rgb(float3 c)
{
  float4 k = float4(1.0f, 2.0f / 3.0f, 1.0f / 3.0f, 3.0f);
  float3 p = abs(fract(c.xxx + k.xyz) * 6.0f - k.www);
  return c.z * mix(k.xxx, clamp(p - k.xxx, 0.0f, 1.0f), c.y);
}
float overlay_ch(float d, float s)
{
  return (d > 0.5f) ? (1.0f - (1.0f - 2.0f * (d - 0.5f)) * (1.0f - s)) : (2.0f * d * s);
}
float colorburn_ch(float d, float s)
{
  return (s == 0.0f) ? 0.0f : max(1.0f - (1.0f - d) / s, 0.0f);
}
float colordodge_ch(float d, float s)
{
  return (s >= 1.0f) ? 1.0f : min(d / max(1.0f - s, 1.0e-6f), 1.0f);
}
float softlight_ch(float d, float s)
{
  if (s < 0.5f) {
    return d - (1.0f - 2.0f * s) * d * (1.0f - d);
  }
  float m = (d < 0.25f) ? ((16.0f * d - 12.0f) * d + 4.0f) * d : sqrt(d);
  return d + (2.0f * s - 1.0f) * (m - d);
}
float pinlight_ch(float d, float s)
{
  return (s > 0.5f) ? max(d, 2.0f * s - 1.0f) : min(d, 2.0f * s);
}
float vivid_ch(float d, float s)
{
  if (s == 1.0f) {
    return (d == 0.0f) ? 0.5f : 1.0f;
  }
  if (s == 0.0f) {
    return (d == 1.0f) ? 0.5f : 0.0f;
  }
  if (s > 0.5f) {
    return min(d / (2.0f * (1.0f - s)), 1.0f);
  }
  return max(1.0f - (1.0f - d) / (2.0f * s), 0.0f);
}
float4 blend_straight(float4 dst, float4 src, int mode)
{
  float t = src.a;
  if (t < 0.001f) {
    return dst;
  }
  float mt = 1.0f - t;
  float3 d = dst.rgb;
  float3 s = src.rgb;
  float3 r = d;
  float na = dst.a;
  if (mode == 0) {
    na = clamp(mt * dst.a + t, 0.0f, 1.0f);
    r = (na > 1.0e-6f) ? (mt * dst.a * d + t * s) / na : d;
  }
  else if (mode == 1) {
    r = clamp(d + s * t, 0.0f, 1.0f);
  }
  else if (mode == 2) {
    r = max(d - s * t, 0.0f);
  }
  else if (mode == 3) {
    r = mt * d + t * d * s;
  }
  else if (mode == 4) {
    r = mt * d + t * max(d, s);
  }
  else if (mode == 5) {
    r = mt * d + t * min(d, s);
  }
  else if (mode == 6) {
    r = d;
    na = max(dst.a - t, 0.0f);
  }
  else if (mode == 7) {
    r = d;
    na = min(dst.a + t, 1.0f);
  }
  else if (mode == 8) {
    r = clamp(mt * d + t * float3(overlay_ch(d.r, s.r), overlay_ch(d.g, s.g), overlay_ch(d.b, s.b)),
              0.0f,
              1.0f);
  }
  else if (mode == 9) {
    r = clamp(mt * d + t * float3(overlay_ch(s.r, d.r), overlay_ch(s.g, d.g), overlay_ch(s.b, d.b)),
              0.0f,
              1.0f);
  }
  else if (mode == 10) {
    r = clamp(mt * d +
                  t * float3(colorburn_ch(d.r, s.r), colorburn_ch(d.g, s.g), colorburn_ch(d.b, s.b)),
              0.0f,
              1.0f);
  }
  else if (mode == 11) {
    r = clamp(mt * d + t * (d + s - 1.0f), 0.0f, 1.0f);
  }
  else if (mode == 12) {
    r = clamp(mt * d + t * float3(colordodge_ch(d.r, s.r),
                                  colordodge_ch(d.g, s.g),
                                  colordodge_ch(d.b, s.b)),
              0.0f,
              1.0f);
  }
  else if (mode == 13) {
    r = clamp(mt * d + t * (1.0f - (1.0f - d) * (1.0f - s)), 0.0f, 1.0f);
  }
  else if (mode == 14) {
    r = clamp(mt * d + t * float3(softlight_ch(d.r, s.r),
                                  softlight_ch(d.g, s.g),
                                  softlight_ch(d.b, s.b)),
              0.0f,
              1.0f);
  }
  else if (mode == 15) {
    r = clamp(mt * d +
                  t * float3(pinlight_ch(d.r, s.r), pinlight_ch(d.g, s.g), pinlight_ch(d.b, s.b)),
              0.0f,
              1.0f);
  }
  else if (mode == 16) {
    r = clamp(mt * d +
                  t * float3(vivid_ch(d.r, s.r), vivid_ch(d.g, s.g), vivid_ch(d.b, s.b)),
              0.0f,
              1.0f);
  }
  else if (mode == 17) {
    r = clamp(mt * d + t * (d + 2.0f * s - 1.0f), 0.0f, 1.0f);
  }
  else if (mode == 18) {
    r = mt * d + t * abs(d - s);
  }
  else if (mode == 19) {
    /* Official blend_color_exclusion_float: 0.5 - 2*(d-0.5)*(s-0.5). */
    float3 ex = float3(0.5f) - (2.0f * (d - 0.5f) * (s - 0.5f));
    r = mt * d + t * ex;
  }
  else if (mode >= 20 && mode <= 23) {
    float3 h1 = rgb_to_hsv(d);
    float3 h2 = rgb_to_hsv(s);
    if (mode == 20) {
      h1.x = h2.x;
    }
    else if (mode == 21) {
      if (h1.y > 0.0005f) {
        h1.y = h2.y;
      }
    }
    else if (mode == 22) {
      h1.z = h2.z;
    }
    else {
      h1.x = h2.x;
      h1.y = h2.y;
    }
    r = clamp(mt * d + t * hsv_to_rgb(h1), 0.0f, 1.0f);
  }
  else {
    na = clamp(mt * dst.a + t, 0.0f, 1.0f);
    r = (na > 1.0e-6f) ? (mt * dst.a * d + t * s) / na : d;
  }
  return float4(r, na);
}
)";

static constexpr const char *bake_frag_src = R"(
float4 overlay_texel(int2 q, int2 size, float z)
{
  if (any(lessThan(q, int2(0))) || any(greaterThanEqual(q, size))) {
    return float4(0.0f);
  }
  float4 o = texelFetch(overlay_tx, q, 0);
  if (do_occlude != 0) {
    float d = texelFetch(depth_tx, q, 0).r;
    /* Air only. Do not z-test taps here — UV-interpolated z vs screen min-z
     * disagreement punched a texel grid, and a loose epsilon sampled backs. */
    if (d > 1.0e9f) {
      return float4(0.0f);
    }
  }
  return o;
}

/* Bilinear overlay. Mixing painted texels with empty neighbors is the 8K
 * 毛边 (one screen pixel of silhouette AA becomes many UV texels). Fall back
 * to nearest whenever any tap is empty. */
float4 overlay_bilinear(float2 screen_px, int2 size, float z, bool addend)
{
  float2 p = screen_px - 0.5f;
  int2 p0 = int2(floor(p));
  float2 f = fract(p);
  float4 s00 = overlay_texel(p0, size, z);
  float4 s10 = overlay_texel(p0 + int2(1, 0), size, z);
  float4 s01 = overlay_texel(p0 + int2(0, 1), size, z);
  float4 s11 = overlay_texel(p0 + int2(1, 1), size, z);
  if (s00.a < 1.0e-5f || s10.a < 1.0e-5f || s01.a < 1.0e-5f || s11.a < 1.0e-5f) {
    return overlay_texel(int2(floor(screen_px)), size, z);
  }
  if (!addend) {
    s00 = float4(s00.rgb * s00.a, s00.a);
    s10 = float4(s10.rgb * s10.a, s10.a);
    s01 = float4(s01.rgb * s01.a, s01.a);
    s11 = float4(s11.rgb * s11.a, s11.a);
  }
  float4 m = mix(mix(s00, s10, f.x), mix(s01, s11, f.x), f.y);
  if (!addend) {
    m.rgb = (m.a > 1.0e-6f) ? m.rgb / m.a : float3(0.0f);
  }
  return m;
}

void main()
{
  int2 p = int2(gl_FragCoord.xy);
  float4 orig = texelFetch(orig_tx, p, 0);
  int2 overlay_size = textureSize(overlay_tx, 0);
  float w = max(v_screen.w, 1.0e-8f);
  float2 scr = v_screen.xy / w;
  float z = v_screen.z / w;
  int mode = (brush_type == 0) ? blend : 0;
  float4 over = overlay_bilinear(scr, overlay_size, z, mode == 1 || mode == 2);
  if (over.a < 1.0e-5f) {
    discard;
  }
  if (do_occlude != 0) {
    /* Closest of 2x2 screen depths. Back faces sit well behind the min; a
     * 1e-4 single-tap test also discarded the front (interpolation mismatch)
     * and left a UV hole grid. */
    int2 p0 = int2(floor(scr - 0.5f));
    float dmin = 1.0e20f;
    for (int y = 0; y <= 1; y++) {
      for (int x = 0; x <= 1; x++) {
        int2 q = p0 + int2(x, y);
        if (any(lessThan(q, int2(0))) || any(greaterThanEqual(q, overlay_size))) {
          continue;
        }
        float d = texelFetch(depth_tx, q, 0).r;
        if (d < dmin) {
          dmin = d;
        }
      }
    }
    if (dmin > 1.0e9f || z > dmin + 1.0e-3f) {
      discard;
    }
  }

  float3 src_rgb = over.rgb;
  if (src_is_linear != 0 && canvas_is_data == 0 && is_float == 0) {
    src_rgb = linear_to_srgb(clamp(src_rgb, 0.0f, 1.0f));
  }
  else if (src_is_linear == 0 && is_float != 0 && canvas_is_data == 0) {
    src_rgb = srgb_to_linear(src_rgb);
  }

  float4 orig_s = orig;
  if (is_float != 0) {
    orig_s = (orig.a > 1.0e-6f) ? float4(orig.rgb / orig.a, orig.a) : float4(0.0f);
  }

  float cov = over.a;
  /* Empty overlay RGB with leftover coverage would mix the canvas toward black
   * (a fat dark band around thin strokes). */
  if (mode == 0 && dot(src_rgb, src_rgb) < 1.0e-6f) {
    discard;
  }
  float4 out_s;
  if (mode == 0) {
    /* Straight-alpha over. Opaque Background stays lerp(RGB)+keep-A; empty
     * layers (alpha 0) raise coverage so Screen Space paint is visible. */
    float na = cov + orig_s.a * (1.0f - cov);
    out_s.rgb = (na > 1.0e-6f) ? ((src_rgb * cov + orig_s.rgb * orig_s.a * (1.0f - cov)) / na) :
                                 orig_s.rgb;
    out_s.a = na;
  }
  else if (mode == 1) {
    /* overlay.rgb is accumulated paint*coverage. */
    out_s = float4(clamp(orig_s.rgb + src_rgb, 0.0f, 1.0f), min(orig_s.a + cov, 1.0f));
  }
  else if (mode == 2) {
    out_s = float4(max(orig_s.rgb - src_rgb, 0.0f), max(orig_s.a, cov));
  }
  else {
    out_s = blend_straight(orig_s, float4(src_rgb, cov), mode);
  }
  if (lock_alpha != 0) {
    out_s.a = orig_s.a;
  }

  if (is_float != 0) {
    fragColor = float4(out_s.rgb * out_s.a, out_s.a);
  }
  else {
    fragColor = out_s;
  }
}
)";

static constexpr const char *dilate_frag_src = R"(
void main()
{
  int2 p = int2(gl_FragCoord.xy);
  int2 size = textureSize(paint_tx, 0);
  float4 c = texelFetch(paint_tx, p, 0);
  float4 o = texelFetch(orig_tx, p, 0);
  float painted = distance(c.rgb, o.rgb) + abs(c.a - o.a);
  if (painted > 0.002f) {
    fragColor = c;
    return;
  }
  /* Do NOT grow the stroke across island texels — that dilated falloff into a
   * ring thicker than a thin brush. Only bleed into UV padding (empty orig). */
  const bool island = (o.a > 0.05f) || (dot(o.rgb, o.rgb) > 0.002f);
  if (island) {
    fragColor = c;
    return;
  }
  float4 best = c;
  float bestp = 0.0f;
  for (int y = -1; y <= 1; y++) {
    for (int x = -1; x <= 1; x++) {
      if (x == 0 && y == 0) {
        continue;
      }
      int2 q = clamp(p + int2(x, y), int2(0), size - 1);
      float4 n = texelFetch(paint_tx, q, 0);
      float4 no = texelFetch(orig_tx, q, 0);
      float np = distance(n.rgb, no.rgb) + abs(n.a - no.a);
      if (np > bestp) {
        bestp = np;
        best = n;
      }
    }
  }
  fragColor = (bestp > 0.002f) ? best : c;
}
)";

static gpu::Shader *compile_generated_shader(ShaderCreateInfo &info)
{
  return GPU_shader_create_from_info_python(reinterpret_cast<const GPUShaderCreateInfo *>(&info));
}

static bool paint_proj_gpu_ensure_shaders()
{
  if (g_gpu.shader_failed) {
    return false;
  }
  if (g_gpu.stamp_shader && g_gpu.depth_shader && g_gpu.bake_shader && g_gpu.dilate_shader &&
      g_gpu.blit_shader && g_gpu.canvas_shader)
  {
    return true;
  }

  {
    ShaderCreateInfo info("pyGPU_Shader");
    info.builtins(gpu::shader::BuiltinBits::FRAG_COORD);
    info.vertex_in(0, Type::float2_t, "pos");
    info.fragment_out(0, Type::float4_t, "fragColor");
    info.sampler(0, gpu::shader::ImageType::Float2D, "curve_tx");
    info.sampler(1, gpu::shader::ImageType::Float2D, "overlay_src");
    info.sampler(2, gpu::shader::ImageType::Float2D, "screen_canvas");
    info.push_constant(Type::float2_t, "mouse");
    info.push_constant(Type::float2_t, "mouse_prev");
    info.push_constant(Type::float_t, "brush_radius");
    info.push_constant(Type::float_t, "brush_alpha");
    info.push_constant(Type::float3_t, "paint_color");
    info.push_constant(Type::int_t, "brush_type");
    info.push_constant(Type::int_t, "blend");
    info.push_constant(Type::float2_t, "clone_offset");
    info.push_constant(Type::int_t, "sharpen");
    info.push_constant(Type::int_t, "hard_edge");
    info.push_constant(Type::int_t, "disk_stamp");
    info.vertex_source_generated = fs_vert_src;
    info.fragment_source_generated = stamp_frag_src;
    g_gpu.stamp_shader = compile_generated_shader(info);
  }

  {
    StageInterfaceInfo iface("paint_proj_ovl_depth_iface", "");
    iface.smooth(Type::float_t, "v_z");

    ShaderCreateInfo info("paint_proj_ovl_depth");
    info.vertex_in(0, Type::float2_t, "uv");
    info.vertex_in(1, Type::float4_t, "screen");
    info.vertex_out(iface);
    info.fragment_out(0, Type::float_t, "fragZ");
    info.push_constant(Type::float2_t, "viewport_inv");
    info.vertex_source_generated = depth_vert_src;
    info.fragment_source_generated = depth_frag_src;
    g_gpu.depth_shader = compile_generated_shader(info);
  }

  {
    StageInterfaceInfo iface("paint_proj_ovl_canvas_iface", "");
    iface.smooth(Type::float2_t, "v_uv");
    iface.smooth(Type::float3_t, "v_screen");

    ShaderCreateInfo info("pyGPU_Shader");
    info.vertex_in(0, Type::float2_t, "uv");
    info.vertex_in(1, Type::float4_t, "screen");
    info.vertex_out(iface);
    info.fragment_out(0, Type::float4_t, "fragColor");
    info.sampler(0, gpu::shader::ImageType::Float2D, "orig_tx");
    info.sampler(1, gpu::shader::ImageType::Float2D, "depth_tx");
    info.push_constant(Type::float2_t, "viewport_inv");
    info.push_constant(Type::int_t, "do_occlude");
    info.vertex_source_generated = canvas_vert_src;
    info.fragment_source_generated = canvas_frag_src;
    g_gpu.canvas_shader = compile_generated_shader(info);
  }

  {
    StageInterfaceInfo iface("paint_proj_ovl_bake_iface", "");
    iface.smooth(Type::float4_t, "v_screen");

    /* GPU_shader_create_from_info_python only expands CREATE_INFO_RES_PASS_pyGPU_Shader
     * (samplers). The name must match or orig_tx / overlay_tx / depth_tx are undeclared. */
    ShaderCreateInfo info("pyGPU_Shader");
    info.builtins(gpu::shader::BuiltinBits::FRAG_COORD);
    info.vertex_in(0, Type::float2_t, "uv");
    info.vertex_in(1, Type::float4_t, "screen");
    info.vertex_out(iface);
    info.fragment_out(0, Type::float4_t, "fragColor");
    info.sampler(0, gpu::shader::ImageType::Float2D, "orig_tx");
    info.sampler(1, gpu::shader::ImageType::Float2D, "overlay_tx");
    info.sampler(2, gpu::shader::ImageType::Float2D, "depth_tx");
    info.push_constant(Type::int_t, "do_occlude");
    info.push_constant(Type::int_t, "blend");
    info.push_constant(Type::int_t, "brush_type");
    info.push_constant(Type::int_t, "canvas_is_srgb");
    info.push_constant(Type::int_t, "canvas_is_data");
    info.push_constant(Type::int_t, "is_float");
    info.push_constant(Type::int_t, "lock_alpha");
    info.push_constant(Type::int_t, "src_is_linear");
    info.push_constant(Type::int_t, "hard_edge");
    info.vertex_source_generated = bake_vert_src;
    info.fragment_source_generated = std::string(blend_lib_src) + bake_frag_src;
    g_gpu.bake_shader = compile_generated_shader(info);
  }

  {
    ShaderCreateInfo info("pyGPU_Shader");
    info.builtins(gpu::shader::BuiltinBits::FRAG_COORD);
    info.vertex_in(0, Type::float2_t, "pos");
    info.fragment_out(0, Type::float4_t, "fragColor");
    info.sampler(0, gpu::shader::ImageType::Float2D, "paint_tx");
    info.sampler(1, gpu::shader::ImageType::Float2D, "orig_tx");
    info.vertex_source_generated = fs_vert_src;
    info.fragment_source_generated = dilate_frag_src;
    g_gpu.dilate_shader = compile_generated_shader(info);
  }

  {
    ShaderCreateInfo info("pyGPU_Shader");
    info.builtins(gpu::shader::BuiltinBits::FRAG_COORD);
    info.vertex_in(0, Type::float2_t, "pos");
    info.fragment_out(0, Type::float4_t, "fragColor");
    info.sampler(0, gpu::shader::ImageType::Float2D, "image");
    info.sampler(1, gpu::shader::ImageType::Float2D, "underlay");
    info.push_constant(Type::int_t, "src_is_linear");
    info.push_constant(Type::int_t, "canvas_is_srgb");
    info.push_constant(Type::int_t, "canvas_is_data");
    info.push_constant(Type::int_t, "is_float");
    info.push_constant(Type::int_t, "lock_alpha");
    info.push_constant(Type::int_t, "blend");
    info.push_constant(Type::int_t, "brush_type");
    info.push_constant(Type::int_t, "dest_lerp");
    info.push_constant(Type::float2_t, "underlay_scale");
    info.vertex_source_generated = fs_vert_src;
    info.fragment_source_generated = std::string(blend_lib_src) + blit_frag_src;
    g_gpu.blit_shader = compile_generated_shader(info);
  }

  if (!g_gpu.stamp_shader || !g_gpu.depth_shader || !g_gpu.bake_shader || !g_gpu.dilate_shader ||
      !g_gpu.blit_shader || !g_gpu.canvas_shader)
  {
    g_gpu.shader_failed = true;
    GPU_SHADER_FREE_SAFE(g_gpu.stamp_shader);
    GPU_SHADER_FREE_SAFE(g_gpu.depth_shader);
    GPU_SHADER_FREE_SAFE(g_gpu.bake_shader);
    GPU_SHADER_FREE_SAFE(g_gpu.dilate_shader);
    GPU_SHADER_FREE_SAFE(g_gpu.blit_shader);
    GPU_SHADER_FREE_SAFE(g_gpu.canvas_shader);
    return false;
  }

  if (!g_gpu.curve_tx) {
    g_gpu.curve_tx = GPU_texture_create_2d("paint_proj_ovl_curve",
                                           64,
                                           1,
                                           1,
                                           gpu::TextureFormat::UNORM_8,
                                           GPU_TEXTURE_USAGE_SHADER_READ,
                                           nullptr);
    if (g_gpu.curve_tx) {
      GPU_texture_filter_mode(g_gpu.curve_tx, false);
      GPU_texture_extend_mode(g_gpu.curve_tx, GPU_SAMPLER_EXTEND_MODE_EXTEND);
    }
  }

  if (!g_gpu.fs_batch) {
    GPUVertFormat format = {};
    GPU_vertformat_attr_add(&format, "pos", gpu::VertAttrType::SFLOAT_32_32);
    gpu::VertBuf *vbo = GPU_vertbuf_create_with_format_ex(format, GPU_USAGE_STATIC);
    const float pos[3][2] = {{-1.0f, -1.0f}, {3.0f, -1.0f}, {-1.0f, 3.0f}};
    GPU_vertbuf_data_alloc(*vbo, 3);
    GPU_vertbuf_attr_fill(vbo, 0, pos);
    g_gpu.fs_batch = GPU_batch_create_ex(GPU_PRIM_TRIS, vbo, nullptr, GPU_BATCH_OWNS_VBO);
  }
  return true;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Texture / FBO ensure
 * \{ */

static bool paint_proj_gpu_ensure_depth(int w, int h)
{
  if (g_gpu.depth_tx && g_gpu.depth_w == w && g_gpu.depth_h == h) {
    return true;
  }
  paint_proj_gpu_free_depth();
  g_gpu.depth_tx = GPU_texture_create_2d("paint_proj_ovl_depth",
                                         w,
                                         h,
                                         1,
                                         gpu::TextureFormat::SFLOAT_32,
                                         GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_SHADER_READ,
                                         nullptr);
  if (!g_gpu.depth_tx) {
    return false;
  }
  GPU_texture_filter_mode(g_gpu.depth_tx, false);
  GPU_framebuffer_ensure_config(&g_gpu.depth_fb,
                                {
                                    GPU_ATTACHMENT_NONE,
                                    GPU_ATTACHMENT_TEXTURE(g_gpu.depth_tx),
                                });
  g_gpu.depth_w = w;
  g_gpu.depth_h = h;
  return true;
}

static gpu::TextureFormat orig_format_from_ibuf(const ImBuf *ibuf)
{
  return ibuf->float_data() ? gpu::TextureFormat::SFLOAT_16_16_16_16 :
                              gpu::TextureFormat::UNORM_8_8_8_8;
}

static bool paint_proj_gpu_ensure_canvas(Image *ima, ImageUser *iuser, const ImBuf *ibuf)
{
  if (ibuf == nullptr || ibuf->x < 1 || ibuf->y < 1) {
    return false;
  }
  const int w = ibuf->x;
  const int h = ibuf->y;
  const int max_tex = GPU_max_texture_size();
  /* Full-canvas HOST_READ FBOs of 4K/8K routinely fail to allocate. CPU overlay
   * projection writes those images; keep GPU bake for modest canvases only. */
  if (w > max_tex || h > max_tex || size_t(w) * size_t(h) > size_t(2048) * size_t(2048)) {
    return false;
  }
  const bool is_float = ibuf->float_data() != nullptr;
  /* Always the ImBuf format. Copying the viewport GPU image (often SRGBA) into
   * UNORM made Mix sample linearized-black and write neon RGB. */
  const gpu::TextureFormat orig_fmt = orig_format_from_ibuf(ibuf);
  const gpu::TextureFormat paint_fmt = orig_fmt;
  (void)ima;
  (void)iuser;

  if (g_gpu.paint_tx && g_gpu.canvas_w == w && g_gpu.canvas_h == h && g_gpu.is_float == is_float &&
      g_gpu.orig_format == orig_fmt && g_gpu.paint_format == paint_fmt)
  {
    return true;
  }

  paint_proj_gpu_free_canvas();

  const eGPUTextureUsage orig_usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_SHADER_READ |
                                      GPU_TEXTURE_USAGE_HOST_READ;
  const eGPUTextureUsage paint_usage = GPU_TEXTURE_USAGE_ATTACHMENT |
                                       GPU_TEXTURE_USAGE_SHADER_READ |
                                       GPU_TEXTURE_USAGE_HOST_READ;

  g_gpu.orig_tx = GPU_texture_create_2d("paint_proj_ovl_orig", w, h, 1, orig_fmt, orig_usage, nullptr);
  g_gpu.paint_tx = GPU_texture_create_2d(
      "paint_proj_ovl_paint", w, h, 1, paint_fmt, paint_usage, nullptr);
  if (!g_gpu.orig_tx || !g_gpu.paint_tx) {
    paint_proj_gpu_free_canvas();
    return false;
  }
  GPU_texture_filter_mode(g_gpu.orig_tx, false);
  GPU_texture_filter_mode(g_gpu.paint_tx, true);
  GPU_texture_mipmap_mode(g_gpu.paint_tx, false, true);
  GPU_texture_extend_mode(g_gpu.paint_tx, GPU_SAMPLER_EXTEND_MODE_REPEAT);

  GPU_framebuffer_ensure_config(&g_gpu.apply_fb,
                                {
                                    GPU_ATTACHMENT_NONE,
                                    GPU_ATTACHMENT_TEXTURE(g_gpu.paint_tx),
                                });
  g_gpu.canvas_w = w;
  g_gpu.canvas_h = h;
  g_gpu.is_float = is_float;
  g_gpu.orig_format = orig_fmt;
  g_gpu.paint_format = paint_fmt;
  return true;
}

static void bind_apply_fb()
{
  /* Raw UNORM/float store — bake already works in canvas (sRGB byte) space. */
  GPU_framebuffer_bind_no_srgb(g_gpu.apply_fb);
}

static bool paint_proj_gpu_upload_orig(Image * /*ima*/, ImageUser * /*iuser*/, ImBuf *ibuf)
{
  /* CPU canvas only — same bytes official mix_byte reads. GPU image textures
   * are often SRGBA (sampled linear), which made Mix lerp against black. */
  if (ibuf == nullptr || g_gpu.orig_tx == nullptr || g_gpu.paint_tx == nullptr) {
    return false;
  }
  IMB_ensure_host_buffer(ibuf);
  if (ibuf->float_data()) {
    GPU_texture_update(g_gpu.orig_tx, GPU_DATA_FLOAT, ibuf->float_data());
    GPU_texture_update(g_gpu.paint_tx, GPU_DATA_FLOAT, ibuf->float_data());
  }
  else if (ibuf->byte_data()) {
    GPU_texture_update(g_gpu.orig_tx, GPU_DATA_UBYTE, ibuf->byte_data());
    GPU_texture_update(g_gpu.paint_tx, GPU_DATA_UBYTE, ibuf->byte_data());
  }
  else {
    return false;
  }
  return true;
}

static gpu::Texture *create_overlay_tex(const char *name, int w, int h)
{
  const eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_SHADER_READ |
                                 GPU_TEXTURE_USAGE_HOST_READ;
  gpu::Texture *tx = GPU_texture_create_2d(name, w, h, 1, overlay_format, usage, nullptr);
  if (tx) {
    GPU_texture_filter_mode(tx, false);
  }
  return tx;
}

static void overlay_clear_tx(gpu::FrameBuffer *fb)
{
  gpu::FrameBuffer *prev = GPU_framebuffer_active_get();
  GPU_framebuffer_bind(fb);
  const double4 clear_col(0.0, 0.0, 0.0, 0.0);
  GPU_framebuffer_clear_color(fb, clear_col);
  if (prev) {
    GPU_framebuffer_bind(prev);
  }
  else {
    GPU_framebuffer_restore();
  }
}

static bool paint_proj_gpu_ensure_overlay(int w, int h)
{
  if (g_gpu.overlay_tx && g_gpu.overlay_src_tx && g_gpu.overlay_w == w && g_gpu.overlay_h == h) {
    return true;
  }
  paint_proj_gpu_free_overlay_textures();

  g_gpu.overlay_tx = create_overlay_tex("paint_proj_ovl", w, h);
  g_gpu.overlay_src_tx = create_overlay_tex("paint_proj_ovl_src", w, h);
  if (!g_gpu.overlay_tx || !g_gpu.overlay_src_tx) {
    paint_proj_gpu_free_overlay_textures();
    return false;
  }
  GPU_framebuffer_ensure_config(&g_gpu.overlay_fb,
                                {
                                    GPU_ATTACHMENT_NONE,
                                    GPU_ATTACHMENT_TEXTURE(g_gpu.overlay_tx),
                                });
  overlay_clear_tx(g_gpu.overlay_fb);
  g_gpu.overlay_w = w;
  g_gpu.overlay_h = h;
  g_gpu.overlay_has_paint = false;
  BLI_rcti_init_minmax(&g_gpu.overlay_dirty);
  return true;
}

static bool overlay_view_changed(const ARegion *region)
{
  if (region == nullptr) {
    return true;
  }
  const RegionView3D *rv3d = static_cast<const RegionView3D *>(region->regiondata);
  if (rv3d == nullptr) {
    return true;
  }
  if (region->winx != g_gpu.view_winx || region->winy != g_gpu.view_winy) {
    return true;
  }
  if (int(rv3d->persp) != g_gpu.persp) {
    return true;
  }
  /* Exact viewmat equality is too noisy (ULP from the every-draw recompute). The user-facing
   * orbit/pan/zoom state is quat + dist + ofs + camera offsets. */
  if (fabsf(rv3d->dist - g_gpu.dist) > 1.0e-5f) {
    return true;
  }
  if (!compare_v3v3(rv3d->ofs, g_gpu.ofs, 1.0e-5f)) {
    return true;
  }
  /* q and -q are the same rotation; exact component compare would false-trigger a bake. */
  if (fabsf(dot_v4v4(rv3d->viewquat, g_gpu.viewquat)) < 0.99999f) {
    return true;
  }
  if (fabsf(rv3d->camzoom - g_gpu.camzoom) > 1.0e-5f) {
    return true;
  }
  if (fabsf(rv3d->camdx - g_gpu.camdx) > 1.0e-5f || fabsf(rv3d->camdy - g_gpu.camdy) > 1.0e-5f) {
    return true;
  }
  return false;
}

static void overlay_store_view(const ARegion *region, const ProjPaintGPUDab &dab)
{
  if (dab.viewmat) {
    copy_m4_m4(g_gpu.viewmat, dab.viewmat);
  }
  if (dab.winmat) {
    copy_m4_m4(g_gpu.winmat, dab.winmat);
  }
  if (dab.project_mat) {
    copy_m4_m4(g_gpu.project_mat, dab.project_mat);
  }
  g_gpu.winx = dab.winx;
  g_gpu.winy = dab.winy;
  g_gpu.region = dab.region;
  g_gpu.ob = dab.ob;
  g_gpu.do_occlude = dab.do_occlude;
  g_gpu.do_backfacecull = dab.do_backfacecull;
  g_gpu.is_ortho = dab.is_ortho;
  g_gpu.clip_start = dab.clip_start;
  g_gpu.seam_bleed_px = dab.seam_bleed_px;
  g_gpu.blend = dab.blend;
  g_gpu.brush_type = dab.brush_type;
  g_gpu.canvas_is_srgb = dab.canvas_is_srgb;
  g_gpu.canvas_is_data = dab.canvas_is_data;
  g_gpu.lock_alpha = dab.lock_alpha;
  g_gpu.src_is_linear = dab.src_is_linear;
  g_gpu.is_float = dab.ibuf && dab.ibuf->float_data() != nullptr;
  g_gpu.hard_edge = dab.hard_edge;
  g_gpu.ima = dab.ima;
  if (dab.iuser) {
    g_gpu.iuser = *dab.iuser;
  }
  g_gpu.ibuf = dab.ibuf;

  overlay_refresh_view(region);
}

static void overlay_refresh_view(const ARegion *region)
{
  if (region == nullptr) {
    return;
  }
  const RegionView3D *rv3d = static_cast<const RegionView3D *>(region->regiondata);
  if (rv3d == nullptr) {
    return;
  }
  g_gpu.view_winx = region->winx;
  g_gpu.view_winy = region->winy;
  copy_v4_v4(g_gpu.viewquat, rv3d->viewquat);
  copy_v3_v3(g_gpu.ofs, rv3d->ofs);
  g_gpu.dist = rv3d->dist;
  g_gpu.persp = int(rv3d->persp);
  g_gpu.camzoom = rv3d->camzoom;
  g_gpu.camdx = rv3d->camdx;
  g_gpu.camdy = rv3d->camdy;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Mesh upload
 * \{ */

static gpu::Batch *ensure_mesh_batch(Span<float2> uv, Span<float4> screen)
{
  const uint n = uint(uv.size());
  if (!g_gpu.mesh_vbo || g_gpu.mesh_vbo_alloc < n) {
    GPU_BATCH_DISCARD_SAFE(g_gpu.mesh_batch);
    g_gpu.mesh_vbo = nullptr;
    GPUVertFormat format = {};
    GPU_vertformat_attr_add(&format, "uv", gpu::VertAttrType::SFLOAT_32_32);
    GPU_vertformat_attr_add(&format, "screen", gpu::VertAttrType::SFLOAT_32_32_32_32);
    g_gpu.mesh_vbo = GPU_vertbuf_create_with_format_ex(format, GPU_USAGE_DYNAMIC);
    g_gpu.mesh_vbo_alloc = std::max(n, 256u);
    GPU_vertbuf_data_alloc(*g_gpu.mesh_vbo, g_gpu.mesh_vbo_alloc);
    g_gpu.mesh_batch = GPU_batch_create_ex(
        GPU_PRIM_TRIS, g_gpu.mesh_vbo, nullptr, GPU_BATCH_OWNS_VBO);
  }
  GPU_vertbuf_data_len_set(*g_gpu.mesh_vbo, n);
  GPU_vertbuf_attr_fill(g_gpu.mesh_vbo, 0, uv.data());
  GPU_vertbuf_attr_fill(g_gpu.mesh_vbo, 1, screen.data());
  return g_gpu.mesh_batch;
}

bool paint_proj_gpu_has_mesh()
{
  return g_gpu.mesh_ready && g_gpu.mesh_batch != nullptr;
}

bool paint_proj_gpu_upload_mesh(Span<float2> uv, Span<float4> screen)
{
  if (uv.is_empty() || uv.size() != screen.size()) {
    return false;
  }
  g_gpu.mesh_uv_cpu.clear();
  g_gpu.mesh_uv_cpu.extend(uv);
  g_gpu.mesh_screen_cpu.clear();
  g_gpu.mesh_screen_cpu.extend(screen);
  /* GPU VBO is optional: 8K bake uses the CPU copies. */
  if (ensure_mesh_batch(uv, screen) == nullptr) {
    g_gpu.mesh_batch = nullptr;
    g_gpu.mesh_vbo = nullptr;
  }
  g_gpu.mesh_ready = true;
  return true;
}

static bool project_stored_vert(const float pos[3], float4 &ss)
{
  if (g_gpu.is_ortho) {
    float co[3];
    mul_v3_m4v3(co, g_gpu.project_mat, pos);
    ss.x = float(g_gpu.winx) * 0.5f + float(g_gpu.winx) * 0.5f * co[0];
    ss.y = float(g_gpu.winy) * 0.5f + float(g_gpu.winy) * 0.5f * co[1];
    ss.z = co[2];
    ss.w = 1.0f;
    return true;
  }
  float co[4] = {pos[0], pos[1], pos[2], 1.0f};
  mul_m4_v4(g_gpu.project_mat, co);
  if (co[3] > g_gpu.clip_start) {
    ss.x = float(g_gpu.winx) * 0.5f + float(g_gpu.winx) * 0.5f * co[0] / co[3];
    ss.y = float(g_gpu.winy) * 0.5f + float(g_gpu.winy) * 0.5f * co[1] / co[3];
    ss.z = co[2] / co[3];
    ss.w = co[3];
    return true;
  }
  return false;
}

static bool paint_proj_gpu_pack_mesh(const Mesh *mesh, const Object *ob)
{
  if (mesh == nullptr || ob == nullptr || mesh->uv_map_names().is_empty() || mesh->faces_num < 1)
  {
    return false;
  }

  const Span<float3> positions = mesh->vert_positions();
  const Span<int> corner_verts = mesh->corner_verts();
  const Span<int3> corner_tris = mesh->corner_tris();
  if (corner_tris.is_empty()) {
    return false;
  }

  const bke::AttributeAccessor attributes = mesh->attributes();
  const StringRef uv_name = mesh->active_uv_map_name();
  const float2 *uv_map = nullptr;
  std::optional<float2> uv_single;
  if (const bke::GAttributeReader attr = attributes.lookup(uv_name)) {
    if (attr.domain == bke::AttrDomain::Corner && attr.varray.type().is<float2>()) {
      if (attr.varray.is_span()) {
        uv_map = attr.varray.get_internal_span().typed<float2>().data();
      }
      else if (attr.varray.is_single()) {
        uv_single = attr.varray.typed<float2>().get_internal_single();
      }
    }
  }
  if (uv_map == nullptr && !uv_single) {
    return false;
  }

  const bool flip = (ob->transflag & OB_NEG_SCALE) != 0;

  Vector<float2> vuv;
  Vector<float4> vscreen;
  vuv.reserve(size_t(corner_tris.size()) * 3);
  vscreen.reserve(size_t(corner_tris.size()) * 3);

  for (const int t : corner_tris.index_range()) {
    const int3 &tri = corner_tris[t];
    float2 u[3];
    float4 s[3];
    bool skip = false;
    for (int i = 0; i < 3; i++) {
      if (uv_single) {
        u[i] = *uv_single;
      }
      else {
        u[i] = uv_map[tri[i]];
      }
      if (u[i].x < -0.5f || u[i].x > 1.5f || u[i].y < -0.5f || u[i].y > 1.5f) {
        skip = true;
        break;
      }
      const int v = corner_verts[tri[i]];
      if (!project_stored_vert(positions[v], s[i])) {
        skip = true;
        break;
      }
    }
    if (skip) {
      continue;
    }
    if (g_gpu.do_backfacecull) {
      const float cross = (s[1].x - s[0].x) * (s[2].y - s[0].y) -
                          (s[1].y - s[0].y) * (s[2].x - s[0].x);
      const bool backface = flip ? (cross > 0.0f) : (cross < 0.0f);
      if (backface) {
        continue;
      }
    }
    /* Do not expand UVs here. Offsetting UV while keeping the original screen
     * positions warps the UV→screen map, so bake samples a different overlay
     * pixel than the blit — empty/black samples around every stroke. Bleed is
     * done by dilating painted texels after the projection. */
    for (int i = 0; i < 3; i++) {
      vuv.append(u[i]);
      vscreen.append(s[i]);
    }
  }

  if (vuv.is_empty()) {
    return false;
  }
  return paint_proj_gpu_upload_mesh(vuv, vscreen);
}

/** Project the mesh with the view stored at paint time. Called on view-change bake, not LMB. */
static bool paint_proj_gpu_ensure_bake_mesh(const bContext *C)
{
  if (g_gpu.mesh_ready && !g_gpu.mesh_uv_cpu.is_empty()) {
    return true;
  }
  Object *ob = g_gpu.ob;
  if (ob == nullptr && C != nullptr) {
    ob = CTX_data_active_object(C);
  }
  if (ob == nullptr || ob->type != OB_MESH) {
    return false;
  }

  const Mesh *mesh_eval = nullptr;
  if (C != nullptr) {
    Depsgraph *depsgraph = CTX_data_ensure_evaluated_depsgraph(C);
    if (depsgraph) {
      Object *ob_eval = DEG_get_evaluated(depsgraph, ob);
      mesh_eval = BKE_object_get_evaluated_mesh(ob_eval);
    }
  }
  const Mesh *mesh_orig = BKE_mesh_from_object(ob);

  /* The viewport draws the evaluated mesh. Subdivision and other modifiers
   * move the silhouette off the base cage, so baking the base mesh leaves a
   * gap along every edge the user could see. Prefer that mesh when its
   * topology actually differs. Sculpt-session eval meshes sometimes drop UVs;
   * those still fall through to the original mesh below. */
  const bool eval_differs = mesh_eval && mesh_orig &&
                            (mesh_eval->verts_num != mesh_orig->verts_num ||
                             mesh_eval->faces_num != mesh_orig->faces_num);
  if (eval_differs && paint_proj_gpu_pack_mesh(mesh_eval, ob)) {
    return true;
  }
  if (BKE_object_use_sculptsession(ob->mode)) {
    if (paint_proj_gpu_pack_mesh(mesh_orig, ob)) {
      return true;
    }
  }
  if (paint_proj_gpu_pack_mesh(mesh_eval, ob)) {
    return true;
  }
  if (mesh_orig != mesh_eval && paint_proj_gpu_pack_mesh(mesh_orig, ob)) {
    return true;
  }
  return false;
}

static bool paint_proj_gpu_ensure_screen_canvas()
{
  if (g_gpu.screen_canvas_ready && g_gpu.screen_canvas_tx &&
      GPU_texture_width(g_gpu.screen_canvas_tx) == g_gpu.overlay_w &&
      GPU_texture_height(g_gpu.screen_canvas_tx) == g_gpu.overlay_h)
  {
    return true;
  }
  if (g_gpu.overlay_w < 1 || g_gpu.overlay_h < 1) {
    return false;
  }
  if (g_gpu.ima == nullptr || g_gpu.ibuf == nullptr) {
    return false;
  }
  if (!paint_proj_gpu_ensure_canvas(g_gpu.ima, &g_gpu.iuser, g_gpu.ibuf)) {
    return false;
  }
  if (!paint_proj_gpu_upload_orig(g_gpu.ima, &g_gpu.iuser, g_gpu.ibuf)) {
    return false;
  }
  if (!paint_proj_gpu_ensure_bake_mesh(nullptr) || !g_gpu.mesh_ready ||
      g_gpu.mesh_batch == nullptr || g_gpu.canvas_shader == nullptr)
  {
    return false;
  }

  paint_proj_gpu_free_screen_canvas();
  g_gpu.screen_canvas_tx = create_overlay_tex("paint_proj_ovl_canvas", g_gpu.overlay_w, g_gpu.overlay_h);
  if (!g_gpu.screen_canvas_tx) {
    return false;
  }
  GPU_framebuffer_ensure_config(&g_gpu.screen_canvas_fb,
                                {
                                    GPU_ATTACHMENT_NONE,
                                    GPU_ATTACHMENT_TEXTURE(g_gpu.screen_canvas_tx),
                                });

  gpu::FrameBuffer *prev = GPU_framebuffer_active_get();
  const float viewport_inv[2] = {1.0f / float(g_gpu.overlay_w), 1.0f / float(g_gpu.overlay_h)};

  /* Closest-face depth so smear/blur do not pick backfaces (pixel-grid junk). */
  if (g_gpu.do_occlude) {
    if (!paint_proj_gpu_ensure_depth(g_gpu.overlay_w, g_gpu.overlay_h)) {
      if (prev) {
        GPU_framebuffer_bind(prev);
      }
      return false;
    }
    GPU_framebuffer_bind(g_gpu.depth_fb);
    GPU_framebuffer_viewport_set(g_gpu.depth_fb, 0, 0, g_gpu.overlay_w, g_gpu.overlay_h);
    const double4 far_z(1.0e10, 1.0e10, 1.0e10, 1.0e10);
    GPU_framebuffer_clear_color(g_gpu.depth_fb, far_z);
    GPU_depth_test(GPU_DEPTH_NONE);
    GPU_depth_mask(false);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_blend(GPU_BLEND_MIN);
    GPU_batch_set_shader(g_gpu.mesh_batch, g_gpu.depth_shader);
    GPU_shader_uniform_2fv(g_gpu.depth_shader, "viewport_inv", viewport_inv);
    GPU_batch_draw(g_gpu.mesh_batch);
    GPU_blend(GPU_BLEND_NONE);
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_FRAMEBUFFER);
  }

  GPU_framebuffer_bind(g_gpu.screen_canvas_fb);
  GPU_framebuffer_viewport_set(g_gpu.screen_canvas_fb, 0, 0, g_gpu.overlay_w, g_gpu.overlay_h);
  const double4 clear_col(0.0, 0.0, 0.0, 0.0);
  GPU_framebuffer_clear_color(g_gpu.screen_canvas_fb, clear_col);
  GPU_blend(GPU_BLEND_NONE);
  GPU_depth_test(GPU_DEPTH_NONE);
  GPU_face_culling(GPU_CULL_NONE);
  GPU_batch_set_shader(g_gpu.mesh_batch, g_gpu.canvas_shader);
  GPU_shader_uniform_2fv(g_gpu.canvas_shader, "viewport_inv", viewport_inv);
  GPU_shader_uniform_1i(g_gpu.canvas_shader, "do_occlude", g_gpu.do_occlude ? 1 : 0);
  GPU_texture_bind(g_gpu.orig_tx, GPU_shader_get_sampler_binding(g_gpu.canvas_shader, "orig_tx"));
  gpu::Texture *depth_bind = g_gpu.depth_tx ? g_gpu.depth_tx : g_gpu.orig_tx;
  GPU_texture_bind(depth_bind, GPU_shader_get_sampler_binding(g_gpu.canvas_shader, "depth_tx"));
  GPU_batch_draw(g_gpu.mesh_batch);
  GPU_texture_unbind(g_gpu.orig_tx);
  GPU_texture_unbind(depth_bind);
  GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_FRAMEBUFFER);
  if (prev) {
    GPU_framebuffer_bind(prev);
  }
  else {
    GPU_framebuffer_restore();
  }
  g_gpu.screen_canvas_ready = true;
  return true;
}

/** \} */

static rcti uv_dirty_rect_from_overlay(int w, int h)
{
  rcti rect;
  BLI_rcti_init(&rect, 0, w, 0, h);
  return rect;
}

static void write_ibuf_from_paint(ImBuf *ibuf, const rcti &rect)
{
  if (ibuf == nullptr || BLI_rcti_is_empty(&rect) || !g_gpu.apply_fb) {
    return;
  }
  const int x = rect.xmin;
  const int y = rect.ymin;
  const int w = BLI_rcti_size_x(&rect);
  const int h = BLI_rcti_size_y(&rect);
  if (w < 1 || h < 1 || ibuf->x < 1 || ibuf->y < 1) {
    return;
  }
  bind_apply_fb();
  GPU_framebuffer_viewport_set(g_gpu.apply_fb, 0, 0, ibuf->x, ibuf->y);
  IMB_ensure_host_buffer(ibuf);
  if (ibuf->float_data()) {
    Vector<float> tmp(size_t(w) * size_t(h) * 4);
    GPU_framebuffer_read_color(g_gpu.apply_fb, x, y, w, h, 4, 0, GPU_DATA_FLOAT, tmp.data());
    float *dst = ibuf->float_data_for_write();
    if (dst == nullptr) {
      return;
    }
    for (int row = 0; row < h; row++) {
      memcpy(dst + ((y + row) * ibuf->x + x) * 4,
             tmp.data() + size_t(row) * w * 4,
             size_t(w) * 4 * sizeof(float));
    }
  }
  else if (ibuf->byte_data()) {
    Vector<uint8_t> tmp(size_t(w) * size_t(h) * 4);
    GPU_framebuffer_read_color(g_gpu.apply_fb, x, y, w, h, 4, 0, GPU_DATA_UBYTE, tmp.data());
    uint8_t *dst = ibuf->byte_data_for_write();
    if (dst == nullptr) {
      return;
    }
    for (int row = 0; row < h; row++) {
      memcpy(dst + ((y + row) * ibuf->x + x) * 4, tmp.data() + size_t(row) * w * 4, size_t(w) * 4);
    }
  }
}

bool paint_proj_gpu_overlay_pending()
{
  return g_gpu.overlay_has_paint;
}

bool paint_proj_gpu_is_screen_space_stroke()
{
  /* Only this stroke's overlay path. A pending overlay from Draw must not skip
   * undo for smear/soften/clone, which write the canvas immediately. */
  return g_gpu.screen_space_stroke;
}

void paint_proj_gpu_set_screen_space_stroke(bool value)
{
  g_gpu.screen_space_stroke = value;
}

void paint_proj_gpu_overlay_discard()
{
  g_gpu.force_project = false;
  g_gpu.ibuf = nullptr;
  if (!g_gpu.overlay_tx) {
    g_gpu.overlay_has_paint = false;
    return;
  }
  overlay_clear_tx(g_gpu.overlay_fb);
  g_gpu.overlay_has_paint = false;
  g_gpu.has_overlay_backup = false;
  g_gpu.screen_canvas_ready = false;
  BLI_rcti_init_minmax(&g_gpu.overlay_dirty);
  if (g_gpu.region && GPU_is_init()) {
    ED_region_tag_redraw(g_gpu.region);
  }
}

bool paint_proj_gpu_overlay_available()
{
  return paint_proj_gpu_ensure_shaders();
}

static OverlayUndoSnap overlay_snap_capture()
{
  OverlayUndoSnap snap;
  snap.had_paint = g_gpu.overlay_has_paint && g_gpu.overlay_tx != nullptr;
  if (!snap.had_paint) {
    return snap;
  }
  snap.tex = create_overlay_tex("paint_proj_ovl_undo", g_gpu.overlay_w, g_gpu.overlay_h);
  if (snap.tex == nullptr) {
    snap.had_paint = false;
    return snap;
  }
  GPU_texture_copy(snap.tex, g_gpu.overlay_tx);
  return snap;
}

static void overlay_snap_restore(const OverlayUndoSnap &snap)
{
  if (!snap.had_paint || snap.tex == nullptr) {
    if (g_gpu.overlay_fb) {
      overlay_clear_tx(g_gpu.overlay_fb);
    }
    g_gpu.overlay_has_paint = false;
    BLI_rcti_init_minmax(&g_gpu.overlay_dirty);
    return;
  }
  const int w = GPU_texture_width(snap.tex);
  const int h = GPU_texture_height(snap.tex);
  if (!paint_proj_gpu_ensure_overlay(w, h) || g_gpu.overlay_tx == nullptr) {
    return;
  }
  GPU_texture_copy(g_gpu.overlay_tx, snap.tex);
  g_gpu.overlay_has_paint = true;
  BLI_rcti_init(&g_gpu.overlay_dirty, 0, w, 0, h);
}

static void overlay_undo_push()
{
  overlay_stack_clear(g_overlay_redo);
  g_overlay_redo_armed = false;
  g_overlay_undo.append(overlay_snap_capture());
  while (int(g_overlay_undo.size()) > overlay_undo_limit) {
    overlay_snap_free(g_overlay_undo[0]);
    g_overlay_undo.remove(0);
  }
}

bool paint_proj_gpu_overlay_undo_stroke()
{
  if (g_overlay_undo.is_empty()) {
    overlay_stack_clear(g_overlay_redo);
    g_overlay_redo_armed = false;
    return false;
  }
  if (!GPU_is_init() || GPU_context_active_get() == nullptr) {
    return false;
  }
  g_overlay_redo.append(overlay_snap_capture());
  OverlayUndoSnap snap = g_overlay_undo.pop_last();
  overlay_snap_restore(snap);
  overlay_snap_free(snap);
  g_overlay_redo_armed = true;
  g_overlay_undo_pushed = false;
  if (g_gpu.region) {
    ED_region_tag_redraw(g_gpu.region);
  }
  return true;
}

bool paint_proj_gpu_overlay_redo_stroke()
{
  if (!g_overlay_redo_armed || g_overlay_redo.is_empty()) {
    return false;
  }
  if (!GPU_is_init() || GPU_context_active_get() == nullptr) {
    return false;
  }
  g_overlay_undo.append(overlay_snap_capture());
  OverlayUndoSnap snap = g_overlay_redo.pop_last();
  overlay_snap_restore(snap);
  overlay_snap_free(snap);
  if (g_overlay_redo.is_empty()) {
    g_overlay_redo_armed = false;
  }
  if (g_gpu.region) {
    ED_region_tag_redraw(g_gpu.region);
  }
  return true;
}

void paint_proj_gpu_overlay_stroke_begin()
{
  g_gpu.stroke_active = true;
  g_gpu.has_overlay_backup = false;
  g_overlay_undo_pushed = false;
  /* Snapshot before the dab. The first stroke has no overlay yet: that empty
   * state is what Ctrl+Z restores. Mouse-up does not project. */
  if (g_gpu.screen_space_stroke || g_gpu.overlay_has_paint) {
    overlay_undo_push();
    g_overlay_undo_pushed = true;
  }
  if (!g_gpu.overlay_tx) {
    return;
  }
  if (!g_gpu.overlay_backup_tx || GPU_texture_width(g_gpu.overlay_backup_tx) != g_gpu.overlay_w ||
      GPU_texture_height(g_gpu.overlay_backup_tx) != g_gpu.overlay_h)
  {
    GPU_TEXTURE_FREE_SAFE(g_gpu.overlay_backup_tx);
    g_gpu.overlay_backup_tx = create_overlay_tex(
        "paint_proj_ovl_bak", g_gpu.overlay_w, g_gpu.overlay_h);
  }
  if (g_gpu.overlay_backup_tx) {
    GPU_texture_copy(g_gpu.overlay_backup_tx, g_gpu.overlay_tx);
    g_gpu.has_overlay_backup = true;
  }
}

void paint_proj_gpu_overlay_stroke_cancel()
{
  if (g_overlay_undo_pushed && !g_overlay_undo.is_empty()) {
    OverlayUndoSnap snap = g_overlay_undo.pop_last();
    overlay_snap_free(snap);
    g_overlay_undo_pushed = false;
  }
  if (g_gpu.has_overlay_backup && g_gpu.overlay_backup_tx && g_gpu.overlay_tx) {
    GPU_texture_copy(g_gpu.overlay_tx, g_gpu.overlay_backup_tx);
    if (g_gpu.region) {
      ED_region_tag_redraw(g_gpu.region);
    }
    return;
  }
  paint_proj_gpu_overlay_discard();
}

static bool paint_proj_gpu_overlay_project_ex(const bContext *C, bool push_undo)
{
  g_gpu.force_project = false;
  if (!g_gpu.overlay_has_paint || g_gpu.overlay_tx == nullptr || g_gpu.ima == nullptr) {
    g_gpu.overlay_has_paint = false;
    g_gpu.ibuf = nullptr;
    return false;
  }
  if (!GPU_is_init() || GPU_context_active_get() == nullptr) {
    /* RNA / save callbacks can run with no GL context. Keep the overlay and retry on draw. */
    g_gpu.force_project = true;
    if (g_gpu.region) {
      ED_region_tag_redraw(g_gpu.region);
    }
    return false;
  }
  if (!paint_proj_gpu_ensure_shaders()) {
    return false;
  }
  if (!paint_proj_gpu_ensure_bake_mesh(C) || g_gpu.mesh_uv_cpu.is_empty()) {
    /* Keep the overlay; PRE_VIEW / mouse-up will retry. Dropping it loses the stroke. */
    g_gpu.force_project = true;
    if (g_gpu.region) {
      ED_region_tag_redraw(g_gpu.region);
    }
    return false;
  }

  if (g_gpu.ima) {
    BKE_image_paint_layers_ensure_default(*g_gpu.ima, &g_gpu.iuser);
  }
  OverlayPaintIBuf paint_ibuf(g_gpu.ima, &g_gpu.iuser);
  ImBuf *ibuf = paint_ibuf.ibuf;
  if (ibuf == nullptr || ibuf->x < 1 || ibuf->y < 1) {
    g_gpu.force_project = true;
    if (g_gpu.region) {
      ED_region_tag_redraw(g_gpu.region);
    }
    return false;
  }
  IMB_ensure_host_buffer(ibuf);
  if (ibuf->float_data() == nullptr && ibuf->byte_data() == nullptr) {
    g_gpu.force_project = true;
    if (g_gpu.region) {
      ED_region_tag_redraw(g_gpu.region);
    }
    return false;
  }

  /* Prefer CPU projection: Screen Space overlay is viewport-sized, but the
   * old GPU bake allocated a full-canvas HOST_READ FBO (8K * 8K) and never
   * wrote pixels when that failed. Experimental 3D Texture Paint also left
   * the bake mesh empty. */
  {
    rcti dirty_cpu;
    if (overlay_uv_dirty_from_mesh(ibuf->x, ibuf->y, dirty_cpu)) {
      bool started_undo = false;
      if (push_undo) {
        UndoStack *ustack = ED_undo_stack_get();
        if (ustack && ustack->step_init == nullptr) {
          ED_image_undo_push_begin("Project Paint Overlay", PaintMode::Texture3D);
          started_undo = true;
        }
      }
      ED_imapaint_dirty_region(g_gpu.ima,
                               ibuf,
                               &g_gpu.iuser,
                               dirty_cpu.xmin,
                               dirty_cpu.ymin,
                               BLI_rcti_size_x(&dirty_cpu),
                               BLI_rcti_size_y(&dirty_cpu));
      if (overlay_cpu_write_pixels(ibuf)) {
        overlay_flush_ibuf_to_image(ibuf, dirty_cpu);
        if (started_undo) {
          UndoStack *ustack = ED_undo_stack_get();
          if (ustack && ustack->step_init) {
            ED_image_undo_push_end();
          }
        }
        /* Pixels are on the image now. Per-stroke overlay undo would bring them back. */
        overlay_undo_free_all();
        paint_proj_gpu_overlay_discard();
        paint_proj_gpu_free_canvas();
        paint_proj_gpu_free_depth();
        paint_proj_gpu_free_mesh();
        paint_proj_gpu_free_screen_canvas();
        if (C) {
          WM_event_add_notifier(C, NC_IMAGE | NA_EDITED, g_gpu.ima);
        }
        else {
          WM_main_add_notifier(NC_IMAGE | NA_EDITED, g_gpu.ima);
        }
        return true;
      }
      if (started_undo) {
        UndoStack *ustack = ED_undo_stack_get();
        if (ustack && ustack->step_init) {
          ED_image_undo_push_end();
        }
      }
    }
  }

  gpu::FrameBuffer *prev_fb = GPU_framebuffer_active_get();
  const GPUBlend prev_blend = GPU_blend_get();
  const GPUDepthTest prev_depth = GPU_depth_test_get();
  const GPUFaceCullTest prev_cull = GPU_face_culling_get();
  const bool prev_depth_mask = GPU_depth_mask_get();
  const GPUWriteMask prev_write_mask = GPU_write_mask_get();

  if (g_gpu.mesh_batch == nullptr || g_gpu.bake_shader == nullptr) {
    paint_proj_gpu_restore_draw_state(
        prev_fb, prev_blend, prev_depth, prev_cull, prev_depth_mask, prev_write_mask);
    g_gpu.force_project = true;
    if (g_gpu.region) {
      ED_region_tag_redraw(g_gpu.region);
    }
    return false;
  }

  if (!paint_proj_gpu_ensure_canvas(g_gpu.ima, &g_gpu.iuser, ibuf)) {
    paint_proj_gpu_restore_draw_state(
        prev_fb, prev_blend, prev_depth, prev_cull, prev_depth_mask, prev_write_mask);
    g_gpu.force_project = true;
    if (g_gpu.region) {
      ED_region_tag_redraw(g_gpu.region);
    }
    return false;
  }
  if (!paint_proj_gpu_upload_orig(g_gpu.ima, &g_gpu.iuser, ibuf)) {
    paint_proj_gpu_restore_draw_state(
        prev_fb, prev_blend, prev_depth, prev_cull, prev_depth_mask, prev_write_mask);
    return false;
  }

  const float viewport_inv[2] = {1.0f / float(g_gpu.overlay_w), 1.0f / float(g_gpu.overlay_h)};

  if (g_gpu.do_occlude) {
    if (!paint_proj_gpu_ensure_depth(g_gpu.overlay_w, g_gpu.overlay_h)) {
      paint_proj_gpu_restore_draw_state(
          prev_fb, prev_blend, prev_depth, prev_cull, prev_depth_mask, prev_write_mask);
      return false;
    }
    GPU_framebuffer_bind(g_gpu.depth_fb);
    GPU_framebuffer_viewport_set(g_gpu.depth_fb, 0, 0, g_gpu.overlay_w, g_gpu.overlay_h);
    const double4 far_z(1.0e10, 1.0e10, 1.0e10, 1.0e10);
    GPU_framebuffer_clear_color(g_gpu.depth_fb, far_z);
    GPU_depth_test(GPU_DEPTH_NONE);
    GPU_depth_mask(false);
    GPU_face_culling(GPU_CULL_NONE);
    GPU_blend(GPU_BLEND_MIN);
    GPU_batch_set_shader(g_gpu.mesh_batch, g_gpu.depth_shader);
    GPU_shader_uniform_2fv(g_gpu.depth_shader, "viewport_inv", viewport_inv);
    GPU_batch_draw(g_gpu.mesh_batch);
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_FRAMEBUFFER);
  }

  rcti dirty = uv_dirty_rect_from_overlay(ibuf->x, ibuf->y);
  if (BLI_rcti_is_empty(&dirty)) {
    paint_proj_gpu_restore_draw_state(
        prev_fb, prev_blend, prev_depth, prev_cull, prev_depth_mask, prev_write_mask);
    paint_proj_gpu_overlay_discard();
    return false;
  }

  bool started_undo = false;
  if (push_undo) {
    UndoStack *ustack = ED_undo_stack_get();
    if (ustack && ustack->step_init == nullptr) {
      ED_image_undo_push_begin("Project Paint Overlay", PaintMode::Texture3D);
      started_undo = true;
    }
  }

  ED_imapaint_dirty_region(g_gpu.ima,
                           ibuf,
                           &g_gpu.iuser,
                           dirty.xmin,
                           dirty.ymin,
                           BLI_rcti_size_x(&dirty),
                           BLI_rcti_size_y(&dirty));

  bind_apply_fb();
  GPU_framebuffer_viewport_set(g_gpu.apply_fb, 0, 0, ibuf->x, ibuf->y);
  GPU_scissor_test(true);
  GPU_scissor(dirty.xmin, dirty.ymin, BLI_rcti_size_x(&dirty), BLI_rcti_size_y(&dirty));
  GPU_depth_test(GPU_DEPTH_NONE);
  GPU_depth_mask(false);
  GPU_face_culling(GPU_CULL_NONE);
  GPU_blend(GPU_BLEND_NONE);
  GPU_batch_set_shader(g_gpu.mesh_batch, g_gpu.bake_shader);
  GPU_shader_uniform_1i(g_gpu.bake_shader, "do_occlude", g_gpu.do_occlude ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.bake_shader, "blend", g_gpu.blend);
  GPU_shader_uniform_1i(g_gpu.bake_shader, "brush_type", int(g_gpu.brush_type));
  GPU_shader_uniform_1i(g_gpu.bake_shader, "canvas_is_srgb", g_gpu.canvas_is_srgb ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.bake_shader, "canvas_is_data", g_gpu.canvas_is_data ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.bake_shader, "is_float", g_gpu.is_float ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.bake_shader, "lock_alpha", g_gpu.lock_alpha ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.bake_shader, "src_is_linear", g_gpu.src_is_linear ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.bake_shader, "hard_edge", g_gpu.hard_edge ? 1 : 0);
  GPU_texture_bind(g_gpu.orig_tx, GPU_shader_get_sampler_binding(g_gpu.bake_shader, "orig_tx"));
  GPU_texture_bind(g_gpu.overlay_tx, GPU_shader_get_sampler_binding(g_gpu.bake_shader, "overlay_tx"));
  if (g_gpu.depth_tx) {
    GPU_texture_bind(g_gpu.depth_tx, GPU_shader_get_sampler_binding(g_gpu.bake_shader, "depth_tx"));
  }
  GPU_batch_draw(g_gpu.mesh_batch);
  GPU_texture_unbind(g_gpu.orig_tx);
  GPU_texture_unbind(g_gpu.overlay_tx);
  if (g_gpu.depth_tx) {
    GPU_texture_unbind(g_gpu.depth_tx);
  }
  GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_FRAMEBUFFER);

  if (g_gpu.dilate_shader) {
    gpu::Texture *tmp = GPU_texture_create_2d("paint_proj_ovl_dilate",
                                              g_gpu.canvas_w,
                                              g_gpu.canvas_h,
                                              1,
                                              g_gpu.paint_format,
                                              GPU_TEXTURE_USAGE_ATTACHMENT |
                                                  GPU_TEXTURE_USAGE_SHADER_READ,
                                              nullptr);
    if (tmp) {
      gpu::FrameBuffer *tmp_fb = nullptr;
      GPU_framebuffer_ensure_config(&tmp_fb,
                                    {
                                        GPU_ATTACHMENT_NONE,
                                        GPU_ATTACHMENT_TEXTURE(tmp),
                                    });
      auto dilate_once = [&](gpu::Texture *src, gpu::FrameBuffer *dst_fb) {
        if (g_gpu.paint_format == gpu::TextureFormat::SRGBA_8_8_8_8) {
          GPU_framebuffer_bind_no_srgb(dst_fb);
        }
        else {
          GPU_framebuffer_bind(dst_fb);
        }
        GPU_framebuffer_viewport_set(dst_fb, 0, 0, g_gpu.canvas_w, g_gpu.canvas_h);
        GPU_scissor_test(false);
        GPU_blend(GPU_BLEND_NONE);
        GPU_batch_set_shader(g_gpu.fs_batch, g_gpu.dilate_shader);
        GPU_texture_bind(src, GPU_shader_get_sampler_binding(g_gpu.dilate_shader, "paint_tx"));
        GPU_texture_bind(g_gpu.orig_tx,
                         GPU_shader_get_sampler_binding(g_gpu.dilate_shader, "orig_tx"));
        GPU_batch_draw(g_gpu.fs_batch);
        GPU_texture_unbind(src);
        GPU_texture_unbind(g_gpu.orig_tx);
        GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_FRAMEBUFFER);
      };
      /* At most official seam-bleed. Extra iterations grew island-edge AA
       * into 毛边 on 8K maps. Even count so the result stays on paint_tx. */
      int dilate_iters = math::clamp(int(math::ceil(g_gpu.seam_bleed_px)), 0, 2);
      if (dilate_iters > 0 && (dilate_iters & 1)) {
        dilate_iters++;
      }
      for (int i = 0; i < dilate_iters; i += 2) {
        dilate_once(g_gpu.paint_tx, tmp_fb);
        dilate_once(tmp, g_gpu.apply_fb);
      }
      GPU_FRAMEBUFFER_FREE_SAFE(tmp_fb);
      GPU_TEXTURE_FREE_SAFE(tmp);
    }
  }

  write_ibuf_from_paint(ibuf, dirty);
  overlay_flush_ibuf_to_image(ibuf, dirty);

  if (started_undo) {
    UndoStack *ustack = ED_undo_stack_get();
    if (ustack && ustack->step_init) {
      ED_image_undo_push_end();
    }
  }

  paint_proj_gpu_restore_draw_state(
      prev_fb, prev_blend, prev_depth, prev_cull, prev_depth_mask, prev_write_mask);

  overlay_undo_free_all();
  paint_proj_gpu_overlay_discard();
  paint_proj_gpu_free_canvas();
  paint_proj_gpu_free_depth();
  paint_proj_gpu_free_mesh();
  paint_proj_gpu_free_screen_canvas();

  if (C) {
    WM_event_add_notifier(C, NC_IMAGE | NA_EDITED, g_gpu.ima);
  }
  else {
    WM_main_add_notifier(NC_IMAGE | NA_EDITED, g_gpu.ima);
  }
  return true;
}

void paint_proj_gpu_overlay_project(const bContext *C)
{
  if (!GPU_is_init() || GPU_context_active_get() == nullptr) {
    if (g_gpu.overlay_has_paint) {
      g_gpu.force_project = true;
      if (g_gpu.region) {
        ED_region_tag_redraw(g_gpu.region);
      }
    }
    return;
  }
  paint_proj_gpu_overlay_project_ex(C, true);
}

bool paint_proj_gpu_screen_dab(const bContext *C, const ProjPaintGPUDab &dab)
{
  if (!dab.ibuf || !dab.ima || dab.region == nullptr) {
    return false;
  }
  if (dab.winx < 1 || dab.winy < 1) {
    return false;
  }
  if (dab.brush_radius <= 0.0f) {
    return false;
  }
  if (!paint_proj_gpu_ensure_shaders()) {
    return false;
  }

  const bool tool_changed = g_gpu.overlay_has_paint &&
                            (dab.blend != g_gpu.blend || dab.brush_type != g_gpu.brush_type);
  if (g_gpu.overlay_has_paint &&
      (dab.region != g_gpu.region || overlay_view_changed(dab.region) || tool_changed))
  {
    paint_proj_gpu_overlay_project_ex(C, true);
  }

  gpu::FrameBuffer *prev_fb = GPU_framebuffer_active_get();
  const GPUBlend prev_blend = GPU_blend_get();
  const GPUDepthTest prev_depth = GPU_depth_test_get();
  const GPUFaceCullTest prev_cull = GPU_face_culling_get();
  const bool prev_depth_mask = GPU_depth_mask_get();
  const GPUWriteMask prev_write_mask = GPU_write_mask_get();

  if (!paint_proj_gpu_ensure_overlay(dab.winx, dab.winy)) {
    paint_proj_gpu_restore_draw_state(
        prev_fb, prev_blend, prev_depth, prev_cull, prev_depth_mask, prev_write_mask);
    return false;
  }

  if (!g_gpu.overlay_has_paint) {
    overlay_store_view(dab.region, dab);
    paint_proj_gpu_ensure_draw_callbacks(dab.region);
    paint_proj_gpu_ensure_app_callbacks();
  }

  const bool needs_canvas = ELEM(dab.brush_type,
                                 IMAGE_PAINT_BRUSH_TYPE_SOFTEN,
                                 IMAGE_PAINT_BRUSH_TYPE_SMEAR,
                                 IMAGE_PAINT_BRUSH_TYPE_CLONE);
  if (needs_canvas) {
    if (!paint_proj_gpu_ensure_screen_canvas()) {
      paint_proj_gpu_restore_draw_state(
          prev_fb, prev_blend, prev_depth, prev_cull, prev_depth_mask, prev_write_mask);
      return false;
    }
  }

  const float radius = dab.brush_radius;
  const float2 mouse_prev = dab.mouse_prev;
  rcti dab_rect;
  BLI_rcti_init(&dab_rect,
                int(floorf(min_ff(mouse_prev.x, dab.mouse.x) - radius)) - 2,
                int(ceilf(max_ff(mouse_prev.x, dab.mouse.x) + radius)) + 2,
                int(floorf(min_ff(mouse_prev.y, dab.mouse.y) - radius)) - 2,
                int(ceilf(max_ff(mouse_prev.y, dab.mouse.y) + radius)) + 2);
  rcti bounds;
  BLI_rcti_init(&bounds, 0, dab.winx, 0, dab.winy);
  BLI_rcti_isect(&dab_rect, &bounds, &dab_rect);
  if (BLI_rcti_is_empty(&dab_rect)) {
    paint_proj_gpu_restore_draw_state(
        prev_fb, prev_blend, prev_depth, prev_cull, prev_depth_mask, prev_write_mask);
    return true;
  }

  GPU_texture_copy(g_gpu.overlay_src_tx, g_gpu.overlay_tx);

  GPU_framebuffer_bind(g_gpu.overlay_fb);
  GPU_framebuffer_viewport_set(g_gpu.overlay_fb, 0, 0, dab.winx, dab.winy);
  GPU_scissor_test(true);
  GPU_scissor(dab_rect.xmin, dab_rect.ymin, BLI_rcti_size_x(&dab_rect), BLI_rcti_size_y(&dab_rect));
  GPU_depth_test(GPU_DEPTH_NONE);
  GPU_depth_mask(false);
  GPU_face_culling(GPU_CULL_NONE);
  GPU_blend(GPU_BLEND_NONE);

  if (g_gpu.curve_tx) {
    uint8_t curve_u8[64];
    for (int i = 0; i < 64; i++) {
      const float s = math::clamp(dab.curve[i], 0.0f, 1.0f);
      curve_u8[i] = uint8_t(s * 255.0f + 0.5f);
    }
    GPU_texture_update(g_gpu.curve_tx, GPU_DATA_UBYTE, curve_u8);
  }

  GPU_batch_set_shader(g_gpu.fs_batch, g_gpu.stamp_shader);
  const float mouse[2] = {dab.mouse.x, dab.mouse.y};
  const float mouse_prev_v[2] = {mouse_prev.x, mouse_prev.y};
  const float paint_color[3] = {dab.paint_color.x, dab.paint_color.y, dab.paint_color.z};
  const float clone_offset[2] = {dab.clone_offset.x, dab.clone_offset.y};
  GPU_shader_uniform_2fv(g_gpu.stamp_shader, "mouse", mouse);
  GPU_shader_uniform_2fv(g_gpu.stamp_shader, "mouse_prev", mouse_prev_v);
  GPU_shader_uniform_1f(g_gpu.stamp_shader, "brush_radius", radius);
  GPU_shader_uniform_1f(g_gpu.stamp_shader, "brush_alpha", dab.brush_alpha);
  GPU_shader_uniform_3fv(g_gpu.stamp_shader, "paint_color", paint_color);
  GPU_shader_uniform_1i(g_gpu.stamp_shader, "brush_type", int(dab.brush_type));
  GPU_shader_uniform_1i(g_gpu.stamp_shader, "blend", int(dab.blend));
  GPU_shader_uniform_2fv(g_gpu.stamp_shader, "clone_offset", clone_offset);
  GPU_shader_uniform_1i(g_gpu.stamp_shader, "sharpen", dab.sharpen ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.stamp_shader, "hard_edge", dab.hard_edge ? 1 : 0);
  GPU_shader_uniform_1i(g_gpu.stamp_shader, "disk_stamp", dab.disk_stamp ? 1 : 0);

  GPU_texture_bind(g_gpu.curve_tx, GPU_shader_get_sampler_binding(g_gpu.stamp_shader, "curve_tx"));
  GPU_texture_bind(g_gpu.overlay_src_tx,
                   GPU_shader_get_sampler_binding(g_gpu.stamp_shader, "overlay_src"));
  gpu::Texture *canvas_bind = g_gpu.screen_canvas_tx ? g_gpu.screen_canvas_tx : g_gpu.overlay_src_tx;
  GPU_texture_bind(canvas_bind, GPU_shader_get_sampler_binding(g_gpu.stamp_shader, "screen_canvas"));
  GPU_batch_draw(g_gpu.fs_batch);
  GPU_texture_unbind(g_gpu.curve_tx);
  GPU_texture_unbind(g_gpu.overlay_src_tx);
  GPU_texture_unbind(canvas_bind);
  GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH | GPU_BARRIER_FRAMEBUFFER);

  BLI_rcti_do_minmax_rcti(&g_gpu.overlay_dirty, &dab_rect);
  g_gpu.overlay_has_paint = true;

  paint_proj_gpu_restore_draw_state(
      prev_fb, prev_blend, prev_depth, prev_cull, prev_depth_mask, prev_write_mask);

  ED_region_tag_redraw(dab.region);
  return true;
}

}  // namespace blender
