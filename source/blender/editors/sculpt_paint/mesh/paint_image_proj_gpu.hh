/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edsculpt
 *
 * 3D Texture Paint: stamp into a viewport-sized overlay, then project onto the
 * UV canvas when the view moves. Drawing never rasterizes the mesh or the 4K/8K image.
 */

#pragma once

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

namespace blender {

struct ARegion;
struct Image;
struct ImageUser;
struct ImBuf;
struct bContext;

struct Object;

struct ProjPaintGPUDab {
  Image *ima = nullptr;
  ImageUser *iuser = nullptr;
  ImBuf *ibuf = nullptr;
  ARegion *region = nullptr;
  Object *ob = nullptr;

  const float (*viewmat)[4] = nullptr;
  const float (*winmat)[4] = nullptr;
  const float (*project_mat)[4] = nullptr;

  float2 mouse = float2(0.0f);
  /** Previous sample; coverage is a screen-space capsule from mouse_prev to mouse. */
  float2 mouse_prev = float2(0.0f);
  float brush_radius = 0.0f;
  float brush_alpha = 1.0f;
  /**
   * Brush color for Draw. Byte canvases stamp display-referred (sRGB) color —
   * the same buffer official mix_byte uses. Float canvases stamp scene-linear.
   */
  float3 paint_color = float3(0.0f);
  /** True when #paint_color is scene-linear (float canvas). */
  bool src_is_linear = false;

  int winx = 0;
  int winy = 0;

  bool do_occlude = true;
  bool do_backfacecull = true;
  bool is_ortho = false;
  float clip_start = 0.1f;
  float seam_bleed_px = 2.0f;

  /** #IMB_BlendMode */
  int blend = 0;
  /** #eBrushImagePaintType */
  short brush_type = 0;
  float2 clone_offset = float2(0.0f);
  bool canvas_is_srgb = true;
  bool canvas_is_data = false;
  bool lock_alpha = false;
  bool sharpen = false;
  /**
   * Hard / Constant falloff: stamp and bake as a binary mask.
   * Screen-space AA is 1 overlay pixel; projecting that into a high-res UV island
   * becomes a wide mix-with-black band that mipmaps turn into a dark halo.
   */
  bool hard_edge = false;
  /**
   * Stamp a disk at #mouse instead of a capsule from #mouse_prev.
   * Airbrush / Dots / Smear: spaced or time-based dabs. Paint Soft/Hard keep the capsule.
   */
  bool disk_stamp = false;

  /** Brush falloff: index 0 at center, 63 at radius. */
  float curve[64] = {};
};

void paint_proj_gpu_stroke_end();

bool paint_proj_gpu_has_mesh();
bool paint_proj_gpu_upload_mesh(Span<float2> uv, Span<float4> screen);

/** True while unprojected screen-space paint is waiting to be baked onto the canvas. */
bool paint_proj_gpu_overlay_pending();

/** True when this stroke is the screen-space overlay path (even before the first dab). */
bool paint_proj_gpu_is_screen_space_stroke();
void paint_proj_gpu_set_screen_space_stroke(bool value);

bool paint_proj_gpu_overlay_available();

void paint_proj_gpu_overlay_stroke_begin();
void paint_proj_gpu_overlay_stroke_cancel();

/**
 * Project the screen overlay onto the UV canvas and clear it.
 * \param C: Optional; used for notifiers. Undo is pushed when no image-undo step is open.
 */
void paint_proj_gpu_overlay_project(const bContext *C);

/** Drop the overlay without writing the canvas (undo / cancel). */
void paint_proj_gpu_overlay_discard();

/** Undo one screen-space stroke that has not been projected yet. */
bool paint_proj_gpu_overlay_undo_stroke();
/** Redo a stroke undone by #paint_proj_gpu_overlay_undo_stroke. */
bool paint_proj_gpu_overlay_redo_stroke();

/** Free all persistent GPU/CPU overlay resources. Safe to call during shutdown. */
void paint_proj_gpu_exit();

/**
 * Stamp a screen-space dab onto the viewport overlay. Does not touch the canvas.
 * Returns false to fall back to the CPU projection painter.
 */
bool paint_proj_gpu_screen_dab(const bContext *C, const ProjPaintGPUDab &dab);

}  // namespace blender
