/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 *
 * Guide Geometry overlay. Implementation lives in overlay_guide_geometry.cc so it cannot
 * be left stale inside overlay_instance.obj (header-only inlines were not rebuilding).
 */

#pragma once

#include "overlay_base.hh"

namespace blender::draw::overlay {

class GuideGeometry : Overlay {
 private:
  PassSimple occluded_ps_ = {"guide_geometry_occluded"};
  PassSimple xray_ps_ = {"guide_geometry_xray"};
  Vector<gpu::Batch *> owned_batches_;

 public:
  ~GuideGeometry();

  void begin_sync(Resources &res, const State &state) final;
  void object_sync(Manager &manager,
                   const ObjectRef &ob_ref,
                   Resources &res,
                   const State &state) final;
  void draw_line(Framebuffer &framebuffer, Manager &manager, View &view) final;
  void draw_color_only(Framebuffer &framebuffer, Manager &manager, View &view) final;

 private:
  void discard_batches();
};

}  // namespace blender::draw::overlay
