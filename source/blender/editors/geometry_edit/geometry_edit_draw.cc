/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgeometry_edit
 *
 * Native Edit Mode draws the temp mesh (Workbench + Overlay).
 * Gesture overlays unused; stubs kept for API stability.
 */

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

#include "ED_geometry_edit.hh"

#include "geometry_edit_intern.hh"

namespace blender::ed::geometry_edit {

void draw_set_gesture_box(const int /*start*/[2], const int /*curr*/[2]) {}
void draw_set_gesture_lasso(const Span<int2> /*points*/) {}
void draw_set_gesture_circle(const int /*center*/[2], const float /*radius*/) {}
void draw_clear_gesture() {}

}  // namespace blender::ed::geometry_edit
