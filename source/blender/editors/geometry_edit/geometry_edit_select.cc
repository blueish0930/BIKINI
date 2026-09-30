/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgeometry_edit
 *
 * Selection is handled by native Mesh Edit Mode operators.
 * These entry points are unused stubs for API compatibility.
 */

#include "BKE_context.hh"

#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

#include "ED_geometry_edit.hh"

#include "geometry_edit_intern.hh"

namespace blender::ed::geometry_edit {

bool select_pick_xy(bContext * /*C*/,
                    const int /*xy*/[2],
                    bool /*extend*/,
                    bool /*deselect*/,
                    bool /*toggle*/)
{
  return false;
}

bool select_all(bContext * /*C*/, int /*action*/)
{
  return false;
}

bool select_box_xy(bContext * /*C*/,
                   const int /*xy_min*/[2],
                   const int /*xy_max*/[2],
                   bool /*extend*/,
                   bool /*deselect*/)
{
  return false;
}

bool select_circle_xy(
    bContext * /*C*/, const int /*xy*/[2], float /*radius_px*/, bool /*extend*/, bool /*deselect*/)
{
  return false;
}

bool select_lasso_xy(bContext * /*C*/,
                     Span<int2> /*mcoords_window*/,
                     bool /*extend*/,
                     bool /*deselect*/)
{
  return false;
}

bool select_more(bContext * /*C*/)
{
  return false;
}

bool select_less(bContext * /*C*/)
{
  return false;
}

}  // namespace blender::ed::geometry_edit
