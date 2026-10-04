/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgeometry_edit
 *
 * Helpers to resolve the 3D View under the cursor. Geometry Edit is started from
 * the Node Editor, so CTX_wm_region is usually SPACE_NODE — never use that for pick/draw.
 */

#pragma once

#include "BKE_context.hh"
#include "BKE_screen.hh"

#include "BLI_listbase.hh"

#include "DNA_screen_types.h"
#include "DNA_space_types.h"
#include "DNA_windowmanager_types.h"

#include "ED_screen.hh"

#include "WM_api.hh"
#include "WM_types.hh"

namespace blender::ed::geometry_edit {

/** First View3D window region on the active screen (may be null). */
inline ARegion *find_any_view3d_window_region(const bContext *C)
{
  const bScreen *screen = CTX_wm_screen(C);
  if (!screen) {
    return nullptr;
  }
  for (const ScrArea &area : screen->areabase) {
    if (area.spacetype != SPACE_VIEW3D) {
      continue;
    }
    if (ARegion *region = BKE_area_find_region_type(&area, RGN_TYPE_WINDOW)) {
      return region;
    }
  }
  return nullptr;
}

/**
 * View3D window region under window-space \a xy, or any View3D as fallback.
 * Also writes region-relative mouse coordinates into \a r_mval when non-null.
 */
inline ARegion *find_view3d_region_xy(const bContext *C, const int xy[2], int r_mval[2])
{
  const bScreen *screen = CTX_wm_screen(C);
  if (!screen) {
    return nullptr;
  }

  ScrArea *area = BKE_screen_find_area_xy(screen, SPACE_VIEW3D, xy);
  ARegion *region = nullptr;
  if (area) {
    region = BKE_area_find_region_xy(area, RGN_TYPE_WINDOW, xy);
    if (!region) {
      region = BKE_area_find_region_type(area, RGN_TYPE_WINDOW);
    }
  }
  if (!region) {
    region = find_any_view3d_window_region(C);
  }
  if (!region) {
    return nullptr;
  }
  if (r_mval) {
    r_mval[0] = xy[0] - region->winrct.xmin;
    r_mval[1] = xy[1] - region->winrct.ymin;
  }
  return region;
}

inline ARegion *find_view3d_region_event(const bContext *C, const wmEvent *event, int r_mval[2])
{
  return find_view3d_region_xy(C, event->xy, r_mval);
}

/** Redraw every View3D window region (session started from Node Editor). */
inline void tag_all_view3d_redraw(const bContext *C)
{
  wmWindowManager *wm = CTX_wm_manager(C);
  if (!wm) {
    return;
  }
  for (wmWindow &win : wm->windows) {
    const bScreen *screen = WM_window_get_active_screen(&win);
    if (!screen) {
      continue;
    }
    for (const ScrArea &area : screen->areabase) {
      if (area.spacetype != SPACE_VIEW3D) {
        continue;
      }
      for (ARegion &region : area.regionbase) {
        if (region.regiontype == RGN_TYPE_WINDOW) {
          ED_region_tag_redraw(&region);
        }
      }
    }
  }
}

}  // namespace blender::ed::geometry_edit
