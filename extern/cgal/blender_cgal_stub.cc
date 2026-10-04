/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Empty TU so extern_cgal is a real linkable target. CGAL itself is header-only. */

namespace blender::extern_cgal {
int cgal_vendor_stub()
{
  return 0;
}
}  // namespace blender::extern_cgal
