/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** Internal CGAL type aliases. Include only from cgal_bridge translation units. */

#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

namespace blender::cgal_bridge {

/**
 * EPICK: exact predicates, inexact constructions.
 * Robust enough for PMP / hull / alpha shapes; does not require MPFR.
 * When WITH_MPFR is available a future EPECK alias can be swapped here.
 */
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Surface_mesh = CGAL::Surface_mesh<Point_3>;

}  // namespace blender::cgal_bridge
