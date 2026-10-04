/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_geo_sdf_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(compositor_geo_sdf_raster)

void main()
{
  out_occupancy = float4(1.0f);
}
