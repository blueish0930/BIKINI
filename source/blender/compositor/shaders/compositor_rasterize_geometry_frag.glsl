/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_rasterize_geometry_infos.hh"

FRAGMENT_SHADER_CREATE_INFO(compositor_rasterize_geometry)

void main()
{
  float3 n = v_normal;
  const float nlen2 = dot(n, n);
  if (nlen2 > 1e-12f) {
    n *= inversesqrt(nlen2);
  }
  else {
    n = float3(0.0f, 0.0f, 1.0f);
  }

  out_position = float4(v_world_pos, 1.0f);
  out_normal = float4(n, 0.0f);
  out_alpha = float4(1.0f);
  out_attr0 = v_attr0;
  out_attr1 = v_attr1;
  out_attr2 = v_attr2;
  out_attr3 = v_attr3;
}
