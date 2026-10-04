/* SPDX-FileCopyrightText: 2026 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/compositor_rasterize_geometry_infos.hh"

VERTEX_SHADER_CREATE_INFO(compositor_rasterize_geometry)

void main()
{
  v_world_pos = world_pos;
  v_normal = normal;
  v_attr0 = attr0;
  v_attr1 = attr1;
  v_attr2 = attr2;
  v_attr3 = attr3;
  /* pos_ndc.xy = NDC (-1..1), pos_ndc.z = linear depth 0..1 (smaller = closer). */
  const float z_clip = pos_ndc.z * 2.0f - 1.0f;
  gl_Position = float4(pos_ndc.x, pos_ndc.y, z_clip, 1.0f);
}
