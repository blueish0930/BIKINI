/* SPDX-FileCopyrightText: 2020-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/overlay_edit_mode_infos.hh"

VERTEX_SHADER_CREATE_INFO(overlay_edit_uv_face_dots)

#include "draw_model_lib.glsl"
#include "draw_view_lib.glsl"

void main()
{
  float3 world_pos = float3(au, 0.0f);
  gl_Position = drw_point_world_to_homogenous(world_pos);

  /* .x = v_flag (select). .y = e_flag (mirror). A lone uint would drop .y. */
  uint flags = flag.x | (flag.y << 8u);
  bool is_mirror = (flags & UV_FACE_MIRROR) != 0u && theme.colors.mirror_select.a > 0.0f;
  if (is_mirror) {
    final_color = float4(theme.colors.mirror_select.rgb, 1.0f);
  }
  else {
    final_color = ((flags & FACE_UV_SELECT) != 0u) ? theme.colors.facedot :
                                                      float4(theme.colors.wire.rgb, 1.0f);
  }
  gl_PointSize = dot_size;
}
