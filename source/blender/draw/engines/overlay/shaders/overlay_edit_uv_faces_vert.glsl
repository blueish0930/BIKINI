/* SPDX-FileCopyrightText: 2020-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "infos/overlay_edit_mode_infos.hh"

VERTEX_SHADER_CREATE_INFO(overlay_edit_uv_faces)

#include "draw_model_lib.glsl"
#include "draw_object_infos_lib.glsl"
#include "draw_view_lib.glsl"
#include "gpu_shader_utildefines.bsl.hh"

void main()
{
  float3 world_pos = float3(au, 0.0f);
  gl_Position = drw_point_world_to_homogenous(world_pos);

  /* .x = v_flag (select). .y = e_flag (mirror). A lone uint would drop .y. */
  uint flags = flag.x | (flag.y << 8u);
  bool is_selected = (flags & FACE_UV_SELECT) != 0u;
  bool is_active = (flags & FACE_UV_ACTIVE) != 0u;
  bool is_mirror = (flags & UV_FACE_MIRROR) != 0u && theme.colors.mirror_select.a > 0.0f;
  eObjectInfoFlag ob_flag = drw_object_infos().flag;
  bool is_object_active = flag_test(ob_flag, OBJECT_ACTIVE_EDIT_MODE);

  final_color = (is_selected) ? theme.colors.face_select : theme.colors.face;
  if (is_mirror) {
    /* Unselected face color is often fully transparent, which hid the fill. */
    float alpha = max(theme.colors.face_select.a, 0.35f);
    final_color = float4(theme.colors.mirror_select.rgb, alpha);
  }
  final_color = (is_active) ? theme.colors.edit_mesh_active : final_color;
  final_color.a *= is_object_active ? uv_opacity : (uv_opacity * 0.25f);
}
