# SPDX-FileCopyrightText: 2026 BIKINI
# SPDX-License-Identifier: GPL-3.0-or-later

from __future__ import annotations

import bpy
from bpy.types import Panel

from . import core


def draw_mesh_options(self, context):
    obj = context.edit_object
    if obj is None or obj.type != 'MESH':
        return
    prefs = core.addon_prefs()
    layout = self.layout
    col = layout.column(align=True)
    col.separator()
    col.label(text="Topology Operations")
    if prefs is not None:
        col.prop(prefs, "enable_topology_ops", text="Mirror Topology Ops")
        sub = col.column(align=True)
        sub.active = prefs.enable_topology_ops
        sub.prop(prefs, "show_overlay", text="Highlight Counterparts")
    col.operator("mesh.topology_mirror_expand", icon='MOD_MIRROR')


class VIEW3D_PT_topology_mirror(Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Tool"
    bl_label = "Topology Mirror"
    bl_context = "mesh_edit"
    bl_options = {'DEFAULT_CLOSED'}

    def draw(self, context):
        layout = self.layout
        obj = context.edit_object
        prefs = core.addon_prefs()
        if obj is not None and obj.type == 'MESH':
            mesh = obj.data
            col = layout.column(align=True)
            row = col.row(align=True)
            row.prop(mesh, "use_mirror_x", text="X", toggle=True)
            row.prop(mesh, "use_mirror_y", text="Y", toggle=True)
            row.prop(mesh, "use_mirror_z", text="Z", toggle=True)
            col.prop(mesh, "use_mirror_topology")
        if prefs is not None:
            layout.prop(prefs, "enable_topology_ops")
            layout.prop(prefs, "show_overlay")
            layout.prop(prefs, "overlay_color")
            layout.prop(prefs, "spatial_threshold")
        layout.operator("mesh.topology_mirror_expand", icon='MOD_MIRROR')
        layout.operator("mesh.topology_mirror_loopcut", icon='MESH_GRID')
        box = layout.box()
        box.label(text="When X/Y/Z Mirror is on:")
        box.label(text="Extrude, Inset, Bevel, Delete,")
        box.label(text="Loop Cut, Subdivide, Rip…")
        box.label(text="run on both sides.")
        box.label(text="Knife still needs Symmetrize.")


classes = (VIEW3D_PT_topology_mirror,)


def register():
    if hasattr(bpy.types, "VIEW3D_PT_tools_meshedit_options"):
        bpy.types.VIEW3D_PT_tools_meshedit_options.append(draw_mesh_options)


def unregister():
    if hasattr(bpy.types, "VIEW3D_PT_tools_meshedit_options"):
        try:
            bpy.types.VIEW3D_PT_tools_meshedit_options.remove(draw_mesh_options)
        except ValueError:
            pass
