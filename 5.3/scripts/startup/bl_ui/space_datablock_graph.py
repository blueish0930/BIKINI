# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy


class DATABLOCK_GRAPH_HT_header(bpy.types.Header):
    bl_space_type = 'DATABLOCK_GRAPH'

    def draw(self, context):
        layout = self.layout
        space = context.space_data

        layout.template_header()
        DATABLOCK_GRAPH_MT_editor_menus.draw_collapsible(context, layout)

        layout.separator_spacer()

        layout.prop(space, "use_recursive", text="Recursive", toggle=True)

        row = layout.row(align=True)
        row.enabled = not space.use_recursive
        row.prop(space, "max_depth", text="Depth")

        layout.operator("datablock_graph.relayout", text="Layout", icon='NODETREE')

        # Force re-walk Main relations / rebuild ref graph.
        layout.operator("datablock_graph.refresh", text="", icon='FILE_REFRESH')

        # Snap magnet toggle (same pattern as Geometry Nodes header).
        row = layout.row(align=True)
        row.prop(space, "use_snap", text="", icon_only=True)

        layout.operator("datablock_graph.clear", text="", icon='X')


class DATABLOCK_GRAPH_MT_editor_menus(bpy.types.Menu):
    bl_idname = "DATABLOCK_GRAPH_MT_editor_menus"
    bl_label = ""

    def draw(self, context):
        del context
        layout = self.layout
        layout.menu("DATABLOCK_GRAPH_MT_view")
        layout.menu("DATABLOCK_GRAPH_MT_select")


class DATABLOCK_GRAPH_MT_view(bpy.types.Menu):
    bl_label = "View"

    def draw(self, context):
        layout = self.layout
        space = context.space_data
        layout.prop(space, "show_region_toolbar")
        layout.prop(space, "show_region_channels", text="Browser")
        layout.separator()
        layout.operator("datablock_graph.relayout", text="Frame All / Layout", icon='NODETREE')
        layout.operator("datablock_graph.refresh", text="Refresh Graph", icon='FILE_REFRESH')
        layout.separator()
        layout.menu("INFO_MT_area")


class DATABLOCK_GRAPH_MT_select(bpy.types.Menu):
    bl_label = "Select"

    def draw(self, context):
        del context
        layout = self.layout
        layout.operator("datablock_graph.select_all", text="All").action = 'SELECT'
        layout.operator("datablock_graph.select_all", text="None").action = 'DESELECT'
        layout.operator("datablock_graph.select_all", text="Invert").action = 'INVERT'
        layout.separator()
        layout.operator("datablock_graph.select_box", text="Box Select")
        layout.operator("datablock_graph.select_circle", text="Circle Select")
        layout.operator("datablock_graph.select_lasso", text="Lasso Select")
        layout.label(text="Double-click: select connected tree")
        layout.separator()
        layout.operator("datablock_graph.delete", text="Delete", icon='X')
        layout.separator()
        layout.operator("datablock_graph.translate_node", text="Move (G)")
        layout.operator("datablock_graph.rotate_node", text="Rotate (R)")
        layout.operator("datablock_graph.scale_node", text="Scale (S)")
        layout.operator("datablock_graph.align_selection", text="Align Selection (Hold U)")


def _ensure_keymap():
    wm = bpy.context.window_manager
    for kc in (wm.keyconfigs.default, wm.keyconfigs.user, wm.keyconfigs.addon):
        if kc is None:
            continue
        km = kc.keymaps.get("Data-Block Graph Generic")
        if km is None:
            km = kc.keymaps.new(
                name="Data-Block Graph Generic",
                space_type='DATABLOCK_GRAPH',
                region_type='WINDOW',
            )

        def has(idname, type_=None, value=None, shift=None, ctrl=None, alt=None):
            for kmi in km.keymap_items:
                if kmi.idname != idname:
                    continue
                if type_ is not None and kmi.type != type_:
                    continue
                if value is not None and kmi.value != value:
                    continue
                if shift is not None and bool(kmi.shift) != bool(shift):
                    continue
                if ctrl is not None and bool(kmi.ctrl) != bool(ctrl):
                    continue
                if alt is not None and bool(kmi.alt) != bool(alt):
                    continue
                return True
            return False

        if not has("wm.context_toggle", 'T', 'PRESS'):
            kmi = km.keymap_items.new("wm.context_toggle", 'T', 'PRESS')
            kmi.properties.data_path = "space_data.show_region_toolbar"

        for key, idname in (
            ('G', "datablock_graph.translate_node"),
            ('R', "datablock_graph.rotate_node"),
            ('S', "datablock_graph.scale_node"),
            ('U', "datablock_graph.align_selection"),
            ('B', "datablock_graph.select_box"),
            ('C', "datablock_graph.select_circle"),
            ('A', "datablock_graph.select_all"),
            ('X', "datablock_graph.delete"),
            ('DEL', "datablock_graph.delete"),
        ):
            if not has(idname, key, 'PRESS'):
                km.keymap_items.new(idname, key, 'PRESS')

        if not has("datablock_graph.delete", 'X', 'PRESS', False, True):
            km.keymap_items.new("datablock_graph.delete", 'X', 'PRESS', ctrl=True)

        if not has("datablock_graph.select", 'LEFTMOUSE', 'CLICK', False):
            km.keymap_items.new("datablock_graph.select", 'LEFTMOUSE', 'CLICK')
        if not has("datablock_graph.select", 'LEFTMOUSE', 'CLICK', True):
            kmi = km.keymap_items.new(
                "datablock_graph.select", 'LEFTMOUSE', 'CLICK', shift=True)
            kmi.properties.extend = True
        if not has("datablock_graph.translate_node", 'LEFTMOUSE', 'PRESS'):
            km.keymap_items.new("datablock_graph.translate_node", 'LEFTMOUSE', 'PRESS')

        found_tweak = False
        for kmi in km.keymap_items:
            if kmi.idname == "datablock_graph.select_box" and kmi.type == 'LEFTMOUSE':
                if getattr(kmi, "value", None) in {'CLICK_DRAG', 'PRESS'}:
                    if hasattr(kmi.properties, "tweak"):
                        kmi.properties.tweak = True
                    found_tweak = True
        if not found_tweak:
            kmi = km.keymap_items.new(
                "datablock_graph.select_box", 'LEFTMOUSE', 'CLICK_DRAG')
            if hasattr(kmi.properties, "tweak"):
                kmi.properties.tweak = True

        def ensure_tool_km(name, items_fn):
            tkm = kc.keymaps.get(name)
            if tkm is None:
                tkm = kc.keymaps.new(
                    name=name,
                    space_type='DATABLOCK_GRAPH',
                    region_type='WINDOW',
                    tool=True,
                )
            items_fn(tkm)
            return tkm

        def empty_or_select(tkm):
            if not any(k.idname == "datablock_graph.select" for k in tkm.keymap_items):
                tkm.keymap_items.new("datablock_graph.select", 'LEFTMOUSE', 'CLICK')
            if not any(k.idname == "datablock_graph.translate_node" for k in tkm.keymap_items):
                tkm.keymap_items.new("datablock_graph.translate_node", 'LEFTMOUSE', 'PRESS')

        def box_tool(tkm):
            if not any(k.idname == "datablock_graph.select" for k in tkm.keymap_items):
                tkm.keymap_items.new("datablock_graph.select", 'LEFTMOUSE', 'CLICK')
            if not any(k.idname == "datablock_graph.select_box" for k in tkm.keymap_items):
                kmi = tkm.keymap_items.new(
                    "datablock_graph.select_box", 'LEFTMOUSE', 'CLICK_DRAG')
                kmi.properties.tweak = True

        def circle_tool(tkm):
            if not any(k.idname == "datablock_graph.select_circle" for k in tkm.keymap_items):
                kmi = tkm.keymap_items.new(
                    "datablock_graph.select_circle", 'LEFTMOUSE', 'PRESS')
                if hasattr(kmi.properties, "wait_for_input"):
                    kmi.properties.wait_for_input = False

        def lasso_tool(tkm):
            if not any(k.idname == "datablock_graph.select_lasso" for k in tkm.keymap_items):
                kmi = tkm.keymap_items.new(
                    "datablock_graph.select_lasso", 'LEFTMOUSE', 'CLICK_DRAG')
                kmi.properties.tweak = True

        ensure_tool_km("Data-Block Graph Tool: Tweak", empty_or_select)
        ensure_tool_km("Data-Block Graph Tool: Select Box", box_tool)
        ensure_tool_km("Data-Block Graph Tool: Select Circle", circle_tool)
        ensure_tool_km("Data-Block Graph Tool: Select Lasso", lasso_tool)


classes = (
    DATABLOCK_GRAPH_HT_header,
    DATABLOCK_GRAPH_MT_editor_menus,
    DATABLOCK_GRAPH_MT_view,
    DATABLOCK_GRAPH_MT_select,
)


if __name__ == "__main__":
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
    _ensure_keymap()
