# SPDX-FileCopyrightText: 2026 BIKINI Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
from bpy.app.translations import (
    contexts as i18n_contexts,
)
from bl_ui import node_add_menu


class LUXCORE_OT_new_material_node_tree(bpy.types.Operator):
    bl_idname = "luxcore.new_material_node_tree"
    bl_label = "New LuxCore Node Tree"
    bl_description = "Create a LuxCore material node tree on the active material"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        ob = context.object
        return ob is not None and ob.active_material is not None

    def execute(self, context):
        ma = context.object.active_material
        nt = bpy.data.node_groups.new(ma.name, "LuxCoreMaterialNodeTree")
        out = nt.nodes.new("ShaderNodeLuxOutput")
        out.location = (280.0, 0.0)
        disney = nt.nodes.new("ShaderNodeLuxDisney")
        disney.location = (0.0, 0.0)
        if disney.outputs and out.inputs:
            nt.links.new(disney.outputs[0], out.inputs[0])
        ma.luxcore_node_tree = nt
        return {'FINISHED'}


class NODE_MT_luxcore_node_output_base(node_add_menu.NodeMenu):
    bl_label = "Output"

    def draw(self, _context):
        layout = self.layout
        self.node_operator(layout, "ShaderNodeLuxOutput")
        self.draw_assets_for_catalog(layout, self.bl_label)


class NODE_MT_luxcore_node_material_base(node_add_menu.NodeMenu):
    bl_label = "Material"

    def draw(self, _context):
        layout = self.layout
        self.node_operator(layout, "ShaderNodeLuxDisney")
        self.node_operator(layout, "ShaderNodeLuxMatte")
        self.node_operator(layout, "ShaderNodeLuxGlossy")
        self.node_operator(layout, "ShaderNodeLuxGlossyCoating")
        self.node_operator(layout, "ShaderNodeLuxMetal")
        self.node_operator(layout, "ShaderNodeLuxMirror")
        self.node_operator(layout, "ShaderNodeLuxVelvet")
        self.node_operator(layout, "ShaderNodeLuxCarpaint")
        self.node_operator(layout, "ShaderNodeLuxCloth")
        layout.separator()
        self.node_operator(layout, "ShaderNodeLuxGlass")
        self.node_operator(layout, "ShaderNodeLuxRoughGlass")
        self.node_operator(layout, "ShaderNodeLuxArchGlass")
        self.node_operator(layout, "ShaderNodeLuxGlossyTranslucent")
        self.node_operator(layout, "ShaderNodeLuxMatteTranslucent")
        self.node_operator(layout, "ShaderNodeLuxNull")
        layout.separator()
        self.node_operator(layout, "ShaderNodeLuxMix")
        self.node_operator(layout, "ShaderNodeLuxTwoSided")
        self.node_operator(layout, "ShaderNodeLuxFrontBackOpacity")
        self.node_operator(layout, "ShaderNodeLuxEmission")
        self.draw_assets_for_catalog(layout, self.bl_label)


class NODE_MT_luxcore_node_volume_base(node_add_menu.NodeMenu):
    bl_label = "Volume"

    def draw(self, _context):
        layout = self.layout
        self.node_operator(layout, "ShaderNodeLuxVolumeClear")
        self.node_operator(layout, "ShaderNodeLuxVolumeHomogeneous")
        self.node_operator(layout, "ShaderNodeLuxVolumeHeterogeneous")
        self.draw_assets_for_catalog(layout, self.bl_label)


class NODE_MT_luxcore_node_layout_base(node_add_menu.NodeMenu):
    bl_label = "Layout"

    def draw(self, _context):
        layout = self.layout
        self.node_operator(layout, "NodeFrame")
        self.node_operator(layout, "NodeReroute")
        self.draw_assets_for_catalog(layout, self.bl_label)


class NODE_MT_luxcore_node_all_base(node_add_menu.NodeMenu):
    bl_label = ""
    menu_path = "Root"
    bl_translation_context = i18n_contexts.operator_default

    def draw(self, context):
        del context
        layout = self.layout
        self.draw_menu(layout, "Output")
        self.draw_menu(layout, "Material")
        self.draw_menu(layout, "Volume")
        layout.separator()
        self.draw_menu(layout, "Layout")
        self.draw_root_assets(layout)


add_menus = {
    "NODE_MT_category_luxcore_output": NODE_MT_luxcore_node_output_base,
    "NODE_MT_category_luxcore_material": NODE_MT_luxcore_node_material_base,
    "NODE_MT_category_luxcore_volume": NODE_MT_luxcore_node_volume_base,
    "NODE_MT_category_luxcore_layout": NODE_MT_luxcore_node_layout_base,
    "NODE_MT_luxcore_node_add_all": NODE_MT_luxcore_node_all_base,
}
add_menus = node_add_menu.generate_menus(
    add_menus,
    template=node_add_menu.AddNodeMenu,
    base_dict=node_add_menu.add_base_pathing_dict,
)

swap_menus = {
    "NODE_MT_luxcore_node_output_swap": NODE_MT_luxcore_node_output_base,
    "NODE_MT_luxcore_node_material_swap": NODE_MT_luxcore_node_material_base,
    "NODE_MT_luxcore_node_volume_swap": NODE_MT_luxcore_node_volume_base,
    "NODE_MT_luxcore_node_layout_swap": NODE_MT_luxcore_node_layout_base,
    "NODE_MT_luxcore_node_swap_all": NODE_MT_luxcore_node_all_base,
}
swap_menus = node_add_menu.generate_menus(
    swap_menus,
    template=node_add_menu.SwapNodeMenu,
    base_dict=node_add_menu.swap_base_pathing_dict,
)

classes = (
    LUXCORE_OT_new_material_node_tree,
    *add_menus,
    *swap_menus,
)

if __name__ == "__main__":
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
