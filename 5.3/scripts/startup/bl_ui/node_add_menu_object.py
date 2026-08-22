# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

from bpy.app.translations import (
    contexts as i18n_contexts,
)
from bl_ui import node_add_menu


class NODE_MT_object_node_input_base(node_add_menu.NodeMenu):
    bl_label = "Input"

    def draw(self, _context):
        layout = self.layout
        self.node_operator(layout, "ObjectNodeObject")
        self.node_operator(layout, "ObjectNodeCreate")
        self.node_operator(layout, "NodeGroupInput")
        self.draw_assets_for_catalog(layout, self.bl_label)


class NODE_MT_object_node_output_base(node_add_menu.NodeMenu):
    bl_label = "Output"

    def draw(self, _context):
        layout = self.layout
        self.node_operator(layout, "ObjectNodeOutput")
        self.node_operator(layout, "NodeGroupOutput")
        self.draw_assets_for_catalog(layout, self.bl_label)


class NODE_MT_object_node_object_base(node_add_menu.NodeMenu):
    bl_label = "Object"

    def draw(self, _context):
        layout = self.layout
        self.node_operator(layout, "ObjectNodeSetTransform")
        self.node_operator(layout, "ObjectNodeSetVisibility")
        self.node_operator(layout, "ObjectNodeViewportDisplay")
        layout.separator()
        self.node_operator(layout, "ObjectNodeSetMaterialSlots")
        self.node_operator(layout, "ObjectNodeSetModifiers")
        layout.separator()
        self.node_operator(layout, "ObjectNodeSetParent")
        self.node_operator(layout, "ObjectNodeDelete")
        self.draw_assets_for_catalog(layout, self.bl_label)


class NODE_MT_object_node_layout_base(node_add_menu.NodeMenu):
    bl_label = "Layout"

    def draw(self, _context):
        layout = self.layout
        self.node_operator(layout, "NodeFrame")
        self.node_operator(layout, "NodeReroute")
        self.draw_assets_for_catalog(layout, self.bl_label)


class NODE_MT_object_node_all_base(node_add_menu.NodeMenu):
    bl_label = ""
    menu_path = "Root"
    bl_translation_context = i18n_contexts.operator_default

    def draw(self, context):
        del context
        layout = self.layout
        self.draw_menu(layout, "Input")
        self.draw_menu(layout, "Output")
        layout.separator()
        self.draw_menu(layout, "Object")
        layout.separator()
        self.draw_menu(layout, "Layout")
        self.draw_root_assets(layout)


add_menus = {
    "NODE_MT_category_object_input": NODE_MT_object_node_input_base,
    "NODE_MT_category_object_output": NODE_MT_object_node_output_base,
    "NODE_MT_category_object_object": NODE_MT_object_node_object_base,
    "NODE_MT_category_object_layout": NODE_MT_object_node_layout_base,
    "NODE_MT_object_node_add_all": NODE_MT_object_node_all_base,
}
add_menus = node_add_menu.generate_menus(
    add_menus,
    template=node_add_menu.AddNodeMenu,
    base_dict=node_add_menu.add_base_pathing_dict,
)


swap_menus = {
    "NODE_MT_object_node_input_swap": NODE_MT_object_node_input_base,
    "NODE_MT_object_node_output_swap": NODE_MT_object_node_output_base,
    "NODE_MT_object_node_object_swap": NODE_MT_object_node_object_base,
    "NODE_MT_object_node_layout_swap": NODE_MT_object_node_layout_base,
    "NODE_MT_object_node_swap_all": NODE_MT_object_node_all_base,
}
swap_menus = node_add_menu.generate_menus(
    swap_menus,
    template=node_add_menu.SwapNodeMenu,
    base_dict=node_add_menu.swap_base_pathing_dict,
)


classes = (
    *add_menus,
    *swap_menus,
)


if __name__ == "__main__":  # only for live edit.
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
