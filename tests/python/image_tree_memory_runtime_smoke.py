# SPDX-FileCopyrightText: 2026 Blender Authors
# SPDX-License-Identifier: GPL-2.0-or-later
"""Runtime smoke: Image SpaceNode tree memory (restore + non-sticky reselect).

Run:
  blender --background --factory-startup --python tests/python/image_tree_memory_runtime_smoke.py
"""
import bpy
import sys
import traceback


def find_node_editor():
    for window in bpy.context.window_manager.windows:
        screen = window.screen
        if screen is None:
            continue
        for area in screen.areas:
            if area.type == "NODE_EDITOR":
                space = area.spaces.active
                region = next((r for r in area.regions if r.type == "WINDOW"), None)
                return window, screen, area, region, space
    window = bpy.context.window
    screen = window.screen
    area = screen.areas[0]
    area.type = "NODE_EDITOR"
    space = area.spaces.active
    region = next(r for r in area.regions if r.type == "WINDOW")
    return window, screen, area, region, space


def force_context_sync(window, screen, area, region, space):
    with bpy.context.temp_override(
        window=window, screen=screen, area=area, region=region, space_data=space
    ):
        g = space.selected_node_group
        space.selected_node_group = g
        _ = space.node_tree
        _ = space.edit_tree
    bpy.context.view_layer.update()


def main() -> int:
    bpy.ops.wm.read_factory_settings(use_empty=False)

    tree_a = bpy.data.node_groups.new("ImageMemA", "ImageNodeTree")
    tree_b = bpy.data.node_groups.new("ImageMemB", "ImageNodeTree")
    tree_a.nodes.new("ImageNodeBlankImage")
    tree_b.nodes.new("ImageNodeViewer")

    window, screen, area, region, space = find_node_editor()

    space.tree_type = "ImageNodeTree"
    space.selected_node_group = tree_a
    force_context_sync(window, screen, area, region, space)
    if space.selected_node_group != tree_a:
        print("FAIL: selected_node_group is not A after pick")
        return 1

    space.tree_type = "CompositorNodeTree"
    force_context_sync(window, screen, area, region, space)
    space.tree_type = "ImageNodeTree"
    force_context_sync(window, screen, area, region, space)
    if space.selected_node_group != tree_a:
        print("FAIL: A not restored after switch away/back", space.selected_node_group)
        return 1
    print("RESTORE_OK")

    space.selected_node_group = tree_b
    for i in range(3):
        force_context_sync(window, screen, area, region, space)
        if space.selected_node_group != tree_b:
            print("FAIL: sticky override on sync", i, space.selected_node_group)
            return 1
        if space.node_tree is not None and space.node_tree != tree_b:
            print("FAIL: node_tree sticky override", i, space.node_tree)
            return 1
    print("RESELECT_OK")

    space.node_tree = tree_a
    force_context_sync(window, screen, area, region, space)
    if space.selected_node_group != tree_a:
        print("FAIL: node_tree assign did not sync selected_node_group")
        return 1
    print("NODE_TREE_SYNC_OK")

    print("TREE_MEMORY_RUNTIME_OK")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception:
        traceback.print_exc()
        sys.exit(1)
