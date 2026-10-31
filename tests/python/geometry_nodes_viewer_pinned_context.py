# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""GUI regression for a pinned Geometry Nodes datablock without an object context.

Run with a window:
  blender --factory-startup --python geometry_nodes_viewer_pinned_context.py -- <output-dir>

Assigning SpaceNodeEditor.node_tree starts a datablock-only editor path. Pinning
that path and saving used to leave nested Viewers permanently unactivatable,
even though node.activate_viewer returned FINISHED. Verify context recovery on
redraw, preservation of the nested path, and actual Spreadsheet Viewer paths
when switching Viewers and reopening the saved file.
"""

import json
from pathlib import Path
import sys
import traceback

import bpy


OUTPUT_DIR = Path(sys.argv[sys.argv.index("--") + 1])
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
STATE = {"checks": [], "phase": 0}


def tree(name):
    group = bpy.data.node_groups.new(name, "GeometryNodeTree")
    group.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    group.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    group_input = group.nodes.new("NodeGroupInput")
    group_output = group.nodes.new("NodeGroupOutput")
    group.links.new(group_input.outputs["Geometry"], group_output.inputs["Geometry"])
    return group


def editor():
    window = bpy.context.window_manager.windows[0]
    area = next(area for area in window.screen.areas if area.type == "NODE_EDITOR")
    region = next(region for region in area.regions if region.type == "WINDOW")
    return window, area, region


def check(condition, name):
    assert condition, name
    STATE["checks"].append(name)


def setup():
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.object
    obj.name = "Pinned Viewer Object"
    root = tree("Pinned Viewer Root")
    child = tree("Pinned Viewer Child")
    call = root.nodes.new("GeometryNodeGroup")
    call.node_tree = child
    call.name = "Child Call"
    group_input = next(n for n in root.nodes if n.bl_idname == "NodeGroupInput")
    root.links.new(group_input.outputs["Geometry"], call.inputs["Geometry"])
    for name in ("Viewer A", "Viewer B"):
        viewer = child.nodes.new("GeometryNodeViewer")
        viewer.name = name
        viewer.label = name
        viewer.viewer_items.new("GEOMETRY", "Geometry")
        child_input = next(n for n in child.nodes if n.bl_idname == "NodeGroupInput")
        child.links.new(child_input.outputs["Geometry"], viewer.inputs["Geometry"])
    modifier = obj.modifiers.new("Pinned Viewer Modifier", "NODES")
    modifier.node_group = root
    window = bpy.context.window_manager.windows[0]
    area = next(area for area in window.screen.areas if area.type == "PROPERTIES")
    area.type = "NODE_EDITOR"
    area.ui_type = "GeometryNodeTree"
    space = area.spaces.active
    space.pin = True
    space.node_tree = root
    check(space.id is None, "datablock-only pin reproduced")
    root.nodes.active = call
    region = next(region for region in area.regions if region.type == "WINDOW")
    with bpy.context.temp_override(window=window, area=area, region=region):
        bpy.ops.node.group_edit(exit=False)
    check(len(space.path) == 2, "nested path entered before context recovery")
    spreadsheet = next(area for area in window.screen.areas if area.type == "OUTLINER")
    spreadsheet.type = "SPREADSHEET"
    STATE["phase"] = 1


def activate(name):
    window, area, region = editor()
    space = area.spaces.active
    child = bpy.data.node_groups["Pinned Viewer Child"]
    viewer = child.nodes[name]
    child.nodes.active = viewer
    with bpy.context.temp_override(window=window, area=area, region=region):
        check(bpy.ops.node.activate_viewer() == {"FINISHED"}, name + " activation")


def verify(name):
    window, area, _ = editor()
    space = area.spaces.active
    check(space.pin and space.id == bpy.data.objects["Pinned Viewer Object"], "pinned object recovered")
    check([p.node_tree.name for p in space.path] == ["Pinned Viewer Root", "Pinned Viewer Child"],
          "nested editor path preserved")
    spreadsheet = next(area for area in window.screen.areas if area.type == "SPREADSHEET")
    path = spreadsheet.spaces.active.viewer_path.path
    check([p.type for p in path] == ["ID", "MODIFIER", "GROUP_NODE", "VIEWER_NODE"],
          "complete nested Viewer path")
    check(path[-1].ui_name == name, "Spreadsheet follows " + name)


def tick():
    try:
        phase = STATE["phase"]
        if phase == 0:
            setup()
        elif phase == 1:
            activate("Viewer A")
            STATE["phase"] = 2
        elif phase == 2:
            verify("Viewer A")
            activate("Viewer B")
            STATE["phase"] = 3
        elif phase == 3:
            verify("Viewer B")
            bpy.ops.wm.save_as_mainfile(filepath=str(OUTPUT_DIR / "pinned-viewer.blend"))
            STATE["phase"] = 4
            bpy.ops.wm.open_mainfile(filepath=str(OUTPUT_DIR / "pinned-viewer.blend"))
        elif phase == 4:
            verify("Viewer B")
            activate("Viewer A")
            STATE["phase"] = 5
        else:
            verify("Viewer A")
            (OUTPUT_DIR / "result.json").write_text(json.dumps(STATE, indent=2), encoding="utf8")
            print("PINNED_VIEWER_CONTEXT_PASS", len(STATE["checks"]), flush=True)
            bpy.ops.wm.quit_blender()
            return None
        return 1.0
    except Exception:
        traceback.print_exc()
        (OUTPUT_DIR / "failed.txt").write_text(traceback.format_exc(), encoding="utf8")
        bpy.ops.wm.quit_blender()
        return None


bpy.app.timers.register(tick, first_interval=1.0, persistent=True)
