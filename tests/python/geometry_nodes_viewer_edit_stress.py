# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""GUI regression test for Geometry Nodes Viewer geometry in mesh edit mode.

Run with a real window, for example:
  blender --factory-startup --python geometry_nodes_viewer_edit_wireframe.py -- <output-dir>

The fixture puts the modifier result at the origin and a translated Viewer copy
to its right. It captures the wireframe viewport before edit mode, immediately
after entering edit mode, and after a topology edit. The delayed final capture
also exercises redraws after the edit, where stale draw data used to crash.
"""

import os
from pathlib import Path
import sys
import traceback

import bpy


OUTPUT_DIR = Path(sys.argv[sys.argv.index("--") + 1]) if "--" in sys.argv else Path.cwd()
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

STATE = {}


def log(message):
    print(f"VIEWER_EDIT_WIREFRAME {message}", flush=True)


def fail_and_quit():
    traceback.print_exc()
    log("FAILED")
    bpy.ops.wm.quit_blender()


def viewport_override():
    area = STATE["area"]
    region = next(region for region in area.regions if region.type == "WINDOW")
    return bpy.context.temp_override(window=STATE["window"], area=area, region=region)


def capture(name):
    filepath = OUTPUT_DIR / f"{name}.png"
    with viewport_override():
        scene = bpy.context.scene
        scene.render.image_settings.file_format = "PNG"
        scene.render.filepath = str(filepath)
        bpy.ops.render.opengl(write_still=True, view_context=True)
    assert filepath.exists() and filepath.stat().st_size > 0
    log(f"CAPTURE {name} {filepath.stat().st_size}")


def setup():
    try:
        bpy.ops.object.select_all(action="SELECT")
        bpy.ops.object.delete(use_global=False)

        mesh = bpy.data.meshes.new("Viewer Edit Source")
        mesh.from_pydata(
            [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
             (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)], [],
            [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)])
        mesh.update()
        obj = bpy.data.objects.new("Viewer Edit Source", mesh)
        bpy.context.collection.objects.link(obj)
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj

        tree = bpy.data.node_groups.new("Viewer Edit Tree", "GeometryNodeTree")
        tree.interface.new_socket(
            name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry"
        )
        tree.interface.new_socket(
            name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry"
        )
        group_input = tree.nodes.new("NodeGroupInput")
        group_output = tree.nodes.new("NodeGroupOutput")
        transform = tree.nodes.new("GeometryNodeTransform")
        transform.inputs["Translation"].default_value = (1.5, 0.0, 0.0)
        transform.inputs["Scale"].default_value = (0.5, 0.5, 0.5)
        viewer = tree.nodes.new("GeometryNodeViewer")
        viewer.viewer_items.new("GEOMETRY", "Geometry")
        viewer.label = viewer.name
        hull = tree.nodes.new("GeometryNodeConvexHull")
        tree.links.new(group_input.outputs["Geometry"], hull.inputs["Geometry"])
        tree.links.new(hull.outputs["Convex Hull"], group_output.inputs["Geometry"])
        tree.links.new(group_input.outputs["Geometry"], transform.inputs["Geometry"])
        tree.links.new(transform.outputs["Geometry"], viewer.inputs["Geometry"])
        tree.nodes.active = viewer

        modifier = obj.modifiers.new("Viewer Edit Regression", "NODES")
        modifier.node_group = tree
        modifier.show_in_editmode = True

        window = bpy.context.window_manager.windows[0]
        area = max(
            (item for item in window.screen.areas if item.type == "VIEW_3D"),
            key=lambda item: item.width * item.height,
        )
        node_area = max(
            (item for item in window.screen.areas if item != area),
            key=lambda item: item.width * item.height,
        )
        node_area.type = "NODE_EDITOR"
        node_area.ui_type = "GeometryNodeTree"
        node_area.spaces.active.pin = False
        node_area.tag_redraw()

        spreadsheet_area = next(a for a in window.screen.areas if a not in (area, node_area))
        spreadsheet_area.type = "SPREADSHEET"
        STATE["spreadsheet"] = spreadsheet_area.spaces.active

        STATE.update(
            window=window, area=area, node_area=node_area, obj=obj, viewer=viewer
        )
        bpy.app.timers.register(activate_viewer, first_interval=0.5)
    except Exception:
        fail_and_quit()


def activate_viewer():
    try:
        node_area = STATE["node_area"]
        with bpy.context.temp_override(window=STATE["window"], area=node_area):
            result = bpy.ops.node.activate_viewer()
        assert result == {"FINISHED"}
        path = STATE["spreadsheet"].viewer_path.path
        assert path[-1].ui_name == STATE["viewer"].name
        log("VIEWER_ACTIVATE FINISHED")

        area = STATE["area"]
        view = area.spaces.active
        view.show_viewer = True
        view.shading.type = "WIREFRAME"
        view.shading.show_xray_wireframe = False
        view.overlay.show_overlays = True
        area.tag_redraw()
        bpy.context.view_layer.update()
        with viewport_override():
            bpy.ops.view3d.view_axis(type="TOP", align_active=False)
            bpy.ops.view3d.view_all(center=True)
        bpy.app.timers.register(capture_object_mode, first_interval=0.75)
    except Exception:
        fail_and_quit()


def capture_object_mode():
    try:
        capture("01_object_wireframe")
        with viewport_override():
            result = bpy.ops.object.mode_set(mode="EDIT")
        assert result == {"FINISHED"}
        STATE["area"].tag_redraw()
        bpy.context.view_layer.update()
        log("ENTER_EDIT_MODE")
        bpy.app.timers.register(capture_edit_mode, first_interval=0.75)
    except Exception:
        fail_and_quit()


def capture_edit_mode():
    try:
        capture("02_edit_wireframe_before")
        with viewport_override():
            bpy.ops.mesh.select_all(action="SELECT")
            result = bpy.ops.mesh.subdivide(number_cuts=1)
        assert result == {"FINISHED"}
        STATE["area"].tag_redraw()
        bpy.context.view_layer.update()
        log("SUBDIVIDE_FINISHED")
        bpy.app.timers.register(capture_after_edit, first_interval=1.0)
    except Exception:
        fail_and_quit()


def capture_after_edit():
    try:
        capture("03_edit_wireframe_after")
        # Keep redrawing after the topology update to expose deferred cache use.
        STATE["redraws"] = 0
        bpy.app.timers.register(redraw_after_edit, first_interval=0.1)
    except Exception:
        fail_and_quit()


def redraw_after_edit():
    try:
        with viewport_override():
            bpy.ops.transform.translate(value=(0.005, 0.003, 0.001))
        bpy.context.view_layer.update()
        STATE["area"].tag_redraw()
        STATE["redraws"] += 1
        if STATE["redraws"] < 20:
            return 0.1
        capture("04_edit_wireframe_delayed")
        STATE["switches"] = 0
        bpy.app.timers.register(switch_viewer, first_interval=0.2)
        return None
    except Exception:
        traceback.print_exc()
        log("FAILED")
    bpy.ops.wm.quit_blender()
    return None


def switch_viewer():
    try:
        tree = STATE["obj"].modifiers[0].node_group
        node_area = STATE["node_area"]
        if "viewer2" not in STATE:
            transform2 = tree.nodes.new("GeometryNodeTransform")
            transform2.inputs["Translation"].default_value = (-2.0,0,0)
            viewer2 = tree.nodes.new("GeometryNodeViewer")
            viewer2.viewer_items.new("GEOMETRY", "Geometry")
            viewer2.label = viewer2.name
            source = next(n for n in tree.nodes if n.bl_idname == "NodeGroupInput")
            tree.links.new(source.outputs["Geometry"],transform2.inputs["Geometry"])
            tree.links.new(transform2.outputs["Geometry"],viewer2.inputs["Geometry"])
            STATE["viewer2"] = viewer2
        viewer = STATE["viewer"] if STATE["switches"] % 2 else STATE["viewer2"]
        tree.nodes.active = viewer
        with bpy.context.temp_override(window=STATE["window"],area=node_area):
            assert bpy.ops.node.activate_viewer() == {"FINISHED"}
            assert STATE["spreadsheet"].viewer_path.path[-1].ui_name == viewer.name
            # Verify reactivation after explicitly disabling, while staying in edit mode.
            assert bpy.ops.node.deactivate_viewer() == {"FINISHED"}
            assert bpy.ops.node.activate_viewer() == {"FINISHED"}
            assert STATE["spreadsheet"].viewer_path.path[-1].ui_name == viewer.name
        bpy.context.view_layer.update()
        STATE["switches"] += 1
        if STATE["switches"] < 8:
            return .2
        bpy.app.timers.register(finish_viewer, first_interval=0.6)
        return None
    except Exception:
        traceback.print_exc()
        log("FAILED")
    bpy.ops.wm.quit_blender()
    return None


def finish_viewer():
    try:
        capture("05_edit_after_viewer_switches")
        path = STATE["spreadsheet"].viewer_path.path
        assert path[-1].ui_name == STATE["viewer"].name
        log("COMPLETE translations=20 switches=8 final=" + path[-1].ui_name)
    except Exception:
        traceback.print_exc()
        log("FAILED")
    bpy.ops.wm.quit_blender()
    return None


os.dup2(sys.stdout.fileno(), sys.stderr.fileno())
bpy.app.timers.register(setup, first_interval=0.2)
