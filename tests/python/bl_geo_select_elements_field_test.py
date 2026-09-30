# SPDX-FileCopyrightText: 2026 Blender Authors
# SPDX-License-Identifier: GPL-2.0-or-later
"""
Drive the shipped Select Elements indices writeback path and verify Selection
field semantics on concrete mesh geometry.

Usage:
  blender.exe --background --factory-startup --python bl_geo_select_elements_field_test.py
"""

import sys

import bpy
from mathutils import Vector


def fail(msg: str) -> None:
    print(f"FAIL: {msg}", file=sys.stderr)
    raise SystemExit(1)


def setup_gn_select_elements():
    bpy.ops.wm.read_factory_settings(use_empty=True)

    mesh = bpy.data.meshes.new("CubeMesh")
    # Explicit unit cube: 8 verts, 6 faces.
    verts = [
        Vector((-1, -1, -1)),
        Vector((1, -1, -1)),
        Vector((1, 1, -1)),
        Vector((-1, 1, -1)),
        Vector((-1, -1, 1)),
        Vector((1, -1, 1)),
        Vector((1, 1, 1)),
        Vector((-1, 1, 1)),
    ]
    faces = [
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (2, 3, 7, 6),
        (1, 2, 6, 5),
        (0, 3, 7, 4),
    ]
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    obj = bpy.data.objects.new("Cube", mesh)
    bpy.context.scene.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    tree = bpy.data.node_groups.new("SelectElementsTest", "GeometryNodeTree")
    # Interface for modifier I/O.
    tree.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    tree.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")

    nodes = tree.nodes
    links = tree.links
    nodes.clear()

    n_in = nodes.new("NodeGroupInput")
    n_out = nodes.new("NodeGroupOutput")
    select = nodes.new("GeometryNodeSelectElements")
    delete = nodes.new("GeometryNodeDeleteGeometry")

    select.domain = "FACE"
    delete.domain = "FACE"
    delete.mode = "ALL"

    links.new(n_in.outputs[0], select.inputs["Geometry"])
    links.new(select.outputs["Geometry"], delete.inputs["Geometry"])
    links.new(select.outputs["Selection"], delete.inputs["Selection"])
    links.new(delete.outputs["Geometry"], n_out.inputs[0])

    mod = obj.modifiers.new(name="GN", type="NODES")
    mod.node_group = tree

    return obj, tree, select


def write_indices_via_shipped_operator(tree, select_node, indices_csv: str) -> None:
    """Use NODE_OT_select_elements_set_indices (same write path as Esc confirm)."""
    screen = bpy.context.window.screen
    # Find or create a node editor area.
    node_area = None
    for area in screen.areas:
        if area.type == "NODE_EDITOR":
            node_area = area
            break
    if node_area is None:
        # Reuse first area as node editor.
        node_area = screen.areas[0]
        node_area.type = "NODE_EDITOR"

    space = node_area.spaces.active
    space.tree_type = "GeometryNodeTree"
    space.node_tree = tree

    for n in tree.nodes:
        n.select = False
    select_node.select = True
    tree.nodes.active = select_node

    region = None
    for reg in node_area.regions:
        if reg.type == "WINDOW":
            region = reg
            break

    with bpy.context.temp_override(
        window=bpy.context.window,
        screen=screen,
        area=node_area,
        region=region,
        space_data=space,
    ):
        result = bpy.ops.node.select_elements_set_indices(domain="FACE", indices=indices_csv)
        if set(result) != {"FINISHED"}:
            fail(f"select_elements_set_indices returned {result}")


def main() -> None:
    if not hasattr(bpy.types, "GeometryNodeSelectElements"):
        fail("GeometryNodeSelectElements type missing")
    if not hasattr(bpy.ops.node, "select_elements_edit"):
        fail("NODE_OT_select_elements_edit not registered")
    if not hasattr(bpy.ops.node, "select_elements_set_indices"):
        fail("NODE_OT_select_elements_set_indices not registered")

    obj, tree, select = setup_gn_select_elements()

    # Write faces 0,2,4 via shipped writeback API.
    write_indices_via_shipped_operator(tree, select, "0,2,4")

    # Confirm storage was written (selected count label uses indices_num).
    # domain property is RNA; indices are storage-only — verify via eval effect.

    depsgraph = bpy.context.evaluated_depsgraph_get()
    depsgraph.update()
    obj_eval = obj.evaluated_get(depsgraph)
    mesh_eval = obj_eval.to_mesh()
    faces_after = len(mesh_eval.polygons)
    verts_after = len(mesh_eval.vertices)
    obj_eval.to_mesh_clear()

    print(f"INFO: evaluated mesh faces={faces_after} verts={verts_after}")

    # Cube 6 faces; delete faces 0,2,4 → 3 faces remain.
    if faces_after != 3:
        # Diagnostic: empty selection would leave 6 faces; all-true would leave 0.
        fail(
            f"expected 3 faces after delete of faces 0,2,4; got faces={faces_after} verts={verts_after}"
        )

    print("PASS: select_elements field writeback (faces 0,2,4 → 3 faces remain)")
    print("PASS: interactive operators registered (select_elements_edit + set_indices)")
    raise SystemExit(0)


if __name__ == "__main__":
    main()
