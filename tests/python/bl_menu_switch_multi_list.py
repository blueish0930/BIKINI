# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""
Regression: multi-selection Menu Switch must use List shape + List data.

Gates:
  multi on  → Output display_shape == LIST; List Length is selected count (0/1/2)
  multi off → Output display_shape != LIST; no crash
  multi + {A}     → length 1 (length-1 list, not bare geometry)
  multi + {A,B}   → length 2
  multi + set()   → length 0

Run:
  blender --background --factory-startup --python tests/python/bl_menu_switch_multi_list.py
"""

from __future__ import annotations

import re
import sys
import traceback
from pathlib import Path

import bpy


def _assert(cond: bool, msg: str) -> None:
    if not cond:
        raise AssertionError(msg)


def _clear_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)


def _make_mesh_object(name: str, n_verts: int, location) -> bpy.types.Object:
    mesh = bpy.data.meshes.new(name + "Mesh")
    verts = [(float(i), 0.0, 0.0) for i in range(n_verts)]
    edges = [(i, i + 1) for i in range(n_verts - 1)]
    mesh.from_pydata(verts, edges, [])
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    obj.location = location
    bpy.context.scene.collection.objects.link(obj)
    return obj


def _build_multi_eval_setup(selected_names: set[str]):
    """
    Menu (multi) → Menu Switch ← Object Info ×3
    Menu Switch Output → List Length
    Mesh Grid → StoreNamedAttribute(ms_list_len = Length) → Output
    """
    tree = bpy.data.node_groups.new("MS_MultiList_Tree", "GeometryNodeTree")

    menu_sock = tree.interface.new_socket(
        name="Menu", in_out="INPUT", socket_type="NodeSocketMenu"
    )
    _assert(hasattr(menu_sock, "menu_multi_selection"), "menu_multi_selection RNA missing")
    menu_sock.menu_multi_selection = True
    tree.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    tree.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")

    gi = tree.nodes.new("NodeGroupInput")
    go = tree.nodes.new("NodeGroupOutput")
    ms = tree.nodes.new("GeometryNodeMenuSwitch")
    ms.data_type = "GEOMETRY"
    while len(list(ms.enum_items)) < 3:
        ms.enum_items.new(chr(ord("A") + len(list(ms.enum_items))))

    for i, n_verts in enumerate((3, 4, 5)):
        o = _make_mesh_object(f"MS_Src{i}", n_verts, (float(i * 2), 0, 0))
        oi = tree.nodes.new("GeometryNodeObjectInfo")
        oi.inputs["Object"].default_value = o
        oi.transform_space = "RELATIVE"
        geo_inputs = [s for s in ms.inputs if s.type == "GEOMETRY"]
        _assert(i < len(geo_inputs), f"missing geometry item {i}")
        tree.links.new(oi.outputs["Geometry"], geo_inputs[i])

    list_len = tree.nodes.new("GeometryNodeListLength")
    list_len.data_type = "GEOMETRY"

    # Rebuild List Length Geometry/List sockets and Menu Switch List Output
    bpy.context.view_layer.update()

    tree.links.new(gi.outputs["Menu"], ms.inputs["Menu"])
    _assert(ms.outputs[0].display_shape == "LIST", f"shape {ms.outputs[0].display_shape!r}")
    tree.links.new(ms.outputs[0], list_len.inputs["List"])

    grid = tree.nodes.new("GeometryNodeMeshGrid")
    grid.inputs["Vertices X"].default_value = 2
    grid.inputs["Vertices Y"].default_value = 2

    store = tree.nodes.new("GeometryNodeStoreNamedAttribute")
    store.data_type = "INT"
    store.domain = "POINT"
    store.inputs["Name"].default_value = "ms_list_len"

    tree.links.new(grid.outputs["Mesh"], store.inputs["Geometry"])
    tree.links.new(list_len.outputs["Length"], store.inputs["Value"])
    tree.links.new(store.outputs["Geometry"], go.inputs["Geometry"])

    menu_sock.default_value_multi = set(selected_names)

    host_mesh = bpy.data.meshes.new("MS_HostMesh")
    host_mesh.from_pydata([(0, 0, 0)], [], [])
    host = bpy.data.objects.new("MS_Host", host_mesh)
    bpy.context.scene.collection.objects.link(host)

    mod = host.modifiers.new("MS_MultiList", "NODES")
    mod.node_group = tree
    menu_sock.default_value_multi = set(selected_names)
    bpy.context.view_layer.update()

    return host, ms, tree, menu_sock


def _read_list_length(host: bpy.types.Object) -> int:
    depsgraph = bpy.context.evaluated_depsgraph_get()
    host_eval = host.evaluated_get(depsgraph)
    mesh = host_eval.to_mesh()
    try:
        print("  eval verts", len(mesh.vertices), "attrs", [a.name for a in mesh.attributes])
        attr = mesh.attributes.get("ms_list_len")
        _assert(attr is not None, "ms_list_len attribute missing after eval")
        return int(attr.data[0].value)
    finally:
        host_eval.to_mesh_clear()


def test_source_static() -> None:
    candidates = [
        Path(__file__).resolve().parents[2]
        / "source/blender/nodes/geometry/nodes/node_geo_menu_switch.cc",
        Path(r"E:\Blender_Source\source\blender\nodes\geometry\nodes\node_geo_menu_switch.cc"),
    ]
    src_path = next((p for p in candidates if p.is_file()), None)
    _assert(src_path is not None, "node_geo_menu_switch.cc not found")
    text = src_path.read_text(encoding="utf-8", errors="replace")

    m2 = re.search(r"static bool menu_switch_is_multi_selection\(.*?\n\}", text, re.DOTALL)
    _assert(m2 is not None, "menu_switch_is_multi_selection not found")
    body2 = m2.group(0)
    _assert("inputs.first" in body2, "must walk DNA inputs.first")
    _assert("dynamic_cast" not in body2, "must not dynamic_cast")
    _assert(
        "menu_switch_output_is_list" not in body2,
        "must not sticky-read Output List as multi (blocks multi-off)",
    )

    _assert("StructureType::List" in text, "declare must set List when multi")
    _assert("should_output_list" in text, "missing should_output_list")
    _assert("execute_single_multi_selection" in text, "missing multi list pack")
    _assert("GList::create" in text, "must create GList")
    _assert("if (count == 1)" not in text, "must not special-case count==1")

    upd_candidates = [
        Path(__file__).resolve().parents[2]
        / "source/blender/blenkernel/intern/node_tree_update.cc",
        Path(r"E:\Blender_Source\source\blender\blenkernel\intern\node_tree_update.cc"),
    ]
    upd = next((p for p in upd_candidates if p.is_file()), None)
    _assert(upd is not None, "node_tree_update.cc not found")
    upd_text = upd.read_text(encoding="utf-8", errors="replace")
    _assert(
        "GeometryNodeMenuSwitch" in upd_text and "requires_dependent_tree_updates" in upd_text,
        "interface multi must re-declare Menu Switch",
    )
    print("PASS source_static_checks")


def test_list_shape_and_lengths() -> dict:
    results = {}
    cases = [
        ("multi1", {"A"}, 1),
        ("multi2", {"A", "B"}, 2),
        ("multi0", set(), 0),
    ]
    for label, names, expect in cases:
        _clear_scene()
        host, ms, _tree, menu_sock = _build_multi_eval_setup(selected_names=names)
        out = ms.outputs[0]
        print(f"  {label} Output display_shape={out.display_shape!r}")
        _assert(
            out.display_shape == "LIST",
            f"{label}: multi Output must be LIST shape, got {out.display_shape!r}",
        )
        results[f"{label}_shape"] = out.display_shape
        try:
            menu_sock.default_value_multi = set(names)
            print(f"  {label} dvm={menu_sock.default_value_multi}")
        except Exception as e:
            print(f"  {label} dvm warn: {e}")
        length = _read_list_length(host)
        results[label] = length
        print(f"{label}: list_length={length} expect={expect}")
        _assert(length == expect, f"{label}: expected list length {expect}, got {length}")
        print(f"PASS {label}")

    # Same tree: multi True → LIST, then multi False → not LIST (sticky-List bug regression)
    _clear_scene()
    tree = bpy.data.node_groups.new("MS_Toggle_Tree", "GeometryNodeTree")
    menu_sock = tree.interface.new_socket(
        name="Menu", in_out="INPUT", socket_type="NodeSocketMenu"
    )
    menu_sock.menu_multi_selection = True
    tree.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    tree.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    gi = tree.nodes.new("NodeGroupInput")
    go = tree.nodes.new("NodeGroupOutput")
    ms = tree.nodes.new("GeometryNodeMenuSwitch")
    ms.data_type = "GEOMETRY"
    while len(list(ms.enum_items)) < 2:
        ms.enum_items.new(chr(ord("A") + len(list(ms.enum_items))))
    tree.links.new(gi.outputs["Menu"], ms.inputs["Menu"])
    bpy.context.view_layer.update()
    out = ms.outputs[0]
    print(f"  toggle_on Output display_shape={out.display_shape!r}")
    _assert(out.display_shape == "LIST", f"toggle on expected LIST, got {out.display_shape!r}")
    menu_sock.menu_multi_selection = False
    bpy.context.view_layer.update()
    out = ms.outputs[0]
    print(f"  toggle_off Output display_shape={out.display_shape!r}")
    _assert(
        out.display_shape != "LIST",
        f"toggle multi True→False must leave non-LIST shape, got {out.display_shape!r}",
    )
    results["toggle_on_shape"] = "LIST"
    results["toggle_off_shape"] = out.display_shape

    # multi off from start (never multi)
    _clear_scene()
    tree = bpy.data.node_groups.new("MS_Single_Tree", "GeometryNodeTree")
    menu_sock = tree.interface.new_socket(
        name="Menu", in_out="INPUT", socket_type="NodeSocketMenu"
    )
    menu_sock.menu_multi_selection = False
    tree.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    tree.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    gi = tree.nodes.new("NodeGroupInput")
    go = tree.nodes.new("NodeGroupOutput")
    ms = tree.nodes.new("GeometryNodeMenuSwitch")
    ms.data_type = "GEOMETRY"
    while len(list(ms.enum_items)) < 2:
        ms.enum_items.new(chr(ord("A") + len(list(ms.enum_items))))
    tree.links.new(gi.outputs["Menu"], ms.inputs["Menu"])
    bpy.context.view_layer.update()
    out = ms.outputs[0]
    print(f"  multi_off Output display_shape={out.display_shape!r}")
    _assert(
        out.display_shape != "LIST",
        f"multi off Output must not be LIST, got {out.display_shape!r}",
    )
    grid = tree.nodes.new("GeometryNodeMeshGrid")
    grid.inputs["Vertices X"].default_value = 2
    grid.inputs["Vertices Y"].default_value = 2
    tree.links.new(grid.outputs["Mesh"], go.inputs["Geometry"])
    host_mesh = bpy.data.meshes.new("MS_SingleHostMesh")
    host_mesh.from_pydata([(0, 0, 0)], [], [])
    host = bpy.data.objects.new("MS_SingleHost", host_mesh)
    bpy.context.scene.collection.objects.link(host)
    mod = host.modifiers.new("MS_Single", "NODES")
    mod.node_group = tree
    depsgraph = bpy.context.evaluated_depsgraph_get()
    he = host.evaluated_get(depsgraph)
    nverts = len(he.to_mesh().vertices)
    he.to_mesh_clear()
    print("single_no_crash verts", nverts)
    _assert(nverts >= 1, f"multi off eval crashed/empty, verts={nverts}")
    results["single_shape"] = out.display_shape
    results["single_no_crash"] = True
    print("PASS multi_off_shape")
    print("PASS toggle_multi_off")

    return results


def main() -> int:
    try:
        test_source_static()
        results = test_list_shape_and_lengths()
        print("RUNTIME_RESULTS", results)
    except Exception as exc:
        print("FAIL:", type(exc).__name__, exc)
        traceback.print_exc()
        return 1
    print("bl_menu_switch_multi_list: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
