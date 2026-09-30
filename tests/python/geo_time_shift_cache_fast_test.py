# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Regression: Geometry Nodes Time Shift own-cache fill is local (not Bake-class).

Exercises the shipped modifier evaluation path:
- Time Shift stores geometry at the cooked absolute frame.
- Requesting that frame from another scene frame is a cache hit (non-empty, correct values).
- Cook cost is a single frame update class (not multi-frame bake walk).

Run:
  blender -b -P tests/python/geo_time_shift_cache_fast_test.py
"""

import sys
import time

import bpy


def _stats(obj):
    deps = bpy.context.evaluated_depsgraph_get()
    deps.update()
    eo = obj.evaluated_get(deps)
    me = eo.to_mesh()
    try:
        n = len(me.vertices)
        z = float(me.vertices[0].co.z) if n else None
        return n, z
    finally:
        eo.to_mesh_clear()


def main() -> int:
    bpy.ops.wm.read_homefile(use_empty=True)
    scene = bpy.context.scene

    mesh = bpy.data.meshes.new("TSFastMesh")
    obj = bpy.data.objects.new("TSFastObj", mesh)
    scene.collection.objects.link(obj)

    mod = obj.modifiers.new("GN", "NODES")
    ng = bpy.data.node_groups.new("TSFastTree", "GeometryNodeTree")
    mod.node_group = ng
    ng.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")

    nodes, links = ng.nodes, ng.links
    out = nodes.new("NodeGroupOutput")
    grid = nodes.new("GeometryNodeMeshGrid")
    grid.inputs["Vertices X"].default_value = 8
    grid.inputs["Vertices Y"].default_value = 8
    st = nodes.new("GeometryNodeInputSceneTime")
    comb = nodes.new("ShaderNodeCombineXYZ")
    tr = nodes.new("GeometryNodeTransform")
    ts = nodes.new("GeometryNodeTimeShift")

    links.new(grid.outputs["Mesh"], tr.inputs["Geometry"])
    links.new(st.outputs["Frame"], comb.inputs["Z"])
    links.new(comb.outputs["Vector"], tr.inputs["Translation"])
    links.new(tr.outputs["Geometry"], ts.inputs["Geometry"])
    links.new(ts.outputs["Geometry"], out.inputs["Geometry"])

    # Request absolute frame 65 while scene is at 1. Fill is driven by evaluation + ensure path.
    scene.frame_set(1)
    ts.inputs["Frame"].default_value = 65
    v0, z0 = _stats(obj)
    print(f"after_request scene=1 ts=65 verts={v0} z={z0}", flush=True)

    # Explicit single-frame cook (same class as light_cook_object_at_frame in the C++ fill).
    t0 = time.perf_counter()
    scene.frame_set(65)
    bpy.context.view_layer.update()
    cook_s = time.perf_counter() - t0
    v_cook, z_cook = _stats(obj)
    print(f"cook_at_65 verts={v_cook} z={z_cook} s={cook_s:.6f}", flush=True)

    # Cache hit: scene back to 1, still requesting 65.
    t1 = time.perf_counter()
    scene.frame_set(1)
    bpy.context.view_layer.update()
    v_hit, z_hit = _stats(obj)
    hit_s = time.perf_counter() - t1
    print(f"hit scene=1 ts=65 verts={v_hit} z={z_hit} s={hit_s:.6f}", flush=True)

    t2 = time.perf_counter()
    v_hit2, z_hit2 = _stats(obj)
    hit2_s = time.perf_counter() - t2
    print(f"hit2 scene=1 ts=65 verts={v_hit2} z={z_hit2} s={hit2_s:.6f}", flush=True)

    # Different absolute frame still works.
    ts.inputs["Frame"].default_value = 1
    bpy.context.view_layer.update()
    v1, z1 = _stats(obj)
    print(f"ts=1 scene=1 verts={v1} z={z1}", flush=True)

    ok = (
        v_cook > 0
        and z_cook is not None
        and abs(z_cook - 65.0) < 0.01
        and v_hit > 0
        and z_hit is not None
        and abs(z_hit - 65.0) < 0.01
        and v_hit2 > 0
        and abs(z_hit2 - 65.0) < 0.01
        and v1 > 0
        and abs(z1 - 1.0) < 0.01
        # Single-frame class: well under multi-frame bake times (seconds).
        and cook_s < 0.5
        and hit_s < 0.5
    )
    print("GEO_TIME_SHIFT_CACHE_FAST: " + ("PASS" if ok else "FAIL"), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
