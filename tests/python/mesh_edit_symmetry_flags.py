# SPDX-FileCopyrightText: 2026 Blender Authors
# SPDX-License-Identifier: GPL-2.0-or-later
"""Regression: Mesh Symmetry must copy mark-sharp / mark-seam / shade to the other side."""

import sys
import bpy
import bmesh


def fail(msg):
    print("FAIL:", msg, flush=True)
    raise SystemExit(1)


def ok(msg):
    print("OK:", msg, flush=True)


def setup_cube():
    bpy.ops.wm.read_homefile(use_empty=True)
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    mesh = obj.data
    mesh.use_mirror_x = True
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="DESELECT")
    bpy.ops.mesh.select_mode(type="EDGE")
    return obj, mesh


def selected_and_mirror_sharp(mesh):
    bm = bmesh.from_edit_mesh(mesh)
    bm.edges.ensure_lookup_table()
    selected = [e for e in bm.edges if e.select]
    if not selected:
        fail("no selected edges")
    e = selected[0]
    mx, my, mz = e.verts[0].co.x + e.verts[1].co.x, e.verts[0].co.y + e.verts[1].co.y, e.verts[0].co.z + e.verts[1].co.z
    mirror = None
    for other in bm.edges:
        if other is e:
            continue
        ox = other.verts[0].co.x + other.verts[1].co.x
        oy = other.verts[0].co.y + other.verts[1].co.y
        oz = other.verts[0].co.z + other.verts[1].co.z
        if abs(ox + mx) < 1e-5 and abs(oy - my) < 1e-5 and abs(oz - mz) < 1e-5:
            mirror = other
            break
    if mirror is None:
        fail("could not find mirrored edge")
    return e, mirror


def test_mark_sharp():
    obj, mesh = setup_cube()
    bm = bmesh.from_edit_mesh(mesh)
    bm.edges.ensure_lookup_table()
    # Pick an edge with x > 0 (not on the mirror plane).
    for e in bm.edges:
        mid_x = 0.5 * (e.verts[0].co.x + e.verts[1].co.x)
        if mid_x > 0.1:
            e.select = True
            break
    bmesh.update_edit_mesh(mesh)
    bpy.ops.mesh.mark_sharp()
    e, mirror = selected_and_mirror_sharp(mesh)
    if e.smooth:
        fail("selected edge should be sharp")
    if mirror.smooth:
        fail("mirrored edge should be sharp too")
    if mirror.select:
        fail("mirrored edge must not become selected")
    ok("mark_sharp")

    bpy.ops.mesh.mark_sharp(clear=True)
    e, mirror = selected_and_mirror_sharp(mesh)
    if not e.smooth or not mirror.smooth:
        fail("clear sharp should unmark both sides")
    ok("clear_sharp")


def test_mark_seam():
    obj, mesh = setup_cube()
    bm = bmesh.from_edit_mesh(mesh)
    bm.edges.ensure_lookup_table()
    for e in bm.edges:
        mid_x = 0.5 * (e.verts[0].co.x + e.verts[1].co.x)
        if mid_x > 0.1:
            e.select = True
            break
    bmesh.update_edit_mesh(mesh)
    bpy.ops.mesh.mark_seam()
    e, mirror = selected_and_mirror_sharp(mesh)
    if not e.seam:
        fail("selected edge should be a seam")
    if not mirror.seam:
        fail("mirrored edge should be a seam too")
    if mirror.select:
        fail("mirrored edge must not become selected")
    ok("mark_seam")


def test_shade_smooth():
    obj, mesh = setup_cube()
    bpy.ops.mesh.select_mode(type="FACE")
    bm = bmesh.from_edit_mesh(mesh)
    bm.faces.ensure_lookup_table()
    for f in bm.faces:
        cx = sum(v.co.x for v in f.verts) / len(f.verts)
        if cx > 0.1:
            f.select = True
            break
    bmesh.update_edit_mesh(mesh)
    bpy.ops.mesh.faces_shade_flat()
    bm = bmesh.from_edit_mesh(mesh)
    selected = [f for f in bm.faces if f.select]
    if not selected:
        fail("no selected face")
    f = selected[0]
    cx = sum(v.co.x for v in f.verts) / len(f.verts)
    cy = sum(v.co.y for v in f.verts) / len(f.verts)
    cz = sum(v.co.z for v in f.verts) / len(f.verts)
    mirror = None
    for other in bm.faces:
        if other is f:
            continue
        ox = sum(v.co.x for v in other.verts) / len(other.verts)
        oy = sum(v.co.y for v in other.verts) / len(other.verts)
        oz = sum(v.co.z for v in other.verts) / len(other.verts)
        if abs(ox + cx) < 1e-5 and abs(oy - cy) < 1e-5 and abs(oz - cz) < 1e-5:
            mirror = other
            break
    if mirror is None:
        fail("could not find mirrored face")
    if f.smooth:
        fail("selected face should be flat")
    if mirror.smooth:
        fail("mirrored face should be flat too")
    if mirror.select:
        fail("mirrored face must not become selected")
    ok("faces_shade_flat")


def main():
    test_mark_sharp()
    test_mark_seam()
    test_shade_smooth()
    print("ALL PASSED", flush=True)


if __name__ == "__main__":
    main()
    sys.exit(0)
