# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Registration + socket gating for ShaderNodeParallaxOcclusion."""

import inspect
import sys

import bpy


def fail(msg):
    print("FAIL:", msg)
    raise RuntimeError(msg)


def check(cond, msg):
    if not cond:
        fail(msg)
    print("PASS:", msg)


def test_registration():
    check(hasattr(bpy.types, "ShaderNodeParallaxOcclusion"), "bpy.types.ShaderNodeParallaxOcclusion")

    import bl_ui.node_add_menu_shader as menu_sh

    src = inspect.getsource(menu_sh)
    check(
        'node_operator(layout, "ShaderNodeParallaxOcclusion")' in src,
        "shader Displacement menu lists Parallax Occlusion",
    )

    mat = bpy.data.materials.new("POM_TEST")
    mat.use_nodes = True
    n = mat.node_tree.nodes.new("ShaderNodeParallaxOcclusion")
    check(n.bl_idname == "ShaderNodeParallaxOcclusion", "node idname")
    check(n.mode == "POM", f"default mode is POM (got {n.mode})")
    check(n.channel == "R", f"default channel is R (got {n.channel})")
    check(n.invert is False, "default invert off")
    check(n.clip is False, "default clip off")

    modes = {item.identifier for item in n.bl_rna.properties["mode"].enum_items}
    for e in ("OFFSET", "STEEP", "POM", "POM_SHADOW"):
        check(e in modes, f"mode enum has {e}")
    check("SPOM" not in modes, "Parallax Occlusion mode enum has no SPOM")

    ins = {s.identifier: s for s in n.inputs}
    outs = {s.identifier: s for s in n.outputs}
    for name in ("Image", "Vector", "Scale", "Midlevel", "Samples", "Refine", "Normal", "Incoming"):
        check(name in ins, f"input {name}")
    check("Light" in ins, "input Light exists (may be hidden)")
    for name in ("Vector", "Height", "Shadow"):
        check(name in outs, f"output {name}")

    check(ins["Samples"].enabled, "Samples visible in POM mode")
    check(ins["Refine"].enabled, "Refine visible in POM mode")
    check(not ins["Light"].enabled, "Light hidden in POM mode")

    n.mode = "OFFSET"
    check(not ins["Samples"].enabled, "Samples hidden in Offset mode")
    n.mode = "POM_SHADOW"
    check(ins["Light"].enabled, "Light visible in POM + Shadow")
    check(abs(ins["Scale"].default_value - 0.05) < 1e-5, "default Scale 0.05")

    print("OK: ShaderNodeParallaxOcclusion")

    check(hasattr(bpy.types, "ShaderNodeSPOM"), "bpy.types.ShaderNodeSPOM")
    check(
        'node_operator(layout, "ShaderNodeSPOM")' in src,
        "shader Displacement menu lists SPOM",
    )
    spom = mat.node_tree.nodes.new("ShaderNodeSPOM")
    check(spom.bl_idname == "ShaderNodeSPOM", "SPOM node idname")
    spom_ins = {s.identifier: s for s in spom.inputs}
    spom_outs = {s.identifier: s for s in spom.outputs}
    for name in ("Image", "Vector", "Scale", "Midlevel", "Samples", "Refine", "Incoming"):
        check(name in spom_ins, f"SPOM input {name}")
    for name in ("Vector", "Height", "Alpha"):
        check(name in spom_outs, f"SPOM output {name}")
    check("mode" not in spom.bl_rna.properties, "SPOM node has no mode enum")
    print("OK: ShaderNodeSPOM")


if __name__ == "__main__":
    try:
        test_registration()
    except Exception:
        import traceback

        traceback.print_exc()
        sys.exit(1)
