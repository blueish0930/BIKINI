# SPDX-FileCopyrightText: 2026 Blender Authors
# SPDX-License-Identifier: GPL-2.0-or-later
"""Smoke test for expanded ImageNodeTree (compositor category parity + Viewer).

Run:
  blender --background --factory-startup --python tests/python/image_nodes_expansion_smoke.py
"""
import bpy
import sys


def main() -> int:
    bpy.ops.wm.read_factory_settings(use_empty=True)

    tree = bpy.data.node_groups.new("ImageExpansionSmoke", "ImageNodeTree")
    types = [
        "CompositorNodeImage",
        "CompositorNodeBrightContrast",
        "CompositorNodeBlur",
        "ShaderNodeTexNoise",
        "CompositorNodeRotate",
        "ShaderNodeMath",
        "ImageNodeViewer",
        "CompositorNodeRGB",
    ]
    nodes = []
    for tid in types:
        nodes.append(tree.nodes.new(tid))

    rgb = next(n for n in nodes if n.bl_idname == "CompositorNodeRGB")
    viewer = next(n for n in nodes if n.bl_idname == "ImageNodeViewer")
    tree.links.new(rgb.outputs[0], viewer.inputs[0])

    ok, used_gpu = tree.evaluate()
    if not ok:
        print("evaluate failed")
        return 1

    print("SMOKE_OK", "gpu=", used_gpu, "types=", types)
    return 0


if __name__ == "__main__":
    sys.exit(main())
