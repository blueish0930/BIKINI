# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Run with Blender -b --python-exit-code 1 --python this_file.py -- [steps] [depth] [B depth]."""

import bpy
import sys

args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
steps = int(args[0]) if args else 2
depth = int(args[1]) if len(args) > 1 else 3
forwarding_depth = int(args[2]) if len(args) > 2 else None
bpy.ops.wm.read_factory_settings(use_empty=True)


def new_group(name):
    tree = bpy.data.node_groups.new(name, "GeometryNodeTree")
    tree.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    n = tree.interface.new_socket(name="Steps", in_out="INPUT", socket_type="NodeSocketInt")
    n.default_value = steps
    tree.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    inp = tree.nodes.new("NodeGroupInput")
    out = tree.nodes.new("NodeGroupOutput")
    out.is_active_output = True
    return tree, inp, out


a, ai, ao = new_group("RecursiveA")
b, bi, bo = new_group("ForwardingB")
a.is_modifier = True
a.is_recursive = True
a.recursive_depth = depth
if forwarding_depth is not None:
    b.is_recursive = True
    b.recursive_depth = forwarding_depth

call_a = b.nodes.new("GeometryNodeGroup")
call_a.node_tree = a
b.links.new(bi.outputs["Geometry"], call_a.inputs["Geometry"])
b.links.new(bi.outputs["Steps"], call_a.inputs["Steps"])
b.links.new(call_a.outputs["Geometry"], bo.inputs["Geometry"])

call_b = a.nodes.new("GeometryNodeGroup")
call_b.node_tree = b
transform = a.nodes.new("GeometryNodeTransform")
transform.inputs["Translation"].default_value = (1, 0, 0)
subtract = a.nodes.new("ShaderNodeMath")
subtract.operation = "SUBTRACT"
subtract.inputs[1].default_value = 1
compare = a.nodes.new("FunctionNodeCompare")
compare.data_type = "INT"
compare.operation = "LESS_EQUAL"
compare.inputs["B"].default_value = 0
switch = a.nodes.new("GeometryNodeSwitch")
switch.input_type = "GEOMETRY"
a.links.new(ai.outputs["Geometry"], transform.inputs["Geometry"])
a.links.new(transform.outputs["Geometry"], call_b.inputs["Geometry"])
a.links.new(ai.outputs["Steps"], subtract.inputs[0])
a.links.new(subtract.outputs[0], call_b.inputs["Steps"])
a.links.new(ai.outputs["Steps"], compare.inputs["A"])
a.links.new(compare.outputs[0], switch.inputs["Switch"])
a.links.new(call_b.outputs["Geometry"], switch.inputs["False"])
a.links.new(ai.outputs["Geometry"], switch.inputs["True"])
a.links.new(switch.outputs[0], ao.inputs["Geometry"])

mesh = bpy.data.meshes.new("Triangle")
mesh.from_pydata([(0, 0, 0), (1, 0, 0), (0, 1, 0)], [], [(0, 1, 2)])
obj = bpy.data.objects.new("Probe", mesh)
bpy.context.scene.collection.objects.link(obj)
obj.modifiers.new("GN", "NODES").node_group = a
bpy.context.view_layer.update()
evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
result = evaluated.to_mesh()
xs = [v.co.x for v in result.vertices]
print(f"RECURSION_INDIRECT steps={steps} x={xs}", flush=True)
limit = min(depth, forwarding_depth) if forwarding_depth is not None else depth
assert xs == ([float(steps), float(steps + 1), float(steps)] if steps <= limit else [])
evaluated.to_mesh_clear()
