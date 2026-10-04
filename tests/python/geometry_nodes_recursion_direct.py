# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Run with Blender -b --python-exit-code 1 --python this_file.py -- [steps] [depth] [branch]."""

import bpy
import sys


args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
steps = int(args[0]) if args else 3
depth = int(args[1]) if len(args) > 1 else 4
branch = len(args) > 2 and args[2] == "branch"
save_path = args[3] if branch and len(args) > 3 else (args[2] if len(args) > 2 and not branch else None)


def mark(message):
    print("RECURSION_TEST", message, flush=True)


bpy.ops.wm.read_factory_settings(use_empty=True)
tree = bpy.data.node_groups.new("RecursiveGeometryProbe", "GeometryNodeTree")
tree.is_modifier = True
tree.is_recursive = True
tree.recursive_depth = depth
tree.interface.new_socket(name="Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
steps_socket = tree.interface.new_socket(name="Steps", in_out="INPUT", socket_type="NodeSocketInt")
steps_socket.default_value = steps
tree.interface.new_socket(name="Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
nodes = tree.nodes
links = tree.links
group_in = nodes.new("NodeGroupInput")
group_out = nodes.new("NodeGroupOutput")
group_out.is_active_output = True
mark("interface")
call = nodes.new("GeometryNodeGroup")
call.node_tree = tree
mark("self-reference-added")
transform = nodes.new("GeometryNodeTransform")
transform.inputs["Translation"].default_value = (1, 0, 0)
subtract = nodes.new("ShaderNodeMath")
subtract.operation = "SUBTRACT"
subtract.inputs[1].default_value = 1
compare = nodes.new("FunctionNodeCompare")
compare.data_type = "INT"
compare.operation = "LESS_EQUAL"
compare.inputs["B"].default_value = 0
switch = nodes.new("GeometryNodeSwitch")
switch.input_type = "GEOMETRY"
links.new(group_in.outputs["Geometry"], transform.inputs["Geometry"])
links.new(transform.outputs["Geometry"], call.inputs["Geometry"])
links.new(group_in.outputs["Steps"], subtract.inputs[0])
links.new(subtract.outputs[0], call.inputs["Steps"])
links.new(group_in.outputs["Steps"], compare.inputs["A"])
links.new(compare.outputs[0], switch.inputs["Switch"])
if branch:
    second_call = nodes.new("GeometryNodeGroup")
    second_call.node_tree = tree
    links.new(transform.outputs["Geometry"], second_call.inputs["Geometry"])
    links.new(subtract.outputs[0], second_call.inputs["Steps"])
    join = nodes.new("GeometryNodeJoinGeometry")
    links.new(call.outputs["Geometry"], join.inputs["Geometry"])
    links.new(second_call.outputs["Geometry"], join.inputs["Geometry"])
    links.new(join.outputs["Geometry"], switch.inputs["False"])
else:
    links.new(call.outputs["Geometry"], switch.inputs["False"])
links.new(group_in.outputs["Geometry"], switch.inputs["True"])
links.new(switch.outputs[0], group_out.inputs["Geometry"])
mark("links-added")
obj = bpy.data.objects.new("ProbeCube", bpy.data.meshes.new("ProbeMesh"))
bpy.context.scene.collection.objects.link(obj)
mesh = obj.data
mesh.from_pydata([(0, 0, 0), (1, 0, 0), (0, 1, 0)], [], [(0, 1, 2)])
mesh.update()
mod = obj.modifiers.new("RecursiveProbe", "NODES")
mod.node_group = tree
mark("modifier-added")
bpy.context.view_layer.update()
mark("depsgraph-updated")
depsgraph = bpy.context.evaluated_depsgraph_get()
evaluated = obj.evaluated_get(depsgraph)
result = evaluated.to_mesh()
xs = [vertex.co.x for vertex in result.vertices]
mark(f"result vertices={len(result.vertices)} polygons={len(result.polygons)} x={xs}")
expected_vertices = 3 * (2 ** steps if branch else 1) if steps <= depth else 0
assert len(result.vertices) == expected_vertices, (steps, depth, xs)
if expected_vertices:
    assert xs.count(float(steps)) == expected_vertices * 2 // 3, (steps, depth, xs)
    assert xs.count(float(steps + 1)) == expected_vertices // 3, (steps, depth, xs)
evaluated.to_mesh_clear()
if save_path:
    bpy.ops.wm.save_as_mainfile(filepath=save_path)
    mark(f"saved {save_path}")
