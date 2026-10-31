# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""Check recursive cached geometry against an independent translation/count oracle.

Run in Blender with --background --factory-startup --python-exit-code 1
--python this_file.py -- <output-json>. Covers direct recursion, an indirect
A -> B -> A cycle, and two distinct recursive branches in the same invocation.
"""

from collections import Counter
import json
import math
from pathlib import Path
import sys

import bpy


RESULTS = []


def new_tree(name):
    tree = bpy.data.node_groups.new(name, 'GeometryNodeTree')
    tree.is_modifier = True
    tree.interface.new_socket(name='Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
    steps = tree.interface.new_socket(name='Steps', in_out='INPUT', socket_type='NodeSocketInt')
    steps.default_value = 3
    tree.interface.new_socket(name='Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')
    return tree, tree.nodes.new('NodeGroupInput'), tree.nodes.new('NodeGroupOutput')


def fixture(mode):
    root, root_in, root_out = new_tree('Cache Root ' + mode)
    rec, inp, out = new_tree('Recursive ' + mode)
    rec.is_recursive = True
    rec.recursive_depth = 16
    called = rec
    trees = [root, rec]
    if mode == 'indirect':
        called, forward_in, forward_out = new_tree('Forwarding ' + mode)
        forward = called.nodes.new('GeometryNodeGroup')
        forward.node_tree = rec
        called.links.new(forward_in.outputs['Geometry'], forward.inputs['Geometry'])
        called.links.new(forward_in.outputs['Steps'], forward.inputs['Steps'])
        called.links.new(forward.outputs['Geometry'], forward_out.inputs['Geometry'])
        trees.append(called)

    outer = root.nodes.new('GeometryNodeGroup')
    outer.node_tree = rec
    outer.name = 'Cached outer ' + mode
    post = root.nodes.new('GeometryNodeTransform')
    root.links.new(root_in.outputs['Geometry'], outer.inputs['Geometry'])
    root.links.new(root_in.outputs['Steps'], outer.inputs['Steps'])
    root.links.new(outer.outputs['Geometry'], post.inputs['Geometry'])
    root.links.new(post.outputs['Geometry'], root_out.inputs['Geometry'])
    cmp = rec.nodes.new('FunctionNodeCompare')
    cmp.data_type = 'INT'
    cmp.operation = 'LESS_EQUAL'
    sub = rec.nodes.new('FunctionNodeIntegerMath')
    sub.operation = 'SUBTRACT'
    sub.inputs[1].default_value = 1
    switch = rec.nodes.new('GeometryNodeSwitch')
    switch.input_type = 'GEOMETRY'
    rec.links.new(inp.outputs['Steps'], cmp.inputs['A'])
    rec.links.new(inp.outputs['Steps'], sub.inputs[0])
    rec.links.new(cmp.outputs[0], switch.inputs['Switch'])
    rec.links.new(inp.outputs['Geometry'], switch.inputs['True'])
    rec.links.new(switch.outputs[0], out.inputs['Geometry'])
    shifts = []
    calls = []
    for branch in range(2 if mode == 'branch' else 1):
        shift = rec.nodes.new('GeometryNodeTransform')
        shift.name = 'Cached shift ' + str(branch)
        call = rec.nodes.new('GeometryNodeGroup')
        call.node_tree = called
        call.name = 'Cached call ' + str(branch)
        rec.links.new(inp.outputs['Geometry'], shift.inputs['Geometry'])
        rec.links.new(shift.outputs['Geometry'], call.inputs['Geometry'])
        rec.links.new(sub.outputs[0], call.inputs['Steps'])
        shifts.append(shift)
        calls.append(call)
    if len(calls) == 2:
        join = rec.nodes.new('GeometryNodeJoinGeometry')
        for call in calls:
            rec.links.new(call.outputs['Geometry'], join.inputs['Geometry'])
        rec.links.new(join.outputs[0], switch.inputs['False'])
    else:
        rec.links.new(calls[0].outputs['Geometry'], switch.inputs['False'])

    cached = [n for tree in trees for n in tree.nodes
              if n.bl_idname in {'GeometryNodeTransform', 'GeometryNodeGroup'} and n != post]
    for node in cached:
        node.use_output_cache = True
    mesh = bpy.data.meshes.new(mode)
    mesh.from_pydata([(0, 0, 0), (1, 0, 0), (0, 1, 0)], [], [(0, 1, 2)])
    obj = bpy.data.objects.new(mode, mesh)
    bpy.context.scene.collection.objects.link(obj)
    mod = obj.modifiers.new('Cache Test', 'NODES')
    mod.node_group = root
    steps_id = next(s.identifier for s in root.interface.items_tree if s.name == 'Steps' and s.in_out == 'INPUT')

    def run(stage, steps, dx, base, offset, depth=16):
        print('RECURSIVE_CACHE_STEP', mode, stage, flush=True)
        for i, shift in enumerate(shifts):
            value = (dx + 10 * i, 0, 0)
            if tuple(shift.inputs['Translation'].default_value) != value:
                shift.inputs['Translation'].default_value = value
        if rec.recursive_depth != depth:
            rec.recursive_depth = depth
        if mesh.vertices[0].co.x != base:
            mesh.vertices[0].co.x = base
            mesh.update()
        if tuple(post.inputs['Translation'].default_value) != (offset, 0, 0):
            post.inputs['Translation'].default_value = (offset, 0, 0)
        if getattr(mod.properties.inputs, steps_id).value != steps:
            getattr(mod.properties.inputs, steps_id).value = steps
        obj.update_tag()
        bpy.context.view_layer.update()
        evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
        actual = Counter(tuple(round(v, 5) for v in vert.co) for vert in evaluated.data.vertices)
        expected = Counter()
        if steps <= depth:
            for k in range(steps + 1 if mode == 'branch' else 1):
                multiplicity = math.comb(steps, k) if mode == 'branch' else 1
                translation = steps * dx + 10 * k + offset
                for x, y, z in [(base, 0, 0), (1, 0, 0), (0, 1, 0)]:
                    expected[(round(x + translation, 5), y, z)] += multiplicity
        assert actual == expected, (mode, stage, actual, expected)
        entry_counts = {tree.name + '/' + n.name: n.output_cache_debug_count
                        for tree in trees for n in tree.nodes if n in cached}
        if stage == 'cache disabled':
            assert not any(entry_counts.values()), entry_counts
        if stage == 'initial':
            assert all(n.output_cache_debug_count > 0 for n in calls), entry_counts
        RESULTS.append({'mode': mode, 'stage': stage, 'vertices': sum(actual.values()), 'entries': entry_counts})
        print('RECURSIVE_CACHE_END', mode, stage, flush=True)

    run('initial', 3, 1, 0, 0)
    run('downstream edit', 3, 1, 0, .5)
    run('recursive node edit', 3, 2, 0, .5)
    run('input geometry edit', 3, 2, 10, .5)
    run('modifier input edit', 2, 2, 10, .5)
    run('base case', 0, 2, 10, .5)
    run('grow recursion', 4, 2, 10, .5)
    for node in cached:
        node.use_output_cache = False
    run('cache disabled', 3, 2, 10, .5)
    for node in cached:
        node.use_output_cache = True
    run('cache reenabled', 3, 2, 10, .5)
    run('depth limit', 5, 2, 10, .5, depth=4)
    run('depth restored', 5, 2, 10, .5)


for mode in ['direct', 'indirect', 'branch']:
    fixture(mode)

path = Path(sys.argv[sys.argv.index('--') + 1])
path.write_text(json.dumps({'build': bpy.app.build_hash.decode(), 'checks': RESULTS}, indent=2), encoding='utf8')
print('RECURSIVE_CACHE_PASS', len(RESULTS), flush=True)
