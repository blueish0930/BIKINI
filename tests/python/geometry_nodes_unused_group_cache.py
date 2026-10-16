"""Regression: unused node groups must not invalidate the modifier output cache.

Run with a regular Python interpreter:
  python geometry_nodes_unused_group_cache.py --blender /path/to/blender
The fixture runs in Blender with its node editor pinned to the tested tree.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def fixture(reset=True):
    import bpy
    if reset:
        bpy.ops.wm.read_factory_settings(use_empty=True)

    def mesh_object(name, x):
        mesh = bpy.data.meshes.new(name)
        mesh.from_pydata([(x, 0, 0)], [], [])
        obj = bpy.data.objects.new(name, mesh)
        bpy.context.collection.objects.link(obj)
        return obj

    owner = mesh_object('Owner', 0)
    first = mesh_object('Used dependency', 1)
    second = mesh_object('Group dependency', 2)
    third = mesh_object('Nested unused dependency', 3)
    bpy.context.view_layer.objects.active = owner
    owner.select_set(True)

    def tree(name, source):
        tree = bpy.data.node_groups.new(name, 'GeometryNodeTree')
        tree.interface.new_socket(name='Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
        tree.interface.new_socket(name='Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')
        output = tree.nodes.new('NodeGroupOutput')
        info = tree.nodes.new('GeometryNodeObjectInfo')
        info.inputs['Object'].default_value = source
        tree.links.new(info.outputs['Geometry'], output.inputs['Geometry'])
        return tree, info, output

    root, info, output = tree('Root', first)
    child, child_info, child_output = tree('Child', second)
    nested, _, _ = tree('Nested', third)
    mod = owner.modifiers.new('Regression', 'NODES')
    mod.node_group = root
    area = bpy.context.screen.areas[0]
    area.type = 'NODE_EDITOR'
    area.ui_type = 'GeometryNodeTree'
    area.spaces.active.pin = True
    area.spaces.active.node_tree = root

    def step(name, change, expected, should_recompute=False):
        print('CACHE_STEP ' + json.dumps([name, should_recompute]), flush=True)
        change()
        bpy.context.view_layer.update()
        obj = owner.evaluated_get(bpy.context.evaluated_depsgraph_get())
        coords = [tuple(v.co) for v in obj.data.vertices]
        assert len(coords) == 1 and abs(coords[0][0] - expected) < 1e-6, (name, coords)
        print('CACHE_END ' + name, flush=True)

    step('initial', lambda: None, 1, True)
    step('ordinary unused', lambda: root.nodes.new('ShaderNodeMath'), 1)
    group = root.nodes.new('GeometryNodeGroup')
    step('assign unused group', lambda: setattr(group, 'node_tree', child), 1)
    step('drag link into unused group',
         lambda: root.links.new(info.outputs['Geometry'], group.inputs['Geometry']), 1)
    tail = root.nodes.new('GeometryNodeTransform')
    step('drag group output to unused branch',
         lambda: root.links.new(group.outputs['Geometry'], tail.inputs['Geometry']), 1)

    def add_operator():
        with bpy.context.temp_override(area=area):
            bpy.ops.node.add_group(name=child.name)
    step('add group operator', add_operator, 1)
    step('edit unused dependency', lambda: setattr(second.data.vertices[0], 'co', (4, 0, 0)), 1)
    second.data.update()
    step('flush unused dependency', lambda: None, 1)
    step('connect group to output',
         lambda: root.links.new(group.outputs['Geometry'], output.inputs['Geometry']), 4, True)

    def edit_used():
        second.data.vertices[0].co.x = 5
        second.data.update()
    step('edit used dependency', edit_used, 5, True)
    nested_node = child.nodes.new('GeometryNodeGroup')
    step('add unused group inside used group', lambda: setattr(nested_node, 'node_tree', nested), 5)
    step('connect nested group',
         lambda: child.links.new(nested_node.outputs['Geometry'], child_output.inputs['Geometry']), 3, True)
    step('disconnect group',
         lambda: root.links.new(info.outputs['Geometry'], output.inputs['Geometry']), 1, True)
    print('CACHE_FIXTURE_COMPLETE', flush=True)


def run(blender):
    env = dict(os.environ, BLENDER_GN_CACHE_DEBUG='1')
    args = [blender, '-b', '--factory-startup', '--python-exit-code', '1',
                             '--python', str(Path(__file__).resolve()), '--', '--fixture']
    result = subprocess.run(args, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(result.stdout)
    assert result.returncode == 0, result.returncode
    assert 'CACHE_FIXTURE_COMPLETE' in result.stdout
    current = None
    checks = []
    for line in result.stdout.splitlines():
        if line.startswith('CACHE_STEP '):
            current = json.loads(line[len('CACHE_STEP '):])
            misses = 0
            side_effect_misses = 0
        elif current and 'GN_CACHE miss modifier output' in line:
            misses += 1
        elif current and 'GN_CACHE miss side effects' in line:
            side_effect_misses += 1
        elif current and line.startswith('CACHE_END '):
            name, expected = current
            assert bool(misses) == expected, (name, misses, expected)
            if not expected:
                assert side_effect_misses == 0, (name, 'viewer recomputed')
            checks.append(name)
            current = None
    assert len(checks) == 13, checks
    print('PASS: {} cache and geometry checks'.format(len(checks)))


if __name__ == '__main__':
    if '--fixture' in sys.argv:
        fixture()
    else:
        parser = argparse.ArgumentParser()
        parser.add_argument('--blender', required=True)
        args = parser.parse_args()
        run(args.blender)
