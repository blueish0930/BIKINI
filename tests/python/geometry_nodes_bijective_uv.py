# SPDX-FileCopyrightText: 2026 Blender Authors
# SPDX-License-Identifier: GPL-2.0-or-later

"""Post-build acceptance and matched benchmarks for the native Bijective UV node.

Run with the built Blender, --background --factory-startup --python this_file --
--output report.json [--asset initial_horse.npz] [--iterations 500].
No source file or existing blend is modified. A missing native node is an error.
The NPZ asset must contain xyz, tri and lb_xprev (or UVMap), with optional
bij_epsilon and bij_weight. Every method receives identical float inputs.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import bpy
import numpy as np


METHODS = ('STANDARD_LBFGS', 'PRECONDITIONED_LBFGS', 'ASSEMBLED_SYSTEM')
MENU_LABELS = dict(zip(METHODS, ('Standard L-BFGS', 'Preconditioned L-BFGS',
                               'Assembled Linear System')))


def fixture(n=12):
    x, y = np.meshgrid(np.linspace(-1, 1, n), np.linspace(-1, 1, n))
    xyz = np.column_stack((x.ravel(), y.ravel(), (.35 * np.cos(2*x)*np.cos(2*y)).ravel()))
    triangles = []
    for j in range(n-1):
        for i in range(n-1):
            a = j*n+i
            triangles.extend(((a, a+1, a+n+1), (a, a+n+1, a+n)))
    return xyz, np.array(triangles), xyz[:, :2].copy()


def metrics(xyz, tri, uv):
    """Independent NumPy distortion plus boundary intersection verification."""
    e = xyz[tri[:, 1]]-xyz[tri[:, 0]]
    d = xyz[tri[:, 2]]-xyz[tri[:, 0]]
    length = np.linalg.norm(e, axis=1)
    x = np.sum(e*d, axis=1)/length
    height = np.linalg.norm(np.cross(e, d), axis=1)/length
    a = (uv[tri[:, 1]]-uv[tri[:, 0]])/length[:, None]
    b = (uv[tri[:, 2]]-uv[tri[:, 0]]-x[:, None]*a)/height[:, None]
    det = a[:, 0]*b[:, 1]-a[:, 1]*b[:, 0]
    sd = (np.sum(a*a, axis=1)+np.sum(b*b, axis=1))*(1+1/det**2)
    area = length*height*.5
    edges = np.sort(np.concatenate((tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]])), axis=1)
    edges, counts = np.unique(edges, axis=0, return_counts=True)
    boundary = edges[counts == 1]
    crossings = 0
    cross = lambda a, b: a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]
    for i, edge in enumerate(boundary):
        other = boundary[i+1:]
        if len(other) == 0:
            continue
        disjoint = np.all(other != edge[0], axis=1) & np.all(other != edge[1], axis=1)
        other = other[disjoint]
        a, b = uv[edge]
        c, d = uv[other[:, 0]], uv[other[:, 1]]
        bounds = np.all(np.minimum(a, b) <= np.maximum(c, d), axis=1)
        bounds &= np.all(np.minimum(c, d) <= np.maximum(a, b), axis=1)
        c1, c2 = cross(b-a, c-a), cross(b-a, d-a)
        c3, c4 = cross(d-c, a-c), cross(d-c, b-c)
        crossings += int(np.sum(bounds & (c1*c2 <= 0) & (c3*c4 <= 0)))
    return dict(sd_mean=float(area@sd/area.sum()), sd_max=float(sd.max()),
                min_det=float(det.min()), flips=int(np.sum(det <= 0)),
                boundary_crossings=crossings)


def evaluate(xyz, tri, initial, method, iterations, tolerance, epsilon=0, weight=1):
    mesh = bpy.data.meshes.new('Bijective acceptance input')
    mesh.from_pydata(xyz.tolist(), [], tri.tolist())
    obj = bpy.data.objects.new('Bijective acceptance', mesh)
    bpy.context.collection.objects.link(obj)
    group = bpy.data.node_groups.new('Bijective acceptance', 'GeometryNodeTree')
    group.interface.new_socket(name='Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
    group.interface.new_socket(name='Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')
    nodes, links = group.nodes, group.links
    source, target = nodes.new('NodeGroupInput'), nodes.new('NodeGroupOutput')
    solver = nodes.new('GeometryNodeBijectiveUV')
    solver.inputs['Method'].default_value = MENU_LABELS[method]
    solver.inputs['Iterations'].default_value = iterations
    solver.inputs['Tolerance'].default_value = tolerance
    solver.inputs['Barrier Distance'].default_value = epsilon
    solver.inputs['Barrier Weight'].default_value = weight
    links.new(source.outputs['Geometry'], solver.inputs['Mesh'])
    if initial is not None:
        attr = mesh.attributes.new('initial_uv', 'FLOAT_VECTOR', 'POINT')
        values = np.column_stack((initial[:, :2], np.zeros(len(initial)))).astype(np.float32)
        attr.data.foreach_set('vector', values.ravel())
        read = nodes.new('GeometryNodeInputNamedAttribute')
        read.data_type = 'FLOAT_VECTOR'
        read.inputs['Name'].default_value = 'initial_uv'
        solver.inputs['Use Initial UV'].default_value = True
        links.new(read.outputs['Attribute'], solver.inputs['Initial UV'])
    output = solver.outputs['Mesh']
    for name, kind, domain in [('UV', 'FLOAT_VECTOR', 'CORNER'),
                               ('Status', 'INT', 'POINT'),
                               ('Iterations Used', 'INT', 'POINT'),
                               ('Line Search Trials', 'INT', 'POINT'),
                               ('Distortion', 'FLOAT', 'POINT'),
                               ('Barrier Energy', 'FLOAT', 'POINT'),
                               ('Solve Time', 'FLOAT', 'POINT')]:
        store = nodes.new('GeometryNodeStoreNamedAttribute')
        store.data_type, store.domain = kind, domain
        store.inputs['Name'].default_value = 'result_'+name
        links.new(output, store.inputs['Geometry'])
        links.new(solver.outputs[name], store.inputs['Value'])
        output = store.outputs['Geometry']
    links.new(output, target.inputs['Geometry'])
    mod = obj.modifiers.new('Bijective acceptance', 'NODES')
    mod.node_group = group
    start = time.perf_counter()
    evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    result_mesh = evaluated.to_mesh()
    seconds = time.perf_counter()-start
    row = {'method': method, 'wall_seconds': seconds}
    for name in ['Status', 'Iterations Used', 'Line Search Trials', 'Distortion',
                 'Barrier Energy', 'Solve Time']:
        row[name] = result_mesh.attributes['result_'+name].data[0].value
    corner_uv = np.empty((len(result_mesh.loops), 3), dtype=np.float32)
    result_mesh.attributes['result_UV'].data.foreach_get('vector', corner_uv.ravel())
    uv = np.zeros((len(xyz), 2))
    for loop, value in zip(result_mesh.loops, corner_uv):
        uv[loop.vertex_index] = value[:2]
    evaluated.to_mesh_clear()
    bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.node_groups.remove(group)
    bpy.data.meshes.remove(mesh)
    return row, uv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--iterations', type=int, default=500)
    parser.add_argument('--tolerance', type=float, default=1e-8)
    args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    if not hasattr(bpy.types, 'GeometryNodeBijectiveUV'):
        raise RuntimeError('Native node is not in this Blender; build/install first')
    xyz, tri, initial = fixture()
    epsilon, weight = 0, 1
    if args.asset:
        data = np.load(args.asset)
        xyz, tri = data['xyz'], data['tri']
        initial = data['lb_xprev'] if 'lb_xprev' in data else data['UVMap']
        epsilon = float(data['bij_epsilon'][0]) if 'bij_epsilon' in data else 0
        weight = float(data['bij_weight'][0]) if 'bij_weight' in data else 1
    xyz = xyz.astype(np.float32).astype(float)
    initial = initial[:, :2].astype(np.float32).astype(float)
    before = metrics(xyz, tri, initial)
    assert before['flips'] == 0 and before['boundary_crossings'] == 0, before
    rows = []
    for method in METHODS:
        row, uv = evaluate(xyz, tri, initial, method, args.iterations, args.tolerance, epsilon, weight)
        assert row['Status'] != 3, row
        row.update(metrics(xyz, tri, uv))
        assert row['flips'] == 0 and row['boundary_crossings'] == 0, row
        assert np.isclose(row['sd_mean'], row['Distortion'], rtol=2e-5), row
        row['termination'] = {0: 'budget', 1: 'tolerance', 2: 'stalled'}[row['Status']]
        rows.append(row)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        np.savez(args.output.with_name(args.output.stem+'_'+method+'.npz'), xyz=xyz, tri=tri, UVMap=uv)
        print(json.dumps(row), flush=True)
    report = dict(build=bpy.app.build_hash.decode(), asset=str(args.asset),
                  iterations=args.iterations, tolerance=args.tolerance,
                  epsilon=epsilon, weight=weight, initial=before, results=rows)
    args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
