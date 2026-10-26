# SPDX-License-Identifier: GPL-2.0-or-later
import bpy,bmesh
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.mesh.primitive_cube_add();o=bpy.context.object
t=bpy.data.node_groups.new('Convex hull edit regression','GeometryNodeTree')
t.interface.new_socket(name='Geometry',in_out='INPUT',socket_type='NodeSocketGeometry');t.interface.new_socket(name='Geometry',in_out='OUTPUT',socket_type='NodeSocketGeometry')
a=t.nodes.new('NodeGroupInput');z=t.nodes.new('NodeGroupOutput');h=t.nodes.new('GeometryNodeConvexHull');t.links.new(a.outputs[0],h.inputs[0]);t.links.new(h.outputs[0],z.inputs[0])
m=o.modifiers.new('Hull','NODES');m.node_group=t;m.show_in_editmode=True
print('CONVEX_EDIT_BEGIN',flush=True)
bpy.ops.object.mode_set(mode='EDIT')
for step in range(20):
 bm=bmesh.from_edit_mesh(o.data);bm.verts.ensure_lookup_table();bm.verts[0].co.x-=.01
 bmesh.update_edit_mesh(o.data,loop_triangles=True,destructive=False)
 bpy.context.view_layer.update();e=o.evaluated_get(bpy.context.evaluated_depsgraph_get());me=e.to_mesh();assert len(me.vertices)>=4;e.to_mesh_clear()
 print('CONVEX_EDIT_STEP',step,flush=True)
bpy.ops.object.mode_set(mode='OBJECT')
print('CONVEX_EDIT_PASS',flush=True)
