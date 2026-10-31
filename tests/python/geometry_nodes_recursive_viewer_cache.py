# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy,json,sys,time,traceback
from pathlib import Path
# Run with --factory-startup --enable-event-simulate and an output directory after --.
OUT=Path(sys.argv[sys.argv.index('--')+1]).resolve()
OUT.mkdir(parents=True,exist_ok=True)
STATE={'events':[],'start':time.time()}
def context():
    return bpy.context.temp_override(window=STATE['window'],area=STATE['area'],region=next(r for r in STATE['area'].regions if r.type=='WINDOW'))
def group(name):
    g=bpy.data.node_groups.new(name,'GeometryNodeTree');g.is_modifier=True
    g.interface.new_socket(name='Geometry',in_out='INPUT',socket_type='NodeSocketGeometry')
    s=g.interface.new_socket(name='Steps',in_out='INPUT',socket_type='NodeSocketInt');s.default_value=3
    g.interface.new_socket(name='Geometry',in_out='OUTPUT',socket_type='NodeSocketGeometry')
    return g,g.nodes.new('NodeGroupInput'),g.nodes.new('NodeGroupOutput')
def snap(stage):
    w=STATE['window'];a=STATE['area'];s=a.spaces.active
    obj=STATE['obj'];dg=bpy.context.evaluated_depsgraph_get();eo=obj.evaluated_get(dg)
    points=[list(v.co) for v in eo.data.vertices]
    sp=next(a.spaces.active for a in w.screen.areas if a.type=='SPREADSHEET')
    result={'stage':stage,'build':bpy.app.build_hash.decode(),'pin':s.pin,'object':s.id.name if s.id else None,'editor_path':[p.node_tree.name for p in s.path],'viewer_path':[(p.type,p.ui_name) for p in sp.viewer_path.path],'modifier_points':points,'caches':{g.name+'/'+n.name:n.output_cache_debug_count for g in [STATE['root'],STATE['rec']] for n in g.nodes if n.use_output_cache}}
    STATE['events'].append(result)
    (OUT/'events.json').write_text(json.dumps(STATE['events'],indent=2),encoding='utf8')
    with bpy.context.temp_override(window=w):bpy.ops.screen.screenshot(filepath=str(OUT/(stage+'.png')))
    print('PROBE_STATE',json.dumps(result),flush=True)
    print('PROBE_END',stage,file=sys.stderr,flush=True)
def setup():
    try:
        w=bpy.context.window;w.workspace=bpy.data.workspaces['Geometry Nodes'];STATE['window']=w
        for o in list(bpy.data.objects):bpy.data.objects.remove(o,do_unlink=True)
        mesh=bpy.data.meshes.new('Triangle');mesh.from_pydata([(0,0,0),(1,0,0),(0,1,0)],[],[(0,1,2)]);mesh.update()
        obj=bpy.data.objects.new('Recursive Viewer Cache',mesh);bpy.context.scene.collection.objects.link(obj)
        bpy.context.view_layer.objects.active=obj;obj.select_set(True)
        root,ri,ro=group('Viewer Cache Wrapper')
        rec,inp,out=group('Recursive Viewer Cache Test');rec.is_recursive=True;rec.recursive_depth=8
        call=root.nodes.new('GeometryNodeGroup');call.node_tree=rec;call.name='Cached Recursive Group';call.use_output_cache=True
        root.links.new(ri.outputs['Geometry'],call.inputs['Geometry']);root.links.new(ri.outputs['Steps'],call.inputs['Steps']);root.links.new(call.outputs['Geometry'],ro.inputs['Geometry'])
        shift=rec.nodes.new('GeometryNodeTransform');shift.name='Cached Shift';shift.inputs['Translation'].default_value=(1,0,0);shift.use_output_cache=True
        selfcall=rec.nodes.new('GeometryNodeGroup');selfcall.node_tree=rec;selfcall.name='Recursive Self Call';selfcall.use_output_cache=True
        sub=rec.nodes.new('FunctionNodeIntegerMath');sub.operation='SUBTRACT';sub.inputs[1].default_value=1
        cmp=rec.nodes.new('FunctionNodeCompare');cmp.data_type='INT';cmp.operation='LESS_EQUAL';cmp.inputs['B'].default_value=0
        switch=rec.nodes.new('GeometryNodeSwitch');switch.input_type='GEOMETRY'
        rec.links.new(inp.outputs['Geometry'],shift.inputs['Geometry']);rec.links.new(shift.outputs['Geometry'],selfcall.inputs['Geometry'])
        rec.links.new(inp.outputs['Steps'],sub.inputs[0]);rec.links.new(sub.outputs[0],selfcall.inputs['Steps'])
        rec.links.new(inp.outputs['Steps'],cmp.inputs['A']);rec.links.new(cmp.outputs[0],switch.inputs['Switch'])
        rec.links.new(inp.outputs['Geometry'],switch.inputs['True']);rec.links.new(selfcall.outputs['Geometry'],switch.inputs['False']);rec.links.new(switch.outputs[0],out.inputs['Geometry'])
        preview=rec.nodes.new('GeometryNodeTransform');preview.name='Viewer Only Offset';preview.inputs['Translation'].default_value=(.25,0,0)
        rec.links.new(shift.outputs['Geometry'],preview.inputs['Geometry'])
        for name,source in [('Viewer A',shift),('Viewer B',preview)]:
            n=rec.nodes.new('GeometryNodeViewer');n.name=n.label=name;n.viewer_items.new('GEOMETRY','Geometry');rec.links.new(source.outputs['Geometry'],n.inputs['Geometry'])
        for i,n in enumerate(rec.nodes):n.location=(i*210,0)
        rec.nodes['Viewer A'].location=(0,400);rec.nodes['Viewer B'].location=(260,400);shift.location=(-250,400)
        obj.modifiers.new('Recursive Cache Probe','NODES').node_group=root
        area=next(a for a in w.screen.areas if a.type=='NODE_EDITOR');STATE.update(area=area,obj=obj,root=root,rec=rec)
        area.spaces.active.pin=False
        for a in w.screen.areas:
            if a.type=='VIEW_3D':
                a.spaces.active.shading.type='WIREFRAME';a.spaces.active.show_viewer=True
        bpy.app.timers.register(enter,first_interval=1)
    except:traceback.print_exc();bpy.ops.wm.quit_blender()
def enter():
    try:
        with context():
            STATE['root'].nodes.active=STATE['root'].nodes['Cached Recursive Group']
            bpy.ops.node.group_edit(exit=False)
        bpy.app.timers.register(ready,first_interval=1)
    except:traceback.print_exc();bpy.ops.wm.quit_blender()
def ready():
    try:
        with context():
            g=STATE['rec'];v=g.nodes['Viewer A'];g.nodes.active=v
            for n in g.nodes:n.select=n==v or n.name in ['Viewer B','Cached Shift']
            bpy.ops.node.activate_viewer();bpy.ops.node.view_selected()
        bpy.app.timers.register(open_tree,first_interval=1)
    except:traceback.print_exc();bpy.ops.wm.quit_blender()
def open_tree():
    try:
        with context():bpy.ops.node.recursion_tree()
        bpy.app.timers.register(monitor_start,first_interval=1)
    except:traceback.print_exc();bpy.ops.wm.quit_blender()
def monitor_start():
    snap('00-ready')
    bpy.app.timers.register(monitor,first_interval=.3)
def monitor():
    command=OUT/'command.json'
    try:
        if time.time()-STATE['start']>1800:bpy.ops.wm.quit_blender();return None
        if command.exists():
            c=json.loads(command.read_text(encoding='utf-8-sig'));command.unlink()
            action=c.get('action','snapshot')
            print('PROBE_ACTION',json.dumps(c),flush=True)
            print('PROBE_ACTION',json.dumps(c),file=sys.stderr,flush=True)
            if action=='quit':bpy.ops.wm.quit_blender();return None
            if action=='viewer':
                with context():
                    STATE['rec'].nodes.active=STATE['rec'].nodes[c['viewer']]
                    bpy.ops.node.activate_viewer()
            elif action=='depth':
                tree_window=next(w for w in bpy.context.window_manager.windows if w!=STATE['window'])
                area=next(a for a in tree_window.screen.areas if a.type=='NODE_EDITOR')
                region=next(r for r in area.regions if r.type=='WINDOW')
                scale=bpy.context.preferences.system.ui_scale
                x,y=region.view2d.view_to_region(0,-64*scale*c['depth'],clip=False)
                x+=region.x;y+=region.y
                print('PROBE_CLICK',c['depth'],x,y,region.width,region.height,scale,flush=True)
                tree_window.event_simulate(type='MOUSEMOVE',value='NOTHING',x=x,y=y)
                def click():
                    tree_window.event_simulate(type='LEFTMOUSE',value='PRESS',x=x,y=y)
                    tree_window.event_simulate(type='LEFTMOUSE',value='RELEASE',x=x,y=y)
                bpy.app.timers.register(click,first_interval=.15)
            elif action=='shift':STATE['rec'].nodes['Cached Shift'].inputs['Translation'].default_value=(c['x'],0,0)
            elif action=='input':
                STATE['obj'].data.vertices[0].co.x=c['x'];STATE['obj'].data.update()
            elif action=='downstream':STATE['rec'].nodes['Viewer Only Offset'].inputs['Translation'].default_value=(c['x'],0,0)
            elif action=='disable':STATE['rec'].nodes['Cached Shift'].use_output_cache=False
            elif action=='enable':STATE['rec'].nodes['Cached Shift'].use_output_cache=True
            elif action=='steps':
                sock=next(i for i in STATE['root'].interface.items_tree if i.name=='Steps' and i.in_out=='INPUT')
                getattr(STATE['obj'].modifiers[0].properties.inputs,sock.identifier).value=c['steps'];STATE['obj'].update_tag()
            elif action=='unused':STATE['rec'].nodes.new('ShaderNodeMath')
            elif action=='save':bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'Recursive Viewer Cache - Validation.blend'))
            bpy.app.timers.register(lambda:snap(c['stage']),first_interval=.7)
    except:traceback.print_exc();(OUT/'failed.txt').write_text(traceback.format_exc())
    return .3
def start():
    bpy.context.window.workspace=bpy.data.workspaces['Geometry Nodes']
    bpy.app.timers.register(setup,first_interval=1)
bpy.app.timers.register(start,first_interval=1)
