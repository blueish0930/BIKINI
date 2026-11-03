import bpy,json,sys,hashlib,traceback,os
from pathlib import Path
import numpy as np
import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--output-dir',type=Path,required=True)
parser.add_argument('--label',default='jit')
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
ROOT=args.output_dir;ROOT.mkdir(parents=True,exist_ok=True)
label=args.label
os.environ['BLENDER_VEX_NO_GPU']='1' # Isolate the CPU backend being replaced.
cases=json.loads(Path(__file__).with_name('wrangle_jit_cases.json').read_text(encoding='utf-8'))
for obj in list(bpy.data.objects):bpy.data.objects.remove(obj,do_unlink=True)
mesh=bpy.data.meshes.new('JIT compatibility input')
points=[(x*.125,y*.125,((x+3*y)%7)*.03125) for y in range(8) for x in range(8)]
faces=[]
for y in range(7):
 for x in range(7):
  a=y*8+x;faces.extend([(a,a+1,a+9),(a,a+9,a+8)])
mesh.from_pydata(points,[],faces)
sentinel=mesh.attributes.new('keep_me','FLOAT','POINT');sentinel.data.foreach_set('value',np.arange(64,dtype=np.float32))
obj=bpy.data.objects.new('JIT compatibility',mesh);bpy.context.collection.objects.link(obj)
g=bpy.data.node_groups.new('JIT compatibility','GeometryNodeTree')
g.interface.new_socket(name='Geometry',in_out='INPUT',socket_type='NodeSocketGeometry')
g.interface.new_socket(name='Geometry',in_out='OUTPUT',socket_type='NodeSocketGeometry')
gi=g.nodes.new('NodeGroupInput');n=g.nodes.new('GeometryNodeWrangle');go=g.nodes.new('NodeGroupOutput')
g.links.new(gi.outputs['Geometry'],n.inputs['0']);g.links.new(n.outputs['Geometry'],go.inputs['Geometry'])
m=obj.modifiers.new('JIT compatibility','NODES');m.node_group=g
area=next(a for a in bpy.context.screen.areas if a.type=='VIEW_3D');area.type='NODE_EDITOR';area.ui_type='GeometryNodeTree';area.spaces.active.node_tree=g
g.nodes.active=n;n.select=True
rows=[]
only=None
prior_rows=json.loads((ROOT/f'compatibility_{label}.json').read_text()) if only is not None else []
def plain(value):
 return value if isinstance(value,(int,float,bool,str)) else [plain(x) for x in value]
for index,case in enumerate(cases):
 if only is not None and index!=only:continue
 row=dict(index=index,name=case['name'],source=case['code'])
 setup=None
 try:
  if case.get('setup_code'):
   row['setup_source']=case['setup_code']
   setup=g.nodes.new('GeometryNodeWrangle');setup.code=case['setup_code'];setup.domain=case.get('setup_domain',case['domain'])
   g.links.new(gi.outputs['Geometry'],setup.inputs['0']);g.links.new(setup.outputs['Geometry'],n.inputs['0'])
   g.nodes.active=setup
   with bpy.context.temp_override(area=area,space_data=area.spaces.active):bpy.ops.node.wrangle_compile()
  g.nodes.active=n
  n.code=case['code'];n.domain=case['domain']
  with bpy.context.temp_override(area=area,space_data=area.spaces.active):bpy.ops.node.wrangle_compile()
  obj.update_tag();ev=obj.evaluated_get(bpy.context.evaluated_depsgraph_get());out=ev.to_mesh()
  row['warnings']=[(w.type,w.message) for w in m.node_warnings]
  row['vertices']=len(out.vertices);row['faces']=len(out.polygons)
  row['topology']=[list(p.vertices) for p in out.polygons]
  xyz=np.empty(len(out.vertices)*3,dtype=np.float32);out.vertices.foreach_get('co',xyz)
  values={'positions':hashlib.sha256(xyz.tobytes()).hexdigest()}
  for a in out.attributes:
   if a.name.startswith('.'):continue
   if not hasattr(a,'data'):values[a.name]=dict(type=a.data_type,domain=a.domain);continue
   data=[]
   for value_index,item in enumerate(a.data):
    # Existing geometry creation leaves inherited/unassigned new attribute slots
    # unspecified. Check the original slots and explicitly assigned new values.
    if a.domain=='POINT' and value_index>=64:
     if a.name=='keep_me':data.append(None);continue
     if case['name']=='addpoint_and_setattribute_signatures_compile' and a.name=='scale' and (value_index-64)%2==1:data.append(None);continue
     if case['name']=='addpoint_appends_and_setattr_writes' and a.name=='scale':data.append(None);continue
    if a.domain=='FACE' and value_index>=98 and a.name=='sharp_face':data.append(None);continue
    for key in ('value','vector','color'):
     if hasattr(item,key):
      data.append(plain(getattr(item,key)));break
   values[a.name]=dict(type=a.data_type,domain=a.domain,data=data)
  row['values']=values
  ev.to_mesh_clear()
 except Exception as error:row['exception']=str(error)
 if setup:
  g.nodes.remove(setup);g.links.new(gi.outputs['Geometry'],n.inputs['0'])
 rows.append(row)
 if only is not None:
  prior_rows[index]=row
 print('COMPATIBILITY_CASE',index,case['name'],'warning_count',len(row.get('warnings',[])),flush=True)
 (ROOT/f'compatibility_{label}.json').write_text(json.dumps(prior_rows if only is not None else rows,indent=2,allow_nan=True,default=lambda value:list(value)),encoding='utf-8')
print('COMPATIBILITY_CORPUS_DONE',label,len(rows),flush=True)
