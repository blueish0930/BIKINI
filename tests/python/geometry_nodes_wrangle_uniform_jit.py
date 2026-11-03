# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
# Run with blender --background --factory-startup --python THIS_FILE --
# --output-dir DIRECTORY --label jit (also vm, uniform, helpers).
# Compare results_LABEL.json with sorted-key JSON serialization, retaining signed zero.
import bpy,os,sys,json,hashlib,time,math,struct
from pathlib import Path
import numpy as np
import argparse
parser=argparse.ArgumentParser(description="Evaluate bound numeric channels across JIT, VM and helper paths.")
parser.add_argument('--output-dir',type=Path,required=True)
parser.add_argument('--label',choices=['jit','uniform','vm','helpers','baseline'],default='jit')
args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
root=args.output_dir;root.mkdir(parents=True,exist_ok=True);label=args.label
os.environ['BLENDER_VEX_NO_GPU']='1'
if label=='vm':os.environ['BLENDER_VEX_DISABLE_JIT']='1'
else:os.environ['BLENDER_VEX_JIT_FORCE']='1'
if label=='uniform':os.environ['BLENDER_VEX_JIT_NO_PARAMETER_SPECIALIZATION']='1'
if label=='helpers':os.environ['BLENDER_VEX_JIT_DISABLE_UNIFORM_PARAMETERS']='1'
os.environ['BLENDER_VEX_JIT_LOG']='1'
for o in list(bpy.data.objects):bpy.data.objects.remove(o,do_unlink=True)
area=next(a for a in bpy.context.screen.areas if a.type=='VIEW_3D');area.type='NODE_EDITOR';area.ui_type='GeometryNodeTree'
rows=[];timings=[]
def make(name,code,count=64):
 mesh=bpy.data.meshes.new(name);mesh.from_pydata([(i*.125,0,0) for i in range(count)],[],[])
 for attr,typ in [('Mode','INT'),('Weight','FLOAT')]:
  a=mesh.attributes.new(attr,typ,'POINT');a.data.foreach_set('value',np.arange(count,dtype=np.int32 if typ=='INT' else np.float32)+1)
 obj=bpy.data.objects.new(name,mesh);bpy.context.collection.objects.link(obj)
 g=bpy.data.node_groups.new(name,'GeometryNodeTree');g.interface.new_socket(name='Geometry',in_out='INPUT',socket_type='NodeSocketGeometry');g.interface.new_socket(name='Geometry',in_out='OUTPUT',socket_type='NodeSocketGeometry')
 gi=g.nodes.new('NodeGroupInput');n=g.nodes.new('GeometryNodeWrangle');go=g.nodes.new('NodeGroupOutput');n.code=code
 g.links.new(gi.outputs['Geometry'],n.inputs['0']);g.links.new(n.outputs['Geometry'],go.inputs['Geometry'])
 obj.modifiers.new('Uniform JIT','NODES').node_group=g;g.nodes.active=n;area.spaces.active.node_tree=g
 with bpy.context.temp_override(area=area,space_data=area.spaces.active):assert 'FINISHED' in bpy.ops.node.wrangle_compile()
 return obj,n
def setp(n,**kw):
 for k,v in kw.items():
  try:n.inputs[k].default_value=v
  except ValueError:
   current=n.inputs[k].default_value
   n.inputs[k].default_value=v[:len(current)]
def capture(name,obj,n,expected=None):
 obj.update_tag();start=time.perf_counter();ev=obj.evaluated_get(bpy.context.evaluated_depsgraph_get());mesh=ev.to_mesh();seconds=time.perf_counter()-start
 assert not obj.modifiers[0].node_warnings,[(w.type,w.message) for w in obj.modifiers[0].node_warnings]
 attrs={}
 for a in mesh.attributes:
  if a.name.startswith('.') or not hasattr(a,'data'):continue
  data=[]
  for item in a.data:
   for key in ('value','vector','color'):
    if hasattr(item,key):
     v=getattr(item,key);data.append(v if isinstance(v,(int,float,bool,str)) else list(v));break
  attrs[a.name]=dict(type=a.data_type,data=data)
 if expected is not None:expected(attrs)
 rows.append(dict(name=name,attributes=attrs));timings.append(dict(name=name,seconds=seconds))
 ev.to_mesh_clear();print('UNIFORM_CASE',name,flush=True)
 (root/f'results_{label}.json').write_text(json.dumps(rows,indent=2,allow_nan=True),encoding='utf-8')
code='''int mode=chi("Mode"),count=chi("Iterations");float weight=chf("Weight");int total=0;
for(int k=0;k<count;k++){if(mode==0)total+=k+i@index;else if(mode==1)total+=k%7;else total-=k;}
i@sum=total;i@mode=mode;f@result=weight*total;f@weight=weight;'''
obj,n=make('Scalar parameters',code)
sequence=[(i%3,2**(-i%6),17+i) for i in range(24)]+[(0,1.,17),(1,2.,18),(0,-0.,17),(2,0.125,20)]
for i,(mode,weight,count) in enumerate(sequence):
 setp(n,Mode=mode,Weight=weight,Iterations=count)
 def expected(a,mode=mode,weight=weight,count=count):
  sums=[sum(k+j if mode==0 else k%7 if mode==1 else -k for k in range(count)) for j in range(64)]
  assert a['sum']['data']==sums
  assert np.array_equal(np.asarray(a['result']['data'],dtype=np.float32),np.asarray(sums,dtype=np.float32)*np.float32(weight))
 capture('scalar_'+str(i),obj,n,expected)
other,on=make('Independent instance',code);setp(on,Mode=2,Iterations=9,Weight=-2.)
capture('instance_B',other,on);capture('instance_A_again',obj,n)
for frame in (1,10):
 setp(n,Mode=0 if frame==1 else 2,Weight=.25 if frame==1 else 2.,Iterations=20)
 n.inputs['Mode'].keyframe_insert('default_value',frame=frame);n.inputs['Weight'].keyframe_insert('default_value',frame=frame)
for frame in (1,5,10,5,1):
 bpy.context.scene.frame_set(frame);capture('animation_'+str(frame),obj,n)
typed='''i@out_i=chi("Int");f@out_f=chf("Float");b@out_b=chb("Bool");
v@out_v=chv("Vector");u@out_u=chu("Vector2");q@out_q=chq("Vector4");c@out_c=chc("Color");r@out_r=chr("Rotation");'''
t,tn=make('Numeric types',typed)
for i in range(3):
 setp(tn,Int=16777217+i,Float=-0. if i==0 else i*.125,Bool=bool(i%2),Vector=(i+.25,-.5,1.5),Vector2=(.5,i+.25),Vector4=(1.,2.,3.,4.+i),Color=(.25,.5,.75,1.),Rotation=(.1*i,.2,.3))
 capture('types_'+str(i),t,tn)
fallback,fn=make('Attribute fallback','i@result=chi("Mode");f@f=chf("Weight");')
while fn.input_items:fn.input_items.remove(fn.input_items[-1])
capture('varying_attribute_fallback',fallback,fn,lambda a: (None if a['result']['data']==list(range(1,65)) else (_ for _ in ()).throw(AssertionError('attribute fallback changed'))))
renamed,rn=make('Renamed parameter',code.replace('"Mode"','"Kind"'));setp(rn,Kind=2,Iterations=17,Weight=.5);capture('different_name_same_program_shape',renamed,rn)
dynamic,dn=make('Computed name','int keep=chi("Mode");string name="Mo";name+="de";i@result=chi(name);');setp(dn,Mode=7);capture('computed_name',dynamic,dn)
lists,ln=make('List parameter','int values[]=array(chi("Values"));int first=chi("Values");i@result=first+len(values);');capture('list_semantics',lists,ln)
bad,bn=make('Unexecuted nonfinite','float f=0;if(i@index<0)f=chf("Nonfinite");f@result=f;')
for v in (float('nan'),float('inf')):
 setp(bn,Nonfinite=v);capture('unexecuted_nonfinite_'+str(v),bad,bn)
(root/f'timings_{label}.json').write_text(json.dumps(timings,indent=2),encoding='utf-8')
print('UNIFORM_PARAMETER_VALIDATION_DONE',label,len(rows),flush=True)
