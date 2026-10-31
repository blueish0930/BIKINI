# SPDX-FileCopyrightText: 2026 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import json,math,re,sys,time
from pathlib import Path
OUT=Path(sys.argv[1]).resolve()
ROOT=OUT.parent
mode=OUT.name
# Reads <output-dir-name>.err beside the fixture output directory.
deadline=time.time()+180
def wait_stage(stage):
    while time.time()<deadline:
        path=OUT/'events.json'
        if path.exists():
            try:
                events=json.loads(path.read_text())
                if events and events[-1]['stage']==stage:return events[-1]
            except json.JSONDecodeError:pass
        failed=OUT/'failed.txt'
        if failed.exists():raise AssertionError(failed.read_text())
        time.sleep(.1)
    raise TimeoutError(stage)
wait_stage('00-ready')
steps=[
    ({'action':'depth','depth':2},3,2,'hit'),
    ({'action':'depth','depth':0},1,0,'hit'),
    ({'action':'depth','depth':1},2,1,'hit'),
    ({'action':'depth','depth':3},4,3,'miss'),
    ({'action':'depth','depth':3},4,3,'hit'),
    ({'action':'depth','depth':2},3,2,'hit'),
    ({'action':'viewer','viewer':'Viewer B'},3.25,2,'hit'),
    ({'action':'viewer','viewer':'Viewer A'},3,2,'hit'),
    ({'action':'shift','x':2},6,2,'miss'),
    ({'action':'depth','depth':2},6,2,'hit'),
    ({'action':'viewer','viewer':'Viewer B'},6.25,2,'hit'),
    ({'action':'downstream','x':.5},6.5,2,'hit'),
    ({'action':'input','x':10},16.5,2,'miss'),
    ({'action':'depth','depth':1},14.5,1,'hit'),
    ({'action':'unused'},14.5,1,None),
    ({'action':'disable'},14.5,1,None),
    ({'action':'enable'},14.5,1,'miss'),
    ({'action':'steps','steps':4},14.5,1,'miss'),
    ({'action':'depth','depth':4},20.5,4,'miss'),
    ({'action':'depth','depth':0},12.5,0,'hit'),
    ({'action':'steps','steps':2},12.5,0,'miss'),
    ({'action':'depth','depth':2},16.5,2,'miss'),
    ({'action':'depth','depth':2},16.5,2,'hit'),
    ({'action':'save'},16.5,2,None),
]
for i,(command,expected,depth,cache) in enumerate(steps,1):
    stage=f'{i:02d}-{command["action"]}'
    command=dict(command,stage=stage)
    (OUT/'command.pending').write_text(json.dumps(command))
    (OUT/'command.pending').replace(OUT/'command.json')
    event=wait_stage(stage)
    assert len(event['viewer_path'])==4+depth,(stage,event['viewer_path'])
    assert event['object']=='Recursive Viewer Cache'
    assert event['caches']['Recursive Viewer Cache Test/Recursive Self Call']>0,event
    print('GUI_STEP_DONE',stage,'viewer first x expected',expected,flush=True)
    time.sleep(.1)
log=(ROOT/(mode+'.err')).read_text(encoding='utf8')
parts={}
current='00-ready';parts[current]=''
for line in log.splitlines():
    if line.startswith('PROBE_ACTION '):
        current=json.loads(line[len('PROBE_ACTION '):])['stage'];parts[current]=''
    elif not line.startswith('PROBE_END '):parts[current]+=line+'\n'
pattern=re.compile(r'GN_VIEWER node="([^"]+)" hash=(\d+:\d+) points=(\d+) first=([^,]+),([^,]+),([^\s]+)')
viewer='Viewer A';records=[];hashes={};last=None;steps_value=3;dx=1;base=0
events={e['stage']:e for e in json.loads((OUT/'events.json').read_text())}
for i,(command,expected,depth,cache) in enumerate(steps,1):
    stage=f'{i:02d}-{command["action"]}'
    if command['action']=='viewer':viewer=command['viewer']
    if command['action']=='shift':dx=command['x']
    if command['action']=='input':base=command['x']
    if command['action']=='steps':steps_value=command['steps']
    body=parts[stage]
    matches=[m for m in pattern.findall(body) if m[0]==viewer]
    if matches:
        # Other viewports can request their previous context until the Viewer-path notifier
        # reaches them. Select the inspected invocation, not the last thread to log a Viewer.
        if depth in hashes:
            matches=[m for m in matches if m[1]==hashes[depth]]
        else:
            matches=[m for m in matches if math.isclose(float(m[3]),expected,abs_tol=1e-6)]
        assert matches,(stage,'No Viewer log for the inspected invocation',body)
        value=matches[-1];last=value
    else:
        assert command['action'] in {'unused','save'},(stage,'Viewer was not evaluated',body)
        value=last
    assert int(value[2])==3,(stage,value)
    assert math.isclose(float(value[3]),expected,abs_tol=1e-6),(stage,value,expected,body)
    assert float(value[4])==0 and float(value[5])==0,(stage,value)
    if depth in hashes:assert hashes[depth]==value[1],(stage,hashes,value)
    hashes[depth]=value[1]
    if cache is not None:
        assert re.search('GN_CACHE '+cache+r'\s+node="Cached Shift"',body),(stage,cache,body)
    expected_points=[[base+steps_value*dx,0,0],[1+steps_value*dx,0,0],[steps_value*dx,1,0]]
    assert events[stage]['modifier_points']==expected_points,(stage,events[stage],expected_points)
    if command['action']=='disable':assert 'Recursive Viewer Cache Test/Cached Shift' not in events[stage]['caches']
    records.append({'stage':stage,'viewer':viewer,'depth':depth,'first_x':float(value[3]),'context_hash':value[1],'cached_shift_expected':cache,'cache_events':re.findall(r'GN_CACHE (hit|miss|store)\s+node="([^"]+)"',body)})
assert len(set(hashes.values()))==len(hashes),hashes
(OUT/'verification.json').write_text(json.dumps({'status':'pass','checks':records,'distinct_contexts':hashes},indent=2))
print('RECURSIVE_VIEWER_CACHE_GUI_PASS',len(records),flush=True)
(OUT/'command.pending').write_text(json.dumps({'action':'quit','stage':'quit'}))
(OUT/'command.pending').replace(OUT/'command.json')
