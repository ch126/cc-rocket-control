"""Run existing attitude regressions against flight.lua, then vertical tests."""
import sys
sys.argv=[sys.argv[0], 'flight.lua']
import test_stabilize as t

# Pure altitude model includes fractional hover requirements and variable sample times.
for hover in (4.7,5.0,5.3):
    for gain in (.5,1,3):
        a=t.table(hover=5,target=60,floor=57,integral=0,descending=False)
        y,vy=60.,0.
        for i in range(1500):
            dt=(.1,.15,.2)[i%3]
            power=t.mod.heightStep(a,y,vy,dt,0,5)
            assert 3<=power<=8
            vy+=(power-hover)*gain*dt
            y+=vy*dt
        assert abs(y-60)<.8 and abs(vy)<.8, (hover,gain,y,vy)
        a.descending=True
        for i in range(1500):
            dt=.15
            power=t.mod.heightStep(a,y,vy,dt,0,5)
            vy+=(power-hover)*gain*dt; y+=vy*dt
        assert abs(y-57)<.8 and not a.descending, (hover,gain,y,vy)

# Full application: ground UP, attitude calibration, release, near hover, V, N.
for delay in (0,.05,.1):
    setup=t.setup+[t.key('h')]+t.ticks*60+[('lift',8)]+t.ticks*60+[t.key('v')]
    r=t.simulate(setup+t.ticks*60+[t.key('n')]+t.ticks*220,api_delay=delay,gain=.05)
    text='\n'.join(' '.join(x[2]) for x in r)
    assert 'HOLD ALT' in text and 'HOLD DESCEND' in text, text[-1800:]
    assert abs(r[-1][5]-55)<.8, (delay,r[-1])
    assert all(1<=x[1]['bottom']<=10 for x in r if 'HOLD ALT' in ' '.join(x[2]) or 'HOLD DESCEND' in ' '.join(x[2]))
    for override in ('r','f','c','h','v','space'):
        r=t.simulate(setup+t.ticks*2+[t.key(override)]+t.ticks*2,api_delay=delay,gain=.05)
        assert 'HOLD ALT' not in ' '.join(r[-1][2]), (override,r[-1])
    r=t.simulate(setup+[t.key('n')]+t.ticks*10+[t.key('n')]+t.ticks*2,api_delay=delay,gain=.05)
    assert 'HOLD ALT' in ' '.join(r[-1][2]) and 'DESCEND' not in ' '.join(r[-1][2])
    r=t.simulate(setup+[('vertical_kick',6)]+t.ticks*2,api_delay=delay,gain=.05)
    assert r[-1][1]['bottom']==0

r=t.simulate(t.setup+[t.key('h')]+t.ticks*30+[t.key('v')],gain=.05)
assert 'HOLD ALT' not in ' '.join(r[-1][2]), 'must not enable at launch/reference height'
print('PASS: fractional hover, varied vertical response/sample intervals, limited descent and settling, actual V/N state transitions, manual overrides, pause descent, reference-height gate, fast-fall stop, delayed APIs.')

# Cross-run persistence: use actual file lifecycle with in-memory CC filesystem.
import json
path='/rocket-calibration-v1.cfg'
records={}
r=t.simulate(t.setup,records=records)
assert path in records, r[-1]
saved=json.loads(records[path])
assert saved['version']==1 and saved['throttle']==5 and saved['id']=='test-craft'
r=t.simulate(t.ticks*3,records=records)
assert 'calibration OK' in ' '.join(r[-1][2]) and r[-1][1]['bottom']==0
assert 'Loaded; verify craft unchanged' in ' '.join(r[-1][2])
r=t.simulate(t.ticks+[t.key('j'),t.key('e')]+[t.key('r')]*5+[t.key('h')]+t.ticks*5,records=records)
assert 'ARMED HOLD' in ' '.join(r[-1][2])
for bad in ('broken',json.dumps(dict(saved,id='other')),json.dumps(dict(saved,mass=50)),json.dumps(dict(saved,throttle=0)),json.dumps(dict(saved,version=99))):
    r=t.simulate(t.ticks,records={path:bad})
    assert 'LOAD REJECTED' in ' '.join(r[-1][2]) and 'calibration needed' in ' '.join(r[-1][2])
    assert r[-1][1]['bottom']==0
r=t.simulate(t.ticks,records=records.copy(),craft_id='new-craft')
assert 'LOAD REJECTED' in ' '.join(r[-1][2])
backup={path+'.bak':records[path]}
r=t.simulate(t.ticks,records=backup)
assert 'calibration OK' in ' '.join(r[-1][2])
r=t.simulate(t.setup,records={},write_fail=True)
assert 'SAVE FAILED' in ' '.join(r[-1][2]) and 'calibration OK' in ' '.join(r[-1][2])
r=t.simulate(t.ticks+[t.key('p'),t.key('l')]+t.ticks,records=records)
assert path+'.bak' in records and r[-1][1]['bottom']==0

# Genuine coroutine suspension: sensor waits for task_complete while keys arrive.
boot=[('api',)]*5+t.ticks+[('api',)]
armed=boot+[t.key('u'),t.key('e'),t.key('r')]+t.ticks
r=t.simulate(armed+[t.key('r'),('held','r',.13),('held','r',.01),t.key('space')]+[('api',)],yielding=True)
assert [x[1]['bottom'] for x in r[-5:]]==[2,3,3,0,0], r[-7:]
for exit_key in ('q','space'):
    r=t.simulate(armed+[t.key(exit_key)],yielding=True)
    if exit_key=='space': assert r[-1][1]['bottom']==0
r=t.simulate(armed+[('held','e',.2)],yielding=True)
assert r[-1][1]['bottom']==1, 'held E must not toggle'
print('PASS: saved calibration across restart, zero-thrust load, identity/mass/schema/corrupt-file rejection, backup recovery, disk failure, P/L, live keys and emergency stop during suspended sensor API, rate-limited repeats, toggle debounce.')
