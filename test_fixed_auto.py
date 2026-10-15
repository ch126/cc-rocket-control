"""Automatic fixed sequence; actual bundled Lua in async CC mock."""
from pathlib import Path
import json
import test_wired as t
from build_bundle import build_probe
bundle=Path(__file__).with_name('fixed_probe_bundle.lua').read_text(encoding='utf-8')
assert bundle==build_probe()
source='''textutils.serializeJSON=textutils.serialize
sleep=function(dt) local id=os.startTimer(dt); while true do local e,n=os.pullEventRaw(); if e=="timer" and n==id then return end end end
'''+bundle
def run(actions,**kwargs):
    return t.simulate(actions,source=source,engine_count=9,duration=20,program_args=('--auto',),**kwargs)
records={'/rocket-fixed-check-0001.json':'old report'}
h,c,g=run([t.key(1.7,'p'),t.key(2.5,'p')],records=records)
assert g.outcome[1],g.outcome[2]
assert records['/rocket-fixed-check-0001.json']=='old report'
report=json.loads(records['/rocket-fixed-check-0002.json'])
assert report['complete'] and report['cleanupConfirmed'] and not report['flightCalibrationValid']
assert len(report['engines'])==9
for i,e in enumerate(report['engines']):
    assert e['id']==i and e['interfaceCheckPassed'] and e['onObservedAt']<=e['stopRequestedAt']<e['zeroObservedAt']
    if i: assert e['requestedAt']>=report['engines'][i-1]['zeroObservedAt']+.5-1e-8
for state in h:
    assert sum(p>0 for p in state['powers'])<=1 and max(state['powers'])<=1000
assert all(e.gate==0 and e.output==0 for e in g.motors.values())
for event in (t.key(2,'space'),t.key(2,'q'),(2,'terminate'),(2,'lift',.2),
              (2,'disconnect','creative_vector_thruster_8'),(2,'slow_sensor',.5),(2,'slow_output',1)):
    records={}; h,c,g=run([t.key(1.7,'p'),event],records=records)
    assert g.outcome[1],g.outcome[2]
    report=json.loads(records['/rocket-fixed-check-0001.json'])
    assert not report['complete'] and not report['flightCalibrationValid'],event
    assert not any(x[2]=='setThrust' and x[3]>0 and x[1]!='creative_vector_thruster_0' for x in c),event
h,c,g=run([t.key(1.7,'p')],disk_capacity=1000)
assert not any(x[2]=='setThrust' and x[3]>0 for x in c)
print('PASS auto: 9 isolated pulses, confirmed zero/rest gates, report/no overwrite, restart ignored, operator/motion/link/timing aborts, disk gate.')
