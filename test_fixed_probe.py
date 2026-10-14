"""Actual single-file fixed probe in async simulated CC; no game ignition."""
from pathlib import Path
import test_wired as t
from build_bundle import build_probe
bundle=Path(__file__).with_name('fixed_probe_bundle.lua').read_text(encoding='utf-8')
assert bundle==build_probe()
source='''sleep=function(dt) local id=os.startTimer(dt); while true do local e,n=os.pullEventRaw(); if e=="timer" and n==id then return end end end
'''+bundle
def run(actions): return t.simulate(actions,source=source,engine_count=9,duration=4)
for index in range(9):
    actions=[t.key(.8+i*.1,'right') for i in range(index)]
    pulse=1.7
    h,c,g=run(actions+[t.key(pulse,'p'),t.key(3,'q')])
    assert g.outcome[1],g.outcome[2]
    active=[x for x in c if x[2]=='setThrust' and x[3]>0]
    assert active and all(x[1]==f'creative_vector_thruster_{index}' for x in active),active
    assert all(engine.gate==0 and engine.output==0 for engine in g.motors.values())
    assert max(max(s['powers']) for s in h)<=1000
    assert all(max(s['powers'])==0 for s in h if s['time']>2.4)
for action in (t.key(1.85,'space'),t.key(1.85,'q'),(1.85,'terminate'),(1.85,'lift',.2),(1.85,'kick',.06,0),(1.85,'slow_sensor',.5)):
    h,c,g=run([t.key(1.7,'p'),action])
    assert g.outcome[1],g.outcome[2]
    assert all(engine.gate==0 and engine.output==0 for engine in g.motors.values())
h,c,g=run([t.key(1,'q')]); assert not any(x[2]=='setThrust' and x[3]>0 for x in c)
print('PASS fixed probe: all nine selections isolated, 1 kN cap, timed stop, no auto-ignite, stop/exit/terminate/movement/rotation/stale telemetry cleanup.')
