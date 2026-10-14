"""Real console entry in asynchronous CC mock. No physical flight claim."""
from pathlib import Path
import json
import test_wired as t
root=Path(__file__).parent
from build_bundle import build
bundle=(root/'rocket_bundle.lua').read_text(encoding='utf-8')
assert bundle==build(), 'Test the up-to-date released artifact'
source='''
colors={black=32768,white=1,cyan=512,orange=2,red=16384,yellow=16}
textutils.unserializeJSON=textutils.unserialize
sleep=function(dt) local id=os.startTimer(dt); while true do local e,n=os.pullEventRaw(); if e=="timer" and n==id then return end end end
'''
source+=bundle
for actions in ([t.key(.5,'e'),t.key(.7,'r'),t.key(1,'q')],[(.8,'terminate')],
                [(.5,'click',30,8),(.7,'click',1,9),(1,'click',30,12)]):
    h,c,g=t.simulate(actions,source=source,gui=True,engine_count=9,duration=2,expected_top=9)
    assert g.outcome[1],g.outcome[2]
    assert not c, c
    assert g.redstoneOutputs.top==9
print('PASS actual console: default read-only, control keys do not write, Q and terminate exit.')
invalid_source='local function entry(...)\n'+source+'\nend\nvalidationOutcome=table.pack(pcall(entry,...)); while os.pullEventRaw()~="terminate" do end'
h,c,g=t.simulate([],source=invalid_source,gui=True,engine_count=9,duration=1,
    expected_top=9,program_args=('--live','bad.json'),records={'bad.json':json.dumps({'caps':[733]*5+[4]*4,'profile':{}})})
assert not g.validationOutcome[1] and not c and g.redstoneOutputs.top==9
print('PASS actual console: invalid live profile rejected before initialization/writes.')

# Synthetic profile ONLY. Exercise real console + telemetry + output workers.
import test_allocator as a
profile=dict(version=2,craftId='test-craft',mass=100,gravityMin=9,gravityMax=10,
    accelPerThrottleMin=35,accelPerThrottleMax=37,maxThrottle=.9,hoverThrottle=.27,
    ignitionDelay=.4,margin=8,rcsVerified=True,verticalVerified=True,coupledRcsVerified=True,
    lateralAccelMin=1,lateralResponseDelay=.2)
config=dict(profile=profile,caps=[733]*5+[100]*4,limits=a.limits,
    craft=dict(id='test-craft',profile=a.p,main=a.main,top=a.top,
        basis=dict(up=a.vec(0,1,0),a=a.vec(1,0,0),b=a.vec(0,0,-1))))
for quit_event in (t.key(2.3,'q'),(2.3,'terminate')):
    h,c,g=t.simulate([t.key(1.2,'j'),t.key(1.4,'e'),t.key(1.6,'r'),quit_event],
        source=source,gui=True,engine_count=9,duration=3,program_args=('--live','synthetic.json'),
        records={'synthetic.json':json.dumps(config)})
    assert g.outcome[1],g.outcome[2]
    assert any(call[2]=='setThrust' and call[3]==15 for call in c),c
    assert all(engine.gate==0 and engine.output==0 for engine in g.motors.values())
print('PASS actual console + sampler + workers: explicit J/E/R produce output, Q/terminate drain and clear all nine.')
h,c,g=t.simulate([(1.2,'click',1,8),(1.4,'click',30,8),(1.6,'click',1,9),
                  (2.1,'click',1,12),(2.4,'click',30,12)],
    source=source,gui=True,engine_count=9,duration=3,program_args=('--live','synthetic.json'),
    records={'synthetic.json':json.dumps(config)})
assert g.outcome[1],g.outcome[2]
assert any(call[2]=='setThrust' and call[3]==15 for call in c)
assert all(engine.gate==0 and engine.output==0 for engine in g.motors.values())
print('PASS bundled clickable interface: pad/arm/throttle, emergency stop and exit share guarded action path.')
