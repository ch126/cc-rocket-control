"""Entry/worker/save integration. Stub sequence completion, NOT flight evidence."""
import json
import test_wired as t
import test_allocator as a
from build_bundle import build_calibration

bundle=build_calibration()
stub='''
factories.calibration_flight=function() return {
  start=function(c,s,now) return {phase="RAMP",pad={x=s.x,y=s.y,z=s.z},window=1,trial=1,windows={}} end,
  step=function(_,state,s,now)
    state.phase="COMPLETE_HOVER"
    local commands={}
    for i=1,9 do commands[i]={name="creative_vector_thruster_"..(i-1),kn=i<=5 and 1 or 0,vectorX=0,vectorY=0} end
    return {commands=commands,result={testOnly=true,flightCalibrationValid=false}}
  end} end
'''
marker='-- Experimental flight commissioning. Explicit configuration'
assert bundle.count(marker)==1
source='''
textutils.unserializeJSON=textutils.unserialize
textutils.serializeJSON=textutils.serialize
os.queueEvent=function(name) enqueue(os.clock(),name) end
sleep=function(dt) local id=os.startTimer(dt); while true do local e,n=os.pullEventRaw(); if e=="timer" and n==id then return end end end
'''+bundle.replace(marker,stub+marker)
config=dict(craft=dict(id='test-craft',profile=a.p,main=a.main,top=a.top,
    basis=dict(up=a.vec(0,1,0),a=a.vec(1,0,0),b=a.vec(0,0,-1))))
for failure in (False,True):
    records={'config.json':json.dumps(config),'/rocket-flight-calibration-0001.json':'OLD-KEEP'}
    h,c,g=t.simulate([t.key(1.2,'p'),t.key(1.6,'p'),t.key(2.8,'q')],source=source,gui=True,
        engine_count=9,duration=4,program_args=('--live','config.json'),records=records,log_failure=failure)
    assert g.outcome[1],g.outcome[2]
    assert records['/rocket-flight-calibration-0001.json']=='OLD-KEEP'
    assert any(call[2]=='setThrust' and call[3]==15 and call[0]>2 for call in c), 'Must keep controlling after save/failure'
    assert all(e.gate==0 and e.output==0 for e in g.motors.values())
    report=records.get('/rocket-flight-calibration-0002.json')
    if failure: assert report is None
    else: assert json.loads(report)['result']['testOnly']
print('PASS real app + sampler + asynchronous workers: explicit start, one new report, no overwrite, hold after save/failure, Q drains outputs. Sequence stubbed; not flight evidence.')
