"""Synthetic measured-window acceptance, never actual flight approval."""
from pathlib import Path
import copy
import test_wired as t
lua=t.LuaRuntime(unpack_returned_tuples=True)
module=lua.execute(Path(__file__).with_name('calibration_evidence.lua').read_text(encoding='utf-8'))
run=dict(craftId='fixture',mass=93.5,windows=[])
for trial,u in enumerate((.23,.31,.23,.31)):
    window=[]
    for i in range(9):
        time=trial*2+i*.1
        window.append(dict(time=time,mass=93.5,craftId='fixture',relativeHeight=10,tilt=0,
            vy=(36*u-9.5)*i*.1,throttle=u,commandAge=.05,settledFor=.6,
            velocitySource='sublevel.getLinearVelocity',topKN=0,vectorX=0,vectorY=0,allMainEqual=True))
    run['windows'].append(window)
def evaluate(value): return module.vertical(lua.table_from(value,recursive=True))
candidate,why=evaluate(run)
assert candidate is not None,why
assert candidate.gravityMin<9.5<candidate.gravityMax
assert candidate.accelPerThrottleMin<36<candidate.accelPerThrottleMax
assert abs(candidate.hoverThrottle-9.5/36)<1e-9
assert candidate.testedThrottleMax==.31 and not candidate.flightCalibrationValid
planner=lua.execute(Path(__file__).with_name('recovery_plan.lua').read_text(encoding='utf-8'))
assert planner.validate(candidate)[0] is False, 'A partial fit must never authorize recovery'
for field,value in (('relativeHeight',0),('mass',71),('craftId','other'),('tilt',.1),
    ('velocitySource','filtered'),('commandAge',.4),('settledFor',.1),('topKN',1),
    ('vectorX',1/15),('allMainEqual',False),('throttle',float('nan'))):
    invalid=copy.deepcopy(run); invalid['windows'][1][2][field]=value
    result,why=evaluate(invalid); assert result is None and why,field
invalid=copy.deepcopy(run); invalid['windows'][1][3]['time']+=1
assert evaluate(invalid)[0] is None
invalid=copy.deepcopy(run); invalid['windows'][1]=invalid['windows'][0]
assert evaluate(invalid)[0] is None
invalid=copy.deepcopy(run)
for w in invalid['windows']:
    for s in w: s['vy']=0
assert evaluate(invalid)[0] is None, 'Fixed craft must not produce a flight calibration'
invalid=copy.deepcopy(run)
for s in invalid['windows'][2]: s['vy']+=10*(s['time']-4)
assert evaluate(invalid)[0] is None, 'Conflicting repetitions must fail'
print('PASS measured vertical identification, uncertainty bounds, no extrapolated thrust approval; fixed/filtered/stale/mixed/noisy trials rejected.')
