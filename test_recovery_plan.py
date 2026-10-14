"""Synthetic SI-like plant: verifies planning math, NOT the user's game physics."""
from pathlib import Path
import math
import sys
sys.path.insert(0,str(Path(__file__).parent/'.test-deps'))
from lupa.lua52 import LuaRuntime
lua=LuaRuntime(unpack_returned_tuples=True)
source=Path(__file__).with_name('recovery_plan.lua').read_text(encoding='utf-8')
m=lua.execute(source)
def table(**kw): return lua.table_from(kw)
profile=dict(version=2,craftId='simulation',mass=100,gravityMin=9,gravityMax=10,
             accelPerThrottleMin=28,accelPerThrottleMax=32,maxThrottle=.9,
             hoverThrottle=.32,ignitionDelay=.4,margin=8,rcsVerified=True,verticalVerified=True,
             coupledRcsVerified=True,lateralAccelMin=2,lateralResponseDelay=.2)
def sample(now=0,y=100,vy=0,**kw):
    data=dict(time=now,x=0,y=y,z=0,vx=0,vy=vy,vz=0,tilt=0,angularSpeed=0,
              mass=100,craftId='simulation',allNineReady=True)
    data.update(kw); return table(**data)
pad=table(x=0,y=0,z=0)
for key,val in [('rcsVerified',False),('verticalVerified',False),('mass',float('nan')),
                ('accelPerThrottleMin',1),('ignitionDelay',-1),('margin',0),
                ('version',1),('coupledRcsVerified',False),('lateralAccelMin',0),
                ('lateralResponseDelay',-1)]:
    p=profile|{key:val}
    state,reason=m.start(table(**p),pad,sample(),0)
    assert state is None and reason
for overrides in [dict(y=30),dict(x=65),dict(craftId='changed'),dict(mass=103),
                  dict(allNineReady=False),dict(time=-1),dict(vy=-3)]:
    state,reason=m.start(table(**profile),pad,sample(**overrides),0)
    assert state is None and reason
last=0
for speed in (0,-1,-5,-10,-20):
    height,_=m.stoppingHeight(table(**profile),speed)
    assert height>last; last=height

def start(**kw):
    state,why=m.start(table(**profile),pad,sample(**kw),0)
    assert state is not None,why
    return state
state=start(x=1)
out=m.step(state,sample(.1,x=1),.1)
assert out.phase=='COAST' and out.mainThrottle==0
assert out.poweredPositionHold==False and out.shadowOnly==True
assert out.lateralAcceleration.x<0 and abs(out.lateralAcceleration.z)<1e-9
# More error/time required causes early ignition despite ample vertical height.
out=m.step(state,sample(.2,x=12),.2)
assert out.phase=='BRAKE' and state.ignited
assert out.ignitionReason=='LATERAL_CORRECTION_TIME' and out.mainThrottle>0
assert out.remainingHeight>out.brakeHeight
out=m.step(state,sample(.3,y=120),.3)
assert out.phase=='BRAKE'  # ignition is latched, no repeated cutoff
out=m.step(state,sample(.4,allNineReady=False),.4)
assert out.phase=='FAULT' and out.mainThrottle is None
out=m.step(state,sample(.5),.5)
assert out.phase=='FAULT'
state=start()
out=m.step(state,sample(.1,y=40,vy=-15),.1)
assert out.phase=='BRAKE' and out.ignitionReason=='VERTICAL_BRAKING_HEIGHT'
state.burnDeadline=.15
out=m.step(state,sample(.2,y=40,vy=-15),.2)
assert out.phase=='FAULT' and out.mainThrottle is None
state=start()
out=m.step(state,sample(.1,tilt=math.radians(6)),.1)
assert out.ignitionReason=='ATTITUDE_MARGIN'
state=start()
out=m.step(state,sample(-1),.1)
assert out.phase=='FAULT' and out.mainThrottle is None
state=start()
out=m.step(state,sample(.1,vx=float('nan')),.1)
assert out.phase=='FAULT'
for vx,vz in ((1,0),(0,-1),(.7,.7)):
    lp=m.lateralPlan(table(**profile),pad,sample(vx=vx,vz=vz),80)
    assert lp.ax*vx+lp.az*vz<0
    assert math.hypot(lp.ax,lp.az)<=profile['lateralAccelMin']+1e-9
    assert lp.predictedMiss>0

for gravity,gain in ((9,32),(9.5,30),(10,28)):
    state=start(); y=100.; vy=0.; throttle=gravity/gain; pending=[]; phases=[]
    lowest=100
    for n in range(1,2001):
        now=n*.05
        while pending and pending[0][0]<=now: _,throttle=pending.pop(0)
        vy+=(throttle*gain-gravity)*.05; y+=vy*.05
        out=m.step(state,sample(now,y,vy),now); phases.append(out.phase)
        assert out.phase!='FAULT',out.reason
        if out.mainThrottle is not None:
            pending.append((now+.4,float(out.mainThrottle)))
        lowest=min(lowest,y)
        if out.phase=='HOVER': break
    assert 'COAST' in phases and 'BRAKE' in phases and phases[-1]=='HOVER',(gravity,gain,y,vy)
    assert lowest>18,(gravity,gain,lowest)
    print('PASS synthetic plant',gravity,gain,'minimum Y',round(lowest,2),'hover t',round(now,1))
print('PASS: profile/identity/height/link gates, delay reserve, early ignition, latch and fault semantics.')

# Ideal, independent horizontal actuator for planning tests only. This does NOT
# validate the real side-engine torque/translation coupling or allocator.
for initial_x,initial_z,initial_vx in ((4,3,0),(0,0,.4),(12,0,0)):
    state=start(y=200,x=initial_x,z=initial_z,vx=initial_vx)
    x,z,vx,vz=initial_x,initial_z,initial_vx,0.
    y,vy,throttle=200.,0.,9.5/30
    ax=az=0.; main_queue=[]; side_queue=[]; coast_corrected=False
    for n in range(1,2401):
        now=n*.05
        while main_queue and main_queue[0][0]<=now: _,throttle=main_queue.pop(0)
        while side_queue and side_queue[0][0]<=now: _,ax,az=side_queue.pop(0)
        vy+=(throttle*30-9.5)*.05; y+=vy*.05
        vx+=ax*.05; vz+=az*.05; x+=vx*.05; z+=vz*.05
        out=m.step(state,sample(now,y,vy,x=x,z=z,vx=vx,vz=vz),now)
        assert out.phase!='FAULT',(out.reason,x,z,y)
        if out.phase=='COAST' and math.hypot(out.lateralAcceleration.x,out.lateralAcceleration.z)>1e-6:
            coast_corrected=True
        if out.mainThrottle is not None: main_queue.append((now+.4,float(out.mainThrottle)))
        side_queue.append((now+.2,float(out.lateralAcceleration.x),float(out.lateralAcceleration.z)))
        if out.phase=='HOVER': break
    assert out.phase=='HOVER',(initial_x,x,z,y,vy)
    assert math.hypot(x,z)<1 and abs(y-20)<.75
    assert coast_corrected
    print('PASS simultaneous coast correction',initial_x,initial_z,initial_vx,'final miss',round(math.hypot(x,z),3))

# The downloadable inventory program must not access any engine write method.
check=Path(__file__).with_name('recovery_check.lua').read_text(encoding='utf-8')
for missing in (None,6):
    lua.execute('''
      inventoryLines={}
      print=function(s) inventoryLines[#inventoryLines+1]=s end
      peripheral={hasType=function(name,kind) return not string.find(name,'_'..tostring(missingEngine)..'$') end,
        call=function(name,method)
          assert(method=='getMaxThrustOutputPn','UNEXPECTED ENGINE WRITE')
          return 733000
        end}
      sublevel={isInPlotGrid=function() return true end,getMass=function() return 72 end,
        getLogicalPose=function() return {position={y=100}} end}
    ''')
    lua.globals().missingEngine=missing
    lua.execute(check)
    lines=' '.join(lua.globals().inventoryLines.values())
    assert ('Nine links found' in lines)==(missing is None)
    assert 'No engine commands' in lines
print('PASS: nine-engine inventory is read-only; missing top engine is reported.')
