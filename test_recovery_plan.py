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
profile=dict(version=1,craftId='simulation',mass=100,gravityMin=9,gravityMax=10,
             accelPerThrottleMin=28,accelPerThrottleMax=32,maxThrottle=.9,
             hoverThrottle=.32,ignitionDelay=.4,margin=8,rcsVerified=True,verticalVerified=True)
def sample(now=0,y=100,vy=0,**kw):
    data=dict(time=now,x=0,y=y,z=0,vx=0,vy=vy,vz=0,tilt=0,angularSpeed=0,
              mass=100,craftId='simulation',allNineReady=True)
    data.update(kw); return table(**data)
pad=table(x=0,y=0,z=0)
for key,val in [('rcsVerified',False),('verticalVerified',False),('mass',float('nan')),
                ('accelPerThrottleMin',1),('ignitionDelay',-1),('margin',0)]:
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

def start():
    state,why=m.start(table(**profile),pad,sample(),0)
    assert state is not None,why
    return state
state=start()
for n in range(1,33): out=m.step(state,sample(n*.1),n*.1)
assert out.phase=='COAST' and out.mainThrottle==0
assert out.poweredPositionHold==False and out.shadowOnly==True
# A large lateral deviation lights early rather than trying to coast sideways.
out=m.step(state,sample(3.3,x=3),3.3)
assert out.phase=='BRAKE' and state.ignited
out=m.step(state,sample(3.4,y=120),3.4)
assert out.phase=='BRAKE'  # ignition is latched, no repeated cutoff
out=m.step(state,sample(3.5,allNineReady=False),3.5)
assert out.phase=='FAULT' and out.mainThrottle is None
out=m.step(state,sample(3.6),3.6)
assert out.phase=='FAULT'

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
        elif out.phase=='ALIGN': throttle=gravity/gain
        lowest=min(lowest,y)
        if out.phase=='HOVER': break
    assert 'COAST' in phases and 'BRAKE' in phases and phases[-1]=='HOVER',(gravity,gain,y,vy)
    assert lowest>18,(gravity,gain,lowest)
    print('PASS synthetic plant',gravity,gain,'minimum Y',round(lowest,2),'hover t',round(now,1))
print('PASS: profile/identity/height/link gates, delay reserve, early ignition, latch and fault semantics.')

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
