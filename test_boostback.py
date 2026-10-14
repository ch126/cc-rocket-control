"""Return-burn planner tests; ideal attitude tracking is explicitly assumed here."""
from pathlib import Path
import math
import test_wired as t
lua=t.LuaRuntime(unpack_returned_tuples=True)
def load(n): return lua.execute(Path(__file__).with_name(n).read_text(encoding='utf-8'))
r=load('recovery_plan.lua'); b=load('boostback_plan.lua')
def tab(v): return lua.table_from(v,recursive=True)
p=dict(version=2,craftId='test',mass=100,gravityMin=9,gravityMax=10,
    accelPerThrottleMin=35,accelPerThrottleMax=37,maxThrottle=.9,hoverThrottle=.27,
    ignitionDelay=.4,margin=8,rcsVerified=True,verticalVerified=True,coupledRcsVerified=True,
    lateralAccelMin=1, lateralResponseDelay=.2,boostbackVerified=True,boostbackAccel=3)
pad=tab(dict(x=0,y=0,z=0))
def sample(now=0,x=20,y=150,vx=5,vy=0,angle=0):
    return tab(dict(time=now,x=x,y=y,z=0,vx=vx,vy=vy,vz=0,mass=100,craftId='test',
        allNineReady=True,angularSpeed=0,q=dict(x=0,y=0,z=math.sin(angle/2),w=math.cos(angle/2)),tilt=abs(angle)))
for changes in ({'boostbackVerified':False},{'boostbackAccel':float('nan')},{'mass':99}):
    pp=p|changes
    # 1% mass change is allowed, use 10% for the mismatch fixture.
    if 'mass' in changes: pp['mass']=90
    state,why=b.start(r,tab(pp),pad,sample(),0)
    assert state is None and why
for bad in (sample(y=40),sample(x=65),sample(angle=math.radians(31))):
    state,why=b.start(r,tab(p),pad,bad,0); assert state is None and why
for startX,startVX in ((20,5),(-20,-5),(10,0)):
    x,y,vx,vy,angle=startX,150.,startVX,0.,0.
    state,why=b.start(r,tab(p),pad,sample(x=x,vx=vx),0); assert state is not None,why
    phases=set()
    for i in range(1,451):
        now=i*.1; s=sample(now,x,y,vx,vy,angle)
        out=b.step(r,state,s,now); phases.add(out.phase)
        assert out.phase!='FAULT',(out.reason,x,y,vx,vy)
        if out.phase=='HANDOFF':
            assert out.recoveryState.phase=='COAST' and out.retainPreviousCommand
            assert out.mainThrottle is None
            # Unlike manual recovery activation, this accepts intentional return velocity.
            nxt=r.step(out.recoveryState,sample(now+.1,x,y,vx,vy,angle),now+.1)
            assert nxt.phase in ('COAST','BRAKE')
            break
        angle=-math.atan2(out.targetUp.x,out.targetUp.y)
        ay=36.65*out.mainThrottle*math.cos(angle)-9.5
        ax=-36.65*out.mainThrottle*math.sin(angle)
        vx+=ax*.1; vy+=ay*.1; x+=vx*.1; y+=vy*.1
    else: raise AssertionError(('No handoff',x,y,vx,vy))
    assert 'TURN' in phases and 'BOOSTBACK' in phases
    print('PASS boostback from',startX,startVX,'handoff',round(now,1),'x/vx',round(x,2),round(vx,2))
state,_=b.start(r,tab(p),pad,sample(),0)
out=b.step(r,state,sample(.1,y=40,vy=-5),.1)
assert out.phase=='HANDOFF'  # already upright: immediately hand off to braking planner
nxt=r.step(out.recoveryState,sample(.2,y=39,vy=-5),.2)
assert nxt.phase=='BRAKE' and nxt.mainThrottle>0
state,_=b.start(r,tab(p),pad,sample(),0)
out=b.step(r,state,sample(),1)
assert out.phase=='FAULT' and out.mainThrottle is None
print('PASS: verified-profile gates, return velocity handoff, insufficient-height braking, stale telemetry, no stage-boundary cutoff.')
