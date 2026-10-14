"""End-to-end planner/allocation contract (not actual engine I/O or flight)."""
from pathlib import Path
import test_allocator as a
lua=a.lua
def load(n,*args): return lua.execute(Path(__file__).with_name(n).read_text(encoding='utf-8'),*args)
planner=load('recovery_plan.lua'); controller=load('recovery_control.lua')
mathlib=load('home.lua','--test')
deps=lua.table_from(dict(planner=planner,allocator=a.allocator,geometry=a.geo,math=mathlib))
profile=dict(version=2,craftId='test',mass=100,gravityMin=9,gravityMax=10,
    accelPerThrottleMin=35,accelPerThrottleMax=37,maxThrottle=.9,hoverThrottle=.27,
    ignitionDelay=.4,margin=8,rcsVerified=True,verticalVerified=True,coupledRcsVerified=True,
    lateralAccelMin=2,lateralResponseDelay=.2)
craft=a.tab(dict(id='test',profile=a.p,main=a.main,top=a.top,basis=dict(up=a.vec(0,1,0),a=a.vec(1,0,0),b=a.vec(0,0,-1))))
def sample(now=0,x=0):
    return a.tab(dict(time=now,x=x,y=100,z=0,vx=0,vy=0,vz=0,tilt=0,angularSpeed=0,
        mass=100,craftId='test',allNineReady=True,q=dict(x=0,y=0,z=0,w=1),w=a.vec()))
def start(x):
    state,why=planner.start(a.tab(profile),a.tab(a.vec()),sample(x=x),0)
    assert state is not None,why
    return state
state=start(0)
out=controller.step(deps,state,sample(.1),.1,craft,a.tab(a.limits))
assert out.plan.phase=='COAST' and len(out.commands)==9
assert all(out.commands[i].kn==0 for i in range(1,10))
state=start(2)
out=controller.step(deps,state,sample(.1,x=2),.1,craft,a.tab(a.limits))
assert out.plan.phase=='BRAKE' and out.plan.ignitionReason=='CONTROL_AUTHORITY'
assert out.commands[1].kn>0 and out.commands[1].kn<=733
assert len(out.commands)==9 and out.shadowOnly and not out.flightValidated
for i in range(6,10): assert 0<=out.commands[i].kn<=100
out=controller.step(deps,state,sample(.2,x=2),.2,craft,a.tab(a.limits))
assert out.plan.phase=='BRAKE'
bad=sample(.3); bad.w.x=float('nan')
out=controller.step(deps,state,bad,.3,craft,a.tab(a.limits))
assert out.commands is None and out.manualTakeover and state.phase=='FAULT'
print('PASS: shadow nine commands, centered coast, coupled-authority early ignition, burn latch and invalid-rate takeover.')

# Integrate an independent, coupled planar rigid body, not an ideal sideways actuator.
import math
from types import SimpleNamespace
from collections import deque
for delay,lag in [(0,False),(2,True),(4,True)]:
    state=start(1)
    x,y,vx,vy,angle,rate=1.,100.,0.,0.,0.,0.
    lim=a.tab(a.limits); pending=deque(); qx=qy=0
    zero=SimpleNamespace(mainX=0,mainY=0,mainKN=0,topKN={i:0 for i in range(1,5)})
    applied=zero
    for step in range(1,801):
        now=step*.1
        s=sample(now,x); s.y=y; s.vx=vx; s.vy=vy
        s.q.z=math.sin(angle/2); s.q.w=math.cos(angle/2); s.w.z=rate
        out=controller.step(deps,state,s,now,craft,lim)
        assert not out.manualTakeover,(delay,step,out.error,out.plan and out.plan.reason,x,y,vx,vy,angle)
        if out.handoffHover: break
        pending.append(out.allocation)
        if len(pending)>delay: applied=pending.popleft()
        alpha=1-.8**2 if lag else 1
        qx+=(applied.mainX-qx)*alpha; qy+=(applied.mainY-qy)*alpha
        physical=SimpleNamespace(mainX=qx,mainY=qy,mainKN=applied.mainKN,topKN=applied.topKN)
        acc=a.actual(physical)
        co,si=math.cos(angle),math.sin(angle)
        vx+=(co*acc[0]-si*acc[1])*.1
        vy+=(si*acc[0]+co*acc[1]-9.5)*.1
        rate+=acc[5]*.1; angle+=rate*.1; x+=vx*.1; y+=vy*.1
        lim.previous=a.tab(dict(x=out.allocation.mainX,y=out.allocation.mainY))
        assert y>18 and abs(angle)<math.radians(10),(delay,step,x,y,angle)
    else: raise AssertionError(('No hover handoff',delay,x,y,vx,vy,angle))
    assert abs(x)<1 and abs(y-20)<.75 and abs(vx)<.15 and abs(vy)<.2
    print('PASS: coupled planar descent, delay',delay*.1,'nozzle lag',lag,'hover at',round(now,1),'s.')
