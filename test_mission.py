"""Coupled planar rigid body across return burn and descent stage boundaries."""
from pathlib import Path
from types import SimpleNamespace
from collections import deque
import math
import test_allocator as a
lua=a.lua
def load(n,*args): return lua.execute(Path(__file__).with_name(n).read_text(encoding='utf-8'),*args)
deps=lua.table_from(dict(planner=load('recovery_plan.lua'),allocator=a.allocator,geometry=a.geo,
    control=load('recovery_control.lua'),boostback=load('boostback_plan.lua'),math=load('home.lua','--test')))
missionmod=load('mission_control.lua')
p=dict(version=2,craftId='test',mass=100,gravityMin=9,gravityMax=10,
    accelPerThrottleMin=35,accelPerThrottleMax=37,maxThrottle=.9,hoverThrottle=.27,
    ignitionDelay=.4,margin=8,rcsVerified=True,verticalVerified=True,coupledRcsVerified=True,
    lateralAccelMin=1,lateralResponseDelay=.2,boostbackVerified=True,boostbackAccel=1.5)
craft=a.tab(dict(id='test',profile=a.p,main=a.main,top=a.top,basis=dict(up=a.vec(0,1,0),a=a.vec(1,0,0),b=a.vec(0,0,-1))))
for delay in (0,2,4):
    x,y,vx,vy,angle,rate=20.,150.,1.,0.,0.,0.
    def sample(now):
        return a.tab(dict(time=now,x=x,y=y,z=0,vx=vx,vy=vy,vz=0,mass=100,craftId='test',
            allNineReady=True,angularSpeed=abs(rate),tilt=abs(angle),w=a.vec(0,0,rate),
            q=dict(x=0,y=0,z=math.sin(angle/2),w=math.cos(angle/2))))
    mission,why=missionmod.start(deps,a.tab(p),a.tab(a.vec()),sample(0),0)
    assert mission is not None,why
    lim=a.tab(a.limits); pending=deque(); stages=set(); qx=qy=0.
    applied=SimpleNamespace(mainX=0,mainY=0,mainKN=0,topKN={i:0 for i in range(1,5)})
    last=applied; recent=deque(maxlen=8)
    for step in range(1,1601):
        now=step*.1
        out=missionmod.step(deps,mission,sample(now),now,craft,lim)
        assert not out.manualTakeover,(delay,step,mission.error,x,y,vx,vy,angle,rate,list(recent))
        if out.plan: stages.add(out.plan.phase)
        if out.handoffHover: break
        if out.allocation:
            last=out.allocation
            if out.plan.phase=='COAST': assert last.mainKN==0
            recent.append((step,out.plan.phase,round(last.mainKN,2),last.mainX,round(rate,3),[round(last.topKN[j],1) for j in range(1,5)]))
            lim.previous=a.tab(dict(x=last.mainX,y=last.mainY))
        else: assert out.retainPreviousCommand
        pending.append(last)
        if len(pending)>delay: applied=pending.popleft()
        qx+=(applied.mainX-qx)*(1-.8**2); qy+=(applied.mainY-qy)*(1-.8**2)
        acc=a.actual(SimpleNamespace(mainX=qx,mainY=qy,mainKN=applied.mainKN,topKN=applied.topKN))
        co,si=math.cos(angle),math.sin(angle)
        vx+=(co*acc[0]-si*acc[1])*.1; vy+=(si*acc[0]+co*acc[1]-9.5)*.1
        rate+=acc[5]*.1; angle+=rate*.1; x+=vx*.1; y+=vy*.1
        assert y>18 and abs(x)<64,(delay,step,x,y)
    else: raise AssertionError(('No hover',delay,x,y,vx,vy,stages))
    assert {'TURN','BOOSTBACK','HANDOFF','BRAKE','HOVER'}<=stages,stages
    assert abs(x)<1 and abs(y-20)<.75 and abs(vx)<.15 and abs(vy)<.2
    print('PASS return-to-hover coupled mission delay',delay*.1,'duration',round(now,1),'stages',sorted(stages))
