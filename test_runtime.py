"""Persistent hover handoff and explicit live submission contract, synthetic plant."""
import math
from types import SimpleNamespace
from collections import deque
import test_mission as t
a=t.a; lua=t.lua; deps=t.deps
deps.mission=t.missionmod; deps.hover=t.load('hover_control.lua')
runtime=t.load('mission_runtime.lua')

for mode,entry in (('shadow','boostback'),('live','boostback'),('shadow','descent'),('live','descent')):
    driver=lua.eval('''function() return {ready=true,armed=true,caps={733,733,733,733,733,100,100,100,100},
      count=0,submit=function(self,c) assert(#c==9); self.count=self.count+1; self.last=c; return true end} end''')()
    x,y,vx,vy,angle,rate=20.,150.,1.,0.,0.,0.
    profile=dict(t.p)
    if entry=='descent':
        x,y,vx=1.,100.,0.
        profile.pop('boostbackVerified',None)
    def sample(now):
        return a.tab(dict(time=now,x=x,y=y,z=0,vx=vx,vy=vy,vz=0,mass=100,craftId='test',
            allNineReady=True,angularSpeed=abs(rate),tilt=abs(angle),w=a.vec(0,0,rate),
            q=dict(x=0,y=0,z=math.sin(angle/2),w=math.cos(angle/2))))
    state,why=runtime.start(deps,driver,a.tab(profile),a.tab(a.vec()),sample(0),0,t.craft,a.tab(a.limits),mode,entry)
    assert state is not None,why
    pending=deque(); qx=qy=0.; hoverAt=None; disturbed=False; phases=set()
    applied=SimpleNamespace(mainX=0,mainY=0,mainKN=0,topKN={i:0 for i in range(1,5)})
    for step in range(1,1801):
        now=step*.1
        out=runtime.step(deps,state,sample(now),now)
        assert not out.manualTakeover,(mode,step,out.error,x,y,angle,rate)
        assert len(out.commands)==9 and out.submitted==(mode=='live')
        if out.plan: phases.add(out.plan.phase)
        if state.phase=='HOVER':
            hoverAt=hoverAt or now
            if now-hoverAt>5 and not disturbed: x+=2; vy-=.3; disturbed=True
            if now-hoverAt>35: break
        c=out.commands
        pending.append(SimpleNamespace(mainX=c[1].vectorX*15,mainY=c[1].vectorY*15,mainKN=c[1].kn,
            topKN={i:c[i+5].kn for i in range(1,5)}))
        if len(pending)>2: applied=pending.popleft()
        qx+=(applied.mainX-qx)*(1-.8**2); qy+=(applied.mainY-qy)*(1-.8**2)
        acc=a.actual(SimpleNamespace(mainX=qx,mainY=qy,mainKN=applied.mainKN,topKN=applied.topKN))
        co,si=math.cos(angle),math.sin(angle)
        vx+=(co*acc[0]-si*acc[1])*.1; vy+=(si*acc[0]+co*acc[1]-9.5)*.1
        rate+=acc[5]*.1; angle+=rate*.1; x+=vx*.1; y+=vy*.1
        assert y>18 and abs(x)<64
    else: raise AssertionError(('No persistent hover',mode,x,y))
    assert disturbed and abs(x)<.5 and abs(y-20)<.5 and abs(vx)<.15 and abs(vy)<.15,(mode,x,y,vx,vy)
    if entry=='descent':
        assert 'BRAKE' in phases and 'HOVER' in phases,phases
        assert not phases.intersection({'TURN','BOOSTBACK','LEVEL'}),phases
    assert driver.count==(step if mode=='live' else 0)
    s=sample(now+.1); s.time=now-1
    before=driver.count
    out=runtime.step(deps,state,s,now+.1)
    assert out.manualTakeover and out.commands is None and driver.count==before
    print('PASS runtime',mode,entry,'continuous frames + 35s hover, disturbance recovery and stale-sample rejection.')
driver.armed=False
state,why=runtime.start(deps,driver,a.tab(t.p),a.tab(a.vec()),sample(0),0,t.craft,a.tab(a.limits),'live')
assert state is None and why
print('PASS: runtime never auto-arms an unarmed output backend.')

# Descent entry is not a ground-launch button or recovery from an arbitrary fall.
for height,vertical,tilt in ((0,0,0),(100,-4,0),(100,0,.2)):
    y,vy,angle=height,vertical,tilt
    state,why=runtime.start(deps,None,a.tab(profile),a.tab(a.vec()),sample(0),0,t.craft,a.tab(a.limits),'shadow','descent')
    assert state is None and why,(height,vertical,tilt)
state,why=runtime.start(deps,None,a.tab(profile),a.tab(a.vec()),sample(0),0,t.craft,a.tab(a.limits),'shadow','capture')
assert state is None and why=='Unknown mission entry'
print('PASS: descent rejects ground start, uncontrolled fall, excessive tilt and unsupported hardware stages.')
