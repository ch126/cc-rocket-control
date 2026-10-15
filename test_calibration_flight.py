"""Commissioning in an independent rigid-body plant. No game/output access."""
from pathlib import Path
from collections import deque
import math
import test_allocator as a

lua=a.lua
def load(name,*args): return lua.execute(Path(__file__).with_name(name).read_text(encoding='utf-8'),*args)
controller=load('calibration_flight.lua')
deps=a.tab(dict(math=load('home.lua','--test'),geometry=a.geo,allocator=a.allocator,evidence=load('calibration_evidence.lua')))

def add(x,y): return [u+v for u,v in zip(x,y)]
def scale(x,k): return [v*k for v in x]
def dot(x,y): return sum(u*v for u,v in zip(x,y))
def cross(x,y): return [x[1]*y[2]-x[2]*y[1],x[2]*y[0]-x[0]*y[2],x[0]*y[1]-x[1]*y[0]]
def norm(x): return math.sqrt(dot(x,x))
def product(q,r): return add(add(scale(r[:3],q[3]),scale(q[:3],r[3])),cross(q[:3],r[:3]))+[q[3]*r[3]-dot(q[:3],r[:3])]
def rotate(q,v): return add(v,add(scale(cross(q[:3],v),2*q[3]),scale(cross(q[:3],cross(q[:3],v)),2)))

def inverse(m):
    det=dot(m[0],cross(m[1],m[2]))
    columns=[scale(cross(m[1],m[2]),1/det),scale(cross(m[2],m[0]),1/det),scale(cross(m[0],m[1]),1/det)]
    return [[columns[j][i] for j in range(3)] for i in range(3)]
def mv(m,v): return [dot(row,v) for row in m]

def run(delay=0,blocked=False,reverse_top=False,asymmetric=False,period=.1):
    inertia=[[3300.,0,0],[0,200.,0],[0,0,3300.]]
    geometry=dict(a.p); main=a.main; top=a.top
    if asymmetric:
        # Observed-size layout/inertia, but synthetic ID and synthetic physics.
        # No private world coordinates or genuine flight measurements included.
        inertia=[[3289.5654,3.39839,-.497326],[3.39839,204.48424,3.3984],[-.497326,3.3984,3336.24509]]
        geometry['mass']=93.5
        main=[dict(center=a.vec(x+.0053476,-8.20320856,z+.0053476),facing='up') for x,z in [(-1,-1),(-1,1),(1,1),(1,-1),(0,0)]]
        top=[dict(center=a.vec(x+.0053476,6.79679144,z+.0053476),facing=f) for x,z,f in [(0,2,'north'),(2,0,'west'),(0,-2,'south'),(-2,0,'east')]]
    inv=inverse(inertia); geometry['inverseInertia']=inv; mass=geometry['mass']
    craft=a.tab(dict(id='commission-copy',profile=geometry,main=main,top=top,
        basis=dict(up=a.vec(0,1,0),a=a.vec(1,0,0),b=a.vec(0,0,-1))))
    position=[0.,0.,0.]; velocity=[0.,0.,0.]; q=[0.,0.,0.,1.]; omega=[0.,0.,0.]
    pending=deque(); applied=[(0.,0.,0.)]*9; nozzle=[0.,0.]
    def sample(now):
        up=rotate(q,[0,1,0])
        return a.tab(dict(time=now,x=position[0],y=position[1],z=position[2],vx=velocity[0],vy=velocity[1],vz=velocity[2],
            mass=mass,craftId='commission-copy',allNineReady=True,tilt=math.acos(max(-1,min(1,up[1]))),
            angularSpeed=norm(omega),w=a.vec(*omega),q=dict(zip(('x','y','z','w'),q)),
            applied=[dict(kn=c[0],vectorX=c[1],vectorY=c[2],ackTime=now) for c in applied]))
    state=controller.start(craft,sample(0),0)
    stages=[]; completed=None
    for tick in range(1,6201):
        now=tick*period; out=controller.step(deps,state,sample(now),now)
        if not stages or stages[-1]!=state.phase:
            stages.append(state.phase)
            print(round(now,1),state.phase,'h',round(position[1],2),'vy',round(velocity[1],2),'trial',state.trial)
        if out.manualTakeover:
            if blocked or reverse_top:
                assert out.commands is None
                return str(out.error)
            raise AssertionError((tick,out.error,position,velocity,omega))
        if out.result:
            completed=completed or now
            assert len(out.result.trials)==13
            assert not out.result.flightCalibrationValid and out.result.requiresEnvelopeReview
            assert out.result.vertical.gravityMin<9.5<out.result.vertical.gravityMax
            assert out.result.vertical.accelPerThrottleMin<3665/mass<out.result.vertical.accelPerThrottleMax
            if now-completed>10:
                assert abs(position[1]-12)<.3 and norm(velocity)<.1
                print('PASS full commissioning simulation',delay,'asymmetric',asymmetric,round(now,1))
                return out.result
        pending.append([(out.commands[i].kn,out.commands[i].vectorX,out.commands[i].vectorY) for i in range(1,10)])
        if len(pending)>delay: applied=pending.popleft()
        for substep in range(5):
            dt=period/5
            for axis in range(2): nozzle[axis]+=(applied[0][axis+1]-nozzle[axis])*(1-.8**(dt/.05))
            direction=[nozzle[0]*math.tan(math.pi/6),1,nozzle[1]*math.tan(math.pi/6)]
            direction=scale(direction,1/norm(direction))
            force=[0.,0.,0.]; torque=[0.,0.,0.]
            for command,engine in zip(applied[:5],main):
                f=scale(direction,command[0]); force=add(force,f)
                torque=add(torque,cross([engine['center'][k] for k in ('x','y','z')],f))
            for command,engine,d in zip(applied[5:],top,
                                       ([0,0,-1],[-1,0,0],[0,0,1],[1,0,0])):
                lever=[engine['center'][k] for k in ('x','y','z')]
                f=scale(d,command[0]*(-1 if reverse_top else 1)); force=add(force,f); torque=add(torque,cross(lever,f))
            accel=add(scale(rotate(q,force),1/mass),[0,-9.5,0])
            velocity=add(velocity,scale(accel,dt)); position=add(position,scale(velocity,dt))
            if position[1]<0 or blocked: position[1]=0; velocity[1]=0
            gyro=cross(omega,mv(inertia,omega))
            omega=add(omega,scale(mv(inv,add(torque,scale(gyro,-1))),dt))
            angle=norm(omega)*dt
            dq=scale(omega,math.sin(angle/2)/norm(omega)) if angle>1e-12 else [0,0,0]
            q=product(q,dq+[math.cos(angle/2)]); q=scale(q,1/norm(q))
    raise AssertionError(('Incomplete commissioning',stages))

if __name__=='__main__':
    run()
    run(2)
    run(2,asymmetric=True)
    run(1,asymmetric=True,period=.15)
    assert 'No liftoff' in run(blocked=True)
    assert run(reverse_top=True)
