"""Allocator residuals checked against a separately coded rigid-body plant."""
from pathlib import Path
import math
import test_wired as t
lua=t.LuaRuntime(unpack_returned_tuples=True)
def load(n): return lua.execute(Path(__file__).with_name(n).read_text(encoding='utf-8'))
geo=load('geometry_model.lua'); allocator=load('control_allocator.lua')
def tab(v): return lua.table_from(v,recursive=True)
def vec(x=0,y=0,z=0): return dict(x=x,y=y,z=z)
main=[dict(center=vec(x,-8,z),facing='up') for x,z in [(-1,-1),(-1,1),(1,1),(1,-1),(0,0)]]
top=[dict(center=vec(0,7,2),facing='north'),dict(center=vec(2,7,0),facing='west'),
     dict(center=vec(0,7,-2),facing='south'),dict(center=vec(-2,7,0),facing='east')]
p=dict(mass=100,com=vec(),inverseInertia=[[1/3300,0,0],[0,1/200,0],[0,0,1/3300]])
limits=dict(mainKN=733,topKN=[100]*4,steps=3,slewSteps=1)
def solve(linear,angular=vec(),lim=None):
    return allocator.solve(geo,tab(p),tab(main),tab(top),tab(dict(linear=linear,angular=angular)),tab(lim or limits))
def actual(out):
    slope=math.tan(math.pi/6)
    dx,dz=out.mainX/15*slope,out.mainY/15*slope
    length=math.sqrt(1+dx*dx+dz*dz)
    fx,fy,fz=(5*out.mainKN*q/length for q in (dx,1,dz))
    # Main force at Y=-8; top at Y=+7. Symmetric horizontal lever torques cancel.
    tx,tz=-8*fz,8*fx
    sx=-out.topKN[2]+out.topKN[4]; sz=-out.topKN[1]+out.topKN[3]
    return [(fx+sx)/100,fy/100,(fz+sz)/100,(tx+7*sz)/3300,0,(tz-7*sx)/3300]

for ax,az in [(.3,0),(-.3,0),(0,.3),(0,-.3),(.2,-.2)]:
    out=solve(vec(ax,10,az))
    got=actual(out)
    assert all(abs(got[i]-out.predicted[i+1])<1e-9 for i in range(6))
    assert math.hypot(got[3],got[5])<.006,(ax,az,got)
    assert got[0]*ax+got[2]*az>0,(ax,az,got)
    assert abs(out.mainX)<=1 and abs(out.mainY)<=1
    assert all(0<=out.topKN[j]<=100 for j in range(1,5))
# Coast cannot independently translate and cancel the same RCS torque.
out=solve(vec(.3,0,0))
assert out.mainKN==0 and out.requiresPoweredSupport and out.authorityLimited
assert out.lateralError>.05
out=solve(vec(0,10,0),vec(.02,0,-.02))
got=actual(out)
assert got[3]>0 and got[5]<0
out=solve(vec(0,100,0))
assert out.mainKN==733 and out.verticalError>60 and out.authorityLimited
out=solve(vec(),lim=limits|{'topKN':[0]*4})
assert out.mainKN==0 and all(out.topKN[j]==0 for j in range(1,5))
assert out.shadowOnly and not out.flightValidated
for bad in ({'mainKN':float('nan')},{'topKN':[-1,1,1,1]},{'steps':4},{'slewSteps':2}):
    try: solve(vec(),lim=limits|bad)
    except Exception: pass
    else: raise AssertionError(bad)
print('PASS: coupled independent plant, four lateral directions, attitude correction, gimbal slew, thrust caps, insufficient coast authority and validation.')

# Sub-step requests must not disappear forever at quantized main nozzles.
lim=limits.copy(); sums=[0.]*6
for _ in range(200):
    out=solve(vec(.1,10,0),lim=lim)
    got=actual(out)
    for i in range(6): sums[i]+=got[i]
    lim['balance']=[out.balance[i] for i in range(1,7)]
    lim['previous']=dict(x=out.mainX,y=out.mainY)
assert abs(sums[0]/200-.1)<.01 and abs(sums[5]/200)<.001,sums
print('PASS: bounded acceleration-time residual realizes sub-step lateral demand on average.')
