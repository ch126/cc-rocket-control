"""Independent analytical identities. No claim of in-game closed-loop validation."""
from pathlib import Path
import math
import test_wired as t
lua=t.LuaRuntime(unpack_returned_tuples=True)
model=lua.execute(Path(__file__).with_name('geometry_model.lua').read_text(encoding='utf-8'))
def tab(v): return lua.table_from(v,recursive=True)
def v(x,y,z): return dict(x=x,y=y,z=z)
p=tab(dict(mass=100,com=v(0,0,0),inverseInertia=[[.001,0,0],[0,.005,0],[0,0,.002]]))
for facing,expected in [('up',(0,1,0)),('down',(0,-1,0)),('east',(1,0,0)),('west',(-1,0,0)),('north',(0,0,-1)),('south',(0,0,1))]:
    d=model.direction(facing,0,0)
    assert tuple(d[k] for k in ('x','y','z'))==expected
d=model.direction('up',1/15,0)
assert d.x>0 and d.y>0 and d.z==0
d=model.direction('up',0,1/15)
assert d.z>0 and d.y>0 and d.x==0
e=tab(dict(center=v(0,10,0),facing='east'))
r=model.response(p,e,1,0,0)
assert abs(r.linear.x-.01)<1e-12 and abs(r.angular.z+.02)<1e-12
assert not r.flightValidated
engines=tab([dict(center=v(x,-8,z),facing='up') for x,z in [(-1,-1),(-1,1),(1,1),(1,-1),(0,0)]])
c=model.mainColumns(p,engines,200)
s=math.tan(math.pi/6)/15
assert abs(c.neutral.force.y-1000)<1e-10
assert abs(c.neutral.angular.x)+abs(c.neutral.angular.z)<1e-10
assert abs(c.x.z-8*1000*s/math.sqrt(1+s*s)*.002)<1e-10
assert abs(c.y.x+8*1000*s/math.sqrt(1+s*s)*.001)<1e-10
# Translating all points together cannot change a body response.
p2=tab(dict(mass=100,com=v(20000000,100,20000000),inverseInertia=[[.001,0,0],[0,.005,0],[0,0,.002]]))
r2=model.response(p2,tab(dict(center=v(20000000,110,20000000),facing='east')),1,0,0)
assert abs(r2.angular.z-r.angular.z)<1e-12
for bad in ([[1,0,0],[0,-1,0],[0,0,1]],[[1,2,0],[0,1,0],[0,0,1]]):
    try: model.response(tab(dict(mass=100,com=v(0,0,0),inverseInertia=bad)),e,1,0,0)
    except Exception: pass
    else: raise AssertionError('invalid inertia accepted')
print('PASS: six facings, both main gimbal signs, force scale, torque coupling, five-engine columns, plot translation, invalid inertia rejection.')
