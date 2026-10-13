"""Return/position hold and damped attitude tests using the actual async controller."""
from pathlib import Path
import math
import test_wired as t

SOURCE=Path(__file__).with_name('home.lua').read_text(encoding='utf-8')
def sim(actions=(),**kw): return t.simulate(actions,source=SOURCE,**kw)
def txt(h): return '\n'.join(s['lines'] for s in h)
setup=t.setup()

if __name__=='__main__':
    lua=t.LuaRuntime(unpack_returned_tuples=True)
    mod=lua.execute(SOURCE,'--test')
    def table(**kw): return lua.table_from(kw)
    origin=table(x=0,y=0,z=0)
    target=table(x=30,y=0,z=0)
    slope=table(x=0,z=0)
    up,distance=mod.navigation(target,origin,origin,slope,.1)
    assert distance==30 and up.x>0 and abs(up.z)<1e-9
    assert math.acos(up.y)<=math.radians(1.5)*.1+1e-8
    # Guidance is in world coordinates: rotating the body must rotate the error,
    # not change the requested direction of travel.
    for yaw in (0,math.pi/2,math.pi):
        q=table(x=0,y=math.sin(yaw/2),z=0,w=math.cos(yaw/2))
        basis=mod.basis(q)
        e1,e2,angle=mod.error(q,basis,up)
        error=table(**{k:basis.a[k]*e1+basis.b[k]*e2 for k in ('x','y','z')})
        world=mod.rotate(q,error)
        assert abs(world.x)<1e-8 and world.z<0
    for _ in range(100):
        up,_=mod.navigation(target,origin,table(x=-10,y=0,z=10),slope,.1)
        assert math.acos(up.y)<=math.radians(4)+1e-8
    print('PASS: world-frame guidance, tilt limit, slew limit and rotated-body error.')
    records={}
    h,c,g=sim(setup+[(15,'kick',.06,-.04)],duration=38,records=records)
    assert t.PATH in records and 'ARMED HOLD' in txt(h),h[-1]
    assert math.hypot(t.state_at(h,37)['x'],t.state_at(h,37)['z'])<.04,t.state_at(h,37)
    hover=setup+[(18,'lift',16),t.key(26,'v')]
    h,c,g=sim(hover+[(28,'drift',.4,-.3)],duration=110,horizontal=True,hover=4.95)
    assert 'ALT POS' in txt(h),h[-1]
    assert all('Position/return aborted' not in s['lines'] for s in h)
    capture=t.state_at(h,26)
    end=t.state_at(h,109)
    assert math.hypot(end['px']-capture['px'],end['pz']-capture['pz'])<2,end
    assert math.hypot(end['vx'],end['vz'])<.3,end
    print('PASS: upright hold and horizontal disturbance recovery.')

    # Start 8 blocks from launch, capture hover, return without changing height.
    returning=setup+[(18,'lift',16),(18,'position',8,-6),t.key(28,'v'),t.key(45,'b')]
    h,c,g=sim(returning,duration=155,horizontal=True,hover=4.95)
    assert 'ALT RETURN' in txt(h),h[-1]
    end=t.state_at(h,154)
    assert math.hypot(end['px'],end['pz'])<1.5,end
    # Quantized nozzles do not promise exact stillness: check a sustained window,
    # not a lucky zero-velocity crossing at the last sample.
    settled=[s for s in h if 130<s['time']<154]
    assert max(math.hypot(s['px'],s['pz']) for s in settled)<1.5,end
    assert max(math.hypot(s['vx'],s['vz']) for s in settled)<.3,end
    assert 'ALT HOME' in txt(h),end
    assert abs(end['y']-66)<1,end
    print('PASS: returns above pad, brakes, holds position and retains height.')

    # Disarmed pad capture and saved calibration have independent lifetimes.
    h,c,g=sim([t.key(1,'j')],records=records.copy(),duration=2)
    assert 'Pad XYZ captured' in txt(h)
    h,c,g=sim(records=records.copy(),duration=2)
    assert 'Pad NOT SET' in txt(h)
    for action in ('r','f','t','g','c','h','v','space','b','n'):
        h,c,g=sim(returning+[t.key(47,action)],duration=50,horizontal=True,hover=4.95)
        end=t.state_at(h,49)
        assert ' RETURN' not in end['lines'],(action,end)
    for actions in (setup+[t.key(15,'b')],
                    setup+[(18,'lift',5),t.key(26,'v'),t.key(28,'b')],
                    setup+[(18,'lift',16),(18,'position',100,0),t.key(28,'v'),t.key(30,'b')]):
        h,c,g=sim(actions,duration=33,hover=4.95)
        assert 'ALT RETURN' not in txt(h)
    h,c,g=sim(returning+[(47,'drift',4,0)],duration=50,horizontal=True,hover=4.95)
    assert 'Position/return aborted; upright height hold.' in txt(h)
    assert 'ARMED HOLD ALT' in t.state_at(h,49)['lines']
    assert ' RETURN' not in t.state_at(h,49)['lines']
    print('PASS: pad capture, no stale pad on reload, all manual overrides, altitude/range/activation gates.')
