"""Regression for observed small-angle dead zone and sensitive vertical response."""
import math
import test_home as h
import test_wired as t

lua=t.LuaRuntime(unpack_returned_tuples=True)
mod=lua.execute(h.SOURCE,'--test')
matrix=mod.matrix(lua.table_from([lua.table_from([.0000026822,-.0741644]),
                                lua.table_from([-.0755996,-.00401651])]))
for sign in (-1,1):
    state=lua.table_from(dict(i1=0,i2=0,u=0,v=0))
    outputs=[mod.command(matrix,sign*math.radians(.9),sign*math.radians(.9),0,0,
                         21.6/22.1,state,.2) for _ in range(25)]
    assert any(u or v for u,v in outputs),outputs
    assert all(abs(u)<=2 and abs(v)<=2 for u,v in outputs)
print('PASS: measured response matrix + ~1.27deg static tilt produces correction within 5 seconds.')

records={}
t.simulate(t.setup(),duration=15,records=records)
arm=[t.key(1,'j'),t.key(1.1,'e')]+[t.key(1.3+i*.02,'r') for i in range(21)]
arm += [t.key(1.8+i*.02,'t') for i in range(6)]
actions=arm+[t.key(2.5,'h'),(5,'lift',16),t.key(10,'v'),(15,'fall',-.25)]
for gain,latency in ((1,.05),(3,.05),(8,.05),(15,.05),(3,.1),(15,.1)):
    hist,commands,g=h.sim(actions,records=records.copy(),duration=70,hover=3.24,
                          vertical_gain=gain,latency=latency)
    tail=[s for s in hist if 45<s['time']<69]
    end=t.state_at(hist,69)
    assert 'ARMED HOLD ALT' in end['lines'],(gain,end)
    amplitude=max(s['y'] for s in tail)-min(s['y'] for s in tail)
    assert amplitude<.5,(gain,amplitude,end)
    assert abs(end['y']-66)<.5,(gain,end)
    # Check per-engine automatic force slope, after takeover/power-up transients.
    writes=[c for c in commands if 11<c[0]<69 and c[1]==t.NAMES[0] and c[2]=='setThrustOutput']
    assert all(0<=c[3]<=733000 for c in writes)
    print('PASS: vertical gain',gain,'API delay',latency,'late height peak-to-peak',round(amplitude,3),flush=True)

a=lua.table_from(dict(hover=3.24,target=66,floor=63,integral=0,descending=False))
last=3.24
for i in range(100):
    out=mod.heightStep(a,65 if i%2 else 67,1 if i%2 else -1,.2,0,3.315)
    assert abs(out-last)<=.3*.2+1e-9
    assert 2.64-1e-9<=out<=3.84+1e-9
    last=out
print('PASS: automatic thrust stays within hover +/-4 percentage points and 2 percentage points/second.')
