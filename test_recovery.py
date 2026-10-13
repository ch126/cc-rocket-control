"""Actual controller: large-angle recovery, navigation cancellation and limits."""
import math
import test_home as h
import test_wired as t

records={}
t.simulate(t.setup(),duration=15,records=records)
arm=[t.key(1,'j'),t.key(1.1,'e')]+[t.key(1.3+i*.02,'r') for i in range(33)]
for degrees in (30,45,60):
    for direction in (1,-1):
        actions=arm+[(2,'kick',math.radians(degrees)*direction,0),t.key(2.5,'h')]
        hist,commands,g=h.sim(actions,records=records.copy(),duration=35)
        assert 'ARMED RECOVER' in h.txt(hist)
        end=t.state_at(hist,34)
        assert 'ARMED HOLD' in end['lines'],end
        assert math.hypot(end['x'],end['z'])<math.radians(6),end
        assert all(p>0 for s in hist if 3<s['time']<34 for p in s['powers'])
print('PASS: 30/45/60 degree recovery in both directions, continuous thrust, settled HOLD.')
for scale in (.7,1.3):
    actions=arm+[(2,'kick',.5,-.5),t.key(2.5,'h')]
    hist,commands,g=h.sim(actions,records=records.copy(),duration=35,
                          gain=.05*scale,latency=.1,nozzle_lag=.07)
    end=t.state_at(hist,34)
    assert 'ARMED HOLD' in end['lines'],end
    assert math.hypot(end['x'],end['z'])<math.radians(6),end
print('PASS: two-axis recovery with delayed APIs/nozzles and response uncertainty.')

# A real angular-rate disturbance, not an instantaneous impossible pose jump.
flight=arm+[t.key(2.5,'h'),(5,'lift',80),t.key(10,'v'),t.key(14,'b'),(15,'spin',.35,0)]
hist,commands,g=h.sim(flight,records=records.copy(),duration=48,hover=4.95,horizontal=True)
assert 'ALT RETURN' in h.txt(hist)
recovering=[s for s in hist if 'ARMED RECOVER' in s['lines']]
assert recovering
assert all(' ALT' not in s['lines'] and ' RETURN' not in s['lines'] for s in recovering)
end=t.state_at(hist,47)
assert 'ARMED HOLD' in end['lines'] and ' ALT' not in end['lines'],end
assert math.hypot(end['x'],end['z'])<math.radians(6),end
print('PASS: airborne navigation yields to recovery, no automatic stale-target resumption.')

for key in ('space','c','h','q'):
    hist,commands,g=h.sim(flight+[t.key(17,key)],records=records.copy(),duration=20,hover=4.95)
    if key in ('space','q'):
        assert all(e.gate==0 and e.output==0 for e in g.motors.values())
    else:
        assert 'ARMED MANUAL' in t.state_at(hist,19)['lines']

hist,commands,g=h.sim(arm+[(2,'kick',math.radians(75),0),t.key(2.5,'h')],records=records.copy(),duration=5)
assert 'ARMED RECOVER' not in h.txt(hist) # manual pilot must not get surprise thrust changes
# Once already controlling, an out-of-envelope event must still stop.
hist,commands,g=h.sim(arm+[t.key(2.5,'h'),(3,'kick',math.radians(75),0)],records=records.copy(),duration=5)
assert all(p==0 for p in t.state_at(hist,4.8)['powers'])
hist,commands,g=h.sim(arm+[(2,'kick',math.radians(45),0),t.key(2.5,'h')],
                     records=records.copy(),duration=35,blocked=True)
assert 'Recovery timeout >30s' in h.txt(hist)
assert all(p==0 for p in t.state_at(hist,34)['powers'])
print('PASS: manual cancellation, emergency stop, large-angle activation rejection and envelope stop.')
