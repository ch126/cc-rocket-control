"""Short probe and active braking at the user's 21.6% thrust, without game access."""
import json
import math
import subprocess
import test_home as h
import test_wired as t

old=subprocess.check_output(['git','show','3971100:home.lua'],text=True,encoding='utf-8')
actions=[t.key(1,'u'),t.key(1.1,'e')]+[t.key(1.3+i*.02,'r') for i in range(21)]+[t.key(1.8+i*.03,'t') for i in range(6)]+[t.key(2.5,'k')]
for gain in (.05,.1,.2,.3):
    for latency in (.05,.1):
        records={}
        hist,cmd,g=h.sim(actions,duration=26,gain=gain,latency=latency,records=records)
        during=[s for s in hist if 'ARMED CALIBRATE' in s['lines']]
        peak=max(math.degrees(math.hypot(s['x'],s['z'])) for s in during)
        success=t.PATH in records
        print('gain',gain,'delay',latency,'peak',round(peak,2),'saved',success,flush=True)
        assert peak<12,(gain,latency,peak)
        if gain==.3 and not success:
            assert 'did not settle' in h.txt(hist)
            assert all(p==0 for p in t.state_at(hist,25)['powers'])
            continue
        assert success,(gain,latency,t.state_at(hist,25)['lines'])
        r=json.loads(records[t.PATH]); expected=3*(3.24/15)*gain
        for got in (r['matrix']['a'],r['matrix']['d']):
            assert abs(abs(got)/expected-1)<.35,(gain,latency,got,expected)
    hist,_,_=t.simulate(actions,source=old,duration=14,gain=gain)
    old_peak=max(math.degrees(math.hypot(s['x'],s['z'])) for s in hist if 'ARMED CALIBRATE' in s['lines'])
    print('old peak',round(old_peak,2),flush=True)

for reverse in (False,True):
    disk={}
    hist,_,_=h.sim(actions,duration=26,gain=.12,reverse=reverse,records=disk)
    assert t.PATH in disk
    r=json.loads(disk[t.PATH]); assert (r['matrix']['a']>0)==(not reverse)
for interrupt in ('space','c','q'):
    disk={}
    hist,_,g=h.sim(actions+[t.key(3.6,interrupt)],duration=8,records=disk)
    assert t.PATH not in disk
    if interrupt=='space': assert all(p==0 for p in t.state_at(hist,7)['powers'])
    assert all(e.output==0 for e in g.motors.values())
hist,_,_=h.sim(actions,duration=8,blocked=True,records={})
assert 'response too weak' in h.txt(hist)
print('PASS: response fit, swapped signs, weak/blocked rejection, cancel/stop and no partial save.')
