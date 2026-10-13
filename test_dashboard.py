"""Dashboard hit boxes, explicit navigation references and bounded disk writes."""
import csv
import io
import math
import test_home as h
import test_wired as t

lua=t.LuaRuntime(unpack_returned_tuples=True)
m=lua.execute(h.SOURCE,'--test')
point=lambda x,y,z:lua.table_from(dict(x=x,y=y,z=z))
assert m.padDistance(point(-10,100,20),point(-7,-100,24))==5
assert m.padDistance(point(0,0,0),point(64,1000,0))==64
assert m.padDistance(point(0,0,0),point(50,0,50))>64

# Actual native-terminal render path, then mouse click the same actions as keys.
hist,cmd,g=h.sim([(1,'click',3,11),(1.2,'click',34,12),
                 (1.4,'click',23,12),(1.8,'click',5,15)],gui=True,duration=3)
assert all(abs(p-8063)<1e-6 for p in t.state_at(hist,1.6)['powers'])
assert all(p==0 for p in t.state_at(hist,2.8)['powers'])
assert 'ROCKET v0.5.4' in h.txt(hist)
assert g.outcome[1],g.outcome[2]
for size in ((70,25),(26,10)):
    hist,cmd,g=h.sim(gui=True,screen_size=size,duration=2)
    assert g.outcome[1],g.outcome[2]

records={}
t.simulate(t.setup(),duration=15,records=records)
arm=[t.key(1,'j'),t.key(1.1,'e')]+[t.key(1.3+i*.02,'r') for i in range(33)]+[t.key(2.5,'h')]

for distance,allowed in ((10,True),(64,True),(65,False)):
    disk=records.copy()
    actions=arm+[(4,'lift',16),(4,'position',distance,0),
                 (8,'click',23,11),(10,'click',34,11)]
    hist,cmd,g=h.sim(actions,records=disk,gui=True,duration=12,hover=4.95)
    text=h.txt(hist)
    assert ('ALT RETURN' in text)==allowed,(distance,text[-3000:])
    assert 'PAD dist' in text and 'HOVER target' in text
    if not allowed: assert 'PAD distance 65.00 >64' in text
    content=next(s for p,s in disk.items() if p.startswith('/rocket-flight-'))
    rows=list(csv.DictReader(io.StringIO(content)))
    assert all(None not in r for r in rows)
    padrow=next(r for r in rows if r['event']=='PAD_SET')
    assert float(padrow['pad_x'])==0 and float(padrow['pad_y'])==50
    assert any(r['event']=='ACTION' for r in rows)
print('PASS: horizontal pad distance, real GUI bounds/clicks, V/B/stop, 64-block boundary and pad CSV.')

# Existing files preserved, enough reserve for configuration and software updates.
for capacity in (1000,80000,1000000):
    disk=records.copy(); disk['/old-log.csv']='DO NOT DELETE'
    hist,cmd,g=h.sim(arm,records=disk,duration=180,disk_capacity=capacity)
    assert disk['/old-log.csv']=='DO NOT DELETE'
    assert 'ARMED HOLD' in t.state_at(hist,79)['lines']
    for name,content in disk.items():
        if name.startswith('/rocket-flight-'): assert len(content)<=131072
    assert 'log FULL' in h.txt(hist),(capacity,h.txt(hist)[-3000:])
    if capacity>=80000: assert sum(map(len,disk.values()))<=capacity-65536
print('PASS: low-space startup, free-space reserve, per-flight cap; no deletion or flight shutdown.')
