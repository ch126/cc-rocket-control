"""Timing tolerance and CSV diagnostics without relaxing real-stall protection."""
import csv
import io
import test_home as h
import test_wired as t

records={}
t.simulate(t.setup(),duration=15,records=records)
arm=[t.key(1,'e')]+[t.key(1.3+i*.02,'r') for i in range(33)]+[t.key(2.5,'h')]
disk=records.copy()
hist,commands,g=h.sim(arm+[(3.5,'slow_sensor',.3),(3.9,'slow_sensor',.05)],records=disk,duration=8)
end=t.state_at(hist,7.8)
assert 'ARMED HOLD' in end['lines'],end
assert all(p>0 for p in end['powers'])
logs=[(p,s) for p,s in disk.items() if p.startswith('/rocket-flight-')]
assert len(logs)==1
rows=list(csv.DictReader(io.StringIO(logs[0][1])))
assert any(r['event']=='LATE_SAMPLE' for r in rows),[(r['event'],r['dt_s']) for r in rows]
assert any(r['event']=='EXIT' for r in rows)
assert all(None not in r for r in rows)
assert all(r['engine4_gate_command']!='' for r in rows)
assert disk[t.PATH]==records[t.PATH]
old_log=logs[0]
h.sim(records=disk,duration=2)
assert disk[old_log[0]]==old_log[1]
assert len([p for p in disk if p.startswith('/rocket-flight-')])==2
print('PASS: isolated 0.4s gap keeps HOLD/thrust/calibration; CSV fields, exit flush and non-overwrite.')

disk={}
hist,commands,g=h.sim(t.setup()+[(18,'lift',8),t.key(24,'v'),
    (27,'slow_sensor',.3),(27.4,'slow_sensor',.05)],records=disk,duration=31,hover=4.95)
assert 'ARMED HOLD ALT' in t.state_at(hist,30)['lines']
rows=list(csv.DictReader(io.StringIO(next(s for p,s in disk.items() if p.startswith('/rocket-flight-')))))
late=next(r for r in rows if r['event']=='LATE_SAMPLE')
after=next(r for r in rows if r['event']=='SAMPLE' and r['time_s']==late['time_s'])
assert after['power_pct']==late['power_pct'] and after['target_y']==late['target_y']
print('PASS: isolated late altitude sample freezes thrust and target rather than integrating the gap.')

for delay in (.3,1):
    disk=records.copy()
    hist,commands,g=h.sim(arm+[(3.5,'slow_sensor',delay)],records=disk,duration=8)
    end=t.state_at(hist,7.8)
    assert all(p==0 for p in end['powers']),end
    assert 'calibration OK' in end['lines'],end
    assert disk[t.PATH]==records[t.PATH]
    assert ('Repeated lag' if delay==.3 else 'stale') in h.txt(hist)
print('PASS: repeated late samples and real sensor stall stop, without erasing valid calibration.')

hist,commands,g=h.sim(arm,records=records.copy(),duration=6,log_failure=True)
assert 'ARMED HOLD' in t.state_at(hist,5.8)['lines']
assert 'log ERR' in h.txt(hist)
hist,commands,g=h.sim(arm+[(3.5,'mass')],records=records.copy(),duration=6)
assert 'calibration needed' in t.state_at(hist,5.8)['lines']
hist,commands,g=h.sim(t.setup()+[(4,'slow_sensor',.3)],duration=7)
assert 'Calibration interrupted by lag' in h.txt(hist)
assert all(p==0 for p in t.state_at(hist,6.8)['powers'])
print('PASS: logging failure cannot stop flight; mass changes invalidate, lag interrupts calibration.')
