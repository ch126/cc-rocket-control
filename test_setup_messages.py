"""Missing calibration is not a tilt fault, and logging cannot hide setup results."""
import json
import test_home as h
import test_wired as t

hist,_,g=h.sim([t.key(1,'l'),t.key(1.2,'i'),t.key(1.8,'j')],
               gui=True,disk_capacity=1000,duration=3)
before=t.state_at(hist,1.6)['lines']
assert 'no calibration file' in before and 'No saved calibration' in before
assert 'log FULL' in before
assert 'UP reference missing' in t.state_at(hist,2.8)['lines']
assert g.outcome[1],g.outcome[2]

records={}
t.simulate(t.setup(),duration=15,records=records)
original=records[t.PATH]
bad=json.loads(original); bad['id']='different-craft'
hist,_,_=h.sim([t.key(1,'l'),t.key(1.2,'i')],gui=True,disk_capacity=2000,
               records={t.PATH:json.dumps(bad)},duration=3)
assert 'LOAD REJECTED' in h.txt(hist) and 'craft' in h.txt(hist),t.state_at(hist,2.8)['lines']
assert 'log FULL' in h.txt(hist)

hist,_,_=h.sim([t.key(1,'l'),t.key(1.2,'j')],records=records,gui=True,duration=3)
assert 'Pad XYZ captured' in h.txt(hist)
assert records[t.PATH]==original
hist,_,_=h.sim([t.key(1,'e'),t.key(1.2,'j')],records=records.copy(),gui=True,duration=3)
assert 'J blocked: ARMED' in h.txt(hist)
assert 'ARMED MANUAL' in t.state_at(hist,2.8)['lines']
hist,_,_=h.sim([(1,'kick',.15,0),t.key(1.5,'j')],records=records.copy(),gui=True,duration=3)
assert 'J blocked: tilt' in h.txt(hist)
hist,_,_=h.sim([t.key(1,'u'),t.key(1.5,'j')],gui=True,duration=3)
assert 'Pad XYZ captured' in h.txt(hist)  # UP is sufficient; response calibration is separate.
print('PASS: missing file, hidden log warning, rejected ID, valid load/J, armed/tilt gates and fresh UP.')
