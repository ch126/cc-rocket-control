"""Nine-output plumbing tests; synthetic motion is NOT flight certification."""
from pathlib import Path
import test_wired as t

SOURCE=Path(__file__).with_name('nine.lua').read_text(encoding='utf-8')
def sim(actions=(), **kw):
    return t.simulate(actions, source=SOURCE, engine_count=9, **kw)
def text(h): return '\n'.join(s['lines'] for s in h)

if __name__=='__main__':
    h,c,g=sim([t.key(1,'e'),t.key(1.2,'r')], initial_throttle=15)
    s=t.state_at(h,2)
    assert all(abs(p-7330)<1e-6 for p in s['powers'][:5]),s
    assert s['powers'][5:]==[0]*4
    assert all(e.gate==0 and e.output==0 for e in g.motors.values())
    assert not any(v[2]=='setThrust' and v[3]>0 and int(v[1].split('_')[-1])>=5 for v in c)
    for index in range(5,9):
        name=f'creative_vector_thruster_{index}'
        h,c,g=sim(missing=name,initial_throttle=15)
        assert not g.outcome[1]
        assert all(e.gate==0 and e.output==0 for n,e in g.motors.items() if n!=name)
        h,c,g=sim([t.key(1,'e'),t.key(1.2,'r'),(1.5,'disconnect',name)])
        assert all(e.gate==0 and e.output==0 for n,e in g.motors.items() if n!=name)
    for key in ('space','q'):
        h,c,g=sim([t.key(1,'e'),t.key(1.2,'r'),t.key(1.225,key)])
        assert all(e.gate==0 and e.output==0 for e in g.motors.values())
    h,c,g=sim([t.key(1,'o'),t.key(1.2,'x'),t.key(1.4,'right')],gui=True)
    assert not any(v[2]=='setThrust' and v[3]>0 for v in c)
    print('PASS: nine startup/cleanup, five-only manual thrust, all top disconnects/missing, emergency stops, GUI and probe guards.')
    records={}
    h,c,g=sim(t.setup(),duration=16,records=records)
    assert '/rocket-nine-main-calibration-v2.cfg' in records,text(h)[-2000:]
    # Record launch pad after calibration, lift safely, capture position/height.
    actions=[t.key(1,'j'),t.key(1.2,'e')]+[t.key(1.4+i*.02,'r') for i in range(33)]
    actions += [t.key(3,'h'),(4,'lift',25),t.key(8,'v'),t.key(9,'o')]
    h,c,g=sim(actions,duration=13,records=records.copy(),hover=4.95)
    assert 'RCS short probe' in text(h),text(h)[-3000:]
    assert 'Not calibrated' in text(h),text(h)[-3000:]
    top=[v for v in c if int(v[1].split('_')[-1])>=5 and v[2]=='setThrust' and v[3]>0]
    assert len(top)==1 and top[0][1].endswith('_5'),top
    assert all(v[3]<=1000.000001 for v in c if int(v[1].split('_')[-1])>=5 and v[2]=='setThrustOutput')
    assert all(e.gate==0 and e.output==0 for e in g.motors.values())
    print('PASS: isolated calibration file, guarded single top pulse, one-kN default, acknowledged sample and nine-output exit.')
    on=top[0][0]
    for stop in ('c','h','e','space','q'):
        h,c,g=sim(actions+[t.key(on+.01,stop)],duration=13,records=records.copy(),hover=4.95)
        assert all(e.gate==0 and e.output==0 for e in g.motors.values()),stop
        assert all(s['powers'][5:]==[0]*4 for s in h if s['time']>on+.5),stop
    for index in range(5,9):
        selected=actions[:-1]+[t.key(8.2+i*.15,'right') for i in range(index-5)]+[t.key(9,'o')]
        h,c,g=sim(selected,duration=13,records=records.copy(),hover=4.95)
        enabled=[v[1] for v in c if v[2]=='setThrust' and v[3]>0 and int(v[1].split('_')[-1])>=5]
        assert enabled==[f'creative_vector_thruster_{index}'],enabled
    for fault in ((on+.01,'slow_sensor',1),(on+.01,'slow_output',1)):
        h,c,g=sim(actions+[fault],duration=14,records=records.copy(),hover=4.95)
        assert all(e.gate==0 and e.output==0 for e in g.motors.values())
        assert 'Not calibrated' not in text(h)
    print('PASS: all four selected outputs, pulse cancellation/disarm/emergency exit, delayed sensor/output rejection.')
