"""Compare response to the same disturbance, using identical calibration records."""
import math
from pathlib import Path
import test_wired as t
SOURCE=Path(__file__).with_name('home.lua').read_text(encoding='utf-8')

if __name__=='__main__':
    records={}
    t.simulate(t.setup(),duration=15,records=records)
    actions=[t.key(1,'e')]+[t.key(1.3+i*.02,'r') for i in range(33)]+[t.key(2.5,'h'),(4,'kick',.1,-.07)]
    scores={}
    for scale in (.7,1,1.3):
        for name,source in (('old',t.SOURCE),('new',SOURCE)):
            h,c,g=t.simulate(actions,duration=28,source=source,records=records.copy(),
                latency=.1,gain=.05*scale,nozzle_lag=.07)
            end=t.state_at(h,27)
            assert all(p>0 for p in end['powers']), (name,scale,end)
            if name=='new': assert math.hypot(end['x'],end['z'])<.045,(name,scale,end)
            # Vector total variation reflects gimbal activity/chatter under lag.
            axis=[x for x in c if x[0]>=8 and x[1]==t.NAMES[0] and x[2]=='setVector']
            variation=sum(abs(a[3]-b[3])+abs(a[4]-b[4]) for a,b in zip(axis,axis[1:]))
            peak=max(math.hypot(s['x'],s['z']) for s in h if 8<s['time']<27)
            scores[name,scale]=(variation,peak)
            print(name,scale,'gimbal variation',round(variation,3),'late peak deg',round(math.degrees(peak),2),flush=True)
    assert sum(scores['new',k][0] for k in (.7,1,1.3))<sum(scores['old',k][0] for k in (.7,1,1.3)),scores
    print('PASS: calibration-response uncertainty, delayed nozzles, reduced aggregate gimbal switching, bounded settling.')
