"""Actual Lua backend in async CC task simulator; no live Minecraft access."""
from pathlib import Path
import test_wired as t

backend=Path(__file__).with_name('engine_io.lua').read_text(encoding='utf-8')
SOURCE='local Backend=(function()\n'+backend+'\nend)()\n'+'''
local function sleep(dt)
  local id=os.startTimer(dt)
  while true do local e,n=os.pullEventRaw(); if e=='timer' and n==id then return end end
end
driver=Backend.new({clock=os.clock,peripheral=peripheral,redstone=redstone,parallel=parallel,sleep=sleep},
  {733,733,733,733,733,4,4,4,4})
local ready,why=driver:initialize(); assert(ready,why)
local frame={}
for i=1,9 do frame[i]={name='creative_vector_thruster_'..(i-1),kn=0,vectorX=0,vectorY=0} end
local ok,errors=driver:run(function(d)
  local timer=os.startTimer(.1); local sending=false
  while true do
    local e,k=os.pullEventRaw()
    if e=='terminate' then return end
    if e=='key' then
      if k==keys.q then return
      elseif k==keys.e then d:arm()
      elseif k==keys.space then sending=false; d:stop('Emergency stop')
      elseif k==keys.r then
        for i=1,5 do frame[i].kn=10 end
        sending=true
      elseif k==keys.o then
        for i=6,9 do frame[i].kn=(i-5)*.5 end
        sending=true
      elseif k==keys.c then for i=1,5 do frame[i].kn=0 end
      elseif k==keys.w then for i=1,5 do frame[i].vectorX=1/15 end
      elseif k==keys.x then frame[9].kn=500
      elseif k==keys.f then sending=false end
      if sending then d:submit(frame) end
    end
    if e=='timer' and k==timer then
      if sending and d.armed then d:submit(frame) end
      timer=os.startTimer(.1)
    end
  end
end)
assert(ok,table.concat(errors,'; '))
print(driver.fault or 'OK')
'''
def sim(actions=(),**kw): return t.simulate(actions,source=SOURCE,engine_count=9,**kw)
def cleared(g): return all(e.gate==0 and e.output==0 for e in g.motors.values())

if __name__=='__main__':
    h,c,g=sim([t.key(1,'e'),t.key(1.2,'r'),t.key(1.4,'o')],initial_throttle=15)
    assert g.outcome[1],g.outcome[2]
    s=t.state_at(h,2.5)
    assert s['powers']==[10000]*5+[500,1000,1500,2000],s
    assert cleared(g)
    for stop in ('space','q'):
        for at in (1.225,1.275,1.5):
            h,c,g=sim([t.key(1,'e'),t.key(1.2,'r'),t.key(at,stop)],duration=2)
            assert cleared(g) and g.outcome[1]
            if at==1.225: assert not any(v[0]>at and v[2]=='setThrust' and v[3]>0 for v in c)
    h,c,g=sim([t.key(1,'e'),t.key(1.2,'r'),t.key(1.4,'o'),t.key(1.8,'c')])
    assert t.state_at(h,2.5)['powers']==[0]*5+[500,1000,1500,2000]
    for key in ('f','x'):
        h,c,g=sim([t.key(1,'e'),t.key(1.2,'r'),t.key(1.5,key)])
        assert g.driver.fault and cleared(g)
    for name in (f'creative_vector_thruster_{i}' for i in range(9)):
        h,c,g=sim(missing=name,initial_throttle=15)
        assert not g.outcome[1]
        assert all(e.gate==0 and e.output==0 for n,e in g.motors.items() if n!=name)
    for method in ('setThrust','setThrustOutput','setVector'):
        h,c,g=sim([t.key(1,'e'),(1.15,'writefail'),t.key(1.2,'r'),t.key(1.5,'w'),t.key(1.6,'o')],fail_method=method)
        # A failed gate-off may leave its acknowledged gate nonzero; base zero
        # still removes thrust. Persistent API errors must not claim clean exit.
        assert g.driver.fault and all(e.gate==0 or e.output==0 for e in g.motors.values())
    h,c,g=sim([t.key(1,'e'),t.key(1.2,'r'),(1.4,'slow_output',1),t.key(1.5,'w')],duration=4)
    assert g.driver.fault and cleared(g)
    print('PASS: async nine-output routing, startup zero, pending-write drain, coast, stale stream, cap rejection, missing engines, write faults and slow-output watchdog.')
