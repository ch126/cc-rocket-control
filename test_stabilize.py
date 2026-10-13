"""Math and end-to-end tests, executing the actual single-file Lua program."""
import math
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent / '.test-deps'))
from lupa.lua52 import LuaRuntime

SOURCE = Path(__file__).with_name(sys.argv[1] if len(sys.argv)>1 else 'stabilize.lua').read_text(encoding='utf-8')
lua = LuaRuntime(unpack_returned_tuples=True)
mod = lua.execute(SOURCE, '--test')
def table(**kw): return lua.table_from(kw)
def quat(axis, angle):
    values = dict(x=0, y=0, z=0, w=math.cos(angle/2))
    values[axis] = math.sin(angle/2)
    return table(**values)
identity = quat('x',0)
basis = mod.basis(identity)
for axis in ('x','z'):
    q = quat(axis, .2)
    e1,e2,tilt = mod.error(q,basis)
    assert abs(tilt-.2)<1e-8
    rate = mod.rate(identity,q,.1)
    assert abs(rate[axis]-2)<1e-8
    opposite = table(**{k:-v for k,v in dict(q).items()})
    assert abs(mod.rate(q,opposite,.1).x)<1e-8
assert mod.error(quat('y',1),basis)[2]<1e-8  # yaw is deliberately free
wrapped = table(v=table(x=0,y=0,z=0),a=1)
assert mod.quaternion(wrapped).w==1
v=mod.rotate(quat('y',math.pi/2),table(x=5,y=0,z=0))
assert abs(v.z+5)<1e-8  # rotation must preserve magnitudes
for bad in (table(x=0,y=0,z=0,w=0),table(x=math.nan,y=0,z=0,w=1)):
    try: mod.quaternion(bad); raise AssertionError('accepted invalid quaternion')
    except Exception as exc: assert 'quaternion' in str(exc) or 'vector' in str(exc)
for columns in ([[0,0],[0,0]], [[1,0],[2,0]]):
    try: mod.matrix(lua.table_from([lua.table_from(v) for v in columns])); raise AssertionError('accepted bad axes')
    except Exception as exc: assert 'response' in str(exc) or 'overlap' in str(exc)

def simulate(actions, blocked=False, reverse=False, gain=.08, api_delay=0, hover=5, vertical_gain=1, records=None, craft_id='test-craft', write_fail=False, yielding=False):
    runtime=LuaRuntime(unpack_returned_tuples=True)
    runtime.execute('''
      now=0; currentTimer=0; outputs={top=9}; snapshots={}; frame={}
      keys={e=1,r=2,f=3,w=4,s=5,d=6,a=7,c=8,space=9,q=10,u=11,k=12,h=13,v=14,n=15,j=16,p=17,l=18}
      redstone={setAnalogOutput=function(side,v)
        assert(v%1==0 and v>=0 and v<=15); outputs[side]=v
        assert(not ((outputs.front or 0)>0 and (outputs.back or 0)>0))
        assert(not ((outputs.right or 0)>0 and (outputs.left or 0)>0))
      end}
      lines={}
      print=function(s) lines[#lines+1]=tostring(s or '') end
      term={clear=function() lines={} end,setCursorPos=function() end}
      sublevel={
        isInPlotGrid=function() return not frame.detached end,
        getUniqueId=function() return frame.id or 'test-craft' end,
        getMass=function() return frame.mass or 100 end,
        getLinearVelocity=function() return {x=0,y=frame.vy or 0,z=0} end,
        getLogicalPose=function()
          if frame.sensorError then error('SENSOR FAILURE') end
          return {position={x=0,y=frame.y or 50,z=0},orientation={
            v={x=frame.qx or 0,y=0,z=frame.qz or 0},a=frame.qw or 1}}
        end}
      os={clock=function() return now end,
          startTimer=function() currentTimer=currentTimer+1; return currentTimer end}
    ''')
    g=runtime.globals()
    g.frame.id=craft_id
    g.disk=runtime.table_from(records or {})
    g.writeFail=write_fail
    def plain(v):
        return {k:plain(x) for k,x in v.items()} if hasattr(v,'items') else v
    g.encode=lambda v:json.dumps(plain(v))
    def decode(s):
        try: return runtime.table_from(json.loads(s),recursive=True)
        except (ValueError,TypeError): return None
    g.decode=decode
    runtime.execute('''
      textutils={serialize=function(v) return encode(v) end,unserialize=function(v) return decode(v) end}
      fs={exists=function(p) return disk[p]~=nil end,
        getSize=function(p) return #disk[p] end,
        delete=function(p) disk[p]=nil end,
        move=function(a,b) assert(disk[a] and not disk[b]); disk[b]=disk[a]; disk[a]=nil end,
        open=function(p,m)
          if m=='w' then
            if writeFail then return nil,'Disk full' end
            disk[p]=''
            return {write=function(s) disk[p]=disk[p]..s end,close=function() end}
          end
          return {readAll=function() return disk[p] end,close=function() end}
        end}
      parallel={waitForAny=function(...)
        parallelActive=true
        local rootPull=os.pullEventRaw
        os.pullEventRaw=function(filter) return coroutine.yield(filter) end
        local tasks,filters={},{}
        for i,fn in ipairs({...}) do tasks[i]=coroutine.create(fn) end
        local event={n=0}
        while true do
          for i,co in ipairs(tasks) do
            if not filters[i] or filters[i]==event[1] or event[1]=='terminate' then
              local ok,f=coroutine.resume(co,table.unpack(event,1,event.n))
              while ok and f=='sim_api' do
                api_wait()
                ok,f=coroutine.resume(co)
              end
              if not ok then error(f) end
              if coroutine.status(co)=='dead' then os.pullEventRaw=rootPull; return end
              filters[i]=f
            end
          end
          event=table.pack(rootPull())
        end
      end}
    ''')
    results=[]
    ax=az=wx=wz=0.0
    y,vy,airborne=50.0,0.0,False
    events=iter(actions)
    def advance(dt):
        nonlocal ax,az,wx,wz,y,vy
        # Simple free-body plant: two independently measurable torque axes.
        # Reversed/crossed wiring is supported by measuring the response matrix.
        out={s:g.outputs[s] for s in ('front','back','left','right','bottom','top')}
        p=out.get('front',0)-out.get('back',0)
        steer=out.get('right',0)-out.get('left',0)
        scale=out.get('bottom',0)/5
        if not blocked:
            for _ in range(10):
                h=dt/10
                tx,tz=(-steer*gain,p*gain) if reverse else (p*gain,steer*gain)
                wx+=tx*scale*h; wz+=tz*scale*h
                ax+=wx*h; az+=wz*h
                if airborne:
                    vy+=(out.get('bottom',0)*math.cos(math.hypot(ax,az))-hover)*vertical_gain*h
                    y+=vy*h
        angle=math.hypot(ax,az)
        f=math.sin(angle/2)/angle if angle else .5
        g.frame.qx=ax*f; g.frame.qz=az*f; g.frame.qw=math.cos(angle/2)
        g.frame.y=y; g.frame.vy=vy
        g.now+=dt
    def pull():
        nonlocal ax,az,y,vy,airborne
        results.append((float(g.now),dict(g.outputs),list(g.lines.values()),ax,az,y,vy))
        action=next(events,('quit',))
        kind=action[0]
        if kind=='key': return 'key',g['keys'][action[1]],False
        if kind=='held':
            advance(action[2])
            return 'key',g['keys'][action[1]],True
        if kind=='api': return 'task_complete',None,None
        if kind=='keyup': return 'key_up',g['keys'][action[1]],None
        if kind=='quit': return 'terminate',None,None
        if kind=='fault': g.frame.sensorError=True
        if kind=='detach': g.frame.detached=True
        if kind=='mass': g.frame.mass=120
        if kind=='kick': ax+=action[1]; az+=action[2]
        if kind=='lift': y+=action[1]; vy=0; airborne=True
        if kind=='vertical_kick': vy+=action[1]
        advance(action[1] if kind=='delay' else .1)
        return 'timer',g.currentTimer,None
    def api_wait(): advance(api_delay)
    g.api_wait=api_wait
    g.yielding=yielding
    runtime.execute('''
      for name,fn in pairs(sublevel) do
        sublevel[name]=function(...)
          if yielding then os.pullEventRaw('task_complete') end
          if parallelActive then coroutine.yield('sim_api') else api_wait() end
          if frame.detached and name~='isInPlotGrid' then error('NO SUBLEVEL') end
          return fn(...)
        end
      end
    ''')
    g.os.pullEventRaw=pull
    runtime.execute(SOURCE)
    if records is not None:
        records.clear(); records.update(dict(g.disk))
    assert g.outputs.top==9
    assert all(g.outputs[s]==0 for s in ['bottom','front','back','left','right'])
    return results

def key(name): return ('key',name)
ticks=[('tick',)]
setup=ticks+ [key('u'),key('e')]+[key('r')]*5 +ticks+[key('k')]+ticks*45
for reverse in (False,True):
    r=simulate(setup+[key('h'),('kick',.05,-.04)]+ticks*100+[key('space')],reverse=reverse)
    assert any('calibration OK' in ' '.join(x[2]) for x in r), ('calibration failed',r[8:15],r[-1])
    assert any('ARMED HOLD' in ' '.join(x[2]) for x in r), 'hold did not enable'
    after=[x for x in r if x[0]>13 and x[1]['bottom']==5]
    assert after and max(math.hypot(x[3],x[4]) for x in after)<.08, 'failed to settle'
    assert all(max(x[1]['front'],x[1]['back'],x[1]['left'],x[1]['right'])<=3 for x in r)
for fault in [('fault',),('detach',),('mass',),('delay',.6)]:
    r=simulate(setup+[key('h'),fault]+ticks*15)
    assert r[-1][1]['bottom']==0, fault
r=simulate(setup,blocked=True)
assert not any('calibration OK' in ' '.join(x[2]) for x in r)
assert r[-1][1]['bottom']==0
r=simulate(ticks+[key('e'),key('r'),key('h')]+ticks)
assert not any('ARMED HOLD' in ' '.join(x[2]) for x in r)
r=simulate(setup,gain=.3)
assert r[-1][1]['bottom']==0 and any('Calibration tilt exceeded' in ' '.join(x[2]) for x in r)
r=simulate(setup+[key('h'),key('f')]*1+[key('f')]*4+ticks)
assert r[-1][1]['bottom']==0 and not any('ARMED HOLD' in s for s in r[-1][2])
r=simulate(ticks+[key('u'),key('e'),key('r'),key('k'),key('c')]+ticks)
assert r[-1][1]['bottom']==1 and r[-1][1]['front']==0
print('PASS: quaternion formats/signs/rotation, body-rate math, yaw freedom, singular-response rejection, real calibration state machine, swapped/reversed wiring, hold settling, bounds, missing calibration, blocked craft, sensor loss, craft detach, mass change, delayed loop, exit cleanup.')

# Regression: each real Sable call can wait a server tick (or two).
# The old five-call loop exceeds 0.35s at 0.06s/call plus 0.1s timer.
for latency in (.05,.06,.1):
    r=simulate(setup+[key('h')]+ticks*60,api_delay=latency,gain=.05)
    text='\n'.join(' '.join(x[2]) for x in r)
    assert 'calibration OK' in text, (latency, text[-1500:])
    assert 'ARMED HOLD' in text, (latency, text[-1500:])
    assert 'allowed 0.025-0.350s' not in text, (latency, text[-1500:])
    assert r[-1][1]['bottom']==5, (latency, text[-1500:])
r=simulate(setup+[key('h'),('delay',.6)]+ticks*2,api_delay=.05,gain=.05)
assert r[-1][1]['bottom']==0
assert any('Sample dt=' in ' '.join(x[2]) and 'allowed' in ' '.join(x[2]) for x in r)
print('PASS: 50/60/100ms per-API scheduling delay, calibration and hold with slow status refresh, genuine 600ms stall still stops and reports interval.')
