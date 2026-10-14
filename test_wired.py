"""Actual Lua 5.2 controller, asynchronous CC tasks and five physical actuators.

No game or network access. Peripheral writes take effect only at task completion.
This deliberately models 1.1.5's normalized-throttle quantization, not ideal APIs.
"""
import heapq
import itertools
import json
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent / '.test-deps'))
from lupa.lua52 import LuaRuntime

SOURCE=Path(__file__).with_name(sys.argv[1] if len(sys.argv)>1 else 'wired.lua').read_text(encoding='utf-8')
NAMES=[f'creative_vector_thruster_{i}' for i in range(5)]
PATH='/rocket-wired-calibration-v2.cfg'
# Test-only workaround for observed Windows Lua 5.2/Lupa native crashes around
# cross-language callbacks: retain runtimes and disable their Lua GC. This does
# not change the shipped controller or Minecraft's garbage collection.
_runtimes=[]

def simulate(actions=(), duration=3, latency=.05, records=None, missing=None,
             fail_method=None, blocked=False, reverse=False, gain=.05, hover=5,
             initial_throttle=0, cap=6666666.666667, smoothing=True,
             source=SOURCE, horizontal=False, gravity_scale=1, nozzle_lag=.05, log_failure=False,
             vertical_gain=1, gui=False, disk_capacity=None, screen_size=(51,19), engine_count=5, program_args=(), expected_top=None):
    lua=LuaRuntime(unpack_returned_tuples=True)
    _runtimes.append(lua)
    lua.execute('collectgarbage("stop")')
    g=lua.globals()
    queue=[]; counter=itertools.count(); history=[]; commands=[]
    def enqueue(at,event,a=None,b=None,callback=None):
        heapq.heappush(queue,(float(at),next(counter),event,a,b,callback))
    g.enqueue=enqueue
    g.source=source; g.latency=latency; g.maximum=cap
    g.engineCount=engine_count
    g.programArgs=lua.table_from(program_args)
    names=[f'creative_vector_thruster_{i}' for i in range(engine_count)]
    g.disk=lua.table_from(records or {})
    g.initialThrottle=initial_throttle
    g.absent=missing
    g.failMethod=fail_method
    g.logFailure=log_failure
    g.gui=gui; g.diskCapacity=disk_capacity
    g.screenWidth,g.screenHeight=screen_size
    def plain(v):
        if not hasattr(v,'items'): return v
        d={k:plain(x) for k,x in v.items()}
        if d and set(d)==set(range(1,len(d)+1)): return [d[i] for i in range(1,len(d)+1)]
        return d
    g.encode=lambda v:json.dumps(plain(v))
    def decode(s):
        try: return lua.table_from(json.loads(s),recursive=True)
        except (ValueError,TypeError): return None
    g.decode=decode
    g.logCommand=lambda *a:commands.append(a)
    lua.execute('''
      now=0; nextId=0; lines={}; motors={}; redstoneOutputs={top=9}
      frame={x=0,z=0,vx=0,vz=0,y=50,vy=0,qx=0,qz=0,qw=1,mass=100,id='test-craft'}
      keys={e=1,r=2,f=3,w=4,s=5,d=6,a=7,c=8,space=9,q=10,u=11,k=12,h=13,
        v=14,n=15,j=16,p=17,l=18,t=19,g=20,b=21,i=22,o=23,x=24,left=25,right=26}
      os={clock=function() return now end,
        pullEventRaw=function(filter) return coroutine.yield(filter) end,
        startTimer=function(dt) nextId=nextId+1; enqueue(now+dt,'timer',nextId); return nextId end}
      local function run(waitAll,...)
        local tasks,filters={},{}
        for i,fn in ipairs({...}) do tasks[i]=coroutine.create(fn) end
        local alive=#tasks
        local event={n=0}
        while alive>0 do
          for i,co in ipairs(tasks) do
            if coroutine.status(co)~='dead' and (not filters[i] or filters[i]==event[1] or event[1]=='terminate') then
              local ok,f=coroutine.resume(co,table.unpack(event,1,event.n))
              if not ok then error(f,0) end
              filters[i]=f
              if coroutine.status(co)=='dead' then
                alive=alive-1; if not waitAll then return i end
              end
            end
          end
          if alive>0 then event=table.pack(os.pullEventRaw()) end
        end
      end
      parallel={waitForAny=function(...) return run(false,...) end,
        waitForAll=function(...) return run(true,...) end}
      function task(fn,delay)
        nextId=nextId+1; local id=nextId
        local result
        enqueue(now+(delay or latency),'task_complete',id,nil,function() result=table.pack(pcall(fn)) end)
        while true do local event,got=os.pullEventRaw('task_complete'); if got==id then break end end
        if not result[1] then error(result[2],0) end
        return table.unpack(result,2,result.n)
      end
      print=function(s) lines[#lines+1]=tostring(s or '') end
      term={clear=function() lines={} end,setCursorPos=function() end}
      if gui then
        local cx,cy=1,1
        term.getSize=function() return screenWidth,screenHeight end
        term.setCursorPos=function(x,y)
          assert(x>=1 and x<=screenWidth and y>=1 and y<=screenHeight,'cursor out of bounds')
          cx,cy=x,y
        end
        term.setTextColor=function() end; term.setBackgroundColor=function() end
        term.write=function(s)
          assert(cx+#s-1<=screenWidth,'text overflows screen')
          local row=lines[cy] or string.rep(' ',screenWidth)
          lines[cy]=row:sub(1,cx-1)..s..row:sub(cx+#s)
          cx=cx+#s
        end
      end
      redstone={setAnalogOutput=function(s,v) assert(v==0,'wired must not drive redstone'); redstoneOutputs[s]=v end}
      sublevel={
        isInPlotGrid=function() return task(function() return not frame.detached end) end,
        getUniqueId=function() return task(function() return frame.id end) end,
        getMass=function() return task(function() return frame.mass end) end,
        getLinearVelocity=function() return task(function() return {x=frame.vx,y=frame.vy,z=frame.vz} end) end,
        getLogicalPose=function() return task(function()
          assert(not frame.fault,'SENSOR FAILURE'); assert(not frame.detached,'NO SUBLEVEL')
          return {position={x=frame.x,y=frame.y,z=frame.z},orientation={v={x=frame.qx,y=0,z=frame.qz},a=frame.qw}}
        end,frame.sensorLatency) end}
      peripheral={isPresent=function(n) return n=='bottom' or (motors[n] and motors[n].present) or false end,
        hasType=function(n,t) return (n=='bottom' and t=='modem') or (motors[n] and motors[n].present and t=='creative_vector_thruster') or false end}
      for i=0,engineCount-1 do
        local name='creative_vector_thruster_'..i
        local e={present=name~=absent,gate=initialThrottle,output=733000,x=0,y=0,cx=0,cy=0}
        motors[name]=e
        local p={getMaxThrustOutputPn=function() return maximum end}
        for _,method in ipairs({'setThrust','setThrustOutput','setVector','setThrustNormalized'}) do
          p[method]=function(a,b) return task(function()
            assert(e.present,'DISCONNECTED '..name)
            assert(not (frame.writeFault and name=='creative_vector_thruster_2' and method==failMethod),'WRITE FAILURE')
            logCommand(now,name,method,a,b)
            if method=='setThrust' then assert(a==0 or a==15); e.gate=a
            elseif method=='setThrustNormalized' then e.gate=math.floor(math.max(0,math.min(1,a))*15+1e-6)
            elseif method=='setThrustOutput' then assert(a>=0 and a<=733000+1e-6); e.output=a
            else
              assert(math.abs(a)<=.2+1e-9 and math.abs(b)<=.2+1e-9)
              local function quantize(v) return (v<0 and -1 or 1)*math.floor(math.abs(v)*15+.5)/15 end
              e.x,e.y=quantize(a),quantize(b)
            end
          end,frame.outputLatency) end
        end
        e.p=p
      end
      peripheral.wrap=function(n)
        if n=='bottom' then return {isWireless=function() return false end} end
        if peripheral.isPresent(n) then return motors[n].p end
      end
      peripheral.call=function(n,m,...) return assert(peripheral.wrap(n),'MISSING '..n)[m](...) end
      textutils={serialize=function(v) return encode(v) end,unserialize=function(s) return decode(s) end}
      fs={exists=function(p) return disk[p]~=nil end,getSize=function(p) return #disk[p] end,
        getFreeSpace=function()
          if not diskCapacity then return 'unlimited' end
          local used=0; for _,s in pairs(disk) do used=used+#s end
          return diskCapacity-used
        end,
        delete=function(p) disk[p]=nil end,
        move=function(a,b) assert(disk[a] and not disk[b]); disk[b]=disk[a]; disk[a]=nil end,
        open=function(p,m)
          if logFailure and string.find(p,'rocket-flight',1,true) then return nil,'DISK FAILURE' end
          if m=='w' or m=='a' then
            if m=='w' or not disk[p] then disk[p]='' end
            return {write=function(s)
              local free=fs.getFreeSpace('/')
              assert(type(free)~='number' or free>=#s,'Out of space')
              disk[p]=disk[p]..s
            end,close=function() end}
          end
          return {readAll=function() return disk[p] end,close=function() end}
        end}
      controller=coroutine.create(function() outcome=table.pack(pcall(assert(load(source)),table.unpack(programArgs))) end)
      function resumeController(...) return coroutine.resume(controller,...) end
      function finished() return coroutine.status(controller)=='dead' end
    ''')
    ax=az=wx=wz=0.; y=50.; vy=0.; airborne=False
    px=pz=vx=vz=0.
    def advance(dt):
        nonlocal ax,az,wx,wz,y,vy,px,pz,vx,vz
        motors=[g.motors[n] for n in NAMES]
        power=sum(e.gate/15*e.output/733000*15 for e in motors)/5
        count=max(1,math.ceil(dt/.01)); h=dt/count
        if not blocked:
            for _ in range(count):
                for e in motors:
                    # Mod tween is 20% toward target per 50ms game tick.
                    alpha=1-.8**(h/nozzle_lag) if smoothing else 1
                    e.cx+=(e.x-e.cx)*alpha; e.cy+=(e.y-e.cy)*alpha
                pitch=sum(e.cx*15*(e.gate/15*e.output/733000*3) for e in motors)/5
                yaw=sum(e.cy*15*(e.gate/15*e.output/733000*3) for e in motors)/5
                tx,tz=(-yaw*gain,pitch*gain) if reverse else (pitch*gain,yaw*gain)
                wx+=tx*h; wz+=tz*h; ax+=wx*h; az+=wz*h
                if airborne:
                    vy+=(power*math.cos(math.hypot(ax,az))-hover)*vertical_gain*h
                    y+=vy*h
                    if horizontal:
                        angle=math.hypot(ax,az)
                        sn=math.sin(angle)/angle if angle else 1
                        # Tilt accelerates laterally; gimbal initially pushes opposite
                        # to the commanded tilt (thrust-vectoring nonminimum-phase response).
                        direct_x=(tz/gain)*5*math.tan(math.pi/6)/15
                        direct_z=-(tx/gain)*5*math.tan(math.pi/6)/15
                        vx+=(-az*sn*power+direct_x)*gravity_scale*h
                        vz+=(ax*sn*power+direct_z)*gravity_scale*h
                        px+=vx*h; pz+=vz*h
        angle=math.hypot(ax,az); f=math.sin(angle/2)/angle if angle else .5
        g.frame.qx=ax*f; g.frame.qz=az*f; g.frame.qw=math.cos(angle/2)
        g.frame.y=y; g.frame.vy=vy
        g.frame.x=px; g.frame.z=pz; g.frame.vx=vx; g.frame.vz=vz
    for action in actions:
        at,kind,*rest=action
        enqueue(at,kind,rest[0] if rest else None,rest[1] if len(rest)>1 else None)
    enqueue(duration,'terminate')
    ok,*rest=as_tuple(g.resumeController())
    assert ok,rest
    while queue:
        at,_,event,a,b,callback=heapq.heappop(queue)
        assert at<duration+10,'cleanup hung'
        advance(max(0,at-g.now)); g.now=at
        if callback is not None: callback()
        if event=='key': a=g['keys'][a]; b=bool(b)
        elif event=='fault': g.frame.fault=True
        elif event=='mass': g.frame.mass=120
        elif event=='disconnect':
            g.motors[a].present=False; event='peripheral_detach'
        elif event=='writefail': g.frame.writeFault=True
        elif event=='slow_sensor': g.frame.sensorLatency=a
        elif event=='slow_output': g.frame.outputLatency=a
        elif event=='lift': y+=a; vy=0; airborne=True
        elif event=='kick': ax+=a; az+=b
        elif event=='spin': wx+=a; wz+=b
        elif event=='fall': vy=a
        elif event=='drift': vx+=a; vz+=b
        elif event=='position': px=a; pz=b
        if event=='click':
            ok,*rest=as_tuple(g.resumeController('mouse_click',1,a,b))
        else:
            ok,*rest=as_tuple(g.resumeController(event,a,b))
        assert ok,rest
        history.append(dict(time=at,lines=' '.join(g.lines.values()),
            powers=[g.motors[n].gate/15*g.motors[n].output for n in names],
            x=ax,z=az,y=y,vy=vy,px=px,pz=pz,vx=vx,vz=vz))
        if g.finished(): break
    assert g.finished(),'controller never exited'
    assert g.redstoneOutputs.top==(expected_top if expected_top is not None else (0 if engine_count==9 else 9))
    if records is not None: records.clear(); records.update(dict(g.disk))
    return history,commands,g

def as_tuple(v): return v if isinstance(v,tuple) else (v,)
def key(at,k,held=False): return (at,'key',k,held)
def state_at(h,at): return next(s for s in reversed(h) if s['time']<=at)
def setup():
    return [key(1,'u'),key(1.1,'e')]+[key(1.3+i*.02,'r') for i in range(33)]+[key(2.5,'k'),key(14,'h')]

if __name__=='__main__':
    # Fine thrust is real base-force control, NOT quantized normalized throttle.
    h,c,g=simulate([key(1,'e'),key(1.2,'r'),key(1.4,'t'),key(1.6,'w')],initial_throttle=15)
    assert all(abs(p-8063)<1e-5 for p in state_at(h,2.5)['powers'])
    assert not any(x[2]=='setThrustNormalized' for x in c)
    assert all(e.gate==0 and e.output==0 for e in g.motors.values())
    assert all(abs(e.x)<1e-9 and abs(e.y)<1e-9 for e in g.motors.values())
    print('PASS: five-engine startup zero, 1%+0.1%=8.063 kN, unquantized base force, zero cleanup.')

    # Press stop during an in-flight write; rearm attempts cannot bypass zero ACKs.
    for stop_key in ('space','q'):
        h,c,g=simulate([key(1,'e'),key(1.2,'r'),key(1.225,stop_key),key(1.23,'e')])
        assert all(p==0 for p in state_at(h,2)['powers']) if stop_key=='space' else all(e.gate==0 and e.output==0 for e in g.motors.values())
        assert not any(x[0]>1.225 and x[2]=='setThrust' and x[3]>0 for x in c)
    h,c,g=simulate([key(1,'e'),key(1.2,'r'),key(1.4,'e',True)])
    assert all(abs(p-7330)<1e-5 for p in state_at(h,2)['powers'])
    for method in ('setThrustOutput','setVector','setThrust'):
        h,c,g=simulate([key(1,'e'),(1.15,'writefail'),key(1.2,'r'),key(1.5,'w'),key(2,'e')],fail_method=method)
        assert any('ENGINE FAULT' in s['lines'] for s in h)
        assert all(e.gate==0 or e.output==0 for e in g.motors.values())
    h,c,g=simulate([key(1,'e'),key(1.2,'r'),(1.5,'disconnect',NAMES[2])])
    assert all(g.motors[n].gate==0 for n in NAMES if n!=NAMES[2])
    assert 'CLEAR FAILED' in ' '.join(g.lines.values())
    h,c,g=simulate(missing=NAMES[3],initial_throttle=15)
    assert not g.outcome[1] and all(g.motors[n].gate==0 for n in NAMES if n!=NAMES[3])
    print('PASS: stop/quit during pending command, zero-ack rearm gate, held-toggle debounce, independent failure cleanup, missing engine and disconnect.')

    for latency in (.025,.05,.1):
        records={}
        h,c,g=simulate(setup()+[(15,'kick',.04,-.03)],duration=28,latency=latency,records=records)
        assert g.outcome[1],g.outcome[2]
        assert PATH in records, h[-1]
        assert any('ARMED HOLD' in s['lines'] for s in h)
        end=state_at(h,27)
        assert math.hypot(end['x'],end['z'])<.035,(latency,end)
        assert all(p>0 for p in end['powers']),end
        h,c,g=simulate(records=records,duration=2)
        assert any('Loaded; verify craft unchanged' in s['lines'] for s in h)
        assert all(p==0 for s in h for p in s['powers'])
    print('PASS: asynchronous 25/50/100ms APIs, actual calibration and hold recovery, record reload with zero thrust.')

    for fault in ('fault','mass'):
        h,c,g=simulate(setup()+[(15,fault)],duration=18)
        assert all(p==0 for p in state_at(h,17.8)['powers'])
    h,c,g=simulate(setup(),duration=15,blocked=True)
    assert not any('calibration OK' in s['lines'] for s in h)
    assert all(e.gate==0 for e in g.motors.values())
    # End-to-end altitude capture, bounded descent and manual override.
    h,c,g=simulate(setup()+[(18,'lift',8),key(24,'v'),key(27,'n')],duration=48,hover=4.95)
    assert any('HOLD ALT' in s['lines'] for s in h),h[-1]
    assert any('HOLD DESCEND' in s['lines'] for s in h)
    assert abs(state_at(h,47)['y']-55)<1, state_at(h,47)
    print('PASS: sensor/mass fault stop, blocked calibration rejection, altitude capture and limited descent.')

    for fault in ('slow_sensor','slow_output'):
        h,c,g=simulate([key(1,'e'),key(1.2,'r'),(1.5,fault,1),key(1.6,'t')],duration=5)
        assert all(p==0 for p in state_at(h,4.8)['powers']),h[-1]
        assert any(('stale' if fault=='slow_sensor' else 'Output timeout') in s['lines'] for s in h)
    # An actual stop while fully powered, not just while enabling.
    for stop_key in ('space','q'):
        h,c,g=simulate([key(1,'e'),key(1.2,'r'),key(1.5,'t'),key(1.525,stop_key)],duration=3)
        assert all(e.gate==0 and e.output==0 for e in g.motors.values())
        assert not any(x[0]>1.525 and x[2]=='setThrust' and x[3]>0 for x in c)
    h,c,g=simulate([key(1,'e'),key(1.2,'r'),key(1.4,'r',True),key(1.41,'r',True)],duration=3)
    assert all(abs(p-14660)<1e-5 for p in state_at(h,2)['powers'])
    saved=json.loads(records[PATH])
    for change in ({'backend':'redstone'},{'fullThrustKN':999},{'id':'different'},
                   {'mass':50},{'engines':NAMES[:-1]},{'throttle':float('nan')}):
        bad={PATH:json.dumps(dict(saved,**change))}
        h,c,g=simulate(records=bad,duration=2)
        assert any('LOAD REJECTED' in s['lines'] for s in h),change
    for contents in ('not a table','x'*17000):
        h,c,g=simulate(records={PATH:contents},duration=2)
        assert any('LOAD REJECTED' in s['lines'] for s in h)
    # Altitude takeover keys must return to a manual throttle, even for fine trim.
    altitude_setup=setup()+[(18,'lift',8),key(24,'v')]
    for override in ('r','f','t','g','c','h','v','space'):
        h,c,g=simulate(altitude_setup+[key(25,override)],duration=27,hover=4.95)
        assert any('HOLD ALT' in s['lines'] for s in h)
        assert 'HOLD ALT' not in state_at(h,26)['lines'],override
    print('PASS: stalled sensor/output watchdogs, powered in-flight stop, held-key rate limiting, backend/cap/identity/mass/corrupt-record rejection, all altitude overrides.')
