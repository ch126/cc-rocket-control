"""Read-only sampler contracts, synthetic CC API; no game flight claim."""
from pathlib import Path
import test_wired as t

lua=t.LuaRuntime(unpack_returned_tuples=True)
root=Path(__file__).parent
lua.globals().telemetry=lua.execute((root/'telemetry.lua').read_text(encoding='utf-8'))
lua.globals().mathlib=lua.execute((root/'home.lua').read_text(encoding='utf-8'),'--test')
lua.execute('''
local time,delay,angle,mass,id,missing,broken=0,0,0,100,"craft",nil,nil
local function read(k,v)
  time=time+delay
  if broken==k then error("read failed") end
  return v
end
local api={clock=function() return time end,sublevel={
  isInPlotGrid=function() return read("plot",true) end,
  getUniqueId=function() return read("id",id) end,
  getMass=function() return read("mass",mass) end,
  getLogicalPose=function() return read("pose",{position={x=10,y=20,z=30},
    orientation={v={x=0,y=0,z=math.sin(angle/2)},a=math.cos(angle/2)}}) end,
  getLinearVelocity=function() return read("velocity",{x=1,y=-2,z=3}) end},
  parallel={waitForAll=function(...) for _,f in ipairs({...}) do f() end end},
  peripheral={hasType=function(name) return name~=missing end}}
-- Deliberately serial mock: variable total acquisition duration tests freshness.
local driver={ready=true,engines={}}
local function ack() for i=1,9 do driver.engines[i]={name="creative_vector_thruster_"..(i-1),lastAck=time} end end
local basis=mathlib.basis({x=0,y=0,z=0,w=1})
local sampler=telemetry.new(api,mathlib,basis)
ack()
local s,err=sampler:read(driver); assert(not s and err:find("two fresh"))
time=.1; angle=.01; ack(); s,err=sampler:read(driver)
assert(s and not err and s.allNineReady and math.abs(s.w.z-.1)<1e-8)
assert(s.x==10 and s.y==20 and s.z==30 and s.vy==-2 and s.craftId=="craft")
time=.2; missing="creative_vector_thruster_8"; s=sampler:read(driver); assert(s and not s.allNineReady)
missing=nil; time=.3; driver.engines[1].lastAck=-1
s=sampler:read(driver); assert(s and not s.allNineReady)
time=.4; ack(); driver.engines[1].lastAck=1
s=sampler:read(driver); assert(s and not s.allNineReady)
time=.5; ack(); s=sampler:read(nil); assert(s and not s.allNineReady)
time=.6; delay=.06; s,err=sampler:read(driver); assert(not s and err:find("too slow"))
delay=0; s,err=sampler:read(driver); assert(not s and err:find("two fresh"))
time=time+.1; broken="velocity"; s,err=sampler:read(driver); assert(not s and err:find("read failed"))
broken=nil; s=sampler:read(driver); assert(not s)
time=time+.1; ack(); s=sampler:read(driver); assert(s)
time=time+.1; id="other"; s,err=sampler:read(driver); assert(not s and err:find("Craft changed"))
s=sampler:read(driver); assert(not s)
time=time+.1; mass=110; s,err=sampler:read(driver); assert(not s and err:find("Craft changed"))
s=sampler:read(driver); assert(not s)
time=time+.1; delay=.01; local before=time; ack(); s=sampler:read(driver)
assert(s and s.time==before and s.receivedAt>s.time and s.fieldTimes.getMass)
time=time+.3; delay=0; s,err=sampler:read(driver); assert(not s and err:find("gap invalid"))
-- Run the actual diagnostic entry below with no write APIs available.
sublevel=api.sublevel; parallel=api.parallel; peripheral=api.peripheral
os={clock=api.clock}; sleep=function(dt) time=time+dt end
require=function(name) assert(name=="telemetry"); return telemetry end
loadfile=function(name) assert(name=="home.lua"); return function(arg) assert(arg=="--test"); return mathlib end end
lines={}; print=function(line) lines[#lines+1]=line end
''')
lua.execute((root/'telemetry_probe.lua').read_text(encoding='utf-8'))
lua.execute('assert(table.concat(lines,"\\n"):find("Valid 29/30, rejected 1",1,true))')
print('PASS telemetry: rate, warmup, conservative timestamps, identity/mass change, missing/stale/future engine acknowledgements, slow/failed reads and gaps.')
print('PASS actual read-only probe entry: 30 samples, no engine or filesystem write APIs exposed.')
