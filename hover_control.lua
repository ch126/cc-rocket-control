-- Persistent post-recovery hover. Pure computations, no peripheral calls.
local M={version=1}
local function finite(x) return type(x)=="number" and x==x and math.abs(x)<math.huge end
local function clamp(x,a,b) return math.max(a,math.min(b,x)) end
function M.start(profile,pad,now)
  return {profile=profile,target={x=pad.x,y=pad.y+20,z=pad.z},integral=0,lastTime=now}
end
function M.step(deps,state,sample,now,craft,limits)
  local ok,out=pcall(function()
    local p=state.profile
    local dt=now-state.lastTime; assert(finite(dt) and dt>0 and dt<=.25,"Hover interval invalid")
    for _,key in ipairs({"time","x","y","z","vx","vy","vz","mass"}) do assert(finite(sample[key]),"Invalid hover "..key) end
    assert(now>=sample.time and now-sample.time<=.25,"Hover telemetry stale")
    assert(sample.craftId==p.craftId and sample.mass>0 and math.abs(sample.mass/p.mass-1)<=.02,"Hover craft changed")
    assert(sample.allNineReady==true,"Hover link lost")
    local q=deps.math.quaternion(sample.q)
    local _,_,tilt=deps.math.error(q,craft.basis)
    assert(tilt<math.rad(10),"Hover attitude limit")
    assert(type(sample.w)=="table" and finite(sample.w.x) and finite(sample.w.y) and finite(sample.w.z),"Invalid hover angular rate")
    assert(sample.w.x^2+sample.w.y^2+sample.w.z^2<=.15^2,"Hover rate limit")
    state.lastTime=now
    local desiredVy=clamp(.35*(state.target.y-sample.y),-.5,.5)
    local errorV=desiredVy-sample.vy
    local g=(p.gravityMin+p.gravityMax)/2
    local gain=(p.accelPerThrottleMin+p.accelPerThrottleMax)/2
    local denominator=gain*math.cos(tilt)
    local raw=(g+2*errorV+state.integral)/denominator
    if (raw>=0 and raw<=p.maxThrottle) or (raw<0 and errorV>0) or (raw>p.maxThrottle and errorV<0) then
      state.integral=clamp(state.integral+.3*errorV*dt,-.2*g,.2*g)
    end
    local ax,az=.15*(state.target.x-sample.x)-.8*sample.vx,.15*(state.target.z-sample.z)-.8*sample.vz
    local norm=math.sqrt(ax*ax+az*az)
    if norm>p.lateralAccelMin then ax,az=ax*p.lateralAccelMin/norm,az*p.lateralAccelMin/norm end
    local plan={phase="HOVER",mainThrottle=clamp((g+2*errorV+state.integral)/denominator,0,p.maxThrottle),
      targetUp={x=0,y=1,z=0},lateral={ax=ax,az=az},shadowOnly=true}
    return deps.control.step(deps,state,sample,now,craft,limits,plan)
  end)
  if not ok then return {manualTakeover=true,commands=nil,shadowOnly=true,error=tostring(out)} end
  return out
end
return M
