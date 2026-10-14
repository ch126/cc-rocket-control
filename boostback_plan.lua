-- Shadow return-burn stages: TURN -> BOOSTBACK -> LEVEL -> recovery handoff.
-- This is a bounded small-craft maneuver, not a validated full inversion.
-- No peripherals. Input position/velocity is world-frame, quaternion body->world.
local M={version=1}
local function finite(v) return type(v)=="number" and v==v and math.abs(v)<math.huge end
local function clamp(v,a,b) return math.max(a,math.min(b,v)) end
local function norm(x,z) return math.sqrt(x*x+z*z) end
local function up(q)
  assert(type(q)=="table","Missing quaternion")
  local x,y,z=q.x,q.y,q.z; local w=q.w
  for _,v in ipairs({x,y,z,w}) do assert(finite(v),"Invalid quaternion") end
  assert(finite(x) and finite(y) and finite(z) and finite(w),"Missing quaternion field")
  local n=math.sqrt(x*x+y*y+z*z+w*w); assert(n>1e-6,"Zero quaternion")
  x,y,z,w=x/n,y/n,z/n,w/n
  return {x=2*(x*y-w*z),y=1-2*(x*x+z*z),z=2*(y*z+w*x)}
end
local function validate(p,s,now)
  assert(type(s)=="table" and finite(now),"Missing sample/time")
  for _,k in ipairs({"time","x","y","z","vx","vy","vz","mass","angularSpeed"}) do assert(finite(s[k]),"Invalid "..k) end
  assert(now>=s.time and now-s.time<=.25,"Stale sample")
  assert(s.craftId==p.craftId and s.mass>0 and math.abs(s.mass/p.mass-1)<=.02,"Craft changed")
  assert(s.allNineReady==true,"Engine link missing")
  local bodyUp=up(s.q)
  assert(bodyUp.y>=math.cos(math.rad(30)) and s.angularSpeed>=0 and s.angularSpeed<=.25,"Outside boostback envelope")
  return bodyUp
end
function M.start(recovery,profile,pad,sample,now)
  local ok,result=pcall(function()
    local valid,why=recovery.validate(profile); assert(valid,why)
    assert(profile.boostbackVerified==true,"Boostback maneuver not verified")
    assert(finite(profile.boostbackAccel) and profile.boostbackAccel>0 and profile.boostbackAccel<=3,"Invalid boostback authority")
    assert(type(pad)=="table" and finite(pad.x) and finite(pad.y) and finite(pad.z),"No pad")
    validate(profile,sample,now)
    assert(sample.y-pad.y-20>=50,"Boostback needs at least 50 blocks above hover plane")
    assert(norm(sample.x-pad.x,sample.z-pad.z)<=64,"Outside return envelope")
    local p={}; for k,v in pairs(profile) do p[k]=v end
    return {phase="TURN",profile=p,pad={x=pad.x,y=pad.y,z=pad.z},
      lastTime=now,startTime=now,holdY=sample.y,integral=0}
  end)
  if not ok then return nil,tostring(result) end
  return result,nil
end
function M.step(recovery,state,s,now)
  if state.phase=="HANDOFF" then return {phase="HANDOFF",recoveryState=state.recoveryState,retainPreviousCommand=true,shadowOnly=true} end
  if state.phase=="FAULT" then return {phase="FAULT",reason=state.reason,manualTakeover=true,shadowOnly=true} end
  local ok,out=pcall(function()
    local p=state.profile; local bodyUp=validate(p,s,now)
    local dt=now-state.lastTime; assert(dt>0 and dt<=.25,"Invalid interval"); state.lastTime=now
    assert(now-state.startTime<=45,"Boostback timeout")
    local dx,dz=state.pad.x-s.x,state.pad.z-s.z
    assert(norm(dx,dz)<=64,"Return envelope exceeded")
    local h=s.y-state.pad.y-20
    local time=h>0 and (s.vy+math.sqrt(s.vy*s.vy+2*p.gravityMax*h))/p.gravityMax or 0
    local horizon=math.max(1,time)
    local ex,ez=dx/horizon-s.vx,dz/horizon-s.vz
    local miss=norm(dx-s.vx*time,dz-s.vz*time)
    local ax,az=.8*ex,.8*ez
    local limit=math.min(p.boostbackAccel,p.gravityMin*math.tan(math.rad(20)))
    local length=norm(ax,az); if length>limit then ax,az=ax*limit/length,az*limit/length end
    local g=(p.gravityMin+p.gravityMax)/2
    local gain=(p.accelPerThrottleMin+p.accelPerThrottleMax)/2
    local stopHeight=recovery.stoppingHeight(p,s.vy)
    if h<=stopHeight+p.margin then state.phase="EMERGENCY_LEVEL" end
    local levelling=state.phase=="LEVEL" or state.phase=="EMERGENCY_LEVEL"
    local tx,tz=levelling and 0 or ax/g,levelling and 0 or az/g
    local lengthUp=math.sqrt(1+tx*tx+tz*tz)
    local target={x=tx/lengthUp,y=1/lengthUp,z=tz/lengthUp}
    local error=math.acos(clamp(bodyUp.x*target.x+bodyUp.y*target.y+bodyUp.z*target.z,-1,1))
    if state.phase=="TURN" and error<math.rad(3) and s.angularSpeed<.08 then
      state.settled=state.settled or now
      if now-state.settled>=.3 then state.phase="BOOSTBACK"; state.settled=nil end
    elseif state.phase=="TURN" then state.settled=nil end
    if state.phase=="BOOSTBACK" and miss<1 and norm(ex,ez)<.2 then state.phase="LEVEL" end
    if state.phase=="LEVEL" and miss>3 then state.phase="BOOSTBACK" end
    if (state.phase=="LEVEL" or state.phase=="EMERGENCY_LEVEL") and bodyUp.y>math.cos(math.rad(3)) and s.angularSpeed<.08 then
      local input={}; for k,v in pairs(s) do input[k]=v end
      input.tilt=math.acos(clamp(bodyUp.y,-1,1))
      local nextState,why=recovery.startFromBoostback(p,state.pad,input,now)
      assert(nextState,why)
      state.phase="HANDOFF"
      state.recoveryState=nextState
      -- Main output remains powered until the receiving planner supplies its
      -- first frame. Never introduce an unexamined cutoff at the stage boundary.
      return {phase="HANDOFF",recoveryState=nextState,retainPreviousCommand=true,shadowOnly=true}
    end
    local desiredVy=clamp(.35*(state.holdY-s.y),-.5,.5)
    local ev=desiredVy-s.vy
    local denom=gain*math.max(.5,bodyUp.y)
    local raw=(g+2*ev+state.integral)/denom
    if (raw>=0 and raw<=p.maxThrottle) or (raw<0 and ev>0) or (raw>p.maxThrottle and ev<0) then
      state.integral=clamp(state.integral+.3*ev*dt,-.2*g,.2*g)
    end
    local throttle=clamp((g+2*ev+state.integral)/denom,0,p.maxThrottle)
    if state.phase=="EMERGENCY_LEVEL" then throttle=p.maxThrottle end
    return {phase=state.phase,targetUp=target,alignThrustWithTarget=true,mainThrottle=throttle,
      lateral={ax=ax,az=az},predictedMiss=miss,ballisticTime=time,shadowOnly=true}
  end)
  if not ok then state.phase="FAULT"; state.reason=tostring(out); return {phase="FAULT",reason=state.reason,manualTakeover=true,shadowOnly=true} end
  return out
end
return M
