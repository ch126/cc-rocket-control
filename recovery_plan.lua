-- Experimental, side-effect-free recovery planner. NOT an engine controller.
-- No peripheral, filesystem, redstone or event-loop calls are permitted here.
-- Units: blocks, seconds, radians, throttle fraction [0,1].
local M={version="0.6.0-preview",hoverAbovePad=20}
local function finite(x) return type(x)=="number" and x==x and math.abs(x)<math.huge end
local function clamp(x,a,b) return math.max(a,math.min(b,x)) end
local function positive(x) return finite(x) and x>0 end
local function distance(a,b) return math.sqrt((a.x-b.x)^2+(a.z-b.z)^2) end

function M.validate(profile)
  if type(profile)~="table" then return false,"Missing measured flight profile" end
  if profile.version~=1 or type(profile.craftId)~="string" or profile.craftId=="" then
    return false,"Missing craft-specific profile identity"
  end
  -- These bounds must come from measured thrust/acceleration trials, not from
  -- the GUI's kN label or an assumed Earth gravity. Simulation supplies its own.
  if not positive(profile.mass) or not positive(profile.gravityMin) or
     not positive(profile.gravityMax) or profile.gravityMin>profile.gravityMax or
     not positive(profile.accelPerThrottleMin) or not positive(profile.accelPerThrottleMax) or
     profile.accelPerThrottleMin>profile.accelPerThrottleMax then
    return false,"Invalid measured gravity/acceleration bounds"
  end
  if not positive(profile.maxThrottle) or profile.maxThrottle>1 or
     not positive(profile.hoverThrottle) or profile.hoverThrottle>=profile.maxThrottle then
    return false,"Invalid measured hover/max thrust"
  end
  if not finite(profile.ignitionDelay) or profile.ignitionDelay<0 or profile.ignitionDelay>2 or
     not positive(profile.margin) or profile.margin<5 then
    return false,"Require measured delay <=2s and at least 5 blocks margin"
  end
  if profile.rcsVerified~=true or profile.verticalVerified~=true then
    return false,"Top-engine response and vertical braking trials not verified"
  end
  local net=profile.accelPerThrottleMin*profile.maxThrottle*math.cos(math.rad(10))-profile.gravityMax
  if net<=0 then return false,"Insufficient conservative upward braking acceleration" end
  return true
end

function M.stoppingHeight(profile,verticalSpeed)
  assert(M.validate(profile))
  assert(finite(verticalSpeed),"Invalid vertical speed")
  local down=math.max(0,-verticalSpeed)
  local delay=profile.ignitionDelay+0.25 -- planning/telemetry interval reserve
  local later=down+profile.gravityMax*delay
  local net=profile.accelPerThrottleMin*profile.maxThrottle*math.cos(math.rad(10))-profile.gravityMax
  return down*delay+0.5*profile.gravityMax*delay*delay+
         later*later/(2*net)+profile.margin,net
end

local function validSample(s,now,p)
  if type(s)~="table" or not finite(now) then return false,"No live sample" end
  for _,key in ipairs({"time","x","y","z","vx","vy","vz","tilt","angularSpeed","mass"}) do
    if not finite(s[key]) then return false,"Invalid sample: "..key end
  end
  if now<s.time or now-s.time>0.25 then return false,"Telemetry stale or clock reversed" end
  if s.craftId~=p.craftId or s.mass<=0 or math.abs(s.mass/p.mass-1)>0.02 then
    return false,"Craft identity/mass changed; invalidate recovery profile"
  end
  if s.allNineReady~=true then return false,"All nine engine links/acknowledgements required" end
  if s.tilt<0 or s.tilt>math.rad(10) or s.angularSpeed<0 or s.angularSpeed>0.15 then
    return false,"Outside verified attitude envelope"
  end
  return true
end

function M.start(profile,pad,sample,now)
  local ok,why=M.validate(profile)
  if not ok then return nil,why end
  if type(pad)~="table" or not finite(pad.x) or not finite(pad.y) or not finite(pad.z) then
    return nil,"Record launch reference while supported and upright"
  end
  ok,why=validSample(sample,now,profile)
  if not ok then return nil,why end
  if distance(pad,sample)>64 then return nil,"Pad outside 64-block horizontal envelope" end
  if sample.tilt>math.rad(3) or math.abs(sample.vy)>0.3 or math.sqrt(sample.vx^2+sample.vz^2)>1 then
    return nil,"Start from stable powered hover, not an existing uncontrolled fall"
  end
  local required=M.stoppingHeight(profile,-profile.gravityMax)
  if sample.y-pad.y-M.hoverAbovePad<math.max(30,required) then
    return nil,"Not enough height above the 20-block hover target"
  end
  local copy={}; for k,v in pairs(profile) do copy[k]=v end
  return {phase="ALIGN",profile=copy,pad={x=pad.x,y=pad.y,z=pad.z},
          targetY=pad.y+M.hoverAbovePad,alignY=sample.y,startTime=now,lastTime=now,
          ignited=false,velocityIntegral=0},nil
end

local function fail(state,reason)
  state.phase="FAULT"; state.reason=reason
  -- Deliberately no throttle=0 recommendation: a control/telemetry fault in
  -- descent cannot be equated with a safe engine shutdown.
  return {phase="FAULT",reason=reason,manualTakeover=true,shadowOnly=true}
end

function M.step(state,sample,now)
  if state.phase=="FAULT" then return fail(state,state.reason) end
  local p=state.profile
  local ok,why=validSample(sample,now,p)
  if not ok then return fail(state,why) end
  if now<=state.lastTime or now-state.lastTime>0.25 then return fail(state,"Invalid planner interval") end
  local dt=now-state.lastTime
  state.lastTime=now
  local lateral=distance(state.pad,sample)
  local lateralSpeed=math.sqrt(sample.vx^2+sample.vz^2)
  local height=sample.y-state.targetY
  local brakeHeight,net=M.stoppingHeight(p,sample.vy)
  local out={phase=state.phase,targetX=state.pad.x,targetZ=state.pad.z,
             targetY=state.targetY,remainingHeight=height,brakeHeight=brakeHeight,
             rcsRequired=true,shadowOnly=true}
  if state.phase=="ALIGN" then
    out.targetY=state.alignY
    out.poweredPositionHold=true
    -- An existing validated position-hold controller must execute ALIGN. This
    -- module does not pretend to supply a lateral/attitude allocator.
    if now-state.startTime>180 then return fail(state,"Powered alignment timeout") end
    local ready=lateral<1 and lateralSpeed<0.15 and math.abs(sample.vy)<0.2 and sample.tilt<math.rad(3)
    if ready then state.readySince=state.readySince or now else state.readySince=nil end
    if state.readySince and now-state.readySince>=3 then
      if height<math.max(30,brakeHeight+10) then return fail(state,"Insufficient height before engine cutoff") end
      state.phase="COAST"; state.coastStart=now
    end
  end
  if state.phase=="COAST" then
    out.poweredPositionHold=false
    if height<=brakeHeight or lateral>2 or lateralSpeed>0.6 or sample.tilt>math.rad(5) or now-state.coastStart>15 then
      state.phase="BRAKE"; state.ignited=true; state.burnStart=now
    else out.mainThrottle=0 end
  end
  if state.phase=="BRAKE" then
    if now-state.burnStart>60 then return fail(state,"Braking/hover acquisition timeout") end
    -- Conservative descent envelope into a hover above the reference plane,
    -- not a touchdown burn. Once lit, never re-enter COAST during this run.
    local desiredVy=height>=0 and -math.min(3,0.5*height,math.sqrt(2*net*0.3*height)) or math.min(0.5,-0.5*height)
    local errorV=desiredVy-sample.vy
    local gravity=(p.gravityMin+p.gravityMax)/2
    local gain=(p.accelPerThrottleMin+p.accelPerThrottleMax)/2
    local denominator=gain*math.cos(sample.tilt)
    local raw=(gravity+2*errorV+state.velocityIntegral)/denominator
    if (raw>=0 and raw<=p.maxThrottle) or (raw<0 and errorV>0) or (raw>p.maxThrottle and errorV<0) then
      state.velocityIntegral=clamp(state.velocityIntegral+0.3*errorV*dt,-gravity*0.2,gravity*0.2)
    end
    out.mainThrottle=clamp((gravity+2*errorV+state.velocityIntegral)/denominator,0,p.maxThrottle)
    -- Ignition delay is already spent once burning. Reusing the COAST trigger
    -- here would repeatedly command full power near hover and cause a bounce.
    out.estimatedInsufficientHeight=height<math.max(0,-sample.vy)^2/(2*net)
    out.poweredPositionHold=true
    if math.abs(height)<0.75 and math.abs(sample.vy)<0.2 and lateral<1 and lateralSpeed<0.15 then
      state.hoverSince=state.hoverSince or now
      if now-state.hoverSince>=2 then state.phase="HOVER" end
    else state.hoverSince=nil end
  end
  if state.phase=="HOVER" then
    -- Handoff to independently validated height/position control is required.
    out.mainThrottle=nil; out.poweredPositionHold=true; out.handoffHover=true
  end
  out.phase=state.phase
  return out
end
return M
