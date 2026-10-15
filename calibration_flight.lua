-- Experimental model-assisted commissioning controller. Pure/shadow only.
-- No old flight profile is accepted or generated. Geometry is a prior, not proof.
local M={version=1}
local function finite(x) return type(x)=="number" and x==x and math.abs(x)<math.huge end
local function clamp(x,a,b) return math.max(a,math.min(b,x)) end
local function vec(x,y,z) return {x=x,y=y,z=z} end
local function dot(a,b) return a.x*b.x+a.y*b.y+a.z*b.z end
function M.start(craft,s,now)
  assert(craft.id==s.craftId and math.abs(craft.profile.mass/s.mass-1)<.02,"Geometry mismatch")
  assert(s.tilt<math.rad(1) and math.abs(s.vy)<.05 and s.vx*s.vx+s.vz*s.vz<.0025,"Start supported and still")
  return {phase="RAMP",craft=craft,pad=vec(s.x,s.y,s.z),start=now,last=now,phaseAt=now,
    throttle=0,integral=0,gain=5*733/s.mass,previousVy=s.vy,window=1,windows={},
    trial=1,trials={},nextTrial="VERTICAL",topCap=40}
end
function M.step(deps,state,s,now)
  if state.phase=="FAULT" then return {manualTakeover=true,error=state.reason,commands=nil} end
  local ok,out=pcall(function()
    local dt=now-state.last; assert(finite(dt) and dt>0 and dt<=.25,"Commissioning timing invalid")
    for _,key in ipairs({"time","x","y","z","vx","vy","vz","tilt","angularSpeed","mass"}) do assert(finite(s[key]),"Invalid "..key) end
    assert(now>=s.time and now-s.time<=.25 and s.allNineReady,"Commissioning telemetry/link stale")
    local c=state.craft
    assert(c.id==s.craftId and math.abs(c.profile.mass/s.mass-1)<.02,"Craft changed")
    local height=s.y-state.pad.y
    assert(height>=-.25 and height<=25,"Commissioning height envelope")
    assert((s.x-state.pad.x)^2+(s.z-state.pad.z)^2<=9,"Commissioning drift envelope")
    assert(s.tilt<math.rad(8) and s.angularSpeed<.15 and math.abs(s.vy)<5,"Commissioning motion envelope")
    assert(now-state.start<600 or state.phase=="COMPLETE_HOVER","Commissioning timeout")
    local acceleration=(s.vy-state.previousVy)/dt
    state.last=now; state.previousVy=s.vy
    local q=deps.math.quaternion(s.q)
    local inverse={x=-q.x,y=-q.y,z=-q.z,w=q.w}
    local e1,e2=deps.math.error(q,c.basis)
    local a1=.65*e1-1.43*dot(s.w,c.basis.a)
    local a2=.65*e2-1.43*dot(s.w,c.basis.b)
    local angular=vec(c.basis.a.x*a1+c.basis.b.x*a2,c.basis.a.y*a1+c.basis.b.y*a2,c.basis.a.z*a1+c.basis.b.z*a2)
    local function enter(phase) state.phase=phase; state.phaseAt=now; state.stableSince=nil end
    if state.phase=="RAMP" then
      assert(now-state.phaseAt<70,"No liftoff; check support/mass/output")
      state.throttle=math.min(.5,state.throttle+.008*dt)
      if height>.15 and s.vy>.1 then
        state.gravity=state.gain*state.throttle-acceleration
        assert(state.gravity>.5 and state.gravity<30,"Unusable initial gravity estimate")
        enter("SETTLE")
      end
    end
    if state.phase~="RAMP" then
      local desiredVy=clamp(.4*(12-height),-.7,.7)
      local err=desiredVy-s.vy
      if state.phase=="SETTLE" or state.phase=="COMPLETE_HOVER" then
        state.integral=clamp(state.integral+.3*err*dt,-.3*state.gravity,.3*state.gravity)
      end
      state.throttle=clamp((state.gravity+2*err+state.integral)/(state.gain*math.cos(s.tilt)),0,.55)
    end
    local lateral=vec(clamp(.15*(state.pad.x-s.x)-.8*s.vx,-.3,.3),0,
      clamp(.15*(state.pad.z-s.z)-.8*s.vz,-.3,.3))
    local directions={vec(1,0,0),vec(-1,0,0),vec(0,0,1),vec(0,0,-1)}
    if state.phase=="COUPLED" then
      local d=directions[state.trial]; lateral=vec(.12*d.x,0,.12*d.z)
    end
    local body=deps.math.rotate(inverse,vec(lateral.x,state.gain*state.throttle*math.cos(s.tilt),lateral.z))
    body.y=math.max(0,body.y)
    local limits={mainKN=.55*733,topKN={state.topCap,state.topCap,state.topCap,state.topCap},steps=1,slewSteps=1,
      previous=state.previous,balance=state.balance,dt=dt}
    local allocation=deps.allocator.solve(deps.geometry,c.profile,c.main,c.top,{linear=body,angular=angular},limits)
    state.previous={x=allocation.mainX,y=allocation.mainY}; state.balance=allocation.balance
    local commands={}
    for i=1,9 do commands[i]={name="creative_vector_thruster_"..(i-1),kn=i<=5 and allocation.mainKN or allocation.topKN[i-5],
      vectorX=i<=5 and allocation.mainX/15 or 0,vectorY=i<=5 and allocation.mainY/15 or 0} end
    if state.phase=="SETTLE" then
      local stable=math.abs(height-12)<.25 and math.abs(s.vy)<.1 and s.vx*s.vx+s.vz*s.vz<.01 and s.tilt<math.rad(1) and s.angularSpeed<.02
      if stable then state.stableSince=state.stableSince or now else state.stableSince=nil end
      if state.stableSince and now-state.stableSince>2 then
        if state.nextTrial=="VERTICAL" and state.window>4 then
          local candidate,why=deps.evidence.vertical({craftId=c.id,mass=c.profile.mass,windows=state.windows})
          assert(candidate,why); state.candidate=candidate
          state.gravity=(candidate.gravityMin+candidate.gravityMax)/2
          state.gain=(candidate.accelPerThrottleMin+candidate.accelPerThrottleMax)/2
          state.hoverEstimate=candidate.hoverThrottle; state.integral=0
          state.nextTrial="RESPONSE"; state.trial=1
        end
        if state.nextTrial=="VERTICAL" then
          state.hoverEstimate=state.hoverEstimate or state.throttle
          state.trialThrottle=state.hoverEstimate+(state.window%2==1 and -.04 or .04)
          assert(state.trialThrottle>0 and state.trialThrottle<.55,"Trial throttle outside cap")
          state.windows[state.window]={}; state.appliedSince=nil; enter("VERTICAL")
        elseif state.nextTrial=="RESPONSE" then
          state.responseStart=nil; state.appliedSince=nil; enter("RESPONSE")
        elseif state.nextTrial=="COUPLED" then
          state.responseStart=nil; enter("COUPLED")
        elseif state.nextTrial=="BRAKE" then
          assert(height>11.5,"Insufficient brake-test altitude")
          state.dropAt=now; enter("DROP")
        elseif state.nextTrial=="COMPLETE" then
          enter("COMPLETE_HOVER")
        end
      end
    end
    if state.phase=="VERTICAL" then
      for i=1,9 do commands[i].kn=i<=5 and state.trialThrottle*733 or 0; commands[i].vectorX=0; commands[i].vectorY=0 end
      state.balance=nil; state.previous={x=0,y=0}
      -- Applied state must come from the output backend, not the requested frame.
      local matched=type(s.applied)=="table" and #s.applied==9
      local age=0
      if matched then for i,e in ipairs(s.applied) do
        if not finite(e.kn) or not finite(e.ackTime) or e.ackTime>s.time or now-e.ackTime>.3 or
          math.abs(e.kn-commands[i].kn)>.001 or e.vectorX~=0 or e.vectorY~=0 then matched=false; break end
        age=math.max(age,now-e.ackTime)
      end end
      if matched then state.appliedSince=state.appliedSince or now else state.appliedSince=nil end
      if state.appliedSince and now-state.appliedSince>=.5 then
        local window=state.windows[state.window]
        window[#window+1]={time=s.time,craftId=s.craftId,mass=s.mass,relativeHeight=height,tilt=s.tilt,vy=s.vy,
          throttle=state.trialThrottle,commandAge=age,settledFor=now-state.appliedSince,
          velocitySource="sublevel.getLinearVelocity",topKN=0,vectorX=0,vectorY=0,allMainEqual=true}
        if #window>=7 and window[#window].time-window[1].time>=.6 then state.window=state.window+1; enter("SETTLE") end
      end
      assert(now-state.phaseAt<2.5,"Vertical window output/settling timeout")
    end
    if state.phase=="RESPONSE" then
      -- Open-loop short, settled pulses verify all four main-vector signs and
      -- each top engine against the measured geometry. Between pulses the
      -- closed-loop controller returns to the original hover point.
      local index=state.trial
      local vx,vy=0,0
      if index<=4 then
        vx=index<=2 and (index==1 and 1/15 or -1/15) or 0
        vy=index>=3 and (index==3 and 1/15 or -1/15) or 0
      end
      for i=1,9 do
        commands[i].kn=i<=5 and state.hoverEstimate*733 or (i==index+1 and 20 or 0)
        commands[i].vectorX=i<=5 and vx or 0; commands[i].vectorY=i<=5 and vy or 0
      end
      state.balance=nil; state.previous={x=vx*15,y=vy*15}
      local matched=type(s.applied)=="table" and #s.applied==9
      if matched then for i,e in ipairs(s.applied) do
        if not finite(e.ackTime) or e.ackTime>s.time or now-e.ackTime>.3 or
          not finite(e.kn) or math.abs(e.kn-commands[i].kn)>.001 or e.vectorX~=commands[i].vectorX or e.vectorY~=commands[i].vectorY then matched=false; break end
      end end
      if matched then state.appliedSince=state.appliedSince or now else state.appliedSince=nil; state.responseStart=nil end
      if state.appliedSince and now-state.appliedSince>=.4 then
        state.responseStart=state.responseStart or {time=s.time,w=vec(s.w.x,s.w.y,s.w.z)}
        local first=state.responseStart; local span=s.time-first.time
        if span>=.2 then
          local expected=deps.geometry.total(c.profile,c.main,state.hoverEstimate*733,vx,vy).angular
          if index>4 then
            local r=deps.geometry.response(c.profile,c.top[index-4],20,0,0).angular
            expected=vec(expected.x+r.x,expected.y+r.y,expected.z+r.z)
          end
          local measured=vec((s.w.x-first.w.x)/span,(s.w.y-first.w.y)/span,(s.w.z-first.w.z)/span)
          local square=dot(expected,expected); assert(square>1e-6,"Unobservable angular trial")
          local ratio=dot(measured,expected)/square
          local residual=math.sqrt((measured.x-expected.x)^2+(measured.y-expected.y)^2+(measured.z-expected.z)^2)
          assert(ratio>.65 and ratio<1.35 and residual<math.max(.005,math.sqrt(square)*.35),"Angular response disagrees with geometry; no calibration saved")
          state.trials[#state.trials+1]={kind="response",index=index,ratio=ratio,residual=residual,span=span}
          state.trial=index+1
          if state.trial>8 then state.nextTrial="COUPLED"; state.trial=1 end
          enter("SETTLE")
        end
      end
      assert(now-state.phaseAt<2,"Angular response output timeout")
    end
    if state.phase=="COUPLED" then
      -- A one-second baseline excludes initial command/nozzle transients.
      if now-state.phaseAt>=1 then
        state.responseStart=state.responseStart or {time=s.time,vx=s.vx,vz=s.vz}
        local first=state.responseStart; local span=s.time-first.time
        if span>=2 then
          local d=directions[state.trial]
          local measured=((s.vx-first.vx)*d.x+(s.vz-first.vz)*d.z)/span
          assert(s.tilt<math.rad(2) and measured>.04 and measured<.22,"Coupled translation/attitude trial failed: "..tostring(measured))
          state.trials[#state.trials+1]={kind="coupled",index=state.trial,acceleration=measured,span=span}
          state.lateralMin=math.min(state.lateralMin or math.huge,measured*.6)
          state.trial=state.trial+1
          if state.trial>4 then state.nextTrial="BRAKE" end
          enter("SETTLE")
        end
      end
    end
    if state.phase=="DROP" then
      for i=1,9 do commands[i].kn=0; commands[i].vectorX=0; commands[i].vectorY=0 end
      assert(now-state.dropAt<.8,"No observed free-fall response")
      if s.vy<-.8 then
        state.brakeVy=s.vy; state.brakeHeight=height; state.brakeAt=now
        state.brakeResponse=nil; enter("BRAKE")
      end
    end
    if state.phase=="BRAKE" then
      for i=1,9 do commands[i].kn=i<=5 and .4*733 or 0; commands[i].vectorX=0; commands[i].vectorY=0 end
      if acceleration>1 then state.brakeResponse=state.brakeResponse or now end
      assert(height>5 and now-state.brakeAt<2,"Braking trial did not arrest descent within envelope")
      if s.vy>=0 and now-state.brakeAt>.2 then
        assert(state.brakeVy<-.3 and state.brakeResponse,"No measured descent/ignition response")
        local delay=state.brakeResponse-state.brakeAt+dt+.15
        assert(delay<=1,"Excessive ignition response delay")
        state.trials[#state.trials+1]={kind="brake",delayBound=delay,startVy=state.brakeVy,
          startHeight=state.brakeHeight,endHeight=height,duration=now-state.brakeAt,maxThrottle=.4}
        -- This is a narrow commissioning result, not authorization for arbitrary
        -- descent speeds, full-throttle operation, boostback or tower capture.
        state.result={kind="nine-flight-commissioning",sequenceCompleted=true,craftId=c.id,mass=c.profile.mass,
          vertical=state.candidate,trials=state.trials,brakeTestThrottle=.4,mainCommandCap=.55,
          topPulseTestKN=20,topCommandCap=40,lateralAccelMin=state.lateralMin,ignitionDelayBound=delay,
          flightCalibrationValid=false,requiresEnvelopeReview=true}
        state.nextTrial="COMPLETE"; enter("SETTLE")
      end
    end
    return {phase=state.phase,commands=commands,allocation=allocation,candidate=state.candidate,
      result=state.phase=="COMPLETE_HOVER" and state.result or nil,shadowOnly=true,flightCalibrationValid=false}
  end)
  if not ok then state.phase="FAULT"; state.reason=tostring(out); return {manualTakeover=true,error=state.reason,commands=nil} end
  return out
end
return M
