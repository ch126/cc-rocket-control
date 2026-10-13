-- v0.4.0 wired creative-vector controller for Create Propulsion: Simulated 1.1.5.
-- One controller only. Remove old redstone inputs before using this backend.
-- IMPORTANT: this build quantizes setThrustNormalized into 15 steps internally.
-- Use setThrustOutput (unquantized base force) + setThrust(15/0) instead.
-- setVector is also rounded to 1/15 steps by the installed mod; do not fake precision.
local args = {...}
local M = {}
local FULL_THRUST_KN=733 -- User-confirmed per-engine full-scale base thrust.
local UNITS_PER_KN=1000 -- Verified createpropulsion-common.toml, physics section.
local ENGINE_NAMES={}
for i=0,4 do ENGINE_NAMES[#ENGINE_NAMES+1]="creative_vector_thruster_"..i end
local function finite(x) return type(x)=="number" and x==x and math.abs(x)<math.huge end
local function clamp(x,a,b) return math.max(a,math.min(b,x)) end
local function vec(x,y,z) return {x=x,y=y,z=z} end
local function add(a,b) return vec(a.x+b.x,a.y+b.y,a.z+b.z) end
local function mul(a,s) return vec(a.x*s,a.y*s,a.z*s) end
local function dot(a,b) return a.x*b.x+a.y*b.y+a.z*b.z end
local function cross(a,b) return vec(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x) end
local function norm(a) return math.sqrt(dot(a,a)) end
local function unit(a) local n=norm(a); assert(n>1e-9,"Zero vector"); return mul(a,1/n) end
local function checked(v)
  assert(type(v)=="table" and finite(v.x) and finite(v.y) and finite(v.z),"Invalid vector")
  return vec(v.x,v.y,v.z)
end
function M.quaternion(q)
  assert(type(q)=="table","Missing orientation")
  local v=q.v or q
  local w=q.a
  if w==nil then w=q.w end
  v=checked(v); assert(finite(w),"Invalid quaternion")
  local n=math.sqrt(dot(v,v)+w*w)
  assert(finite(n) and n>1e-6,"Invalid quaternion norm")
  return {x=v.x/n,y=v.y/n,z=v.z/n,w=w/n}
end
local function conjugate(q) return {x=-q.x,y=-q.y,z=-q.z,w=q.w} end
local function product(a,b)
  local v=add(add(mul(a,b.w),mul(b,a.w)),cross(a,b))
  return {x=v.x,y=v.y,z=v.z,w=a.w*b.w-dot(a,b)}
end
function M.rotate(q,v)
  local t=mul(cross(q,v),2)
  return add(v,add(mul(t,q.w),cross(q,t)))
end
-- Orientation-difference rate is in body coordinates (rad/s). This avoids
-- assuming a frame or units for a physics backend's angular-velocity API.
function M.rate(previous,current,dt)
  assert(finite(dt) and dt>0,"Invalid sample time")
  local d=product(conjugate(previous),current)
  if d.w<0 then d={x=-d.x,y=-d.y,z=-d.z,w=-d.w} end
  local n=norm(d)
  if n<1e-8 then return vec(0,0,0) end
  return mul(d,2*math.atan2(n,d.w)/(n*dt))
end
function M.basis(q)
  local up=unit(M.rotate(conjugate(q),vec(0,1,0)))
  local seed=math.abs(up.x)<0.8 and vec(1,0,0) or vec(0,0,1)
  local a=unit(add(seed,mul(up,-dot(seed,up))))
  return {up=up,a=a,b=unit(cross(up,a))}
end
function M.error(q,basis)
  local target=M.rotate(conjugate(q),vec(0,1,0))
  local angle=math.acos(clamp(dot(basis.up,target),-1,1))
  local axis=cross(basis.up,target)
  local n=norm(axis)
  if n<1e-8 then return 0,0,angle end
  axis=mul(axis,angle/n)
  return dot(axis,basis.a),dot(axis,basis.b),angle
end
function M.matrix(columns)
  local a,c=columns[1][1],columns[1][2]
  local b,d=columns[2][1],columns[2][2]
  for _,v in ipairs({a,b,c,d}) do assert(finite(v),"Invalid response") end
  local n1,n2=math.sqrt(a*a+c*c),math.sqrt(b*b+d*d)
  local det=a*d-b*c
  assert(n1>0.02 and n2>0.02,"No useful response: blocked craft or insufficient thrust")
  assert(math.abs(det)/(n1*n2)>0.35,"Axes overlap: check engine orientations")
  return {a=a,b=b,c=c,d=d,det=det}
end
function M.command(matrix,e1,e2,w1,w2,ratio)
  local x,y=1.5*e1-1.8*w1,1.5*e2-1.8*w2
  local u=(matrix.d*x-matrix.b*y)/matrix.det/ratio
  local v=(-matrix.c*x+matrix.a*y)/matrix.det/ratio
  local function quantize(n) return (n<0 and -1 or 1)*math.floor(math.abs(clamp(n,-3,3))+0.5) end
  return quantize(u),quantize(v)
end
-- Height loop uses a recorded near-hover throttle, not an assumed gravity.
function M.heightStep(a,y,vy,dt,tilt,calibrationThrottle)
  assert(finite(y) and finite(vy) and finite(dt) and dt>0,"Invalid height sample")
  local feed=0
  if a.descending then
    a.target=math.max(a.floor,a.target-0.3*dt)
    if a.target<=a.floor+1e-6 then a.descending=false
    else feed=-0.3 end
  end
  local errorY=a.target-y
  local targetV=clamp(0.6*errorY+feed,-0.6,0.6)
  local errorV=targetV-vy
  local lo=math.max(0.015,calibrationThrottle*0.5,a.hover-3)
  local hi=math.min(15,calibrationThrottle*2,a.hover+3)
  local base=a.hover/math.max(0.9,math.cos(tilt))
  local raw=base+1.6*errorV+a.integral
  -- Conditional integration prevents windup at either throttle boundary.
  if (raw>=lo and raw<=hi) or (raw<lo and errorV>0) or (raw>hi and errorV<0) then
    a.integral=clamp(a.integral+0.5*errorV*dt,-2,2)
  end
  raw=base+1.6*errorV+a.integral
  local output=clamp(raw,lo,hi)
  a.saturated=raw<lo or raw>hi
  return output
end
function M.validateRecord(r,id,mass)
  assert(type(r)=="table" and r.version==2 and r.backend=="wired-output-v1","Wrong calibration backend/version; run U/K")
  assert(r.fullThrustKN==FULL_THRUST_KN and r.unitsPerKN==UNITS_PER_KN,"Saved thrust scale changed")
  assert(type(r.engines)=="table" and #r.engines==#ENGINE_NAMES,"Saved engine list changed")
  for i,name in ipairs(ENGINE_NAMES) do assert(r.engines[i]==name,"Saved engine list changed") end
  assert(r.id==id,"Saved calibration belongs to another craft")
  assert(finite(r.mass) and r.mass>0 and math.abs(mass/r.mass-1)<=0.05,"Saved mass differs >5%")
  assert(finite(r.throttle) and r.throttle>0 and r.throttle<=15,"Invalid saved throttle")
  assert(type(r.basis)=="table","Missing saved UP")
  local b={up=checked(r.basis.up),a=checked(r.basis.a),b=checked(r.basis.b)}
  for _,v in pairs(b) do assert(math.abs(norm(v)-1)<1e-5,"Invalid saved axis") end
  assert(math.abs(dot(b.up,b.a))<1e-5 and norm(add(cross(b.up,b.a),mul(b.b,-1)))<1e-5,"Invalid saved basis")
  assert(type(r.matrix)=="table","Missing saved response")
  local m=M.matrix({{r.matrix.a,r.matrix.c},{r.matrix.b,r.matrix.d}})
  return b,m
end
if args[1]=="--test" then return M end

local faces={"bottom","front","back","left","right"}
local armed,throttle,pitch,yaw=false,0,0,0
local mode,message="MANUAL","Stand upright and still; press U to set UP."
local basis,referenceId,matrix,calThrottle,calMass,cal=nil,nil,nil,nil,nil,nil
local lastQ,lastTime,telemetry=nil,nil,nil
local status,statusSlot,statusDue=nil,1,0
local lastInterval=nil
local running=true
local altitude,launchY,lastY,filteredVy=nil,nil,nil,nil
local hoverReadySince,readyThrottle=nil,nil
local recordPath="/rocket-wired-calibration-v2.cfg"
local recordStatus="No saved calibration loaded"
local loadPending=true
local engines={}
local ioFault=nil
local epoch,revision=0,0
local desired={throttle=0,x=0,y=0,epoch=0,revision=0}
local function cancelAltitude()
  altitude=nil; hoverReadySince,readyThrottle=nil,nil
end
local function clearRedstone()
  local errors={}
  for _,s in ipairs(faces) do
    local ok,err=pcall(redstone.setAnalogOutput,s,0)
    if not ok then errors[#errors+1]=s..": "..tostring(err) end
  end
  return errors
end
local function apply()
  if not armed then pitch,yaw,throttle=0,0,0 end
  assert(finite(throttle) and finite(pitch) and finite(yaw),"Invalid control command")
  throttle,pitch,yaw=clamp(throttle,0,15),clamp(pitch,-3,3),clamp(yaw,-3,3)
  if desired.throttle~=throttle or desired.x~=pitch/15 or desired.y~=yaw/15 or desired.epoch~=epoch then
    revision=revision+1
    desired={throttle=throttle,x=pitch/15,y=yaw/15,epoch=epoch,revision=revision}
  end
end
local function stop(reason)
  cancelAltitude()
  armed,throttle,pitch,yaw=false,0,0,0
  mode,cal="MANUAL",nil
  message=reason or "Stopped."
  epoch=epoch+1; apply()
end
local function outputsZero()
  if #engines~=#ENGINE_NAMES then return false end
  for _,e in ipairs(engines) do
    if e.gate~=0 or e.output~=0 or e.x~=0 or e.y~=0 or e.epoch~=epoch then return false end
  end
  return true
end
local function initializeEngines()
  local modem=peripheral.wrap("bottom")
  assert(modem and peripheral.hasType("bottom","modem") and not modem.isWireless(),"Need wired modem on bottom")
  for _,name in ipairs(ENGINE_NAMES) do
    assert(peripheral.hasType(name,"creative_vector_thruster"),"Missing/wrong engine: "..name)
    local p=assert(peripheral.wrap(name),"Cannot wrap "..name)
    for _,method in ipairs({"setThrust","setThrustOutput","setVector","getMaxThrustOutputPn"}) do
      assert(type(p[method])=="function",name.." missing "..method)
    end
    local limit=p.getMaxThrustOutputPn()
    assert(finite(limit) and limit>=FULL_THRUST_KN*UNITS_PER_KN,"733 kN exceeds engine limit: "..name)
    engines[#engines+1]={name=name,p=p,gate=0,output=0,x=0,y=0,epoch=epoch,revision=revision,lastAck=os.clock()}
  end
end
-- Cleanup attempts EVERY known engine even if startup validation failed.
-- Keep base override at zero on exit: clearing the override would restore scroll thrust.
local function clearEngines()
  local errors=clearRedstone()
  local tasks={}
  for _,name in ipairs(ENGINE_NAMES) do
    tasks[#tasks+1]=function()
      for _,call in ipairs({{"setThrust",0},{"setThrustOutput",0},{"setVector",0,0}}) do
        local ok,err=pcall(function()
          assert(peripheral.hasType(name,"creative_vector_thruster"),"not connected")
          peripheral.call(name,table.unpack(call))
        end)
        if not ok then errors[#errors+1]=name.." "..call[1]..": "..tostring(err) end
      end
    end
  end
  parallel.waitForAll(table.unpack(tasks))
  return errors
end
local function failIO(err)
  if not ioFault then ioFault=tostring(err); stop("ENGINE FAULT: "..ioFault) end
end
local function writer(e)
  local function send(method,...)
    local ok,err=pcall(e.p[method],...)
    if not ok then failIO(e.name.." "..method..": "..tostring(err)) end
    return ok
  end
  while running do
    local s=desired
    local ok=true
    if s.throttle==0 then
      -- Independent attempts: failed gate-off must not skip setting base force to zero.
      if e.gate~=0 then if send("setThrust",0) then e.gate=0 else ok=false end end
      if e.output~=0 then if send("setThrustOutput",0) then e.output=0 else ok=false end end
      if e.x~=s.x or e.y~=s.y then
        if send("setVector",s.x,s.y) then e.x,e.y=s.x,s.y else ok=false end
      end
    else
      local output=FULL_THRUST_KN*UNITS_PER_KN*s.throttle/15
      if e.output~=output then if send("setThrustOutput",output) then e.output=output else ok=false end end
      if ok and s.epoch==epoch and (e.x~=s.x or e.y~=s.y) then
        if send("setVector",s.x,s.y) then e.x,e.y=s.x,s.y else ok=false end
      end
      -- Stops invalidate a queued command; never enable using its stale snapshot.
      if ok and s.epoch==epoch and e.gate~=15 then
        if send("setThrust",15) then e.gate=15 else ok=false end
      end
    end
    if ok and s.epoch==epoch then
      e.epoch,e.revision,e.lastAck=s.epoch,s.revision,os.clock()
    end
    if s.epoch==epoch then
      local timer=os.startTimer(0.05)
      repeat local event,id=os.pullEventRaw(); if event=="timer" and id==timer then break end until not running
    end -- A stop arrived mid-call: immediately loop back to flush zeros.
  end
end
local function saveRecord()
  assert(matrix and basis,"Calibrate before saving")
  local r={version=2,backend="wired-output-v1",fullThrustKN=FULL_THRUST_KN,unitsPerKN=UNITS_PER_KN,
    engines=ENGINE_NAMES,id=referenceId,mass=calMass,throttle=calThrottle,basis=basis,matrix=matrix}
  M.validateRecord(r,referenceId,calMass)
  local h,err=fs.open(recordPath..".tmp","w"); assert(h,err)
  local ok,writeError=pcall(h.write,textutils.serialize(r)); h.close()
  assert(ok,writeError)
  if fs.exists(recordPath..".bak") then fs.delete(recordPath..".bak") end
  if fs.exists(recordPath) then fs.move(recordPath,recordPath..".bak") end
  fs.move(recordPath..".tmp",recordPath)
  recordStatus="Calibration saved"
end
local function trySave()
  local ok,err=pcall(saveRecord)
  if not ok then recordStatus="SAVE FAILED: "..tostring(err) end
end
local function loadRecord()
  assert(not armed and telemetry,"Load disarmed with live telemetry")
  local path=recordPath
  -- Backup is used only if promotion of a completed temporary file was interrupted.
  if not fs.exists(path) and fs.exists(path..".bak") then path=path..".bak" end
  if not fs.exists(path) then recordStatus="No saved calibration"; return end
  assert(fs.getSize(path)<=16384,"Calibration file too large")
  local h,err=fs.open(path,"r"); assert(h,err)
  local data=h.readAll(); h.close()
  local r=textutils.unserialize(data)
  local b,m=M.validateRecord(r,telemetry.id,telemetry.mass)
  basis,matrix,referenceId,calMass,calThrottle=b,m,r.id,r.mass,r.throttle
  telemetry.e1,telemetry.e2,telemetry.tilt=M.error(telemetry.q,basis)
  launchY=nil -- Never reuse a previous flight's world-height reference.
  recordStatus="Loaded; verify craft unchanged"
  message="Calibration loaded. J: ground reference. E: arm at zero."
end
-- CC: Sable methods run on the server thread and may each yield a game tick.
-- Bootstrap at zero thrust; thereafter do at most ONE status call per sample.
-- Pose is always read LAST so its timestamp isn't aged by other API calls.
local function statusCheck(slot)
  if slot==1 then
    assert(sublevel.isInPlotGrid(),"Computer must be on an assembled craft")
  elseif slot==2 then status.id=sublevel.getUniqueId()
  elseif slot==3 then
    local mass=sublevel.getMass()
    assert(finite(mass) and mass>0,"Invalid mass")
    status.mass=mass
  else
    status.velocity=checked(sublevel.getLinearVelocity())
    status.velocityTime=os.clock()
  end
end
local function readTelemetry()
  assert(sublevel,"CC: Sable API unavailable")
  if not status then
    status={}
    for i=1,4 do statusCheck(i) end
    statusSlot,statusDue=1,os.clock()+0.25
  elseif os.clock()>=statusDue then
    statusCheck(statusSlot)
    statusSlot=statusSlot%4+1
    statusDue=os.clock()+0.25
  end
  local pose=sublevel.getLogicalPose()
  local q=M.quaternion(pose.orientation)
  local now=os.clock()
  local dt=lastTime and now-lastTime or nil
  local w=lastQ and dt and dt>0 and M.rate(lastQ,q,dt) or vec(0,0,0)
  local t={q=q,w=w,dt=dt,time=now,velocity=status.velocity,
    velocityAge=now-status.velocityTime,mass=status.mass,id=status.id}
  local position=checked(pose.position)
  local measuredVy=lastY and dt and dt>0 and (position.y-lastY)/dt or 0
  local alpha=dt and clamp(dt/(0.2+dt),0,1) or 1
  filteredVy=filteredVy and (filteredVy+alpha*(measuredVy-filteredVy)) or measuredVy
  t.y,t.vy=position.y,filteredVy
  t.rawVy=measuredVy
  lastY=position.y
  lastQ,lastTime=q,now
  if dt then lastInterval=dt end
  if basis then t.e1,t.e2,t.tilt=M.error(q,basis) end
  return t
end
local function rates(t) return dot(t.w,basis.a),dot(t.w,basis.b) end
local function invalidate(reason)
  matrix,calThrottle,calMass=nil,nil,nil
  stop(reason)
end
local stages={{0,0},{1,0},{0,0},{-1,0},{0,0},{0,1},{0,0},{0,-1},{0,0}}
local function stageStart(t)
  cal.start=t.time; cal.measureStart=nil
  pitch,yaw=stages[cal.stage][1],stages[cal.stage][2]
end
local function calibrationTick(t)
  assert(t.tilt<math.rad(12),"Calibration tilt exceeded 12 degrees")
  assert(norm(t.w)<0.6,"Calibration rotation too fast")
  assert(math.abs(t.mass/cal.mass-1)<=0.05,"Mass changed during calibration")
  -- Discard the first part of each step: CC scheduling + nozzle tween settling.
  if not cal.measureStart then
    if t.time-cal.start>=0.45 then cal.measureStart=t.time; cal.wa,cal.wb=rates(t) end
    return
  end
  if t.time-cal.start<0.9-1e-6 or t.time-cal.measureStart<0.25 then return end
  local a,b=rates(t); local dt=t.time-cal.measureStart
  cal.samples[cal.stage]={(a-cal.wa)/dt,(b-cal.wb)/dt}
  cal.stage=cal.stage+1
  if cal.stage<=#stages then stageStart(t); return end
  local s=cal.samples
  local columns={}
  for i,indices in ipairs({{2,4},{6,8}}) do
    local pos,neg=indices[1],indices[2]
    -- Subtract the preceding neutral interval's drift. Compare both signs.
    local p={s[pos][1]-s[pos-1][1],s[pos][2]-s[pos-1][2]}
    local n={s[neg][1]-s[neg-1][1],s[neg][2]-s[neg-1][2]}
    local pn,nn=math.sqrt(p[1]^2+p[2]^2),math.sqrt(n[1]^2+n[2]^2)
    assert(pn>0.02 and nn>0.02,"Calibration response too weak")
    assert((p[1]*n[1]+p[2]*n[2])/(pn*nn)<-0.5,"Opposite signals did not produce opposite rotation")
    assert(pn/nn>0.25 and pn/nn<4,"Calibration response inconsistent")
    columns[i]={(p[1]-n[1])/2,(p[2]-n[2])/2}
  end
  matrix=M.matrix(columns); calThrottle,calMass=throttle,t.mass
  mode,cal,pitch,yaw="MANUAL",nil,0,0
  message="Calibrated. Thrust unchanged. H enables HOLD."
  trySave()
end
local function tick()
  telemetry=readTelemetry()
  local t=telemetry
  if basis and t.id~=referenceId then
    basis=nil; invalidate("Craft changed. Set UP and calibrate again."); return
  end
  if matrix and math.abs(t.mass/calMass-1)>0.05 then
    invalidate("Mass changed >5%. Recalibrate."); return
  end
  if mode~="MANUAL" then
    assert(t.dt and t.dt>=0.025 and t.dt<=0.35+1e-6,
      ("Sample dt=%s; allowed 0.025-0.350s"):format(t.dt and ("%.3fs"):format(t.dt) or "missing"))
    assert(t.tilt<math.rad(25),"Tilt exceeded 25 degrees; test stopped")
    assert(norm(t.w)<1.5,"Rotation too fast; test stopped")
  end
  if mode=="HOLD" and not altitude and math.abs(t.vy)<0.4 and t.tilt<math.rad(8) then
    if readyThrottle~=throttle or not hoverReadySince then hoverReadySince=t.time; readyThrottle=throttle end
  elseif not altitude then hoverReadySince,readyThrottle=nil,nil end
  if altitude then
    if mode~="HOLD" then cancelAltitude()
    else
      assert(t.tilt<math.rad(15),"Altitude test tilt >15deg")
      assert(math.abs(t.rawVy)<4,"Vertical speed >4 blocks/s; test stopped")
      throttle=M.heightStep(altitude,t.y,t.vy,t.dt,t.tilt,calThrottle)
    end
  end
  if mode=="CALIBRATE" then calibrationTick(t)
  elseif mode=="HOLD" then
    local a,b=rates(t)
    pitch,yaw=M.command(matrix,t.e1,t.e2,a,b,throttle/calThrottle)
  end
  apply()
end
local function draw()
  term.clear(); term.setCursorPos(1,1)
  print("ROCKET v0.4.0 WIRED | 5 x 733 kN")
  print((armed and "ARMED " or "DISARMED ")..mode..(altitude and (altitude.descending and " DESCEND" or " ALT") or ""))
  print(("Power %.1f%% | base %.2f kN/engine"):format(throttle/15*100,FULL_THRUST_KN*throttle/15))
  if telemetry then
    print(("Y %.2f | vertical speed %+.2f"):format(telemetry.y,telemetry.vy))
    print(("Tilt %s | calibration %s"):format(telemetry.tilt and ("%.1f deg"):format(math.deg(telemetry.tilt)) or "unset",matrix and "OK" or "needed"))
  else print("Telemetry unavailable") end
  print(("Sample dt: %s | status: slow refresh"):format(lastInterval and ("%.3fs"):format(lastInterval) or "warming up"))
  print("U New UP | J Ground Y (disarmed)")
  print("E Arm | R/F +/-1% | T/G +/-0.1%")
  print(("W/S D/A trim: X%+d Y%+d | C reset"):format(pitch,yaw))
  print("K Calibrate: ~9s pulses, thrust unchanged")
  print("H Hold on/off (after calibration)")
  print("V Capture hover / cancel altitude")
  print("N Descend 3 blocks / pause descent")
  if altitude then
    print(("Target Y %.2f | %s"):format(altitude.target,altitude.saturated and "THRUST LIMIT" or "height control"))
  else print("P Save / L Load (disarmed)") end
  print("SPACE stop | Q exit | no ground sensing")
  print(recordStatus)
  print(message)
end
local function handleKey(k)
  if k==keys.q then stop("Exiting."); running=false; return end
  if k==keys.space then stop("Emergency stop."); return end
  if k==keys.e then
    if not armed and (ioFault or not outputsZero() or not telemetry or os.clock()-telemetry.time>0.25) then
      message=ioFault and "Engine fault latched; fix wiring and restart." or "Wait: five zeros + fresh telemetry required."
      return
    end
    local was=armed; stop(); armed=not was
    if armed then message="Armed at zero. Increase thrust manually." end
    return
  end
  if not armed and k==keys.p then trySave(); return end
  if not armed and k==keys.l then
    local ok,err=pcall(loadRecord)
    if not ok then recordStatus="LOAD REJECTED: "..tostring(err) end
    return
  end
  if k==keys.j and not armed then
    local t=telemetry
    assert(t and t.dt and t.tilt and t.tilt<math.rad(5) and os.clock()-t.time<=0.25 and math.abs(t.vy)<0.2 and norm(t.w)<0.03,"J needs still upright craft at launch position")
    launchY=t.y; message="Ground reference captured; saved calibration retained."
  elseif k==keys.u and not armed then
    local t=telemetry
    assert(t and os.clock()-t.time<=0.25 and t.dt and t.dt>0 and t.velocityAge<2 and norm(t.w)<0.03 and norm(t.velocity)<0.2,"Set UP only while still with fresh telemetry")
    basis=M.basis(t.q); referenceId=t.id; launchY=t.y
    matrix,calThrottle,calMass=nil,nil,nil
    message="UP set. Next: manual test, then K calibration."
  elseif armed then
    if k==keys.c then cancelAltitude(); mode,cal,pitch,yaw="MANUAL",nil,0,0; message="Manual; thrust unchanged."
    elseif mode=="CALIBRATE" then message="Calibrating. C cancels; SPACE stops."
    elseif k==keys.v then
      if altitude then cancelAltitude(); message="Altitude off; current thrust retained."
      elseif mode=="HOLD" and launchY and telemetry and telemetry.y>=launchY+2 and os.clock()-telemetry.time<=0.25 and hoverReadySince and telemetry.time-hoverReadySince>=2 and throttle>=0.15 and throttle<=14.85 then
        altitude={hover=throttle,target=telemetry.y,floor=telemetry.y,integral=0,descending=false}
        message="Height captured. N: descend up to 3 blocks."
      else message="Need HOLD, hover 2s, power 1..99%, Y>=ground+2." end
    elseif k==keys.n then
      if altitude then
        if altitude.descending then
          altitude.target=telemetry.y; altitude.descending=false
          message="Descent paused: hold current height."
        else
          altitude.target=telemetry.y
          altitude.floor=math.max(launchY+2,telemetry.y-3)
          altitude.descending=altitude.target>altitude.floor
          message="Descend 0.3 block/s; stop at target, NOT ground."
        end
      else message="Enable V height hold before N descent." end
    elseif k==keys.h then
      if mode=="HOLD" then cancelAltitude(); mode,pitch,yaw="MANUAL",0,0; message="Manual; thrust unchanged."
      elseif matrix and telemetry and os.clock()-telemetry.time<=0.25 and telemetry.tilt<math.rad(10) and throttle>0 and throttle/calThrottle>=0.5 and throttle/calThrottle<=2 then
        mode="HOLD"; message="Upright hold. No altitude/position control."
      else message="Need calibration and thrust within 0.5-2x calibration." end
    elseif k==keys.k then
      if mode=="MANUAL" and basis and telemetry and os.clock()-telemetry.time<=0.25 and throttle>0 and telemetry.tilt and telemetry.tilt<math.rad(5) and norm(telemetry.w)<0.05 then
        matrix,calThrottle,calMass=nil,nil,nil
        mode="CALIBRATE"; cal={stage=1,samples={},mass=telemetry.mass}
        stageStart(telemetry); message="Calibration pulses: keep craft clear of supports."
      else message="K needs UP, manual, thrust>0, tilt<5, low rotation." end
    elseif k==keys.r or k==keys.f or k==keys.t or k==keys.g then
      cancelAltitude()
      local step=(k==keys.t or k==keys.g) and 0.015 or 0.15
      throttle=clamp(throttle+((k==keys.r or k==keys.t) and step or -step),0,15)
      if throttle<1e-9 then throttle=0; epoch=epoch+1 end
      if mode=="HOLD" and (throttle==0 or throttle/calThrottle<0.5 or throttle/calThrottle>2) then
        mode,pitch,yaw="MANUAL",0,0; message="Outside calibrated thrust range; manual."
      end
    elseif mode=="MANUAL" then
      if k==keys.w then pitch=clamp(pitch+1,-3,3)
      elseif k==keys.s then pitch=clamp(pitch-1,-3,3)
      elseif k==keys.d then yaw=clamp(yaw+1,-3,3)
      elseif k==keys.a then yaw=clamp(yaw-1,-3,3) end
    end
  end
end
local function main()
  stop("Stand upright and still; press U to set UP.")
  local errors=clearEngines()
  assert(#errors==0,table.concat(errors,"; "))
  initializeEngines()
  local function sample()
    local ok,err=pcall(tick)
    if not ok then
      invalidate(tostring(err)); telemetry=nil; lastQ,lastTime,lastY,filteredVy=nil,nil,nil,nil
      status=nil -- Re-bootstrap after an API error, with outputs already zero.
    end
    if loadPending and telemetry then
      loadPending=false
      if not armed then
        local loaded,loadError=pcall(loadRecord)
        if not loaded then recordStatus="LOAD REJECTED: "..tostring(loadError) end
      end
    end
  end
  local function inputLoop()
    local repeats,lastPress={},{}
    for _,k in ipairs({keys.r,keys.f,keys.t,keys.g,keys.w,keys.s,keys.d,keys.a}) do repeats[k]=true end
    while running do
      local event,k,held=os.pullEventRaw()
      if event=="terminate" then stop("Terminated."); running=false; return
      elseif event=="peripheral_detach" then
        for _,name in ipairs(ENGINE_NAMES) do if k==name or k=="bottom" then failIO("Disconnected: "..k); break end end
      elseif event=="key_up" then lastPress[k]=nil
      elseif event=="key" and (not held or (repeats[k] and os.clock()-(lastPress[k] or -math.huge)>=0.12)) then
        lastPress[k]=os.clock()
        local keyOK,keyError=pcall(handleKey,k)
        if not keyOK then invalidate(tostring(keyError)) end
        if running then apply(); draw() end
      end
    end
  end
  local function sampleLoop()
    while running do
      sample(); draw()
      local timer=os.startTimer(0.1)
      repeat local event,id=os.pullEventRaw(); if event=="timer" and id==timer then break end until not running
    end
  end
  local function watchdog()
    while running do
      local oldMessage=message
      for _,e in ipairs(engines) do
        if not peripheral.isPresent(e.name) then failIO("Disconnected: "..e.name) end
        if armed and os.clock()-e.lastAck>0.6 then failIO("Output timeout: "..e.name) end
      end
      if armed and (not telemetry or os.clock()-telemetry.time>0.5) then
        stop("Telemetry stale >0.5s; stopped.")
      end
      if message~=oldMessage then draw() end
      local timer=os.startTimer(0.1)
      repeat local event,id=os.pullEventRaw(); if event=="timer" and id==timer then break end until not running
    end
  end
  -- Each coroutine gets every event: server-thread API waits cannot eat keys.
  local tasks={inputLoop,sampleLoop,watchdog}
  for _,e in ipairs(engines) do tasks[#tasks+1]=function() writer(e) end end
  draw(); parallel.waitForAny(table.unpack(tasks))
end
local ok,err=pcall(main)
stop("Exiting; clearing all five engines.")
local errors=clearEngines()
term.clear(); term.setCursorPos(1,1)
print(#errors==0 and "Stopped. Five engines zero; override kept at zero." or "CLEAR FAILED: "..table.concat(errors,"; "))
if not ok then error(err,0) end
