-- v0.5.5 short calibration probes with measured-response braking.
-- Return holds current altitude. No terrain sensing, auto-landing or tower catch.
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
function M.padDistance(pad,position)
  checked(pad); checked(position)
  return math.sqrt((pad.x-position.x)^2+(pad.z-position.z)^2)
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
function M.error(q,basis,worldUp)
  local target=M.rotate(conjugate(q),worldUp or vec(0,1,0))
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
function M.command(matrix,e1,e2,w1,w2,ratio,state,dt)
  state=state or {i1=0,i2=0,u=0,v=0}
  dt=dt or 0.1
  -- Lower bandwidth and anticipatory braking leave margin for server/nozzle lag.
  local x,y=0.65*(e1-0.2*w1)-1.3*w1,0.65*(e2-0.2*w2)-1.3*w2
  -- Small bounded bias compensation reduces the steady error from 1/15 steps.
  if math.abs(e1)<math.rad(8) and math.abs(e2)<math.rad(8) then
    state.i1=clamp(state.i1+0.06*e1*dt,-0.025,0.025)
    state.i2=clamp(state.i2+0.06*e2*dt,-0.025,0.025)
  else state.i1,state.i2=0,0 end
  x,y=x+state.i1,y+state.i2
  local u=(matrix.d*x-matrix.b*y)/matrix.det/ratio
  local v=(-matrix.c*x+matrix.a*y)/matrix.det/ratio
  local function quantize(n,previous,balance)
    n=clamp(n,-2,2)
    if dt<=0 then
      -- Late sample: update coarse proportional/damping correction, but do not
      -- accumulate pulse time or integral through the uncertain interval.
      local rounded=(n<0 and -1 or 1)*math.floor(math.abs(n)+0.5)
      return clamp(rounded,previous-1,previous+1),balance or 0
    end
    -- Integrate fractional demand in step-seconds. A small persistent demand
    -- produces occasional real one-step pulses instead of rounding to zero forever.
    local accumulated=clamp((balance or 0)+n*dt,-2*dt,2*dt)
    local equivalent=accumulated/dt
    local rounded=(equivalent<0 and -1 or 1)*math.floor(math.abs(equivalent)+0.5)
    local output=clamp(rounded,previous-1,previous+1)
    return output,clamp(accumulated-output*dt,-0.2,0.2)
  end
  state.u,state.bu=quantize(u,state.u,state.bu)
  state.v,state.bv=quantize(v,state.v,state.bv)
  return state.u,state.v
end
function M.navigation(target,p,v,state,dt)
  local ex,ez=target.x-p.x,target.z-p.z
  local distance=math.sqrt(ex*ex+ez*ez)
  -- Slow approach, earlier braking near the target. No integral during transit.
  local speed=math.min(0.6,0.08*distance)
  local vx,vz=0,0
  if distance>1e-6 then vx,vz=ex/distance*speed,ez/distance*speed end
  local sx,sz=0.045*(vx-v.x),0.045*(vz-v.z)
  local maxSlope=math.tan(math.rad(4))
  local magnitude=math.sqrt(sx*sx+sz*sz)
  if magnitude>maxSlope then sx,sz=sx*maxSlope/magnitude,sz*maxSlope/magnitude end
  -- Limit target tilt slew to avoid stepping the attitude controller.
  local dx,dz=sx-state.x,sz-state.z
  local change=math.sqrt(dx*dx+dz*dz); local limit=math.rad(1.5)*dt
  if change>limit then dx,dz=dx*limit/change,dz*limit/change end
  state.x,state.z=state.x+dx,state.z+dz
  return unit(vec(state.x,1,state.z)),distance
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
  local targetV=clamp(0.25*errorY+feed,-0.4,0.4)
  local errorV=targetV-vy
  local lo=math.max(0.015,calibrationThrottle*0.5,a.hover-0.6)
  local hi=math.min(15,calibrationThrottle*2,a.hover+0.6)
  local base=a.hover/math.max(0.9,math.cos(tilt))
  local raw=base+0.22*errorV+a.integral
  -- Conditional integration prevents windup at either throttle boundary.
  if (raw>=lo and raw<=hi) or (raw<lo and errorV>0) or (raw>hi and errorV<0) then
    a.integral=clamp(a.integral+0.025*errorV*dt,-0.3,0.3)
  end
  raw=base+0.22*errorV+a.integral
  -- Limit normal automatic changes to 2 percentage points per second.
  -- Emergency stop bypasses this limiter and still goes directly to zero.
  local previous=a.output or a.hover
  local output=clamp(clamp(raw,lo,hi),previous-0.3*dt,previous+0.3*dt)
  a.output=output
  a.saturated=raw<lo or raw>hi or math.abs(output-clamp(raw,lo,hi))>1e-9
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
local pad,nav=nil,nil
local padCapturedAt=nil
local infoPage=false
local buttons={}
local lastPosition,filteredHorizontal=nil,nil
local attitude={i1=0,i2=0,u=0,v=0}
local recovery=nil
local lateSamples={}
local timingLate=false
local LOG_RESERVE,LOG_LIMIT=65536,131072
local flightLog={queue={},status="OFF",rows=0,bytes=0,dropped=0,lastSample=-math.huge}
local function logSpace(bytes)
  if not fs.getFreeSpace then return true end
  local free=fs.getFreeSpace("/")
  if type(free)=="number" and free<LOG_RESERVE+bytes then
    flightLog.status="FULL"; flightLog.error="Low disk space; archive logs. No files deleted."
    return false
  end
  return true
end
local function csv(value)
  if value==nil then return '""' end
  return '"'..tostring(value):gsub('"','""'):gsub('[\r\n]',' ')..'"'
end
local function trace(kind,detail)
  if flightLog.status~="ON" then return end
  if #flightLog.queue>=200 then flightLog.dropped=flightLog.dropped+1; return end
  local t=telemetry or {}; local p=t.position or {}; local v=t.horizontal or {}
  local q=t.q or {}; local w=t.w or {}
  local values={kind,os.clock(),mode,armed and 1 or 0,throttle/15*100,pitch,yaw,
    t.dt or "",p.x or "",t.y or "",p.z or "",v.x or "",t.vy or "",v.z or "",
    t.tilt and math.deg(t.tilt) or "",w.x or "",w.y or "",w.z or "",
    q.x or "",q.y or "",q.z or "",q.w or "",t.mass or "",
    altitude and altitude.target or "",nav and nav.target.x or "",nav and nav.target.z or "",
    flightLog.dropped,detail or ""}
  for _,e in ipairs(engines) do
    values[#values+1]=e.output; values[#values+1]=e.gate
    values[#values+1]=os.clock()-e.lastAck
    values[#values+1]=e.x; values[#values+1]=e.y
  end
  values[#values+1]=pad and pad.x or ""
  values[#values+1]=pad and pad.y or ""
  values[#values+1]=pad and pad.z or ""
  values[#values+1]=pad and t.position and M.padDistance(pad,t.position) or ""
  values[#values+1]=padCapturedAt or ""
  for i,value in ipairs(values) do values[i]=csv(value) end
  flightLog.queue[#flightLog.queue+1]=table.concat(values,",").."\n"
end
local function startLog()
  local ok,err=pcall(function()
    if not logSpace(2048) then return end
    local index=1
    repeat flightLog.path=("/rocket-flight-%04d.csv"):format(index); index=index+1
    until not fs.exists(flightLog.path) or index>10000
    assert(not fs.exists(flightLog.path),"Log slots full")
    local handle,openError=fs.open(flightLog.path,"w"); assert(handle,openError)
    local header="event,time_s,mode,armed,power_pct,vector_x_step,vector_y_step,dt_s,x,y,z,vx,vy,vz,tilt_deg,wx,wy,wz,qx,qy,qz,qw,mass,target_y,target_x,target_z,dropped,detail"
    for i=0,4 do header=header..(",engine%d_base_command,engine%d_gate_command,engine%d_ack_age_s,engine%d_vector_x,engine%d_vector_y"):format(i,i,i,i,i) end
    header=header..",pad_x,pad_y,pad_z,pad_horizontal_distance,pad_captured_at_s"
    local written,writeError=pcall(handle.write,header.."\n"); handle.close(); assert(written,writeError)
    flightLog.bytes=#header+1
    flightLog.status="ON"
  end)
  if not ok then flightLog.status="ERR"; flightLog.error=tostring(err) end
end
local function flushLog()
  if flightLog.status~="ON" or #flightLog.queue==0 then return end
  local batch=flightLog.queue; flightLog.queue={}
  local ok,err=pcall(function()
    local content=table.concat(batch)
    if flightLog.bytes+#content>LOG_LIMIT then
      flightLog.status="FULL"; flightLog.error="Session log reached 128 KiB; flight continues."
      return
    end
    if not logSpace(#content) then return end
    local handle,openError=fs.open(flightLog.path,"a"); assert(handle,openError)
    local written,writeError=pcall(handle.write,content); handle.close(); assert(written,writeError)
    flightLog.bytes=flightLog.bytes+#content
  end)
  if not ok then flightLog.status="ERR"; flightLog.error=tostring(err); return end
  flightLog.rows=flightLog.rows+#batch
  if flightLog.rows>=20000 then flightLog.status="FULL" end
end
local function resetAttitude() attitude={i1=0,i2=0,u=0,v=0} end
local function cancelAltitude()
  altitude,nav=nil,nil; hoverReadySince,readyThrottle=nil,nil
  resetAttitude()
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
  trace("STOP",reason)
  cancelAltitude()
  armed,throttle,pitch,yaw=false,0,0,0
  mode,cal="MANUAL",nil
  recovery=nil
  lateSamples={}; timingLate=false
  resetAttitude()
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
  if not fs.exists(path) then
    recordStatus="No saved calibration"
    message="L: no calibration file on this computer. Restore it, or U then K for new calibration."
    return
  end
  assert(fs.getSize(path)<=16384,"Calibration file too large")
  local h,err=fs.open(path,"r"); assert(h,err)
  local data=h.readAll(); h.close()
  local r=textutils.unserialize(data)
  local b,m=M.validateRecord(r,telemetry.id,telemetry.mass)
  basis,matrix,referenceId,calMass,calThrottle=b,m,r.id,r.mass,r.throttle
  telemetry.e1,telemetry.e2,telemetry.tilt=M.error(telemetry.q,basis)
  launchY,pad=nil,nil -- Never reuse a previous flight's world-height or pad reference.
  padCapturedAt=nil
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
  local measuredHorizontal=lastPosition and dt and dt>0 and mul(add(position,mul(lastPosition,-1)),1/dt) or vec(0,0,0)
  measuredHorizontal.y=0
  local lateralAlpha=dt and clamp(dt/(0.15+dt),0,1) or 1
  filteredHorizontal=filteredHorizontal and add(filteredHorizontal,mul(add(measuredHorizontal,mul(filteredHorizontal,-1)),lateralAlpha)) or measuredHorizontal
  t.position,t.horizontal,t.rawHorizontal=position,filteredHorizontal,measuredHorizontal
  lastPosition=position
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
local function stageStart(t)
  cal.start=t.time; cal.phase="NEUTRAL"; pitch,yaw=0,0
end
local function calibrationTick(t)
  assert(t.tilt<math.rad(12),"Calibration tilt exceeded 12 degrees")
  assert(norm(t.w)<0.6,"Calibration rotation too fast")
  assert(math.abs(t.mass/cal.mass-1)<=0.05,"Mass changed during calibration")
  assert(t.time-cal.start<24,"Calibration timeout; no record saved")
  local a,b=rates(t)
  local function command(n)
    if cal.axis==1 then pitch,yaw=n,0 else yaw=n end
  end
  if cal.phase=="NEUTRAL" then
    if t.time-cal.start<0.6 then return end
    assert(norm(t.w)<0.03,"Calibration needs still rotation before probe")
    cal.axis=1; cal.columns={}; cal.phase="BEGIN"
  end
  if cal.phase=="BEGIN" then
    cal.axisStart=t.time; cal.wa,cal.wb=a,b
    cal.q,cal.area,cal.sumS,cal.sumSS,cal.sumA,cal.sumB,cal.sumSA,cal.sumSB,cal.n=0,0,0,0,0,0,0,0,0
    cal.previousCommand=0; cal.balance=0; cal.settled=nil; cal.reverseSeen=false
    cal.sumSquares=0
    cal.firstQ,cal.firstArea,cal.previousFirst,cal.firstBalance=0,0,0,0
    cal.phase="PROBE"; command(1); return
  end
  -- Model the installed nozzle's 20%-per-tick tween. Account for ~one tick
  -- transport delay; rates from pose differences represent interval averages.
  -- Regress angular rate against integrated nozzle input, NOT raw command/dt.
  local dt=t.dt
  local target=cal.axis==1 and pitch or yaw
  local delay=math.min(0.05,dt)
  local tau=-0.05/math.log(0.8)
  local oldArea=cal.area
  local function integrate(input,duration)
    local decay=math.exp(-duration/tau)
    cal.area=cal.area+input*duration+(cal.q-input)*tau*(1-decay)
    cal.q=input+(cal.q-input)*decay
  end
  integrate(cal.previousCommand,delay); integrate(target,dt-delay)
  cal.previousCommand=target
  local s=(oldArea+cal.area)/2
  local observedA,observedB=a,b
  if cal.axis==2 then
    local oldFirst=cal.firstArea
    for _,part in ipairs({{cal.previousFirst,delay},{pitch,dt-delay}}) do
      local input,duration=part[1],part[2]; local decay=math.exp(-duration/tau)
      cal.firstArea=cal.firstArea+input*duration+(cal.firstQ-input)*tau*(1-decay)
      cal.firstQ=input+(cal.firstQ-input)*decay
    end
    cal.previousFirst=pitch
    local area=(oldFirst+cal.firstArea)/2
    observedA,observedB=a-cal.columns[1][1]*area,b-cal.columns[1][2]*area
  end
  cal.n=cal.n+1; cal.sumS=cal.sumS+s; cal.sumSS=cal.sumSS+s*s
  cal.sumA=cal.sumA+observedA; cal.sumB=cal.sumB+observedB
  cal.sumSA=cal.sumSA+s*observedA; cal.sumSB=cal.sumSB+s*observedB
  cal.sumSquares=cal.sumSquares+observedA*observedA+observedB*observedB
  local variance=cal.sumSS-cal.sumS*cal.sumS/cal.n
  local ca,cb
  if cal.n>=3 and variance>0.0001 then
    ca=(cal.sumSA-cal.sumS*cal.sumA/cal.n)/variance
    cb=(cal.sumSB-cal.sumS*cal.sumB/cal.n)/variance
  end
  if cal.phase=="PROBE" then
    local delta=math.sqrt((a-cal.wa)^2+(b-cal.wb)^2)
    if t.time-cal.axisStart>=0.2 and (delta>=0.018 or t.time-cal.axisStart>=0.4) then
      assert(delta>0.002,"Probe response too weak; check free craft/thrust")
      local scale=math.max(math.abs(s),0.02)
      cal.ca,cal.cb=(a-cal.wa)/scale,(b-cal.wb)/scale
      cal.phase="BRAKE"; command(-1); cal.reverseSeen=true
    end
    return
  end
  if ca and ca*cal.ca+cb*cal.cb>0 then cal.ca,cal.cb=ca,cb end
  ca,cb=cal.ca,cal.cb
  local strength=math.sqrt(ca*ca+cb*cb)
  assert(strength>0.005 and strength<5,"Probe response outside calibration range")
  -- Project UP error/rate along the measured response. Anticipate residual
  -- nozzle force so braking starts before the rotation crosses zero.
  local errorAlong=(t.e1*ca+t.e2*cb)/strength
  local rateAlong=(a*ca+b*cb)/strength
  local demand=(0.8*errorAlong-1.8*rateAlong)/strength-0.35*cal.q
  if cal.axis==2 then
    local c=cal.columns[1]; local det=c[1]*cb-ca*c[2]
    assert(math.abs(det)>0.0001,"Calibration axes overlap")
    local x,y=0.8*t.e1-1.8*a,0.8*t.e2-1.8*b
    local first=(cb*x-ca*y)/det-0.35*cal.firstQ
    demand=(-c[2]*x+c[1]*y)/det-0.35*cal.q
    cal.firstBalance=clamp(cal.firstBalance+clamp(first,-1,1)*dt,-dt,dt)
    pitch=math.floor(math.abs(cal.firstBalance/dt)+0.5)*(cal.firstBalance<0 and -1 or 1)
    cal.firstBalance=cal.firstBalance-pitch*dt
  end
  cal.balance=clamp(cal.balance+clamp(demand,-1,1)*dt,-dt,dt)
  local pulse=math.floor(math.abs(cal.balance/dt)+0.5)*(cal.balance<0 and -1 or 1)
  cal.balance=cal.balance-pulse*dt; command(pulse)
  if math.abs(errorAlong)<math.rad(1) and norm(t.w)<0.012 and math.abs(cal.q)<0.3 then
    -- Do not generate another minimum-size pulse when already almost still.
    command(0); cal.balance=0
    if cal.axis==2 and math.abs(cal.firstQ)<0.3 then pitch=0; cal.firstBalance=0 end
  end
  assert(t.time-cal.axisStart<10,"Calibration axis did not settle; no record saved")
  if math.abs(errorAlong)<math.rad(1) and norm(t.w)<0.018 and math.abs(cal.q)<0.15 and t.time-cal.axisStart>1.2 then
    cal.settled=cal.settled or t.time
  else cal.settled=nil end
  message=("CAL axis %d BRAKE/SETTLE | tilt %.1f | rate %.3f"):format(cal.axis,math.deg(t.tilt),norm(t.w))
  if not cal.settled or t.time-cal.settled<0.45 then return end
  assert(ca and variance>0.0005 and cal.reverseSeen,"Insufficient calibration excitation")
  local observedVariance=cal.sumSquares-(cal.sumA*cal.sumA+cal.sumB*cal.sumB)/cal.n
  local fit=(ca*ca+cb*cb)*variance/math.max(observedVariance,1e-12)
  assert(fit>=0.75 and fit<=1.05,"Calibration response inconsistent; no record saved")
  cal.columns[cal.axis]={ca,cb}; command(0)
  if cal.axis==1 then cal.axis=2; cal.phase="BEGIN"; return end
  matrix=M.matrix(cal.columns); calThrottle,calMass=throttle,t.mass
  mode,cal,pitch,yaw="MANUAL",nil,0,0
  message="Calibrated. Thrust unchanged. H enables HOLD."
  trySave()
end
local function beginRecovery(t)
  -- Freeze the current thrust, discard navigation/height integrators. Trying to
  -- maintain altitude with a large tilt can amplify lateral acceleration.
  cancelAltitude()
  mode="RECOVER"
  recovery={deadline=t.time+30}
  message="Recovering upright; thrust held. No altitude hold."
end
local function tick()
  telemetry=readTelemetry()
  local t=telemetry
  if basis and t.id~=referenceId then
    basis,pad,launchY,padCapturedAt=nil,nil,nil,nil; invalidate("Craft changed. Set UP and calibrate again."); return
  end
  if matrix and math.abs(t.mass/calMass-1)>0.05 then
    invalidate("Mass changed >5%. Recalibrate."); return
  end
  if mode~="MANUAL" then
    assert(t.dt and t.dt>=0.025 and t.dt<=0.5+1e-6,
      ("Sample dt=%s outside 0.025-0.500s; stopped, calibration retained"):format(t.dt and ("%.3fs"):format(t.dt) or "missing"))
    assert(t.tilt<math.rad(70),"Outside recovery envelope: tilt >=70deg; stopped")
    assert(norm(t.w)<1.5,"Rotation too fast; test stopped")
  end
  timingLate=t.dt and t.dt>0.35+1e-6 or false
  if timingLate and mode~="MANUAL" then
    trace("LATE_SAMPLE",t.dt)
    if mode=="CALIBRATE" then stop("Calibration interrupted by lag; retry K."); return end
    local recent={}
    for _,time in ipairs(lateSamples) do if t.time-time<=5 then recent[#recent+1]=time end end
    recent[#recent+1]=t.time; lateSamples=recent
    if #recent>=3 then stop("Repeated lag: 3 late samples/5s; cal retained."); return end
    hoverReadySince,readyThrottle=nil,nil
    if recovery then recovery.settledSince=nil end
    if nav then nav.arrivedSince,nav.arrived=nil,false end
  end
  if mode=="HOLD" and t.tilt>=math.rad(15) then beginRecovery(t) end
  if mode=="RECOVER" then
    assert(t.time<=recovery.deadline,"Recovery timeout >30s; stopped")
    if not timingLate and t.tilt<math.rad(6) and norm(t.w)<0.08 then
      recovery.settledSince=recovery.settledSince or t.time
      if t.time-recovery.settledSince>=2 then
        recovery=nil; resetAttitude(); mode="HOLD"
        message="Upright recovered. Adjust hover, then V/B manually."
      end
    else recovery.settledSince=nil end
  end
  if not timingLate and mode=="HOLD" and not altitude and math.abs(t.vy)<0.4 and t.tilt<math.rad(8) then
    if readyThrottle~=throttle or not hoverReadySince then hoverReadySince=t.time; readyThrottle=throttle end
  elseif not altitude then hoverReadySince,readyThrottle=nil,nil end
  if altitude then
    if mode~="HOLD" then cancelAltitude()
    else
      assert(math.abs(t.rawVy)<4,"Vertical speed >4 blocks/s; test stopped")
      if not timingLate then throttle=M.heightStep(altitude,t.y,t.vy,t.dt,t.tilt,calThrottle) end
    end
  end
  local targetUp=vec(0,1,0)
  if nav then
    if not altitude or mode~="HOLD" then nav=nil
    elseif norm(t.rawHorizontal)>3 or (nav.returning and ((not nav.completed and t.time>nav.deadline) or not pad or t.y<pad.y+8)) then
      nav=nil; resetAttitude(); message="Position/return aborted; upright height hold."
    else
      targetUp,nav.distance=M.navigation(nav.target,t.position,t.horizontal,nav.slope,timingLate and 0 or t.dt)
      if nav.returning then
        if not timingLate and nav.distance<1.5 and norm(t.horizontal)<0.2 then
          nav.arrivedSince=nav.arrivedSince or t.time
          nav.arrived=t.time-nav.arrivedSince>=3
          if nav.arrived then nav.completed=true end
        else nav.arrivedSince,nav.arrived=nil,false end
      end
    end
  end
  if mode=="CALIBRATE" then calibrationTick(t)
  elseif mode=="HOLD" or mode=="RECOVER" then
    local a,b=rates(t)
    local e1,e2=M.error(t.q,basis,targetUp)
    if mode=="RECOVER" then
      -- Cap the attitude-error demand to brake rotation before approaching UP.
      local length=math.sqrt(e1*e1+e2*e2)
      local limit=math.rad(20)
      if length>limit then e1,e2=e1*limit/length,e2*limit/length end
    end
    pitch,yaw=M.command(matrix,e1,e2,a,b,throttle/calThrottle,attitude,timingLate and 0 or t.dt)
  end
  apply()
end
local function drawText()
  term.clear(); term.setCursorPos(1,1)
  print("ROCKET v0.5.5 HOME | 5 x 733 kN")
  print((armed and "ARMED " or "DISARMED ")..mode..(altitude and (altitude.descending and " DESCEND" or " ALT") or "")..(nav and (nav.returning and (nav.arrived and " HOME" or " RETURN") or " POS") or ""))
  print(("Power %.1f%% | base %.2f kN/engine"):format(throttle/15*100,FULL_THRUST_KN*throttle/15))
  if telemetry then
    print(("Y %.2f | vertical speed %+.2f"):format(telemetry.y,telemetry.vy))
    print(("Tilt %s | calibration %s"):format(telemetry.tilt and ("%.1f deg"):format(math.deg(telemetry.tilt)) or "unset",matrix and "OK" or "needed"))
  else print("Telemetry unavailable") end
  print(("Sample %s | %s | log %s"):format(lastInterval and ("%.3fs"):format(lastInterval) or "warmup",timingLate and "LATE" or "OK",flightLog.status))
  print("U New UP+pad | J Set pad (disarmed)")
  print("E Arm | R/F +/-1% | T/G +/-0.1%")
  print(("W/S D/A trim: X%+d Y%+d | C reset"):format(pitch,yaw))
  print("K Short probe + brake; thrust unchanged")
  print("H Upright | V Position+height hold")
  print("B Return above pad / hold here")
  print("N Descend 3 blocks (cancels return)")
  if altitude then
    print(("Target Y %.2f | %s"):format(altitude.target,altitude.saturated and "THRUST LIMIT" or "height control"))
  else print("P Save / L Load (disarmed)") end
  print("SPACE stop | Q exit | no ground sensing")
  if pad and telemetry then
    print(("Pad distance %.1f | height above pad %+.1f"):format(M.padDistance(pad,telemetry.position),telemetry.y-pad.y))
    print(("Pad XYZ %.2f %.2f %.2f"):format(pad.x,pad.y,pad.z))
    print(("Now XYZ %.2f %.2f %.2f"):format(telemetry.position.x,telemetry.y,telemetry.position.z))
  else print("Pad NOT SET: J at launch position") end
  if nav then print(("Navigation target distance %.1f"):format(nav.distance or 0)) end
  if flightLog.error then print("Log: "..flightLog.error) end
  print(recordStatus)
  print(message)
end
-- Native terminal buttons use the same key handler and safety checks as keys.
-- No external libraries, monitor or mouse is required; keyboard remains active.
local function draw()
  buttons={}
  if not term.getSize or not term.write then return drawText() end
  local width,height=term.getSize()
  local function colour(fg,bg)
    if term.setTextColor then term.setTextColor(fg or 1) end
    if term.setBackgroundColor then term.setBackgroundColor(bg or 32768) end
  end
  local function line(y,s,fg,bg)
    if y>height then return end
    colour(fg,bg); term.setCursorPos(1,y)
    term.write((s..string.rep(" ",width)):sub(1,width))
  end
  colour(); term.clear()
  if width<51 or height<19 then
    line(1,"ROCKET: resize terminal to at least 51 x 19")
    line(2,(armed and "ARMED " or "DISARMED ")..mode)
    line(3,"SPACE: engine stop | Q: exit | keys still work")
    line(4,message); return
  end
  line(1," ROCKET v0.5.5 | 5 x 733 kN | I: INFO",1,2048)
  local state=(armed and "ARMED " or "DISARMED ")..mode..(altitude and (altitude.descending and " DESCEND" or " ALT") or "")..(nav and (nav.returning and (nav.arrived and " HOME" or " RETURN") or " POS") or "")
  line(2,state,armed and 32 or 256)
  local filled=math.floor(throttle/15*16+0.5)
  line(3,("Power %5.1f%% [%s%s] %s"):format(throttle/15*100,string.rep("#",filled),string.rep("-",16-filled),altitude and "AUTO" or "MANUAL"),16)
  local t=telemetry
  line(4,t and ("Y %+.2f | Vy %+.2f blocks/s"):format(t.y,t.vy) or "Telemetry unavailable")
  line(5,t and ("Tilt %s deg | horizontal speed %.2f"):format(t.tilt and ("%.1f"):format(math.deg(t.tilt)) or "?",norm(t.horizontal)) or "Waiting for telemetry")
  line(6,("Sample %s %s | log %s | Cal %s"):format(lastInterval and ("%.3fs"):format(lastInterval) or "?",timingLate and "LATE" or "OK",flightLog.status,matrix and "OK" or "NEEDED"))
  line(7,t and ("NOW X %.2f Y %.2f Z %.2f"):format(t.position.x,t.y,t.position.z) or "NOW --")
  line(8,pad and ("PAD X %.2f Y %.2f Z %.2f"):format(pad.x,pad.y,pad.z) or "Pad NOT SET: J on launch pad, disarmed",pad and 1 or 16)
  line(9,pad and t and ("PAD dist %.2f /64 | dY %+.2f /+12"):format(M.padDistance(pad,t.position),t.y-pad.y) or "PAD distance -- (not hover target)",512)
  line(10,nav and ("%s target %.2f | Y %.2f %s"):format(nav.returning and "RETURN" or "HOVER",nav.distance or 0,altitude.target,altitude.saturated and "LIMIT" or "AUTO") or "H: upright only | V: automatic hover thrust")
  local function button(x,y,label,key,bg)
    colour(1,bg or 128); term.setCursorPos(x,y); term.write(label)
    buttons[#buttons+1]={x=x,y=y,w=#label,key=key}
  end
  button(1,11,armed and " E OFF " or " E ARM ",keys.e,armed and 16384 or 8192)
  button(10,11," H LEVEL ",keys.h); button(21,11," V HOVER ",keys.v); button(32,11," B RETURN ",keys.b)
  button(1,12," F -1% ",keys.f); button(10,12," G -0.1% ",keys.g)
  button(21,12," T +0.1% ",keys.t); button(32,12," R +1% ",keys.r)
  button(1,13," C MAN ",keys.c); button(10,13," N DOWN ",keys.n)
  button(21,13," J PAD ",keys.j); button(32,13," I INFO ",keys.i)
  button(1,14," U UP ",keys.u); button(10,14," K CAL ",keys.k)
  button(21,14," P SAVE ",keys.p); button(32,14," L LOAD ",keys.l)
  button(1,15," SPACE STOP ENGINES ",keys.space,16384); button(32,15," Q EXIT ",keys.q,16384)
  line(16,infoPage and recordStatus or "W/S D/A: trim | no terrain / auto ignition",256)
  line(17,infoPage and (flightLog.error or recordStatus) or ("Log "..(flightLog.path or "not created").." | I: details"),flightLog.error and 16 or 256)
  -- The bottom two rows are reserved for messages, never scrolled off screen.
  line(18,message:sub(1,width),16); line(19,message:sub(width+1,2*width),16)
  colour()
end
local function handleKey(k)
  if k==keys.i then infoPage=not infoPage; return end
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
  if not armed and k==keys.p then trySave(); message=recordStatus; return end
  if not armed and k==keys.l then
    local ok,err=pcall(loadRecord)
    if not ok then
      recordStatus="LOAD REJECTED: "..tostring(err):gsub("^.-:%d+: ","")
      message=recordStatus
    end
    return
  end
  if k==keys.j then
    local t=telemetry
    if armed then message="J blocked: ARMED. Land/support craft, then E to disarm."; return end
    if not basis then message="J blocked: UP reference missing (not a tilt fault). L to load, or U to set UP."; return end
    if not t or not t.dt or not t.tilt or os.clock()-t.time>0.25 then
      message="J blocked: wait for fresh telemetry and a tilt reading."; return
    end
    if t.tilt>=math.rad(5) then message=("J blocked: tilt %.2f deg; need <5. Verify saved UP."):format(math.deg(t.tilt)); return end
    if math.abs(t.vy)>=0.2 then message=("J blocked: vertical speed %+.2f; need abs <0.2."):format(t.vy); return end
    if norm(t.w)>=0.03 then message=("J blocked: rotating %.3f rad/s; need <0.030."):format(norm(t.w)); return end
    if norm(t.horizontal)>=0.2 then message=("J blocked: horizontal speed %.2f; need <0.2."):format(norm(t.horizontal)); return end
    launchY,pad=t.y,checked(t.position); padCapturedAt=t.time
    trace("PAD_SET","J: launch reference captured")
    message="Pad XYZ captured; calibration retained."
  elseif k==keys.u and not armed then
    local t=telemetry
    assert(t and os.clock()-t.time<=0.25 and t.dt and t.dt>0 and t.velocityAge<2 and norm(t.w)<0.03 and norm(t.velocity)<0.2,"Set UP only while still with fresh telemetry")
    basis=M.basis(t.q); referenceId=t.id; launchY,pad=t.y,checked(t.position)
    padCapturedAt=t.time; trace("PAD_SET","U: launch reference captured")
    matrix,calThrottle,calMass=nil,nil,nil
    message="UP set. Next: manual test, then K calibration."
  elseif armed then
    if k==keys.c then cancelAltitude(); recovery=nil; mode,cal,pitch,yaw="MANUAL",nil,0,0; message="Manual; thrust unchanged."
    elseif mode=="CALIBRATE" then message="Calibrating. C cancels; SPACE stops."
    elseif k==keys.b then
      if nav and nav.returning then
        nav={target=checked(telemetry.position),slope={x=0,z=0},returning=false}
        resetAttitude(); message="Return cancelled; hold current position."
      elseif not timingLate and altitude and mode=="HOLD" and pad and telemetry and os.clock()-telemetry.time<=0.25 and telemetry.y>=pad.y+12 and telemetry.tilt<math.rad(8) and norm(telemetry.horizontal)<1 then
        local distance=M.padDistance(pad,telemetry.position)
        if distance<=64 then
          altitude.target=telemetry.y; altitude.descending=false
          nav={target=checked(pad),slope={x=0,z=0},returning=true,deadline=telemetry.time+180}
          resetAttitude(); message="Return above pad at this height. No obstacle avoidance."
        else message=("B blocked: PAD distance %.2f >64. Check PAD/NOW XYZ; hover distance is different."):format(distance) end
      else
        local t=telemetry
        if not pad then message="B blocked: pad unset. J on launch pad, disarmed."
        elseif not altitude or mode~="HOLD" then message="B blocked: enable V first; need HOLD ALT POS."
        elseif timingLate or not t or os.clock()-t.time>0.25 then message="B blocked: wait for fresh, on-time telemetry."
        elseif t.y<pad.y+12 then message=("B blocked: height above PAD %.2f <12 blocks."):format(t.y-pad.y)
        elseif t.tilt>=math.rad(8) then message="B blocked: tilt must be <8 degrees."
        else message=("B blocked: horizontal speed %.2f must be <1."):format(norm(t.horizontal)) end
      end
    elseif k==keys.v then
      if altitude then cancelAltitude(); message="Altitude off; current thrust retained."
      elseif not timingLate and mode=="HOLD" and launchY and telemetry and telemetry.y>=launchY+2 and os.clock()-telemetry.time<=0.25 and hoverReadySince and telemetry.time-hoverReadySince>=2 and throttle>=0.15 and throttle<=14.85 and norm(telemetry.horizontal)<2 then
        altitude={hover=throttle,target=telemetry.y,floor=telemetry.y,integral=0,descending=false}
        nav={target=checked(telemetry.position),slope={x=0,z=0},returning=false}
        resetAttitude(); message="Position + height captured. B: return; N: descend."
      else message="Need HOLD, hover 2s, horizontal speed<2, Y>=pad+2." end
    elseif k==keys.n then
      if altitude then
        nav={target=checked(telemetry.position),slope={x=0,z=0},returning=false}
        resetAttitude()
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
      if mode=="HOLD" or mode=="RECOVER" then cancelAltitude(); recovery=nil; mode,pitch,yaw="MANUAL",0,0; message="Manual; thrust unchanged."
      elseif matrix and telemetry and os.clock()-telemetry.time<=0.25 and telemetry.tilt<math.rad(70) and norm(telemetry.w)<1.5 and throttle>0 and throttle/calThrottle>=0.5 and throttle/calThrottle<=2 then
        if telemetry.tilt>=math.rad(15) then beginRecovery(telemetry)
        else resetAttitude(); mode="HOLD"; message="Damped upright hold. V adds position + height hold." end
      else message="Need calibration, tilt<70, thrust 0.5-2x cal." end
    elseif k==keys.k then
      if mode=="MANUAL" and basis and telemetry and os.clock()-telemetry.time<=0.25 and throttle>0 and telemetry.tilt and telemetry.tilt<math.rad(5) and norm(telemetry.w)<0.05 then
        matrix,calThrottle,calMass=nil,nil,nil
        mode="CALIBRATE"; cal={mass=telemetry.mass}
        stageStart(telemetry); message="CAL short probe + brake. Keep clear; C cancels, SPACE stops."
      else message="K needs UP, manual, thrust>0, tilt<5, low rotation." end
    elseif k==keys.r or k==keys.f or k==keys.t or k==keys.g then
      cancelAltitude()
      local step=(k==keys.t or k==keys.g) and 0.015 or 0.15
      throttle=clamp(throttle+((k==keys.r or k==keys.t) and step or -step),0,15)
      if throttle<1e-9 then throttle=0; epoch=epoch+1 end
      if (mode=="HOLD" or mode=="RECOVER") and (throttle==0 or throttle/calThrottle<0.5 or throttle/calThrottle>2) then
        recovery=nil; mode,pitch,yaw="MANUAL",0,0; message="Outside calibrated thrust range; manual."
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
  startLog(); trace("START","v0.5.5; engine fields are acknowledged commands, not measured force")
  local function sample()
    local ok,err=pcall(tick)
    if not ok then
      stop(tostring(err)); telemetry=nil; lastQ,lastTime,lastY,filteredVy=nil,nil,nil,nil
      lastPosition,filteredHorizontal=nil,nil
      status=nil -- Re-bootstrap after an API error, with outputs already zero.
    end
    if os.clock()-flightLog.lastSample>=0.2 then
      flightLog.lastSample=os.clock(); trace("SAMPLE",message)
    end
    if loadPending and telemetry then
      loadPending=false
      if not armed then
        local loaded,loadError=pcall(loadRecord)
        if not loaded then
          recordStatus="LOAD REJECTED: "..tostring(loadError):gsub("^.-:%d+: ","")
          message=recordStatus
        end
      end
    end
  end
  local function inputLoop()
    local repeats,lastPress={},{}
    for _,k in ipairs({keys.r,keys.f,keys.t,keys.g,keys.w,keys.s,keys.d,keys.a}) do repeats[k]=true end
    while running do
      local event,k,held,mouseY=os.pullEventRaw()
      if event=="mouse_click" and k==1 then
        for _,b in ipairs(buttons) do
          if mouseY==b.y and held>=b.x and held<b.x+b.w then
            event,k,held="key",b.key,false; break
          end
        end
      end
      if event=="terminate" then stop("Terminated."); running=false; return
      elseif event=="term_resize" then draw()
      elseif event=="peripheral_detach" then
        for _,name in ipairs(ENGINE_NAMES) do if k==name or k=="bottom" then failIO("Disconnected: "..k); break end end
      elseif event=="key_up" then lastPress[k]=nil
      elseif event=="key" and (not held or (repeats[k] and os.clock()-(lastPress[k] or -math.huge)>=0.12)) then
        lastPress[k]=os.clock()
        local keyOK,keyError=pcall(handleKey,k)
        if not keyOK then stop(tostring(keyError)) end
        trace("KEY",k)
        trace("ACTION",message)
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
  local function logLoop()
    local due=0
    while running do
      -- Reuse existing timer events; do not add scheduling load to the sampler.
      os.pullEventRaw()
      if os.clock()>=due then flushLog(); due=os.clock()+0.5 end
    end
  end
  local tasks={inputLoop,sampleLoop,watchdog,logLoop}
  for _,e in ipairs(engines) do tasks[#tasks+1]=function() writer(e) end end
  draw(); parallel.waitForAny(table.unpack(tasks))
end
local ok,err=pcall(main)
stop("Exiting; clearing all five engines.")
local errors=clearEngines()
if #errors==0 then
  for _,e in ipairs(engines) do e.output,e.gate,e.x,e.y,e.lastAck=0,0,0,0,os.clock() end
end
trace("EXIT",#errors==0 and "Five engines zero" or table.concat(errors,"; ")); flushLog()
term.clear(); term.setCursorPos(1,1)
print(#errors==0 and "Stopped. Five engines zero; override kept at zero." or "CLEAR FAILED: "..table.concat(errors,"; "))
print("Flight log: "..(flightLog.path or "unavailable").." ["..flightLog.status.."]")
if flightLog.error then print("Log error: "..flightLog.error) end
if not ok then error(err,0) end
