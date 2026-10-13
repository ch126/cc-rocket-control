-- v0.2.1 experimental upright hold. Single-file CC: Tweaked program.
-- Keep engine.lua for manual testing. Run only one controller at a time.
local args = {...}
local M = {}
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
  assert(math.abs(det)/(n1*n2)>0.35,"Axes overlap: check four direction frequencies")
  return {a=a,b=b,c=c,d=d,det=det}
end
function M.command(matrix,e1,e2,w1,w2,ratio)
  local x,y=1.5*e1-1.8*w1,1.5*e2-1.8*w2
  local u=(matrix.d*x-matrix.b*y)/matrix.det/ratio
  local v=(-matrix.c*x+matrix.a*y)/matrix.det/ratio
  local function rounded(n) return math.floor(clamp(n,-3,3)+0.5) end
  return rounded(u),rounded(v)
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
local function clear()
  local errors={}
  for _,s in ipairs(faces) do
    local ok,err=pcall(redstone.setAnalogOutput,s,0)
    if not ok then errors[#errors+1]=s..": "..tostring(err) end
  end
  return errors
end
local function stop(reason)
  armed,throttle,pitch,yaw=false,0,0,0
  mode,cal="MANUAL",nil
  message=reason or "Stopped."
  local errors=clear(); assert(#errors==0,table.concat(errors,"; "))
end
local function pair(a,b,v)
  if v>0 then redstone.setAnalogOutput(b,0); redstone.setAnalogOutput(a,v)
  else redstone.setAnalogOutput(a,0); redstone.setAnalogOutput(b,-v) end
end
local function apply()
  if not armed then pitch,yaw,throttle=0,0,0 end
  pair("front","back",pitch); pair("right","left",yaw)
  redstone.setAnalogOutput("bottom",throttle)
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
  checked(pose.position)
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
local stages={{0,0},{2,0},{0,0},{-2,0},{0,0},{0,2},{0,0},{0,-2},{0,0}}
local function stageStart(t)
  cal.start=t.time; cal.wa,cal.wb=rates(t)
  pitch,yaw=stages[cal.stage][1],stages[cal.stage][2]
end
local function calibrationTick(t)
  assert(t.tilt<math.rad(12),"Calibration tilt exceeded 12 degrees")
  assert(norm(t.w)<0.6,"Calibration rotation too fast")
  assert(math.abs(t.mass/cal.mass-1)<=0.05,"Mass changed during calibration")
  if t.time-cal.start<0.4-1e-6 then return end
  local a,b=rates(t); local dt=t.time-cal.start
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
    assert(pn>0.04 and nn>0.04,"Calibration response too weak")
    assert((p[1]*n[1]+p[2]*n[2])/(pn*nn)<-0.5,"Opposite signals did not produce opposite rotation")
    assert(pn/nn>0.25 and pn/nn<4,"Calibration response inconsistent")
    columns[i]={(p[1]-n[1])/4,(p[2]-n[2])/4}
  end
  matrix=M.matrix(columns); calThrottle,calMass=throttle,t.mass
  mode,cal,pitch,yaw="MANUAL",nil,0,0
  message="Calibrated. Thrust unchanged. H enables HOLD."
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
  if mode=="CALIBRATE" then calibrationTick(t)
  elseif mode=="HOLD" then
    local a,b=rates(t)
    pitch,yaw=M.command(matrix,t.e1,t.e2,a,b,throttle/calThrottle)
  end
  apply()
end
local function draw()
  term.clear(); term.setCursorPos(1,1)
  print("ROCKET v0.2.1 - experimental upright hold")
  print((armed and "ARMED " or "DISARMED ")..mode)
  print(("Thrust %d/15 | F/B %+d R/L %+d"):format(throttle,pitch,yaw))
  if telemetry then
    print(("Mass %.1f | vertical speed %+.2f"):format(telemetry.mass,telemetry.velocity.y))
    print(("Tilt %s | calibration %s"):format(telemetry.tilt and ("%.1f deg"):format(math.deg(telemetry.tilt)) or "unset",matrix and "OK" or "needed"))
  else print("Telemetry unavailable") end
  print(("Sample dt: %s | status: slow refresh"):format(lastInterval and ("%.3fs"):format(lastInterval) or "warming up"))
  print("U Set UP (disarmed, physically upright)")
  print("E Arm/disarm | R/F thrust +/-")
  print("W/S D/A manual trim | C center/manual")
  print("K Calibrate: 3.6s pulses, thrust unchanged")
  print("H Hold on/off (after calibration)")
  print("SPACE stop | Q exit | no height hold")
  print(message)
end
local function handleKey(k)
  if k==keys.q then running=false; return end
  if k==keys.space then stop("Emergency stop."); return end
  if k==keys.e then
    local was=armed; stop(); armed=not was
    if armed then message="Armed at zero. Increase thrust manually." end
    return
  end
  if k==keys.u and not armed then
    local t=telemetry
    assert(t and os.clock()-t.time<=0.25 and t.dt and t.dt>0 and t.velocityAge<2 and norm(t.w)<0.03 and norm(t.velocity)<0.2,"Set UP only while still with fresh telemetry")
    basis=M.basis(t.q); referenceId=t.id
    matrix,calThrottle,calMass=nil,nil,nil
    message="UP set. Next: manual test, then K calibration."
  elseif armed then
    if k==keys.c then mode,cal,pitch,yaw="MANUAL",nil,0,0; message="Manual; thrust unchanged."
    elseif mode=="CALIBRATE" then message="Calibrating. C cancels; SPACE stops."
    elseif k==keys.h then
      if mode=="HOLD" then mode,pitch,yaw="MANUAL",0,0; message="Manual; thrust unchanged."
      elseif matrix and telemetry and os.clock()-telemetry.time<=0.25 and telemetry.tilt<math.rad(10) and throttle>0 and throttle/calThrottle>=0.5 and throttle/calThrottle<=2 then
        mode="HOLD"; message="Upright hold. No altitude/position control."
      else message="Need calibration and thrust within 0.5-2x calibration." end
    elseif k==keys.k then
      if mode=="MANUAL" and basis and telemetry and throttle>0 and telemetry.tilt and telemetry.tilt<math.rad(5) and norm(telemetry.w)<0.05 then
        matrix,calThrottle,calMass=nil,nil,nil
        mode="CALIBRATE"; cal={stage=1,samples={},mass=telemetry.mass}
        stageStart(telemetry); message="Calibration pulses: keep craft clear of supports."
      else message="K needs UP, manual, thrust>0, tilt<5, low rotation." end
    elseif k==keys.r or k==keys.f then
      throttle=clamp(throttle+(k==keys.r and 1 or -1),0,15)
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
  local function sample()
    local ok,err=pcall(tick)
    if not ok then
      invalidate(tostring(err)); telemetry=nil; lastQ,lastTime=nil,nil
      status=nil -- Re-bootstrap after an API error, with outputs already zero.
    end
  end
  sample(); draw()
  local timer=os.startTimer(0.1)
  while running do
    local event,k,held=os.pullEventRaw()
    if event=="terminate" then running=false
    elseif event=="key" and not held then
      local keyOK,keyError=pcall(handleKey,k)
      if not keyOK then invalidate(tostring(keyError)) end
    elseif event=="timer" and k==timer then
      sample(); timer=os.startTimer(0.1)
    end
    -- Keys do not resample the derivative: events may arrive in the same tick.
    if running then apply(); draw() end
  end
end
local ok,err=pcall(main)
local errors=clear()
term.clear(); term.setCursorPos(1,1)
print(#errors==0 and "Stopped. Five outputs cleared." or table.concat(errors,"; "))
if not ok then error(err,0) end
