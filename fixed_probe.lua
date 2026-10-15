-- Fixed-craft interface test, NOT flight calibration. Explicit P per pulse.
local args={...}
local auto=args[1]=="--auto"
assert(not args[1] or auto,"Usage: fixed-test [--auto]")
local Backend=require("engine_io")
local d=Backend.new({clock=os.clock,peripheral=peripheral,redstone=redstone,parallel=parallel,sleep=sleep},
  {1,1,1,1,1,1,1,1,1})
print("FIXED NINE ENGINE TEST - 1 kN / 0.30s request")
print("Craft must be secured. Old controller must be stopped.")
print("Startup clears nine engines. No automatic ignition.")
local ready,why=d:initialize(); assert(ready,why)
local selected,deadline,done,poseReadyAt=1,nil,false,nil
local phase,phaseAt,observedOn,stoppedAt="IDLE",0,false,nil
local report={version=1,kind="fixed-interface-only",flightCalibrationValid=false,engines={},complete=false}
local function reportPath()
  for i=1,9999 do
    local name=("/rocket-fixed-check-%04d.json"):format(i)
    if not fs.exists(name) then return name end
  end
  error("No unused report name")
end
local function room()
  local free=fs.getFreeSpace("/")
  return type(free)~="number" or free>=16384
end
local function zeroFrame()
  local f={}
  for i=1,9 do f[i]={name="creative_vector_thruster_"..(i-1),kn=0,vectorX=0,vectorY=0} end
  return f
end
local function stop(reason)
  deadline=nil; d:stop(reason); print(reason)
end
local function allZero(after)
  for _,e in ipairs(d.engines) do
    if not peripheral.hasType(e.name,"creative_vector_thruster") or e.gate~=0 or e.output~=0 or e.x~=0 or e.y~=0 or
      e.lastAck<after or os.clock()-e.lastAck>.25 then return false end
  end
  return true
end
local function pulse()
  local fresh=poseReadyAt and os.clock()-poseReadyAt<=.25
  if not fresh then return false,"Await fresh fixed-craft telemetry" end
  local armed,err=d:arm(); if not armed then return false,err end
  local f=zeroFrame(); f[selected].kn=1
  local sent,reason=d:submit(f); if not sent then return false,reason end
  deadline=os.clock()+.30; phaseAt=os.clock(); observedOn=false
  print("PULSE engine "..(selected-1).." at 1 kN")
  if auto then
    phase="PULSE"
    report.engines[selected]={id=selected-1,requestedAt=phaseAt,requestedKN=1,requestDuration=.30}
  end
  return true
end
local cleared,errors=d:run(function()
  parallel.waitForAny(function()
    local timer=os.startTimer(.05)
    print(auto and "AUTO: P starts all nine once; SPACE/Q abort" or "Selected engine 0. Left/right select; P pulse; SPACE stop; Q exit")
    while not done do
      local event,key,held=os.pullEventRaw()
      if event=="terminate" or (event=="key" and key==keys.q) then report.reason="Operator exit"; stop("Exit requested"); done=true; return end
      if event=="key" and not held then
        if key==keys.space then report.reason="Operator emergency stop"; stop("Emergency stop"); if auto then done=true; return end
        elseif not auto and not deadline and (key==keys.left or key==keys.right) then
          selected=(selected-1+(key==keys.right and 1 or -1))%9+1
          print("Selected engine "..(selected-1))
        elseif key==keys.p and not deadline then
          if auto then
            if phase=="IDLE" then
              assert(room(),"Need 16 KiB free for report; no files deleted")
              report.path=reportPath(); phase="WAIT_ZERO"; phaseAt=os.clock()
              print("Automatic sequence requested")
            end
          else
            local started,err=pulse(); if not started then print("NOT STARTED: "..tostring(err)) end
          end
        end
      end
      if event=="timer" and key==timer then
        if d.fault then stop("FAULT: "..d.fault); done=true; return end
        if deadline and (not poseReadyAt or os.clock()-poseReadyAt>.25) then d:fail("Fixed-test telemetry stale"); done=true; return end
        if deadline then
          local e=d.engines[selected]
          if e.gate==15 and e.output==1000 then
            observedOn=true
            if auto and not report.engines[selected].onObservedAt then report.engines[selected].onObservedAt=os.clock() end
          end
          if os.clock()>=deadline then
            stoppedAt=os.clock(); stop("Pulse request ended; waiting for output workers to clear")
            if auto then
              report.engines[selected].stopRequestedAt=stoppedAt
              assert(observedOn,"Pulse never acknowledged; remaining engines not tested")
              phase="DRAIN"; phaseAt=stoppedAt
            end
          else local f=zeroFrame(); f[selected].kn=1; assert(d:submit(f)) end
        end
        if auto and phase=="WAIT_ZERO" then
          assert(os.clock()-phaseAt<2,"Zero/telemetry readiness timeout")
          if allZero(phaseAt) and poseReadyAt and os.clock()-poseReadyAt<=.25 then assert(pulse()) end
        elseif auto and phase=="DRAIN" then
          assert(os.clock()-phaseAt<2,"Clear acknowledgement timeout")
          if allZero(stoppedAt) then
            report.engines[selected].zeroObservedAt=os.clock(); phase="REST"; phaseAt=os.clock()
          end
        elseif auto and phase=="REST" then
          assert(allZero(stoppedAt),"Output not zero during rest")
          if os.clock()-phaseAt>=.5 then
            report.engines[selected].interfaceCheckPassed=true
            if selected==9 then report.complete=true; done=true; return end
            selected=selected+1; phase="WAIT_ZERO"; phaseAt=os.clock()
          end
        end
        timer=os.startTimer(.05)
      end
    end
  end,function()
    -- Unexpected motion invalidates a fixed test. This monitor cannot prove
    -- that clamps are safe; API/queued-output latency still exists.
    local origin,originQ
    while not done do
      local started=os.clock()
      local pose=sublevel.getLogicalPose(); local p=pose.position
      assert(os.clock()-started<=.25,"Fixed-test pose read too slow")
      assert(type(p)=="table" and type(p.x)=="number" and type(p.y)=="number" and type(p.z)=="number","Invalid position")
      if not origin then origin={x=p.x,y=p.y,z=p.z} end
      local delta=(p.x-origin.x)^2+(p.y-origin.y)^2+(p.z-origin.z)^2
      if not (delta==delta and delta<=.01) then d:fail("Craft moved >0.1 blocks; fixed test aborted"); done=true; return end
      local raw=pose.orientation; local v=raw.v or raw; local w=raw.a or raw.w
      local n=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z+w*w)
      assert(n==n and n>1e-6 and n<math.huge,"Invalid orientation")
      local q={v.x/n,v.y/n,v.z/n,w/n}; originQ=originQ or q
      local dot=0; for i=1,4 do dot=dot+q[i]*originQ[i] end
      if math.abs(dot)<math.cos(math.rad(1)) then d:fail("Craft rotated >2 degrees; fixed test aborted"); done=true; return end
      poseReadyAt=started
      sleep(.05)
    end
  end)
end)
if not cleared then print("CLEAR FAILED: "..table.concat(errors,"; "))
else print("Nine engines cleared. No flight calibration saved.") end
if d.fault then print("FAULT: "..d.fault) end
if auto then
  report.cleanupConfirmed=cleared; report.reason=d.fault or report.reason
  report.complete=report.complete and cleared and not d.fault
  local ok,err=pcall(function()
    local path=report.path or reportPath()
    assert(not fs.exists(path),"Report already exists; not overwritten")
    local payload=textutils.serializeJSON(report)
    assert(room(),"Insufficient report space; no files deleted")
    local h=assert(fs.open(path,"w")); local wrote,why=pcall(h.write,payload); h.close(); assert(wrote,why)
    print("Saved: "..path)
  end)
  if not ok then print("REPORT NOT SAVED: "..tostring(err)) end
  print(report.complete and "9/9 interface checks completed. Flight NOT calibrated." or "Automatic checks incomplete. Flight NOT calibrated.")
end
print("0.30s is requested duration; queued calls may extend actual pulse.")
