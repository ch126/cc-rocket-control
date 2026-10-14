-- Fixed-craft interface test, NOT flight calibration. Explicit P per pulse.
local Backend=require("engine_io")
local d=Backend.new({clock=os.clock,peripheral=peripheral,redstone=redstone,parallel=parallel,sleep=sleep},
  {1,1,1,1,1,1,1,1,1})
print("FIXED NINE ENGINE TEST - 1 kN / 0.30s request")
print("Craft must be secured. Old controller must be stopped.")
print("Startup clears nine engines. No automatic ignition.")
local ready,why=d:initialize(); assert(ready,why)
local selected,deadline,done,poseReadyAt=1,nil,false,nil
local function zeroFrame()
  local f={}
  for i=1,9 do f[i]={name="creative_vector_thruster_"..(i-1),kn=0,vectorX=0,vectorY=0} end
  return f
end
local function stop(reason)
  deadline=nil; d:stop(reason); print(reason)
end
local cleared,errors=d:run(function()
  parallel.waitForAny(function()
    local timer=os.startTimer(.05)
    print("Selected engine 0. Left/right select; P pulse; SPACE stop; Q exit")
    while not done do
      local event,key,held=os.pullEventRaw()
      if event=="terminate" or (event=="key" and key==keys.q) then stop("Exit requested"); done=true; return end
      if event=="key" and not held then
        if key==keys.space then stop("Emergency stop")
        elseif not deadline and (key==keys.left or key==keys.right) then
          selected=(selected-1+(key==keys.right and 1 or -1))%9+1
          print("Selected engine "..(selected-1))
        elseif key==keys.p and not deadline then
          local fresh=poseReadyAt and os.clock()-poseReadyAt<=.25
          local armed,err=false,"Await fresh fixed-craft telemetry"
          if fresh then armed,err=d:arm() end
          if not armed then print("NOT STARTED: "..tostring(err))
          else
            local f=zeroFrame(); f[selected].kn=1
            local sent,reason=d:submit(f)
            if sent then deadline=os.clock()+.30; print("PULSE engine "..(selected-1).." at 1 kN")
            else stop(tostring(reason)) end
          end
        end
      end
      if event=="timer" and key==timer then
        if d.fault then stop("FAULT: "..d.fault); done=true; return end
        if deadline and (not poseReadyAt or os.clock()-poseReadyAt>.25) then d:fail("Fixed-test telemetry stale"); done=true; return end
        if deadline then
          if os.clock()>=deadline then stop("Pulse request ended; waiting for output workers to clear")
          else local f=zeroFrame(); f[selected].kn=1; assert(d:submit(f)) end
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
print("0.30s is requested duration; queued calls may extend actual pulse.")
