-- Read-only CC:Sable sampler. No engine writes, arming, or calibration.
-- Position is logical-pose position, NOT computer-block altitude.
local M={version=1}
local function finite(n) return type(n)=="number" and n==n and math.abs(n)<math.huge end
local function vector(v)
  assert(type(v)=="table" and finite(v.x) and finite(v.y) and finite(v.z),"Invalid telemetry vector")
  return {x=v.x,y=v.y,z=v.z}
end
function M.new(api,mathlib,basis)
  local self={previous=nil}
  function self:read(driver)
    local ok,result=pcall(function()
      local started=api.clock()
      assert(finite(started),"Invalid telemetry clock")
      local values,times,jobs={},{},{}
      for _,method in ipairs({"isInPlotGrid","getUniqueId","getMass","getLogicalPose","getLinearVelocity"}) do
        local key=method
        jobs[#jobs+1]=function()
          local before=api.clock()
          local value=api.sublevel[key]()
          local after=api.clock()
          assert(finite(before) and finite(after) and before>=started and after>=before,"Telemetry clock reversed")
          values[key]=value; times[key]={start=before,finish=after}
        end
      end
      api.parallel.waitForAll((table.unpack or unpack)(jobs))
      local now=api.clock()
      -- Conservatively date the frame at the earliest possible read, never
      -- relabel old velocity/mass/identity as fresh because pose arrived last.
      assert(finite(now) and now>=started and now-started<=.25,"Telemetry acquisition too slow")
      assert(values.isInPlotGrid==true,"Computer is not on an assembled craft")
      local id,mass=values.getUniqueId,values.getMass
      assert(type(id)=="string" and #id>0 and finite(mass) and mass>0,"Invalid identity/mass")
      local pose=values.getLogicalPose
      assert(type(pose)=="table","Missing logical pose")
      local q=mathlib.quaternion(pose.orientation)
      local position=vector(pose.position); local velocity=vector(values.getLinearVelocity)
      local poseTime=(times.getLogicalPose.start+times.getLogicalPose.finish)/2
      local previous=self.previous
      self.previous={q=q,time=poseTime,id=id,mass=mass}
      if not previous then return {warming=true,reason="Need two fresh pose samples"} end
      assert(previous.id==id and math.abs(mass/previous.mass-1)<=.02,"Craft changed between samples")
      local dt=poseTime-previous.time
      assert(dt>0 and dt<=.25,"Pose sample gap invalid")
      local rate=vector(mathlib.rate(previous.q,q,dt))
      local _,_,tilt=mathlib.error(q,basis)
      local ready=driver~=nil and driver.ready==true and not driver.fault
      for i=1,9 do
        local name="creative_vector_thruster_"..(i-1)
        local e=driver and driver.engines and driver.engines[i]
        if not api.peripheral.hasType(name,"creative_vector_thruster") or not e or e.name~=name or
          not finite(e.lastAck) or e.lastAck>now or now-e.lastAck>.6 then ready=false end
      end
      return {time=started,receivedAt=now,acquisitionSeconds=now-started,fieldTimes=times,
        craftId=id,mass=mass,q=q,w=rate,tilt=tilt,
        angularSpeed=math.sqrt(rate.x^2+rate.y^2+rate.z^2),
        x=position.x,y=position.y,z=position.z,vx=velocity.x,vy=velocity.y,vz=velocity.z,
        allNineReady=ready==true}
    end)
    if not ok then self.previous=nil; return nil,tostring(result) end
    if result.warming then return nil,result.reason end
    return result,nil
  end
  return self
end
return M
