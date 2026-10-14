-- Operator state machine. Initialization/worker lifetime belongs to the app.
local M={version=1}
local function finite(n) return type(n)=="number" and n==n and math.abs(n)<math.huge end
function M.new(deps,driver,config)
  local valid,why=deps.planner.validate(config.profile); assert(valid,why)
  assert(config.craft.id==config.profile.craftId,"Geometry identity mismatch")
  assert(type(config.caps)=="table" and #config.caps==9,"Nine explicit engine limits required")
  for i=1,9 do
    local cap=i<=5 and config.limits.mainKN or config.limits.topKN[i-5]
    assert(finite(cap) and cap>=0 and cap<=config.caps[i],"Allocation/output cap mismatch")
  end
  local self={mode="DISARMED",throttle=0,driver=driver,config=config}
  local function checked(s,now)
    assert(s and finite(s.time) and finite(now) and now>=s.time and now-s.time<=.25,"Fresh telemetry required")
    assert(s.craftId==config.profile.craftId and finite(s.mass) and math.abs(s.mass/config.profile.mass-1)<=.02,"Craft/profile mismatch")
    assert(s.allNineReady==true,"Nine output links required")
    assert(finite(s.tilt) and s.tilt<math.rad(10) and finite(s.angularSpeed) and s.angularSpeed<.15,"Attitude outside envelope")
  end
  function self:stop(reason)
    driver:stop(reason); self.mode="DISARMED"; self.throttle=0; self.runtime=nil
    self.hover=nil; self.previous=nil; self.balance=nil; self.output=nil; self.message=reason
  end
  function self:action(action,s,now)
    if action=="stop" then self:stop("Operator stop"); return true end
    local ok,err=pcall(function()
      checked(s,now)
      if action=="pad" then
        assert(self.mode=="DISARMED","Record pad only disarmed and supported")
        assert(s.tilt<math.rad(3) and math.abs(s.vy)<.05 and s.vx^2+s.vz^2<.0025,"Pad requires still upright craft")
        -- Operator must ensure supported: this is not a contact sensor.
        self.pad={x=s.x,y=s.y,z=s.z}; self.message="Pad recorded in memory"
      elseif action=="arm" then
        assert(self.mode=="DISARMED" and self.pad,"Record pad first")
        assert(s.tilt<math.rad(3) and math.abs(s.vy)<.1 and s.vx^2+s.vz^2<.01,"Arm requires still upright craft")
        local ready,why=driver:arm(); assert(ready,why)
        self.mode="MANUAL"; self.throttle=0; self.lastTime=now
      elseif action=="up" or action=="down" then
        assert(self.mode=="MANUAL","Throttle keys require MANUAL")
        self.throttle=math.max(0,math.min(config.profile.maxThrottle,self.throttle+(action=="up" and .01 or -.01)))
      elseif action=="hover" then
        assert(driver.armed,"Arm first")
        self.hover=deps.hover.start(config.profile,{x=s.x,y=s.y-20,z=s.z},now)
        self.mode="HOVER"; self.balance=nil; self.lastTime=now
      elseif action=="descent" or action=="boostback" then
        assert(driver.armed and self.pad,"Need armed craft and pad")
        local state,why=deps.runtime.start(deps,driver,config.profile,self.pad,s,now,
          config.craft,config.limits,"live",action)
        assert(state,why); self.runtime=state; self.mode="MISSION"; self.lastTime=now
      else error("Unsupported action") end
    end)
    if not ok then self.message=tostring(err); return false,self.message end
    return true
  end
  function self:tick(s,now)
    if self.mode=="DISARMED" then return nil end
    local ok,out=pcall(function()
      checked(s,now); assert(driver.armed and not driver.fault,"Output backend stopped")
      if self.mode=="MISSION" then return deps.runtime.step(deps,self.runtime,s,now) end
      local lim={}; for k,v in pairs(config.limits) do lim[k]=v end
      lim.previous=self.previous; lim.balance=self.balance; lim.dt=now-self.lastTime
      assert(lim.dt>0 and lim.dt<=.25,"Control interval invalid")
      local result
      if self.mode=="MANUAL" and self.throttle==0 then
        local commands={}
        for i=0,8 do commands[#commands+1]={name="creative_vector_thruster_"..i,kn=0,vectorX=0,vectorY=0} end
        self.balance=nil; self.previous=nil
        result={commands=commands,plan={phase="MANUAL",mainThrottle=0}}
      elseif self.mode=="HOVER" then result=deps.hover.step(deps,self.hover,s,now,config.craft,lim)
      else
        result=deps.control.step(deps,{profile=config.profile},s,now,config.craft,lim,
          {phase="MANUAL",mainThrottle=self.throttle,lateral={ax=0,az=0},shadowOnly=true})
      end
      if result.commands then
        local sent,why=driver:submit(result.commands); assert(sent,why)
      end
      return result
    end)
    self.lastTime=now
    if not ok or not out or out.manualTakeover or not out.commands then
      self:stop(not ok and tostring(out) or (out and out.error) or "Guidance unavailable")
      return nil,self.message -- Explicit emergency cutoff, NOT safe landing.
    end
    if out.allocation then self.previous={x=out.allocation.mainX,y=out.allocation.mainY}; self.balance=out.allocation.balance end
    self.output=out; return out
  end
  return self
end
return M
