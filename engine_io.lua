-- Explicit nine-engine actuator backend. Requiring this module writes nothing.
-- Calling initialize/run DOES write engines; use only with the old controller stopped.
-- API completion is an acknowledgement, not measured thrust or instantaneous braking.
local M={version=1}
local function finite(x) return type(x)=="number" and x==x and math.abs(x)<math.huge end
local function name(i) return "creative_vector_thruster_"..(i-1) end
local function zero() return {kn=0,vectorX=0,vectorY=0} end
function M.new(api,caps)
  assert(api and caps and #caps==9,"Need API and nine explicit caps")
  local self={api=api,caps={},engines={},desired={},epoch=0,armed=false,running=false,ready=false}
  for i=1,9 do
    assert(finite(caps[i]) and caps[i]>=0 and caps[i]<=733,"Invalid engine cap")
    self.caps[i]=caps[i]; self.desired[i]=zero()
    self.engines[i]={name=name(i),generation=0,lastAck=-math.huge,gate=nil,output=nil,x=nil,y=nil}
  end
  function self:stop(reason)
    self.epoch=self.epoch+1; self.armed=false; self.reason=reason
    for i=1,9 do self.desired[i]=zero(); self.engines[i].generation=self.engines[i].generation+1 end
  end
  function self:fail(reason)
    if not self.fault then self.fault=tostring(reason); self:stop(self.fault) end
  end
  function self:clearAll()
    local errors,tasks={},{}
    for _,side in ipairs({"top","bottom","front","back","left","right"}) do
      local ok,err=pcall(api.redstone.setAnalogOutput,side,0)
      if not ok then errors[#errors+1]=side..": "..tostring(err) end
    end
    for i=1,9 do
      local e=self.engines[i]
      tasks[#tasks+1]=function()
        local complete=true
        for _,call in ipairs({{"setThrust",0},{"setThrustOutput",0},{"setVector",0,0}}) do
          local ok,err=pcall(api.peripheral.call,e.name,table.unpack(call))
          if not ok then complete=false; errors[#errors+1]=e.name.." "..call[1]..": "..tostring(err) end
        end
        if complete then e.gate,e.output,e.x,e.y,e.lastAck=0,0,0,0,api.clock() end
      end
    end
    api.parallel.waitForAll(table.unpack(tasks))
    return #errors==0,errors
  end
  function self:initialize()
    self:stop("Initializing")
    local cleared,errors=self:clearAll()
    if not cleared then self:fail(table.concat(errors,"; ")); return false,self.fault end
    for i=1,9 do
      local e=self.engines[i]
      if not api.peripheral.hasType(e.name,"creative_vector_thruster") then self:fail("Missing/wrong type: "..e.name); break end
      local ok,limit=pcall(api.peripheral.call,e.name,"getMaxThrustOutputPn")
      if not ok or not finite(limit) or limit<self.caps[i]*1000 then self:fail("Invalid/insufficient cap: "..e.name); break end
    end
    self.ready=not self.fault
    return self.ready,self.fault
  end
  function self:arm()
    if not self.ready or self.fault then return false,"Backend not ready" end
    for _,e in ipairs(self.engines) do
      if e.gate~=0 or e.output~=0 or e.x~=0 or e.y~=0 or api.clock()-e.lastAck>.25 then return false,"Await fresh zero acknowledgements" end
    end
    self.epoch=self.epoch+1; self.armed=true; self.lastInput=api.clock()
    return true
  end
  function self:submit(commands)
    if not self.armed or self.fault then return false,"Not armed" end
    local ok,copy=pcall(function()
      assert(type(commands)=="table" and #commands==9,"Need exactly nine commands")
      local result={}
      for i=1,9 do
        local c=commands[i]; assert(type(c)=="table" and c.name==name(i),"Engine order/identity mismatch")
        assert(finite(c.kn) and c.kn>=0 and c.kn<=self.caps[i],"Power outside configured cap")
        for _,v in ipairs({c.vectorX,c.vectorY}) do
          assert(finite(v) and math.abs(v)<=.2+1e-9 and math.abs(v*15-math.floor(v*15+.5))<1e-7,"Invalid/non-quantized vector")
        end
        assert(finite(c.vectorX) and finite(c.vectorY),"Missing vector")
        if i>5 then assert(c.vectorX==0 and c.vectorY==0,"Top vectoring not supported") end
        result[i]={kn=c.kn,vectorX=c.vectorX,vectorY=c.vectorY}
      end
      return result
    end)
    if not ok then self:fail("Invalid frame: "..tostring(copy)); return false,self.fault end
    for i=1,9 do
      if copy[i].kn==0 and self.desired[i].kn>0 then self.engines[i].generation=self.engines[i].generation+1 end
    end
    self.desired=copy; self.lastInput=api.clock(); return true
  end
  function self:cycle(i)
    local e=self.engines[i]; local s=self.desired[i]
    local epoch,generation=self.epoch,e.generation
    local function valid()
      return self.armed and not self.fault and self.epoch==epoch and e.generation==generation and
        self.desired[i].kn>0 and api.clock()-self.lastInput<=.35
    end
    local complete,didIO=true,false
    local function send(method,...)
      didIO=true
      local ok,err=pcall(api.peripheral.call,e.name,method,...)
      if not ok then complete=false; self:fail(e.name.." "..method..": "..tostring(err)) end
      return ok
    end
    if s.kn==0 or not valid() then
      -- Independent attempts even if gate-off fails. Retain zero base override.
      if e.gate~=0 and send("setThrust",0) then e.gate=0 end
      if e.output~=0 and send("setThrustOutput",0) then e.output=0 end
      if (e.x~=0 or e.y~=0) and send("setVector",0,0) then e.x,e.y=0,0 end
    else
      local output=s.kn*1000
      if e.output~=output and send("setThrustOutput",output) then e.output=output end
      if complete and valid() and (e.x~=s.vectorX or e.y~=s.vectorY) and send("setVector",s.vectorX,s.vectorY) then e.x,e.y=s.vectorX,s.vectorY end
      if complete and valid() and e.gate~=15 and send("setThrust",15) then e.gate=15 end
    end
    -- A cache-only loop is NOT a server acknowledgement. Refresh an unchanged
    -- gate periodically using a real call so a stalled link cannot look fresh.
    if complete and not didIO and epoch==self.epoch and generation==e.generation and api.clock()-e.lastAck>=.15 then
      local gate=valid() and 15 or 0
      if send("setThrust",gate) then e.gate=gate end
    end
    if complete and didIO and epoch==self.epoch and generation==e.generation then e.lastAck=api.clock() end
  end
  function self:watchdog()
    if not self.armed then return end
    if api.clock()-self.lastInput>.35 then self:fail("Command stream stale >0.35s"); return end
    for _,e in ipairs(self.engines) do
      if not api.peripheral.hasType(e.name,"creative_vector_thruster") then self:fail("Disconnected: "..e.name); return end
      if api.clock()-e.lastAck>.6 then self:fail("Output timeout: "..e.name); return end
    end
  end
  function self:run(producer)
    assert(self.ready,"Initialize before run")
    self.running=true
    local tasks={function()
      local ok,err=pcall(producer,self)
      if not ok then self:fail("Producer: "..tostring(err)) end
      self:stop("Producer finished"); self.running=false
    end,function() while self.running do self:watchdog(); api.sleep(.05) end end}
    for i=1,9 do local index=i; tasks[#tasks+1]=function()
      while self.running do self:cycle(index); api.sleep(.05) end
    end end
    -- Drain in-flight calls before final cleanup: do not cancel coroutines and
    -- let their already-queued positive writes complete after the final zeros.
    api.parallel.waitForAll(table.unpack(tasks))
    return self:clearAll()
  end
  return self
end
return M
