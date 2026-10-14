-- Runtime bridge; does not initialize or arm engines automatically.
-- Default shadow mode never invokes the driver. Explicit live mode requires an
-- already initialized/armed engine_io backend and a validated recovery profile.
local M={version=1}
function M.start(deps,driver,profile,pad,sample,now,craft,limits,mode,entry)
  mode=mode or "shadow"
  if mode~="shadow" and mode~="live" then return nil,"Unknown runtime mode" end
  local valid,why=deps.planner.validate(profile); if not valid then return nil,why end
  if craft.id~=profile.craftId then return nil,"Geometry identity mismatch" end
  if mode=="live" then
    if not driver or not driver.ready or not driver.armed or driver.fault then return nil,"Live mode needs an already-armed backend" end
    for i=1,9 do
      local cap=i<=5 and limits.mainKN or limits.topKN[i-5]
      if type(cap)~="number" or cap>driver.caps[i] then return nil,"Planner cap exceeds driver cap" end
    end
  end
  local mission,err=deps.mission.start(deps,profile,pad,sample,now,entry)
  if not mission then return nil,err end
  return {mission=mission,mode=mode,driver=driver,profile=profile,pad=pad,craft=craft,limits=limits,
    phase=mission.phase,lastTime=now},nil
end
function M.step(deps,state,sample,now)
  if state.phase=="FAULT" then return {manualTakeover=true,error=state.reason} end
  local limits={}; for k,v in pairs(state.limits) do limits[k]=v end
  limits.previous=state.previous or limits.previous
  limits.balance=state.balance
  limits.dt=now-state.lastTime
  local out
  if state.phase=="HOVER" then out=deps.hover.step(deps,state.hover,sample,now,state.craft,limits)
  else
    out=deps.mission.step(deps,state.mission,sample,now,state.craft,limits)
    if out.handoffHover then
      state.phase="HOVER"; state.balance=nil; limits.balance=nil
      state.hover=deps.hover.start(state.profile,state.pad,state.lastTime)
      out=deps.hover.step(deps,state.hover,sample,now,state.craft,limits)
    end
  end
  state.lastTime=now
  if out.manualTakeover or not out.commands then
    state.phase="FAULT"; state.reason=out.error or (out.plan and out.plan.reason) or "No complete command frame"
    -- Do not turn invalid/no guidance into a guessed thrust command. Live
    -- producer must expose takeover; engine_io watchdog still has its explicit
    -- stale-stream cutoff, which is an emergency cutoff, NOT a safe landing.
    return {manualTakeover=true,error=state.reason,commands=nil}
  end
  if out.allocation then
    state.previous={x=out.allocation.mainX,y=out.allocation.mainY}
    state.balance=out.allocation.balance
  end
  if state.mode=="live" then
    local ok,err=state.driver:submit(out.commands)
    if not ok then state.phase="FAULT"; state.reason=err; return {manualTakeover=true,error=err,commands=nil} end
    out.submitted=true
  else out.submitted=false end
  return out
end
return M
