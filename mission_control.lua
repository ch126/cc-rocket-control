-- Shadow orchestration of RETURN -> COAST -> BRAKE -> HOVER_HANDOFF.
-- No actual I/O. Hot staging and tower capture are not implemented here yet.
local M={version=1}
function M.start(deps,profile,pad,sample,now)
  local burn,why=deps.boostback.start(deps.planner,profile,pad,sample,now)
  if not burn then return nil,why end
  return {phase="RETURN",burn=burn,profile=profile,previous=nil,lastControlTime=now},nil
end
function M.step(deps,mission,sample,now,craft,limits)
  if mission.phase=="FAULT" then return {manualTakeover=true,shadowOnly=true,error=mission.error} end
  if mission.phase=="HOVER_HANDOFF" then return {handoffHover=true,shadowOnly=true} end
  local bounded={}; for k,v in pairs(limits) do bounded[k]=v end
  bounded.balance=mission.balance
  bounded.dt=now-mission.lastControlTime; mission.lastControlTime=now
  local result
  if mission.phase=="RETURN" then
    local plan=deps.boostback.step(deps.planner,mission.burn,sample,now)
    if plan.phase=="HANDOFF" then
      mission.recovery=plan.recoveryState; mission.phase="RECOVERY"
      mission.balance=nil -- Burn-phase pulse debt must not carry into engine-off coast.
      -- Retain a concrete previous frame only when one exists. A fault before
      -- the first frame cannot invent an engine command to retain.
      return {plan=plan,commands=mission.previous,retainPreviousCommand=true,shadowOnly=true}
    end
    if plan.phase=="FAULT" then result={plan=plan,manualTakeover=true,shadowOnly=true}
    else result=deps.control.step(deps,mission.burn,sample,now,craft,bounded,plan) end
  else
    result=deps.control.step(deps,mission.recovery,sample,now,craft,bounded)
  end
  if result.manualTakeover then mission.phase="FAULT"; mission.error=result.error or (result.plan and result.plan.reason)
  elseif result.handoffHover then mission.phase="HOVER_HANDOFF"
  elseif result.commands then
    mission.previous=result.commands
    mission.balance=result.allocation and result.allocation.balance
  end
  return result
end
return M
