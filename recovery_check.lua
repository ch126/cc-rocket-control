-- v0.6.0-preview: READ-ONLY nine-engine inventory. NEVER ignites or stops engines.
-- Run while safely supported, with the flight controller stopped.
print("RECOVERY PRECHECK - READ ONLY")
print("No engine commands. Not a flight controller.")
local rows={}
local all=true
for i=0,8 do
  local name="creative_vector_thruster_"..i
  local present=peripheral.hasType(name,"creative_vector_thruster")
  local ok,cap=false,nil
  if present then
    ok,cap=pcall(peripheral.call,name,"getMaxThrustOutputPn")
    ok=ok and type(cap)=="number" and cap==cap and cap>=733000 and cap<math.huge
  end
  all=all and ok
  local side=({[5]="rear",[6]="left",[7]="front",[8]="right"})[i]
  rows[#rows+1]=("%d %-10s %s"):format(i,side and ("RCS "..side) or "MAIN",ok and "LINK/CAP OK" or "CHECK LINK/CAP")
end
for _,row in ipairs(rows) do print(row) end
local ok,details=pcall(function()
  assert(sublevel and sublevel.isInPlotGrid(),"Computer not on assembled craft")
  local mass=sublevel.getMass()
  local pose=sublevel.getLogicalPose()
  assert(type(mass)=="number" and mass>0,"Invalid mass")
  return ("Mass %.3f | reference Y %.2f"):format(mass,pose.position.y)
end)
print(ok and details or ("Telemetry: "..tostring(details)))
print("Target: launch reference Y +20 blocks")
print(all and "Nine links found; NOT flight approval." or "Fix missing/wrong engines before development tests.")
print("Still needed: RCS response + vertical brake tests")
print("No measured profile => no free-fall enable.")
