-- Ground-only READ-ONLY hardware inspection; never changes engine outputs.
-- Stop the flight program and support the craft before running this script.
-- Saves a new report, never overwrites logs/calibration or enables flight.
print("ROCKET INSPECT v1 - NO ENGINE COMMANDS")
print("Run landed, with controller stopped.")
local report={version=1,sublevel={},engines={}}
local function read(fn,...)
  if type(fn)~="function" then return {ok=false,error="API unavailable"} end
  local ok,value=pcall(fn,...)
  return ok and {ok=true,value=value} or {ok=false,error=tostring(value)}
end
local function number(v) return type(v)=="number" and v==v and math.abs(v)<math.huge end
for _,method in ipairs({"isInPlotGrid","getUniqueId","getMass","getCenterOfMass",
    "getInertiaTensor","getInverseInertiaTensor","getLogicalPose",
    "getLinearVelocity","getAngularVelocity"}) do
  report.sublevel[method]=read(sublevel and sublevel[method])
end
local mass=report.sublevel.getMass
print(mass.ok and number(mass.value) and ("Mass: %.4f (API units)"):format(mass.value) or "Mass: unavailable")
local center=report.sublevel.getCenterOfMass
local v=center.ok and center.value
if type(v)=="table" and number(v.x) and number(v.y) and number(v.z) then
  print(("COM: %.3f %.3f %.3f"):format(v.x,v.y,v.z))
else print("COM: unavailable") end
for _,method in ipairs({"getInertiaTensor","getInverseInertiaTensor"}) do
  print(method..": "..(report.sublevel[method].ok and "read" or "unavailable"))
end
local found,nonzero=0,0
for i=0,8 do
  local name="creative_vector_thruster_"..i
  local row={name=name,present=peripheral.hasType(name,"creative_vector_thruster"),reads={}}
  if row.present then
    found=found+1
    for _,method in ipairs({"getThrust","getPower","getVectorX","getVectorY",
        "getTargetVectorX","getTargetVectorY","getMaxThrustOutputPn","isCustomThrustOutputActive"}) do
      row.reads[method]=read(peripheral.call,name,method)
    end
    local gate=row.reads.getThrust
    if gate.ok and number(gate.value) and gate.value~=0 then nonzero=nonzero+1 end
    print(("Engine %d: gate %s"):format(i,gate.ok and tostring(gate.value) or "unknown"))
  else print("Engine "..i..": MISSING") end
  report.engines[#report.engines+1]=row
end
print(("Found %d/9; nonzero gates %d"):format(found,nonzero))
print("No ignition, shutdown, or calibration performed.")
local ok,err=pcall(function()
  local path
  for i=1,9999 do
    local candidate=("/rocket-inspect-%04d.txt"):format(i)
    if not fs.exists(candidate) then path=candidate; break end
  end
  assert(path,"Report names exhausted")
  local payload=textutils.serialize(report)
  local free=fs.getFreeSpace("/")
  assert(type(free)~="number" or free>#payload+8192,"Insufficient free space; nothing deleted")
  local h,openError=fs.open(path,"w"); assert(h,openError)
  local wrote,writeError=pcall(h.write,payload); h.close(); assert(wrote,writeError)
  print("Saved: "..path)
end)
if not ok then print("Report not saved: "..tostring(err)) end
