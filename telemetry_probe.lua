-- Read-only timing probe. Run supported, with the flight controller stopped.
-- Does not write files, redstone, or engine commands.
print("ROCKET TELEMETRY - READ ONLY")
print("Run landed, controller stopped. No engine commands.")
local samplerModule=require("telemetry")
local mathlib=assert(loadfile("home.lua"),"Need home.lua math module")("--test")
local sampler=samplerModule.new({clock=os.clock,sublevel=sublevel,parallel=parallel,peripheral=peripheral},
  mathlib,{up={x=0,y=1,z=0},a={x=1,y=0,z=0},b={x=0,y=0,z=-1}})
local valid,failed,total,worst=0,0,0,0
for i=1,30 do
  local s,err=sampler:read(nil)
  if s then
    valid=valid+1; total=total+s.acquisitionSeconds; worst=math.max(worst,s.acquisitionSeconds)
    print(("%02d read %.3fs Y %.2f Vy %.2f tilt %.1f"):format(i,s.acquisitionSeconds,s.y,s.vy,math.deg(s.tilt)))
  else failed=failed+1; print(("%02d rejected: %s"):format(i,err)) end
  sleep(.05)
end
print(("Valid %d/30, rejected %d; worst %.3fs"):format(valid,failed,worst))
if valid>0 then print(("Mean accepted read %.3fs"):format(total/valid)) end
print("First frame needs warmup. Y is logical-pose position.")
print("No output acknowledgements checked; NOT flight approval.")
