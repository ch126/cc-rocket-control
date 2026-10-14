-- Unified experimental console. Default: read-only. No startup installation.
-- --live <config.json> explicitly initializes/zeros outputs; E separately arms.
local args={...}
local live=args[1]=="--live"
assert(not args[1] or live,"Usage: rocket_app [--live verified-config.json]")
local deps={}
for key,file in pairs({planner="recovery_plan",allocator="control_allocator",geometry="geometry_model",
  control="recovery_control",boostback="boostback_plan",mission="mission_control",
  hover="hover_control",runtime="mission_runtime"}) do deps[key]=require(file) end
deps.math=assert(loadfile("home.lua"),"Missing home.lua math module")("--test")
local config,driver,session
if live then
  assert(args[2],"Live mode requires a measured config; no default profile")
  local h=assert(fs.open(args[2],"r"),"Cannot read configuration")
  local content=h.readAll(); h.close()
  config=assert(textutils.unserializeJSON(content),"Invalid JSON configuration")
  driver=require("engine_io").new({clock=os.clock,peripheral=peripheral,redstone=redstone,
    parallel=parallel,sleep=sleep},config.caps)
  -- Validate before initialize: a rejected profile causes NO output writes.
  session=require("flight_session").new(deps,driver,config)
end
local basis=config and config.craft.basis or {up={x=0,y=1,z=0},a={x=1,y=0,z=0},b={x=0,y=0,z=-1}}
local sampler=require("telemetry").new({clock=os.clock,sublevel=sublevel,parallel=parallel,peripheral=peripheral},deps.math,basis)
local running,sample,message=true,nil,"Waiting for fresh telemetry"
local function draw()
  local width,height=term.getSize()
  term.setBackgroundColor(colors.black); term.setTextColor(colors.white); term.clear()
  local function line(y,text,color)
    if y>height then return end
    term.setCursorPos(1,y); term.setTextColor(color or colors.white); term.write(text:sub(1,width))
  end
  line(1,"ROCKET | "..(live and session.mode or "READ ONLY"),colors.cyan)
  line(2,"Stages: RETURN > COAST > BRAKE > HOVER")
  if sample then
    line(3,("Y %.2f  Vy %+.2f  Tilt %.1f"):format(sample.y,sample.vy,math.deg(sample.tilt)))
    line(4,("Read %.3fs | links %s"):format(sample.acquisitionSeconds,sample.allNineReady and "ACK OK" or "NOT READY"))
  else line(3,"Telemetry unavailable",colors.orange) end
  local out=session and session.output
  line(5,"Phase: "..(out and out.plan and out.plan.phase or "--"))
  line(6,"Pad: "..(session and session.pad and "SET (session only)" or "NOT SET"))
  line(8,"J pad (supported) | E arm at zero")
  line(9,"R/F manual +/-1% | V hover here")
  line(10,"D descent return | B boostback return")
  line(11,"SPACE emergency cutoff | Q exit",colors.red)
  line(12,"No upper stage / tower hardware",colors.orange)
  line(14,live and "Fault/exit cuts engines; NOT a safe landing" or "READ ONLY: no engine writes",colors.orange)
  line(height,message,colors.yellow)
end
local function producer()
  parallel.waitForAny(function()
    while running do
      local s,why=sampler:read(driver); sample=s
      if not s then
        message=why
        if session and session.mode~="DISARMED" then session:stop(why) end
      elseif session then
        local _,err=session:tick(s,os.clock()); if err then message=err end
      end
      draw(); sleep(.05)
    end
  end,function()
    local actions={[keys.j]="pad",[keys.e]="arm",[keys.r]="up",[keys.f]="down",
      [keys.v]="hover",[keys.d]="descent",[keys.b]="boostback",[keys.space]="stop"}
    while running do
      local event,key,held=os.pullEventRaw()
      if event=="terminate" or (event=="key" and key==keys.q) then
        if session then session:stop("Operator exit") end
        running=false; return
      end
      if event=="key" and not held and actions[key] then
        if session then
          local ok,why=session:action(actions[key],sample,os.clock())
          message=ok and (session.message or actions[key]) or why
        else message="Read-only mode: controls disabled" end
        draw()
      end
    end
  end)
end
if live then
  print("LIVE initialization clears nine engines. No automatic ignition.")
  local ok,why=driver:initialize(); assert(ok,why)
  local cleared,errors=driver:run(producer)
  if not cleared then print("CLEANUP FAILED: "..table.concat(errors,"; ")) end
else producer() end
