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
local view=require("console_view")
local buttons={}
local function draw()
  buttons=view.draw(term,colors,{live=live,mode=session and session.mode,sample=sample,
    output=session and session.output,pad=session and session.pad,message=message})
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
      local event,key,held,y=os.pullEventRaw()
      local action
      if event=="key" and not held then action=actions[key] end
      if event=="mouse_click" and key==1 then action=view.hit(buttons,held,y) end
      if event=="terminate" or (event=="key" and key==keys.q) or action=="exit" then
        if session then session:stop("Operator exit") end
        running=false; return
      end
      if event=="term_resize" then draw() end
      if action then
        if session then
          local ok,why=session:action(action,sample,os.clock())
          message=ok and (session.message or action) or why
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
