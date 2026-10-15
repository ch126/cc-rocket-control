-- Experimental flight commissioning. Explicit configuration, then P to ignite.
-- Tests can crash the copy. SPACE/Q/Ctrl+T cut thrust; NOT a safe landing.
local args={...}
if args[1]=="--prepare-same-layout" then
  print("READ ONLY engines: preparing geometry for an UNCHANGED copy.")
  print("Requires same blocks, facing and engine IDs. Not auto-discovery.")
  local config=require("calibration_layout").inspect({sublevel=sublevel,peripheral=peripheral})
  local payload=textutils.serializeJSON(config)
  local free=fs.getFreeSpace("/")
  assert(type(free)~="number" or free>#payload+8192,"Insufficient storage")
  local path
  for i=1,9999 do
    local candidate=("/rocket-copy-geometry-%04d.json"):format(i)
    if not fs.exists(candidate) then path=candidate; break end
  end
  assert(path,"Geometry names exhausted")
  local f=assert(fs.open(path,"w")); local ok,why=pcall(f.write,payload); f.close(); assert(ok,why)
  print("Saved "..path..". No ignition/calibration performed.")
  print("Next: this program --live "..path)
  return
end
if args[1]~="--live" or not args[2] then
  print("Usage: calibration_app --live craft-geometry.json")
  print("COPY + clear test area only. Old controller must be stopped.")
  print("No engine commands sent. Config requires verified craft geometry.")
  print("Unchanged inspected copy: --prepare-same-layout")
  return
end
local h=assert(fs.open(args[2],"r"),"Missing geometry configuration")
local raw=h.readAll(); h.close()
local config=assert(textutils.unserializeJSON(raw),"Invalid JSON")
local craft=assert(config.craft,"Missing craft geometry")
local deps={geometry=require("geometry_model"),allocator=require("control_allocator"),evidence=require("calibration_evidence")}
deps.math=assert(loadfile("home.lua"))("--test")
local controller=require("calibration_flight")
assert(type(craft.id)=="string" and #craft.main==5 and #craft.top==4,"Need identity and 5+4 engines")
assert(craft.basis.up.x==0 and craft.basis.up.y==1 and craft.basis.up.z==0,"Only plot +Y upright layout supported")
assert(craft.basis.a.x==1 and craft.basis.a.y==0 and craft.basis.a.z==0 and
  craft.basis.b.x==0 and craft.basis.b.y==0 and craft.basis.b.z==-1,"Invalid control basis")
for _,e in ipairs(craft.main) do assert(e.facing=="up","Main facing mismatch"); deps.geometry.response(craft.profile,e,0,0,0) end
for _,e in ipairs(craft.top) do
  assert(e.facing=="north" or e.facing=="south" or e.facing=="east" or e.facing=="west","Top facing mismatch")
  deps.geometry.response(craft.profile,e,0,0,0)
end
assert(sublevel.getUniqueId()==craft.id,"Different craft: inspect the COPY; do not reuse the original ID")
assert(math.abs(sublevel.getMass()/craft.profile.mass-1)<.02,"Mass changed; geometry inspection required")
local free=fs.getFreeSpace("/")
assert(type(free)~="number" or free>=65536,"Need 64 KiB free for a new report; nothing deleted")
local path
for i=1,9999 do
  local candidate=("/rocket-flight-calibration-%04d.json"):format(i)
  if not fs.exists(candidate) then path=candidate; break end
end
assert(path,"Report names exhausted")
local driver=require("engine_io").new({clock=os.clock,peripheral=peripheral,redstone=redstone,
  parallel=parallel,sleep=sleep},{403.15,403.15,403.15,403.15,403.15,40,40,40,40})
local sampler=require("telemetry").new({clock=os.clock,sublevel=sublevel,parallel=parallel,peripheral=peripheral},deps.math,craft.basis)
local state,sample,output,running,savePending,saved=nil,nil,nil,true,false,false
local message="P: start AUTO FLIGHT on the copy. SPACE: CUT. Q: CUT + exit."
local function draw()
  term.setCursorPos(1,1); term.clear()
  print("NINE AUTO FLIGHT COMMISSIONING - EXPERIMENTAL")
  print(state and state.phase or "DISARMED - P starts flight")
  if sample then
    print(("Y %.2f Vy %+.2f Tilt %.2f deg"):format(sample.y,sample.vy,math.deg(sample.tilt)))
    if state then print(("Height %.2f | vertical %d/4 | trial %d"):format(sample.y-state.pad.y,math.min(state.window,4),state.trial)) end
  end
  print("SPACE/Q: thrust OFF, may fall. No auto landing.")
  print("Finish: HOLD at +12; keep program running.")
  print(message)
end
local function producer()
  parallel.waitForAny(function()
    while running do
      local applied={}
      for i,e in ipairs(driver.engines) do applied[i]=e.acknowledged end
      local s,why=sampler:read(driver); sample=s
      if not s then
        message=why
        if state then driver:stop(why); running=false; return end
      elseif state then
        -- Backend acknowledgement is not a force measurement. Actual response
        -- is measured separately from raw velocity and quaternion changes.
        s.applied=applied
        output=controller.step(deps,state,s,os.clock())
        if output.manualTakeover then message=output.error; driver:stop(message); running=false; return end
        local ok,err=driver:submit(output.commands)
        if not ok then message=err; running=false; return end
        if output.result and not saved and not savePending then savePending=true; os.queueEvent("commission_save") end
      end
      draw(); sleep(.05)
    end
  end,function()
    while running do
      local event,key,held=os.pullEventRaw()
      if event=="terminate" or (event=="key" and (key==keys.q or key==keys.space)) then
        message="Operator cutoff; no landing guaranteed"; driver:stop(message); running=false; return
      end
      if event=="key" and key==keys.p and not held and not state then
        local ok,why=pcall(function()
          assert(sample and os.clock()-sample.time<=.25,"Wait for fresh telemetry")
          local newState=controller.start(craft,sample,os.clock())
          local armed,reason=driver:arm(); assert(armed,reason)
          state=newState; message="Automatic commissioning running"
        end)
        if not ok then message=tostring(why) end
      end
      if event=="commission_save" and output and output.result and not saved then
        local ok,why=pcall(function()
          local payload=textutils.serializeJSON({version=1,result=output.result,windows=state.windows,
            pad=state.pad,geometry=craft,completedAt=os.clock(),scope="local low-speed commissioning only"})
          local available=fs.getFreeSpace("/")
          assert(type(available)~="number" or available>#payload+8192,"Report storage full")
          assert(not fs.exists(path),"Report exists; refusing overwrite")
          local file=assert(fs.open(path,"w"),"Cannot open new report")
          local wrote,err=pcall(file.write,payload); file.close(); assert(wrote,err)
        end)
        -- Saving failure does not terminate airborne control or overwrite old calibration.
        saved=true
        message=ok and ("REPORT SAVED "..path.."; continue hovering. NOT a recovery profile.") or ("SAVE FAILED; hover continues: "..tostring(why))
      end
      if event=="term_resize" then draw() end
    end
  end)
end
print("Initialization clears all nine engines. P separately starts flight.")
local ok,why=driver:initialize(); assert(ok,why)
local cleared,errors=driver:run(producer)
term.setCursorPos(1,1); term.clear(); print(message)
print(cleared and "Nine output cleanup acknowledged." or ("CLEANUP FAILED: "..table.concat(errors,"; ")))
