"""No-engine-write default, explicit layout binding, invalid config rejection."""
from pathlib import Path
import test_wired as t
import build_bundle

lua=t.LuaRuntime(unpack_returned_tuples=True)
lua.globals().entry=build_bundle.build_calibration()
lua.execute('''
local calls=0
print=function() end
peripheral={call=function() calls=calls+1; error("Unexpected call") end}
assert(load(entry,"calibration","t",_ENV))()
assert(calls==0,"Default must be inert")
assert(load(entry,"calibration","t",_ENV))("--invalid")
assert(calls==0)
''')
lua.globals().layout=lua.execute(Path('calibration_layout.lua').read_text(encoding='utf-8'))
lua.execute('''
local mass,id,gate=93.5,"fresh-copy-id",0
local inv={
  {.00030399675208276082,-.0000050530656694836421,.000000050463266118319841},
  {-.0000050530656694836421,.0048905193061743743,-.0000049823819850898534},
  {.000000050463266118319841,-.0000049823819850898534,.00029974325352827909}}
local reads=0
sublevel={isInPlotGrid=function() return true end,getUniqueId=function() return id end,
  getMass=function() return mass end,getInverseInertiaTensor=function() return inv end}
peripheral={hasType=function() return true end,call=function(_,method)
  assert(method=="getThrust","No setter allowed during geometry preparation")
  reads=reads+1; return gate
end}
local config=layout.inspect({sublevel=sublevel,peripheral=peripheral})
assert(config.craft.id==id and reads==9 and #config.craft.main==5 and #config.craft.top==4)
assert(config.craft.top[1].facing=="north" and config.craft.top[4].facing=="east")
mass=71; assert(not pcall(layout.inspect,{sublevel=sublevel,peripheral=peripheral})); mass=93.5
gate=15; assert(not pcall(layout.inspect,{sublevel=sublevel,peripheral=peripheral})); gate=0
inv[1][1]=.5; assert(not pcall(layout.inspect,{sublevel=sublevel,peripheral=peripheral}))
local source='{}'
fs={open=function() return {readAll=function() return source end,close=function() end} end}
textutils={unserializeJSON=function() return config end}
id="different-copy"
local ok,err=pcall(assert(load(entry,"calibration","t",_ENV)),"--live","config.json")
assert(not ok and tostring(err):find("Different craft"),tostring(err))
''')
print('PASS inert default, explicit unchanged-layout inspection, no setters, new copy ID, changed mass/inertia/gate rejection and live identity guard.')
