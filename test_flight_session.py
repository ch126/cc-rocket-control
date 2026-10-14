"""Operator transitions with actual guidance modules and recording driver."""
import test_mission as t
a=t.a; lua=t.lua; deps=t.deps
deps.mission=t.missionmod; deps.hover=t.load('hover_control.lua'); deps.runtime=t.load('mission_runtime.lua')
lua.globals().sessionmod=t.load('flight_session.lua'); lua.globals().deps=deps
lua.globals().config=a.tab(dict(profile=t.p,craft=t.craft,limits=a.limits,caps=[733]*5+[100]*4))
lua.execute('''
local d={ready=true,armed=false,count=0,
  stop=function(self) self.armed=false end,
  arm=function(self) self.armed=true; return true end,
  submit=function(self,c) assert(self.armed); self.count=self.count+1; self.last=c; return true end,
  caps=config.caps}
local s={time=0,x=0,y=0,z=0,vx=0,vy=0,vz=0,tilt=0,angularSpeed=0,mass=100,
  craftId="test",allNineReady=true,q={x=0,y=0,z=0,w=1},w={x=0,y=0,z=0}}
local session=sessionmod.new(deps,d,config)
assert(not session:action("arm",s,0)); assert(not d.armed and d.count==0)
assert(session:action("pad",s,0)); assert(session:action("arm",s,0))
s.time=.1; assert(session:tick(s,.1)); for i=1,9 do assert(d.last[i].kn==0) end
assert(not session:action("descent",s,.1)); assert(session.mode=="MANUAL")
assert(session:action("up",s,.1)); s.time=.2; assert(session:tick(s,.2)); assert(d.last[1].kn>0)
assert(not session:action("pad",s,.2))
assert(session:action("hover",s,.2)); s.time=.3; assert(session:tick(s,.3)); assert(session.mode=="HOVER")
assert(not session:action("up",s,.3))
s.time=.4; s.y=100; assert(session:action("descent",s,.4))
s.time=.5; assert(session:tick(s,.5)); assert(session.mode=="MISSION")
local count=d.count; assert(not session:tick(s,1)); assert(session.mode=="DISARMED" and not d.armed and d.count==count)
assert(session:action("stop",nil,2)); assert(session.output==nil)
config.profile.verticalVerified=false
assert(not pcall(sessionmod.new,deps,d,config))
''')
print('PASS session: guarded arm, zero-output arm, manual/hover/descent, invalid actions, stale telemetry cutoff, unverified profile rejection.')
