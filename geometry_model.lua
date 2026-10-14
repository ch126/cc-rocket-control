-- Pure, steady-state analytical model for Create Propulsion 1.1.5 + Sable.
-- No peripherals, filesystem, calibration records, or flight authorization.
-- Input positions and inertia MUST share the ship/plot coordinate frame.
-- kN numeric values become Sable force units after internal /1000 conversion.
local M={version=1,flightValidated=false}
local function finite(n) return type(n)=="number" and n==n and math.abs(n)<math.huge end
local function vector(v)
  assert(type(v)=="table" and finite(v.x) and finite(v.y) and finite(v.z),"Invalid vector")
  return {x=v.x,y=v.y,z=v.z}
end
local function add(a,b) return {x=a.x+b.x,y=a.y+b.y,z=a.z+b.z} end
local function mul(a,s) return {x=a.x*s,y=a.y*s,z=a.z*s} end
local function cross(a,b) return {x=a.y*b.z-a.z*b.y,y=a.z*b.x-a.x*b.z,z=a.x*b.y-a.y*b.x} end
local function unit(v)
  local n=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z); assert(n>1e-9,"Zero direction")
  return mul(v,1/n)
end
local directions={up={x=0,y=1,z=0},down={x=0,y=-1,z=0},
  east={x=1,y=0,z=0},west={x=-1,y=0,z=0},south={x=0,y=0,z=1},north={x=0,y=0,z=-1}}
function M.direction(facing,x,y)
  local f=assert(directions[facing],"Unknown facing")
  assert(finite(x) and finite(y) and math.abs(x)<=1 and math.abs(y)<=1,"Invalid vector setting")
  local vertical=math.abs(f.y)>.999
  local seed=vertical and directions.south or directions.up
  local right=unit(cross(f,seed)); if vertical then right=mul(right,-1) end
  local up=unit(cross(right,f)); local t=math.tan(math.pi/6)
  return unit(add(f,add(mul(right,-x*t),mul(up,-y*t))))
end
local function matrix(a)
  assert(type(a)=="table","Missing inverse inertia")
  for i=1,3 do
    assert(type(a[i])=="table","Invalid inertia row")
    for j=1,3 do
      assert(finite(a[i][j]),"Invalid inertia value")
      assert(type(a[j])=="table" and finite(a[j][i]) and math.abs(a[i][j]-a[j][i])<1e-8,"Inertia not symmetric")
    end
  end
  local d2=a[1][1]*a[2][2]-a[1][2]^2
  local det=a[1][1]*(a[2][2]*a[3][3]-a[2][3]^2)-a[1][2]*(a[1][2]*a[3][3]-a[2][3]*a[1][3])+a[1][3]*(a[1][2]*a[2][3]-a[2][2]*a[1][3])
  assert(a[1][1]>0 and d2>0 and det>0,"Inertia not positive definite")
  return a
end
local function transform(a,v)
  return {x=a[1][1]*v.x+a[1][2]*v.y+a[1][3]*v.z,
    y=a[2][1]*v.x+a[2][2]*v.y+a[2][3]*v.z,
    z=a[3][1]*v.x+a[3][2]*v.y+a[3][3]*v.z}
end
function M.response(profile,engine,kn,x,y)
  assert(finite(profile.mass) and profile.mass>0,"Invalid mass")
  assert(finite(kn) and kn>=0,"Invalid force")
  local inv=matrix(profile.inverseInertia)
  local lever=add(vector(engine.center),mul(vector(profile.com),-1))
  local force=mul(M.direction(engine.facing,x or 0,y or 0),kn)
  -- Actual application point also adds nozzleOffset * forceDirection.
  -- Its extra torque is identically zero because direction cross force = 0.
  local torque=cross(lever,force)
  return {force=force,torque=torque,linear=mul(force,1/profile.mass),
    angular=transform(inv,torque),flightValidated=false}
end
function M.total(profile,engines,kn,x,y)
  local f,t={x=0,y=0,z=0},{x=0,y=0,z=0}
  assert(type(engines)=="table" and #engines>0,"No engines")
  for _,e in ipairs(engines) do
    local r=M.response(profile,e,kn,x,y); f=add(f,r.force); t=add(t,r.torque)
  end
  return {force=f,torque=t,linear=mul(f,1/profile.mass),angular=transform(profile.inverseInertia,t),flightValidated=false}
end
-- Actual settled +one-step minus neutral, NOT instantaneous servo response.
function M.mainColumns(profile,engines,kn)
  local zero=M.total(profile,engines,kn,0,0)
  local x=M.total(profile,engines,kn,1/15,0)
  local y=M.total(profile,engines,kn,0,1/15)
  return {neutral=zero,x=add(x.angular,mul(zero.angular,-1)),
    y=add(y.angular,mul(zero.angular,-1)),flightValidated=false}
end
return M
