-- Translation-only copy of the inspected nine-engine layout. Not auto-discovery.
-- The operator must confirm unchanged block layout, orientations and engine IDs.
local M={}
local reference={
  {.00030399675208276082,-.0000050530656694836421,.000000050463266118319841},
  {-.0000050530656694836421,.0048905193061743743,-.0000049823819850898534},
  {.000000050463266118319841,-.0000049823819850898534,.00029974325352827909}}
local function finite(x) return type(x)=="number" and x==x and math.abs(x)<math.huge end
function M.inspect(api)
  assert(api.sublevel.isInPlotGrid(),"Computer must be on assembled copy")
  local id=api.sublevel.getUniqueId(); local mass=api.sublevel.getMass()
  assert(type(id)=="string" and #id>0,"Missing copy identity")
  assert(finite(mass) and math.abs(mass/93.5-1)<.005,"Copy mass differs from inspected 93.5 layout; inspect again")
  local inv=api.sublevel.getInverseInertiaTensor()
  for i=1,3 do for j=1,3 do
    local value=type(inv)=="table" and type(inv[i])=="table" and inv[i][j]
    assert(finite(value) and math.abs(value-reference[i][j])<math.max(1e-8,math.abs(reference[i][j])*.01),"Copy inertia differs from inspected layout")
  end end
  for i=0,8 do
    local name="creative_vector_thruster_"..i
    assert(api.peripheral.hasType(name,"creative_vector_thruster"),"Missing engine "..i)
    assert(api.peripheral.call(name,"getThrust")==0,"Stop old controller and all engines first")
  end
  local function center(x,y,z) return {x=x+.0053476,y=y-3.2032085561497,z=z+.005347591} end
  local main={}
  for _,p in ipairs({{-1,-1},{-1,1},{1,1},{1,-1},{0,0}}) do main[#main+1]={center=center(p[1],-5,p[2]),facing="up"} end
  local top={}
  for _,p in ipairs({{0,2,"north"},{2,0,"west"},{0,-2,"south"},{-2,0,"east"}}) do top[#top+1]={center=center(p[1],10,p[2]),facing=p[3]} end
  return {version=1,layoutSource="operator-confirmed unchanged inspected nine-engine layout",
    craft={id=id,profile={mass=mass,com={x=0,y=0,z=0},inverseInertia=inv},main=main,top=top,
      basis={up={x=0,y=1,z=0},a={x=1,y=0,z=0},b={x=0,y=0,z=-1}}}}
end
return M
