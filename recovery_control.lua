-- Connects planner, attitude guidance and physical allocation in SHADOW mode.
-- Returns nine engine requests; never writes peripherals or calibration files.
-- dependency injection: planner, allocator, geometry and home --test math.
local M={version=1}
local function finite(n) return type(n)=="number" and n==n and math.abs(n)<math.huge end
local function vec(x,y,z) return {x=x,y=y,z=z} end
local function dot(a,b) return a.x*b.x+a.y*b.y+a.z*b.z end
function M.step(deps,state,sample,now,craft,limits,stagePlan)
  local ok,result=pcall(function()
    assert(craft.id==state.profile.craftId,"Geometry belongs to another craft")
    assert(finite(craft.profile.mass) and math.abs(craft.profile.mass/state.profile.mass-1)<=0.02,"Geometry mass mismatch")
    assert(type(sample.q)=="table" and type(sample.w)=="table","Missing attitude/rate")
    for _,k in ipairs({"x","y","z"}) do assert(finite(sample.w[k]),"Invalid body rate") end
    local q=deps.math.quaternion(sample.q)
    local inverse={x=-q.x,y=-q.y,z=-q.z,w=q.w}
    local basis=assert(craft.basis,"Missing upright body reference")
    assert(math.abs(basis.up.x)<1e-5 and math.abs(basis.up.y-1)<1e-5 and math.abs(basis.up.z)<1e-5,"Geometry requires body +Y upright")
    local e1,e2=deps.math.error(q,basis,stagePlan and stagePlan.targetUp)
    local _,_,tilt=deps.math.error(q,basis)
    local a1=.65*e1-1.43*dot(sample.w,basis.a)
    local a2=.65*e2-1.43*dot(sample.w,basis.b)
    -- Yaw damping is requested but may be unreachable with radial side jets.
    -- Residuals stay visible; a least-squares fit is never flight verification.
    local spin=-1.43*dot(sample.w,basis.up)
    local angular=vec(basis.a.x*a1+basis.b.x*a2+basis.up.x*spin,
      basis.a.y*a1+basis.b.y*a2+basis.up.y*spin,
      basis.a.z*a1+basis.b.z*a2+basis.up.z*spin)
    local function allocate(lateral,throttle)
      local force=5*733*throttle/craft.profile.mass
      local world=vec(lateral.ax,force*math.cos(tilt),lateral.az)
      if stagePlan and stagePlan.alignThrustWithTarget then
        world=vec(stagePlan.targetUp.x*force,stagePlan.targetUp.y*force,stagePlan.targetUp.z*force)
      end
      local body=deps.math.rotate(inverse,world)
      body.y=math.max(0,body.y)
      local allowed=limits
      if throttle==0 then
        -- Rotating a horizontal world request may produce body-Y demand.
        -- COAST must still mean exactly zero main thrust, not a tiny ignition.
        body.y=0; allowed={}; for k,v in pairs(limits) do allowed[k]=v end
        allowed.mainKN=0
      end
      return deps.allocator.solve(deps.geometry,craft.profile,craft.main,craft.top,
        {linear=body,angular=angular},allowed)
    end
    local input={}; for k,v in pairs(sample) do input[k]=v end
    input.tilt=tilt
    input.angularSpeed=math.sqrt(dot(sample.w,sample.w))
    if not stagePlan and state.phase=="COAST" then
      local l=deps.planner.lateralPlan(state.profile,state.pad,sample,sample.y-state.targetY)
      local coast=allocate(l,0)
      input.controlAuthorityLimited=sample.controlAuthorityLimited==true or coast.requiresPoweredSupport
    end
    local plan=stagePlan or deps.planner.step(state,input,now)
    if plan.phase=="FAULT" or plan.handoffHover then
      return {plan=plan,shadowOnly=true,flightValidated=false,commands=nil,
        handoffHover=plan.handoffHover,manualTakeover=plan.phase=="FAULT"}
    end
    assert(finite(plan.mainThrottle) and plan.mainThrottle>=0 and plan.mainThrottle<=state.profile.maxThrottle,"Planner produced invalid thrust request")
    local allocation=allocate(plan.lateral,plan.mainThrottle)
    local commands={}
    for i=0,4 do commands[i+1]={name="creative_vector_thruster_"..i,
      kn=allocation.mainKN,vectorX=allocation.mainX/15,vectorY=allocation.mainY/15} end
    for i=5,8 do commands[i+1]={name="creative_vector_thruster_"..i,
      kn=allocation.topKN[i-4],vectorX=0,vectorY=0} end
    return {plan=plan,allocation=allocation,commands=commands,shadowOnly=true,flightValidated=false}
  end)
  if not ok then
    -- Invalid inputs produce no commands, not an implicit midair engine cutoff.
    state.phase="FAULT"
    return {shadowOnly=true,flightValidated=false,commands=nil,manualTakeover=true,error=tostring(result)}
  end
  return result
end
return M
