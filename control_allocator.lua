-- Bounded, side-effect-free nine-engine allocation, NOT a peripheral driver.
-- Inputs are body-frame PROPULSIVE accelerations (gravity handled by guidance).
-- Geometry predictions are steady-state; callers must account for nozzle lag.
local M={version=1}
local function finite(x) return type(x)=="number" and x==x and math.abs(x)<math.huge end
local function clamp(x,a,b) return math.max(a,math.min(b,x)) end
local function six(r) return {r.linear.x,r.linear.y,r.linear.z,r.angular.x,r.angular.y,r.angular.z} end
local function norm3(x,y,z) return math.sqrt(x*x+y*y+z*z) end
function M.solve(model,profile,main,top,request,limits)
  assert(#main==5 and #top==4,"Expected five main and four top engines")
  assert(finite(limits.mainKN) and limits.mainKN>=0 and limits.mainKN<=733,"Invalid main cap")
  assert(finite(limits.steps) and limits.steps>=0 and limits.steps<=3 and limits.steps%1==0,"Invalid gimbal cap")
  local target={request.linear.x,request.linear.y,request.linear.z,request.angular.x,request.angular.y,request.angular.z}
  for i=1,6 do assert(finite(target[i]),"Invalid acceleration target") end
  assert(target[2]>=0,"Negative main-axis acceleration unsupported")
  local original={}; for i=1,6 do original[i]=target[i] end
  local dt=limits.dt or .1
  assert(finite(dt) and dt>0 and dt<=.25,"Invalid allocation interval")
  local balance=limits.balance or {0,0,0,0,0,0}
  for i=1,6 do
    assert(finite(balance[i]),"Invalid allocation balance")
    -- Main gimbal steps are coarse. Carry bounded acceleration-time residual
    -- across frames instead of allowing small requests to round to zero forever.
    if i~=2 then target[i]=target[i]+clamp(balance[i],i<=3 and -.2 or -.01,i<=3 and .2 or .01)/dt end
  end
  local previous=limits.previous or {x=0,y=0}
  assert(finite(previous.x) and finite(previous.y) and previous.x%1==0 and previous.y%1==0,"Invalid previous steps")
  local slew=limits.slewSteps or 1
  assert(slew==0 or slew==1,"Slew must be zero or one physical step")
  local weights={1,1,1,10000,10000,10000}
  local cols={}
  for j=1,4 do
    assert(finite(limits.topKN[j]) and limits.topKN[j]>=0 and limits.topKN[j]<=733,"Invalid top cap")
    cols[j]=six(model.response(profile,top[j],1,0,0))
    assert(math.abs(cols[j][2])<1e-9,"Top thruster is not horizontal")
  end
  for _,e in ipairs(main) do assert(e.facing=="up","Main engines must face body up") end
  local best
  for x=-limits.steps,limits.steps do for y=-limits.steps,limits.steps do
    if math.abs(x-previous.x)<=slew and math.abs(y-previous.y)<=slew then
      local d=model.direction("up",x/15,y/15)
      local kn=clamp(target[2]*profile.mass/(5*d.y),0,limits.mainKN)
      local baseline=six(model.total(profile,main,kn,x/15,y/15))
      local u={0,0,0,0}; local residual={}
      for i=1,6 do residual[i]=target[i]-baseline[i] end
      -- Convex box-constrained weighted least squares for the four one-way RCS.
      -- Exact coordinate updates, bounded iteration count and tiny fuel penalty.
      for pass=1,80 do
        local largest=0
        for j=1,4 do
          local numerator,denominator=-1e-8*u[j],1e-8
          for i=1,6 do
            numerator=numerator+weights[i]*cols[j][i]*residual[i]
            denominator=denominator+weights[i]*cols[j][i]^2
          end
          local nextValue=clamp(u[j]+numerator/denominator,0,limits.topKN[j])
          local delta=nextValue-u[j]; u[j]=nextValue; largest=math.max(largest,math.abs(delta))
          for i=1,6 do residual[i]=residual[i]-cols[j][i]*delta end
        end
        if largest<1e-7 then break end
      end
      local cost=0
      for i=1,6 do cost=cost+weights[i]*residual[i]^2 end
      for j=1,4 do cost=cost+1e-8*u[j]^2 end
      -- Resolve exact/near ties without pointless nozzle motion when coasting.
      cost=cost+1e-10*(x*x+y*y)
      if not best or cost<best.cost then
        best={cost=cost,mainKN=kn,mainX=x,mainY=y,topKN=u,residual=residual,
          predicted={},flightValidated=false,shadowOnly=true}
        for i=1,6 do best.predicted[i]=target[i]-residual[i] end
      end
    end
  end end
  assert(best,"No reachable gimbal setting under slew limit")
  best.balance={}
  for i=1,6 do
    best.residual[i]=original[i]-best.predicted[i]
    local cap=i<=3 and .2 or .01
    best.balance[i]=i==2 and 0 or clamp(balance[i]+best.residual[i]*dt,-cap,cap)
  end
  best.angularError=norm3(best.residual[4],best.residual[5],best.residual[6])
  best.lateralError=norm3(best.residual[1],0,best.residual[3])
  best.verticalError=math.abs(best.residual[2])
  best.authorityLimited=best.angularError>0.005 or best.lateralError>0.05 or best.verticalError>0.1
  best.requiresPoweredSupport=best.mainKN==0 and best.authorityLimited
  return best
end
return M
