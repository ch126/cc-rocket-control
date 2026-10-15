-- Flight measurement acceptance. Pure computation: no ignition or file writes.
-- This module cannot manufacture a complete flight profile from partial trials.
local M={version=1}
local function finite(n) return type(n)=="number" and n==n and math.abs(n)<math.huge end
local function fit(x,y)
  local n=#x; assert(n>=4 and #y==n,"Insufficient regression samples")
  local mx,my=0,0
  for i=1,n do mx=mx+x[i]/n; my=my+y[i]/n end
  local xx,xy=0,0
  for i=1,n do xx=xx+(x[i]-mx)^2; xy=xy+(x[i]-mx)*(y[i]-my) end
  assert(xx>1e-8,"No useful input variation")
  local slope=xy/xx; local intercept=my-slope*mx; local residual=0
  for i=1,n do residual=residual+(y[i]-intercept-slope*x[i])^2 end
  return slope,intercept,math.sqrt(residual/n),xx,mx
end
local function validateSample(s,identity,mass)
  for _,key in ipairs({"time","mass","relativeHeight","tilt","vy","throttle","commandAge","settledFor"}) do
    assert(finite(s[key]),"Invalid sample field: "..key)
  end
  assert(s.craftId==identity and s.mass>0 and math.abs(s.mass/mass-1)<=.02,"Craft changed during calibration")
  assert(s.velocitySource=="sublevel.getLinearVelocity","Need raw velocity, not filtered legacy log")
  assert(s.relativeHeight>=3 and s.relativeHeight<=30,"Outside commissioning height envelope")
  assert(s.tilt>=0 and s.tilt<=math.rad(2),"Vertical trial not upright")
  assert(s.throttle>0 and s.throttle<=1,"Invalid applied throttle")
  assert(s.commandAge>=0 and s.commandAge<=.3 and s.settledFor>=.5,"Unconfirmed/unsettled output")
  assert(s.topKN==0 and s.vectorX==0 and s.vectorY==0,"Vertical trial includes lateral inputs")
  assert(s.allMainEqual==true,"Unequal main outputs")
end
function M.vertical(run)
  local ok,result=pcall(function()
    assert(type(run)=="table" and type(run.craftId)=="string" and #run.craftId>0 and finite(run.mass) and run.mass>0,"Missing flight identity")
    assert(type(run.windows)=="table" and #run.windows>=4,"Need repeated high/low thrust windows")
    local inputs,accelerations,errors={}, {}, {}
    local minimum,maximum=1,0; local previousEnd=-math.huge
    for _,window in ipairs(run.windows) do
      assert(type(window)=="table" and #window>=6,"Need at least six samples per window")
      local times,velocities={},{}; local throttle
      for i,s in ipairs(window) do
        validateSample(s,run.craftId,run.mass)
        if i==1 then assert(s.time>previousEnd,"Overlapping/replayed trial windows"); throttle=s.throttle
        else assert(s.time>window[i-1].time and s.time-window[i-1].time<=.25,"Gap/reversal in measurements") end
        assert(math.abs(s.throttle-throttle)<=1e-6,"Thrust changed inside measurement window")
        times[i]=s.time-window[1].time; velocities[i]=s.vy
      end
      previousEnd=window[#window].time
      assert(times[#times]>=.5,"Measurement window too short")
      local acceleration,_,rms,xx=fit(times,velocities)
      assert(rms<=.05,"Velocity response not steady enough")
      local uncertainty=math.max(.1,3*rms*math.sqrt(#times/((#times-2)*xx)))
      inputs[#inputs+1]=throttle; accelerations[#accelerations+1]=acceleration; errors[#errors+1]=uncertainty
      minimum=math.min(minimum,throttle); maximum=math.max(maximum,throttle)
    end
    assert(maximum-minimum>=.04,"Thrust separation too small to identify gravity and gain")
    local low,high=0,0
    for _,u in ipairs(inputs) do if u<=(minimum+maximum)/2 then low=low+1 else high=high+1 end end
    assert(low>=2 and high>=2,"Need repeated trials at both thrust levels")
    local gain,offset,rms,xx,mx=fit(inputs,accelerations)
    local maxError=0; for _,e in ipairs(errors) do maxError=math.max(maxError,e) end
    assert(rms<=math.max(.2,2*maxError),"Inconsistent vertical trials")
    -- Deterministic propagated error envelope, not a statistical guarantee.
    local slopeError=0
    for i,u in ipairs(inputs) do slopeError=slopeError+math.abs(u-mx)*(errors[i]+rms)/xx end
    local interceptError=maxError+rms+math.abs(mx)*slopeError
    local gravity=-offset
    assert(gain-slopeError>0 and gravity-interceptError>0,"Unidentifiable gravity/thrust bounds")
    assert(slopeError/gain<.2 and interceptError/gravity<.2,"Measurement uncertainty too large")
    local hover=gravity/gain
    assert(hover>minimum and hover<maximum,"Trials must bracket hover thrust")
    return {kind="vertical-candidate",craftId=run.craftId,mass=run.mass,
      gravityMin=gravity-interceptError,gravityMax=gravity+interceptError,
      accelPerThrottleMin=gain-slopeError,accelPerThrottleMax=gain+slopeError,
      hoverThrottle=hover,testedThrottleMin=minimum,testedThrottleMax=maximum,
      fitResidual=rms,windowCount=#inputs,flightCalibrationValid=false,
      missing={"ignition-delay","main-vector-response","top-engine-response","coupled-lateral-authority","braking-validation"}}
  end)
  if not ok then return nil,tostring(result) end
  return result,nil
end
return M
