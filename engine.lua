-- CC: Tweaked / Create Propulsion: five engines share five redstone channels.
-- Relative computer faces, not world compass directions. No peripheral binding.
local sides = { "bottom", "front", "back", "left", "right" }
local armed, throttle, pitch, yaw = false, 0, 0, 0
local running = true
local function clamp(n, lo, hi) return math.max(lo, math.min(hi, n)) end

local function clearOutputs()
  -- Try every side even if one write fails. Leave the unused top face alone.
  local errors = {}
  for _, side in ipairs(sides) do
    local ok, err = pcall(redstone.setAnalogOutput, side, 0)
    if not ok then errors[#errors + 1] = side .. ": " .. tostring(err) end
  end
  return errors
end

local function stop()
  armed, throttle, pitch, yaw = false, 0, 0, 0
  local errors = clearOutputs()
  if #errors > 0 then error(table.concat(errors, "; ")) end
end

local function writePair(positiveSide, negativeSide, value)
  -- Remove opposing output FIRST, including when crossing through zero.
  if value > 0 then
    redstone.setAnalogOutput(negativeSide, 0)
    redstone.setAnalogOutput(positiveSide, value)
  else
    redstone.setAnalogOutput(positiveSide, 0)
    redstone.setAnalogOutput(negativeSide, -value)
  end
end

local function apply()
  if not armed then stop(); return end
  writePair("front", "back", pitch)
  writePair("right", "left", yaw)
  redstone.setAnalogOutput("bottom", throttle)
end

local function draw()
  term.clear()
  term.setCursorPos(1, 1)
  print("ROCKET ENGINE CONTROL v0.1.0")
  print(armed and "ARMED - manual control" or "DISARMED - all outputs zero")
  print(("Thrust [bottom]: %2d / 15"):format(throttle))
  print(("Front/back: %+3d  Right/left: %+3d"):format(pitch, yaw))
  print("")
  print("E       Arm at zero / disarm")
  print("R / F   Thrust +1 / -1")
  print("W / S   Front/back trim +1 / -1")
  print("D / A   Right/left trim +1 / -1")
  print("C       Center nozzle (keeps thrust)")
  print("SPACE   Stop + disarm")
  print("Q       Stop + exit")
  print("")
  print("Tap keys; repeats ignored. Trim stays set.")
  print("5 engines share signals; no auto-level.")
end

local function main()
  stop()
  draw()
  while running do
    local event, key, held = os.pullEventRaw()
    if event == "terminate" then
      running = false
    elseif event == "key" and not held then
      if key == keys.q then
        running = false
      elseif key == keys.space then
        stop()
      elseif key == keys.e then
        local wasArmed = armed
        stop()
        armed = not wasArmed
      elseif armed then
        if key == keys.r then throttle = clamp(throttle + 1, 0, 15)
        elseif key == keys.f then throttle = clamp(throttle - 1, 0, 15)
        elseif key == keys.w then pitch = clamp(pitch + 1, -15, 15)
        elseif key == keys.s then pitch = clamp(pitch - 1, -15, 15)
        elseif key == keys.d then yaw = clamp(yaw + 1, -15, 15)
        elseif key == keys.a then yaw = clamp(yaw - 1, -15, 15)
        elseif key == keys.c then pitch, yaw = 0, 0 end
      end
      if running then apply(); draw() end
    elseif event == "term_resize" then
      draw()
    end
  end
end

local ok, err = pcall(main)
local cleanupErrors = clearOutputs()
term.clear()
term.setCursorPos(1, 1)
if #cleanupErrors == 0 then
  print("Stopped. All five outputs are zero.")
else
  print("Could not clear every output: " .. table.concat(cleanupErrors, "; "))
end
if not ok then error(err, 0) end
