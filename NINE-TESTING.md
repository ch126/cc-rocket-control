# Nine-engine experimental controller (v0.6.2-nine)

This is an experimental fork of `home.lua`, not the automatic freefall return
controller. The existing five-engine controller is unchanged. Do not run both.
Starting or exiting this program attempts to zero all nine engines. Start only
with the craft landed/supported and the previous controller stopped.

## Wiring and limits

- Main engines: `creative_vector_thruster_0` through `_4`, 733 kN each.
- Top engines: `_5` rear, `_6` left, `_7` front, `_8` right, nozzles outward.
- Top engines stay off during normal manual/H/V/B operation and main calibration.
- A selected top engine can receive a short diagnostic pulse, default 1 kN.
  The optional disarmed settings are 1, 2, and 4 kN, NOT 733 kN.
- Both `top` and `bottom` modem disconnects stop the controller. All six local
  redstone outputs are cleared. Remove legacy wireless redstone commands.

## Installation

In the CC shell, download to an unused filename:

```text
wget https://raw.githubusercontent.com/ch126/cc-rocket-control/main/nine.lua nine.lua
nine.lua
```

The main attitude calibration is stored separately in
`/rocket-nine-main-calibration-v2.cfg`. Old five-engine calibration is not reused.
Use the existing U/K calibration workflow after verifying the modified craft.
J records the pad while disarmed. Do not set U or J in midair to bypass guards.

## Preliminary top-engine probe

This is a supervised airborne test, not automatic RCS calibration. It requires
valid main calibration, stable V position/height hold, at least pad reference
Y+20, tilt below 2 degrees, low rates/speeds, fresh telemetry, all four top
outputs zero, and an active flight log with at least 20 KiB of session room.

1. Left/right arrows select top engine 5..8 without firing it.
2. X cycles the test power only while disarmed; leave 1 kN initially.
3. O starts a neutral baseline and requests a 0.2-second pulse on that engine.
   Main thrust is held constant, main vectors zero, and height hold is suspended.
4. C or H cancels the probe and returns to attitude HOLD. E disarms.
   Space stops all engines; Q stops and exits. These are not landing commands.
5. After a completed/cancelled probe, height hold is NOT automatically restored.
   Check motion, then V manually when stable. Do not leave the craft unattended.

The log records all nine output acknowledgements, pose/velocity, RCS_ON/OFF,
and acknowledged pulse duration. Async game calls mean the physical pulse is
not guaranteed to last exactly 0.2 seconds. Missing or late acknowledged pulses
are rejected. A captured sample is explicitly **not** a validated RCS model.
Motion limits cancel the probe; I/O/telemetry faults trigger the shared shutdown.

## Remaining work before automatic descent

`recovery_plan.lua` remains a pure shadow planner and never writes engines.
It supports simultaneous predicted-landing-point correction during coast and
early powered braking, but real measured vertical authority and coupled
top-thruster translation/rotation authority are still required. This version
does not enable freefall, boostback, automatic terrain sensing, or tower catch.
Final recovery testing will first end at pad reference Y+20, not at the ground.

`test_nine.py` uses asynchronous mock peripherals. It validates output routing,
guards, failures and shutdown, not actual Minecraft flight stability or RCS
physical authority. Never interpret these tests as a successful in-game flight.
