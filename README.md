# Red Raider Kicker Rover - AI Handoff README

## Project Snapshot

This project is a MicroPython control stack for the ECE 3331 Red Raider Soccer rover (kicker mode focused) on Raspberry Pi Pico 2.

Current state:
- Core kicker flow is implemented and running.
- Line sensing now works with fixed per-sensor black thresholds.
- Diagnostic script exists to validate each subsystem quickly.
- No git repository is initialized in this folder yet.

---

## Hardware + Pin Map (Current)

All values are centralized in `config.py`.

- **Motor driver / overcurrent**
  - IN1/IN2/IN3/IN4: `4, 5, 6, 7`
  - ENA/ENB PWM: `3, 2`
  - OC pins: `15, 14`
- **Front ultrasonic + sweep servo**
  - Trig: `12`
  - Echo: `8`
  - Servo: `10`
- **Rear ultrasonic**
  - Trig/Echo: `17, 18`
- **ToF VL53L1X**
  - I2C0 SDA/SCL: `0, 1`
  - INT: `19`
- **Color sensor**
  - SDA/SCL: `20, 21`
  - Uses `SoftI2C` (`COLOR_USE_SOFT_I2C=True`)
- **Claw servos**
  - `9, 22`
- **Kicker MOSFET gate**
  - `11`
- **Encoders**
  - Left/Right: `28, 16`
- **Line sensors (left/right ADC)**
  - Left pin: `27`
  - Right pin: `26`

---

## Main Runtime Behavior

Entrypoint: `main.py`

High-level finite-state machine includes:
- `START_ADVANCE`
- `SCAN`
- `SEARCH`
- `APPROACH`
- `CAPTURE_VERIFY`
- `CLASSIFY`
- `ROUTE_NOT_RED`
- `ROUTE_RED`
- `RECOVER`

Important behavior currently implemented:
- Startup advance + scan routine.
- ToF-based approach and capture trigger.
- Claw close + secure verification.
- Color classification and route branch.
- Kicker pulse logic with cooldown.
- Obstacle handling and boundary safety integration.
- Debug telemetry stream.

---

## Current Tuning Values (Most Important)

From `config.py`:

- `TOF_CAPTURE_MM = 60`
- `TOF_SECURED_MM = 5`
- `TOF_OFFSET_MM = 26`
- `TOF_APPROACH_MID_MM = 150`
- `TOF_APPROACH_SLOW_MM = 95`

Line detection (fixed mode):
- `LINE_USE_ADAPTIVE = False`
- `LINE_LEFT_BLACK_THRESHOLD = 5600`
- `LINE_RIGHT_BLACK_THRESHOLD = 9000`
- `LINE_STOP_DEBOUNCE_COUNT = 1`
- `LINE_SAMPLE_COUNT = 24`
- `LINE_SAMPLE_DELAY_US = 0`

Safety toggles:
- `ENABLE_LINE_SAFETY = True`
- `ENABLE_OBSTACLE_AVOID = True`

Debug:
- `DEBUG_ENABLE = True`
- `DEBUG_PERIOD_MS = 250`

---

## What Is Working

- Front/rear ultrasonics and sweep servo read and print valid values.
- ToF sensor returns consistent range stream.
- Kicker trigger path and claw actuation both execute.
- Color sensor initializes and returns classification.
- Overcurrent inputs read correctly in diagnostics.
- Line detection now catches black boundaries with current fixed thresholds.

---

## Known Issues / Active Risks

1. **Color classification can bias RED under dim/noisy lighting**
   - Consider adding minimum clear-channel gate in `colorsensor.py`.

2. **Approach/obstacle interactions can still be noisy**
   - `APPROACH` and `OBSTACLE_AVOID` may fight in dense rover traffic.
   - Needs on-field tuning of obstacle thresholds + approach speed profile.

3. **ToF close-range consistency**
   - Capture/secure thresholds are hardware-position sensitive.
   - Re-validate if sensor mount shifts.

4. **No version control baseline**
   - This folder is not a git repo; rollback is manual right now.

---

## Files and Roles

- `main.py`: top-level state machine and runtime behavior.
- `config.py`: all tuning constants and pin assignments.
- `linesensor.py`: left/right black boundary logic.
- `tof_sensor.py`: VL53L1X wrapper and capture/secure helpers.
- `colorsensor.py`: color classification (SoftI2C on pins 20/21).
- `motors.py`: drivetrain control + fault checking.
- `avoidance.py`: obstacle and turn logic.
- `claw.py`: dual servo claw control.
- `kick.py`: kicker pulse API with cooldown.
- `diagnostic.py`: subsystem test suite (recommended before field runs).

---

## Diagnostic + Run Procedure

1. **Bench validation**
   - Run `diagnostic.py` with wheels suspended.
   - Confirm Test 2 line sensor outputs:
     - white floor => `boundary_detected=False`
     - black line => `boundary_detected=True`

2. **Field run**
   - Set `ENABLE_LINE_SAFETY=True`, `ENABLE_OBSTACLE_AVOID=True`.
   - Run `main.py`.
   - Watch debug logs for:
     - `TOF_APPROACH`
     - `CAPTURE_TRIGGER`
     - `BALL_SECURED`
     - `KICK_*`
     - `SAFETY_TRIGGER` frequency

3. **Competition mode**
   - Set `DEBUG_ENABLE=False` after tuning stabilizes.

---

## Suggested Next Actions for Next Agent

1. Add color confidence gating:
   - If clear channel is below threshold, return `NOT RED`.

2. Refine approach vs obstacle arbitration:
   - Prioritize capture when ToF is strong and ultrasonic is noisy.

3. Improve scan targeting:
   - Keep heading memory after successful detections.

4. Add quick calibration helper for line thresholds:
   - Prompt operator for white/black samples and print recommended L/R thresholds.

5. Initialize git and create baseline commit for rollback safety.

---

## Notes for Handoff

- User reported that line sensing is now good after fixed per-sensor threshold tuning.
- Most recent work aimed to keep this exact state stable ("revert all code to here" intent).
- Prefer incremental tuning in `config.py` first before structural logic changes.
