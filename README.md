# Red Raider Soccer Rover Codebase

Autonomous rover software for the ECE 3331 Robotics Project Lab (Spring 2026), built for the **Red Raider Soccer** challenge.

This repository contains both competition roles:
- **Kicker mode** (primary focus in this repo, implemented in `main.py`)
- **Goalie mode** (available in `goalie.py` and `goalie_final.py`)

The project objective is autonomous ball collection, color-based routing, and safe navigation under course constraints.

## Project Context

From the course project brief:
- Match format is 2v2 with one **goalie** and one **kicker** per team.
- Balls are scored by color:
  - **Red**: -2 points
  - **Green**: +2 points
  - **Blue**: +1 point
- Rovers must operate autonomously (no external comms), avoid collisions, and stay within hardware constraints.

This codebase implements those requirements through layered sensor fusion, encoder-based motion control, and explicit autonomous state logic.

## Repository Structure

### Core runtime (kicker)
- `main.py`: **Primary competition entry point** for kicker behavior (4-phase strategy).
- `navigation.py`: Movement primitives and helper logic used by `main.py`.
- `config.py`: Central pin mapping, tuning constants, thresholds, and timing values.

### Hardware abstraction modules
- `motors.py`: L298N direction and PWM control with overcurrent handling.
- `encoders.py`: Interrupt-based wheel pulse tracking.
- `tof_sensor.py`: VL53L1X distance sensing (ball detection/capture range logic).
- `ultrasonics.py`: Front sweeping and rear fixed HC-SR04 wrappers.
- `linesensor.py`: Boundary detection (fixed or adaptive threshold mode).
- `colorsensor.py`: TCS34725 red/non-red classification.
- `claw.py`: Dual-servo capture mechanism control.
- `kick.py`: Solenoid pulse control for payload ejection.
- `vl53l1x.py`: Sensor driver library for ToF hardware.

### Alternate strategy / utility scripts
- `goalie.py`: Advanced goalie FSM with patrol and return-home logic.
- `goalie_final.py`: Simplified goalie shuttle strategy.
- `kicker_mode.py`: Earlier consolidated kicker implementation (legacy reference).
- `kick_avoidance.py`: Kicker variant with explicit obstacle-avoidance integration.
- `avoidance.py`: Obstacle avoidance helper class.
- `strategy_test.py`: Boundary/turn sequence testing script.
- `diagnostic.py`: Full subsystem diagnostics routine.

## Hardware Stack

- **Controller**: Raspberry Pi Pico 2 (MicroPython runtime)
- **Drive**: Dual TT/Dagu motors + L298N driver + overcurrent board
- **Distance sensing**:
  - VL53L1X ToF sensor (front ball range)
  - Front sweeping ultrasonic (HC-SR04 + servo)
  - Rear fixed ultrasonic (HC-SR04)
- **Color sensing**: TCS34725 (I2C)
- **Boundary sensing**: Dual downward line sensors (ADC)
- **Manipulators**:
  - Dual-servo claw
  - Solenoid kicker

Pin assignments and key thresholds are defined in `config.py`.

## Main Kicker Strategy (`main.py`)

`main.py` is the production kicker loop. It repeatedly executes a **4-phase acquisition pipeline**:

1. Detect a target ball
2. Refine alignment
3. Verify and capture
4. Classify and route payload

This cycle continues indefinitely until manually stopped or a runtime fault occurs.

### Pre-loop initialization

Before entering the phase cycle:
- Initializes all actuator/sensor subsystems.
- Builds `Navigator` with motor, encoder, and line-sensor dependencies.
- Applies VL53L1X ROI optimization (`0x0080 <- 0x77`) for tighter sensing.
- Performs startup repositioning drive using `config.START_FORWARD_DIST_CM`.

### Phase 1: Scan and Detect

Goal: locate the nearest viable target in a sweep sector.

Flow:
- Check centerline first via ToF.
- If no center hit, sweep left by half arc (`SWEEP_ARC_HALF` converted to pulses).
- If still no hit, sweep right across full opposite arc.
- If nothing detected:
  - Return toward center
  - Advance by `REPOSITION_DIST_CM`
  - Restart scan

Why it works:
- Prioritizes direct-line captures.
- Limits unnecessary sweeping.
- Maintains area coverage with progressive repositioning.

### Phase 2: Micro-Alignment

Goal: reduce heading error before approach.

Method:
- Capture ToF at center.
- Nudge left (`NUDGE_PULSES`) and sample.
- Nudge right (double nudge) and sample.
- Choose best direction by minimum measured distance.
- Apply corrective pulse turn back to best heading.

This phase gives a low-cost alignment correction without heavy computation.

### Phase 3: Verified Approach and Capture

Goal: approach ball reliably, reject false targets, and secure payload.

Safeguards included:
- **Pre-approach revalidation**: confirms target presence across multiple short reads.
- **False trigger rejection**:
  - Abort if range suddenly increases by `FALSE_TRIGGER_MARGIN_MM`.
  - Abort if too many invalid frames (`MAX_LOST_FRAMES`).
- **Speed ladder**:
  - Cruise at nominal forward speed.
  - Slow down at `SLOWDOWN_THRESHOLD_MM`.
  - Stop and capture at `STOP_THRESHOLD_MM`.
- **Capture verification**:
  - After claw closure, confirm near-zero range (`CAPTURE_SUCCESS_MM` threshold).

If capture fails, the rover opens claw, backs off slightly, and resumes search.

### Phase 4: Color Identification and Payload Routing

Goal: classify captured ball and route based on scoring strategy.

Decision logic:
- Use `RedBallSensor.check_ball()`.
- If **RED**:
  - Open claw
  - Fire kicker (`kick()`) to reject hazardous payload
- If **NOT RED**:
  - Execute boundary navigation sequence:
    - Directional turn based on original detection side (`left`, `right`, `center`)
    - Drive to boundary and align
    - Reverse, rotate, and repeat alignment sequence
  - Release payload at final boundary position

After either path:
- Back away from drop zone
- Apply reset turn
- Return to Phase 1

## How `main.py` Uses Other Modules

- `MotorController` (`motors.py`): smooth start/stop, turning, fault recovery.
- `WheelEncoders` (`encoders.py`): pulse-based movement fences.
- `Navigator` (`navigation.py`): high-level motion primitives and scan helpers.
- `BallDetector` (`tof_sensor.py`): IRQ-backed ToF distance reads.
- `LineSensor` (`linesensor.py`): boundary contact/alignment.
- `RedBallSensor` (`colorsensor.py`): binary color routing decision.
- `Claw` (`claw.py`) + `kick()` (`kick.py`): ball manipulation and ejection.
- `config.py`: all tunable constants for field calibration.

## Running the Code

## 1) Flash environment
- Flash MicroPython to the Raspberry Pi Pico 2.

## 2) Copy project files
- Upload repository files to the Pico filesystem.
- Ensure `main.py` is present at the root as the execution entrypoint.

## 3) Hardware check
- Verify wiring matches `config.py`.
- Confirm all sensors respond with `diagnostic.py` before full runs.

## 4) Start runtime
- On reset/boot, `main.py` executes automatically (or run manually from REPL).

## 5) Stop runtime
- Interrupt from REPL (`KeyboardInterrupt`) or power cycle.

## Tuning and Calibration

All critical tuning is centralized in `config.py`. Most important for kicker:
- Detection and capture thresholds:
  - `DETECT_THRESHOLD_MM`
  - `SLOWDOWN_THRESHOLD_MM`
  - `STOP_THRESHOLD_MM`
  - `CAPTURE_SUCCESS_MM`
- Search and movement behavior:
  - `SWEEP_ARC_HALF`
  - `START_FORWARD_DIST_CM`
  - `REPOSITION_DIST_CM`
  - `BOUNDARY_TURN_PULSES`
  - `BOUNDARY_REVERSE_CM`
- Robustness:
  - `FALSE_TRIGGER_MARGIN_MM`
  - `MAX_LOST_FRAMES`
  - `SETTLE_TIME_S`
  - `NUDGE_PULSES`

Recommended workflow:
1. Run `diagnostic.py` to validate individual subsystems.
2. Tune drivetrain (encoder distances and turn pulse equivalence).
3. Tune ToF thresholds on-field.
4. Tune line sensor thresholds for local lighting/floor reflectivity.
5. Validate full 4-phase cycle under match conditions.

## Safety and Reliability Features

- Motor overcurrent debounce and automatic pause/retry logic.
- Sensor validation and multi-frame confirmation before commits.
- False-positive rejection during final approach.
- Boundary alignment behavior to prevent uncontrolled exits.
- Defensive exception handling in main loops with emergency motor stop.

## Notes on Multiple Entry Points

This repository includes historical and role-specific scripts. For kicker competition runtime, use:
- `main.py` (current primary implementation)

Use these for testing or alternative behaviors:
- `kicker_mode.py`, `kick_avoidance.py`, `strategy_test.py`, `diagnostic.py`, `goalie.py`, `goalie_final.py`

## Troubleshooting Quick Reference

- Rover misses nearby balls:
  - Reduce `TOF_OFFSET_MM`, increase `DETECT_THRESHOLD_MM`, verify ToF alignment.
- Rover overshoots capture:
  - Increase slowdown distance (`SLOWDOWN_THRESHOLD_MM`), reduce forward speeds.
- Frequent false captures:
  - Tighten `FALSE_TRIGGER_MARGIN_MM`, increase validation strictness.
- Boundary alignment unstable:
  - Re-tune `LINE_LEFT_BLACK_THRESHOLD` / `LINE_RIGHT_BLACK_THRESHOLD`.
- Motors cut out:
  - Check overcurrent wiring and load; verify smooth-start speeds in `config.py`.

## Summary

This codebase implements a practical autonomous soccer rover stack for ECE 3331 with a robust kicker pipeline centered on `main.py`. The 4-phase strategy (Scan -> Align -> Verify/Capture -> Color Route) combines sensor fusion, motion control, and field-aware routing to satisfy project competition goals.
