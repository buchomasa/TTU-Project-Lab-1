"""
Red Raider Soccer - Goalie Mode
================================
Self-contained: all goalie-specific tuning, helpers, the FSM, and the bench
odometry test live in THIS file. config.py / colorsensor.py / diagnostic.py
are imported but NOT modified.

Strategy in plain words (see GOALIE_STRATEGY_PLAN.md for the full doc):
  1. Start on home goal pad.
  2. Drive forward 3 ft to the patrol anchor.
  3. Repeat (1–2) at up to four depths: 3 ft, 6 ft, 9 ft, 12 ft into the
     field (12 ft total ≈ field half). Then reverse to the goal line and
     restart from step 2.
  4. If a ball is detected, drive to it (no forward distance cap), close the
     claw.
  5. With the ball clamped, the color sensor in the catch box reads RED
     vs NOT RED immediately.
       RED     -> open claw + kick forward (we are facing the field).
       NOT RED -> turn 180, drive home, drop in goal (+1 or +2).
  6. Back to step 2.

The goalie may cross the blue centerline; odometry is only used for patrol
distance and return-to-goal navigation, not as a centerline stop.

Two ways to use this file:
  A) Game runtime:  rename to main.py on the goalie's Pico filesystem, OR
     import + call goalie_main() from a one-line main.py stub.
  B) Bench test:    from a fresh REPL ->
                        from goalie import run_odometry_test
                        run_odometry_test()

Importing this file does NOT start the FSM or touch hardware until you
explicitly call goalie_main() or run_odometry_test().
"""

from machine import Pin, Timer
import time
import math

import config
from avoidance import ObstacleAvoidance
from claw import Claw
from colorsensor import RedBallSensor
from encoders import WheelEncoders
from kick import kick
from linesensor import LineSensor
from motors import MotorController
from tof_sensor import BallDetector
from ultrasonics import FixedUltrasonicScanner, UltrasonicScanner


# ---------------------------------------------------------------------------
# Goalie-local tuning (lives here; not in config.py)
# ---------------------------------------------------------------------------
# Geometry (encoder-based; odometry for return-to-goal and patrol distance).
GOALIE_PATROL_FORWARD_CM = 91.4    # one leg = 3 ft along field forward
# Field half-depth from the goal line (~12 ft): four legs of 3 ft each.
GOALIE_PATROL_FIELD_DEPTH_FT = 12
GOALIE_PATROL_NUM_LEGS = 4
GOALIE_PATROL_MAX_FORWARD_CM = (
    GOALIE_PATROL_NUM_LEGS * GOALIE_PATROL_FORWARD_CM
)
# Legacy lateral-step constants (kept so old debug references keep compiling
# but no longer used; the new patrol cycle is rotation-only at the anchor).
GOALIE_LATERAL_STEP_CM = 30.5
GOALIE_MAX_LATERAL_CM = 60.0

# === CALIBRATION KNOBS (READ THIS) ===
# DEFAULT IS 1.00 = trust config.py geometry as-is.
# Only lower these AFTER running run_odometry_test() / run_turn_test()
# and physically measuring with a tape / protractor.
#
# DRIVE_CALIBRATION:
#   Rover went 110 cm when commanded 91.4 cm (3 ft) -> 91.4 / 110 = 0.83
#   Rover went 75  cm when commanded 91.4 cm        -> 91.4 / 75  = 1.22
#
# TURN_CALIBRATION:
#   Rover spun 70 deg when commanded 90 deg -> 90 / 70 = 1.29
#   Rover spun 110 deg when commanded 90 deg -> 90 / 110 = 0.82
GOALIE_DRIVE_CALIBRATION = 1.00
GOALIE_TURN_CALIBRATION = 1.00

# Goal mouth / drop
GOALIE_GOAL_REACH_CM = 8.0         # front ultra reading at the goal pad backstop
GOALIE_GOAL_BACKOUT_CM = 25.0      # how far to back out of the goal mouth
GOALIE_KICK_CLEAR_CM = 30.0        # don't punt a red ball into a teammate

# === FRONT ULTRASONIC RANGE FILTER ===
# Anything farther than this is treated as "no obstacle" / out of range.
# HC-SR04 readings are noisy at long range, and the goalie only cares about
# tall objects that are about to collide -- not walls or rovers across the
# field. Filtered reads are returned as GOALIE_FRONT_ULTRA_CLEAR_CM (a large
# sentinel) so threshold checks of the form `d < THRESH` simply fail.
GOALIE_FRONT_ULTRA_MAX_CM = 20.0
GOALIE_FRONT_ULTRA_CLEAR_CM = 999.0

# Patrol sweep cadence (slower than kicker's 250 ms; gives HC-SR04 time to settle).
GOALIE_SWEEP_DURATION_MS = 1500
GOALIE_SWEEP_ANGLES = (110, 128, 146)
GOALIE_SWEEP_PERIOD_MS = 400

# Patrol: from the goal line, advance in GOALIE_PATROL_NUM_LEGS steps of
# GOALIE_PATROL_FORWARD_CM (3 ft) to GOALIE_PATROL_MAX_FORWARD_CM (12 ft).
# At EACH stop (after every forward segment) the rover runs the same look
# pattern: heading 0 -> +30 -> -15 -> back to 0, then either another 3 ft
# forward (if not yet at max depth) or reverse to the goal and repeat.
GOALIE_PATROL_LOOK_RIGHT_DEG = 180
# "15 deg to the left from the first original position" => heading -15 deg.
GOALIE_PATROL_LOOK_LEFT_REL_DEG = 120
# From heading +30 to -30+15=-15: one LEFT turn of 30+15 = 45 deg.
GOALIE_PATROL_RIGHT_TO_LEFT_TURN_DEG = (
    GOALIE_PATROL_LOOK_RIGHT_DEG + GOALIE_PATROL_LOOK_LEFT_REL_DEG
)
# From heading -15 back to 0: RIGHT 15 deg.
GOALIE_PATROL_LEFT_TO_FORWARD_TURN_DEG = GOALIE_PATROL_LOOK_LEFT_REL_DEG

GOALIE_PATROL_LOOK_MS = GOALIE_SWEEP_DURATION_MS

# Drive speeds (mirror kicker forward speeds; tune later if needed).
GOALIE_DRIVE_LEFT_SPEED = config.FWD_LEFT_SPEED
GOALIE_DRIVE_RIGHT_SPEED = config.FWD_RIGHT_SPEED

# Goalie-only ToF detection horizon. The kicker caps at TOF_APPROACH_MM=400
# because it always closes within 60 cm before scanning; the goalie sits at
# the patrol anchor (91 cm from goal) and needs to spot balls anywhere on
# its half. 1200 mm = ~4 ft, which covers the patrol anchor out to the
# centerline. The VL53L1X can see white/orange balls reliably at this range.
GOALIE_TOF_DETECT_MM = 1200

# VL53L1X is normally IRQ-driven (BallDetector.data_ready). Missed edges or a
# slow sample rate can leave latest_tof_mm stale during patrol dwells — the
# ball sits in front but the FSM still "sees" an old far reading until the next
# interrupt. Poll I2C on this cadence during look + approach so distance stays
# current.
GOALIE_TOF_POLL_MS = 25

# Speed used when a ball is first detected and still far away (>= TOF_APPROACH_MID_MM).
# A noticeable slow-down from full forward so the rover lines up smoothly.
GOALIE_APPROACH_FAR_LEFT_SPEED = config.APPROACH_MID_LEFT_SPEED
GOALIE_APPROACH_FAR_RIGHT_SPEED = config.APPROACH_MID_RIGHT_SPEED

# Color sensor robustness (binary RED / NOT RED is enough for goalie).`

# If the TCS34725 clear channel is dimmer than this, we treat the read as
# "NOT RED" so a misread doesn't accidentally throw a green/blue ball away.
COLOR_CLEAR_MIN = 60

# === LINE SAFETY OVERRIDE ===
# The kicker/goalie share config.ENABLE_LINE_SAFETY. If the goalie is being
# bench-tested on a surface that confuses the line sensors (typical: dark
# wood, dark carpet, or anything where left ADC > LINE_LEFT_BLACK_THRESHOLD
# = 5600 by default), every drive aborts on the first read and the FSM gets
# stuck in SAFETY_TRIGGER -> RECOVER -> SAFETY_TRIGGER ...
#
# Set False to bypass line safety from goalie.py only (config.py untouched).
# *** SET TRUE ON THE ACTUAL FIELD *** (currently False so you can bench-test)
GOALIE_LINE_SAFETY = False

# When line safety trips, also print the raw ADC values so you can see WHY.
GOALIE_LOG_LINE_RAW = True

# Cm / pulse for the encoder fence math.
WHEEL_CIRC_CM = math.pi * config.WHEEL_DIAMETER_CM
CM_PER_PULSE = WHEEL_CIRC_CM / config.DISK_SLOTS
# Calibrated cm-per-pulse: if the rover overshoots by 25%, the EFFECTIVE
# cm/pulse is higher than the geometric one. EFFECTIVE_CM_PER_PULSE is what
# the goalie actually trusts for distance / fence math.
EFFECTIVE_CM_PER_PULSE = CM_PER_PULSE / GOALIE_DRIVE_CALIBRATION

# Reused for the avoider helper.
ROVER_CONFIG = {
    "obs_thresh": config.OBSTACLE_THRESHOLD_CM,
    "rear_thresh": config.REAR_THRESHOLD_CM,
    "fwd_speeds": (config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED),
    "trn_speeds": (config.TURN_LEFT_SPEED, config.TURN_RIGHT_SPEED),
    "track_width": config.TRACK_WIDTH_CM,
    "wheel_dia": config.WHEEL_DIAMETER_CM,
    "disk_slots": config.DISK_SLOTS,
}


# ---------------------------------------------------------------------------
# State constants
# ---------------------------------------------------------------------------
G_INIT = "G_INIT"
G_GO_TO_PATROL = "G_GO_TO_PATROL"
G_PATROL_SWEEP = "G_PATROL_SWEEP"          # look forward (heading 0)
G_PATROL_LOOK_RIGHT = "G_PATROL_LOOK_RIGHT"  # look right (heading +30)
G_PATROL_LOOK_LEFT = "G_PATROL_LOOK_LEFT"    # look 15 deg left of fwd (heading -15)
G_PATROL_ADVANCE = "G_PATROL_ADVANCE"      # drive exactly one more 3 ft leg (heading 0)
G_PATROL_RESET_TO_GOAL = "G_PATROL_RESET_TO_GOAL"  # reverse ~12 ft to goal line; restart patrol
G_LATERAL_STEP = "G_LATERAL_STEP"          # legacy (no longer reached, kept for clarity)
G_APPROACH = "G_APPROACH"
G_CAPTURE_VERIFY = "G_CAPTURE_VERIFY"
G_CLASSIFY_IN_BOX = "G_CLASSIFY_IN_BOX"
G_REJECT_HERE = "G_REJECT_HERE"
G_RETURN_HOME = "G_RETURN_HOME"
G_DROP_IN_GOAL = "G_DROP_IN_GOAL"
G_RECOVER = "G_RECOVER"

# ToF must stay fresh: IRQ-only updates lag during patrol stops and approach.
GOALIE_TOF_FAST_POLL_STATES = (
    G_PATROL_SWEEP,
    G_PATROL_LOOK_LEFT,
    G_PATROL_LOOK_RIGHT,
    G_APPROACH,
)


# ---------------------------------------------------------------------------
# Pure helpers (no hardware side effects)
# ---------------------------------------------------------------------------
def _gated_check_ball(sensor):
    """Single color read with a clear-channel guard.

    Reads register 0x14 (clear channel) directly; if it's below
    COLOR_CLEAR_MIN we shortcut to "NOT RED" without dividing on noisy R/G/B.
    Falls back to the standard sensor.check_ball() otherwise. Does NOT
    require any change to colorsensor.py.
    """
    try:
        c = sensor._read16(0x14)
    except Exception:
        # If the I2C register read glitches, fall back to the public method.
        return sensor.check_ball()
    if c < COLOR_CLEAR_MIN:
        return "NOT RED"
    return sensor.check_ball()


def classify_ball_voted(sensor):
    """Majority-vote classifier across config.COLOR_SAMPLES reads."""
    red_votes = 0
    samples = max(1, config.COLOR_SAMPLES)
    for _ in range(samples):
        if _gated_check_ball(sensor) == "RED":
            red_votes += 1
        time.sleep_ms(30)
    return "RED" if red_votes > (samples // 2) else "NOT RED"


def is_tof_ball_candidate(tof_mm):
    """True iff the ToF reading looks like a ball within goalie detect range.

    NOTE: the goalie uses GOALIE_TOF_DETECT_MM (~1.2 m) instead of the kicker's
    config.TOF_APPROACH_MM (~0.4 m). The kicker only ever scans from close
    range; the goalie has to spot balls anywhere on its half of the field.
    """
    return (
        tof_mm is not None
        and tof_mm >= config.TOF_MIN_VALID_MM
        and tof_mm <= GOALIE_TOF_DETECT_MM
    )


def is_rover_close(latest_tof_mm, front_distance_cm, ball_secured_now):
    """Height discrimination: ToF + front ultra both close => rover, not ball.

    Lifted from main.py (kicker).
    """
    return (
        latest_tof_mm is not None
        and latest_tof_mm <= config.ROVER_PROX_TOF_MM
        and front_distance_cm <= config.ROVER_PROX_ULTRA_CM
        and not ball_secured_now
    )


def _enter_state(new_state):
    return new_state, time.ticks_ms()


def _debug_log(msg):
    if config.DEBUG_ENABLE:
        print(msg)


def _front_distance(scanner):
    """Front-ultra read with a hard 20 cm range cutoff.

    Anything farther than GOALIE_FRONT_ULTRA_MAX_CM is treated as "clear"
    and reported as GOALIE_FRONT_ULTRA_CLEAR_CM. This filters out noisy
    long-range echoes and field-wall reflections so the goalie only reacts
    to objects that are genuinely close to the front bumper.
    """
    try:
        d = scanner.get_distance()
    except Exception:
        return GOALIE_FRONT_ULTRA_CLEAR_CM
    if d is None or d <= 0 or d > GOALIE_FRONT_ULTRA_MAX_CM:
        return GOALIE_FRONT_ULTRA_CLEAR_CM
    return d


# ---------------------------------------------------------------------------
# Goalie-local turn helper (avoider.execute_exact_turn coast-stops too)
# ---------------------------------------------------------------------------
def _compute_return_path(forward_cm, lateral_cm):
    """Given the rover's pose relative to the goal pad in field coordinates --
    forward_cm (+ = into the field, - = behind goal) and lateral_cm
    (+ = right of goal-to-goal axis, - = left) -- with the rover currently
    facing +forward, return (turn_deg, direction, drive_cm) that points the
    rover at the goal pad and tells it how far to drive.

    Math:
        Vector from rover to goal in field coords = (-forward_cm, -lateral_cm).
        Rover currently faces +forward. The angle from current heading to the
        goal vector is 180 - atan2(|L|, F), turning toward the side L is on.
        Distance to drive is sqrt(F^2 + L^2).
    """
    if forward_cm <= 0 and abs(lateral_cm) < 1.0:
        # Already at (or behind) goal pad; nothing to do.
        return (0.0, "RIGHT", 0.0)

    # Use absolute values for the math; direction sign comes from lateral_cm.
    f = max(0.0, forward_cm)
    l_abs = abs(lateral_cm)
    distance_cm = math.sqrt(f * f + l_abs * l_abs)
    if f < 0.5:  # essentially at the goal-pad longitudinal axis already
        # Rover is alongside the goal; drop logic should handle this case.
        turn_deg = 90.0
    else:
        turn_deg = 180.0 - math.degrees(math.atan2(l_abs, f))

    # Direction: rover is RIGHT of center (lateral > 0)? Turn RIGHT
    # to swing through 180-ish onto the goal vector. Mirror for LEFT.
    direction = "RIGHT" if lateral_cm >= 0 else "LEFT"
    return (turn_deg, direction, distance_cm)


def goalie_turn(motors, encoders, angle_degrees, direction="RIGHT", calibration=None):
    """Encoder-counted in-place turn that respects GOALIE_TURN_CALIBRATION
    and hard-brakes at the end so the rover lands on its target heading.

    Same kinematic math as avoidance.execute_exact_turn() but:
      - target_pulses are scaled by calibration (so a "90 deg" command
        physically rotates 90 deg even if track / wheel measurements are off)
      - active brake at the end (no coast-past)
      - longer timeout (5 s) so a brief stall doesn't abort the turn
    """
    if calibration is None:
        calibration = GOALIE_TURN_CALIBRATION

    robot_circ = math.pi * config.TRACK_WIDTH_CM
    wheel_circ = math.pi * config.WHEEL_DIAMETER_CM
    revolutions_needed = ((angle_degrees / 360.0) * robot_circ) / wheel_circ
    raw_pulses = revolutions_needed * config.DISK_SLOTS
    # If rover under-rotates (e.g. spins 70 deg for a 90 deg command),
    # calibration > 1 makes us spin longer.
    target_pulses = max(1, int(raw_pulses * calibration))

    encoders.reset()
    if direction == "RIGHT":
        motors.turn_right()
    else:
        motors.turn_left()

    # Ramp the turn PWM the same way the avoider does.
    for step in range(1, 11):
        motors.set_speeds(
            (config.TURN_LEFT_SPEED * step) / 10,
            (config.TURN_RIGHT_SPEED * step) / 10,
        )
        if motors.check_faults():
            _hard_stop(motors)
            return False
        time.sleep_ms(10)

    start_ms = time.ticks_ms()
    while True:
        l_p, r_p = encoders.get_pulses()
        if max(l_p, r_p) >= target_pulses:
            break
        if motors.check_faults():
            break
        if time.ticks_diff(time.ticks_ms(), start_ms) > 5000:
            _debug_log(
                "DBG|event=TURN_TIMEOUT angle={} dir={} pulses_seen={} target={}".format(
                    angle_degrees, direction, max(l_p, r_p), target_pulses
                )
            )
            break
        time.sleep_ms(10)

    _hard_stop(motors)
    return True


# ---------------------------------------------------------------------------
# Active brake (motors.stop() is coast-only on the L298N)
# ---------------------------------------------------------------------------
def _hard_stop(motors_obj, brake_ms=70):
    """L298N dynamic brake: both inputs HIGH on each channel shorts the motor
    terminals through the high-side transistors, killing momentum fast.

    motors.stop() in motors.py only sets inputs LOW + PWM=0, which is COAST
    (free-wheel). At ~20 cm/s the rover coasts 8-15 cm before friction wins.
    We use this brake at every fence trigger and at the end of every
    drive_distance_cm so the rover actually stops where we said.

    NOTE: We do not edit motors.py per the user's "goalie-only" rule.
    We poke motors_obj.in1..in4 directly (they're public Pin objects).
    """
    try:
        motors_obj.in1.value(1)
        motors_obj.in2.value(1)
        motors_obj.in3.value(1)
        motors_obj.in4.value(1)
        motors_obj.set_speeds(85, 85)  # PWM high so brake transistors engage
    except Exception:
        pass
    time.sleep_ms(brake_ms)
    motors_obj.stop()


# ---------------------------------------------------------------------------
# Drive helper (with sensor gating). Used by both the FSM and the bench test.
# ---------------------------------------------------------------------------
def drive_distance_cm(
    motors,
    encoders,
    cm,
    reverse=False,
    scanner_rear=None,
    scanner_front=None,
    front_stop_cm=None,
    lines=None,
    max_ms=None,
):
    """Drive forward or reverse up to `cm`. Returns the cm actually covered.

    Watches: OC faults, rear-ultra clearance (when reversing), front-ultra
    stop threshold (when going forward), black-tape line trip, soft timeout.
    """
    if cm <= 0:
        return 0.0

    target_pulses = int(cm / EFFECTIVE_CM_PER_PULSE)
    if max_ms is None:
        max_ms = max(2000, int((cm / 10.0) * 1000))

    encoders.reset()

    if reverse:
        if scanner_rear is not None and scanner_rear.get_distance() < config.REAR_THRESHOLD_CM:
            return 0.0
        ok = motors.start_smoothly_reverse(GOALIE_DRIVE_LEFT_SPEED, GOALIE_DRIVE_RIGHT_SPEED)
    else:
        ok = motors.start_smoothly(GOALIE_DRIVE_LEFT_SPEED, GOALIE_DRIVE_RIGHT_SPEED)
    if not ok:
        motors.stop()
        return 0.0

    start_ms = time.ticks_ms()
    while True:
        l_p, r_p = encoders.get_pulses()
        cur_pulses = max(l_p, r_p)
        if cur_pulses >= target_pulses:
            break
        if motors.check_faults():
            break
        if reverse and scanner_rear is not None:
            if scanner_rear.get_distance() < config.REAR_THRESHOLD_CM:
                break
        if (not reverse) and scanner_front is not None and front_stop_cm is not None:
            if _front_distance(scanner_front) < front_stop_cm:
                break
        if lines is not None and GOALIE_LINE_SAFETY and lines.boundary_detected():
            break
        if time.ticks_diff(time.ticks_ms(), start_ms) > max_ms:
            break
        time.sleep_ms(15)

    # Active brake the moment we hit target pulses, BEFORE coast eats the budget.
    _hard_stop(motors)
    l_p, r_p = encoders.get_pulses()
    return max(l_p, r_p) * EFFECTIVE_CM_PER_PULSE


def configure_goalie_sweep(scanner):
    """Override the scanner's sweep angles + period for goalie use.

    Use this in place of scanner.start_sweep(); the built-in start_sweep()
    hardcodes period=250 ms, which would clobber our goalie cadence every
    time we resume after a pause.
    """
    try:
        scanner.timer.deinit()
    except Exception:
        pass
    scanner.sweep_angles = list(GOALIE_SWEEP_ANGLES)
    scanner.angle_idx = 0
    scanner.current_angle = scanner.sweep_angles[0]
    scanner.set_servo_angle(scanner.current_angle)
    scanner.timer.init(
        period=GOALIE_SWEEP_PERIOD_MS,
        mode=Timer.PERIODIC,
        callback=scanner._sweep_tick,
    )


# ---------------------------------------------------------------------------
# Bench TEST 9 - Encoder fence round trip (wheels ON the floor!)
# ---------------------------------------------------------------------------
def run_odometry_test():
    """
    Validates the encoder fence the goalie depends on AND helps you tune
    GOALIE_DRIVE_CALIBRATION.

    Drives forward GOALIE_PATROL_FORWARD_CM (intent: 91.4 cm = 3 ft), brakes,
    pauses, then reverses the same intended distance.

    HOW TO TUNE CALIBRATION:
      1. Mark the floor at the front bumper BEFORE running.
      2. Run this test.
      3. Measure the actual physical distance the rover travelled forward
         with a tape measure.
      4. Compute:  new_calibration = current_calibration * (commanded / actual)
         e.g. commanded 91.4 cm, you measured 110 cm physically:
              new = 0.80 * (91.4 / 110) = 0.665
      5. Edit GOALIE_DRIVE_CALIBRATION at the top of goalie.py and re-run.
      6. Repeat until "actual" matches "commanded" within ~3 cm.

    REQUIREMENT: wheels MUST be on a flat surface (NOT suspended).
    """
    print("\n" + "=" * 50)
    print("GOALIE TEST 9: ODOMETRY ROUND TRIP (wheels on ground!)")
    print("=" * 50)

    motors = MotorController()
    encoders = WheelEncoders()
    rear = FixedUltrasonicScanner(config.PIN_REAR_TRIG, config.PIN_REAR_ECHO)
    motors.stop()

    target_cm = GOALIE_PATROL_FORWARD_CM
    target_pulses = int(target_cm / EFFECTIVE_CM_PER_PULSE)
    print(
        "target={} cm  ({} pulses)  geometric={:.3f} cm/pulse  effective={:.3f} cm/pulse".format(
            target_cm, target_pulses, CM_PER_PULSE, EFFECTIVE_CM_PER_PULSE
        )
    )
    print("calibration={}  (lower this if rover overshoots)".format(GOALIE_DRIVE_CALIBRATION))
    print("Mark the floor at the front bumper, then start in 3 s...")
    time.sleep(3)

    try:
        # Forward leg
        encoders.reset()
        if not motors.start_smoothly(GOALIE_DRIVE_LEFT_SPEED, GOALIE_DRIVE_RIGHT_SPEED):
            print("[FAIL] Forward start failed (OC?)")
            return
        start_ms = time.ticks_ms()
        while True:
            l_p, r_p = encoders.get_pulses()
            if max(l_p, r_p) >= target_pulses:
                break
            if motors.check_faults():
                print("[FAIL] OC during forward leg.")
                return
            if time.ticks_diff(time.ticks_ms(), start_ms) > 8000:
                print("[FAIL] Forward leg timed out.")
                return
            time.sleep_ms(15)
        _hard_stop(motors)
        fwd_l, fwd_r = encoders.get_pulses()
        fwd_cm = max(fwd_l, fwd_r) * EFFECTIVE_CM_PER_PULSE
        print("Forward leg done: L={} R={} reported_cm={:.1f}".format(fwd_l, fwd_r, fwd_cm))
        print(">>> NOW measure physical distance with a tape. <<<")
        print(">>> If it != {:.1f} cm, adjust GOALIE_DRIVE_CALIBRATION. <<<".format(target_cm))

        time.sleep_ms(2500)  # give user time to read measurements

        # Reverse leg (rear-ultra gated)
        rear_now = rear.get_distance()
        if rear_now < config.REAR_THRESHOLD_CM:
            print(
                "[WARN] Rear ultra reads {} cm (< {} threshold). Reverse leg skipped.".format(
                    rear_now, config.REAR_THRESHOLD_CM
                )
            )
            return
        encoders.reset()
        if not motors.start_smoothly_reverse(GOALIE_DRIVE_LEFT_SPEED, GOALIE_DRIVE_RIGHT_SPEED):
            print("[FAIL] Reverse start failed.")
            return
        start_ms = time.ticks_ms()
        while True:
            l_p, r_p = encoders.get_pulses()
            if max(l_p, r_p) >= target_pulses:
                break
            if motors.check_faults():
                print("[FAIL] OC during reverse leg.")
                return
            if rear.get_distance() < config.REAR_THRESHOLD_CM:
                print("[WARN] Rear ultra trip during reverse; aborting.")
                break
            if time.ticks_diff(time.ticks_ms(), start_ms) > 8000:
                print("[FAIL] Reverse leg timed out.")
                return
            time.sleep_ms(15)
        _hard_stop(motors)
        rev_l, rev_r = encoders.get_pulses()
        rev_cm = max(rev_l, rev_r) * EFFECTIVE_CM_PER_PULSE
        print("Reverse leg done: L={} R={} cm={:.1f}".format(rev_l, rev_r, rev_cm))

        drift = abs(fwd_cm - rev_cm)
        print("Drift (|forward - reverse| in cm) = {:.1f}".format(drift))
        if drift < 8.0:
            print("[PASS] Drift < 8 cm. Encoder distances look trustworthy on this surface.")
        else:
            print(
                "[WARN] Drift >= 8 cm. Tune GOALIE_DRIVE_CALIBRATION in goalie.py."
            )
    finally:
        motors.stop()


# ---------------------------------------------------------------------------
# Hardware setup helper
# ---------------------------------------------------------------------------
def _setup_hardware():
    """Build all hardware objects in the same order as main.py."""
    pico_led = Pin("LED", Pin.OUT)
    pico_led.value(1)

    motors = MotorController()
    scanner_front = UltrasonicScanner()
    scanner_rear = FixedUltrasonicScanner(config.PIN_REAR_TRIG, config.PIN_REAR_ECHO)
    encoders = WheelEncoders()
    lines = LineSensor()
    detector = BallDetector()
    color_sensor = RedBallSensor()
    claw_obj = Claw()
    avoider = ObstacleAvoidance(motors, scanner_front, scanner_rear, encoders, ROVER_CONFIG)

    return {
        "led": pico_led,
        "motors": motors,
        "scanner_front": scanner_front,
        "scanner_rear": scanner_rear,
        "encoders": encoders,
        "lines": lines,
        "detector": detector,
        "color_sensor": color_sensor,
        "claw": claw_obj,
        "avoider": avoider,
    }


# ---------------------------------------------------------------------------
# Main FSM (call goalie_main() to run, or execute this file as __main__)
# ---------------------------------------------------------------------------
def goalie_main():
    """Run the goalie FSM forever. Returns only on KeyboardInterrupt."""
    hw = _setup_hardware()
    pico_led = hw["led"]
    motors = hw["motors"]
    scanner_front = hw["scanner_front"]
    scanner_rear = hw["scanner_rear"]
    encoders = hw["encoders"]
    lines = hw["lines"]
    detector = hw["detector"]
    color_sensor = hw["color_sensor"]
    claw_obj = hw["claw"]
    avoider = hw["avoider"]

    print("GOALIE MODE: Unified FSM starting.")
    print(
        "MODE|lineSafety={} obstacleAvoid={}".format(
            config.ENABLE_LINE_SAFETY,
            config.ENABLE_OBSTACLE_AVOID,
        )
    )
    print(
        "GEOMETRY|leg={}cm depth_legs={} max_fwd={}cm look_R={}deg look_L_rel={}deg look_ms={}".format(
            GOALIE_PATROL_FORWARD_CM,
            GOALIE_PATROL_NUM_LEGS,
            GOALIE_PATROL_MAX_FORWARD_CM,
            GOALIE_PATROL_LOOK_RIGHT_DEG,
            GOALIE_PATROL_LOOK_LEFT_REL_DEG,
            GOALIE_PATROL_LOOK_MS,
        )
    )

    state = G_INIT
    state_enter_ms = time.ticks_ms()

    # Odometry: signed positions relative to the home goal pad and field centerline.
    forward_from_goal_cm = 0.0       # +X = toward the opponent / blue line
    lateral_from_center_cm = 0.0     # +Y = right of the goal-to-goal axis
    # Rover orientation relative to "facing the field" (heading 0 = forward).
    # Updated only by goalie_turn calls inside the patrol cycle and by the
    # explicit re-align before G_RETURN_HOME. Drop / recover reset to 0.
    patrol_heading_offset_deg = 0
    next_lateral_dir = "LEFT"        # legacy; unused by the new patrol cycle

    is_moving = False
    latest_tof_mm = None
    secured_hits = 0
    last_debug_ms = time.ticks_ms()
    line_stop_hits = 0
    kick_latch_active = False
    clear_hits = 0
    tof_lost_hits = 0
    holding_ball = False             # set after color = NOT RED, cleared on drop
    last_tof_poll_ms = time.ticks_ms()

    claw_obj.open()
    configure_goalie_sweep(scanner_front)

    try:
        while True:
            loop_now = time.ticks_ms()

            if detector.data_ready:
                latest_tof_mm = detector.get_distance()
                detector.data_ready = False
                last_tof_poll_ms = loop_now
            elif (
                state in GOALIE_TOF_FAST_POLL_STATES
                and time.ticks_diff(loop_now, last_tof_poll_ms) >= GOALIE_TOF_POLL_MS
            ):
                try:
                    latest_tof_mm = detector.get_distance()
                except Exception:
                    pass
                last_tof_poll_ms = loop_now

            front_distance = _front_distance(scanner_front)
            look_angle = scanner_front.current_angle
            ball_secured_now = detector.is_secured_in_box(latest_tof_mm)
            rover_close = is_rover_close(latest_tof_mm, front_distance, ball_secured_now)

            # Kick latch clearing (prevents repeated firing on the same just-kicked ball).
            if kick_latch_active:
                if latest_tof_mm is not None and latest_tof_mm >= config.TOF_CLEAR_MM:
                    clear_hits += 1
                else:
                    clear_hits = 0
                if clear_hits >= config.TOF_CLEAR_COUNT:
                    kick_latch_active = False
                    clear_hits = 0
                    _debug_log("DBG|event=KICK_LATCH_CLEARED")

            # Black-tape (left ADC) safety; we never trust the right ADC for blue.
            line_stop_now = (
                lines.boundary_detected() if GOALIE_LINE_SAFETY else False
            )
            if line_stop_now:
                line_stop_hits += 1
            else:
                line_stop_hits = 0
            line_stop_confirmed = line_stop_hits >= config.LINE_STOP_DEBOUNCE_COUNT

            # Global safety triggers route to G_RECOVER (never blue line).
            if motors.check_faults() or line_stop_confirmed:
                motors.stop()
                is_moving = False
                state, state_enter_ms = _enter_state(G_RECOVER)
                if line_stop_confirmed and GOALIE_LOG_LINE_RAW:
                    try:
                        l_raw, r_raw = lines.get_raw_values()
                        _debug_log(
                            "DBG|event=SAFETY_TRIGGER state->RECOVER lineHits={} "
                            "lineRawL={} (thresh {}) lineRawR={} (thresh {}) fwd={:.1f}".format(
                                line_stop_hits,
                                l_raw, config.LINE_LEFT_BLACK_THRESHOLD,
                                r_raw, config.LINE_RIGHT_BLACK_THRESHOLD,
                                forward_from_goal_cm,
                            )
                        )
                    except Exception:
                        _debug_log(
                            "DBG|event=SAFETY_TRIGGER state->RECOVER lineHits={} fwd={:.1f}".format(
                                line_stop_hits, forward_from_goal_cm
                            )
                        )
                else:
                    _debug_log(
                        "DBG|event=SAFETY_TRIGGER state->RECOVER lineHits={} fwd={:.1f}".format(
                            line_stop_hits, forward_from_goal_cm
                        )
                    )

            # Height filter: if both sensors say something close and ball is not yet
            # secured, treat as a rover and recover. Active in patrol / lateral /
            # return only -- NOT in G_APPROACH, because once we've committed to
            # chasing a ball, the ToF guides us through the capture window
            # (the front ultra cone can clip the ball at close range and would
            # otherwise abort every legitimate approach).
            if rover_close and state in (
                G_PATROL_SWEEP,
                G_PATROL_LOOK_LEFT,
                G_PATROL_LOOK_RIGHT,
                G_PATROL_ADVANCE,
                G_PATROL_RESET_TO_GOAL,
                G_RETURN_HOME,
            ):
                motors.stop()
                is_moving = False
                state, state_enter_ms = _enter_state(G_RECOVER)
                _debug_log("DBG|event=ROVER_PROXIMITY state->RECOVER")
                continue

            # Front ultra wall/obstacle (tall objects), only during free motion states.
            if (
                config.ENABLE_OBSTACLE_AVOID
                and front_distance < config.OBSTACLE_THRESHOLD_CM
                and state in (
                    G_PATROL_SWEEP,
                    G_PATROL_LOOK_LEFT,
                    G_PATROL_LOOK_RIGHT,
                    G_PATROL_ADVANCE,
                    G_PATROL_RESET_TO_GOAL,
                    G_APPROACH,
                    G_RETURN_HOME,
                )
                and not ball_secured_now
            ):
                if is_moving:
                    motors.stop()
                    is_moving = False
                avoider.navigate_obstacle(front_distance, look_angle)
                # Avoider re-calls scanner.start_sweep() (period=250); restore goalie cadence.
                configure_goalie_sweep(scanner_front)
                state, state_enter_ms = _enter_state(G_RECOVER)
                _debug_log("DBG|event=OBSTACLE_AVOID state->RECOVER")
                continue

            # Anti-stuck timeout for any state that should not run forever.
            # The three look phases are dwell-only (no motion) so they're
            # excluded from the global timeout watchdog; the look-phase
            # internal timeout (GOALIE_PATROL_LOOK_MS) handles them.
            patrol_look_states = (
                G_PATROL_SWEEP,
                G_PATROL_LOOK_LEFT,
                G_PATROL_LOOK_RIGHT,
            )
            # These can run blocking drives longer than STATE_TIMEOUT_MS.
            patrol_motion_states = (
                G_GO_TO_PATROL,
                G_PATROL_ADVANCE,
                G_PATROL_RESET_TO_GOAL,
                G_APPROACH,
                G_RETURN_HOME,
                G_DROP_IN_GOAL,
            )
            if (
                state not in patrol_look_states
                and state not in patrol_motion_states
                and state != G_INIT
                and time.ticks_diff(loop_now, state_enter_ms) > config.STATE_TIMEOUT_MS
            ):
                motors.stop()
                is_moving = False
                state, state_enter_ms = _enter_state(G_RECOVER)
                _debug_log("DBG|event=STATE_TIMEOUT state->RECOVER")

            # ---------------------------
            # FSM dispatch
            # ---------------------------
            if state == G_INIT:
                forward_from_goal_cm = 0.0
                lateral_from_center_cm = 0.0
                patrol_heading_offset_deg = 0
                holding_ball = False
                state, state_enter_ms = _enter_state(G_GO_TO_PATROL)
                _debug_log("DBG|event=INIT_DONE state->G_GO_TO_PATROL")

            elif state == G_GO_TO_PATROL:
                motors.stop()
                is_moving = False
                # Only drive the REMAINING distance to the patrol anchor.
                # After a drop the rover is already GOALIE_GOAL_BACKOUT_CM into
                # the field, so a flat 91.4 cm would put us past the anchor.
                remaining_cm = max(
                    0.0, GOALIE_PATROL_FORWARD_CM - forward_from_goal_cm
                )
                if remaining_cm > 1.0:
                    covered = drive_distance_cm(
                        motors, encoders,
                        remaining_cm,
                        lines=lines,
                    )
                else:
                    covered = 0.0
                forward_from_goal_cm = forward_from_goal_cm + covered
                lateral_from_center_cm = 0.0
                # Short-fall (line tape, OC, etc.) -> recover instead of sweeping
                # from an unknown position. Threshold scales with what we
                # actually had to drive: if remaining was small we shouldn't
                # require 60% of full patrol to be covered.
                expected_cover = max(remaining_cm, 1.0)
                if covered < (expected_cover * 0.6) and remaining_cm > 5.0:
                    state, state_enter_ms = _enter_state(G_RECOVER)
                    _debug_log(
                        "DBG|event=GO_TO_PATROL_SHORT covered={:.1f}/{:.1f} state->G_RECOVER".format(
                            covered, remaining_cm
                        )
                    )
                else:
                    state, state_enter_ms = _enter_state(G_PATROL_SWEEP)
                    _debug_log(
                        "DBG|event=AT_PATROL fwd={:.1f} state->G_PATROL_SWEEP".format(
                            forward_from_goal_cm
                        )
                    )

            elif state == G_PATROL_SWEEP:
                # FORWARD look (heading 0). Front-arc sweep + ToF.
                if is_moving:
                    motors.stop()
                    is_moving = False

                if ball_secured_now and not kick_latch_active:
                    state, state_enter_ms = _enter_state(G_CLASSIFY_IN_BOX)
                    _debug_log("DBG|event=PRELOAD_SECURED state->G_CLASSIFY_IN_BOX")
                    continue

                if is_tof_ball_candidate(latest_tof_mm) and not rover_close:
                    claw_obj.open()
                    encoders.reset()
                    tof_lost_hits = 0
                    state, state_enter_ms = _enter_state(G_APPROACH)
                    _debug_log(
                        "DBG|event=BALL_DETECTED@FWD tof={}mm state->G_APPROACH".format(
                            latest_tof_mm
                        )
                    )
                    continue

                if time.ticks_diff(loop_now, state_enter_ms) > GOALIE_PATROL_LOOK_MS:
                    goalie_turn(
                        motors, encoders,
                        GOALIE_PATROL_LOOK_RIGHT_DEG, "RIGHT",
                    )
                    patrol_heading_offset_deg = GOALIE_PATROL_LOOK_RIGHT_DEG
                    state, state_enter_ms = _enter_state(G_PATROL_LOOK_RIGHT)
                    _debug_log(
                        "DBG|event=NO_BALL_FWD turn=R{} heading=+{} state->G_PATROL_LOOK_RIGHT".format(
                            GOALIE_PATROL_LOOK_RIGHT_DEG,
                            GOALIE_PATROL_LOOK_RIGHT_DEG,
                        )
                    )

            elif state == G_PATROL_LOOK_RIGHT:
                # RIGHT look (heading +30 deg). ToF still on.
                if is_moving:
                    motors.stop()
                    is_moving = False

                if ball_secured_now and not kick_latch_active:
                    state, state_enter_ms = _enter_state(G_CLASSIFY_IN_BOX)
                    _debug_log("DBG|event=PRELOAD_SECURED state->G_CLASSIFY_IN_BOX")
                    continue

                if is_tof_ball_candidate(latest_tof_mm) and not rover_close:
                    claw_obj.open()
                    encoders.reset()
                    tof_lost_hits = 0
                    state, state_enter_ms = _enter_state(G_APPROACH)
                    _debug_log(
                        "DBG|event=BALL_DETECTED@RIGHT tof={}mm heading={} state->G_APPROACH".format(
                            latest_tof_mm, patrol_heading_offset_deg
                        )
                    )
                    continue

                if time.ticks_diff(loop_now, state_enter_ms) > GOALIE_PATROL_LOOK_MS:
                    # To 15 deg LEFT of original forward -> heading -15.
                    goalie_turn(
                        motors, encoders,
                        GOALIE_PATROL_RIGHT_TO_LEFT_TURN_DEG, "LEFT",
                    )
                    patrol_heading_offset_deg = -GOALIE_PATROL_LOOK_LEFT_REL_DEG
                    state, state_enter_ms = _enter_state(G_PATROL_LOOK_LEFT)
                    _debug_log(
                        "DBG|event=NO_BALL_RIGHT turn=L{} heading={} state->G_PATROL_LOOK_LEFT".format(
                            GOALIE_PATROL_RIGHT_TO_LEFT_TURN_DEG,
                            patrol_heading_offset_deg,
                        )
                    )

            elif state == G_PATROL_LOOK_LEFT:
                # LEFT look (heading -15 deg relative to field forward). ToF on.
                if is_moving:
                    motors.stop()
                    is_moving = False

                if ball_secured_now and not kick_latch_active:
                    state, state_enter_ms = _enter_state(G_CLASSIFY_IN_BOX)
                    _debug_log("DBG|event=PRELOAD_SECURED state->G_CLASSIFY_IN_BOX")
                    continue

                if is_tof_ball_candidate(latest_tof_mm) and not rover_close:
                    claw_obj.open()
                    encoders.reset()
                    tof_lost_hits = 0
                    state, state_enter_ms = _enter_state(G_APPROACH)
                    _debug_log(
                        "DBG|event=BALL_DETECTED@LEFT tof={}mm heading={} state->G_APPROACH".format(
                            latest_tof_mm, patrol_heading_offset_deg
                        )
                    )
                    continue

                if time.ticks_diff(loop_now, state_enter_ms) > GOALIE_PATROL_LOOK_MS:
                    goalie_turn(
                        motors, encoders,
                        GOALIE_PATROL_LEFT_TO_FORWARD_TURN_DEG, "RIGHT",
                    )
                    patrol_heading_offset_deg = 0
                    # At max depth (fourth 3 ft segment, ~12 ft): reverse to
                    # goal; else drive exactly one more 3 ft leg.
                    if forward_from_goal_cm >= (
                        GOALIE_PATROL_MAX_FORWARD_CM - 2.0
                    ):
                        state, state_enter_ms = _enter_state(G_PATROL_RESET_TO_GOAL)
                        _debug_log(
                            "DBG|event=NO_BALL_AT_MAX fwd={:.1f} state->G_PATROL_RESET_TO_GOAL".format(
                                forward_from_goal_cm
                            )
                        )
                    else:
                        state, state_enter_ms = _enter_state(G_PATROL_ADVANCE)
                        _debug_log(
                            "DBG|event=NO_BALL_LEFT turn=R{} heading=0 state->G_PATROL_ADVANCE".format(
                                GOALIE_PATROL_LEFT_TO_FORWARD_TURN_DEG
                            )
                        )

            elif state == G_PATROL_ADVANCE:
                # Exactly one more 3 ft along field forward (never past 12 ft).
                motors.stop()
                is_moving = False
                room_cm = GOALIE_PATROL_MAX_FORWARD_CM - forward_from_goal_cm
                step_cm = min(GOALIE_PATROL_FORWARD_CM, max(0.0, room_cm))
                if step_cm < 1.0:
                    state, state_enter_ms = _enter_state(G_PATROL_RESET_TO_GOAL)
                    _debug_log(
                        "DBG|event=PATROL_ADVANCE_SKIP room={:.1f} state->G_PATROL_RESET_TO_GOAL".format(
                            room_cm
                        )
                    )
                else:
                    covered = drive_distance_cm(
                        motors, encoders,
                        step_cm,
                        lines=lines,
                    )
                    forward_from_goal_cm = forward_from_goal_cm + covered
                    expected = max(step_cm, 1.0)
                    if covered < (expected * 0.6) and step_cm > 5.0:
                        state, state_enter_ms = _enter_state(G_RECOVER)
                        _debug_log(
                            "DBG|event=PATROL_ADVANCE_SHORT covered={:.1f} state->G_RECOVER".format(
                                covered
                            )
                        )
                    else:
                        state, state_enter_ms = _enter_state(G_PATROL_SWEEP)
                        _debug_log(
                            "DBG|event=PATROL_ADVANCE covered={:.1f} fwd={:.1f}/{:.1f} state->G_PATROL_SWEEP".format(
                                covered,
                                forward_from_goal_cm,
                                GOALIE_PATROL_MAX_FORWARD_CM,
                            )
                        )

            elif state == G_PATROL_RESET_TO_GOAL:
                # Reverse along field-forward until odometry says we're back
                # at the goal line (may take more than one loop if a drive leg
                # short-falls).
                motors.stop()
                is_moving = False
                if forward_from_goal_cm <= 3.0:
                    forward_from_goal_cm = 0.0
                    lateral_from_center_cm = 0.0
                    patrol_heading_offset_deg = 0
                    configure_goalie_sweep(scanner_front)
                    state, state_enter_ms = _enter_state(G_GO_TO_PATROL)
                    _debug_log("DBG|event=PATROL_RESET_DONE state->G_GO_TO_PATROL")
                else:
                    retreat_cm = forward_from_goal_cm
                    covered = drive_distance_cm(
                        motors, encoders,
                        retreat_cm,
                        reverse=True,
                        scanner_rear=scanner_rear,
                        lines=lines,
                    )
                    forward_from_goal_cm = max(0.0, forward_from_goal_cm - covered)
                    _debug_log(
                        "DBG|event=PATROL_RESET_LEG covered={:.1f} fwd={:.1f}".format(
                            covered, forward_from_goal_cm
                        )
                    )

            elif state == G_LATERAL_STEP:
                # Legacy state kept for compatibility; never entered by the
                # new patrol cycle. If we somehow land here, divert back to
                # the forward-look sweep so the FSM doesn't deadlock.
                state, state_enter_ms = _enter_state(G_PATROL_SWEEP)
                _debug_log("DBG|event=LEGACY_LATERAL state->G_PATROL_SWEEP")

            elif state == G_APPROACH:
                if latest_tof_mm is None:
                    tof_lost_hits += 1
                    if tof_lost_hits >= config.TOF_LOST_COUNT:
                        motors.stop()
                        is_moving = False
                        tof_lost_hits = 0
                        state, state_enter_ms = _enter_state(G_PATROL_SWEEP)
                        _debug_log("DBG|event=TOF_LOST state->G_PATROL_SWEEP")
                        continue
                    # Brief dropout: keep coasting at approach-far speed, NOT full.
                    if not is_moving:
                        motors.start_smoothly(
                            GOALIE_APPROACH_FAR_LEFT_SPEED,
                            GOALIE_APPROACH_FAR_RIGHT_SPEED,
                        )
                        is_moving = True
                else:
                    if not is_tof_ball_candidate(latest_tof_mm):
                        tof_lost_hits += 1
                        if tof_lost_hits >= config.TOF_LOST_COUNT:
                            motors.stop()
                            is_moving = False
                            tof_lost_hits = 0
                            state, state_enter_ms = _enter_state(G_PATROL_SWEEP)
                            _debug_log("DBG|event=TOF_LOST state->G_PATROL_SWEEP")
                            continue
                    else:
                        tof_lost_hits = 0

                    # Project chase distance into field coords using the
                    # rover's heading offset (set by the patrol cycle:
                    # 0, +30, or -15 deg). cos/sin scale the encoder distance.
                    approach_l, approach_r = encoders.get_pulses()
                    approach_cm = max(approach_l, approach_r) * EFFECTIVE_CM_PER_PULSE
                    heading_rad = math.radians(patrol_heading_offset_deg)
                    approach_dx = approach_cm * math.cos(heading_rad)
                    approach_dy = approach_cm * math.sin(heading_rad)
                    live_forward = forward_from_goal_cm + approach_dx
                    live_lateral = lateral_from_center_cm + approach_dy

                    if detector.is_capture_range(latest_tof_mm):
                        # Creep-and-close: do NOT stop before closing the
                        # claw. At capture range (60 mm) the ball is still
                        # ~6 cm in front of the ToF lens; stopping first
                        # leaves the arms closing on empty air.
                        forward_from_goal_cm = live_forward
                        lateral_from_center_cm = live_lateral
                        motors.forward()
                        motors.set_speeds(
                            config.APPROACH_CREEP_LEFT_SPEED,
                            config.APPROACH_CREEP_RIGHT_SPEED,
                        )
                        is_moving = True
                        claw_obj.close()
                        time.sleep_ms(config.CLAW_SETTLE_MS)
                        _hard_stop(motors)
                        is_moving = False
                        scanner_front.pause_sweep()
                        secured_hits = 0
                        tof_lost_hits = 0
                        state, state_enter_ms = _enter_state(G_CAPTURE_VERIFY)
                        _debug_log(
                            "DBG|event=CAPTURE_TRIGGER_CREEP fwd={:.1f} lat={:.1f} state->G_CAPTURE_VERIFY".format(
                                forward_from_goal_cm, lateral_from_center_cm
                            )
                        )
                        continue

                    # Goalie speed ladder: NEVER full forward during approach.
                    # As soon as we see a ball we cruise at "approach far"
                    # (currently == APPROACH_MID), then progressively slow.
                    if latest_tof_mm <= config.TOF_APPROACH_SLOW_MM:
                        target_left = config.APPROACH_CREEP_LEFT_SPEED
                        target_right = config.APPROACH_CREEP_RIGHT_SPEED
                    elif latest_tof_mm <= config.TOF_APPROACH_MID_MM:
                        target_left = config.APPROACH_MID_LEFT_SPEED
                        target_right = config.APPROACH_MID_RIGHT_SPEED
                    else:
                        target_left = GOALIE_APPROACH_FAR_LEFT_SPEED
                        target_right = GOALIE_APPROACH_FAR_RIGHT_SPEED

                    if not is_moving:
                        if motors.start_smoothly(target_left, target_right):
                            is_moving = True
                    else:
                        motors.forward()
                        motors.set_speeds(target_left, target_right)

            elif state == G_CAPTURE_VERIFY:
                if detector.is_secured_in_box(latest_tof_mm):
                    secured_hits += 1
                else:
                    secured_hits = 0
                if secured_hits >= 2:
                    state, state_enter_ms = _enter_state(G_CLASSIFY_IN_BOX)
                    _debug_log("DBG|event=BALL_SECURED state->G_CLASSIFY_IN_BOX")
                elif time.ticks_diff(loop_now, state_enter_ms) > config.CAPTURE_VERIFY_TIMEOUT_MS:
                    state, state_enter_ms = _enter_state(G_RECOVER)
                    _debug_log("DBG|event=CAPTURE_TIMEOUT state->G_RECOVER")

            elif state == G_CLASSIFY_IN_BOX:
                # Ball is clamped, ToF ~0 mm, ball is under the TCS34725 window.
                try:
                    color = classify_ball_voted(color_sensor)
                except Exception as exc:  # noqa: BLE001 (defensive)
                    _debug_log("DBG|event=COLOR_ERROR err={}".format(exc))
                    color = "NOT RED"
                # Re-align to forward heading BEFORE we route into either the
                # reject-here kick (which fires straight ahead) or the return-
                # home navigation (whose math assumes heading 0). If the ball
                # was caught during a LEFT/RIGHT look, undo that 45 deg turn
                # so the rover ends up facing forward again.
                if patrol_heading_offset_deg != 0:
                    realign_dir = "RIGHT" if patrol_heading_offset_deg < 0 else "LEFT"
                    goalie_turn(
                        motors, encoders,
                        abs(patrol_heading_offset_deg), realign_dir,
                    )
                    _debug_log(
                        "DBG|event=REALIGN_FWD turn={}{} from_heading={}".format(
                            abs(patrol_heading_offset_deg), realign_dir,
                            patrol_heading_offset_deg,
                        )
                    )
                    patrol_heading_offset_deg = 0
                if color == "RED":
                    state, state_enter_ms = _enter_state(G_REJECT_HERE)
                    _debug_log("DBG|event=CLASSIFY_RED state->G_REJECT_HERE")
                else:
                    holding_ball = True
                    state, state_enter_ms = _enter_state(G_RETURN_HOME)
                    _debug_log("DBG|event=CLASSIFY_NOT_RED state->G_RETURN_HOME")

            elif state == G_REJECT_HERE:
                # We are facing the field. Forward kick sends the red ball away
                # from our own goal. Lane safety: if anything tall is within
                # GOALIE_KICK_CLEAR_CM, back off slightly first.
                if front_distance < GOALIE_KICK_CLEAR_CM:
                    drive_distance_cm(
                        motors, encoders,
                        10.0,
                        reverse=True,
                        scanner_rear=scanner_rear,
                        lines=lines,
                    )

                configure_goalie_sweep(scanner_front)
                claw_obj.open()
                time.sleep_ms(config.CLAW_SETTLE_MS)
                kick()
                kick_latch_active = True
                holding_ball = False
                state, state_enter_ms = _enter_state(G_GO_TO_PATROL)
                _debug_log("DBG|event=KICK_RED state->G_GO_TO_PATROL")

            elif state == G_RETURN_HOME:
                if not holding_ball:
                    # Defensive retreat to patrol anchor if we entered here
                    # without a ball (e.g. future extensions). Re-align first
                    # so reverse is toward the goal, not diagonal.
                    if patrol_heading_offset_deg != 0:
                        realign_dir = (
                            "RIGHT" if patrol_heading_offset_deg < 0 else "LEFT"
                        )
                        goalie_turn(
                            motors, encoders,
                            abs(patrol_heading_offset_deg), realign_dir,
                        )
                        patrol_heading_offset_deg = 0
                    retreat_cm = max(0.0, forward_from_goal_cm - GOALIE_PATROL_FORWARD_CM)
                    covered = drive_distance_cm(
                        motors, encoders,
                        retreat_cm,
                        reverse=True,
                        scanner_rear=scanner_rear,
                        lines=lines,
                    )
                    forward_from_goal_cm = max(0.0, forward_from_goal_cm - covered)
                    state, state_enter_ms = _enter_state(G_PATROL_SWEEP)
                    _debug_log(
                        "DBG|event=RETREAT_ANCHOR covered={:.1f} fwd={:.1f} state->G_PATROL_SWEEP".format(
                            covered, forward_from_goal_cm
                        )
                    )
                    continue

                # We have a NOT-RED ball clamped: aim at the goal pad and
                # drive there. This accounts for the lateral offset from any
                # patrol side-steps -- a plain 180 + drive only undoes the
                # forward component and would land us NEXT to the goal mouth.
                turn_deg, turn_dir, distance_cm = _compute_return_path(
                    forward_from_goal_cm, lateral_from_center_cm
                )
                _debug_log(
                    "DBG|event=RETURN_PATH F={:.1f} L={:.1f} turn={:.1f}{} drive={:.1f}".format(
                        forward_from_goal_cm, lateral_from_center_cm,
                        turn_deg, turn_dir, distance_cm,
                    )
                )
                if turn_deg > 0.5:
                    goalie_turn(motors, encoders, turn_deg, turn_dir)
                covered = drive_distance_cm(
                    motors, encoders,
                    distance_cm,
                    scanner_front=scanner_front,
                    front_stop_cm=GOALIE_GOAL_REACH_CM,
                    lines=lines,
                )
                # Pose is now ~(0, 0) facing the goal backstop. Reset.
                forward_from_goal_cm = 0.0
                lateral_from_center_cm = 0.0
                # Heading is irrelevant while at the goal mouth -- the drop
                # state will turn 180 anyway. Mark it as forward for after.
                patrol_heading_offset_deg = 0
                state, state_enter_ms = _enter_state(G_DROP_IN_GOAL)
                _debug_log(
                    "DBG|event=AT_GOAL covered={:.1f} state->G_DROP_IN_GOAL".format(covered)
                )

            elif state == G_DROP_IN_GOAL:
                motors.stop()
                is_moving = False
                scanner_front.pause_sweep()
                claw_obj.open()
                time.sleep_ms(config.CLAW_SETTLE_MS)
                kick()  # gentle pulse over the speedbump
                kick_latch_active = True
                holding_ball = False
                # Reverse out of the goal mouth. While we were facing the
                # backstop, "reverse" pushes us INTO the field (+forward in
                # field coords).
                backout_covered = drive_distance_cm(
                    motors, encoders,
                    GOALIE_GOAL_BACKOUT_CM,
                    reverse=True,
                    scanner_rear=scanner_rear,
                    lines=lines,
                )
                # Now turn 180 to face the field again.
                goalie_turn(motors, encoders, 180, "RIGHT")
                # The rover is now backout_covered cm into the field, facing
                # +forward. THIS is the new pose -- not (0, 0). Setting it
                # to 0 here would cause cumulative drift across ball cycles.
                forward_from_goal_cm = backout_covered
                lateral_from_center_cm = 0.0
                patrol_heading_offset_deg = 0
                configure_goalie_sweep(scanner_front)
                state, state_enter_ms = _enter_state(G_GO_TO_PATROL)
                _debug_log(
                    "DBG|event=DROP_DONE backout={:.1f} state->G_GO_TO_PATROL".format(
                        backout_covered
                    )
                )

            elif state == G_RECOVER:
                claw_obj.open()
                holding_ball = False
                secured_hits = 0
                scanner_front.pause_sweep()
                motors.stop()
                is_moving = False

                if line_stop_confirmed:
                    drive_distance_cm(
                        motors, encoders,
                        20.0,
                        reverse=True,
                        scanner_rear=scanner_rear,
                        lines=None,  # we want to escape the line, don't stop on it
                    )
                    line_stop_hits = 0

                goalie_turn(motors, encoders, config.RECOVERY_TURN_DEG, "RIGHT")
                # Odometry is untrusted. Assume worst case: we might already
                # be at the first patrol stop (~3 ft) so the next G_GO_TO_PATROL
                # skips its forward leg (remaining_cm=0) and goes straight to
                # sweep.
                forward_from_goal_cm = GOALIE_PATROL_FORWARD_CM
                lateral_from_center_cm = 0.0
                patrol_heading_offset_deg = 0
                configure_goalie_sweep(scanner_front)
                state, state_enter_ms = _enter_state(G_GO_TO_PATROL)
                _debug_log(
                    "DBG|event=RECOVER_DONE pose=worst-case fwd={:.1f} state->G_GO_TO_PATROL".format(
                        forward_from_goal_cm
                    )
                )

            # ---------------------------
            # Periodic debug telemetry
            # ---------------------------
            if config.DEBUG_ENABLE and time.ticks_diff(loop_now, last_debug_ms) >= config.DEBUG_PERIOD_MS:
                print(
                    "DBG|t={} st={} tof={} front={} fwd={:.1f} lat={:.1f} hd={:+d} lineHits={}".format(
                        loop_now, state, latest_tof_mm, front_distance,
                        forward_from_goal_cm, lateral_from_center_cm,
                        patrol_heading_offset_deg, line_stop_hits,
                    )
                )
                last_debug_ms = loop_now

            time.sleep_ms(config.MAIN_LOOP_MS)

    except KeyboardInterrupt:
        motors.stop()
        try:
            scanner_front.pause_sweep()
        except Exception:
            pass
        try:
            pico_led.value(0)
        except Exception:
            pass
        print("\nSYS_HALT: Goalie aborted by user.")


# ---------------------------------------------------------------------------
# Bench TURN test (in-place rotation calibration)
# ---------------------------------------------------------------------------
def run_turn_test(angle_deg=90, direction="RIGHT", repeats=4):
    """Spin in place and check whether `angle_deg` commanded == angle observed.

    REQUIREMENT: wheels MUST be on a flat floor (not suspended).
    Place a strip of tape on the floor lined up with the front of the rover.
    After each turn, eyeball / protractor the new heading.

    From the REPL:
        from goalie import run_turn_test
        run_turn_test(90, "RIGHT", 4)   # four 90-deg right turns = 360
        run_turn_test(180, "RIGHT", 2)  # two 180s = 360
        run_turn_test(90, "LEFT", 4)    # symmetry check

    HOW TO TUNE GOALIE_TURN_CALIBRATION:
        Rover spun ~70 deg when commanded 90 deg -> new = 90 / 70 = 1.29
        Rover spun ~110 deg when commanded 90 deg -> new = 90 / 110 = 0.82
    """
    print("\n" + "=" * 50)
    print("GOALIE TURN TEST: {} x {} deg {}".format(repeats, angle_deg, direction))
    print("=" * 50)
    motors = MotorController()
    encoders = WheelEncoders()
    motors.stop()

    print(
        "calibration={}  track={} cm  wheel_dia={} cm  slots={}".format(
            GOALIE_TURN_CALIBRATION,
            config.TRACK_WIDTH_CM,
            config.WHEEL_DIAMETER_CM,
            config.DISK_SLOTS,
        )
    )
    print("Starting in 3 s. Watch the rover and measure each turn...")
    time.sleep(3)

    try:
        for i in range(repeats):
            print("--- turn {} ---".format(i + 1))
            ok = goalie_turn(motors, encoders, angle_deg, direction)
            l, r = encoders.get_pulses()
            print(
                "result ok={} encoder L={} R={}".format(ok, l, r)
            )
            time.sleep(1)
    finally:
        motors.stop()
    print(
        "Total commanded: {} deg.  Compare to physical heading change.".format(
            angle_deg * repeats
        )
    )


# ---------------------------------------------------------------------------
# Live ToF reader (debugging tool)
# ---------------------------------------------------------------------------
def run_tof_live_test(seconds=20):
    """
    Print ToF distance every ~150 ms for `seconds` seconds. Polls the sensor
    directly (NOT IRQ-driven) so you can confirm:
      - The sensor is responding at all
      - At what range it sees your ball (compare to GOALIE_TOF_DETECT_MM)
      - Whether the offset (TOF_OFFSET_MM) is reasonable

    From the REPL:
        from goalie import run_tof_live_test
        run_tof_live_test(20)

    Move a ball through 5 cm, 10 cm, 30 cm, 60 cm, 100 cm, 150 cm in front of
    the sensor while watching the output.
    """
    print("\n" + "=" * 50)
    print("LIVE TOF READER (direct polling, not IRQ)")
    print("Goalie detect window: 0..{} mm".format(GOALIE_TOF_DETECT_MM))
    print("Capture trigger:     <= {} mm".format(config.TOF_CAPTURE_MM))
    print("=" * 50)
    detector = BallDetector()
    end_ms = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
    while time.ticks_diff(end_ms, time.ticks_ms()) > 0:
        try:
            d = detector.get_distance()
        except Exception as exc:
            print("READ ERR: {}".format(exc))
            time.sleep_ms(150)
            continue
        if d is None:
            tag = "no-target"
        elif d > GOALIE_TOF_DETECT_MM:
            tag = "OUTSIDE detect window"
        elif d <= config.TOF_CAPTURE_MM:
            tag = "CAPTURE"
        elif d <= config.TOF_APPROACH_SLOW_MM:
            tag = "creep"
        elif d <= config.TOF_APPROACH_MID_MM:
            tag = "mid"
        else:
            tag = "far (would slow down)"
        print("tof_mm={} -> {}".format(d, tag))
        time.sleep_ms(150)
    print("Live ToF test done.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    goalie_main()
