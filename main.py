from machine import Pin
import time

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
import math


ROVER_CONFIG = {
    "obs_thresh": config.OBSTACLE_THRESHOLD_CM,
    "rear_thresh": config.REAR_THRESHOLD_CM,
    "fwd_speeds": (config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED),
    "trn_speeds": (config.TURN_LEFT_SPEED, config.TURN_RIGHT_SPEED),
    "track_width": config.TRACK_WIDTH_CM,
    "wheel_dia": config.WHEEL_DIAMETER_CM,
    "disk_slots": config.DISK_SLOTS,
}


STATE_SEARCH = "SEARCH"
STATE_START_ADVANCE = "START_ADVANCE"
STATE_SCAN = "SCAN"
STATE_APPROACH = "APPROACH"
STATE_CAPTURE_VERIFY = "CAPTURE_VERIFY"
STATE_CLASSIFY = "CLASSIFY"
STATE_ROUTE_RED = "ROUTE_RED"
STATE_ROUTE_NOT_RED = "ROUTE_NOT_RED"
STATE_RECOVER = "RECOVER"


def classify_ball(sensor):
    red_votes = 0
    samples = max(1, config.COLOR_SAMPLES)
    for _ in range(samples):
        if sensor.check_ball() == "RED":
            red_votes += 1
        time.sleep_ms(30)
    return "RED" if red_votes > (samples // 2) else "NOT RED"


def enter_state(new_state):
    return new_state, time.ticks_ms()


def drive_forward_distance_cm(motors, encoders, cm):
    """Blocking helper for short planned moves (startup/search bump)."""
    if cm <= 0:
        return
    wheel_circ = math.pi * config.WHEEL_DIAMETER_CM
    target_revs = cm / wheel_circ
    target_pulses = int(target_revs * config.DISK_SLOTS)

    encoders.reset()
    if not motors.start_smoothly(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED):
        return

    start_ms = time.ticks_ms()
    while True:
        l_p, r_p = encoders.get_pulses()
        if max(l_p, r_p) >= target_pulses:
            break
        if motors.check_faults():
            break
        if time.ticks_diff(time.ticks_ms(), start_ms) > 3000:
            break
        time.sleep_ms(15)
    motors.stop()


def is_tof_ball_candidate(tof_mm):
    return (
        tof_mm is not None
        and tof_mm >= config.TOF_MIN_VALID_MM
        and tof_mm <= config.TOF_APPROACH_MM
    )


def debug_log(msg):
    if config.DEBUG_ENABLE:
        print(msg)


def debug_telemetry(
    now_ms, state, tof_mm, front_cm, secure_hits, fail_count, left_raw, right_raw, line_hits
):
    if not config.DEBUG_ENABLE:
        return False
    print(
        "DBG|t={} state={} tof_mm={} front_cm={} sec_hits={} fails={} left={} right={} lineHits={}".format(
            now_ms, state, tof_mm, front_cm, secure_hits, fail_count, left_raw, right_raw, line_hits
        )
    )
    return True


pico_led = Pin("LED", Pin.OUT)
pico_led.value(1)

motors = MotorController()
scanner_front = UltrasonicScanner()
scanner_rear = FixedUltrasonicScanner(config.PIN_REAR_TRIG, config.PIN_REAR_ECHO)
encoders = WheelEncoders()
lines = LineSensor()
detector = BallDetector()
color_sensor = RedBallSensor()
claw = Claw()
avoider = ObstacleAvoidance(motors, scanner_front, scanner_rear, encoders, ROVER_CONFIG)

if getattr(config, "LINE_USE_ADAPTIVE", False):
    print(
        "LINE|adaptive baseline left={} right={}".format(
            lines.left_baseline, lines.right_baseline
        )
    )

print("KICKER MODE: Unified FSM starting.")
print(
    "MODE|lineSafety={} obstacleAvoid={}".format(
        config.ENABLE_LINE_SAFETY, config.ENABLE_OBSTACLE_AVOID
    )
)

state = STATE_START_ADVANCE
state_enter_ms = time.ticks_ms()
is_moving = False
latest_tof_mm = None
secured_hits = 0
consecutive_capture_fails = 0
last_debug_ms = time.ticks_ms()
line_stop_hits = 0
kick_latch_active = False
clear_hits = 0
recover_due_to_line = False
tof_lost_hits = 0

claw.open()
scanner_front.start_sweep()

try:
    while True:
        loop_now = time.ticks_ms()

        if detector.data_ready:
            latest_tof_mm = detector.get_distance()
            detector.data_ready = False

        front_distance = scanner_front.get_distance()
        look_angle = scanner_front.current_angle
        left_raw, right_raw = lines.get_raw_values()
        ball_secured_now = detector.is_secured_in_box(latest_tof_mm)

        rover_close_not_ball = (
            latest_tof_mm is not None
            and latest_tof_mm <= config.ROVER_PROX_TOF_MM
            and front_distance <= config.ROVER_PROX_ULTRA_CM
            and not ball_secured_now
        )

        # Prevent repeated kicking of the same preloaded ball.
        if kick_latch_active:
            if latest_tof_mm is not None and latest_tof_mm >= config.TOF_CLEAR_MM:
                clear_hits += 1
            else:
                clear_hits = 0
            if clear_hits >= config.TOF_CLEAR_COUNT:
                kick_latch_active = False
                clear_hits = 0
                debug_log("DBG|event=KICK_LATCH_CLEARED")

        # Global safety checks
        line_stop_now = lines.should_kicker_stop() if config.ENABLE_LINE_SAFETY else False
        if line_stop_now:
            line_stop_hits += 1
        else:
            line_stop_hits = 0

        line_stop_confirmed = line_stop_hits >= config.LINE_STOP_DEBOUNCE_COUNT

        if motors.check_faults() or line_stop_confirmed:
            motors.stop()
            is_moving = False
            recover_due_to_line = line_stop_confirmed
            state, state_enter_ms = enter_state(STATE_RECOVER)
            debug_log(
                "DBG|event=SAFETY_TRIGGER state->RECOVER lineHits={} black={} blue={}".format(
                    line_stop_hits, left_raw, right_raw
                )
            )

        if (
            config.ENABLE_OBSTACLE_AVOID
            and front_distance < config.OBSTACLE_THRESHOLD_CM
            and state in (STATE_SEARCH, STATE_APPROACH)
        ):
            if is_moving:
                motors.stop()
                is_moving = False
            avoider.navigate_obstacle(front_distance, look_angle)
            state, state_enter_ms = enter_state(STATE_SEARCH)
            debug_log("DBG|event=OBSTACLE_AVOID state->SEARCH")
            continue

        # If both sensors say very close but box-secure is false, treat as rover.
        if rover_close_not_ball and state in (STATE_SEARCH, STATE_SCAN, STATE_APPROACH):
            motors.stop()
            is_moving = False
            recover_due_to_line = False
            state, state_enter_ms = enter_state(STATE_RECOVER)
            debug_log("DBG|event=ROVER_PROXIMITY state->RECOVER")
            continue

        # Global anti-stuck timeout for all active states except search
        if state != STATE_SEARCH and time.ticks_diff(loop_now, state_enter_ms) > config.STATE_TIMEOUT_MS:
            motors.stop()
            is_moving = False
            recover_due_to_line = False
            state, state_enter_ms = enter_state(STATE_RECOVER)
            debug_log("DBG|event=STATE_TIMEOUT state->RECOVER")

        if state == STATE_START_ADVANCE:
            motors.stop()
            is_moving = False
            drive_forward_distance_cm(motors, encoders, config.START_FORWARD_DISTANCE_CM)
            state, state_enter_ms = enter_state(STATE_SCAN)
            debug_log("DBG|event=START_ADVANCE_DONE state->SCAN")

        elif state == STATE_SCAN:
            if is_tof_ball_candidate(latest_tof_mm):
                state, state_enter_ms = enter_state(STATE_APPROACH)
                debug_log("DBG|event=SCAN_DETECT_CENTER state->APPROACH")
                continue

            scanner_front.start_sweep()
            avoider.execute_exact_turn(config.START_SCAN_TURN_DEG, "LEFT")
            time.sleep_ms(config.SCAN_SETTLE_MS)
            if is_tof_ball_candidate(latest_tof_mm):
                state, state_enter_ms = enter_state(STATE_APPROACH)
                debug_log("DBG|event=SCAN_DETECT_LEFT state->APPROACH")
                continue

            # Return to original heading before scanning right.
            avoider.execute_exact_turn(config.START_SCAN_TURN_DEG, "RIGHT")
            time.sleep_ms(config.SCAN_SETTLE_MS)

            avoider.execute_exact_turn(config.START_SCAN_TURN_DEG, "RIGHT")
            time.sleep_ms(config.SCAN_SETTLE_MS)
            if is_tof_ball_candidate(latest_tof_mm):
                state, state_enter_ms = enter_state(STATE_APPROACH)
                debug_log("DBG|event=SCAN_DETECT_RIGHT state->APPROACH")
                continue

            # Re-center and bump forward a bit instead of reversing.
            avoider.execute_exact_turn(config.START_SCAN_TURN_DEG, "LEFT")
            motors.forward()
            motors.set_speeds(config.APPROACH_CREEP_LEFT_SPEED, config.APPROACH_CREEP_RIGHT_SPEED)
            time.sleep_ms(config.SCAN_FORWARD_BUMP_MS)
            motors.stop()
            state, state_enter_ms = enter_state(STATE_SCAN)
            debug_log("DBG|event=SCAN_REPOSITION_FORWARD")

        elif state == STATE_SEARCH:
            # If a ball is already in the box at startup/recovery, skip chase.
            if detector.is_secured_in_box(latest_tof_mm) and not kick_latch_active:
                motors.stop()
                is_moving = False
                state, state_enter_ms = enter_state(STATE_CLASSIFY)
                debug_log("DBG|event=PRELOAD_SECURED state->CLASSIFY")
                continue

            if not is_moving:
                if motors.start_smoothly(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED):
                    is_moving = True

            if is_tof_ball_candidate(latest_tof_mm):
                state, state_enter_ms = enter_state(STATE_APPROACH)
                debug_log("DBG|event=TOF_APPROACH state->APPROACH")

        elif state == STATE_APPROACH:
            if latest_tof_mm is None:
                tof_lost_hits += 1
                if tof_lost_hits >= config.TOF_LOST_COUNT:
                    motors.stop()
                    is_moving = False
                    state, state_enter_ms = enter_state(STATE_SCAN)
                    debug_log("DBG|event=TOF_LOST state->SCAN")
                    tof_lost_hits = 0
                    continue
                if not is_moving:
                    motors.start_smoothly(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)
                    is_moving = True
            else:
                if not is_tof_ball_candidate(latest_tof_mm):
                    tof_lost_hits += 1
                    if tof_lost_hits >= config.TOF_LOST_COUNT:
                        motors.stop()
                        is_moving = False
                        state, state_enter_ms = enter_state(STATE_SCAN)
                        debug_log("DBG|event=TOF_LOST state->SCAN")
                        tof_lost_hits = 0
                        continue
                else:
                    tof_lost_hits = 0

                if detector.is_capture_range(latest_tof_mm):
                    # User-requested behavior:
                    # stop when ToF is <=45mm and activate claw.
                    motors.stop()
                    is_moving = False
                    scanner_front.pause_sweep()
                    claw.close()
                    time.sleep_ms(config.CLAW_SETTLE_MS)
                    scanner_front.start_sweep()
                    secured_hits = 0
                    state, state_enter_ms = enter_state(STATE_CAPTURE_VERIFY)
                    debug_log("DBG|event=CAPTURE_TRIGGER state->CAPTURE_VERIFY")
                    tof_lost_hits = 0
                else:
                    # Stage down speed near the ball to reduce push-away overshoot.
                    if latest_tof_mm <= config.TOF_APPROACH_SLOW_MM:
                        motors.forward()
                        motors.set_speeds(
                            config.APPROACH_CREEP_LEFT_SPEED,
                            config.APPROACH_CREEP_RIGHT_SPEED,
                        )
                        is_moving = True
                    elif latest_tof_mm <= config.TOF_APPROACH_MID_MM:
                        motors.forward()
                        motors.set_speeds(
                            config.APPROACH_MID_LEFT_SPEED,
                            config.APPROACH_MID_RIGHT_SPEED,
                        )
                        is_moving = True
                    else:
                        # Keep approaching while not in capture range.
                        if not is_moving:
                            if motors.start_smoothly(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED):
                                is_moving = True

        elif state == STATE_CAPTURE_VERIFY:
            # Ball is considered secured only when ToF is very close to 0mm.
            if detector.is_secured_in_box(latest_tof_mm):
                secured_hits += 1
            else:
                secured_hits = 0

            if secured_hits >= 2:
                state, state_enter_ms = enter_state(STATE_CLASSIFY)
                debug_log("DBG|event=BALL_SECURED state->CLASSIFY")
            elif time.ticks_diff(loop_now, state_enter_ms) > config.CAPTURE_VERIFY_TIMEOUT_MS:
                consecutive_capture_fails += 1
                state, state_enter_ms = enter_state(STATE_RECOVER)
                debug_log("DBG|event=CAPTURE_TIMEOUT state->RECOVER")

        elif state == STATE_CLASSIFY:
            ball_color = classify_ball(color_sensor)
            if ball_color == "RED":
                state, state_enter_ms = enter_state(STATE_ROUTE_RED)
                debug_log("DBG|event=CLASSIFY_RED state->ROUTE_RED")
            else:
                state, state_enter_ms = enter_state(STATE_ROUTE_NOT_RED)
                debug_log("DBG|event=CLASSIFY_NOT_RED state->ROUTE_NOT_RED")

        elif state == STATE_ROUTE_NOT_RED:
            motors.stop()
            is_moving = False
            kick()
            kick_latch_active = True
            consecutive_capture_fails = 0
            state, state_enter_ms = enter_state(STATE_RECOVER)
            debug_log("DBG|event=KICK_NOT_RED state->RECOVER")

        elif state == STATE_ROUTE_RED:
            # Red ball handling: actively send away via turn + release/kick.
            avoider.execute_exact_turn(180, "RIGHT")
            kick()
            kick_latch_active = True
            consecutive_capture_fails = 0
            state, state_enter_ms = enter_state(STATE_RECOVER)
            debug_log("DBG|event=KICK_RED state->RECOVER")

        elif state == STATE_RECOVER:
            claw.open()
            secured_hits = 0
            scanner_front.pause_sweep()
            motors.stop()

            # Recovery uses turn + forward bump (no reverse-only roaming).
            turn_deg = config.RECOVERY_TURN_DEG
            if consecutive_capture_fails >= config.RECOVERY_MAX_CONSECUTIVE_FAILS:
                turn_deg = 120
                consecutive_capture_fails = 0

            avoider.execute_exact_turn(turn_deg, "RIGHT")
            if recover_due_to_line:
                if motors.start_smoothly_reverse(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED):
                    line_rev_start = time.ticks_ms()
                    while time.ticks_diff(time.ticks_ms(), line_rev_start) < config.RECOVERY_REVERSE_MS:
                        if motors.check_faults():
                            break
                        if not lines.should_kicker_stop():
                            break
                        time.sleep_ms(20)
                recover_due_to_line = False
            else:
                motors.forward()
                motors.set_speeds(config.APPROACH_CREEP_LEFT_SPEED, config.APPROACH_CREEP_RIGHT_SPEED)
                time.sleep_ms(config.SCAN_FORWARD_BUMP_MS)
            motors.stop()

            # If still on boundary after normal recovery, perform bounded
            # line-escape maneuvers so we don't thrash RECOVER<->SEARCH.
            escape_attempts = 0
            while (
                config.ENABLE_LINE_SAFETY
                and lines.should_kicker_stop()
                and escape_attempts < config.RECOVERY_LINE_ESCAPE_ATTEMPTS
            ):
                escape_attempts += 1
                debug_log("DBG|event=LINE_ESCAPE_ATTEMPT n={}".format(escape_attempts))
                if motors.start_smoothly_reverse(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED):
                    esc_start = time.ticks_ms()
                    while time.ticks_diff(time.ticks_ms(), esc_start) < config.RECOVERY_LINE_ESCAPE_REVERSE_MS:
                        if motors.check_faults():
                            break
                        if not lines.should_kicker_stop():
                            break
                        time.sleep_ms(20)
                motors.stop()
                if not lines.should_kicker_stop():
                    break
                avoider.execute_exact_turn(config.RECOVERY_LINE_ESCAPE_TURN_DEG, "LEFT")

            if (not config.ENABLE_LINE_SAFETY) or (not lines.should_kicker_stop()):
                line_stop_hits = 0

            scanner_front.start_sweep()
            state, state_enter_ms = enter_state(STATE_SEARCH)
            debug_log("DBG|event=RECOVER_DONE state->SEARCH")

        if time.ticks_diff(loop_now, last_debug_ms) >= config.DEBUG_PERIOD_MS:
            if debug_telemetry(
                loop_now,
                state,
                latest_tof_mm,
                front_distance,
                secured_hits,
                consecutive_capture_fails,
                left_raw,
                right_raw,
                line_stop_hits,
            ):
                last_debug_ms = loop_now

        time.sleep_ms(config.MAIN_LOOP_MS)

except KeyboardInterrupt:
    motors.stop()
    scanner_front.pause_sweep()
    pico_led.value(0)
    print("\nSYS_HALT: Mission safely aborted by user.")