"""
Goalie Stage 3: 4 ft shuttle with line + ultrasonic safety.

Sequence:
  1. Drive forward 4 ft (encoder-based)
  2. One-time 180 deg spin
  3. Loop forever:
       a. Drive forward 4 ft
       b. Reverse 4 ft

Three things can end a leg early (treated identically):
  - Line sensor sees black tape
  - Front ultrasonic sees obstacle within threshold (forward legs)
  - Rear ultrasonic sees obstacle within threshold (reverse legs)
Any trip -> stop, back off briefly, flip direction.

Front sweep servo is locked at center (128) — no arc sweep.

Run from REPL:
    from goalie_stage3 import run; run()
"""
import math
import time
from motors import MotorController
from encoders import WheelEncoders
from linesensor import LineSensor
from ultrasonics import UltrasonicScanner, FixedUltrasonicScanner
import config

# Tuning
SHUTTLE_DISTANCE_CM = 121.92      # 4 ft
PATROL_LEFT_SPEED = 55.0
PATROL_RIGHT_SPEED = 55.0
FLIP_SPIN_MS = 1600
SPIN_LEFT_SPEED = 70.0
SPIN_RIGHT_SPEED = 70.0
BACKOFF_MS = 250
LEG_TIMEOUT_MS = 8000

# Obstacle thresholds (cm)
FRONT_OBSTACLE_CM = 20.0
REAR_OBSTACLE_CM = 20.0

# Drivetrain geometry
WHEEL_DIAMETER_CM = 6.0
DISK_SLOTS = 20


def drive_distance(motors, encoders, lines, front, rear, cm, reverse=False):
    """
    Drive `cm` using encoders, with safety overrides.
    Returns (result, pulses) where result is one of:
      'DONE'  - target distance reached
      'LINE'  - line sensor tripped
      'FRONT' - front ultrasonic saw obstacle (forward legs only)
      'REAR'  - rear ultrasonic saw obstacle (reverse legs only)
      'ABORT' - OC fault or timeout
    """
    wheel_circ = math.pi * WHEEL_DIAMETER_CM
    target_pulses = int((cm / wheel_circ) * DISK_SLOTS)

    encoders.reset()
    if reverse:
        ok = motors.start_smoothly_reverse(PATROL_LEFT_SPEED, PATROL_RIGHT_SPEED)
    else:
        ok = motors.start_smoothly(PATROL_LEFT_SPEED, PATROL_RIGHT_SPEED)
    if not ok:
        return ("ABORT", 0)

    start = time.ticks_ms()
    while True:
        l_p, r_p = encoders.get_pulses()
        pulses = max(l_p, r_p)

        if pulses >= target_pulses:
            motors.stop()
            return ("DONE", pulses)

        if lines.boundary_detected():
            motors.stop()
            return ("LINE", pulses)

        # Only check the relevant ultrasonic for the current direction
        if reverse:
            d = rear.get_distance()
            if d < REAR_OBSTACLE_CM:
                motors.stop()
                return ("REAR", pulses)
        else:
            d = front.get_distance()
            if d < FRONT_OBSTACLE_CM:
                motors.stop()
                return ("FRONT", pulses)

        if motors.check_faults():
            return ("ABORT", pulses)
        if time.ticks_diff(time.ticks_ms(), start) > LEG_TIMEOUT_MS:
            print("  [!] Leg timed out")
            motors.stop()
            return ("ABORT", pulses)

        time.sleep_ms(15)


def back_off(motors, was_forward):
    """Briefly drive opposite the just-completed leg to clear the trip."""
    if was_forward:
        motors.start_smoothly_reverse(PATROL_LEFT_SPEED, PATROL_RIGHT_SPEED)
    else:
        motors.start_smoothly(PATROL_LEFT_SPEED, PATROL_RIGHT_SPEED)
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < BACKOFF_MS:
        if motors.check_faults():
            motors.stop()
            return
        time.sleep_ms(20)
    motors.stop()


def spin_180(motors):
    motors.turn_right()
    motors.set_speeds(SPIN_LEFT_SPEED, SPIN_RIGHT_SPEED)
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < FLIP_SPIN_MS:
        if motors.check_faults():
            motors.stop()
            return False
        time.sleep_ms(20)
    motors.stop()
    return True


def run():
    print("=" * 50)
    print("GOALIE STAGE 3: 4 ft shuttle with line + ultrasonic safety")
    print("=" * 50)
    print("Place rover near goal, facing blue line. Starting in 3 s...")
    time.sleep(3)

    motors = MotorController()
    encoders = WheelEncoders()
    lines = LineSensor()
    front = UltrasonicScanner()
    rear = FixedUltrasonicScanner(config.PIN_REAR_TRIG, config.PIN_REAR_ECHO)

    # Lock front servo at center, no sweeping
    front.set_servo_angle(128)
    motors.stop()

    leg_count = 0
    try:
        # Step 1: initial approach
        print("--- Approach: forward {} cm ---".format(SHUTTLE_DISTANCE_CM))
        result, pulses = drive_distance(motors, encoders, lines, front, rear, SHUTTLE_DISTANCE_CM, reverse=False)
        print("  result={} pulses={}".format(result, pulses))
        if result in ("LINE", "FRONT"):
            print("  [!] Trip during approach, backing off")
            back_off(motors, was_forward=True)
        elif result == "ABORT":
            print("  [!] Approach aborted")
            return

        # Step 2: one-time 180 deg spin
        print("--- Spin 180 ({} ms) ---".format(FLIP_SPIN_MS))
        if not spin_180(motors):
            print("  [!] Spin aborted")
            return

        # Step 3: shuttle forever
        going_forward = True
        while True:
            leg_count += 1
            direction_str = "forward" if going_forward else "reverse"
            print("--- Leg {}: {} {} cm ---".format(leg_count, direction_str, SHUTTLE_DISTANCE_CM))
            result, pulses = drive_distance(
                motors, encoders, lines, front, rear,
                SHUTTLE_DISTANCE_CM, reverse=(not going_forward)
            )
            print("  result={} pulses={}".format(result, pulses))

            if result in ("LINE", "FRONT", "REAR"):
                print("  [!] {} trip, backing off".format(result))
                back_off(motors, was_forward=going_forward)
            elif result == "ABORT":
                print("  [!] Leg aborted, stopping")
                break

            going_forward = not going_forward

    except KeyboardInterrupt:
        print("\nAborted by user.")
    finally:
        motors.stop()
        print("=" * 50)
        print("Stopped after {} shuttle legs.".format(leg_count))
        print("=" * 50)


if __name__ == "__main__":
    run()