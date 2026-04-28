"""
Red Raider Soccer - Main Control
Integrates: motors, encoders, ultrasonic, line sensor, color sensor, role/team
"""
from machine import Pin
import time
import math

from motors import MotorController
from ultrasonic import UltrasonicScanner
from encoders import WheelEncoders
from linesensor import LineSensors
from colorsensor import init_color_sensor, check_ball
from Team import pick_team
from Role import pick_role

# ================================================================
# USER CALIBRATION & TUNING VARIABLES
# ================================================================

# Asymmetrical Straight Driving Speeds
FWD_LEFT_SPEED = 51.2
FWD_RIGHT_SPEED = 60.0

# Asymmetrical Rotation Speeds
TURN_LEFT_SPEED = 68.3
TURN_RIGHT_SPEED = 80.0

# Physical Dimensions
TRACK_WIDTH_CM = 20.0
WHEEL_DIAMETER_CM = 6.0
DISK_SLOTS = 20

# Avoidance Tuning
OBSTACLE_THRESHOLD_CM = 25.0

# ================================================================
# System Initialization
# ================================================================
pico_led = Pin("LED", Pin.OUT)
pico_led.value(1)

motors = MotorController()
scanner = UltrasonicScanner(servo_pin=10)   # moved off GP0 to GP10
encoders = WheelEncoders()
lines = LineSensors()
init_color_sensor()

# ================================================================
# Read Role and Team at Startup
# ================================================================
TEAM = pick_team()       # "PURPLE" or "YELLOW"
ROLE = pick_role()       # "STRIKER" or "GOALIE"

print("=" * 40)
print("Red Raider Soccer - Rover Online")
print("Team: {}".format(TEAM))
print("Role: {}".format(ROLE))
print("=" * 40)

# ================================================================
# Helper Functions
# ================================================================
def execute_exact_turn(angle_degrees, direction="RIGHT"):
    """Encoder-controlled turn using calibrated turning speeds."""
    robot_circ = math.pi * TRACK_WIDTH_CM
    wheel_circ = math.pi * WHEEL_DIAMETER_CM
    revolutions_needed = ((angle_degrees / 360.0) * robot_circ) / wheel_circ
    target_pulses = int(revolutions_needed * DISK_SLOTS)

    encoders.reset()

    if direction == "RIGHT":
        motors.turn_right()
    else:
        motors.turn_left()

    motors.set_speeds(TURN_LEFT_SPEED, TURN_RIGHT_SPEED)

    while True:
        left_p, right_p = encoders.get_pulses()
        avg_pulses = (left_p + right_p) / 2

        if avg_pulses >= target_pulses or motors.check_faults():
            break
        time.sleep(0.01)

    motors.stop()


def assess_and_escape():
    """Use scanner to find an open path and escape."""
    motors.stop()
    time.sleep(0.5)

    scanner.set_servo_angle(160)
    time.sleep(0.4)
    dist_left = scanner.get_distance()

    scanner.set_servo_angle(20)
    time.sleep(0.4)
    dist_right = scanner.get_distance()

    scanner.set_servo_angle(90)  # center

    if dist_left > dist_right and dist_left > OBSTACLE_THRESHOLD_CM:
        motors.turn_left()
        motors.set_speeds(TURN_LEFT_SPEED, TURN_RIGHT_SPEED)
        time.sleep(0.6)
    elif dist_right >= dist_left and dist_right > OBSTACLE_THRESHOLD_CM:
        motors.turn_right()
        motors.set_speeds(TURN_LEFT_SPEED, TURN_RIGHT_SPEED)
        time.sleep(0.6)
    else:
        # Boxed in: reverse then spin
        motors.reverse()
        motors.set_speeds(FWD_LEFT_SPEED, FWD_RIGHT_SPEED)
        time.sleep(1.0)
        motors.turn_right()
        motors.set_speeds(TURN_LEFT_SPEED, TURN_RIGHT_SPEED)
        time.sleep(0.8)
    motors.stop()


# ================================================================
# Main Mission Loop (Placeholder - Chassis Test Behavior)
# ================================================================
# This is currently chassis-test behavior. The full goalie/kicker state
# machine will replace this once navigation.py is written.
# ================================================================
print("Systems Ready. Loaded Asymmetrical Calibration Profile.")
is_moving = False
last_scan_time = time.ticks_ms()
last_color_check = time.ticks_ms()

try:
    while True:
        # 1. Fault Protection Check
        if motors.system_fault:
            motors.check_faults()
            time.sleep(0.5)
            is_moving = False
            continue

        # 2. Line Detection (black tape)
        left_on_line, right_on_line = lines.read()

        if left_on_line or right_on_line:
            print("Line Detected! Evading...")
            motors.stop()
            motors.reverse()
            motors.set_speeds(FWD_LEFT_SPEED, FWD_RIGHT_SPEED)
            time.sleep(0.5)

            if left_on_line and right_on_line:
                execute_exact_turn(180, "RIGHT")
            elif left_on_line:
                execute_exact_turn(90, "RIGHT")
            elif right_on_line:
                execute_exact_turn(90, "LEFT")

            is_moving = False
            continue

        # 3. Obstacle Avoidance (collision-only purpose)
        if time.ticks_diff(time.ticks_ms(), last_scan_time) >= 300:
            last_scan_time = time.ticks_ms()
            distance = scanner.get_distance()

            if distance < OBSTACLE_THRESHOLD_CM:
                print("Obstacle at {}cm!".format(distance))
                is_moving = False
                assess_and_escape()

        # 4. Color Sensor Check (only meaningful after capture, for now: just log)
        if time.ticks_diff(time.ticks_ms(), last_color_check) >= 2000:
            last_color_check = time.ticks_ms()
            ball = check_ball()
            if ball != "NONE":
                print("Ball in box: {}".format(ball))

        # 5. Forward Drive
        if not is_moving:
            motors.start_smoothly(FWD_LEFT_SPEED, FWD_RIGHT_SPEED)
            is_moving = True

        time.sleep(0.01)

except KeyboardInterrupt:
    motors.stop()
    pico_led.value(0)
    print("\nMission safely aborted.")
