"""
Red Raider Soccer - Full Hardware Diagnostic Sequence
Run with wheels suspended off the ground.
"""

import time

import config
from claw import Claw
from colorsensor import RedBallSensor
from encoders import WheelEncoders
from kick import kick
from linesensor import LineSensor
from motors import MotorController
from tof_sensor import BallDetector
from ultrasonics import FixedUltrasonicScanner, UltrasonicScanner


def print_header(title):
    print("\n" + "=" * 56)
    print(title)
    print("=" * 56)


def test_ultrasonics_and_servo(front, rear):
    print_header("TEST 1: FRONT SWEEP SERVO + FRONT/REAR ULTRASONICS")
    angles = [176, 128, 80]
    names = ["LEFT", "CENTER", "RIGHT"]

    for i, angle in enumerate(angles):
        front.set_servo_angle(angle)
        time.sleep_ms(500)
        f_dist = front.get_distance()
        r_dist = rear.get_distance()
        print(
            "Front {} ({} deg): front={} cm | rear={} cm".format(
                names[i], angle, f_dist, r_dist
            )
        )
        time.sleep_ms(600)

    front.set_servo_angle(128)
    print("[PASS] Ultrasonic + sweep servo test sequence complete.")


def test_line_sensors(lines):
    print_header("TEST 2: LINE SENSOR RAW + BLACK-BOUNDARY DETECTION")
    print("Move sensors over field and black tape while watching values.")
    if getattr(config, "LINE_USE_ADAPTIVE", False):
        print(
            "Adaptive mode ON: baseline_left={} baseline_right={} margin_left={} margin_right={}".format(
                lines.left_baseline,
                lines.right_baseline,
                config.LINE_LEFT_MARGIN,
                config.LINE_RIGHT_MARGIN,
            )
        )
    else:
        print(
            "Thresholds: left={} right={}".format(
                config.LINE_LEFT_BLACK_THRESHOLD, config.LINE_RIGHT_BLACK_THRESHOLD
            )
        )
    for _ in range(15):
        left_raw, right_raw = lines.get_raw_values()
        left_hit, right_hit = lines.boundary_hits()
        boundary = left_hit or right_hit
        print(
            "left_raw={} right_raw={} left_hit={} right_hit={} boundary_detected={}".format(
                left_raw, right_raw, left_hit, right_hit, boundary
            )
        )
        time.sleep_ms(250)
    print("[INFO] Tune LINE_LEFT_BLACK_THRESHOLD / LINE_RIGHT_BLACK_THRESHOLD if needed.")


def test_tof(detector):
    print_header("TEST 3: TOF RANGE + CAPTURE/SECURE FLAGS")
    print("Place ball at far, medium, close, and inside-box positions.")
    for _ in range(20):
        d = detector.get_distance()
        capture = detector.is_capture_range(d)
        secure = detector.is_secured_in_box(d)
        status = detector.get_ball_status(d)
        print(
            "tof_mm={} status={} capture={} secured={}".format(
                d, status, capture, secure
            )
        )
        time.sleep_ms(200)
    print("[INFO] Tune TOF_CAPTURE_MM / TOF_SECURED_MM / TOF_OFFSET_MM if needed.")


def test_color_sensor():
    print_header("TEST 4: COLOR SENSOR")
    try:
        color = RedBallSensor()
        for _ in range(10):
            result = color.check_ball(debug=True)
            print("Color result={}".format(result))
            time.sleep_ms(300)
        print("[PASS] Color sensor responded.")
    except Exception as e:
        print("[FAIL] Color sensor init/read failed: {}".format(e))


def test_drivetrain_and_encoders(motors, encoders):
    print_header("TEST 5: LEFT/RIGHT DRIVETRAIN + ENCODERS")

    # Left only
    encoders.reset()
    motors.forward()
    motors.set_speeds(75, 0)
    for _ in range(6):
        time.sleep_ms(300)
        l_p, r_p = encoders.get_pulses()
        print("Left spin ticks -> L:{} R:{}".format(l_p, r_p))
    motors.stop()
    l_final, r_final = encoders.get_pulses()
    if l_final > 5 and r_final == 0:
        print("[PASS] Left motor + left encoder OK.")
    else:
        print("[WARN] Left test unexpected ticks (check wiring/thresholds).")

    time.sleep_ms(700)

    # Right only
    encoders.reset()
    motors.forward()
    motors.set_speeds(0, 75)
    for _ in range(6):
        time.sleep_ms(300)
        l_p, r_p = encoders.get_pulses()
        print("Right spin ticks -> L:{} R:{}".format(l_p, r_p))
    motors.stop()
    l_final, r_final = encoders.get_pulses()
    if r_final > 5 and l_final == 0:
        print("[PASS] Right motor + right encoder OK.")
    else:
        print("[WARN] Right test unexpected ticks (check wiring/thresholds).")


def test_overcurrent_inputs(motors):
    print_header("TEST 6: OVERCURRENT INPUT STATUS")
    a = motors.oc_a.value()
    b = motors.oc_b.value()
    print("oc_a (pin {}) = {}".format(config.PIN_OC_A, a))
    print("oc_b (pin {}) = {}".format(config.PIN_OC_B, b))
    if a == 1 or b == 1:
        print("[WARN] At least one OC pin is HIGH now.")
    else:
        print("[PASS] OC pins resting LOW.")


def test_claw():
    print_header("TEST 7: CLAW OPEN/CLOSE")
    try:
        claw = Claw()
        print("Closing claw...")
        claw.close()
        time.sleep_ms(500)
        print("Opening claw...")
        claw.open()
        print("[PASS] Claw actuation completed.")
    except Exception as e:
        print("[FAIL] Claw test failed: {}".format(e))


def test_kicker():
    print_header("TEST 8: KICKER SOLENOID PULSE")
    print("Firing one kick pulse...")
    fired = kick()
    print("kick_result={}".format(fired))
    if fired:
        print("[PASS] Kicker pulse fired.")
    else:
        print("[WARN] Kick was cooldown-limited or skipped.")


def run_diagnostics():
    print_header("RED RAIDER FULL DIAGNOSTIC START")
    print("Keep wheels clear of the ground.")
    print(
        "Config: front_echo_pin={} tof_capture_mm={} line_mode={} thresholds(L/R)=({}/{})".format(
            config.PIN_FRONT_ECHO,
            config.TOF_CAPTURE_MM,
            "adaptive" if getattr(config, "LINE_USE_ADAPTIVE", False) else "fixed",
            config.LINE_LEFT_BLACK_THRESHOLD,
            config.LINE_RIGHT_BLACK_THRESHOLD,
        )
    )
    time.sleep_ms(1000)

    motors = MotorController()
    front = UltrasonicScanner()
    rear = FixedUltrasonicScanner(config.PIN_REAR_TRIG, config.PIN_REAR_ECHO)
    encoders = WheelEncoders()
    lines = LineSensor()
    detector = BallDetector()

    motors.stop()

    test_ultrasonics_and_servo(front, rear)
    test_line_sensors(lines)
    test_tof(detector)
    test_color_sensor()
    test_drivetrain_and_encoders(motors, encoders)
    test_overcurrent_inputs(motors)
    test_claw()
    test_kicker()

    motors.stop()
    front.pause_sweep()
    print_header("DIAGNOSTICS COMPLETE")


if __name__ == "__main__":
    try:
        run_diagnostics()
    except KeyboardInterrupt:
        try:
            MotorController().stop()
        except Exception:
            pass
        print("\nDiagnostics aborted by user.")
