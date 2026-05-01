from machine import Pin
import time

# System Components
from motors import MotorController
from ultrasonics import UltrasonicScanner, FixedUltrasonicScanner
from encoders import WheelEncoders
from avoidance import ObstacleAvoidance
from tof_sensor import BallDetector
from linesensor import LineSensor

# Initialize the sensor object
sensors = LineSensor(black_pin_id=27, blue_pin_id=26)

print("Starting Robot Main Loop...")

try:
    while True:
        # Get raw data for debugging
        b_raw, u_raw = sensors.get_raw_values()
        
        # Check logic
        goalie_stop = sensors.should_goalie_stop()
        kicker_stop = sensors.should_kicker_stop()

        # Determine actions
        goalie_status = "STOP" if goalie_stop else "FORWARD"
        kicker_status = "STOP" if kicker_stop else "FORWARD"

        print(f"Sensors: B={b_raw} U={u_raw} | Goalie: {goalie_status} | Kicker: {kicker_status}")

        time.sleep(0.1) # Faster polling for real-world use

except KeyboardInterrupt:
    print("Program stopped.")

# Initialize the sensor
detector = BallDetector(i2c_id=0, sda_pin=0, scl_pin=1, int_pin_id=19)

print("ToF ball detector ready (Class-based)\n")

while True:
    # Check the flag managed by the class
    if detector.data_ready:
        dist = detector.get_distance()
        detector.data_ready = False # Reset flag after reading
        
        status = detector.get_ball_status(dist)
        
        if status == 'IN_RANGE':
            print(f"BALL IN RANGE  -> {dist} mm  - ACTION: KICK")
        elif status == 'APPROACH':
            print(f"BALL AHEAD     -> {dist} mm  - ACTION: DRIVE")
        else:
            print(f"Scanning... ({dist} mm)")

    # Brief sleep to yield to the processor
    time.sleep_ms(10)
# ================================================================
# 1. FIELD TUNING & CALIBRATION VARIABLES
# ================================================================

OBSTACLE_THRESHOLD_CM = 20.0  
REAR_THRESHOLD_CM = 20.0      

FWD_LEFT_SPEED = 51.2
FWD_RIGHT_SPEED = 60.0
TURN_LEFT_SPEED = 68.3
TURN_RIGHT_SPEED = 80.0

TRACK_WIDTH_CM = 20.0
WHEEL_DIAMETER_CM = 6.0
DISK_SLOTS = 20

# Packaged for clean delivery to the avoidance class
ROVER_CONFIG = {
    'obs_thresh': OBSTACLE_THRESHOLD_CM,
    'rear_thresh': REAR_THRESHOLD_CM,
    'fwd_speeds': (FWD_LEFT_SPEED, FWD_RIGHT_SPEED),
    'trn_speeds': (TURN_LEFT_SPEED, TURN_RIGHT_SPEED),
    'track_width': TRACK_WIDTH_CM,
    'wheel_dia': WHEEL_DIAMETER_CM,
    'disk_slots': DISK_SLOTS
}

# ================================================================
# 2. HARDWARE INITIALIZATION
# ================================================================
pico_led = Pin("LED", Pin.OUT)
pico_led.value(1)

motors = MotorController()
scanner_front = UltrasonicScanner(trig_pin=12, echo_pin=13, servo_pin=10)
scanner_rear = FixedUltrasonicScanner(trig_pin=17, echo_pin=18)   
encoders = WheelEncoders()

# Initialize High-Level Navigation Logic
avoider = ObstacleAvoidance(motors, scanner_front, scanner_rear, encoders, ROVER_CONFIG)

# ================================================================
# 3. CONTINUOUS DRIVE LOOP
# ================================================================
print("SYS_OK: Beginning Continuous Drive Protocol.")

is_moving = False
scanner_front.start_sweep()

try:
    while True:
        # Check L298N for physical stalls / voltage dips
        if motors.check_faults():
            is_moving = False 
            continue 

        distance = scanner_front.get_distance()
        current_look_angle = scanner_front.current_angle

        # If path is blocked, halt and delegate to Avoidance protocol
        if distance < OBSTACLE_THRESHOLD_CM:
            if is_moving:
                motors.stop()
                is_moving = False
            
            avoider.navigate_obstacle(distance, current_look_angle)

            # Look-Before-Leaping Validation
            scanner_front.pause_sweep()
            scanner_front.set_servo_angle(128) 
            time.sleep(0.2) 
            
            if scanner_front.get_distance() <= OBSTACLE_THRESHOLD_CM:
                print("PATH BLOCKED: Holding position.")
                
            scanner_front.start_sweep()

        # If path is clear, engage forward drive smoothly
        elif not is_moving:
            if motors.start_smoothly(FWD_LEFT_SPEED, FWD_RIGHT_SPEED):
                print("PATH CLEAR: Engaging forward drive.")
                is_moving = True
                avoider.reset_trap_counter() # Successful progression clears the wiggle trap memory

        time.sleep(0.02) 

except KeyboardInterrupt:
    motors.stop()
    scanner_front.pause_sweep()
    pico_led.value(0)
    print("\nSYS_HALT: Mission safely aborted by user.")