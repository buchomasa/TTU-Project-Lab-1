import time
import math
import config
from motors import MotorController
from encoders import WheelEncoders
from linesensor import LineSensor

# ==========================================
# TUNING VARIABLES - Adjust these as needed
# ==========================================

# Turns if the user selects LEFT (1)
TURN_1_LEFT_PULSES = 30.0
TURN_2_LEFT_PULSES = 30.0
TURN_3_LEFT_PULSES = 30.0

# Turns if the user selects RIGHT (2)
TURN_1_RIGHT_PULSES = 30.0
TURN_2_RIGHT_PULSES = 30.0
TURN_3_RIGHT_PULSES = 30.0

# Reverse distance in cm
REVERSE_DISTANCE_CM = 20.0

# Time to let motors rest between sudden direction changes
SETTLE_TIME_MS = 500 

# ==========================================

def turn_by_pulses(motors, encoders, direction, target_pulses):
    """Turns the rover by a specified number of encoder pulses."""
    encoders.reset()
    
    # Helper to re-apply speeds in case of an overcurrent trip
    def apply_movement():
        if direction == 1:
            motors.turn_left()
        else:
            motors.turn_right()
        motors.set_speeds(config.TURN_LEFT_SPEED, config.TURN_RIGHT_SPEED)

    apply_movement()

    while True:
        # Check for hardware faults. If it tripped, the board stopped the motors for 500ms.
        # We instantly re-apply the movement after the safety pause so we don't miss encoder ticks.
        if motors.check_faults():
            apply_movement()

        left_p, right_p = encoders.get_pulses()
        avg_pulses = (left_p + right_p) / 2.0
        if avg_pulses >= target_pulses:
            break
        time.sleep_ms(10)
        
    motors.stop()
    time.sleep_ms(SETTLE_TIME_MS)

def reverse_by_distance_cm(motors, encoders, distance_cm):
    """Reverses the rover a specific distance in cm using wheel odometry."""
    circumference = math.pi * config.WHEEL_DIAMETER_CM
    target_pulses = (distance_cm / circumference) * config.DISK_SLOTS
    
    encoders.reset()
    
    def apply_movement():
        motors.reverse()
        motors.set_speeds(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)
        
    apply_movement()
    
    while True:
        if motors.check_faults():
            apply_movement()

        left_p, right_p = encoders.get_pulses()
        avg_pulses = (left_p + right_p) / 2.0
        if avg_pulses >= target_pulses:
            break
        time.sleep_ms(10)
        
    motors.stop()
    time.sleep_ms(SETTLE_TIME_MS)

def drive_to_boundary_and_align(motors, line_sensor):
    """
    Moves forward until a boundary is detected. If one sensor hits first, 
    it stops that motor and drives the other to square up on the line.
    """
    left_on_line = False
    right_on_line = False
    
    # Dynamic helper that calculates correct motor states based on which sensors are on the line
    def apply_movement():
        motors.forward()
        if left_on_line and not right_on_line:
            motors.set_speeds(0, config.FWD_RIGHT_SPEED)
        elif right_on_line and not left_on_line:
            motors.set_speeds(config.FWD_LEFT_SPEED, 0)
        else:
            motors.set_speeds(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)

    apply_movement()
    
    # Keep moving until BOTH sensors see black
    while not (left_on_line and right_on_line):
        if motors.check_faults():
            # If a fault occurs, it will resume exactly where it left off, 
            # keeping wheels that are already on the boundary locked in place.
            apply_movement()

        l_hit, r_hit = line_sensor.boundary_hits()
        
        # Lock in the detection and update motor states immediately
        if l_hit and not left_on_line:
            left_on_line = True
            apply_movement()
            
        if r_hit and not right_on_line:
            right_on_line = True
            apply_movement()
            
        time.sleep_ms(10)
        
    motors.stop()
    time.sleep_ms(SETTLE_TIME_MS)

def main():
    motors = MotorController()
    encoders = WheelEncoders()
    line_sensor = LineSensor()

    # 1. Prompt user for direction
    while True:
        try:
            user_input = int(input("Enter 1 for LEFT or 2 for RIGHT: "))
            if user_input in [1, 2]:
                break
            print("Invalid input. Please enter 1 or 2.")
        except ValueError:
            print("Invalid input. Please enter a valid number.")

    direction_str = "LEFT" if user_input == 1 else "RIGHT"

    # Map the correct turn variables based on user selection
    if user_input == 1:
        turn_1_target = TURN_1_LEFT_PULSES
        turn_2_target = TURN_2_LEFT_PULSES
        turn_3_target = TURN_3_LEFT_PULSES
    else:
        turn_1_target = TURN_1_RIGHT_PULSES
        turn_2_target = TURN_2_RIGHT_PULSES
        turn_3_target = TURN_3_RIGHT_PULSES

    print(f"\n--- Starting {direction_str} Sequence ---")

    # --- STEP 1 ---
    print(f"[Step 1] First Turn ({turn_1_target} pulses)...")
    turn_by_pulses(motors, encoders, user_input, turn_1_target)
    
    print("[Step 1] Moving to boundary and aligning...")
    drive_to_boundary_and_align(motors, line_sensor)
    
    print(f"[Step 1] Reversing {REVERSE_DISTANCE_CM} cm...")
    reverse_by_distance_cm(motors, encoders, REVERSE_DISTANCE_CM)

    # --- STEP 2 ---
    print(f"\n[Step 2] Second Turn ({turn_2_target} pulses)...")
    turn_by_pulses(motors, encoders, user_input, turn_2_target)
    
    print("[Step 2] Moving to boundary and aligning...")
    drive_to_boundary_and_align(motors, line_sensor)
    
    print(f"[Step 2] Reversing {REVERSE_DISTANCE_CM} cm...")
    reverse_by_distance_cm(motors, encoders, REVERSE_DISTANCE_CM)

    # --- STEP 3 ---
    print(f"\n[Step 3] Third Turn ({turn_3_target} pulses)...")
    turn_by_pulses(motors, encoders, user_input, turn_3_target)
    
    print("[Step 3] Moving to final boundary and aligning...")
    drive_to_boundary_and_align(motors, line_sensor)

    print("\n[!] Sequence Complete. Stopping.")
    motors.stop()

if __name__ == "__main__":
    print("Starting sequence in 2 seconds...")
    time.sleep(2)
    main()