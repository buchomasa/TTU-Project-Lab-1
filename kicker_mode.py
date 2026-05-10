# main.py
import time
import math
import config
from motors import MotorController
from encoders import WheelEncoders
from tof_sensor import BallDetector
from claw import Claw
from kick import kick
from linesensor import LineSensor
from colorsensor import RedBallSensor

def get_valid_distance(tof, timeout_ms=40):
    """Reads distance, handles errors gracefully. Allows slight negatives for point-blank captures."""
    timeout = time.ticks_add(time.ticks_ms(), timeout_ms)
    while not tof.data_ready and time.ticks_diff(timeout, time.ticks_ms()) > 0:
        time.sleep(0.002)
        
    if tof.data_ready:
        dist = tof.get_distance()
        tof.data_ready = False  
        if dist is not None and dist > -15:
            return dist
    return 9999  

def main():
    # =========================================================
    # TUNING VARIABLES (Adjust these during field testing)
    # =========================================================
    
    # Sensor Thresholds
    DETECT_THRESHOLD_MM = 200       # Max distance to register an object
    STOP_THRESHOLD_MM = 70          # Distance to trigger claw capture
    SLOWDOWN_THRESHOLD_MM = 120     # Distance to drop speed to prevent overshoot
    CAPTURE_SUCCESS_MM = 25         # Max distance allowed AFTER claw closes to confirm capture
    
    # False Trigger Validation
    FALSE_TRIGGER_MARGIN_MM = 30    # If distance grows by this much while approaching, abort.
    MAX_LOST_FRAMES = 4             # Number of consecutive "9999" reads before aborting approach.
    
    # Stagnation Progress
    STAGNATION_FRAMES = 5           # Frames to check for required forward progress
    REQUIRED_PROGRESS_MM = 15       # Minimum mm the distance must drop every STAGNATION_FRAMES
    
    # Initial Positioning Sequence
    START_FWD_1_CM = 50.0           # First forward drive distance
    START_TURN_DEG = 90.0           # Turn angle 
    START_TURN_DIR = "right"        # Direction for initial turn ("left" or "right")
    START_FWD_2_CM = 40.0           # Second forward drive distance
    
    # Sweep & Boundary Navigation
    SWEEP_ARC_HALF = 120            # 120 deg Left, 120 deg Right (240 deg total)
    REPOSITION_DIST_CM = 10.0       # Distance to move if sector is clear
    BOUNDARY_TURN_PULSES = 30.0     # Pulses for 90-degree turns during Phase 4 boundary navigation
    BOUNDARY_REVERSE_CM = 20.0      # Distance to reverse off boundary before turning again
    
    # Dynamic Speeds
    SCAN_SPEED_FACTOR = 0.85        # 85% speed during scans
    SLOWDOWN_FACTOR = 0.70          # 70% speed cut during final approach
    SLOW_FWD_LEFT = config.FWD_LEFT_SPEED * SLOWDOWN_FACTOR
    SLOW_FWD_RIGHT = config.FWD_RIGHT_SPEED * SLOWDOWN_FACTOR
    
    # Timing & Nudging
    SETTLE_TIME_S = 0.3             # Mechanical settle before reversing directions
    CLAW_WAIT_TIME_S = 1            # Time for servos to close and ball to physically settle
    NUDGE_PULSES = 1                # 1 pulse = 3 degrees. Keeps micro-alignment gentle.
    
    # =========================================================

    print("=== Rover Navigation & Full Harvester Sequence ===")
    
    motors = MotorController()
    encoders = WheelEncoders()
    line_sensor = LineSensor()
    claw = Claw()
    color_sensor = RedBallSensor()
    
    tof = BallDetector()
    time.sleep(0.1)
    
    # --- VL53L1X HARDWARE FoV PATCH ---
    try:
        tof.tof.writeReg(0x0080, 0x77) 
        print("[HARDWARE] VL53L1X ROI shrunk to 8x8 (Optimized FoV mode engaged).")
    except Exception as e:
        print(f"[WARN] Failed to set ROI: {e}")

    wheel_circumference = math.pi * config.WHEEL_DIAMETER_CM
    cm_per_pulse = wheel_circumference / config.DISK_SLOTS

    # ---------------------------------------------------------
    # Safe Movement Functions
    # ---------------------------------------------------------
    def turn_by_pulses(target_pulses, direction, speed_factor=1.0):
        if target_pulses <= 0: return 0
        encoders.reset()
        
        spd_l = config.TURN_LEFT_SPEED * speed_factor
        spd_r = config.TURN_RIGHT_SPEED * speed_factor
        
        motors.stop()
        time.sleep(SETTLE_TIME_S)
        
        # Uses the updated direction commands from your MotorController
        if direction == "left": motors.turn_left()
        else: motors.turn_right()
            
        for step in range(1, 11): 
            motors.set_speeds((spd_l * step) / 10, (spd_r * step) / 10)
            time.sleep(0.02)
        
        while True:
            l, r = encoders.get_pulses()
            if ((l + r) / 2.0) >= target_pulses:
                break
            if motors.check_faults(): 
                if direction == "left": motors.turn_left()
                else: motors.turn_right()
                motors.set_speeds(spd_l, spd_r)
            time.sleep(0.01)
            
        motors.stop()
        time.sleep(SETTLE_TIME_S)
        
        l_final, r_final = encoders.get_pulses()
        return (l_final + r_final) / 2.0

    def turn_by_degrees(degrees, direction, speed_factor=1.0):
        """Turns by degrees using empirical ratio (30 pulses = 90 deg)."""
        if degrees <= 0: return 0
        pulses = degrees * (30.0 / 90.0) 
        return turn_by_pulses(pulses, direction, speed_factor)

    def drive_forward(distance_cm):
        if distance_cm <= 0: return
        target_pulses = distance_cm / cm_per_pulse
        encoders.reset()
        
        motors.forward()
        motors.set_speeds(0, 0)
        if not motors.start_smoothly(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED):
            motors.set_speeds(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)

        while True:
            l, r = encoders.get_pulses()
            if ((l + r) / 2.0) >= target_pulses:
                break
            if motors.check_faults():
                motors.forward()
                motors.start_smoothly(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)
            time.sleep(0.01)
            
        motors.stop()
        time.sleep(SETTLE_TIME_S)

    def drive_backward(distance_cm):
        if distance_cm <= 0: return
        target_pulses = distance_cm / cm_per_pulse
        encoders.reset()
        
        motors.reverse()
        motors.set_speeds(0, 0)
        if not motors.start_smoothly_reverse(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED):
            motors.set_speeds(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)

        while True:
            l, r = encoders.get_pulses()
            if ((l + r) / 2.0) >= target_pulses:
                break
            if motors.check_faults():
                motors.reverse()
                motors.start_smoothly_reverse(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)
            time.sleep(0.01)
            
        motors.stop()
        time.sleep(SETTLE_TIME_S)
        
    def drive_to_boundary_and_align(timeout_ms=15000):
        left_on_line = False
        right_on_line = False
        start_time = time.ticks_ms()
        
        motors.forward()
        motors.set_speeds(0, 0)
        
        if not motors.start_smoothly(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED):
            motors.set_speeds(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)
            
        while not (left_on_line and right_on_line):
            if time.ticks_diff(time.ticks_ms(), start_time) > timeout_ms:
                print("\n[!] Boundary alignment timeout! Forcing stop.")
                break

            if motors.check_faults():
                motors.forward()
                if left_on_line and not right_on_line:
                    motors.set_speeds(0, config.FWD_RIGHT_SPEED)
                elif right_on_line and not left_on_line:
                    motors.set_speeds(config.FWD_LEFT_SPEED, 0)
                else:
                    motors.start_smoothly(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)

            try:
                l_hit, r_hit = line_sensor.boundary_hits()
                
                if l_hit and not left_on_line:
                    left_on_line = True
                    motors.forward()
                    motors.set_speeds(0, config.FWD_RIGHT_SPEED)
                    
                if r_hit and not right_on_line:
                    right_on_line = True
                    motors.forward()
                    motors.set_speeds(config.FWD_LEFT_SPEED, 0)
                    
            except Exception:
                pass
                
            time.sleep(0.01)
            
        motors.stop()
        time.sleep(SETTLE_TIME_S)

    def sweep_until_found(target_pulses, direction, tof_sensor):
        print(f"[SCAN] Sweeping {direction.upper()} for targets...")
        encoders.reset()
        
        spd_l = config.TURN_LEFT_SPEED * SCAN_SPEED_FACTOR
        spd_r = config.TURN_RIGHT_SPEED * SCAN_SPEED_FACTOR
        
        motors.stop()
        time.sleep(SETTLE_TIME_S)
        
        if direction == "left": motors.turn_left()
        else: motors.turn_right()
            
        for step in range(1, 11): 
            motors.set_speeds((spd_l * step) / 10, (spd_r * step) / 10)
            time.sleep(0.02)

        found_dist = 9999
        pulses_at_detection = 0
        target_found = False

        while True:
            l, r = encoders.get_pulses()
            avg_pulses = (l + r) / 2.0
            
            if avg_pulses >= target_pulses:
                break
                
            dist = get_valid_distance(tof_sensor, timeout_ms=35)
            
            if dist != 9999 and dist <= DETECT_THRESHOLD_MM:
                found_dist = dist
                pulses_at_detection = avg_pulses
                target_found = True
                break
                
            if motors.check_faults():
                if direction == "left": motors.turn_left()
                else: motors.turn_right()
                motors.set_speeds(spd_l, spd_r)
            time.sleep(0.005)
            
        motors.stop()
        time.sleep(SETTLE_TIME_S)
        
        l_final, r_final = encoders.get_pulses()
        total_swept_pulses = (l_final + r_final) / 2.0
        
        opp_dir = "right" if direction == "left" else "left"
        
        if target_found:
            print(f"[DETECT] Target spotted at {found_dist}mm! Halting sweep.")
            coasted = total_swept_pulses - pulses_at_detection
            if coasted > 0:
                print(f"[ALIGN] Unwinding {coasted:.1f} pulses of coasting drift...")
                turn_by_pulses(coasted, opp_dir, speed_factor=SCAN_SPEED_FACTOR)
            return True
        else:
            turn_by_pulses(total_swept_pulses, opp_dir, speed_factor=SCAN_SPEED_FACTOR)
            return False


    try:
        print("\n--- INITIAL: Positioning ---")
        
        # Executes the requested sequence: Forward -> Turn -> Forward
        if START_FWD_1_CM > 0:
            print(f"[INIT] Step 1: Driving forward {START_FWD_1_CM} cm...")
            drive_forward(START_FWD_1_CM)
            
        if START_TURN_DEG > 0:
            print(f"[INIT] Step 2: Turning {START_TURN_DIR.upper()} by {START_TURN_DEG} degrees...")
            turn_by_degrees(START_TURN_DEG, START_TURN_DIR)
            
        if START_FWD_2_CM > 0:
            print(f"[INIT] Step 3: Driving forward {START_FWD_2_CM} cm...")
            drive_forward(START_FWD_2_CM)

        while True:
            # =========================================================
            # PHASE 1: Greedy "First-Found" Scan
            # =========================================================
            print("\n--- PHASE 1: Greedy Radar Scan ---")
            half_arc_pulses = SWEEP_ARC_HALF * (30.0 / 90.0)
            
            # 0. Check Dead Center
            center_dist = get_valid_distance(tof, timeout_ms=50)
            if center_dist != 9999 and center_dist <= DETECT_THRESHOLD_MM:
                print(f"[PASS 1] Target located dead ahead at {center_dist}mm.")
            else:
                # 1. Sweep Left
                found = sweep_until_found(half_arc_pulses, "left", tof)
                
                if not found:
                    # 2. Sweep Right (Only if Left found nothing)
                    found = sweep_until_found(half_arc_pulses, "right", tof)
                    
                    if not found:
                        print(f"[SCAN] Sector completely clear. Repositioning...")
                        drive_forward(REPOSITION_DIST_CM)
                        continue 


            # =========================================================
            # PHASE 2: Micro-Alignment
            # =========================================================
            print("\n--- PHASE 2: Micro-Alignment ---")
            
            dist_center = get_valid_distance(tof, timeout_ms=50)
            
            print(f"[ALIGN] Nudging left by {NUDGE_PULSES} pulses...")
            actual_left_p = turn_by_pulses(NUDGE_PULSES, "left")
            dist_left = get_valid_distance(tof, timeout_ms=50)
            
            print(f"[ALIGN] Nudging right by {NUDGE_PULSES * 2} pulses...")
            actual_right_p = turn_by_pulses(NUDGE_PULSES * 2, "right")
            dist_right = get_valid_distance(tof, timeout_ms=50)
            
            if dist_left < dist_right and dist_left < dist_center:
                print(f"[ALIGN] Left was clearer ({dist_left} mm). Correcting...")
                turn_by_pulses(actual_right_p, "left")
            elif dist_center <= dist_left and dist_center <= dist_right:
                print(f"[ALIGN] Center was best ({dist_center} mm). Correcting...")
                correction = actual_right_p - actual_left_p
                if correction > 0: turn_by_pulses(correction, "left")
                elif correction < 0: turn_by_pulses(abs(correction), "right")
            else:
                print(f"[ALIGN] Right was clearer ({dist_right} mm). Holding heading.")
            
            time.sleep(SETTLE_TIME_S)


            # =========================================================
            # PHASE 3: Approach & Active False-Trigger Checking
            # =========================================================
            print("\n--- PHASE 3: Verified Approach & Capture ---")
            
            # --- PRE-FLIGHT SANITY CHECK ---
            target_still_there = False
            verify_dist = 9999
            for _ in range(3):
                verify_dist = get_valid_distance(tof, timeout_ms=40)
                if verify_dist != 9999 and verify_dist <= (DETECT_THRESHOLD_MM + 50):
                    target_still_there = True
                    break
                time.sleep(0.01)

            if not target_still_there:
                print(f"[FALSE TRIGGER] Target vanished after alignment (Distance jumped to {verify_dist}mm).")
                print("[!] Aborting approach. Renewing scan from Phase 1...")
                time.sleep(0.5)
                continue 

            is_slow_mode = False
            false_trigger_detected = False
            
            lowest_seen_dist = 9999
            lost_frame_count = 0
            progress_ref_dist = None
            stagnation_counter = 0
            
            motors.forward()
            if not motors.start_smoothly(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED):
                motors.set_speeds(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)
            
            while True:
                dist = get_valid_distance(tof, timeout_ms=40)
                
                # --- FALSE TRIGGER VALIDATION LOGIC ---
                if dist != 9999:
                    lost_frame_count = 0
                    print(f"[TELEMETRY] Distance to target: {dist} mm")
                    
                    if progress_ref_dist is None:
                        progress_ref_dist = dist
                    
                    if dist < lowest_seen_dist:
                        lowest_seen_dist = dist
                    elif dist > lowest_seen_dist + FALSE_TRIGGER_MARGIN_MM:
                        print(f"\n[FALSE TRIGGER] Distance grew from {lowest_seen_dist}mm to {dist}mm!")
                        false_trigger_detected = True
                        break
                        
                    stagnation_counter += 1
                    if stagnation_counter >= STAGNATION_FRAMES:
                        if lowest_seen_dist > progress_ref_dist - REQUIRED_PROGRESS_MM:
                            print(f"\n[FALSE TRIGGER] Stagnation! Distance hovering around {dist}mm. Not closing in.")
                            false_trigger_detected = True
                            break
                        else:
                            progress_ref_dist = lowest_seen_dist
                            stagnation_counter = 0
                        
                else:
                    lost_frame_count += 1
                    if lost_frame_count >= MAX_LOST_FRAMES:
                        print(f"\n[FALSE TRIGGER] Target lost completely (sensor returned 9999).")
                        false_trigger_detected = True
                        break

                # --- APPROACH LOGIC ---
                if dist != 9999 and dist <= SLOWDOWN_THRESHOLD_MM and not is_slow_mode:
                    print(f"[APPROACH] Entering final zone. Decelerating to {SLOWDOWN_FACTOR*100}%.")
                    motors.set_speeds(SLOW_FWD_LEFT, SLOW_FWD_RIGHT)
                    is_slow_mode = True
                    
                if dist != 9999 and dist <= STOP_THRESHOLD_MM:
                    motors.stop() 
                    print(f"\n[STOP] Target reached! Final distance: {dist} mm.")
                    time.sleep(0.1) 
                    
                    print("[CAPTURE] Triggering claw mechanism...")
                    claw.close()
                    time.sleep(CLAW_WAIT_TIME_S) 
                    
                    # --- POST-CAPTURE VERIFICATION ---
                    capture_dist = get_valid_distance(tof, timeout_ms=60)
                    
                    if capture_dist != 9999 and capture_dist <= CAPTURE_SUCCESS_MM:
                        print(f"[VERIFY] SUCCESS! Payload secured at {capture_dist}mm.")
                        
                        # =========================================================
                        # PHASE 4: Color Identification & Payload Sorting
                        # =========================================================
                        print("\n--- PHASE 4: Sorting & Navigation ---")
                        
                        ball_status = color_sensor.check_ball(debug=True)
                        is_red_ball = (ball_status == "RED")
                        
                        if is_red_ball:
                            print("[SORT] RED payload confirmed. Executing kick mechanism...")
                            claw.open()
                            time.sleep(0.5) 
                            kick() 
                            time.sleep(0.5)
                            print("[SORT] Payload ejected.")
                            
                        else:
                            print(f"[SORT] NON-RED payload confirmed ({ball_status}). Navigating to boundary...")
                            
                            nav_direction = "right"
                            
                            for step in range(1, 4):
                                print(f"\n[NAV] Step {step}: Turning {nav_direction.upper()}...")
                                turn_by_pulses(BOUNDARY_TURN_PULSES, nav_direction)
                                
                                print(f"[NAV] Step {step}: Driving forward to boundary line...")
                                drive_to_boundary_and_align()
                                
                                if step < 3:
                                    print(f"[NAV] Step {step}: Boundary aligned. Reversing {BOUNDARY_REVERSE_CM}cm...")
                                    drive_backward(BOUNDARY_REVERSE_CM)
                                    
                            print("\n[NAV] Final drop zone reached. Releasing payload...")
                            claw.open()
                            time.sleep(1.0)
                        
                        # --- End of Phase 4. Reset for the next ball ---
                        print("\n[RESET] Clearing drop zone to avoid recapture...")
                        drive_backward(15.0) 
                        turn_by_pulses(60.0, "left") 
                        print("[RESET] Ready for next target.")
                        
                    else:
                        print(f"[VERIFY] FAILED. Claw is empty (Reading: {capture_dist}mm). Ball escaped!")
                        claw.open()
                        time.sleep(0.3)
                        print("[RESET] Re-engaging search...")
                        drive_backward(5.0) 
                    
                    break # Restart Phase 1
                    
                if motors.check_faults():
                    motors.forward()
                    if is_slow_mode:
                        motors.set_speeds(SLOW_FWD_LEFT, SLOW_FWD_RIGHT)
                    else:
                        motors.start_smoothly(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)
                    
                time.sleep(0.01)
                
            if false_trigger_detected:
                motors.stop()
                print("[!] Aborting approach. Renewing scan from Phase 1...")
                time.sleep(0.5)
                continue 

    except KeyboardInterrupt:
        print("\n[!] Script manually interrupted. Stopping motors and opening claw.")
        motors.stop()
        try:
            claw.open()
        except NameError:
            pass
    except Exception as e:
        print(f"\n[!] Unexpected error: {e}")
        motors.stop()

if __name__ == "__main__":
    main()