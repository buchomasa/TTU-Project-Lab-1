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
from avoidance import ObstacleAvoidance
from ultrasonics import UltrasonicScanner, FixedUltrasonicScanner

# --- Global Tracking for Obstacle Consistency ---
obstacle_consistency_count = 0
CONSISTENCY_THRESHOLD = 3  # Must see obstacle 3 times in a row

def get_valid_distance(tof, timeout_ms=40):
    """Reads distance from ToF, handling errors gracefully."""
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
    global obstacle_consistency_count
    
    # =========================================================
    # TUNING VARIABLES
    # =========================================================
    DETECT_THRESHOLD_MM = 200       # Max distance to register an object
    STOP_THRESHOLD_MM = 70          # Distance to trigger claw capture
    SLOWDOWN_THRESHOLD_MM = 120     # Distance to drop speed
    CAPTURE_SUCCESS_MM = 25         # Verify ball is in claw
    FALSE_TRIGGER_MARGIN_MM = 30    
    MAX_LOST_FRAMES = 4             
    
    # Initial Positioning (FORWARD ONLY)
    START_FWD_DISTANCE_CM = 50.0    # Adjust as needed
    
    # Navigation & Speeds
    SWEEP_ARC_HALF = 120            
    REPOSITION_DIST_CM = 20.0       
    BOUNDARY_TURN_PULSES = 30.0     
    SCAN_SPEED_FACTOR = 0.85        
    SLOWDOWN_FACTOR = 0.70          
    SETTLE_TIME_S = 0.3             
    CLAW_WAIT_TIME_S = 1            
    NUDGE_PULSES = 1                
    # =========================================================

    # --- Hardware Initialization ---
    motors = MotorController()
    encoders = WheelEncoders()
    line_sensor = LineSensor()
    claw = Claw()
    color_sensor = RedBallSensor()
    tof = BallDetector()
    front_scanner = UltrasonicScanner()
    rear_scanner = FixedUltrasonicScanner(config.PIN_REAR_TRIG, config.PIN_REAR_ECHO)
    
    # --- Avoidance Setup ---
    avoidance_config = {
        'obs_thresh': config.OBSTACLE_THRESHOLD_CM,
        'rear_thresh': config.REAR_THRESHOLD_CM,
        'fwd_speeds': (config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED),
        'trn_speeds': (config.TURN_LEFT_SPEED, config.TURN_RIGHT_SPEED),
        'track_width': config.TRACK_WIDTH_CM,
        'wheel_dia': config.WHEEL_DIAMETER_CM,
        'disk_slots': config.DISK_SLOTS
    }
    obs_avoider = ObstacleAvoidance(motors, front_scanner, rear_scanner, encoders, avoidance_config)
    front_scanner.start_sweep()
    
    cm_per_pulse = (math.pi * config.WHEEL_DIAMETER_CM) / config.DISK_SLOTS

    # --- HELPER: Dual-Sensor Obstacle Check (Solutions 1 & 2) ---
    def check_for_obstacles():
        global obstacle_consistency_count
        tof_dist = get_valid_distance(tof, timeout_ms=30)
        
        if tof_dist != 9999 and tof_dist <= DETECT_THRESHOLD_MM:
            ultra_dist = front_scanner.get_distance()
            
            # Trigger only if Ultrasonic < 10cm (prevents ball triggers)
            if ultra_dist != 999.0 and ultra_dist <= 10.0:
                obstacle_consistency_count += 1
            else:
                obstacle_consistency_count = 0
            
            # Consistency check to filter ghost readings
            if obstacle_consistency_count >= CONSISTENCY_THRESHOLD:
                print(f"\n[OBSTACLE] Confirmed: ToF {tof_dist}mm, Ultra {ultra_dist}cm")
                motors.stop()
                obs_avoider.navigate_obstacle(ultra_dist, front_scanner.current_angle)
                obstacle_consistency_count = 0 
                return True
        else:
            obstacle_consistency_count = 0 
        return False

    # --- MOVEMENT FUNCTIONS ---
    def turn_by_pulses(target_pulses, direction, speed_factor=1.0, check_obs=False):
        if target_pulses <= 0: return 0
        encoders.reset()
        motors.stop(); time.sleep(SETTLE_TIME_S)
        if direction == "left": motors.turn_left()
        else: motors.turn_right()
        motors.set_speeds(config.TURN_LEFT_SPEED * speed_factor, config.TURN_RIGHT_SPEED * speed_factor)
        while True:
            if check_obs and check_for_obstacles(): return "ABORTED"
            l, r = encoders.get_pulses()
            if ((l + r) / 2.0) >= target_pulses: break
            time.sleep(0.01)
        motors.stop(); time.sleep(SETTLE_TIME_S)
        return (encoders.get_pulses()[0] + encoders.get_pulses()[1]) / 2.0

    def drive_forward(distance_cm, check_obs=False):
        if distance_cm <= 0: return True
        target_pulses = distance_cm / cm_per_pulse
        encoders.reset()
        motors.forward()
        motors.set_speeds(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)
        while True:
            if check_obs and check_for_obstacles(): return False
            l, r = encoders.get_pulses()
            if ((l + r) / 2.0) >= target_pulses: break
            time.sleep(0.01)
        motors.stop(); time.sleep(SETTLE_TIME_S)
        return True

    def drive_to_boundary_and_align(check_obs=False):
        l_hit, r_hit = False, False
        motors.forward(); motors.set_speeds(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)
        while not (l_hit and r_hit):
            if check_obs and check_for_obstacles(): return False
            try:
                lh, rh = line_sensor.boundary_hits()
                if lh and not l_hit: l_hit = True; motors.set_speeds(0, config.FWD_RIGHT_SPEED)
                if rh and not r_hit: r_hit = True; motors.set_speeds(config.FWD_LEFT_SPEED, 0)
            except: pass
            time.sleep(0.01)
        motors.stop(); time.sleep(SETTLE_TIME_S)
        return True

    # --- MAIN EXECUTION ---
    try:
        print("=== Rover Navigation & Full Harvester Sequence ===")
        
        # INITIAL: Positioning (Forward Only)
        print("\n--- INITIAL: Positioning ---")
        drive_forward(START_FWD_DISTANCE_CM, check_obs=True)

        while True:
            # PHASE 1: Scan
            print("\n--- PHASE 1: Greedy Radar Scan ---")
            ball_spotted = False
            half_arc_pulses = SWEEP_ARC_HALF * (30.0 / 90.0)
            
            if check_for_obstacles(): continue 
            
            def run_sweep(pulses, direction):
                nonlocal ball_spotted
                encoders.reset()
                if direction == "left": motors.turn_left()
                else: motors.turn_right()
                motors.set_speeds(config.TURN_LEFT_SPEED*SCAN_SPEED_FACTOR, config.TURN_RIGHT_SPEED*SCAN_SPEED_FACTOR)
                while True:
                    if check_for_obstacles(): return "OBSTACLE"
                    l, r = encoders.get_pulses()
                    if (l+r)/2 >= pulses: break
                    d = get_valid_distance(tof, 35)
                    if d <= DETECT_THRESHOLD_MM:
                        ball_spotted = True; motors.stop(); break
                    time.sleep(0.01)
                return "DONE"

            if run_sweep(half_arc_pulses, "left") == "OBSTACLE": continue
            if not ball_spotted:
                if run_sweep(half_arc_pulses*2, "right") == "OBSTACLE": continue
                if not ball_spotted:
                    drive_forward(REPOSITION_DIST_CM, check_obs=True)
                    continue

            # PHASE 2: Micro-Alignment
            print("\n--- PHASE 2: Micro-Alignment ---")
            dist_center = get_valid_distance(tof, 50)
            turn_by_pulses(NUDGE_PULSES, "left")
            dist_left = get_valid_distance(tof, 50)
            turn_by_pulses(NUDGE_PULSES * 2, "right")
            dist_right = get_valid_distance(tof, 50)
            
            if dist_left < dist_right and dist_left < dist_center:
                turn_by_pulses(NUDGE_PULSES * 2, "left")
            elif dist_center <= dist_left and dist_center <= dist_right:
                turn_by_pulses(NUDGE_PULSES, "left")
            
            # PHASE 3: Approach & Capture
            print("\n--- PHASE 3: Verified Approach & Capture ---")
            motors.forward(); motors.set_speeds(config.FWD_LEFT_SPEED, config.FWD_RIGHT_SPEED)
            is_slow = False; captured = False; lowest_d = 9999; lost_f = 0
            
            while True:
                d = get_valid_distance(tof, 40)
                if d == 9999:
                    lost_f += 1
                    if lost_f >= MAX_LOST_FRAMES: break
                    continue
                lost_f = 0
                if check_for_obstacles(): break

                if d < lowest_d: lowest_d = d
                elif d > lowest_d + FALSE_TRIGGER_MARGIN_MM: break

                if d <= SLOWDOWN_THRESHOLD_MM and not is_slow:
                    motors.set_speeds(config.FWD_LEFT_SPEED*SLOWDOWN_FACTOR, config.FWD_RIGHT_SPEED*SLOWDOWN_FACTOR)
                    is_slow = True
                
                if d <= STOP_THRESHOLD_MM:
                    motors.stop(); claw.close(); time.sleep(CLAW_WAIT_TIME_S)
                    if get_valid_distance(tof, 60) <= CAPTURE_SUCCESS_MM:
                        captured = True
                    break
                time.sleep(0.01)

            # PHASE 4: Sorting (Gated by 'captured')
            if captured:
                print("\n--- PHASE 4: Sorting & Navigation ---")
                status = color_sensor.check_ball(debug=True)
                if status == "RED":
                    claw.open(); time.sleep(0.5); kick()
                else:
                    for _ in range(3):
                        if turn_by_pulses(BOUNDARY_TURN_PULSES, "right", check_obs=True) == "ABORTED": break
                        drive_to_boundary_and_align(check_obs=True)
                        motors.reverse(); time.sleep(0.5); motors.stop()
                    claw.open()
                
                motors.reverse(); time.sleep(1.0); motors.stop()
                turn_by_pulses(60, "left", check_obs=True)
            else:
                print("[SKIP] Capture failed. Returning to Phase 1.")
                claw.open()

    except KeyboardInterrupt:
        motors.stop(); claw.open(); front_scanner.pause_sweep()
        print("\n[!] Script Interrupted.")
    except Exception as e:
        print(f"\n[!] Unexpected error: {e}")
        motors.stop()

if __name__ == "__main__":
    main()