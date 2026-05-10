import time
import config
from motors import MotorController
from encoders import WheelEncoders
from tof_sensor import BallDetector
from claw import Claw
from kick import kick
from linesensor import LineSensor
from colorsensor import RedBallSensor
from navigation import Navigator

def main():
    print("[SYS] Initializing Rover Kicker Mode...")
    
    motors = MotorController()
    encoders = WheelEncoders()
    line_sensor = LineSensor()
    claw = Claw()
    color_sensor = RedBallSensor() 
    tof = BallDetector()
    
    nav = Navigator(motors, encoders, line_sensor)
    time.sleep(0.1)
    
    try:
        tof.tof.writeReg(0x0080, 0x77) 
        print("[SYS] VL53L1X ROI optimized (8x8).")
    except Exception as e:
        print(f"[WARN] Failed to set ROI: {e}")

    try:
        if config.START_FORWARD_DIST_CM > 0:
            print(f"[OP] Initializing positioning drive: {config.START_FORWARD_DIST_CM}cm")
            nav.drive_forward(config.START_FORWARD_DIST_CM)

        while True:
            # =========================================================
            # PHASE 1: Scan & Detect
            # =========================================================
            print("[OP] Phase 1: Commencing Sector Scan")
            half_arc_pulses = config.SWEEP_ARC_HALF * (config.BOUNDARY_TURN_PULSES / 90.0) 
            target_direction = "center" 
            
            center_dist = nav.get_valid_distance(tof, timeout_ms=50)
            if center_dist != 9999 and center_dist <= config.DETECT_THRESHOLD_MM:
                print(f"[OP] Target acquired at center: {center_dist}mm")
                target_direction = "center"
            else:
                found, dist = nav.sweep_until_found(half_arc_pulses, "left", tof)
                if found: 
                    target_direction = "left"
                    print(f"[OP] Target acquired at left: {dist}mm")
                else:
                    found, dist = nav.sweep_until_found(half_arc_pulses * 2, "right", tof)
                    if found: 
                        target_direction = "right"
                        print(f"[OP] Target acquired at right: {dist}mm")
                    else:
                        print("[OP] Sector clear. Repositioning.")
                        nav.turn_by_pulses(half_arc_pulses, "left") 
                        nav.drive_forward(config.REPOSITION_DIST_CM)
                        continue 

            # =========================================================
            # PHASE 2: Micro-Alignment
            # =========================================================
            print("[OP] Phase 2: Micro-Alignment")
            dist_center = nav.get_valid_distance(tof, timeout_ms=50)
            actual_left_p = nav.turn_by_pulses(config.NUDGE_PULSES, "left")
            dist_left = nav.get_valid_distance(tof, timeout_ms=50)
            actual_right_p = nav.turn_by_pulses(config.NUDGE_PULSES * 2, "right")
            dist_right = nav.get_valid_distance(tof, timeout_ms=50)
            
            if dist_left < dist_right and dist_left < dist_center:
                nav.turn_by_pulses(actual_right_p, "left")
            elif dist_center <= dist_left and dist_center <= dist_right:
                correction = actual_right_p - actual_left_p
                if correction > 0: nav.turn_by_pulses(correction, "left")
                elif correction < 0: nav.turn_by_pulses(abs(correction), "right")
            
            time.sleep(config.SETTLE_TIME_S)

            # =========================================================
            # PHASE 3: Verified Approach & Capture
            # =========================================================
            print("[OP] Phase 3: Final Approach")
            target_still_there = False
            for _ in range(3):
                verify_dist = nav.get_valid_distance(tof, timeout_ms=40)
                if verify_dist != 9999 and verify_dist <= (config.DETECT_THRESHOLD_MM + 50):
                    target_still_there = True
                    break
                time.sleep(0.01)

            if not target_still_there:
                print("[WARN] Target lost. Rescanning.")
                continue 

            is_slow_mode = False
            false_trigger_detected = False
            lowest_seen_dist = 9999
            lost_frame_count = 0
            
            motors.forward()
            motors.start_smoothly(config.FWD_SPEED_L, config.FWD_SPEED_R)
            
            while True:
                dist = nav.get_valid_distance(tof, timeout_ms=40)
                if dist != 9999:
                    lost_frame_count = 0
                    if dist < lowest_seen_dist: lowest_seen_dist = dist
                    elif dist > lowest_seen_dist + config.FALSE_TRIGGER_MARGIN_MM:
                        false_trigger_detected = True
                        break
                else:
                    lost_frame_count += 1
                    if lost_frame_count >= config.MAX_LOST_FRAMES:
                        false_trigger_detected = True
                        break

                if dist != 9999 and dist <= config.SLOWDOWN_THRESHOLD_MM and not is_slow_mode:
                    motors.set_speeds(config.SLOW_FWD_LEFT, config.SLOW_FWD_RIGHT)
                    is_slow_mode = True
                    
                if dist != 9999 and dist <= config.STOP_THRESHOLD_MM:
                    motors.stop() 
                    claw.close()
                    time.sleep(config.CLAW_WAIT_TIME_S) 
                    
                    if nav.get_valid_distance(tof, timeout_ms=60) <= config.CAPTURE_SUCCESS_MM:
                        # =========================================================
                        # PHASE 4: Color Identification & Payload Routing
                        # =========================================================
                        print("[OP] Phase 4: Payload Analysis")
                        ball_status = color_sensor.check_ball(debug=True) 
                        
                        if ball_status == "RED": 
                            print("[OP] Hazard detected (RED). Rejecting payload.")
                            claw.open()
                            time.sleep(0.5) 
                            kick() 
                        else:
                            print(f"[OP] Payload accepted. Routing to {target_direction} boundary.")
                            turn_120_pulses = 120.0 * (config.BOUNDARY_TURN_PULSES / 90.0) 
                            
                            if target_direction == "left":
                                nav.turn_by_pulses(turn_120_pulses, "left")
                            elif target_direction == "right":
                                nav.turn_by_pulses(turn_120_pulses, "right")
                            elif target_direction == "center":
                                nav.turn_by_pulses(config.BOUNDARY_TURN_PULSES, "right") 
                            
                            nav.drive_to_boundary_and_align()
                            nav.drive_backward(config.BOUNDARY_REVERSE_CM)
                            nav.turn_by_pulses(config.BOUNDARY_TURN_PULSES, "right")
                            nav.drive_to_boundary_and_align()
                            nav.drive_backward(config.BOUNDARY_REVERSE_CM)
                            nav.drive_to_boundary_and_align()
                            
                            claw.open()
                            time.sleep(1.0)
                        
                        nav.drive_backward(15.0) 
                        nav.turn_by_pulses(60.0, "left") 
                    else:
                        print("[WARN] Capture verification failed.")
                        claw.open()
                        nav.drive_backward(5.0) 
                    break 
                    
                if motors.check_faults():
                    motors.forward()
                    if is_slow_mode:
                        motors.set_speeds(config.SLOW_FWD_LEFT, config.SLOW_FWD_RIGHT) 
                    else:
                        motors.start_smoothly(config.FWD_SPEED_L, config.FWD_SPEED_R)
                time.sleep(0.01)
                
            if false_trigger_detected:
                print("[WARN] Approach aborted due to signal integrity.")
                motors.stop()
                time.sleep(0.5)

    except KeyboardInterrupt:
        print("[SYS] Manual halt requested.")
        motors.stop()
        try: claw.open()
        except: pass
    except Exception as e:
        print(f"[SYS] Runtime Error: {e}")
        motors.stop()

if __name__ == "__main__":
    main()