import math
import time

class ObstacleAvoidance:
    """Executes kinematic turns, escape maneuvers, and trap detection."""
    
    def __init__(self, motors, scanner_front, scanner_rear, encoders, config):
        self.motors = motors
        self.scanner_front = scanner_front
        self.scanner_rear = scanner_rear
        self.encoders = encoders
        
        # Load constraints from main.py config
        self.obs_thresh = config['obs_thresh']
        self.rear_thresh = config['rear_thresh']
        self.fwd_l, self.fwd_r = config['fwd_speeds']
        self.trn_l, self.trn_r = config['trn_speeds']
        self.track_width = config['track_width']
        self.wheel_dia = config['wheel_dia']
        self.disk_slots = config['disk_slots']
        
        # Anti-Wiggle Variables
        self.consecutive_turns = 0
        self.MAX_CONSECUTIVE_TURNS = 3 
        self.REVERSE_ESCAPE_TIME_MS = 1500
        self.TURN_TIMEOUT_MS = 3000

    def reset_trap_counter(self):
        """Clears spatial memory when forward progress is achieved."""
        self.consecutive_turns = 0

    def execute_exact_turn(self, angle_degrees, direction="RIGHT"):
        robot_circ = math.pi * self.track_width
        wheel_circ = math.pi * self.wheel_dia
        revolutions_needed = ((angle_degrees / 360.0) * robot_circ) / wheel_circ
        target_pulses = int(revolutions_needed * self.disk_slots)

        self.encoders.reset()

        if direction == "RIGHT":
            self.motors.turn_right()
        else:
            self.motors.turn_left()

        for step in range(1, 11):
            self.motors.set_speeds((self.trn_l * step) / 10, (self.trn_r * step) / 10)
            time.sleep(0.01) 

        start_time = time.ticks_ms()

        while True:
            left_p, right_p = self.encoders.get_pulses()
            if max(left_p, right_p) >= target_pulses:
                break
                
            if self.motors.check_faults():
                print("[!] Kinematic turn aborted: Stall condition.")
                break
                
            if time.ticks_diff(time.ticks_ms(), start_time) > self.TURN_TIMEOUT_MS:
                break
                
            time.sleep(0.01)

        self.motors.stop()

    def assess_and_escape(self):
        """3-point static sweep to find the optimum escape vector."""
        self.scanner_front.pause_sweep() 
        
        self.scanner_front.set_servo_angle(176)
        time.sleep(0.3)
        dist_left = self.scanner_front.get_distance()

        self.scanner_front.set_servo_angle(80)
        time.sleep(0.3)
        dist_right = self.scanner_front.get_distance()

        self.scanner_front.set_servo_angle(128)
        time.sleep(0.3)

        if dist_left > dist_right and dist_left > self.obs_thresh:
            self.execute_exact_turn(90, "LEFT")
            
        elif dist_right >= dist_left and dist_right > self.obs_thresh:
            self.execute_exact_turn(90, "RIGHT")
            
        else:
            dist_rear = self.scanner_rear.get_distance()
            if dist_rear > self.rear_thresh:
                print("REAR CLEAR: Initiating backing sequence.")
                if self.motors.start_smoothly_reverse(self.fwd_l, self.fwd_r):
                    reverse_start = time.ticks_ms()
                    while time.ticks_diff(time.ticks_ms(), reverse_start) < 1000:
                        if self.motors.check_faults() or self.scanner_rear.get_distance() < self.rear_thresh:
                            break 
                        time.sleep(0.05)
                self.motors.stop()
                self.execute_exact_turn(180, "RIGHT")
            else:
                print("TRAPPED: Executing blind 180 spin.")
                self.execute_exact_turn(180, "RIGHT")
            
        self.scanner_front.start_sweep() 

    def navigate_obstacle(self, distance, current_look_angle):
        """Processes dynamic sensor data and routes to correct avoidance behavior."""
        if self.consecutive_turns >= self.MAX_CONSECUTIVE_TURNS:
            print("\n[!] WIGGLE TRAP DETECTED: Forcing reverse override.")
            self.scanner_front.pause_sweep()
            
            if self.scanner_rear.get_distance() > self.rear_thresh:
                if self.motors.start_smoothly_reverse(self.fwd_l, self.fwd_r):
                    reverse_start = time.ticks_ms()
                    while time.ticks_diff(time.ticks_ms(), reverse_start) < self.REVERSE_ESCAPE_TIME_MS:
                        if self.motors.check_faults() or self.scanner_rear.get_distance() < self.rear_thresh:
                            break
                        time.sleep(0.05)
                self.motors.stop()
                self.execute_exact_turn(90, "RIGHT")
            else:
                self.execute_exact_turn(180, "RIGHT")
            
            self.reset_trap_counter()
            self.scanner_front.start_sweep()
            return

        # Standard reactive veering
        if current_look_angle > 140:
            print(f"\nSide Obstacle Left ({distance}cm). Veering Right.")
            self.execute_exact_turn(60, "RIGHT") 
            self.consecutive_turns += 1
        elif current_look_angle < 110:
            print(f"\nSide Obstacle Right ({distance}cm). Veering Left.")
            self.execute_exact_turn(60, "LEFT") 
            self.consecutive_turns += 1
        else:
            print(f"\nFront Obstacle ({distance}cm). Halting & Assessing.")
            self.assess_and_escape()
            self.consecutive_turns += 1