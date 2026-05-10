import time
import math
import config

class Navigator:
    """Handles odometry-based movement, sweeping, and boundary alignment."""
    
    def __init__(self, motors, encoders, line_sensor):
        self.motors = motors
        self.encoders = encoders
        self.line_sensor = line_sensor
        self.wheel_circumference = math.pi * config.WHEEL_DIAMETER_CM
        self.cm_per_pulse = self.wheel_circumference / config.DISK_SLOTS

    def get_valid_distance(self, tof, timeout_ms=40):
        timeout = time.ticks_add(time.ticks_ms(), timeout_ms)
        while not tof.data_ready and time.ticks_diff(timeout, time.ticks_ms()) > 0:
            time.sleep(0.002)
            
        if tof.data_ready:
            dist = tof.get_distance()
            tof.data_ready = False  
            if dist is not None and dist > -15:
                return dist
        return 9999  

    def turn_by_pulses(self, target_pulses, direction, speed_factor=1.0):
        if target_pulses <= 0: return 0
        self.encoders.reset()
        
        spd_l = config.TURN_SPEED_L * speed_factor
        spd_r = config.TURN_SPEED_R * speed_factor
        
        self.motors.stop() 
        time.sleep(config.SETTLE_TIME_S)
        
        if direction == "left": self.motors.turn_left() 
        else: self.motors.turn_right() 
            
        for step in range(1, 11): 
            self.motors.set_speeds((spd_l * step) / 10, (spd_r * step) / 10) 
            time.sleep(0.02)
        
        while True:
            l, r = self.encoders.get_pulses()
            if ((l + r) / 2.0) >= target_pulses:
                break
            if self.motors.check_faults(): 
                if direction == "left": self.motors.turn_left()
                else: self.motors.turn_right()
                self.motors.set_speeds(spd_l, spd_r)
            time.sleep(0.01)
            
        self.motors.stop()
        time.sleep(config.SETTLE_TIME_S)
        
        l_final, r_final = self.encoders.get_pulses()
        return (l_final + r_final) / 2.0

    def drive_forward(self, distance_cm):
        if distance_cm <= 0: return
        target_pulses = distance_cm / self.cm_per_pulse
        self.encoders.reset()
        
        self.motors.forward() 
        self.motors.set_speeds(0, 0)
        if not self.motors.start_smoothly(config.FWD_SPEED_L, config.FWD_SPEED_R): 
            self.motors.set_speeds(config.FWD_SPEED_L, config.FWD_SPEED_R)

        while True:
            l, r = self.encoders.get_pulses()
            if ((l + r) / 2.0) >= target_pulses:
                break
            if self.motors.check_faults():
                self.motors.forward()
                self.motors.start_smoothly(config.FWD_SPEED_L, config.FWD_SPEED_R)
            time.sleep(0.01)
            
        self.motors.stop()
        time.sleep(config.SETTLE_TIME_S)

    def drive_backward(self, distance_cm):
        if distance_cm <= 0: return
        target_pulses = distance_cm / self.cm_per_pulse
        self.encoders.reset()
        
        self.motors.reverse() 
        self.motors.set_speeds(0, 0)
        if not self.motors.start_smoothly_reverse(config.FWD_SPEED_L, config.FWD_SPEED_R): 
            self.motors.set_speeds(config.FWD_SPEED_L, config.FWD_SPEED_R)

        while True:
            l, r = self.encoders.get_pulses()
            if ((l + r) / 2.0) >= target_pulses:
                break
            if self.motors.check_faults():
                self.motors.reverse()
                self.motors.start_smoothly_reverse(config.FWD_SPEED_L, config.FWD_SPEED_R)
            time.sleep(0.01)
            
        self.motors.stop()
        time.sleep(config.SETTLE_TIME_S)
        
    def drive_to_boundary_and_align(self, timeout_ms=15000):
        left_on_line = False
        right_on_line = False
        start_time = time.ticks_ms()
        
        self.motors.forward()
        self.motors.set_speeds(0, 0)
        
        if not self.motors.start_smoothly(config.FWD_SPEED_L, config.FWD_SPEED_R):
            self.motors.set_speeds(config.FWD_SPEED_L, config.FWD_SPEED_R)
            
        while not (left_on_line and right_on_line):
            if time.ticks_diff(time.ticks_ms(), start_time) > timeout_ms:
                print("[WARN] Boundary alignment timeout.")
                break

            if self.motors.check_faults():
                if left_on_line and not right_on_line:
                    self.motors.turn_right()
                    self.motors.set_speeds(config.ALIGN_SPEED_L, config.ALIGN_SPEED_R)
                elif right_on_line and not left_on_line:
                    self.motors.turn_left()
                    self.motors.set_speeds(config.ALIGN_SPEED_L, config.ALIGN_SPEED_R)
                else:
                    self.motors.forward()
                    self.motors.start_smoothly(config.FWD_SPEED_L, config.FWD_SPEED_R)

            try:
                l_hit, r_hit = self.line_sensor.boundary_hits()
                
                if l_hit and not left_on_line:
                    left_on_line = True
                    if not right_on_line:
                        self.motors.turn_right()
                        self.motors.set_speeds(config.ALIGN_SPEED_L, config.ALIGN_SPEED_R)
                    
                if r_hit and not right_on_line:
                    right_on_line = True
                    if not left_on_line:
                        self.motors.turn_left()
                        self.motors.set_speeds(config.ALIGN_SPEED_L, config.ALIGN_SPEED_R)
                        
            except Exception:
                pass
                
            time.sleep(0.01)
            
        self.motors.stop()
        time.sleep(config.SETTLE_TIME_S)

    def sweep_until_found(self, target_pulses, direction, tof_sensor):
        print(f"[NAV] Sweeping {direction.upper()}...")
        self.encoders.reset()
        
        spd_l = config.TURN_SPEED_L * config.SCAN_SPEED_FACTOR
        spd_r = config.TURN_SPEED_R * config.SCAN_SPEED_FACTOR
        
        self.motors.stop()
        time.sleep(config.SETTLE_TIME_S)
        
        if direction == "left": self.motors.turn_left()
        else: self.motors.turn_right()
            
        for step in range(1, 11): 
            self.motors.set_speeds((spd_l * step) / 10, (spd_r * step) / 10)
            time.sleep(0.02)

        found_dist = 9999
        target_found = False

        while True:
            l, r = self.encoders.get_pulses()
            avg_pulses = (l + r) / 2.0
            
            if avg_pulses >= target_pulses:
                break
                
            dist = self.get_valid_distance(tof_sensor, timeout_ms=35)
            
            if dist != 9999 and dist <= config.DETECT_THRESHOLD_MM:
                found_dist = dist
                target_found = True
                break
                
            if self.motors.check_faults():
                if direction == "left": self.motors.turn_left()
                else: self.motors.turn_right()
                self.motors.set_speeds(spd_l, spd_r)
            time.sleep(0.005)
            
        self.motors.stop()
        time.sleep(config.SETTLE_TIME_S)
        return target_found, found_dist