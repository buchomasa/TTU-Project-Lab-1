from machine import Pin, PWM
import time

class MotorController:
    def __init__(self, in1=4, in2=5, in3=6, in4=7, enA=3, enB=2, oc_a=15, oc_b=14):
        # Direction Control (Purple Board)
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.in3 = Pin(in3, Pin.OUT)
        self.in4 = Pin(in4, Pin.OUT)
        
        # Speed Control - Matched to speed_sensor.py (enA=3 for Left, enB=2 for Right)
        self.enA = PWM(Pin(enA)); self.enA.freq(1000)
        self.enB = PWM(Pin(enB)); self.enB.freq(1000)
        
        # Overcurrent Protection (Green Board)
        self.oc_a = Pin(oc_a, Pin.IN, Pin.PULL_UP)
        self.oc_b = Pin(oc_b, Pin.IN, Pin.PULL_UP)
        
        self.system_fault = False

    def set_speeds(self, left_pct, right_pct):
        """Accepts independent left and right speeds."""
        l_duty = int((max(0, min(100, left_pct)) / 100.0) * 65535)
        r_duty = int((max(0, min(100, right_pct)) / 100.0) * 65535)
        self.enA.duty_u16(l_duty)
        self.enB.duty_u16(r_duty)

    def check_faults(self):
        fault_a = self.oc_a.value() == 1
        fault_b = self.oc_b.value() == 1
        if fault_a or fault_b:
            if not self.system_fault:
                self.stop()
                self.system_fault = True
                print("\n[!] OVERCURRENT DETECTED! Motors disabled.")
            return True
        self.system_fault = False
        return False

    def stop(self):
        self.in1.value(0); self.in2.value(0)
        self.in3.value(0); self.in4.value(0)
        self.set_speeds(0, 0)

    def forward(self):
        self.in1.value(0); self.in2.value(1)
        self.in3.value(1); self.in4.value(0)

    def reverse(self):
        self.in1.value(1); self.in2.value(0)
        self.in3.value(0); self.in4.value(1)

    def turn_left(self):
        self.in1.value(1); self.in2.value(0)
        self.in3.value(1); self.in4.value(0)

    def turn_right(self):
        self.in1.value(0); self.in2.value(1)
        self.in3.value(0); self.in4.value(1)

    def start_smoothly(self, target_left, target_right):
        """Ramps up both motors proportionally to maintain a straight line."""
        self.forward()
        for step in range(1, 11): # 10 steps to full target speed
            cur_left = (target_left * step) / 10
            cur_right = (target_right * step) / 10
            self.set_speeds(cur_left, cur_right)
            if self.check_faults(): return False
            time.sleep(0.05)
        return True