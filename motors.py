from machine import Pin, PWM
import time
import config

class MotorController:
    """Manages L298N H-Bridge states, PWM speed control, and stall detection."""
    def __init__(
        self,
        in1=config.PIN_MOTOR_IN1,
        in2=config.PIN_MOTOR_IN2,
        in3=config.PIN_MOTOR_IN3,
        in4=config.PIN_MOTOR_IN4,
        enA=config.PIN_MOTOR_ENA,
        enB=config.PIN_MOTOR_ENB,
        oc_a=config.PIN_OC_A,
        oc_b=config.PIN_OC_B,
    ):
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.in3 = Pin(in3, Pin.OUT)
        self.in4 = Pin(in4, Pin.OUT)
        
        self.enA = PWM(Pin(enA))
        self.enA.freq(1000)
        self.enB = PWM(Pin(enB))
        self.enB.freq(1000)
        
        # Pull-ups used for analog/noisy current sense outputs
        self.oc_a = Pin(oc_a, Pin.IN, Pin.PULL_UP)
        self.oc_b = Pin(oc_b, Pin.IN, Pin.PULL_UP)

    def set_speeds(self, left_pct, right_pct):
        """Maps 0-100% to 16-bit duty cycle."""
        l_duty = int((max(0, min(100, left_pct)) / 100.0) * 65535)
        r_duty = int((max(0, min(100, right_pct)) / 100.0) * 65535)
        self.enA.duty_u16(l_duty)
        self.enB.duty_u16(r_duty)

    def check_faults(self):
        """Polls current sense pins with a 25ms debounce filter to reject EMI."""
        if self.oc_a.value() == 1 or self.oc_b.value() == 1:
            fault_confirmed = True
            for _ in range(5):
                time.sleep(0.005) 
                if self.oc_a.value() == 0 and self.oc_b.value() == 0:
                    fault_confirmed = False 
                    break
                    
            if fault_confirmed:
                print("\n[!] OVERCURRENT DETECTED: Engaging 500ms safety pause.")
                self.stop()
                time.sleep(0.5) 
                return True
                
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
        """Ramps forward PWM over 200ms to prevent high inrush current."""
        self.forward()
        for step in range(1, 11): 
            self.set_speeds((target_left * step) / 10, (target_right * step) / 10)
            if self.check_faults(): 
                return False 
            time.sleep(0.02) 
        return True
    
    def start_smoothly_reverse(self, target_left, target_right):
        """Ramps reverse PWM over 200ms to prevent high inrush current."""
        self.reverse()
        for step in range(1, 11): 
            self.set_speeds((target_left * step) / 10, (target_right * step) / 10)
            if self.check_faults(): 
                return False 
            time.sleep(0.02) 
        return True