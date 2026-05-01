from machine import Pin, PWM
from time import sleep

class Servo:
    def __init__(self, pin_num):
        self.servo = PWM(Pin(pin_num))
        self.servo.freq(50)
        self.current_angle = 90  # Default middle start position

    def set_servo_angle(self, angle):
        angle = max(0, min(180, angle))
        duty = int(1638 + (angle / 180.0) * (8192 - 1638))
        self.servo.duty_u16(duty)
        self.current_angle = angle

    def move_to(self, target_angle, speed_percent=100):
        speed_percent = max(1, min(100, speed_percent))
        
        if speed_percent == 100:
            self.set_servo_angle(target_angle)
            return

        max_delay = 0.05
        delay = max_delay * (1.0 - (speed_percent / 100.0))
        step = 1 if target_angle > self.current_angle else -1
        
        for angle in range(self.current_angle, target_angle + step, step):
            self.set_servo_angle(angle)
            sleep(delay)


class Claw:
    """A helper class that manages the two servos together."""
    def __init__(self, pin1=9, pin2=22):
        self.servo9 = Servo(pin1)
        self.servo22 = Servo(pin2)
        
        # --- CALIBRATED ANGLES ---
        self.S9_OPEN = 110
        self.S22_OPEN = 90
        self.S9_CLOSED = 180
        self.S22_CLOSED = 10
        
        # --- DEFAULT SPEEDS ---
        self.open_speed = 100  # 100%
        self.close_speed = 90  # 30%
        
        # Initialize by snapping to the open position
        self.servo9.set_servo_angle(self.S9_OPEN)
        self.servo22.set_servo_angle(self.S22_OPEN)

    def open(self):
        self.servo9.move_to(self.S9_OPEN, speed_percent=self.open_speed)
        self.servo22.move_to(self.S22_OPEN, speed_percent=self.open_speed)
        
    def close(self):
        self.servo9.move_to(self.S9_CLOSED, speed_percent=self.close_speed)
        sleep(0.3)  # Slight delay between servo movements
        self.servo22.move_to(self.S22_CLOSED, speed_percent=self.close_speed)