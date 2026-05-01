from machine import Pin, PWM, Timer, time_pulse_us
import time
import config

class UltrasonicScanner:
    """Sweeping HC-SR04 mounted on an SG90 servo for frontal arc mapping."""
    def __init__(
        self,
        trig_pin=config.PIN_FRONT_TRIG,
        echo_pin=config.PIN_FRONT_ECHO,
        servo_pin=config.PIN_FRONT_SWEEP_SERVO,
    ):
        self.trigger = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN, Pin.PULL_DOWN)
        
        self.servo = PWM(Pin(servo_pin))
        self.servo.freq(50)
        
        self.sweep_angles = [163, 128, 93, 128] 
        self.angle_idx = 0
        self.current_angle = 128
        self.set_servo_angle(self.current_angle)
        self.timer = Timer(-1) 

    def start_sweep(self):
        self.timer.init(period=250, mode=Timer.PERIODIC, callback=self._sweep_tick)
        
    def pause_sweep(self):
        self.timer.deinit()

    def _sweep_tick(self, timer):
        self.angle_idx = (self.angle_idx + 1) % len(self.sweep_angles)
        self.current_angle = self.sweep_angles[self.angle_idx]
        self.set_servo_angle(self.current_angle)

    def set_servo_angle(self, angle):
        angle = max(0, min(180, angle))
        duty = int(1638 + (angle / 180.0) * (8192 - 1638))
        self.servo.duty_u16(duty)

    def get_distance(self):
        self.trigger.value(0)
        time.sleep_us(2)
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        
        pulse_time = time_pulse_us(self.echo, 1, 30000)
        if pulse_time <= 0:
            return 999.0 
        return round((pulse_time * 0.0343) / 2.0, 1)


class FixedUltrasonicScanner:
    """Stationary HC-SR04 for rear boundary clearance verification."""
    def __init__(self, trig_pin, echo_pin):
        self.trigger = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN, Pin.PULL_DOWN) 

    def get_distance(self):
        self.trigger.value(0)
        time.sleep_us(2)
        self.trigger.value(1)
        time.sleep_us(10)
        self.trigger.value(0)
        
        pulse_time = time_pulse_us(self.echo, 1, 30000)
        if pulse_time <= 0:
            return 999.0 
        return round((pulse_time * 0.0343) / 2.0, 1)