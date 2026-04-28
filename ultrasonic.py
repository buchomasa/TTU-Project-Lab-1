from machine import Pin, PWM
import time

class UltrasonicScanner:
    def __init__(self, trig_pin=12, echo_pin=13, servo_pin=0):
        self.trigger = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.servo = PWM(Pin(servo_pin))
        self.servo.freq(50)
        self.start_us = 0
        self.end_us = 0
        self.echo.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._echo_isr)
        self.set_servo_angle(90)

    def _echo_isr(self, pin):
        if pin.value() == 1:
            self.start_us = time.ticks_us()
        else:
            self.end_us = time.ticks_us()

    def set_servo_angle(self, angle):
        angle = max(0, min(180, angle))
        duty = int(1638 + (angle / 180.0) * (8192 - 1638))
        self.servo.duty_u16(duty)

    def get_distance(self):
        self.trigger.low()
        time.sleep_us(2)
        self.trigger.high()
        time.sleep_us(10)
        self.trigger.low()
        
        time.sleep(0.03) 
        pulse_time = time.ticks_diff(self.end_us, self.start_us)
        
        if pulse_time <= 0 or pulse_time > 30000:
            return 999.0 
        return round((pulse_time * 0.0343) / 2.0, 1)