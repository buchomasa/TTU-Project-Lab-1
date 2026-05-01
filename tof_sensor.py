from machine import I2C, Pin
import vl53l1x
import time

class BallDetector:
    def __init__(self, i2c_id=0, sda_pin=0, scl_pin=1, int_pin_id=19):
        # Constants
        self.OFFSET = 30 
        self.BALL_DETECT_RANGE = 150   
        self.BALL_APPROACH_RANGE = 400 
        
        # Hardware Setup
        self.i2c = I2C(i2c_id, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=100000)
        self.tof = vl53l1x.VL53L1X(self.i2c)
        
        # Interrupt Setup
        self.data_ready = False
        self.int_pin = Pin(int_pin_id, Pin.IN, Pin.PULL_DOWN)
        self.int_pin.irq(trigger=Pin.IRQ_RISING, handler=self._sensor_isr)
        
        # Initial trigger to start the sensor loop
        self._clear_interrupt()

    def _sensor_isr(self, pin):
        """Internal callback for the hardware interrupt."""
        self.data_ready = True

    def _clear_interrupt(self):
        """Clears the sensor's internal interrupt flag to allow the next reading."""
        self.tof.writeReg(0x0086, 0x01)

    def get_distance(self):
        """Reads raw data, applies offset, and clears interrupt."""
        raw_dist = self.tof.read()
        self._clear_interrupt()
        
        d = raw_dist - self.OFFSET
        # Filter out noise/error codes
        if d <= -10 or d >= 8190:   
            return None
        return d

    def get_ball_status(self, d):
        """Returns the classification string based on distance."""
        if d is None:
            return 'NONE'
        if d <= self.BALL_DETECT_RANGE:
            return 'IN_RANGE'
        if d <= self.BALL_APPROACH_RANGE:
            return 'APPROACH'
        return 'NONE'