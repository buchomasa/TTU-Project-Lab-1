from machine import Pin, ADC

class LineSensors:
    def __init__(self, left_pin=27, right_pin=26, threshold=26000):
        self.left_sensor = ADC(Pin(left_pin))
        self.right_sensor = ADC(Pin(right_pin))
        self.threshold = threshold

    def read(self):
        left_hit = self.left_sensor.read_u16() > self.threshold
        right_hit = self.right_sensor.read_u16() > self.threshold
        return left_hit, right_hit