from machine import Pin, ADC

class LineSensor:
    # Constants for thresholds
    BLACK_THRESHOLD = 3500
    GOALIE_BLUE_THRESHOLD = 2650
    BLUE_BLACK_OVERRIDE = 10000

    def __init__(self, black_pin_id=27, blue_pin_id=26):
        """Initializes the ADC pins for the line sensors."""
        self.black_sensor = ADC(Pin(black_pin_id))
        self.blue_sensor = ADC(Pin(blue_pin_id))

    def get_raw_values(self):
        """Returns a tuple of (black_raw, blue_raw)."""
        return self.black_sensor.read_u16(), self.blue_sensor.read_u16()

    def should_goalie_stop(self):
        """Returns True if the goalie should stop based on current readings."""
        black_raw, blue_raw = self.get_raw_values()
        if black_raw > self.BLACK_THRESHOLD:
            return True
        if blue_raw > self.GOALIE_BLUE_THRESHOLD:
            return True
        return False

    def should_kicker_stop(self):
        """Returns True if the kicker should stop based on current readings."""
        black_raw, blue_raw = self.get_raw_values()
        if black_raw > self.BLACK_THRESHOLD:
            return True
        if blue_raw > self.BLUE_BLACK_OVERRIDE:
            return True
        return False