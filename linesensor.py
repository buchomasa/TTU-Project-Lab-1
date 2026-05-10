from machine import Pin, ADC
import time
import config

class LineSensor:
    # Black/white line boundary detection constants
    LEFT_BLACK_THRESHOLD = getattr(config, "LINE_LEFT_BLACK_THRESHOLD", config.LINE_BLACK_THRESHOLD)
    RIGHT_BLACK_THRESHOLD = getattr(config, "LINE_RIGHT_BLACK_THRESHOLD", config.LINE_BLACK_THRESHOLD)
    SAMPLE_COUNT = config.LINE_SAMPLE_COUNT
    SAMPLE_DELAY_US = config.LINE_SAMPLE_DELAY_US

    def __init__(self, left_pin_id=config.PIN_LINE_BLACK, right_pin_id=config.PIN_LINE_BLUE):
        """Initializes the two downward line sensors (left/right)."""
        self.left_sensor = ADC(Pin(left_pin_id))
        self.right_sensor = ADC(Pin(right_pin_id))
        self.use_adaptive = getattr(config, "LINE_USE_ADAPTIVE", False)
        self.left_margin = getattr(config, "LINE_LEFT_MARGIN", 1800)
        self.right_margin = getattr(config, "LINE_RIGHT_MARGIN", 2500)
        self.left_baseline = 0
        self.right_baseline = 0
        if self.use_adaptive:
            self.calibrate_white_baseline()

    def get_raw_values(self):
        """Returns a tuple of (left_raw, right_raw)."""
        return self.left_sensor.read_u16(), self.right_sensor.read_u16()

    def calibrate_white_baseline(self):
        """
        Capture white-floor baseline for each sensor.
        Run when rover is placed on normal floor (not boundary tape).
        """
        sample_count = getattr(config, "LINE_CALIBRATION_SAMPLES", 60)
        delay_ms = getattr(config, "LINE_CALIBRATION_DELAY_MS", 5)
        left_total = 0
        right_total = 0
        for _ in range(sample_count):
            l, r = self.get_raw_values()
            left_total += l
            right_total += r
            time.sleep_ms(delay_ms)
        self.left_baseline = left_total // sample_count
        self.right_baseline = right_total // sample_count

    def boundary_hits(self):
        """
        Multi-sample black boundary detection.
        Uses independent thresholds for left/right sensors.
        """
        left_hit = False
        right_hit = False
        for _ in range(self.SAMPLE_COUNT):
            left_raw, right_raw = self.get_raw_values()
            if self.use_adaptive:
                if left_raw > (self.left_baseline + self.left_margin):
                    left_hit = True
                if right_raw > (self.right_baseline + self.right_margin):
                    right_hit = True
            else:
                if left_raw > self.LEFT_BLACK_THRESHOLD:
                    left_hit = True
                if right_raw > self.RIGHT_BLACK_THRESHOLD:
                    right_hit = True
            if left_hit or right_hit:
                return left_hit, right_hit
            time.sleep_us(self.SAMPLE_DELAY_US)
        return left_hit, right_hit

    def boundary_detected(self):
        left_hit, right_hit = self.boundary_hits()
        return left_hit or right_hit

    def should_goalie_stop(self):
        """Goalie boundary safety (black boundary only)."""
        return self.boundary_detected()

    def should_kicker_stop(self):
        """Kicker boundary safety (black boundary only)."""
        return self.boundary_detected()