"""
Standalone line sensor test - mode-aware detection.
No imports from other project files.
Tests both goalie mode and kicker mode in one run.

GOALIE: stops on black border AND blue centerline
KICKER: stops only on black border (ignores blue)
"""
from machine import Pin, ADC
import time


# ============================================================
# Pin assignments
# ============================================================
PIN_LINE_BLACK = 27
PIN_LINE_BLUE = 26


# ============================================================
# Thresholds
# ============================================================
LINE_BLACK_THRESHOLD = 3500            # black sensor > this = stop (both modes)
LINE_GOALIE_BLUE_THRESHOLD = 4000      # blue sensor > this = stop in goalie mode
LINE_BLUE_BLACK_OVERRIDE = 10000       # blue sensor > this = stop in kicker mode


# ============================================================
# Sensor setup
# ============================================================
black_sensor = ADC(Pin(PIN_LINE_BLACK))
blue_sensor = ADC(Pin(PIN_LINE_BLUE))


# ============================================================
# Mode-aware detection
# ============================================================
def check_goalie(black_raw, blue_raw):
    """
    Goalie mode - stops on ANY tape (black or blue).
    Returns True if should stop.
    """
    if black_raw > LINE_BLACK_THRESHOLD:
        return True
    if blue_raw > LINE_GOALIE_BLUE_THRESHOLD:
        return True
    return False


def check_kicker(black_raw, blue_raw):
    """
    Kicker mode - stops ONLY on black borders, ignores blue.
    Returns True if should stop.
    """
    if black_raw > LINE_BLACK_THRESHOLD:
        return True
    if blue_raw > LINE_BLUE_BLACK_OVERRIDE:
        return True
    return False


# ============================================================
# Main test loop
# ============================================================
print("=" * 60)
print("Mode-aware line sensor test")
print("=" * 60)
print("Move rover over different surfaces. Watch both columns.")
print()
print("Expected behavior:")
print("  Floor       -> Goalie: FORWARD,  Kicker: FORWARD")
print("  Blue tape   -> Goalie: STOP,     Kicker: FORWARD")
print("  Black tape  -> Goalie: STOP,     Kicker: STOP")
print()
print("Press Ctrl+C to stop.")
print()

last_goalie = None
last_kicker = None

try:
    while True:
        black_raw = black_sensor.read_u16()
        blue_raw = blue_sensor.read_u16()

        goalie_stop = check_goalie(black_raw, blue_raw)
        kicker_stop = check_kicker(black_raw, blue_raw)

        goalie_action = "STOP" if goalie_stop else "FORWARD"
        kicker_action = "STOP" if kicker_stop else "FORWARD"

        # Print every reading
        print("B={:5d} U={:5d}  |  Goalie: {:7s}  Kicker: {:7s}".format(
            black_raw, blue_raw, goalie_action, kicker_action))

        time.sleep(1)  # 1 reading per second

except KeyboardInterrupt:
    print("\nStopped.")
